namespace RW.Toolpaths.Milling;

/// <summary>Open stock edges expressed in the unmirrored image's local XY coordinates.</summary>
[Flags]
public enum HeightmapOpenEdges
{
    None = 0,
    MinX = 1,
    MaxX = 2,
    MinY = 4,
    MaxY = 8,
    All = MinX | MaxX | MinY | MaxY
}

/// <summary>Finite constant-edge extension of a heightmap at actual stock boundaries.</summary>
public static class HeightmapEdgePadding
{
    /// <summary>Extend explicitly open stock edges with their terminal image samples.
    /// The caller must bound cutter centers so the new outer image boundary remains
    /// farther away than the complete cutter contact radius. Interior image borders,
    /// masks, and the original bilinear surface remain unchanged. Both roughing slices
    /// and finishing clearance then use the ordinary surface algorithms.</summary>
    public static (HeightmapGrid Grid, HeightmapPlacement Placement) Extend(HeightmapGrid grid,
        HeightmapPlacement placement, HeightmapOpenEdges edges, double distance,
        CancellationToken cancellationToken = default)
    {
        if ((edges & ~HeightmapOpenEdges.All) != 0 || !double.IsFinite(distance) || distance < 0
            || !double.IsFinite(placement.Width) || !double.IsFinite(placement.Height)
            || placement.Width <= 0 || placement.Height <= 0
            || !double.IsFinite(placement.OriginX) || !double.IsFinite(placement.OriginY)
            || !double.IsFinite(placement.AngleDegrees))
            throw new ArgumentException("Invalid open heightmap edges, placement, or extension distance.");
        if (edges == HeightmapOpenEdges.None || distance == 0) return (grid, placement);
        cancellationToken.ThrowIfCancellationRequested();
        double pixelX = placement.Width / (grid.Width - 1), pixelY = placement.Height / (grid.Height - 1);
        double nx = Math.Ceiling(distance / pixelX), ny = Math.Ceiling(distance / pixelY);
        double minX = edges.HasFlag(HeightmapOpenEdges.MinX) ? nx : 0;
        double maxX = edges.HasFlag(HeightmapOpenEdges.MaxX) ? nx : 0;
        double minY = edges.HasFlag(HeightmapOpenEdges.MinY) ? ny : 0;
        double maxY = edges.HasFlag(HeightmapOpenEdges.MaxY) ? ny : 0;
        double width = grid.Width + minX + maxX, height = grid.Height + minY + maxY;
        if (!double.IsFinite(width + height) || width * height > 16_777_216)
            throw new InvalidOperationException("The image plus cutter edge clearance exceeds sixteen million height samples.");
        int xPadding = (int)minX, yPadding = (int)minY;
        int paddedWidth = (int)width, paddedHeight = (int)height;
        var samples = new float[paddedWidth * paddedHeight];
        var mask = new bool[samples.Length];
        for (int y = 0; y < paddedHeight; y++)
        {
            cancellationToken.ThrowIfCancellationRequested();
            int sourceY = Math.Clamp(y - yPadding, 0, grid.Height - 1);
            for (int x = 0; x < paddedWidth; x++)
            {
                int sourceX = Math.Clamp(x - xPadding, 0, grid.Width - 1);
                int index = y * paddedWidth + x;
                samples[index] = grid[sourceX, sourceY];
                mask[index] = grid.IsIncluded(sourceX, sourceY);
            }
        }
        // ToWorld includes mirroring about the original dimensions. A mirrored image
        // shifts its physical origin by the opposite padding side when dimensions grow.
        var origin = placement.ToWorld(placement.MirrorX ? placement.Width + maxX * pixelX : -minX * pixelX,
            placement.MirrorY ? placement.Height + maxY * pixelY : -minY * pixelY);
        var expandedPlacement = placement with {
            Width = (paddedWidth - 1) * pixelX,
            Height = (paddedHeight - 1) * pixelY,
            OriginX = origin.x, OriginY = origin.y
        };
        Console.WriteLine($"[Carving.EdgeExtension] edges={edges} distance={distance:R} " +
            $"source={grid.Width}x{grid.Height} padded={paddedWidth}x{paddedHeight} " +
            $"padding={minX * pixelX:R},{maxX * pixelX:R},{minY * pixelY:R},{maxY * pixelY:R} " +
            "boundary=edge-replication clearance=original-full-profile");
        return (new HeightmapGrid(paddedWidth, paddedHeight, samples, mask), expandedPlacement);
    }
}
