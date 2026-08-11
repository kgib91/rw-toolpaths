using Clipper2Lib;

namespace RW.Toolpaths.Geometry;

/// <summary>
/// One connected component of a filled region: a single outer contour (CCW)
/// plus the holes immediately inside it (CW).
/// </summary>
public sealed class PolygonComponent
{
    public List<PointD> Outer { get; set; } = new();

    public List<List<PointD>> Holes { get; } = new();
}
