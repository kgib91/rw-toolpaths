using Clipper2Lib;

namespace RW.Toolpaths.Milling;

/// <summary>Evaluates the finished surface at a fixed physical inspection scale.
/// The complete original stock field is the reference for EVERY omission, so
/// individually small overlapping removals cannot accumulate without rechecking.</summary>
internal static class VCarveAppearancePruner
{
    // User's closest normal inspection distance is six inches. A nominal
    // one-arcminute detail model is oversampled 2x, independent of camera zoom.
    // Acuity reference: https://www.ncbi.nlm.nih.gov/books/NBK11509/
    // These are preview/appearance parameters, not
    // a claim that every observer, material and lighting has identical acuity.
    internal const double InspectionDistanceMm = 152.4;
    internal const double InspectionDetail = InspectionDistanceMm * Math.PI / (180 * 60);
    internal const double PixelPitch = InspectionDetail / 2;
    private const int Halo = 3, MaximumPatchPixels = 100000;
    private const double MaximumShadeDifference = .035, MaximumRmsShadeDifference = .0125;
    internal static readonly AsyncLocal<Action<object>?> Observation = new();
    private static readonly (double X, double Y, double Z)[] Lights = BuildLights();
    private readonly record struct Contact(double Depth, int Owner);
    private sealed class Pixel
    {
        internal readonly double Original;
        internal Contact Current;
        internal Pixel(Contact original) { Original = original.Depth; Current = original; }
    }

    internal static List<TaggedToolpath> Filter(IReadOnlyList<TaggedToolpath> source,
        VCarveCutterProfile profile, IReadOnlyList<VCarveRoughingChord> roughing,
        Func<Point3D, Point3D> command, Func<double, double, double> artworkDistance,
        CancellationToken token, Action<string>? diagnostic = null)
    {
        token.ThrowIfCancellationRequested();
        if (source.Any(p => p.Spans.Any(s => s.Kind != ToolpathSpanKind.Cut)))
        {
            diagnostic?.Invoke("stage=visit-appearance event=retain-all reason=non-cut-span-stock-evidence-unsupported");
            return source.ToList();
        }
        var points = source.Select(p => p.Points.Select(command).ToArray()).ToArray();
        if (points.Any(row => row.Any(p => !double.IsFinite(p.X + p.Y + p.Z)
            || Math.Abs(p.X) > 10000 || Math.Abs(p.Y) > 10000 || Math.Abs(p.Z) > 100)))
        {
            diagnostic?.Invoke("stage=visit-appearance event=retain-all reason=command-domain-unsupported");
            return source.ToList();
        }
        var active = Enumerable.Repeat(true, source.Count).ToArray();
        var reference = new Dictionary<(int X, int Y), Pixel>();
        var work = new List<(int Owner, int X, int Y, int Width, int Height, double Radius)>();
        int oversized = 0, edges = 0, visible = 0, removed = 0, unchanged = 0, observed = 0, deepObserved = 0;
        long retainedWitnessHits = 0;
        foreach (int owner in Enumerable.Range(0, source.Count))
        {
            var row = points[owner];
            if (row.Length < 2 || row.Length > 512 || source[owner].Spans.Any(s => s.Kind != ToolpathSpanKind.Cut)) continue;
            double depth = row.Max(p => -p.Z), radius = profile.PhysicalRadiusAtDepth(Math.Max(0, depth));
            double minX = row.Min(p => p.X), maxX = row.Max(p => p.X);
            double minY = row.Min(p => p.Y), maxY = row.Max(p => p.Y);
            // An isolated plunge may itself cut. It cannot be omitted merely
            // because the scene excludes XY-degenerate stock witnesses.
            if (maxX == minX && maxY == minY) continue;
            if (maxX - minX > 2 || maxY - minY > 2) { oversized++; continue; }
            int x = (int)Math.Floor((minX - radius) / PixelPitch) - Halo;
            int y = (int)Math.Floor((minY - radius) / PixelPitch) - Halo;
            int width = (int)Math.Ceiling((maxX + radius) / PixelPitch) - x + Halo + 1;
            int height = (int)Math.Ceiling((maxY + radius) / PixelPitch) - y + Halo + 1;
            if ((long)width * height > MaximumPatchPixels) { oversized++; continue; }
            work.Add((owner, x, y, width, height, radius));
        }
        if (work.Count == 0)
        {
            diagnostic?.Invoke($"stage=visit-appearance event=complete sourceVisits={source.Count} removedVisits=0 "
                + $"largerVisitsRetained={oversized} reason=no-eligible-local-patches unknownRetained=True");
            return source.ToList();
        }
        var scene = new StockScene(points, profile, roughing, active, token);
        if (!scene.Supported)
        {
            diagnostic?.Invoke("stage=visit-appearance event=retain-all reason=scene-input-budget-or-domain");
            return source.ToList();
        }
        foreach (var patch in work.OrderBy(w => w.Width * w.Height).ThenBy(w => w.Owner))
        {
            token.ThrowIfCancellationRequested();
            if (scene.BudgetReached) break;
            var before = new double[patch.Width * patch.Height];
            var after = new double[before.Length];
            var cells = new Pixel[before.Length];
            var witnesses = new Contact[before.Length];
            bool complete = true, contourChanged = false, changedPixel = false;
            for (int y = 0; y < patch.Height && complete; y++)
            for (int x = 0; x < patch.Width; x++)
            {
                var key = (patch.X + x, patch.Y + y);
                if (!reference.TryGetValue(key, out var cell))
                {
                    var original = scene.Sample(key.Item1 * PixelPitch, key.Item2 * PixelPitch, -1, original: true);
                    if (!double.IsFinite(original.Depth)) { complete = false; break; }
                    cell = new(original); reference.Add(key, cell);
                }
                // A retained pose attaining the cached maximum proves that
                // omitting some OTHER owner cannot change this pixel. Deletions
                // only lower the envelope; no stale maximum can rise again.
                var current = cell.Current;
                if (current.Owner != patch.Owner && (current.Owner < 0 || active[current.Owner])) retainedWitnessHits++;
                else current = scene.Sample(key.Item1 * PixelPitch, key.Item2 * PixelPitch, patch.Owner, original: false);
                if (!double.IsFinite(current.Depth)) { complete = false; break; }
                int index = y * patch.Width + x;
                cells[index] = cell; witnesses[index] = current;
                before[index] = cell.Original;
                after[index] = current.Depth;
                changedPixel |= cell.Original != current.Depth;
                // Protect the actual stock adjacent to the authored contour.
                // A footprint touching the outline is not itself a difference.
                // This stricter geometry bound is tied to inspection scale too.
                if (Math.Abs(cell.Original - current.Depth) > InspectionDetail / 4
                    && artworkDistance(key.Item1 * PixelPitch, key.Item2 * PixelPitch) <= InspectionDetail)
                { contourChanged = true; complete = false; break; }
            }
            if (contourChanged) { edges++; continue; }
            if (!complete) break;
            var score = changedPixel ? Compare(before, after, patch.Width, patch.Height, PixelPitch,
                preserveMicrotexture: source[patch.Owner].Category != "clearing")
                : new Score(false, 0, 0, 0, 0);
            if (!score.Visible)
            {
                active[patch.Owner] = false; removed++;
                // Only committed omissions update the current witness. A failed
                // trial never grants its temporarily excluded owner stock credit.
                for (int i = 0; i < cells.Length; i++) cells[i].Current = witnesses[i];
                if (score.MaximumHeightDifference <= 1e-10) unchanged++;
                if (removed <= 8)
                    diagnostic?.Invoke($"stage=visit-appearance event=omit owner={patch.Owner} region={source[patch.Owner].RegionIndex} "
                        + $"origin=({patch.X * PixelPitch:R},{patch.Y * PixelPitch:R}) pixels={patch.Width}x{patch.Height} "
                        + $"maxHeightChange={score.MaximumHeightDifference:R} normalDisplacement={score.MaximumNormalDisplacement:R} "
                        + $"maxShadeChange={score.MaximumShadeDifference:R} rmsShadeChange={score.RmsShadeDifference:R} "
                        + $"rmsNormalDisplacement={score.RmsNormalDisplacement:R} acceptance={(source[patch.Owner].Category == "clearing" ? "local-finished-form" : "local-detail")} "
                        + "reference=original-complete-stock cumulativeRemovalsIncluded=True");
                double maximumPostedDepth = points[patch.Owner].Max(p => -p.Z);
                if (Observation.Value is { } observe && (observed++ < 4 || maximumPostedDepth >= .25 && deepObserved++ < 4))
                    observe(new { owner = patch.Owner, x = patch.X * PixelPitch, y = patch.Y * PixelPitch,
                        patch.Width, patch.Height, pitch = PixelPitch, maximumPostedDepth, before, after, score });
            }
            else visible++;
        }
        diagnostic?.Invoke($"stage=visit-appearance event=complete sourceVisits={source.Count} removedVisits={removed} "
            + $"stockUnchangedAtSamples={unchanged} visibleRetained={visible} outlineRetained={edges} largerVisitsRetained={oversized} "
            + $"inspectionDistanceMm={InspectionDistanceMm:R} inspectionDetailMm={InspectionDetail:R} pixelPitchMm={PixelPitch:R} referenceSamples={reference.Count} "
            + $"contactEvaluations={scene.Evaluations} budgetReached={scene.BudgetReached} "
            + $"retainedWitnessHits={retainedWitnessHits} maximumPatchPixels={MaximumPatchPixels} "
            + "policy=part-scale-surface-normal-and-17-matte-lights depthCutoff=False originalReferenceFixed=True "
            + "clearingAcceptance=local-form-RMS finalCarveAcceptance=local-detail "
            + "unknownRetained=True fullCutterContact=True beforeSurfaceOriginStepdown=True");
        return source.Where((_, i) => active[i]).ToList();
    }

    internal sealed record Score(bool Visible, double MaximumHeightDifference,
        double MaximumNormalDisplacement, double MaximumShadeDifference, double RmsShadeDifference,
        double RmsNormalDisplacement = 0);

    internal static Score Compare(double[] before, double[] after, int width, int height, double pitch,
        bool preserveMicrotexture = true)
    {
        var a = Blur(before, width, height); var b = Blur(after, width, height);
        double maximumHeight = 0, maximumNormal = 0, maximumShade = 0, sum = 0, normalSum = 0;
        long samples = 0;
        for (int y = 1; y < height - 1; y++)
        for (int x = 1; x < width - 1; x++)
        {
            int i = y * width + x;
            double delta = Math.Abs(before[i] - after[i]);
            maximumHeight = Math.Max(maximumHeight, delta);
            var na = Normal(a, i, width, pitch); var nb = Normal(b, i, width, pitch);
            double normalDisplacement = delta * Math.Max(na.Z, nb.Z);
            maximumNormal = Math.Max(maximumNormal, normalDisplacement);
            normalSum += normalDisplacement * normalDisplacement;
            foreach (var light in Lights)
            {
                double sa = .25 + .75 * Math.Max(0, na.X * light.X + na.Y * light.Y + na.Z * light.Z);
                double sb = .25 + .75 * Math.Max(0, nb.X * light.X + nb.Y * light.Y + nb.Z * light.Z);
                double difference = Math.Abs(sa - sb);
                maximumShade = Math.Max(maximumShade, difference); sum += difference * difference; samples++;
            }
        }
        double rms = samples > 0 ? Math.Sqrt(sum / samples) : 0;
        double rmsNormal = samples > 0 ? Math.Sqrt(normalSum / (samples / (double)Lights.Length)) : 0;
        // The user's actual stock comparisons established that local scallop
        // texture is acceptable when the overall form reads the same. Clearing
        // therefore measures error across the complete physical cutter patch;
        // a single bright subpixel or vertical height delta on a steep wall is
        // not an independent veto. The fixed original surface and cumulative
        // masks prevent this texture allowance accumulating into a form change.
        // Authored-contour protection remains a separate stricter test.
        bool visible = preserveMicrotexture
            ? maximumNormal > InspectionDetail / 2 || maximumHeight > 2 * InspectionDetail
                || maximumShade > MaximumShadeDifference || rms > MaximumRmsShadeDifference
            : rmsNormal > InspectionDetail || rms > .04;
        return new(visible, maximumHeight, maximumNormal, maximumShade, rms, rmsNormal);
    }

    private static (double X, double Y, double Z)[] BuildLights()
    {
        var result = new List<(double, double, double)> { (0, 0, 1) };
        foreach (double elevation in new[] { 20d, 60d })
        for (int azimuth = 0; azimuth < 8; azimuth++)
        {
            double e = elevation * Math.PI / 180, angle = azimuth * Math.PI / 4;
            result.Add((Math.Cos(e) * Math.Cos(angle), Math.Cos(e) * Math.Sin(angle), Math.Sin(e)));
        }
        return result.ToArray();
    }

    private static double[] Blur(double[] input, int width, int height)
    {
        var output = (double[])input.Clone();
        for (int y = 1; y < height - 1; y++)
        for (int x = 1; x < width - 1; x++)
        {
            double value = 0;
            for (int dy = -1; dy <= 1; dy++)
            for (int dx = -1; dx <= 1; dx++)
                value += input[(y + dy) * width + x + dx] * (dx == 0 ? 2 : 1) * (dy == 0 ? 2 : 1);
            output[y * width + x] = value / 16;
        }
        return output;
    }

    private static (double X, double Y, double Z) Normal(double[] field, int i, int width, double pitch)
    {
        double x = (field[i + 1] - field[i - 1]) / (2 * pitch), y = (field[i + width] - field[i - width]) / (2 * pitch);
        double length = Math.Sqrt(x * x + y * y + 1);
        return (x / length, y / length, 1 / length);
    }

    private sealed class StockScene
    {
        private readonly record struct Cut(Point3D A, Point3D B, int Owner, double FlatRadius,
            double Radius, double Depth);
        private sealed class Node
        {
            internal double MinX, MinY, MaxX, MaxY, Depth;
            internal int First, Count;
            internal Node? Left, Right;
        }
        private readonly Cut[] cuts;
        private readonly Node? root;
        private readonly CarvingCutterProfile profile;
        private readonly bool[] active;
        private readonly CancellationToken token;
        private long queries;
        internal long Evaluations { get; private set; }
        internal bool BudgetReached { get; private set; }
        internal bool Supported { get; }

        internal StockScene(Point3D[][] paths, VCarveCutterProfile cutter,
            IReadOnlyList<VCarveRoughingChord> roughing, bool[] active, CancellationToken token)
        {
            this.active = active; this.token = token; profile = cutter.PhysicalProfile;
            var inventory = new List<Cut>();
            for (int owner = 0; owner < paths.Length; owner++)
            {
                token.ThrowIfCancellationRequested();
                var row = paths[owner];
                if (!row.Any(p => p.X != row[0].X || p.Y != row[0].Y)) continue;
                for (int i = 1; i < row.Length; i++)
                {
                    double depth = Math.Max(-row[i - 1].Z, -row[i].Z);
                    if (depth <= 0) continue;
                    if (!Finite(row[i - 1]) || !Finite(row[i])) { cuts = Array.Empty<Cut>(); return; }
                    inventory.Add(new(row[i - 1], row[i], owner, 0, cutter.PhysicalRadiusAtDepth(depth), depth));
                    if (inventory.Count > 1_000_000) { cuts = Array.Empty<Cut>(); return; }
                }
            }
            foreach (var chord in roughing)
            {
                double depth = Math.Max(-chord.A.Z, -chord.B.Z);
                if (!(chord.StockSurfaceRadius > 0) || !double.IsFinite(chord.StockSurfaceRadius)
                    || chord.StockSurfaceRadius > 100 || !Finite(chord.A) || !Finite(chord.B))
                { cuts = Array.Empty<Cut>(); return; }
                if (depth > 0)
                    inventory.Add(new(chord.A, chord.B, -1, chord.StockSurfaceRadius, chord.StockSurfaceRadius, depth));
                if (inventory.Count > 1_000_000) { cuts = Array.Empty<Cut>(); return; }
            }
            cuts = inventory.ToArray();
            if (cuts.Length > 1_000_000) return;
            root = cuts.Length > 0 ? Build(0, cuts.Length) : null;
            Supported = true;
        }

        private static bool Finite(Point3D p) => double.IsFinite(p.X + p.Y + p.Z)
            && Math.Abs(p.X) <= 10000 && Math.Abs(p.Y) <= 10000 && Math.Abs(p.Z) <= 100;

        private Node Build(int first, int count)
        {
            token.ThrowIfCancellationRequested();
            var node = new Node { First = first, Count = count,
                MinX = double.PositiveInfinity, MinY = double.PositiveInfinity,
                MaxX = double.NegativeInfinity, MaxY = double.NegativeInfinity };
            for (int i = first; i < first + count; i++)
            {
                var c = cuts[i]; double r = c.Radius + 1e-8;
                node.MinX = Math.Min(node.MinX, Math.Min(c.A.X, c.B.X) - r);
                node.MinY = Math.Min(node.MinY, Math.Min(c.A.Y, c.B.Y) - r);
                node.MaxX = Math.Max(node.MaxX, Math.Max(c.A.X, c.B.X) + r);
                node.MaxY = Math.Max(node.MaxY, Math.Max(c.A.Y, c.B.Y) + r);
                node.Depth = Math.Max(node.Depth, c.Depth);
            }
            if (count > 12)
            {
                bool x = node.MaxX - node.MinX >= node.MaxY - node.MinY;
                Array.Sort(cuts, first, count, Comparer<Cut>.Create((a, b) =>
                    (x ? a.A.X + a.B.X : a.A.Y + a.B.Y).CompareTo(x ? b.A.X + b.B.X : b.A.Y + b.B.Y)));
                int half = count / 2;
                node.Left = Build(first, half); node.Right = Build(first + half, count - half);
            }
            return node;
        }

        internal Contact Sample(double x, double y, int omitted, bool original)
        {
            token.ThrowIfCancellationRequested();
            // Deterministic work limits keep cold-cache motion independent of
            // CPU load; exhausted evidence retains every unexamined visit.
            if (BudgetReached || ++queries > 4_000_000)
            { BudgetReached = true; return new(double.NaN, -1); }
            double best = 0; int bestOwner = -1;
            if (root is not null) Visit(root);
            return new(BudgetReached ? double.NaN : best, bestOwner);
            void Visit(Node node)
            {
                if (node.Depth <= best || x < node.MinX || x > node.MaxX || y < node.MinY || y > node.MaxY) return;
                if (node.Left is { } left && node.Right is { } right)
                {
                    if (left.Depth >= right.Depth) { Visit(left); Visit(right); }
                    else { Visit(right); Visit(left); }
                    return;
                }
                for (int i = node.First; i < node.First + node.Count; i++)
                {
                    var c = cuts[i];
                    if (c.Depth <= best || c.Owner >= 0 && (!original && !active[c.Owner] || c.Owner == omitted)) continue;
                    double reach = c.Radius + 1e-8;
                    if (x < Math.Min(c.A.X, c.B.X) - reach || x > Math.Max(c.A.X, c.B.X) + reach
                        || y < Math.Min(c.A.Y, c.B.Y) - reach || y > Math.Max(c.A.Y, c.B.Y) + reach) continue;
                    if (++Evaluations > 120_000_000) { BudgetReached = true; return; }
                    double z = c.FlatRadius > 0 ? Flat(c, x, y)
                        : CarvingCutterContactBounds.MinimumTipAndBottom(profile, new PointD(x, y),
                            new PointD(c.A.X, c.A.Y), c.A.Z, new PointD(c.B.X, c.B.Y), c.B.Z, 0);
                    if (double.IsNaN(z)) { BudgetReached = true; return; }
                    if (-z > best) { best = -z; bestOwner = c.Owner; }
                }
            }
        }

        private static double Flat(Cut c, double x, double y)
        {
            double vx = c.B.X - c.A.X, vy = c.B.Y - c.A.Y, ux = x - c.A.X, uy = y - c.A.Y;
            double length = vx * vx + vy * vy, radius2 = c.FlatRadius * c.FlatRadius;
            if (length == 0) return ux * ux + uy * uy <= radius2 ? Math.Min(c.A.Z, c.B.Z) : double.PositiveInfinity;
            double t = (ux * vx + uy * vy) / length;
            double perpendicular = Math.Max(0, ux * ux + uy * uy - t * t * length);
            if (perpendicular > radius2) return double.PositiveInfinity;
            double half = Math.Sqrt(Math.Max(0, (radius2 - perpendicular) / length));
            double lo = Math.Max(0, t - half), hi = Math.Min(1, t + half);
            if (lo > hi) return double.PositiveInfinity;
            double dz = c.B.Z - c.A.Z;
            return Math.Min(c.A.Z + lo * dz, c.A.Z + hi * dz);
        }
    }
}
