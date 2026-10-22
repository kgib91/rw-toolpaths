using Clipper2Lib;
using System.Diagnostics;

namespace RW.Toolpaths.Geometry;

/// <summary>One immutable canonical NonZero coverage operand. Whole-contour
/// bounds omit only paths with zero winding throughout a query; reusable input
/// retains every selected original integer vertex and its signed orientation.</summary>
internal sealed class PreparedResidualCoverage
{
    private readonly Paths64 _paths;
    private readonly Rect64[] _bounds;
    private readonly ReuseableDataContainer64?[] _inputs;
    private PolygonOps.BoundarySegmentIndex? _boundary;
    private readonly bool _supportsWitness;
    internal long OriginalVertices { get; }
    internal int OriginalContours => _paths.Count;
    internal int DifferenceQueries { get; private set; }
    internal long ConsideredVertices { get; private set; }
    internal long SelectedVertices { get; private set; }
    internal long SelectedContours { get; private set; }
    internal int FullSelectionQueries { get; private set; }
    internal int EmptySelectionQueries { get; private set; }
    internal int InputBuilds { get; private set; }
    internal int InputReuses { get; private set; }
    internal long BoundsTicks { get; private set; }
    internal long InputTicks { get; private set; }
    internal long ComponentPreparationTicks { get; private set; }
    internal long ComponentExecuteTicks { get; private set; }
    internal long ComponentMaterializeTicks { get; private set; }
    internal int WitnessQueries { get; private set; }
    internal int WitnessProbes { get; private set; }
    internal int FreshDiscProofs { get; private set; }
    internal long WitnessTicks { get; private set; }
    internal double LargestWitnessRadius { get; private set; }
    internal double LastWitnessRadius { get; private set; }

    internal PreparedResidualCoverage(Paths64 canonicalCoverage, CancellationToken token)
    {
        token.ThrowIfCancellationRequested();
        _paths = new Paths64();
        var bounds = new List<Rect64>();
        bool supportsWitness = true;
        foreach (var path in canonicalCoverage)
        {
            token.ThrowIfCancellationRequested();
            if (path.Count < 3) continue;
            var copy = new Path64(path.Count);
            long left = long.MaxValue, top = long.MaxValue, right = long.MinValue, bottom = long.MinValue;
            foreach (var point in path)
            {
                if ((copy.Count & 1023) == 0) token.ThrowIfCancellationRequested();
                copy.Add(point);
                left = Math.Min(left, point.X); right = Math.Max(right, point.X);
                top = Math.Min(top, point.Y); bottom = Math.Max(bottom, point.Y);
                supportsWitness &= point.X >= -1_000_000_000_000L && point.X <= 1_000_000_000_000L
                    && point.Y >= -1_000_000_000_000L && point.Y <= 1_000_000_000_000L;
            }
            _paths.Add(copy);
            bounds.Add(new Rect64(left, top, right, bottom));
            OriginalVertices += copy.Count;
        }
        _bounds = bounds.ToArray();
        _inputs = new ReuseableDataContainer64?[_paths.Count];
        _supportsWitness = supportsWitness;
        token.ThrowIfCancellationRequested();
    }

    internal List<List<PointD>> Difference(Paths64 canonicalSubject, CancellationToken token)
    {
        var engine = PrepareEngine(canonicalSubject, token, boundsGuard: 0);
        if (engine is null) return new List<List<PointD>>();
        var solution = new Paths64();
        if (!engine.Execute(ClipType.Difference, FillRule.NonZero, solution))
            throw new InvalidOperationException("Prepared exact coverage difference could not be completed.");
        token.ThrowIfCancellationRequested();
        return PolygonOps.NormalizeOrder(PolygonOps.MaterializeIntegerPaths(solution));
    }

    /// <summary>Subtracts the same immutable signed contours while retaining the
    /// original PolyTree traversal, component order and cycle starts. A one-tick
    /// bounds guard only includes additional complete clip contours; no contour
    /// geometry or signed winding is changed.</summary>
    internal List<PolygonComponent> DifferenceComponents(Paths64 canonicalSubject, CancellationToken token)
    {
        long started = Stopwatch.GetTimestamp();
        Clipper64? engine;
        try { engine = PrepareEngine(canonicalSubject, token, boundsGuard: 1); }
        finally { ComponentPreparationTicks += Stopwatch.GetTimestamp() - started; }
        if (engine is null) return new List<PolygonComponent>();
        token.ThrowIfCancellationRequested();
        var tree = new PolyTree64();
        started = Stopwatch.GetTimestamp();
        try
        {
            RequireSuccessfulComponentDifference(engine.Execute(ClipType.Difference, FillRule.NonZero, tree), tree);
        }
        finally { ComponentExecuteTicks += Stopwatch.GetTimestamp() - started; }
        token.ThrowIfCancellationRequested();
        started = Stopwatch.GetTimestamp();
        try { return PolygonOps.MaterializeComponents(tree, token); }
        finally { ComponentMaterializeTicks += Stopwatch.GetTimestamp() - started; }
    }

    internal static void RequireSuccessfulComponentDifference(bool success, PolyTree64 tree)
    {
        if (success) return;
        tree.Clear();
        throw new InvalidOperationException("Prepared protection component difference could not be completed; partial components were discarded.");
    }

    private Clipper64? PrepareEngine(Paths64 canonicalSubject, CancellationToken token, int boundsGuard)
    {
        token.ThrowIfCancellationRequested();
        var subjects = new Paths64(canonicalSubject.Where(path => path.Count >= 3));
        if (subjects.Count == 0) return null;
        long boundsStarted = Stopwatch.GetTimestamp();
        Rect64 query = Clipper.GetBounds(subjects);
        var selected = new List<int>();
        for (int i = 0; i < _paths.Count; i++)
        {
            token.ThrowIfCancellationRequested();
            var bounds = _bounds[i];
            if (Separated(bounds.left, query.right) || Separated(query.left, bounds.right)
                || Separated(bounds.top, query.bottom) || Separated(query.top, bounds.bottom)) continue;
            selected.Add(i);
        }
        BoundsTicks += Stopwatch.GetTimestamp() - boundsStarted;
        DifferenceQueries++;
        ConsideredVertices += OriginalVertices;
        SelectedContours += selected.Count;
        if (selected.Count == _paths.Count) FullSelectionQueries++;
        if (selected.Count == 0) EmptySelectionQueries++;
        var engine = new Clipper64();
        token.ThrowIfCancellationRequested();
        engine.AddSubject(subjects);
        token.ThrowIfCancellationRequested();
        foreach (int i in selected)
        {
            token.ThrowIfCancellationRequested();
            SelectedVertices += _paths[i].Count;
            var prepared = _inputs[i];
            if (prepared is null)
            {
                long inputStarted = Stopwatch.GetTimestamp();
                prepared = new ReuseableDataContainer64();
                prepared.AddPaths(new Paths64 { _paths[i] }, PathType.Clip, false);
                token.ThrowIfCancellationRequested();
                _inputs[i] = prepared;
                InputBuilds++;
                InputTicks += Stopwatch.GetTimestamp() - inputStarted;
            }
            else InputReuses++;
            // Original contour order and orientation matter for NonZero holes.
            // Enclosing contours meet the complete box even when their edges do not.
            engine.AddReuseableData(prepared);
            token.ThrowIfCancellationRequested();
        }
        token.ThrowIfCancellationRequested();
        return engine;

        bool Separated(long lower, long upper)
            => lower > upper && (Int128)lower - upper > boundsGuard;
    }

    /// <summary>Disc discovery proves positive uncovered material only. Every
    /// accepted center has its complete disc inside the signed candidate and
    /// outside complete captured coverage, including enclosed holes. Missing a
    /// witness leaves the original subtraction/erosion policy in charge.</summary>
    internal bool TryFreshDisc(Paths64 canonicalCandidate, double coverageError,
        double proofArc, CancellationToken token, out Point64 witness)
    {
        token.ThrowIfCancellationRequested();
        long started = Stopwatch.GetTimestamp();
        witness = default;
        WitnessQueries++;
        try
        {
            if (!_supportsWitness || canonicalCandidate.Count == 0
                || !double.IsFinite(coverageError) || coverageError < 0
                || !double.IsFinite(proofArc) || proofArc <= 0) return false;
            foreach (var path in canonicalCandidate)
            foreach (var point in path)
            {
                token.ThrowIfCancellationRequested();
                if (point.X < -1_000_000_000_000L || point.X > 1_000_000_000_000L
                    || point.Y < -1_000_000_000_000L || point.Y > 1_000_000_000_000L) return false;
            }
            // Same conservative fresh-material contract as FilledFrontCoverage:
            // erosion radius, full proof-arc reserve, and eight native quanta per
            // candidate contour plus eight for conversion/Boolean construction.
            // The captured coverage itself is exact; unrelated historical unions
            // introduce no new reserve here. ContainsCapsule adds four more ticks.
            double radius = coverageError + proofArc + (8d * canonicalCandidate.Count + 8) / PathUtils.Scale;
            LastWitnessRadius = radius;
            LargestWitnessRadius = Math.Max(LargestWitnessRadius, radius);
            double nativeRadius = radius * PathUtils.Scale + 4;
            _boundary ??= new PolygonOps.BoundarySegmentIndex(_paths);
            token.ThrowIfCancellationRequested();
            var candidateBoundary = new PolygonOps.BoundarySegmentIndex(canonicalCandidate);
            token.ThrowIfCancellationRequested();
            foreach (var path in canonicalCandidate)
            {
                if (path.Count < 3 || !Clipper.IsPositive(path)) continue;
                var bounds = Clipper.GetBounds(path);
                var center = new Point64(bounds.left + (bounds.right - bounds.left) / 2,
                    bounds.top + (bounds.bottom - bounds.top) / 2);
                if (Proves(center)) { witness = center; FreshDiscProofs++; return true; }
                // Discovery probes the interior side of original edges. Winding
                // and complete-disc distance establish authority, not the probes.
                int count = Math.Min(16, path.Count);
                for (int sample = 0; sample < count; sample++)
                {
                    int first = (int)((long)sample * path.Count / count);
                    var a = path[first]; var b = path[(first + 1) % path.Count];
                    double dx = (double)b.X - a.X, dy = (double)b.Y - a.Y;
                    double length = Math.Sqrt(dx * dx + dy * dy);
                    if (length == 0 || !double.IsFinite(length)) continue;
                    double inset = 2 * nativeRadius;
                    var probe = new Point64((long)Math.Round(a.X + dx / 2 - dy / length * inset),
                        (long)Math.Round(a.Y + dy / 2 + dx / length * inset));
                    if (Proves(probe)) { witness = probe; FreshDiscProofs++; return true; }
                }
            }
            return false;

            bool Proves(Point64 point)
            {
                token.ThrowIfCancellationRequested();
                WitnessProbes++;
                // The outside test and distance include every signed boundary;
                // a candidate enclosing a virgin hole cannot claim that hole cut.
                return !_boundary.IsInside(point)
                    && _boundary.DistanceSquared(point) > nativeRadius * nativeRadius
                    && candidateBoundary.ContainsCapsule(point, point, radius * PathUtils.Scale);
            }
        }
        finally { WitnessTicks += Stopwatch.GetTimestamp() - started; }
    }
}
