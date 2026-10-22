using Clipper2Lib;
using RW.Toolpaths.Geometry;
using System.Diagnostics;

namespace RW.Toolpaths.Milling;

/// <summary>Zero-lateral prior-stepdown authority from actually executed F3 chords.</summary>
internal sealed class PocketPostedStepdownStock
{
    private const int BatchSize = 2048;
    // This shrinks stock evidence; it never grants a machining allowance.
    // A clipped endpoint conversion, round-cap vertex conversion, offset union
    // and explicit union each move a vertex by less than one XY integer quantum.
    // Sixteen quanta conservatively cover both axes and these four stages.
    internal readonly record struct PostedPoint(long X, long Y, long Z);
    private readonly record struct LineKey(long X, long Y, Int128 Intercept);
    private readonly record struct Interval(long Low, long High, long LowZ, long HighZ, int Ordinal);
    private readonly record struct Chord(PostedPoint A, PostedPoint B, int Ordinal)
    {
        internal long MinX => Math.Min(A.X, B.X);
        internal long MinY => Math.Min(A.Y, B.Y);
        internal long MaxX => Math.Max(A.X, B.X);
        internal long MaxY => Math.Max(A.Y, B.Y);
    }
    private readonly Dictionary<LineKey, List<Interval>> _lines = new();
    private readonly Dictionary<(long X, long Y), List<(long Z, int Ordinal)>> _points = new();
    private readonly List<Chord> _pending = new();
    private readonly List<Batch> _batches = new();
    private readonly float _surface;
    private readonly long _rise, _startingFloor;
    private readonly double _radius;
    private readonly bool _permitFlatFootprintProof;
    private readonly bool _exactRectangleEnabled;
    private readonly bool _joinFullChords;
    private readonly bool _cachedInnerFirst;
    private readonly bool _preparedNegativeArithmetic;
    private readonly CancellationToken _token;
    private int _recorded;
    private PostedPoint? _cursor;
    private Point3D? _nominalEnd;
    internal long Queries { get; private set; }
    internal long SurfaceProofs { get; private set; }
    internal long ExactProofs { get; private set; }
    internal long PhysicalProofs { get; private set; }
    internal long Refusals { get; private set; }
    internal long CandidateChords { get; private set; }
    internal long InnerProofBuilds { get; private set; }
    internal long ExactRectangleQueries { get; private set; }
    internal long ExactRectangleProofs { get; private set; }
    internal long ExactRectangleTriangles { get; private set; }
    internal long ExactRectangleDistanceChecks { get; private set; }
    internal long ExactRectangleRefinements { get; private set; }
    internal long ExactRectangleBudgetRefusals { get; private set; }
    internal long ExactRectangleNegativeCandidates { get; private set; }
    internal long ExactRectangleNegativeProbes { get; private set; }
    internal long ExactRectangleNegativeWitnesses { get; private set; }
    internal long ExactRectangleNegativeDistanceChecks { get; private set; }
    internal long ExactRectangleNegativeBudgetRefusals { get; private set; }
    internal long ExactRectangleNegativePreparedVertices { get; private set; }
    internal long ExactRectangleNegativeAxisProofs { get; private set; }
    internal long ExactRectangleNegativeFullDistanceChecks { get; private set; }
    internal bool PreparedNegativeArithmeticEnabled => _preparedNegativeArithmetic;
    internal long ProofMemoHits { get; private set; }
    internal long ProofMemoBypasses { get; private set; }
    internal long ProofMemoEvictions { get; private set; }
    internal long ProofMemoPeakEntries { get; private set; }
    internal long CachedInnerQueries { get; private set; }
    internal long CachedInnerProofs { get; private set; }
    internal long CachedInnerMisses { get; private set; }
    internal long InnerCapsuleQueries { get; private set; }
    internal long InnerFullInputChords { get; private set; }
    internal long InnerUniqueFullChords { get; private set; }
    internal long InnerDuplicateChords { get; private set; }
    internal long InnerJoinedPaths { get; private set; }
    internal long InnerJoinedPoints { get; private set; }
    internal long InnerClosedCycles { get; private set; }
    internal long InnerBranchVertices { get; private set; }
    internal long InnerPartialChords { get; private set; }
    internal long HeightOnlyCursorCarries { get; private set; }
    internal long PlanarBridgeCursorAnchors { get; private set; }
    internal double ExactRectangleMilliseconds { get; private set; }
    internal double ExactRectangleNegativeMilliseconds { get; private set; }
    internal double InnerPrepareMilliseconds { get; private set; }
    internal double InnerCapsuleMilliseconds { get; private set; }
    internal double ProofMilliseconds { get; private set; }
    internal double RecordMilliseconds { get; private set; }

    internal PocketPostedStepdownStock(DepthSchedule depth, double radius, CancellationToken token,
        bool permitFlatFootprintProof = true, bool permitExactRectangle = true, bool joinFullChords = true,
        bool permitCachedInnerFirst = true, bool permitPreparedNegativeArithmetic = true)
    {
        _surface = (float)depth.SurfaceZ;
        double step = depth.DepthPerPass > 0 ? depth.DepthPerPass : depth.Depth - depth.StartDepth;
        if (!double.IsFinite(step) || step < 0 || !double.IsFinite(_surface))
            throw new ArgumentOutOfRangeException(nameof(depth));
        _rise = checked((long)Math.Floor(step * 1000));
        _startingFloor = Posted(new Point3D(0, 0, depth.SurfaceZ - depth.StartDepth)).Z;
        _radius = radius;
        _permitFlatFootprintProof = permitFlatFootprintProof;
        _exactRectangleEnabled = permitExactRectangle
            && Environment.GetEnvironmentVariable("RW_TOOLPATHS_PRIOR_CAPSULE_REFERENCE") != "1";
        _joinFullChords = joinFullChords
            && Environment.GetEnvironmentVariable("RW_TOOLPATHS_PRIOR_SWEEP_JOIN_REFERENCE") != "1";
        _cachedInnerFirst = permitCachedInnerFirst
            && Environment.GetEnvironmentVariable("RW_TOOLPATHS_PRIOR_INNER_ORDER_REFERENCE") != "1";
        // Capture the arithmetic mode once per stock instance. It changes no
        // selected capsules, query order, certificate limits or stock authority.
        _preparedNegativeArithmetic = permitPreparedNegativeArithmetic
            && Environment.GetEnvironmentVariable("RW_TOOLPATHS_PRIOR_NEGATIVE_ARITHMETIC_REFERENCE") != "1";
        _token = token;
    }

    internal PostedPoint Posted(Point3D point)
    {
        static long Grid(float value)
        {
            if (!float.IsFinite(value) || Math.Abs(value) > 1e9)
                throw new InvalidOperationException("Pocket posted stock requires finite command coordinates.");
            return checked((long)Math.Round((double)value * 1000, MidpointRounding.ToEven));
        }
        // Shared casts native coordinates to binary32 before IlProgramBuilder
        // subtracts the binary32 stock surface and emits F3 with ties to even.
        return new PostedPoint(Grid((float)point.X), Grid((float)point.Y), Grid((float)point.Z - _surface));
    }

    internal void Record(IReadOnlyList<Point3D> stroke)
    {
        long started = Stopwatch.GetTimestamp();
        try
        {
            _token.ThrowIfCancellationRequested();
            if (stroke.Count > 0 && (_nominalEnd is null || !Continuous(_nominalEnd.Value, stroke[0])))
            {
                var start = Posted(stroke[0]);
                if (_nominalEnd is { } previous && _cursor is { } actual && !PlanarBridge(previous, stroke[0]))
                {
                    // Shared GenerateBridgeSegments emits a planar safe-plane
                    // traverse ONLY when either nominal XY axis changes by
                    // more than1e-4 binary32. A height-only retract/approach
                    // carries any prior omitted-XY drift; it cannot reanchor
                    // the machine at this stroke's unposted nominal XY.
                    _cursor = start with { X = actual.X, Y = actual.Y };
                    HeightOnlyCursorCarries++;
                }
                else
                {
                    // A generated safe-plane traverse has dy=0, hence emits
                    // XYZ even when its planar run is at most.001mm.
                    _cursor = start;
                    if (_nominalEnd is not null) PlanarBridgeCursorAnchors++;
                }
            }
            for (int index = 1; index < stroke.Count; index++)
            {
                if ((index & 255) == 0) _token.ThrowIfCancellationRequested();
                var a = _cursor ?? Posted(stroke[index - 1]);
                var b = Posted(stroke[index]);
                // Pure vertical Shared commands omit both planar axes. Carry
                // their ACTUAL posted cursor into the following XYZ command;
                // a nominal source vertex is not executed stock authority.
                float dx = (float)stroke[index].X - (float)stroke[index - 1].X;
                float dy = (float)stroke[index].Y - (float)stroke[index - 1].Y;
                float dz = (float)stroke[index].Z - (float)stroke[index - 1].Z;
                float run = MathF.Sqrt(dx * dx + dy * dy);
                if (MathF.Abs(dz) > .001f && run <= .001f) b = b with { X = a.X, Y = a.Y };
                _cursor = b;
                _nominalEnd = stroke[index];
                int ordinal = _recorded++;
                Remember(a, ordinal);
                Remember(b, ordinal);
                if (Math.Min(a.Z, b.Z) >= 0) continue;
                if (Key(a, b) is { } key)
                {
                    if (!_lines.TryGetValue(key, out var intervals)) _lines.Add(key, intervals = new());
                    long low = key.X != 0 ? Math.Min(a.X, b.X) : Math.Min(a.Y, b.Y);
                    long high = key.X != 0 ? Math.Max(a.X, b.X) : Math.Max(a.Y, b.Y);
                    bool aFirst = key.X != 0 ? a.X <= b.X : a.Y <= b.Y;
                    intervals.Add(new Interval(low, high, aFirst ? a.Z : b.Z, aFirst ? b.Z : a.Z, ordinal));
                }
                _pending.Add(new Chord(a, b, ordinal));
                if (_pending.Count >= BatchSize)
                {
                    _batches.Add(new Batch(_pending.ToArray()));
                    _pending.Clear();
                }
            }
        }
        finally { RecordMilliseconds += Stopwatch.GetElapsedTime(started).TotalMilliseconds; }

        void Remember(PostedPoint point, int ordinal)
        {
            if (!_points.TryGetValue((point.X, point.Y), out var heights))
                _points.Add((point.X, point.Y), heights = new());
            heights.Add((point.Z, ordinal));
        }

        static bool Continuous(Point3D a, Point3D b) => MathF.Abs((float)a.X - (float)b.X) <= 1e-4f
            && MathF.Abs((float)a.Y - (float)b.Y) <= 1e-4f && MathF.Abs((float)a.Z - (float)b.Z) <= 1e-4f;
        static bool PlanarBridge(Point3D a, Point3D b) => MathF.Abs((float)a.X - (float)b.X) > 1e-4f
            || MathF.Abs((float)a.Y - (float)b.Y) > 1e-4f;
    }

    internal Proof Capture() => new(this, _recorded);

    internal sealed class Proof
    {
        private readonly PocketPostedStepdownStock _owner;
        private readonly int _prefix;
        private readonly PostedPoint? _cursor, _nominalEnd;
        private readonly record struct MemoResult((PostedPoint A, PostedPoint B) Key, bool Safe, byte Kind);
        private readonly Dictionary<(PostedPoint A, PostedPoint B), LinkedListNode<MemoResult>> _memo = new();
        private readonly LinkedList<MemoResult> _memoOrder = new();
        private PolygonOps.BoundarySegmentIndex? _inner;
        private (double MinX, double MinY, double MaxX, double MaxY) _window;
        internal Proof(PocketPostedStepdownStock owner, int prefix)
        {
            _owner = owner; _prefix = prefix; _cursor = owner._cursor;
            _nominalEnd = owner._nominalEnd is { } end ? owner.Posted(end) : null;
        }
        private long? _threshold;

        internal bool IsSafe(Point3D from, Point3D to)
        {
            long started = Stopwatch.GetTimestamp();
            double alreadyMeasured = _owner.ProofMilliseconds;
            try
            {
                _owner._token.ThrowIfCancellationRequested();
                var key = (_owner.Posted(from), _owner.Posted(to));
                if (_memo.TryGetValue(key, out var cached))
                {
                    _memoOrder.Remove(cached);
                    _memoOrder.AddLast(cached);
                    _owner.Queries++; _owner.ProofMemoHits++;
                    if (!cached.Value.Safe) _owner.Refusals++;
                    else if (cached.Value.Kind == 1) _owner.SurfaceProofs++;
                    else if (cached.Value.Kind == 2) _owner.ExactProofs++;
                    else _owner.PhysicalProofs++;
                    return cached.Value.Safe;
                }
                long surfaces = _owner.SurfaceProofs, exacts = _owner.ExactProofs;
                bool safe = IsSafeCore(from, to);
                byte kind = !safe ? (byte)0 : _owner.SurfaceProofs > surfaces ? (byte)1
                    : _owner.ExactProofs > exacts ? (byte)2 : (byte)3;
                // This Proof captures one immutable prefix, actual cursor and
                // radius. Ordered full F3 XYZ endpoints completely identify
                // its stock query. Only completed answers enter a bounded memo.
                LinkedListNode<MemoResult> node;
                if (_memo.Count < 128)
                {
                    node = new LinkedListNode<MemoResult>(new MemoResult(key, safe, kind));
                    _owner.ProofMemoPeakEntries = Math.Max(_owner.ProofMemoPeakEntries, _memo.Count + 1);
                }
                else
                {
                    // Reuse the evicted node: later repair proposals receive
                    // the same bounded memo without per-miss node allocations.
                    node = _memoOrder.First!;
                    _memoOrder.RemoveFirst();
                    _memo.Remove(node.Value.Key);
                    node.Value = new(key, safe, kind);
                    _owner.ProofMemoEvictions++;
                }
                _memo.Add(key, node);
                _memoOrder.AddLast(node);
                return safe;
            }
            finally
            {
                // Core already owns its elapsed work; add only the surrounding
                // lookup/record work so priorStepdownMs stays an exclusive owner.
                _owner.ProofMilliseconds += Math.Max(0, Stopwatch.GetElapsedTime(started).TotalMilliseconds
                    - (_owner.ProofMilliseconds - alreadyMeasured));
            }
        }

        private bool IsSafeCore(Point3D from, Point3D to)
        {
            long started = Stopwatch.GetTimestamp();
            try
            {
                var owner = _owner;
                owner._token.ThrowIfCancellationRequested();
                owner.Queries++;
                var a = owner.Posted(from);
                var b = owner.Posted(to);
                if (a.Z != b.Z) { owner.Refusals++; return false; }
                long threshold = a.Z + owner._rise;
                if (threshold >= owner._startingFloor)
                { owner.SurfaceProofs++; return true; }
                if (_nominalEnd is { } nominal && _cursor is { } cursor && a == nominal
                    && (a.X != cursor.X || a.Y != cursor.Y))
                {
                    // A pending Link begins at the real cursor, not at its
                    // omitted nominal vertex. Refuse this native proposal so
                    // the established command-vertex reentry reanchors it.
                    owner.Refusals++; return false;
                }
                if (owner.Exact(a, b, threshold, _prefix))
                { owner.ExactProofs++; return true; }
                // Same posted-chord depth dominance is valid for every radial
                // profile. A two-dimensional constant-radius stock section is
                // complete physical authority only for a flat cylinder.
                if (!owner._permitFlatFootprintProof) { owner.Refusals++; return false; }
                double minX = Math.Min(a.X, b.X) / 1000d, minY = Math.Min(a.Y, b.Y) / 1000d;
                double maxX = Math.Max(a.X, b.X) / 1000d, maxY = Math.Max(a.Y, b.Y) / 1000d;
                bool compatibleInner = _inner is not null && _threshold == threshold
                    && minX >= _window.MinX && minY >= _window.MinY
                    && maxX <= _window.MaxX && maxY <= _window.MaxY;
                bool checkedInner = false;
                if (owner._cachedInnerFirst && compatibleInner)
                {
                    // This index is an INNER subset of this Proof's captured
                    // actual prefix. Only a complete positive capsule proof
                    // can skip exact subdivision; a miss retains every fallback.
                    owner.CachedInnerQueries++;
                    checkedInner = true;
                    if (ContainsInner(a, b))
                    {
                        owner.CachedInnerProofs++; owner.PhysicalProofs++; return true;
                    }
                    owner.CachedInnerMisses++;
                }
                if (owner._exactRectangleEnabled && owner.Endpoint(a, threshold, _prefix)
                    && owner.Endpoint(b, threshold, _prefix))
                {
                    long rectangleStarted = Stopwatch.GetTimestamp();
                    owner.ExactRectangleQueries++;
                    var segments = owner.EarlierConstantCapsules(a, b, threshold, _prefix);
                    var rectangle = PocketExactCapsuleCoverage.Prove(new(a.X, a.Y, b.X, b.Y), segments,
                        owner._radius, owner._token, usePreparedNegativeArithmetic: owner._preparedNegativeArithmetic);
                    owner.ExactRectangleTriangles += rectangle.Triangles;
                    owner.ExactRectangleDistanceChecks += rectangle.DistanceChecks;
                    owner.ExactRectangleRefinements += rectangle.Refinements;
                    if (rectangle.BudgetExhausted) owner.ExactRectangleBudgetRefusals++;
                    owner.ExactRectangleNegativeCandidates += rectangle.NegativeCandidates;
                    owner.ExactRectangleNegativeProbes += rectangle.NegativeProbes;
                    owner.ExactRectangleNegativeWitnesses += rectangle.NegativeWitnesses;
                    owner.ExactRectangleNegativeDistanceChecks += rectangle.NegativeDistanceChecks;
                    owner.ExactRectangleNegativeBudgetRefusals += rectangle.NegativeBudgetRefusals;
                    owner.ExactRectangleNegativePreparedVertices += rectangle.NegativePreparedVertices;
                    owner.ExactRectangleNegativeAxisProofs += rectangle.NegativeAxisProofs;
                    owner.ExactRectangleNegativeFullDistanceChecks += rectangle.NegativeFullDistanceChecks;
                    owner.ExactRectangleNegativeMilliseconds += rectangle.NegativeMilliseconds;
                    owner.ExactRectangleMilliseconds += Stopwatch.GetElapsedTime(rectangleStarted).TotalMilliseconds;
                    if (rectangle.Proven)
                    {
                        // Each endpoint disc is exactly an earlier actual
                        // physical cutter pose. The rectangle's finite convex
                        // triangle witnesses cover the remaining whole capsule.
                        owner.ExactRectangleProofs++; owner.PhysicalProofs++; return true;
                    }
                }
                if (!compatibleInner)
                {
                    long prepareStarted = Stopwatch.GetTimestamp();
                    try
                    {
                        double pad = 2 * owner._radius;
                        var window = (minX - pad, minY - pad, maxX + pad, maxY + pad);
                        var lines = owner.Nearby(window, threshold, _prefix);
                        var inner = PostedSweepInnerRegion.Build(lines, owner._radius, owner._token);
                        _inner = inner; _window = window; _threshold = threshold;
                        owner.InnerProofBuilds++;
                    }
                    finally { owner.InnerPrepareMilliseconds += Stopwatch.GetElapsedTime(prepareStarted).TotalMilliseconds; }
                }
                if (!checkedInner && ContainsInner(a, b))
                { owner.PhysicalProofs++; return true; }
                owner.Refusals++;
                return false;
            }
            finally { _owner.ProofMilliseconds += Stopwatch.GetElapsedTime(started).TotalMilliseconds; }
        }

        private bool ContainsInner(PostedPoint a, PostedPoint b)
        {
            long started = Stopwatch.GetTimestamp();
            try
            {
                _owner._token.ThrowIfCancellationRequested();
                _owner.InnerCapsuleQueries++;
                var start = Quantize(a.X / 1000d, a.Y / 1000d);
                var end = Quantize(b.X / 1000d, b.Y / 1000d);
                // ContainsCapsule adds four integer ticks to the OUTER query.
                // Earlier stock is INNER and receives no lateral dilation.
                return _inner!.ContainsCapsule(start, end, _owner._radius * PathUtils.Scale);
            }
            finally { _owner.InnerCapsuleMilliseconds += Stopwatch.GetElapsedTime(started).TotalMilliseconds; }
        }
    }

    private bool Endpoint(PostedPoint point, long threshold, int prefix)
        => _points.TryGetValue((point.X, point.Y), out var heights)
            && heights.Any(old => old.Ordinal < prefix && old.Z <= threshold);

    private IReadOnlyList<PocketExactCapsuleCoverage.Segment> EarlierConstantCapsules(
        PostedPoint a, PostedPoint b, long threshold, int prefix)
    {
        // A bounded SUBSET of genuine old capsules is sufficient for a positive
        // certificate. Omitted chords and partial ramp intervals grant nothing.
        long pad = (long)Math.Ceiling(2 * _radius * 1000);
        long minX = Math.Min(a.X, b.X) - pad, maxX = Math.Max(a.X, b.X) + pad;
        long minY = Math.Min(a.Y, b.Y) - pad, maxY = Math.Max(a.Y, b.Y) + pad;
        var result = new List<PocketExactCapsuleCoverage.Segment>();
        foreach (var batch in _batches) batch.Visit(minX, minY, maxX, maxY, Add, _token);
        foreach (var chord in _pending)
            if (chord.MaxX >= minX && chord.MinX <= maxX && chord.MaxY >= minY && chord.MinY <= maxY) Add(chord);
        return result;
        void Add(Chord chord)
        {
            if (result.Count >= 256 || chord.Ordinal >= prefix || chord.A.Z != chord.B.Z
                || chord.A.Z > threshold) return;
            result.Add(new(chord.A.X, chord.A.Y, chord.B.X, chord.B.Y));
        }
    }

    private bool Exact(PostedPoint a, PostedPoint b, long threshold, int prefix)
    {
        if (Key(a, b) is not { } key)
            return _points.TryGetValue((a.X, a.Y), out var heights)
                && heights.Any(point => point.Ordinal < prefix && point.Z <= threshold);
        if (!_lines.TryGetValue(key, out var intervals)) return false;
        long low = key.X != 0 ? Math.Min(a.X, b.X) : Math.Min(a.Y, b.Y);
        long high = key.X != 0 ? Math.Max(a.X, b.X) : Math.Max(a.Y, b.Y);
        var eligible = new List<(long Low, long High)>();
        foreach (var interval in intervals)
        {
            if (interval.Ordinal >= prefix || Math.Min(interval.LowZ, interval.HighZ) > threshold) continue;
            long first = interval.Low, last = interval.High;
            // Restrict a sloped older F3 chord to its exact rational prior-floor
            // interval. Round the bounds inward to integer query coordinates;
            // no interpolated XY contact or lateral tolerance is manufactured.
            if (interval.LowZ > threshold)
            {
                Int128 numerator = (Int128)(interval.LowZ - threshold) * (interval.High - interval.Low);
                long denominator = interval.LowZ - interval.HighZ;
                first += (long)((numerator + denominator - 1) / denominator);
            }
            else if (interval.HighZ > threshold)
            {
                Int128 numerator = (Int128)(threshold - interval.LowZ) * (interval.High - interval.Low);
                last = first + (long)(numerator / (interval.HighZ - interval.LowZ));
            }
            if (first <= last) eligible.Add((first, last));
        }
        eligible.Sort((a, b) => a.Low.CompareTo(b.Low));
        foreach (var interval in eligible)
        {
            if (interval.High < low) continue;
            if (interval.Low > low) return false;
            low = Math.Max(low, interval.High);
            if (low >= high) return true;
        }
        return false;
    }

    private static LineKey? Key(PostedPoint a, PostedPoint b)
    {
        long x = b.X - a.X, y = b.Y - a.Y;
        long first = Math.Abs(x), second = Math.Abs(y);
        while (second != 0) { long remainder = first % second; first = second; second = remainder; }
        if (first == 0) return null;
        x /= first; y /= first;
        if (x < 0 || x == 0 && y < 0) { x = -x; y = -y; }
        return new LineKey(x, y, (Int128)a.X * y - (Int128)a.Y * x);
    }

    private Paths64 Nearby((double MinX, double MinY, double MaxX, double MaxY) window,
        long threshold, int prefix)
    {
        // Every omitted chord has a complete AABB outside this local center
        // window plus two radii. This proof is a subset of earlier stock even
        // when some omitted chord could help; rejection grants no authority.
        long pad = (long)Math.Ceiling(2 * _radius * 1000);
        long minX = (long)Math.Floor(window.MinX * 1000) - pad;
        long minY = (long)Math.Floor(window.MinY * 1000) - pad;
        long maxX = (long)Math.Ceiling(window.MaxX * 1000) + pad;
        long maxY = (long)Math.Ceiling(window.MaxY * 1000) + pad;
        var result = new Paths64();
        var full = new List<PocketPostedChordChains.Segment>();
        foreach (var batch in _batches) batch.Visit(minX, minY, maxX, maxY, Add, _token);
        foreach (var chord in _pending)
            if (chord.MaxX >= minX && chord.MinX <= maxX && chord.MaxY >= minY && chord.MinY <= maxY) Add(chord);
        InnerFullInputChords += full.Count;
        if (_joinFullChords)
        {
            var chains = PocketPostedChordChains.Build(full, _token);
            InnerUniqueFullChords += chains.UniqueEdges;
            InnerDuplicateChords += chains.DuplicateEdges;
            InnerJoinedPaths += chains.Paths.Count;
            InnerJoinedPoints += chains.Paths.Sum(path => path.Count);
            InnerClosedCycles += chains.ClosedCycles;
            InnerBranchVertices += chains.BranchVertices;
            foreach (var chain in chains.Paths)
                result.Add(new Path64(chain.Select(point => Quantize(point.X / 1000d, point.Y / 1000d))));
        }
        else
        {
            InnerUniqueFullChords += full.Count;
            InnerJoinedPaths += full.Count;
            InnerJoinedPoints += full.Count * 2;
            foreach (var segment in full)
                result.Add(new Path64 { Quantize(segment.A.X / 1000d, segment.A.Y / 1000d),
                    Quantize(segment.B.X / 1000d, segment.B.Y / 1000d) });
        }
        return result;

        void Add(Chord chord)
        {
            if (chord.Ordinal >= prefix || Math.Min(chord.A.Z, chord.B.Z) > threshold) return;
            _token.ThrowIfCancellationRequested();
            CandidateChords++;
            if (Math.Max(chord.A.Z, chord.B.Z) <= threshold)
            {
                if (chord.A.X != chord.B.X || chord.A.Y != chord.B.Y)
                    full.Add(new(new(chord.A.X, chord.A.Y), new(chord.B.X, chord.B.Y)));
                return;
            }
            InnerPartialChords++;
            double low = 0, high = 1;
            if (chord.A.Z > threshold) low = (double)(chord.A.Z - threshold) / (chord.A.Z - chord.B.Z);
            else if (chord.B.Z > threshold) high = (double)(threshold - chord.A.Z) / (chord.B.Z - chord.A.Z);
            Point64 Point(double t) => Quantize((chord.A.X + (chord.B.X - chord.A.X) * t) / 1000,
                (chord.A.Y + (chord.B.Y - chord.A.Y) * t) / 1000);
            var a = Point(low); var b = Point(high);
            if (a == b)
            {
                // A round-ended one-point path is not uniformly handled by
                // ClipperOffset. Rejecting that evidence is conservative; the
                // exact stationary-pose dictionary handles true coincidences.
                return;
            }
            result.Add(new Path64 { a, b });
        }
    }

    private static Point64 Quantize(double x, double y) => new(
        (long)Math.Round(x * PathUtils.Scale), (long)Math.Round(y * PathUtils.Scale));

    private sealed class Batch
    {
        private sealed record Node(long MinX, long MinY, long MaxX, long MaxY, int Start, int Count,
            Node? First = null, Node? Second = null);
        private readonly Chord[] _chords;
        private readonly Node _root;
        internal Batch(Chord[] chords) { _chords = chords; _root = Build(0, chords.Length); }
        private Node Build(int start, int count)
        {
            long minX = long.MaxValue, minY = long.MaxValue, maxX = long.MinValue, maxY = long.MinValue;
            for (int i = start; i < start + count; i++)
            {
                minX = Math.Min(minX, _chords[i].MinX); minY = Math.Min(minY, _chords[i].MinY);
                maxX = Math.Max(maxX, _chords[i].MaxX); maxY = Math.Max(maxY, _chords[i].MaxY);
            }
            if (count <= 12) return new(minX, minY, maxX, maxY, start, count);
            bool x = maxX - minX >= maxY - minY;
            Array.Sort(_chords, start, count, Comparer<Chord>.Create((a, b) => x
                ? (a.MinX + a.MaxX).CompareTo(b.MinX + b.MaxX)
                : (a.MinY + a.MaxY).CompareTo(b.MinY + b.MaxY)));
            int first = count / 2;
            return new(minX, minY, maxX, maxY, 0, 0, Build(start, first), Build(start + first, count - first));
        }
        internal void Visit(long minX, long minY, long maxX, long maxY, Action<Chord> visit, CancellationToken token)
        {
            VisitNode(_root);
            void VisitNode(Node node)
            {
                token.ThrowIfCancellationRequested();
                if (node.MaxX < minX || node.MinX > maxX || node.MaxY < minY || node.MinY > maxY) return;
                if (node.Count == 0) { VisitNode(node.First!); VisitNode(node.Second!); return; }
                for (int i = node.Start; i < node.Start + node.Count; i++)
                {
                    var chord = _chords[i];
                    if (chord.MaxX >= minX && chord.MinX <= maxX && chord.MaxY >= minY && chord.MinY <= maxY)
                        visit(chord);
                }
            }
        }
    }
}
