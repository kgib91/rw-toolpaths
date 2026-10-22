using Clipper2Lib;
using RW.Toolpaths.Geometry;

namespace RW.Toolpaths.Milling;

/// <summary>One operation-local original-source diagram and its conservative chord index.
/// Queries retain original native ordering; omitted chords cannot meet the integer clip bounds.</summary>
internal sealed class PreparedTerminalMedial
{
    private readonly IReadOnlyList<MedialSegment> _segments;
    private readonly Entry[] _entries;
    private readonly Node[] _nodes;
    private const int LeafSize = 16;
    private readonly record struct Entry(long MinX, long MinY, long MaxX, long MaxY, int Ordinal);
    private readonly record struct Node(long MinX, long MinY, long MaxX, long MaxY,
        int First, int Count, int Left, int Right);
    private readonly MillingRegion _source;
    private readonly CancellationToken _token;
    private readonly bool _lazy;
    private readonly bool _boundOriginalSource;
    private readonly bool _preserveNativeGrid;
    private PreparedTerminalMedial? _full;
    private MedialAxisGeometry.PreparedLocalSource? _localSource;
    private readonly Dictionary<(long, long, long, long), IReadOnlyList<MedialSegment>> _localGraphs = new();
    internal IReadOnlyList<MedialSegment> Segments => _lazy ? Full().Segments : _segments;
    internal int MaterializedSegmentCount => _full?.MaterializedSegmentCount
        ?? (_lazy ? _localGraphs.Values.Sum(graph => graph.Count) : _segments.Count);
    internal bool LastQueryWasLocal { get; private set; }
    internal bool FullGraphMaterialized => !_lazy || _full is not null;
    internal PolygonOps.BoundarySegmentIndex Boundary { get; }
    // Canonical signed finite-site authority used by Boundary and TraceNative.
    // Consumers treat this operation-owned geometry as immutable.
    internal Paths64 CanonicalSource { get; }

    internal PreparedTerminalMedial(MillingRegion source, CancellationToken token,
        IReadOnlyList<MedialSegment>? segments = null, bool lazy = false,
        bool boundOriginalSource = false, bool preserveNativeGrid = false)
    {
        token.ThrowIfCancellationRequested();
        long started = PerfLog.Start();
        _source = source; _token = token; _lazy = lazy && segments is null;
        _preserveNativeGrid = preserveNativeGrid;
        // Resolve the reference policy before publishing either a lazy source
        // graph or its immutable full-graph cache identity.
        _boundOriginalSource = boundOriginalSource
            && Environment.GetEnvironmentVariable("RW_TOOLPATHS_SOURCE_MEDIAL_REFERENCE") != "1";
        _segments = _lazy ? Array.Empty<MedialSegment>() : segments
            ?? OffsetTerminalPaths.BuildSourceMedial(source, token, _boundOriginalSource, _preserveNativeGrid);
        long indexStarted = PerfLog.Start();
        var sourceMaterial = PolygonOps.Difference(new[] { source.Outer }, source.Holes);
        CanonicalSource = PathUtils.ToClipper(sourceMaterial);
        Boundary = new PolygonOps.BoundarySegmentIndex(CanonicalSource);
        _entries = new Entry[_segments.Count];
        for (int i = 0; i < _entries.Length; i++)
        {
            if ((i & 1023) == 0) token.ThrowIfCancellationRequested();
            var segment = _segments[i];
            var a = Quantize(segment.Point0); var b = Quantize(segment.Point1);
            _entries[i] = new(Math.Min(a.X, b.X), Math.Min(a.Y, b.Y),
                Math.Max(a.X, b.X), Math.Max(a.Y, b.Y), i);
        }
        var nodes = new List<Node>();
        if (_entries.Length > 0) Build(0, _entries.Length);
        _nodes = nodes.ToArray();
        PerfLog.Stop("Offset.Terminal.Index", indexStarted,
            $"segments={_segments.Count} nodes={_nodes.Length} sourceEdges={Boundary.SegmentCount}");
        PerfLog.Stop("Offset.Terminal.Source", started,
            $"segments={_segments.Count} lazyOriginalSites={_lazy} boundedOriginalSource={_boundOriginalSource}");

        int Build(int first, int count)
        {
            token.ThrowIfCancellationRequested();
            long minX = long.MaxValue, minY = long.MaxValue, maxX = long.MinValue, maxY = long.MinValue;
            for (int i = first; i < first + count; i++)
            {
                var entry = _entries[i];
                minX = Math.Min(minX, entry.MinX); minY = Math.Min(minY, entry.MinY);
                maxX = Math.Max(maxX, entry.MaxX); maxY = Math.Max(maxY, entry.MaxY);
            }
            int index = nodes.Count;
            nodes.Add(default);
            int left = -1, right = -1;
            if (count > LeafSize)
            {
                int half = count / 2;
                Partition(first, count, maxX - minX >= maxY - minY);
                left = Build(first, half); right = Build(first + half, count - half);
            }
            nodes[index] = new(minX, minY, maxX, maxY, first, count, left, right);
            return index;
        }

        void Partition(int first, int count, bool x)
        {
            int lo = first, hi = first + count - 1, median = first + count / 2;
            int budget = 2 * (int)Math.Log2(count) + 2;
            int Compare(Entry a, Entry b)
            {
                double ac = x ? (double)a.MinX + a.MaxX : (double)a.MinY + a.MaxY;
                double bc = x ? (double)b.MinX + b.MaxX : (double)b.MinY + b.MaxY;
                int comparison = ac.CompareTo(bc);
                return comparison == 0 ? a.Ordinal.CompareTo(b.Ordinal) : comparison;
            }
            while (lo < hi)
            {
                token.ThrowIfCancellationRequested();
                if (--budget < 0)
                {
                    Array.Sort(_entries, lo, hi - lo + 1, Comparer<Entry>.Create(Compare));
                    break;
                }
                Entry pivot = _entries[lo + (hi - lo) / 2];
                int a = lo, b = hi;
                while (a <= b)
                {
                    while (Compare(_entries[a], pivot) < 0) a++;
                    while (Compare(_entries[b], pivot) > 0) b--;
                    if (a > b) break;
                    (_entries[a], _entries[b]) = (_entries[b], _entries[a]); a++; b--;
                }
                if (median <= b) hi = b;
                else if (median >= a) lo = a;
                else break;
            }
        }
    }

    internal IReadOnlyList<MedialSegment> Intersecting(Paths64 clip, CancellationToken token)
    {
        LastQueryWasLocal = false;
        if (_lazy && clip.Count > 0 && _source.Outer.Count + _source.Holes.Sum(hole => hole.Count) >= 1024
            && Environment.GetEnvironmentVariable("RW_TOOLPATHS_COHERENCE_REFERENCE") != "1"
            && Environment.GetEnvironmentVariable("RW_TOOLPATHS_LOCAL_MEDIAL_REFERENCE") != "1")
        {
            Rect64 bounds = Clipper.GetBounds(clip);
            var key = (bounds.left, bounds.top, bounds.right, bounds.bottom);
            if (!_localGraphs.TryGetValue(key, out var local))
            {
                _localSource ??= new(_source, token, _preserveNativeGrid);
                local = _localSource.TryBuild(bounds, token);
                if (local is not null) _localGraphs.Add(key, local);
            }
            if (local is not null) { LastQueryWasLocal = true; return local; }
        }
        return FullIntersecting(clip, token);
    }

    internal IReadOnlyList<MedialSegment> FullIntersecting(Paths64 clip, CancellationToken token)
    {
        LastQueryWasLocal = false;
        return _lazy ? Full().FullIntersecting(clip, token) : IntersectingIndex(clip, token);
    }

    private PreparedTerminalMedial Full()
        => _full ??= new(_source, _token, boundOriginalSource: _boundOriginalSource, preserveNativeGrid: _preserveNativeGrid);

    private IReadOnlyList<MedialSegment> IntersectingIndex(Paths64 clip, CancellationToken token)
    {
        long started = PerfLog.Start();
        Rect64 bounds = Clipper.GetBounds(clip);
        var ordinals = new List<int>();
        int nodeVisits = 0, candidateVisits = 0;
        if (_nodes.Length > 0 && clip.Count > 0) Visit(0);
        ordinals.Sort();
        var result = ordinals.Select(index => _segments[index]).ToArray();
        PerfLog.Stop("Offset.Terminal.Candidates", started,
            $"originalSegments={_segments.Count} nodeVisits={nodeVisits} candidateVisits={candidateVisits} retained={result.Length}");
        return result;

        void Visit(int index)
        {
            if ((++nodeVisits & 255) == 0) token.ThrowIfCancellationRequested();
            Node node = _nodes[index];
            if (!Meets(node.MinX, node.MinY, node.MaxX, node.MaxY)) return;
            if (node.Left >= 0) { Visit(node.Left); Visit(node.Right); return; }
            for (int i = node.First; i < node.First + node.Count; i++)
            {
                candidateVisits++;
                var entry = _entries[i];
                if (Meets(entry.MinX, entry.MinY, entry.MaxX, entry.MaxY)) ordinals.Add(entry.Ordinal);
            }
        }
        bool Meets(long x0, long y0, long x1, long y1)
            => x0 <= bounds.right && x1 >= bounds.left && y0 <= bounds.bottom && y1 >= bounds.top;
    }

    private static Point64 Quantize(MedialPoint p)
        => new((long)Math.Round(p.X * PathUtils.Scale), (long)Math.Round(p.Y * PathUtils.Scale));
}
