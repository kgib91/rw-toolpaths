using System.Collections;
using Clipper2Lib;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class PocketFloorReplayTests
{
    [Fact]
    public void InterleavedFloorQueriesPreserveEagerProjectionAndUnionOrderExactly()
    {
        var actual = new PocketContinuationRouter.ClearedStock(.5, CancellationToken.None);
        var eager = new EagerProjectionReference(.5);
        void Record(Point3D[] stroke, PointD[][] shallow, PointD[][] middle, PointD[][] deep)
        {
            actual.Record(stroke);
            eager.Record(new Dictionary<double, PointD[][]> { [-1] = shallow, [-2] = middle, [-3] = deep });
        }
        void Query(double floor)
        {
            var expected = eager.AtFloor(floor);
            var result = actual.AtFloor(floor);
            Assert.Equal(expected.Count, result.Count);
            for (int index = 0; index < expected.Count; index++)
                Assert.Equal(expected[index].ToArray(), result[index].ToArray());
        }

        // Explicit analytical projections exercise descending and ascending floor
        // crossings, flat cuts, landing discs and disjoint/overlapping batches.
        // The oracle never calls the router's BelowFloor clipping implementation.
        Record([new(0, 0, 0), new(6, 0, -3), new(12, 0, -3)],
            [[new(2, 0), new(6, 0), new(12, 0)]],
            [[new(4, 0), new(6, 0), new(12, 0)]],
            [[new(6, 0), new(6, 0), new(12, 0)]]);
        Record([new(0, 4, -1), new(8, 4, -1)], [[new(0, 4), new(8, 4)]], [], []);
        Query(-1);
        Record([new(0, 8, -3), new(6, 8, 0)],
            [[new(0, 8), new(4, 8)]], [[new(0, 8), new(2, 8)]], [[new(0, 8), new(0, 8)]]);
        Query(-2);
        Query(-1);
        Query(-1);
        Assert.Equal(6, actual.ReplayedStrokes);
        Assert.Equal(3, actual.FloorUnionBatches);

        Record([new(0, 12, 0), new(6, 12, -3), new(12, 12, 0)],
            [[new(2, 12), new(6, 12), new(10, 12)]],
            [[new(4, 12), new(6, 12), new(8, 12)]],
            [[new(6, 12), new(6, 12), new(6, 12)]]);
        Record([new(-3, 0, -4), new(15, 0, -4)],
            [[new(-3, 0), new(15, 0)]], [[new(-3, 0), new(15, 0)]], [[new(-3, 0), new(15, 0)]]);
        Query(-2);
        Query(-1);
        Query(-3); // A newly queried floor must replay the complete executed prefix.
        Query(-3);
        Assert.Equal(5, actual.RecordedStrokes);
        Assert.Equal(3, actual.FloorCount);
        Assert.Equal(8, actual.FloorQueries);
        Assert.Equal(15, actual.ReplayedStrokes);
        Assert.Equal(6, actual.FloorUnionBatches);
        Assert.Equal(0, actual.DeferredFloorStrokes);
    }

    [Fact]
    public void RecordingDoesNoProjectionAndOnlyAQueriedFloorConsumesNewStrokes()
    {
        var stock = new PocketContinuationRouter.ClearedStock(.5, CancellationToken.None);
        Assert.Empty(stock.AtFloor(-1));
        Assert.Empty(stock.AtFloor(-2));
        Assert.Empty(stock.AtFloor(-3));
        var first = new ObservedStroke([new(0, 0, -4), new(5, 0, -4)]);
        stock.Record(first);
        Assert.Equal(0, first.Reads);
        Assert.Equal(0, stock.ReplayedStrokes);
        Assert.Equal(3, stock.DeferredFloorStrokes);
        Assert.NotEmpty(stock.AtFloor(-2));
        int firstReads = first.Reads;
        Assert.True(firstReads > 0);
        Assert.Equal(1, stock.ReplayedStrokes);
        Assert.Equal(2, stock.DeferredFloorStrokes);
        stock.AtFloor(-2);
        Assert.Equal(firstReads, first.Reads);

        var second = new ObservedStroke([new(5, 0, -4), new(8, 3, -4)]);
        stock.Record(second);
        Assert.Equal(0, second.Reads);
        stock.AtFloor(-2);
        Assert.Equal(firstReads, first.Reads);
        Assert.True(second.Reads > 0);
        Assert.Equal(2, stock.ReplayedStrokes);
        Assert.Equal(4, stock.DeferredFloorStrokes);
        Assert.Equal(2, stock.FloorUnionBatches);
    }

    [Fact]
    public void ReplayCancellationStopsBeforeTheNextChronologicalStroke()
    {
        using var cancellation = new CancellationTokenSource();
        var stock = new PocketContinuationRouter.ClearedStock(.5, cancellation.Token);
        var first = new ObservedStroke([new(0, 0, 1), new(5, 0, 1)], cancellation.Cancel);
        var second = new ObservedStroke([new(0, 5, -2), new(5, 5, -2)]);
        stock.Record(first);
        stock.Record(second);
        Assert.Throws<OperationCanceledException>(() => stock.AtFloor(-1));
        Assert.True(first.Reads > 0);
        Assert.Equal(0, second.Reads);
        Assert.Equal(0, stock.FloorUnionBatches);
        Assert.Throws<OperationCanceledException>(() => stock.Record(second));
        Assert.Equal(2, stock.RecordedStrokes);
    }

    private sealed class ObservedStroke(Point3D[] points, Action? firstRead = null) : IReadOnlyList<Point3D>
    {
        internal int Reads { get; private set; }
        public int Count => points.Length;
        public Point3D this[int index]
        {
            get
            {
                if (Reads++ == 0) firstRead?.Invoke();
                return points[index];
            }
        }
        public IEnumerator<Point3D> GetEnumerator() => ((IEnumerable<Point3D>)points).GetEnumerator();
        IEnumerator IEnumerable.GetEnumerator() => GetEnumerator();
    }

    // Preserve the former eager update/flush schedule using independently specified
    // projected strokes. Exact polygon order catches changed Boolean batching too.
    private sealed class EagerProjectionReference(double radius)
    {
        private sealed class Floor
        {
            internal List<List<PointD>> Swept = new();
            internal List<IReadOnlyList<PointD>> Pending = new();
        }
        private readonly List<IReadOnlyDictionary<double, PointD[][]>> _strokes = new();
        private readonly Dictionary<double, Floor> _floors = new();
        internal void Record(IReadOnlyDictionary<double, PointD[][]> stroke)
        {
            _strokes.Add(stroke);
            foreach (var (z, floor) in _floors) floor.Pending.AddRange(stroke[z]);
        }
        internal IReadOnlyList<List<PointD>> AtFloor(double z)
        {
            if (!_floors.TryGetValue(z, out var floor))
            {
                floor = new Floor();
                foreach (var stroke in _strokes) floor.Pending.AddRange(stroke[z]);
                _floors.Add(z, floor);
            }
            if (floor.Pending.Count > 0)
            {
                var added = PolygonOps.BufferOpenCenterlines(floor.Pending, radius, .001, CancellationToken.None);
                floor.Swept = PolygonOps.Union(floor.Swept.Concat(added));
                floor.Pending.Clear();
            }
            return floor.Swept;
        }
    }
}
