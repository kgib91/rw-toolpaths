using Clipper2Lib;
using Classifier = RW.Toolpaths.MedialAxisEdgeClassifier;

namespace RW.Toolpaths.Tests;

public sealed class MedialAxisClassificationIndexTests
{
    [Fact]
    public void LegacyAngleArgumentsCannotPruneInnerMedialBranches()
    {
        PointD[] ring = Enumerable.Range(0,16).Select(i => {
            double angle = 2 * Math.PI * i / 16;
            return new PointD(10000 * Math.Cos(angle), 10000 * Math.Sin(angle));
        }).ToArray();
        var provider = BoostVoronoiProvider.CreateDefault();
        var complete = provider.ConstructMedialAxis(ring, [], .03, 30000, Math.PI);
        var legacy = provider.ConstructMedialAxis(ring, [], .03, 30000, 3 * Math.PI / 4);
        Assert.NotEmpty(complete);
        Assert.Equal(complete, legacy);
    }

    [Fact]
    public void IndexedRayCrossingMatchesOriginalPredicateForDenseConcaveRingAndBoundaryTies()
    {
        var ring = Enumerable.Range(0, 16384).Select(i =>
        {
            double angle = 2 * Math.PI * i / 16384;
            double radius = 1000 + 180 * Math.Cos(17 * angle);
            return new PointD(Math.Round(200000 + radius * Math.Cos(angle)),
                Math.Round(-500000 + radius * Math.Sin(angle)));
        }).ToArray();
        var prepared = new Classifier.PreparedRingContainment(ring);
        var random = new Random(41711);
        for (int i = 0; i < 2048; i++)
            Check(new(198700 + 2600 * random.NextDouble(), -501300 + 2600 * random.NextDouble()));
        foreach (var vertex in ring.Where((_, index) => index % 17 == 0))
        {
            Check(new(vertex.x, vertex.y));
            Check(new(Math.BitDecrement(vertex.x), vertex.y));
            Check(new(Math.BitIncrement(vertex.x), vertex.y));
            Check(new(vertex.x, Math.BitDecrement(vertex.y)));
            Check(new(vertex.x, Math.BitIncrement(vertex.y)));
        }
        Assert.True(prepared.EdgeTests < 5_000_000,
            $"Prepared ray tests should skip distant Y edges; tested={prepared.EdgeTests}.");
        void Check(Classifier.EdgePoint point) => Assert.Equal(ReferenceContains(point, ring), prepared.Contains(point));
    }

    [Fact]
    public void IndexedPredicatePreservesHorizontalDegenerateAndExplicitClosureCases()
    {
        PointD[][] rings = [[], [new(1, 1)], [new(0, 0), new(1, 0), new(2, 0)],
            [new(0, 0), new(10, 0), new(10, 10), new(0, 10), new(0, 0)],
            [new(0, 0), new(5, 0), new(5, 0), new(10, 0), new(10, 10), new(0, 10)]];
        foreach (var ring in rings)
        {
            var prepared = new Classifier.PreparedRingContainment(ring);
            foreach (double x in new[] { -1d, 0, double.Epsilon, 1, 5, 10, Math.BitIncrement(10d), 11 })
            foreach (double y in new[] { -1d, 0, double.Epsilon, 1, 5, 10, Math.BitIncrement(10d), 11 })
            {
                var point = new Classifier.EdgePoint(x, y);
                Assert.Equal(ReferenceContains(point, ring), prepared.Contains(point));
            }
        }
    }

    [Fact]
    public void ClassificationPreservesHolesTwinColorsAndExactBoundaryConvention()
    {
        PointD[] outer = [new(0, 0), new(100, 0), new(100, 100), new(0, 100)];
        IReadOnlyList<PointD>[] holes = [
            new PointD[] { new(10, 10), new(30, 10), new(30, 30), new(10, 30) },
            new PointD[] { new(60, 60), new(80, 70), new(60, 80) }];
        var queries = new List<Classifier.EdgePoint>();
        for (int y = -5; y <= 105; y += 5)
        for (int x = -5; x <= 105; x += 5) queries.Add(new(x, y));
        var edges = new List<Classifier.EdgeData>();
        foreach (var query in queries)
        {
            int i = edges.Count;
            var cell = new Classifier.CellSite(true, new(-999, -999), null);
            edges.Add(new(i, i + 1, true, false, false, -1, -1, query, query, cell, cell));
            edges.Add(new(i + 1, i, true, false, false, -1, -1, query, query, cell, cell));
        }
        var logs = new List<string>();
        Classifier.ClassifyEdges(edges, outer, holes, diagnostic: logs.Add);
        for (int i = 0; i < queries.Count; i++)
        {
            bool inside = ReferenceContains(queries[i], outer) && !holes.Any(hole => ReferenceContains(queries[i], hole));
            int expected = inside ? Classifier.Colors.InnerPrimary : Classifier.Colors.Unclassified;
            Assert.Equal(expected, edges[2 * i].Color);
            Assert.Equal(expected, edges[2 * i + 1].Color);
        }
        Assert.Contains(logs, line => line.Contains("event=complete") && line.Contains("edgeTests="));
    }

    [Fact]
    public void ProviderProgressPreservesSegmentsAndReportsOpaqueConstruction()
    {
        PointD[] boundary = [new(0, 0), new(40000, 0), new(40000, 30000), new(0, 30000)];
        var provider = BoostVoronoiProvider.CreateDefault();
        var expected = provider.ConstructMedialAxis(boundary, [], 0.03, 2000);
        var reports = new List<(double Fraction, string Stage, bool Indeterminate)>();
        var actual = provider.ConstructMedialAxis(boundary, [], 0.03, 2000, 3 * Math.PI / 4, true,
            CancellationToken.None, progress: (fraction, stage, indeterminate) => reports.Add((fraction, stage, indeterminate)));
        Assert.Equal(expected, actual);
        Assert.Contains(reports, report => report.Indeterminate && report.Stage == "Building Voronoi diagram");
        Assert.Equal(1, reports[^1].Fraction);
        for (int i = 1; i < reports.Count; i++) Assert.True(reports[i].Fraction >= reports[i - 1].Fraction);
    }

    [Fact]
    public void ProviderCancelsBeforeClassificationAndRemainsReusable()
    {
        PointD[] boundary = [new(0, 0), new(40000, 0), new(40000, 30000), new(0, 30000)];
        var provider = BoostVoronoiProvider.CreateDefault();
        using var cancellation = new CancellationTokenSource();
        Assert.Throws<OperationCanceledException>(() => provider.ConstructMedialAxis(boundary, [], 0.03,
            2000, 3 * Math.PI / 4, true, cancellation.Token,
            progress: (_, stage, _) => { if (stage == "Classifying medial edges") cancellation.Cancel(); }));
        Assert.NotEmpty(provider.ConstructMedialAxis(boundary, [], 0.03, 2000));
    }

    [Fact]
    public void IndexedTraversalPreservesOldOrderedWalkAcrossCyclesTwinsAndNewlyEligibleStarts()
    {
        var random = new Random(78941);
        var cell = new Classifier.CellSite(true, new(-1, -1), null);
        for (int trial = 0; trial < 160; trial++)
        {
            const int count = 240;
            var edges = Enumerable.Range(0, count).Select(index => new Classifier.EdgeData(
                index, index ^ 1, true, false, false,
                random.Next(-1, count), random.Next(-1, count), new(index, 0), new(index, 1), cell, cell)
            { Color = random.Next(5) == 0 ? Classifier.Colors.OuterPrimary : Classifier.Colors.InnerPrimary }).ToArray();
            Assert.Equal(ReferenceTraversal(edges), Classifier.SelectInnerPrimaryTraversal(edges));
        }
    }

    [Fact]
    public void IndexedTraversalPreservesWalkThroughSecondaryEdgesAndCycleFallback()
    {
        var cell = new Classifier.CellSite(true, new(-1, -1), null);
        Classifier.EdgeData Edge(int index, int twin, int previous, int next, bool secondary = false) => new(
            index, twin, !secondary, secondary, false, previous, next, new(index, 0), new(index, 1), cell, cell)
            { Color = secondary ? Classifier.Colors.Secondary : Classifier.Colors.InnerPrimary };
        Classifier.EdgeData[] edges = [
            Edge(0, 6, -1, 2), Edge(1, 7, 3, -1),
            Edge(2, 4, -1, -1, true), Edge(3, 5, -1, -1, true),
            Edge(4, 2, -1, 1, true), Edge(5, 3, 0, -1, true),
            Edge(6, 0, -1, 7), Edge(7, 1, 6, -1),
            Edge(8, -1, 9, 9), Edge(9, -1, 8, 8)];
        Assert.Equal(new[] { 0, 1, 8, 9 }, ReferenceTraversal(edges));
        Assert.Equal(ReferenceTraversal(edges), Classifier.SelectInnerPrimaryTraversal(edges));
    }

    private static List<int> ReferenceTraversal(IReadOnlyList<Classifier.EdgeData> edges)
    {
        var inner = edges.Where(edge => edge.Color == Classifier.Colors.InnerPrimary).ToList();
        var remaining = new HashSet<int>(inner.Select(edge => edge.Index));
        var walked = new List<int>();
        Classifier.EdgeData? Get(int index) => index >= 0 && index < edges.Count ? edges[index] : null;
        Classifier.EdgeData? Previous(Classifier.EdgeData edge)
        {
            var current = Get(edge.PrevIndex);
            while (current is not null && current.IsSecondary && current.Index != edge.Index)
            {
                var twin = Get(current.TwinIndex);
                current = twin is null ? null : Get(twin.PrevIndex);
            }
            return current;
        }
        Classifier.EdgeData? Next(Classifier.EdgeData edge)
        {
            var current = Get(edge.NextIndex);
            while (current is not null && current.IsSecondary && current.Index != edge.Index)
            {
                var twin = Get(current.TwinIndex);
                current = twin is null ? null : Get(twin.NextIndex);
            }
            return current;
        }
        while (remaining.Count > 0)
        {
            var start = edges[remaining.First()];
            foreach (var candidate in inner)
            {
                if (!remaining.Contains(candidate.Index)) continue;
                var previous = Previous(candidate);
                if (previous is null || !remaining.Contains(previous.Index)) { start = candidate; break; }
            }
            var path = new List<int>();
            var current = start;
            while (current is not null && remaining.Contains(current.Index))
            {
                walked.Add(current.Index); path.Add(current.Index); remaining.Remove(current.Index);
                current = Next(current);
            }
            foreach (int index in path)
                if (edges[index].TwinIndex >= 0) remaining.Remove(edges[index].TwinIndex);
        }
        return walked;
    }

    private static bool ReferenceContains(Classifier.EdgePoint point, IReadOnlyList<PointD> ring)
    {
        bool inside = false;
        for (int index = 0, previous = ring.Count - 1; index < ring.Count; previous = index++)
        {
            var current = ring[index]; var prior = ring[previous];
            if ((current.y > point.Y) == (prior.y > point.Y)) continue;
            double x = (prior.x - current.x) * (point.Y - current.y) / (prior.y - current.y) + current.x;
            if (x > point.X) inside = !inside;
        }
        return inside;
    }
}
