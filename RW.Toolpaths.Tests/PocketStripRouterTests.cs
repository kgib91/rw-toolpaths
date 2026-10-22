using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class PocketStripRouterTests
{
    [Fact]
    public void NearbyFragmentsJoinAtBothEndsWithoutLosingAnyOriginalChord()
    {
        var traces = new List<List<PointD>>
        {
            Trace((-4, 0), (-3, 0)),
            Trace((0, 0), (1, 1), (2, 0)),
            Trace((5, 0), (4, 0)),
        };
        var originals = traces.Select(trace => trace.ToArray()).ToArray();
        var proofCalls = new List<(PointD A, PointD B)>();
        var routes = PocketStripRouter.Route(traces, 3, (a, b) =>
        {
            proofCalls.Add((a, b));
            return true;
        }, CancellationToken.None, out int bridges, out int rejected);

        List<PointD> route = Assert.Single(routes);
        Assert.Equal(2, bridges);
        Assert.Equal(0, rejected);
        Assert.Equal((-4d, 0d), Coordinates(route[0]));
        Assert.Equal((5d, 0d), Coordinates(route[^1]));
        AssertOriginalChordsPreserved(traces, routes);
        AssertEveryNewChordWasProved(traces, routes, proofCalls, 3);
        for (int index = 0; index < traces.Count; index++)
            Assert.Equal(originals[index].Select(Coordinates), traces[index].Select(Coordinates));
    }

    [Fact]
    public void FullSegmentProofRefusesTheNearestConnectionThroughAnIsland()
    {
        var island = Trace((10, -2), (11, -2), (11, 2), (10, 2), (10, -2));
        var traces = new List<List<PointD>>
        {
            Trace((7, 0), (8, 0)),
            Trace((13, 0), (14, 0)),
        };
        var proofCalls = new List<(PointD A, PointD B)>();
        var routes = PocketStripRouter.Route(traces, 6, (a, b) =>
        {
            proofCalls.Add((a, b));
            return ClosedEdges(island).All(edge => SegmentDistanceSquared(a, b, edge.A, edge.B) >= 1);
        }, CancellationToken.None, out int bridges, out int rejected);

        Assert.Equal(2, routes.Count);
        Assert.Equal(0, bridges);
        Assert.True(rejected > 0, "The nearby island-crossing chord must reach the physical proof and be rejected.");
        AssertOriginalChordsPreserved(traces, routes);
        foreach (var move in routes.SelectMany(OpenEdges))
        foreach (var boundary in ClosedEdges(island))
            Assert.True(SegmentDistanceSquared(move.A, move.B, boundary.A, boundary.B) >= 1);
    }

    [Fact]
    public void RejectedNearestEndpointDoesNotHideASafeAlternativeAroundAnIsland()
    {
        var island = Trace((10.4, -.3), (10.6, -.3), (10.6, 3), (10.4, 3));
        var traces = new List<List<PointD>>
        {
            Trace((0, 0), (10, 0)),
            Trace((11, 0), (12, 0)),
            Trace((11, -2), (12, -2)),
        };
        var proofCalls = new List<(PointD A, PointD B)>();
        var routes = PocketStripRouter.Route(traces, 3, (a, b) =>
        {
            proofCalls.Add((a, b));
            return ClosedEdges(island).All(edge => SegmentDistanceSquared(a, b, edge.A, edge.B) >= .01);
        }, CancellationToken.None, out int bridges, out int rejected);

        Assert.Single(routes);
        Assert.Equal(2, bridges);
        Assert.True(rejected > 0);
        AssertOriginalChordsPreserved(traces, routes);
        AssertEveryNewChordWasProved(traces, routes, proofCalls, 3);
        foreach (var move in routes.SelectMany(OpenEdges))
        foreach (var boundary in ClosedEdges(island))
            Assert.True(SegmentDistanceSquared(move.A, move.B, boundary.A, boundary.B) >= .01,
                $"Unsafe routed chord {Coordinates(move.A)} -> {Coordinates(move.B)}.");
    }

    [Fact]
    public void AClosedSourceCycleRetainsItsEntireCycleWhenATrailAttaches()
    {
        var traces = new List<List<PointD>>
        {
            Trace((0, 0), (2, 0), (2, 2), (0, 2), (0, 0)),
            Trace((-2, 0), (-1, 0)),
        };
        var proofCalls = new List<(PointD A, PointD B)>();
        var routes = PocketStripRouter.Route(traces, 2, (a, b) =>
        {
            proofCalls.Add((a, b));
            return true;
        }, CancellationToken.None, out int bridges);

        Assert.Single(routes);
        Assert.Equal(1, bridges);
        AssertOriginalChordsPreserved(traces, routes);
        AssertEveryNewChordWasProved(traces, routes, proofCalls, 2);
        Assert.Equal(2, routes[0].Count(point => Coordinates(point) == (0d, 0d)));
    }

    [Fact]
    public void SharedEndpointsAreMergedOnlyWhenTheSuppliedProofAllowsThem()
    {
        var traces = new List<List<PointD>>
        {
            Trace((0, 0), (1, 0)),
            Trace((1, 0), (2, 0)),
        };
        Assert.Equal(2, PocketStripRouter.Route(traces, 1, (_, _) => false,
            CancellationToken.None, out int refusedBridges).Count);
        Assert.Equal(0, refusedBridges);

        var joined = PocketStripRouter.Route(traces, 1, (_, _) => true,
            CancellationToken.None, out int bridges);
        Assert.Single(joined);
        Assert.Equal(0, bridges);
        AssertOriginalChordsPreserved(traces, joined);
        Assert.Equal(3, joined[0].Count);
    }

    [Fact]
    public void DisconnectedComponentsBeyondOneDiameterRemainSeparate()
    {
        var traces = new List<List<PointD>>
        {
            Trace((-20, 0), (-10, 0)),
            Trace((0, 0), (1, 0)),
        };
        int proofs = 0;
        var routes = PocketStripRouter.Route(traces, 6, (_, _) =>
        {
            proofs++;
            return true;
        }, CancellationToken.None, out int bridges);

        Assert.Equal(2, routes.Count);
        Assert.Equal(0, bridges);
        Assert.Equal(0, proofs);
        AssertOriginalChordsPreserved(traces, routes);
    }

    [Fact]
    public void CancellationStopsBeforeInvokingThePhysicalProof()
    {
        var traces = new List<List<PointD>> { Trace((0, 0), (1, 0)) };
        using var cancellation = new CancellationTokenSource();
        cancellation.Cancel();
        Assert.Throws<OperationCanceledException>(() => PocketStripRouter.Route(traces, 6,
            (_, _) => throw new InvalidOperationException("Proof must not run after cancellation."),
            cancellation.Token, out _));
    }

    private static void AssertOriginalChordsPreserved(
        IEnumerable<List<PointD>> traces, IEnumerable<List<PointD>> routes)
    {
        var routed = routes.SelectMany(OpenEdges).Select(move => Chord(move.A, move.B)).ToList();
        foreach (var move in traces.SelectMany(OpenEdges))
        {
            var chord = Chord(move.A, move.B);
            int index = routed.IndexOf(chord);
            Assert.True(index >= 0, $"Original trace chord lost: {Coordinates(move.A)} -> {Coordinates(move.B)}.");
            routed.RemoveAt(index);
        }
    }

    private static void AssertEveryNewChordWasProved(
        IEnumerable<List<PointD>> traces,
        IEnumerable<List<PointD>> routes,
        List<(PointD A, PointD B)> proofCalls,
        double pitch)
    {
        var originals = traces.SelectMany(OpenEdges).Select(move => Chord(move.A, move.B)).ToList();
        foreach (var move in routes.SelectMany(OpenEdges))
        {
            var chord = Chord(move.A, move.B);
            int index = originals.IndexOf(chord);
            if (index >= 0)
            {
                originals.RemoveAt(index);
                continue;
            }
            Assert.Contains(proofCalls, proof => Coordinates(proof.A) == Coordinates(move.A)
                && Coordinates(proof.B) == Coordinates(move.B));
            Assert.True(PointDistanceSquared(move.A, move.B) <= pitch * pitch,
                $"Bridge exceeds diameter: {Coordinates(move.A)} -> {Coordinates(move.B)}, pitch={pitch:R}.");
        }
    }

    private static ((double X, double Y) A, (double X, double Y) B) Chord(PointD a, PointD b)
    {
        var first = Coordinates(a);
        var second = Coordinates(b);
        return first.CompareTo(second) <= 0 ? (first, second) : (second, first);
    }

    private static (double X, double Y) Coordinates(PointD point) => (point.x, point.y);

    private static List<PointD> Trace(params (double X, double Y)[] points)
        => points.Select(point => new PointD(point.X, point.Y)).ToList();

    private static IEnumerable<(PointD A, PointD B)> OpenEdges(List<PointD> path)
    {
        for (int index = 1; index < path.Count; index++)
            yield return (path[index - 1], path[index]);
    }

    private static IEnumerable<(PointD A, PointD B)> ClosedEdges(List<PointD> path)
    {
        for (int index = 0; index < path.Count; index++)
            yield return (path[index], path[(index + 1) % path.Count]);
    }

    private static double SegmentDistanceSquared(PointD a, PointD b, PointD c, PointD d)
    {
        double abx = b.x - a.x, aby = b.y - a.y, cdx = d.x - c.x, cdy = d.y - c.y;
        double determinant = abx * cdy - aby * cdx;
        if (Math.Abs(determinant) > 1e-20)
        {
            double acx = c.x - a.x, acy = c.y - a.y;
            double t = (acx * cdy - acy * cdx) / determinant;
            double u = (acx * aby - acy * abx) / determinant;
            if (t >= 0 && t <= 1 && u >= 0 && u <= 1)
                return 0;
        }
        return Math.Min(Math.Min(PointSegmentDistanceSquared(a, c, d), PointSegmentDistanceSquared(b, c, d)),
            Math.Min(PointSegmentDistanceSquared(c, a, b), PointSegmentDistanceSquared(d, a, b)));
    }

    private static double PointSegmentDistanceSquared(PointD p, PointD a, PointD b)
    {
        double dx = b.x - a.x, dy = b.y - a.y;
        double lengthSquared = dx * dx + dy * dy;
        double t = lengthSquared == 0 ? 0
            : Math.Clamp(((p.x - a.x) * dx + (p.y - a.y) * dy) / lengthSquared, 0, 1);
        var nearest = new PointD(a.x + t * dx, a.y + t * dy);
        return PointDistanceSquared(p, nearest);
    }

    private static double PointDistanceSquared(PointD a, PointD b)
        => (b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y);
}
