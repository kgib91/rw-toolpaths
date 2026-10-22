using Clipper2Lib;
using System.Runtime.InteropServices;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

[Collection("Full source medial reference controls")]
public sealed class PocketProtectionTests
{
    [Fact]
    public void GenericToleranceDefaultsDoNotEraseAConcentricReachableProtectedChannel()
    {
        var source = new MillingRegion(Box(0, 0, 50, 25));
        IReadOnlyList<PointD>[] protection = [Box(0, 0, 20, 25), Box(26.5, 0, 50, 25)];
        var options = new PocketOptions { ProtectedBoundary = protection, StepOver = .5,
            DetectCurves = false, CalculateSweptArea = false, Ramp = RampSettings.FromRatio(3) };
        var domain = PocketProtection.PrepareConcentricMask(options, CancellationToken.None, denseFirstArcExperiment: false);
        Assert.Equal(.001 + 5 / PathUtils.Scale, domain.RequiredGuard, 12);
        Assert.InRange(domain.AppliedOffset, domain.RequiredGuard, .003);
        Assert.Empty(PolygonOps.Difference(protection, domain.Paths));
        // The generic defaults formerly excluded another half millimeter at
        // each wall and erased this physically reachable 6.5mm channel entirely.
        var plan = PocketToolpaths.Generate([source], ToolGeometry.Flat(3.175), new DepthSchedule(1, 1), options);
        Assert.NotEmpty(plan.Toolpaths);
        var floorChords = plan.Toolpaths.SelectMany(path => Enumerable.Range(1, path.Points.Count - 1)
            .Select(i => (A: path.Points[i - 1], B: path.Points[i])))
            .Where(edge => edge.A.Z == -1 && edge.B.Z == -1).ToArray();
        Assert.Contains(floorChords, edge => Math.Min(edge.A.Y, edge.B.Y) <= 12.5
            && Math.Max(edge.A.Y, edge.B.Y) >= 12.5);
        foreach (var edge in floorChords)
        {
            var capsule = PolygonOps.BufferOpenCenterlines(
                [new PointD[] { new(edge.A.X, edge.A.Y), new(edge.B.X, edge.B.Y) }], 3.175, .00001);
            Assert.Empty(PolygonOps.Intersect(capsule, protection));
        }
    }

    [Theory]
    [InlineData(0)]
    [InlineData(-.35)]
    public void OuterMaskPreservesCompleteSignedHolesTinyIslandsAndActualClampedFidelity(double leave)
    {
        IReadOnlyList<PointD>[] raw = [Box(10, 10, 20, 20), Box(12, 12, 18, 18).Reverse().ToArray(),
            Box(14, 14, 15, 15), Box(24, 14, 24.0004, 14.0003)];
        var coarse = new PocketOptions { ProtectedBoundary = raw, StockToLeave = leave };
        var fine = coarse with { Tolerances = new GeometryTolerances { ArcTolerance = .001, SimplifyTolerance = .7 } };
        var a = PocketProtection.PrepareConcentricMask(coarse, CancellationToken.None, denseFirstArcExperiment: false);
        var b = PocketProtection.PrepareConcentricMask(fine, CancellationToken.None, denseFirstArcExperiment: false);
        Assert.Equal(.001 + 5 / PathUtils.Scale + Math.Max(0, -leave), a.RequiredGuard, 12);
        Assert.Equal(a.RequiredGuard, b.RequiredGuard);
        Assert.Empty(PolygonOps.Difference(a.Paths, b.Paths));
        Assert.Empty(PolygonOps.Difference(b.Paths, a.Paths));
        Assert.Empty(PolygonOps.Difference(raw, a.Paths));
        Assert.False(PolygonOps.Contains(a.Paths, 16.5, 16.5));
        Assert.True(PolygonOps.Contains(a.Paths, 14.5, 14.5));
        Assert.True(PolygonOps.Contains(a.Paths, 24.0002, 14.00015));
        // Independent axis-aligned distances expose the complete outward guard
        // on outer walls and the reverse direction at a signed hole boundary.
        Assert.True(PolygonOps.Contains(a.Paths, 10 - a.RequiredGuard, 15));
        Assert.True(PolygonOps.Contains(a.Paths, 12 + a.RequiredGuard, 16));
        Assert.False(PolygonOps.Contains(a.Paths, 10 - a.AppliedOffset - .001, 15));
    }

    [Theory]
    [InlineData(0)]
    [InlineData(-.35)]
    public void DenseFirstArcExperimentRetainsTheCoarseGuardAndCompleteRawContainment(double leave)
    {
        IReadOnlyList<PointD>[] raw = [Box(10, 10, 20, 20), Box(12, 12, 18, 18).Reverse().ToArray(),
            Box(14, 14, 15, 15), Box(24, 14, 24.0004, 14.0003)];
        var options = new PocketOptions { ProtectedBoundary = raw, StockToLeave = leave };
        var nativeFirst = PocketProtection.PrepareConcentricMask(options, CancellationToken.None,
            denseFirstArcExperiment: false);
        var coarseFirst = PocketProtection.PrepareConcentricMask(options, CancellationToken.None,
            denseFirstArcExperiment: true);
        Assert.Equal(1 / PathUtils.Scale, nativeFirst.EffectiveArcTolerance);
        Assert.Equal(.00025, coarseFirst.EffectiveArcTolerance);
        Assert.Equal(.00125 + 4 / PathUtils.Scale + Math.Max(0, -leave), coarseFirst.RequiredGuard, 12);
        Assert.Equal(.00025 - 1 / PathUtils.Scale,
            coarseFirst.RequiredGuard - nativeFirst.RequiredGuard, 12);
        Assert.Empty(PolygonOps.Difference(raw, coarseFirst.Paths));
        Assert.Empty(PolygonOps.Difference(nativeFirst.Paths, coarseFirst.Paths));
        Assert.False(PolygonOps.Contains(nativeFirst.Paths, 16.5, 16.5));
        Assert.True(PolygonOps.Contains(nativeFirst.Paths, 24.0002, 14.00015));
        Assert.True(PolygonOps.Contains(coarseFirst.Paths, 10 - coarseFirst.RequiredGuard, 15));
    }

    [Fact]
    public void RequestedFirstFidelityBelowOneTickDoesNotSpendTheLaterConstructionBudget()
    {
        var options = new PocketOptions { ProtectedBoundary = [Box(0, 0, 10, 10)],
            Tolerances = new GeometryTolerances { ArcTolerance = 1 / (2 * PathUtils.Scale), SimplifyTolerance = .25 } };
        var domain = PocketProtection.PrepareConcentricMask(options, CancellationToken.None,
            denseFirstArcExperiment: false);
        Assert.Equal(options.Tolerances.ArcTolerance, domain.EffectiveArcTolerance);
        Assert.Equal(.001 + options.Tolerances.ArcTolerance + 4 / PathUtils.Scale, domain.RequiredGuard, 12);
        Assert.Empty(PolygonOps.Difference(options.ProtectedBoundary, domain.Paths));
    }

    [Fact]
    public void CanceledOuterMaskNeverPublishesUnprovedProtection()
    {
        using var canceled = new CancellationTokenSource();
        canceled.Cancel();
        Assert.Throws<OperationCanceledException>(() => PocketProtection.PrepareConcentricMask(
            new PocketOptions { ProtectedBoundary = [Box(0, 0, 20, 20)] }, canceled.Token));
    }

    [Theory]
    [InlineData(false, 0)]
    [InlineData(true, 0)]
    [InlineData(false, -.35)]
    [InlineData(true, -.35)]
    public void ConnectedProtectionIsSubtractedBeforeCompensationAndAllRampLinksRemainClear(
        bool scanline, double edgeOffset)
    {
        var protectedBay = Box(12, 4, 19, 16);
        var options = new PocketOptions
        {
            ProtectedBoundary = [protectedBay], StockToLeave = edgeOffset,
            StepOver = .65, InsideOut = true, DetectCurves = false, CalculateSweptArea = false,
            Tolerances = new GeometryTolerances { ArcTolerance = .01, SimplifyTolerance = .01 },
            Ramp = RampSettings.FromRatio(3)
        };
        var source = new MillingRegion(Box(0, 0, 20, 20), [Box(3, 7, 6, 12)]);
        var tool = ToolGeometry.Flat(.8);
        var depth = new DepthSchedule(3, 1) { SurfaceZ = 7 };
        var plan = scanline
            ? ScanlinePocketToolpaths.Generate([source], tool, depth, options, 27)
            : PocketToolpaths.Generate([source], tool, depth, options);
        Assert.NotEmpty(plan.Toolpaths);
        int checkedChords = 0;
        foreach (var path in plan.Toolpaths)
        for (int index = 1; index < path.Points.Count; index++)
        {
            var a = path.Points[index - 1];
            var b = path.Points[index];
            if (Math.Min(a.Z, b.Z) >= depth.SurfaceZ) continue;
            checkedChords++;
            // Independent, finely polygonized complete capsule oracle. Check all
            // chords, irrespective of whether the routing labels them cut/ramp/link.
            var capsule = PolygonOps.BufferCenterlines(
                [new List<PointD> { new(a.X, a.Y), new(b.X, b.Y) }], tool.Radius, .00001);
            Assert.Empty(PolygonOps.Intersect(capsule, [protectedBay]));
        }
        Assert.True(checkedChords > 20);
        Assert.Contains(plan.Toolpaths.SelectMany(path => path.Points), point => point.X < 5);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void EntireIsolatedRegionIsOmittedWithoutChangingItsNeighbour(bool scanline)
    {
        var ocean = new MillingRegion(Box(0, 0, 12, 12));
        var puddle = new MillingRegion(Box(20, 0, 23, 3));
        var options = new PocketOptions { ProtectedBoundary = [puddle.Outer], DetectCurves = false };
        var tool = ToolGeometry.Flat(.6);
        var depth = new DepthSchedule(2, .7);
        ToolpathPlan Generate(IReadOnlyList<MillingRegion> regions, PocketOptions settings) => scanline
            ? ScanlinePocketToolpaths.Generate(regions, tool, depth, settings)
            : PocketToolpaths.Generate(regions, tool, depth, settings);
        var coordinated = Generate([ocean, puddle], options);
        var expected = Generate([ocean], options with { ProtectedBoundary = null });
        Assert.Equal(expected.Toolpaths.SelectMany(path => path.Points),
            coordinated.Toolpaths.SelectMany(path => path.Points));
        Assert.Empty(Generate([puddle], options).Toolpaths);
    }

    [Fact]
    public void CompleteChordProofDetectsContactMissedByEndpointsAndHonoursNestedHoles()
    {
        var protection = PolygonOps.Difference([Box(4, 4, 16, 16)], [Box(7, 7, 13, 13)]);
        var proof = new PreparedSweepExclusion(protection);
        Assert.False(proof.IsClear(new(0, 10), new(20, 10), .1));
        Assert.False(proof.IsClear(new(3.5, 0), new(3.5, 20), .6));
        Assert.True(proof.IsClear(new(8, 10), new(12, 10), .9));
        Assert.False(proof.IsClear(new(8, 10), new(12, 10), 1.001));
        Assert.False(proof.IsClear(new(5, 5), new(5, 5), .1));
        Assert.True(proof.IsClear(new(0, 0), new(0, 0), .1));
        Assert.False(proof.IsClear(new(3, 4), new(3, 16), 1));
    }

    [Theory]
    [InlineData(false, .5)]
    [InlineData(false, 1)]
    [InlineData(false, 2)]
    [InlineData(true, .5)]
    [InlineData(true, 1)]
    [InlineData(true, 2)]
    public void PositiveLeaveIsCreditedOnceAtBelowEqualAndAboveTheLiteralSafetyReserve(bool denseFirst, double multiple)
    {
        InReserveMode(false, () =>
        {
            double arc = denseFirst ? .00025 : 1 / PathUtils.Scale;
            double safety = .001 + arc + 4 / PathUtils.Scale;
            double leave = multiple * safety;
            IReadOnlyList<PointD>[] raw = [Box(10, 10, 20, 20), Box(12, 12, 18, 18).Reverse().ToArray(),
                Box(14, 14, 15, 15), Box(24, 14, 24.0004, 14.0003)];
            var domain = PocketProtection.PrepareConcentricMask(new PocketOptions
                { ProtectedBoundary = raw, StockToLeave = leave }, CancellationToken.None, denseFirst);
            Assert.Equal(Math.Max(0, safety - leave), domain.RequiredGuard);
            Assert.True(domain.RequiredGuard + leave >= safety);
            Assert.True(domain.AppliedOffset >= domain.RequiredGuard + 9 / PathUtils.Scale);
            Assert.Empty(PolygonOps.Difference(raw, domain.Paths));
            Assert.False(PolygonOps.Contains(domain.Paths, 16.5, 16.5));
            Assert.True(PolygonOps.Contains(domain.Paths, 14.5, 14.5));
            Assert.True(PolygonOps.Contains(domain.Paths, 24.0002, 14.00015));
            var index = new PolygonOps.BoundarySegmentIndex(PathUtils.ToClipper(domain.Paths));
            foreach (var contour in PathUtils.ToClipper(raw))
                for (int end = 0, start = contour.Count - 1; end < contour.Count; start = end++)
                    Assert.True(index.ContainsCapsule(contour[start], contour[end], domain.RequiredGuard * PathUtils.Scale));
            return true;
        });
    }

    [Theory]
    [InlineData(false, 0)]
    [InlineData(true, 0)]
    [InlineData(false, -.35)]
    [InlineData(true, -.35)]
    [InlineData(false, -3.175)]
    [InlineData(true, -3.5)]
    public void ZeroAndNegativeLeaveRetainEveryReferenceMaskCoordinateByte(bool denseFirst, double leave)
    {
        var options = new PocketOptions { StockToLeave = leave,
            ProtectedBoundary = [Box(10, 10, 20, 20), Box(12, 12, 18, 18).Reverse().ToArray(), Box(14, 14, 15, 15)] };
        var actual = InReserveMode(false, () => PocketProtection.PrepareConcentricMask(options, CancellationToken.None, denseFirst));
        var reference = InReserveMode(true, () => PocketProtection.PrepareConcentricMask(options, CancellationToken.None, denseFirst));
        Assert.Equal(reference.RequiredGuard, actual.RequiredGuard);
        Assert.Equal(reference.AppliedOffset, actual.AppliedOffset);
        Assert.Equal(reference.ProofChords, actual.ProofChords);
        Assert.Equal(reference.ExpansionAttempts, actual.ExpansionAttempts);
        Assert.Equal(reference.Paths.Count, actual.Paths.Count);
        for (int i = 0; i < actual.Paths.Count; i++)
            Assert.True(MemoryMarshal.AsBytes(actual.Paths[i].ToArray().AsSpan()).SequenceEqual(
                MemoryMarshal.AsBytes(reference.Paths[i].ToArray().AsSpan())));
    }

    [Fact]
    public void ReferenceModeRestoresTheOldPositiveLeaveGuardAndCompleteRawProof()
    {
        var options = new PocketOptions { StockToLeave = 1.5875,
            ProtectedBoundary = [Box(10, 10, 20, 20)] };
        var actual = InReserveMode(false, () => PocketProtection.PrepareConcentricMask(options, CancellationToken.None, true));
        var reference = InReserveMode(true, () => PocketProtection.PrepareConcentricMask(options, CancellationToken.None, true));
        Assert.Equal(0, actual.RequiredGuard);
        Assert.Equal(.00125 + 4 / PathUtils.Scale, reference.RequiredGuard);
        Assert.True(reference.AppliedOffset > actual.AppliedOffset);
        Assert.Empty(PolygonOps.Difference(actual.Paths, reference.Paths));
    }

    [Theory]
    [InlineData(double.NaN)]
    [InlineData(double.PositiveInfinity)]
    [InlineData(double.NegativeInfinity)]
    public void NonfiniteStockAllowanceIsRejectedBeforeAnyEmptyOrFilledMaskIsPublished(double leave)
    {
        foreach (var raw in new IReadOnlyList<PointD>[][] { [], [Box(0, 0, 10, 10)] })
        {
            var failure = Assert.Throws<ArgumentOutOfRangeException>(() => PocketProtection.PrepareConcentricMask(
                new PocketOptions { StockToLeave = leave, ProtectedBoundary = raw }, CancellationToken.None));
            Assert.Equal("options", failure.ParamName);
        }
    }

    [Theory]
    [InlineData(0)]
    [InlineData(.0005)]
    [InlineData(.0013720703125)]
    [InlineData(.003)]
    [InlineData(1.5875)]
    [InlineData(-.35)]
    [InlineData(-3.175)]
    [InlineData(-3.5)]
    public void CompleteSourceClearanceAndF3PhysicalCylinderKeepCutRampAndLinkOutsideRawProtection(double leave)
    {
        InReserveMode(false, () =>
        {
            const double physicalRadius = 3.175;
            double nativeRadius = (double)(float)physicalRadius;
            double inset = Math.Max(0, nativeRadius + leave);
            IReadOnlyList<PointD>[] raw = [Box(10.0002, 10.0002, 20.0002, 20.0002)];
            var options = new PocketOptions { StockToLeave = leave, ProtectedBoundary = raw };
            var domain = PocketProtection.PrepareConcentricMask(options, CancellationToken.None, true);
            double sourceRight = domain.Paths.SelectMany(path => path).Max(point => point.x);
            double centerX = sourceRight + (inset > 8 / PathUtils.Scale ? inset - 3 / PathUtils.Scale : 5 / PathUtils.Scale);
            var regions = PolygonOps.DifferenceToComponents([Box(-50, -50, 50, 50)], domain.Paths)
                .Select(component => new MillingRegion(component.Outer,
                    component.Holes.Select(hole => (IReadOnlyList<PointD>)hole).ToArray())).ToArray();
            var sourceProof = new PocketToolpaths.FillSourceClearance(regions, inset, null, null);
            foreach (var role in new[] { ToolpathSpanKind.Cut, ToolpathSpanKind.Ramp, ToolpathSpanKind.Link })
            {
                double firstZ = role == ToolpathSpanKind.Ramp ? 0 : -1;
                var a = new Point3D(centerX, 12.0006, firstZ); var b = new Point3D(centerX, 18.0006, -1);
                Assert.True(sourceProof.IsTravelSafe(a, b, nativeRadius));
                var postedA = Posted(a); var postedB = Posted(b);
                // These parallel finite chords' exact complete distance to the
                // RAW rectangle is the horizontal separation, including endpoints.
                Assert.True(postedA.X - 20.0002 >= physicalRadius);
                Assert.Equal(postedA.X, postedB.X);
                var posted = new TaggedToolpath([postedA, postedB], [new ToolpathSpan(0, 1, role)], 0,
                    "posted-protection-control", 0, detectCurves: false);
                PocketProtection.Verify([posted], physicalRadius, 0, raw, CancellationToken.None);
                var unsafeA = Posted(a with { X = 20.0002 + physicalRadius - .0008 });
                var unsafeB = Posted(b with { X = unsafeA.X });
                Assert.False(sourceProof.IsTravelSafe(unsafeA, unsafeB, nativeRadius));
                var unsafePath = posted with { Points = new Point3D[] { unsafeA, unsafeB } };
                Assert.Throws<InvalidOperationException>(() => PocketProtection.Verify([unsafePath],
                    physicalRadius, 0, raw, CancellationToken.None));
            }
            return true;
        });
    }

    private static Point3D Posted(Point3D point) => new(
        Math.Round((double)(float)point.X, 3, MidpointRounding.ToEven),
        Math.Round((double)(float)point.Y, 3, MidpointRounding.ToEven),
        Math.Round((double)(float)point.Z, 3, MidpointRounding.ToEven));

    private static T InReserveMode<T>(bool reference, Func<T> action)
    {
        const string flag = "RW_TOOLPATHS_PROTECTION_RESERVE_REFERENCE";
        string? previous = Environment.GetEnvironmentVariable(flag);
        try { Environment.SetEnvironmentVariable(flag, reference ? "1" : null); return action(); }
        finally { Environment.SetEnvironmentVariable(flag, previous); }
    }

    private static IReadOnlyList<PointD> Box(double x0, double y0, double x1, double y1) =>
        [new(x0, y0), new(x1, y0), new(x1, y1), new(x0, y1)];
}
