using System.Reflection;
using Clipper2Lib;
using RW.Toolpaths.Milling;
using Cert = RW.Toolpaths.Milling.VCarveCoverageCertificate;

namespace RW.Toolpaths.Tests;

public sealed class VCarveAffineRadiusBoundsTests
{
    [Fact]
    public void CertifiedLowerBoundsEveryLiteralDiscSampleAcrossScaleSlopeAndDirection()
    {
        var cases = new[] {
            new Case(Cone(), new(0, 0, .2), new(2, 0, 1.2), new(.9, .6)),
            new Case(Cone(), new(2, 0, 1.2), new(0, 0, .2), new(.9, .6)),
            new Case(Cone(.5), new(0, 0, 1.2), new(2, 0, 1.8), new(.9, .6)),
            new Case(Cone(), new(9998, -9999, .2), new(10000, -9999, 1.2), new(9998.9, -9998.4)),
            new Case(Cone(), new(-10000, 9999, .2), new(-9998, 9999, 1.2), new(-9999.1, 9999.6)),
            new Case(Cone(), new(0, 0, .2), new(2, 0, 1.2), new(.5, 1e-9)),
            new Case(new([new(CarvingCutterSectionKind.Linear, 0, .999, .0001)], .0001),
                new(-10000, 9999, .00000004), new(-9998, 9999, .00009996), new(-9999, 9999.8)),
            new Case(new([new(CarvingCutterSectionKind.Linear, 1, 1, 4)], 4),
                new(0, 0, .2), new(2, 0, 1.2), new(.9, .6)),
            // Sections beyond the nominal cap are physical authority even when
            // their active radius stays below an already-completed section.
            new Case(new([new(CarvingCutterSectionKind.Linear, 0, 1, 1),
                new(CarvingCutterSectionKind.Linear, .5, .75, 1)], .5),
                new(0, 0, 1.2), new(2, 0, 1.8), new(.9, .6)),
            new Case(new([new(CarvingCutterSectionKind.Linear, 0, 1, 1),
                new(CarvingCutterSectionKind.Linear, .5, 1.5, 1)], .5),
                new(0, 0, 1.2), new(2, 0, 1.8), new(.9, .6)),
            new Case(Stack(), new(0, 0, 1.00000002), new(2, 0, 2.8), new(.9, .6)),
            new Case(new(Enumerable.Range(0, 8).Select(i => new CarvingCutterSection(
                CarvingCutterSectionKind.Linear, i / 10d, (i + 1) / 10d, 1)), 8),
                new(0, 0, 7.2), new(2, 0, 7.8), new(.9, .6)) };
        foreach (var item in cases) AssertSampledLower(item, 16_384);

        var random = new Random(308291);
        for (int i = 0; i < 80; i++)
        {
            var a = new Point3D(random.NextDouble() * 10 - 5, random.NextDouble() * 10 - 5, .1 + random.NextDouble() * 3.8);
            var b = new Point3D(random.NextDouble() * 10 - 5, random.NextDouble() * 10 - 5, .1 + random.NextDouble() * 3.8);
            var q = new PointD(random.NextDouble() * 10 - 5, random.NextDouble() * 10 - 5);
            AssertSampledLower(new(Cone(), a, b, q), 2_048);
        }
    }

    [Theory]
    [InlineData(2e-8, true)]
    [InlineData(5e-9, false)]
    [InlineData(0, false)]
    [InlineData(-5e-9, false)]
    public void JunctionPremiseRefusesUncertainOrCrossSectionIntervals(double separation, bool expected)
    {
        var item = new Case(Stack(), new(0, 0, .2), new(2, 0, 1 - separation), new(.9, .6));
        bool bounded = VCarveAffineRadiusBounds.TryLowerBound(item.Q, item.A, item.B, item.Profile, out double lower);
        Assert.Equal(expected, bounded);
        if (bounded) AssertSampledLower(item, 16_384);
        else Assert.Equal(double.NegativeInfinity, lower);
    }

    [Fact]
    public void UnsupportedAndDegeneratePremisesSupplyNoNegativeAuthority()
    {
        var cases = new[] {
            new Case(Cone(), new(0, 0, .2), new(0, 0, 1.2), new(.9, .6)),
            new Case(Cone(), new(1e-200, 0, .2), new(2e-200, 0, 1.2), new(.9, .6)),
            new Case(Cone(), new(0, 0, .2), new(2, 0, 1.2), new(1, 0)),
            new Case(Cone(), new(0, 0, -.2), new(2, 0, 1.2), new(.9, .6)),
            new Case(Cone(), new(0, 0, double.PositiveInfinity), new(2, 0, 1.2), new(.9, .6)),
            new Case(Cone(), new(0, 0, .2), new(2, 0, 1.2), new(double.NaN, .6)),
            new Case(Cone(), new(0, 0, .2), new(2, 0, 1.2), new(Math.BitIncrement(10000d), .6)),
            new Case(new([new(CarvingCutterSectionKind.Hemisphere, 0, 1, 1)], 1),
                new(0, 0, .2), new(2, 0, .8), new(.9, .6)),
            new Case(new([new(CarvingCutterSectionKind.Linear, 0, 1, .00001)], .00001),
                new(0, 0, .000002), new(2, 0, .000008), new(.9, .6)),
            new Case(new([new(CarvingCutterSectionKind.Linear, 0, 2, .0001)], .0001),
                new(0, 0, .00002), new(2, 0, .00008), new(.9, .6)),
            new Case(new(Enumerable.Range(0, 9).Select(i => new CarvingCutterSection(
                CarvingCutterSectionKind.Linear, i, i + 1, 1)), 9),
                new(0, 0, .2), new(2, 0, .8), new(.9, .6)) };
        foreach (var item in cases)
        {
            Assert.False(VCarveAffineRadiusBounds.TryLowerBound(item.Q, item.A, item.B, item.Profile, out double lower));
            Assert.Equal(double.NegativeInfinity, lower);
        }
    }

    [Fact]
    public void CertifiedMissSkipsOnlyOldNegativeSearchBeforePositiveNomination()
    {
        var profile = Cone(); var q = new PointD(.9, .6);
        double desired = OptimalConeValue(q) - .001;
        var reference = BuildTree(new(0, 0, .2), new(2, 0, 1.2), profile, new());
        var actual = BuildTree(new(0, 0, .2), new(2, 0, 1.2), profile,
            new() { UseAffineRadiusPruning = true, UseAnalyticPoseNomination = true });
        var expected = Query(reference.Tree, q, desired, profile);
        Assert.False(expected.Accepted); Assert.Equal(expected, Query(actual.Tree, q, desired, profile));
        Assert.Equal(45, Discs(reference.Work)); Assert.Equal(3, Discs(actual.Work));
        Assert.Equal(1, Counter(actual.Work, "AffineRadiusCalls"));
        Assert.Equal(1, Counter(actual.Work, "AffineRadiusRejections"));
        Assert.Equal(0, Counter(actual.Work, "AffineRadiusFallbacks"));
        Assert.Equal(0, Counter(actual.Work, "AnalyticPoseCalls"));
    }

    [Fact]
    public void GuardDeductionRetainsAllOldSearchWorkOnANearBoundaryMiss()
    {
        var profile = Cone(); var q = new PointD(.9, .6);
        var reference = BuildTree(new(0, 0, .2), new(2, 0, 1.2), profile, new());
        var actual = BuildTree(new(0, 0, .2), new(2, 0, 1.2), profile, new() { UseAffineRadiusPruning = true });
        double desired = OptimalConeValue(q) - 1e-6;
        var expected = Query(reference.Tree, q, desired, profile);
        Assert.False(expected.Accepted); Assert.Equal(expected, Query(actual.Tree, q, desired, profile));
        Assert.Equal(45, Discs(reference.Work)); Assert.Equal(45, Discs(actual.Work));
        Assert.Equal(1, Counter(actual.Work, "AffineRadiusFallbacks"));
        Assert.Equal(0, Counter(actual.Work, "AffineRadiusGuardRefusals"));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CompleteCertificatesKeepOwnersBoxesAndPositiveSearchSamples(bool reverse)
    {
        var q = new PointD(.9, .6);
        double half = (.004 - OptimalConeValue(q) - 1e-9 - Cert.ArithmeticReserve) / Math.Sqrt(2);
        Point3D a = new(0, 0, -.2), b = new(2, 0, -1.2);
        var path = reverse ? Path(b, a) : Path(a, b);
        Compare([Rectangle(q.x - half, q.y - half, q.x + half, q.y + half)],
            [path, path], Cone(), new() { RecordWitnessPathIndices = true, OptionalWitnessPathIndices = new HashSet<int> { 0 } });
        var sphere = new VCarveCutterProfile([new(CarvingCutterSectionKind.Hemisphere, 0, 1.5875, 1.5875),
            new(CarvingCutterSectionKind.Linear, 1.5875, 3, 20)], 21.5875);
        Compare([Rectangle(.49, .033, .51, .036)], [Path(new(0, 0, 0), new(1, 0, -.001))], sphere,
            new() { RecordWitnessPathIndices = true });
    }

    [Fact]
    public void CompleteMissPreservesBoxDecisionAndReportsSavedSamples()
    {
        var q = new PointD(.9, .6);
        double half = (.004 - OptimalConeValue(q) + .001 - Cert.ArithmeticReserve) / Math.Sqrt(2);
        IReadOnlyList<IReadOnlyList<PointD>> target = [Rectangle(q.x - half, q.y - half, q.x + half, q.y + half)];
        IReadOnlyList<TaggedToolpath> paths = [Path(new(0, 0, -.2), new(2, 0, -1.2))];
        var limits = new Cert.Limits { MaximumBoxes = 1, RecordWitnessPathIndices = true };
        var reference = Cert.Certify(target, paths, Cone(), limits: limits);
        var logs = new List<string>();
        var actual = Cert.Certify(target, paths, Cone(), limits: limits with { UseAffineRadiusPruning = true }, diagnostic: logs.Add);
        AssertSameOutcome(reference, actual);
        Assert.False(actual.Certified); Assert.Equal("box-budget", actual.Reason);
        Assert.Equal(45, reference.DiscEvaluations); Assert.Equal(3, actual.DiscEvaluations);
        Assert.Equal(1, actual.AffineRadiusRejections);
        string complete = Assert.Single(logs.Where(line => line.Contains("event=complete")));
        Assert.Contains("affineRadiusCalls=1 ", complete); Assert.Contains("affineRadiusRejections=1 ", complete);
        Assert.Contains("affineRadiusFallbacks=0 ", complete); Assert.Contains("affineRadiusGuardRefusals=0 ", complete);
    }

    [Fact]
    public void EarlierCertifiedNegativeKeepsTheLaterPositiveOwnerInACompleteCertificate()
    {
        var q = new PointD(.9, .6);
        double half = (.004 - OptimalConeValue(q) + .001 - Cert.ArithmeticReserve) / Math.Sqrt(2);
        IReadOnlyList<IReadOnlyList<PointD>> target = [Rectangle(q.x - half, q.y - half, q.x + half, q.y + half)];
        IReadOnlyList<TaggedToolpath> paths = [Path(new(0, 0, -.2), new(2, 0, -1.2)),
            Path(new(q.x, q.y, -.3), new(q.x, q.y, -.3))];
        var limits = new Cert.Limits { RecordWitnessPathIndices = true };
        var reference = Cert.Certify(target, paths, Cone(), limits: limits);
        var actual = Cert.Certify(target, paths, Cone(), limits: limits with { UseAffineRadiusPruning = true });
        AssertSameOutcome(reference, actual); Assert.True(actual.Certified, actual.ToString());
        Assert.Equal(1, actual.VisitedBoxes); Assert.Equal(new[] { 1 }, actual.WitnessPathIndices);
        Assert.Equal(48, reference.DiscEvaluations); Assert.Equal(6, actual.DiscEvaluations);
        Assert.Equal(1, actual.AffineRadiusRejections);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void PositiveDiscQuotaClockAndCancellationRemainAuthoritative(bool enabled)
    {
        Assert.False(new Cert.Limits().UseAffineRadiusPruning);
        IReadOnlyList<IReadOnlyList<PointD>> target = [Rectangle(.82, .52, .98, .68)];
        IReadOnlyList<TaggedToolpath> paths = [Path(new(0, 0, -.2), new(2, 0, -1.2))];
        var limits = new Cert.Limits { UseAffineRadiusPruning = enabled, MaximumDiscEvaluations = 3 };
        var result = Cert.Certify(target, paths, Cone(), limits: limits);
        Assert.False(result.Certified); Assert.Equal("disc-budget", result.Reason); Assert.Equal(3, result.DiscEvaluations);
        var timed = Cert.Certify(target, paths, Cone(), limits: limits with { MaximumElapsed = TimeSpan.FromTicks(1) });
        Assert.False(timed.Certified); Assert.Equal("time-budget", timed.Reason);
        using var cancellation = new CancellationTokenSource(); cancellation.Cancel();
        Assert.Throws<OperationCanceledException>(() => Cert.Certify(target, paths, Cone(), limits: limits,
            cancellationToken: cancellation.Token));
    }

    private sealed record Case(VCarveCutterProfile Profile, Point3D A, Point3D B, PointD Q);
    private static void AssertSampledLower(Case item, int samples)
    {
        Assert.True(VCarveAffineRadiusBounds.TryLowerBound(item.Q, item.A, item.B, item.Profile, out double lower));
        for (int i = 0; i <= samples; i++)
        {
            double t = i / (double)samples;
            var p = new Point3D(item.A.X + (item.B.X - item.A.X) * t,
                item.A.Y + (item.B.Y - item.A.Y) * t, item.A.Z + (item.B.Z - item.A.Z) * t);
            double value = Math.Sqrt((p.X - item.Q.x) * (p.X - item.Q.x) + (p.Y - item.Q.y) * (p.Y - item.Q.y))
                - item.Profile.PhysicalRadiusAtDepth(p.Z);
            Assert.True(lower <= value, $"lower={lower:R} value={value:R} t={t:R} case={item}");
        }
    }
    private static void Compare(IReadOnlyList<IReadOnlyList<PointD>> target, IReadOnlyList<TaggedToolpath> paths,
        VCarveCutterProfile profile, Cert.Limits limits)
    {
        var reference = Cert.Certify(target, paths, profile, limits: limits);
        var actual = Cert.Certify(target, paths, profile, limits: limits with { UseAffineRadiusPruning = true });
        AssertSameOutcome(reference, actual); Assert.True(actual.Certified, actual.ToString());
        Assert.Equal(reference.DiscEvaluations, actual.DiscEvaluations); Assert.Equal(0, actual.AffineRadiusRejections);
    }
    private static void AssertSameOutcome(Cert.Result reference, Cert.Result actual)
    {
        Assert.Equal(reference.Certified, actual.Certified); Assert.Equal(reference.Reason, actual.Reason);
        Assert.Equal(reference.VisitedBoxes, actual.VisitedBoxes); Assert.Equal(reference.RemainingBoxes, actual.RemainingBoxes);
        Assert.Equal(reference.UnresolvedPoint, actual.UnresolvedPoint); Assert.Equal(reference.CommandIntervals, actual.CommandIntervals);
        Assert.Equal(reference.WitnessPathsRecorded, actual.WitnessPathsRecorded); Assert.Equal(reference.WitnessPathIndices, actual.WitnessPathIndices);
    }
    private sealed record TreeObjects(object Tree, object Work);
    private const BindingFlags Flags = BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic;
    private static TreeObjects BuildTree(Point3D a, Point3D b, VCarveCutterProfile profile, Cert.Limits limits)
    {
        var segmentType = typeof(Cert).GetNestedType("Segment", BindingFlags.NonPublic)!;
        var input = Array.CreateInstance(segmentType, 1);
        input.SetValue(segmentType.GetConstructors(Flags).Single().Invoke(new object[] {
            a, b, profile.PhysicalRadiusAtDepth(Math.Max(a.Z, b.Z)), 7 }), 0);
        var workType = typeof(Cert).GetNestedType("Budget", BindingFlags.NonPublic)!;
        var work = workType.GetConstructors(Flags).Single().Invoke(new object[] { limits, CancellationToken.None });
        var treeType = typeof(Cert).GetNestedType("Tree", BindingFlags.NonPublic)!;
        return new(treeType.GetConstructors(Flags).Single().Invoke(new object[] { input, work }), work);
    }
    private static (bool Accepted, int Path) Query(object tree, PointD point, double desired, VCarveCutterProfile profile)
    {
        object[] arguments = { point, desired, profile, 0 };
        bool accepted = (bool)tree.GetType().GetMethod("HasDiscWitness", Flags)!.Invoke(tree, arguments)!;
        return (accepted, (int)arguments[3]);
    }
    private static long Counter(object work, string name) => (long)work.GetType().GetField(name, Flags)!.GetValue(work)!;
    private static long Discs(object work) => (long)work.GetType().GetProperty("DiscEvaluations", Flags)!.GetValue(work)!;
    private static double OptimalConeValue(PointD q)
    {
        double t = q.x / 2 + Math.Abs(q.y) / Math.Sqrt(12);
        return Math.Sqrt(Math.Pow(2 * t - q.x, 2) + q.y * q.y) - (.2 + t);
    }
    private static VCarveCutterProfile Cone(double maximumDepth = 4)
        => new([new(CarvingCutterSectionKind.Linear, 0, 4, 4)], maximumDepth);
    private static VCarveCutterProfile Stack()
        => new([new(CarvingCutterSectionKind.Linear, 0, 1, 1), new(CarvingCutterSectionKind.Linear, 1, 3, 2)], 3);
    private static PointD[] Rectangle(double x0, double y0, double x1, double y1)
        => [new(x0, y0), new(x1, y0), new(x1, y1), new(x0, y1)];
    private static TaggedToolpath Path(params Point3D[] points)
        => new(points, Array.Empty<ToolpathSpan>(), 0, "final-carve", null, detectCurves: false);
}
