using System.Reflection;
using Clipper2Lib;
using RW.Toolpaths.Milling;
using Cert = RW.Toolpaths.Milling.VCarveCoverageCertificate;

namespace RW.Toolpaths.Tests;

public sealed class VCarveSphereDiscBoundsTests
{
    [Fact]
    public void EnclosureAndSupportBoundEveryLiteralPoseAcrossDepthDesiredAndScale()
    {
        var cases = new[] {
            new Case(Ball(), new(0, 0, 0), new(1, 0, .001), new(.5, .05), .004),
            new Case(Ball(), new(1, 0, .001), new(0, 0, 0), new(.5, .05), 0),
            new Case(Ball(), new(0, 0, .001), new(1, 0, .002), new(.5, .08), -.02),
            new Case(Ball(maximumDepth: .0005), new(0, 0, .001), new(1, 0, .002), new(.5, .03), -.01),
            new Case(Ball(2), new(0, 0, .5), new(1, 1, 1.5), new(.4, .8), -.1),
            new Case(Ball(100), new(9998, -9999, 1e-9), new(10000, -9999, .01), new(9998.9, -9998.4), -.01),
            new Case(Ball(1e-6), new(0, 0, 1e-8), new(1e-6, 0, 8e-7), new(.5e-6, 1e-6), 0),
            new Case(Ball(1), new(0, 0, 1 - 2e-8), new(1, 0, 1 - 1.5e-8), new(.5, 1.000001), 0),
            new Case(Ball(1), new(0, 0, 1 - 2e-8), new(1, 0, 1 - 1.5e-8), new(.5, 1), 0),
            new Case(Ball(1), new(0, 0, .2), new(1, 0, .8), new(.5, Math.Sqrt(.75) - .1), -.1) };
        foreach (var item in cases) AssertSampledBounds(item, 16_384, mustBound: true);
        var random = new Random(739283);
        for (int i = 0; i < 100; i++)
        {
            double radius = .2 + random.NextDouble() * 3;
            var a = new Point3D(random.NextDouble() * 4 - 2, random.NextDouble() * 4 - 2, random.NextDouble() * .9 * radius);
            var b = new Point3D(random.NextDouble() * 4 - 2, random.NextDouble() * 4 - 2, random.NextDouble() * .9 * radius);
            var q = new PointD(random.NextDouble() * 4 - 2, random.NextDouble() * 4 - 2);
            AssertSampledBounds(new(Ball(radius), a, b, q, random.NextDouble() * .1 - .05), 2_048, mustBound: true);
        }
    }

    [Theory]
    [InlineData(0)]
    [InlineData(.004)]
    [InlineData(-.005)]
    public void CertifiedRoundedTipMissSkipsAllFortyNegativeSearchSteps(double desired)
    {
        var profile = Ball(); var q = new PointD(.5, .05);
        var reference = BuildTree(new(0, 0, 0), new(1, 0, .001), profile, false);
        var actual = BuildTree(new(0, 0, 0), new(1, 0, .001), profile, true);
        var expected = Query(reference.Tree, q, desired, profile);
        Assert.False(expected.Accepted); Assert.Equal(expected, Query(actual.Tree, q, desired, profile));
        Assert.Equal(45, Discs(reference.Work)); Assert.Equal(3, Discs(actual.Work));
        Assert.Equal(1, Counter(actual.Work, "SphereDiscCalls")); Assert.Equal(1, Counter(actual.Work, "SphereDiscRejections"));
        Assert.Equal(0, Counter(actual.Work, "SphereDiscFallbacks")); Assert.Equal(0, Counter(actual.Work, "SphereDiscGuardRefusals"));
    }

    [Fact]
    public void CompleteNegativeThenPositiveOwnerKeepsTheSameBoxAndOwner()
    {
        var q = new PointD(.5, .05); double half = .0025 / Math.Sqrt(2);
        IReadOnlyList<IReadOnlyList<PointD>> target = [Rectangle(q.x - half, q.y - half, q.x + half, q.y + half)];
        IReadOnlyList<TaggedToolpath> paths = [Path(new(0, 0, 0), new(1, 0, -.001)),
            Path(new(q.x, q.y, -.002), new(q.x, q.y, -.002))];
        var limits = new Cert.Limits { RecordWitnessPathIndices = true };
        var reference = Cert.Certify(target, paths, Ball(), limits: limits);
        var logs = new List<string>();
        var actual = Cert.Certify(target, paths, Ball(), limits: limits with { UseSphereDiscPruning = true }, diagnostic: logs.Add);
        AssertSame(reference, actual); Assert.True(actual.Certified, actual.ToString());
        Assert.Equal(1, actual.VisitedBoxes); Assert.Equal(new[] { 1 }, actual.WitnessPathIndices);
        Assert.Equal(48, reference.DiscEvaluations); Assert.Equal(6, actual.DiscEvaluations);
        Assert.Equal(1, actual.SphereDiscRejections);
        string completion = Assert.Single(logs.Where(line => line.Contains("event=complete")));
        Assert.Contains("sphereDiscCalls=1 ", completion); Assert.Contains("sphereDiscRejections=1 ", completion);
        Assert.Contains("sphereDiscFallbacks=0 ", completion); Assert.Contains("sphereDiscGuardRefusals=0 ", completion);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ExistingSphericalCertificatePreservesLiteralSearchBoxesAndMandatoryOwner(bool reverse)
    {
        Point3D a = new(0, 0, 0), b = new(1, 0, -.001);
        var path = reverse ? Path(b, a) : Path(a, b);
        var limits = new Cert.Limits { RecordWitnessPathIndices = true, OptionalWitnessPathIndices = new HashSet<int> { 0 } };
        IReadOnlyList<IReadOnlyList<PointD>> target = [Rectangle(.49, .033, .51, .036)];
        var reference = Cert.Certify(target, [path, path], Ball(), limits: limits);
        var actual = Cert.Certify(target, [path, path], Ball(), limits: limits with { UseSphereDiscPruning = true });
        AssertSame(reference, actual); Assert.True(actual.Certified, actual.ToString());
        Assert.Equal(reference.DiscEvaluations, actual.DiscEvaluations); Assert.Equal(96, actual.DiscEvaluations);
        Assert.Equal(new[] { 1 }, actual.WitnessPathIndices); Assert.Equal(0, actual.SphereDiscRejections);
    }

    [Theory]
    [InlineData(-1e-12)]
    [InlineData(0)]
    [InlineData(1e-12)]
    public void NearTangencyCannotSupplyAFalseNegative(double margin)
    {
        var profile = Ball(1);
        var a = new Point3D(0, 0, .2); var b = new Point3D(1, 0, .8);
        double radius = profile.PhysicalRadiusAtDepth(.5);
        var item = new Case(profile, a, b, new(.5, radius + margin), 0);
        AssertSampledBounds(item, 16_384, mustBound: true);
        Assert.True(VCarveSphereDiscBounds.TryBounds(item.Q, a, b, profile, 0, out double lower, out double upper));
        Assert.False(lower > upper);
    }

    [Fact]
    public void UnsupportedShapeDomainAndUpperJunctionRetainTheOriginalSearch()
    {
        var cases = new[] {
            new Case(Ball(), new(0, 0, 0), new(1, 0, .001), new(Math.BitIncrement(10000d), .05), 0),
            new Case(Ball(), new(0, 0, -.0001), new(1, 0, .001), new(.5, .05), 0),
            new Case(Ball(), new(0, 0, double.NaN), new(1, 0, .001), new(.5, .05), 0),
            new Case(Ball(), new(0, 0, 0), new(1, 0, .001), new(.5, .05), double.PositiveInfinity),
            new Case(Ball(101), new(0, 0, 0), new(1, 0, .001), new(.5, .05), 0),
            new Case(Ball(1), new(0, 0, .2), new(1, 0, 1 - 5e-9), new(.5, .05), 0),
            new Case(Ball(1), new(0, 0, .2), new(1, 0, 1), new(.5, .05), 0),
            new Case(new([new(CarvingCutterSectionKind.Linear, 0, 1, 1)], 1),
                new(0, 0, .2), new(1, 0, .8), new(.5, .05), 0) };
        foreach (var item in cases)
        {
            Assert.False(VCarveSphereDiscBounds.TryBounds(item.Q, item.A, item.B, item.Profile, item.Desired, out double lower, out double upper));
            Assert.Equal(double.NegativeInfinity, lower); Assert.Equal(double.PositiveInfinity, upper);
        }
        var cone = cases[^1];
        var reference = BuildTree(cone.A, cone.B, cone.Profile, false);
        var actual = BuildTree(cone.A, cone.B, cone.Profile, true);
        double desired = -.49;
        Assert.Equal(Query(reference.Tree, cone.Q, desired, cone.Profile), Query(actual.Tree, cone.Q, desired, cone.Profile));
        Assert.Equal(Discs(reference.Work), Discs(actual.Work));
        Assert.Equal(1, Counter(actual.Work, "SphereDiscGuardRefusals"));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void OriginalDiscQuotaDeadlineAndCancellationRemainAuthoritative(bool enabled)
    {
        Assert.False(new Cert.Limits().UseSphereDiscPruning);
        IReadOnlyList<IReadOnlyList<PointD>> target = [Rectangle(.49, .033, .51, .036)];
        IReadOnlyList<TaggedToolpath> paths = [Path(new(0, 0, 0), new(1, 0, -.001))];
        var limits = new Cert.Limits { UseSphereDiscPruning = enabled, MaximumDiscEvaluations = 3 };
        var result = Cert.Certify(target, paths, Ball(), limits: limits);
        Assert.False(result.Certified); Assert.Equal("disc-budget", result.Reason); Assert.Equal(3, result.DiscEvaluations);
        var timed = Cert.Certify(target, paths, Ball(), limits: limits with { MaximumElapsed = TimeSpan.FromTicks(1) });
        Assert.False(timed.Certified); Assert.Equal("time-budget", timed.Reason);
        using var cancellation = new CancellationTokenSource(); cancellation.Cancel();
        Assert.Throws<OperationCanceledException>(() => Cert.Certify(target, paths, Ball(), limits: limits,
            cancellationToken: cancellation.Token));
    }

    private sealed record Case(VCarveCutterProfile Profile, Point3D A, Point3D B, PointD Q, double Desired);
    private static void AssertSampledBounds(Case item, int samples, bool mustBound)
    {
        bool bounded = VCarveSphereDiscBounds.TryBounds(item.Q, item.A, item.B, item.Profile, item.Desired, out double lower, out double upper);
        if (mustBound) Assert.True(bounded, item.ToString());
        if (!bounded) return;
        double radius = item.Profile.PhysicalProfile.Sections[0].TopRadius;
        for (int i = 0; i <= samples; i++)
        {
            double t = i / (double)samples;
            var p = new Point3D(item.A.X + (item.B.X - item.A.X) * t,
                item.A.Y + (item.B.Y - item.A.Y) * t, item.A.Z + (item.B.Z - item.A.Z) * t);
            double dx = p.X - item.Q.x, dy = p.Y - item.Q.y, dz = radius - p.Z;
            double centerDistance = Math.Sqrt(dx * dx + dy * dy + dz * dz);
            Assert.True(lower <= centerDistance, $"lower={lower:R} center={centerDistance:R} t={t:R} case={item}");
            double value = p.Z <= 0 ? double.PositiveInfinity
                : Math.Sqrt(dx * dx + dy * dy) - item.Profile.PhysicalRadiusAtDepth(p.Z);
            if (value <= item.Desired) Assert.True(centerDistance <= upper,
                $"upper={upper:R} center={centerDistance:R} value={value:R} t={t:R} case={item}");
            if (lower > upper) Assert.True(value > item.Desired,
                $"False exclusion: value={value:R} desired={item.Desired:R} t={t:R} case={item}");
        }
    }
    private static void AssertSame(Cert.Result reference, Cert.Result actual)
    {
        Assert.Equal(reference.Certified, actual.Certified); Assert.Equal(reference.Reason, actual.Reason);
        Assert.Equal(reference.VisitedBoxes, actual.VisitedBoxes); Assert.Equal(reference.RemainingBoxes, actual.RemainingBoxes);
        Assert.Equal(reference.UnresolvedPoint, actual.UnresolvedPoint); Assert.Equal(reference.CommandIntervals, actual.CommandIntervals);
        Assert.Equal(reference.WitnessPathsRecorded, actual.WitnessPathsRecorded); Assert.Equal(reference.WitnessPathIndices, actual.WitnessPathIndices);
    }
    private sealed record TreeObjects(object Tree, object Work);
    private const BindingFlags Flags = BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic;
    private static TreeObjects BuildTree(Point3D a, Point3D b, VCarveCutterProfile profile, bool enabled)
    {
        var segmentType = typeof(Cert).GetNestedType("Segment", BindingFlags.NonPublic)!;
        var input = Array.CreateInstance(segmentType, 1);
        input.SetValue(segmentType.GetConstructors(Flags).Single().Invoke(new object[] {
            a, b, profile.PhysicalRadiusAtDepth(Math.Max(a.Z, b.Z)), 7 }), 0);
        var workType = typeof(Cert).GetNestedType("Budget", BindingFlags.NonPublic)!;
        var work = workType.GetConstructors(Flags).Single().Invoke(new object[] {
            new Cert.Limits { UseSphereDiscPruning = enabled }, CancellationToken.None });
        var treeType = typeof(Cert).GetNestedType("Tree", BindingFlags.NonPublic)!;
        return new(treeType.GetConstructors(Flags).Single().Invoke(new object[] { input, work }), work);
    }
    private static (bool Accepted, int Path) Query(object tree, PointD q, double desired, VCarveCutterProfile profile)
    {
        object[] arguments = { q, desired, profile, 0 };
        bool accepted = (bool)tree.GetType().GetMethod("HasDiscWitness", Flags)!.Invoke(tree, arguments)!;
        return (accepted, (int)arguments[3]);
    }
    private static long Counter(object work, string name) => (long)work.GetType().GetField(name, Flags)!.GetValue(work)!;
    private static long Discs(object work) => (long)work.GetType().GetProperty("DiscEvaluations", Flags)!.GetValue(work)!;
    private static VCarveCutterProfile Ball(double radius = 1.5875, double? maximumDepth = null)
        => new([new(CarvingCutterSectionKind.Hemisphere, 0, radius, radius)], maximumDepth ?? radius);
    private static PointD[] Rectangle(double x0, double y0, double x1, double y1)
        => [new(x0, y0), new(x1, y0), new(x1, y1), new(x0, y1)];
    private static TaggedToolpath Path(params Point3D[] points)
        => new(points, Array.Empty<ToolpathSpan>(), 0, "final-carve", null, detectCurves: false);
}
