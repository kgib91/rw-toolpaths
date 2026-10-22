using System.Reflection;
using Clipper2Lib;
using RW.Toolpaths.Milling;
using Cert = RW.Toolpaths.Milling.VCarveCoverageCertificate;

namespace RW.Toolpaths.Tests;

public sealed class VCarvePreparedSphereDiscTests
{
    [Fact]
    public void UnnormalizedSquaredProofCannotRejectAnyLiteralPassingPose()
    {
        var random = new Random(972831);
        for (int i = 0; i < 96; i++)
        {
            double radius = i == 0 ? 100 : .2 + random.NextDouble() * 3;
            var profile = Ball(radius);
            var a = new Point3D(random.NextDouble() * 4 - 2, random.NextDouble() * 4 - 2, i == 1 ? 0 : random.NextDouble() * .9 * radius);
            var b = new Point3D(random.NextDouble() * 4 - 2, random.NextDouble() * 4 - 2, random.NextDouble() * .9 * radius);
            var q = new PointD(random.NextDouble() * 4 - 2, random.NextDouble() * 4 - 2);
            double desired = random.NextDouble() * .1 - .05;
            AssertSampled(profile, a, b, q, desired);
        }
        AssertSampled(Ball(), new(9998, -9999, 0), new(10000, -9999, .001), new(9998.9, -9998.4), .004);
        AssertSampled(Ball(), new(1, 0, .001), new(0, 0, 0), new(.5, .05), -.005);
        AssertSampled(Ball(1), new(0, 0, 1 - 2e-8), new(1, 0, 1 - 1.5e-8), new(.5, 1), 0);
        AssertSampled(Ball(1e-6), new(0, 0, 1e-8), new(1e-6, 0, 8e-7), new(.5e-6, 1e-6), 0);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(.004)]
    [InlineData(-.005)]
    public void PreparedNegativeUsesThreeOriginalSamplesAndReusesItsInterval(double desired)
    {
        var profile = Ball(); var q = new PointD(.5, .05);
        var reference = BuildTree(new(0, 0, 0), new(1, 0, .001), profile, new());
        var actual = BuildTree(new(0, 0, 0), new(1, 0, .001), profile, new() { UsePreparedSphereDiscPruning = true });
        var expected = Query(reference.Tree, q, desired, profile);
        Assert.False(expected.Accepted); Assert.Equal(expected, Query(actual.Tree, q, desired, profile));
        Assert.Equal(45, Discs(reference.Work)); Assert.Equal(3, Discs(actual.Work));
        Assert.Equal(1, Counter(actual.Work, "SpherePreparedIntervals")); Assert.Equal(1, Counter(actual.Work, "SpherePreparedSlots"));
        Assert.Equal(1, Counter(actual.Work, "SphereDiscRejections"));
        Assert.False(Query(actual.Tree, q, desired, profile).Accepted);
        Assert.Equal(6, Discs(actual.Work)); Assert.Equal(2, Counter(actual.Work, "SphereDiscRejections"));
        Assert.Equal(1, Counter(actual.Work, "SpherePreparedIntervals"));
    }

    [Fact]
    public void SparsePreparedOrdinalsSurviveBvhPartitionAndRetainAllOwners()
    {
        IReadOnlyList<IReadOnlyList<PointD>> target = Enumerable.Range(0, 24).Select(i =>
            (IReadOnlyList<PointD>)Rectangle(3 * i + .49, .033, 3 * i + .51, .036)).Reverse().ToArray();
        IReadOnlyList<TaggedToolpath> paths = Enumerable.Range(0, 24).Select(i =>
            Path(new(3 * i, 0, 0), new(3 * i + 1, 0, -.001))).Reverse().ToArray();
        var limits = new Cert.Limits { RecordWitnessPathIndices = true };
        var reference = Cert.Certify(target, paths, Ball(), limits: limits);
        var actual = Cert.Certify(target, paths, Ball(), limits: limits with { UsePreparedSphereDiscPruning = true });
        AssertSame(reference, actual); Assert.True(actual.Certified, actual.ToString());
        Assert.Equal(24, actual.SpherePreparedIntervals); Assert.Equal(24, actual.SpherePreparedSlots);
        Assert.Equal(0, actual.SpherePreparationGuardRefusals);
    }

    [Fact]
    public void CompleteNegativeThenPositiveKeepsTheSelectedOwner()
    {
        var q = new PointD(.5, .05); double half = .0025 / Math.Sqrt(2);
        IReadOnlyList<IReadOnlyList<PointD>> target = [Rectangle(q.x - half, q.y - half, q.x + half, q.y + half)];
        IReadOnlyList<TaggedToolpath> paths = [Path(new(0, 0, 0), new(1, 0, -.001)),
            Path(new(q.x, q.y, -.002), new(q.x, q.y, -.002))];
        var limits = new Cert.Limits { RecordWitnessPathIndices = true };
        var reference = Cert.Certify(target, paths, Ball(), limits: limits);
        var actual = Cert.Certify(target, paths, Ball(), limits: limits with { UsePreparedSphereDiscPruning = true });
        AssertSame(reference, actual); Assert.True(actual.Certified);
        Assert.Equal(48, reference.DiscEvaluations); Assert.Equal(6, actual.DiscEvaluations);
        Assert.Equal(new[] { 1 }, actual.WitnessPathIndices);
        Assert.Equal(1, actual.SpherePreparedSlots); Assert.Equal(1, actual.SpherePreparedIntervals);
    }

    [Fact]
    public void PreparationAndQueryPremisesRefuseUnsupportedAuthority()
    {
        var cone = new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear, 0, 1, 1)], 1);
        Assert.False(VCarvePreparedSphereDiscBounds.TryPrepareProfile(cone, out _));
        Assert.True(VCarvePreparedSphereDiscBounds.TryPrepareProfile(Ball(1), out var profile));
        Assert.False(VCarvePreparedSphereDiscBounds.TryPrepareInterval(new(0, 0, 0), new(1, 0, 1), in profile, out _));
        Assert.False(VCarvePreparedSphereDiscBounds.TryPrepareInterval(new(0, 0, -.001), new(1, 0, .001), in profile, out _));
        Assert.True(VCarvePreparedSphereDiscBounds.TryPrepareInterval(new(0, 0, 0), new(1, 0, .001), in profile, out var interval));
        Assert.False(VCarvePreparedSphereDiscBounds.TryReject(new(double.NaN, 0), new(0, 0, 0), in interval, profile.SphereSquaredUpper, 0, out bool rejected));
        Assert.False(rejected);
    }

    [Fact]
    public void CachedTreeBoundsPreserveEveryFrozenBoxSampleAndOwner()
    {
        IReadOnlyList<IReadOnlyList<PointD>> target = Enumerable.Range(0, 24).Select(i =>
            (IReadOnlyList<PointD>)Rectangle(3 * i - .2, -.2, 3 * i + .2, .2)).Reverse().ToArray();
        IReadOnlyList<TaggedToolpath> paths = Enumerable.Range(0, 24).Select(i =>
            Path(new(3 * i, 0, -.1), new(3 * i, 0, -.1))).Reverse().ToArray();
        var profile = new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear, .5, .5, 1)], 1);
        var limits = new Cert.Limits { RecordWitnessPathIndices = true };
        var reference = Cert.Certify(target, paths, profile, limits: limits);
        var actual = Cert.Certify(target, paths, profile, limits: limits with { UseCachedTreeBounds = true });
        AssertSame(reference, actual); Assert.Equal(reference.DiscEvaluations, actual.DiscEvaluations);
        Assert.Equal(303, actual.VisitedBoxes); Assert.Equal(168, actual.DiscEvaluations);
        Assert.True(actual.CachedTreeBoundReuses > 0); Assert.Equal(0, reference.CachedTreeBoundReuses);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void OriginalPositiveSearchBudgetsAndCancellationRemainAuthoritative(bool enabled)
    {
        Assert.False(new Cert.Limits().UsePreparedSphereDiscPruning); Assert.False(new Cert.Limits().UseCachedTreeBounds);
        IReadOnlyList<IReadOnlyList<PointD>> target = [Rectangle(.49, .033, .51, .036)];
        IReadOnlyList<TaggedToolpath> paths = [Path(new(0, 0, 0), new(1, 0, -.001))];
        var limits = new Cert.Limits { UsePreparedSphereDiscPruning = enabled, UseCachedTreeBounds = enabled, MaximumDiscEvaluations = 3 };
        var result = Cert.Certify(target, paths, Ball(), limits: limits);
        Assert.False(result.Certified); Assert.Equal("disc-budget", result.Reason); Assert.Equal(3, result.DiscEvaluations);
        using var cancellation = new CancellationTokenSource(); cancellation.Cancel();
        Assert.Throws<OperationCanceledException>(() => Cert.Certify(target, paths, Ball(), limits: limits, cancellationToken: cancellation.Token));
    }

    private static void AssertSampled(VCarveCutterProfile profile, Point3D a, Point3D b, PointD q, double desired)
    {
        Assert.True(VCarvePreparedSphereDiscBounds.TryPrepareProfile(profile, out var preparedProfile));
        Assert.True(VCarvePreparedSphereDiscBounds.TryPrepareInterval(a, b, in preparedProfile, out var interval));
        Assert.True(VCarvePreparedSphereDiscBounds.TryReject(q, a, in interval, preparedProfile.SphereSquaredUpper, desired, out bool rejected));
        if (!rejected) return;
        for (int i = 0; i <= 8_192; i++)
        {
            double t = i / 8192d;
            var p = new Point3D(a.X + (b.X - a.X) * t, a.Y + (b.Y - a.Y) * t, a.Z + (b.Z - a.Z) * t);
            double value = p.Z <= 0 ? double.PositiveInfinity
                : Math.Sqrt((p.X - q.x) * (p.X - q.x) + (p.Y - q.y) * (p.Y - q.y)) - profile.PhysicalRadiusAtDepth(p.Z);
            Assert.True(value > desired, $"False rejection: t={t:R} value={value:R} desired={desired:R}");
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
    private static TreeObjects BuildTree(Point3D a, Point3D b, VCarveCutterProfile profile, Cert.Limits limits)
    {
        var segmentType = typeof(Cert).GetNestedType("Segment", BindingFlags.NonPublic)!;
        var input = Array.CreateInstance(segmentType, 1);
        input.SetValue(segmentType.GetConstructors(Flags).Single().Invoke(new object[] { a, b, profile.PhysicalRadiusAtDepth(Math.Max(a.Z, b.Z)), 7 }), 0);
        var workType = typeof(Cert).GetNestedType("Budget", BindingFlags.NonPublic)!;
        var work = workType.GetConstructors(Flags).Single().Invoke(new object[] { limits, CancellationToken.None });
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
    private static VCarveCutterProfile Ball(double radius = 1.5875)
        => new([new(CarvingCutterSectionKind.Hemisphere, 0, radius, radius)], radius);
    private static PointD[] Rectangle(double x0, double y0, double x1, double y1)
        => [new(x0, y0), new(x1, y0), new(x1, y1), new(x0, y1)];
    private static TaggedToolpath Path(params Point3D[] points)
        => new(points, Array.Empty<ToolpathSpan>(), 0, "final-carve", null, detectCurves: false);
}
