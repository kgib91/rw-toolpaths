using System.Reflection;
using Clipper2Lib;
using RW.Toolpaths.Milling;
using Cert = RW.Toolpaths.Milling.VCarveCoverageCertificate;

namespace RW.Toolpaths.Tests;

public sealed class VCarveAnalyticPoseNominationTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ExistingContactMinimumReturnsItsCorrelatedPoseWithoutChangingReturnMath(bool reverse)
    {
        var cutter = Cone(4).PhysicalProfile;
        var q = new PointD(.9, .6);
        var a = new PointD(0, 0); var b = new PointD(2, 0);
        double az = -.2, bz = -1.2;
        if (reverse) { (a, b) = (b, a); (az, bz) = (bz, az); }
        double reference = CarvingCutterContactBounds.MinimumTipAndBottom(cutter, q, a, az, b, bz, 0);
        double actual = CarvingCutterContactBounds.MinimumTipAndBottom(cutter, q, a, az, b, bz, 0, out double pose);
        Assert.Equal(BitConverter.DoubleToInt64Bits(reference), BitConverter.DoubleToInt64Bits(actual));
        double expected = .45 + Math.Sqrt(.36 / 12);
        Assert.Equal(reverse ? 1 - expected : expected, pose, 12);
        double x = a.x + pose * (b.x - a.x), y = a.y + pose * (b.y - a.y);
        double physical = az + pose * (bz - az) + cutter.Bottom(Math.Sqrt((x - q.x) * (x - q.x) + (y - q.y) * (y - q.y)));
        Assert.Equal(actual, physical, 12);
    }

    [Fact]
    public void ReturnMathAndFirstTiedPoseSurviveSphereReserveNoContactAndOverflow()
    {
        var profiles = new[] { Cone(4).PhysicalProfile,
            new CarvingCutterProfile([new(CarvingCutterSectionKind.Hemisphere, 0, 1, 1)], 1),
            new CarvingCutterProfile([new(CarvingCutterSectionKind.Linear, 1, 1, 4)], 2) };
        foreach (var profile in profiles)
        foreach (double reserve in new[] { 0d, .2 })
        foreach (var q in new[] { new PointD(.8, .5), new PointD(100, 100) })
        {
            double expected = CarvingCutterContactBounds.MinimumTipAndBottom(profile, q, new(0, 0), 1, new(1, 0), 2, reserve);
            double actual = CarvingCutterContactBounds.MinimumTipAndBottom(profile, q, new(0, 0), 1, new(1, 0), 2, reserve, out double pose);
            Assert.Equal(BitConverter.DoubleToInt64Bits(expected), BitConverter.DoubleToInt64Bits(actual));
            if (double.IsFinite(actual)) Assert.InRange(pose, 0, 1);
            else Assert.True(double.IsNaN(pose));
        }
        double tied = CarvingCutterContactBounds.MinimumTipAndBottom(profiles[2], new(0, 0),
            new(0, 0), 1, new(.5, 0), 1, 0, out double first);
        Assert.Equal(1, tied); Assert.Equal(0, first);
        double oldOverflow = CarvingCutterContactBounds.MinimumTipAndBottom(profiles[0], new(0, 0),
            new(-double.MaxValue, 0), 1, new(double.MaxValue, 0), 2, 0);
        double overflow = CarvingCutterContactBounds.MinimumTipAndBottom(profiles[0], new(0, 0),
            new(-double.MaxValue, 0), 1, new(double.MaxValue, 0), 2, 0, out double invalid);
        Assert.Equal(BitConverter.DoubleToInt64Bits(oldOverflow), BitConverter.DoubleToInt64Bits(overflow));
        Assert.True(double.IsNaN(overflow)); Assert.True(double.IsNaN(invalid));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CompleteConeCertificateKeepsMandatoryOwnerAndBoxDecisionWithFewerSamples(bool reverse)
    {
        var q = new PointD(.9, .6);
        double desired = OptimalConeValue(.2, 1.2, q) + 1e-9;
        double half = (.004 - desired - Cert.ArithmeticReserve) / Math.Sqrt(2);
        IReadOnlyList<IReadOnlyList<PointD>> target = [Rectangle(q.x - half, q.y - half, q.x + half, q.y + half)];
        Point3D a = new(0, 0, -.2), b = new(2, 0, -1.2);
        var path = reverse ? Path(b, a) : Path(a, b);
        var limits = new Cert.Limits { RecordWitnessPathIndices = true, OptionalWitnessPathIndices = new HashSet<int> { 0 } };
        var reference = Cert.Certify(target, [path, path], Cone(4), limits: limits);
        var messages = new List<string>();
        var actual = Cert.Certify(target, [path, path], Cone(4),
            limits: limits with { UseAnalyticPoseNomination = true }, diagnostic: messages.Add);
        AssertEquivalent(reference, actual);
        Assert.True(actual.Certified, actual.ToString()); Assert.Equal(1, actual.VisitedBoxes);
        Assert.Equal(new[] { 1 }, actual.WitnessPathIndices);
        Assert.Equal(4, actual.DiscEvaluations);
        Assert.True(reference.DiscEvaluations > 10, reference.ToString());
        Assert.Equal(0, reference.AnalyticPoseCalls);
        Assert.Equal(1, actual.AnalyticPoseCalls); Assert.Equal(1, actual.AnalyticPoseHits);
        Assert.Equal(1, actual.AnalyticPoseEvaluations); Assert.Equal(0, actual.AnalyticPoseFallbacks);
        string completion = Assert.Single(messages.Where(message => message.Contains("event=complete")));
        Assert.Contains("analyticPoseCalls=1 ", completion); Assert.Contains("analyticPoseHits=1 ", completion);
        Assert.Contains("analyticPoseFallbacks=0 ", completion); Assert.Contains("analyticPoseEvaluations=1 ", completion);
    }

    [Fact]
    public void FailedNominationRetainsAllOriginalFortySearchSteps()
    {
        var q = new PointD(.9, .6); var profile = Cone(4);
        double desired = OptimalConeValue(.2, 1.2, q) - 1e-6;
        var reference = BuildTree(new(0, 0, .2), new(2, 0, 1.2), profile, false);
        var actual = BuildTree(new(0, 0, .2), new(2, 0, 1.2), profile, true);
        var expected = Query(reference.Tree, q, desired, profile);
        Assert.Equal(expected, Query(actual.Tree, q, desired, profile)); Assert.False(expected.Accepted);
        Assert.Equal(45, DiscEvaluations(reference.Work)); Assert.Equal(46, DiscEvaluations(actual.Work));
        Assert.Equal(1, Counter(actual.Work, "AnalyticPoseCalls"));
        Assert.Equal(0, Counter(actual.Work, "AnalyticPoseHits"));
        Assert.Equal(1, Counter(actual.Work, "AnalyticPoseFallbacks"));
        Assert.Equal(1, Counter(actual.Work, "AnalyticPoseEvaluations"));
    }

    [Fact]
    public void EndpointDuplicateNominationCostsNoExtraDiscAndDoesNotRejectTheOldSearch()
    {
        var profile = Cone(4); var q = new PointD(0, 0);
        var reference = BuildTree(new(0, 0, .2), new(2, 0, 1.2), profile, false);
        var actual = BuildTree(new(0, 0, .2), new(2, 0, 1.2), profile, true);
        Assert.Equal(Query(reference.Tree, q, -.3, profile), Query(actual.Tree, q, -.3, profile));
        Assert.Equal(45, DiscEvaluations(reference.Work)); Assert.Equal(45, DiscEvaluations(actual.Work));
        Assert.Equal(0, Counter(actual.Work, "AnalyticPoseEvaluations"));
        Assert.Equal(1, Counter(actual.Work, "AnalyticPoseFallbacks"));
        Assert.Equal(1, Counter(actual.Work, "AnalyticPoseDuplicates"));
    }

    [Fact]
    public void NominalContactCapCannotRejectARealUncappedPhysicalWitness()
    {
        var profile = Cone(.5); var q = new PointD(.9, .6);
        double desired = OptimalConeValue(1.2, 1.8, q) + 1e-9;
        var reference = BuildTree(new(0, 0, 1.2), new(2, 0, 1.8), profile, false);
        var actual = BuildTree(new(0, 0, 1.2), new(2, 0, 1.8), profile, true);
        var expected = Query(reference.Tree, q, desired, profile); var result = Query(actual.Tree, q, desired, profile);
        Assert.True(expected.Accepted); Assert.Equal(expected, result);
        Assert.Equal(DiscEvaluations(reference.Work), DiscEvaluations(actual.Work));
        Assert.Equal(1, Counter(actual.Work, "AnalyticPoseFallbacks"));
        Assert.Equal(0, Counter(actual.Work, "AnalyticPoseEvaluations"));
        Assert.Equal(1, Counter(actual.Work, "AnalyticPoseInvalid"));
    }

    [Fact]
    public void DegenerateChordAndOverflowCannotTurnInvalidContactIntoAuthority()
    {
        var profile = Cone(4);
        foreach (var ends in new[] {
            (A: new Point3D(0, 0, .2), B: new Point3D(0, 0, 1.2), Q: new PointD(.6, 0), Desired: -.6000000005),
            (A: new Point3D(-double.MaxValue, 0, .2), B: new Point3D(double.MaxValue, 0, 1.2), Q: new PointD(0, 0), Desired: -.3),
            (A: new Point3D(0, 0, double.PositiveInfinity), B: new Point3D(2, 0, 1.2), Q: new PointD(0, 0), Desired: -.3) })
        {
            var reference = BuildTree(ends.A, ends.B, profile, false);
            var actual = BuildTree(ends.A, ends.B, profile, true);
            var expected = Query(reference.Tree, ends.Q, ends.Desired, profile);
            Assert.False(expected.Accepted); Assert.Equal(expected, Query(actual.Tree, ends.Q, ends.Desired, profile));
            Assert.Equal(DiscEvaluations(reference.Work), DiscEvaluations(actual.Work));
            Assert.Equal(0, Counter(actual.Work, "AnalyticPoseEvaluations"));
            Assert.Equal(1, Counter(actual.Work, "AnalyticPoseFallbacks"));
            Assert.Equal(1, Counter(actual.Work, ends.A.X == ends.B.X ? "AnalyticPoseDuplicates" : "AnalyticPoseInvalid"));
        }
    }

    [Fact]
    public void SlopedConeQueriesRetainPhysicalMissesAndWitnessOwners()
    {
        var profile = Cone(4);
        foreach (double x in new[] { -.5, .3, .9, 1.7, 2.5 })
        foreach (double y in new[] { 0d, .3, .6, 1.2 })
        foreach (double desired in new[] { -.3, 0, .3 })
        {
            var q = new PointD(x, y);
            var reference = BuildTree(new(0, 0, .2), new(2, 0, 1.2), profile, false);
            var actual = BuildTree(new(0, 0, .2), new(2, 0, 1.2), profile, true);
            var expected = Query(reference.Tree, q, desired, profile);
            Assert.Equal(expected, Query(actual.Tree, q, desired, profile));
            if (expected.Accepted) Assert.Equal(7, expected.Path);
            else Assert.Equal(-1, expected.Path);
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void DiscQuotaDeadlineAndCancellationRemainAuthoritative(bool enabled)
    {
        var q = new PointD(.9, .6);
        double half = (.004 - OptimalConeValue(.2, 1.2, q) - Cert.ArithmeticReserve) / Math.Sqrt(2);
        IReadOnlyList<IReadOnlyList<PointD>> target = [Rectangle(q.x - half, q.y - half, q.x + half, q.y + half)];
        IReadOnlyList<TaggedToolpath> paths = [Path(new(0, 0, -.2), new(2, 0, -1.2))];
        var limits = new Cert.Limits { UseAnalyticPoseNomination = enabled, MaximumDiscEvaluations = 3 };
        var result = Cert.Certify(target, paths, Cone(4), limits: limits);
        Assert.False(result.Certified); Assert.Equal("disc-budget", result.Reason); Assert.Equal(3, result.DiscEvaluations);
        var timed = Cert.Certify(target, paths, Cone(4), limits: limits with { MaximumElapsed = TimeSpan.FromTicks(1) });
        Assert.False(timed.Certified); Assert.Equal("time-budget", timed.Reason);
        using var cancellation = new CancellationTokenSource(); cancellation.Cancel();
        Assert.Throws<OperationCanceledException>(() => Cert.Certify(target, paths, Cone(4),
            cancellationToken: cancellation.Token, limits: limits));
    }

    [Fact]
    public void DefaultModeAndUnengagedCommandsRetainReferenceWork()
    {
        Assert.False(new Cert.Limits().UseAnalyticPoseNomination);
        var profile = new VCarveCutterProfile([new(CarvingCutterSectionKind.Hemisphere, 0, 1.5875, 1.5875),
            new(CarvingCutterSectionKind.Linear, 1.5875, 3, 20)], 21.5875);
        var reference = Cert.Certify([Rectangle(.49, .033, .51, .036)], [Path(new(0, 0, 0), new(1, 0, -.001))], profile);
        Assert.True(reference.Certified); Assert.Equal(3, reference.VisitedBoxes); Assert.Equal(51, reference.DiscEvaluations);
        Assert.Equal(0, reference.AnalyticPoseCalls);
        var unengaged = Cert.Certify([Rectangle(-.2, -.2, .2, .2)],
            [Path(new(-1, 0, 0), new(1, 0, 0))], Cone(4),
            limits: new() { UseAnalyticPoseNomination = true, MaximumBoxes = 1 });
        Assert.False(unengaged.Certified); Assert.Equal(0, unengaged.CommandIntervals);
        Assert.Equal(0, unengaged.DiscEvaluations); Assert.Equal(0, unengaged.AnalyticPoseCalls);
    }

    private static void AssertEquivalent(Cert.Result reference, Cert.Result actual)
    {
        Assert.Equal(reference.Certified, actual.Certified); Assert.Equal(reference.Reason, actual.Reason);
        Assert.Equal(reference.VisitedBoxes, actual.VisitedBoxes); Assert.Equal(reference.CommandIntervals, actual.CommandIntervals);
        Assert.Equal(reference.RemainingBoxes, actual.RemainingBoxes); Assert.Equal(reference.UnresolvedPoint, actual.UnresolvedPoint);
        Assert.Equal(reference.WitnessPathsRecorded, actual.WitnessPathsRecorded); Assert.Equal(reference.WitnessPathIndices, actual.WitnessPathIndices);
    }
    private static double OptimalConeValue(double a, double b, PointD q)
    {
        double dz = b - a;
        double t = q.x / 2 + dz * Math.Abs(q.y) / Math.Sqrt(4 * (4 - dz * dz));
        return Math.Sqrt(Math.Pow(2 * t - q.x, 2) + q.y * q.y) - (a + t * dz);
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
            new Cert.Limits { UseAnalyticPoseNomination = enabled }, CancellationToken.None });
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
    private static long DiscEvaluations(object work) => (long)work.GetType().GetProperty("DiscEvaluations", Flags)!.GetValue(work)!;
    private static VCarveCutterProfile Cone(double depth)
        => new([new(CarvingCutterSectionKind.Linear, 0, 4, 4)], depth);
    private static PointD[] Rectangle(double x0, double y0, double x1, double y1)
        => [new(x0, y0), new(x1, y0), new(x1, y1), new(x0, y1)];
    private static TaggedToolpath Path(params Point3D[] points)
        => new(points, Array.Empty<ToolpathSpan>(), 0, "final-carve", null, detectCurves: false);
}
