using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class VCarveWitnessPruningTests
{
    [Fact]
    public void MandatoryDiscOwnsBoxWhenOptionalTerminalAlsoCoversIt()
    {
        var terminal = Path(new(0, 0, -.1), new(0, 0, -.1));
        var mandatory = Path(new(0, 0, -.1), new(0, 0, -.1));
        var paths = new[] { terminal, mandatory };
        IReadOnlyList<PointD>[] target = [Rectangle(-.2, -.2, .2, .2)];
        var optional = new HashSet<int> { 0 };
        var certificate = Record(target, paths, Cylinder(1), optional);
        Assert.True(certificate.Certified, certificate.ToString());
        Assert.Equal(new[] { 1 }, certificate.WitnessPathIndices);
        var retained = VCarveSmoothing.PruneTerminalPaths(paths, optional, certificate);
        Assert.Equal(mandatory, Assert.Single(retained));
        Assert.True(VCarveCoverageCertificate.Certify(target, retained, Cylinder(1)).Certified);
    }

    [Fact]
    public void OptionalTerminalStillOwnsRequiredBoxWhenMandatoryDiscsCannotReachIt()
    {
        var terminal = Path(new(0, 0, -.1), new(0, 0, -.1));
        var mandatory = Path(new(20, 0, -.1), new(20, 0, -.1));
        var paths = new[] { terminal, mandatory };
        IReadOnlyList<PointD>[] target = [Rectangle(-.2, -.2, .2, .2)];
        var optional = new HashSet<int> { 0 };
        var certificate = Record(target, paths, Cylinder(1), optional);
        Assert.True(certificate.Certified, certificate.ToString());
        Assert.Equal(new[] { 0 }, certificate.WitnessPathIndices);
        Assert.Equal(paths, VCarveSmoothing.PruneTerminalPaths(paths, optional, certificate));
    }

    [Fact]
    public void UnsupportedTerminalCopiesAreRemovedAndMandatoryPathsStayUnchanged()
    {
        var mandatory = Path(new(20, 0, -.1), new(20, 0, -.1));
        var required = Path(new(0, 0, -.1), new(0, 0, -.1));
        var paths = new[] { mandatory }.Concat(Enumerable.Repeat(required, 100)).ToArray();
        IReadOnlyList<PointD>[] target = [Rectangle(-.5, -.5, .5, .5)];
        var certificate = Record(target, paths, Cylinder(1));
        Assert.True(certificate.Certified, certificate.ToString());
        Assert.True(certificate.WitnessPathsRecorded);
        Assert.Single(certificate.WitnessPathIndices);
        var retained = VCarveSmoothing.PruneTerminalPaths(paths,
            Enumerable.Range(1, 100).ToHashSet(), certificate);
        Assert.Equal(2, retained.Count);
        Assert.Equal(mandatory, retained[0]);
        Assert.Equal(required, retained[1]);
        Assert.True(VCarveCoverageCertificate.Certify(target, retained, Cylinder(1)).Certified);
    }

    [Fact]
    public void TwoDisconnectedNecessaryTerminalsAndEveryNonterminalAreRetained()
    {
        var paths = new[]
        {
            Path(new(-20, 0, -.1), new(-20, 0, -.1)), // Mandatory source.
            Path(new(0, 0, -.1), new(0, 0, -.1)),
            Path(new(30, 0, -.1), new(30, 0, -.1)),  // Unsupported terminal.
            Path(new(10, 0, -.1), new(10, 0, -.1)),
            Path(new(40, 0, -.1), new(40, 0, -.1)),  // Mandatory concentric path.
        };
        IReadOnlyList<PointD>[] target = [Rectangle(-.2, -.2, .2, .2), Rectangle(9.8, -.2, 10.2, .2)];
        var certificate = Record(target, paths, Cylinder(.5));
        Assert.True(certificate.Certified, certificate.ToString());
        Assert.Equal(new[] { 1, 3 }, certificate.WitnessPathIndices);
        var retained = VCarveSmoothing.PruneTerminalPaths(paths, new HashSet<int> { 1, 2, 3 }, certificate);
        Assert.Equal(new[] { paths[0], paths[1], paths[3], paths[4] }, retained);
        Assert.True(VCarveCoverageCertificate.Certify(target, retained, Cylinder(.5)).Certified);
    }

    [Fact]
    public void MultiChordOwnerSurvivesSectionExpansionAndBvhPermutation()
    {
        var profile = new VCarveCutterProfile([
            new(CarvingCutterSectionKind.Hemisphere, 0, 1, 1),
            new(CarvingCutterSectionKind.Linear, 1, 2, 3)], 4);
        var paths = Enumerable.Range(0, 24).Select(i =>
            Path(new(100 + 3 * i, 5, -.1), new(100 + 3 * i, 5, -.1))).ToList();
        var required = Path(new(0, 0, 0), new(0, 0, -2), new(3, 0, -2));
        paths.Insert(17, required);
        IReadOnlyList<PointD>[] target = [Rectangle(.1, .1, .2, .2)];
        var certificate = Record(target, paths, profile);
        Assert.True(certificate.Certified, certificate.ToString());
        Assert.Equal(paths.Count, certificate.CommandPathCount);
        Assert.True(certificate.CommandIntervals > paths.Count);
        Assert.Equal(new[] { 17 }, certificate.WitnessPathIndices);
        var retained = VCarveSmoothing.PruneTerminalPaths(paths,
            Enumerable.Range(0, paths.Count).ToHashSet(), certificate);
        Assert.Equal(required, Assert.Single(retained));
        Assert.Equal(3, retained[0].Points.Count);
        Assert.True(VCarveCoverageCertificate.Certify(target, retained, profile).Certified);
    }

    [Fact]
    public void ContinuousRampOwnerIsKeptWhenNeitherEndpointDiscCoversTarget()
    {
        var profile = new VCarveCutterProfile([
            new(CarvingCutterSectionKind.Hemisphere, 0, 1.5875, 1.5875),
            new(CarvingCutterSectionKind.Linear, 1.5875, 3, 20)], 21.5875);
        var required = Path(new(0, 0, 0), new(1, 0, -.001));
        var paths = new[] { Path(new(20, 0, -.001), new(21, 0, -.001)), required };
        IReadOnlyList<PointD>[] target = [Rectangle(.49, .033, .51, .036)];
        var certificate = Record(target, paths, profile);
        Assert.True(certificate.Certified, certificate.ToString());
        Assert.Equal(new[] { 1 }, certificate.WitnessPathIndices);
        var retained = VCarveSmoothing.PruneTerminalPaths(paths, new HashSet<int> { 0, 1 }, certificate);
        Assert.Equal(required, Assert.Single(retained));
        Assert.True(VCarveCoverageCertificate.Certify(target, retained, profile).Certified);
    }

    [Fact]
    public void HoleCoverageRemainsCertifiedAndFillingTheHoleStillRejects()
    {
        PointD[] outer = Rectangle(-1, -1, 1, 1), hole = Rectangle(-.5, -.5, .5, .5).Reverse().ToArray();
        var frame = Path(new(-.75, -.75, -.1), new(.75, -.75, -.1), new(.75, .75, -.1),
            new(-.75, .75, -.1), new(-.75, -.75, -.1));
        var paths = new[] { Path(new(10, 10, -.1), new(10, 10, -.1)), frame };
        IReadOnlyList<PointD>[] target = [outer, hole];
        var certificate = Record(target, paths, Cylinder(.4));
        Assert.True(certificate.Certified, certificate.ToString());
        var retained = VCarveSmoothing.PruneTerminalPaths(paths, new HashSet<int> { 0, 1 }, certificate);
        Assert.Equal(frame, Assert.Single(retained));
        Assert.True(VCarveCoverageCertificate.Certify(target, retained, Cylinder(.4)).Certified);
        Assert.False(VCarveCoverageCertificate.Certify([outer], retained, Cylinder(.4)).Certified);
    }

    [Fact]
    public void FailedExhaustedAndUnrecordedCertificatesHaveNoRemovalAuthority()
    {
        var paths = new[] { Path(new(0, 0, -.1), new(0, 0, -.1)), Path(new(10, 0, -.1), new(10, 0, -.1)) };
        IReadOnlyList<PointD>[] target = [Rectangle(-.2, -.2, .2, .2)];
        var exhausted = VCarveCoverageCertificate.Certify(target, paths, Cylinder(.5),
            limits: new() { MaximumBoxes = 0, RecordWitnessPathIndices = true });
        var missed = Record([Rectangle(30, -.2, 30.4, .2)], paths, Cylinder(.5));
        var unrecorded = VCarveCoverageCertificate.Certify(target, paths, Cylinder(.5));
        Assert.True(unrecorded.Certified);
        foreach (var certificate in new[] { exhausted, missed, unrecorded })
        {
            Assert.False(certificate.WitnessPathsRecorded);
            Assert.Empty(certificate.WitnessPathIndices);
            Assert.Equal(paths, VCarveSmoothing.PruneTerminalPaths(paths, new HashSet<int> { 0, 1 }, certificate));
        }
    }

    [Fact]
    public void UnengagedAndRetractChordsCannotOwnACoverageBox()
    {
        var air = Path(new(-1, 0, 0), new(1, 0, 0));
        var retract = new TaggedToolpath(new[] { new Point3D(-1, 0, -.1), new Point3D(1, 0, -.1) },
            new[] { new ToolpathSpan(0, 1, ToolpathSpanKind.Retract) }, 0, "clearing", null, detectCurves: false);
        var required = Path(new(0, 0, -.1), new(0, 0, -.1));
        var paths = new[] { air, retract, required };
        IReadOnlyList<PointD>[] target = [Rectangle(-.2, -.2, .2, .2)];
        var certificate = Record(target, paths, Cylinder(1));
        Assert.True(certificate.Certified, certificate.ToString());
        Assert.Equal(new[] { 2 }, certificate.WitnessPathIndices);
        Assert.Equal(required, Assert.Single(VCarveSmoothing.PruneTerminalPaths(paths,
            new HashSet<int> { 0, 1, 2 }, certificate)));
    }

    [Fact]
    public void CertificateCannotPruneAChangedOrdinalSnapshot()
    {
        var paths = new[] { Path(new(0, 0, -.1), new(0, 0, -.1)), Path(new(10, 0, -.1), new(10, 0, -.1)) };
        var certificate = Record([Rectangle(-.2, -.2, .2, .2)], paths, Cylinder(.5));
        Assert.True(certificate.Certified);
        Assert.Throws<InvalidOperationException>(() => VCarveSmoothing.PruneTerminalPaths(
            paths.Take(1).ToArray(), new HashSet<int> { 0 }, certificate));
    }

    [Fact]
    public void F3DuplicateAndCollinearCollapsePreservesNominalPathOwner()
    {
        var required = Path(new(.0004, 0, -.1004), new(.0001, 0, -.1001),
            new(.5004, 0, -.1001), new(1.0004, 0, -.1001), new(1.0001, 0, -.1004));
        var paths = new[] { Path(new(20, 0, -.1), new(21, 0, -.1)), required };
        var commands = VCarveSmoothing.CommandPaths(paths, 12.7);
        Assert.Equal(paths.Length, commands.Count);
        Assert.True(commands[1].Points.Count < required.Points.Count);
        IReadOnlyList<PointD>[] target = [Rectangle(.49, -.1, .51, .1)];
        var certificate = Record(target, commands, Cylinder(.2));
        Assert.True(certificate.Certified, certificate.ToString());
        Assert.Equal(new[] { 1 }, certificate.WitnessPathIndices);
        var retained = VCarveSmoothing.PruneTerminalPaths(paths, new HashSet<int> { 0, 1 }, certificate);
        Assert.Equal(required, Assert.Single(retained));
        Assert.True(VCarveCoverageCertificate.Certify(target,
            VCarveSmoothing.CommandPaths(retained, 12.7), Cylinder(.2)).Certified);
    }

    private static VCarveCoverageCertificate.Result Record(IReadOnlyList<IReadOnlyList<PointD>> target,
        IReadOnlyList<TaggedToolpath> paths, VCarveCutterProfile profile, IReadOnlySet<int>? optional = null)
        => VCarveCoverageCertificate.Certify(target, paths, profile,
            limits: new() { RecordWitnessPathIndices = true, OptionalWitnessPathIndices = optional });
    private static TaggedToolpath Path(params Point3D[] points)
        => new(points, Array.Empty<ToolpathSpan>(), 0, "clearing", null, detectCurves: false);
    private static VCarveCutterProfile Cylinder(double radius)
        => new([new(CarvingCutterSectionKind.Linear, radius, radius, 1)], 1);
    private static PointD[] Rectangle(double x0, double y0, double x1, double y1)
        => [new(x0, y0), new(x1, y0), new(x1, y1), new(x0, y1)];
}
