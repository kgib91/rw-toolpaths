using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class VCarveCutterProfileTests
{
    [Fact]
    public void TaperedBallRetainsTheSphericalTipAndConeAxialOffset()
    {
        var profile = TaperedBall();

        Assert.True(profile.RoundedTip);
        Assert.Equal(0, profile.TipRadius);
        Assert.Equal(1.58, profile.ClearingDiameter);
        Assert.Equal(20.79, profile.MaximumDepth);
        Assert.Equal(0, profile.RadiusAtDepth(0));
        Assert.Equal(0.79, profile.RadiusAtDepth(0.79), 12);
        Assert.Equal(1.089455, profile.RadiusAtDepth(3.5), 12);
        Assert.Equal(1.40438, profile.RadiusAtDepth(6.35), 12);
        Assert.Equal(6.35, profile.DepthAtRadius(1.40438), 12);
        Assert.Equal(3, profile.RadiusAtDepth(25), 12);
        Assert.Equal(20.79, profile.DepthAtRadius(10), 12);
    }

    [Theory]
    [InlineData(0.00000001)]
    [InlineData(0.2)]
    [InlineData(0.6)]
    public void ShallowBallContactSatisfiesTheSphereEquation(double clearance)
    {
        const double radius = 0.79;
        var profile = TaperedBall();
        double depth = profile.DepthAtRadius(clearance);

        Assert.True(depth > 0);
        Assert.InRange(depth, 0, radius);
        Assert.Equal(radius * radius, clearance * clearance + (radius - depth) * (radius - depth), 14);
        Assert.InRange(Math.Abs(profile.RadiusAtDepth(depth) - clearance), 0, 1e-14);
    }

    [Fact]
    public void StraightBallCanReachItsFullCuttingDepthWhenItsDiameterFits()
    {
        var profile = new VCarveCutterProfile([
            new(CarvingCutterSectionKind.Hemisphere, 0, 0.8, 0.8),
            new(CarvingCutterSectionKind.Linear, 0.8, 0.8, 10)
        ], 8);

        Assert.Equal(8, profile.DepthAtRadius(0.8));
        Assert.Equal(0.8, profile.RadiusAtDepth(8));
        Assert.True(profile.DepthAtRadius(0.799) < 0.8);
    }

    [Fact]
    public void InverseContinuesAcrossAnIntermediateCylindricalPlateau()
    {
        var profile = new VCarveCutterProfile([
            new(CarvingCutterSectionKind.Linear, 0, 1, 2),
            new(CarvingCutterSectionKind.Linear, 1, 1, 3),
            new(CarvingCutterSectionKind.Linear, 1, 2, 4)
        ], 9);

        Assert.Equal(5, profile.DepthAtRadius(1));
        Assert.Equal(7, profile.DepthAtRadius(1.5));
    }

    [Fact]
    public void FlattenedConePreservesLegacyDepthAndRadiusRelationship()
    {
        var profile = new VCarveCutterProfile([
            new(CarvingCutterSectionKind.Linear, 0.05, 2.05, 8)
        ], 8);

        Assert.False(profile.RoundedTip);
        Assert.Equal(0.05, profile.TipRadius);
        Assert.Equal(0, profile.DepthAtRadius(0.04));
        Assert.Equal(0, profile.DepthAtRadius(0.05));
        Assert.Equal(1.05, profile.RadiusAtDepth(4), 12);
        Assert.Equal(4, profile.DepthAtRadius(1.05), 12);
    }

    [Fact]
    public void PartialSphericalContactDepthCapsBothDirections()
    {
        var profile = new VCarveCutterProfile([
            new(CarvingCutterSectionKind.Hemisphere, 0, 1, 1)
        ], 0.2);

        Assert.Equal(0.6, profile.RadiusAtDepth(0.2), 12);
        Assert.Equal(0.6, profile.RadiusAtDepth(1), 12);
        Assert.Equal(0.2, profile.DepthAtRadius(0.8));
    }

    [Fact]
    public void ProfileOwnsItsSectionSnapshot()
    {
        CarvingCutterSection[] source = [new(CarvingCutterSectionKind.Linear, 0, 1, 2)];
        var profile = new VCarveCutterProfile(source, 2);
        source[0] = new(CarvingCutterSectionKind.Linear, 0, 5, 2);

        Assert.Equal(1, profile.RadiusAtDepth(2));
        Assert.Equal(1, profile.Sections[0].TopRadius);
    }

    [Fact]
    public void RejectsUnsupportedActiveDiscontinuitiesAndDepthBeyondTheProfile()
    {
        Assert.Throws<ArgumentException>(() => new VCarveCutterProfile([
            new(CarvingCutterSectionKind.Hemisphere, 0, 0.8, 0.8),
            new(CarvingCutterSectionKind.Linear, 0.9, 3, 20)
        ], 6));
        Assert.Throws<ArgumentException>(() => new VCarveCutterProfile([
            new(CarvingCutterSectionKind.Linear, 2, 1, 5)
        ], 4));
        Assert.Throws<ArgumentException>(() => new VCarveCutterProfile([
            new(CarvingCutterSectionKind.Linear, 0, 1, 2)
        ], 3));
    }

    [Theory]
    [InlineData(20, false)]
    [InlineData(19.999, true)]
    public void SectionStartingAtTheContactPlaneCannotEscapeValidation(double contactDepth, bool accepted)
    {
        CarvingCutterSection[] sections = [new(CarvingCutterSectionKind.Linear, .75, .75, 20),
            new(CarvingCutterSectionKind.Linear, .793, 3, 10)];
        if (accepted)
        {
            var profile = new VCarveCutterProfile(sections, contactDepth);
            Assert.Equal(.75, profile.RadiusAtDepth(contactDepth));
            Assert.Equal(2, profile.Sections.Count);
            Assert.Equal(.793, profile.PhysicalRadiusAtDepth(20));
        }
        else Assert.Throws<ArgumentException>(() => new VCarveCutterProfile(sections, contactDepth));
    }

    private static VCarveCutterProfile TaperedBall() => new([
        new(CarvingCutterSectionKind.Hemisphere, 0, 0.79, 0.79),
        new(CarvingCutterSectionKind.Linear, 0.79, 3, 20),
        new(CarvingCutterSectionKind.Linear, 3, 3, 70)
    ], 20.79);
}
