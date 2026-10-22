using Clipper2Lib;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;
using Xunit;

namespace RW.Toolpaths.Tests;

public sealed class MaterialLayerAsymmetricOpeningControls
{
    [Fact]
    public void TwoMicrometrePhysicalPoseUsesOneMicrometreLayerRadiusAndOriginalMask()
    {
        var profile = new VCarveCutterProfile(new[] {
            new CarvingCutterSection(CarvingCutterSectionKind.Hemisphere,0,1.5875,1.5875),
            new CarvingCutterSection(CarvingCutterSectionKind.Linear,1.5875,3,20) },3);
        double erosion = profile.PhysicalRadiusAtDepth(.002), dilation = profile.PhysicalRadiusAtDepth(.001);
        Assert.True(erosion > dilation);
        var outer = new List<PointD>(); const int steps=2048;
        for(int i=0;i<steps;i++) outer.Add(new(i/(double)steps*10,0));
        for(int i=0;i<steps;i++) outer.Add(new(10,i/(double)steps*10));
        for(int i=0;i<steps;i++) outer.Add(new(10-i/(double)steps*10,10));
        for(int i=0;i<steps;i++) outer.Add(new(0,10-i/(double)steps*10));
        var source = new List<List<PointD>> { outer,
            new() {new(4,4),new(4,6),new(6,6),new(6,4)} };
        var original = VCarveSmoothing.ReachableReferenceUpper(source,erosion);
        var equalRadius = VCarveSmoothing.ReachableReferenceUpperForPlane(source,erosion,erosion);
        Exact(original,equalRadius);
        var asymmetric = VCarveSmoothing.ReachableReferenceUpperForPlane(source,erosion,dilation);
        var bounds = Clipper.GetBounds(PathUtils.ToClipper(asymmetric));
        long expectedInset = (long)Math.Floor((erosion-dilation)*PathUtils.Scale-18);
        Assert.True(expectedInset>0); Assert.True(bounds.left>=expectedInset&&bounds.top>=expectedInset);
        var conditioned = VCarveConditionedReferenceOpeningGoal.BuildMaskedGoalForPlane(source,source,
            erosion,dilation,CancellationToken.None,null,out bool applied);
        Assert.True(applied); Assert.NotEmpty(conditioned);
        Assert.Empty(Clipper.Difference(PathUtils.ToClipper(conditioned),PathUtils.ToClipper(source),FillRule.NonZero));
        var hole = new Paths64 {new() {new(147456,147456),new(180224,147456),new(180224,180224),new(147456,180224)}};
        Assert.Empty(Clipper.Intersect(PathUtils.ToClipper(conditioned),hole,FillRule.NonZero));
        Assert.Equal(8192,outer.Count);
        // Bounds here include original eight-tick erosion + dilation reserves.
        // This is goal construction, not an all-depth reach or stock theorem.
    }
    private static void Exact(IReadOnlyList<List<PointD>> expected,IReadOnlyList<List<PointD>> actual)
    {
        Assert.Equal(expected.Count,actual.Count);
        for(int r=0;r<expected.Count;r++)
        {
            Assert.Equal(expected[r].Count,actual[r].Count);
            for(int p=0;p<expected[r].Count;p++)
            {
                Assert.Equal(BitConverter.DoubleToInt64Bits(expected[r][p].x),BitConverter.DoubleToInt64Bits(actual[r][p].x));
                Assert.Equal(BitConverter.DoubleToInt64Bits(expected[r][p].y),BitConverter.DoubleToInt64Bits(actual[r][p].y));
            }
        }
    }
}
