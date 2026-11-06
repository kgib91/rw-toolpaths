using Clipper2Lib;
using RW.Toolpaths.Geometry;

namespace RW.Toolpaths.Tests;

public sealed class PocketGuideConservativeExpansionTests
{
    private static IReadOnlyList<PointD> Box(double x0,double y0,double x1,double y1)
        =>[new(x0,y0),new(x1,y0),new(x1,y1),new(x0,y1)];
    private static string[] Words(IEnumerable<IReadOnlyList<PointD>> paths)=>paths.Select(p=>
        string.Join(";",p.Select(v=>$"{v.x:R},{v.y:R}"))).ToArray();

    public static IEnumerable<object[]> Domains()
    {
        yield return ["convex",new[]{Box(0,0,10,8)},.7];
        yield return ["concave",new IReadOnlyList<PointD>[]{[new(0,0),new(10,0),new(10,2),new(3,2),new(3,9),new(0,9)]},1.3];
        yield return ["signed-holes",new[]{Box(-10,-9,12,10),Box(-4,-3,2,4).Reverse().ToArray()},1.1];
        yield return ["island-in-hole",new[]{Box(-12,-11,12,11),Box(-8,-7,8,7).Reverse().ToArray(),Box(-2,-2,2,2)},1.4];
        yield return ["shallow-gap",new[]{Box(-8,-4,-.01,5),Box(.01,-4,8,5)},.4];
        yield return ["separate-negative",new[]{Box(-30,-25,-20,-14),Box(-14,-25,-5,-15)},.75];
        var gear=Enumerable.Range(0,600).Select(i=>{double a=2*Math.PI*i/600,r=8+(i%2==0?.015:0);return new PointD(-20+r*Math.Cos(a),-15+r*Math.Sin(a));}).ToArray();
        yield return ["dense-concavity",new IReadOnlyList<PointD>[]{gear},1.7];
    }

    [Theory]
    [MemberData(nameof(Domains))]
    public void ScopedConstructionRetainsEverySignedTargetAndOriginalReferenceControl(string name,
        IReadOnlyList<IReadOnlyList<PointD>> source,double radius)
    {
        _=name;var captures=new List<object>();
        using(PolygonOps.BeginPocketGuideCapture(captures.Add))
        {
            var candidate=PolygonOps.OffsetPocketGuide(source,radius,.0001,default);
            Assert.Empty(PolygonOps.Difference(source,candidate));
            var reference=PolygonOps.OffsetPocketGuide(source,radius,.0001,default,useConservativeConstruction:false);
            Assert.Equal(Words(PolygonOps.Offset(source,radius,.0001,0,pruneCollapsedHoles:true)),Words(reference));
            Assert.Empty(PolygonOps.Difference(source,reference));
            Assert.All(candidate.SelectMany(p=>p),p=>Assert.True(double.IsFinite(p.x)&&double.IsFinite(p.y)));
        }
        var accepted=Assert.IsType<PolygonOps.PocketGuideSnapshot>(captures[0]);
        Assert.True(accepted.CompleteTargetCovered);Assert.True(accepted.ConservativeConstruction);
        Assert.Equal("conservative-complete-target",accepted.Outcome);
    }

    [Fact]
    public void UnsupportedSmallRadiusUsesTheLiteralOriginalOffset()
    {
        IReadOnlyList<IReadOnlyList<PointD>> source=[Box(-2,-3,4,5)];
        double radius=2/PathUtils.Scale;
        var captures=new List<object>();using var scope=PolygonOps.BeginPocketGuideCapture(captures.Add);
        var candidate=PolygonOps.OffsetPocketGuide(source,radius,.0001,default);
        Assert.Equal(Words(PolygonOps.Offset(source,radius,.0001,0,pruneCollapsedHoles:true)),Words(candidate));
        var captured=Assert.IsType<PolygonOps.PocketGuideSnapshot>(Assert.Single(captures));
        Assert.False(captured.ConservativeConstruction);Assert.Equal("unsupported-original-fallback",captured.Outcome);
    }

    [Fact]
    public void OracleCaptureIsDetachedBoundedAndHasNoObserverAuthority()
    {
        var ring=Box(0,0,4,4).ToArray();IReadOnlyList<IReadOnlyList<PointD>> source=[ring];
        var captures=new List<object>();List<List<PointD>>? output=null;
        using(PolygonOps.BeginPocketGuideCapture(captures.Add))
            for(int i=0;i<6;i++)output=PolygonOps.OffsetPocketGuide(source,.2,.0001,default);
        Assert.Equal(4,captures.Count);
        var snapshot=Assert.IsType<PolygonOps.PocketGuideSnapshot>(captures[0]);
        var originalPoint=snapshot.Input[0][0];var expandedPoint=snapshot.Expanded[0][0];
        ring[0]=new(500,500);output![0][0]=new(600,600);
        Assert.Equal(originalPoint,snapshot.Input[0][0]);Assert.Equal(expandedPoint,snapshot.Expanded[0][0]);
        IReadOnlyList<IReadOnlyList<PointD>> original=[Box(0,0,4,4)];
        var control=PolygonOps.OffsetPocketGuide(original,.2,.0001,default);
        using(PolygonOps.BeginPocketGuideCapture(_=>throw new InvalidOperationException("observer only")))
            Assert.Equal(Words(control),Words(PolygonOps.OffsetPocketGuide(original,.2,.0001,default)));
    }

    [Fact]
    public void CancellationBeforeConstructionAdmitsNoCaptureOrPartialGuide()
    {
        using var cancel=new CancellationTokenSource();cancel.Cancel();var captures=new List<object>();
        using var scope=PolygonOps.BeginPocketGuideCapture(captures.Add);
        Assert.ThrowsAny<OperationCanceledException>(()=>PolygonOps.OffsetPocketGuide([Box(0,0,10,10)],1,.0001,cancel.Token));
        Assert.Empty(captures);
    }
}
