using System.Globalization;
using RW.Toolpaths.Milling;
using Xunit;

namespace RW.Toolpaths.Tests;

// Real original nominal/posted grade predicates, synthetic true floor/wall
// owners only. These facts grant no floor/wall/coverage/stock authority.
public sealed class ProductionF3DualGradeControls
{
    const double Angle=15;
    static readonly double Grade=Math.Tan(Angle*Math.PI/180);
    static readonly double Surface=(double)12.7f;
    static Point3D Command(Point3D p,double surface)
    {
        static double F3(float x)=>double.Parse(x.ToString("F3",CultureInfo.InvariantCulture),CultureInfo.InvariantCulture);
        return new(F3((float)p.X),F3((float)p.Y),F3((float)(surface+p.Z)-(float)surface));
    }
    static bool OriginalGrade(TaggedToolpath p)=>VCarveSmoothing.MaximumGrade(new[]{p})<=Grade+1e-12;
    static TaggedToolpath Path(Point3D[] p,ToolpathSpan[]? spans=null)
        =>new(p,spans??Array.Empty<ToolpathSpan>(),7,"final-carve",3,11,detectCurves:false);
    static ExactF3TrajectorySimplifier.GuardDecision ActualGrade(TaggedToolpath p)
        =>new(true,true,true,OriginalGrade(p),true,"real-grade-synthetic-other-owners");

    [Fact]
    public void CapturedNativeChordRefusesOriginalRouterAlthoughItsPostedViewPasses()
    {
        Point3D a=new(59.10284423828125,116.88131713867188,11.452866522078077);
        Point3D b=new(59.135894775390625,116.8712158203125,11.443606241913713);
        double run=Math.Sqrt((b.X-a.X)*(b.X-a.X)+(b.Y-a.Y)*(b.Y-a.Y));
        double rise=Math.Abs(b.Z-a.Z),permittedDrop=Grade*run,excess=rise-permittedDrop;
        Assert.Equal(.034559725632817465,run);
        Assert.Equal(.009260280164363976,rise);
        Assert.Equal(2.9590409365842785e-8,excess);
        Assert.True(excess>1e-8); // Original predicate uses mm, not slope ratio.
        Assert.InRange(excess*1_000_000,.029590409365,.029590409367);
        Assert.False(OriginalGrade(Path(new[]{a,b})));
        Point3D relativeA=a with{Z=a.Z-Surface},relativeB=b with{Z=b.Z-Surface};
        Assert.True(OriginalGrade(Path(new[]{Command(relativeA,Surface),Command(relativeB,Surface)})));
        // Invoke the ORIGINAL private surface conversion, rather than a copied
        // approximation: nominal-relative -> CompactAndClamp -> entry -> guard.
        var compactMethod=typeof(VCarveToolpaths).GetMethod("CompactAndClamp",
            System.Reflection.BindingFlags.NonPublic|System.Reflection.BindingFlags.Static)!;
        var compacted=(List<Point3D>)compactMethod.Invoke(null,
            new object[]{new[]{relativeA,relativeB},Surface,6.35,true})!;
        Assert.Equal(new[]{a,b},compacted);
        Assert.True(VCarveToolpaths.HasCommandedPenetration(compacted,Surface));
        var capturedPath=new TaggedToolpath(compacted,Array.Empty<ToolpathSpan>(),0,"clearing",null,detectCurves:false);
        var logs=new List<string>();
        var failure=Assert.Throws<InvalidOperationException>(()=>VCarvePathRouter.Route(
            new[]{capturedPath},Array.Empty<IReadOnlyList<Clipper2Lib.PointD>>(),
            ToolGeometry.Flat(1),new DepthSchedule(6.35,2){SurfaceZ=Surface,StartDepth=6.35},
            new VCarveOptions{SmoothVCarving=true,SmoothRampAngleDegrees=Angle,DetectCurves=false,
                CompactCollinearCommands=false},0,null,diagnostic:logs.Add));
        Assert.Equal("Smooth V-carve routing exceeded the requested ramp angle.",failure.Message);
        Assert.Contains(logs,l=>l.Contains("stage=smooth-routing event=reject"));
    }

    [Fact]
    public void ActualPassNominalFailKeepsTheOriginallyLegalDetourAndItsOrdinals()
    {
        Point3D a=new(59.10284423828125,116.88131713867188,11.452866522078077-Surface);
        Point3D b=new(59.135894775390625,116.8712158203125,11.443606241913713-Surface);
        double dx=b.X-a.X,dy=b.Y-a.Y,length=Math.Sqrt(dx*dx+dy*dy);
        const double detour=.00008; // Fixture geometry, no production allowance.
        Point3D middle=new((a.X+b.X)/2-dy/length*detour,(a.Y+b.Y)/2+dx/length*detour,(a.Z+b.Z)/2);
        var points=new[]{a with{X=a.X-.002},a,middle,b};var original=Path(points);
        Assert.True(OriginalGrade(original));
        Assert.False(OriginalGrade(Path(new[]{a,b})));
        Assert.True(OriginalGrade(Path(new[]{Command(a,Surface),Command(b,Surface)})));
        var owner=new VCarveActualF3TrajectoryCompaction(Surface,p=>Command(p,Surface),ActualGrade,
            OriginalGrade,default,null);
        var result=Assert.Single(owner.Compact(new[]{original},"actual-pass-nominal-refusal"));
        Assert.Equal(original.Points,result.Points);Assert.Equal(original.Spans,result.Spans);
        Assert.True(owner.NominalGradeChecks>0);Assert.True(owner.NominalGradeRefusals>0);
        Assert.Equal(0,owner.ActualGradeRefusals);Assert.Equal(0,owner.Guarded);
        Assert.Equal(points,original.Points);
    }

    [Fact]
    public void ActualFailNominalPassKeepsEveryOriginalPointWithoutBorrowingTheOtherProof()
    {
        Point3D a=new(0,0,-.1),b=new(.01,0,-.1026);
        var points=new[]{a with{X=-.005},a,new Point3D(.005,0,-.1013),b};var original=Path(points);
        Assert.True(OriginalGrade(Path(new[]{a,b})));
        Assert.False(OriginalGrade(Path(new[]{Command(a,Surface),Command(b,Surface)})));
        var owner=new VCarveActualF3TrajectoryCompaction(Surface,p=>Command(p,Surface),ActualGrade,
            OriginalGrade,default,null);
        var result=Assert.Single(owner.Compact(new[]{original},"nominal-pass-actual-refusal"));
        Assert.Equal(original.Points,result.Points);Assert.Equal(original.Spans,result.Spans);
        Assert.True(owner.ActualGradeRefusals>0);Assert.True(owner.NominalGradeChecks>0);
        Assert.Equal(0,owner.NominalGradeRefusals);Assert.Equal(0,owner.Guarded);
    }

    [Fact]
    public void BothGradeProofsReceiveTheirOwnCoordinatesAndSelectedSourceCustody()
    {
        var points=Enumerable.Range(0,16).Select(i=>new Point3D(i*.01+.00001,(i&1)*.001,-.1)).ToArray();
        ToolpathSpan[] spans={new(0,4,ToolpathSpanKind.Cut),new(4,15,ToolpathSpanKind.Ramp)};
        var original=Path(points,spans);int nominalCalls=0,actualCalls=0;
        var owner=new VCarveActualF3TrajectoryCompaction(Surface,p=>Command(p,Surface),actual=>
        {
            actualCalls++;foreach(var p in actual.Points)Assert.Contains(p,points.Select(q=>Command(q,Surface)));
            return ActualGrade(actual);
        },nominal=>
        {
            nominalCalls++;Assert.Equal(original.RegionIndex,nominal.RegionIndex);
            Assert.Equal(original.Category,nominal.Category);Assert.Equal(original.DepthPassIndex,nominal.DepthPassIndex);
            Assert.Equal(original.VisitIdentity,nominal.VisitIdentity);
            foreach(var p in nominal.Points)Assert.Contains(p,points);
            return OriginalGrade(nominal);
        },default,null);
        var result=Assert.Single(owner.Compact(new[]{original},"dual-grade-coordinate-custody"));
        Assert.True(result.Points.Count<original.Points.Count);Assert.True(nominalCalls>1);
        Assert.Equal(actualCalls,nominalCalls);Assert.Equal(nominalCalls,owner.NominalGradeChecks);
        Assert.Contains(points[1],result.Points);Assert.Contains(points[4],result.Points);
        Assert.Equal(points[0],result.Points[0]);Assert.Equal(points[^1],result.Points[^1]);
        foreach(var p in result.Points)Assert.Contains(p,points);
        Assert.Equal(spans.Select(s=>s.Kind),result.Spans.Select(s=>s.Kind));
    }

    [Fact]
    public void NominalGradeCallbackCancellationCannotPublishOrMutateTheSource()
    {
        var points=Enumerable.Range(0,16).Select(i=>new Point3D(i*.01,(i&1)*.001,-.1)).ToArray();
        var original=Path(points);var before=points.ToArray();using var cancel=new CancellationTokenSource();
        var owner=new VCarveActualF3TrajectoryCompaction(Surface,p=>Command(p,Surface),ActualGrade,
            nominal=>{cancel.Cancel();return true;},cancel.Token,null);
        Assert.Throws<OperationCanceledException>(()=>owner.Compact(new[]{original},"nominal-callback-cancel"));
        Assert.Equal(before,points);Assert.Equal(before,original.Points);Assert.Equal(0,owner.Guarded);
    }
}
