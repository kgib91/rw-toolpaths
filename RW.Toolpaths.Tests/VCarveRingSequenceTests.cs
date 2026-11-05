using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class VCarveRingSequenceTests
{
    private static TaggedToolpath Cut(params Point3D[] points)=>new(points,Array.Empty<ToolpathSpan>(),0,"clearing",0,detectCurves:false);
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ConfiguredOffsetsStayOnTheLocalStripBeforeADistantPiece(bool bottomUp)
    {
        int[] construction=bottomUp?new[]{3,1,2,0}:new[]{0,2,1,3};
        var plan=new VCarveRingOrder.Plan(new[]{-1,0,0,1},construction,bottomUp);
        var paths=Enumerable.Range(0,4).Select(r=>Cut(new(r==2?100:.1*r,0,-1),new(r==2?101:.1*r+1,0,-1))
            with{ConcentricSource=new(plan,r),ContinuationSourceOrdinal=r}).ToArray();
        var order=new VCarveRingOrder(paths,bottomUp,default,.7);var actual=new List<int>();
        for(int i=0;i<paths.Length;i++)
        {
            int owner=Assert.Single(order.Candidates(new(0,0,-1)));actual.Add(owner);order.Complete(owner);
        }
        Assert.Equal(bottomUp?new[]{3,1,0,2}:new[]{0,1,3,2},actual);Assert.Empty(order.Candidates(null));
        Console.WriteLine($"[VCarve.ConstructionOrder.Test] bottomUp={bottomUp} pieces={string.Join(',',actual)} configuredLocalOffsetProgression=True");
    }
    [Fact]
    public void LocalSeamUsesTheClosestFiniteSideOfTheSelectedRow()
    {
        var longRing=Cut(new(.1,-100,-1),new(1,-100,-1),new(1,100,-1),new(.1,100,-1),new(.1,-100,-1));
        var decoy=Cut(new(.2,0,-1),new(.3,0,-1));
        var seams=new VCarveRingSeams(new[]{longRing,decoy},default);
        var nearest=seams.Nearest(new(0,0,-1),new[]{0},double.PositiveInfinity).First();
        Assert.Equal(0,nearest.Owner);Assert.Equal(new Point3D(.1,0,-1),nearest.Point);
        var split=seams.Rebase(nearest,(_,_)=>true)!.Value;
        Assert.Equal(nearest.Point,split.Points[0]);Assert.Equal(split.Points[0],split.Points[^1]);
        double Length(TaggedToolpath p)=>p.Points.Zip(p.Points.Skip(1)).Sum(x=>ToolpathContinuationPlanner.Distance(x.First,x.Second));
        Assert.Equal(Length(longRing),Length(split),9);Assert.Equal(5,longRing.Points.Count);
    }
    [Fact]
    public void ANewSeamStillRequiresBothActualSubdivisionsToPassTheOriginalGuard()
    {
        var path=Cut(new(.1,-100,-1),new(.1,100,-1));
        var seams=new VCarveRingSeams(new[]{path},default);
        var landing=seams.Nearest(new(0,0,-1),new[]{0},1).First();int calls=0;
        Assert.Null(seams.Rebase(landing,(_,_)=>++calls!=2));
        Assert.Equal(2,calls);Assert.Equal(2,path.Points.Count);Assert.Equal(1,seams.SplitRefusals);
    }
    [Fact]
    public void AnImmediateCutUsesItsExactNativeContactDespiteArithmeticHeightDifference()
    {
        var plan=new VCarveRingOrder.Plan(new[]{-1,0});
        var first=Cut(new(0,0,-1),new(5,0,-1),new(5,5,-1),new(0,5,-1),new(0,0,-1))
            with{ConcentricSource=new(plan,0),ContinuationSourceOrdinal=0};
        var second=Cut(new(.2,.2,-1.000000000000008),new(.4,.2,-1.000000000000008),new(.4,.4,-1.000000000000008),
            new(.2,.4,-1.000000000000008),new(.2,.2,-1.000000000000008)) with{ConcentricSource=new(plan,1),ContinuationSourceOrdinal=1};
        var result=VCarvePathRouter.Route(new[]{first,second},[],new ToolGeometry(2){BottomRadius=0,TopRadius=2,ConeLength=4},
            new DepthSchedule(2,1),new VCarveOptions{SmoothVCarving=true,DetectCurves=false,StepOver=.45,Ramp=new RampSettings(.1)},
            0,null,continuationCutGuard:(_,_)=>true,materialEntryHeight:p=>p.Z);
        Assert.Equal(result[0].Points[^1],result[1].Points[0]);Assert.Contains(second.Points[0],result[1].Points);
    }
    [Fact]
    public void PiecesOfTheSameConstructedRowMayStillJoinAtALiteralContact()
    {
        var plan=new VCarveRingOrder.Plan(new[]{-1});
        var a=Cut(new(0,0,-1),new(2,0,-1)) with{ConcentricSource=new(plan,0)};
        var b=Cut(new(1,-1,-1),new(1,1,-1)) with{ConcentricSource=new(plan,0)};
        var result=ToolpathSourceTraversal.Detours(new[]{a,b},default,null,(_,_)=>true);
        Assert.Equal(0,Assert.Single(result).ConcentricSource!.Ring);
    }
}
