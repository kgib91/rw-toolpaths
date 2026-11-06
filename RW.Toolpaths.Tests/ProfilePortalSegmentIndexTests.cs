using System.Text.Json;
using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class ProfilePortalSegmentIndexTests
{
    private static PointD[] Circle(double x,double y,double radius,int count=96)=>Enumerable.Range(0,count)
        .Select(i=>new PointD(x+radius*Math.Cos(Math.Tau*i/count),y+radius*Math.Sin(Math.Tau*i/count))).ToArray();
    private static PointD[] DenseBox(double x0,double y0,double x1,double y1,int copies=10)=>
        new[]{new PointD(x0,y0),new PointD(x1,y0),new PointD(x1,y1),new PointD(x0,y1)}
            .SelectMany(p=>Enumerable.Repeat(p,copies)).ToArray();
    private static long Bits(double x)=>BitConverter.DoubleToInt64Bits(x);
    private static long[] Words(ProfilePortal p)=>[p.FromRingIndex,Bits(p.FromPoint.x),Bits(p.FromPoint.y),
        p.ToRingIndex,Bits(p.ToPoint.x),Bits(p.ToPoint.y),Bits(p.Distance)];

    public static IEnumerable<object[]> Cases()
    {
        yield return ["separated-dense",Circle(0,0,10,320),Circle(27,3,8,240)];
        yield return ["nested-hole-boundaries",Circle(-4,-7,12),Circle(-4,-7,3)];
        yield return ["crossing-first-intersection",DenseBox(-9,-1,9,1),DenseBox(-1,-9,1,9)];
        yield return ["touching",Circle(0,0,3),Circle(6,0,3)];
        yield return ["zero-length-and-equal-ties",DenseBox(-5,-5,5,5),DenseBox(-4,-4,4,4)];
        yield return ["reversed-cycle-ties",DenseBox(-3,0,3,1).Reverse().ToArray(),DenseBox(-3,2,3,3)];
        yield return ["near-coordinate-limit",Circle(1e12-100,1e12-100,8),Circle(1e12-70,1e12-100,8)];
        yield return ["extreme-literal-fallback",DenseBox(1e15,1e15,1e15+10,1e15+10),DenseBox(1e15+20,1e15,1e15+30,1e15+10)];
        var nonfinite=Circle(0,0,3);nonfinite[15]=new(double.NaN,2);
        yield return ["nonfinite-literal-fallback",Circle(12,0,3),nonfinite];
        yield return ["very-short",DenseBox(0,0,1e-10,1e-10),DenseBox(2e-10,0,3e-10,1e-10)];
    }

    [Theory]
    [MemberData(nameof(Cases))]
    public void IndexMatchesIndependentOriginalLiteralLoopBitForBit(string name,PointD[] first,PointD[] second)
    {
        _=name;var expected=Original(7,first,11,second);
        var metrics=new ProfileRoutePlanner.PortalSearchMetrics();
        var actual=ProfileRoutePlanner.FindClosestPortal(7,first,11,second,metrics);
        Assert.Equal(Words(expected),Words(actual));
        if(name.Contains("fallback"))Assert.Equal(0,metrics.IndexBuilds);
    }

    [Fact]
    public void RandomDenseCyclesKeepStrictNearestAndLiteralEndpointTies()
    {
        var random=new Random(18764);
        for(int trial=0;trial<32;trial++)
        {
            PointD[] Polygon(double x)=>Enumerable.Range(0,64).Select(i=>{
                double a=Math.Tau*i/64,r=2+random.NextDouble();return new PointD(x+r*Math.Cos(a),r*Math.Sin(a));}).ToArray();
            var first=Polygon(-4);var second=Polygon(4+random.NextDouble());
            Assert.Equal(Words(Original(0,first,1,second)),Words(ProfileRoutePlanner.FindClosestPortal(0,first,1,second)));
        }
    }

    [Fact]
    public void IndexRejectsPairWalksWithoutChangingOriginalSolveAndBudgetFallback()
    {
        var first=Circle(0,0,10,512);var second=Circle(40,3,8,512);
        var indexed=new ProfileRoutePlanner.PortalSearchMetrics();var literal=new ProfileRoutePlanner.PortalSearchMetrics();
        var result=ProfileRoutePlanner.FindClosestPortal(0,first,1,second,indexed);
        var control=ProfileRoutePlanner.FindClosestPortal(0,first,1,second,literal,useSegmentIndex:false);
        Assert.Equal(Words(control),Words(result));Assert.True(indexed.IndexBuilds>0);Assert.True(indexed.IndexQueries>0);
        Assert.True(indexed.IndexRejectedPairs>0);Assert.True(indexed.SegmentPairsConsidered<literal.SegmentPairsConsidered/2);
        Assert.InRange(indexed.IndexReservedBytes,1,ProfileRoutePlanner.MaximumSegmentIndexBytes);
        var budget=new ProfileRoutePlanner.PortalSearchMetrics();
        Assert.Equal(Words(control),Words(ProfileRoutePlanner.FindClosestPortal(0,first,1,second,budget,maximumSegmentIndexBytes:0)));
        Assert.Equal(0,budget.IndexBuilds);Assert.Equal(1,budget.IndexFallbackSolves);
    }

    [Fact]
    public void CompletePlanAndEmittedMetadataAndMachineEstimateMatchLiteralPrim()
    {
        var rings=Enumerable.Range(0,9).Select(i=>RingOffsetEngine.CreateRing(Circle(i*6,0,2.5),0,0)).ToArray();
        var sources=new[]{new ProfileSourceComponent(0,[DenseBox(-5,-5,55,5)])};
        var literal=ProfileRoutePlanner.Plan(rings,sources,.5,default,useSegmentIndex:false);
        var indexed=ProfileRoutePlanner.Plan(rings,sources,.5,default);
        var noBudget=ProfileRoutePlanner.Plan(rings,sources,.5,default,maximumSegmentIndexBytes:0);
        var json=new JsonSerializerOptions{IncludeFields=true};
        Assert.Equal(JsonSerializer.Serialize(literal,json),JsonSerializer.Serialize(indexed,json));
        Assert.Equal(JsonSerializer.Serialize(literal,json),JsonSerializer.Serialize(noBudget,json));
        var motion=new MillingMotionSettings(600,100,3000,5);
        var options=new ProfileOptions{Side=ProfileSide.OnLine,CalculateSweptArea=false,Motion=motion};
        var depth=new DepthSchedule(.5,.5);var tool=ToolGeometry.Flat(.5);
        var oldPlan=ProfileRouteMaterializer.Emit(rings,literal,depth,options,tool,default,null);
        var newPlan=ProfileRouteMaterializer.Emit(rings,indexed,depth,options,tool,default,null);
        Assert.Equal(JsonSerializer.Serialize(oldPlan,json),JsonSerializer.Serialize(newPlan,json));
        var cost=new ToolpathMotionCost(motion,0);
        Assert.Equal(Bits(cost.SequenceSeconds(null,oldPlan.Toolpaths,null)),Bits(cost.SequenceSeconds(null,newPlan.Toolpaths,null)));
    }

    [Fact]
    public void SourceClusterBoundarySolvesShareTheBoundedIndexAndRetainForestAndCycle()
    {
        var sources=new[]{new ProfileSourceComponent(0,[Circle(0,0,12,160),Circle(0,0,5).Reverse().ToArray()]),
            new ProfileSourceComponent(1,[Circle(16,0,2)]),new ProfileSourceComponent(2,[Circle(29,0,7,128)])};
        var rings=new[]{RingOffsetEngine.CreateRing(Circle(0,0,11),0,0),
            RingOffsetEngine.CreateRing(Circle(0,0,6),0,0,isHole:true),
            RingOffsetEngine.CreateRing(Circle(16,0,1),0,1),RingOffsetEngine.CreateRing(Circle(29,0,6),0,2)};
        var literal=ProfileRoutePlanner.Plan(rings,sources,2.1,default,useSegmentIndex:false);
        var indexed=ProfileRoutePlanner.Plan(rings,sources,2.1,default);
        var noBudget=ProfileRoutePlanner.Plan(rings,sources,2.1,default,maximumSegmentIndexBytes:0);
        Assert.Equal(2,indexed.Trees.Count);
        var json=new JsonSerializerOptions{IncludeFields=true};
        Assert.Equal(JsonSerializer.Serialize(literal,json),JsonSerializer.Serialize(indexed,json));
        Assert.Equal(JsonSerializer.Serialize(literal,json),JsonSerializer.Serialize(noBudget,json));
        var motion=new MillingMotionSettings(600,100,3000,5);
        var options=new ProfileOptions{Side=ProfileSide.OnLine,CalculateSweptArea=false,Motion=motion};
        var depth=new DepthSchedule(.5,.5);var tool=ToolGeometry.Flat(.5);
        var oldPlan=ProfileRouteMaterializer.Emit(rings,literal,depth,options,tool,default,null);
        var newPlan=ProfileRouteMaterializer.Emit(rings,indexed,depth,options,tool,default,null);
        Assert.Equal(JsonSerializer.Serialize(oldPlan,json),JsonSerializer.Serialize(newPlan,json));
        var cost=new ToolpathMotionCost(motion,0);
        Assert.Equal(Bits(cost.SequenceSeconds(null,oldPlan.Toolpaths,null)),Bits(cost.SequenceSeconds(null,newPlan.Toolpaths,null)));
    }

    [Fact]
    public void CancellationAndInvalidIndexBudgetCannotProducePartialPlan()
    {
        var rings=new[]{RingOffsetEngine.CreateRing(Circle(0,0,3),0,0)};
        var sources=new[]{new ProfileSourceComponent(0,[Circle(0,0,3)])};
        using var cancel=new CancellationTokenSource();cancel.Cancel();
        Assert.ThrowsAny<OperationCanceledException>(()=>ProfileRoutePlanner.Plan(rings,sources,1,cancel.Token));
        Assert.Throws<ArgumentOutOfRangeException>(()=>ProfileRoutePlanner.Plan(rings,sources,1,default,maximumSegmentIndexBytes:-1));
        Assert.Throws<ArgumentOutOfRangeException>(()=>ProfileRoutePlanner.FindClosestPortal(0,rings[0].Points,1,rings[0].Points,
            maximumSegmentIndexBytes:ProfileRoutePlanner.MaximumSegmentIndexBytes+1));
    }

    // Independent copy of the original literal algorithm, including its raw
    // bounds predicate, first-intersection return, projection and strict ties.
    private static ProfilePortal Original(int ai,IReadOnlyList<PointD> a,int bi,IReadOnlyList<PointD> b)
    {
        PointD bestA=a[0],bestB=b[0];double best=double.MaxValue;
        for(int i=0;i<a.Count;i++)
        for(int j=0;j<b.Count;j++)
        {
            var a0=a[i];var a1=a[(i+1)%a.Count];var b0=b[j];var b1=b[(j+1)%b.Count];
            double dx=Math.Max(0,Math.Max(Math.Min(a0.x,a1.x)-Math.Max(b0.x,b1.x),Math.Min(b0.x,b1.x)-Math.Max(a0.x,a1.x)));
            double dy=Math.Max(0,Math.Max(Math.Min(a0.y,a1.y)-Math.Max(b0.y,b1.y),Math.Min(b0.y,b1.y)-Math.Max(a0.y,a1.y)));
            if(dx*dx+dy*dy>=best)continue;
            double ax=a1.x-a0.x,ay=a1.y-a0.y,bx=b1.x-b0.x,by=b1.y-b0.y,den=ax*by-ay*bx;
            if(Math.Abs(den)>=1e-12)
            {
                double ox=b0.x-a0.x,oy=b0.y-a0.y,t=(ox*by-oy*bx)/den,u=(ox*ay-oy*ax)/den;
                if(t>=0&&t<=1&&u>=0&&u<=1){var p=At(a0,a1,t);return new(ai,p,bi,p,0);}
            }
            Consider(a0,Project(a0,b0,b1));Consider(a1,Project(a1,b0,b1));
            Consider(Project(b0,a0,a1),b0);Consider(Project(b1,a0,a1),b1);
        }
        return new(ai,bestA,bi,bestB,Math.Sqrt(best));
        void Consider(PointD x,PointD y){double dx=x.x-y.x,dy=x.y-y.y,d=dx*dx+dy*dy;if(d>=best)return;best=d;bestA=x;bestB=y;}
        static PointD At(PointD x,PointD y,double t)=>new(x.x+(y.x-x.x)*t,x.y+(y.y-x.y)*t);
        static PointD Project(PointD p,PointD x,PointD y)
        {double dx=y.x-x.x,dy=y.y-x.y,l=dx*dx+dy*dy;if(l<=1e-18)return x;
            double t=Math.Clamp(((p.x-x.x)*dx+(p.y-x.y)*dy)/l,0,1);return At(x,y,t);}
    }
}
