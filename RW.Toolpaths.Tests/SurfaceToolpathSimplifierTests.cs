using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class SurfaceToolpathSimplifierTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void DenseCurvedCutsBecomeShortPathsWhileTheSweptBallSurfaceStaysWithinFiveMicrons(bool perSegmentCutSpans)
    {
        var source=Enumerable.Range(0,2001).Select(i=>{
            double t=i/2000d;return new Point3D(20*t,2*Math.Sin(t*4),-2-.3*Math.Cos(t*7));
        }).ToArray();
        var sourceSpans=perSegmentCutSpans?Enumerable.Range(1,source.Length-1)
            .Select(i=>new ToolpathSpan(i-1,i,ToolpathSpanKind.Cut)).ToArray():Array.Empty<ToolpathSpan>();
        var original=new TaggedToolpath(source,sourceSpans,3,"clearing",2,17,detectCurves:false);
        var candidate=SurfaceToolpathSimplifier.Simplify(original,.005,default);
        Assert.InRange(candidate.Points.Count,4,source.Length/10);
        Assert.Equal(source[0],candidate.Points[0]);Assert.Equal(source[^1],candidate.Points[^1]);
        Assert.Equal(3,candidate.RegionIndex);Assert.Equal(2,candidate.DepthPassIndex);Assert.Equal(17,candidate.VisitIdentity);
        Assert.Equal(candidate.Points.Count-1,candidate.Moves.Count);
        // Independent continuous swept-sphere SDF. The physical surface can
        // change by the spatial budget; an identical old point list is not required.
        var random=new Random(5189);
        for(int i=0;i<800;i++)
        {
            var q=new Point3D(random.NextDouble()*24-2,random.NextDouble()*8-4,random.NextDouble()*8-6);
            double a=BallSweep(q,source,.8),b=BallSweep(q,candidate.Points,.8);
            Assert.InRange(Math.Abs(a-b),0,.0050000001);
        }
    }

    [Fact]
    public void KeepsTinyClosedDetailsSpanBoundariesAndRetraces()
    {
        Point3D[] p=[new(0,0,-1),new(.0001,0,-1),new(.001,.001,-1),new(.002,0,-1),new(.001,-.001,-1),new(0,0,-1)];
        var path=new TaggedToolpath(p,[new(0,2,ToolpathSpanKind.Cut),new(2,5,ToolpathSpanKind.Link)],0,"final-carve",0,detectCurves:false);
        var result=SurfaceToolpathSimplifier.Simplify(path,.005,default);
        Assert.Contains(p[2],result.Points);Assert.Contains(p[3],result.Points);
        Assert.Equal(p[0],result.Points[0]);Assert.Equal(p[^1],result.Points[^1]);
        Assert.Equal(ToolpathSpanKind.Cut,result.Spans[0].Kind);
        Assert.Equal(p[2],result.Points[result.Spans[0].EndIndex]);
        Assert.Equal(result.Spans[0].EndIndex,result.Spans[1].StartIndex);
        Assert.True(BallSweep(new(.001,.001,-1),result.Points,.001)<=0);
    }

    [Fact]
    public void FreshPhysicalRefusalsKeepTheRequiredOriginalChords()
    {
        var p=Enumerable.Range(0,101).Select(i=>new Point3D(i*.01,0,-1)).ToArray();
        var path=new TaggedToolpath(p,Array.Empty<ToolpathSpan>(),0,"clearing",0,detectCurves:false);
        var result=SurfaceToolpathSimplifier.Simplify(path,.005,default,
            (a,b)=>b.X-a.X<=.031);
        Assert.InRange(result.Points.Count,35,100);
        Assert.All(result.Points.Zip(result.Points.Skip(1)),s=>Assert.InRange(s.Second.X-s.First.X,0,.031));
    }

    [Fact]
    public void ASubF3PlanarAnchorAfterAVerticalPrefixIsRetained()
    {
        Point3D[] p=[new(50,50,5),new(50,50,4),new(50,50,3),new(50.0001,50,3),
            new(50.0001,50,2),new(50.001,50,1),new(50.02,50,-1)];
        var path=new TaggedToolpath(p,[],0,"clearing",0,detectCurves:false);
        var result=SurfaceToolpathSimplifier.Simplify(path,.002,default);
        Assert.Contains(p[2],result.Points);Assert.Contains(p[3],result.Points);
    }

    [Fact]
    public void CancellationDoesNotPublishAPartialNomination()
    {
        using var cancel=new CancellationTokenSource();cancel.Cancel();
        var path=new TaggedToolpath(new Point3D[]{new(0,0,-1),new(1,0,-1)},Array.Empty<ToolpathSpan>(),0,"clearing",0,detectCurves:false);
        Assert.Throws<OperationCanceledException>(()=>SurfaceToolpathSimplifier.Nominate([path],.005,cancel.Token));
    }

    private static double BallSweep(Point3D q,IReadOnlyList<Point3D> points,double radius)
    {
        double nearest=double.PositiveInfinity;
        for(int i=1;i<points.Count;i++)
        {
            var a=points[i-1];var b=points[i];
            double vx=b.X-a.X,vy=b.Y-a.Y,vz=b.Z-a.Z;
            double denominator=vx*vx+vy*vy+vz*vz;
            double t=denominator==0?0:((q.X-a.X)*vx+(q.Y-a.Y)*vy+(q.Z-a.Z)*vz)/denominator;
            t=Math.Max(0,Math.Min(1,t));
            double dx=q.X-a.X-t*vx,dy=q.Y-a.Y-t*vy,dz=q.Z-a.Z-t*vz;
            nearest=Math.Min(nearest,Math.Sqrt(dx*dx+dy*dy+dz*dz)-radius);
        }
        return nearest;
    }
}
