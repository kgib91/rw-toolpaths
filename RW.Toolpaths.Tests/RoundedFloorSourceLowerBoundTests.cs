using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class RoundedFloorSourceLowerBoundTests
{
    [Fact]
    public void PrivatePathWorkersPreserveEverySubdivisionAndTheAggregateBudget()
    {
        var profile=new VCarveCutterProfile([new(CarvingCutterSectionKind.Hemisphere,0,1,1),
            new(CarvingCutterSectionKind.Linear,1,1,30)],31);
        var ramp=new TaggedToolpath(Enumerable.Range(0,513).Select(i=>new Point3D(i/16d,0,-i/32d)).ToArray(),
            [],0,"final-carve",null,detectCurves:false);
        var flat=new TaggedToolpath([new(15,3,-2),new(30,3,-2)],[],0,"final-carve",null,detectCurves:false);
        var paths=Enumerable.Range(0,16).Select(row=>new TaggedToolpath(
            Enumerable.Range(0,513).Select(i=>new Point3D(i/16d,.1+row/40d,-16)).ToArray(),
            [],row,"final-carve",null,detectCurves:false)).ToArray();
        VCarveSmoothing.FloorField Field()=>new([ramp,flat],profile,.5,16);
        var sequential=Field();var parallel=Field();var log=new List<string>();
        var expected=VCarveSmoothing.Constrain(paths,sequential,.5,default,200_000);
        var actual=VCarveSmoothing.Constrain(paths,parallel,.5,default,200_000,true,log.Add);
        Assert.Equal(expected.Count,actual.Count);
        for(int i=0;i<actual.Count;i++)
        {
            Assert.Equal(expected[i].RegionIndex,actual[i].RegionIndex);
            Assert.Equal(expected[i].Points,actual[i].Points);
        }
        if(RW.Toolpaths.Geometry.GeometryParallelism.WorkerLimit>1)
            Assert.Contains(log,s=>s.Contains("event=refined-paths")&&s.Contains("originalAggregatePointBudget=True"));
        long charged=actual.Sum(p=>(long)p.Points.Count-1);
        Assert.True(charged>100);
        foreach(bool useWorkers in new[]{false,true})
        {
            var refused=Field();
            var error=Assert.Throws<InvalidOperationException>(()=>VCarveSmoothing.Constrain(paths,refused,.5,default,100,useWorkers));
            Assert.Contains("floor proof exceeds 100 points",error.Message);
            if(useWorkers&&RW.Toolpaths.Geometry.GeometryParallelism.WorkerLimit>1)Assert.Equal(0,refused.CachedQueryCount);
        }
    }

    [Fact]
    public void PrivateQueryWorkersPublishTheExactOriginalDepthsAndContactWitnesses()
    {
        var profile=new VCarveCutterProfile([new(CarvingCutterSectionKind.Hemisphere,0,1,1),
            new(CarvingCutterSectionKind.Linear,1,1,30)],31);
        var ramp=new TaggedToolpath(Enumerable.Range(0,513).Select(i=>new Point3D(i/16d,0,-i/32d)).ToArray(),
            [],0,"final-carve",null,detectCurves:false);
        var flat=new TaggedToolpath([new(15,3,-2),new(30,3,-2)],[],0,"final-carve",null,detectCurves:false);
        var field=new VCarveSmoothing.FloorField([ramp,flat],profile,.5,16);
        var serial=new VCarveSmoothing.FloorField([ramp,flat],profile,.5,16);
        var points=Enumerable.Range(0,5000).Select(i=>new Clipper2Lib.PointD((i%13==0?2000d:0d)+i/160d,(i%37)/31d)).ToArray();
        var expected=points.Select(p=>serial.Depth(p.x,p.y)).ToArray();var log=new List<string>();
        int prepared=field.PrewarmQueries(points,4096,log.Add);
        if(RW.Toolpaths.Geometry.GeometryParallelism.WorkerLimit>1)
        {
            Assert.Equal(4096,prepared);Assert.Equal(4096,field.CachedQueryCount);
            Assert.Contains(log,s=>s.Contains("privateQueryState=True")&&s.Contains("proofBudgetsUnchanged=True"));
        }
        for(int i=0;i<points.Length;i++)
        {
            var p=points[i];Assert.Equal(expected[i],field.Depth(p.x,p.y));
            var pose=new Point3D(p.x,p.y,-expected[i]);
            Assert.Equal(serial.NominateFloorContact(pose),field.NominateFloorContact(pose));
        }
    }

    [Theory]
    [InlineData(.5)] [InlineData(1.19175359259421)] [InlineData(1.7320508075688772)] [InlineData(4)]
    public void EndpointAndGeneralTreesMatchTheIndependentCompleteContinuousFloor(double grade)
    {
        const double radius=1,cap=16;
        var profile=new VCarveCutterProfile([new(CarvingCutterSectionKind.Hemisphere,0,1,1),
            new(CarvingCutterSectionKind.Linear,1,1,160)],161);
        var ramp=new TaggedToolpath(Enumerable.Range(0,513).Select(i=>new Point3D(i/16d,0,-grade*i/16d)).ToArray(),
            [],0,"final-carve",null,detectCurves:false);
        var flat=new TaggedToolpath([new(15,3,-2),new(30,3,-2)],[],0,"final-carve",null,detectCurves:false);
        var field=new VCarveSmoothing.FloorField([ramp,ramp,flat],profile,grade,cap);
        var original=new VCarveSmoothing.FloorField([ramp,ramp,flat],profile,grade,cap,
            endpointCertificatesEnabled:false,directionalBoundsEnabled:false);
        var random=new Random(9121);
        double G(double distance)
        {
            double r=Math.Min(distance,radius*grade/Math.Sqrt(1+grade*grade));
            return r*r/(radius+Math.Sqrt(radius*radius-r*r))+grade*(distance-r);
        }
        for(int i=0;i<300;i++)
        {
            double x=i<20?i/10000d:random.NextDouble()*40-1,y=i<20?0:random.NextDouble()*6-1;
            double expected=Math.Min(cap,Math.Min(G(Math.Sqrt(x*x+y*y)),
                2+G(Math.Sqrt(Math.Pow(x-Math.Clamp(x,15,30),2)+(y-3)*(y-3)))));
            double actual=field.Depth(x,y),reference=original.Depth(x,y);
            Assert.InRange(actual,Math.Max(0,expected-.000201),expected+1e-10);
            Assert.InRange(Math.Abs(actual-reference),0,expected<.01?1.1e-7:.000201);
        }
        Assert.Contains("floorEndpointRepresentedSources=1024 ",field.QueryScratchDiagnostics);
        Assert.Contains("floorEndpointCertificates=512 ",field.QueryScratchDiagnostics);
        Assert.Contains("floorOriginalSourceCount=1025 ",field.QueryScratchDiagnostics);
        Assert.Contains("floorOriginalTreePrepared=False ",field.QueryScratchDiagnostics);
        Assert.Equal(original.Depth(2000,0),field.Depth(2000,0));
        Assert.Contains("floorOriginalTreePrepared=True ",field.QueryScratchDiagnostics);
        Assert.Contains("floorEndpointTreeFallbacks=1 ",field.QueryScratchDiagnostics);
    }

    [Fact]
    public void CompleteNearlyGradeLimitedSourcesStayAboveTheEndpointCertificate()
    {
        var random=new Random(41981);
        for(int i=0;i<500;i++)
        {
            double grade=.05+3.95*random.NextDouble(),radius=.1+4*random.NextDouble();
            var a=new Point3D(900+random.NextDouble(),-900+random.NextDouble(),-4);
            double dx=.001+2*random.NextDouble(),dy=.001+2*random.NextDouble();
            double rise=grade*Math.Sqrt(dx*dx+dy*dy)+(random.NextDouble()-.5)*1e-7;
            var b=new Point3D(a.X+dx,a.Y+dy,a.Z-rise);
            if((i&1)==0)(a,b)=(b,a);
            double x=a.X+4*random.NextDouble()-2,y=a.Y+4*random.NextDouble()-2;
            double F(double t)
            {
                double px=a.X+(b.X-a.X)*t,py=a.Y+(b.Y-a.Y)*t,z=a.Z+(b.Z-a.Z)*t;
                double distance=Math.Sqrt((x-px)*(x-px)+(y-py)*(y-py));
                double radial=Math.Min(distance,radius*grade/Math.Sqrt(1+grade*grade));
                return -z+radial*radial/(radius+Math.Sqrt(radius*radius-radial*radial))+grade*(distance-radial);
            }
            Assert.True(RoundedFloorSourceLowerBound.TryEndpointDrop(a,b,grade,out double drop));
            double lower=Math.BitDecrement(F(a.Z>=b.Z?0:1)-drop);
            for(int j=0;j<=1000;j++)Assert.True(lower<=F(j/1000d),$"case={i} t={j/1000d:R}");
            Assert.InRange(drop,2*RoundedFloorSourceLowerBound.RoundoffReserve,2e-7);
        }
        Assert.False(RoundedFloorSourceLowerBound.TryEndpointDrop(new(2000,0,0),new(2001,0,-1),.5,out _));
        Assert.False(RoundedFloorSourceLowerBound.TryEndpointDrop(new(0,0,0),new(1,0,-1),4.01,out _));
    }

    [Theory]
    [InlineData(false)] [InlineData(true)]
    public void CompleteSourceLowerPlaneStaysBelowIndependentConvexMinimum(bool poorNominee)
    {
        var random=new Random(44173);
        for(int i=0;i<400;i++)
        {
            double r=.1+3*random.NextDouble(),grade=.05+3.95*random.NextDouble();
            var a=new Point3D(900+random.NextDouble(),-900+random.NextDouble(),-12*random.NextDouble());
            var b=new Point3D(a.X+2*random.NextDouble(),a.Y+2*random.NextDouble(),a.Z+random.NextDouble()-.5);
            double x=a.X+3*random.NextDouble()-1,y=a.Y+3*random.NextDouble()-1;
            double lo=.3*random.NextDouble(),hi=.7+.3*random.NextDouble();
            Point3D At(double t)=>new(a.X+(b.X-a.X)*t,a.Y+(b.Y-a.Y)*t,a.Z+(b.Z-a.Z)*t);
            double F(double t)
            {
                var p=At(t);double d=Math.Sqrt((x-p.X)*(x-p.X)+(y-p.Y)*(y-p.Y));
                double radial=Math.Min(d,r*grade/Math.Sqrt(1+grade*grade));
                return -p.Z+radial*radial/(r+Math.Sqrt(r*r-radial*radial))+grade*(d-radial);
            }
            double l=lo,h=hi;
            for(int j=0;j<120;j++)
            {
                double u=l+(h-l)/3,v=h-(h-l)/3;
                if(F(u)<F(v))h=v;else l=u;
            }
            double t=(l+h)/2,expected=Math.Min(F(t),Math.Min(F(lo),F(hi)));
            Assert.True(RoundedFloorSourceLowerBound.TryLower(a,b,lo,hi,x,y,r,grade,
                At(poorNominee?lo:t),out double lower));
            Assert.True(lower<=expected+1e-10,$"case={i} lower={lower:R} independent={expected:R}");
            // A ternary nominee loses gradient precision when rounded values
            // tie. That can widen this gap; it never raises the lower proof.
            if(!poorNominee)Assert.InRange(expected-lower,0,1e-5);
        }
    }

    [Fact]
    public void UnsupportedCoordinatesAndProfilesRetainTheOriginalQuery()
    {
        var a=new Point3D(0,0,-1);var b=new Point3D(1,0,-2);
        Assert.False(RoundedFloorSourceLowerBound.TryLower(a,b,0,1,2000,0,1,.5,a,out _));
        Assert.False(RoundedFloorSourceLowerBound.TryLower(a,b,0,1,0,0,.01,.5,a,out _));
        Assert.False(RoundedFloorSourceLowerBound.TryLower(a,b,0,1,0,0,1,4.01,a,out _));
        Assert.False(RoundedFloorSourceLowerBound.TryLower(a,b,double.NaN,1,0,0,1,.5,a,out _));
    }
}
