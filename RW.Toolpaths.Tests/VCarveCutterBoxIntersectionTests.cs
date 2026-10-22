using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class VCarveCutterBoxIntersectionTests
{
    [Fact]
    public void StationaryFootprintMatchesGeneralSweepIncludingNearTangenciesAndTranslatedCells()
    {
        var random=new Random(8932);
        VCarveCutterProfile[] profiles=[Profile,
            new([new(CarvingCutterSectionKind.Linear,.2,4,8)],8),
            new([new(CarvingCutterSectionKind.Linear,2,2,100)],100)];
        foreach(var profile in profiles)
        for(int i=0;i<4000;i++)
        {
            double x=i%3==0?1_000_000:i%3==1?-1_000_000:0;
            double floor=random.NextDouble()*10,depth=random.NextDouble()*Math.Min(20,profile.MaximumDepth);
            var point=new Point3D(x,0,-floor-depth);
            double radius=profile.PhysicalRadiusAtDepth(depth),angle=random.NextDouble()*Math.Tau;
            double offset=i%2==0?random.NextDouble()*2-1:(random.NextDouble()-.5)*1e-8;
            double qx=x+(radius+offset)*Math.Cos(angle),qy=(radius+offset)*Math.Sin(angle);
            double w=random.NextDouble();
            Assert.Equal(VCarveCutterBoxIntersection.IntersectsGeneral(point,point,profile,floor,qx,qy,qx+w,qy+w),
                VCarveCutterBoxIntersection.Intersects(point,point,profile,floor,qx,qy,qx+w,qy+w));
            Assert.True(VCarveCutterBoxIntersection.TryPointRadiusSquared(profile,-point.Z-floor,out double square));
            Assert.Equal(VCarveCutterBoxIntersection.IntersectsGeneral(point,point,profile,floor,qx,qy,qx+w,qy+w),
                VCarveCutterBoxIntersection.PointBoxDistanceSquared(point,qx,qy,qx+w,qy+w)-square<-1e-12);
        }
    }

    private static readonly VCarveCutterProfile Profile = new([
        new(CarvingCutterSectionKind.Hemisphere,0,1.5875,1.5875),
        new(CarvingCutterSectionKind.Linear,1.5875,3,20)],21.5875);

    [Fact]
    public void ExactMovingProfileNeverMissesIndependentlySampledProtectedMaterial()
    {
        var random=new Random(3917);
        for(int c=0;c<250;c++)
        {
            var a=new Point3D(random.NextDouble()*8-4,random.NextDouble()*8-4,-random.NextDouble()*12.7);
            var b=new Point3D(random.NextDouble()*8-4,random.NextDouble()*8-4,-random.NextDouble()*12.7);
            double floor=random.NextDouble()*6;
            bool sampled=false;
            for(int i=0;i<=2000;i++)
            {
                double t=i/2000d,x=a.X+(b.X-a.X)*t,y=a.Y+(b.Y-a.Y)*t,d=-(a.Z+(b.Z-a.Z)*t)-floor;
                double qx=Math.Max(0,Math.Max(-.3-x,x-.7)),qy=Math.Max(0,Math.Max(-.6-y,y-.2));
                if(d>0 && Profile.PhysicalRadiusAtDepth(d)>Math.Sqrt(qx*qx+qy*qy)+1e-7) {sampled=true;break;}
            }
            bool exact=VCarveCutterBoxIntersection.Intersects(a,b,Profile,floor,-.3,-.6,.7,.2);
            Assert.Equal(sampled,exact);
            Assert.Equal(exact,VCarveCutterBoxIntersection.Intersects(b,a,Profile,floor,-.3,-.6,.7,.2));
        }
    }
}
