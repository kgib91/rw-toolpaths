using Clipper2Lib;
using RW.Toolpaths.Geometry;
using Xunit;

namespace RW.Toolpaths.Tests;

public sealed class SignedOffsetClusterTests
{
    [Theory]
    [InlineData(.00318310546875,.00025)]
    [InlineData(.0023837890625,.00025)]
    [InlineData(.007,1d/32768)]
    public void SeparatedContourClustersKeepTheLiteralBoundaryInsideItsIntegerReserve(double radius,double arc)
    {
        var source=Source();
        Exact(source,radius,arc);
        // Reversed enumeration and overlapping padded boxes retain their joint
        // signed context, including a positive island inside a negative hole.
        source.Reverse();Exact(source,radius,arc);
        source.Add(Ring(0,0,1,700,false));Exact(source,radius,arc);
        source.Add(Ring(.002,0,3,700,true));Exact(source,radius,arc);
    }

    [Fact]
    public void ChangedContourAndDetachedResultCannotReuseAnOldOffset()
    {
        var source=Source();double radius=.00318310546875,arc=.00025;
        var first=PolygonOps.Buffer(source,radius,arc);first[0][0]=new(999,999);
        Exact(source,radius,arc);
        source[5]=source[5].Select((p,i)=>i==10?new PointD(p.x+.02,p.y):p).ToList();
        Exact(source,radius,arc);
        // No enclosing rectangle: admission must keep the complete builder.
        source[0]=Ring(0,0,150,100,false);Exact(source,radius,arc);
    }

    private static List<List<PointD>> Source()
    {
        var paths=new List<List<PointD>>{new(){new(-100,-100),new(100,-100),new(100,100),new(-100,100)}};
        for(int y=-2;y<=2;y++)for(int x=-2;x<=2;x++)paths.Add(Ring(x*20,y*20,3,420,true));
        return paths;
    }

    private static List<PointD> Ring(double x,double y,double radius,int count,bool hole)
    {
        var ring=Enumerable.Range(0,count).Select(i=>{
            double theta=2*Math.PI*i/count,r=radius*(1+.18*Math.Cos(7*theta));
            return new PointD(x+r*Math.Cos(theta),y+r*Math.Sin(theta));
        }).ToList();
        if(hole)ring.Reverse();return ring;
    }

    private static void Exact(List<List<PointD>> source,double radius,double arc)
    {
        var expected=PolygonOps.BufferWithLiteralSignedOffset(source,radius,arc);
        var actual=PolygonOps.Buffer(source,radius,arc);
        var difference=Clipper.Xor(PathUtils.ToClipper(expected),PathUtils.ToClipper(actual),FillRule.NonZero);
        Console.WriteLine($"[SignedOffset.Reference] radius={radius:R} sourcePaths={source.Count} expectedPoints={expected.Sum(p=>p.Count)} "
            +$"actualPoints={actual.Sum(p=>p.Count)} xorPaths={difference.Count} xorArea={Clipper.Area(difference)/PathUtils.Scale/PathUtils.Scale:R}");
        Assert.Equal(expected.Count,actual.Count);
        double tickBound=4,maximum=0;
        void Certify(List<List<PointD>> from,List<List<PointD>> to)
        {
            var index=new PolygonOps.BoundarySegmentIndex(PathUtils.ToClipper(to));
            double Distance(PointD p,Point64 a,Point64 b)
            {
                double dx=b.X-a.X,dy=b.Y-a.Y,wx=p.x-a.X,wy=p.y-a.Y;
                double square=dx*dx+dy*dy,t=square==0?0:Math.Clamp((wx*dx+wy*dy)/square,0,1);
                return Math.Sqrt(Math.Pow(wx-t*dx,2)+Math.Pow(wy-t*dy,2));
            }
            void Edge(PointD a,PointD b,int depth)
            {
                var middle=new PointD((a.x+b.x)/2,(a.y+b.y)/2);
                var site=index.NearestSegmentForCertificate(middle.x,middle.y);
                // Distance to one finite segment is convex: both endpoints
                // inside its capsule certify the COMPLETE source interval.
                double bound=Math.Max(Distance(a,site.Start,site.End),Distance(b,site.Start,site.End))+1e-6;
                if(bound<=tickBound){maximum=Math.Max(maximum,bound);return;}
                Assert.True(depth<20,$"Unproved boundary interval at {middle.x:R},{middle.y:R}; boundTicks={bound:R}");
                Edge(a,middle,depth+1);Edge(middle,b,depth+1);
            }
            foreach(var path in PathUtils.ToClipper(from))
                for(int i=0;i<path.Count;i++)Edge(new(path[i].X,path[i].Y),new(path[(i+1)%path.Count].X,path[(i+1)%path.Count].Y),0);
        }
        Certify(expected,actual);Certify(actual,expected);
        Assert.Equal(expected.Count(p=>PolygonOps.SignedArea(p)>0),actual.Count(p=>PolygonOps.SignedArea(p)>0));
        Assert.True(tickBound/PathUtils.Scale<PreparedRegionDistance.Uncertainty);
        Console.WriteLine($"[SignedOffset.BoundaryCertificate] maximumTicks={maximum:R} boundTicks={tickBound:R} "
            +$"existingDistanceReserve={PreparedRegionDistance.Uncertainty:R} bothDirections=True completeFiniteEdgeCapsules=True originalContourSignsAndCounts=True");
    }
}
