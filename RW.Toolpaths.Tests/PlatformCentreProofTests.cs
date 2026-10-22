using Clipper2Lib;
using RW.Toolpaths;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class PlatformCentreProofTests
{
    [Theory]
    [InlineData(0)] [InlineData(.005)] [InlineData(5)]
    public void ProtectedWallBandRetainsEveryOriginalErodedDomainWall(double holeY)
    {
        PointD[] Box(double x,double y,double w,double h)=>new PointD[]{new(x,y),new(x+w,y),new(x+w,y+h),new(x,y+h)};
        var part=Box(0,0,30,20);var hole=Box(10,holeY,10,10);
        var access=new MillingBoundaryAccess(Array.Empty<MillingRegion>(),part)
            .Resolve(new[]{new MillingRegion(part,new[]{hole})},ToolGeometry.Flat(1),new(3,1),true,30,0,default);
        var source=new IReadOnlyList<PointD>[]{part,hole};
        var physical=PolygonOps.Union(access.Regions.SelectMany(r=>r.AsRingGroup()));
        var interior=PolygonOps.Offset(physical,-2*VCarvePreparation.GeometricErrorBound,.0001,0);
        var original=CheckedOpenClip.Require(source.Select(p=>(IReadOnlyList<PointD>)p.Append(p[0]).ToArray()),
            interior,ClipType.Difference,default,"wall-band-reference");
        var current=access.RetainedRampEdges(source,default);
        foreach(var path in original)for(int i=1;i<path.Count;i++)
        foreach(double t in new[]{0d,.25,.5,.75,1d})
        {
            var p=new PointD(path[i-1].x+(path[i].x-path[i-1].x)*t,path[i-1].y+(path[i].y-path[i-1].y)*t);
            Assert.Contains(current,edge=>Distance(p,edge.A,edge.B)<=2/PathUtils.Scale);
        }
        static double Distance(PointD p,PointD a,PointD b)
        {
            double dx=b.x-a.x,dy=b.y-a.y,den=dx*dx+dy*dy;
            double t=den==0?0:Math.Clamp(((p.x-a.x)*dx+(p.y-a.y)*dy)/den,0,1);
            return Math.Sqrt(Math.Pow(p.x-a.x-t*dx,2)+Math.Pow(p.y-a.y-t*dy,2));
        }
    }

    [Fact]
    public void RectangularCentreProofAgreesWithTheOriginalIntegerClipIncludingBoundaryContact()
    {
        var rectangle=new PointD[]{new(0,0),new(20,0),new(20,10),new(0,10)};
        var access=new MillingBoundaryAccess(Array.Empty<MillingRegion>(),rectangle)
            .Resolve(new[]{new MillingRegion(rectangle)},ToolGeometry.Flat(1),new(1,1),false,30,0,default);
        var reference=new PolygonOps.BoundarySegmentIndex(PathUtils.ToClipper(new[]{rectangle}));
        var points=new Point3D[]{new(0,0,-1),new(20,10,-1),new(10,5,-1),new(-.001,5,-1),new(20.001,5,-1),new(0,10,-1)};
        Point64 Native(Point3D p)=>new((long)Math.Round(p.X*PathUtils.Scale),(long)Math.Round(p.Y*PathUtils.Scale));
        foreach(var a in points)foreach(var b in points)
        {
            var intervals=reference.ContactIntervals(Native(a),Native(b),0,default,out _);
            Assert.Equal(intervals.Count==1&&intervals[0].Start==0&&intervals[0].End==1,access.CentreChord(a,b));
        }
    }
}
