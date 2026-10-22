using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class DensePlateauRasterTests
{
    [Theory]
    [InlineData(1d)]
    [InlineData(3.175d)]
    public void DenseProtectedRoughingCoversTheFlatFloorAndKeepsTheWholeCutterOutsideTheRamp(double radius)
    {
        var protectedRing=Enumerable.Range(0,20001).Select(i=>{
            double angle=i*2*Math.PI/20001;return new PointD(25+5*Math.Cos(angle),20+5*Math.Sin(angle));
        }).ToArray();
        var source=new MillingRegion(new PointD[]{new(0,0),new(50,0),new(50,40),new(0,40)});
        var profile=new VCarveCutterProfile(new[]{new CarvingCutterSection(CarvingCutterSectionKind.Linear,radius,radius,10)},10);
        var plan=PocketToolpaths.Generate(new[]{source},new ToolGeometry((double)(float)radius){VCarveProfile=profile},new DepthSchedule(2,.75),new PocketOptions {
            SmoothRoughing=true,ProtectedBoundary=new[]{protectedRing},StockToLeave=.2,StepOver=.55,
            DetectCurves=false,CalculateSweptArea=false,Ramp=RampSettings.FromRatio(0)});
        Assert.NotEmpty(plan.Toolpaths);
        var final=new List<(Point3D,Point3D)>();
        foreach(var path in plan.Toolpaths)
        for(int i=1;i<path.Points.Count;i++) {
            var a=path.Points[i-1];var b=path.Points[i];
            if(Math.Min(a.Z,b.Z)>=0)continue;
            Assert.True(Distance(25,20,a,b)>=5+radius+.2-.006,"The full cutter entered protected ramp stock.");
            if(Math.Abs(a.Z+2)<1e-9&&Math.Abs(b.Z+2)<1e-9)final.Add((a,b));
        }
        int checkedFloor=0;
        for(double y=2;y<39;y+=2)for(double x=2;x<49;x+=2) {
            if(Math.Sqrt((x-25)*(x-25)+(y-20)*(y-20))<5.22)continue;
            Assert.True(final.Any(c=>Distance(x,y,c.Item1,c.Item2)<=radius+1e-6),$"Uncleared flat floor at ({x},{y}).");checkedFloor++;
        }
        Assert.True(checkedFloor>300);
        double transfers=plan.Toolpaths.Zip(plan.Toolpaths.Skip(1)).Sum(pair=>{
            var a=pair.First.Points[^1];var b=pair.Second.Points[0];return Math.Sqrt((a.X-b.X)*(a.X-b.X)+(a.Y-b.Y)*(a.Y-b.Y));});
        Assert.True(transfers<900,$"Excessive inter-visit XY travel: {transfers:R} mm.");
    }

    private static double Distance(double x,double y,Point3D a,Point3D b)
    {
        double dx=b.X-a.X,dy=b.Y-a.Y,l=dx*dx+dy*dy;
        double t=l>0?Math.Clamp(((x-a.X)*dx+(y-a.Y)*dy)/l,0,1):0;
        return Math.Sqrt(Math.Pow(x-a.X-t*dx,2)+Math.Pow(y-a.Y-t*dy,2));
    }
}
