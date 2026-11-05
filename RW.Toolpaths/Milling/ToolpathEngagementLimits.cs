namespace RW.Toolpaths.Milling;

/// <summary>Bounds a new cutting transition from a genuinely executed pose.
/// Geometry/stock adapters separately prove the complete cutter corridor.</summary>
internal static class ToolpathEngagementLimits
{
    internal static bool Allows(Point3D from,Point3D to,double radialStep,double axialStep,
        double grade,Func<Point3D,Point3D> command,Point3D? clearedTo=null)
        =>Check(from,to,radialStep,axialStep,grade,clearedTo)
            &&Check(command(from),command(to),radialStep,axialStep,grade,clearedTo is {} p?command(p):null);

    private static bool Check(Point3D a,Point3D b,double radialStep,double axialStep,double grade,Point3D? clearedTo)
    {
        double dx=b.X-a.X,dy=b.Y-a.Y,run2=dx*dx+dy*dy,rise=Math.Abs(b.Z-a.Z);
        double lateral2=run2,drop=a.Z-b.Z;
        if(clearedTo is {} guide)
        {
            double gx=guide.X-a.X,gy=guide.Y-a.Y,length2=gx*gx+gy*gy;
            if(!(length2>0))return false;
            double t=(dx*gx+dy*gy)/length2;if(t<0||t>1)return false;
            lateral2=Math.Pow(dx-t*gx,2)+Math.Pow(dy-t*gy,2);
            drop=a.Z+t*(guide.Z-a.Z)-b.Z;
        }
        return double.IsFinite(run2+rise+lateral2+drop+radialStep+axialStep+grade)&&radialStep>0&&axialStep>0
            &&lateral2<=radialStep*radialStep+1e-12&&drop<=axialStep+1e-12
            &&rise*rise<=grade*grade*run2+1e-12;
    }
}
