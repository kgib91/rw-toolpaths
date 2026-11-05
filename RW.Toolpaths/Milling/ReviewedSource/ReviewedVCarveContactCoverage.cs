using Clipper2Lib;
namespace RW.Toolpaths.Milling.ReviewedSource;

/// <summary>Complete operation source sweeps for contact nomination. A literal
/// cutter-pose witness can disprove a local-only missing-contact claim. This
/// index never grants stock clearance, final native coverage or execution.</summary>
public sealed class ReviewedVCarveContactCoverage
{
    readonly Dictionary<(int X,int Y),List<(Point3D A,Point3D B)>> cells=[];
    readonly VCarveCutterProfile profile;readonly double width,nominationAllowance;
    public ReviewedVCarveContactCoverage(IEnumerable<IReadOnlyList<Point3D>> reference,VCarveCutterProfile profile,double nominationAllowance)
    {
        this.profile=profile;this.nominationAllowance=nominationAllowance;width=profile.PhysicalRadiusAtDepth(profile.MaximumDepth)+nominationAllowance;
        if(!double.IsFinite(width)||width<=0)throw new ArgumentOutOfRangeException(nameof(nominationAllowance));
        foreach(var path in reference)for(int i=1;i<path.Count;i++)
        {
            var a=path[i-1];var b=path[i];
            int left=(int)Math.Floor((Math.Min(a.X,b.X)-width)/width),right=(int)Math.Floor((Math.Max(a.X,b.X)+width)/width),
                bottom=(int)Math.Floor((Math.Min(a.Y,b.Y)-width)/width),top=(int)Math.Floor((Math.Max(a.Y,b.Y)+width)/width);
            for(int x=left;x<=right;x++)for(int y=bottom;y<=top;y++)
            {if(!cells.TryGetValue((x,y),out var row))cells[(x,y)]=row=[];row.Add((a,b));}
        }
    }
    public double WitnessGapUpper(PointD q,double materialDepth)
    {
        double result=double.PositiveInfinity;int remaining=65536;
        if(!cells.TryGetValue(((int)Math.Floor(q.x/width),(int)Math.Floor(q.y/width)),out var chords))return result;
        foreach(var (a,b) in chords)
        {
            double dx=b.X-a.X,dy=b.Y-a.Y,den=dx*dx+dy*dy;
            double t=den==0?0:Math.Clamp(((q.x-a.X)*dx+(q.y-a.Y)*dy)/den,0,1);
            void Try(double u)
            {
                var p=new Point3D(a.X+u*dx,a.Y+u*dy,a.Z+u*(b.Z-a.Z));
                double h=-p.Z-materialDepth;if(h<0)return;
                result=Math.Min(result,Math.Sqrt(Math.Pow(q.x-p.X,2)+Math.Pow(q.y-p.Y,2))-profile.PhysicalRadiusAtDepth(h));
            }
            Try(t);Try(0);Try(1);Try(.25);Try(.5);Try(.75);
            if(a.Z!=b.Z)
            {
                double height=0;
                foreach(var section in profile.Sections)
                {height+=section.Height;double u=(-materialDepth-height-a.Z)/(b.Z-a.Z);if(u>0&&u<1)Try(u);}
            }
            // A missed sparse witness is not evidence of missing material.
            // Refine complete source-chord intervals whose conservative radial
            // bound could contain a witness at the configured resolution.
            void Search(double low,double high,int level)
            {
                if(result<=nominationAllowance||remaining--<=0)return;
                var x=VCarveNativeCurveMath.Lerp(a,b,low);var y=VCarveNativeCurveMath.Lerp(a,b,high);
                double h=Math.Max(-x.Z,-y.Z)-materialDepth;if(h<0)return;
                double distance=SurfaceToolpathSimplifier.DistanceSquared(new(q.x,q.y,0),new(x.X,x.Y,0),new(y.X,y.Y,0));
                double lower=Math.Sqrt(distance)-profile.PhysicalRadiusAtDepth(h);
                if(lower>nominationAllowance)return;
                double mid=(low+high)/2;Try(mid);
                if(level>=14||result<=nominationAllowance)return;
                Search(low,mid,level+1);Search(mid,high,level+1);
            }
            if(result>nominationAllowance&&den>0)Search(0,1,0);
            if(result<=nominationAllowance)return result;
        }
        return result;
    }
}
