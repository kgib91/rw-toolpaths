using System.Diagnostics;

namespace RW.Toolpaths.Milling;

// Only the initial source -> Constrain -> artwork Apply -> LimitGraph chain.
// This carries an ordered source shell, no union/coverage/finished-stock credit.
internal static class VCarveInitialSourceChordShell
{
    internal sealed record Result(bool Accepted,string Reason,long SourceChords,long CandidatePoints,
        long ZeroXYChords,int LastPathOrdinal,int LastSourceChordOrdinal,int LastCandidatePointOrdinal,double MaximumNominalXYResidual,double MaximumNominalDepthRoundoff,
        double MaximumActualXYDisplacement,double MaximumActualPositiveDepthDifference,double Seconds);
    internal static Result Check(IReadOnlyList<TaggedToolpath> source,IReadOnlyList<TaggedToolpath> candidate,
        Func<Point3D,Point3D> command,CancellationToken token,long maximumPoints)
    {
        var timer=Stopwatch.StartNew();long sourceChords=0,points=0,zeros=0;
        double nominalResidual=0,depthRoundoff=0,actualXY=0,actualDepth=0;int lastPath=-1,lastEdge=-1,lastPoint=-1;
        Result Finish(bool ok,string reason)=>new(ok,reason,sourceChords,points,zeros,lastPath,lastEdge,lastPoint,nominalResidual,
            depthRoundoff,actualXY,actualDepth,timer.Elapsed.TotalSeconds);
        token.ThrowIfCancellationRequested();
        if(source.Count!=candidate.Count)return Finish(false,"path-ordinal-owner");
        for(int row=0;row<source.Count;row++)
        {
            token.ThrowIfCancellationRequested();lastPath=row;lastEdge=-1;lastPoint=0;var s=source[row];var c=candidate[row];
            if(s.RegionIndex!=c.RegionIndex||s.Category!=c.Category||s.DepthPassIndex!=c.DepthPassIndex
                ||s.VisitIdentity!=c.VisitIdentity)return Finish(false,"path-metadata-owner");
            if(s.Points.Count==0||c.Points.Count==0)
            {if(s.Points.Count!=c.Points.Count)return Finish(false,"empty-path-owner");continue;}
            if(!SameXY(s.Points[0],c.Points[0]))return Finish(false,"first-source-vertex-owner");
            int next=1;
            if(!Point(s.Points[0],s.Points[0],c.Points[0],0,0,out string reason))return Finish(false,reason);
            for(int edge=1;edge<s.Points.Count;edge++)
            {
                sourceChords++;lastEdge=edge-1;var a=s.Points[edge-1];var b=s.Points[edge];
                if(!Finite(a)||!Finite(b))return Finish(false,"nonfinite-source");
                double dx=b.X-a.X,dy=b.Y-a.Y,square=dx*dx+dy*dy;
                if(square==0)
                {
                    zeros++;
                    // Literal zero-XY source edges have no planar interpolation
                    // ambiguity. Consume one preserved original endpoint; any
                    // extra zero children/unknown inventory delegates whole old.
                    if(next>=c.Points.Count||!SameXY(b,c.Points[next]))return Finish(false,"zero-XY-source-endpoint-owner");
                    lastPoint=next;
                    if(!Point(a,b,c.Points[next++],1,0,out reason))return Finish(false,reason);
                    continue;
                }
                if(!double.IsFinite(square))return Finish(false,"source-projection-numeric-domain");
                double length=Math.Sqrt(square),previous=0;bool found=false;
                while(next<c.Points.Count)
                {
                    lastPoint=next;var p=c.Points[next];bool endpoint=SameXY(p,b);
                    double t=endpoint?1:((p.X-a.X)*dx+(p.Y-a.Y)*dy)/square;
                    double arith=ArithmeticAllowance(a,b,p),parameterReserve=arith/length;
                    if(!double.IsFinite(t)||!double.IsFinite(parameterReserve)||parameterReserve>=.25
                        ||t < -parameterReserve||t>1+parameterReserve||t<previous-parameterReserve)
                        return Finish(false,"ordered-finite-source-parameter-owner");
                    t=Math.Clamp(t,0,1);previous=t;
                    double qx=a.X+t*dx,qy=a.Y+t*dy;
                    double ex=p.X-qx,ey=p.Y-qy,residual=Math.Sqrt(ex*ex+ey*ey);
                    nominalResidual=Math.Max(nominalResidual,residual);
                    if(!double.IsFinite(residual)||residual>arith)return Finish(false,"nominal-source-chord-XY-owner");
                    if(!Point(a,b,p,t,parameterReserve,out reason))return Finish(false,reason);
                    next++;
                    if(endpoint){found=true;break;}
                }
                if(!found)return Finish(false,"preserved-source-vertex-order");
            }
            if(next!=c.Points.Count)return Finish(false,"candidate-tail-or-loop-owner");
        }
        token.ThrowIfCancellationRequested();return Finish(true,"ordered-source-chords-depth-raised-with-measured-F3-shell");

        bool Point(Point3D a,Point3D b,Point3D p,double t,double parameterReserve,out string reason)
        {
            reason="";if((++points&255)==0)token.ThrowIfCancellationRequested();
            if(points>maximumPoints){reason="original-point-budget";return false;}
            if(!Finite(p)){reason="nonfinite-candidate";return false;}
            double sourceZ=a.Z+t*(b.Z-a.Z),positive=Math.Max(0,-p.Z)-Math.Max(0,-sourceZ);
            // Two original bounded subdivision stages (28 each), followed by
            // finite projection. 256 ULPs encloses their ordinary interpolation
            // rounding; this is an arithmetic admission, not a stock tolerance.
            double roundoff=ArithmeticAllowance(a,b,p)+Math.Abs(b.Z-a.Z)*parameterReserve;
            depthRoundoff=Math.Max(depthRoundoff,Math.Max(0,positive));
            if(!double.IsFinite(roundoff)||positive>roundoff){reason="nonmonotone-source-depth-owner";return false;}
            var ca=command(a);var cb=command(b);var cp=command(p);
            if(!Finite(ca)||!Finite(cb)||!Finite(cp)){reason="nonfinite-actual-command";return false;}
            double sx=ca.X+t*(cb.X-ca.X),sy=ca.Y+t*(cb.Y-ca.Y),sz=ca.Z+t*(cb.Z-ca.Z);
            double ex=cp.X-sx,ey=cp.Y-sy;
            double displacement=Math.Sqrt(ex*ex+ey*ey),depthDifference=Math.Max(0,sz-cp.Z);
            if(!double.IsFinite(displacement)||!double.IsFinite(depthDifference)){reason="actual-source-shell-numeric-domain";return false;}
            actualXY=Math.Max(actualXY,displacement);
            actualDepth=Math.Max(actualDepth,depthDifference);
            return true;
        }
    }
    private static bool SameXY(Point3D a,Point3D b)=>a.X==b.X&&a.Y==b.Y;
    private static bool Finite(Point3D p)=>double.IsFinite(p.X)&&double.IsFinite(p.Y)&&double.IsFinite(p.Z);
    private static double ArithmeticAllowance(Point3D a,Point3D b,Point3D p)
    {
        double magnitude=Math.Max(1,Math.Max(Math.Max(Math.Abs(a.X),Math.Abs(a.Y)),Math.Abs(a.Z)));
        magnitude=Math.Max(magnitude,Math.Max(Math.Max(Math.Abs(b.X),Math.Abs(b.Y)),Math.Abs(b.Z)));
        magnitude=Math.Max(magnitude,Math.Max(Math.Max(Math.Abs(p.X),Math.Abs(p.Y)),Math.Abs(p.Z)));
        return 256*(Math.BitIncrement(magnitude)-magnitude);
    }
}
