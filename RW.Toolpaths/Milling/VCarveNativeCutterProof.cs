using RW.Toolpaths;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Milling;

// Nomination-time feasibility. The final candidate is replayed independently
// by CubicGuard; sampled poses here can reject, but cannot grant acceptance.
internal static class VCarveNativeCutterProof
{
    internal static bool Certified(ToolMove move,VCarveNativeContainment guard,VCarveCutterProfile profile,out string reason,int maximumDepth=14,int maximumIntervals=65536)
    {
        if(guard.StockAware)
        {
            bool certified=guard.CertifyStockCurve(move);
            reason=certified?"certified-continuous-stock-enclosure":"stock-curve-not-certified";return certified;
        }
        int count=0;string outcome="certified-continuous-enclosure";
        bool Verify(ToolMove m,int depth)
        {
            count++;guard.Token.ThrowIfCancellationRequested();
            if(m.Kind==ToolMoveKind.Line){bool legal=guard.MinimumSquaredSeparation(m.Start,m.End)>=-1e-10;if(!legal)outcome="analytic-line-violation";return legal;}
            double controlMin=Math.Min(Math.Min(m.Start.Z,m.End.Z),Math.Min(m.Control1.Z,m.Control2.Z));
            if(controlMin>=0)return true;
            if(Math.Max(m.Start.Z,m.End.Z)<=0&&Math.Min(m.Start.Z,m.End.Z)<0)
            {
                double margin=guard.MinimumSquaredSeparation(m.Start,m.End);
                var a=VCarveNativeCurveMath.Second(m,0);var b=VCarveNativeCurveMath.Second(m,1);
                double exy=Math.Max(Math.Sqrt(a.X*a.X+a.Y*a.Y),Math.Sqrt(b.X*b.X+b.Y*b.Y))/8;
                double low=profile.PhysicalRadiusAtDepth(Math.Max(0,-Math.Max(m.Start.Z,m.End.Z)));
                double high=profile.PhysicalRadiusAtDepth(Math.Max(0,-Math.Min(m.Start.Z,m.End.Z)));
                double upper=profile.PhysicalRadiusAtDepth(Math.Max(0,-controlMin));
                double inflation=Math.Max(0,upper-low)+exy;
                if(margin>=2*high*inflation+inflation*inflation)return true;
            }
            foreach(double t in new[]{0d,.5,1d})
            {
                var p=VCarveNativeCurveMath.At(m,t);
                if(guard.MinimumSquaredSeparation(p,p)<-1e-10){outcome="actual-cutter-pose-violation";return false;}
            }
            if(depth>=maximumDepth||count>=maximumIntervals){outcome="continuous-proof-budget";return false;}
            var halves=VCarveNativeCurveMath.Split(m);
            return Verify(halves.Left,depth+1)&&Verify(halves.Right,depth+1);
        }
        bool accepted=Verify(move,0);reason=outcome;return accepted;
    }
}
