using RW.Toolpaths;
using RW.Toolpaths.Milling;
namespace RW.Toolpaths.Milling.ReviewedSource;

internal static class VCarveContactCurveCorridor
{
    [Flags] internal enum LockedPorts { None=0,Start=1,End=2 }
    internal sealed record Result(double First,double Last,ToolMove SafeReference);
    internal static Result? Find(ToolMove candidate,LockedPorts ports,Func<ToolMove,bool> fullStock,double maximumDepth,double availableFitBudget)
    {
        // Prefer a complete certified reference when a uniform control lift
        // fits inside the remaining geometric allowance. This avoids chasing
        // adjacent tiny proof cells with unrelated local constraints. It is a
        // candidate construction; the final whole native proof is mandatory.
        ToolMove WholeLift(double d)=>candidate with
        {
            Start=candidate.Start with{Z=candidate.Start.Z+(ports.HasFlag(LockedPorts.Start)?0:d)},
            Control1=candidate.Control1 with{Z=candidate.Control1.Z+d},
            Control2=candidate.Control2 with{Z=candidate.Control2.Z+d},
            End=candidate.End with{Z=candidate.End.Z+(ports.HasFlag(LockedPorts.End)?0:d)}
        };
        double wholeHigh=Math.Min(maximumDepth,Math.Max(0,availableFitBudget));
        if(wholeHigh>0&&fullStock(WholeLift(wholeHigh)))
        {
            double wholeLow=0;
            for(int i=0;i<18;i++){double mid=(wholeLow+wholeHigh)/2;if(fullStock(WholeLift(mid)))wholeHigh=mid;else wholeLow=mid;}
            var safe=WholeLift(wholeHigh);
            if(new[]{safe.Start,safe.Control1,safe.Control2,safe.End}.All(p=>p.Z<=0)&&fullStock(safe))return new(0,1,safe);
        }
        // Child tests locate an unresolved interval only. They NEVER replace
        // the mandatory unchanged proof on the final entire native primitive.
        var piece=candidate;double first=0,last=1;
        for(int i=0;i<6;i++){var split=VCarveNativeCurveMath.Split(piece);double mid=(first+last)/2;if(!fullStock(split.Left)){piece=split.Left;last=mid;}else if(!fullStock(split.Right)){piece=split.Right;first=mid;}else break;}
        bool start=ports.HasFlag(LockedPorts.Start)&&first==0,end=ports.HasFlag(LockedPorts.End)&&last==1;
        ToolMove Lift(double d)=>piece with{Start=piece.Start with{Z=piece.Start.Z+(start?0:d)},Control1=piece.Control1 with{Z=piece.Control1.Z+d},Control2=piece.Control2 with{Z=piece.Control2.Z+d},End=piece.End with{Z=piece.End.Z+(end?0:d)}};
        double low=0,high=maximumDepth;if(!fullStock(Lift(high)))return null;
        for(int i=0;i<18;i++){double mid=(low+high)/2;if(fullStock(Lift(mid)))high=mid;else low=mid;}
        var reference=Lift(high);return fullStock(reference)?new(first,last,reference):null;
    }
}
