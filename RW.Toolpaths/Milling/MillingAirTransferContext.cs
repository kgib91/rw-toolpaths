using Clipper2Lib;

namespace RW.Toolpaths.Milling;

/// <summary>Immutable original-stock geometry for nominating or pricing an
/// above-stock transfer. It contains no executed-stock or command authority.
/// Emission still requires its actual same-region prefix and posted cursor.</summary>
public sealed class MillingAirTransferContext
{
    private readonly StockRectangle rectangle;
    internal float MaterialSurface { get; }
    internal double FullProfileEnvelopeMm { get; }
    private MillingAirTransferContext(StockRectangle rectangle,float materialSurface,double envelope)
    {this.rectangle=rectangle;MaterialSurface=materialSurface;FullProfileEnvelopeMm=envelope;}

    /// <summary>Snapshot a known rectangular board and the complete physical
    /// cutter envelope. Unsupported or missing bounds retain global pricing.</summary>
    public static MillingAirTransferContext? Create(VCarveCutterProfile profile,float materialSurface,
        IReadOnlyList<PointD>? stockBoundary)
    {
        ArgumentNullException.ThrowIfNull(profile);
        if(!float.IsFinite(materialSurface)||StockRectangle.Read(stockBoundary) is not {} rectangle)return null;
        double envelope=profile.Sections.Max(s=>Math.Max(s.BottomRadius,s.TopRadius))+MillingLocalTravel.Clearance;
        if(!double.IsFinite(envelope)||envelope<0)return null;
        return new(rectangle,materialSurface,envelope);
    }

    internal readonly record struct Construction(IReadOnlyList<MillingTransition> Moves,
        IReadOnlyList<MillingTransition> GlobalMoves,float Plane,float NativeReserve,double PostedReserve);

    // Pure common construction. The caller owns execution context, cancellation,
    // pricing scope and policy switches; no synthetic stock session is created.
    internal Construction? TryBuild(Point3D from,Point3D to,Point3D postedFrom,float safeZ,
        Func<Point3D,Point3D,bool>? clearedTravelGuard,out string refusal)
    {
        refusal="";
        if(!float.IsFinite(safeZ)||!Finite(from)||!Finite(to)||!Finite(postedFrom)
            ||Math.Max(from.Z,to.Z)>=safeZ){refusal="invalid-or-safe-plane-endpoint";return null;}
        var a=new Point3D((float)from.X,(float)from.Y,(float)from.Z);
        var b=new Point3D((float)to.X,(float)to.Y,(float)to.Z);
        if(!MillingTransitionPlanner.RequiresPlanarTraverse(a,b)){refusal="not-planar-transfer";return null;}
        if(!rectangle.Contains(a.X,a.Y,FullProfileEnvelopeMm)||!rectangle.Contains(b.X,b.Y,FullProfileEnvelopeMm)
            ||!rectangle.Contains(postedFrom.X,postedFrom.Y,FullProfileEnvelopeMm)
            ||!rectangle.Contains(Q((float)b.X),Q((float)b.Y),FullProfileEnvelopeMm))
            {refusal="full-profile-envelope-outside-stock-boundary";return null;}
        float plane=MathF.Max((float)((double)MaterialSurface+MillingLocalTravel.Clearance+.002),MathF.Max((float)a.Z,(float)b.Z));
        float nativeReserve=plane-MaterialSurface;double postedReserve=Q(nativeReserve);
        if(!float.IsFinite(plane)||plane>=safeZ||nativeReserve<MillingLocalTravel.Clearance+.001||postedReserve<MillingLocalTravel.Clearance)
            {refusal="original-stock-top-reserve-unproved";return null;}
        if(a.Z==plane&&postedFrom.Z<MillingLocalTravel.Clearance)
            {refusal="retained-posted-height-below-air-plane";return null;}

        // The original immutable approach proof and controlled residual descent
        // are retained exactly, including their original motion modes.
        var original=MillingTransitionPlanner.Build(a,b,safeZ,MaterialSurface,clearedTravelGuard,localTravel:null);
        var raisedA=a with{Z=plane};var raisedB=b with{Z=plane};var result=new List<MillingTransition>(original.Count+1);
        if(a.Z<plane)result.Add(new(a,raisedA));
        result.Add(new(raisedA,raisedB,ClearedTravel:true));
        foreach(var move in original){
            if(move.End.Z>=move.Start.Z||move.End.Z>=plane)continue;
            if(move.Start.X!=b.X||move.Start.Y!=b.Y||move.End.X!=b.X||move.End.Y!=b.Y)
                {refusal="unsupported-original-descent-shape";return null;}
            var start=move.Start with{Z=Math.Min(move.Start.Z,plane)};
            if(start.Z>move.End.Z)result.Add(new(start,move.End,move.ClearedTravel));
        }
        if(result[^1].End!=b){refusal="original-approach-endpoint-not-retained";return null;}
        return new(result,original,plane,nativeReserve,postedReserve);
    }

    private readonly record struct StockRectangle(double MinX,double MinY,double MaxX,double MaxY)
    {
        internal bool Contains(double x,double y,double radius)=>double.IsFinite(x+y+radius)
            &&x-radius>=MinX&&x+radius<=MaxX&&y-radius>=MinY&&y+radius<=MaxY;
        internal static StockRectangle? Read(IReadOnlyList<PointD>? boundary){
            if(boundary is null)return null;
            int count=boundary.Count;
            if(count==5&&boundary[0].x==boundary[4].x&&boundary[0].y==boundary[4].y)count--;
            if(count!=4||boundary.Take(count).Any(p=>!double.IsFinite(p.x+p.y)))return null;
            double minX=boundary.Take(count).Min(p=>p.x),maxX=boundary.Take(count).Max(p=>p.x),
                minY=boundary.Take(count).Min(p=>p.y),maxY=boundary.Take(count).Max(p=>p.y);
            if(minX>=maxX||minY>=maxY)return null;
            var corners=new HashSet<(double,double)>();
            for(int i=0;i<count;i++){
                var a=boundary[i];var b=boundary[(i+1)%count];
                if((a.x!=minX&&a.x!=maxX)||(a.y!=minY&&a.y!=maxY)
                    ||(a.x==b.x)==(a.y==b.y)||!corners.Add((a.x,a.y)))return null;
            }
            return new(minX,minY,maxX,maxY);
        }
    }
    private static double Q(float v)=>ToolpathCommandRounding.Round(v);
    private static bool Finite(Point3D p)=>double.IsFinite(p.X)&&double.IsFinite(p.Y)&&double.IsFinite(p.Z);
}
