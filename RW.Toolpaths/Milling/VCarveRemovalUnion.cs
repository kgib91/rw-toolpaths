using Clipper2Lib;
using RW.Toolpaths.Geometry;

namespace RW.Toolpaths.Milling;

/// <summary>A conservative union-of-cuts removal certificate. Inputs are
/// stock-relative physical cuts. This class grants NO executed-stock or travel
/// permission. Each accepted XY cell is covered to the deepest target height
/// by one actual donor sweep; different cells may have different donors.</summary>
internal sealed class VCarveRemovalUnion
{
    private readonly record struct Chord(Point3D A,Point3D B,double Error);
    private readonly record struct Box(double X0,double Y0,double X1,double Y1,int Level);
    private readonly VCarveCutterProfile profile;
    private readonly Chord[] chords;
    private readonly CutterSweepStock affine;
    private readonly PolygonOps.BoundarySegmentIndex index;
    private readonly CancellationToken token;
    private readonly double maximumRadius,transportReserve;
    private readonly int maximumCells,maximumCandidates,maximumBoxDepth;
    private const double Reserve=1e-7;
    internal long Queries{get;private set;}
    internal long Accepted{get;private set;}
    internal long Cells{get;private set;}
    internal long BudgetRefusals{get;private set;}
    internal long UnprovedPointRefusals{get;private set;}
    internal long CandidateBudgetRefusals{get;private set;}
    internal long CellBudgetRefusals{get;private set;}
    internal long DepthBudgetRefusals{get;private set;}
    internal int Chords=>chords.Length;
    internal (double X,double Y,double TargetFloor)? LastUnprovedPoint{get;private set;}

    internal VCarveRemovalUnion(VCarveCutterProfile profile,IReadOnlyList<TaggedToolpath> cuts,
        double envelopeError,CancellationToken token,int maximumCells=30_000,int maximumCandidates=8192,double transportReserve=0,int maximumBoxDepth=28)
    {
        if(!(envelopeError>=0)||!double.IsFinite(envelopeError))throw new ArgumentOutOfRangeException(nameof(envelopeError));
        if(!(transportReserve>=0)||!double.IsFinite(transportReserve))throw new ArgumentOutOfRangeException(nameof(transportReserve));
        this.maximumBoxDepth=maximumBoxDepth;this.transportReserve=transportReserve;this.profile=profile;this.token=token;affine=new(profile,p=>p,0,token);this.maximumCells=maximumCells;this.maximumCandidates=maximumCandidates;
        var rows=new List<Chord>();double deepest=0;
        foreach(var path in cuts){
            token.ThrowIfCancellationRequested();
            if(path.Points.Any(p=>!Valid(p)))throw new ArgumentException("Removal donors must be finite stock-relative cuts within the represented profile depth.");
            foreach(var span in path.Spans.Count==0?new[]{new ToolpathSpan(0,path.Points.Count-1,ToolpathSpanKind.Cut)}:path.Spans){
                if(span.Kind!=ToolpathSpanKind.Cut||span.EndIndex<=span.StartIndex)continue;
                for(int i=span.StartIndex+1;i<=span.EndIndex;i++)affine.RecordExecuted(path.Points[i-1],path.Points[i]);
                var input=new TaggedToolpath(path.Points.Skip(span.StartIndex).Take(span.EndIndex-span.StartIndex+1).ToArray(),[],0,"removal-proof",null,detectCurves:false);
                var points=envelopeError>0?SurfaceToolpathSimplifier.Simplify(input,envelopeError,token).Points:input.Points;
                // The finite 3D capsule bound is bidirectional for a connected
                // polyline. Raising the virtual donor and eroding its radius
                // by Error makes its entire sweep a subset of the actual cut.
                for(int i=1;i<points.Count;i++){
                    var a=points[i-1];var b=points[i];if(Math.Min(a.Z,b.Z)>=0)continue;
                    int n=Math.Max(1,(int)Math.Ceiling(Math.Abs(a.Z-b.Z)/.01));
                    for(int j=0;j<n;j++){
                        Point3D At(double t)=>new(a.X+t*(b.X-a.X),a.Y+t*(b.Y-a.Y),a.Z+t*(b.Z-a.Z));
                        var u=j==0?a:At((double)j/n);var v=j+1==n?b:At((double)(j+1)/n);
                        rows.Add(new(u,v,envelopeError+transportReserve));deepest=Math.Max(deepest,-Math.Min(u.Z,v.Z));
                        if(rows.Count>2_000_000)throw new InvalidOperationException("Removal union exceeds the shared proof inventory.");
                    }
                }
            }
        }
        affine.Seal();chords=rows.ToArray();maximumRadius=profile.PhysicalRadiusAtDepth(deepest);
        index=new(new Paths64(chords.Select(c=>new Path64{XY(c.A),XY(c.B)})));
    }

    // First contact, not DepthAtRadius's deepest fitting penetration. Equality
    // at a cylinder's radius belongs to its LOWER end, not the cylinder top.
    private double Height(double radius)
    {
        if(radius<=profile.TipRadius)return 0;
        double height=0;
        foreach(var section in profile.Sections){
            double active=Math.Min(section.Height,profile.MaximumDepth-height);if(active<=0)break;
            if(radius<=section.BottomRadius)return height;
            double top=section.Kind==CarvingCutterSectionKind.Hemisphere
                ?Math.Sqrt(active*(2*section.TopRadius-active))
                :section.BottomRadius+(section.TopRadius-section.BottomRadius)*active/section.Height;
            if(radius<=top){
                if(section.Kind==CarvingCutterSectionKind.Hemisphere)
                    return height+radius*radius/(section.TopRadius+Math.Sqrt(Math.Max(0,section.TopRadius*section.TopRadius-radius*radius)));
                return height+(radius-section.BottomRadius)*section.Height/(section.TopRadius-section.BottomRadius);
            }
            height+=active;
        }
        return double.PositiveInfinity;
    }
    private bool Valid(Point3D p)=>double.IsFinite(p.X+p.Y+p.Z)&&Math.Abs(p.X)<=10000&&Math.Abs(p.Y)<=10000&&p.Z>=-profile.MaximumDepth-Reserve&&p.Z<=100;
    private static Point64 XY(Point3D p)=>new((long)Math.Round(p.X*PathUtils.Scale),(long)Math.Round(p.Y*PathUtils.Scale));
    private static double Distance(double x,double y,Point3D a,Point3D b)
    {
        double dx=b.X-a.X,dy=b.Y-a.Y,den=dx*dx+dy*dy;
        double t=den>0?Math.Clamp(((x-a.X)*dx+(y-a.Y)*dy)/den,0,1):0;
        return Math.Sqrt(Math.Pow(x-a.X-t*dx,2)+Math.Pow(y-a.Y-t*dy,2));
    }
    internal bool ContainsPath(TaggedToolpath target,double tolerance,double envelopeError=.001,double targetTransportReserve=0)
    {
        if(!(tolerance>0)||!double.IsFinite(tolerance)||!(envelopeError>=0)||!double.IsFinite(envelopeError))throw new ArgumentOutOfRangeException(nameof(tolerance));
        if(!(targetTransportReserve>=0)||!double.IsFinite(targetTransportReserve))throw new ArgumentOutOfRangeException(nameof(targetTransportReserve));
        double allowance=tolerance/Math.Sqrt(2)-envelopeError-targetTransportReserve;
        if(allowance<=0)return false;
        var original=new TaggedToolpath(target.Points,[],0,"removal-target",null,detectCurves:false);
        var points=envelopeError>0?SurfaceToolpathSimplifier.Simplify(original,envelopeError,token).Points:original.Points;
        // A lower tip and wider radius bound the entire original target trace.
        // Consume that envelope from BOTH existing allowance components.
        for(int i=1;i<points.Count;i++){
            var a=points[i-1];var b=points[i];int n=Math.Max(1,(int)Math.Ceiling(Math.Abs(a.Z-b.Z)/.01));
            Point3D At(double t)=>new(a.X+t*(b.X-a.X),a.Y+t*(b.Y-a.Y),a.Z+t*(b.Z-a.Z));
            for(int j=0;j<n;j++)if(!Contains(j==0?a:At((double)j/n),j+1==n?b:At((double)(j+1)/n),allowance,allowance))return false;
        }
        return true;
    }
    internal bool Contains(Point3D a,Point3D b,double verticalAllowance,double radialAllowance)
    {
        token.ThrowIfCancellationRequested();Queries++;LastUnprovedPoint=null;
        if(!Valid(a)||!Valid(b)||verticalAllowance<0||radialAllowance<0||!double.IsFinite(verticalAllowance+radialAllowance))return false;
        if(verticalAllowance>=transportReserve&&affine.Contains(a,b,verticalAllowance-transportReserve,-radialAllowance+transportReserve)){Accepted++;return true;}
        double z=Math.Min(a.Z,b.Z)+verticalAllowance;
        if(z>=Reserve){Accepted++;return true;}
        double radius=profile.PhysicalRadiusAtDepth(-z)-radialAllowance;
        if(radius<0){Accepted++;return true;}
        var initial=new Box(Math.Min(a.X,b.X)-radius,Math.Min(a.Y,b.Y)-radius,Math.Max(a.X,b.X)+radius,Math.Max(a.Y,b.Y)+radius,0);
        double expand=maximumRadius+2/PathUtils.Scale;var matches=new List<int>();
        if(!index.QueryBounds((long)Math.Floor((initial.X0-expand)*PathUtils.Scale),(long)Math.Floor((initial.Y0-expand)*PathUtils.Scale),
            (long)Math.Ceiling((initial.X1+expand)*PathUtils.Scale),(long)Math.Ceiling((initial.Y1+expand)*PathUtils.Scale),matches,2*maximumCandidates,token))
        {BudgetRefusals++;CandidateBudgetRefusals++;return false;}
        var candidates=matches.Select(i=>i/2).Distinct().ToArray();if(candidates.Length>maximumCandidates){BudgetRefusals++;CandidateBudgetRefusals++;return false;}
        int witness=-1;
        double Donor(Chord c,Box box){
            double d=Math.Max(Math.Max(Distance(box.X0,box.Y0,c.A,c.B),Distance(box.X0,box.Y1,c.A,c.B)),
                Math.Max(Distance(box.X1,box.Y0,c.A,c.B),Distance(box.X1,box.Y1,c.A,c.B)));
            // Distance to a convex segment is convex, so its maximum over a
            // rectangle occurs at a corner. Max endpoint Z is a conservative
            // upper floor for the complete affine donor chord.
            return Math.Min(0,Math.Max(c.A.Z,c.B.Z)+c.Error+Height(d+c.Error)+Reserve);
        }
        bool Covered(Box box,double targetFloor){
            if(witness>=0&&Donor(chords[witness],box)<=targetFloor)return true;
            foreach(int i in candidates)if(Donor(chords[i],box)<=targetFloor){witness=i;return true;}
            return false;
        }
        var pending=new Stack<Box>();pending.Push(initial);int cells=0;
        while(pending.TryPop(out var box)){
            if((++cells&127)==0)token.ThrowIfCancellationRequested();Cells++;
            if(cells>maximumCells){BudgetRefusals++;CellBudgetRefusals++;return false;}
            if(box.Level>maximumBoxDepth){BudgetRefusals++;DepthBudgetRefusals++;return false;}
            double x=(box.X0+box.X1)/2,y=(box.Y0+box.Y1)/2;
            double half=Math.Sqrt(Math.Pow(box.X1-box.X0,2)+Math.Pow(box.Y1-box.Y0,2))/2;
            double distance=Distance(x,y,a,b);
            double floor=z+Height(Math.Max(0,distance-half)+radialAllowance)-Reserve;
            if(floor>=0||Covered(box,floor))continue;
            double centre=z+Height(distance+radialAllowance)-Reserve;
            if(centre<0&&!Covered(new(x,y,x,y,box.Level),centre)){LastUnprovedPoint=(x,y,centre);UnprovedPointRefusals++;return false;}
            if(box.X1-box.X0>=box.Y1-box.Y0){pending.Push(new(x,box.Y0,box.X1,box.Y1,box.Level+1));pending.Push(new(box.X0,box.Y0,x,box.Y1,box.Level+1));}
            else{pending.Push(new(box.X0,y,box.X1,box.Y1,box.Level+1));pending.Push(new(box.X0,box.Y0,box.X1,y,box.Level+1));}
        }
        Accepted++;return true;
    }
}
