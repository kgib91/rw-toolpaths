using System.Diagnostics;

namespace RW.Toolpaths.Milling.Acceleration;

// Untrusted center nomination only. The callbacks bind the ORIGINAL immutable
// floor and artwork. Unknown/corner/escape/nonconvergence retains the old point.
internal static class AdaptiveInitialReachContourNominator
{
    internal const string MarkerCategory = "clearing-inner-reach-nomination";
    private const int MaximumTrials = 6, MaximumMarkedPoints = 500_000;
    // One machine XY tick is a NOM stopping condition, never physical/stock credit.
    private const double NominalReachStop = .001;
    private sealed class Counts
    {
        internal long Paths, Points, Trials, Caps, Radii, Distances, Moved, Kept, Corners, Unknown, Escapes, Nonconvergent;
        internal double MaxShift, MaxAcceptedResidual, MaxInitialResidual;
    }
    internal static List<TaggedToolpath> NormalizeMarkers(IReadOnlyList<TaggedToolpath> source,
        CancellationToken token = default)
    {
        var result = new List<TaggedToolpath>(source.Count);
        foreach (var path in source)
        {
            token.ThrowIfCancellationRequested();
            result.Add(path.Category == MarkerCategory ? path with { Category = "clearing" } : path);
        }
        return result;
    }
    // floorCap(x,y) is FloorField.CappedDepth(halfNominalDepth,x,y).
    // artworkDistance returns NaN outside/unresolved and original inside distance.
    // commandedRadius(x,y,cappedDepth) uses the exact existing CommandPoint and
    // unchanged profile.PhysicalRadiusAtDepth; no Nom radius supplies proof.
    internal static List<TaggedToolpath> Apply(IReadOnlyList<TaggedToolpath> source,
        VCarveCutterProfile profile, double halfNominalDepth, double authoredWidth, double fullRadius,
        Func<double,double,double> floorCap, Func<double,double,double> artworkDistance,
        Func<double,double,double,double> commandedRadius, CancellationToken token,
        Action<string>? diagnostic = null)
    {
        ArgumentNullException.ThrowIfNull(source); ArgumentNullException.ThrowIfNull(profile);
        ArgumentNullException.ThrowIfNull(floorCap); ArgumentNullException.ThrowIfNull(artworkDistance);
        ArgumentNullException.ThrowIfNull(commandedRadius); token.ThrowIfCancellationRequested();
        long started = Stopwatch.GetTimestamp(); var c = new Counts();
        long marked = source.Where(p => p.Category == MarkerCategory).Sum(p => (long)p.Points.Count);
        double reach = authoredWidth + fullRadius;
        bool eligible = marked <= MaximumMarkedPoints && profile.Sections.Count is > 0 and <= 64
            && double.IsFinite(halfNominalDepth) && halfNominalDepth > 0 && halfNominalDepth <= profile.MaximumDepth
            && double.IsFinite(authoredWidth) && authoredWidth > 0 && authoredWidth <= 100
            && double.IsFinite(fullRadius) && fullRadius > 0 && fullRadius <= 100 && double.IsFinite(reach);
        if (!eligible)
        {
            var literal = NormalizeMarkers(source,token);
            diagnostic?.Invoke($"stage=adaptive-initial-reach event=complete eligible=False reason=bounded-input-domain markedPoints={marked} originalPointFallback=True physicalCredit=False stockCredit=False");
            return literal;
        }
        var result = new List<TaggedToolpath>(source.Count);
        foreach (var path in source)
        {
            token.ThrowIfCancellationRequested();
            if (path.Category != MarkerCategory) { result.Add(path); continue; }
            c.Paths++; c.Points += path.Points.Count;
            bool closed = path.Points.Count >= 4 && path.Points[0] == path.Points[^1];
            int count = closed ? path.Points.Count - 1 : path.Points.Count;
            if (!closed || path.Spans.Count != 0)
            { c.Corners += count; c.Kept += count; result.Add(path with { Category = "clearing" }); continue; }
            var output = new List<Point3D>(path.Points.Count);
            for (int i = 0; i < count; i++)
            {
                token.ThrowIfCancellationRequested(); var old = path.Points[i];
                if (old.Z != -halfNominalDepth || !Normal(path.Points,count,i,out double nx,out double ny,out double cosine))
                { c.Corners++; c.Kept++; output.Add(old); continue; }
                Point3D candidate = Move(old,nx,ny,cosine,halfNominalDepth,reach,fullRadius,
                    floorCap,artworkDistance,commandedRadius,token,c);
                output.Add(candidate);
            }
            output.Add(output[0]);
            result.Add(path with { Points = output, Category = "clearing" });
        }
        token.ThrowIfCancellationRequested();
        diagnostic?.Invoke($"stage=adaptive-initial-reach event=complete eligible=True paths={c.Paths} markedPoints={c.Points} " +
            $"trials={c.Trials} capQueries={c.Caps} physicalRadiusQueries={c.Radii} artworkDistanceQueries={c.Distances} " +
            $"movedPoints={c.Moved} retainedPoints={c.Kept} cornerFallbacks={c.Corners} unknownFallbacks={c.Unknown} " +
            $"escapeFallbacks={c.Escapes} nonconvergentFallbacks={c.Nonconvergent} maximumShift={c.MaxShift:R} " +
            $"maximumInitialReachResidual={c.MaxInitialResidual:R} maximumAcceptedReachResidual={c.MaxAcceptedResidual:R} " +
            $"authoredWidth={authoredWidth:R} fullRadius={fullRadius:R} targetInnerReach={reach:R} nominalDepth={halfNominalDepth:R} " +
            $"maximumTrials={MaximumTrials} nominalReachStop={NominalReachStop:R} elapsedMs={Stopwatch.GetElapsedTime(started).TotalMilliseconds:F3} " +
            "acceptedZ=originalCappedDepth allSignedTinyRingsRetained=True markerNormalizedBeforeRouting=True " +
            "originalFloorSourceUnchanged=True originalFloorArtworkGradeRequired=True physicalCredit=False stockCredit=False");
        token.ThrowIfCancellationRequested(); return result;
    }
    private static bool Normal(IReadOnlyList<Point3D> p,int count,int i,out double nx,out double ny,out double cosine)
    {
        nx=ny=cosine=0; Point3D a=p[i],before=a,after=a;
        for(int k=1;k<=Math.Min(8,count-1);k++) { before=p[(i-k+count)%count]; if(before.X!=a.X||before.Y!=a.Y)break; }
        for(int k=1;k<=Math.Min(8,count-1);k++) { after=p[(i+k)%count]; if(after.X!=a.X||after.Y!=a.Y)break; }
        double ax=a.X-before.X,ay=a.Y-before.Y,bx=after.X-a.X,by=after.Y-a.Y;
        double la=Math.Sqrt(ax*ax+ay*ay),lb=Math.Sqrt(bx*bx+by*by);
        if(!double.IsFinite(la)||!double.IsFinite(lb)||la<=1e-12||lb<=1e-12)return false;
        double ux=-ay/la,uy=ax/la,vx=-by/lb,vy=bx/lb;
        double sx=ux+vx,sy=uy+vy,length=Math.Sqrt(sx*sx+sy*sy);
        if(!double.IsFinite(length)||length<=1e-12)return false;
        nx=sx/length;ny=sy/length;cosine=Math.Min(nx*ux+ny*uy,nx*vx+ny*vy);
        return double.IsFinite(cosine)&&cosine>=.5; // miter at most2; NOM only.
    }
    private static Point3D Move(Point3D old,double nx,double ny,double cosine,double requested,
        double reach,double maximumShift,Func<double,double,double> cap,
        Func<double,double,double> distance,Func<double,double,double,double> radius,
        CancellationToken token,Counts c)
    {
        double shift=0,low=double.NaN,high=double.NaN,lowGap=0,highGap=0;
        for(int trial=0;trial<MaximumTrials;trial++)
        {
            token.ThrowIfCancellationRequested(); c.Trials++;
            double x=old.X+nx*shift,y=old.Y+ny*shift;
            if(!double.IsFinite(x)||!double.IsFinite(y)||Math.Abs(x)>1e6||Math.Abs(y)>1e6)
            {c.Unknown++;c.Kept++;return old;}
            c.Distances++; double d=distance(x,y);
            if(!double.IsFinite(d)||d<0) {c.Unknown++;c.Kept++;return old;}
            if(d>reach) {c.Escapes++;c.Kept++;return old;}
            c.Caps++;double depth=cap(x,y);
            if(!double.IsFinite(depth)||depth<0||depth>requested) {c.Unknown++;c.Kept++;return old;}
            c.Radii++;double r=radius(x,y,depth);
            if(!double.IsFinite(r)||r<0||r>maximumShift) {c.Unknown++;c.Kept++;return old;}
            double gap=reach-(d+r);
            if(!double.IsFinite(gap)) {c.Unknown++;c.Kept++;return old;}
            if(trial==0)c.MaxInitialResidual=Math.Max(c.MaxInitialResidual,Math.Abs(gap));
            if(Math.Abs(gap)<=NominalReachStop)
            {
                var accepted=new Point3D(x,y,-depth);c.Moved++;c.MaxShift=Math.Max(c.MaxShift,shift);
                c.MaxAcceptedResidual=Math.Max(c.MaxAcceptedResidual,Math.Abs(gap));return accepted;
            }
            if(gap>0) {low=shift;lowGap=gap;} else {high=shift;highGap=gap;}
            double next=double.IsFinite(low)&&double.IsFinite(high)&&high>low
                ? (low*(-highGap)+high*lowGap)/(lowGap-highGap) : shift+gap/cosine;
            next=Math.Clamp(next,0,maximumShift);
            if(!double.IsFinite(next)||next==shift)break;
            shift=next;
        }
        c.Nonconvergent++;c.Kept++;return old;
    }
}
