using System.Globalization;

namespace RW.Toolpaths.Milling;

/// <summary>V-carve geometry/stock adapter for the common continuation policy.
/// Keeps all scheduled Cut traces; only their entry is replanned from proved air.</summary>
internal sealed class VCarveContinuationAdapter
{
    private readonly ToolpathContinuationPlanner planner;
    private readonly VCarveCutterProfile profile;
    private readonly VCarveOptions options;
    private readonly VCarveRoughingChord[] roughing;
    private readonly DepthSchedule depth;
    private readonly Func<Point3D, Point3D, bool>? originalCutGuard;
    private readonly CancellationToken token;
    private readonly Action<string>? log;
    private readonly double distance, grade, step;
    private readonly double[] airDepths;
    private ClearedColumnStock? prior;
    private long airLinks, freshLinks, guardQueries,stockProvedEntries;
    private bool freshCutAllowed;
    private bool lastExitAnchored=true;
    internal const double MinimumTravelClearance=.1;
    private readonly Dictionary<(Point3D,Point3D),bool> travelCache=new();
    private readonly Queue<(Point3D,Point3D)> travelOrder=new();
    private long positiveTravelQueries,positiveTravelAccepted;
    private readonly HashSet<(double,double)> pinned=new();
    private readonly HashSet<(double,double,double,double)> certifiedPortals=new();
    private readonly HashSet<int> visitedSources=new();

    internal VCarveContinuationAdapter(ToolGeometry tool, DepthSchedule depth, VCarveOptions options,
        Func<Point3D, Point3D, bool>? originalCutGuard, CancellationToken token, Action<string>? log,
        IReadOnlyList<TaggedToolpath> paths)
    {
        options = options.WithPriorStock(token);
        this.depth = depth; this.options = options; this.originalCutGuard = originalCutGuard;
        prior = options.PriorStock?.CreateQuerySession(8_000_000);
        roughing=options.RoughingRestChords.ToArray();
        this.token = token; this.log = log;
        profile = VCarvePhysicalSweep.ProfileFor(tool, depth.Depth);
        distance = Math.Max(.1, 2 * options.StepOver * Math.Max(profile.ClearingDiameter, 2 * profile.PhysicalRadiusAtDepth(Math.Min(depth.Depth, depth.DepthPerPass))));
        grade = Math.Tan(options.SmoothRampAngleDegrees * Math.PI / 180);
        step = depth.DepthPerPass > 0 ? depth.DepthPerPass : depth.Depth;
        var travelDepths=new List<double>{2*MinimumTravelClearance,step/2,step};
        for(double level=2*step;level<depth.Depth && travelDepths.Count<12;level+=step)travelDepths.Add(level);
        travelDepths.Add(depth.Depth);
        airDepths=travelDepths.Where(d=>d>0&&double.IsFinite(d)).Distinct().OrderByDescending(d=>d).ToArray();
        planner = new("vcarve", distance, token, log);
        foreach(var plan in paths.Select(p=>p.ContinuationPlan).Where(p=>p is not null).Distinct())
        foreach(var edge in plan!.Edges)
        {
            var a=edge.A.Point;var b=edge.B.Point;
            pinned.Add((a.X,a.Y));pinned.Add((b.X,b.Y));
            certifiedPortals.Add((a.X,a.Y,b.X,b.Y));certifiedPortals.Add((b.X,b.Y,a.X,a.Y));
        }
        if (paths.Count>0 && roughing.Length>0)
        {
            var points=paths.SelectMany(p=>p.Points).ToArray();
            double minX=points.Min(p=>p.X)-distance,maxX=points.Max(p=>p.X)+distance;
            double minY=points.Min(p=>p.Y)-distance,maxY=points.Max(p=>p.Y)+distance;
            // A cleared column is usable at shallower levels too. Keeping only
            // its deepest centreline strands wall visits above the bulk-air graph.
            // These levels nominate routes; each complete inflated chord is still
            // independently contained in the real earlier flat-cutter column.
            foreach(var chord in roughing.Take(100_000))
            foreach(double level in airDepths)
            {
                if (Math.Max(chord.A.X,chord.B.X)<minX || Math.Min(chord.A.X,chord.B.X)>maxX
                    || Math.Max(chord.A.Y,chord.B.Y)<minY || Math.Min(chord.A.Y,chord.B.Y)>maxY
                    || !(chord.ClearedColumnRadius>0)) continue;
                var a=chord.A with {Z=depth.SurfaceZ+Math.Min(0,Math.Max(-level,chord.A.Z+2*MinimumTravelClearance))};
                var b=chord.B with {Z=depth.SurfaceZ+Math.Min(0,Math.Max(-level,chord.B.Z+2*MinimumTravelClearance))};
                planner.RecordProvedAir(a,b,(x,y)=> {
                    var ca=Command(x);var cb=Command(y);
                    return ca.X==chord.A.X && ca.Y==chord.A.Y && cb.X==chord.B.X && cb.Y==chord.B.Y
                        && ca.Z-MinimumTravelClearance>=chord.A.Z && cb.Z-MinimumTravelClearance>=chord.B.Z && Grade(x,y)
                        && profile.PhysicalRadiusAtDepth(Math.Max(0,Math.Max(-ca.Z,-cb.Z))+MinimumTravelClearance)
                            +MinimumTravelClearance+1e-8<chord.ClearedColumnRadius;
                },positiveTravelClearance:options.CompactCollinearCommands);
            }
        }
    }

    internal ToolpathContinuationPlanner.Connection? Connect(Point3D from, ref TaggedToolpath path)
    {
        if(!lastExitAnchored)return null;
        var original=path;
        double budget=EntryLength(original,depth.SurfaceZ);
        // Compare complete alternatives: a separate visit also retracts,
        // traverses at safe height, and approaches its surface entry. Omitting
        // this air time unfairly rejects short stay-down links between tiny cuts.
        budget+=ToolpathContinuationPlanner.SafeHeightTransferCost(from,original.Points[0] with{Z=depth.SurfaceZ},
            options.ContinuationSafeZ,options.EntryTransferCostRatio);
        var entries=original.ContinuationPlan is { } plan && original.ContinuationSourceOrdinal is { } owner
            ? plan.Entries(owner).Where(p=>visitedSources.Contains(p.Other.Owner)).ToArray()
            : Array.Empty<(ToolpathContinuationComponents.Portal Own,ToolpathContinuationComponents.Portal Other)>();
        // Preserve a cheap original seam before considering an interior portal.
        // An open-path portal otherwise adds a backwards prefix to every visit.
        var indices=new List<int>{0};
        foreach(var entry in entries.OrderBy(p=>Math.Pow(from.X-p.Own.Point.X,2)+Math.Pow(from.Y-p.Own.Point.Y,2)))
        {
            int index=-1;
            for(int i=0;i<original.Points.Count;i++)
                if(original.Points[i].X==entry.Own.Point.X && original.Points[i].Y==entry.Own.Point.Y){index=i;break;}
            if(index>=0 && !indices.Contains(index))indices.Add(index);
            if(indices.Count>=8)break;
        }
        // A forest portal is an XY relationship, not a permanent first-layer Z.
        // Nominate its literal executed poses at ALL completed depths. Otherwise
        // a valid next-layer connector is missed and the search climbs back to
        // the first layer even after its parent ring was just cut at this depth.
        var preferred=entries.SelectMany(p=>planner.ClearedEntries(
                p.Other.Point with{Z=depth.SurfaceZ-depth.Depth},depth.SurfaceZ,upwardProfileIsContained:true)
            .Append(p.Other.Point with{Z=depth.SurfaceZ+p.Other.Point.Z})).Distinct().ToArray();
        foreach(int index in indices)
        {
            var candidate=Rebase(original,index);
            if(Length(candidate.Points)>budget+1e-9)continue;
            var connection=ConnectCore(from,candidate,preferred,budget);
            if(connection is null)continue;
            path=candidate;return connection;
        }
        return null;
    }

    private ToolpathContinuationPlanner.Connection? ConnectCore(Point3D from,TaggedToolpath path,
        IReadOnlyList<Point3D> preferred,double budget)
    {
        freshCutAllowed=path.Category=="clearing";
        var targets = new List<Point3D> { path.Points[0] };
        // These are literal earlier executed cutter poses, not an authored
        // StartDepth, pass-number inference, or future finished-stock envelope.
        targets.AddRange(planner.ClearedEntries(path.Points[0], depth.SurfaceZ, upwardProfileIsContained: true));
        // A previous flat rougher need not have visited this V-stroke's exact
        // centreline. Nominate raised entry poses, then prove their entire
        // physical cutter in real earlier stock. Authored StartDepth supplies
        // no authority; these poses become executed only through Connect.
        if(roughing.Length>0)
        {
            prior ??= new(roughing.Select(c=>new ClearedColumnStock.Column(c.A,c.B,c.ClearedColumnRadius)),token,8_000_000);
            foreach(double level in airDepths)
            {
                var point=path.Points[0] with{Z=depth.SurfaceZ-level};
                if(point.Z<path.Points[0].Z || point.Z>=depth.SurfaceZ || targets.Contains(point))continue;
                var command=Command(point);
                if(prior.Contains(command,command,profile.PhysicalRadiusAtDepth))
                {targets.Add(point);stockProvedEntries++;}
            }
        }
        var entries=targets.Distinct().ToArray();
        // Exact cost includes contact-to-seam recovery on an open path and the
        // complete retained cutting trace. A lower nominal Z alone is not a
        // promise that an entry or interior-portal retrace will be cheaper.
        var remaining=entries.Select(p=>EntryLength(path,p.Z)).ToArray();
        return planner.Connect(from,entries,AllowsNewChord,PositiveTravelClearance,
            options.ClearedTravelCostRatio,remaining,preferred,maximumCost:budget);
    }

    private double EntryLength(TaggedToolpath path,double z)=>Length(RampPlanner.PlanPathEntry(path.Points,z,
        options.Ramp with{AngleRadians=Math.Min(options.Ramp.AngleRadians>0 ? options.Ramp.AngleRadians : double.PositiveInfinity,
            options.SmoothRampAngleDegrees*Math.PI/180),Strategy=RampStrategy.Linear},token,diagnostic:null,
        landAtSourceVertex:true,preserveClosedExit:false).Points);
    private static double Length(IReadOnlyList<Point3D> points)
    {
        double length=0;
        for(int i=1;i<points.Count;i++)length+=ToolpathContinuationPlanner.Distance(points[i-1],points[i]);
        return length;
    }

    internal void Record(TaggedToolpath path)
    {
        var graphPath=path;
        if(options.CompactCollinearCommands)
        {
            // Forest portals remain graph anchors even when surrounding source
            // subdivisions can be compacted exactly in native and posted space.
            var spans=new List<ToolpathSpan>();
            var originalSpans=path.Spans.Count>0 ? path.Spans : new[]{new ToolpathSpan(0,path.Points.Count-1,ToolpathSpanKind.Cut)};
            foreach(var span in originalSpans)
            {
                int start=span.StartIndex;
                for(int i=start+1;i<span.EndIndex;i++)
                    if(pinned.Contains((path.Points[i].X,path.Points[i].Y)))
                    {spans.Add(new(start,i,span.Kind));start=i;}
                if(span.EndIndex>start)spans.Add(new(start,span.EndIndex,span.Kind));
            }
            graphPath=new TaggedToolpath(path.Points,spans,path.RegionIndex,path.Category,path.DepthPassIndex,
                path.VisitIdentity,detectCurves:false);
            graphPath=ToolpathCollinearCompactor.Compact(graphPath,depth.SurfaceZ,token);
        }
        // Exact replay of a retained native chord inherits its literal posted
        // sweep. Do not reject its existing F3 grade rounding as if it were a
        // newly invented cutting connector. New connectors retain both checks.
        lastExitAnchored=planner.Record(graphPath,(a,b)=>NativeGrade(a,b,1e-8));
        if(path.ContinuationSourceOrdinal is { } owner)visitedSources.Add(owner);
    }

    private static TaggedToolpath Rebase(TaggedToolpath path,int index)
    {
        if(index==0 || path.Spans.Any(s=>s.Kind!=ToolpathSpanKind.Cut))return path;
        var source=path.Points;var points=new List<Point3D>();
        if(source[0]==source[^1])
        {
            int count=source.Count-1;index%=count;
            for(int i=0;i<count;i++)points.Add(source[(index+i)%count]);
            points.Add(points[0]);
        }
        else
        {
            // Reach the original seam through the original prefix, then retain
            // every original directed cutting chord. No synthetic XY is added.
            for(int i=index;i>=0;i--)points.Add(source[i]);
            points.AddRange(source.Skip(1));
        }
        return new TaggedToolpath(points,new[]{new ToolpathSpan(0,points.Count-1,ToolpathSpanKind.Cut)},
            path.RegionIndex,path.Category,path.DepthPassIndex,path.VisitIdentity,path.DetectCurves)
            {ContinuationComponentIndex=path.ContinuationComponentIndex,ContinuationPlan=path.ContinuationPlan,
                ContinuationSourceOrdinal=path.ContinuationSourceOrdinal};
    }
    internal void Reentry(Point3D? from, Point3D target) => planner.Reentry(from is null
        ? "first-entry" : !lastExitAnchored ? "exit-not-in-proved-network" : planner.LastFailure, from, target);
    internal void Report()
    {
        planner.Report();
        log?.Invoke($"stage=continuation event=physical-guards owner=vcarve priorAirLinks={airLinks} "
            + $"boundedStepCutLinks={freshLinks} stockProvedEntryPoses={stockProvedEntries} newChordQueries={guardQueries} "
            + $"priorColumnQueries={prior?.Queries ?? 0} priorColumnBudgetReached={prior?.QueryBudgetReached ?? false} "
            + $"priorColumnCacheHits={prior?.CachedQueries ?? 0} "
            + $"travelProofQueries={positiveTravelQueries} travelProofAccepted={positiveTravelAccepted} minimumTravelClearance={MinimumTravelClearance:R} "
            + $"entryCostIncludesTransfer={double.IsFinite(options.ContinuationSafeZ)} safeTransferZ={options.ContinuationSafeZ:R} cutToRapidRatio={options.EntryTransferCostRatio:R} "
            + "originalFloorAndArtwork=True nominalAndPostedGrade=True retainedScheduledCuts=True entryPose=executed-or-whole-volume-proved-XYZ");
    }

    private bool AllowsNewChord(Point3D a, Point3D b)
    {
        token.ThrowIfCancellationRequested(); guardQueries++;
        if (!Grade(a,b)) return false;
        var ca = Command(a); var cb = Command(b);
        if (roughing.Length > 0)
        {
            prior ??= new(roughing.Select(c =>
                new ClearedColumnStock.Column(c.A,c.B,c.ClearedColumnRadius)), token,8_000_000);
            if (prior.Contains(ca,cb,profile.PhysicalRadiusAtDepth)) { airLinks++; return true; }
        }
        // A NEW link has no pre-existing deeper precursor. Restrict fresh cutting
        // to the first physical step from the stock surface; original floor and
        // full native/posted artwork guards still own the entire chord.
        if ((!freshCutAllowed && !certifiedPortals.Contains((a.X,a.Y,b.X,b.Y))) || originalCutGuard is null
            || ToolpathContinuationPlanner.Distance(a,b) > distance) return false;
        if(Math.Max(-ca.Z,-cb.Z)>step+1e-10 && !planner.HasStepdownPrecursor(a,b,step,Command))return false;
        bool allowed = originalCutGuard(a with { Z = a.Z-depth.SurfaceZ }, b with { Z = b.Z-depth.SurfaceZ });
        if (allowed) freshLinks++;
        return allowed;
    }

    private bool PositiveTravelClearance(Point3D a,Point3D b)
    {
        if(!options.CompactCollinearCommands || roughing.Length==0 || !Grade(a,b))return false;
        if(travelCache.TryGetValue((a,b),out bool cached))return cached;
        positiveTravelQueries++;
        prior ??= new(roughing.Select(c=>new ClearedColumnStock.Column(c.A,c.B,c.ClearedColumnRadius)),token,8_000_000);
        var ca=Command(a);var cb=Command(b);
        // Inflate the entire query cylinder radially and below its tip. Faster
        // travel never follows merely touching old cutter sweeps or tip disks.
        bool allowed=prior.Contains(ca with{Z=ca.Z-MinimumTravelClearance},cb with{Z=cb.Z-MinimumTravelClearance},
            d=>profile.PhysicalRadiusAtDepth(d)+MinimumTravelClearance);
        if(allowed)positiveTravelAccepted++;
        if(travelCache.Count>=262144)
        {var old=travelOrder.Dequeue();travelCache.Remove(old);}
        travelCache[(a,b)]=allowed;travelOrder.Enqueue((a,b));
        return allowed;
    }

    private bool Grade(Point3D a, Point3D b)
    {
        var ca = Command(a); var cb = Command(b);
        return NativeGrade(a,b,1e-12) && NativeGrade(ca,cb,1e-12);
    }
    private bool NativeGrade(Point3D x,Point3D y,double tolerance)=>Math.Abs(x.Z-y.Z)<=grade*
        Math.Sqrt((x.X-y.X)*(x.X-y.X)+(x.Y-y.Y)*(x.Y-y.Y))+tolerance;
    private Point3D Command(Point3D p) => new(Round((float)p.X), Round((float)p.Y),
        Round((float)p.Z - (float)depth.SurfaceZ));
    private static double Round(float value) => double.Parse(value.ToString("F3",CultureInfo.InvariantCulture),CultureInfo.InvariantCulture);
}
