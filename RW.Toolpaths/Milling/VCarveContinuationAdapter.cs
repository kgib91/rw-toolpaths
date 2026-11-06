using System.Globalization;
using RW.Toolpaths.Geometry;

namespace RW.Toolpaths.Milling;

using System.Diagnostics;

/// <summary>V-carve geometry/stock adapter for the common continuation policy.
/// Keeps all scheduled Cut traces; only their entry is replanned from proved air.</summary>
internal sealed class VCarveContinuationAdapter
{
    internal static readonly AsyncLocal<bool?> RoughingPreparationEnabled=new();

    /// <summary>One routing request's detached immutable prior-cut inventory.
    /// The ordinal BVH only omits chords that the literal family-box test would
    /// reject. It grants no stock/clearance and owns no planner/query history.</summary>
    internal sealed class RoughingPreparation
    {
        internal const long MaximumBytes=64L<<20;
        internal const int MaximumNominationChords=100_000;
        private const int LeafSize=16;
        private readonly IReadOnlyList<VCarveRoughingChord> source;
        private readonly int sourceCount;
        private readonly long maximumBytes;
        private readonly VCarveRoughingChord[]? ownedSnapshot;
        private VCarveRoughingChord[]? snapshot;
        private int[]? ordinals;
        private BoundsNode[]? nodes;
        private bool indexAttempted;
        private string refusal="none";
        private readonly record struct BoundsNode(double MinX,double MinY,double MaxX,double MaxY,
            int First,int Count,int Left,int Right);
        internal long SnapshotBuilds{get;private set;}
        internal long SnapshotReuses{get;private set;}
        internal long SnapshotCopies{get;private set;}
        internal long IndexBuilds{get;private set;}
        internal long Queries{get;private set;}
        internal long LiteralFallbackQueries{get;private set;}
        internal long ConsideredChords{get;private set;}
        internal long BoundsChordTests{get;private set;}
        internal long BoundsNodeTests{get;private set;}
        internal long SelectedChords{get;private set;}
        internal long RetainedBytes{get;private set;}
        private long preparationTicks,queryTicks;
        internal RoughingPreparation(IReadOnlyList<VCarveRoughingChord> source,long maximumBytes=MaximumBytes,
            VCarveRoughingChord[]? ownedSnapshot=null)
        {
            ArgumentNullException.ThrowIfNull(source);
            if(maximumBytes<0||maximumBytes>MaximumBytes)throw new ArgumentOutOfRangeException(nameof(maximumBytes));
            this.source=source;sourceCount=source.Count;this.maximumBytes=maximumBytes;
            if(ownedSnapshot is not null&&ownedSnapshot.Length!=sourceCount)throw new ArgumentException("Owned roughing snapshot length changed.",nameof(ownedSnapshot));
            this.ownedSnapshot=ownedSnapshot;
        }
        internal static long EstimateRetainedBytes(int sourceCount)
        {
            int count=Math.Min(sourceCount,MaximumNominationChords);
            long slots=4L*((count+LeafSize-1)/LeafSize);
            return 512+96L*sourceCount+4L*count+64L*slots;
        }
        internal bool TrySnapshot(IReadOnlyList<VCarveRoughingChord> actual,CancellationToken token,
            out VCarveRoughingChord[]? result)
        {
            result=null;
            if(!ReferenceEquals(source,actual)||sourceCount!=actual.Count)return false;
            if(snapshot is not null){SnapshotReuses++;result=snapshot;return true;}
            // Conservative bounds include the complete 72-byte chord values,
            // array alignment and ordinal/tree storage. No source is truncated
            // for later full prior-stock queries when the snapshot is admitted.
            long bytes=EstimateRetainedBytes(sourceCount);
            if(bytes>maximumBytes){refusal="snapshot-byte-budget";return false;}
            token.ThrowIfCancellationRequested();long started=Stopwatch.GetTimestamp();
            var copy=ownedSnapshot??actual.ToArray();
            token.ThrowIfCancellationRequested();
            if(copy.Length!=sourceCount||actual.Count!=sourceCount){refusal="source-count-changed";return false;}
            snapshot=copy;RetainedBytes=bytes;SnapshotBuilds++;result=copy;
            if(ownedSnapshot is null)SnapshotCopies++;
            preparationTicks+=Stopwatch.GetTimestamp()-started;
            return true;
        }
        private void PrepareIndex(CancellationToken token)
        {
            if(indexAttempted)return;
            token.ThrowIfCancellationRequested();long started=Stopwatch.GetTimestamp();
            var cuts=snapshot!;int count=Math.Min(cuts.Length,MaximumNominationChords);
            for(int i=0;i<count;i++)
            {
                if((i&1023)==0)token.ThrowIfCancellationRequested();var c=cuts[i];
                if(!Coordinate(c.A.X)||!Coordinate(c.A.Y)||!Coordinate(c.B.X)||!Coordinate(c.B.Y))
                {indexAttempted=true;refusal="literal-coordinate-domain";return;}
            }
            var order=Enumerable.Range(0,count).ToArray();
            var tree=new BoundsNode[4*((count+LeafSize-1)/LeafSize)];int next=0;
            int Build(int first,int length)
            {
                token.ThrowIfCancellationRequested();int at=next++;
                double minX=double.PositiveInfinity,minY=minX,maxX=double.NegativeInfinity,maxY=maxX;
                for(int i=first;i<first+length;i++)
                {
                    var c=cuts[order[i]];minX=Math.Min(minX,Math.Min(c.A.X,c.B.X));minY=Math.Min(minY,Math.Min(c.A.Y,c.B.Y));
                    maxX=Math.Max(maxX,Math.Max(c.A.X,c.B.X));maxY=Math.Max(maxY,Math.Max(c.A.Y,c.B.Y));
                }
                int left=-1,right=-1;
                if(length>LeafSize)
                {
                    bool x=maxX-minX>=maxY-minY;
                    int Compare(int a,int b)
                    {
                        var c=cuts[a];var d=cuts[b];
                        int comparison=(x?c.A.X+c.B.X:c.A.Y+c.B.Y).CompareTo(x?d.A.X+d.B.X:d.A.Y+d.B.Y);
                        return comparison!=0?comparison:a.CompareTo(b);
                    }
                    SpatialMedianPartition.Partition(order,first,length,Compare,token);
                    int half=length/2;left=Build(first,half);right=Build(first+half,length-half);
                }
                tree[at]=new(minX,minY,maxX,maxY,first,length,left,right);return at;
            }
            if(count>0)Build(0,count);
            token.ThrowIfCancellationRequested();
            ordinals=order;nodes=tree;indexAttempted=true;IndexBuilds++;
            preparationTicks+=Stopwatch.GetTimestamp()-started;
        }
        private static bool Coordinate(double value)=>double.IsFinite(value)&&Math.Abs(value)<=1e12;
        internal IReadOnlyList<int> Select(double minX,double minY,double maxX,double maxY,CancellationToken token)
        {
            token.ThrowIfCancellationRequested();long started=Stopwatch.GetTimestamp();
            if(snapshot is null)throw new InvalidOperationException("A roughing candidate index requires its owned snapshot.");
            int count=Math.Min(snapshot.Length,MaximumNominationChords);Queries++;ConsideredChords+=count;
            try
            {
                PrepareIndex(token);
                if(nodes is null||!Coordinate(minX)||!Coordinate(minY)||!Coordinate(maxX)||!Coordinate(maxY)
                    ||minX>maxX||minY>maxY)
                {LiteralFallbackQueries++;SelectedChords+=count;return Enumerable.Range(0,count).ToArray();}
                var selected=new List<int>();
                void Visit(int at)
                {
                    if((BoundsNodeTests++&255)==0)token.ThrowIfCancellationRequested();var node=nodes[at];
                    if(node.MaxX<minX||node.MinX>maxX||node.MaxY<minY||node.MinY>maxY)return;
                    if(node.Left>=0){Visit(node.Left);Visit(node.Right);return;}
                    for(int i=node.First;i<node.First+node.Count;i++)
                    {
                        BoundsChordTests++;int ordinal=ordinals![i];var chord=snapshot[ordinal];
                        // Same literal endpoint bounds and strict comparisons;
                        // touching boundaries are retained in original order.
                        if(Math.Max(chord.A.X,chord.B.X)<minX||Math.Min(chord.A.X,chord.B.X)>maxX
                            ||Math.Max(chord.A.Y,chord.B.Y)<minY||Math.Min(chord.A.Y,chord.B.Y)>maxY)continue;
                        selected.Add(ordinal);
                    }
                }
                if(count>0)Visit(0);
                selected.Sort();token.ThrowIfCancellationRequested();SelectedChords+=selected.Count;
                return selected;
            }
            finally{queryTicks+=Stopwatch.GetTimestamp()-started;}
        }
        internal void Report(Action<string>? log)=>log?.Invoke($"stage=continuation-roughing-preparation sourceChords={sourceCount} "
            +$"snapshotBuilds={SnapshotBuilds} snapshotCopies={SnapshotCopies} snapshotReuses={SnapshotReuses} indexBuilds={IndexBuilds} queries={Queries} "
            +$"consideredChords={ConsideredChords} boundsChordTests={BoundsChordTests} boundsNodeTests={BoundsNodeTests} selectedChords={SelectedChords} "
            +$"literalFallbackQueries={LiteralFallbackQueries} retainedBytes={RetainedBytes} maximumBytes={maximumBytes} refusal={refusal} "
            +$"preparationMs={1000d*preparationTicks/Stopwatch.Frequency:F3} queryMs={1000d*queryTicks/Stopwatch.Frequency:F3} "
            +"sourceIdentity=route-reference-and-count snapshot=complete-detached-values nominationCap=first-100000-original-ordinals "
            +"nominationOrder=original-chord-then-original-air-depth privatePlannerAndQuerySessions=True fullAirProofUnchanged=True");
    }
    private readonly ToolpathContinuationPlanner planner;
    private readonly ToolpathMotionCost? motionCost;
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
    private long airLinks, freshLinks, guardQueries,stockProvedEntries,engagementRefused,floorRefused,seamCandidates,seamConnections;
    private bool freshCutAllowed;
    private bool lastExitAnchored=true;
    internal const double MinimumTravelClearance=MillingTransitionPlanner.PositiveTravelClearance;
    private readonly Dictionary<(Point3D,Point3D),bool> travelCache=new();
    private readonly Queue<(Point3D,Point3D)> travelOrder=new();
    private long positiveTravelQueries,positiveTravelAccepted;
    private readonly HashSet<(double,double)> pinned=new();
    private readonly HashSet<(double,double,double,double)> certifiedPortals=new();
    private readonly HashSet<int> visitedSources=new();

    internal VCarveContinuationAdapter(ToolGeometry tool, DepthSchedule depth, VCarveOptions options,
        Func<Point3D, Point3D, bool>? originalCutGuard, CancellationToken token, Action<string>? log,
        IReadOnlyList<TaggedToolpath> paths,RoughingPreparation? roughingPreparation=null)
    {
        options = options.WithPriorStock(token);
        motionCost=options.Motion is null?null:new(options.Motion,depth.SurfaceZ);
        this.depth = depth; this.options = options; this.originalCutGuard = originalCutGuard;
        prior = options.PriorStock?.CreateQuerySession(8_000_000);
        if(roughingPreparation is not null&&roughingPreparation.TrySnapshot(options.RoughingRestChords,token,out var preparedRoughing))
            roughing=preparedRoughing!;
        else{roughingPreparation=null;roughing=options.RoughingRestChords.ToArray();}
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
            var candidates=roughingPreparation is null?roughing.Take(100_000)
                :roughingPreparation.Select(minX,minY,maxX,maxY,token).Select(i=>roughing[i]);
            foreach(var chord in candidates)
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

    internal ToolpathContinuationPlanner.Connection? Connect(Point3D from, ref TaggedToolpath path, TaggedToolpath? independentEntry=null)
    {
        if(!lastExitAnchored)return null;
        var original=path;
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
            if(indices.Count>=4)break;
        }
        // Keep a bounded nearest-seam shortlist in one linear pass over this
        // source. No all-pairs path search and no new source subdivision.
        var nearest=new List<(double Distance,int Index)>();
        for(int i=1;i<original.Points.Count;i++)
        {
            if((i&1023)==0)token.ThrowIfCancellationRequested();
            var p=original.Points[i];double d=(p.X-from.X)*(p.X-from.X)+(p.Y-from.Y)*(p.Y-from.Y);
            nearest.Add((d,i));nearest.Sort();if(nearest.Count>4)nearest.RemoveAt(4);
        }
        foreach(var candidate in nearest)if(!indices.Contains(candidate.Index))indices.Add(candidate.Index);
        // A forest portal is an XY relationship, not a permanent first-layer Z.
        // Nominate its literal executed poses at ALL completed depths. Otherwise
        // a valid next-layer connector is missed and the search climbs back to
        // the first layer even after its parent ring was just cut at this depth.
        var preferred=entries.SelectMany(p=>planner.ClearedEntries(
                p.Other.Point with{Z=depth.SurfaceZ-depth.Depth},depth.SurfaceZ,upwardProfileIsContained:true)
            .Append(p.Other.Point with{Z=depth.SurfaceZ+p.Other.Point.Z})).Distinct().ToArray();
        ToolpathContinuationPlanner.Connection? best=null;var bestPath=original;double bestCost=motionCost is not null && independentEntry is {} retainedEntry
            ? motionCost.CompleteVisitSeconds(from,retainedEntry):double.PositiveInfinity;
        foreach(int index in indices)
        {
            seamCandidates++;
            var candidate=Rebase(original,index);
            var connection=ConnectCore(from,candidate,preferred,bestCost);
            if(connection is null)continue;
            double cost=EntryLength(candidate,connection.Points[^1].Z);
            for(int i=1;i<connection.Points.Count;i++)cost+=motionCost?.ChordSeconds(
                connection.Points[i-1],connection.Points[i],connection.ClearedTravel?[i-1]==true)
                ?? ToolpathContinuationPlanner.Distance(connection.Points[i-1],connection.Points[i])
                    *(connection.ClearedTravel?[i-1]==true?options.ClearedTravelCostRatio:1);
            if(cost<=bestCost+1e-9){best=connection;bestPath=candidate;bestCost=cost;}
        }
        if(best is not null){path=bestPath;seamConnections++;}
        return best;
    }

    private ToolpathContinuationPlanner.Connection? ConnectCore(Point3D from,TaggedToolpath path,
        IReadOnlyList<Point3D> preferred,double budget)
    {
        freshCutAllowed=path.Category=="clearing";
        var targets = new List<Point3D> { RampPlanner.PlanTransitionEntry(from,path.Points[0],options.Ramp) };
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
            options.ClearedTravelCostRatio,remaining,preferred,maximumCost:budget,motionCost:motionCost);
    }

    private double EntryLength(TaggedToolpath path,double z)
    {
        var entry=RampPlanner.PlanPathEntry(path.Points,z,
            options.Ramp with{AngleRadians=Math.Min(options.Ramp.AngleRadians>0 ? options.Ramp.AngleRadians : double.PositiveInfinity,
                options.SmoothRampAngleDegrees*Math.PI/180),Strategy=RampStrategy.Linear},token,diagnostic:null,
            landAtSourceVertex:true,preserveClosedExit:false,preserveOpenExit:false);
        return motionCost?.VisitSeconds(new TaggedToolpath(entry.Points,entry.Spans,path.RegionIndex,
            path.Category,path.DepthPassIndex,detectCurves:false)) ?? Length(entry.Points);
    }
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

    internal static TaggedToolpath Rebase(TaggedToolpath path,int index)
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
            // Cover the original prefix. Starting at the far endpoint completes
            // the source in reverse, without an unnecessary forward replay.
            for(int i=index;i>=0;i--)points.Add(source[i]);
            if(index<source.Count-1)points.AddRange(source.Skip(1));
        }
        return path with{Points=points,Moves=new LineToolMoves(points),
            Spans=new[]{new ToolpathSpan(0,points.Count-1,ToolpathSpanKind.Cut)}};
    }
    internal void Reentry(Point3D? from, Point3D target) => planner.Reentry(from is null
        ? "first-entry" : !lastExitAnchored ? "exit-not-in-proved-network" : planner.LastFailure, from, target);
    internal void Report()
    {
        planner.Report();
        log?.Invoke($"stage=continuation event=physical-guards owner=vcarve priorAirLinks={airLinks} "
            + $"boundedStepCutLinks={freshLinks} stockProvedEntryPoses={stockProvedEntries} newChordQueries={guardQueries} "
            + $"engagementRefused={engagementRefused} floorOrWallRefused={floorRefused} "
            + $"seamCandidates={seamCandidates} connectedVisits={seamConnections} maximumSeamsPerVisit=8 completeEntryCostsCompared=True machineTimeContext={motionCost is not null} alternative=full-safe-plane-entry "
            + $"priorColumnQueries={prior?.Queries ?? 0} priorColumnBudgetReached={prior?.QueryBudgetReached ?? false} "
            + $"priorColumnCacheHits={prior?.CachedQueries ?? 0} "
            + $"travelProofQueries={positiveTravelQueries} travelProofAccepted={positiveTravelAccepted} minimumTravelClearance={MinimumTravelClearance:R} "
            + "connectionPolicy=shared-connector-then-source-ramp graphScope=source-family "
            + "originalFloorAndArtwork=True nominalAndPostedGrade=True linkSlopePolicy=authored-surface-angle retainedScheduledCuts=True entryPose=certified-connector-or-executed-XYZ");
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
        // A new link starts at a literal executed cutter pose. Bound its extra
        // radial engagement and axial drop relative to that pose; original floor
        // and full native/posted artwork guards still own the entire chord.
        if ((!freshCutAllowed && !certifiedPortals.Contains((a.X,a.Y,b.X,b.Y))) || originalCutGuard is null) return false;
        double engaged=2*profile.PhysicalRadiusAtDepth(Math.Max(0,Math.Max(-ca.Z,-cb.Z)));
        double radial=profile.RoundedTip?profile.NoseLimitedPitch(engaged,options.StepOver):options.StepOver*engaged;
        if(!ToolpathEngagementLimits.Allows(a,b,radial,step,grade,Command)
            &&!planner.HasLateralPrecursor(a,b,radial,step,grade,Command))
        {engagementRefused++;return false;}
        bool allowed = originalCutGuard(a with { Z = a.Z-depth.SurfaceZ }, b with { Z = b.Z-depth.SurfaceZ });
        if (allowed) freshLinks++;else floorRefused++;
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
        bool allowed=ContainsPositiveTravel(prior,profile,ca,cb);
        if(allowed)positiveTravelAccepted++;
        if(travelCache.Count>=262144)
        {var old=travelOrder.Dequeue();travelCache.Remove(old);}
        travelCache[(a,b)]=allowed;travelOrder.Enqueue((a,b));
        return allowed;
    }

    private static bool ContainsPositiveTravel(ClearedColumnStock stock,VCarveCutterProfile cutter,
        Point3D a,Point3D b) => stock.Contains(
            a with{Z=a.Z-MinimumTravelClearance},b with{Z=b.Z-MinimumTravelClearance},
            d=>cutter.PhysicalRadiusAtDepth(d)+MinimumTravelClearance);

    // The same immutable full-body proof also permits a faster prefix of a
    // final vertical approach. Native source geometry/grade/finish checks have
    // already completed; the host retains controlled descent where proof fails.
    internal static Func<Point3D,Point3D,bool>? CreatePriorStockTravelGuard(VCarveOptions options,
        VCarveCutterProfile cutter,double surfaceZ,CancellationToken token,Action<string>? log)
    {
        if(!options.CompactCollinearCommands)return null;
        // Ordinary material-surface finishing intentionally keeps rest-routing
        // input empty. Consume its existing actual stock ONLY for approach proof;
        // do not expose it to legacy source omission or continuation nomination.
        var chords=options.RoughingRestChords.Count>0?options.RoughingRestChords:options.ClearedStock?.Chords;
        var snapshot=options.PriorStock;
        if(snapshot is null && chords is {Count:>0})
            snapshot=new ClearedColumnStock(chords.Select(c=>new ClearedColumnStock.Column(
                c.A,c.B,c.ClearedColumnRadius)),token);
        if(snapshot is null)return null;
        var stock=snapshot.CreateQuerySession();
        bool reportedBudget=false;
        log?.Invoke($"stage=approach-travel-proof event=start priorChords={chords?.Count??0} "
            +$"clearanceMm={MinimumTravelClearance:R} nominalAndF3=True earlierStockOnly=True");
        Point3D Nominal(Point3D p)=>new((float)p.X,(float)p.Y,(float)p.Z-(float)surfaceZ);
        Point3D Posted(Point3D p)=>new(ToolpathCommandRounding.Round((float)p.X),
            ToolpathCommandRounding.Round((float)p.Y),ToolpathCommandRounding.Round((float)p.Z));
        return (a,b)=>
        {
            token.ThrowIfCancellationRequested();
            var na=Nominal(a);var nb=Nominal(b);
            if(na==nb || Math.Min(na.Z,nb.Z)>=0)return false;
            bool allowed=ContainsPositiveTravel(stock,cutter,na,nb)
                && ContainsPositiveTravel(stock,cutter,Posted(na),Posted(nb));
            if(stock.QueryBudgetReached&&!reportedBudget)
            {
                reportedBudget=true;
                log?.Invoke($"stage=approach-travel-proof event=budget-reached queries={stock.Queries} "
                    +$"cachedQueries={stock.CachedQueries} unprovedRemainAtCutFeed=True");
            }
            return allowed;
        };
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
