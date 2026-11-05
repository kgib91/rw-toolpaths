using System.Diagnostics;
using Clipper2Lib;
using RW.Toolpaths.Geometry;

namespace RW.Toolpaths.Milling;

/// <summary>The material above the floor of the Pocket's continuous grade envelope.
/// Existing pocket compensation keeps the complete cutter outside this exclusion.</summary>
internal static class SmoothRoughingPlateau
{
    internal static double CutterSupport(double delta, double grade, VCarveCutterProfile? profile)
        => profile is null ? 0 : Math.Max(0,
            profile.PhysicalProfile.SupportUpTo(grade, delta) / grade - profile.TipRadius);

    internal static double ExclusionRadius(double delta, double grade, VCarveCutterProfile? profile)
        => delta / grade + CutterSupport(delta, grade, profile);

    internal static List<IReadOnlyList<PointD>> Build(IReadOnlyList<TaggedToolpath> original,
        IReadOnlyList<IReadOnlyList<PointD>> boundary, double floor, double grade,
        CancellationToken token, IReadOnlyList<TaggedToolpath>? smoothed = null,
        VCarveCutterProfile? plateauProfile = null,
        IReadOnlyList<IReadOnlyList<PointD>>? target = null, bool pruneContainedPrimitives = true,
        bool parallelLeafUnions=true,bool reuseTargetContact=true,bool indexDiscDominance=true)
    {
        long started = Stopwatch.GetTimestamp();
        const double quantum = 1 / PathUtils.Scale;
        double arc = VCarvePreparation.GeometricErrorBound / 4;
        // Includes source/offset quantization, bounded leaf
        // unions, final clipping and the existing canonicalizer's one-tick trim.
        double reserve = arc + 16 * quantum;
        var constant = new Dictionary<double, List<Path64>>();
        var pending = new Paths64();
        var batches = new Paths64();
        var unionLevels = new List<List<Paths64>>();
        var targetPaths = target == null ? null : PathUtils.ToClipper(target);
        var targetIndex = targetPaths == null ? null : new PolygonOps.BoundarySegmentIndex(targetPaths);
        List<bool>? targetContactMemo=pruneContainedPrimitives&&reuseTargetContact&&targetIndex is not null?new():null;
        long targetContactQueries=0,targetContactMemoHits=0;
        long leafBatches = 0, irrelevantChords = 0, identicalPrimitives = 0;
        int leafSize = target == null ? 64 : 128;
        long pendingPoints=0;
        using var leafUnions=parallelLeafUnions&&targetPaths is not null&&GeometryParallelism.WorkerLimit>1
            &&original.Sum(p=>(long)p.Points.Count)>=128
            ?new VCarveOrderedUnionQueue(token,Math.Min(4,GeometryParallelism.WorkerLimit),maximumPathsPerLeaf:128):null;
        var leafSlots=new List<VCarveOrderedUnionQueue.Slot>(4);
        Path64? previousPrimitive = null;
        bool collectOnly = pruneContainedPrimitives;
        bool collectAnchorData = true;
        PlateauDiscCoverage? discCoverage = null;
        long containedPrimitives = 0;
        long containedSourceChords = 0;
        void Carry(Paths64 paths,int level)
        {
            if(level>8) throw new InvalidOperationException("Outside-edge plateau union exceeds its certified hierarchy reserve.");
            while(unionLevels.Count<=level) unionLevels.Add(new());
            var entries=unionLevels[level]; entries.Add(paths);
            if(entries.Count<8) return;
            token.ThrowIfCancellationRequested();
            var operands=new Paths64(); foreach(var entry in entries) operands.AddRange(entry);
            var combined=Clipper.Union(operands,FillRule.NonZero); entries.Clear();
            Carry(combined,level+1);
        }
        void KeepBatch(Paths64 paths)
        {
            token.ThrowIfCancellationRequested();
            if(paths.Count==0) return;
            if(targetPaths==null) batches.AddRange(paths); else Carry(paths,0);
        }
        void DrainLeaves()
        {
            if(leafUnions is null||leafSlots.Count==0)return;
            leafUnions.Drain();
            foreach(var slot in leafSlots)KeepBatch(leafUnions.ReadDrained(slot));
            leafSlots.Clear();
        }
        void UnionLeaf(Paths64 input)
        {
            if(input.Count==0)return;
            if(leafUnions is null){KeepBatch(Clipper.Union(input,FillRule.NonZero));return;}
            leafSlots.Add(leafUnions.Submit(input));
            if(leafSlots.Count==4)DrainLeaves();
        }
        var discs = new HashSet<(double X, double Y, double Radius)>();
        var circles = new Dictionary<int, (double X, double Y)[]>();
        using var hull = new VCarveOrderedDiscHull();
        long sourceChords = 0, flatChords = 0, taperedChords = 0, containedChords = 0, raisedSpans = 0;
        long identicalDiscRowsAvoided=0;
        Console.WriteLine($"[Pocket.SmoothRoughing.Plateau] event=start paths={original.Count} floor={floor:R} grade={grade:R}");

        // Match the finishing floor owner: ONLY spans actually raised by the
        // grade limiter introduce a ramp floor. Unchanged shallow wall/corner
        // detail is not a new floor anchor. Extending every shallow raw feature
        // into a cone erases broad, otherwise reachable flat-floor regions.
        // A selected shallow anchor excludes a material disc of radius
        // (floor-depth)/grade. Linear spans sweep the convex hull of endpoint
        // discs, including the lower interpolation between opposing fronts.
        double Delta(Point3D p) => Math.Max(0, floor + p.Z);
        // The finish field constrains its centre. Its flat tip already contacts
        // a disk at full depth; remove that contact radius to obtain the actual
        // material plateau. A pointed/rounded tip has zero full-depth disk.
        double Support(double delta) => CutterSupport(delta, grade, plateauProfile);
        double Radius(Point3D p) => Delta(p) / grade + Support(Delta(p));
        void Publish(Path64 polygon)
        {
            if(targetPaths!=null && previousPrimitive!=null && polygon.SequenceEqual(previousPrimitive))
            { identicalPrimitives++;return; }
            previousPrimitive=polygon;
            if (polygon.Count >= 3) { pending.Add(polygon);pendingPoints+=polygon.Count; }
            if (pending.Count < leafSize && pendingPoints<32768) return;
            if(targetPaths!=null && (leafBatches<4 || leafBatches%128==0)) Console.WriteLine($"[Pocket.SmoothRoughing.Plateau] event=leaf-union-start batch={leafBatches} polygons={pending.Count} points={pending.Sum(p=>p.Count)} targetRestricted=True");
            UnionLeaf(pending); leafBatches++;
            pending.Clear();pendingPoints=0;
        }
        void Polygon(Point3D a, double ra, Point3D b, double rb)
        {
            if(collectOnly) return;
            double maximum = Math.Max(ra, rb) + reserve;
            int count = Math.Max(16, (int)Math.Ceiling(Math.PI / Math.Acos(Math.Clamp(1 - arc / maximum, -1, 1))));
            double factor = 1 / Math.Cos(Math.PI / count);
            if(discCoverage!=null && discCoverage.ContainsHull(
                new Point64(a.X*PathUtils.Scale,a.Y*PathUtils.Scale),(long)Math.Ceiling((ra+reserve)*factor*PathUtils.Scale)+2,
                new Point64(b.X*PathUtils.Scale,b.Y*PathUtils.Scale),(long)Math.Ceiling((rb+reserve)*factor*PathUtils.Scale)+2))
            {containedPrimitives++;return;}
            if (!circles.TryGetValue(count, out var circle))
            {
                circle = Enumerable.Range(0, count).Select(i =>
                    (Math.Cos(2 * Math.PI * i / count), Math.Sin(2 * Math.PI * i / count))).ToArray();
                circles.Add(count, circle);
            }
            // Circumscription plus integer reserve contains each real disc;
            // the convex hull therefore contains every interpolated disc.
            hull.BeginCircle(circle);
            hull.AddDisc(a, (ra + reserve) * factor);
            // Identical rows merge to the same ordered distinct integer
            // points. Retain the original hull walk and literal rounding.
            if(a.X==b.X&&a.Y==b.Y&&ra==rb)identicalDiscRowsAvoided++;
            else hull.AddDisc(b, (rb + reserve) * factor);
            var polygon=hull.FinishHull();
            Publish(polygon);
        }
        void Taper(Point3D a, double ra, Point3D b, double rb)
        {
            double length = Math.Sqrt(Math.Pow(a.X - b.X, 2) + Math.Pow(a.Y - b.Y, 2));
            double gap = length - Math.Abs(ra - rb);
            if (gap <= quantum)
            {
                if(collectAnchorData) containedChords++;
                if (ra < rb) { a = b; ra = rb; }
                if(collectAnchorData) discs.Add((a.X, a.Y, ra + Math.Max(0, gap)));
            }
            else Polygon(a, ra, b, rb);
        }
        void ProfileTaper(Point3D a, Point3D b)
        {
            double da = Delta(a), db = Delta(b);
            double length = Math.Sqrt(Math.Pow(a.X - b.X, 2) + Math.Pow(a.Y - b.Y, 2));
            // The inverse support radius increases by at least delta/grade.
            // A grade-limited flank is therefore contained in its shallow end
            // disc, including nonlinear ball support between the endpoints.
            double gap = length - Math.Abs(da - db) / grade;
            if (gap <= quantum)
            {
                var p = da >= db ? a : b;
                if(collectAnchorData) discs.Add((p.X, p.Y, Radius(p) + Math.Max(0, gap)));
                if(collectAnchorData) containedChords++;
                return;
            }
            double sa = Support(da), sb = Support(db);
            if (Math.Abs(sa - sb) <= arc)
            {
                // Upper affine radius uses the larger continuous support.
                // This retains every interior disc, not just endpoint samples.
                double support = Math.Max(sa, sb);
                Taper(a, da / grade + support, b, db / grade + support);
                return;
            }
            var mid = new Point3D((a.X + b.X) / 2, (a.Y + b.Y) / 2, (a.Z + b.Z) / 2);
            ProfileTaper(a, mid); ProfileTaper(mid, b);
        }
        var anchors = smoothed is null ? original.ToList() : VCarveSmoothing.RaisedFloorSpans(original, smoothed, token);
        raisedSpans = smoothed is null ? 0 : anchors.Count;
        void VisitAnchors()
        {
        int targetSite=0;
        foreach (var path in anchors)
        {
            token.ThrowIfCancellationRequested();
            if(collectAnchorData) sourceChords += Math.Max(0, path.Points.Count - 1);
            if (path.Points.Count == 1 && Radius(path.Points[0]) > 0)
                Taper(path.Points[0], Radius(path.Points[0]), path.Points[0], Radius(path.Points[0]));
            for (int i = 1; i < path.Points.Count;)
            {
                if ((i & 255) == 0) token.ThrowIfCancellationRequested();
                var a = path.Points[i - 1]; var b = path.Points[i];
                double ra = Radius(a), rb = Radius(b);
                if (ra == 0 && rb == 0) { i++; continue; }
                if(targetIndex is not null)
                {
                    bool contacts;
                    if(!collectAnchorData&&targetContactMemo is not null)
                    {
                        if(targetSite>=targetContactMemo.Count)throw new InvalidOperationException("Plateau source-contact sequence changed between immutable passes.");
                        contacts=targetContactMemo[targetSite];targetContactMemoHits++;
                    }
                    else
                    {
                        targetContactQueries++;
                        contacts=targetIndex.ContactIntervals(new Point64(a.X*PathUtils.Scale,a.Y*PathUtils.Scale),
                            new Point64(b.X*PathUtils.Scale,b.Y*PathUtils.Scale),(Math.Max(ra,rb)+reserve)*PathUtils.Scale,token,out _).Count>0;
                        // Reuse only this deterministic immutable anchor walk,
                        // with identical original target, literal endpoints and
                        // radius. Larger inputs retain both original query passes.
                        if(targetContactMemo is not null)
                        {
                            if(targetContactMemo.Count==2_000_000)targetContactMemo=null;
                            else targetContactMemo.Add(contacts);
                        }
                    }
                    targetSite++;
                    if(!contacts){if(collectAnchorData)irrelevantChords++;i++;continue;}
                }
                if (ra != rb)
                {
                    // Every recursive profile support is monotone in depth and
                    // bounded by the larger endpoint radius. Its circumscribed
                    // polygon exceeds that radius by at most two arc budgets.
                    // Prove the complete upper capsule redundant before splitting.
                    double upper=Math.Max(ra,rb)+reserve+2*arc;
                    long upperTicks=(long)Math.Ceiling(upper*PathUtils.Scale)+2;
                    if(!collectAnchorData && discCoverage!=null && discCoverage.ContainsHull(
                        new(a.X*PathUtils.Scale,a.Y*PathUtils.Scale),upperTicks,
                        new(b.X*PathUtils.Scale,b.Y*PathUtils.Scale),upperTicks))
                    {containedSourceChords++;i++;continue;}
                    if(collectAnchorData) taperedChords++;
                    if (plateauProfile is null) Taper(a, ra, b, rb); else ProfileTaper(a, b);
                    i++; continue;
                }
                var line = new Path64 { new(a.X * PathUtils.Scale, a.Y * PathUtils.Scale) };
                do
                {
                    var point = path.Points[i++];
                    line.Add(new Point64(point.X * PathUtils.Scale, point.Y * PathUtils.Scale));
                    if(collectAnchorData) flatChords++;
                } while (i < path.Points.Count && Radius(path.Points[i]) == ra);
                if(collectAnchorData)
                {
                    if (!constant.TryGetValue(ra, out var lines)) constant.Add(ra, lines = new());
                    lines.Add(line);
                }
            }
        }
        if(!collectAnchorData&&targetContactMemo is not null&&targetSite!=targetContactMemo.Count)
            throw new InvalidOperationException("Plateau source-contact inventory changed between immutable passes.");
        }
        VisitAnchors();
        // Remove only analytically contained discs. Larger retained discs own
        if(targetPaths!=null) Console.WriteLine($"[Pocket.SmoothRoughing.Plateau] event=anchors-complete raised={raisedSpans} irrelevantChords={irrelevantChords} discs={discs.Count} groups={constant.Count} batches={leafBatches}");
        // their complete descendants; noncontained span hulls stay untouched.
        double cell = Math.Max(quantum, discs.Select(d => d.Radius).DefaultIfEmpty(quantum).Max());
        var cells = new Dictionary<(long X, long Y), List<(double X, double Y, double Radius)>>();
        int retainedDiscs = 0, dominatedDiscs = 0;
        var certificates=new List<PlateauDiscCoverage.Disc>();
        bool deferredPolygons=collectOnly; collectOnly=false;
        var orderedDiscs=discs.OrderByDescending(d=>d.Radius).ToArray();
        long discIndexStarted=Stopwatch.GetTimestamp();
        var dominance=indexDiscDominance&&orderedDiscs.Length>=512?new PlateauDiscDominance(orderedDiscs,token):null;
        double discIndexMs=Stopwatch.GetElapsedTime(discIndexStarted).TotalMilliseconds;
        for(int ordinal=0;ordinal<orderedDiscs.Length;ordinal++)
        {
            var disc=orderedDiscs[ordinal];
            token.ThrowIfCancellationRequested();
            long x = (long)Math.Floor(disc.X / cell), y = (long)Math.Floor(disc.Y / cell);
            bool covered = dominance?.IsCovered(ordinal)??false;
            for (long dx = -1; dominance is null && dx <= 1 && !covered; dx++)
            for (long dy = -1; dy <= 1 && !covered; dy++)
                if (cells.TryGetValue((x + dx, y + dy), out var neighbours))
                    covered = neighbours.Any(other => other.Radius - disc.Radius > 1e-10
                        && Math.Pow(other.X - disc.X, 2) + Math.Pow(other.Y - disc.Y, 2)
                            <= Math.Pow(other.Radius - disc.Radius - 1e-10, 2));
            if (covered) { dominatedDiscs++; continue; }
            if(dominance is not null)dominance.Keep(ordinal);
            else
            {
                if (!cells.TryGetValue((x, y), out var bucket)) cells.Add((x, y), bucket = new());
                bucket.Add(disc);
            }
            retainedDiscs++;
            var point = new Point3D(disc.X, disc.Y, 0);
            Polygon(point, disc.Radius, point, disc.Radius);
            // Circumscription contains radius+reserve before integer rounding.
            // Withhold two ticks for rounded vertices and the rounded centre;
            // the inner disk is a subset of this exact published convex polygon.
            if(deferredPolygons && (Math.Max(Math.Abs(disc.X),Math.Abs(disc.Y))+disc.Radius+reserve)*PathUtils.Scale<(1L<<40))
                certificates.Add(new(new Point64(disc.X*PathUtils.Scale,disc.Y*PathUtils.Scale),
                    (long)Math.Floor((disc.Radius+reserve)*PathUtils.Scale)-2));
        }
        if(deferredPolygons)
        {
            discCoverage=new(certificates,token);
            collectAnchorData=false;VisitAnchors();
            Console.WriteLine($"[Pocket.SmoothRoughing.Plateau] event=contained-primitives certificates={certificates.Count} "
                + $"queries={discCoverage.Queries} candidates={discCoverage.Candidates} pruned={containedPrimitives} "
                + $"priorDiscTrials={discCoverage.PriorDiscTrials} priorDiscWins={discCoverage.PriorDiscWins} "
                + $"wholeSourceChordsPruned={containedSourceChords} "
                + $"authority=integer-convex-subset-of-published-disc sourceGradeAndSupportUnchanged=True");
        }
        DrainLeaves();
        foreach (var group in constant)
        {
            token.ThrowIfCancellationRequested();
            long offsetStarted = Stopwatch.GetTimestamp();
            var strokes=group.Value;
            if(strokes.Count>=128&&strokes.All(p=>p.Count==2&&p[0]!=p[1]))
            {
                // Buffering a connected polyline is the same disk sweep as
                // buffering all its edges separately. Keep the ORIGINAL integer
                // vertices and every edge; no fitting, snapping or bridge earns
                // material exclusion. Bound chunks so offset arrangements stay small.
                var chains=PocketPostedChordChains.Build(strokes.Select(p=>new PocketPostedChordChains.Segment(
                    new(p[0].X,p[0].Y),new(p[1].X,p[1].Y))).ToArray(),token);
                strokes=new List<Path64>();
                foreach(var chain in chains.Paths)
                for(int first=0;first<chain.Count-1;first+=256)
                    strokes.Add(new Path64(chain.Skip(first).Take(257).Select(p=>new Point64(p.X,p.Y))));
                Console.WriteLine($"[Pocket.SmoothRoughing.Plateau] event=exact-crest-chains sourcePaths={group.Value.Count} "
                    +$"uniqueEdges={chains.UniqueEdges} duplicateEdges={chains.DuplicateEdges} chains={chains.Paths.Count} chunks={strokes.Count} "
                    +$"integerScale={PathUtils.Scale:R} maximumChunkEdges=256 literalEdgesPreserved=True noNewConnections=True sameRoundDiskSweep=True");
            }
            Console.WriteLine($"[Pocket.SmoothRoughing.Plateau] event=buffer-start radius={group.Key:R} paths={strokes.Count} points={strokes.Sum(p => p.Count)}");
            Paths64 OffsetConstant(Paths64 input)
            {
                var offset = new ClipperOffset(2, arc * PathUtils.Scale);
                offset.AddPaths(input, JoinType.Round, EndType.Round);
                var buffered = new Paths64();
                offset.Execute((group.Key + reserve) * PathUtils.Scale, buffered);
                return buffered;
            }
            using var constantOffsets = parallelLeafUnions && GeometryParallelism.WorkerLimit > 1
                && strokes.Sum(p=>(long)p.Count)>=8192
                ? new VCarveOrderedUnionQueue(token,GeometryParallelism.WorkerLimit,OffsetConstant,
                    maximumPathsPerLeaf:128) : null;
            var constantSlots = new List<VCarveOrderedUnionQueue.Slot>();
            long bufferedPoints = 0;
            void PublishConstant(Paths64 buffered)
            { bufferedPoints += buffered.Sum(p=>(long)p.Count); KeepBatch(buffered); }
            void DrainConstants()
            {
                if(constantOffsets is null || constantSlots.Count==0)return;
                constantOffsets.Drain();
                foreach(var slot in constantSlots)PublishConstant(constantOffsets.ReadDrained(slot));
                constantSlots.Clear();
            }
            for (int first = 0; first < strokes.Count;)
            {
                token.ThrowIfCancellationRequested();
                int last=first,vertices=0;
                do{vertices+=strokes[last++].Count;}
                while(last<strokes.Count&&last-first<leafSize&&vertices+strokes[last].Count<=2048);
                // Same round primitives, bounded leaf unions: avoid constructing
                // one arrangement containing thousands of overlapping end caps.
                // Round joins are inscribed, so pay their sagitta and integer
                // arithmetic before using the result as an exclusion.
                var input = new Paths64(strokes.Skip(first).Take(last-first));
                if(constantOffsets is null)PublishConstant(OffsetConstant(input));
                else
                {
                    constantSlots.Add(constantOffsets.Submit(input));
                    if(constantSlots.Count==constantOffsets.WorkerCount)DrainConstants();
                }
                first=last;
            }
            DrainConstants();constantOffsets?.Complete();
            Console.WriteLine($"[Pocket.SmoothRoughing.Plateau] event=buffer-complete radius={group.Key:R} points={bufferedPoints} leafPaths={leafSize} "
                +$"elapsedMs={Stopwatch.GetElapsedTime(offsetStarted).TotalMilliseconds:F2} offsetWorkers={constantOffsets?.WorkerCount??1} "
                +$"offsetComputeMs={constantOffsets?.ComputeMilliseconds??0:F1} offsetWaitMs={constantOffsets?.WaitMilliseconds??0:F1} "
                +$"offsetRetainedInputBytes={constantOffsets?.PeakRetainedInputBytes??0} originalIntegerOperandsAndHierarchy=True");
        }
        if(targetPaths==null) batches.AddRange(pending); else UnionLeaf(pending);
        DrainLeaves();leafUnions?.Complete();
        if(targetPaths!=null)
        {
            // Streaming signed unions retain at most seven groups per level.
            // Identical integer primitives are idempotent, never new removal credit.
            // The sixteen-tick reserve covers at most eight additional Boolean levels.
            foreach(var level in unionLevels) foreach(var group in level) batches.AddRange(group);
            Console.WriteLine($"[Pocket.SmoothRoughing.Plateau] event=target-union levels={unionLevels.Count} identicalPrimitives={identicalPrimitives} retainedGroups={unionLevels.Sum(l=>l.Count)}");
        }
        token.ThrowIfCancellationRequested();
        if(targetPaths!=null) Console.WriteLine($"[Pocket.SmoothRoughing.Plateau] event=final-union-start polygons={batches.Count} points={batches.Sum(p=>p.Count)}");
        var united = Clipper.Union(batches, FillRule.NonZero);
        if(targetPaths!=null)
        {
            Console.WriteLine($"[Pocket.SmoothRoughing.Plateau] event=target-clip sourcePoints={united.Sum(p=>p.Count)} targetPoints={targetPaths.Sum(p=>p.Count)}");
            united=Clipper.Intersect(united,targetPaths,FillRule.NonZero);
        }
        var clipped = Clipper.Intersect(united, PathUtils.ToClipper(boundary), FillRule.NonZero);
        token.ThrowIfCancellationRequested();
        clipped=ConservativeSurfaceMask.Compact(clipped,token);
        var result = PathUtils.FromClipper(clipped).Select(p => (IReadOnlyList<PointD>)p).ToList();
        Console.WriteLine($"[Pocket.SmoothRoughing.Plateau] floor={floor:R} grade={grade:R} "
            + $"sourceChords={sourceChords} flatChords={flatChords} taperedChords={taperedChords} containedChords={containedChords} "
            + $"raisedSpans={raisedSpans} constantRadiusGroups={constant.Count} maskContours={result.Count} maskPoints={result.Sum(p => p.Count)} "
            + $"retainedDiscs={retainedDiscs} dominatedDiscs={dominatedDiscs} finishingSupport={plateauProfile is not null} "
            + $"arcReserve={arc:R} integerAndGradeReserve={16 * quantum:R} elapsedMs={Stopwatch.GetElapsedTime(started).TotalMilliseconds:F2} "
            + $"leafSize={leafSize} pendingVertexFlushThreshold=32768 leafUnionWorkers={leafUnions?.WorkerCount??1} leafUnionComputeMs={leafUnions?.ComputeMilliseconds??0:F1} "
            + $"leafUnionWaitMs={leafUnions?.WaitMilliseconds??0:F1} leafUnionMaxInFlight={leafUnions?.PeakInFlight??0} "
            + $"targetContactQueries={targetContactQueries} targetContactMemoHits={targetContactMemoHits} targetContactMemoEntries={targetContactMemo?.Count??0} "
            + $"discDominanceIndexed={dominance is not null} discDominanceBuildMs={discIndexMs:F1} discDominanceQueries={dominance?.Queries??0} "
            + $"discDominanceNodeVisits={dominance?.NodeVisits??0} discDominanceCandidates={dominance?.Candidates??0} "
            + $"identicalDiscRowsAvoided={identicalDiscRowsAvoided} "
            + $"continuousRampEnvelope=True anchorPolicy={(smoothed is null ? "complete-supplied-segments" : "raised-medial-spans")} completeCutterCompensationRequired=True futureOperationsRead=0");
        return result;
    }
}
