using Clipper2Lib;
using RW.Toolpaths.Geometry;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace RW.Toolpaths.Milling;

/// <summary>Continuous height authority from actual flat columns and complete
/// prior V-carver profiles. An
/// adaptive planar cover proves complete physical sweeps; no sampled heightmap
/// or nominal ramp plane supplies clearance. Original target XY stays intact.</summary>
internal sealed class VCarveProjectedStock
{
    private readonly record struct Node(double Left, double Bottom, double Right, double Top,
        double Depth, int First, int Count, int Low, int High,
        double SourceLeft,double SourceBottom,double SourceRight,double SourceTop,VCarveCutterProfile? CommonProfile);
    private readonly record struct Cell(double Left, double Bottom, double Right, double Top, int Level,
        int GeometryFlags=0,double FloorLower=0);
    private readonly record struct StockChord(Point3D A, Point3D B, double Radius, VCarveCutterProfile? Profile = null);
    private readonly StockChord[] columns;
    private readonly Node[] nodes;
    private readonly int[] roots;
    internal int ProfileForests=>roots.Length;
    private readonly PreparedRegionDistance target, part;
    private readonly VCarveCutterProfile profile;
    private readonly CancellationToken token;
    private readonly Action<string>? diagnostic;
    private readonly double maximumDepth;
    private readonly object queryDomain = new();
    private readonly bool memoRefusals=true;
    private readonly bool inheritCellClearance=true;
    internal long InheritedStockCellProofs;
    internal long InheritedRegionDistanceQueriesAvoided;
    internal long ReusedCellIntersectionPredicates;
    private readonly record struct RefusalWitness(double X,double Y,double UpperDepth,bool ClearedOnly);
    private readonly RefusalWitness[] refusalWitnesses=new RefusalWitness[16];
    private int refusalWitnessCount,nextRefusalWitness;
    internal long RefusalWitnessHits,RefusalWitnessPublications,RefusalWitnessTrials;
    private readonly Dictionary<(Point3D, Point3D, bool, bool), bool> proofs = new();
    private readonly Queue<(Point3D, Point3D, bool, bool)> proofOrder = new();
    private readonly Dictionary<(double, double, bool), double> depths = new();
    private readonly Queue<(double, double, bool)> depthOrder = new();
    // These values describe ACTUAL stock only, independently of the proposed
    // cutter, target permission, depth and posting reserve. Fixed dyadic cells
    // let neighbouring fits reuse the same continuous whole-disk lower bound.
    private struct CellFacts
    {
        internal double PartDistance, TargetDistance, Clearance, PointClearance, SeedClearance;
    }
    // Every dyadic cell through level 14 has an exact centre on the 1/2048 mm
    // grid. Its two centre coordinates identify BOTH position and level (the
    // lowest set centre bit identifies the cell width), without a tuple hash.
    // UInt64's default hash XORs its two halves. Here those halves are X and Y:
    // a complete N by N dyadic grid can collapse to only O(N) hashes. Mix both
    // coordinates before indexing; equality and FIFO admission remain literal.
    private readonly record struct CellKey(ulong Packed)
    {
        // A value key keeps Dictionary's hash/equality path statically known;
        // the mixed hash and literal packed equality do not need virtual calls.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override int GetHashCode() => System.HashCode.Combine((uint)Packed, (uint)(Packed >> 32));
    }
    private readonly Dictionary<CellKey,CellFacts> stockCells = new();
    private readonly Queue<CellKey> stockCellOrder = new();
    private CellFacts uncachedCellFacts;
    private const double StockCellRootWidth = 16;
    private const int StockCellCacheLimit = 1048576;
    private readonly int stockCellCacheLimit = StockCellCacheLimit;
    internal long StockCellCacheHits, StockCellCacheMisses, StockCellCacheEvictions;
    internal long CellGeometryCacheHits, CellGeometryCacheMisses;
    internal long ProfileCandidatesPruned;
    internal long ProfileNodesPruned;
    private int lastProfileWinner=-1;
    internal long PriorPoseSeedTrials,PriorPoseSeedPositive;
    internal long SeededCellProofs,SeededCellTrials;
    private const int CacheLimit = 131072;
    internal const int SweepProofCacheLimit=1048576;
    internal int CachedSweepProofs=>proofs.Count;
    internal long SweepProofCacheEvictions;
    internal long ReversedSweepProofHits,StrongerSweepProofHits;
    internal long DepthWitnessQueries,DepthWitnessBounds,DepthWitnessSeedFits;
    private (double X,double Y,double Depth,bool ClearedOnly)? lastFittedDepth;
    internal long NearbyDepthTrials,NearbyDepthFits,NearbyDepthRefusals;
    internal long RejectedHighDepthTrials,IntermediateDepthFits;
    // Entry interpolation can introduce new F3 XY points inside a proved source
    // chord. Reserve their planar rounding shell throughout the continuous proof.
    private const double PlanarMargin = .002;
    internal long Queries, Cells, ColumnQueries, ColumnTests, CacheHits, Refusals, ImprovedSites, FlatWitnessRefusals;
    internal double LargestImprovement;
    internal int ProfileChords { get; }
    internal int IndexedChords => columns.Length;

    // The geometric index and regions are immutable. Query workers share them
    // while retaining private adaptive cells, certificates and search hints.
    internal VCarveProjectedStock CreateQueryWorker() => new(this);

    internal bool TryPositiveProof(Point3D a,Point3D b,bool postingReserve=true)
    {
        if(proofs.TryGetValue((a,b,false,postingReserve),out bool exact)&&exact
            ||a!=b&&proofs.TryGetValue((b,a,false,postingReserve),out bool reversed)&&reversed
            ||!postingReserve&&(proofs.TryGetValue((a,b,false,true),out bool stronger)&&stronger
                ||a!=b&&proofs.TryGetValue((b,a,false,true),out stronger)&&stronger))
        {CacheHits++;return true;}
        return false;
    }

    internal long ImportPositiveProofs(VCarveProjectedStock worker)
    {
        if (!ReferenceEquals(queryDomain, worker.queryDomain))
            throw new ArgumentException("Stock certificates require the identical immutable query domain.", nameof(worker));
        long imported = 0;
        foreach (var (key, result) in worker.proofs)
        {
            token.ThrowIfCancellationRequested();
            if (!result) continue;
            if (proofs.ContainsKey(key)) { proofs[key] = true; continue; }
            if (proofs.Count >= SweepProofCacheLimit)
            { proofs.Remove(proofOrder.Dequeue()); SweepProofCacheEvictions++; }
            proofs.Add(key, true); proofOrder.Enqueue(key); imported++;
        }
        return imported;
    }

    private VCarveProjectedStock(VCarveProjectedStock source)
    {
        columns = source.columns; nodes = source.nodes; roots = source.roots;
        queryDomain = source.queryDomain;
        target = source.target.CreateQueryWorker(); part = source.part.CreateQueryWorker(); profile = source.profile;
        maximumDepth = source.maximumDepth; token = source.token;
        stockCellCacheLimit = source.stockCellCacheLimit;
        memoRefusals = source.memoRefusals;
        inheritCellClearance = source.inheritCellClearance;
        ProfileChords = source.ProfileChords;
        diagnostic = null;

    }

    internal VCarveProjectedStock(VCarveRoughingChord[] columns,
        IEnumerable<IReadOnlyList<PointD>> target, IReadOnlyList<PointD> part,
        VCarveCutterProfile profile, double maximumDepth, CancellationToken token,
        IReadOnlyList<VCarveClearedSweep>? profileSweeps = null,Action<string>? diagnostic=null,
        int cellCacheLimit=StockCellCacheLimit,bool memoRefusals=true,bool inheritCellClearance=true,IReadOnlyList<VCarveRemovalGaugePose>? removalGauge=null)
    {
        if(cellCacheLimit<1||cellCacheLimit>StockCellCacheLimit)throw new ArgumentOutOfRangeException(nameof(cellCacheLimit));
        stockCellCacheLimit=cellCacheLimit;
        this.memoRefusals=memoRefusals;
        this.inheritCellClearance=inheritCellClearance;
        bool Posted(Point3D p) => double.IsFinite(p.X+p.Y+p.Z) && Math.Abs(p.X)<=10000 && Math.Abs(p.Y)<=10000 && Math.Abs(p.Z)<=100
            && p.X==Math.Round(p.X*1000)/1000d && p.Y==Math.Round(p.Y*1000)/1000d && p.Z==Math.Round(p.Z*1000)/1000d;
        var rows = new List<StockChord>();
        void Add(StockChord chord)
        {
            if(rows.Count>0)
            {
                var last=rows[^1];
                if(last.B==chord.A && last.Profile==chord.Profile && (last.Profile is not null || last.Radius==chord.Radius))
                {
                    long Q(double d)=>(long)Math.Round(d*1000);
                    long ax=Q(last.B.X)-Q(last.A.X),ay=Q(last.B.Y)-Q(last.A.Y),az=Q(last.B.Z)-Q(last.A.Z);
                    long bx=Q(chord.B.X)-Q(chord.A.X),by=Q(chord.B.Y)-Q(chord.A.Y),bz=Q(chord.B.Z)-Q(chord.A.Z);
                    if(ax*by==ay*bx && ax*bz==az*bx && ay*bz==az*by && ax*bx+ay*by+az*bz>=0)
                    {rows[^1]=last with { B=chord.B, Radius=Math.Max(last.Radius,chord.Radius) };return;}
                }
            }
            rows.Add(chord);
        }
        int ignoredEmptyColumns=0;
        foreach(var c in columns)
            if(c.ClearedColumnRadius>0)Add(new(c.A,c.B,c.ClearedColumnRadius));else ignoredEmptyColumns++;
        foreach(var sweep in profileSweeps ?? Array.Empty<VCarveClearedSweep>())
        foreach(var chord in sweep.Chords)
        {
            if(!Posted(chord.A) || !Posted(chord.B) || Math.Min(chord.A.Z,chord.B.Z)>=0)continue;
            Add(new(chord.A,chord.B,sweep.Profile.PhysicalRadiusAtDepth(Math.Max(-chord.A.Z,-chord.B.Z)),sweep.Profile));
            ProfileChords++;
        }
        // Same literal posted XY trace and same physical profile: a deeper
        // chord contains the shallower chord's material sweep at EVERY t.
        // Crossed depth gradients retain both actual sources.
        var retained=new List<StockChord>(rows.Count);
        var twins=new Dictionary<(int,int,int,int,VCarveCutterProfile?,double),int>();
        foreach(var row in rows)
        {
            int Q(double v)=>(int)Math.Round(v*1000);
            var a=row.A;var b=row.B;
            if(a.X>b.X || a.X==b.X && a.Y>b.Y)(a,b)=(b,a);
            var key=(Q(a.X),Q(a.Y),Q(b.X),Q(b.Y),row.Profile,row.Profile is null?row.Radius:0);
            if(twins.TryGetValue(key,out int index))
            {
                var old=retained[index];var oa=old.A;var ob=old.B;
                if(oa.X>ob.X || oa.X==ob.X && oa.Y>ob.Y)(oa,ob)=(ob,oa);
                if(oa.Z<=a.Z && ob.Z<=b.Z)continue;
                if(a.Z<=oa.Z && b.Z<=ob.Z){retained[index]=row;continue;}
            }
            twins[key]=retained.Count;retained.Add(row);
        }
        // Keep each immutable physical profile in its own spatial tree. Mixing
        // flat columns and profiled sweeps makes CommonProfile null throughout
        // overlapping branches and defeats their existing nonlinear upper bound.
        // This changes search order only; every original retained chord remains.
        // Proof-only removal-deviation gauges bypass posted F3 merging. Never
        // feed these translated solids to the physical no-gouge authority.
        foreach(var evidence in removalGauge??Array.Empty<VCarveRemovalGaugePose>())
        {
            var pose=evidence.Pose;
            if(!double.IsFinite(pose.X+pose.Y+pose.Z))throw new ArgumentException("Nonfinite removal gauge");
            retained.Add(new(pose,pose,evidence.Profile.PhysicalRadiusAtDepth(-pose.Z),evidence.Profile));ProfileChords++;
        }
        var groups=retained.GroupBy(c=>c.Profile).OrderBy(g=>g.Key is null?0:1).ToArray();
        this.columns = groups.SelectMany(g=>g).ToArray(); this.target = new(target); this.part = new(new[] { part });
        this.profile = profile; this.maximumDepth = maximumDepth; this.token = token;this.diagnostic=diagnostic;
        diagnostic?.Invoke($"stage=material-surface-index ignoredZeroVolumeColumns={ignoredEmptyColumns} indexedChords={this.columns.Length} "
            +$"profileChords={ProfileChords} emptyColumnsGrantNoStock=True homogeneousProfileBoundsRetained=True cellMemoHash=full-xy-mix");
        if (this.columns.Length == 0) { nodes = Array.Empty<Node>(); roots=Array.Empty<int>(); return; }
        var tree = new List<Node>();var forest=new List<int>();int offset=0;
        foreach(var group in groups){forest.Add(Build(offset,group.Count()));offset+=group.Count();}
        nodes = tree.ToArray();roots=forest.ToArray();
        diagnostic?.Invoke($"stage=material-surface-index profileForests={roots.Length} treeNodes={nodes.Length} "
            +$"homogeneousProfileNodes={nodes.Count(n=>n.CommonProfile is not null)} "
            +"grouping=immutable-physical-profile allRetainedChordsPreserved=True nonlinearBoundsUnchanged=True");
        int Build(int first, int count)
        {
            token.ThrowIfCancellationRequested();
            double left = double.PositiveInfinity, bottom = left, right = double.NegativeInfinity, top = right, depth = 0;
            double sourceLeft=left,sourceBottom=bottom,sourceRight=right,sourceTop=top;
            var commonProfile=this.columns[first].Profile;
            for (int i = first; i < first + count; i++)
            {
                var c = this.columns[i]; double r = c.Radius;
                left = Math.Min(left, Math.Min(c.A.X, c.B.X) - r);
                bottom = Math.Min(bottom, Math.Min(c.A.Y, c.B.Y) - r);
                right = Math.Max(right, Math.Max(c.A.X, c.B.X) + r);
                top = Math.Max(top, Math.Max(c.A.Y, c.B.Y) + r);
                depth = Math.Max(depth, Math.Max(-c.A.Z, -c.B.Z));
                sourceLeft=Math.Min(sourceLeft,Math.Min(c.A.X,c.B.X));sourceBottom=Math.Min(sourceBottom,Math.Min(c.A.Y,c.B.Y));
                sourceRight=Math.Max(sourceRight,Math.Max(c.A.X,c.B.X));sourceTop=Math.Max(sourceTop,Math.Max(c.A.Y,c.B.Y));
                if(c.Profile!=commonProfile)commonProfile=null;
            }
            int ordinal = tree.Count; tree.Add(default);
            int low = -1, high = -1;
            if (count > 12)
            {
                bool x = right - left >= top - bottom;
                Array.Sort(this.columns, first, count, x ? XComparer : YComparer);
                int half = count / 2; low = Build(first, half); high = Build(first + half, count - half);
            }
            tree[ordinal] = new(left, bottom, right, top, depth, first, count, low, high,
                sourceLeft,sourceBottom,sourceRight,sourceTop,commonProfile);
            return ordinal;
        }
    }

    private static readonly IComparer<StockChord> XComparer = Comparer<StockChord>.Create(
        (a, b) => (a.A.X + a.B.X).CompareTo(b.A.X + b.B.X));
    private static readonly IComparer<StockChord> YComparer = Comparer<StockChord>.Create(
        (a, b) => (a.A.Y + a.B.Y).CompareTo(b.A.Y + b.B.Y));

    /// <summary>Lower bound on removal below every point of a complete disk.
    /// Each accepted depth belongs to one actual pose along a posted chord.</summary>
    internal double ClearedDepth(double x, double y, double diskRadius, bool innerReserve = true)
    {
        ColumnQueries++;
        if (nodes.Length == 0) return 0;
        double best = 0;
        if(lastProfileWinner>=0)
        {
            // Only nominate an actual immutable PRIOR chord. Re-evaluate its
            // complete physical profile at this new disk; no previous query's
            // height or target permission is transferred.
            PriorPoseSeedTrials++;
            best=Math.Max(0,ProfileDepth(columns[lastProfileWinner],x,y,diskRadius,innerReserve));
            if(best>0)PriorPoseSeedPositive++;
        }
        Span<int> pending = stackalloc int[64];
        foreach(int root in roots)
        {
            int count=1;pending[0]=root;
            while(count>0)
            {
            var n = nodes[pending[--count]];
            if (n.Depth <= best || n.Left > x - diskRadius || n.Right < x + diskRadius
                || n.Bottom > y - diskRadius || n.Top < y + diskRadius) continue;
            if(n.CommonProfile is { } common)
            {
                // An independent minimum radial distance and maximum depth
                // bound every real pose in a homogeneous subtree from above.
                // This only skips a subtree incapable of improving the known
                // actual clearance; it never supplies positive stock credit.
                double dx=Math.Max(0,Math.Max(n.SourceLeft-x,x-n.SourceRight));
                double dy=Math.Max(0,Math.Max(n.SourceBottom-y,y-n.SourceTop));
                if(!CanImprove(common,n.Depth,best,dx*dx+dy*dy,
                    diskRadius+(innerReserve?PostedSweepInnerRegion.Reserve:0)))
                {ProfileNodesPruned++;continue;}
            }
            if (n.Low >= 0)
            {
                // Deep columns first make the immutable subtree depth bound useful.
                if (nodes[n.Low].Depth > nodes[n.High].Depth) { pending[count++] = n.High; pending[count++] = n.Low; }
                else { pending[count++] = n.Low; pending[count++] = n.High; }
                continue;
            }
            for (int i = n.First; i < n.First + n.Count; i++)
            {
                var c = columns[i];
                if (Math.Max(-c.A.Z, -c.B.Z) <= best) continue;
                if(c.Profile is { } cutter)
                {
                    ColumnTests++;
                    double depth=ProfileDepth(c,x,y,diskRadius,innerReserve,best);
                    if(depth>best){best=depth;lastProfileWinner=i;}
                    continue;
                }
                double radius = c.Radius - diskRadius - (innerReserve ? PostedSweepInnerRegion.Reserve : 0);
                if (radius <= 0) continue;
                ColumnTests++;
                double vx = c.B.X - c.A.X, vy = c.B.Y - c.A.Y, l2 = vx * vx + vy * vy;
                double px = x - c.A.X, py = y - c.A.Y, lo, hi;
                if (l2 <= 1e-24)
                {
                    if (px * px + py * py > radius * radius) continue;
                    lo = 0; hi = 1;
                }
                else
                {
                    double projection = (px * vx + py * vy) / l2;
                    double qx = px - projection * vx, qy = py - projection * vy;
                    double square = radius * radius - qx * qx - qy * qy;
                    if (square < 0) continue;
                    double half = Math.Sqrt(square / l2);
                    lo = Math.Max(0, projection - half); hi = Math.Min(1, projection + half);
                    if (lo > hi) continue;
                }
                double t = c.B.Z < c.A.Z ? hi : lo;
                best = Math.Max(best, -(c.A.Z + (c.B.Z - c.A.Z) * t));
            }
            }
        }
        return Math.Min(maximumDepth, best);
    }

    private double ProfileDepth(StockChord c,double x,double y,double diskRadius,bool innerReserve,double bestKnown=0)
    {
        var cutter=c.Profile!;
        double vx=c.B.X-c.A.X,vy=c.B.Y-c.A.Y,l2=vx*vx+vy*vy;
        double projection=l2>0 ? ((x-c.A.X)*vx+(y-c.A.Y)*vy)/l2 : 0;
        double closest=Math.Clamp(projection,0,1);
        double extra=diskRadius+(innerReserve ? PostedSweepInnerRegion.Reserve : 0);
        double nearX=x-c.A.X-vx*closest,nearY=y-c.A.Y-vy*closest;
        if(innerReserve&&!CanImprove(cutter,Math.Max(-c.A.Z,-c.B.Z),bestKnown,nearX*nearX+nearY*nearY,extra))
        {ProfileCandidatesPruned++;return 0;}
        double upper=Math.Max(-c.A.Z,-c.B.Z)-cutter.PhysicalProfile.Bottom(Math.Sqrt(nearX*nearX+nearY*nearY)+extra);
        if(innerReserve && upper<=bestKnown){ProfileCandidatesPruned++;return 0;}
        // A vertical posted trace's deepest endpoint is an actual pose and
        // dominates every other pose on that same XY trace.
        if(l2==0||c.A.Z==c.B.Z)return upper;
        double At(double t)
        {
            double qx=x-c.A.X-vx*t,qy=y-c.A.Y-vy*t;
            return -(c.A.Z+(c.B.Z-c.A.Z)*t)-cutter.PhysicalProfile.Bottom(Math.Sqrt(qx*qx+qy*qy)+extra);
        }
        // For nomination refusal only, independent extrema give an UPPER
        // bound; never use that combination to prove actual stock clearance.
        if(!innerReserve)
        {
            return upper;
        }
        double best=Math.Max(At(0),At(1));
        if(closest>0&&closest<1)best=Math.Max(best,At(closest));
        if(l2>0 && c.A.Z!=c.B.Z)
        {
            double px=x-c.A.X-vx*projection,py=y-c.A.Y-vy*projection;
            double perpendicular=Math.Sqrt(px*px+py*py),length=Math.Sqrt(l2),dd=c.A.Z-c.B.Z;
            foreach(var section in cutter.PhysicalProfile.Sections)
            {
                double offset;
                if(section.Kind==CarvingCutterSectionKind.Hemisphere)
                {
                    if(section.TopRadius*section.TopRadius<=perpendicular*perpendicular)continue;
                    offset=dd*Math.Sqrt(Math.Max(0,section.TopRadius*section.TopRadius-perpendicular*perpendicular))
                        /(length*Math.Sqrt(l2+dd*dd));
                }
                else
                {
                    if(section.TopRadius==section.BottomRadius)continue;
                    double k=dd*(section.TopRadius-section.BottomRadius)/section.Height;
                    if(l2<=k*k)continue;
                    offset=k*perpendicular/(length*Math.Sqrt(l2-k*k));
                }
                double at=Math.Clamp(projection+offset,0,1);
                if(at>0&&at<1&&at!=closest)best=Math.Max(best,At(at));
            }
        }
        return best;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool CanImprove(VCarveCutterProfile cutter,double depth,double known,
        double distanceSquared,double extra)
    {
        double height=depth-known;
        if(height<=0)return false;
        if(height>cutter.MaximumDepth||!VCarveCutterBoxIntersection.TryPointRadiusSquared(cutter,height,out double radiusSquared)
            ||!double.IsFinite(radiusSquared))return true;
        // Bottom(sqrt(distanceSquared)+extra)>height iff the complete physical
        // cross-section at height is too small. Compare squared quantities to
        // avoid two square roots and inverse-section walks in rejected leaves.
        // Strict outward bounds retain tangencies and cylindrical plateaus.
        if(distanceSquared>radiusSquared+1e-12)return false;
        double extraSquared=extra*extra,delta=radiusSquared-distanceSquared-extraSquared;
        if(delta < -1e-12)return false;
        return !(delta>=0 && 4*extraSquared*distanceSquared-delta*delta>1e-12);
    }

    internal double RefineDepth(double x, double y, double original, double flatCeiling, bool clearedOnly, PointD? witness)
    {
        if (nodes.Length == 0 || original >= maximumDepth - .002) return original;
        var key = (x, y, clearedOnly);
        if (depths.TryGetValue(key, out double cached)) return Math.Max(original, cached);
        if (witness is { } q && (clearedOnly || target.SignedDistance(q) <= PreparedRegionDistance.Uncertainty))
        {
            double floor = part.SignedDistance(q) >= PreparedRegionDistance.Uncertainty
                ? ClearedDepth(q.x,q.y,0,innerReserve:false) : 0;
            double radius = Math.Sqrt(Math.Pow(x-q.x,2)+Math.Pow(y-q.y,2));
            double ceiling = floor + profile.DepthAtRadius(radius + PreparedRegionDistance.Uncertainty);
            // This witness can exclude improvement only above the CURRENT
            // reserved lower result. The unreserved inverse-profile ceiling
            // can itself jump to the depth cap at a full-radius plateau.
            if (ceiling <= original + .002)
            {
                FlatWitnessRefusals++;
                return original;
            }
        }
        // Exact XY keys never lend clearance to another point in the same cell.
        double low = original, high = maximumDepth;
        bool witnessedUpper=TryDepthUpper(x,y,clearedOnly,witness,original,out double witnessedDepth);
        if(witnessedUpper)high=witnessedDepth;
        bool boundedUpper=witnessedUpper;
        // The unreserved inverse profile can jump across a cylindrical plateau.
        // Its high proposal is only a nomination, never a test of every depth
        // between it and the reserved layer result. Keep the complete bracket
        // even when that proposal exceeds a witnessed upper or fails proof.
        double trial = Math.Min(high, Math.Max(original + .004, flatCeiling + .002));
        if(lastFittedDepth is { } previous&&previous.ClearedOnly==clearedOnly
            &&(previous.X-x)*(previous.X-x)+(previous.Y-y)*(previous.Y-y)<=.02*.02)
        {
            // Neighbouring XY supplies proposals only. Both sides still run
            // the COMPLETE current physical cutter proof. A refusal narrows
            // the search exactly as an original binary-search refusal does;
            // it never lends clearance or an accepted depth to this point.
            double proposal=Math.Max(trial,Math.Min(high-.002,previous.Depth-.004));
            if(proposal>trial)
            {
                NearbyDepthTrials++;
                if(Prove(new(x,y,-proposal),new(x,y,-proposal),clearedOnly))
                {low=proposal;NearbyDepthFits++;}
            }
            proposal=Math.Min(high,Math.Max(trial,previous.Depth+.004));
            if(proposal>low+.002&&proposal<high)
            {
                NearbyDepthTrials++;
                if(Prove(new(x,y,-proposal),new(x,y,-proposal),clearedOnly))
                {low=proposal;NearbyDepthFits++;}
                else{high=proposal;boundedUpper=true;NearbyDepthRefusals++;}
            }
        }
        if(witnessedUpper&&high>trial&&low<=original)
        {
            // A nearby proposal only narrows the LOWER bracket after the same
            // full cutter proof passes. A rejected proposal changes no bound.
            double span=Math.Max(.016,(profile.ClearingDiameter>0?profile.ClearingDiameter:
                2*profile.PhysicalRadiusAtDepth(maximumDepth))/8);
            double seed=Math.Max(trial,high-span);
            if(seed>trial&&Prove(new(x,y,-seed),new(x,y,-seed),clearedOnly))
            {low=seed;DepthWitnessSeedFits++;}
        }
        bool rejectedHighTrial=false;
        if(low<=original&&high-low>.002)
        {
            trial=Math.Min(trial,high);
            if(Prove(new(x,y,-trial),new(x,y,-trial),clearedOnly))low=trial;
            else
            {
                high=trial;boundedUpper=true;rejectedHighTrial=true;
                RejectedHighDepthTrials++;
            }
        }
        if(high-low>.002)
        {
            if (!boundedUpper&&Prove(new(x,y,-high),new(x,y,-high),clearedOnly)) low = high;
            else for (int i = 0; i < 13 && high - low > .002; i++)
            {
                double middle = (low + high) / 2;
                if (Prove(new(x,y,-middle),new(x,y,-middle),clearedOnly)) low = middle;
                else high = middle;
            }
        }
        if(rejectedHighTrial&&low>original+.003)
        {
            if(++IntermediateDepthFits<=8)diagnostic?.Invoke($"stage=material-surface event=intermediate-depth-fit x={x:R} y={y:R} "
                +$"originalDepth={original:R} unreservedCeiling={flatCeiling:R} rejectedTrial={trial:R} fittedDepth={low:R} "
                +"highTrialWasNotNecessityProof=True completePhysicalProofRequired=True");
        }
        if (low > original + .003) { ImprovedSites++; LargestImprovement = Math.Max(LargestImprovement, low - original); }
        if (depths.Count >= CacheLimit) depths.Remove(depthOrder.Dequeue());
        depths.Add(key, low); depthOrder.Enqueue(key);
        lastFittedDepth=(x,y,low,clearedOnly);
        return low;
    }

    private bool TryDepthUpper(double x,double y,bool clearedOnly,PointD? witness,double original,out double upper)
    {
        upper=maximumDepth;DepthWitnessQueries++;
        if(maximumDepth>profile.MaximumDepth)return false;
        PointD? nominee=clearedOnly?new PointD(x,y):witness??target.NearestBoundaryPoint(new(x,y));
        if(nominee is not { } q)return false;
        if(!clearedOnly&&target.SignedDistance(q)>=-PreparedRegionDistance.Uncertainty)
        {
            double dx=q.x-x,dy=q.y-y,length=Math.Sqrt(dx*dx+dy*dy);
            if(!(length>0)||!double.IsFinite(length))return false;
            bool found=false;
            for(int i=0;i<5;i++)
            {
                double step=4*PreparedRegionDistance.Uncertainty*(1<<i);
                var outside=new PointD(q.x+step*dx/length,q.y+step*dy/length);
                if(target.SignedDistance(outside)<-PreparedRegionDistance.Uncertainty)
                {q=outside;found=true;break;}
            }
            if(!found)return false;
        }
        if(part.SignedDistance(q)<PreparedRegionDistance.Uncertainty)return false;
        // Independent pose extrema bound the removal MODEL used by ProveCore
        // from above at this exact protected point. Additional unmodeled cuts
        // cannot supply clearance to that solver either. This bounds its search
        // without granting any cleared-volume credit or discarding a provable fit.
        double floorUpper=ClearedDepth(q.x,q.y,0,innerReserve:false);
        double radius=Math.Sqrt((x-q.x)*(x-q.x)+(y-q.y)*(y-q.y));
        if(!double.IsFinite(floorUpper)||!double.IsFinite(radius))return false;
        double proposed=Math.Min(maximumDepth,floorUpper+profile.DepthAtRadius(radius+PreparedRegionDistance.Uncertainty)+.002);
        if(!(proposed>original))return false;
        var pose=new Point3D(x,y,-proposed);
        // Certify the bound with the original complete physical predicate:
        // even the model's UPPER removal height is intersected here.
        // Nested cutter footprints make every deeper pose invalid as well.
        if(!VCarveCutterBoxIntersection.Intersects(pose,pose,profile,floorUpper,q.x,q.y,q.x,q.y))return false;
        upper=proposed;DepthWitnessBounds++;
        if(DepthWitnessBounds<=4)diagnostic?.Invoke($"stage=material-surface event=protected-depth-bound x={x:R} y={y:R} "
            +$"witness=({q.x:R},{q.y:R}) modeledRemovalUpper={floorUpper:R} tipUpper={upper:R} "
            +"protectedPointVerified=True physicalIntersectionVerified=True boundOwner=solver-removal-model stockCredit=False completeFitProofRequired=True");
        return true;
    }

    internal bool Prove(Point3D a, Point3D b, bool clearedOnly = false, bool postingReserve = true)
    {
        if (Math.Max(-a.Z, -b.Z) <= 0) return true;
        var key = (a, b, clearedOnly, postingReserve);
        if (proofs.TryGetValue(key, out bool cached)) { CacheHits++; return cached; }
        // A linear sweep occupies the SAME physical set in reverse. A proof
        // with the posting shell also proves the smaller unexpanded cutter.
        // Reuse POSITIVE complete proofs only, in this immutable stock/target/
        // profile owner and with identical target permissions. A refusal can
        // be a work-budget result, and never transfers through these relations.
        if(a!=b&&proofs.TryGetValue((b,a,clearedOnly,postingReserve),out bool reversed)&&reversed)
        {CacheHits++;ReversedSweepProofHits++;return true;}
        if(!postingReserve&&(proofs.TryGetValue((a,b,clearedOnly,true),out bool stronger)&&stronger
            ||a!=b&&proofs.TryGetValue((b,a,clearedOnly,true),out stronger)&&stronger))
        {CacheHits++;StrongerSweepProofHits++;return true;}
        Queries++;
        bool result = ProveCore(a, b, clearedOnly, postingReserve);
        if (proofs.Count >= SweepProofCacheLimit)
        {proofs.Remove(proofOrder.Dequeue());SweepProofCacheEvictions++;}
        proofs.Add(key, result); proofOrder.Enqueue(key);
        return result;
    }

    internal bool ProveNativeEnvelope(Point3D a,Point3D b,double additionalPlanarMargin)
    {
        if(!double.IsFinite(additionalPlanarMargin)||additionalPlanarMargin<0)return false;
        double depth=Math.Max(-a.Z,-b.Z);
        if(!double.IsFinite(depth)||depth>profile.Sections.Sum(s=>s.Height))return false;
        // A deepened enclosing chord can exceed the actual operation depth.
        // Extend the QUERY profile so its body is never accidentally truncated;
        // the actual native curve's exact depth is checked separately.
        var envelope=depth>profile.MaximumDepth?new VCarveCutterProfile(profile.Sections,depth):profile;
        return ProveCore(a,b,false,true,additionalPlanarMargin,envelope);
    }
    internal bool DiagnoseProof(Point3D a,Point3D b,Action<string> trace)
        =>ProveCore(a,b,false,true,refusalDiagnostic:trace);

    private bool ProveCore(Point3D a, Point3D b, bool clearedOnly, bool postingReserve, double additionalMargin=0,VCarveCutterProfile? envelopeProfile=null,Action<string>? refusalDiagnostic=null)
    {
        var cutter=envelopeProfile??profile;
        double margin=(postingReserve?PlanarMargin:0)+additionalMargin;
        double radius = cutter.PhysicalRadiusAtDepth(Math.Max(-a.Z, -b.Z)) + margin;
        double left=Math.Min(a.X,b.X)-radius,bottom=Math.Min(a.Y,b.Y)-radius;
        double right=Math.Max(a.X,b.X)+radius,top=Math.Max(a.Y,b.Y)+radius;
        if(!double.IsFinite(left+bottom+right+top)){refusalDiagnostic?.Invoke("reason=nonfinite-envelope");return false;}
        if(memoRefusals)
        for(int i=0;i<refusalWitnessCount;i++)
        {
            var witness=refusalWitnesses[(nextRefusalWitness-1-i+refusalWitnesses.Length)%refusalWitnesses.Length];
            if(witness.ClearedOnly!=clearedOnly || witness.X<left || witness.X>right || witness.Y<bottom || witness.Y>top)continue;
            RefusalWitnessTrials++;
            // An UPPER bound on actual prior removal at this fixed protected
            // point, with outward depth padding. Test the current complete
            // cutter and its original posting shell at that point. A collision
            // can only refuse work under that unchanged required authority.
            // No cached refusal or neighbouring lower bound grants authority.
            if(VCarveCutterBoxIntersection.Intersects(a,b,cutter,witness.UpperDepth,
                witness.X-margin,witness.Y-margin,witness.X+margin,witness.Y+margin))
            {RefusalWitnessHits++;refusalDiagnostic?.Invoke($"reason=protected-witness x={witness.X:R} y={witness.Y:R} removalUpper={witness.UpperDepth:R} margin={margin:R}");return false;}
        }
        double originalRadiusSquared=0;
        bool stationary=a==b && VCarveCutterBoxIntersection.TryPointRadiusSquared(cutter,-a.Z,out originalRadiusSquared);
        Span<Cell> pending = stackalloc Cell[64]; int visited = 0;
        // Cover the entire original sweep box. One root at a time keeps the
        // bounded DFS stack independent of a routed chord's length. Grid
        // coordinates and all subdivisions are exact powers of two.
        for(double rootY=Math.Floor(bottom/StockCellRootWidth)*StockCellRootWidth;rootY<=top;rootY+=StockCellRootWidth)
        for(double rootX=Math.Floor(left/StockCellRootWidth)*StockCellRootWidth;rootX<=right;rootX+=StockCellRootWidth)
        {
            int count=1;
            pending[0]=new(rootX,rootY,rootX+StockCellRootWidth,rootY+StockCellRootWidth,0);
            while(count>0)
            {
                token.ThrowIfCancellationRequested();
                var cell = pending[--count];
                Cells++;
                if (++visited > 16384) { Refusals++; refusalDiagnostic?.Invoke($"reason=cell-budget visited={visited} stationary={stationary}");return false; }
                if(cell.Left>right||cell.Right<left||cell.Bottom>top||cell.Top<bottom)continue;
                // No stock query is needed where even the complete cutter below
                // the original top cannot intersect this cell.
                double distanceSquared=stationary?VCarveCutterBoxIntersection.PointBoxDistanceSquared(a,
                    cell.Left-margin,cell.Bottom-margin,cell.Right+margin,cell.Top+margin):0;
                if(stationary?!(distanceSquared-originalRadiusSquared<-1e-12):!VCarveCutterBoxIntersection.Intersects(a,b,cutter,0,
                    cell.Left-margin,cell.Bottom-margin,cell.Right+margin,cell.Top+margin))
                continue;
                bool IntersectsAt(double depth)=>stationary
                    &&VCarveCutterBoxIntersection.TryPointRadiusSquared(cutter,-a.Z-depth,out double squared)
                    ?distanceSquared-squared<-1e-12
                    :VCarveCutterBoxIntersection.Intersects(a,b,cutter,depth,
                        cell.Left-margin,cell.Bottom-margin,cell.Right+margin,cell.Top+margin);
                // A parent's disk contains every child box. Its physical stock
                // lower bound, admitted wholly inside the Part, therefore also
                // holds throughout each child. Test the COMPLETE current cutter
                // against that inherited floor before rebuilding child facts.
                // Target permission is never inherited or enlarged.
                if(cell.FloorLower>0&&!IntersectsAt(cell.FloorLower))
                {InheritedStockCellProofs++;continue;}
                double x = (cell.Left + cell.Right) / 2, y = (cell.Bottom + cell.Top) / 2;
                double halfX = (cell.Right - cell.Left) / 2, halfY = (cell.Top - cell.Bottom) / 2;
                double coverRadius = Math.Sqrt(halfX * halfX + halfY * halfY);
                ref var facts = ref GetCellFacts(cell,x,y);
                // Whole-parent disk containment also proves the same strict
                // Part membership / target exclusion for each child disk.
                // Keep exact region distances lazy; these two inherited facts
                // cannot grant target permission or cross a Part boundary.
                bool inheritedPart=(cell.GeometryFlags&1)!=0;
                bool inheritedOutsideTarget=(cell.GeometryFlags&2)!=0;
                if(!inheritedPart&&double.IsNaN(facts.PartDistance))facts.PartDistance=part.SignedDistanceUncached(new(x,y));
                if(!clearedOnly&&!inheritedOutsideTarget&&double.IsNaN(facts.TargetDistance))facts.TargetDistance=target.SignedDistanceUncached(new(x,y));
                if(inheritedPart)InheritedRegionDistanceQueriesAvoided++;
                if(!clearedOnly&&inheritedOutsideTarget)InheritedRegionDistanceQueriesAvoided++;
                double partDistance = inheritedPart?double.PositiveInfinity:facts.PartDistance;
                double targetDistance = clearedOnly||inheritedOutsideTarget ? double.NegativeInfinity : facts.TargetDistance;
                bool inPart = partDistance >= coverRadius + PreparedRegionDistance.Uncertainty;
                if (inPart && !clearedOnly && targetDistance >= coverRadius + PreparedRegionDistance.Uncertainty)
                continue;
                if(inPart&&double.IsNaN(facts.Clearance)&&lastProfileWinner>=0)
                {
                    if(double.IsNaN(facts.SeedClearance))
                    {
                        // A literal prior pose can already cover this ENTIRE
                        // immutable stock disk. Re-evaluate its full cutter;
                        // no neighbouring height or target permission transfers.
                        SeededCellTrials++;
                        facts.SeedClearance=Math.Max(0,ProfileDepth(columns[lastProfileWinner],x,y,coverRadius,true));
                    }
                    if(facts.SeedClearance>0&&facts.SeedClearance!=cell.FloorLower&&!IntersectsAt(facts.SeedClearance))
                    {SeededCellProofs++;continue;}
                }
                double floor = Math.Max(cell.FloorLower,inPart ? CellClearance(ref facts,x,y,coverRadius,point:false) : 0);
                // The original-top test already proved intersection at zero;
                // a positive inherited floor likewise passed its complete test
                // above. Equal child clearance repeats that IDENTICAL pure
                // cutter/box predicate, including the original posting shell.
                bool knownIntersection=floor==cell.FloorLower;
                if(knownIntersection)ReusedCellIntersectionPredicates++;
                bool intersects=knownIntersection||IntersectsAt(floor);
                if(!intersects)continue;
                // A penetrating protected witness can refuse a nomination without
                // subdividing the rest of an already invalid cutter footprint.
                // Refusal grants no stock credit; the inner-column reserve remains.
                if (clearedOnly || targetDistance < -PreparedRegionDistance.Uncertainty
                    || partDistance < -PreparedRegionDistance.Uncertainty)
                {
                    double atPoint = partDistance >= PreparedRegionDistance.Uncertainty
                        ? CellClearance(ref facts,x,y,0,point:true) : 0;
                    if (VCarveCutterBoxIntersection.Intersects(a, b, cutter, atPoint,
                        x-margin,y-margin,x+margin,y+margin))
                    {
                        if(memoRefusals && partDistance>=PreparedRegionDistance.Uncertainty
                            && (clearedOnly||targetDistance < -PreparedRegionDistance.Uncertainty))
                        {
                            int previousWinner=lastProfileWinner;
                            // innerReserve:false uses independent cutter extrema
                            // from ABOVE, including variable-depth prior sweeps.
                            double upper=ClearedDepth(x,y,0,innerReserve:false)+PreparedRegionDistance.Uncertainty;
                            lastProfileWinner=previousWinner;
                            if(VCarveCutterBoxIntersection.Intersects(a,b,cutter,upper,
                                x-margin,y-margin,x+margin,y+margin))
                            {
                                refusalWitnesses[nextRefusalWitness]=new(x,y,upper,clearedOnly);
                                nextRefusalWitness=(nextRefusalWitness+1)%refusalWitnesses.Length;
                                refusalWitnessCount=Math.Min(refusalWitnessCount+1,refusalWitnesses.Length);
                                RefusalWitnessPublications++;
                            }
                        }
                        refusalDiagnostic?.Invoke($"reason=point-intersection x={x:R} y={y:R} removalLower={atPoint:R} cellLevel={cell.Level} margin={margin:R} stationary={stationary}");
                        return false;
                    }
                }
                if(cell.Level>=14||Math.Max(halfX,halfY)<.0005){Refusals++;refusalDiagnostic?.Invoke($"reason=unresolved-cell x={x:R} y={y:R} floor={floor:R} cellLevel={cell.Level} halfX={halfX:R} halfY={halfY:R} margin={margin:R} stationary={stationary}");return false;}
                int level = cell.Level + 1;
                double inheritedFloor=inheritCellClearance?floor:0;
                int flags=inheritCellClearance&&inPart?1:0;
                if(inheritCellClearance&&!clearedOnly&&targetDistance < -coverRadius-PreparedRegionDistance.Uncertainty)flags|=2;
                pending[count++] = new(cell.Left, cell.Bottom, x, y, level,flags,inheritedFloor);
                pending[count++] = new(x, cell.Bottom, cell.Right, y, level,flags,inheritedFloor);
                pending[count++] = new(cell.Left, y, x, cell.Top, level,flags,inheritedFloor);
                pending[count++] = new(x, y, cell.Right, cell.Top, level,flags,inheritedFloor);
            }
        }
        return true;

    }

    private ref CellFacts GetCellFacts(Cell cell,double x,double y)
    {
        CellFacts Fresh()=>new(){PartDistance=double.NaN,TargetDistance=double.NaN,
            Clearance=double.NaN,PointClearance=double.NaN,SeedClearance=double.NaN};
        double gridX=x*2048,gridY=y*2048;
        if(gridX<int.MinValue||gridX>int.MaxValue||gridY<int.MinValue||gridY>int.MaxValue)
        {
            // Do not narrow or alias coordinates outside the packed domain.
            // A fresh literal query retains the same proof without memoization.
            CellGeometryCacheMisses++;
            uncachedCellFacts=Fresh();return ref uncachedCellFacts;
        }
        var key=new CellKey(((ulong)(uint)(int)gridX<<32)|(uint)(int)gridY);
        ref var found=ref CollectionsMarshal.GetValueRefOrNullRef(stockCells,key);
        if(!Unsafe.IsNullRef(ref found)){CellGeometryCacheHits++;return ref found;}
        CellGeometryCacheMisses++;
        if(stockCells.Count==stockCellCacheLimit)
        {stockCells.Remove(stockCellOrder.Dequeue());StockCellCacheEvictions++;}
        ref var added=ref CollectionsMarshal.GetValueRefOrAddDefault(stockCells,key,out _);
        // Both immutable regions belong to this operation. Store literal signed
        // distances, never a permission decision: clearedOnly and the candidate
        // sweep must still apply their own original tests on every visit.
        added=Fresh();
        stockCellOrder.Enqueue(key);
        return ref added;
    }

    private double CellClearance(ref CellFacts facts,double x,double y,double radius,bool point)
    {
        ref double cached=ref (point?ref facts.PointClearance:ref facts.Clearance);
        if(!double.IsNaN(cached)){StockCellCacheHits++;return cached;}
        StockCellCacheMisses++;
        return cached=ClearedDepth(x,y,radius);
    }
}
