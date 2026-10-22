using Clipper2Lib;
using RW.Toolpaths.Geometry;
using System.Diagnostics;

namespace RW.Toolpaths.Milling.Certification;

/// <summary>
/// Optional original-source terminal coverage proof. Candidate Booleans and
/// decomposition are untrusted choices: exact signed-chain support certifies
/// every excluded source point, and full capsules plus enclosed-hole checks
/// certify the complete reachable cutter footprint. Uncertainty keeps the
/// established full original diagram. No candidate front supplies cut credit.
/// </summary>
internal static class ExactFirstCoverageGate
{
    internal const double CoverageReserve = .003 + 4 / PathUtils.Scale;
    private const int MinimumSourcePoints = 200_000;
    private const int MaximumSourcePoints = 400_000;
    private const int MaximumSidePoints = 3_000_000;
    private const int MaximumComponentNodes = 65_536;
    private const double MaximumMilliseconds = 10_000;
    internal const int SourceRadiusFractionBits = 16;
    internal const double TraceDistanceArithmeticReserveTicks = 1.0 / 16_384;
    private const double FirstProposalSimplify = .001;
    private const double FirstProposalExpansion = .0015;
    private const double OuterProposalSimplify = .00025;
    private const double OuterProposalArc = .00025;
    private const double OuterProposalExtraRadius = .0005;
    private const double MaximumOuterAidHoleArea = .00001;

    internal sealed class Diagnostics
    {
        internal bool Certified;
        internal bool GapComputed;
        internal bool PhysicalCertified;
        internal bool DeadlineExceededAfterCompletedProof;
        internal double MaximumBudgetMilliseconds = MaximumMilliseconds;
        internal string Reason = "not-started";
        internal string ActiveStage = "input";
        internal int SourcePoints, SidePoints, CenterPoints, FootprintPoints;
        internal int ClosedRings, OpenRings, PrunedComponents, RetainedComponents, ComponentNodes;
        internal int PhysicalChords, EnclosedFootprintVertices;
        internal int FirstProposalInputPoints, FirstProposalSimplifiedPoints, OuterProposalInputPoints, RemovedOuterAidHoles;
        internal int UnresolvedHulls, RetainedHulls, HullCuts, HullRefusals, GapPaths;
        internal int SourceBooleanAidPoints;
        internal long NominationQueries, NominationMemoHits;
        internal double HullComplementMilliseconds;
        internal long AidSimplifyComparisons;
        internal string ProposalContract = "not-started";
        internal int StrictSourceRadiusFractionBits;
        internal long StrictSourceRadius;
        internal double CandidateMilliseconds, SourceCertificateMilliseconds,
            PhysicalMilliseconds, AcceptedSweepMilliseconds, GapMilliseconds, TotalMilliseconds;
        internal SignedBoundaryCapsuleCertificate.Diagnostics? SourceCertificate;
    }

    internal static bool Enabled(RingOffsetOptions options)
        => options.ExactFirstCoverageModeCaptured && options.ExactFirstCoverageProposals;

    internal static bool TryCertify(Paths64 canonicalSource, PolygonOps.BoundarySegmentIndex original,
        IReadOnlyList<OffsetRing> acceptedRings, int regionIndex, double firstInset,
        double cutterRadius, IReadOnlyList<PointD>? clipBoundary, CancellationToken token,
        out Diagnostics diagnostics)
    {
        token.ThrowIfCancellationRequested();
        var result = new Diagnostics(); diagnostics = result;
        var total = Stopwatch.StartNew();
        bool Fail(string reason) { result.Reason = reason; result.TotalMilliseconds = total.Elapsed.TotalMilliseconds; return false; }
        void CheckBudget()
        {
            token.ThrowIfCancellationRequested();
            if (total.Elapsed.TotalMilliseconds > MaximumMilliseconds) throw new GateBudgetException();
        }
        long sourcePoints = canonicalSource.Sum(path => (long)path.Count);
        if (sourcePoints <= MaximumSourcePoints) result.SourcePoints = (int)sourcePoints;
        if (!double.IsFinite(firstInset) || !double.IsFinite(cutterRadius) || cutterRadius <= 0
            || firstInset * PathUtils.Scale <= 20 || firstInset * PathUtils.Scale >= 1L << 28
            || sourcePoints < MinimumSourcePoints || sourcePoints > MaximumSourcePoints
            || canonicalSource.SelectMany(path => path).Any(p => !InRange(p)))
            return Fail("bounded-input-or-source-below-dense-work-gate");
        var rings = acceptedRings.Where(ring => ring.RegionIndex == regionIndex).ToArray();
        result.ClosedRings = rings.Count(ring => ring.IsClosed); result.OpenRings = rings.Length-result.ClosedRings;
        if (rings.Length == 0 || rings.Length > 4096 || rings.Sum(r => (long)r.Points.Count) > 500_000)
            return Fail("accepted-evidence-count-bound");
        try
        {
            CheckBudget(); var timer = Stopwatch.StartNew();
            result.ActiveStage = "candidate";
            var centers = ProposeFirstCenters(rings, result, CheckBudget);
            result.CandidateMilliseconds = timer.Elapsed.TotalMilliseconds; CheckBudget();
            if (centers.Count == 0) return Fail("no-first-closed-proposal");
            var choice = new PreparedFractionalSiteChoice(original);
            result.StrictSourceRadius = StrictSourceRadiusForTraceNative(firstInset);
            result.StrictSourceRadiusFractionBits = SourceRadiusFractionBits;
            var partition = new SignedBoundaryCapsuleCertificate.PartitionPolicy();
            var signed = new Paths64(canonicalSource);
            foreach (var path in centers) { var reversed = new Path64(path); reversed.Reverse(); signed.Add(reversed); }
            timer.Restart(); result.ActiveStage = "source-support";
            // The rounded Boolean supplies ONLY the decomposition nomination.
            // The complete literal signed SOURCE-CENTER chain above remains
            // the authority for every main/correction triangle and enclosure.
            var aidEngine = new Clipper64(); aidEngine.AddSubject(canonicalSource); aidEngine.AddClip(centers);
            var canonicalAid = new Paths64();
            if (!aidEngine.Execute(ClipType.Difference, FillRule.NonZero, canonicalAid))
                return Fail("untrusted-source-boolean-aid-refused");
            BoundPoints(canonicalAid,CheckBudget); result.SourceBooleanAidPoints=canonicalAid.Sum(p=>p.Count);
            var sourceProof = new SignedBoundaryCapsuleCertificate.Diagnostics();
            try
            {
                SignedBoundaryCapsuleCertificate.TryCertify(signed, choice.Choose, result.StrictSourceRadius,
                    token, out sourceProof, workCheckpoint: CheckBudget, originalBoundary: canonicalSource,
                    repairIntersections: false, radiusFractionBits: SourceRadiusFractionBits,
                    partition: partition, fractionalOriginalSiteChoice: choice.Choose,
                    triangulationAidBoundary: canonicalAid);
            }
            finally
            {
                result.SourceCertificate = sourceProof; result.SourceCertificateMilliseconds = timer.Elapsed.TotalMilliseconds;
                result.NominationQueries = choice.Queries; result.NominationMemoHits = choice.MemoHits;
                Console.WriteLine($"[Offset.Terminal.ExactFirstGate.SourceSupport] sourcePoints={result.SourcePoints} centerPoints={result.CenterPoints} "
                    + $"proposal={result.ProposalContract} radiusNumerator={result.StrictSourceRadius} radiusFractionBits={SourceRadiusFractionBits} "
                    + $"certified={sourceProof.Certified} partitionCertified={sourceProof.SupportOutsideEnclosuresCertified} reason={sourceProof.Reason} "
                    + $"mainTriangles={sourceProof.MainTriangles} corrections={sourceProof.CorrectionTriangles} finalResidualEdges={sourceProof.FinalResidualEdges} "
                    + $"booleanAidPoints={result.SourceBooleanAidPoints} "
                    + $"tasks={sourceProof.TriangleTasks} enclosures={partition.Enclosures.Count} ownersEnforced={sourceProof.OriginalOwnerRegistryEnforced} "
                    + $"originalEdges={sourceProof.OriginalSiteRegistryEdges} inheritedCertified={sourceProof.InheritedSiteCertificates} "
                    + $"bigInteger={sourceProof.BigIntegerFallbacks} nominationQueries={choice.Queries} memoHits={choice.MemoHits} "
                    + $"triangulationMs={sourceProof.TriangulationMilliseconds:F3} chainMs={sourceProof.ChainMilliseconds:F3} "
                    + $"capsuleMs={sourceProof.CapsuleMilliseconds:F3} totalMs={result.SourceCertificateMilliseconds:F3}");
            }
            if (sourceProof.FinalResidualEdges != 0 || !sourceProof.OriginalOwnerRegistryEnforced
                || (!sourceProof.Certified && !sourceProof.SupportOutsideEnclosuresCertified))
                return Fail("source-center-" + sourceProof.Reason);
            CheckBudget(); timer.Restart(); result.ActiveStage = "exact-enclosure-complements";
            var owners = new HashSet<(Point64 Start, Point64 End)>();
            foreach (var path in canonicalSource) for (int i=0;i<path.Count;i++)
            {
                if ((i & 1023) == 0) CheckBudget();
                Point64 a=path[i], b=path[(i+1)%path.Count];
                if(a!=b)owners.Add(OwnerKey(a,b));
            }
            var retained = new Paths64(); var stats = new OriginalHullComplement.Statistics();
            result.UnresolvedHulls = partition.Enclosures.Count;
            foreach (var hull in partition.Enclosures)
            {
                CheckBudget(); var nominated = new List<OriginalHullComplement.Site>(4);
                void Nominate(double x,double y)
                { var s=choice.Choose(x,y);var site=new OriginalHullComplement.Site(s.Start,s.End);if(nominated.Count<4&&!nominated.Contains(site))nominated.Add(site); }
                Nominate(hull.Average(p=>(double)p.X),hull.Average(p=>(double)p.Y));
                foreach(var p in hull){if(nominated.Count>=4)break;Nominate(p.X,p.Y);}
                bool proved = OriginalHullComplement.TryTrim(hull,nominated,
                    s=>owners.Contains(OwnerKey(s.Start,s.End)),result.StrictSourceRadius,SourceRadiusFractionBits,
                    token,CheckBudget,stats,out var remainder);
                // A rejected nomination removes nothing. EVERY exact support
                // triangle remains represented by its entire outward hull.
                if(proved)retained.AddRange(remainder);else retained.Add(hull);
                if(retained.Count>4096||retained.Sum(p=>(long)p.Count)>131_072)throw new GateBudgetException();
            }
            result.RetainedHulls=retained.Count; result.HullCuts=stats.Cuts; result.HullRefusals=stats.Refused;
            result.HullComplementMilliseconds=timer.Elapsed.TotalMilliseconds; CheckBudget();
            timer.Restart(); result.ActiveStage="physical-enclosure";
            var proposedUnion=Clipper.Union(centers,retained,FillRule.NonZero);
            BoundPoints(proposedUnion,CheckBudget);
            var proposed = SimplifyAid(proposedUnion,.00005,false,result,CheckBudget);
            var offsetter = new ClipperOffset(miterLimit:2,arcTolerance:.00005*PathUtils.Scale,preserveCollinear:false,reverseSolution:false);
            offsetter.AddPaths(proposed,JoinType.Round,EndType.Polygon);var outer = new Paths64();
            offsetter.Execute((cutterRadius+.00012)*PathUtils.Scale+4,outer);BoundPoints(outer,CheckBudget);
            var footprint=PathUtils.FromClipper(outer);
            if(clipBoundary is {Count:>=3})
            {
                if(!TryCheckedWorldBoolean(ClipType.Intersection,footprint,new[]{clipBoundary},out var clippedFootprint))
                    return Fail("physical-footprint-clip-refused");
                footprint=clippedFootprint;
            }
            var footprintNative=PathUtils.ToClipper(footprint);BoundPoints(footprintNative,CheckBudget);
            result.FootprintPoints=footprintNative.Sum(p=>p.Count);
            bool physical=PotentialCenterFootprint.Certify(centers,retained,footprintNative,cutterRadius,token,CheckBudget,
                out int chords,out int enclosed);
            result.PhysicalCertified=physical;
            result.PhysicalChords=chords;result.EnclosedFootprintVertices=enclosed;
            result.PhysicalMilliseconds=timer.Elapsed.TotalMilliseconds;CheckBudget();
            if(!physical)return Fail("physical-outer-capsule-or-enclosed-hole");
            timer.Restart();result.ActiveStage="accepted-sweep";
            var closed=rings.Where(r=>r.IsClosed).Select(r=>(IReadOnlyList<PointD>)r.Points);
            var open=rings.Where(r=>!r.IsClosed).Select(r=>(IReadOnlyList<PointD>)r.Points).ToList();
            var closedSweep=RingOffsetEngine.BufferClosedSweeps(closed,cutterRadius,.001,token,explicitChords:true,clipBoundary);
            if(clipBoundary is {Count:>=3}&&open.Count>0)
            {
                var engine=new Clipper64();engine.AddOpenSubject(PathUtils.ToClipper(open));engine.AddClip(PathUtils.ToClipper(new[]{clipBoundary}));
                var clipped=new Paths64();if(!engine.Execute(ClipType.Intersection,FillRule.NonZero,new Paths64(),clipped))
                    return Fail("accepted-open-clip-refused");
                open=PathUtils.FromClipper(clipped).Select(p=>(IReadOnlyList<PointD>)p).ToList();
            }
            if(!TryCheckedWorldBoolean(ClipType.Union,
                closedSweep.Concat(PolygonOps.BufferOpenCenterlines(open,cutterRadius,.001,token)),null,out var sweep))
                return Fail("accepted-sweep-union-refused");
            result.AcceptedSweepMilliseconds=timer.Elapsed.TotalMilliseconds;CheckBudget();
            timer.Restart();result.ActiveStage="gap";
            if(!TryCheckedWorldBoolean(ClipType.Difference,footprint,sweep,out var residual))
                return Fail("authoritative-coverage-subtraction-refused");
            var gap=PolygonOps.Offset(residual,-CoverageReserve,.00025,0);
            result.GapMilliseconds=timer.Elapsed.TotalMilliseconds;
            result.GapPaths=gap.Count;result.GapComputed=true;
            token.ThrowIfCancellationRequested();
            if(gap.Count!=0)return Fail("unchanged-native-whole-coverage-gap");
            if(!CompleteComputedProof(result,token,total.Elapsed.TotalMilliseconds))
                return Fail("incomplete-final-proof-evidence");
            if(result.DeadlineExceededAfterCompletedProof)
                Console.WriteLine($"[Offset.Terminal.ExactFirstGate.CompletedProof] certified={result.Certified} "
                    + $"physicalCertified={result.PhysicalCertified} gapComputed={result.GapComputed} gapPaths={result.GapPaths} "
                    + $"completedProofAfterDeadline=true totalMs={result.TotalMilliseconds:F3} budgetMs={MaximumMilliseconds:F3}");
            return true;
        }
        catch(GateBudgetException){return Fail("gate-time-or-work-budget-"+result.ActiveStage);}
        catch(OverflowException){return Fail("candidate-integer-bound");}
        catch(InvalidOperationException){return Fail("untrusted-candidate-invariant");}
        catch(InsufficientExecutionStackException){return Fail("candidate-stack-budget");}
    }

    // The final Boolean/offset kernel is cooperative and can finish after the
    // proposal deadline. Once every authoritative proof has completed, time
    // cannot invalidate that evidence. Keep all earlier work-budget checks;
    // admitting only this completed state avoids an unnecessary full diagram.
    internal static bool CompleteComputedProof(Diagnostics result, CancellationToken token,
        double elapsedMilliseconds)
    {
        token.ThrowIfCancellationRequested();
        var source=result.SourceCertificate;
        if(!double.IsFinite(elapsedMilliseconds)||elapsedMilliseconds<0
            ||!result.GapComputed||result.GapPaths!=0||!result.PhysicalCertified
            ||source is null||source.FinalResidualEdges!=0||!source.OriginalOwnerRegistryEnforced
            ||(!source.Certified&&!source.SupportOutsideEnclosuresCertified))
            return false;
        result.Certified=true;
        result.Reason="complete-original-reachable-stock-covered";
        result.TotalMilliseconds=elapsedMilliseconds;
        result.DeadlineExceededAfterCompletedProof=elapsedMilliseconds>MaximumMilliseconds;
        return true;
    }

    private static bool InRange(Point64 p)=>Math.Abs((double)p.X)<1L<<30&&Math.Abs((double)p.Y)<1L<<30;
    private static (Point64 Start,Point64 End) OwnerKey(Point64 a,Point64 b)
        =>a.X<b.X||(a.X==b.X&&a.Y<b.Y)?(a,b):(b,a);
    private static void BoundPoints(Paths64 paths,Action checkpoint)
    { checkpoint();if(paths.Sum(p=>(long)p.Count)>500_000||paths.SelectMany(p=>p).Any(p=>!InRange(p)))throw new GateBudgetException(); }

    // Same integer inputs, filtering, normalization and empty-input identities
    // as PolygonOps' world Boolean helpers, with an explicit success premise.
    // The executor seam is internal and used only by refusal controls; live
    // generation always executes the original Clipper64 operation directly.
    internal static bool TryCheckedWorldBoolean(ClipType operation,
        IEnumerable<IReadOnlyList<PointD>> subjects, IEnumerable<IReadOnlyList<PointD>>? clips,
        out List<List<PointD>> output,
        Func<Clipper64,ClipType,FillRule,Paths64,bool>? executor=null)
    {
        output=new();
        var subjectPaths=PathUtils.ToClipper(subjects.Where(p=>p.Count>=3));
        var clipPaths=clips is null?null:PathUtils.ToClipper(clips.Where(p=>p.Count>=3));
        if(subjectPaths.Count==0 || operation==ClipType.Intersection&&(clipPaths is null||clipPaths.Count==0))
            return true;
        if(operation==ClipType.Difference&&(clipPaths is null||clipPaths.Count==0))
        {output=PolygonOps.MaterializeIntegerPaths(subjectPaths);return true;}
        var engine=new Clipper64();engine.AddSubject(subjectPaths);
        if(clipPaths is not null)engine.AddClip(clipPaths);
        var native=new Paths64();
        bool success=executor is null?engine.Execute(operation,FillRule.NonZero,native)
            :executor(engine,operation,FillRule.NonZero,native);
        if(!success)return false;
        output=PolygonOps.NormalizeOrder(PolygonOps.MaterializeIntegerPaths(native));
        return true;
    }

    private static Paths64 ProposeFirstCenters(IReadOnlyList<OffsetRing> rings,Diagnostics result,Action checkpoint)
    {
        var first=new Paths64();
        foreach(var ring in rings.Where(r=>r.Level==0&&r.IsClosed))
        {
            checkpoint();var native=PathUtils.ToClipper(new[]{(IReadOnlyList<PointD>)ring.Points})[0];
            if(ring.IsHole)native.Reverse();first.Add(native);result.FirstProposalInputPoints+=native.Count;
            if(result.FirstProposalInputPoints>MaximumSourcePoints)throw new GateBudgetException();
        }
        result.ProposalContract="untrusted-first-finite-rdp-.00025-expand-.0015-full-source-partition";
        first=SimplifyAid(first,.00025,true,result,checkpoint);result.FirstProposalSimplifiedPoints=first.Sum(p=>p.Count);
        var centers=PolygonOps.BuildOuterCandidateForCertificate(first,.0015);BoundPoints(centers,checkpoint);
        result.CenterPoints=centers.Sum(p=>p.Count);return centers;
    }

    private static Paths64 SimplifyAid(Paths64 paths,double epsilon,bool finite,Diagnostics result,Action checkpoint)
    {
        var output=new Paths64();
        foreach(var path in paths)
        {
            checkpoint();var points=PathUtils.FromClipper(new Paths64{path})[0];int count=points.Count;
            if(count<4){output.Add(path);continue;}
            var keep=new bool[count];keep[0]=true;keep[count-1]=true;var stack=new Stack<(int Start,int End)>();stack.Push((0,count-1));
            while(stack.TryPop(out var span))
            {
                if(span.End-span.Start<2)continue;
                double ax=points[span.Start].x,ay=points[span.Start].y,dx=points[span.End].x-ax,dy=points[span.End].y-ay;
                double lengthSquared=dx*dx+dy*dy,length=Math.Sqrt(lengthSquared),maximum=0;int farthest=span.Start;
                for(int i=span.Start+1;i<span.End;i++)
                {
                    if((++result.AidSimplifyComparisons&1023)==0)checkpoint();
                    if(result.AidSimplifyComparisons>20_000_000)throw new GateBudgetException();
                    double px=points[i].x-ax,py=points[i].y-ay,distance;
                    if(finite){double t=lengthSquared<1e-18?0:Math.Clamp((px*dx+py*dy)/lengthSquared,0,1);double ex=px-t*dx,ey=py-t*dy;distance=Math.Sqrt(ex*ex+ey*ey);}
                    else distance=lengthSquared<1e-18?Math.Sqrt(px*px+py*py):Math.Abs(px*dy-py*dx)/length;
                    if(distance>maximum){maximum=distance;farthest=i;}
                }
                if(maximum>epsilon){keep[farthest]=true;stack.Push((span.Start,farthest));stack.Push((farthest,span.End));}
            }
            var compact=new Path64();for(int i=0;i<count;i++)if(keep[i])compact.Add(path[i]);
            output.Add(compact.Count>=3&&Clipper.Area(path)*Clipper.Area(compact)>0?compact:path);
        }
        return output;
    }

    internal static bool CertifyCenterEnclosure(Paths64 source, Paths64 centers,
        PolygonOps.BoundarySegmentIndex original, long strictRadius, CancellationToken token,
        out SignedBoundaryCapsuleCertificate.Diagnostics diagnostics, Action? workCheckpoint = null, int radiusFractionBits = 0)
    {
        var complementBoundary = new Paths64(source);
        foreach (var path in centers) { var reverse = new Path64(path); reverse.Reverse(); complementBoundary.Add(reverse); }
        return SignedBoundaryCapsuleCertificate.TryCertify(complementBoundary,
            original.NearestSegmentForCertificate, strictRadius, token, out diagnostics,
            workCheckpoint: workCheckpoint, originalBoundary: source, radiusFractionBits: radiusFractionBits);
    }

    internal static long StrictSourceRadiusForTraceNative(double firstInset)
    {
        double nativeRadius = firstInset * PathUtils.Scale;
        if (!double.IsFinite(nativeRadius) || nativeRadius <= 20 || nativeRadius >= 1L << 28)
            throw new ArgumentOutOfRangeException(nameof(firstInset));
        // Power-of-two scaling is exact in this bounded range. Outward
        // decrement also encloses the threshold/subtraction rounding. Ceil-1
        // guarantees a strict dyadic radius even for integral thresholds.
        double lower = Math.BitDecrement(Math.BitDecrement(nativeRadius - 4)
            - TraceDistanceArithmeticReserveTicks);
        return checked((long)Math.Ceiling(Math.BitDecrement(lower * (1L << SourceRadiusFractionBits))) - 1);
    }

    internal static bool CertifyPhysicalEnclosure(Paths64 centers, Paths64 footprint, double cutterRadius,
        CancellationToken token, out int physicalChords, out int enclosedFootprintVertices)
        => CertifyPhysicalEnclosureBounded(centers, footprint, cutterRadius, token,
            out physicalChords, out enclosedFootprintVertices, null);

    private static bool CertifyPhysicalEnclosureBounded(Paths64 centers, Paths64 footprint, double cutterRadius,
        CancellationToken token, out int physicalChords, out int enclosedFootprintVertices, Action? checkpoint)
    {
        token.ThrowIfCancellationRequested();
        physicalChords = 0; enclosedFootprintVertices = 0;
        if (!double.IsFinite(cutterRadius) || cutterRadius <= 0 || footprint.Count == 0
            || centers.Concat(footprint).SelectMany(path => path).Any(p => Math.Abs((double)p.X) >= 1L << 30
                || Math.Abs((double)p.Y) >= 1L << 30)) return false;
        var centerIndex = new PolygonOps.BoundarySegmentIndex(centers);
        var stockIndex = new PolygonOps.BoundarySegmentIndex(footprint);
        // ContainsCapsule adds four native ticks. The net requirement is the
        // complete physical radius PLUS one arithmetic guard tick. This is an
        // outward enclosure requirement, never an extra wall/stock allowance.
        // With abs(integer coordinates)<2^30, integer differences<2^31
        // are exact and nonzero squared lengths >=1. Cancellation-aware dot
        // error and the 1-Lipschitz clamped projection give a conservative
        // distance error <128*2^-53*2^31*sqrt(2)<2^-14 native ticks.
        double radiusArgument = cutterRadius * PathUtils.Scale - 3;
        foreach (var ring in centers)
        for (int i = 0; i < ring.Count; i++)
        {
            if ((++physicalChords & 255) == 0) { token.ThrowIfCancellationRequested(); checkpoint?.Invoke(); }
            if (!stockIndex.ContainsCapsule(ring[i], ring[(i + 1) % ring.Count], radiusArgument)) return false;
        }
        int work = 0;
        foreach (var ring in footprint)
        foreach (var vertex in ring)
        {
            if ((++work & 255) == 0) { token.ThrowIfCancellationRequested(); checkpoint?.Invoke(); }
            if (centerIndex.IsInside(vertex)) enclosedFootprintVertices++;
        }
        // Full boundary capsules prohibit crossings. This separate test blocks
        // an entire protected hole enclosed by a filled center component. The
        // filled center plus its boundary capsules is its complete Minkowski
        // cutter footprint, including cutter discs inside the center fill.
        return enclosedFootprintVertices == 0;
    }

    private sealed class GateBudgetException : Exception { }
}
