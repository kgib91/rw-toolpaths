using System.Diagnostics;
using Clipper2Lib;
using RW.Toolpaths.Geometry;

namespace RW.Toolpaths.Milling;

/// <summary>Actual preceding posted cutter columns, in stock-relative coordinates.
/// Floor depths nominate slices only; a slice earns clearance exclusively from
/// chords whose two ends reached it. PartBoundary bounds the physical platform.</summary>
public sealed record VCarveClearedStock(
    IReadOnlyList<VCarveRoughingChord> Chords,
    IReadOnlyList<double> FloorDepths,
    IReadOnlyList<PointD> PartBoundary,
    IReadOnlyList<VCarveClearedSweep>? ProfileSweeps = null);

/// <summary>Actual earlier posted motion of a complete monotone physical cutter,
/// including rounded V-carvers that cannot supply a full tip-radius column.</summary>
public sealed record VCarveClearedSweep(VCarveCutterProfile Profile, IReadOnlyList<VCarveRoughingChord> Chords);

/// <summary>Height-dependent cutter fit against remaining stock. The original
/// target still owns every medial XY branch. Unions here describe clearance at
/// particular heights, never geometry from which to generate a new medial axis.</summary>
internal sealed class VCarveMaterialSurface
{
    private sealed record Layer(double TopDepth, PreparedRegionDistance Distance, PreparedRegionDistance ClearedDistance,
        MedialAxisToolpaths.SpatialSegmentIndex Boundary);
    private readonly List<Layer> layers = new();
    private readonly VCarveCutterProfile profile;
    private readonly VCarveProjectedStock projectedStock;
    private readonly PreparedRegionDistance originalTarget;
    private readonly double maximumDepth, surface;
    private readonly CancellationToken token;
    private readonly Action<string>? diagnostic;
    private long sourcePoints, outputPoints, loweredPoints, chordChecks, subdivisions, retreats, refusedSources, shortRefusals, fittedRetreats;
    private double largestDescent;
    private long shoulderTrials, shoulderMovedSites, shoulderUnreachableSites;
    private long shoulderInwardCorrections;
    private long verticalSweepsReduced;
    private long shoulderRunJoins, shoulderJoinRefusals;
    private long shoulderUnusedMidpointFitsAvoided;
    private long raisedSourceProofs, qualifiedSourceChords;
    internal long RaisedSourceProofs => raisedSourceProofs;
    private readonly Dictionary<(double, double, double, double), List<(double A, double B)>> qualifiedSources = new();
    private readonly Dictionary<(double X, double Y), double> shoulderSurfaceDepths = new();
    // Accounts for F3 planar posting and integer boundary construction. Every
    // resulting chord is additionally proved with the complete physical profile.
    internal const double PlanarReserve = .002;

    internal VCarveMaterialSurface(IEnumerable<IReadOnlyList<PointD>> target,
        VCarveClearedStock stock, VCarveCutterProfile profile, double maximumDepth,
        double surface, CancellationToken token, Action<string>? diagnostic)
    {
        this.profile = profile; this.maximumDepth = maximumDepth; this.surface = surface;
        this.token = token; this.diagnostic = diagnostic;
        long started = Stopwatch.GetTimestamp();
        var original = Clipper.Union(PathUtils.ToClipper(target), FillRule.NonZero);
        originalTarget = new PreparedRegionDistance(PathUtils.FromClipper(original));
        if (stock.PartBoundary.Count < 3 || stock.PartBoundary.Any(p => !double.IsFinite(p.x + p.y)))
            throw new ArgumentException("Cleared V-carve stock requires a finite physical Part boundary.", nameof(stock));
        var part = PathUtils.ToClipper(new[] { stock.PartBoundary });
        bool Posted(Point3D p) => double.IsFinite(p.X + p.Y + p.Z)
            && Math.Abs(p.X) <= 10000 && Math.Abs(p.Y) <= 10000 && Math.Abs(p.Z) <= 100
            && p.X == Math.Round(p.X * 1000) / 1000d && p.Y == Math.Round(p.Y * 1000) / 1000d
            && p.Z == Math.Round(p.Z * 1000) / 1000d;
        var executed = stock.Chords.Where(c => Posted(c.A) && Posted(c.B)
            && c.ClearedColumnRadius > 0 && double.IsFinite(c.ClearedColumnRadius)).ToArray();
        projectedStock = new(executed, PathUtils.FromClipper(original), stock.PartBoundary,
            profile, maximumDepth, token, stock.ProfileSweeps,diagnostic);
        diagnostic?.Invoke($"stage=material-surface event=continuous-index flatChords={executed.Length} "
            +$"profileChords={projectedStock.ProfileChords} indexedChords={projectedStock.IndexedChords} "
            +"compaction=literal-F3-directed-collinearity-and-same-trace-depth-dominance unionGeometryPreserved=True");
        // Coalescing shallow slices into a deeper one is conservative: fewer
        // executed columns qualify. Keep work bounded for long operation lists.
        double[] floors = stock.FloorDepths.Where(double.IsFinite)
            .Select(d => Math.Min(maximumDepth, Math.Floor(d * 1000 + 1e-7) / 1000))
            .Where(d => d > 0).Append(maximumDepth).Distinct().Order().TakeLast(8).ToArray();
        double top = 0;
        foreach (double floor in floors)
        {
            token.ThrowIfCancellationRequested();
            var swept = new Paths64();
            int credited = 0;
            foreach (var group in executed.Where(c => Math.Max(c.A.Z, c.B.Z) <= -floor + 1e-9)
                .GroupBy(c => c.ClearedColumnRadius))
            {
                var columns = group.ToArray();
                credited += columns.Length;
                var segments = columns.Select(c => new PocketPostedChordChains.Segment(
                    new((long)Math.Round(c.A.X * 1000), (long)Math.Round(c.A.Y * 1000)),
                    new((long)Math.Round(c.B.X * 1000), (long)Math.Round(c.B.Y * 1000)))).ToArray();
                var chains = PocketPostedChordChains.Build(segments, token);
                var paths = new Paths64(chains.Paths.Select(path => new Path64(path.Select(p =>
                    new Point64((long)Math.Round(p.X / 1000d * PathUtils.Scale),
                        (long)Math.Round(p.Y / 1000d * PathUtils.Scale))))));
                paths.AddRange(columns.Where(c => c.A.X == c.B.X && c.A.Y == c.B.Y)
                    .Select(c => new Path64 { new((long)Math.Round(c.A.X * PathUtils.Scale),
                        (long)Math.Round(c.A.Y * PathUtils.Scale)) }));
                if (paths.Count > 0)
                    swept.AddRange(PostedSweepInnerRegion.BuildPaths(paths, group.Key, token));
            }
            swept = Clipper.Intersect(Clipper.Union(swept, FillRule.NonZero), part, FillRule.NonZero);
            var free = Clipper.Intersect(Clipper.Union(new Paths64(original.Concat(swept)), FillRule.NonZero), part, FillRule.NonZero);
            var rings = PathUtils.FromClipper(free).Select(r => (IReadOnlyList<PointD>)r).ToList();
            var edges = rings.SelectMany(ring => Enumerable.Range(0, ring.Count)
                .Select(i => (ring[i], ring[(i + 1) % ring.Count]))).ToList();
            layers.Add(new(top, new PreparedRegionDistance(rings), new PreparedRegionDistance(PathUtils.FromClipper(swept)),
                new MedialAxisToolpaths.SpatialSegmentIndex(edges)));
            diagnostic?.Invoke($"stage=material-surface layerTopDepth={top:R} layerBottomDepth={floor:R} "
                + $"postedColumnChords={credited} boundaryEdges={edges.Count} "
                + "targetBranchesUnchanged=True credit=entire-executed-column platformClipped=True");
            top = floor;
        }
        diagnostic?.Invoke($"stage=material-surface event=prepared layers={layers.Count} "
            + $"elapsedMs={Stopwatch.GetElapsedTime(started).TotalMilliseconds:F1} "
            + $"receivedChords={stock.Chords.Count} admittedPostedColumns={executed.Length} "
            + $"finalTipDepth={maximumDepth:R} startingDepthIsNotGlobalCutterDatum=True");
    }

    internal double AllowedDepth(double x, double y)
        => AllowedDepth(x, y, clearedOnly: false);

    private double AllowedDepth(double x, double y, bool clearedOnly)
    {
        double result = maximumDepth, unreserved = maximumDepth;
        PreparedRegionDistance? limiting = null;
        double limitingClearance = 0;
        foreach (var layer in layers)
        {
            double clearance = (clearedOnly ? layer.ClearedDistance : layer.Distance).SignedDistance(new(x, y));
            // Outside a layer, the tip may reach its upper plane but cannot
            // enter the protected material below that plane.
            double penetration = clearance <= PlanarReserve ? 0
                : profile.DepthAtRadius(clearance - PlanarReserve);
            result = Math.Min(result, layer.TopDepth + penetration);
            double upper = layer.TopDepth + (clearance <= 0 ? 0
                : profile.DepthAtRadius(clearance + PreparedRegionDistance.Uncertainty));
            if (upper < unreserved)
            {
                unreserved = upper; limiting = clearedOnly ? layer.ClearedDistance : layer.Distance;
                limitingClearance = clearance;
            }
        }
        PointD? witness = result < maximumDepth && limiting is not null
            ? limitingClearance <= 0 ? new PointD(x,y) : limiting.NearestBoundaryPoint(new(x,y)) : null;
        return projectedStock.RefineDepth(x, y, result, unreserved, clearedOnly, witness);
    }

    internal double EntryHeight(Point3D absolutePoint)
    {
        var command = Command(new(absolutePoint.X, absolutePoint.Y, absolutePoint.Z - surface));
        double clearedDepth = Math.Min(AllowedDepth(absolutePoint.X, absolutePoint.Y, true),
            AllowedDepth(command.X, command.Y, true));
        return surface - Math.Max(0, clearedDepth - .001);
    }

    private Point3D Command(Point3D p) => new(VCarveCommandRounding.Round((float)p.X),
        VCarveCommandRounding.Round((float)p.Y),
        VCarveCommandRounding.Round((float)(surface + p.Z) - (float)surface));

    private Point3D Fit(Point3D source, bool preserveDepthCap = false)
    {
        var command = Command(source);
        double cap = preserveDepthCap ? Math.Min(maximumDepth, Math.Max(0, -source.Z)) : maximumDepth;
        double depth = Math.Min(cap, Math.Min(AllowedDepth(source.X, source.Y), AllowedDepth(command.X, command.Y)));
        // Reserve axial posting error; never exceed the authored absolute cap.
        depth = depth >= cap ? cap : Math.Max(0, depth - .001);
        var fitted = new Point3D(source.X, source.Y, -depth);
        if (!IsLegalChord(fitted, fitted))
        {
            if (++fittedRetreats <= 8)
                diagnostic?.Invoke($"stage=material-surface event=fit-retreat point={fitted} commanded={Command(fitted)} "
                    + "layers=" + string.Join(";", layers.Select(l => $"{l.TopDepth:R}:{l.Distance.SignedDistance(new(fitted.X,fitted.Y)):R}/{l.Boundary.QueryDistance(fitted.X,fitted.Y):R}")));
            double low=0, high=depth;
            for(int i=0;i<20;i++)
            {
                double candidate=(low+high)/2;
                var point=new Point3D(source.X,source.Y,-candidate);
                if(IsLegalChord(point,point))low=candidate;else high=candidate;
            }
            fitted=new(source.X,source.Y,-low);
        }
        if (-fitted.Z > -source.Z + .002) { loweredPoints++; largestDescent = Math.Max(largestDescent, source.Z - fitted.Z); }
        return fitted;
    }

    internal bool IsLegalChord(Point3D a, Point3D b, bool postingReserve = true)
    {
        chordChecks++;
        return Legal(a, b, postingReserve) && Legal(Command(a), Command(b), postingReserve);
    }

    private bool Legal(Point3D a, Point3D b, bool postingReserve = true)
    {
        // For the supported nondecreasing radial profile, withdrawing at the
        // same XY sweeps no additional stock below the original top. The whole
        // deepest cutter pose is therefore the exact vertical-sweep authority.
        if (a.X == b.X && a.Y == b.Y && Math.Max(-a.Z,-b.Z) <= profile.MaximumDepth)
        {
            if (a.Z != b.Z) verticalSweepsReduced++;
            a = b = a.Z < b.Z ? a : b;
        }
        return ContainsQualifiedRaisedChord(a,b)
            || LayerLegal(a, b) || projectedStock.Prove(a, b, postingReserve: postingReserve);
    }

    private static (double, double, double, double) SourceKey(ref Point3D a, ref Point3D b)
    {
        if (a.X > b.X || a.X == b.X && a.Y > b.Y) (a,b)=(b,a);
        return (a.X,a.Y,b.X,b.Y);
    }

    private bool ContainsQualifiedRaisedChord(Point3D a, Point3D b)
    {
        var key=SourceKey(ref a,ref b);
        if (!qualifiedSources.TryGetValue(key,out var depths)) return false;
        foreach (var source in depths)
        {
            // Exact planar endpoints in this coordinate domain, with both new
            // endpoints at or above an independently proved whole source chord.
            // At every stock height the monotone cutter cross-section is then
            // a subset of the source sweep. This credits clearance, not future
            // removal. No interpolation, endpoint snapping or tolerance is used.
            if (a.Z < source.A || b.Z < source.B) continue;
            raisedSourceProofs++;
            return true;
        }
        return false;
    }

    private void QualifySourceChords(IReadOnlyList<TaggedToolpath> paths)
    {
        void Add(Point3D a,Point3D b)
        {
            if (Math.Max(-a.Z,-b.Z)>profile.MaximumDepth) return;
            // Establish authority directly from original stock, without using
            // any previously registered source certificate.
            if (!LayerLegal(a,b) && !projectedStock.Prove(a,b)) return;
            var key=SourceKey(ref a,ref b);
            if (!qualifiedSources.TryGetValue(key,out var depths))
                qualifiedSources.Add(key,depths=new());
            if (depths.Any(d=>d.A<=a.Z && d.B<=b.Z)) return;
            depths.RemoveAll(d=>d.A>=a.Z && d.B>=b.Z);
            depths.Add((a.Z,b.Z));
            qualifiedSourceChords++;
        }
        foreach (var path in paths)
        for (int i=1;i<path.Points.Count;i++)
        {
            token.ThrowIfCancellationRequested();
            var a=path.Points[i-1];var b=path.Points[i];
            Add(a,b);Add(Command(a),Command(b));
        }
    }

    private bool LayerLegal(Point3D a, Point3D b)
    {
        foreach (var layer in layers)
        {
            if (Math.Max(-a.Z, -b.Z) <= layer.TopDepth + 1e-10) continue;
            foreach (var point in new[] { a, b })
            {
                double penetration = -point.Z - layer.TopDepth;
                if (penetration <= 1e-10) continue;
                double clearance = layer.Distance.SignedDistance(new(point.X, point.Y));
                if (clearance < 0 || profile.PhysicalRadiusAtDepth(penetration) > clearance + 1e-9) return false;
            }
            if (layer.Boundary.TryFindDepthViolation(a, b, -layer.TopDepth,
                0, 1, 0, out double witness, profile))
            {
                if (Math.Pow(a.X-b.X,2)+Math.Pow(a.Y-b.Y,2)<1e-8 && ++shortRefusals <= 8)
                {
                    double x=a.X+(b.X-a.X)*witness, y=a.Y+(b.Y-a.Y)*witness;
                    double d=-(a.Z+(b.Z-a.Z)*witness)-layer.TopDepth;
                    diagnostic?.Invoke($"stage=material-surface event=short-chord-witness a={a} b={b} top={layer.TopDepth:R} t={witness:R} "
                        + $"penetration={d:R} radius={profile.PhysicalRadiusAtDepth(d):R} nearest={layer.Boundary.QueryDistance(x,y):R} signed={layer.Distance.SignedDistance(new(x,y)):R}");
                }
                return false;
            }
        }
        return true;
    }

    private double ShoulderFloor(PointD edge)
    {
        if(shoulderSurfaceDepths.TryGetValue((edge.x,edge.y),out double floor))return floor;
        floor=projectedStock.ClearedDepth(edge.x,edge.y,0);
        if(shoulderSurfaceDepths.Count>=131072)shoulderSurfaceDepths.Clear();
        shoulderSurfaceDepths[(edge.x,edge.y)]=floor;
        return floor;
    }

    private Point3D? FitShoulder(Point3D source)
    {
        if (projectedStock.IndexedChords == 0) return null;
        if (originalTarget.NearestBoundaryPoint(new(source.X, source.Y)) is not { } edge)
            return null;
        double dx = source.X - edge.x, dy = source.Y - edge.y;
        double length = Math.Sqrt(dx * dx + dy * dy);
        if (length <= 1e-10) return null;
        dx /= length; dy /= length;
        // A simplified nomination may lie across a concave original edge.
        // Its side must not reverse the contact ray: the original filled
        // target, rather than the approximate nomination, owns the inward side.
        if(originalTarget.SignedDistance(new(edge.x+dx*PlanarReserve,edge.y+dy*PlanarReserve))<0)
        {
            dx=-dx;dy=-dy;
            if(++shoulderInwardCorrections<=8)
                diagnostic?.Invoke($"stage=material-surface event=shoulder-inward-correction source={source} edge=({edge.x:R},{edge.y:R}) "
                    +"inwardOwner=original-filled-target nominationSideIsNotAuthoritative=True");
        }
        double cap = Math.Min(maximumDepth, Math.Max(0, -source.Z));
        double floor=ShoulderFloor(edge);
        if (floor >= cap - .0005) return null;
        double high = profile.RadiusAtDepth(Math.Max(0, cap - floor)) + PlanarReserve;
        Point3D Candidate(double offset)
        {
            double depth = Math.Min(cap, floor + profile.DepthAtRadius(Math.Max(0, offset - PlanarReserve)));
            // Choose an actual shallower F3 depth. Rounding a sagitta downward
            // in Z can otherwise create disconnected false-fit intervals.
            depth = Math.Floor(depth * 1000) / 1000;
            return new(edge.x + dx * offset, edge.y + dy * offset, -depth);
        }
        bool Fits(Point3D candidate)
        {
            shoulderTrials++;
            return IsLegalChord(candidate, candidate);
        }
        var result = Candidate(high);
        bool OriginalStockContains(Point3D point) => originalTarget.SignedDistance(new(point.X, point.Y))
            >= profile.PhysicalRadiusAtDepth(-point.Z) + PreparedRegionDistance.Uncertainty;
        // The original carving already supplies poses contained by the original
        // stock domain. Add only contact enabled by actual preceding removal.
        if (OriginalStockContains(result)) return null;
        bool found = Fits(result);
        double localDepth = Math.Max(0, cap-floor);
        if (!found && localDepth >= profile.FirstCylinderHeight
            && profile.RadiusAtDepth(localDepth) == profile.RadiusAtDepth(Math.Max(0,localDepth-.001)))
        {
            // A cylinder's contact radius does not select a unique depth.
            // Fit its actual stock-limited depth at this same contact axis.
            var limited = Fit(result, preserveDepthCap:true);
            if (-limited.Z > floor+.0005
                && profile.RadiusAtDepth(-limited.Z-floor) == profile.RadiusAtDepth(localDepth))
            { result=limited; found=true; }
        }
        if (!found || -result.Z <= floor + .0005)
        {
            double low = 0;
            for (int trial = 0; trial < 18 && high - low > .0001; trial++)
            {
                double mid = (low + high) / 2;
                var candidate = Candidate(mid);
                if (Fits(candidate)) { low = mid; result = candidate; found = true; }
                else high = mid;
            }
        }
        if (!found)
        {
            if (++shoulderUnreachableSites <= 8)
                diagnostic?.Invoke($"stage=material-surface event=shoulder-contact-refused source={source} floor={floor:R} "
                    + "fallback=omit-supplemental-interval fullPhysicalAndF3Guard=True");
            return null;
        }
        if (OriginalStockContains(result)) return null;
        if (++shoulderMovedSites <= 8)
            diagnostic?.Invoke($"stage=material-surface event=shoulder-contact source={source} fitted={result} floor={floor:R} "
                + "actualCutterXYAndDepth=True fullPhysicalAndF3Guard=True");
        return result;
    }

    private List<TaggedToolpath> FitShoulderPath(TaggedToolpath path)
    {
        var runs = new List<List<Point3D>>();
        var run = new List<Point3D>();
        void Finish()
        {
            if (run.Count > 1) runs.Add(run);
            run = new();
        }
        void Segment(Point3D sourceA, Point3D sourceB, Point3D? a, Point3D? b, int level)
        {
            token.ThrowIfCancellationRequested();
            var sourceMid = new Point3D((sourceA.X + sourceB.X) / 2,
                (sourceA.Y + sourceB.Y) / 2, (sourceA.Z + sourceB.Z) / 2);
            double sourceLength = Math.Pow(sourceA.X-sourceB.X,2)+Math.Pow(sourceA.Y-sourceB.Y,2);
            // At command-scale spans the full chord proof already decides
            // acceptance. The midpoint fit was calculated and then ignored
            // by both the accepted-short-chord and terminal-refusal branches.
            if(sourceLength<=.000004 && a is { } shortA && b is { } shortB && IsLegalChord(shortA,shortB))
            {
                if(run.Count>0&&run[^1]!=shortA)Finish();
                if(run.Count==0)run.Add(shortA);
                if(run[^1]!=shortB)run.Add(shortB);
                shoulderUnusedMidpointFitsAvoided++;return;
            }
            if(level>=16 || sourceLength<=Math.Pow(ToolpathCommandRounding.CommandIncrement,2))
            {
                Finish();if(b is { } terminal)run.Add(terminal);
                shoulderUnusedMidpointFitsAvoided++;return;
            }
            var mid = FitShoulder(sourceMid);
            bool smooth=a is { } fitStart && b is { } fitEnd && mid is { } middle
                && Math.Pow(fitStart.X-fitEnd.X,2)+Math.Pow(fitStart.Y-fitEnd.Y,2)<=.25
                && Math.Abs(middle.X-(fitStart.X+fitEnd.X)/2)<=.002 && Math.Abs(middle.Y-(fitStart.Y+fitEnd.Y)/2)<=.002
                && Math.Abs(middle.Z-(fitStart.Z+fitEnd.Z)/2)<=.01;
            if (a is { } pa && b is { } pb && mid is { } pm && IsLegalChord(pa,pb)
                && (sourceLength <= .000004 || smooth))
            {
                if (run.Count > 0 && run[^1] != pa) Finish();
                if (run.Count == 0) run.Add(pa);
                if (run[^1] != pb) run.Add(pb);
                return;
            }
            // A refused interval below the posted XY increment cannot supply
            // another machinable contact site. Preserve the qualified endpoints
            // and let the fully proved local withdrawal join them; subdividing
            // to tenths of a micron only repeats the same command-cell failures.
            if (a is null && b is null && mid is null && sourceLength <= .25)
            {
                Finish();
                if (b is { } end) run.Add(end);
                return;
            }
            Segment(sourceA, sourceMid, a, mid, level+1);
            Segment(sourceMid, sourceB, mid, b, level+1);
        }
        var first = FitShoulder(path.Points[0]);
        var previous = first;
        for (int i=1;i<path.Points.Count;i++)
        {
            var next = i==path.Points.Count-1 && path.Points[0]==path.Points[^1]
                ? first : FitShoulder(path.Points[i]);
            Segment(path.Points[i-1],path.Points[i],previous,next,0);
            previous=next;
        }
        Finish();
        if (runs.Count>1 && runs[^1][^1]==runs[0][0])
        {
            runs[^1].AddRange(runs[0].Skip(1));
            runs.RemoveAt(0);
        }
        // Nearby fit intervals can end separately. A new surface
        // entry for every such interval creates thousands of tiny ramp laps.
        // Retain both contact traces and use the same half-mm construction span,
        // with a fully proved local withdrawal when a direct chord cannot fit.
        var joined=new List<List<Point3D>>();
        foreach (var next in runs)
        {
            if (joined.Count==0 || !Join(joined[^1],next)) joined.Add(next);
        }
        if(joined.Count>1 && Join(joined[^1],joined[0]))joined.RemoveAt(0);
        return joined.Select(points=>new TaggedToolpath(points,Array.Empty<ToolpathSpan>(),path.RegionIndex,
            path.Category,path.DepthPassIndex,path.VisitIdentity,detectCurves:false)).ToList();

        bool Join(List<Point3D> previous,List<Point3D> next)
        {
            var a=previous[^1];var b=next[0];
            if (Math.Pow(a.X-b.X,2)+Math.Pow(a.Y-b.Y,2)>.25) return false;
            if (!IsLegalChord(a,b))
            {
                double low=Math.Max(a.Z,b.Z),high=Math.Min(0,low+.01);
                bool Fits(double z)=>IsLegalChord(a with{Z=z},b with{Z=z});
                if (!Fits(high)) {shoulderJoinRefusals++;return false;}
                for(int trial=0;trial<10;trial++)
                {
                    double mid=(low+high)/2;
                    if(Fits(mid))high=mid;else low=mid;
                }
                var safeA=a with{Z=high};var safeB=b with{Z=high};
                if(!IsLegalChord(a,safeA)||!IsLegalChord(safeB,b))
                {shoulderJoinRefusals++;return false;}
                if(previous[^1]!=safeA)previous.Add(safeA);
                if(previous[^1]!=safeB)previous.Add(safeB);
            }
            if(previous[^1]!=b)previous.Add(b);
            previous.AddRange(next.Skip(1));
            shoulderRunJoins++;
            return true;
        }
    }

    internal List<TaggedToolpath> Apply(List<TaggedToolpath> paths, IReadOnlySet<TaggedToolpath>? clearingNominations = null)
    {
        long started=System.Diagnostics.Stopwatch.GetTimestamp();
        var result = new List<TaggedToolpath>(paths.Count);
        foreach (var path in paths)
        {
            // Original-stock clearing already has a complete physical proof.
            // Preserve its literal level moves; fit only the new shoulder fronts.
            if (clearingNominations?.Contains(path) == true && path.Points.Count > 1)
            {
                result.AddRange(FitShoulderPath(path));
                continue;
            }
            if (path.Category != "final-carve" || path.Points.Count < 2) { result.Add(path); continue; }
            token.ThrowIfCancellationRequested();
            sourcePoints += path.Points.Count;
            var points = new List<Point3D>(path.Points.Count) { Fit(path.Points[0]) };
            for (int i = 1; i < path.Points.Count; i++)
                Append(path.Points[i - 1], path.Points[i], points[^1],
                    i == path.Points.Count - 1 && path.Points[^1] == path.Points[0] ? points[0] : Fit(path.Points[i]),
                    points, 0);
            outputPoints += points.Count;
            result.Add(new TaggedToolpath(points, Array.Empty<ToolpathSpan>(), path.RegionIndex,
                path.Category, path.DepthPassIndex, path.VisitIdentity, detectCurves: false));
        }
        QualifySourceChords(result);
        diagnostic?.Invoke($"stage=material-surface event=fitted-region paths={paths.Count} originalMedialPoints={sourcePoints} retainedMedialPoints={outputPoints} "
            +$"qualifiedSourceChords={qualifiedSourceChords} sourceCertificatePolicy=exact-XY-axially-raised-native-and-F3 "
            +$"shoulderTrials={shoulderTrials} shoulderContactSites={shoulderMovedSites} shoulderUnreachableSites={shoulderUnreachableSites} originalClearingPreserved=True "
            +$"minimumShoulderNominationSpan={ToolpathCommandRounding.CommandIncrement:R} physicalProofToleranceUnchanged=True "
            +$"shoulderUnusedMidpointFitsAvoided={shoulderUnusedMidpointFitsAvoided} smallChordAuthority=complete-native-and-F3-sweep "
            +$"shoulderInwardCorrections={shoulderInwardCorrections} inwardOwner=original-filled-target "
            +$"seededStockCellProofs={projectedStock.SeededCellProofs} seededStockCellTrials={projectedStock.SeededCellTrials} seedCellAuthority=whole-prior-pose-disk-and-current-full-cutter "
            +$"shoulderRunJoins={shoulderRunJoins} shoulderJoinRefusals={shoulderJoinRefusals} maximumJoinGap=0.5 localJoinLift=0.01 fullNativeAndF3JoinProof=True "
            +$"nearbyDepthTrials={projectedStock.NearbyDepthTrials} nearbyDepthFits={projectedStock.NearbyDepthFits} nearbyDepthRefusals={projectedStock.NearbyDepthRefusals} "
            +$"stockCellCacheHits={projectedStock.StockCellCacheHits} stockCellCacheMisses={projectedStock.StockCellCacheMisses} stockCellCacheEvictions={projectedStock.StockCellCacheEvictions} cellGeometryCacheHits={projectedStock.CellGeometryCacheHits} cellGeometryCacheMisses={projectedStock.CellGeometryCacheMisses} cachedSweepProofs={projectedStock.CachedSweepProofs} sweepProofCacheLimit={VCarveProjectedStock.SweepProofCacheLimit} sweepProofCacheEvictions={projectedStock.SweepProofCacheEvictions} reversedSweepProofHits={projectedStock.ReversedSweepProofHits} strongerSweepProofHits={projectedStock.StrongerSweepProofHits} depthWitnessQueries={projectedStock.DepthWitnessQueries} depthWitnessBounds={projectedStock.DepthWitnessBounds} depthWitnessSeedFits={projectedStock.DepthWitnessSeedFits} "
            +$"profileCandidatesPruned={projectedStock.ProfileCandidatesPruned} profileNodesPruned={projectedStock.ProfileNodesPruned} priorPoseSeedTrials={projectedStock.PriorPoseSeedTrials} priorPoseSeedPositive={projectedStock.PriorPoseSeedPositive} elapsedMs={System.Diagnostics.Stopwatch.GetElapsedTime(started).TotalMilliseconds:F1} deletedBranches=0");
        return result;
    }

    private void Append(Point3D sourceA, Point3D sourceB, Point3D a, Point3D b,
        List<Point3D> output, int level)
    {
        token.ThrowIfCancellationRequested();
        var midSource = new Point3D((sourceA.X + sourceB.X) / 2,
            (sourceA.Y + sourceB.Y) / 2, (sourceA.Z + sourceB.Z) / 2);
        var mid = Fit(midSource);
        double lengthSquared = Math.Pow(a.X - b.X, 2) + Math.Pow(a.Y - b.Y, 2);
        double sourceLengthSquared = Math.Pow(sourceA.X - sourceB.X, 2) + Math.Pow(sourceA.Y - sourceB.Y, 2);
        bool legal = IsLegalChord(a, b);
        if (!legal && level == 0 && ++refusedSources <= 8)
            diagnostic?.Invoke($"stage=material-surface event=refine-refused-chord a={a} b={b} "
                + $"nativeA={Legal(a,a)} nativeB={Legal(b,b)} postedA={Legal(Command(a),Command(a))} postedB={Legal(Command(b),Command(b))} "
                + $"layers=" + string.Join(";", layers.Select(l => $"{l.TopDepth:R}:{l.Distance.SignedDistance(new(a.X,a.Y)):R}/{l.Boundary.QueryDistance(a.X,a.Y):R}")));
        // F3 planar-cell boundaries make the posted clearance field discrete.
        // A proved chord at command resolution must not keep subdividing to
        // chase that discontinuity; doing so adds no machinable XY detail.
        if (legal && (sourceLengthSquared <= .000004
            || lengthSquared <= .25 && Math.Abs(mid.Z - (a.Z + b.Z) / 2) <= .01
                && Math.Abs(mid.X - (a.X + b.X) / 2) <= .002
                && Math.Abs(mid.Y - (a.Y + b.Y) / 2) <= .002))
        { output.Add(b); return; }
        if (level < 16 && sourceLengthSquared > .00000001)
        {
            subdivisions++;
            Append(sourceA, midSource, a, mid, output, level + 1);
            Append(midSource, sourceB, output[^1], b, output, level + 1);
            return;
        }
        // Never delete the branch or credit an unproved chord. Very short
        // posted XY transitions may need a shallower common tool pose.
        double low = 0, high = Math.Min(-a.Z, -b.Z);
        for (int i = 0; i < 24; i++)
        {
            double candidate = (low + high) / 2;
            if (IsLegalChord(new(a.X, a.Y, -candidate), new(b.X, b.Y, -candidate))) low = candidate;
            else high = candidate;
        }
        var safeA = new Point3D(a.X, a.Y, -low);
        var safeB = new Point3D(b.X, b.Y, -low);
        // The extra axial segment only retracts inside the already-proved
        // cutter envelope at A. B has no following descent into unproved stock.
        output.Add(safeA); output.Add(safeB);
        if (++retreats <= 8)
            diagnostic?.Invoke($"stage=material-surface event=quantized-chord-retreat xy=({a.X:R},{a.Y:R})-({b.X:R},{b.Y:R}) depth={low:R}");
    }

    internal void Verify(IReadOnlyList<TaggedToolpath> paths)
    {
        foreach (var path in paths)
        for (int i = 1; i < path.Points.Count; i++)
        {
            token.ThrowIfCancellationRequested();
            var a = path.Points[i - 1]; var b = path.Points[i];
            if (!IsLegalChord(new(a.X, a.Y, a.Z - surface), new(b.X, b.Y, b.Z - surface), postingReserve:false))
                throw new InvalidOperationException($"Height-aware V-carve cutter clearance failed: region={path.RegionIndex}, category={path.Category}, chord={i-1}, spans={string.Join(';',path.Spans)}, a={a}, b={b}.");
        }
        diagnostic?.Invoke($"stage=material-surface event=verified originalMedialPoints={sourcePoints} "
            + $"retainedMedialPoints={outputPoints} loweredSamples={loweredPoints} maximumAdditionalDepth={largestDescent:R} "
            + $"chordChecks={chordChecks} verticalSweepsReduced={verticalSweepsReduced} qualifiedSourceChords={qualifiedSourceChords} raisedSourceProofs={raisedSourceProofs} subdivisions={subdivisions} quantizedRetreats={retreats} fittedRetreats={fittedRetreats} "
            + $"continuousQueries={projectedStock.Queries} continuousCells={projectedStock.Cells} "
            + $"continuousColumnQueries={projectedStock.ColumnQueries} continuousColumnTests={projectedStock.ColumnTests} "
            + $"continuousImprovedSites={projectedStock.ImprovedSites} continuousMaximumImprovement={projectedStock.LargestImprovement:R} "
            + $"continuousCacheHits={projectedStock.CacheHits} continuousRefusals={projectedStock.Refusals} "
            + $"flatWitnessRefusals={projectedStock.FlatWitnessRefusals} priorProfileChords={projectedStock.ProfileChords} "
            + $"stockCellCacheHits={projectedStock.StockCellCacheHits} stockCellCacheMisses={projectedStock.StockCellCacheMisses} stockCellCacheEvictions={projectedStock.StockCellCacheEvictions} cellGeometryCacheHits={projectedStock.CellGeometryCacheHits} cellGeometryCacheMisses={projectedStock.CellGeometryCacheMisses} cachedSweepProofs={projectedStock.CachedSweepProofs} sweepProofCacheLimit={VCarveProjectedStock.SweepProofCacheLimit} sweepProofCacheEvictions={projectedStock.SweepProofCacheEvictions} reversedSweepProofHits={projectedStock.ReversedSweepProofHits} strongerSweepProofHits={projectedStock.StrongerSweepProofHits} depthWitnessQueries={projectedStock.DepthWitnessQueries} depthWitnessBounds={projectedStock.DepthWitnessBounds} depthWitnessSeedFits={projectedStock.DepthWitnessSeedFits} stockCellRootWidth=16 stockCellCacheLimit=1048576 "
            + $"nearbyDepthTrials={projectedStock.NearbyDepthTrials} nearbyDepthFits={projectedStock.NearbyDepthFits} nearbyDepthRefusals={projectedStock.NearbyDepthRefusals} "
            + "deletedBranches=0 nativeAndActualF3=True fullPhysicalProfile=True");
    }
}
