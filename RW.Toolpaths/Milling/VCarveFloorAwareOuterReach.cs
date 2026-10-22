using System.Diagnostics;

namespace RW.Toolpaths.Milling;

// New OUTER center nomination. This does not reuse the rejected INNER reach
// equation or grant physical, filled-set, or finished-stock authority.
internal static class VCarveFloorAwareOuterReach
{
    private const int MaximumTrials = 6, MaximumMarkedPoints = 500_000;
    private const double GradientStep = .001, NominalReachStop = .001;
    private sealed class Counts
    {
        internal long Paths, InputPoints, OutputPoints, Trials, Caps, Radii, Distances,
            Moved, Kept, Corners, Unknown, Nonconvergent, StructuralPaths, BudgetPaths, Partial;
        internal double MaximumShift, MaximumInitialGap, MaximumAcceptedGap, MaximumPartialGap;
    }

    // floorCap(x,y) is ORIGINAL Field.CappedDepth(nominalDepth,x,y).
    // artworkDistance is ORIGINAL inside distance (NaN outside/unresolved).
    // commandedRadius(x,y,depth) uses literal CommandPoint then the unchanged
    // full profile.PhysicalRadiusAtDepth. No NOM offset radius supplies proof.
    internal static List<TaggedToolpath> Apply(IReadOnlyList<TaggedToolpath> source,
        VCarveCutterProfile profile, double nominalDepth,
        Func<double,double,double> floorCap, Func<double,double,double> artworkDistance,
        Func<double,double,double,double> commandedRadius, CancellationToken token,
        Action<string>? diagnostic = null,double additionalWallOffset=0,double samplingTolerance=NominalReachStop)
    {
        ArgumentNullException.ThrowIfNull(source); ArgumentNullException.ThrowIfNull(profile);
        ArgumentNullException.ThrowIfNull(floorCap); ArgumentNullException.ThrowIfNull(artworkDistance);
        ArgumentNullException.ThrowIfNull(commandedRadius); token.ThrowIfCancellationRequested();
        long started = Stopwatch.GetTimestamp(); var c = new Counts();
        MillingGeometryTrace.Capture("outer-reach-input",0,source);
        string marker = VCarveInitialReachSelection.QuarterOuterMarkerCategory;
        long marked = source.Where(path => path.Category == marker).Sum(path => (long)path.Points.Count);
        double nominalRadius = double.IsFinite(nominalDepth) && nominalDepth > 0
            ? profile.PhysicalRadiusAtDepth(nominalDepth) : double.NaN;
        // The sampling step is a NOM circle-sagitta scale, never a certificate
        // for an unknown floor curve or the material swept between samples.
        double sampleStep = Math.Sqrt(8 * nominalRadius * samplingTolerance);
        bool eligible = marked <= MaximumMarkedPoints && profile.Sections.Count is > 0 and <= 64
            && double.IsFinite(nominalDepth) && nominalDepth > 0 && nominalDepth <= profile.MaximumDepth
            && double.IsFinite(nominalRadius) && nominalRadius > 0 && nominalRadius <= 100
            && double.IsFinite(sampleStep) && sampleStep > 0
            && double.IsFinite(samplingTolerance) && samplingTolerance>0
            && double.IsFinite(additionalWallOffset) && additionalWallOffset>=0;
        var result = new List<TaggedToolpath>(source.Count);
        long remainingInput = marked, outputPoints = 0;
        foreach (var path in source)
        {
            token.ThrowIfCancellationRequested();
            if (path.Category != marker) { result.Add(path); continue; }
            c.Paths++; c.InputPoints += path.Points.Count; remainingInput -= path.Points.Count;
            bool closed = path.Points.Count >= 4 && path.Points[0] == path.Points[^1];
            if (!eligible || !closed || path.Spans.Count != 0)
            { c.StructuralPaths++; KeepPath(); continue; }
            long required = 1;
            bool planned = true;
            for (int i = 1; i < path.Points.Count; i++)
            {
                if ((i & 255) == 0) token.ThrowIfCancellationRequested();
                Point3D a = path.Points[i - 1], b = path.Points[i];
                double dx = b.X - a.X, dy = b.Y - a.Y;
                double length = Math.Sqrt(dx * dx + dy * dy), pieces = Math.Max(1, Math.Ceiling(length / sampleStep));
                if (!double.IsFinite(length) || !double.IsFinite(pieces) || pieces > MaximumMarkedPoints
                    || required + pieces + outputPoints + remainingInput > MaximumMarkedPoints)
                { planned = false; break; }
                required += (long)pieces;
            }
            if (!planned) { c.BudgetPaths++; KeepPath(); continue; }
            var output = new List<Point3D>((int)required);
            output.Add(Move(path.Points[0]));
            for (int i = 1; i < path.Points.Count; i++)
            {
                token.ThrowIfCancellationRequested();
                Point3D a = path.Points[i - 1], b = path.Points[i];
                double dx = b.X - a.X, dy = b.Y - a.Y;
                int pieces = (int)Math.Max(1, Math.Ceiling(Math.Sqrt(dx * dx + dy * dy) / sampleStep));
                for (int j = 1; j <= pieces; j++)
                {
                    token.ThrowIfCancellationRequested();
                    if (i == path.Points.Count - 1 && j == pieces) { output.Add(output[0]); continue; }
                    double t = (double)j / pieces;
                    Point3D raw = j == pieces ? b : new(a.X + dx * t, a.Y + dy * t, a.Z + (b.Z - a.Z) * t);
                    output.Add(Move(raw));
                }
            }
            if (output.Count != required || output[0] != output[^1])
                throw new InvalidOperationException("Outer NOM sampling lost its complete closed literal ring inventory.");
            result.Add(path with { Points = output, Category = "clearing" });
            outputPoints += output.Count; c.OutputPoints += output.Count;

            void KeepPath()
            {
                result.Add(path with { Category = "clearing" });
                outputPoints += path.Points.Count; c.OutputPoints += path.Points.Count;
                c.Kept += path.Points.Count;
            }
        }
        token.ThrowIfCancellationRequested();
        diagnostic?.Invoke($"stage=floor-aware-outer-reach event=complete eligible={eligible} paths={c.Paths} inputPoints={c.InputPoints} outputPoints={c.OutputPoints} " +
            $"trials={c.Trials} floorCapQueries={c.Caps} physicalRadiusQueries={c.Radii} artworkDistanceQueries={c.Distances} " +
            $"movedPoints={c.Moved} retainedPoints={c.Kept} cornerFallbacks={c.Corners} unknownFallbacks={c.Unknown} nonconvergentFallbacks={c.Nonconvergent} " +
            $"partialProjections={c.Partial} maximumPartialGap={c.MaximumPartialGap:R} " +
            $"structuralPathFallbacks={c.StructuralPaths} samplingBudgetPathFallbacks={c.BudgetPaths} maximumShift={c.MaximumShift:R} " +
            $"maximumInitialGap={c.MaximumInitialGap:R} maximumAcceptedGap={c.MaximumAcceptedGap:R} nominalDepth={nominalDepth:R} nominalRadius={nominalRadius:R} " +
            $"gradientStep={GradientStep:R} sampleStep={sampleStep:R} nominalSampleSagitta={samplingTolerance:R} additionalWallOffset={additionalWallOffset:R} maximumTrials={MaximumTrials} maximumMarkedOutputPoints={MaximumMarkedPoints} " +
            $"elapsedMs={Stopwatch.GetElapsedTime(started).TotalMilliseconds:F3} " +
            "equation=originalArtworkDistance-minus-actualCommandPhysicalRadius acceptedZ=originalCappedDepth " +
            "quarterMarkerNormalizedOnly=True halfAndOtherPathsUnchanged=True allClosedSignedTinyRingsRetained=True originalFloorSourcesUnchanged=True " +
            "contactSlope=measured-secant boundedFallback=best-evaluated-interior " +
            "continuousNominationProof=False physicalCredit=False stockCredit=False freshOriginalFloorArtworkAndGradeRequired=True");
        MillingGeometryTrace.Capture("outer-reach-output",0,result);
        token.ThrowIfCancellationRequested(); return result;

        Point3D Move(Point3D old)
        {
            token.ThrowIfCancellationRequested();
            double bestShift=0,bestDepth=-old.Z,bestGap=double.PositiveInfinity;
            double nx=0,ny=0;
            if (old.Z != -nominalDepth || !FiniteXY(old.X, old.Y)) { c.Unknown++; return Keep(); }
            if (!Evaluate(old.X, old.Y, out double initialDistance, out double initialDepth, out double initialRadius)) return Keep();
            double initialGap = initialDistance - initialRadius - additionalWallOffset;
            bestDepth=initialDepth;bestGap=initialGap;
            c.MaximumInitialGap = Math.Max(c.MaximumInitialGap, Math.Abs(initialGap));
            if (Math.Abs(initialGap) <= NominalReachStop) return Accept(old.X, old.Y, initialDepth, 0, initialGap);
            if (initialGap < 0) { c.Corners++; return Keep(); }
            c.Distances += 4;
            double xm = artworkDistance(old.X - GradientStep, old.Y), xp = artworkDistance(old.X + GradientStep, old.Y);
            double ym = artworkDistance(old.X, old.Y - GradientStep), yp = artworkDistance(old.X, old.Y + GradientStep);
            double gx = (xp - xm) / (2 * GradientStep), gy = (yp - ym) / (2 * GradientStep);
            double gradient = Math.Sqrt(gx * gx + gy * gy);
            if (!double.IsFinite(xm) || !double.IsFinite(xp) || !double.IsFinite(ym) || !double.IsFinite(yp)
                || !double.IsFinite(gradient) || gradient < .25 || gradient > 2)
            { c.Corners++; return Keep(); }
            nx = -gx / gradient; ny = -gy / gradient;
            double low = 0, lowGap = initialGap, high = double.NaN, highGap = 0;
            double previousShift=0,previousGap=initialGap;
            double shift = Math.Clamp(initialGap / gradient, 0, nominalRadius);
            for (int trial = 0; trial < MaximumTrials; trial++)
            {
                token.ThrowIfCancellationRequested(); c.Trials++;
                double x = old.X + nx * shift, y = old.Y + ny * shift;
                if (!FiniteXY(x, y) || !Evaluate(x, y, out double d, out double depth, out double r)) return Keep();
                if (!(d < initialDistance)) { c.Corners++; return Keep(); }
                double gap = d - r - additionalWallOffset;
                if (Math.Abs(gap) <= NominalReachStop) return Accept(x, y, depth, shift, gap);
                // Keep the closest evaluated interior nomination. Exhausting
                // the bounded solve must not snap one vertex back to the old
                // full-depth ring while its neighbours remain projected.
                // This carries no physical authority: original Protect still
                // checks every proposed chord and commanded cutter pose.
                if(gap>=0&&gap<bestGap){bestShift=shift;bestDepth=depth;bestGap=gap;}
                if (gap > 0) { low = shift; lowGap = gap; } else { high = shift; highGap = gap; }
                // The solved equation includes the changing floor and cutter
                // radius. Artwork distance's gradient alone is not its slope.
                // Reuse consecutive evaluations for the actual secant slope;
                // once bracketed, keep the proposal inside that bracket.
                double slope=(gap-previousGap)/(shift-previousShift);
                double next = double.IsFinite(high) && high > low
                    ? (low * (-highGap) + high * lowGap) / (lowGap - highGap)
                    : double.IsFinite(slope)&&slope<0?shift-gap/slope:shift+gap/gradient;
                previousShift=shift;previousGap=gap;
                next = Math.Clamp(next, 0, nominalRadius);
                if (!double.IsFinite(next) || next == shift) break;
                shift = next;
            }
            c.Nonconvergent++; return Keep();

            Point3D Keep()
            {
                if(bestShift>0)
                {
                    c.Partial++;c.MaximumPartialGap=Math.Max(c.MaximumPartialGap,bestGap);
                    if(c.Partial<=8)diagnostic?.Invoke($"stage=floor-aware-outer-reach event=partial-projection x={old.X:R} y={old.Y:R} shift={bestShift:R} residualGap={bestGap:R}");
                    return Accept(old.X+nx*bestShift,old.Y+ny*bestShift,bestDepth,bestShift,bestGap);
                }
                c.Kept++;return new(old.X,old.Y,-bestDepth);
            }
            Point3D Accept(double x, double y, double depth, double move, double gap)
            {
                c.Moved++; c.MaximumShift = Math.Max(c.MaximumShift, move);
                c.MaximumAcceptedGap = Math.Max(c.MaximumAcceptedGap, Math.Abs(gap));
                return new(x, y, -depth);
            }
        }
        bool Evaluate(double x, double y, out double d, out double depth, out double r)
        {
            token.ThrowIfCancellationRequested(); c.Distances++; d = artworkDistance(x, y);
            depth = r = double.NaN;
            if (!double.IsFinite(d) || d < 0) { c.Unknown++; return false; }
            c.Caps++; depth = floorCap(x, y);
            if (!double.IsFinite(depth) || depth < 0 || depth > nominalDepth) { c.Unknown++; return false; }
            c.Radii++; r = commandedRadius(x, y, depth);
            if (!double.IsFinite(r) || r < 0 || r > 100) { c.Unknown++; return false; }
            return true;
        }
    }
    private static bool FiniteXY(double x, double y) => double.IsFinite(x) && double.IsFinite(y) && Math.Abs(x) <= 1e6 && Math.Abs(y) <= 1e6;
}
