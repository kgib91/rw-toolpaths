using System.Diagnostics;
using Clipper2Lib;
using RW.Toolpaths.Geometry;

namespace RW.Toolpaths.Milling;

// Explicit fixed-goal construction policy. The conditioned finite chains are
// untrusted operands, not original reachability, machining or stock evidence.
// The complete original target masks the result; caller freezes it before
// correction and must certify that whole goal before publishing commands.
internal static class VCarveConditionedReferenceOpeningGoal
{
    internal const string SelectionEnvironment = "RW_TOOLPATHS_VCARVE_CONDITIONED_REFERENCE_OPENING";
    internal const int MaximumSourcePoints = 4_000_000;
    internal const long MaximumAbsoluteNativeCoordinate = 1L << 29;

    internal static List<List<PointD>> BuildMaskedGoal(List<List<PointD>> originalTarget,
        double radius, CancellationToken token, Action<string>? diagnostic, out bool conditioned)
        => BuildCore(originalTarget, originalTarget, radius, null, token, diagnostic, out conditioned);

    internal static List<List<PointD>> BuildMaskedGoalForPlane(List<List<PointD>> originalTarget,
        List<List<PointD>> goalMask, double radius, double dilationRadius,
        CancellationToken token, Action<string>? diagnostic, out bool conditioned)
        => BuildCore(originalTarget, goalMask, radius, dilationRadius, token, diagnostic, out conditioned);

    private static List<List<PointD>> BuildCore(List<List<PointD>> originalTarget, List<List<PointD>> goalMask,
        double radius, double? dilationRadius, CancellationToken token, Action<string>? diagnostic, out bool conditioned)
    {
        token.ThrowIfCancellationRequested(); conditioned = false;
        long started = Stopwatch.GetTimestamp(); long points = 0;
        double preparationMs = 0, openingMs = 0, maskingMs = 0;
        string reason = "unsupported";
        List<List<PointD>>? result = null;
        int conditionedPoints = 0;
        string originalHash = "none", constructionHash = "none";
        var native = new Paths64();
        bool valid = double.IsFinite(radius) && radius > 8 / PathUtils.Scale;
        if (valid)
        {
            foreach (var ring in originalTarget)
            {
                token.ThrowIfCancellationRequested();
                if (ring.Count < 3 || ring.Count > 1_000_000
                    || (points += ring.Count) > MaximumSourcePoints) { valid = false; break; }
                var path = new Path64(ring.Count);
                foreach (var point in ring)
                {
                    double x = Math.Round(point.x * PathUtils.Scale), y = Math.Round(point.y * PathUtils.Scale);
                    if (!double.IsFinite(x) || !double.IsFinite(y)
                        || Math.Abs(x) >= MaximumAbsoluteNativeCoordinate || Math.Abs(y) >= MaximumAbsoluteNativeCoordinate)
                    { valid = false; break; }
                    path.Add(new((long)x, (long)y));
                }
                if (!valid) break;
                native.Add(path);
            }
        }
        if (valid)
        {
            try
            {
                long phase = Stopwatch.GetTimestamp();
                bool prepared = VCarveConditionedCorrectionSource.TryCreate(native, token, null,
                    out var construction, out reason);
                preparationMs = Stopwatch.GetElapsedTime(phase).TotalMilliseconds;
                if (prepared)
                {
                    conditionedPoints = construction!.NominationPoints;
                    originalHash = construction.OriginalHash; constructionHash = construction.NominationHash;
                    var boundedSource = construction.Crop(Clipper.GetBounds(native), token);
                    // The original opening radius, round join/caps, arc1 and
                    // eight-tick erosion/dilation reserves stay literal.
                    phase = Stopwatch.GetTimestamp();
                    var reachable = dilationRadius is { } goalRadius
                        ? VCarveSmoothing.ReachableReferenceUpperForPlane(boundedSource, radius, goalRadius, token)
                        : VCarveSmoothing.ReachableReferenceUpper(boundedSource, radius, token);
                    openingMs = Stopwatch.GetElapsedTime(phase).TotalMilliseconds;
                    phase = Stopwatch.GetTimestamp();
                    var masked = PolygonOps.Intersect(goalMask, reachable);
                    maskingMs = Stopwatch.GetElapsedTime(phase).TotalMilliseconds;
                    token.ThrowIfCancellationRequested();
                    if (masked.Count != 0)
                    { result = masked; reason = "complete-conditioned-opening-goal"; }
                    else reason = "conditioned-whole-goal-empty";
                }
            }
            catch (OperationCanceledException) { throw; }
            catch (Exception error) when (error is ArgumentException or InvalidOperationException or OverflowException)
            { reason = "conditioned-construction-" + error.GetType().Name; }
        }
        token.ThrowIfCancellationRequested();
        conditioned = result is not null;
        if (!conditioned)
        {
            // Ordered original source, original opening and original mask.
            // Never publish a partial or empty conditioned goal as fallback.
            var reachable = dilationRadius is { } goalRadius
                        ? VCarveSmoothing.ReachableReferenceUpperForPlane(originalTarget, radius, goalRadius, token)
                        : VCarveSmoothing.ReachableReferenceUpper(originalTarget, radius, token);
            result = PolygonOps.Intersect(goalMask, reachable);
        }
        token.ThrowIfCancellationRequested();
        diagnostic?.Invoke($"stage=conditioned-reference-opening event=fixed-goal applied={conditioned} reason={reason} " +
            $"originalPoints={points} constructionPoints={conditionedPoints} goalPoints={result!.Sum(r => (long)r.Count)} " +
            $"goalRings={result.Count} erosionRadius={radius:R} dilationRadius={dilationRadius ?? radius:R} finiteChainMoveNative={VCarveConditionedCorrectionSource.MaximumGridMove} " +
            $"preparationMs={preparationMs:F3} openingMs={openingMs:F3} maskingMs={maskingMs:F3} " +
            $"elapsedMs={Stopwatch.GetElapsedTime(started).TotalMilliseconds:F3} originalHash={originalHash} constructionHash={constructionHash} " +
            "goalPolicy=conditioned-original0-erosion-source-opening-masked-by-goal-reference-target arcNative=1 openingReserveNative=8 " +
            "originalSourceChannelFloorArtworkAndGradeRetained=True goalFrozenBeforeCorrection=True " +
            "completeWholeGoalCertificateRequired=True originalOpeningSupersetClaim=False " +
            "finiteChainBoundIsNotFinishedStockBound=True globalVisibilityAccepted=False unsupportedUsesWholeLiteralGoal=True");
        token.ThrowIfCancellationRequested();
        return result;
    }
}
