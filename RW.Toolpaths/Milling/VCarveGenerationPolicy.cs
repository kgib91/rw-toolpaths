using System.Text;

namespace RW.Toolpaths.Milling;

/// <summary>
/// Immutable requested smooth V-carve nominations. Capture before cache lookup;
/// the same snapshot follows the request into generation and device factories.
/// These switches select proposals, never physical stock or coverage authority.
/// </summary>
public sealed class VCarveGenerationPolicy
{
    // Keep the established GPU cache bits, including the retired 6..8 slots.
    private static readonly (string Name, int Bit)[] GpuModes = {
        ("RW_TOOLPATHS_VCARVE_GPU", 1),
        ("RW_TOOLPATHS_VCARVE_GPU_FP32", 2),
        ("RW_TOOLPATHS_VCARVE_GPU_TRIANGLES", 4),
        ("RW_TOOLPATHS_VCARVE_GPU_BOUNDARY", 8),
        ("RW_TOOLPATHS_VCARVE_GPU_BOUNDARY_FP32", 16),
        ("RW_TOOLPATHS_VCARVE_GPU_FRONTS", 32),
        ("RW_TOOLPATHS_GPU_GOLDEN_REFERENCE", 512) };
    private static readonly string[] InitialModes = {
        Acceleration.RawInitialClearingInsetNomination.EnvironmentName,
        "RW_TOOLPATHS_VCARVE_INITIAL_FRONT_CONDITIONING",
        "RW_TOOLPATHS_VCARVE_INITIAL_STOCK_COHORT",
        "RW_TOOLPATHS_VCARVE_SUPPLEMENTAL_INITIAL_BAND",
        "RW_TOOLPATHS_VCARVE_PARALLEL_SWEEP_UNIONS" };

    public bool GpuDiscWitnesses => (GpuMask & 1) != 0;
    public bool GpuFp32 => (GpuMask & 2) != 0;
    public bool GpuTriangles => (GpuMask & 4) != 0;
    public bool GpuBoundary => (GpuMask & 8) != 0;
    public bool GpuBoundaryFp32 => (GpuMask & 16) != 0;
    public bool GpuFronts => (GpuMask & 32) != 0;
    public bool GpuGoldenReference => (GpuMask & 512) != 0;
    public int GpuMask { get; }
    public bool FloorAwareRamps { get; }
    public bool ContourSeeds { get; }
    public bool TaperedSweeps { get; }
    public bool LowerSweepBudget { get; }
    public bool ActualF3Compaction { get; }
    public bool BoxNeighborhoods { get; }
    public bool ConditionedCorrectionSource { get; }
    public bool ArtworkFringeGoal { get; }
    public bool MaterialLayerGoal { get; }
    public bool ConditionedReferenceOpening { get; }
    internal int ExplicitInitialMask { get; }
    internal bool OriginalInitialRequested { get; }
    internal bool InvalidInitialRequest { get; }
    internal bool RawInitialInset => (ExplicitInitialMask & 1) != 0;
    internal bool InitialFrontConditioning => (ExplicitInitialMask & 2) != 0;
    internal bool InitialStockCohort => (ExplicitInitialMask & 4) != 0;
    internal bool SupplementalInitialBand => (ExplicitInitialMask & 8) != 0;
    internal bool ParallelSweepUnions => (ExplicitInitialMask & 16) != 0;

    private VCarveGenerationPolicy()
    {
        foreach (var mode in GpuModes)
            if (Enabled(mode.Name)) GpuMask |= mode.Bit;
        FloorAwareRamps = Enabled("RW_TOOLPATHS_VCARVE_FLOOR_AWARE_RAMPS");
        ContourSeeds = Enabled("RW_TOOLPATHS_VCARVE_CONTOUR_SEEDS");
        TaperedSweeps = Enabled("RW_TOOLPATHS_VCARVE_TAPERED_SWEEPS");
        LowerSweepBudget = Enabled("RW_TOOLPATHS_VCARVE_LOWER_SWEEP_BUDGET");
        ActualF3Compaction = Enabled("RW_TOOLPATHS_VCARVE_ACTUAL_F3_COMPACTION");
        BoxNeighborhoods = Enabled("RW_TOOLPATHS_VCARVE_BOX_NEIGHBORHOODS");
        ConditionedCorrectionSource = Enabled(VCarveConditionedCorrectionSource.SelectionEnvironment);
        ArtworkFringeGoal = Enabled(VCarveArtworkFringeGoal.SelectionEnvironment);
        MaterialLayerGoal = Enabled(VCarveMaterialLayerGoal.SelectionEnvironment);
        ConditionedReferenceOpening = Enabled(VCarveConditionedReferenceOpeningGoal.SelectionEnvironment);
        for (int i = 0; i < InitialModes.Length; i++)
        {
            string? value = Environment.GetEnvironmentVariable(InitialModes[i]);
            if (value == "0") OriginalInitialRequested = true;
            else if (value == "1") ExplicitInitialMask |= 1 << i;
            else if (!string.IsNullOrEmpty(value)) InvalidInitialRequest = true;
        }
    }

    public static VCarveGenerationPolicy Capture() => new();

    /// <summary>Preserves existing active-policy signature bytes and ordering.</summary>
    public string CacheSignature(VCarveInitialStrategy? initialStrategy = null)
    {
        var signature = new StringBuilder();
        if (GpuMask != 0) signature.Append($"|gpuNominationModes-v1:{GpuMask:X3}");
        if (initialStrategy is not null) signature.Append(initialStrategy.CacheSignature);
        Append(FloorAwareRamps, "floorAwareCorrectionRamps-v1");
        Append(ContourSeeds, "wholeTargetContourBoxSeeds-v1");
        Append(LowerSweepBudget, "containedLowerSweepBudget-v1");
        Append(ActualF3Compaction, "guardedActualF3TrajectoryCompaction-v2");
        Append(initialStrategy?.RawInitialInset ?? RawInitialInset, "rawInitialClearingInsetNomination-v2");
        Append(initialStrategy?.InitialFrontConditioning ?? InitialFrontConditioning, "conditionedInitialClearingFrontNomination-v1");
        Append(ConditionedCorrectionSource, "conditionedCorrectionSourceNomination-v1");
        Append(ArtworkFringeGoal, "originalArtworkFringeGoal-v1");
        Append(initialStrategy?.ParallelSweepUnions ?? ParallelSweepUnions, "parallelPhysicalSweepLeafUnions-v1");
        Append(MaterialLayerGoal, "materialLayerStockGoal-v2");
        Append(ConditionedReferenceOpening, "conditionedReferenceOpeningGoal-v1");
        Append(initialStrategy?.InitialStockCohort ?? InitialStockCohort, "initialProtectedStockCohort-v1");
        Append(initialStrategy?.SupplementalInitialBand ?? SupplementalInitialBand, "fullDepthSupplementalReach-v10");
        Append(TaperedSweeps, "taperedLowerSurfaceSweeps-v1");
        Append(BoxNeighborhoods, "componentBoxCorrectionNomination-v3");
        return signature.ToString();

        void Append(bool enabled, string marker)
        {
            if (enabled) signature.Append('|').Append(marker);
        }
    }

    private static bool Enabled(string name) => Environment.GetEnvironmentVariable(name) == "1";
}
