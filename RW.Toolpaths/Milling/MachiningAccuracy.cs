namespace RW.Toolpaths.Milling;

/// <summary>Shared construction/posting defaults. Operation quality is configured
/// on each request; these defaults are not a global finished-surface requirement.</summary>
public static class MachiningAccuracy
{
    public const string Policy = "operation-owned-approximation-v1";
    // Approximation/finish allowances are operation-owned, not a global rule.
    public const double ArtworkChord = .001;
    public const double FinalPathChord = .002;
    internal const double PathChord = FinalPathChord;
    internal const double ProtectedChord = .002;
    internal const double SmoothCoverage = .008;
    internal const double FloorSearch = .0002;
}
