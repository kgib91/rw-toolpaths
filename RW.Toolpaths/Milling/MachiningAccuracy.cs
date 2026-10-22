namespace RW.Toolpaths.Milling;

/// <summary>Millimetre error budgets for the finished surface, independently
/// of the point inventory or construction algorithm used to reach it.</summary>
public static class MachiningAccuracy
{
    public const string Policy = "surface-20um-v14-continuous-crest";
    public const double SurfaceDeviation = .02;
    public const double ArtworkChord = .001;
    internal const double PathChord = .002;
    internal const double ProtectedChord = .002;
    internal const double SmoothCoverage = .008;
    internal const double FloorSearch = .0002;
}
