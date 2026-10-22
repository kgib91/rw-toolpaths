namespace RW.Toolpaths.Milling;

// Transient initial NOM role tags. The selected inventory is always passed to
// the original physical floor, artwork and grade owners before routing.
internal static class VCarveInitialReachSelection
{
    internal const string HalfCoreMarkerCategory = "clearing-half-core-nomination";
    internal const string QuarterOuterMarkerCategory = "clearing-quarter-outer-nomination";
    internal const string EighthCoreMarkerCategory = "clearing-eighth-core-nomination";
    internal static bool IsSupplemental(TaggedToolpath path) => path.Category is
        HalfCoreMarkerCategory or QuarterOuterMarkerCategory or EighthCoreMarkerCategory;
    internal static List<TaggedToolpath> NormalizeMarkers(IReadOnlyList<TaggedToolpath> source,
        CancellationToken token = default)
    {
        var result = new List<TaggedToolpath>(source.Count);
        foreach (var path in source)
        {
            token.ThrowIfCancellationRequested();
            result.Add(IsSupplemental(path) ? path with { Category = "clearing" } : path);
        }
        token.ThrowIfCancellationRequested(); return result;
    }
}
