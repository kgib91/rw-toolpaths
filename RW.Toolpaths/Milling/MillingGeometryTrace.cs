namespace RW.Toolpaths.Milling;

/// <summary>Read-only, per-execution stage snapshots for diagnosing where a
/// geometric feature first appears. Production leaves the observer null.</summary>
internal static class MillingGeometryTrace
{
    internal static readonly AsyncLocal<Action<string,int,IReadOnlyList<TaggedToolpath>>?> Observer=new();
    internal static void Capture(string stage,int batch,IReadOnlyList<TaggedToolpath> paths)
        =>Observer.Value?.Invoke(stage,batch,paths);
}
