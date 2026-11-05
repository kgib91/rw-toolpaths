namespace RW.Toolpaths.Milling;

/// <summary>One source-span interpretation for planning, emission and timing.
/// Untagged moves are cuts; later explicit spans override earlier ones.</summary>
public static class ToolpathSpanKinds
{
    public static ToolpathSpanKind[] Expand(TaggedToolpath path)
    {
        var kinds = new ToolpathSpanKind[Math.Max(0, path.Points.Count - 1)];
        Array.Fill(kinds, ToolpathSpanKind.Cut);
        foreach (var span in path.Spans)
            for (int i = Math.Max(0, span.StartIndex); i < Math.Min(kinds.Length, span.EndIndex); i++)
                kinds[i] = span.Kind;
        return kinds;
    }
}
