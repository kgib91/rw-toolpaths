namespace RW.Toolpaths;

/// <summary>What the cutter is doing over a run of a toolpath.</summary>
public enum ToolpathSpanKind
{
    /// <summary>Removing material at the pass depth.</summary>
    Cut = 0,

    /// <summary>Descending into the material; feed is limited by the plunge rate.</summary>
    Ramp = 1,

    /// <summary>Lifted over a holding tab, leaving material behind to retain the part.</summary>
    TabLift = 2,

    /// <summary>Connecting move between rings at cutting depth.</summary>
    Link = 3,

    /// <summary>Lifted clear of the material to reposition.</summary>
    Retract = 4,
}

/// <summary>
/// Classifies a run of a toolpath, so downstream code can assign feed rates and recognise tab
/// lifts without re-deriving them from the geometry.
///
/// <para>
/// Indices are inclusive at both ends and consecutive spans share a boundary point, so the move
/// leading into a span belongs to that span. A ramp therefore owns its descent and the following
/// cut owns the move that carries the cutter away at depth.
/// </para>
/// </summary>
public readonly record struct ToolpathSpan(int StartIndex, int EndIndex, ToolpathSpanKind Kind)
{
    /// <summary>Number of moves this span owns.</summary>
    public int MoveCount => Math.Max(0, EndIndex - StartIndex);

    /// <summary>Number of points this span covers, including both shared endpoints.</summary>
    public int PointCount => Math.Max(0, EndIndex - StartIndex + 1);
}
