using Clipper2Lib;

namespace RW.Toolpaths.Geometry;

/// <summary>The final command-front audit's closed sweep, with its complete
/// immutable ordered construction identity. This owner is created only after
/// the selected front passes its final audit; rejected trial sweeps are never
/// published as executed-history evidence.</summary>
internal sealed class PreparedClosedSweep
{
    private readonly PointD[][] _commands;
    private readonly PointD[]? _clip;
    private readonly double _radius, _arc;
    private readonly bool _explicitChords;
    private readonly List<List<PointD>> _sweep;

    internal PreparedClosedSweep(IReadOnlyList<List<PointD>> commands,
        List<List<PointD>> auditedSweep, double radius, double arc, bool explicitChords,
        IReadOnlyList<PointD>? clip, CancellationToken token)
    {
        token.ThrowIfCancellationRequested();
        _commands = new PointD[commands.Count][];
        for (int i = 0; i < commands.Count; i++) _commands[i] = Copy(commands[i], token);
        _clip = clip is null ? null : Copy(clip, token);
        _radius = radius; _arc = arc; _explicitChords = explicitChords;
        // Takes ownership of the final audit's result. Boolean/history readers
        // convert this owner to fresh integer input; they never mutate its paths.
        _sweep = auditedSweep;
    }

    internal bool TryReuse(IReadOnlyList<List<PointD>> commands, double radius,
        double arc, bool explicitChords, IReadOnlyList<PointD>? clip,
        CancellationToken token, out List<List<PointD>>? sweep)
    {
        token.ThrowIfCancellationRequested();
        sweep = null;
        if (!double.IsFinite(radius) || !double.IsFinite(arc)
            || Bits(radius) != Bits(_radius) || Bits(arc) != Bits(_arc)
            || explicitChords != _explicitChords || commands.Count != _commands.Length
            || (_clip is null) != (clip is null)) return false;
        if (_clip is not null && !Same(_clip, clip!, token)) return false;
        for (int i = 0; i < _commands.Length; i++)
            if (!Same(_commands[i], commands[i], token)) return false;
        // Cardinality and exact order are part of the identity: native open
        // sweep chunk boundaries and closing chords may depend on both.
        sweep = _sweep;
        return true;
    }

    private static PointD[] Copy(IReadOnlyList<PointD> path, CancellationToken token)
    {
        var copy = new PointD[path.Count];
        for (int i = 0; i < path.Count; i++)
        {
            if ((i & 1023) == 0) token.ThrowIfCancellationRequested();
            copy[i] = path[i];
        }
        return copy;
    }

    private static bool Same(IReadOnlyList<PointD> expected, IReadOnlyList<PointD> actual,
        CancellationToken token)
    {
        if (expected.Count != actual.Count) return false;
        for (int i = 0; i < expected.Count; i++)
        {
            if ((i & 1023) == 0) token.ThrowIfCancellationRequested();
            var a = expected[i]; var b = actual[i];
            if (!double.IsFinite(b.x) || !double.IsFinite(b.y)
                || Bits(a.x) != Bits(b.x) || Bits(a.y) != Bits(b.y)) return false;
        }
        return true;
    }

    private static long Bits(double value) => BitConverter.DoubleToInt64Bits(value);

    internal static bool SameIntegerSweep(IEnumerable<IReadOnlyList<PointD>> actual,
        IEnumerable<IReadOnlyList<PointD>> expected, CancellationToken token)
    {
        token.ThrowIfCancellationRequested();
        var a = PathUtils.ToClipper(actual); var b = PathUtils.ToClipper(expected);
        if (a.Count != b.Count) return false;
        for (int i = 0; i < a.Count; i++)
        {
            if (a[i].Count != b[i].Count) return false;
            for (int j = 0; j < a[i].Count; j++)
            {
                if ((j & 1023) == 0) token.ThrowIfCancellationRequested();
                if (a[i][j].X != b[i][j].X || a[i][j].Y != b[i][j].Y) return false;
            }
        }
        return true;
    }
}
