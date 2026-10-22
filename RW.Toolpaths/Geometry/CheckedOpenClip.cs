using Clipper2Lib;

namespace RW.Toolpaths.Geometry;

/// <summary>Failed open Booleans never return partial commanded paths or removal evidence.</summary>
internal static class CheckedOpenClip
{
    internal delegate bool Execute(Clipper64 engine, Paths64 closed, Paths64 open);

    internal static bool Try(IEnumerable<IReadOnlyList<PointD>> paths,
        IEnumerable<IReadOnlyList<PointD>> clips, ClipType operation, CancellationToken token,
        out List<List<PointD>> output, Execute? execute = null)
    {
        token.ThrowIfCancellationRequested();
        var engine = new Clipper64(); engine.AddOpenSubject(PathUtils.ToClipper(paths));
        engine.AddClip(PathUtils.ToClipper(clips)); var result = new Paths64();
        bool completed = execute is null
            ? engine.Execute(operation, FillRule.NonZero, new Paths64(), result)
            : execute(engine, new Paths64(), result);
        token.ThrowIfCancellationRequested();
        output = completed ? PathUtils.FromClipper(result) : new List<List<PointD>>();
        return completed;
    }

    internal static List<List<PointD>> Require(IEnumerable<IReadOnlyList<PointD>> paths,
        IEnumerable<IReadOnlyList<PointD>> clips, ClipType operation, CancellationToken token,
        string owner, Execute? execute = null)
    {
        if (Try(paths, clips, operation, token, out var output, execute)) return output;
        Console.Error.WriteLine($"[Geometry.OpenClipRefusal] owner={owner} operation={operation} "
            + "completed=False partialPathsCredited=False decision=abort");
        throw new InvalidOperationException($"{owner}: open clipping could not be completed; partial paths cannot be emitted or credited.");
    }
}
