using System.Diagnostics;
using Clipper2Lib;
using RW.Toolpaths.Geometry;

namespace RW.Toolpaths.Milling;

/// <summary>Conservative planar removal from actual preceding posted flat
/// cutter columns. A declared starting floor never supplies removal.</summary>
internal sealed class VCarvePostedFloorSweep
{
    private readonly VCarveRoughingChord[] source;
    private readonly Dictionary<double,IReadOnlyList<IReadOnlyList<PointD>>> floors=new();

    internal VCarvePostedFloorSweep(IEnumerable<VCarveRoughingChord> chords)
        => source=chords.Where(c=>Posted(c.A)&&Posted(c.B)
            &&double.IsFinite(c.ClearedColumnRadius)&&c.ClearedColumnRadius>0).ToArray();

    internal IReadOnlyList<IReadOnlyList<PointD>> AtDepth(double depth,CancellationToken token,Action<string>? diagnostic)
    {
        token.ThrowIfCancellationRequested();
        if(floors.TryGetValue(depth,out var cached))return cached;
        long started=Stopwatch.GetTimestamp();
        var swept=new Paths64(); int credited=0;
        foreach(var group in source.Where(c=>Math.Max(c.A.Z,c.B.Z)<=-depth+1e-9).GroupBy(c=>c.ClearedColumnRadius))
        {
            token.ThrowIfCancellationRequested();
            var columns=group.ToArray(); credited+=columns.Length;
            var segments=columns.Select(c=>new PocketPostedChordChains.Segment(
                new((long)Math.Round(c.A.X*1000),(long)Math.Round(c.A.Y*1000)),
                new((long)Math.Round(c.B.X*1000),(long)Math.Round(c.B.Y*1000)))).ToArray();
            var chains=PocketPostedChordChains.Build(segments,token);
            var paths=new Paths64(chains.Paths.Select(p=>new Path64(p.Select(v=>
                new Point64((long)Math.Round(v.X/1000d*PathUtils.Scale),(long)Math.Round(v.Y/1000d*PathUtils.Scale))))));
            paths.AddRange(columns.Where(c=>c.A.X==c.B.X&&c.A.Y==c.B.Y).Select(c=>
                new Path64 {new((long)Math.Round(c.A.X*PathUtils.Scale),(long)Math.Round(c.A.Y*PathUtils.Scale))}));
            if(paths.Count>0)swept.AddRange(PostedSweepInnerRegion.BuildPaths(paths,group.Key,token));
        }
        token.ThrowIfCancellationRequested();
        var result=PathUtils.FromClipper(Clipper.Union(swept,FillRule.NonZero));
        IReadOnlyList<IReadOnlyList<PointD>> value=result;
        floors.Add(depth,value);
        diagnostic?.Invoke($"stage=rest-fill-stock floorDepth={depth:R} sourceChords={source.Length} creditedChords={credited} "
            +$"sweepRings={result.Count} sweptArea={Math.Abs(result.Sum(PolygonOps.SignedArea)):R} "
            +$"elapsedMs={Stopwatch.GetElapsedTime(started).TotalMilliseconds:R} "
            +"evidence=actual-posted-full-column minimumDepth=both-chord-ends startingDepthCredit=False innerSweep=True");
        return value;
    }

    private static bool Posted(Point3D p)=>double.IsFinite(p.X+p.Y+p.Z)
        &&Math.Abs(p.X)<=10000&&Math.Abs(p.Y)<=10000&&Math.Abs(p.Z)<=100
        &&p.X==Math.Round(p.X*1000)/1000d&&p.Y==Math.Round(p.Y*1000)/1000d&&p.Z==Math.Round(p.Z*1000)/1000d;
}
