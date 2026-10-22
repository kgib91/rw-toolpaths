using System.Diagnostics;
using Clipper2Lib;
using RW.Toolpaths.Geometry;

namespace RW.Toolpaths.Milling;

/// <summary>Compact a roughing exclusion, then grow it to contain ALL original
/// protected material. The finite boundary capsules bound the added stock;
/// a signed filled-set inclusion verifies that no protection was lost.</summary>
internal static class ConservativeSurfaceMask
{
    internal static Paths64 Compact(Paths64 original,CancellationToken token)
    {
        token.ThrowIfCancellationRequested();
        int before=original.Sum(p=>p.Count);
        if(before<2048)return original;
        long started=Stopwatch.GetTimestamp();
        const double chord=.003,arc=.000125;
        double expansion=chord+arc+8/PathUtils.Scale;
        var proposed=new Paths64();
        foreach(var ring in original)
        {
            token.ThrowIfCancellationRequested();
            if(ring.Count<3){proposed.Add(ring);continue;}
            var points=ring.Select(p=>new Point3D(p.X/PathUtils.Scale,p.Y/PathUtils.Scale,0)).ToList();
            points.Add(points[0]);
            var path=new TaggedToolpath(points,Array.Empty<ToolpathSpan>(),0,"roughing-exclusion",null,detectCurves:false);
            var simple=SurfaceToolpathSimplifier.Simplify(path,chord,token);
            proposed.Add(new Path64(simple.Points.Take(simple.Points.Count-1)
                .Select(p=>new Point64(p.X*PathUtils.Scale,p.Y*PathUtils.Scale))));
        }
        proposed=Clipper.Union(proposed,FillRule.NonZero);
        var offset=new ClipperOffset(2,arc*PathUtils.Scale);
        offset.AddPaths(proposed,JoinType.Round,EndType.Polygon);
        var expanded=new Paths64();offset.Execute(expansion*PathUtils.Scale,expanded);
        token.ThrowIfCancellationRequested();
        int after=expanded.Sum(p=>p.Count);
        bool smaller=after<before*.8;
        bool contains=smaller&&Clipper.Difference(original,expanded,FillRule.NonZero).Count==0;
        token.ThrowIfCancellationRequested();
        Console.WriteLine($"[Pocket.SmoothRoughing.SurfaceMask] sourcePoints={before} proposedPoints={after} "
            +$"accepted={contains} originalProtectionFullyContained={contains} chordMm={chord:R} expansionMm={expansion:R} "
            +$"maximumAdditionalStockMm={2*chord+arc+12/PathUtils.Scale:R} elapsedMs={Stopwatch.GetElapsedTime(started).TotalMilliseconds:F1} "
            +"authority=signed-original-filled-set-inclusion removedProtectedMaterial=0 fallback=original-mask");
        return contains?expanded:original;
    }
}
