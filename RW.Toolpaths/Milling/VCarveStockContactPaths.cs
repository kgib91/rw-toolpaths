namespace RW.Toolpaths.Milling;

/// <summary>Cutting-source projection only. Safe entry/retract belongs to the
/// later emitter. One immutable stock owner governs crests, cutting sources,
/// and the smooth artwork proof; authored floor and channel remain separate.</summary>
internal static class VCarveStockContactPaths
{
    internal static List<TaggedToolpath> Project(IReadOnlyList<TaggedToolpath> paths,
        VCarveMaterialSurface owner,double tolerance,CancellationToken token)
    {
        var poses=new Dictionary<Point3D,Point3D>();
        Point3D Contact(Point3D p)
        {if(!poses.TryGetValue(p,out var q))poses.Add(p,q=owner.ProjectContactPose(p));return q;}
        var output=new List<TaggedToolpath>(paths.Count);
        foreach(var path in paths)
        {
            token.ThrowIfCancellationRequested();
            if(path.Points.Count==0){output.Add(path);continue;}
            var points=new List<Point3D>{Contact(path.Points[0])};
            for(int i=1;i<path.Points.Count;i++)
                points.AddRange(owner.ProjectCanonicalContactEdge(path.Points[i-1],path.Points[i],
                    Contact(path.Points[i-1]),Contact(path.Points[i]),tolerance).Skip(1));
            output.Add(path with{Points=points,Moves=new LineToolMoves(points),Spans=Array.Empty<ToolpathSpan>()});
        }
        return output;
    }
}
