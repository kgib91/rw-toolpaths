namespace RW.Toolpaths.Milling;

/// <summary>Positive whole-cutter clearance on the literal XY chords of an
/// already completed circuit. An inscribed cylinder in the deeper physical
/// sweep must contain the entire inflated raised cutter. No endpoint merging,
/// sampled tip disks, or credit from a future circuit is permitted.</summary>
internal sealed class CompletedCircuitStock
{
    private readonly IReadOnlyList<TaggedToolpath> source;
    private readonly VCarveCutterProfile profile;
    private readonly Func<Point3D,Point3D> command;
    private readonly CancellationToken token;
    private readonly Dictionary<(double,double,double,double),double>?[] chords;
    internal long Queries {get;private set;}
    internal long Accepted {get;private set;}
    internal int IndexedCircuits {get;private set;}
    internal CompletedCircuitStock(IReadOnlyList<TaggedToolpath> source,VCarveCutterProfile profile,
        Func<Point3D,Point3D> command,CancellationToken token)
    {this.source=source;this.profile=profile;this.command=command;this.token=token;chords=new Dictionary<(double,double,double,double),double>?[source.Count];}

    internal bool ContainsRaisedChord(int owner,Point3D a,Point3D b,double clearance)
    {
        if((++Queries&1023)==0)token.ThrowIfCancellationRequested();
        var index=chords[owner];
        if(index is null)
        {
            index=new();var points=source[owner].Points;
            var previous=command(points[0]);
            for(int i=1;i<points.Count;i++)
            {
                if((i&1023)==0)token.ThrowIfCancellationRequested();
                var next=command(points[i]);
                var key=Key(points[i-1],points[i]);double depth=Math.Min(-previous.Z,-next.Z);
                index[key]=Math.Max(depth,index.GetValueOrDefault(key));previous=next;
            }
            chords[owner]=index;IndexedCircuits++;
        }
        if(!index.TryGetValue(Key(a,b),out double oldDepth))return false;
        var ca=command(a);var cb=command(b);
        double queryDepth=Math.Max(-ca.Z,-cb.Z)+clearance;
        bool contains=clearance>0 && oldDepth>queryDepth
            && profile.PhysicalRadiusAtDepth(oldDepth-queryDepth)
                >=profile.PhysicalRadiusAtDepth(queryDepth)+clearance+1e-8;
        if(contains)Accepted++;
        return contains;
    }
    private static (double,double,double,double) Key(Point3D a,Point3D b)
        => a.X<b.X || a.X==b.X&&a.Y<=b.Y ? (a.X,a.Y,b.X,b.Y) : (b.X,b.Y,a.X,a.Y);
}
