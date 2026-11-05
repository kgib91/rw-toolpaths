using System.Collections.Frozen;
namespace RW.Toolpaths.Milling;

/// <summary>Topology of the complete immutable source, retained while some
/// branches are replaced by native geometry. A remaining fitting chunk does
/// not define a new graph or promote all of its ends to authored junctions.</summary>
public sealed class VCarveNativeTopologyContext
{
    readonly FrozenDictionary<Point3D,int> degree;
    readonly FrozenSet<Point3D> visitPorts;
    public long SourcePointCount {get;}
    VCarveNativeTopologyContext(Dictionary<Point3D,HashSet<Point3D>> neighbours,HashSet<Point3D> ports,long count)
    {degree=neighbours.ToFrozenDictionary(p=>p.Key,p=>p.Value.Count);visitPorts=ports.ToFrozenSet();SourcePointCount=count;}
    public int Degree(Point3D point)=>degree.TryGetValue(point,out int value)?value:throw new ArgumentException("Native topology does not own this source position");
    public bool IsVisitPort(Point3D point)=>visitPorts.Contains(point);
    public static VCarveNativeTopologyContext Create(IReadOnlyList<TaggedToolpath> source,CancellationToken token=default)
    {
        var neighbours=new Dictionary<Point3D,HashSet<Point3D>>();var ports=new HashSet<Point3D>();long count=0;
        foreach(var path in source)
        {
            token.ThrowIfCancellationRequested();count+=path.Points.Count;
            foreach(var point in path.Points)
            {
                if(!double.IsFinite(point.X)||!double.IsFinite(point.Y)||!double.IsFinite(point.Z))throw new ArgumentException("Native topology coordinates must be finite");
                neighbours.TryAdd(point,[]);
            }
            if(path.Points.Count>0){ports.Add(path.Points[0]);ports.Add(path.Points[^1]);}
            for(int i=1;i<path.Points.Count;i++){var a=path.Points[i-1];var b=path.Points[i];neighbours[a].Add(b);neighbours[b].Add(a);}
        }
        return new(neighbours,ports,count);
    }
}
