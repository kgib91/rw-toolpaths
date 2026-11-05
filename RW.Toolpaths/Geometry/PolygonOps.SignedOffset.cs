using Clipper2Lib;
using System.Diagnostics;

namespace RW.Toolpaths.Geometry;

public static partial class PolygonOps
{
    private static readonly FilledFrontEvidenceCache.Cache<CenterlineBufferEvidence> SignedOffsetClusterCache=new(32L<<20,1024);
    private static long signedOffsetClusterRefusals;
    private readonly record struct OffsetBox(long Left,long Bottom,long Right,long Top)
    {
        internal bool Intersects(OffsetBox b)=>Left<=b.Right&&Right>=b.Left&&Bottom<=b.Top&&Top>=b.Bottom;
        internal bool Contains(OffsetBox b)=>Left<=b.Left&&Right>=b.Right&&Bottom<=b.Bottom&&Top>=b.Top;
    }

    // Round-offset vertices lie within delta of their original contour (plus
    // integer rounding). Disjoint padded contour boxes therefore cannot share
    // offset primitives or intersections. Every component retains the SAME
    // original positive rectangle as its winding context; no hole is offset as
    // an independently filled polygon. Unsupported contexts retain the literal
    // complete offset. No source simplification or tolerance changes occur.
    private static bool TryBufferSeparatedSignedClusters(Paths64 source,double radius,double arcTolerance,out Paths64 result)
    {
        result=new();
        if(source.Count<4||source.Count>2048||GeometryParallelism.WorkerLimit<=1)return false;
        long started=Stopwatch.GetTimestamp();
        bool Refuse(string reason)
        {
            if(source.Sum(p=>p.Count)>40000&&Interlocked.Increment(ref signedOffsetClusterRefusals)<=8)
                Console.WriteLine($"[PolygonOps.SignedOffsetClusters] admitted=False reason={reason} sourcePaths={source.Count} "
                    +$"sourcePoints={source.Sum(p=>p.Count)} fallback=literal-complete-offset");
            return false;
        }
        int root=-1;
        for(int i=0;i<source.Count;i++)
        {
            var p=source[i];if(p.Count!=4||Clipper.Area(p)<=0)continue;
            bool rectangle=true;
            for(int j=0;j<4;j++)
                rectangle&=(p[j].X==p[(j+1)%4].X)!=(p[j].Y==p[(j+1)%4].Y);
            if(rectangle&&(root<0||Clipper.Area(p)>Clipper.Area(source[root])))root=i;
        }
        if(root<0)return Refuse("no-positive-rectangle-context");
        static OffsetBox BoundsOf(Path64 p,long pad=0)
        {
            long l=long.MaxValue,b=l,r=long.MinValue,t=r;
            foreach(var q in p){l=Math.Min(l,q.X);b=Math.Min(b,q.Y);r=Math.Max(r,q.X);t=Math.Max(t,q.Y);}
            return new(l-pad,b-pad,r+pad,t+pad);
        }
        var frame=BoundsOf(source[root]);
        if(frame.Left<-1_000_000_000_000||frame.Bottom<-1_000_000_000_000
            ||frame.Right>1_000_000_000_000||frame.Top>1_000_000_000_000)return false;
        long padding=(long)Math.Ceiling(radius*PathUtils.Scale)+8;
        var entries=new List<(int Index,OffsetBox Bounds)>();
        for(int i=0;i<source.Count;i++)if(i!=root)
        {
            if(source[i].Count<3)return false;
            var bounds=BoundsOf(source[i]);
            if(!frame.Contains(bounds)||bounds.Bottom<=frame.Bottom||bounds.Top>=frame.Top
                ||bounds.Left<=frame.Left||bounds.Right>=frame.Right)return Refuse("contour-not-strictly-within-rectangle");
            entries.Add((i,BoundsOf(source[i],padding)));
        }
        int[] parents=Enumerable.Range(0,entries.Count).ToArray();
        int Find(int i){while(parents[i]!=i){parents[i]=parents[parents[i]];i=parents[i];}return i;}
        for(int i=0;i<entries.Count;i++)for(int j=0;j<i;j++)
            if(entries[i].Bounds.Intersects(entries[j].Bounds))parents[Find(i)]=Find(j);
        var clusters=Enumerable.Range(0,entries.Count).GroupBy(Find).Select(g=>g.ToArray()).ToArray();
        if(clusters.Length<2)return Refuse("one-coupled-cluster");
        Paths64 Offset(Paths64 paths)
        {
            var builder=CreateOffsetter(arcTolerance);builder.AddPaths(paths,JoinType.Round,EndType.Polygon);
            var solution=new Paths64();builder.Execute(radius*PathUtils.Scale,solution);return solution;
        }
        static bool SameCycle(Path64 a,Path64 b)
        {
            if(a.Count!=b.Count)return false;
            for(int first=0;first<b.Count;first++)if(a[0]==b[first])
            {
                bool same=true;for(int i=1;i<a.Count&&same;i++)same=a[i]==b[(first+i)%b.Count];
                if(same)return true;
            }
            return false;
        }
        var frameSolution=Offset(new Paths64{source[root]});
        if(frameSolution.Count!=1||Clipper.Area(frameSolution[0])<=0)return false;
        var outputs=new Paths64[clusters.Length];long hits=0;
        int workers=Math.Min(GeometryParallelism.WorkerLimit,clusters.Length);
        Parallel.For(0,clusters.Length,new ParallelOptions{MaxDegreeOfParallelism=workers},cluster=>{
            using var scope=GeometryParallelism.EnterWorker();
            var paths=new Paths64{source[root]};
            paths.AddRange(clusters[cluster].Select(i=>source[entries[i].Index]));
            var key=FilledFrontEvidenceCache.Enabled?FilledFrontEvidenceCache.Key.Capture(
                FromPaths64(paths),radius,arcTolerance,0,13,null,default):null;
            var cached=SignedOffsetClusterCache.Find(key,default);
            if(cached is not null){outputs[cluster]=ToPaths64(cached.Copy());Interlocked.Increment(ref hits);return;}
            var next=Offset(paths);outputs[cluster]=next;
            if(key is not null)
            {
                var evidence=new CenterlineBufferEvidence(FromPaths64(next));
                SignedOffsetClusterCache.Publish(key,evidence,evidence.Bytes,default);
            }
        });
        result=new Paths64{frameSolution[0]};
        for(int cluster=0;cluster<clusters.Length;cluster++)
        {
            var ids=clusters[cluster];
            var bounds=new OffsetBox(ids.Min(i=>entries[i].Bounds.Left),ids.Min(i=>entries[i].Bounds.Bottom),
                ids.Max(i=>entries[i].Bounds.Right),ids.Max(i=>entries[i].Bounds.Top));
            bool foundFrame=false;
            foreach(var path in outputs[cluster])
            {
                if(!foundFrame&&SameCycle(path,frameSolution[0])){foundFrame=true;continue;}
                if(!bounds.Contains(BoundsOf(path)))return Refuse("component-outside-proved-support");
                result.Add(path);
            }
            if(!foundFrame)return Refuse("changed-rectangle-frame");
        }
        Console.WriteLine($"[PolygonOps.SignedOffsetClusters] sourcePaths={source.Count} sourcePoints={source.Sum(p=>p.Count)} "
            +$"clusters={clusters.Length} workers={workers} cacheHits={hits} resultPoints={result.Sum(p=>p.Count)} "
            +$"elapsedMs={Stopwatch.GetElapsedTime(started).TotalMilliseconds:F1} radius={radius:R} arcTolerance={arcTolerance:R} "
            +$"supportPaddingTicks={padding} cacheByteLimit=33554432 originalPositiveRectangleContext=True disjointOriginalRoundPrimitiveSupports=True "
            +$"originalFrameCycleVerified=True originalSignedContours=True sourceVerticesUnchanged=True fallback=literal-complete-offset");
        return true;
    }
}
