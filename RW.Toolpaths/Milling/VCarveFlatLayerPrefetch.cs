using System.Diagnostics;
using Clipper2Lib;
using RW.Toolpaths.Geometry;

namespace RW.Toolpaths.Milling;

/// <summary>One optional, bounded preparation of flat prior-stock layer geometry.
/// No cutter query, profile sweep, clearance certificate or mutable query memo is shared.</summary>
public sealed class VCarveFlatLayerPrefetch : IDisposable
{
    internal const long MaximumOwnedBytes=64L*1024*1024;
    private readonly object gate=new();
    private readonly CancellationToken callerToken;
    private readonly CancellationTokenSource lifetime;
    private readonly Action<string>? diagnostic;
    private Task? work;
    private byte[]? stockKey,targetKey;
    private VCarveMaterialSurface.FlatLayerGeometry[]? completed;
    private bool started,disposed,consumed;
    private long ownedBytes;
    public VCarveFlatLayerPrefetch(CancellationToken token=default,Action<string>? diagnostic=null)
    {callerToken=token;lifetime=CancellationTokenSource.CreateLinkedTokenSource(token);this.diagnostic=diagnostic;}

    /// <summary>The target callback must own a frozen request. Its output is
    /// snapped before use; original construction still runs on any refusal.</summary>
    public bool Start(Func<CancellationToken,IReadOnlyList<IReadOnlyList<PointD>>> targetFactory,
        VCarveClearedStock stock,double maximumDepth)
    {
        callerToken.ThrowIfCancellationRequested();
        if(GeometryParallelism.WorkerLimit<=1)return false;
        lock(gate){if(started||disposed)return false;started=true;}
        var columns=VCarveMaterialSurface.AdmittedPostedColumns(stock.Chords);
        var part=stock.PartBoundary.ToArray();var floors=stock.FloorDepths.ToArray();
        long inputBytes=StockKeyLength(columns.Length,part.Length,floors.Length)+96L*columns.Length+32L*part.Length+8L*floors.Length;
        if(inputBytes>MaximumOwnedBytes){Log($"event=refused reason=input-budget bytes={inputBytes}");return false;}
        var key=StockKey(columns,part,floors,maximumDepth);
        lock(gate)
        {
            if(disposed)return false;stockKey=key;
            work=Task.Run(()=>
            {
                long begun=Stopwatch.GetTimestamp();
                try
                {
                    using var worker=GeometryParallelism.EnterWorker();
                    var target=targetFactory(lifetime.Token).Select(r=>(IReadOnlyList<PointD>)r.ToArray()).ToArray();
                    lifetime.Token.ThrowIfCancellationRequested();
                    var original=Clipper.Union(PathUtils.ToClipper(target),FillRule.NonZero);
                    var targetBytes=TargetKey(original);
                    long baseBytes=inputBytes+targetBytes.LongLength+32L*original.Sum(r=>(long)r.Count);
                    if(baseBytes>MaximumOwnedBytes){Log($"event=refused reason=target-budget bytes={baseBytes}");return;}
                    var result=VCarveMaterialSurface.BuildFlatLayerGeometry(original,PathUtils.ToClipper(new[]{part}),columns,
                        VCarveMaterialSurface.FlatFloorSchedule(floors,maximumDepth),lifetime.Token,null).ToArray();
                    long bytes=baseBytes+result.Sum(r=>64L+48L*(r.Swept.Sum(p=>(long)p.Count)+r.Free.Sum(p=>(long)p.Count))
                        +64L*(r.Swept.Count+r.Free.Count));
                    lifetime.Token.ThrowIfCancellationRequested();
                    lock(gate)
                    {
                        if(disposed||lifetime.IsCancellationRequested)return;
                        if(bytes<=MaximumOwnedBytes){targetKey=targetBytes;completed=result;ownedBytes=bytes;}
                    }
                    Log($"event={(bytes<=MaximumOwnedBytes?"published":"refused")} reason={(bytes<=MaximumOwnedBytes?"complete":"result-budget")} layers={result.Length} bytes={bytes} maximumBytes={MaximumOwnedBytes} elapsedMs={Stopwatch.GetElapsedTime(begun).TotalMilliseconds:F3} completeOnly=True queryProofsShared=False workers=1");
                }
                catch(Exception error){Log($"event={(lifetime.IsCancellationRequested?"canceled":"failed")} partialPublished=False detail='{error.Message}' literalFallback=True");}
            },CancellationToken.None);
        }
        Log($"event=started columns={columns.Length} bytes={inputBytes} maximumBytes={MaximumOwnedBytes} stockSource=actual-published-prefix profileSweepsRead=False workers=1");return true;
    }

    internal VCarveMaterialSurface.FlatLayerGeometry[]? Take(VCarveRoughingChord[] columns,Paths64 target,
        IReadOnlyList<PointD> part,IReadOnlyList<double> floors,double maximumDepth,CancellationToken token)
    {
        token.ThrowIfCancellationRequested();Task? pending;
        lock(gate)if(disposed||consumed||work is null||stockKey is null)return null;
        if(StockKeyLength(columns.Length,part.Count,floors.Count)>MaximumOwnedBytes)return null;
        var key=StockKey(columns,part,floors,maximumDepth);
        lock(gate)
        {
            if(disposed||consumed||work is null||stockKey is null)return null;
            if(!key.AsSpan().SequenceEqual(stockKey)){Log("event=miss reason=exact-stock-input-mismatch literalFallback=True");return null;}
            pending=work;
        }
        long begun=Stopwatch.GetTimestamp();pending.GetAwaiter().GetResult();token.ThrowIfCancellationRequested();
        var targetBytes=TargetKey(target);
        lock(gate)
        {
            if(completed is null||targetKey is null||!targetBytes.AsSpan().SequenceEqual(targetKey))
            {Log("event=miss reason=unavailable-or-exact-target-mismatch literalFallback=True");return null;}
            var result=completed;completed=null;stockKey=null;targetKey=null;consumed=true;
            Log($"event=hit layers={result.Length} bytes={ownedBytes} waitMs={Stopwatch.GetElapsedTime(begun).TotalMilliseconds:F3} exactOrderedInputs=True singleConsume=True queryProofsShared=False");
            ownedBytes=0;return result;
        }
    }

    private static byte[] StockKey(IReadOnlyList<VCarveRoughingChord> columns,IReadOnlyList<PointD> part,IReadOnlyList<double> floors,double maximumDepth)
    {
        using var bytes=new MemoryStream();using var writer=new BinaryWriter(bytes);
        void D(double value)=>writer.Write(BitConverter.DoubleToInt64Bits(value));
        void P(Point3D p){D(p.X);D(p.Y);D(p.Z);}
        D(maximumDepth);writer.Write(columns.Count);
        foreach(var c in columns){P(c.A);P(c.B);D(c.TipRadius);D(c.ClearedColumnRadius);D(c.StockSurfaceRadius);}
        writer.Write(part.Count);foreach(var p in part){D(p.x);D(p.y);}
        writer.Write(floors.Count);foreach(double floor in floors)D(floor);
        writer.Flush();return bytes.ToArray();
    }
    private static long StockKeyLength(int columns,int part,int floors)=>20L+72L*columns+16L*part+8L*floors;
    private static byte[] TargetKey(Paths64 target)
    {
        using var bytes=new MemoryStream();using var writer=new BinaryWriter(bytes);
        writer.Write(target.Count);foreach(var ring in target){writer.Write(ring.Count);foreach(var p in ring){writer.Write(p.X);writer.Write(p.Y);}}
        writer.Flush();return bytes.ToArray();
    }
    private void Log(string text){try{diagnostic?.Invoke("stage=flat-layer-prefetch "+text);}catch{}}
    public void Dispose()
    {
        Task? pending;lock(gate){if(disposed)return;disposed=true;pending=work;completed=null;stockKey=null;targetKey=null;}
        lifetime.Cancel();try{pending?.GetAwaiter().GetResult();}finally{lifetime.Dispose();}
        Log($"event=joined completed={pending?.IsCompleted??true} detachedWorker=False");callerToken.ThrowIfCancellationRequested();
    }
}

internal sealed partial class VCarveMaterialSurface
{
    internal sealed record FlatLayerGeometry(double Top,double Floor,int Credited,Paths64 Swept,Paths64 Free);
    internal static VCarveRoughingChord[] AdmittedPostedColumns(IEnumerable<VCarveRoughingChord> source)
    {
        bool Posted(Point3D p)=>double.IsFinite(p.X+p.Y+p.Z)&&Math.Abs(p.X)<=10000&&Math.Abs(p.Y)<=10000&&Math.Abs(p.Z)<=100
            &&p.X==Math.Round(p.X*1000)/1000d&&p.Y==Math.Round(p.Y*1000)/1000d&&p.Z==Math.Round(p.Z*1000)/1000d;
        return source.Where(c=>Posted(c.A)&&Posted(c.B)&&c.ClearedColumnRadius>0&&double.IsFinite(c.ClearedColumnRadius)).ToArray();
    }
    internal static double[] FlatFloorSchedule(IEnumerable<double> source,double maximumDepth)=>source.Where(double.IsFinite)
        .Select(d=>Math.Min(maximumDepth,Math.Floor(d*1000+1e-7)/1000)).Where(d=>d>0).Append(maximumDepth).Distinct().Order().TakeLast(8).ToArray();
    internal static IEnumerable<FlatLayerGeometry> BuildFlatLayerGeometry(Paths64 original,Paths64 part,VCarveRoughingChord[] executed,
        double[] floors,CancellationToken token,Action<string,int>? stage)
    {
        double top=0;int layer=0;
        foreach(double floor in floors)
        {
            token.ThrowIfCancellationRequested();var swept=new Paths64();int credited=0;
            foreach(var group in executed.Where(c=>Math.Max(c.A.Z,c.B.Z)<=-floor+1e-9).GroupBy(c=>c.ClearedColumnRadius))
            {
                var columns=group.ToArray();credited+=columns.Length;
                var segments=columns.Select(c=>new PocketPostedChordChains.Segment(
                    new((long)Math.Round(c.A.X*1000),(long)Math.Round(c.A.Y*1000)),
                    new((long)Math.Round(c.B.X*1000),(long)Math.Round(c.B.Y*1000)))).ToArray();
                var chains=PocketPostedChordChains.Build(segments,token);
                var paths=new Paths64(chains.Paths.Select(path=>new Path64(path.Select(p=>new Point64(
                    (long)Math.Round(p.X/1000d*PathUtils.Scale),(long)Math.Round(p.Y/1000d*PathUtils.Scale))))));
                paths.AddRange(columns.Where(c=>c.A.X==c.B.X&&c.A.Y==c.B.Y).Select(c=>new Path64{new(
                    (long)Math.Round(c.A.X*PathUtils.Scale),(long)Math.Round(c.A.Y*PathUtils.Scale))}));
                if(paths.Count>0)swept.AddRange(PostedSweepInnerRegion.BuildPaths(paths,group.Key,token));
            }
            stage?.Invoke("posted-chain-inner-offsets",layer);
            swept=Clipper.Intersect(Clipper.Union(swept,FillRule.NonZero),part,FillRule.NonZero);
            var free=Clipper.Intersect(Clipper.Union(new Paths64(original.Concat(swept)),FillRule.NonZero),part,FillRule.NonZero);
            stage?.Invoke("layer-booleans",layer);
            yield return new(top,floor,credited,swept,free);top=floor;layer++;
        }
    }
}
