using System.Runtime.CompilerServices;
using Clipper2Lib;

namespace RW.Toolpaths.Milling;

/// <summary>Optional detached evidence for offline whole-sweep backend comparison.
/// Capture is off by default and never supplies geometry to a generator.</summary>
public static class VCarvePhysicalSweepTrace
{
    public const long MaximumOwnedBytes=96L*1024*1024;
    private static readonly AsyncLocal<Session?> Current=new();
    internal static bool IsEnabled=>Current.Value is not null;
    public sealed record Parameters(double OutputSurfaceZ,double SpatialTolerance,double EffectiveTolerance,bool UpperBound,
        bool AccelerateConstantDepthRuns,bool OrderedDiscHulls,bool ConsolidateHemisphereChords,bool EnableTaperedRadiusRuns,
        bool EnableParallelLeafUnions,int GeometryWorkerLimit,bool NativeLeafBackend,int UnionBatchSize,int FlatRunChunkChords,
        int MaximumUnionHierarchyDepth,int AutomaticParallelMinimumSourceChords,double GridReserve,double PostUnionReserve,
        double PostUnionArcTolerance)
    {
        public bool UseInputVertexPool {get;init;}
        public bool? DeferredHullPreparationRequested {get;init;}
    }
    public sealed record Path(Point3D[] Points,ToolpathSpan[] Spans,int RegionIndex,string Category,int? DepthPassIndex,int? VisitIdentity);
    public sealed record InputVertexPoolWork(int Owners,long Executions,long Rented,long Reused,long Allocated,
        long Overflow,int Retained,int PeakUsed,int Capacity,bool AllWorkersJoined);
    public sealed record BackendWork(long UnionAttempts,long UnionCompleted,long UnionFallbacks,long OffsetCleanupAttempts,long OffsetCleanupCompleted,long OffsetCleanupFallbacks)
    {public InputVertexPoolWork? InputVertices {get;init;} public DeferredHullWork? DeferredHulls {get;init;}}
    public sealed record DeferredHullWork(bool Enabled,long Pieces,long CallerFallbackPieces,long MaterializedCarryPieces,
        long Batches,long Recipes,bool WorkersJoined)
    {
        public long MaximumCarryChargedBytes {get;init;}
        public long MaximumWorkerScratchBytes {get;init;}
        public double WorkerHullMilliseconds {get;init;}
    }
    public sealed record Evidence(Parameters Settings,CarvingCutterSection[] ProfileSections,double ProfileMaximumDepth,
        Path[] Source,Point64[][] CompletedResult,long SourcePoints,long SourceChords,long ChargedBytes,double IntegerScale)
    {public BackendWork? Backend {get;init;}}
    public sealed record Status(long ObservedRequests,long EligibleRequests,bool Selected,bool Completed,long IncompleteRequests,
        string? Refusal,long ChargedBytes,int MinimumSourcePoints,long MaximumBytes);

    public static Session BeginCapture(int minimumSourcePoints=1_000_000,long maximumOwnedBytes=MaximumOwnedBytes)
    {
        if(minimumSourcePoints<0||maximumOwnedBytes<=0||maximumOwnedBytes>MaximumOwnedBytes)
            throw new ArgumentOutOfRangeException(nameof(maximumOwnedBytes));
        var session=new Session(Current.Value,minimumSourcePoints,maximumOwnedBytes);Current.Value=session;return session;
    }

    public sealed class Session:IDisposable
    {
        private readonly object gate=new();
        private readonly Session? previous;
        private readonly int minimum;
        private readonly long maximum;
        private long observed,eligible,incomplete,charged;
        private bool selected,disposed;
        private string? refusal;
        private Evidence? evidence;
        internal Session(Session? previous,int minimum,long maximum){this.previous=previous;this.minimum=minimum;this.maximum=maximum;}
        public Evidence? Completed {get{lock(gate)return evidence;}}
        public Status CaptureStatus {get{lock(gate)return new(observed,eligible,selected,evidence is not null,incomplete,refusal,charged,minimum,maximum);}}
        internal Lease? Begin(IReadOnlyList<TaggedToolpath> paths,VCarveCutterProfile profile,Parameters settings,
            CancellationToken token,bool nomination)
        {
            long points=0,chords=0,bytes=256+64L*profile.Sections.Count;
            lock(gate){observed++;if(disposed||selected||nomination||!settings.UpperBound)return null;}
            foreach(var path in paths)
            {
                token.ThrowIfCancellationRequested();points+=path.Points.Count;chords+=Math.Max(0,path.Points.Count-1);
                bytes+=160L+(long)Unsafe.SizeOf<Point3D>()*path.Points.Count+(long)Unsafe.SizeOf<ToolpathSpan>()*path.Spans.Count
                    +2L*(path.Category?.Length??0);
            }
            if(points<minimum)return null;
            lock(gate)
            {
                eligible++;if(disposed||selected)return null;selected=true;
                if(bytes>maximum){refusal="source-byte-limit";return null;}
            }
            try
            {
                var source=new Path[paths.Count];
                for(int i=0;i<source.Length;i++)
                {
                    token.ThrowIfCancellationRequested();var path=paths[i];
                    source[i]=new(path.Points.ToArray(),path.Spans.ToArray(),path.RegionIndex,path.Category,path.DepthPassIndex,path.VisitIdentity);
                }
                var sections=profile.Sections.Select(s=>new CarvingCutterSection(s.Kind,s.BottomRadius,s.TopRadius,s.Height)).ToArray();
                lock(gate){if(disposed)return null;charged=bytes;}
                return new(this,settings,sections,profile.MaximumDepth,source,points,chords,bytes);
            }
            catch(OperationCanceledException){Fail("canceled-source-copy");throw;}
            catch(Exception error){Fail("source-copy:"+error.GetType().Name);return null;}
        }
        internal bool Publish(Evidence value)
        {
            lock(gate)
            {
                if(disposed)return false;
                if(value.ChargedBytes>maximum){refusal="result-byte-limit";charged=0;return false;}
                evidence=value;charged=value.ChargedBytes;return true;
            }
        }
        internal long Maximum=>maximum;
        internal void Fail(string reason){lock(gate){refusal=reason;charged=0;}}
        internal void Incomplete(){lock(gate){incomplete++;charged=0;refusal??="build-not-completed";}}
        public void Dispose()
        {
            lock(gate){if(disposed)return;disposed=true;}
            if(ReferenceEquals(Current.Value,this))Current.Value=previous;
        }
    }

    internal sealed class Lease:IDisposable
    {
        private readonly Session owner;
        private readonly Parameters settings;
        private readonly CarvingCutterSection[] sections;
        private readonly double profileMaximumDepth;
        private Path[]? source;
        private readonly long points,chords,sourceBytes;
        private bool completed;
        internal Lease(Session owner,Parameters settings,CarvingCutterSection[] sections,double profileMaximumDepth,Path[] source,
            long points,long chords,long sourceBytes)
        {this.owner=owner;this.settings=settings;this.sections=sections;this.profileMaximumDepth=profileMaximumDepth;this.source=source;
            this.points=points;this.chords=chords;this.sourceBytes=sourceBytes;}
        internal void Complete(Paths64 result,CancellationToken token,BackendWork? work=null)
        {
            token.ThrowIfCancellationRequested();if(source is null||completed)return;completed=true;
            long bytes=sourceBytes+32+8L*result.Count+result.Sum(r=>32L+(long)Unsafe.SizeOf<Point64>()*r.Count);
            if(bytes>owner.Maximum){owner.Fail("result-byte-limit");source=null;return;}
            try
            {
                var snapshot=new Point64[result.Count][];
                for(int i=0;i<snapshot.Length;i++){token.ThrowIfCancellationRequested();snapshot[i]=result[i].ToArray();}
                token.ThrowIfCancellationRequested();
                owner.Publish(new(settings,sections,profileMaximumDepth,source,snapshot,points,chords,bytes,PathUtils.Scale){Backend=work});source=null;
            }
            catch(OperationCanceledException){owner.Fail("canceled-result-copy");throw;}
            catch(Exception error){owner.Fail("result-copy:"+error.GetType().Name);source=null;}
        }
        public void Dispose(){if(!completed)owner.Incomplete();source=null;}
    }

    internal static Lease? Begin(IReadOnlyList<TaggedToolpath> paths,VCarveCutterProfile profile,Parameters settings,
        CancellationToken token,bool nomination)=>Current.Value?.Begin(paths,profile,settings,token,nomination);
}
