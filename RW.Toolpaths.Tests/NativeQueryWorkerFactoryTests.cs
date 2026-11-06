using Clipper2Lib;
using RW.Toolpaths.Milling;
using RW.Toolpaths.Milling.ReviewedSource;

namespace RW.Toolpaths.Tests;

public sealed class NativeQueryWorkerFactoryTests
{
    private static PointD[] Box(double left,double bottom,double right,double top)
        =>[new(left,bottom),new(right,bottom),new(right,top),new(left,top)];
    private static readonly VCarveCutterProfile Cutter=new([new(CarvingCutterSectionKind.Linear,.1,.1,2)],1);
    private static readonly PointD[][][] Target=[[Box(-.3,-.3,.3,.3)],[Box(-4.5,-.5,-3.5,.5)]];
    private static VCarveClearedStock Stock(bool cleared=true)=>new([],[],Box(-8,-8,8,8),cleared?
        [new(new([new(CarvingCutterSectionKind.Linear,2,2,3)],2),[new(new(4,0,-1),new(4,0,-1),0)])]:[]);
    private static ToolpathPlan Plan(double surface=6,bool cleared=true,VCarveCutterProfile? cutter=null,
        CancellationToken token=default,Action<string>? diagnostic=null)
        =>ReviewedVCarvePlan.AttachClearance([],Target,cutter??Cutter,new(1,1){SurfaceZ=surface},Stock(cleared),token,diagnostic);

    [Theory]
    [InlineData(0d)]
    [InlineData(6d)]
    public async Task ExplicitWorkersKeepTheCompleteStockTargetCutterAndWorkspaceFrame(double surface)
    {
        var plan=Plan(surface);
        (Point3D A,Point3D B,bool Legal)[] chords=[
            (new(0,-.05,surface-.5),new(0,.05,surface-.5),true),
            (new(-4,-.1,surface-.5),new(-4,.1,surface-.5),true), // Complete target absent from raw output.
            (new(4,-.1,surface-.5),new(4,.1,surface-.5),true), // Only actual preceding physical sweep permits this.
            (new(7,0,surface-.5),new(7,.1,surface-.5),false),
            (new(9,0,surface-.5),new(9,.1,surface-.5),false)
        ];
        foreach(var (a,b,legal) in chords)Assert.Equal(legal,plan.NativeReplacementChordGuard!(a,b));
        ToolMove Curve(double x,double z)=>ToolMove.CubicBezier(new(x,-.1,z),new(x+.02,-.04,z),
            new(x+.02,.04,z),new(x,.1,z),0);
        (ToolMove Move,bool Legal)[] curves=[(Curve(4,surface-.5),true),(Curve(-4,surface-.5),true),
            (Curve(7,surface-.5),false),(Curve(4,surface-2),false)];
        foreach(var (move,legal) in curves)Assert.Equal(legal,plan.NativeReplacementCurveGuard!(move));
        Assert.NotNull(plan.NativeReplacementQueryWorkerFactory);
        // Sequential startup happens before any worker can query its context.
        var workers=Enumerable.Range(0,4).Select(_=>plan.NativeReplacementQueryWorkerFactory!()).ToArray();
        Assert.Single(workers.Select(w=>w.ImmutableDomainProvenance).Distinct());
        foreach(var worker in workers)
        {
            Assert.InRange(worker.ImportedPositiveSweepProofs,0,VCarveMaterialSurface.NativeWorkerWarmEntries);
            Assert.InRange(worker.ImportedPositivePoseProofs,0,VCarveMaterialSurface.NativeWorkerWarmEntries);
            Assert.InRange(worker.ImportedPositiveLayerPoses,0,VCarveMaterialSurface.NativeWorkerWarmEntries);
            Assert.True(worker.EstimatedImportedProofBytes>=256);Assert.True(worker.StartupMilliseconds>=0);
        }
        await Task.WhenAll(workers.Select(worker=>Task.Run(()=>{
            foreach(var (a,b,legal) in chords)Assert.Equal(legal,worker.Chord(a,b));
            foreach(var (move,legal) in curves)Assert.Equal(legal,worker.Curve(move));
        })));
        // Join precedes all serial owner queries and independent rechecks.
        foreach(var (move,legal) in curves)Assert.Equal(legal,plan.NativeReplacementCurveGuard!(move));
        Assert.False(Plan(surface,cleared:false).NativeReplacementQueryWorkerFactory!().Curve(Curve(4,surface-.5)));
        var wide=new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear,.5,.5,2)],1);
        Assert.False(Plan(surface,cutter:wide).NativeReplacementQueryWorkerFactory!().Chord(chords[0].A,chords[0].B));
    }

    [Fact]
    public void WorkerQueriesDoNotMutateTheOriginalWarmProofInventoryOrAnotherWorkersHints()
    {
        var log=new List<string>();var plan=Plan(diagnostic:log.Add);
        Assert.True(plan.NativeReplacementChordGuard!(new(0,0,5.5),new(0,0,5.5)));
        var first=plan.NativeReplacementQueryWorkerFactory!();
        Assert.True(first.ImportedPositivePoseProofs>0);Assert.True(first.ImportedPositiveLayerPoses>0);
        for(int i=0;i<10;i++)Assert.True(first.Chord(new(0,i*.005,5.5),new(0,i*.005,5.5)));
        Assert.False(first.Chord(new(7,0,5.5),new(7,0,5.5)));
        // No worker result is merged into the original owner's mutable caches.
        var second=plan.NativeReplacementQueryWorkerFactory!();
        Assert.Equal(first.ImportedPositivePoseProofs,second.ImportedPositivePoseProofs);
        Assert.Equal(first.ImportedPositiveLayerPoses,second.ImportedPositiveLayerPoses);
        Assert.Equal(first.ImportedPositiveSweepProofs,second.ImportedPositiveSweepProofs);
        Assert.Contains(log,l=>l.Contains("event=native-query-worker-created")&&l.Contains("importedRefusals=0")
            &&l.Contains("importedDepthHints=0")&&l.Contains("importedQueryStackCells=0")&&l.Contains("privateQueryHints=True")
            &&l.Contains("stockCellValuesAndNaNsCopiedExactly=True"));
    }

    [Fact]
    public void PositiveSweepImportIsBoundedAndRejectsASeparateEquivalentDomain()
    {
        VCarveProjectedStock Source()=>new([], [Box(-2,-2,2,2)],Box(-4,-4,4,4),Cutter,1,default);
        var source=Source();var a=new Point3D(0,0,-.5);var b=new Point3D(0,1,-.5);
        var c=new Point3D(1,0,-.5);var d=new Point3D(1,1,-.5);
        Assert.True(source.Prove(a,b));Assert.True(source.Prove(c,d));
        Assert.False(source.Prove(new(8,0,-.5),new(8,1,-.5)));
        var bounded=source.CreateQueryWorker();Assert.Equal(1,bounded.ImportPositiveProofs(source,1));
        Assert.True(bounded.TryPositiveProof(a,b));Assert.False(bounded.TryPositiveProof(c,d));
        var complete=source.CreateQueryWorker();Assert.Equal(2,complete.ImportPositiveProofs(source));
        Assert.False(complete.TryPositiveProof(new(8,0,-.5),new(8,1,-.5)));
        var cold=source.CreateQueryWorker();Assert.Equal(0,cold.ImportPositiveProofs(source,0));Assert.Equal(0,cold.CachedSweepProofs);
        Assert.Throws<ArgumentOutOfRangeException>(()=>cold.ImportPositiveProofs(source,-1));
        Assert.Throws<ArgumentException>(()=>Source().ImportPositiveProofs(source));
    }

    [Fact]
    public void GenericOrOriginalSolidCallbacksDoNotImplicitlyAdvertisePrivateStockWorkers()
    {
        var plan=ReviewedVCarvePlan.AttachClearance([],Target,Cutter,new(1,1),null);
        Assert.NotNull(plan.NativeReplacementChordGuard);Assert.NotNull(plan.NativeReplacementCurveGuard);
        Assert.Null(plan.NativeReplacementQueryWorkerFactory);
        Assert.Null(ToolpathPlan.Empty.NativeReplacementQueryWorkerFactory);
    }

    [Fact]
    public void CanceledOwnerCannotPublishANewQueryWorker()
    {
        using var cancellation=new CancellationTokenSource();var plan=Plan(token:cancellation.Token);cancellation.Cancel();
        Assert.ThrowsAny<OperationCanceledException>(()=>plan.NativeReplacementQueryWorkerFactory!());
    }
}
