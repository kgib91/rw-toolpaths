using Clipper2Lib;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;
using Xunit;

namespace RW.Toolpaths.Tests;

public sealed class DeferredPhysicalHullWorkerTests
{
    [Theory]
    [InlineData(8)] [InlineData(32)] [InlineData(256)]
    public void OwnedRecipesRetainEveryLiteralRoundedVertexAndDirection(int count)
    {
        var circle=Circle(count);var owned=new VCarvePhysicalHullRecipe.Circle(circle);
        var cases=new (VCarvePhysicalHullRecipe.Row A,VCarvePhysicalHullRecipe.Row B,int Rows)[] {
            (Disc(0,0,.1),default,1),
            (Disc(-12.7,12.7,.1),Disc(-12.3,12.71,.2),2),
            (Disc(.1,.2,.4),Disc(.1,.2,.4),2),
            (Disc(3,2,.4),Disc(3,2,.05),2),
            (Disc(3,2,.05),Disc(3,2,.4),2),
            (VCarvePhysicalHullRecipe.Row.Vertex(new Point64(1,2)),Disc(.2,.3,.1),2),
            (Disc(.2,.3,.1),VCarvePhysicalHullRecipe.Row.Vertex(new Point64(1,2)),2),
            (Disc(Math.BitIncrement(1d),Math.BitDecrement(-1d),1d/32768),Disc(1,1,.01),2),
            (Disc(10_000,-10_000,100),Disc(9999,-9999,.01),2) };
        using var worker=new VCarveOrderedDiscHull();
        foreach(var item in cases)
        {
            var recipe=new VCarvePhysicalHullRecipe(owned,item.A,item.B,item.Rows);
            Assert.True(recipe.CertifiedNondegenerate);
            var expected=Literal(circle,item.A,item.B,item.Rows);
            var actual=recipe.Materialize(worker);
            Assert.True(expected.Count>=3);Assert.Equal(expected.ToArray(),actual.ToArray());
        }
        Assert.Equal(cases.Length,worker.Calls);
    }

    [Fact]
    public void CertificateRefusalDoesNotReplaceTheLiteralHullAdmissionRule()
    {
        var circle=Circle(32);var owned=new VCarvePhysicalHullRecipe.Circle(circle);
        using var worker=new VCarveOrderedDiscHull();
        var tiny=new VCarvePhysicalHullRecipe(owned,Disc(0,0,1e-12),default,1);
        Assert.False(tiny.CertifiedNondegenerate);
        Assert.Equal(Literal(circle,Disc(0,0,1e-12),default,1).ToArray(),tiny.Materialize(worker).ToArray());
        Assert.Throws<ArgumentException>(()=>new VCarvePhysicalHullBatch(new[]{tiny}));
        var outside=new VCarvePhysicalHullRecipe(owned,Disc(10_000.01,0,.1),default,1);
        Assert.False(outside.CertifiedNondegenerate);
        var expected=Literal(circle,Disc(10_000.01,0,.1),default,1);
        Assert.True(expected.Count>=3);Assert.Equal(expected.ToArray(),outside.Materialize(worker).ToArray());
        var line=new VCarvePhysicalHullRecipe(owned,VCarvePhysicalHullRecipe.Row.Vertex(new(0,0)),
            VCarvePhysicalHullRecipe.Row.Vertex(new(1,1)),2);
        Assert.False(line.CertifiedNondegenerate);Assert.Equal(2,line.Materialize(worker).Count);
        Assert.Throws<ArgumentOutOfRangeException>(()=>new VCarvePhysicalHullBatch(Array.Empty<VCarvePhysicalHullRecipe>()));
        Assert.Throws<ArgumentOutOfRangeException>(()=>new VCarvePhysicalHullBatch(Enumerable.Repeat(outside,33).ToArray()));
        using var cancellation=new CancellationTokenSource();cancellation.Cancel();
        var batch=new VCarvePhysicalHullBatch(new[]{Recipe(owned,0)});
        Assert.ThrowsAny<OperationCanceledException>(()=>batch.Materialize(worker,cancellation.Token));
    }

    [Fact]
    public void PreparedLeavesKeepOriginalThirtyTwoPiecesAndInterleavedOffsetSlots()
    {
        var circle=Circle(32);var owned=new VCarvePhysicalHullRecipe.Circle(circle);
        using var queue=new VCarveOrderedUnionQueue(default,2,enablePreparedHulls:true);
        using var metrics=new VCarveOrderedDiscHull();
        var slots=new List<VCarveOrderedUnionQueue.Slot>();var expected=new List<Paths64>();
        int recipes=0;
        foreach(int length in new[]{31,32,1,17})
        {
            // Offset insertion precedes the later hull slot even when the
            // producer's pending hull inventory straddles that insertion.
            var source=new Paths64{new(){new(0,0),new(50,10)}};
            expected.Add(Offset(source));slots.Add(queue.Submit(source,input=>{
                Assert.Equal(1,GeometryParallelism.WorkerLimit);return Offset(input);}));source.Clear();
            var list=Enumerable.Range(recipes,length).Select(i=>Recipe(owned,i)).ToList();
            var batch=new VCarvePhysicalHullBatch(list);
            using(var literal=new VCarveOrderedDiscHull())expected.Add(Clipper.Union(batch.Materialize(literal,default),FillRule.NonZero));
            slots.Add(queue.Submit(batch));list.Clear();recipes+=length;
        }
        Assert.Throws<InvalidOperationException>(()=>queue.Read(slots[0]));
        Assert.Throws<InvalidOperationException>(()=>queue.MergePreparedHullMetrics(metrics));
        queue.Complete();Assert.True(queue.OwnedWorkersJoined);
        for(int i=0;i<slots.Count;i++)Exact(expected[i],queue.Read(slots[i]));
        Assert.Equal(4,queue.PreparedHullBatches);Assert.Equal(recipes,queue.PreparedHullRecipes);
        Assert.InRange(queue.PeakInFlight,1,2);
        Assert.InRange(queue.PeakRetainedInputBytes,VCarvePhysicalHullRecipe.MaximumWorkerScratchBytes,
            VCarveOrderedUnionQueue.MaximumRetainedInputBytes);
        Assert.Equal(2*VCarvePhysicalHullRecipe.MaximumWorkerScratchBytes,queue.MaximumPreparedHullScratchBytes);
        queue.MergePreparedHullMetrics(metrics);Assert.Equal(recipes,metrics.Calls);
        Assert.Throws<InvalidOperationException>(()=>queue.MergePreparedHullMetrics(metrics));
    }

    [Fact]
    public void OversizedRecipeUsesLiteralCallerFallbackWithCompleteDetachedOutput()
    {
        var circle=new VCarvePhysicalHullRecipe.Circle(Circle(256));
        var batch=new VCarvePhysicalHullBatch(Enumerable.Range(0,32).Select(i=>Recipe(circle,i)).ToArray());
        Assert.True(batch.EstimatedBytes>VCarvePhysicalHullRecipe.MaximumWorkerScratchBytes);
        using var literal=new VCarveOrderedDiscHull();
        var expected=Clipper.Union(batch.Materialize(literal,default),FillRule.NonZero);
        using var queue=new VCarveOrderedUnionQueue(default,2,maximumBytes:512,enablePreparedHulls:true);
        var slot=queue.Submit(batch);queue.Complete();Exact(expected,queue.Read(slot));
        Assert.Equal(0,queue.Submitted);Assert.Equal(1,queue.LiteralFallbacks);
        Assert.Equal(0,queue.PeakRetainedInputBytes);Assert.True(queue.OwnedWorkersJoined);
        using var merged=new VCarveOrderedDiscHull();queue.MergePreparedHullMetrics(merged);Assert.Equal(32,merged.Calls);
    }

    [Theory]
    [InlineData(false)] [InlineData(true)]
    public void FailureOrCancellationJoinsEveryPrivateHullOwnerBeforePublication(bool cancel)
    {
        using var cancellation=new CancellationTokenSource();
        using var entered=new ManualResetEventSlim();using var release=new ManualResetEventSlim();
        var queue=new VCarveOrderedUnionQueue(cancellation.Token,2,enablePreparedHulls:true);
        var slots=new List<VCarveOrderedUnionQueue.Slot>();
        try
        {
            slots.Add(queue.Submit(new Paths64(),input=>{
                entered.Set();if(!release.Wait(TimeSpan.FromSeconds(10)))throw new TimeoutException("test release");
                if(cancel){cancellation.Cancel();return input;}throw new FormatException("offset fixture failed");}));
            Assert.True(entered.Wait(TimeSpan.FromSeconds(10)));
            var circle=new VCarvePhysicalHullRecipe.Circle(Circle(256));
            slots.Add(queue.Submit(new VCarvePhysicalHullBatch(Enumerable.Range(0,32).Select(i=>Recipe(circle,i)).ToArray())));
            release.Set();
            if(cancel)Assert.ThrowsAny<OperationCanceledException>(()=>queue.Complete());
            else Assert.Equal("offset fixture failed",Assert.Throws<FormatException>(()=>queue.Complete()).Message);
            foreach(var slot in slots)Assert.Throws<InvalidOperationException>(()=>queue.Read(slot));
        }
        finally{release.Set();queue.Dispose();}
        Assert.True(queue.OwnedWorkersJoined);
    }

    [Fact]
    public void OpaqueOrUnpreparedQueuesCannotReceiveHiddenPreparedGeometry()
    {
        var circle=new VCarvePhysicalHullRecipe.Circle(Circle(8));
        var batch=new VCarvePhysicalHullBatch(new[]{Recipe(circle,0)});int opaqueCalls=0;
        Assert.Throws<ArgumentException>(()=>new VCarveOrderedUnionQueue(default,2,
            p=>{opaqueCalls++;return p;},enablePreparedHulls:true));
        using var queue=new VCarveOrderedUnionQueue(default,2,p=>{opaqueCalls++;return p;});
        Assert.Throws<InvalidOperationException>(()=>queue.Submit(batch));queue.Complete();
        Assert.Equal(0,opaqueCalls);Assert.Equal(0,queue.PreparedHullBatches);
    }

    [Theory]
    [InlineData(false,false)] [InlineData(false,true)]
    [InlineData(true,false)] [InlineData(true,true)]
    public void WholeMixedSweepKeepsRawIntegerOrderSourceAndPhysicalRecipe(bool upper,bool rounded)
    {
        var source=Source();var before=source.Select(p=>p.Points.ToArray()).ToArray();
        var profile=Profile(rounded);var old=VCarvePhysicalSweep.DeferredHullPreparationEnabled.Value;
        try
        {
            VCarvePhysicalSweep.DeferredHullPreparationEnabled.Value=null;
            Paths64 expected;
            using(var defaultCapture=VCarvePhysicalSweepTrace.BeginCapture(0))
            {
                expected=PathUtils.ToClipper(VCarvePhysicalSweep.BuildWithParallelLeafUnions(source,profile,12.7,.004,upperBound:upper));
                if(upper)
                {
                    var evidence=Assert.IsType<VCarvePhysicalSweepTrace.Evidence>(defaultCapture.Completed);
                    Assert.Null(evidence.Settings.DeferredHullPreparationRequested);
                    var receipt=Assert.IsType<VCarvePhysicalSweepTrace.DeferredHullWork>(evidence.Backend!.DeferredHulls);
                    Assert.False(receipt.Enabled);Assert.Equal(0,receipt.Pieces);Assert.Equal(0,receipt.Recipes);
                    Assert.True(receipt.WorkersJoined);
                }
            }
            if(upper&&!rounded)
            {
                VCarvePhysicalSweep.DeferredHullPreparationEnabled.Value=false;
                Exact(expected,PathUtils.ToClipper(VCarvePhysicalSweep.BuildWithParallelLeafUnions(source,profile,12.7,.004,upperBound:true)));
            }
            VCarvePhysicalSweep.DeferredHullPreparationEnabled.Value=true;
            using var capture=VCarvePhysicalSweepTrace.BeginCapture(0);
            var actual=PathUtils.ToClipper(VCarvePhysicalSweep.BuildWithParallelLeafUnions(source,profile,12.7,.004,upperBound:upper));
            Exact(expected,actual);
            for(int i=0;i<source.Length;i++)Assert.Equal(before[i],source[i].Points);
            if(upper)
            {
                var evidence=Assert.IsType<VCarvePhysicalSweepTrace.Evidence>(capture.Completed);
                Assert.True(evidence.Settings.DeferredHullPreparationRequested);
                var receipt=Assert.IsType<VCarvePhysicalSweepTrace.DeferredHullWork>(evidence.Backend!.DeferredHulls);
                Assert.True(receipt.Enabled);Assert.True(receipt.Pieces>0);Assert.True(receipt.Recipes>0);
                Assert.True(receipt.Batches>0);Assert.True(receipt.CallerFallbackPieces>0);
                Assert.True(receipt.MaterializedCarryPieces>0);Assert.True(receipt.WorkersJoined);
                Assert.InRange(receipt.MaximumCarryChargedBytes,1,VCarveOrderedUnionQueue.MaximumRetainedInputBytes);
            }
        }
        finally{VCarvePhysicalSweep.DeferredHullPreparationEnabled.Value=old;}
    }

    [Theory]
    [InlineData(false)] [InlineData(true)]
    public void OpaqueNominationReceivesTheSameLiteralOperandsInTheSameOrder(bool constantRuns)
    {
        var old=VCarvePhysicalSweep.DeferredHullPreparationEnabled.Value;
        try
        {
            var source=Source();var controls=new List<Point64[][]>();var observed=new List<Point64[][]>();
            VCarvePhysicalSweep.DeferredHullPreparationEnabled.Value=false;
            var expected=VCarvePhysicalSweep.BuildNomination(source,Profile(false),12.7,.004,p=>{
                controls.Add(p.Select(row=>row.ToArray()).ToArray());return Clipper.Union(p,FillRule.NonZero);},
                accelerateConstantDepthRuns:constantRuns).Paths;
            VCarvePhysicalSweep.DeferredHullPreparationEnabled.Value=true;
            var actual=VCarvePhysicalSweep.BuildNomination(source,Profile(false),12.7,.004,p=>{
                observed.Add(p.Select(row=>row.ToArray()).ToArray());return Clipper.Union(p,FillRule.NonZero);},
                accelerateConstantDepthRuns:constantRuns).Paths;
            Exact(PathUtils.ToClipper(expected),PathUtils.ToClipper(actual));Assert.Equal(controls.Count,observed.Count);
            for(int call=0;call<controls.Count;call++)
            {
                Assert.Equal(controls[call].Length,observed[call].Length);
                for(int path=0;path<controls[call].Length;path++)Assert.Equal(controls[call][path],observed[call][path]);
            }
        }
        finally{VCarvePhysicalSweep.DeferredHullPreparationEnabled.Value=old;}
    }

    [Theory]
    [InlineData(false)] [InlineData(true)]
    public void NestedOwnerAndUnorderedHullModesRemainLiteral(bool unordered)
    {
        var old=VCarvePhysicalSweep.DeferredHullPreparationEnabled.Value;
        using var nested=unordered?null:GeometryParallelism.EnterWorker();
        try
        {
            var source=Source();
            List<List<PointD>> Build()=>VCarvePhysicalSweep.BuildWithParallelLeafUnions(source,Profile(false),12.7,.004,
                upperBound:true,orderedDiscHulls:!unordered);
            VCarvePhysicalSweep.DeferredHullPreparationEnabled.Value=false;var expected=PathUtils.ToClipper(Build());
            VCarvePhysicalSweep.DeferredHullPreparationEnabled.Value=true;
            using var capture=VCarvePhysicalSweepTrace.BeginCapture(0);
            var actual=PathUtils.ToClipper(Build());
            Exact(expected,actual);
            var receipt=Assert.IsType<VCarvePhysicalSweepTrace.DeferredHullWork>(capture.Completed!.Backend!.DeferredHulls);
            Assert.False(receipt.Enabled);Assert.Equal(0,receipt.Recipes);Assert.True(receipt.WorkersJoined);
        }
        finally{VCarvePhysicalSweep.DeferredHullPreparationEnabled.Value=old;}
    }

    private static VCarveCutterProfile Profile(bool rounded)=>rounded
        ?new(new[]{new CarvingCutterSection(CarvingCutterSectionKind.Hemisphere,0,.3,.3),
            new CarvingCutterSection(CarvingCutterSectionKind.Linear,.3,2,4)},2)
        :new(new[]{new CarvingCutterSection(CarvingCutterSectionKind.Linear,0,2,4)},2);
    private static TaggedToolpath[] Source()
    {
        TaggedToolpath Path(IReadOnlyList<Point3D> points)=>new(points,Array.Empty<ToolpathSpan>(),3,
            "deferred-hull-control",2,detectCurves:false);
        TaggedToolpath Vary(int count,double x,double y)=>Path(Enumerable.Range(0,count+1).Select(i=>
            new Point3D(x+i*.06,y+Math.Sin(i*.2)*.01,12.7-(.12+(i%3)*.01))).ToArray());
        return new[]{Vary(31,0,0),
            Path(Enumerable.Range(0,10).Select(i=>new Point3D(i*.09,.05,12.5)).ToArray()),
            // Outside the certificate's conservative numeric domain, while
            // still ordinary legal Clipper coordinates. Resolve prior31
            // recipes without moving the interleaved offset or leaf boundary.
            Vary(1,10_000.25,0),Vary(40,0,.1),
            Path(new Point3D[]{new(0,0,12.5),new(2,0,12.5),new(2,2,12.5),new(0,2,12.5),new(0,0,12.5)}),
            Vary(69,0,.2)};
    }

    private static (double X,double Y)[] Circle(int count)=>Enumerable.Range(0,count).Select(i=>{
        double angle=2*Math.PI*i/count;return(Math.Cos(angle),Math.Sin(angle));}).ToArray();
    private static VCarvePhysicalHullRecipe.Row Disc(double x,double y,double r)
        =>VCarvePhysicalHullRecipe.Row.Disc(new(x,y,-.2),r);
    private static VCarvePhysicalHullRecipe Recipe(VCarvePhysicalHullRecipe.Circle circle,int i)
        =>new(circle,Disc(i*.04,Math.Sin(i*.1)*.03,.06),Disc(i*.04+.02,.02,.07),2);
    private static Path64 Literal((double X,double Y)[] circle,VCarvePhysicalHullRecipe.Row a,
        VCarvePhysicalHullRecipe.Row b,int rows)
    {
        using var literal=new VCarveOrderedDiscHull(literalFinalRowValidation:true);
        literal.BeginCircle(circle);if(rows>0)Add(a);if(rows>1)Add(b);return literal.FinishHull();
        void Add(VCarvePhysicalHullRecipe.Row row){if(row.IsPoint)literal.AddPoint(row.Point);else literal.AddDisc(row.Center,row.Radius);}
    }
    private static Paths64 Offset(Paths64 source)
    {
        var offset=new ClipperOffset(2,1);offset.AddPaths(source,JoinType.Round,EndType.Round);
        var result=new Paths64();offset.Execute(3,result);return result;
    }
    private static void Exact(Paths64 expected,Paths64 actual)
    {
        Assert.Equal(expected.Count,actual.Count);
        for(int i=0;i<expected.Count;i++)Assert.Equal(expected[i].ToArray(),actual[i].ToArray());
    }
}
