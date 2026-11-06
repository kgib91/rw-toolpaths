using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class LazyBoundedFinishPreparationTests
{
    static readonly VCarveCutterProfile Profile=new([new(CarvingCutterSectionKind.Linear,0,2,4)],2);
    static readonly ToolGeometry Tool=new(2){VCarveProfile=Profile};
    static readonly DepthSchedule Depth=new(2,1);
    static readonly VCarveOptions Options=new(){SmoothVCarving=true,SmoothRampAngleDegrees=45,
        IncludeInteriorFill=false,EdgeFillWidth=1,StockToLeave=0,RampOutsideEdges=false,
        DetectCurves=false,CalculateSweptArea=false};
    static PointD[] Box(double half)=>[new(-half,-half),new(half,-half),new(half,half),new(-half,half)];
    static MillingRegion[] Regions(double half=20)=>[new(Box(half))];

    // A deterministic dense provider lets the tests isolate depth preparation,
    // using the complete identical graph in both controls. The physical source
    // radius is still re-evaluated on the original boundary by production code.
    sealed class DenseProvider(int count=4096):IMedialAxisProvider
    {
        public IReadOnlyList<MedialSegment> ConstructMedialAxis(IReadOnlyList<PointD> boundary,
            IReadOnlyList<IReadOnlyList<PointD>> holes,double tolerance,double maxRadius,
            double filteringAngle=Math.PI,bool useBigIntegers=true)
        {
            double width=boundary.Max(p=>p.x)-boundary.Min(p=>p.x);
            return Enumerable.Range(0,count).Select(i=>{
                double x=(i%64)*width*.000001,y=(i/64)*width*.000001;
                return new MedialSegment(new(x,y,0),new(x+width*.0001,y,0));
            }).ToArray();
        }
    }
    static VCarvePreparation Prepare(bool lazy,VCarveOptions? options=null,double half=20,CancellationToken token=default)
        =>lazy?VCarvePreparation.PrepareBoundedFinish(Regions(half),Tool,Depth,options??Options,new DenseProvider(),token)
            :VCarveToolpaths.Prepare(Regions(half),Tool,Depth,options??Options,new DenseProvider(),token);
    static void SameMask(VCarveRoughingAnalysis expected,VCarveRoughingAnalysis actual)
    {
        Assert.Equal(expected.ProtectedArea,actual.ProtectedArea);
        Assert.Equal(expected.EntireProtectedSourceRegionIndices.Order(),actual.EntireProtectedSourceRegionIndices.Order());
        Assert.Equal(expected.ProtectedSourceRegionIndices.Order(),actual.ProtectedSourceRegionIndices.Order());
        Assert.Equal(expected.ProtectedBoundary.Count,actual.ProtectedBoundary.Count);
        for(int i=0;i<expected.ProtectedBoundary.Count;i++)Assert.Equal(expected.ProtectedBoundary[i],actual.ProtectedBoundary[i]);
    }

    [Fact]
    public void ExactUpperSampleDefersOnlyUnusedDepthsAndRetainsTheLiteralPlateauMask()
    {
        var full=Prepare(false);var lazy=Prepare(true);var pending=Assert.Single(lazy.Regions);
        Assert.True(pending.HasDeferredDepths);Assert.False(pending.HasCompletedDepths);
        Assert.Equal(2,pending.SampledUpperWitness);Assert.Equal(full.Regions[0].OriginalMedialGraph,pending.OriginalMedialGraph);
        var expected=full.AnalyzePocketPlateau(2,45);var actual=lazy.AnalyzePocketPlateau(2,45);
        SameMask(expected,actual);Assert.Equal(expected.DecisionCount,actual.DecisionCount);
        Assert.Equal(expected.OmittedDecisionCount,actual.OmittedDecisionCount);Assert.False(pending.HasCompletedDepths);
        // Asking for diagnostics restores the full original values rather than
        // exposing witness bounds as measured peaks or basin statistics.
        Assert.Equal(expected.Basins,actual.Basins);Assert.True(pending.HasCompletedDepths);
        var complete=pending.Complete(default);var control=full.Regions[0];
        Assert.Equal(control.MaximumChordLength,complete.MaximumChordLength);
        for(int i=0;i<control.OriginalMedialPaths.Count;i++)
        {
            Assert.Equal(control.OriginalMedialPaths[i].Points,complete.OriginalMedialPaths[i].Points);
            Assert.Equal(control.SmoothedMedialPaths[i].Points,complete.SmoothedMedialPaths[i].Points);
            Assert.Equal(control.UpperMedialPaths[i].Points,complete.UpperMedialPaths[i].Points);
            Assert.Equal(control.SourceClearancePaths[i],complete.SourceClearancePaths[i]);
        }
    }

    [Fact]
    public void DeferredCachedDataDoesNotRetainTheDisposedPocketCancellationToken()
    {
        VCarvePreparation lazy;VCarveRoughingAnalysis analysis;
        using(var cancellation=new CancellationTokenSource())
        {
            lazy=Prepare(true,token:cancellation.Token);analysis=lazy.AnalyzePocketPlateau(2,45,cancellationToken:cancellation.Token);
            Assert.False(lazy.Regions[0].HasCompletedDepths);cancellation.Cancel();
        }
        var scope=lazy.CreateIndependentFinishScope();var scoped=analysis.WithPreparation(scope);
        Assert.Same(lazy.Regions,scope.Regions);
        Assert.Same(analysis.Basins,scoped.Basins);Assert.True(lazy.Regions[0].HasCompletedDepths);
        Assert.All(analysis.Basins,b=>Assert.True(double.IsFinite(b.OriginalPeakDepth+b.SmoothedPeakDepth+b.BoundedMaximumDepth)));
    }

    [Fact]
    public void ParallelDetailedReadersPublishOneCompleteOriginalDataOwner()
    {
        var lazy=Prepare(true);var pending=lazy.Regions[0];
        var values=new IReadOnlyList<TaggedToolpath>[6];
        Parallel.For(0,values.Length,i=>values[i]=pending.OriginalMedialPaths);
        Assert.True(pending.HasCompletedDepths);Assert.All(values,v=>Assert.Same(values[0],v));
        Assert.Equal(4096,values[0].Count);
    }

    [Theory]
    [InlineData("full-fill")]
    [InlineData("outside-ramps")]
    [InlineData("allowance")]
    [InlineData("stock-contact")]
    public void UnsupportedPreparationContractsKeepTheCompleteOriginalPath(string mode)
    {
        var options=mode switch {
            "full-fill"=>Options with{IncludeInteriorFill=true},
            "outside-ramps"=>Options with{RampOutsideEdges=true},
            "allowance"=>Options with{StockToLeave=.01},
            _=>Options with{ClearedStock=new VCarveClearedStock([],[],Box(20))} };
        var lazy=Prepare(true,options);Assert.All(lazy.Regions,r=>Assert.False(r.HasDeferredDepths));
        var full=Prepare(false,options);SameMask(full.AnalyzePocketPlateau(2,45),lazy.AnalyzePocketPlateau(2,45));
    }

    [Fact]
    public void MissingWitnessAndSmallGraphsUseFullPreparation()
    {
        var narrow=Prepare(true,half:.5);Assert.False(narrow.Regions[0].HasDeferredDepths);
        var full=Prepare(false,half:.5);SameMask(full.AnalyzePocketPlateau(2,45),narrow.AnalyzePocketPlateau(2,45));
        var small=VCarvePreparation.PrepareBoundedFinish(Regions(),Tool,Depth,Options,new DenseProvider(3));
        Assert.False(small.Regions[0].HasDeferredDepths);
    }

    [Fact]
    public void ExplicitCanceledConsumersCannotStartOrPublishDeferredDepths()
    {
        var lazy=Prepare(true);using var cancellation=new CancellationTokenSource();cancellation.Cancel();
        Assert.Throws<OperationCanceledException>(()=>lazy.AnalyzeRoughing(2,cancellation.Token));
        Assert.False(lazy.Regions[0].HasCompletedDepths);
        Assert.Throws<OperationCanceledException>(()=>lazy.AnalyzePocketPlateau(2,45,cancellationToken:cancellation.Token));
        Assert.False(lazy.Regions[0].HasCompletedDepths);
        Assert.Throws<OperationCanceledException>(()=>Prepare(true,token:cancellation.Token));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ChangedAngleOrActualOutsideWallsCannotInheritTheOldWitness(bool accessChanged)
    {
        var lazy=Prepare(true);var full=Prepare(false);VCarveRoughingAnalysis expected,actual;
        if(accessChanged)
        {
            PointD[] platform=[new(-25,-25),new(25,-25),new(25,25),new(-25,25)];
            var access=new MillingBoundaryAccess([new(Box(25))],platform){AllowedRegions=[new(Box(25))],ProtectedRegions=[]};
            expected=full.AnalyzePocketPlateau(2,45,outsideAccess:access,rampOutsideEdges:false);
            actual=lazy.AnalyzePocketPlateau(2,45,outsideAccess:access,rampOutsideEdges:false);
        }
        else
        {
            expected=full.AnalyzePocketPlateau(2,5);
            actual=lazy.AnalyzePocketPlateau(2,5);
        }
        Assert.True(lazy.Regions[0].HasCompletedDepths);SameMask(expected,actual);Assert.Equal(expected.Basins,actual.Basins);
        Assert.All(actual.Preparation.Regions,r=>Assert.False(r.HasDeferredDepths));
    }

    [Fact]
    public void EditingADepthRecordMaterializesAndDropsItsOldWitness()
    {
        var lazy=Prepare(true);var pending=lazy.Regions[0];
        var edited=pending with{MaximumChordLength=.125};
        Assert.True(pending.HasCompletedDepths);Assert.False(edited.HasDeferredDepths);
        Assert.Null(edited.SampledUpperWitness);Assert.Equal(.125,edited.MaximumChordLength);
        Assert.Equal(pending.OriginalMedialPaths,edited.OriginalMedialPaths);
    }

    [Fact]
    public void BoundedSmoothSurfaceKeepsIdenticalGuardedSourceWithoutForcingUnusedDepths()
    {
        var full=Prepare(false);var lazy=Prepare(true);
        TaggedToolpath[] source=[new([new(-2,0,-2),new(0,0,-2),new(2,0,-2)],[],0,"clearing",0,17,detectCurves:false)];
        var expected=VCarveSmoothing.Apply(source,[Box(20)],Tool,Depth,Options,default,null,preparation:full.Regions[0]);
        var actual=VCarveSmoothing.Apply(source,[Box(20)],Tool,Depth,Options,default,null,preparation:lazy.Regions[0]);
        Assert.False(lazy.Regions[0].HasCompletedDepths);Assert.Equal(expected.Count,actual.Count);
        var wall=new VCarveSmoothing.CommandClearance([Box(20)],Profile,Depth.SurfaceZ,default,null);
        for(int i=0;i<expected.Count;i++)
        {
            Assert.Equal(expected[i].Points,actual[i].Points);
            Assert.Equal(expected[i].Spans,actual[i].Spans);Assert.Equal(expected[i].Category,actual[i].Category);
            for(int p=1;p<actual[i].Points.Count;p++)Assert.True(wall.IsLegalCommandedChord(actual[i].Points[p-1],actual[i].Points[p]));
        }
    }
}
