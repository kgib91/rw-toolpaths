using System.Collections.Concurrent;
using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class NativeRegionQueryWorkerTests
{
    static readonly VCarveCutterProfile Profile=new([new(CarvingCutterSectionKind.Linear,0,2,2)],2);
    static readonly IReadOnlyList<IReadOnlyList<IReadOnlyList<PointD>>> Boundaries=new PointD[][][]
    {
        [new PointD[]{new(-10,-10),new(10,-10),new(10,10),new(-10,10)}],
        [new PointD[]{new(-10,-10),new(10,-10),new(10,10),new(-10,10)}]
    };
    static TaggedToolpath[] Sources(double surface=10)
    {
        TaggedToolpath Path(int region,int visit,params Point3D[] p)=>new(p,[new(0,p.Length-1,ToolpathSpanKind.Cut)],
            region,"native-worker",0,visit,detectCurves:false);
        // Interleaved regions share a degree-four junction at an INTERNAL
        // vertex. A private per-region graph would lose this protected port.
        return [Path(1,71,new(0,0,surface-1),new(1,0,surface-1),new(2,0,surface-1)),
            Path(0,72,new(5,0,surface-1),new(6,0,surface-1),new(7,0,surface-1)),
            Path(0,73,new(1,-1,surface-1),new(1,0,surface-1),new(1,1,surface-1))];
    }
    static IReadOnlyList<VCarveNativeCurvePlan> Run(TaggedToolpath[] source,Func<Point3D,Point3D,bool> chord,
        Func<ToolMove,bool> curve,Func<VCarveNativeQueryWorker>? factory=null,CancellationToken token=default,
        Action<string>? log=null,VCarveNativeFitLimits? limits=null,
        IReadOnlyList<IReadOnlyList<int>>? selected=null,IReadOnlyList<IReadOnlyList<int>>? required=null,
        VCarveNativeTopologyContext? topology=null)
        =>VCarveNativeCurvePlanning.Nominate(source,Boundaries,Profile,10,token,limits,log,new(.3,.3),
            chord,curve,selected,required,canonicalTopology:topology,stockQueryWorkerFactory:factory);
    static void WithWorkers(int count,Action action)
    {
        var previous=VCarveNativeCurvePlanning.RegionWorkerLimitOverride.Value;
        try{VCarveNativeCurvePlanning.RegionWorkerLimitOverride.Value=count;action();}
        finally{VCarveNativeCurvePlanning.RegionWorkerLimitOverride.Value=previous;}
    }

    [Fact]
    public void PrivateRegionQueriesRetainGlobalTopologyMapsDatumAndOrderedObservers()
    {
        WithWorkers(2,()=>
        {
            var source=Sources();IReadOnlyList<IReadOnlyList<int>> selected=[[0,1,2],[0,1,2],[0,1,2]],required=[[0,2],[0,1,2],[0,2]];
            var serial=Run(source,(_,_)=>true,_=>true,selected:selected,required:required);
            int created=0,active=0,sourceSnapshots=0,candidateSnapshots=0;var observed=new List<int>();var logs=new List<string>();
            var oldSource=VCarveNativeCurvePlanning.EvidenceObserver.Value;
            var oldCandidate=VCarveNativeCurvePlanning.CandidateObserver.Value;
            var oldRegion=VCarveNativeCurvePlanning.RegionObserver.Value;
            try
            {
                VCarveNativeCurvePlanning.EvidenceObserver.Value=(paths,_,_,_)=>{sourceSnapshots++;Assert.Equal(3,paths.Count);};
                VCarveNativeCurvePlanning.CandidateObserver.Value=plans=>{Assert.Equal(0,active);candidateSnapshots++;Assert.Equal(3,plans.Count);};
                VCarveNativeCurvePlanning.RegionObserver.Value=(index,_)=>{Assert.Equal(0,active);observed.Add(index);};
                VCarveNativeQueryWorker Factory()
                {
                    created++;
                    bool Chord(Point3D a,Point3D b)
                    {
                        Interlocked.Increment(ref active);
                        try{Assert.Equal(-1,a.Z);Assert.Equal(-1,b.Z);return true;}
                        finally{Interlocked.Decrement(ref active);}
                    }
                    return new(Chord,m=>Chord(m.Start,m.End)){ImmutableDomainProvenance="test-private-native-domain"};
                }
                var parallel=Run(source,(_,_)=>throw new InvalidOperationException("opaque owner used concurrently"),
                    _=>throw new InvalidOperationException("opaque owner used concurrently"),Factory,log:logs.Add,selected:selected,required:required);
                Assert.Equal(2,created);Assert.Equal(1,sourceSnapshots);Assert.Equal(1,candidateSnapshots);
                Assert.Equal(new[]{0,1,2},observed);
                Assert.Equal(serial.Select(p=>p.RegionIndex),parallel.Select(p=>p.RegionIndex));
                Assert.Equal(serial.Select(p=>p.VisitIdentity),parallel.Select(p=>p.VisitIdentity));
                Assert.Equal(serial.SelectMany(p=>p.Primitives),parallel.SelectMany(p=>p.Primitives));
                Assert.All(parallel,p=>Assert.True(p.CandidateVerified));
                Assert.Contains(parallel[0].Primitives,p=>p.SourceFirst==1||p.SourceLast==1);
                Assert.Contains(logs,l=>l.Contains("allWorkersJoined=True")&&l.Contains("stableSourceOrder=True"));
                Assert.Equal(2,logs.Count(l=>l.Contains("event=worker ")&&l.Contains("provenance=test-private-native-domain")));
            }
            finally
            {
                VCarveNativeCurvePlanning.EvidenceObserver.Value=oldSource;
                VCarveNativeCurvePlanning.CandidateObserver.Value=oldCandidate;
                VCarveNativeCurvePlanning.RegionObserver.Value=oldRegion;
            }
        });
    }

    [Fact]
    public void OpaqueCallbacksStaySerialWithoutTheExplicitFactory()
    {
        WithWorkers(4,()=>
        {
            int active=0,maximum=0,queries=0;
            bool Chord(Point3D _,Point3D __)
            {
                int value=Interlocked.Increment(ref active);maximum=Math.Max(maximum,value);queries++;
                try{return true;}finally{Interlocked.Decrement(ref active);}
            }
            Assert.All(Run(Sources(),Chord,m=>Chord(m.Start,m.End)),p=>Assert.True(p.CandidateVerified));
            Assert.True(queries>0);Assert.Equal(1,maximum);
        });
    }

    [Fact]
    public void SerialControlDoesNotCreatePrivateOwners()
    {
        WithWorkers(1,()=>
        {
            int created=0;
            VCarveNativeQueryWorker Factory(){created++;return new((_,_)=>true,_=>true);}
            Assert.All(Run(Sources(),(_,_)=>true,_=>true,Factory),p=>Assert.True(p.CandidateVerified));
            Assert.Equal(0,created);
        });
    }

    [Fact]
    public void GlobalSourceBudgetIsCheckedBeforeFactoryOrPartition()
    {
        WithWorkers(2,()=>
        {
            int created=0;var source=Sources();
            VCarveNativeQueryWorker Factory(){created++;return new((_,_)=>true,_=>true);}
            var plans=Run(source,(_,_)=>true,_=>true,Factory,limits:new(MaximumSourcePoints:8),
                topology:VCarveNativeTopologyContext.Create(source));
            Assert.Equal(0,created);Assert.Equal(source.Length,plans.Count);
            Assert.All(plans,p=>{Assert.True(p.BudgetExhausted);Assert.Equal("source-point-budget-exceeded",p.Status);});
        });
    }

    [Fact]
    public void InvalidSourceMapsAreRejectedBeforeWorkerCreation()
    {
        WithWorkers(2,()=>
        {
            int created=0;VCarveNativeQueryWorker Factory(){created++;return new((_,_)=>true,_=>true);}
            Assert.Throws<ArgumentException>(()=>Run(Sources(),(_,_)=>true,_=>true,Factory,
                required:[[0,2],[0,1],[0,2]]));Assert.Equal(0,created);
        });
    }

    [Theory]
    [InlineData(1)]
    [InlineData(2)]
    public void AnExactOnePointMapRetainsTheNonExecutableEmptySourceStatus(int workers)
    {
        WithWorkers(workers,()=>
        {
            var source=Sources().Take(2).ToArray();
            source[0]=new([source[0].Points[0]],[],1,"one-point",0,71,detectCurves:false);
            var plans=Run(source,(_,_)=>true,_=>true,()=>new((_,_)=>true,_=>true),
                selected:[[0],[0,1,2]],required:[[0],[0,2]]);
            Assert.Equal("empty-source",plans[0].Status);Assert.False(plans[0].CandidateVerified);
            Assert.Empty(plans[0].Primitives);Assert.Equal(0,plans[0].SourceSegments);
            Assert.True(plans[1].CandidateVerified);
        });
    }

    [Theory]
    [InlineData("region")]
    [InlineData("candidate")]
    [InlineData("diagnostic")]
    public void CancellationDuringFinalCallbackPublicationCannotReturnAPlan(string phase)
    {
        WithWorkers(2,()=>
        {
            using var cancellation=new CancellationTokenSource();
            var previousRegion=VCarveNativeCurvePlanning.RegionObserver.Value;
            var previousCandidate=VCarveNativeCurvePlanning.CandidateObserver.Value;
            try
            {
                VCarveNativeCurvePlanning.RegionObserver.Value=(_,_)=>{if(phase=="region")cancellation.Cancel();};
                VCarveNativeCurvePlanning.CandidateObserver.Value=_=>{if(phase=="candidate")cancellation.Cancel();};
                void Log(string message)
                {if(phase=="diagnostic"&&message.StartsWith("[VCarve.NativeRegions] event=complete "))cancellation.Cancel();}
                Assert.Throws<OperationCanceledException>(()=>Run(Sources(),(_,_)=>true,_=>true,
                    ()=>new((_,_)=>true,_=>true),cancellation.Token,Log));
            }
            finally
            {
                VCarveNativeCurvePlanning.RegionObserver.Value=previousRegion;
                VCarveNativeCurvePlanning.CandidateObserver.Value=previousCandidate;
            }
        });
    }

    [Fact]
    public void CancellationInsideAQueryJoinsWorkersBeforeThrowingOrPublishing()
    {
        WithWorkers(2,()=>
        {
            using var cancellation=new CancellationTokenSource();int active=0,queries=0,candidates=0;
            var old=VCarveNativeCurvePlanning.CandidateObserver.Value;
            try
            {
                VCarveNativeCurvePlanning.CandidateObserver.Value=_=>candidates++;
                bool Chord(Point3D _,Point3D __)
                {
                    Interlocked.Increment(ref active);Interlocked.Increment(ref queries);
                    try{cancellation.Cancel();Thread.SpinWait(1000);return true;}
                    finally{Interlocked.Decrement(ref active);}
                }
                Assert.Throws<OperationCanceledException>(()=>Run(Sources(),(_,_)=>true,_=>true,
                    ()=>new(Chord,m=>Chord(m.Start,m.End)),cancellation.Token));
                Assert.True(queries>0);Assert.Equal(0,active);Assert.Equal(0,candidates);
            }
            finally{VCarveNativeCurvePlanning.CandidateObserver.Value=old;}
        });
    }

    [Fact]
    public void AFailedPrivateQueryJoinsEveryWorkerBeforePropagatingItsOriginalError()
    {
        WithWorkers(2,()=>
        {
            int active=0;
            bool Chord(Point3D _,Point3D __)
            {
                Interlocked.Increment(ref active);
                try{throw new InvalidOperationException("private-query-failure");}
                finally{Interlocked.Decrement(ref active);}
            }
            var error=Assert.Throws<InvalidOperationException>(()=>Run(Sources(),(_,_)=>true,_=>true,
                ()=>new(Chord,m=>Chord(m.Start,m.End))));
            Assert.Equal("private-query-failure",error.Message);Assert.Equal(0,active);
        });
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(1000)]
    public void ConcurrentOrdinaryAndFastCandidatesShareOneExactHardBudget(int maximum)
    {
        var budget=new VCarveNativeSearchBudget(default,maximum);int accepted=0;
        Parallel.For(0,8,_=>
        {
            while(true)
            {
                try{if(!budget.TryCheckFastCandidate())budget.CheckCandidate();Interlocked.Increment(ref accepted);}
                catch(VCarveNativeBudgetException){break;}
            }
        });
        Assert.Equal(maximum,accepted);Assert.Equal(maximum,budget.Attempts);Assert.Equal(0,budget.Remaining);
        Assert.InRange(budget.FastProbeAttempts,0,maximum/20);
    }
}
