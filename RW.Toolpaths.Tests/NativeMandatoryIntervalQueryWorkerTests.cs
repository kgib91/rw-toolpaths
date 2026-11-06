using System.Text.RegularExpressions;
using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class NativeMandatoryIntervalQueryWorkerTests
{
    static readonly VCarveCutterProfile Profile=new([new(CarvingCutterSectionKind.Linear,0,2,2)],2);
    static readonly IReadOnlyList<IReadOnlyList<IReadOnlyList<PointD>>> Boundaries=
        new PointD[][][]{[new PointD[]{new(-10,-10),new(10,-10),new(10,10),new(-10,10)}]};

    static TaggedToolpath Path(Point3D[] points,int visit,IReadOnlyList<ToolpathSpan>? spans=null)
        =>new(points,spans??[new(0,points.Length-1,ToolpathSpanKind.Cut)],0,"mandatory-worker",0,visit,detectCurves:false);

    static (TaggedToolpath[] Source,IReadOnlyList<IReadOnlyList<int>> Selected,IReadOnlyList<IReadOnlyList<int>> Required)
        Fixture(double surface=10)
    {
        var p=Enumerable.Range(0,9).Select(i=>new Point3D(i*.4,0,surface-1)).ToArray();
        // Required degree-two splits remain fitting boundaries, while the
        // crossing is a genuine degree-four physical junction in the FULL graph.
        TaggedToolpath[] source=[Path(p,91,[new(0,4,ToolpathSpanKind.Cut),new(4,8,ToolpathSpanKind.Link)]),
            Path([new(p[3].X,-.4,surface-1),p[3],new(p[3].X,.4,surface-1)],92),
            Path([new(5,0,surface-1),new(5.08,.005,surface-1),new(5.18,0,surface-1)],93)];
        return(source,[[0,1,2,4,5,6,8],[0,1,2],[0,1,2]],[[0,2,3,4,6,8],[0,2],[0,2]]);
    }

    static T WithWorkers<T>(int workers,Func<T> action,bool intervals=true)
    {
        var previousWorkers=VCarveNativeCurvePlanning.RegionWorkerLimitOverride.Value;
        var previousIntervals=VCarveNativeCurvePlanning.MandatoryIntervalWorkersEnabled.Value;
        try
        {
            VCarveNativeCurvePlanning.RegionWorkerLimitOverride.Value=workers;
            VCarveNativeCurvePlanning.MandatoryIntervalWorkersEnabled.Value=intervals;
            return action();
        }
        finally
        {
            VCarveNativeCurvePlanning.RegionWorkerLimitOverride.Value=previousWorkers;
            VCarveNativeCurvePlanning.MandatoryIntervalWorkersEnabled.Value=previousIntervals;
        }
    }

    static IReadOnlyList<VCarveNativeCurvePlan> Run(TaggedToolpath[] source,double surface,
        IReadOnlyList<IReadOnlyList<int>> selected,IReadOnlyList<IReadOnlyList<int>> required,
        Func<Point3D,Point3D,bool> chord,Func<ToolMove,bool> curve,Func<VCarveNativeQueryWorker>? factory=null,
        Action<string>? log=null,CancellationToken token=default,VCarveNativeFitLimits? limits=null)
        =>VCarveNativeCurvePlanning.Nominate(source,Boundaries,Profile,surface,token,limits,log,new(.3,.3),
            chord,curve,selected,required,canonicalTopology:VCarveNativeTopologyContext.Create(source,token),
            stockQueryWorkerFactory:factory);

    static VCarveNativeQueryWorker PhysicalWorker(CancellationToken token=default)
    {
        var guard=new VCarveNativeContainment(Boundaries[0],Profile,token);
        return new(guard.Legal,m=>VCarveNativeCutterProof.Certified(m,guard,Profile,out _))
            {ImmutableDomainProvenance="independent-solid-boundary"};
    }

    static void SamePlans(IReadOnlyList<VCarveNativeCurvePlan> expected,IReadOnlyList<VCarveNativeCurvePlan> actual)
    {
        Assert.Equal(expected.Count,actual.Count);
        for(int i=0;i<expected.Count;i++)
        {
            Assert.Equal(expected[i].RegionIndex,actual[i].RegionIndex);
            Assert.Equal(expected[i].VisitIdentity,actual[i].VisitIdentity);
            Assert.Equal(expected[i].SourceSegments,actual[i].SourceSegments);
            Assert.Equal(expected[i].Status,actual[i].Status);
            Assert.Equal(expected[i].CandidateVerified,actual[i].CandidateVerified);
            Assert.Equal(expected[i].Primitives,actual[i].Primitives);
        }
    }

    [Theory]
    [InlineData(0)]
    [InlineData(10)]
    [InlineData(-12.7)]
    public void ExistingIntervalsRetainFullSourceOrdinalsKindsTopologyAndPhysicalProofs(double surface)
    {
        var (source,selected,required)=Fixture(surface);var original=source.Select(p=>p.Points.ToArray()).ToArray();
        var serialGuard=PhysicalWorker();
        var serial=WithWorkers(1,()=>Run(source,surface,selected,required,serialGuard.Chord,serialGuard.Curve));
        int created=0,active=0;var observations=new List<int>();var messages=new List<string>();
        var oldRegion=VCarveNativeCurvePlanning.RegionObserver.Value;
        var oldCandidate=VCarveNativeCurvePlanning.CandidateObserver.Value;
        try
        {
            VCarveNativeCurvePlanning.RegionObserver.Value=(i,_)=>{Assert.Equal(0,active);observations.Add(i);};
            VCarveNativeCurvePlanning.CandidateObserver.Value=_=>Assert.Equal(0,active);
            VCarveNativeQueryWorker Factory()
            {
                created++;var authority=PhysicalWorker();
                bool Chord(Point3D a,Point3D b)
                {
                    Interlocked.Increment(ref active);
                    try{Assert.Equal(-1,a.Z);Assert.Equal(-1,b.Z);return authority.Chord(a,b);}
                    finally{Interlocked.Decrement(ref active);}
                }
                return new(Chord,authority.Curve){ImmutableDomainProvenance=authority.ImmutableDomainProvenance};
            }
            var actual=WithWorkers(4,()=>Run(source,surface,selected,required,
                (_,_)=>throw new InvalidOperationException("opaque parent queried"),
                _=>throw new InvalidOperationException("opaque parent queried"),Factory,messages.Add));
            Assert.InRange(created,2,4);SamePlans(serial,actual);Assert.Equal(new[]{0,1,2},observations);
            Assert.All(actual,p=>Assert.True(p.CandidateVerified));
            Assert.Contains(messages,m=>m.StartsWith("[VCarve.NativeIntervals] event=complete ")
                &&m.Contains("allWorkersJoined=True")&&m.Contains("fullSourceAndGlobalTopology=True"));
            Assert.Contains(actual[0].Primitives,p=>p.SourceLast==3);
            Assert.Contains(actual[0].Primitives,p=>p.SourceFirst==3);
            Assert.Contains(actual[0].Primitives,p=>p.Kind==ToolpathSpanKind.Link);
            Assert.All(actual[2].Primitives,p=>Assert.Equal(new VCarveNativeShortFeature(0,2),p.ShortSourceFeature));
            var independent=PhysicalWorker();
            for(int i=0;i<actual.Count;i++)
            {
                Assert.Equal(original[i],source[i].Points);
                var relative=source[i].Points.Select(p=>p with{Z=p.Z-surface}).ToArray();
                foreach(var primitive in actual[i].Primitives)
                {
                    Assert.Equal(source[i].Points[primitive.SourceFirst],primitive.Move.Start);
                    Assert.Equal(source[i].Points[primitive.SourceLast],primitive.Move.End);
                    var move=VCarveNativeCurvePlanning.TranslateZ(primitive.Move,-surface);
                    Assert.True(VCarveNativeGeometricProof.Prove(move,relative,primitive.SourceFirst,primitive.SourceLast,.3).Certified);
                    Assert.True(independent.Curve(move));
                    Assert.True(VCarveNativeCurvePlanning.PreservesPlanarProgress(primitive.Move,source[i].Points,
                        primitive.SourceFirst,primitive.SourceLast,out _));
                }
            }
        }
        finally
        {
            VCarveNativeCurvePlanning.RegionObserver.Value=oldRegion;
            VCarveNativeCurvePlanning.CandidateObserver.Value=oldCandidate;
        }
    }

    [Fact]
    public void DegreeTwoPreparationSplitsNeverBecomeWholeShortFeatures()
    {
        Point3D[] p=[new(0,0,9),new(.1,0,9),new(.5,0,9),new(1,0,9),new(1.5,0,9),new(2,0,9)];
        TaggedToolpath[] source=[Path(p,1)];IReadOnlyList<IReadOnlyList<int>> maps=[[0,1,2,3,4,5]];
        var authority=PhysicalWorker();
        var serial=WithWorkers(1,()=>Run(source,10,maps,maps,authority.Chord,authority.Curve));
        var actual=WithWorkers(4,()=>Run(source,10,maps,maps,authority.Chord,authority.Curve,()=>PhysicalWorker()));
        SamePlans(serial,actual);Assert.False(actual[0].CandidateVerified);
        Assert.All(actual[0].Primitives,p=>Assert.Null(p.ShortSourceFeature));
        Assert.Contains(actual[0].Primitives,p=>p.SourceFirst==0&&p.SourceLast==1&&p.LengthUpperMm<.3);
    }

    [Fact]
    public void ReversedRepeatedGeometrySharesOneOwnerButKeepsOccurrenceMapsAndKinds()
    {
        Point3D[] p=Enumerable.Range(0,19).Select(i=>new Point3D(i*.2,.002*i*i,9)).ToArray();
        TaggedToolpath[] source=[Path(p,10),Path(p,11,[new(0,18,ToolpathSpanKind.Link)]),Path(p.Reverse().ToArray(),12)];
        IReadOnlyList<IReadOnlyList<int>> selected=[Enumerable.Range(0,19).ToArray(),[0,2,4,6,8,10,12,15,18],[0,1,4,7,10,14,18]],
            required=[[0,4,8,12,18],[0,4,8,12,18],[0,6,10,14,18]];
        var authority=PhysicalWorker();var log=new List<string>();
        var serial=WithWorkers(1,()=>Run(source,10,selected,required,authority.Chord,authority.Curve));
        var actual=WithWorkers(4,()=>Run(source,10,selected,required,authority.Chord,authority.Curve,()=>PhysicalWorker(),log.Add));
        SamePlans(serial,actual);Assert.All(actual,p=>Assert.True(p.CandidateVerified));
        var groups=log.Where(m=>m.StartsWith("[VCarve.NativeIntervals] event=group ")).ToArray();
        Assert.Equal(4,groups.Length);Assert.All(groups,m=>Assert.Contains("occurrences=3 ",m));
        Assert.All(actual[1].Primitives,p=>Assert.Equal(ToolpathSpanKind.Link,p.Kind));
        Assert.Equal(p[^1],actual[2].Primitives[0].Move.Start);Assert.Equal(p[0],actual[2].Primitives[^1].Move.End);
        for(int i=0;i<actual.Count;i++)
        for(int h=1;h<required[i].Count;h++)
        {
            int first=required[i][h-1],last=required[i][h];
            var knots=selected[i].Where(k=>k>=first&&k<=last).Append(first).Append(last).Distinct().Order().ToArray();
            var block=actual[i].Primitives.Where(f=>f.SourceFirst>=first&&f.SourceLast<=last).ToArray();
            Assert.NotEmpty(block);
            var expectedStart=VCarveNativeCurveMath.Unit(VCarveNativeCurveMath.Sub(source[i].Points[knots[1]],source[i].Points[first]));
            var expectedEnd=VCarveNativeCurveMath.Unit(VCarveNativeCurveMath.Sub(source[i].Points[last],source[i].Points[knots[^2]]));
            Assert.True(VCarveNativeCurveMath.Distance(expectedStart,VCarveNativeCurveMath.Unit(VCarveNativeCurveMath.Derivative(block[0].Move,0)))<1e-8);
            Assert.True(VCarveNativeCurveMath.Distance(expectedEnd,VCarveNativeCurveMath.Unit(VCarveNativeCurveMath.Derivative(block[^1].Move,1)))<1e-8);
        }
    }

    [Fact]
    public void OpaqueAndOptedOutOwnersRetainTheSerialSingleRegionPath()
    {
        var(source,selected,required)=Fixture();int created=0,active=0,maximum=0;
        bool Chord(Point3D _,Point3D __)
        {int n=Interlocked.Increment(ref active);maximum=Math.Max(maximum,n);try{return true;}finally{Interlocked.Decrement(ref active);}}
        VCarveNativeQueryWorker Factory(){created++;return new(Chord,m=>Chord(m.Start,m.End));}
        var serial=WithWorkers(4,()=>Run(source,10,selected,required,Chord,m=>Chord(m.Start,m.End)));
        var optOut=WithWorkers(4,()=>Run(source,10,selected,required,Chord,m=>Chord(m.Start,m.End),Factory),intervals:false);
        SamePlans(serial,optOut);Assert.Equal(0,created);Assert.Equal(1,maximum);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CancellationAndQueryFailureJoinAllWorkersBeforeAnyObserverPublication(bool cancel)
    {
        var(source,selected,required)=Fixture();using var cancellation=new CancellationTokenSource();
        int active=0,queries=0,published=0;var oldRegion=VCarveNativeCurvePlanning.RegionObserver.Value;
        var oldCandidate=VCarveNativeCurvePlanning.CandidateObserver.Value;
        try
        {
            VCarveNativeCurvePlanning.RegionObserver.Value=(_,_)=>published++;
            VCarveNativeCurvePlanning.CandidateObserver.Value=_=>published++;
            bool Chord(Point3D _,Point3D __)
            {
                Interlocked.Increment(ref active);Interlocked.Increment(ref queries);
                try
                {
                    if(cancel){cancellation.Cancel();Thread.SpinWait(1000);return true;}
                    throw new InvalidOperationException("mandatory-private-query-failure");
                }
                finally{Interlocked.Decrement(ref active);}
            }
            void Act()=>WithWorkers(4,()=>Run(source,10,selected,required,(_,_)=>true,_=>true,
                ()=>new(Chord,m=>Chord(m.Start,m.End)),token:cancellation.Token));
            if(cancel)Assert.Throws<OperationCanceledException>(Act);
            else Assert.Equal("mandatory-private-query-failure",Assert.Throws<InvalidOperationException>(Act).Message);
            Assert.True(queries>0);Assert.Equal(0,active);Assert.Equal(0,published);
        }
        finally
        {
            VCarveNativeCurvePlanning.RegionObserver.Value=oldRegion;
            VCarveNativeCurvePlanning.CandidateObserver.Value=oldCandidate;
        }
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(3)]
    public void MandatoryIntervalsShareOneOperationCandidateBudget(int maximum)
    {
        var(source,selected,required)=Fixture();var log=new List<string>();
        var result=WithWorkers(4,()=>Run(source,10,selected,required,(_,_)=>true,_=>true,
            ()=>new((_,_)=>true,_=>true),log.Add,limits:new(MaximumCandidateEdges:maximum)));
        var complete=Assert.Single(log.Where(m=>m.StartsWith("[VCarve.NativeIntervals] event=complete ")));
        int attempts=int.Parse(Regex.Match(complete,@"candidateAttempts=(\d+)").Groups[1].Value);
        int fast=int.Parse(Regex.Match(complete,@"fastProbeAttempts=(\d+)").Groups[1].Value);
        Assert.InRange(attempts,0,maximum);Assert.InRange(fast,0,maximum/20);
        Assert.Contains(result,p=>p.BudgetExhausted);Assert.Contains($"candidateLimit={maximum} ",complete);
    }

    [Theory]
    [InlineData("region")]
    [InlineData("candidate")]
    [InlineData("diagnostic")]
    public void CancellationDuringOrderedPublicationCannotReturnACompletedIntervalPlan(string phase)
    {
        var(source,selected,required)=Fixture();using var cancellation=new CancellationTokenSource();
        var oldRegion=VCarveNativeCurvePlanning.RegionObserver.Value;
        var oldCandidate=VCarveNativeCurvePlanning.CandidateObserver.Value;
        try
        {
            VCarveNativeCurvePlanning.RegionObserver.Value=(_,_)=>{if(phase=="region")cancellation.Cancel();};
            VCarveNativeCurvePlanning.CandidateObserver.Value=_=>{if(phase=="candidate")cancellation.Cancel();};
            void Log(string message)
            {if(phase=="diagnostic"&&message.StartsWith("[VCarve.NativeIntervals] event=complete "))cancellation.Cancel();}
            Assert.Throws<OperationCanceledException>(()=>WithWorkers(4,()=>Run(source,10,selected,required,
                (_,_)=>true,_=>true,()=>new((_,_)=>true,_=>true),Log,cancellation.Token)));
        }
        finally
        {
            VCarveNativeCurvePlanning.RegionObserver.Value=oldRegion;
            VCarveNativeCurvePlanning.CandidateObserver.Value=oldCandidate;
        }
    }

    [Fact]
    public void GlobalSourceBudgetAndInvalidMapsAreCheckedBeforePrivateOwnerCreation()
    {
        var(source,selected,required)=Fixture();int created=0;
        VCarveNativeQueryWorker Factory(){created++;return new((_,_)=>true,_=>true);}
        var result=WithWorkers(4,()=>Run(source,10,selected,required,(_,_)=>true,_=>true,Factory,
            limits:new(MaximumSourcePoints:14)));
        Assert.Equal(0,created);Assert.All(result,p=>Assert.Equal("source-point-budget-exceeded",p.Status));
        IReadOnlyList<IReadOnlyList<int>> invalid=[[0,2,3,4,6,7],[0,2],[0,2]];
        Assert.Throws<ArgumentException>(()=>WithWorkers(4,()=>Run(source,10,selected,invalid,(_,_)=>true,_=>true,Factory)));
        Assert.Equal(0,created);
    }
}
