using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class MillingEndpointCandidateTests
{
    private static VCarveCutterProfile Cone=>new([new(CarvingCutterSectionKind.Linear,.05,3.175,10)],5);
    private static VCarveCutterProfile Rounded=>new([new(CarvingCutterSectionKind.Hemisphere,0,.8,.8),
        new(CarvingCutterSectionKind.Linear,.8,3.175,9.2)],5);
    private static CutterSweepStock Stock(VCarveCutterProfile cutter,IEnumerable<(Point3D A,Point3D B)> cuts,
        bool enabled=true,CancellationToken token=default)
    {
        var stock=new CutterSweepStock(cutter,p=>p,0,token,mergeIncrementally:true,endpointCandidateDiscovery:enabled);
        foreach(var (a,b) in cuts)stock.RecordExecuted(a,b);stock.Seal();return stock;
    }
    private static bool Oracle(CutterSweepStock stock,Point3D a,Point3D b,out bool refused)
    {
        long queries=stock.Queries,trials=stock.PriorWholeChordTrials,accepted=stock.Accepted;
        var discovery=stock.FindTravelEndpointCandidates(a,b,.1);
        Assert.Equal(queries,stock.Queries);Assert.Equal(trials,stock.PriorWholeChordTrials);Assert.Equal(accepted,stock.Accepted);
        bool full=stock.ContainsTravel(a,b,.1);refused=!discovery.Possible;
        Assert.True(discovery.Possible||!full,$"Candidate discovery refused an ORIGINAL full proof: {a} -> {b}.");
        return full;
    }

    [Fact]
    public void NegativeCandidatesAreFalsifiedAgainstTheUnchangedCompleteProofAcrossProfilesAndPrefixes()
    {
        int refusals=0,positives=0;var random=new Random(63728);
        foreach(var cutter in new[]{Cone,Rounded})
        {
            var cuts=new List<(Point3D,Point3D)>{(new(-2,0,-5),new(2,0,-5))};
            for(int i=0;i<20;i++)cuts.Add((new(random.NextDouble()*10-5,random.NextDouble()*10-5,-4-random.NextDouble()),
                new(random.NextDouble()*10-5,random.NextDouble()*10-5,-4-random.NextDouble())));
            var stock=Stock(cutter,cuts);
            Assert.True(Oracle(stock,new(-1,0,-3.9),new(1,0,-3.9),out _));positives++;
            for(int i=0;i<150;i++)
            {
                double z=-.2-random.NextDouble()*3.8;
                var a=new Point3D(random.NextDouble()*18-9,random.NextDouble()*18-9,z);
                var b=new Point3D(random.NextDouble()*18-9,random.NextDouble()*18-9,z);
                if(Oracle(stock,a,b,out bool refused))positives++;if(refused)refusals++;
            }
        }
        Assert.True(refusals>0);Assert.True(positives>0);
    }

    [Fact]
    public void RoundedJoinsAndCapsStillReceiveTheirOriginalCompleteIntervalUnionCertificate()
    {
        var stock=Stock(Cone,[(new(-2,0,-5),new(0,0,-5)),(new(.0004,0,-5),new(2,0,-5))]);
        Assert.True(Oracle(stock,new(-1,0,-3.9),new(1,0,-3.9),out bool refused));Assert.False(refused);
        Assert.True(stock.UnionAccepted>0,"Fixture must require the original multi-chord interval union.");
        var single=Stock(Cone,[(new(0,0,-5),new(0,0,-5))]);
        double boundary=single.RadialMargin(1,4)-.1-1e-7;
        foreach(double x in new[]{Math.BitDecrement(boundary),boundary,Math.BitIncrement(boundary)})
            Oracle(single,new(x,0,-3.9),new(.15,0,-3.9),out _);
    }

    [Fact]
    public void OutwardBoundsRetainLargeCoordinateAndEndpointReconstructionCandidates()
    {
        foreach(var (a,b) in new[]{
            (new Point3D(9999.123456789,9999.4,-5),new Point3D(-9999.987654321,9999.4,-5)),
            (new Point3D(-10000,-9999.4,-5),new Point3D(10000,-9999.4,-5))})
        {
            var stock=Stock(Cone,[(a,b)]);
            Assert.True(Oracle(stock,b with{Z=-3.9},new(b.X+(b.X<0?1:-1),b.Y,-3.9),out bool refused));
            Assert.False(refused);
        }
    }

    [Fact]
    public void AFalseDegeneratePoseIsNeverUsedAsAWholeIntervalNecessityTest()
    {
        var stock=Stock(Cone,[(new(0,0,-12),new(10,0,-8))]);
        // RadialMargin's represented-height limit can refuse the endpoint pose
        // while the interval's smaller shared delta still earns a certificate.
        Assert.False(stock.Contains(new(0,0,-4),new(0,0,-4),0,.1));
        Assert.True(Oracle(stock,new(0,0,-3.9),new(10,0,-3.9),out bool refused));Assert.False(refused);
    }

    [Fact]
    public void UnsupportedUnsealedVariableDepthAndGenericCallersBypassWithoutBudgetMutation()
    {
        var generic=Stock(Cone,[],enabled:false);
        Assert.True(generic.FindTravelEndpointCandidates(new(0,0,-2),new(10,0,-2),.1).Bypassed);
        var stock=Stock(Cone,[]);stock.RecordExecuted(new(0,0,-5),new(2,0,-5));
        Assert.True(stock.FindTravelEndpointCandidates(new(0,0,-2),new(10,0,-2),.1).Bypassed);
        stock.Seal();
        Assert.True(stock.FindTravelEndpointCandidates(new(0,0,-2),new(10,0,-3),.1).Bypassed);
        Assert.True(stock.FindTravelEndpointCandidates(new(10001,0,-2),new(10,0,-2),.1).Bypassed);
        var extreme=new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear,0,1,1e-7)],1e-7);
        Assert.True(Stock(extreme,[]).FindTravelEndpointCandidates(new(0,0,-2),new(10,0,-2),.1).Bypassed);
        Assert.Equal(0,stock.Queries);
    }

    [Theory]
    [InlineData(0f)]
    [InlineData(12.7f)]
    [InlineData(237.123f)]
    public void LocalEmitterKeepsExactLegacyTransfersAcrossFramesAndNeverUsesFutureOrOtherRegionCuts(float surface)
    {
        (IReadOnlyList<MillingTransition>[] Moves,List<string> Log) Run(bool enabled)
        {
            var old=MillingLocalTravel.EndpointCandidateOptimizationOverride.Value;
            try
            {
                MillingLocalTravel.EndpointCandidateOptimizationOverride.Value=enabled;
                var log=new List<string>();var owner=new MillingLocalTravel(Cone,surface,default,log.Add);
                owner.BeginRegion(0);owner.RecordExecutedCut(new(20,20,-5),new(21,20,-5));
                var from=new Point3D(21,20,surface-5);var to=new Point3D(30,20,surface-5);
                IReadOnlyList<MillingTransition> Build()=>MillingTransitionPlanner.Build(from,to,surface+5,surface,
                    localTravel:owner,postedFrom:new(21,20,-5));
                var beforeFuture=Build();owner.RecordExecutedCut(new(30,20,-5),new(21,20,-5));
                var afterExecuted=Build();owner.BeginRegion(1);var otherRegion=Build();owner.Report();
                return([beforeFuture,afterExecuted,otherRegion],log);
            }
            finally{MillingLocalTravel.EndpointCandidateOptimizationOverride.Value=old;}
        }
        var control=Run(false);var actual=Run(true);
        for(int i=0;i<control.Moves.Length;i++)Assert.Equal(control.Moves[i],actual.Moves[i]);
        Assert.Contains(actual.Moves[0],m=>m.End.Z==surface+5);
        Assert.Contains(actual.Moves[1],m=>m.ClearedTravel&&m.Start.Z<surface);
        Assert.Contains(actual.Moves[2],m=>m.End.Z==surface+5);
        Assert.Contains(actual.Log,l=>l.Contains("event=prefix-endpoint-without-candidate")&&l.Contains("mutableHintsTouched=False"));
    }

    [Fact]
    public void CylinderColumnUnionRemainsOutsideThisOptionalCandidatePolicy()
    {
        var log=new List<string>();var flat=new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear,.5,.5,8)],5);
        var owner=new MillingLocalTravel(flat,10,default,log.Add);owner.BeginRegion(0);
        owner.RecordExecutedCut(new(-.4,-5,-3),new(-.4,5,-3));owner.RecordExecutedCut(new(.4,-5,-3),new(.4,5,-3));
        var moves=MillingTransitionPlanner.Build(new(0,0,7),new(0,3,8),15,10,localTravel:owner,postedFrom:new(0,0,-3));
        Assert.Contains(moves,m=>m.ClearedTravel);owner.Report();
        Assert.Contains(log,l=>l.Contains("event=summary")&&l.Contains("endpointCandidateChecks=0 "));
    }

    [Fact]
    public void CancellationPreventsCandidateDiscoveryWithoutSpendingAFullProofQuery()
    {
        using var cancel=new CancellationTokenSource();var stock=Stock(Cone,[(new(0,0,-5),new(1,0,-5))],token:cancel.Token);
        cancel.Cancel();Assert.ThrowsAny<OperationCanceledException>(()=>stock.FindTravelEndpointCandidates(new(0,0,-2),new(10,0,-2),.1));
        Assert.Equal(0,stock.Queries);
    }
}
