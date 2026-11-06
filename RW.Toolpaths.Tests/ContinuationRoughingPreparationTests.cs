using System.Collections;
using System.Reflection;
using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class ContinuationRoughingPreparationTests
{
    private static readonly ToolGeometry Tool=new(1){VCarveProfile=new([new(CarvingCutterSectionKind.Linear,0,1,2)],2)};
    private static TaggedToolpath Path(int owner,double x,double surface=0)=>new(
        new Point3D[]{new(x,0,surface-.5),new(x+1,0,surface-1),new(x+2,0,surface-1)},
        [new(0,2,ToolpathSpanKind.Cut)],0,"clearing",0,detectCurves:false)
        {ContinuationSourceOrdinal=owner,ContinuationComponentIndex=owner};
    private static VCarveRoughingChord[] Cuts(int count)=>Enumerable.Range(0,count).Select(i=>
        new VCarveRoughingChord(new(i*4,-1,-3),new(i*4+2,1,-3),2,2)).ToArray();
    private static bool Overlaps(VCarveRoughingChord c,double left,double bottom,double right,double top)
        =>!(Math.Max(c.A.X,c.B.X)<left||Math.Min(c.A.X,c.B.X)>right
            ||Math.Max(c.A.Y,c.B.Y)<bottom||Math.Min(c.A.Y,c.B.Y)>top);

    [Fact]
    public void ExactOrderedSelectionMatchesLiteralBoundsIncludingTouchingAndReversedChords()
    {
        var cuts=Cuts(1000);
        cuts[3]=new(new(10,-1,-2),new(10,1,-3),1,1);
        cuts[5]=new(new(12,1,-2),new(8,-1,-3),1,0);
        var preparation=new VCarveContinuationAdapter.RoughingPreparation(cuts);
        Assert.True(preparation.TrySnapshot(cuts,default,out var detached));Assert.NotSame(cuts,detached);
        var random=new Random(72561);
        for(int trial=0;trial<80;trial++)
        {
            double left=trial==0?10:random.Next(-5,4010),right=left+random.Next(0,30);
            double bottom=trial%2==0?-1:1,top=bottom+1;
            int[] expected=Enumerable.Range(0,cuts.Length).Where(i=>Overlaps(cuts[i],left,bottom,right,top)).ToArray();
            Assert.Equal(expected,preparation.Select(left,bottom,right,top,default));
        }
        Assert.Equal(1,preparation.IndexBuilds);Assert.True(preparation.BoundsChordTests<preparation.ConsideredChords/4);
        Assert.Equal(0,preparation.LiteralFallbackQueries);
    }

    [Fact]
    public void TheFirstHundredThousandCapNeverAdmitsLaterNearbyStockNominees()
    {
        var cuts=Cuts(100_003);
        cuts[2]=new(new(-1,0,-3),new(1,0,-3),2,2);
        cuts[99_999]=new(new(0,-1,-3),new(0,1,-3),2,2);
        cuts[100_000]=cuts[2];cuts[100_002]=cuts[2];
        var preparation=new VCarveContinuationAdapter.RoughingPreparation(cuts);
        Assert.True(preparation.TrySnapshot(cuts,default,out var snapshot));Assert.Equal(cuts.Length,snapshot!.Length);
        var expected=Enumerable.Range(0,100_000).Where(i=>Overlaps(cuts[i],-1,-1,1,1)).ToArray();
        Assert.Equal(expected,preparation.Select(-1,-1,1,1,default));
        Assert.All(preparation.Select(-1,-1,1,1,default),i=>Assert.True(i<100_000));
        Assert.InRange(preparation.RetainedBytes,1,VCarveContinuationAdapter.RoughingPreparation.MaximumBytes);
    }

    [Theory]
    [InlineData(double.NaN)] [InlineData(double.PositiveInfinity)] [InlineData(1e13)]
    public void UncertainIndexCoordinatesUseTheCompleteLiteralNominationInventory(double exceptional)
    {
        var cuts=Cuts(64);cuts[17]=cuts[17] with{A=new(exceptional,0,-3)};
        var preparation=new VCarveContinuationAdapter.RoughingPreparation(cuts);
        Assert.True(preparation.TrySnapshot(cuts,default,out _));
        Assert.Equal(Enumerable.Range(0,cuts.Length),preparation.Select(-1,-1,1,1,default));
        Assert.Equal(1,preparation.LiteralFallbackQueries);Assert.Equal(0,preparation.IndexBuilds);
    }

    [Fact]
    public void SnapshotIsDetachedAndUnrelatedOrChangedCountOwnersCannotBorrowIt()
    {
        var source=Cuts(32).ToList();var preparation=new VCarveContinuationAdapter.RoughingPreparation(source);
        Assert.True(preparation.TrySnapshot(source,default,out var captured));var first=captured![0];
        source[0]=new(new(100,100,-3),new(101,100,-3),2,2);
        Assert.Equal(first,captured[0]);Assert.True(preparation.TrySnapshot(source,default,out var reused));Assert.Same(captured,reused);
        Assert.False(preparation.TrySnapshot(source.ToArray(),default,out _));
        source.Add(first);Assert.False(preparation.TrySnapshot(source,default,out _));
        Assert.Equal(1,preparation.SnapshotBuilds);Assert.Equal(1,preparation.SnapshotReuses);
        var refused=new VCarveContinuationAdapter.RoughingPreparation(source,maximumBytes:0);
        Assert.False(refused.TrySnapshot(source,default,out _));Assert.Equal(0,refused.RetainedBytes);
    }

    [Fact]
    public void CancelledCaptureOrSelectionPublishesNoPartialOwnedPreparation()
    {
        var cuts=Cuts(128);var preparation=new VCarveContinuationAdapter.RoughingPreparation(cuts);
        using var cancellation=new CancellationTokenSource();cancellation.Cancel();
        Assert.Throws<OperationCanceledException>(()=>preparation.TrySnapshot(cuts,cancellation.Token,out _));
        Assert.Equal(0,preparation.SnapshotBuilds);
        Assert.True(preparation.TrySnapshot(cuts,default,out _));
        Assert.Throws<OperationCanceledException>(()=>preparation.Select(-1,-1,20,20,cancellation.Token));
        Assert.Equal(0,preparation.IndexBuilds);Assert.Equal(0,preparation.Queries);
        Assert.Equal(Enumerable.Range(0,cuts.Length).Where(i=>Overlaps(cuts[i],-1,-1,20,20)),
            preparation.Select(-1,-1,20,20,default));
    }

    [Theory]
    [InlineData(0)] [InlineData(12.7)]
    public void PrivateAdaptersHaveIdenticalSeedGraphsFullGuardCallsAndConnections(double surface)
    {
        var cuts=Cuts(128);var depth=new DepthSchedule(2,2){SurfaceZ=surface};
        var options=Options(surface,cuts).WithPriorStock(default);
        var preparation=new VCarveContinuationAdapter.RoughingPreparation(cuts);
        for(int family=0;family<6;family++)
        {
            var oldCalls=new List<(Point3D,Point3D)>();var newCalls=new List<(Point3D,Point3D)>();
            var source=Path(family,family*4,surface);
            bool Guard(Point3D a,Point3D b,List<(Point3D,Point3D)> calls){calls.Add((a,b));return Math.Abs(a.Z-b.Z)<=2;}
            var literal=new VCarveContinuationAdapter(Tool,depth,options,(a,b)=>Guard(a,b,oldCalls),default,null,[source]);
            var actual=new VCarveContinuationAdapter(Tool,depth,options,(a,b)=>Guard(a,b,newCalls),default,null,[source],preparation);
            Assert.Equal(PlannerSnapshot(literal),PlannerSnapshot(actual));
            var oldPath=source;var newPath=source;Point3D from=new(family*4-1,0,surface-.7);
            var oldConnection=literal.Connect(from,ref oldPath);var newConnection=actual.Connect(from,ref newPath);
            Assert.Equal(oldConnection?.Points,newConnection?.Points);
            Assert.Equal(oldConnection?.ClearedTravel,newConnection?.ClearedTravel);
            Assert.Equal(oldConnection?.Target,newConnection?.Target);Assert.Equal(oldConnection?.Kind,newConnection?.Kind);
            Assert.Equal(oldPath.Points,newPath.Points);Assert.Equal(oldPath.Spans,newPath.Spans);
            Assert.Equal(oldCalls,newCalls);Assert.Equal(PlannerSnapshot(literal),PlannerSnapshot(actual));
        }
        Assert.Equal(1,preparation.SnapshotBuilds);Assert.Equal(5,preparation.SnapshotReuses);
        Assert.Equal(6,preparation.Queries);Assert.True(preparation.SelectedChords<preparation.ConsideredChords);
    }

    [Fact]
    public void CompleteRouteRetainsTheLiteralSourcesCallbackOrderAndPerFamilyPrivateHistory()
    {
        var source=Enumerable.Range(0,7).Select(i=>Path(i,i*4)).ToArray();var cuts=Cuts(256);
        var boundary=new PointD[][]{[new(-20,-20),new(2000,-20),new(2000,20),new(-20,20)]};
        (List<TaggedToolpath> Paths,List<(Point3D,Point3D)> Guard,List<(Point3D,Point3D)> Travel,List<Point3D> Heights,List<string> Log) Run(bool enabled)
        {
            var before=VCarveContinuationAdapter.RoughingPreparationEnabled.Value;
            try
            {
                VCarveContinuationAdapter.RoughingPreparationEnabled.Value=enabled;
                var guards=new List<(Point3D,Point3D)>();var travel=new List<(Point3D,Point3D)>();
                var heights=new List<Point3D>();var log=new List<string>();
                var physical=new VCarveNativeContainment(boundary,Tool.VCarveProfile!);
                var options=Options(0,cuts).WithOwnedRoughingPreparation();
                options=options with{Motion=options.Motion! with{ClearedTravelGuard=(a,b)=>{travel.Add((a,b));return false;}}};
                var result=VCarvePathRouter.Route(source,boundary,Tool,new(2,2),options,0,new(-1,0,-.7),
                    default,log.Add,(a,b)=>{guards.Add((a,b));return physical.Legal(a,b);},p=>{heights.Add(p);return 0;});
                return(result,guards,travel,heights,log);
            }
            finally{VCarveContinuationAdapter.RoughingPreparationEnabled.Value=before;}
        }
        var literal=Run(false);var actual=Run(true);
        Assert.Equal(literal.Paths.Count,actual.Paths.Count);
        for(int i=0;i<literal.Paths.Count;i++)
        {
            Assert.Equal(literal.Paths[i].Points,actual.Paths[i].Points);Assert.Equal(literal.Paths[i].Spans,actual.Paths[i].Spans);
            Assert.Equal(literal.Paths[i].ContinuationSourceOrdinal,actual.Paths[i].ContinuationSourceOrdinal);
            Assert.Equal(literal.Paths[i].RegionIndex,actual.Paths[i].RegionIndex);
        }
        Assert.Equal(literal.Guard,actual.Guard);Assert.Equal(literal.Travel,actual.Travel);Assert.Equal(literal.Heights,actual.Heights);
        var independent=new VCarveNativeContainment(boundary,Tool.VCarveProfile!);
        foreach(var path in actual.Paths)for(int i=1;i<path.Points.Count;i++)
            Assert.True(independent.Legal(path.Points[i-1],path.Points[i]));
        var summary=Assert.Single(actual.Log.Where(s=>s.Contains("stage=continuation-roughing-preparation")));
        Assert.Contains("snapshotBuilds=1 ",summary);Assert.Contains("snapshotReuses=6 ",summary);
        Assert.Contains("snapshotCopies=0 ",summary);
        Assert.Contains("queries=7 ",summary);Assert.Contains("privatePlannerAndQuerySessions=True",summary);
        Assert.DoesNotContain(literal.Log,s=>s.Contains("stage=continuation-roughing-preparation"));
    }

    [Fact]
    public void OpaqueEntryFailureIsNotHiddenByPreparationSummaryFailure()
    {
        var previous=VCarveContinuationAdapter.RoughingPreparationEnabled.Value;
        try
        {
            VCarveContinuationAdapter.RoughingPreparationEnabled.Value=true;
            var source=Path(0,0);var options=Options(0,Cuts(64)).WithOwnedRoughingPreparation();
            var error=Assert.Throws<InvalidOperationException>(()=>VCarvePathRouter.Route([source],
                new PointD[][]{[new(-10,-10),new(500,-10),new(500,10),new(-10,10)]},Tool,new(2,2),options,0,
                new(-1,0,-.7),default,message=>{
                    if(message.Contains("stage=continuation-roughing-preparation"))throw new InvalidOperationException("summary-failure");
                },(_,_)=>true,_=>throw new InvalidOperationException("entry-owner-failure")));
            Assert.Equal("entry-owner-failure",error.Message);
        }
        finally{VCarveContinuationAdapter.RoughingPreparationEnabled.Value=previous;}
    }

    [Fact]
    public void ExplicitFactoryOwnsReadOnlyValuesAndPublicSourceReplacementInvalidatesCapability()
    {
        var mutable=Cuts(32);var generic=Options(0,mutable);
        Assert.Null(generic.OwnedRouteRoughingValues);
        var owned=generic.WithOwnedRoughingPreparation();var captured=owned.RoughingRestChords[0];
        Assert.NotSame(mutable,owned.RoughingRestChords);Assert.NotNull(owned.OwnedRouteRoughingValues);
        mutable[0]=mutable[0] with{ClearedColumnRadius=100};Assert.Equal(captured,owned.RoughingRestChords[0]);
        Assert.Throws<NotSupportedException>(()=>((IList<VCarveRoughingChord>)owned.RoughingRestChords)[0]=mutable[0]);
        Assert.Same(owned,owned.WithOwnedRoughingPreparation());
        Assert.Null((owned with{RoughingRestChords=mutable}).OwnedRouteRoughingValues);
        var oversized=generic with{RoughingRestChords=new OversizedCuts()};
        Assert.Same(oversized,oversized.WithOwnedRoughingPreparation());Assert.Null(oversized.OwnedRouteRoughingValues);
    }

    [Fact]
    public void UnknownSameCountMutableInputsKeepLiteralPerAdapterCaptureAndOpaqueCallbackOrder()
    {
        var source=Enumerable.Range(0,3).Select(i=>Path(i,i*4)).ToArray();
        var boundary=new PointD[][]{[new(-20,-20),new(2000,-20),new(2000,20),new(-20,20)]};
        (List<TaggedToolpath> Paths,List<(Point3D,Point3D)> Calls,List<string> Log,int HeightCalls) Run(bool enabled)
        {
            var previous=VCarveContinuationAdapter.RoughingPreparationEnabled.Value;
            try
            {
                VCarveContinuationAdapter.RoughingPreparationEnabled.Value=enabled;
                var mutable=Cuts(64);var calls=new List<(Point3D,Point3D)>();var log=new List<string>();int heights=0;
                var options=Options(0,mutable);Assert.Null(options.OwnedRouteRoughingValues);
                var actual=VCarvePathRouter.Route(source,boundary,Tool,new(2,2),options,0,new(-1,0,-.7),default,log.Add,
                    (a,b)=>{calls.Add((a,b));return true;},p=>{
                        heights++;mutable[0]=mutable[0] with{A=new(heights*10,0,-3)};return 0;});
                Assert.Equal(64,mutable.Length);return(actual,calls,log,heights);
            }
            finally{VCarveContinuationAdapter.RoughingPreparationEnabled.Value=previous;}
        }
        var literal=Run(false);var actual=Run(true);
        Assert.True(actual.HeightCalls>0);Assert.Equal(literal.HeightCalls,actual.HeightCalls);
        Assert.Equal(literal.Calls,actual.Calls);Assert.Equal(literal.Paths.Count,actual.Paths.Count);
        for(int i=0;i<literal.Paths.Count;i++)
        {Assert.Equal(literal.Paths[i].Points,actual.Paths[i].Points);Assert.Equal(literal.Paths[i].Spans,actual.Paths[i].Spans);}
        Assert.DoesNotContain(actual.Log,s=>s.Contains("stage=continuation-roughing-preparation"));
    }

    private sealed class OversizedCuts:IReadOnlyList<VCarveRoughingChord>
    {
        public int Count=>1_000_000;
        public VCarveRoughingChord this[int index]=>throw new InvalidOperationException("Oversized source must not be read or copied.");
        public IEnumerator<VCarveRoughingChord> GetEnumerator()=>throw new InvalidOperationException("Oversized source must not be enumerated.");
        IEnumerator IEnumerable.GetEnumerator()=>GetEnumerator();
    }

    private static VCarveOptions Options(double surface,IReadOnlyList<VCarveRoughingChord> cuts)=>new(){
        SmoothVCarving=true,DetectCurves=false,OptimizeContourConnections=true,CompactCollinearCommands=true,
        Motion=new(600,300,3000,surface+5,surface),Ramp=new(Math.Atan(.5)),StepOver=.45,SmoothRampAngleDegrees=50,
        RoughingRestChords=cuts};
    private static object Field(object owner,string name)=>owner.GetType().GetField(name,
        BindingFlags.Instance|BindingFlags.Public|BindingFlags.NonPublic)!.GetValue(owner)!;
    private static long[] PlannerSnapshot(VCarveContinuationAdapter adapter)
    {
        var planner=Field(adapter,"planner");var result=new List<long>();
        foreach(var node in (IEnumerable)Field(planner,"nodes"))
        {
            var p=(Point3D)Field(node,"Point");result.Add(BitConverter.DoubleToInt64Bits(p.X));
            result.Add(BitConverter.DoubleToInt64Bits(p.Y));result.Add(BitConverter.DoubleToInt64Bits(p.Z));
            var neighbours=((IEnumerable)Field(node,"Neighbours")).Cast<int>().ToArray();result.Add(neighbours.Length);
            foreach(int ordinal in neighbours)result.Add(ordinal);
        }
        return result.ToArray();
    }
}
