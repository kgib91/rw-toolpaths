using System.Collections;
using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class RampWallPreservationProofTests
{
    static PointD[] Box(double x,double y,double z,double w)=>[new(x,y),new(z,y),new(z,w),new(x,w)];

    [Theory]
    [InlineData(false,false)] [InlineData(false,true)]
    [InlineData(true,false)] [InlineData(true,true)]
    public void ProofOnlyMatchesOriginalOutProofAndAvoidsTheDiscardedRetainedClip(bool outsideWindow,bool retained)
    {
        var part=Box(0,0,20,20);var target=Box(5,5,15,15);var clearance=Box(0,5,5,15);
        var island=Box(7,7,10,10);
        var region=!outsideWindow&&retained?new MillingRegion(target,[island]):new MillingRegion(target);
        var access=new MillingBoundaryAccess([new MillingRegion(clearance)],part)
            .Resolve([region],ToolGeometry.Flat(1),new(3,1),true,30,0,default);
        var boundary=new List<IReadOnlyList<PointD>>();
        if(outsideWindow)boundary.Add(Box(-100,-100,100,100));
        if(!outsideWindow||!retained)boundary.Add(retained?island:target);
        var snapshot=boundary.Select(r=>r.ToArray()).ToArray();
        var original=access.RetainedRampEdges(boundary,default,out bool oldProof);
        Assert.Equal(2,access.RampWallClipOperations);
        Assert.Equal(retained,oldProof);
        bool proof=access.RampWallsPreserved(boundary,default);
        Assert.Equal(oldProof,proof);Assert.Equal(3,access.RampWallClipOperations);
        // A caller whose complement was nonempty requests only retained
        // geometry; the complementary query is never repeated on that path.
        if(!proof)
        {
            var fallback=access.RetainedRampEdges(boundary,default);
            Assert.Equal(original,fallback);Assert.Equal(4,access.RampWallClipOperations);
        }
        else Assert.Equal(3,access.RampWallClipOperations);
        for(int i=0;i<boundary.Count;i++)Assert.Equal(snapshot[i],boundary[i]);
    }

    sealed class SparseProvider:IMedialAxisProvider
    {
        public IReadOnlyList<MedialSegment> ConstructMedialAxis(IReadOnlyList<PointD> boundary,
            IReadOnlyList<IReadOnlyList<PointD>> holes,double tolerance,double maxRadius,
            double filteringAngle=Math.PI,bool useBigIntegers=true)
        {
            double width=boundary.Max(p=>p.x)-boundary.Min(p=>p.x);
            return[new(new(0,0,0),new(width*.1,0,0))];
        }
    }
    static T Mode<T>(bool enabled,Func<T> work)
    {
        var old=VCarvePreparation.RampWallProofOnlyEnabled.Value;
        try{VCarvePreparation.RampWallProofOnlyEnabled.Value=enabled;return work();}
        finally{VCarvePreparation.RampWallProofOnlyEnabled.Value=old;}
    }

    [Theory]
    [InlineData(false,30)] [InlineData(true,30)]
    [InlineData(false,45)] [InlineData(true,45)]
    public void AnalyticalReuseAndFallbackKeepOriginalGraphDepthsMasksAndChangedPolicy(bool openWall,double angle)
    {
        var target=Box(5,5,15,15);var part=Box(0,0,20,20);
        var region=new MillingRegion(target,[Box(7,7,8,8)]);
        var profile=new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear,0,2,4)],2);
        var tool=new ToolGeometry(2){VCarveProfile=profile};var depth=new DepthSchedule(2,1);
        var options=new VCarveOptions{SmoothVCarving=true,SmoothRampAngleDegrees=30,IncludeInteriorFill=false,
            EdgeFillWidth=1,StockToLeave=0,RampOutsideEdges=false,DetectCurves=false,CalculateSweptArea=false};
        var prepared=VCarveToolpaths.Prepare([region],tool,depth,options,new SparseProvider());
        var access=new MillingBoundaryAccess(openWall?[new MillingRegion(Box(0,5,5,15))]:[],part);
        var expected=Mode(false,()=>prepared.AnalyzePocketPlateau(1,angle,outsideAccess:access,rampOutsideEdges:false));
        var actual=Mode(true,()=>prepared.AnalyzePocketPlateau(1,angle,outsideAccess:access,rampOutsideEdges:false));
        Assert.Equal(expected.ProtectedArea,actual.ProtectedArea);
        Assert.Equal(expected.Basins,actual.Basins);
        Assert.Equal(expected.ProtectedSourceRegionIndices.Order(),actual.ProtectedSourceRegionIndices.Order());
        Assert.Equal(expected.EntireProtectedSourceRegionIndices.Order(),actual.EntireProtectedSourceRegionIndices.Order());
        Assert.Equal(expected.ProtectedBoundary.Count,actual.ProtectedBoundary.Count);
        for(int i=0;i<expected.ProtectedBoundary.Count;i++)Assert.Equal(expected.ProtectedBoundary[i],actual.ProtectedBoundary[i]);
        var first=Assert.Single(expected.Preparation.Regions);var second=Assert.Single(actual.Preparation.Regions);
        Assert.Equal(first.OriginalMedialGraph,second.OriginalMedialGraph);
        Assert.Equal(first.MaximumChordLength,second.MaximumChordLength);
        Assert.Equal(first.Boundary,second.Boundary);
        Assert.Equal(first.OriginalMedialPaths.Count,second.OriginalMedialPaths.Count);
        for(int i=0;i<first.OriginalMedialPaths.Count;i++)
        {
            Assert.Equal(first.SourceClearancePaths[i],second.SourceClearancePaths[i]);
            Assert.Equal(first.OriginalMedialPaths[i].Points,second.OriginalMedialPaths[i].Points);
            Assert.Equal(first.SmoothedMedialPaths[i].Points,second.SmoothedMedialPaths[i].Points);
            Assert.Equal(first.UpperMedialPaths[i].Points,second.UpperMedialPaths[i].Points);
        }
        if(!openWall&&angle==30)Assert.Same(prepared.Regions[0],second);
        else Assert.NotSame(prepared.Regions[0],second);
    }

    [Fact]
    public void PreCancellationAndCancellationDuringSourceReadCannotPublishAReuseCertificate()
    {
        var part=Box(0,0,20,20);var target=Box(5,5,15,15);
        var access=new MillingBoundaryAccess([],part).Resolve([new MillingRegion(target)],ToolGeometry.Flat(1),new(3,1),true,30,0,default);
        using var before=new CancellationTokenSource();before.Cancel();int reads=0;
        IReadOnlyList<IReadOnlyList<PointD>> untouched=[new ReadObservedRing(target,()=>reads++)];
        Assert.ThrowsAny<OperationCanceledException>(()=>access.RampWallsPreserved(untouched,before.Token));
        Assert.Equal(0,reads);Assert.Equal(0,access.RampWallClipOperations);
        using var during=new CancellationTokenSource();
        IReadOnlyList<IReadOnlyList<PointD>> interrupted=[new ReadObservedRing(target,()=>{reads++;during.Cancel();})];
        Assert.ThrowsAny<OperationCanceledException>(()=>access.RampWallsPreserved(interrupted,during.Token));
        Assert.True(reads>0);Assert.Equal(0,access.RampWallClipOperations);
    }

    sealed class ReadObservedRing(IReadOnlyList<PointD> points,Action read):IReadOnlyList<PointD>
    {
        public int Count=>points.Count;
        public PointD this[int index]{get{read();return points[index];}}
        public IEnumerator<PointD> GetEnumerator(){foreach(var point in points){read();yield return point;}}
        IEnumerator IEnumerable.GetEnumerator()=>GetEnumerator();
    }
}
