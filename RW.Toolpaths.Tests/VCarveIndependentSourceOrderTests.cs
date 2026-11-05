using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class VCarveIndependentSourceOrderTests
{
    private static readonly VCarveCutterProfile Profile=new([new(CarvingCutterSectionKind.Linear,0,3,6)],3);
    private static TaggedToolpath Source(int owner,double x,int pass=0,int component=7)
        =>new([new Point3D(x,0,-1-pass),new(x+1,0,-1.1-pass)],[],0,"clearing",pass,detectCurves:false)
        {ContinuationSourceOrdinal=owner,ContinuationComponentIndex=component};
    private static (List<TaggedToolpath> Paths,List<string> Logs) Route(TaggedToolpath[] source,bool? enabled,
        Point3D? start=null)
    {
        var previous=VCarvePathRouter.SpatialIndependentSourcesOverride.Value;
        VCarvePathRouter.SpatialIndependentSourcesOverride.Value=enabled;
        try{
            var logs=new List<string>();var result=VCarvePathRouter.Route(source,Array.Empty<IReadOnlyList<PointD>>(),
                new ToolGeometry(3){VCarveProfile=Profile},new(3,1),new(){SmoothVCarving=true,DetectCurves=false,
                    SmoothRampAngleDegrees=50,StepOver=.45,Motion=new(1000,500,5000,5),Ramp=new(Math.Atan(.5))},
                0,start??new Point3D(0,0,-1),diagnostic:logs.Add,continuationCutGuard:(_,_)=>false,
                materialEntryHeight:p=>p.Z);
            return(result,logs);
        }finally{VCarvePathRouter.SpatialIndependentSourcesOverride.Value=previous;}
    }
    private static double Transfers(IReadOnlyList<TaggedToolpath> paths,Point3D from)
    {
        double length=0;foreach(var p in paths){length+=double.Hypot(p.Points[0].X-from.X,p.Points[0].Y-from.Y);from=p.Points[^1];}
        return length;
    }
    private static void SameSources(TaggedToolpath[] original,IReadOnlyList<TaggedToolpath> actual)
    {
        Assert.Equal(original.Length,actual.Count);
        foreach(var p in original){var match=Assert.Single(actual.Where(q=>q.ContinuationSourceOrdinal==p.ContinuationSourceOrdinal));
            var kinds=ToolpathSpanKinds.Expand(match);
            for(int edge=0;edge<p.Points.Count-1;edge++)Assert.Contains(Enumerable.Range(0,kinds.Length),i=>
                kinds[i]==ToolpathSpanKind.Cut&&match.Points[i]==p.Points[edge]&&match.Points[i+1]==p.Points[edge+1]);
            Assert.Equal(p.DepthPassIndex,match.DepthPassIndex);Assert.Equal(p.ContinuationComponentIndex,match.ContinuationComponentIndex);}
    }

    [Fact]
    public void UnrelatedSmoothClearingUsesExistingSpatialOrderWithoutChangingAnySourceXYZ()
    {
        var source=new[]{Source(2,20),Source(0,0),Source(1,10)};var before=Route(source,false);var after=Route(source,true);
        Assert.Equal(new int?[]{2,0,1},before.Paths.Select(p=>p.ContinuationSourceOrdinal));
        Assert.Equal(new int?[]{0,1,2},after.Paths.Select(p=>p.ContinuationSourceOrdinal));
        Assert.True(Transfers(after.Paths,new(0,0,-1))<Transfers(before.Paths,new(0,0,-1)));
        foreach(var p in source)Assert.Equal(p.Points,after.Paths.Single(q=>q.ContinuationSourceOrdinal==p.ContinuationSourceOrdinal).Points);
        SameSources(source,after.Paths);Assert.Contains(after.Logs,l=>l.Contains("independentClearingOrder=True"));
    }

    [Fact]
    public void DefaultPolicyRetainsLegacyNonconcentricOrder()
    {
        var source=new[]{Source(2,20),Source(0,0),Source(1,10)};
        var disabled=Route(source,false);var normal=Route(source,null);
        Assert.Equal(disabled.Paths.SelectMany(p=>p.Points),normal.Paths.SelectMany(p=>p.Points));
        Assert.Equal(disabled.Paths.Select(p=>p.ContinuationSourceOrdinal),normal.Paths.Select(p=>p.ContinuationSourceOrdinal));
    }

    [Fact]
    public void SpatialOrderingCannotCrossExistingComponentOrDepthPassGroups()
    {
        var source=new[]{Source(2,20,0,7),Source(0,0,0,7),Source(3,21,1,7),Source(1,1,1,7),
            Source(5,22,0,9),Source(4,2,0,9)};
        var result=Route(source,true);SameSources(source,result.Paths);
        Assert.Equal(new int?[]{0,0,1,1,0,0},result.Paths.Select(p=>p.DepthPassIndex));
        Assert.Equal(new int?[]{7,7,7,7,9,9},result.Paths.Select(p=>p.ContinuationComponentIndex));
    }

    [Fact]
    public void ActualConcentricRowsKeepTheirOriginalOrderUnderTheOverride()
    {
        var plan=new VCarveRingOrder.Plan([-1,0]);
        var source=new[]{Source(2,20) with{ConcentricSource=new(plan,0)},Source(0,0) with{ConcentricSource=new(plan,1)}};
        // Omitting a cut guard exercises the ordinary core's refusal directly.
        var previous=VCarvePathRouter.SpatialIndependentSourcesOverride.Value;
        try{VCarvePathRouter.SpatialIndependentSourcesOverride.Value=true;
            var result=VCarvePathRouter.Route(source,Array.Empty<IReadOnlyList<PointD>>(),new ToolGeometry(3){VCarveProfile=Profile},
                new(3,1),new(){SmoothVCarving=true,DetectCurves=false,SmoothRampAngleDegrees=50,Motion=new(1000,500,5000,5)},
                0,new(0,0,-1),materialEntryHeight:p=>p.Z);
            Assert.Equal(new int?[]{2,0},result.Select(p=>p.ContinuationSourceOrdinal));
        }finally{VCarvePathRouter.SpatialIndependentSourcesOverride.Value=previous;}
    }

    [Fact]
    public void FixedSeamTypedCircuitGroupKeepsTheCompleteLegacyTourAndOrder()
    {
        var points=new Point3D[]{new(20,0,-1),new(22,0,-1.2),new(20,2,-1.1),new(20,0,-1)};
        var definition=ToolpathCircuitBuilder.Definition.FromGraph([points[0]],
            [new(0,0,points,1.2,7)], [new List<int>{0,0}],0,default);
        var typed=new TaggedToolpath(points,[],0,"clearing",0,detectCurves:false){ContinuationSourceOrdinal=2,
            ContinuationComponentIndex=7,ContinuationCircuit=true,ContinuationCircuitDefinition=definition};
        var source=new[]{typed,Source(0,0)};var before=Route(source,false);var after=Route(source,true);
        Assert.Equal(before.Paths.Select(p=>p.ContinuationSourceOrdinal),after.Paths.Select(p=>p.ContinuationSourceOrdinal));
        Assert.Equal(before.Paths.SelectMany(p=>p.Points),after.Paths.SelectMany(p=>p.Points));
        Assert.Contains(after.Logs,l=>l.Contains("independentClearingOrder=False")&&l.Contains("fixedTypedCircuitOrderRetained=True"));
        foreach(var p in points)Assert.Contains(p,after.Paths.Single(q=>q.ContinuationSourceOrdinal==2).Points);
    }
}
