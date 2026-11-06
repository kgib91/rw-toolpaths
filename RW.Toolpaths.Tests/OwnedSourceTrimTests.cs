using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class OwnedSourceTrimTests
{
    private static readonly VCarveCutterProfile Profile=new(
        [new(CarvingCutterSectionKind.Linear,.1,1.1,2)],2);
    private static PointD[] Box(double x,double y,double z,double w)=>[new(x,y),new(z,y),new(z,w),new(x,w)];
    private static MillingBoundaryAccess.Resolved Access()=>new MillingBoundaryAccess([],Box(-12,-12,12,12))
        .Resolve([new MillingRegion(Box(-10,-10,10,10),[Box(-2,-2,2,2).Reverse().ToArray()])],
            new(.6){VCarveProfile=Profile},new(2,2),true,50,0,default);
    private static TaggedToolpath[] Sources(double surface)
    {
        var plan=new VCarveRingOrder.Plan([-1]);
        TaggedToolpath Path(int visit,Point3D[] points)=>new(points,
            [new(0,points.Length/2,ToolpathSpanKind.Cut),new(points.Length/2,points.Length-1,ToolpathSpanKind.Link)],
            3,"contour",2,visit,detectCurves:false){ConcentricSource=new(plan,0)};
        return [Path(1,Enumerable.Range(0,81).Select(i=>new Point3D(-8+i*.1,-7,surface-.6)).ToArray()),
            Path(2,[new(-8,0,surface-.5),new(-1,0,surface-.6),new(1,0,surface-.7),new(8,0,surface-.8)]),
            Path(3,[new(-14,7,surface-.5),new(-8,7,surface-.5),new(8,7,surface-.5),new(14,7,surface-.5)])];
    }

    [Theory]
    [InlineData(0d,false)] [InlineData(0d,true)]
    [InlineData(12.7,false)] [InlineData(12.7,true)]
    [InlineData(-12.7,false)] [InlineData(-12.7,true)]
    public void PhysicalProfileRetainsExactSourceFragmentsWholeChordCustodyAndPlatformRepairs(double surface,bool retain)
    {
        var access=Access();var source=Sources(surface);var snapshots=source.Select(p=>Bits(p.Points)).ToArray();
        var oldInserted=new List<int>();var newInserted=new List<int>();var logs=new List<string>();
        var expected=access.Trim(source,Profile.PhysicalRadiusAtDepth,default,surface:surface,
            retainContactingChords:retain,interpolatedFragments:oldInserted);
        var actual=Mode(true,()=>access.TrimOwnedPhysicalProfile(source,Profile,default,surface,retain,newInserted,logs.Add));
        EqualPaths(expected,actual);Assert.Equal(oldInserted,newInserted);
        Assert.NotEmpty(newInserted); // Platform crossings still demand fresh protection.
        Assert.Contains(logs,m=>m.Contains("stage=owned-profile-trim")&&m.Contains("radiusOwner=immutable-physical-profile")
            &&!m.Contains("interiorShortcutHits=0 "));
        EqualPaths(expected,Mode(false,()=>access.TrimOwnedPhysicalProfile(source,Profile,default,surface,retain)));
        for(int i=0;i<source.Length;i++)Assert.Equal(snapshots[i],Bits(source[i].Points));
        foreach(var path in actual)Assert.Same(source[0].ConcentricSource!.Plan,path.ConcentricSource!.Plan);
    }

    [Theory]
    [InlineData(0d)] [InlineData(12.7)] [InlineData(-12.7)]
    public void FlatRadiusRetainsLiteralIntervalsAndEnumeratesTheOwnedSourceOnce(double surface)
    {
        var access=Access();var source=Sources(surface);int enumerations=0;
        IEnumerable<TaggedToolpath> Owned(){enumerations++;foreach(var path in source)yield return path;}
        var expected=access.Trim(source,_=>.6,default,surface:surface);var logs=new List<string>();
        var actual=Mode(true,()=>access.TrimOwnedFlatRadius(Owned(),.6,default,surface,logs.Add));
        Assert.Equal(1,enumerations);EqualPaths(expected,actual);
        Assert.Contains(logs,m=>m.Contains("stage=owned-flat-trim")&&m.Contains("radiusOwner=fixed-flat-radius")
            &&!m.Contains("interiorShortcutHits=0 "));
        EqualPaths(expected,Mode(false,()=>access.TrimOwnedFlatRadius(source,.6,default,surface)));
    }

    [Fact]
    public void OpaqueRadiusDelegateKeepsItsLiteralCallbackAndEnumerationOrder()
    {
        var access=Access();var source=Sources(0);
        (List<TaggedToolpath> Paths,string[] Calls) Execute(bool ownedControl)
        {
            var calls=new List<string>();int radii=0;
            IEnumerable<TaggedToolpath> Opaque(){foreach(var path in source){calls.Add($"path:{path.VisitIdentity}");yield return path;}}
            double Radius(double depth){calls.Add($"radius:{BitConverter.DoubleToInt64Bits(depth)}");return .6+ ++radii*.00001;}
            return (Mode(ownedControl,()=>access.Trim(Opaque(),Radius,default)),calls.ToArray());
        }
        var literal=Execute(false);var actual=Execute(true);
        Assert.Equal(literal.Calls,actual.Calls);EqualPaths(literal.Paths,actual.Paths);
        Assert.Equal(source.Sum(p=>p.Points.Count-1),actual.Calls.Count(s=>s.StartsWith("radius:")));
    }

    [Fact]
    public void OwnedAdaptersHonorCancellationBeforeSourceEnumeration()
    {
        var access=Access();using var cancellation=new CancellationTokenSource();cancellation.Cancel();int enumerations=0;
        IEnumerable<TaggedToolpath> Source(){enumerations++;foreach(var path in Sources(0))yield return path;}
        Assert.Throws<OperationCanceledException>(()=>access.TrimOwnedPhysicalProfile(Source(),Profile,cancellation.Token));
        Assert.Throws<OperationCanceledException>(()=>access.TrimOwnedFlatRadius(Source(),.6,cancellation.Token));
        Assert.Equal(0,enumerations);
    }

    private static T Mode<T>(bool enabled,Func<T> work)
    {
        var old=MillingBoundaryAccess.Resolved.ContourInteriorFirstEnabled.Value;
        try{MillingBoundaryAccess.Resolved.ContourInteriorFirstEnabled.Value=enabled;return work();}
        finally{MillingBoundaryAccess.Resolved.ContourInteriorFirstEnabled.Value=old;}
    }
    private static long[] Bits(IReadOnlyList<Point3D> points)=>points.SelectMany(p=>new[]{
        BitConverter.DoubleToInt64Bits(p.X),BitConverter.DoubleToInt64Bits(p.Y),BitConverter.DoubleToInt64Bits(p.Z)}).ToArray();
    private static void EqualPaths(IReadOnlyList<TaggedToolpath> expected,IReadOnlyList<TaggedToolpath> actual)
    {
        Assert.Equal(expected.Count,actual.Count);
        for(int i=0;i<expected.Count;i++)
        {
            Assert.Equal(Bits(expected[i].Points),Bits(actual[i].Points));Assert.Equal(expected[i].Spans,actual[i].Spans);
            Assert.Equal(expected[i].RegionIndex,actual[i].RegionIndex);Assert.Equal(expected[i].Category,actual[i].Category);
            Assert.Equal(expected[i].DepthPassIndex,actual[i].DepthPassIndex);Assert.Equal(expected[i].VisitIdentity,actual[i].VisitIdentity);
            Assert.Equal(expected[i].DetectCurves,actual[i].DetectCurves);Assert.Same(expected[i].ConcentricSource,actual[i].ConcentricSource);
        }
    }
}
