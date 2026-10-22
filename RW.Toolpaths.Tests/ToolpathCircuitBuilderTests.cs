using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class ToolpathCircuitBuilderTests
{
    [Fact]
    public void SubdividingASteepCircuitChordDoesNotAddAFullCircuitReturn()
    {
        Point3D a=new(0,0,-.02),b=new(.004,0,-.012);
        double Length(IEnumerable<TaggedToolpath> paths)=>paths.Sum(p=>Enumerable.Range(1,p.Points.Count-1).Sum(i=>
            ToolpathContinuationPlanner.Distance(p.Points[i-1],p.Points[i])));
        List<TaggedToolpath> Run(int pieces)
        {
            var front=Enumerable.Range(0,pieces+1).Select(i=>new Point3D(a.X+(b.X-a.X)*i/pieces,0,a.Z+(b.Z-a.Z)*i/pieces));
            var source=new[]{Cut(front.Concat(new Point3D[]{new(10,0,-.02),new(10,10,-.02),new(0,10,-.02),a}).ToArray()),
                Cut(new(10.1,0,-.02),new(14,0,-.02),new(14,4,-.02),new(10.1,4,-.02),new(10.1,0,-.02))};
            var circuits=ToolpathCircuitBuilder.Build(source,(_,_)=>false,.2,default);
            var log=new List<string>();
            var output=ToolpathCircuitExecutor.Execute(circuits,.001,.2,new(.02,.02),new(Math.Atan(.5)),
                Array.Empty<(double,double)>(),(x,y)=>x.Z>=-.001&&y.Z>=-.001,default,log.Add,2.01);
            Assert.Contains(log,l=>l.Contains("partialAxisFallbacks=0 "));
            foreach(var required in source.SelectMany(p=>p.Points))Assert.Contains(output,p=>p.Points.Contains(required));
            for(int i=1;i<output.Count;i++)Assert.Equal(output[i-1].Points[^1],output[i].Points[0]);
            return output;
        }
        var coarse=Run(1);var fine=Run(5);
        // Entry contact is snapped to a source vertex, so refinement may change
        // the outward/return contact by at most the original short chord twice.
        // It must never add another traversal of either complete circuit.
        double contactBound=2*ToolpathContinuationPlanner.Distance(a,b);
        Assert.InRange(Math.Abs(Length(coarse)-Length(fine)),0,contactBound);
    }

    private static TaggedToolpath Cut(params Point3D[] p) => new(p, [], 0, "clearing", null, detectCurves:false);

    [Fact]
    public void BothEndsOfALongOpenCutCanUseCarrierPortalsInsteadOfDoublingTheCut()
    {
        var carrier=Cut(new(0,0,-1),new(10,0,-1),new(10,10,-1),new(0,10,-1),new(0,0,-1));
        var stroke=Cut(new(.1,0,-1),new(9,0,-1),new(9,9,-1),new(.1,10,-1));
        var route=Assert.Single(ToolpathCircuitBuilder.Build(new[]{carrier,stroke},(_,_)=>true,.2,default));
        double Length(IReadOnlyList<Point3D> points)=>Enumerable.Range(1,points.Count-1)
            .Sum(i=>ToolpathContinuationPlanner.Distance(points[i-1],points[i]));
        Assert.True(Length(route.Points)<Length(carrier.Points)+2*Length(stroke.Points)-10);
    }

    [Fact]
    public void CarrierPausesAtBodyPortalAndRetainsEveryDirectedCut()
    {
        var carrier=Cut(new(0,0,-2),new(10,0,-2),new(20,0,-2),new(20,10,-2),new(0,10,-2),new(0,0,-2));
        var child=Cut(new(10,.1,-2),new(10,4,-2));
        var route=Assert.Single(ToolpathCircuitBuilder.Build([carrier,child],(_,_)=>true,.2,default));
        Assert.Equal(route.Points[0],route.Points[^1]);
        foreach(var source in new[]{carrier,child})
        for(int i=1;i<source.Points.Count;i++)
            Assert.Contains(Enumerable.Range(1,route.Points.Count-1),j=>route.Points[j-1]==source.Points[i-1]&&route.Points[j]==source.Points[i]);
        int portal=route.Points.ToList().IndexOf(new(10,0,-2));
        Assert.Equal(new Point3D(10,.1,-2),route.Points[portal+1]);
        Assert.Equal(new Point3D(10,4,-2),route.Points[portal+2]);
        Assert.Equal(new Point3D(10,.1,-2),route.Points[portal+3]);
        Assert.Equal(new Point3D(10,0,-2),route.Points[portal+4]);
    }

    [Fact]
    public void AllDepthsOfConnectedStrokesUseOneSurfaceEntryAndNoGraphSearch()
    {
        var routes=ToolpathCircuitBuilder.Build([Cut(new(0,0,-3),new(10,0,-3)),
            Cut(new(10,.1,-3),new(10,10,-3))],(_,_)=>true,.2,default);
        var scheduled=VCarveToolpaths.ScheduleFreshStockVisits(routes,new(3,1),default,null);
        var log=new List<string>();
        var result=VCarvePathRouter.Route(scheduled,[],new ToolGeometry(1),new(3,1),
            new VCarveOptions{SmoothVCarving=true},0,null,diagnostic:log.Add);
        Assert.Equal(3,result.Count);
        Assert.Equal(0,result[0].Points[0].Z);
        for(int i=1;i<result.Count;i++)Assert.Equal(result[i-1].Points[^1],result[i].Points[0]);
        Assert.Contains(log,l=>l.Contains("surfaceEntries=1 ")&&l.Contains("continuousDepthPasses=2 ")&&l.Contains("emittedGraphSearch=False"));
        Assert.All(result.SelectMany(p=>p.Points),p=>Assert.InRange(p.Z,-3,0));
    }

    [Fact]
    public void UnknownPhysicalConnectionKeepsSeparateSafeEntries()
    {
        var routes=ToolpathCircuitBuilder.Build([Cut(new(0,0,-1),new(2,0,-1)),
            Cut(new(2,.1,-1),new(2,2,-1))],(_,_)=>false,.2,default);
        Assert.Equal(2,routes.Count);
        Assert.NotEqual(routes[0].ContinuationComponentIndex,routes[1].ContinuationComponentIndex);
    }

    [Fact]
    public void DifferentFloorComponentsShareAShallowCarrierAndReturnOverActualRamps()
    {
        var sources=new[]{Cut(new(0,0,-2),new(4,0,-2),new(4,4,-2),new(0,4,-2),new(0,0,-2)),
            Cut(new(4.1,0,-3),new(8,0,-3),new(8,4,-3),new(4.1,4,-3),new(4.1,0,-3))};
        // A shallow corridor is legal, but the unequal final floors cannot be
        // joined by a steep shortcut. Both full floors must still be executed.
        var circuits=ToolpathCircuitBuilder.Build(sources,(_,_)=>false,.2,default);
        var log=new List<string>();
        var result=ToolpathCircuitExecutor.Execute(circuits,.25,.2,new(3,1),new(Math.Atan(.5)),
            new[]{(1d,1d),(2d,2d)},(a,b)=>a.Z>=-.25&&b.Z>=-.25,default,log.Add,.5);
        Assert.Contains(log,l=>l.Contains("surfaceEntries=1 ")&&l.Contains("servicedCircuits=2 "));
        for(int i=1;i<result.Count;i++)Assert.Equal(result[i-1].Points[^1],result[i].Points[0]);
        foreach(var source in sources)
        for(int i=1;i<source.Points.Count;i++)
            Assert.Contains(result,p=>Enumerable.Range(1,p.Points.Count-1).Any(j=>
                p.Points[j-1]==source.Points[i-1]&&p.Points[j]==source.Points[i]
                ||p.Points[j]==source.Points[i-1]&&p.Points[j-1]==source.Points[i]));
        Assert.All(result.SelectMany(p=>Enumerable.Range(1,p.Points.Count-1).Select(i=>(p.Points[i-1],p.Points[i]))),
            chord=>Assert.True(Math.Abs(chord.Item2.Z-chord.Item1.Z)<=.5*Math.Sqrt(
                Math.Pow(chord.Item2.X-chord.Item1.X,2)+Math.Pow(chord.Item2.Y-chord.Item1.Y,2))+1e-8));
    }

    [Fact]
    public void CompletedSweepClearanceRequiresAWholeLiteralChordAndPhysicalRoom()
    {
        var profile=new VCarveCutterProfile(new[]{new CarvingCutterSection(CarvingCutterSectionKind.Hemisphere,0,1,1),
            new CarvingCutterSection(CarvingCutterSectionKind.Linear,1,1,4)},4);
        var deep=Cut(new(0,0,-2),new(10,0,-2));
        var stock=new CompletedCircuitStock(new[]{deep},profile,p=>p,default);
        Assert.True(stock.ContainsRaisedChord(0,new(0,0,-.001),new(10,0,-.001),.1));
        Assert.False(stock.ContainsRaisedChord(0,new(0,0,-1.9),new(10,0,-1.9),.1));
        Assert.False(stock.ContainsRaisedChord(0,new(0,.001,-.001),new(10,.001,-.001),.1));
        // Two deep endpoints from different chords cannot synthesize a column
        // along the intervening shallow sweep.
        var valley=Cut(new(0,0,-2),new(5,0,-.01),new(10,0,-2));
        var incomplete=new CompletedCircuitStock(new[]{valley},profile,p=>p,default);
        Assert.False(incomplete.ContainsRaisedChord(0,new(0,0,-.001),new(10,0,-.001),.1));
    }

    [Fact]
    public void OriginalWallProofFollowsRaisedCommandedChordsIntoTheFinalAudit()
    {
        var profile=new VCarveCutterProfile(new[]{new CarvingCutterSection(CarvingCutterSectionKind.Hemisphere,0,1,1),
            new CarvingCutterSection(CarvingCutterSectionKind.Linear,1,2,5)},6);
        var tool=new ToolGeometry(2){VCarveProfile=profile,BottomRadius=1,TopRadius=2,ConeLength=5};
        var options=new VCarveOptions{SmoothVCarving=true,EdgeFillWidth=.1,IncludeInteriorFill=false};
        options=options with{InitialStrategy=VCarveInitialStrategy.Capture(options,profile,3)};
        var clearing=Cut(new(5,5,-3),new(15,5,-3),new(15,15,-3),new(5,15,-3),new(5,5,-3));
        var medial=Cut(new(7,10,0),new(9,10,-3),new(13,10,-3)) with{Category="final-carve"};
        var log=new List<string>();
        var result=VCarveSmoothing.Apply(new[]{clearing,medial},
            new[]{(IReadOnlyList<PointD>)new PointD[]{new(0,0),new(20,0),new(20,20),new(0,20)}},
            tool,new(3,1),options,default,log.Add);
        Assert.NotEmpty(result);
        var custody=Assert.Single(log.Where(l=>l.Contains("stage=protected-wall-custody event=complete")));
        Assert.DoesNotContain("reusedChords=0 ",custody);
        Assert.Contains("literalChords=0 ",custody);
        Assert.Contains("actualCommandsRechecked=True",custody);
        Assert.Contains("monotoneRaisedSameXYOnly=True",custody);
    }
}
