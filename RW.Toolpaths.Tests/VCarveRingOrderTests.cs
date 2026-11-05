using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class VCarveRingOrderTests
{
    [Fact]
    public void DifferentWallsSplitFromOneOriginalSourceKeepTheirOwnDepthPassJobs()
    {
        var plan=new VCarveRingOrder.Plan([-1]);
        TaggedToolpath Row(int wall,int pass)=>new([new(wall*10,0,-pass-1),new(wall*10+1,0,-pass-1)],[],0,"clearing",pass,detectCurves:false)
            {ConcentricSource=new(plan,0,Wall:wall),ContinuationSourceOrdinal=7};
        var paths=new[]{Row(0,0),Row(1,0),Row(0,1),Row(1,1)};
        var order=new VCarveRingOrder(paths,false,default,.7);var actual=new List<int>();
        for(int i=0;i<4;i++){int next=order.Candidates(null)[0];actual.Add(next);order.Complete(next);}
        Assert.Equal(new[]{0,2,1,3},actual);Assert.Equal(2,order.WallCount);
    }
    [Theory]
    [InlineData(VCarveProfileDirection.TopDown)]
    [InlineData(VCarveProfileDirection.BottomUp)]
    public void RingTreeFinishesEachOutlineFamilyBeforeAnother(VCarveProfileDirection direction)
    {
        IReadOnlyList<PointD> outline=[new(0,0),new(24,0),new(24,8),new(16,8),new(16,14),new(24,14),
            new(24,22),new(0,22),new(0,14),new(8,14),new(8,8),new(0,8)];
        IReadOnlyList<PointD> outer=[new(0,0),new(160,0),new(160,70),new(0,70)];
        var paths=VCarveRampSurface.Contours([outer,
            outline.Select(p=>new PointD(p.x+20,p.y+20)).Reverse().ToArray(),
            outline.Select(p=>new PointD(p.x+100,p.y+20)).Reverse().ToArray()],
            new VCarveOptions{SmoothVCarving=true,IncludeInteriorFill=false,EdgeFillWidth=4,StepOver=.8,ProfileDirection=direction},
            new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear,0,3,3)],3),3,0,default,null);
        Assert.True(paths.Count>10);
        var families=paths.Where(p=>p.Points[0]==p.Points[^1]).Select(p=>p.Points.Max(v=>v.X)-p.Points.Min(v=>v.X)>100 ? 0
            : (p.Points.Max(v=>v.X)+p.Points.Min(v=>v.X))/2<80 ? 1 : 2).ToArray();
        Assert.Equal(3,families.Distinct().Count());
        Assert.Equal(2,Enumerable.Range(1,families.Length-1).Count(i=>families[i]!=families[i-1]));
    }

    [Fact]
    public void NarrowComponentsUseTheSamePitchAsWideComponents()
    {
        var profile=new VCarveCutterProfile([new(CarvingCutterSectionKind.Hemisphere,0,.5,.5),
            new(CarvingCutterSectionKind.Linear,.5,1.5,10)],10.5);
        var options=new VCarveOptions {SmoothVCarving=true,IncludeInteriorFill=false,EdgeFillWidth=6,StepOver=.35};
        double[] Offsets(double height)=>VCarveRampSurface.Contours(
            [new PointD[]{new(0,0),new(80,0),new(80,height),new(0,height)}],options,profile,2,0,default,null)
            .Select(p=>p.Points.Min(v=>v.X)).ToArray();
        var wide=Offsets(40);var narrow=Offsets(3);
        Assert.True(narrow.Length>2);
        Assert.True(narrow.Length<wide.Length);
        Assert.Equal(wide.Take(narrow.Length),narrow);
    }

    [Theory]
    [InlineData(VCarveProfileDirection.TopDown,1)]
    [InlineData(VCarveProfileDirection.BottomUp,1)]
    [InlineData(VCarveProfileDirection.TopDown,6)]
    [InlineData(VCarveProfileDirection.BottomUp,6)]
    public void PostedRingsHaveOneUniformPitchAndKeepDirectionThroughDepthPasses(
        VCarveProfileDirection direction,double fill)
    {
        var profile=new VCarveCutterProfile([
            new(CarvingCutterSectionKind.Hemisphere,0,.5,.5),
            new(CarvingCutterSectionKind.Linear,.5,1.5,10)],10.5);
        var sources=new List<TaggedToolpath>();
        var previous=MillingGeometryTrace.Observer.Value;
        ToolpathPlan plan;
        try
        {
            MillingGeometryTrace.Observer.Value=(stage,batch,paths)=>{
                previous?.Invoke(stage,batch,paths);
                if(stage=="stock-selected")sources.AddRange(paths.Where(p=>p.Category=="clearing"));
            };
            plan=VCarveToolpaths.Generate([new MillingRegion(new PointD[]{new(0,0),new(80,0),new(80,40),new(0,40)})],
                new ToolGeometry(1.5) {VCarveProfile=profile},new(2,.5) {SurfaceZ=12.7},
                new VCarveOptions {SmoothVCarving=true,IncludeInteriorFill=false,EdgeFillWidth=fill,
                    ProfileDirection=direction,StepOver=.35,DetectCurves=false,CalculateSweptArea=false},
                BoostVoronoiProvider.CreateDefault());
        }
        finally {MillingGeometryTrace.Observer.Value=previous;}
        Assert.True(sources.Count>2);
        // The complete slope and nearest-edge cleanup share closed source
        // rings and one uniform pitch, including the handoff at the floor.
        double[] offsets=sources.Where(p=>p.Points[0]==p.Points[^1]).Select(p=>p.Points.Min(v=>v.X)).Order().ToArray();
        double pitch=offsets[1]-offsets[0];
        Assert.InRange(pitch,0,.35/Math.Sqrt(1+Math.Pow(Math.Tan(Math.PI/6),2))+.0001);
        for(int i=1;i<offsets.Length;i++)Assert.InRange(offsets[i]-offsets[i-1],pitch-.0001,pitch+.0001);
        double reach=profile.RampFillWidth(2,30)+fill;
        Assert.InRange(offsets[^1],reach-pitch-.001,reach+.001);
        // Audit actual routed visits, including their surface entries and each
        // assigned stepdown. Medial precursors have a different XY envelope.
        var visited=new List<int>();
        foreach(var path in plan.Toolpaths.Where(p=>p.Category=="clearing"))
        {
            double inset=path.Spans.Where(s=>s.Kind==ToolpathSpanKind.Cut)
                .SelectMany(s=>path.Points.Skip(s.StartIndex).Take(s.PointCount)).Select(p=>p.X)
                .DefaultIfEmpty(double.PositiveInfinity).Min();
            int rank=Array.FindIndex(offsets,d=>Math.Abs(d-inset)<.001);
            if(rank<0)continue;
            if(visited.Count==0||visited[^1]!=rank)visited.Add(rank);
        }
        int[] expected=Enumerable.Range(0,offsets.Length).ToArray();
        if(direction==VCarveProfileDirection.BottomUp)Array.Reverse(expected);
        Assert.Equal(expected,visited);
        int links=plan.Toolpaths.Count(p=>p.Spans.Any(s=>s.Kind==ToolpathSpanKind.Link));
        int airEntries=plan.Toolpaths.Select((p,i)=>i==0||plan.Toolpaths[i-1].Points[^1]!=p.Points[0]).Count(p=>p);
        if(fill==6){Assert.True(links>0);Assert.True(airEntries<plan.Toolpaths.Count);}
        Console.WriteLine($"[VCarve.RingOrder.Test] direction={direction} fill={fill:R} rings={offsets.Length} "
            +$"pitch={pitch:R} routedVisits={plan.Toolpaths.Count} links={links} airEntries={airEntries} completeRingBeforeNext=True oneSpacingOwner=True");
    }
}
