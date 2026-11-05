using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class VCarveSmoothRoutingTests
{
    [Theory]
    [InlineData(0,true)]
    [InlineData(12.7,true)]
    [InlineData(12.7,false)]
    public void BranchingConcentricFamilyUsesBoundedConnectorAndRemainingRingRamp(double surface,bool clear)
    {
        IReadOnlyList<PointD> outline=[new(0,0),new(24,0),new(24,8),new(16,8),new(16,14),new(24,14),
            new(24,22),new(0,22),new(0,14),new(8,14),new(8,8),new(0,8)];
        var rings=MedialAxisGeometry.BuildConcentricFill([new MillingRegion(outline)],
            new RingOffsetOptions(1,.5){RingLimit=9,MaximumInset=4,MinimumRingArea=0},0,
            preserveTerminalPaths:false);
        var source=rings.Select(r=>{
            var points=r.Points.Select(p=>new Point3D(p.x,p.y,surface-.3-.3*r.Level)).ToList();
            if(points[0]!=points[^1])points.Add(points[0]);
            return new TaggedToolpath(points,[],0,"clearing",0,detectCurves:false){ContinuationComponentIndex=0};
        }).ToArray();
        Assert.True(source.Length>9); // The neck collapses into separate child rings.
        var log=new List<string>();
        var routed=VCarvePathRouter.Route(source,[outline],
            new ToolGeometry(3){TipAngleRadians=Math.PI/2,BottomRadius=0,TopRadius=3,ConeLength=3},
            new DepthSchedule(3,3){SurfaceZ=surface},new VCarveOptions{SmoothVCarving=true,StepOver=.8,
                Ramp=RampSettings.FromRatio(4),SmoothRampAngleDegrees=50,DetectCurves=false,
                CompactCollinearCommands=true},0,null,diagnostic:log.Add,continuationCutGuard:(_,_)=>clear);
        Assert.Equal(source.Length,routed.Count);
        int ramps=0;
        for(int i=0;i<routed.Count;i++)
        {
            var path=routed[i];
            RampEntryStockTests.AssertFullDirectedSourceChords(source[i].Points,
                new RampedPath(path.Points.ToList(),path.Spans.ToList(),RampStrategy.Linear,0));
            if(i==0)continue;
            if(!clear){Assert.Equal(surface,path.Points[0].Z);continue;}
            Assert.Equal(routed[i-1].Points[^1],path.Points[0]);
            ramps+=path.Spans.Count(s=>s.Kind is ToolpathSpanKind.Ramp or ToolpathSpanKind.Link
                &&path.Points[s.EndIndex].Z<path.Points[s.StartIndex].Z-1e-9);
            foreach(var span in path.Spans.Where(s=>s.Kind==ToolpathSpanKind.Link))
            for(int j=span.StartIndex+1;j<=span.EndIndex;j++)
            {
                var a=path.Points[j-1];var b=path.Points[j];
                Assert.True(a.Z-b.Z<=.25*Math.Sqrt(Math.Pow(a.X-b.X,2)+Math.Pow(a.Y-b.Y,2))+1e-9);
            }
        }
        if(clear)Assert.True(ramps>=4,$"Only {ramps} descending transitions for {rings.Count} rings.");
        Console.WriteLine($"[VCarve.BranchRamps.Test] surface={surface:R} clear={clear} rings={rings.Count} coordinatedDescents={ramps}");
    }

    [Theory]
    [InlineData(0,false)]
    [InlineData(12.7,false)]
    [InlineData(0,true)]
    [InlineData(12.7,true)]
    public void ClippedRingReturnFinishesUnderRampWithoutForwardReplayOrRetract(double surface,bool reverseNext)
    {
        TaggedToolpath Stroke(int component,params Point3D[] points)=>new(points.Select(p=>p with{Z=p.Z+surface}).ToArray(),
            Array.Empty<ToolpathSpan>(),0,"clearing",0,detectCurves:false){ContinuationComponentIndex=component};
        var first=Stroke(0,new(0,0,-1),new(4,0,-1));
        Point3D[] nextPoints=[new(1,-.3,-.001),new(1.1,-.3,-.002)];
        var next=Stroke(1,reverseNext?nextPoints.Reverse().ToArray():nextPoints);
        var routed=VCarvePathRouter.Route([first,next],[],
            new ToolGeometry(5){TipAngleRadians=Math.PI/2,BottomRadius=0,TopRadius=5,ConeLength=5},
            new DepthSchedule(2,2){SurfaceZ=surface},new VCarveOptions{SmoothVCarving=true,StepOver=.2,
                Ramp=new(Math.Atan(.5)),SmoothRampAngleDegrees=50,DetectCurves=false},
            0,null,continuationCutGuard:(a,b)=>!reverseNext||a.Y==b.Y||Math.Max(a.X,b.X)<=1.05);
        Assert.Equal(first.Points[0],routed[0].Points[^1]);
        var cut=Assert.Single(routed[0].Spans.Where(s=>s.Kind==ToolpathSpanKind.Cut));
        Assert.Equal(first.Points.Reverse(),routed[0].Points.Skip(cut.StartIndex));
        Assert.Equal(routed[0].Points[^1],routed[1].Points[0]);
        Assert.Contains(routed[1].Spans,s=>s.Kind==ToolpathSpanKind.Link&&routed[1].Points[s.EndIndex].Z>routed[1].Points[s.StartIndex].Z);
    }

    [Theory]
    [InlineData(0,true,false)]
    [InlineData(12.7,true,false)]
    [InlineData(0,false,false)]
    [InlineData(12.7,false,false)]
    [InlineData(0,true,true)]
    [InlineData(12.7,true,true)]
    [InlineData(0,false,true)]
    [InlineData(12.7,false,true)]
    public void CompletedRingExitsAtRampContactAndRampsDirectlyToNextRing(double surface,bool clear,bool descending)
    {
        TaggedToolpath Ring(int component,params Point3D[] points)=>new(points.Select(p=>new Point3D(p.X/10,p.Y/10,surface+(p.Z+(descending?(component==0?.5:-.5):0))/10)).ToArray(),
            Array.Empty<ToolpathSpan>(),0,"clearing",0,detectCurves:false){ContinuationComponentIndex=component};
        var inner=Ring(0,new(0,0,-2),new(10,0,-2),new(10,10,-2),new(0,10,-2),new(0,0,-2));
        var outer=Ring(1,new(10,-.5,-1.5),new(10.5,-.5,-1.5),new(10.5,10.5,-1.5),new(-.5,10.5,-1.5),new(-.5,-.5,-1.5),new(10,-.5,-1.5));
        var log=new List<string>();
        var routed=VCarvePathRouter.Route([inner,outer],[],
            new ToolGeometry(.5){TipAngleRadians=Math.PI/2,BottomRadius=0,TopRadius=.5,ConeLength=.5},
            new DepthSchedule(.2,.2){SurfaceZ=surface},new VCarveOptions{SmoothVCarving=true,StepOver=.2,
                Ramp=new(Math.Atan(.5)),SmoothRampAngleDegrees=50,CompactCollinearCommands=true,
                DetectCurves=false},0,null,diagnostic:log.Add,
            continuationCutGuard:(_,_)=>clear);
        Assert.Equal(2,routed.Count);
        Assert.Equal(inner.Points[1],routed[0].Points[^1]);
        RampEntryStockTests.AssertFullDirectedSourceChords(inner.Points,new RampedPath(routed[0].Points.ToList(),
            routed[0].Spans.ToList(),RampStrategy.Linear,0));
        var cut=Assert.Single(routed[0].Spans.Where(s=>s.Kind==ToolpathSpanKind.Cut));
        Assert.Equal(4,Enumerable.Range(cut.StartIndex+1,cut.MoveCount).Sum(i=>
            ToolpathContinuationPlanner.Distance(routed[0].Points[i-1],routed[0].Points[i])),8);
        if(clear)
        {
            Assert.Equal(routed[0].Points[^1],routed[1].Points[0]);
            Assert.Contains(routed[1].Spans,s=>s.Kind==ToolpathSpanKind.Link&&Math.Sign(routed[1].Points[s.EndIndex].Z-routed[1].Points[s.StartIndex].Z)==(descending?-1:1));
            Assert.InRange(routed[1].Spans.Where(s=>s.Kind==ToolpathSpanKind.Link).Sum(s=>Enumerable.Range(s.StartIndex+1,s.MoveCount)
                .Sum(i=>ToolpathContinuationPlanner.Distance(routed[1].Points[i-1],routed[1].Points[i]))),0,.075);
            Assert.Contains(log,l=>l.Contains("entries=1 "));
            if(descending)Assert.Contains(routed[1].Spans,s=>s.Kind==ToolpathSpanKind.Ramp);
        }
        else Assert.Equal(surface,routed[1].Points[0].Z);
        Console.WriteLine($"[VCarve.RingExit.Test] surface={surface:R} clear={clear} redundantContourLength=0 continuous={routed[0].Points[^1]==routed[1].Points[0]}");
    }

    [Fact]
    public void EverySmoothCutAndEntryUsesOriginalSourcesWithExplicitCertifiedLinks()
    {
        IReadOnlyList<PointD> outer=[new(0,0),new(40,0),new(40,24),new(0,24)];
        IReadOnlyList<PointD> island=[new(14,8),new(14,16),new(26,16),new(26,8)];
        var profile=new VCarveCutterProfile([
            new(CarvingCutterSectionKind.Hemisphere,0,.5,.5),
            new(CarvingCutterSectionKind.Linear,.5,1.5,10)],10.5);
        var originals=new List<TaggedToolpath>();
        var selected=new List<TaggedToolpath>();
        var stages=new List<string>();
        var previous=MillingGeometryTrace.Observer.Value;
        ToolpathPlan plan;
        try
        {
            MillingGeometryTrace.Observer.Value=(stage,batch,paths)=>{
                previous?.Invoke(stage,batch,paths);
                stages.Add(stage);
                if(stage=="protect-input" && batch==1)originals.AddRange(paths);
                if(stage=="stock-selected")selected.AddRange(paths);
            };
            plan=VCarveToolpaths.Generate([new MillingRegion(outer,[island])],
                new ToolGeometry(1.5) {VCarveProfile=profile},new(2,1) {SurfaceZ=12.7},
                new VCarveOptions {SmoothVCarving=true,IncludeInteriorFill=false,
                    EdgeFillWidth=profile.RampFillWidth(2,30),StepOver=.4,RoughingRestWidth=8,
                    RoughingRestChords=[new(new(0,0,-2),new(40,0,-2),.5,2)],
                    DetectCurves=false,CalculateSweptArea=false},BoostVoronoiProvider.CreateDefault());
        }
        finally {MillingGeometryTrace.Observer.Value=previous;}
        Assert.NotEmpty(originals);
        Assert.Contains(originals,p=>p.Category=="clearing" && p.Points[0]==p.Points[^1]);
        Assert.All(originals.Where(p=>p.Category=="clearing"),p=>Assert.Equal(p.Points[0],p.Points[^1]));
        Assert.Equal(originals.Count,selected.Count);
        Assert.NotEmpty(originals.Where(p=>p.ConcentricSource is not null));
        Assert.Equal(originals.Count(p=>p.ConcentricSource is not null),selected.Count(p=>p.ConcentricSource is not null));
        Assert.DoesNotContain("source-circuits",stages);
        Assert.NotEmpty(plan.Toolpaths);
        Assert.Contains(plan.Toolpaths,p=>p.ConcentricSource is not null);
        foreach(var path in plan.Toolpaths)
        {
            Assert.False(path.ContinuationCircuit);
            Assert.Null(path.ContinuationCircuitDefinition);
            foreach(var span in path.Spans.Where(s=>s.Kind is ToolpathSpanKind.Cut or ToolpathSpanKind.Ramp))
            for(int i=span.StartIndex+1;i<=span.EndIndex;i++)
            {
                var chord=new TaggedToolpath([path.Points[i-1],path.Points[i]],[],path.RegionIndex,path.Category,null,detectCurves:false);
                Assert.True(originals.Any(source=>OnSource(chord,source)),
                    $"Off-source required cut: category={path.Category} a={chord.Points[0]} b={chord.Points[1]}");
            }
        }
        Console.WriteLine($"[VCarve.SourcePaths.Test] originalSources={originals.Count} selectedSources={selected.Count} "
            +$"routedVisits={plan.Toolpaths.Count} requiredCutsOnOriginalSources=True connectorsExplicitlyTagged=True");

        static bool OnSource(TaggedToolpath path,TaggedToolpath source)
        {
            if(path.Category!=source.Category)return false;
            for(int i=1;i<path.Points.Count;i++)
            {
                var a=path.Points[i-1];var b=path.Points[i];
                foreach(double t in new[]{0d,.25,.5,.75,1d})
                {
                    double x=a.X+t*(b.X-a.X),y=a.Y+t*(b.Y-a.Y),best=double.PositiveInfinity;
                    for(int j=1;j<source.Points.Count;j++)
                    {
                        var c=source.Points[j-1];var d=source.Points[j];
                        double dx=d.X-c.X,dy=d.Y-c.Y,den=dx*dx+dy*dy;
                        double u=den>0?Math.Clamp(((x-c.X)*dx+(y-c.Y)*dy)/den,0,1):0;
                        best=Math.Min(best,Math.Sqrt(Math.Pow(x-c.X-u*dx,2)+Math.Pow(y-c.Y-u*dy,2)));
                    }
                    if(best>.003)return false;
                }
            }
            return true;
        }
    }

    [Theory]
    [InlineData(0)]
    [InlineData(4)]
    public void SmoothVisitsCarryCompleteSurfaceRampsEvenWithAuthoredStartingFloor(double startingDepth)
    {
        const double surface = 12.7;
        var sources = new[]
        {
            Path([new(5, 5, surface - 1), new(15, 5, surface - 2)]),
            Path([new(15, 5.1, surface - 2), new(5, 5.1, surface - 1)])
        };
        var diagnostics = new List<string>();
        var result = VCarvePathRouter.Route(sources,
            [new PointD[] { new(0, 0), new(20, 0), new(20, 10), new(0, 10) }],
            new ToolGeometry(3) { BottomRadius = 0, TopRadius = 3, ConeLength = 20, TipAngleRadians = 2 * Math.Atan(.15) },
            new DepthSchedule(4, 2) { SurfaceZ = surface, StartDepth = startingDepth },
            new VCarveOptions
            {
                SmoothVCarving = true, SmoothRampAngleDegrees = 15,
                IncludeInteriorFill = true, Ramp = RampSettings.Plunge, DetectCurves = false
            }, 0, null, diagnostic: diagnostics.Add);
        Assert.Equal(sources.Length, result.Count);
        Assert.All(result, path =>
        {
            Assert.Equal(surface, path.Points[0].Z);
            Assert.Contains(path.Spans, span => span.Kind == ToolpathSpanKind.Ramp);
            Assert.DoesNotContain(path.Spans, span => span.Kind == ToolpathSpanKind.Link);
            Assert.All(path.Points, point => Assert.Contains(sources.SelectMany(source => source.Points),
                source => source.X == point.X && source.Y == point.Y));
            for (int i = 1; i < path.Points.Count; i++)
            {
                var a = path.Points[i - 1]; var b = path.Points[i];
                double length = Math.Sqrt(Math.Pow(a.X - b.X, 2) + Math.Pow(a.Y - b.Y, 2));
                Assert.True(Math.Abs(a.Z - b.Z) <= length * Math.Tan(Math.PI / 12) + 1e-10,
                    $"Grade violation: {a}->{b}");
            }
        });
        Assert.Contains(diagnostics, message => message.Contains("smooth=True") && message.Contains("event=complete"));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CertifiedEntryKeepsSourceVerticesAcrossRepeatedTraversals(bool closed)
    {
        var source = new List<Point3D> { new(.0031, .0027, -1), new(.2037, .0027, -1) };
        if (closed)
        {
            source.Add(new(.2037, .2032, -1));
            source.Add(source[0]);
        }
        double angle = Math.PI / 12, grade = Math.Tan(angle);
        var result = RampPlanner.PlanPathEntry(source, 0,
            new RampSettings(angle) { Strategy = RampStrategy.Linear },
            CancellationToken.None, diagnostic: null, landAtSourceVertex: true,
            preserveClosedExit: false);
        Assert.Equal(0, result.Points[0].Z);
        Assert.True(result.Points.Count > source.Count * 2);
        Assert.All(result.Points, point => Assert.Contains(source,
            candidate => candidate.X == point.X && candidate.Y == point.Y));
        for (int i = 1; i < result.Points.Count; i++)
        {
            var a = result.Points[i - 1]; var b = result.Points[i];
            double length = Math.Sqrt(Math.Pow(a.X - b.X, 2) + Math.Pow(a.Y - b.Y, 2));
            Assert.True(Math.Abs(a.Z - b.Z) <= length * grade + 1e-10);
            Assert.True(a.Z >= -1 && b.Z >= -1);
        }
        if (closed) RampEntryStockTests.AssertFullDirectedSourceChords(source, result);
        else Assert.Equal(source, result.Points.Skip(result.Points.Count - source.Count));
    }

    private static TaggedToolpath Path(IReadOnlyList<Point3D> points)
        => new(points, Array.Empty<ToolpathSpan>(), 0, "clearing", 0, detectCurves: false);
}
