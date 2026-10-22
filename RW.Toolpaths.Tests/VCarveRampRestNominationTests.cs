using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class VCarveRampRestNominationTests
{
    [Fact]
    public void FloorCoverageMemoRetainsExactNominationsAcrossCommittedStockUpdates()
    {
        (List<TaggedToolpath> Raw,List<TaggedToolpath> Protected,List<string> Log) Run(bool reuse)
        {
            var accepted=new List<TaggedToolpath>();var log=new List<string>();
            var paths=VCarveRampRestNominator.NominateContours([Box],
                [Path(new(5,-1,-2),new(5,11,-2))],Profile,3,6,
                (x,y)=>1.5+.03*x,(x,y)=>Math.Min(Math.Min(x,10-x),Math.Min(y,10-y)),
                p=>p,default,log.Add,[],p=>p.ToList(),accepted,.5,
                floorContact:p=>p,maximumFloorContactRadius:1/Math.Sqrt(5),reuseFloorCoverage:reuse);
            return(paths,accepted,log);
        }
        var reference=Run(false);var cached=Run(true);
        Assert.NotEmpty(reference.Raw);Assert.Equal(reference.Raw.Count,cached.Raw.Count);
        Assert.Equal(reference.Protected.Count,cached.Protected.Count);
        for(int i=0;i<reference.Raw.Count;i++)Assert.Equal(reference.Raw[i].Points,cached.Raw[i].Points);
        for(int i=0;i<reference.Protected.Count;i++)Assert.Equal(reference.Protected[i].Points,cached.Protected[i].Points);
        Assert.Contains(cached.Log,line=>line.Contains("event=level ")&&
            long.Parse(line.Split(' ').Single(s=>s.StartsWith("floorCoverageMemoHits=")).Split('=')[1])>0);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ClippedOpenContourRefusesUnknownEndpointClearanceAndRetainsValidRamp(bool reverse)
    {
        var log = new List<string>();
        var accepted = new List<TaggedToolpath>();
        IReadOnlyList<PointD> fragment = reverse
            ? [new(7, 5), new(3, 5)] : [new(3, 5), new(7, 5)];
        bool supplied = false;
        int unknownQueries = 0, unknownFloorQueries = 0;
        double Wall(double x, double y)
        {
            if (x == 7) { unknownQueries++; return double.NaN; }
            return 4;
        }
        double Floor(double x, double y)
        {
            if (x == 7) unknownFloorQueries++;
            return 3;
        }
        IEnumerable<VCarveRampRestNominator.RestContour> Restrict(IReadOnlyList<PointD> _, bool __)
        {
            if (supplied) yield break;
            supplied = true;
            yield return new(fragment, false);
        }
        var paths = VCarveRampRestNominator.NominateContours([Box], [], Profile, 3, 1,
            Floor, Wall, p => p, default, log.Add, [], p => p.ToList(), accepted, .5,
            restrictContours: Restrict);
        var path = Assert.Single(paths);
        Assert.Equal(path.Points, Assert.Single(accepted).Points);
        Assert.All(path.Points, p => Assert.True(double.IsFinite(p.Z)));
        Assert.Contains(path.Points, p => p.X == 7 && p.Z == 0);
        Assert.Contains(path.Points, p => p.X == 3 && p.Z == -3);
        Assert.True(unknownQueries > 0);
        Assert.Equal(0, unknownFloorQueries);
        Assert.Contains(log, s => s.Contains("event=unknown-wall-clearance")
            && s.Contains("policy=surface-cap") && s.Contains("physicalProtectionRequired=True"));
    }

    [Fact]
    public void FineFloorSamplingDoesNotBecomeAnUndeclaredLateralStepover()
    {
        var log = new List<string>();
        var paths = VCarveRampRestNominator.NominateContours([Box], [], Profile, 3, 4,
            (_, _) => 3, (_, _) => 4, p => p, default, log.Add, [], p => p.ToList(), [], .5,
            stepOver: .5, initialBand: 3.6);
        Assert.NotEmpty(paths);
        string policy = Assert.Single(log, s => s.Contains("event=prepare "));
        double Number(string field) => double.Parse(policy.Split(' ').Single(s => s.StartsWith(field + "="))
            .Split('=')[1], System.Globalization.CultureInfo.InvariantCulture);
        Assert.Equal(2.6 / 3, Number("pitch"), 10);
        Assert.Equal(2.6 / 11, Number("samplePitch"), 10);
        Assert.Equal(2.6 / 6, Number("refinementPitch"), 10);
        Assert.True(Number("refinementPitch") > Number("samplePitch"));
    }

    static readonly VCarveCutterProfile Profile = new([new(CarvingCutterSectionKind.Hemisphere, 0, 1, 1),
        new(CarvingCutterSectionKind.Linear, 1, 1, 4)], 4);
    static TaggedToolpath Path(Point3D a, Point3D b) => new([a, b], [], 0, "clearing", null, detectCurves: false);
    static IReadOnlyList<PointD> Box => new PointD[] { new(0, 0), new(10, 0), new(10, 10), new(0, 10) };

    [Fact]
    public void FittingANarrowBandDoesNotDemandAHiddenFinerFinishOverTheInterior()
    {
        var cuts=Enumerable.Range(0,11).Select(x=>new TaggedToolpath(new Point3D[]{new(x,-2,-3),
            new(x,0,-3),new(x,12,-3)},[],0,"clearing",null,detectCurves:false)).ToArray();
        var paths=VCarveRampRestNominator.NominateContours([Box],cuts,Profile,3,3,
            (_,_)=>3,(x,y)=>Math.Min(Math.Min(x,10-x),Math.Min(y,10-y)),p=>p,default,null,[],
            p=>p.ToList(),[],.5,stepOver:.5,initialBand:2.6,
            floorContact:p=>p,maximumFloorContactRadius:1/Math.Sqrt(5));
        Assert.DoesNotContain(paths,path=>Enumerable.Range(1,path.Points.Count-1).Any(i=>
            Math.Abs(path.Points[i-1].Y-3)<.0001&&Math.Abs(path.Points[i].Y-3)<.0001
            &&Math.Min(path.Points[i-1].X,path.Points[i].X)>=3
            &&Math.Max(path.Points[i-1].X,path.Points[i].X)<=7));
    }

    [Fact]
    public void AnAlreadyCutAuthoredContourFinishDoesNotDemandFinerRestTexture()
    {
        List<TaggedToolpath> Nominate(IReadOnlyList<TaggedToolpath> old)=>
            VCarveRampRestNominator.NominateContours([Box],old,Profile,3,3,
                (_,_)=>3,(_,_)=>4,p=>p,default,null,[],p=>p.ToList(),[],.5);
        var original=Nominate([]);Assert.NotEmpty(original);
        // Repeat the first chord after closure so its actual start is known;
        // omission intentionally grants no incoming-axis credit to a visit.
        var executed=original.Select(p=>p with{Points=p.Points.Concat(p.Points.Take(2)).ToArray()}).ToArray();
        Assert.Empty(Nominate(executed));
    }

    [Fact]
    public void ARealFloorWitnessCannotOmitTheSeparateFullProfileContactFront()
    {
        var guard=Path(new(1.2,0,-1.5),new(1.2,10,-1.5));
        var paths=VCarveRampRestNominator.NominateContours([Box],[guard],Profile,3,3,
            (_,_)=>1.5,(x,y)=>Math.Min(Math.Min(x,10-x),Math.Min(y,10-y)),p=>p,
            default,null,[],p=>p.ToList(),[],.5);
        // The old sphere covers the new tip within the floor nomination
        // allowance, while its cylindrical flank cannot reach the wall.
        Assert.True(1.5-Profile.PhysicalProfile.Bottom(.198)>1.5-.025);
        Assert.Contains(paths,path=>Enumerable.Range(1,path.Points.Count-1).Any(i=>
            Math.Abs(path.Points[i-1].X-1.002)<.0011&&Math.Abs(path.Points[i].X-1.002)<.0011
            &&Math.Min(path.Points[i-1].Y,path.Points[i].Y)<=5
            &&Math.Max(path.Points[i-1].Y,path.Points[i].Y)>=5));
    }

    [Fact]
    public void CollapsedStripRetainsItsFullDepthCenterContactWithoutAnInventedClosingChord()
    {
        IReadOnlyList<PointD> strip=[new(0,0),new(7.8,0),new(7.8,20),new(0,20)];
        var paths=VCarveRampRestNominator.NominateContours([strip],[],Profile,3,10,
            (_,_)=>3,(x,y)=>Math.Min(Math.Min(x,7.8-x),Math.Min(y,20-y)),p=>p,
            default,null,[],p=>p.ToList(),[],.5);
        // The last complete scheduled ring misses the centre by .5 mm,
        // leaving a .134 mm ball-nose ridge. A native medial terminal must
        // actually cut the strip's centre, rather than closing its endpoints.
        Assert.Contains(paths,path=>Enumerable.Range(1,path.Points.Count-1).Any(i=>
            Math.Abs(path.Points[i-1].X-3.9)<.001&&Math.Abs(path.Points[i].X-3.9)<.001
            &&Math.Min(path.Points[i-1].Y,path.Points[i].Y)<=10
            &&Math.Max(path.Points[i-1].Y,path.Points[i].Y)>=10
            &&path.Points[i-1].Z<-2.999&&path.Points[i].Z<-2.999));
    }

    [Fact]
    public void ACollapsedBandShorterThanOneAuthoredIntervalRetainsItsTerminalRidge()
    {
        IReadOnlyList<PointD> strip = [new(0,0), new(3.8,0), new(3.8,20), new(0,20)];
        var paths = VCarveRampRestNominator.NominateContours([strip], [], Profile, 3, 1.95,
            (_,_) => 3, (x,y) => Math.Min(Math.Min(x,3.8-x),Math.Min(y,20-y)), p=>p,
            default, null, [], p=>p.ToList(), [], .5, stepOver:.5);
        Assert.Contains(paths, path => Enumerable.Range(1,path.Points.Count-1).Any(i =>
            Math.Abs(path.Points[i-1].X-1.9)<.001 && Math.Abs(path.Points[i].X-1.9)<.001
            && Math.Min(path.Points[i-1].Y,path.Points[i].Y)<=10
            && Math.Max(path.Points[i-1].Y,path.Points[i].Y)>=10
            && path.Points[i-1].Z<-2.999 && path.Points[i].Z<-2.999));
    }

    [Fact]
    public void RestContoursUseOriginalProtectionWithoutReversingCleanupTeeth()
    {
        var accepted=new List<TaggedToolpath>();int protects=0;
        var paths=VCarveRampRestNominator.NominateContours([Box],[],Profile,3,1,
            (x,y)=>1+.02*x,(_,_)=>4,p=>p,default,null,[],
            rows=>{protects++;return rows.ToList();},accepted,.5);
        Assert.NotEmpty(paths);Assert.Equal(paths.Count,accepted.Count);Assert.True(protects>0);
        Assert.Contains(paths,path=>path.Points[0]==path.Points[^1]);
        Assert.All(paths,path=>{
            Assert.All(path.Points,p=>Assert.Equal(-1-.02*p.X,p.Z,9));
            // No source tooth returns or isolated single-sample extensions.
            for(int i=1;i<path.Points.Count-1;i++)
            {
                var a=path.Points[i-1];var b=path.Points[i];var c=path.Points[i+1];
                Assert.True((b.X-a.X)*(c.X-b.X)+(b.Y-a.Y)*(c.Y-b.Y)>=-1e-10);
            }
        });
    }

    [Fact]
    public void WholeRougherWitnessCanAvoidAnEntireRestContour()
    {
        var rough=new VCarveRoughingChord(new(0,5,-3),new(10,5,-3),10,10);
        var paths=VCarveRampRestNominator.NominateContours([Box],[],Profile,3,1,
            (_,_)=>1,(_,_)=>4,p=>p,default,null,[rough],p=>p.ToList(),[],.5);
        Assert.Empty(paths);
    }

    [Fact]
    public void TipDiskWithoutAClearedColumnDoesNotOmitTheFinishingFlank()
    {
        var rough=new VCarveRoughingChord(new(0,5,-3),new(10,5,-3),10);
        var paths=VCarveRampRestNominator.NominateContours([Box],[],Profile,3,1,
            (_,_)=>1,(_,_)=>4,p=>p,default,null,[rough],p=>p.ToList(),[],.5);
        Assert.NotEmpty(paths);
    }

    [Fact]
    public void LocalRestDemandProducesContinuousContourArcsWithoutCirclingClearedStock()
    {
        var accepted=new List<TaggedToolpath>();
        var paths=VCarveRampRestNominator.NominateContours([Box],[],Profile,3,1,
            (x,y)=>x>=4&&x<=5?1:0,(_,_)=>4,p=>p,default,null,[],p=>p.ToList(),accepted,.5);
        Assert.NotEmpty(paths);
        Assert.All(paths,path=>{
            Assert.NotEqual(path.Points[0],path.Points[^1]);
            Assert.True(path.Points.Max(p=>p.X)>path.Points.Min(p=>p.X));
            for(int i=1;i<path.Points.Count-1;i++)
            {
                var a=path.Points[i-1];var b=path.Points[i];var c=path.Points[i+1];
                Assert.True((b.X-a.X)*(c.X-b.X)+(b.Y-a.Y)*(c.Y-b.Y)>=-1e-10);
            }
            Assert.All(path.Points,p=>Assert.InRange(p.X,2,7));
        });
    }

    [Fact]
    public void FullDepthContactContourPreservesConicalSideCutNearTheWall()
    {
        var cutter=new VCarveCutterProfile(new[]{new CarvingCutterSection(CarvingCutterSectionKind.Hemisphere,0,1,1),
            new CarvingCutterSection(CarvingCutterSectionKind.Linear,1,1.2,20)},6);
        var paths=VCarveRampRestNominator.NominateContours([Box],[],cutter,6,3,
            (_,_)=>6,(x,y)=>Math.Min(Math.Min(x,10-x),Math.Min(y,10-y)),p=>p,default,null,[],p=>p.ToList(),[],.5);
        // A tip-only uniform-pitch scallop check misses this side stock:
        // the shallow spherical pass removes almost nothing at x=.02,
        // while the full-depth conical flank removes more than 1 mm there.
        const double queryX=.02,queryY=5;
        double removed=0;
        foreach(var path in paths)
        for(int i=1;i<path.Points.Count;i++)
        {
            var a=path.Points[i-1];var b=path.Points[i];
            if(a.X!=b.X||a.Y==b.Y||queryY<Math.Min(a.Y,b.Y)||queryY>Math.Max(a.Y,b.Y))continue;
            double t=(queryY-a.Y)/(b.Y-a.Y),z=a.Z+(b.Z-a.Z)*t;
            removed=Math.Max(removed,-z-cutter.PhysicalProfile.Bottom(Math.Abs(a.X-queryX)));
        }
        Assert.True(removed>1,$"Conical wall-side stock was missed: removed={removed:R}");
    }

    [Fact]
    public void ContourDepthSamplesRetainPermittedFloorCrestsBetweenTheirEndpoints()
    {
        double Floor(double x,double y)=>Math.Max(0,1-.5*Math.Abs(x-4.125));
        var paths=VCarveRampRestNominator.NominateContours([Box],[],Profile,1,2,
            Floor,(_,_)=>4,p=>p,default,null,[],p=>p.ToList(),[],.5);
        Assert.Contains(paths.SelectMany(p=>p.Points),p=>-p.Z>.99);
        Assert.All(paths.SelectMany(p=>p.Points),p=>Assert.True(-p.Z<=Floor(p.X,p.Y)+1e-12));
    }

    [Fact]
    public void ClearedMaximumDepthSamplesDoNotSpendAFieldQuery()
    {
        var log=new List<string>();
        var rough=new VCarveRoughingChord(new(0,5,-3),new(10,5,-3),10,10);
        var paths=VCarveRampRestNominator.NominateContours([Box],[],Profile,3,1,
            (_,_)=>1,(_,_)=>4,p=>p,default,log.Add,[rough],p=>p.ToList(),[],.5);
        Assert.Empty(paths);
        string complete=Assert.Single(log,s=>s.Contains("event=complete contours="));
        Assert.Contains(" queries=0 ",complete);
        Assert.Contains(log,s=>s.Contains("event=level ")&&s.Contains(" sampleQueries=0 crestQueries=0 flatSkipped="));
    }

    [Fact]
    public void InnerRingsOffsetTheWallContactFamilyInsteadOfReprojectingAgainstInteriorFloorChanges()
    {
        var cutter=new VCarveCutterProfile([new(CarvingCutterSectionKind.Hemisphere,0,1,1),
            new(CarvingCutterSectionKind.Linear,1,1.2,20)],6);
        double Wall(double x,double y)=>Math.Min(Math.Min(x,10-x),Math.Min(y,10-y));
        // The shallow wall establishes local cutter reach. A deeper interior
        // must change the cutting Z without shifting each concentric XY front
        // independently back toward a full-depth radius construction.
        var paths=VCarveRampRestNominator.NominateContours([Box],[],cutter,6,3,
            (x,y)=>Wall(x,y)<1.5?.5:6,Wall,p=>p,default,null,[],p=>p.ToList(),[],.5);
        double expected=cutter.PhysicalRadiusAtDepth(.5)+3-cutter.PhysicalRadiusAtDepth(6);
        double innermost=paths.Where(p=>p.Points[0]==p.Points[^1]).Max(p=>p.Points.Min(q=>q.X));
        Assert.InRange(innermost,expected-.011,expected+.011);
        Assert.Contains(paths.SelectMany(p=>p.Points),p=>p.Z<-5.99);
    }

    [Fact]
    public void RestFrontUsesTheLocallyEngagedCutterRadiusInsteadOfTheFullDepthRadius()
    {
        var cutter=new VCarveCutterProfile(new[]{new CarvingCutterSection(CarvingCutterSectionKind.Hemisphere,0,1,1),
            new CarvingCutterSection(CarvingCutterSectionKind.Linear,1,1.2,20)},6);
        var paths=VCarveRampRestNominator.NominateContours([Box],[],cutter,6,3,
            (_,_)=>4,(x,y)=>Math.Min(Math.Min(x,10-x),Math.Min(y,10-y)),p=>p,default,null,[],p=>p.ToList(),[],.5);
        double contact=cutter.PhysicalRadiusAtDepth(4)+.002;
        Assert.Contains(paths,path=>Enumerable.Range(1,path.Points.Count-1).Any(i=>
            Math.Abs(path.Points[i-1].X-contact)<.0011&&Math.Abs(path.Points[i].X-contact)<.0011
            &&Math.Min(path.Points[i-1].Y,path.Points[i].Y)<=5
            &&Math.Max(path.Points[i-1].Y,path.Points[i].Y)>=5
            &&path.Points[i-1].Z<-3.99&&path.Points[i].Z<-3.99));
    }

    [Fact]
    public void StreamingProtectionPreservesCompleteCutterNominationsRegardlessOfTipWitnesses()
    {
        List<TaggedToolpath> Run(Func<IReadOnlyList<TaggedToolpath>,List<TaggedToolpath>>? protect,
            List<TaggedToolpath> accepted) => VCarveRampRestNominator.NominateRasterReference([Box],[],Profile,3,10,
                (_,y)=>Math.Max(.1,3-.3*y),(_,_)=>4,p=>p,default,null,protect:protect,protectedOutput:accepted);
        var all=Run(null,[]);
        var kept=new List<TaggedToolpath>();
        var incremental=Run(paths=>paths.ToList(),kept);
        Assert.Equal(all.Count,incremental.Count);
        Assert.Equal(incremental.Count,kept.Count);
        var raised=Run(paths=>paths.Select(path=>path with {Points=path.Points.Select(p=>p with{Z=0}).ToArray()}).ToList(),[]);
        Assert.Equal(all.Count,raised.Count);
        Assert.All(Enumerable.Range(0,all.Count),i=>Assert.Equal(all[i].Points,incremental[i].Points));
    }

    [Fact]
    public void DeeperTipAtDifferentXYDoesNotEraseUncutRampDemand()
    {
        var visits = VCarveRampRestNominator.NominateRasterReference([Box], [Path(new(5, 3, -1.7), new(5, 7, -1.7))],
            Profile, 3, 10, (x, y) => x >= 4 && x <= 4.4 && y >= 4 && y <= 6 ? 1.5 : 0,
            (_, _) => 3, p => p, default, Console.WriteLine);
        Assert.Contains(visits.SelectMany(path => path.Points), p => p.X < 4.5 && p.Y > 4 && p.Y < 6);
        // The nearby tip is deeper, but its sphere underside removes less at
        // the demanded XY. This is a full physical pose test, not depth ranking.
        Assert.True(1.7 - Profile.PhysicalProfile.Bottom(.8) < 1.5 - .025);
    }

    [Fact]
    public void PermittedRampCrestsBetweenGridSitesReceiveFullDepthFinishing()
    {
        var visits = VCarveRampRestNominator.NominateRasterReference([Box], [], Profile, 1, 10,
            (x, _) => Math.Max(0, 1 - .5 * Math.Abs(x - 4.125)), (_, _) => 4,
            p => p, default, Console.WriteLine);
        Assert.Contains(visits.SelectMany(path => path.Points), p => p.X == 4.125 && p.Z == -1);
        Assert.All(visits.SelectMany(path => path.Points), p => Assert.True(-p.Z <= Math.Max(0, 1 - .5 * Math.Abs(p.X - 4.125))));
    }

    [Fact]
    public void RoughingOnlySuppressesCleanupInsideAWholeClearedCell()
    {
        var chord = new VCarveRoughingChord(new(2, 5, -3), new(8, 5, -3), 2);
        var visits = VCarveRampRestNominator.NominateRasterReference([Box], [], Profile, 3, 10,
            (x, y) => x >= 3 && x <= 7 && y >= 3 && y <= 5 ? 3 : 0,
            (_, _) => 4, p => p, default, Console.WriteLine, [chord]);
        // The centreline interior needs no cleanup; a cell straddling the
        // rougher's y=3 footprint edge retains finishing overlap.
        Assert.DoesNotContain(visits.SelectMany(p => p.Points), p => p.X > 3 && p.X < 7 && p.Y > 4 && p.Y < 5);
        Assert.Contains(visits.SelectMany(p => p.Points), p => p.X >= 3 && p.X <= 7 && p.Y <= 3.25);
    }

    [Fact]
    public void ExistingPhysicalPosesAvoidRedundantFinishingAndSignedHolesStayEmpty()
    {
        var source = new List<TaggedToolpath>();
        for (int y = 0; y <= 50; y++) source.Add(Path(new(0, y * .2, -1), new(10, y * .2, -1)));
        var complete = VCarveRampRestNominator.NominateRasterReference([Box], source, Profile, 1, 10,
            (x, y) => x >= 2 && x <= 8 && y >= 2 && y <= 8 ? 1 : 0,
            (_, _) => 3, p => p, default, Console.WriteLine);
        Assert.Empty(complete);
        IReadOnlyList<PointD> hole = new PointD[] { new(3, 3), new(3, 7), new(7, 7), new(7, 3) };
        var rest = VCarveRampRestNominator.NominateRasterReference([Box, hole], [], Profile, 3, 10,
            (_, _) => 1, (_, _) => 3, p => p, default, Console.WriteLine);
        Assert.NotEmpty(rest);
        Assert.DoesNotContain(rest.SelectMany(path => path.Points), p => p.X > 3 && p.X < 7 && p.Y > 3 && p.Y < 7);
    }
}
