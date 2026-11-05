using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class VCarveMaterialSurfaceTests
{
    [Fact]
    public void InheritedCellFloorsKeepLiteralProofDecisionsAtPriorSweepAndPartEdges()
    {
        var part=Rectangle(-6,-3,6,3);
        var target=Rectangle(-.35,-2,.35,2);
        var sweep=new VCarveClearedSweep(Rounded,[new(new(-4,0,-2),new(4,0,-6),0)]);
        var control=new VCarveProjectedStock([], [target],part,Rounded,8,default,[sweep],
            memoRefusals:false,inheritCellClearance:false);
        var inherited=new VCarveProjectedStock([], [target],part,Rounded,8,default,[sweep],memoRefusals:false);
        int accepted=0,refused=0;
        foreach(bool clearedOnly in new[]{false,true})
        foreach(bool postingReserve in new[]{false,true})
        foreach(double depth in new[]{.5,1.5,3.5,5.5})
        for(int x=-5;x<=5;x++)
        foreach(double y in new[]{0,.4,2.9})
        {
            var a=new Point3D(x+.037,y,-depth);
            var b=a with{X=a.X+.017,Z=a.Z+.023};
            foreach(var end in new[]{a,b})
            {
                bool expected=control.Prove(a,end,clearedOnly,postingReserve);
                Assert.Equal(expected,inherited.Prove(a,end,clearedOnly,postingReserve));
                if(expected)accepted++;else refused++;
            }
        }
        Assert.True(accepted>0);Assert.True(refused>0);
        Assert.True(inherited.InheritedStockCellProofs>0);
        Console.WriteLine($"[Stock.InheritedFloors] accepted={accepted} refused={refused} "
            +$"inheritedWholeChildProofs={inherited.InheritedStockCellProofs} literalReferenceDecisions=True partAndTargetPermissionsSeparate=True");
    }

    [Fact]
    public void ParallelMedialFitsRetainEveryOriginalSiteWithinTheExistingSurfaceBound()
    {
        var target=Rectangle(-5,-8,5,8);
        var stock=new VCarveClearedStock(Sweeps(6.35),[6.35],Rectangle(-12,-12,12,12));
        var source=Enumerable.Range(0,8).Select(row=>{
            var points=Enumerable.Range(0,801).Select(i=>new Point3D(-4+i*.01,row*.005,-3)).ToArray();
            if(row%2!=0)Array.Reverse(points);
            return new TaggedToolpath(points,[],row,"final-carve",0,detectCurves:false);
        }).ToList();
        var serial=new VCarveMaterialSurface([target],stock,Rounded,12.7,12.7,default,null,
            parallelMedialFits:false).Apply(source);
        var parallel=new VCarveMaterialSurface([target],stock,Rounded,12.7,12.7,default,null).Apply(source);
        var independent=new VCarveMaterialSurface([target],stock,Rounded,12.7,12.7,default,null,
            parallelMedialFits:false);
        Assert.Equal(serial.Count,parallel.Count);
        double maximum=0;
        for(int i=0;i<source.Count;i++)
        {
            var before=serial[i].Points.GroupBy(p=>(p.X,p.Y)).ToDictionary(g=>g.Key,g=>g.Min(p=>p.Z));
            var after=parallel[i].Points.GroupBy(p=>(p.X,p.Y)).ToDictionary(g=>g.Key,g=>g.Min(p=>p.Z));
            foreach(var point in source[i].Points)
            {
                Assert.True(before.ContainsKey((point.X,point.Y)));
                Assert.True(after.ContainsKey((point.X,point.Y)));
                maximum=Math.Max(maximum,Math.Abs(before[(point.X,point.Y)]-after[(point.X,point.Y)]));
            }
            foreach(var (a,b) in parallel[i].Points.Zip(parallel[i].Points.Skip(1)))
                Assert.True(independent.IsLegalChord(a,b));
        }
        Assert.InRange(maximum,0,.004);
        Console.WriteLine($"[MedialFit.Equivalence] originalSites={source.Sum(p=>p.Points.Count)} maximumSiteDepthChange={maximum:R} "
            +"originalXYRetained=True fullIndependentNativeAndF3Proof=True surfaceBoundMm=0.004");
    }

    [Fact]
    public void RefusalWitnessesKeepFullProofDecisionsAgainstVariableDepthPriorProfiles()
    {
        var sweep=new VCarveClearedSweep(Rounded,[new(new(-4,0,-1),new(4,0,-6),0)]);
        var part=Rectangle(-12,-12,12,12);
        var control=new VCarveProjectedStock([],[],part,Rounded,10,default,[sweep],memoRefusals:false);
        var cached=new VCarveProjectedStock([],[],part,Rounded,10,default,[sweep]);
        int accepted=0,refused=0;
        foreach(double shift in new[]{-.2,.05,.15,.5})
        for(int i=1;i<20;i++)
        foreach(double y in new[]{.2,0,-.2,.03})
        {
            double t=i/20d;
            var point=new Point3D(-4+8*t,y,-1-5*t+shift);
            bool expected=control.Prove(point,point);
            Assert.Equal(expected,cached.Prove(point,point));
            if(expected)accepted++;else refused++;
        }
        Assert.True(accepted>0);Assert.True(refused>0);
        Assert.True(cached.RefusalWitnessHits>0);
    }

    [Fact]
    public void BoundedDyadicCellMemoRetainsIndependentHoleClearanceAcrossEviction()
    {
        var outer=Rectangle(-10,-10,10,10);
        var hole=Rectangle(-1,-1,1,1);Array.Reverse(hole);
        var cylinder=new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear,1,1,10)],10);
        var small=new VCarveProjectedStock([], [outer,hole],outer,cylinder,2,default,cellCacheLimit:64);
        var control=new VCarveProjectedStock([], [outer,hole],outer,cylinder,2,default,memoRefusals:false);
        for(int iy=-20;iy<=20;iy++)
        for(int ix=-20;ix<=20;ix++)
        {
            var pose=new Point3D(ix*.3+.07,iy*.3+.11,-1);
            double dx=Math.Max(0,Math.Abs(pose.X)-1),dy=Math.Max(0,Math.Abs(pose.Y)-1);
            double distance=Math.Sqrt(dx*dx+dy*dy);
            if(Math.Abs(distance-1.002)<.01)continue;
            bool expected=distance>1.002;
            Assert.Equal(expected,control.Prove(pose,pose));
            Assert.Equal(expected,small.Prove(pose,pose));
        }
        Assert.True(small.StockCellCacheEvictions>0);
        var outsidePackedDomain=new Point3D(1_100_000,0,-1);
        Assert.False(small.Prove(outsidePackedDomain,outsidePackedDomain));
    }

    [Fact]
    public void ProfileForestsPreserveTheIndependentMixedToolRemovalEnvelope()
    {
        var cone=new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear,0,3,6)],6);
        var cylinder=new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear,1.25,1.25,10)],10);
        var flatPose=new Point3D(-4,0,-2);
        var ballPose=new Point3D(4,0,-8);
        var conePose=new Point3D(0,-4,-4);
        var cylinderPose=new Point3D(0,4,-5);
        var stock=new VCarveProjectedStock([new(flatPose,flatPose,1.5,1.5)],[],Rectangle(-12,-12,12,12),
            Rounded,10,default,[new(Rounded,[new(ballPose,ballPose,0)]),
                new(cone,[new(conePose,conePose,0)]),new(cylinder,[new(cylinderPose,cylinderPose,0)])]);
        foreach(bool reserved in new[]{false,true})
        for(int ix=-16;ix<=16;ix++)
        for(int iy=-16;iy<=16;iy++)
        {
            double x=ix*.5,y=iy*.5,disk=.125;
            double extra=disk+(reserved?PostedSweepInnerRegion.Reserve:0);
            double Distance(Point3D pose)=>Math.Sqrt(Math.Pow(x-pose.X,2)+Math.Pow(y-pose.Y,2))+extra;
            double r=Distance(ballPose);
            double noseBottom=r<=1.5875?1.5875-Math.Sqrt(Math.Max(0,1.5875*1.5875-r*r))
                :r<=3?1.5875+20*(r-1.5875)/(3-1.5875):double.PositiveInfinity;
            double expected=Math.Max(0,Math.Max(Distance(flatPose)<=1.5?2:0,
                Math.Max(8-noseBottom,Math.Max(4-2*Distance(conePose),Distance(cylinderPose)<=1.25?5:0))));
            Assert.InRange(stock.ClearedDepth(x,y,disk,reserved),expected-1e-9,expected+1e-9);
        }
        Assert.Equal(4,stock.ProfileForests);
        Assert.True(stock.ProfileNodesPruned>0);
    }

    [Fact]
    public void PriorPoseNomineesNeverTransferHeightToADistantQuery()
    {
        var cutter=new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear,0,3,6)],6);
        var stock=new VCarveProjectedStock([], [Rectangle(-20,-20,20,20)],Rectangle(-30,-30,30,30),
            cutter,6,default,[new(cutter,[new(new(-5,0,-6),new(-5,0,-6),0),new(new(5,0,-2),new(5,0,-2),0)])]);
        for(int i=0;i<4;i++)
        {
            Assert.InRange(stock.ClearedDepth(-5,0,.05),5.85,5.91);
            Assert.InRange(stock.ClearedDepth(5,0,.05),1.85,1.91);
            Assert.Equal(0,stock.ClearedDepth(0,0,.05));
        }
        Assert.True(stock.PriorPoseSeedTrials>0);
    }

    private static readonly VCarveCutterProfile Rounded = new([
        new(CarvingCutterSectionKind.Hemisphere, 0, 1.5875, 1.5875),
        new(CarvingCutterSectionKind.Linear, 1.5875, 3, 20),
        new(CarvingCutterSectionKind.Linear, 3, 3, 90.4)], 111.9875);

    [Fact]
    public void ActualPostedProofDistinguishesThePhysicalToolFromThePlanningPostingShell()
    {
        var priorCylinder=new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear,2,2,20)],20);
        var prior=Sweeps(6.35).Select(c=>c with{ClearedColumnRadius=0}).ToArray();
        var stock=new VCarveClearedStock([],[],Rectangle(-12,-12,12,12),[new(priorCylinder,prior)]);
        var field=new VCarveMaterialSurface([Rectangle(-.5,-8,.5,8)],stock,Rounded,12.7,12.7,default,null);
        var pose=new Point3D(0,0,-6.4303);
        // Independent spherical footprint at the actual preceding floor.
        double h=-pose.Z-6.35;
        double radius=Math.Sqrt(h*(2*1.5875-h));
        Assert.True(radius<.5);
        Assert.True(radius+.002>.5);
        Assert.True(field.IsLegalChord(pose,pose,postingReserve:false));
        Assert.False(field.IsLegalChord(pose,pose,postingReserve:true));
        Assert.False(field.IsLegalChord(new(0,0,-6.432),new(0,0,-6.432)));
    }

    [Fact]
    public void NeighbouringFitsReuseActualStockCellsWithoutBorrowingTargetPermission()
    {
        var background=new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear,0,3,6)],6);
        var cutter=new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear,0,2,10)],10);
        var target=Rectangle(-2.7,-.2,-.3,.2);
        var part=Rectangle(-12,-12,12,12);
        var sweeps=new VCarveClearedSweep[]{new(background,[new(new(-3,0,-6),new(-3,0,-6),0)])};
        var stock=new VCarveProjectedStock([], [target],part,cutter,8,default,sweeps);
        for(int i=0;i<16;i++)
        {
            double x=-1.5+i*.0002;
            Assert.True(stock.Prove(new(x,0,-3.5),new(x,0,-3.5)));
            // The tip itself is below the preceding cone here. This proof
            // requires the authored Void permission, not cleared stock alone.
            Assert.False(stock.Prove(new(x,0,-3.5),new(x,0,-3.5),clearedOnly:true));
        }
        Assert.True(stock.StockCellCacheHits>stock.StockCellCacheMisses);
        var a=new Point3D(-1.5,0,-3.5);var b=new Point3D(-1.499,0,-3.5);
        Assert.True(stock.Prove(a,b));
        long provedQueries=stock.Queries;
        Assert.True(stock.Prove(b,a));
        Assert.True(stock.Prove(a,b,postingReserve:false));
        Assert.Equal(provedQueries,stock.Queries);
        Assert.True(stock.ReversedSweepProofHits>0);
        Assert.True(stock.StrongerSweepProofHits>0);
        Assert.False(stock.Prove(a,b,clearedOnly:true));
        var outsideA=new Point3D(20,0,-3.5);var outsideB=new Point3D(21,0,-3.5);
        Assert.False(stock.Prove(outsideA,outsideB));
        long refusedQueries=stock.Queries;
        Assert.False(stock.Prove(outsideB,outsideA,postingReserve:false));
        Assert.True(stock.Queries>refusedQueries);
        double refined=stock.RefineDepth(-1.5,0,.1,.1,false,null);
        Assert.InRange(refined,3.5,5);
        Assert.True(stock.Prove(new(-1.5,0,-refined),new(-1.5,0,-refined)));
        Assert.True(stock.DepthWitnessBounds>0);
        Assert.True(stock.DepthWitnessSeedFits>0);
        for(int i=1;i<=12;i++)
        {
            double x=-1.5+i*.0001;
            double adjacent=stock.RefineDepth(x,0,.1,.1,false,null);
            Assert.InRange(adjacent,3.5,5);
            Assert.True(stock.Prove(new(x,0,-adjacent),new(x,0,-adjacent)));
            Assert.False(stock.Prove(new(x,0,-adjacent),new(x,0,-adjacent),clearedOnly:true));
        }
        Assert.True(stock.NearbyDepthTrials>0);
        Assert.True(stock.NearbyDepthFits>0);
        var unrelated=new VCarveProjectedStock([], [target],part,cutter,8,default);
        Assert.Equal(0,unrelated.StockCellCacheHits);
        Assert.False(unrelated.Prove(new(-1.5,0,-3.5),new(-1.5,0,-3.5)));
        // A long chord traverses several independent root cells. Any
        // intervening uncleared protected gap must still refuse the sweep.
        Assert.False(stock.Prove(new(-1.5,0,-3.5),new(20,0,-3.5)));
    }

    [Fact]
    public void NarrowBranchesSurviveAndDescendFromTheActuallyRoughedBackground()
    {
        var field = Field(Rectangle(-.5, -8, .5, 8), Sweeps(6.35));
        Point3D[] source = [new(0, -5, -.08), new(0, 0, -.08), new(0, 5, -.08)];
        var path = Assert.Single(field.Apply([new(source, Array.Empty<ToolpathSpan>(),
            0, "final-carve", null, detectCurves: false)]));

        foreach (var original in source)
            Assert.Contains(path.Points, point => point.X == original.X && point.Y == original.Y);
        // Independent ball sagitta expectation: the half-mm Void clearance
        // applies to the nose below the 6.35-mm Background floor, not Part top.
        double expected = 6.35 + 1.5875 - Math.Sqrt(1.5875 * 1.5875 - .5 * .5);
        Assert.All(path.Points, point => Assert.InRange(-point.Z, expected - .004, expected));
        Assert.All(path.Points.Zip(path.Points.Skip(1)), pair =>
            Assert.True(field.IsLegalChord(pair.First, pair.Second)));
    }

    [Fact]
    public void RaisedEntryReusesOnlyTheExactProvedNativeAndPostedSourceTrace()
    {
        var field=Field(Rectangle(-.5,-8,.5,8),Sweeps(6.35));
        var source=Assert.Single(field.Apply([new(new Point3D[]{new(0,-1,-.08),new(0,1,-.08)},
            [],0,"final-carve",null,detectCurves:false)]));
        var a=source.Points[0];var b=source.Points[1];
        long before=field.RaisedSourceProofs;
        Assert.True(field.IsLegalChord(a with{Z=a.Z+.0000286312526},b));
        Assert.True(field.IsLegalChord(b,a with{Z=a.Z+.0000286312526}));
        Assert.True(field.RaisedSourceProofs>=before+4);
        Assert.False(field.IsLegalChord(a with{Z=a.Z-.02},b with{Z=b.Z-.02}));
        Assert.False(field.IsLegalChord(a with{X=3},b with{X=3}));
    }

    [Fact]
    public void UntouchedTallWallStillLimitsDepthDespiteLowerStockOnTheOtherSide()
    {
        var field = Field(Rectangle(-.5, -8, .5, 8), Sweeps(6.35, lastX: -2));
        Assert.InRange(field.AllowedDepth(0, 0), .079, .082);
        Assert.False(field.IsLegalChord(new(0, -1, -6.4), new(0, 1, -6.4)));
    }

    [Fact]
    public void OneDeepEndpointDoesNotCreditTheWholeSlopingSweepAtItsFinalFloor()
    {
        var traverses = Sweeps(6.35).Select(c => c with { A = c.A with { Z = 0 } }).ToArray();
        var field = Field(Rectangle(-.5, -8, .5, 8), traverses);
        Assert.InRange(field.AllowedDepth(0, 0), 3, 5);
        Assert.False(field.IsLegalChord(new(0,-9,-6.4),new(0,-8,-6.4)));
    }

    [Fact]
    public void DeclaredSlicesWithoutActualColumnsNeverLowerDetail()
    {
        var field = Field(Rectangle(-.5,-8,.5,8), []);
        Assert.InRange(field.AllowedDepth(0,0), .079, .082);
    }

    [Fact]
    public void APrecedingVCarversPhysicalTaperSuppliesTheSlopingSupportSurface()
    {
        var background=new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear,0,3,6)],6);
        var finisher=new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear,0,2,10)],10);
        var stock=new VCarveClearedStock([], [6], Rectangle(-12,-12,12,12), [new(background,[
            new(new(-3,0,-6),new(-3,0,-6),0,0)])]);
        var target=Rectangle(-2.7,-.2,-.3,.2);
        var field=new VCarveMaterialSurface([target],stock,finisher,8,12.7,default,null);
        foreach(double x in new[]{-2d,-1.5,-1})
        {
            // Independent dense contact minimization against the conical
            // Background sweep's h=6-2*r surface on the two channel walls.
            double expected=double.PositiveInfinity;
            for(int i=0;i<=10000;i++)
            {
                double q=x-.6+1.2*i/10000;
                double height=Math.Max(0,6-2*Math.Sqrt((q+3)*(q+3)+.04));
                expected=Math.Min(expected,height+5*Math.Sqrt((q-x)*(q-x)+.04));
            }
            double actual=field.AllowedDepth(x,0);
            Assert.InRange(actual,expected-.025,expected+.0001);
            Assert.True(actual>6-2*(x+3)+.8,$"Finishing did not cut into Background's taper: x={x}, depth={actual}");
        }
        var index=new VCarveProjectedStock([], [target],stock.PartBoundary,finisher,8,default,stock.ProfileSweeps);
        Assert.InRange(index.ClearedDepth(-1.5,0,0),2.997,3.001);
        Assert.Equal(0,index.ClearedDepth(1,0,0));
    }

    [Fact]
    public void ProfileSweepCompactionRetainsCrossedDepthsAndOnlyDropsDominatedActualSweeps()
    {
        var cutter=new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear,0,3,6)],6);
        VCarveRoughingChord[] source=[new(new(-2,0,-2),new(2,0,-6),0),new(new(-2,0,-6),new(2,0,-2),0),
            new(new(-2,0,-1),new(2,0,-1),0),new(new(3,0,-3),new(4,0,-3),0),new(new(4,0,-3),new(5,0,-3),0)];
        var index=new VCarveProjectedStock([], [Rectangle(-.5,-8,.5,8)], Rectangle(-12,-12,12,12),cutter,6,default,[new(cutter,source)]);
        Assert.InRange(index.IndexedChords,3,4);
        foreach(double x in new[]{-2d,-1,0,1,2,3.5,4.5})
        {
            double sampled=0;
            foreach(var chord in source)
            for(int i=0;i<=2000;i++)
            {
                double t=i/2000d,px=chord.A.X+(chord.B.X-chord.A.X)*t;
                double depth=-(chord.A.Z+(chord.B.Z-chord.A.Z)*t)-2*Math.Abs(px-x);
                sampled=Math.Max(sampled,depth);
            }
            Assert.InRange(index.ClearedDepth(x,0,0),sampled-.003,sampled+.00001);
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void DetailProjectsOntoTheContinuousRampWithFullRoundedCutterContact(bool reverse)
    {
        var a = new Point3D(0,-10,-2); var b = new Point3D(0,10,-4);
        var field = Field(Rectangle(-.5,-8,.5,8), [new(reverse ? b : a, reverse ? a : b, 2, 2)]);
        Point3D[] source = [new(0,-4,-.08),new(0,0,-.08),new(0,4,-.08)];
        var path = Assert.Single(field.Apply([new(source,Array.Empty<ToolpathSpan>(),0,"final-carve",null,detectCurves:false)]));
        const double slope = .1, nose = 1.5875, halfWidth = .5, roughRadius = 2;
        foreach(var original in source)
        {
            var point = path.Points.Single(p => p.X == original.X && p.Y == original.Y);
            // Independent closed-form contact: the actual swept flat cutter
            // makes h(x,y)=3+.1*(y+sqrt(2^2-x^2)). On either protected
            // channel wall x=+/- .5, minimize h plus spherical sagitta over y.
            double expected = 3+slope*(point.Y+Math.Sqrt(roughRadius*roughRadius-halfWidth*halfWidth))
                +nose-Math.Sqrt((nose*nose-halfWidth*halfWidth)*(1+slope*slope));
            Assert.InRange(-point.Z,expected-.007,expected+.00001);
            Assert.True(-point.Z > 3+slope*(point.Y+roughRadius)+.05,
                $"Tip does not actually cut below the sloping surface: {point}");
        }
        Assert.All(path.Points.Zip(path.Points.Skip(1)), pair => Assert.True(field.IsLegalChord(pair.First,pair.Second)));
        Assert.InRange(path.Points.Count,3,300);
    }

    [Fact]
    public void WideDetailReachesAbsoluteFinalDepthAndNeverAddsStartingDepthToTheCap()
    {
        var field = Field(Rectangle(-5, -8, 5, 8), Sweeps(6.35));
        Assert.Equal(12.7, field.AllowedDepth(0, 0));
        var path = Assert.Single(field.Apply([new(new Point3D[] {new(0, -1, -12.7), new(0, 1, -12.7)},
            Array.Empty<ToolpathSpan>(), 0, "final-carve", null, detectCurves: false)]));
        Assert.All(path.Points, p => Assert.InRange(-p.Z, 12.698, 12.7));
    }

    [Fact]
    public void ClearedSweepsNeverGrantPermissionToMoveTheCentreOffThePart()
    {
        var stock = new VCarveClearedStock(Sweeps(6.35), [6.35], Rectangle(-.25, -12, 12, 12));
        var field = new VCarveMaterialSurface([Rectangle(-.5, -8, .5, 8)], stock,
            Rounded, 12.7, 12.7, default, null);
        Assert.Equal(0, field.AllowedDepth(-.3, 0));
        Assert.False(field.IsLegalChord(new(-.3, -1, -6.4), new(-.3, 1, -6.4)));
    }

    [Fact]
    public void PriorStockKeepsNativePerimetersAndMedialSourcesWithoutSupplementalContours()
    {
        var original = new List<TaggedToolpath>();
        var fitted = new List<TaggedToolpath>();
        var stages = new List<string>();
        var previous = MillingGeometryTrace.Observer.Value;
        try
        {
            MillingGeometryTrace.Observer.Value = (stage, region, paths) => {
                previous?.Invoke(stage, region, paths);
                stages.Add(stage);
                if (stage == "vcarve-original-sources") original.AddRange(paths);
                if (stage == "vcarve-material-fitted") fitted.AddRange(paths);
            };
            VCarveToolpaths.Generate([new MillingRegion(Rectangle(-5, -8, 5, 8))],
                new ToolGeometry(3) { VCarveProfile = Rounded },
                new DepthSchedule(12.7, 2) { StartDepth = 12.7, SurfaceZ = 12.7 },
                new VCarveOptions { IncludeInteriorFill = false, DetectCurves = false,
                    CalculateSweptArea = false,
                    ClearedStock = new(Sweeps(6.35), [6.35], Rectangle(-12, -12, 12, 12)) },
                BoostVoronoiProvider.CreateDefault());
        }
        finally { MillingGeometryTrace.Observer.Value = previous; }

        Assert.Contains(original, p => p.Category == "clearing");
        Assert.Contains(original, p => p.Category == "final-carve");
        Assert.Equal(original.Count, fitted.Count);
        Assert.DoesNotContain("vcarve-shoulder-nominations", stages);
        foreach (var (source, result) in original.Zip(fitted))
        {
            Assert.Equal(source.Category, result.Category);
            Assert.Equal(source.DepthPassIndex, result.DepthPassIndex);
            if (source.Category != "final-carve") Assert.Equal(source.Points, result.Points);
            else
            {
                int next = 0;
                foreach (var point in result.Points)
                    if (next < source.Points.Count && point.X == source.Points[next].X
                        && point.Y == source.Points[next].Y) next++;
                Assert.Equal(source.Points.Count, next);
            }
        }
    }

    [Fact]
    public void PublicGenerationKeepsTheCompleteNarrowTerminalContactBelowTheLowerSurface()
    {
        var tool = new ToolGeometry(3) { VCarveProfile = Rounded };
        var region = new MillingRegion(Rectangle(-.5, -8, .5, 8));
        var options = new VCarveOptions { IncludeInteriorFill = false, DetectCurves = false,
            CalculateSweptArea = false, ClearedStock = new(Sweeps(6.35), [6.35], Rectangle(-12, -12, 12, 12)) };
        var plan = VCarveToolpaths.Generate([region], tool,
            new DepthSchedule(12.7, 2) { StartDepth = 12.7, SurfaceZ = 12.7 },
            options, BoostVoronoiProvider.CreateDefault());

        var cuts=plan.Toolpaths.Where(p=>p.Category=="final-carve").SelectMany(p=>
            (p.Spans.Count > 0 ? p.Spans : new[] { new ToolpathSpan(0, p.Points.Count - 1, ToolpathSpanKind.Cut) })
            .Where(s=>s.Kind==ToolpathSpanKind.Cut).SelectMany(s=>Enumerable.Range(s.StartIndex,s.MoveCount)
                .Select(i=>(A:p.Points[i],B:p.Points[i+1])))).ToArray();
        double expectedZ=6.35-(1.5875-Math.Sqrt(1.5875*1.5875-.5*.5));
        for(double y=-7;y<=7;y+=.5)
            Assert.Contains(cuts,c=>Math.Min(c.A.Y,c.B.Y)<=y&&Math.Max(c.A.Y,c.B.Y)>=y
                &&Math.Abs(c.A.X)<.01&&Math.Abs(c.B.X)<.01
                &&Math.Abs(c.A.Z-expectedZ)<.004&&Math.Abs(c.B.Z-expectedZ)<.004);
    }

    [Fact]
    public void PostedPlanarCellTransitionsDoNotCreateUnboundedDepthRefinement()
    {
        var field = Field(Rectangle(-1.59, -8, 1.59, 8), Sweeps(6.35));
        var path = Assert.Single(field.Apply([new(new Point3D[] {
            new(.001, -3, -1.5), new(.021, 3, -1.5)}, Array.Empty<ToolpathSpan>(),
            0, "final-carve", null, detectCurves: false)]));

        Assert.InRange(path.Points.Count, 2, 1000);
        Assert.All(path.Points.Zip(path.Points.Skip(1)), pair =>
            Assert.True(field.IsLegalChord(pair.First, pair.Second)));
    }

    [Fact]
    public void LegalEndpointsCannotBridgeProtectedMaterialBetweenSeparateOpenings()
    {
        var stock = new VCarveClearedStock(Sweeps(6.35), [6.35], Rectangle(-12,-12,12,12));
        var field = new VCarveMaterialSurface([
            Rectangle(-5,-5,-1,5), Rectangle(1,-5,5,5)], stock, Rounded, 12.7, 12.7, default, null);
        Assert.True(field.IsLegalChord(new(-3,0,-6.5),new(-3,1,-6.5)));
        Assert.True(field.IsLegalChord(new(3,0,-6.5),new(3,1,-6.5)));
        Assert.False(field.IsLegalChord(new(-3,0,-6.5),new(3,0,-6.5)));
    }

    private static VCarveMaterialSurface Field(PointD[] target, IReadOnlyList<VCarveRoughingChord> chords)
        => new([target], new(chords, [6.35], Rectangle(-12, -12, 12, 12)), Rounded, 12.7, 12.7, default, null);

    [Fact]
    public void VerticalWithdrawalRetainsTheCompleteDeepestCutterPoseProof()
    {
        var background=new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear,0,3,6)],6);
        var cutter=new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear,0,2,10)],10);
        var field=new VCarveMaterialSurface([Rectangle(-2.7,-.2,-.3,.2)],
            new VCarveClearedStock([],[],Rectangle(-12,-12,12,12),
                [new(background,[new(new(-3,0,-6),new(-3,0,-6),0)])]), cutter,8,12.7,default,null);
        var deep=new Point3D(-1.5,0,-3.5); var raised=new Point3D(-1.5,0,-1);
        Assert.True(field.IsLegalChord(deep,deep));
        Assert.True(field.IsLegalChord(deep,raised));
        Assert.True(field.IsLegalChord(raised,deep));
    }

    private static VCarveRoughingChord[] Sweeps(double depth, int lastX = 8)
        => Enumerable.Range(0, (lastX + 8) / 2 + 1).Select(i => new VCarveRoughingChord(
            new(-8 + i * 2, -10, -depth), new(-8 + i * 2, 10, -depth), 2, 2)).ToArray();

    private static PointD[] Rectangle(double left, double bottom, double right, double top)
        => [new(left, bottom), new(right, bottom), new(right, top), new(left, top)];
}
