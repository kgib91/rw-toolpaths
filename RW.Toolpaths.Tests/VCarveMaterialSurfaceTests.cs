using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class VCarveMaterialSurfaceTests
{
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
    public void ShoulderNominationsAcrossTheOriginalEdgeStillPointIntoTheFilledTarget()
    {
        var field=Field(Rectangle(-5,-8,5,8),Sweeps(6.35));
        // A simplified concave front can cross to the protected side. The
        // source is only a nomination; its side must not reverse compensation.
        var nomination=new TaggedToolpath(new Point3D[]{new(-5.001,-3,-12.7),new(-5.001,3,-12.7)},
            [],0,"clearing",0,detectCurves:false);
        var paths=field.Apply([nomination],new HashSet<TaggedToolpath>{nomination});
        Assert.NotEmpty(paths);
        double radius=1.5875+(6.35-1.5875)*(3-1.5875)/20;
        Assert.All(paths.SelectMany(p=>p.Points),p=> {
            Assert.InRange(p.X,-5+radius,-5+radius+.004);
            Assert.Equal(-12.7,p.Z,8);
        });
        var independent=Field(Rectangle(-5,-8,5,8),Sweeps(6.35));
        Assert.All(paths.SelectMany(p=>p.Points.Zip(p.Points.Skip(1))),
            chord=>Assert.True(independent.IsLegalChord(chord.First,chord.Second)));
    }

    [Fact]
    public void BoundedCurvedNominationsRetainPhysicalShoulderContactAtOriginalEdges()
    {
        var target=Enumerable.Range(0,200).Select(i=>new PointD(6*Math.Cos(i*Math.PI/100),
            8*Math.Sin(i*Math.PI/100))).ToArray();
        var nominations=VCarveToolpaths.BuildShoulderBoundary(new(target),0).Select(r=> {
            var points=r.Select(p=>new Point3D(p.x,p.y,-12.7)).ToList();points.Add(points[0]);
            return new TaggedToolpath(points,[],0,"clearing",0,detectCurves:false);
        }).ToList();
        var paths=Field(target,Sweeps(6.35)).Apply(nominations,new HashSet<TaggedToolpath>(nominations));
        Assert.NotEmpty(paths);
        var chords=paths.SelectMany(p=>p.Points.Zip(p.Points.Skip(1))).ToArray();
        // Independent floor-plane taper footprint; every original edge gets a
        // contact check, regardless of which nomination vertices survived.
        foreach(var edge in target.Zip(target.Skip(1).Append(target[0])))
        {
            double x=(edge.First.x+edge.Second.x)/2,y=(edge.First.y+edge.Second.y)/2;
            double gap=chords.Min(c=> {
                double dx=c.Second.X-c.First.X,dy=c.Second.Y-c.First.Y;
                double l2=dx*dx+dy*dy;
                double t=l2>0?Math.Clamp(((x-c.First.X)*dx+(y-c.First.Y)*dy)/l2,0,1):0;
                double penetration=-(c.First.Z+t*(c.Second.Z-c.First.Z))-6.35;
                double radius=penetration<=0?0:penetration<1.5875
                    ?Math.Sqrt(penetration*(3.175-penetration))
                    :1.5875+(penetration-1.5875)*(3-1.5875)/20;
                return Math.Sqrt(Math.Pow(x-c.First.X-t*dx,2)+Math.Pow(y-c.First.Y-t*dy,2))-radius;
            });
            Assert.InRange(gap,-.00001,MachiningAccuracy.SurfaceDeviation);
        }
        var independent=Field(target,Sweeps(6.35));
        Assert.All(chords,c=>Assert.True(independent.IsLegalChord(c.First,c.Second)));
    }

    [Fact]
    public void CurvedShoulderNominationsRetainContactWithTheOriginalBoundary()
    {
        var source=Enumerable.Range(0,200).Select(i=>new PointD(6*Math.Cos(i*Math.PI/100),
            8*Math.Sin(i*Math.PI/100))).ToArray();
        var front=VCarveToolpaths.BuildShoulderBoundary(new(source),0);
        double Distance(PointD p,PointD a,PointD b)
        {
            double dx=b.x-a.x,dy=b.y-a.y;
            double t=Math.Clamp(((p.x-a.x)*dx+(p.y-a.y)*dy)/(dx*dx+dy*dy),0,1);
            return Math.Sqrt(Math.Pow(p.x-a.x-t*dx,2)+Math.Pow(p.y-a.y-t*dy,2));
        }
        for(int i=0;i<source.Length;i++)
        {
            var a=source[i];var b=source[(i+1)%source.Length];var middle=new PointD((a.x+b.x)/2,(a.y+b.y)/2);
            double distance=front.Min(r=>Enumerable.Range(0,r.Count).Min(j=>Distance(middle,r[j],r[(j+1)%r.Count])));
            // Only nomination geometry uses this budget. Physical fitting
            // independently projects onto and protects the original boundary.
            double error=MachiningAccuracy.SurfaceDeviation/2+1/PathUtils.Scale;
            Assert.InRange(distance,Math.Max(0,VCarveMaterialSurface.PlanarReserve-error),VCarveMaterialSurface.PlanarReserve+error);
        }
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
    public void PublicGenerationKeepsTheCompleteNarrowTerminalContactBelowTheLowerSurface()
    {
        var tool = new ToolGeometry(3) { VCarveProfile = Rounded };
        var region = new MillingRegion(Rectangle(-.5, -8, .5, 8));
        var options = new VCarveOptions { IncludeInteriorFill = false, DetectCurves = false,
            CalculateSweptArea = false, ClearedStock = new(Sweeps(6.35), [6.35], Rectangle(-12, -12, 12, 12)) };
        var plan = VCarveToolpaths.Generate([region], tool,
            new DepthSchedule(12.7, 2) { StartDepth = 12.7, SurfaceZ = 12.7 },
            options, BoostVoronoiProvider.CreateDefault());

        // Terminal contact belongs to the same coherent contour family. Its
        // source can link to the preceding lap without becoming a separate
        // legacy medial/category path. Require its complete deepest flat Cut
        // and the same actual material section, independent of that grouping.
        var laps=plan.Toolpaths.Where(p=>p.Category=="clearing").SelectMany(p=>p.Spans
            .Where(s=>s.Kind==ToolpathSpanKind.Cut).Select(s=>p.Points.Skip(s.StartIndex).Take(s.PointCount).ToArray()))
            .Where(p=>p.Length>=4&&p[0]==p[^1]&&p.All(q=>q.Z==p[0].Z)).ToArray();
        Assert.NotEmpty(laps);
        double deepest=laps.Min(p=>p[0].Z);
        var detail=Assert.Single(laps.Where(p=>p[0].Z==deepest));
        Assert.All(detail,p=>Assert.InRange(p.Z,6.25,6.351));
        Assert.Contains(CutContactsAtYZero(plan,"clearing"),p=>Math.Abs(p.X)<.01
            &&p.Z>=6.25&&p.Z<=6.351);
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

    [Theory]
    [InlineData(8, 6.35)]
    [InlineData(-2, 6.35)]
    [InlineData(8, 12.7)]
    [InlineData(-2, 12.7)]
    public void LoweredShoulderFrontReachesStraightEdgesWithoutLosingTallWallProtection(int lastX, double scheduledStart)
    {
        const double floor = 6.35;
        var stock = new VCarveClearedStock(Sweeps(floor, lastX), [floor], Rectangle(-12, -12, 12, 12));
        var target = Rectangle(-5, -8, 5, 8);
        var plan = VCarveToolpaths.Generate([new(target)], new ToolGeometry(3) { VCarveProfile = Rounded },
            new DepthSchedule(12.7, 6.35) { StartDepth = scheduledStart, SurfaceZ = 12.7 },
            new VCarveOptions { IncludeInteriorFill = false, DetectCurves = false,
                CalculateSweptArea = false, ClearedStock = stock },
            BoostVoronoiProvider.CreateDefault());
        // Independently evaluate the actual taper radius at the cleared plane.
        double radius = 1.5875 + (floor - 1.5875) * (3 - 1.5875) / 20;
        var cuts = plan.Toolpaths.Where(p => p.Category == "clearing").SelectMany(p =>
            p.Points.Zip(p.Points.Skip(1))).ToArray();
        foreach (double sign in new[] { -1d, 1d })
        {
            double contactRadius = lastX < 0 && sign > 0
                ? 1.5875 + (12.7 - 1.5875) * (3 - 1.5875) / 20 : radius;
            Assert.True(cuts.Any(chord =>
                Math.Abs(chord.First.Z) < .02 && Math.Abs(chord.Second.Z) < .02
                && Math.Min(chord.First.Y, chord.Second.Y) <= 0 && Math.Max(chord.First.Y, chord.Second.Y) >= 0
                && Math.Abs(chord.First.X - sign * (5 - contactRadius)) < .004
                && Math.Abs(chord.Second.X - sign * (5 - contactRadius)) < .004),
                $"Missing contact at x={sign * (5-contactRadius):R}; " + string.Join("; ", plan.Toolpaths
                    .Where(p=>p.Category=="clearing").Select(p=>$"points={p.Points.Count} x={p.Points.Min(q=>q.X):R}..{p.Points.Max(q=>q.X):R} minZ={p.Points.Min(q=>q.Z):R}")));
        }
        var guard = new VCarveMaterialSurface([target], stock, Rounded, 12.7, 12.7, default, null);
        foreach (var path in plan.Toolpaths)
        foreach (var chord in path.Points.Zip(path.Points.Skip(1)))
            Assert.True(guard.IsLegalChord(chord.First with { Z = chord.First.Z - 12.7 },
                chord.Second with { Z = chord.Second.Z - 12.7 }));
    }

    [Fact]
    public void FittingClearingFrontDoesNotDeepenItsScheduledPass()
    {
        var field = Field(Rectangle(-.5, -8, .5, 8), Sweeps(6.35));
        var nomination = new TaggedToolpath(new Point3D[] { new(0,-3,-6.4), new(0,3,-6.4) },
            Array.Empty<ToolpathSpan>(), 0, "clearing", 0, detectCurves:false);
        var path = Assert.Single(field.Apply([nomination], new HashSet<TaggedToolpath> { nomination }));
        Assert.All(path.Points, point => Assert.InRange(point.Z, -6.4, -6.399));
    }

    [Fact]
    public void NarrowMixedHeightChannelContactsTheLowerEdgeByShiftingItsCutterAxis()
    {
        var stock = new VCarveClearedStock(Sweeps(6.35, -2), [6.35], Rectangle(-12,-12,12,12));
        var plan = VCarveToolpaths.Generate([new(Rectangle(-1.9,-8,1.9,8))],
            new ToolGeometry(3) { VCarveProfile = Rounded },
            new DepthSchedule(12.7,6.35) { StartDepth=12.7, SurfaceZ=12.7 },
            new VCarveOptions { IncludeInteriorFill=false, DetectCurves=false, CalculateSweptArea=false, ClearedStock=stock },
            BoostVoronoiProvider.CreateDefault());
        var contacts = plan.Toolpaths.Where(p=>p.Category=="clearing").SelectMany(p=>p.Points.Zip(p.Points.Skip(1)))
            .Where(c=>Math.Min(c.First.Y,c.Second.Y)<=0 && Math.Max(c.First.Y,c.Second.Y)>=0
                && Math.Abs(c.Second.Y-c.First.Y)>1e-12)
            .Select(c=> {
                double t=-c.First.Y/(c.Second.Y-c.First.Y);
                return new Point3D(c.First.X+t*(c.Second.X-c.First.X),0,c.First.Z+t*(c.Second.Z-c.First.Z));
            }).ToArray();
        // Independent taper contact equations: outside = R(depth-6.35),
        // inside = R(depth); their sum must fit the 3.8-mm opening.
        double expectedOuterRadius=(3.8-6.35*(3-1.5875)/20)/2;
        Assert.Contains(contacts,p=>Math.Abs(p.X-(-1.9+expectedOuterRadius))<.005
            && 12.7-p.Z>8.9 && 12.7-p.Z<9.2);
        Assert.Contains(contacts,p=> {
            double penetration=12.7-p.Z-6.35;
            if(penetration<=0)return false;
            double footprint=penetration<=1.5875 ? Math.Sqrt(penetration*(3.175-penetration))
                : 1.5875+(penetration-1.5875)*(3-1.5875)/20;
            return Math.Abs(p.X+1.9-footprint)<.006;
        });
    }

    [Fact]
    public void StraightBallContactsALoweredEdgeWhenItsFullDiameterCannotFitTheOpening()
    {
        var ball = new VCarveCutterProfile([new(CarvingCutterSectionKind.Hemisphere,0,1,1),
            new(CarvingCutterSectionKind.Linear,1,1,10)],11);
        var plan = VCarveToolpaths.Generate([new(Rectangle(-.6,-8,.6,8))], new ToolGeometry(1) { VCarveProfile=ball },
            new DepthSchedule(10,5) { StartDepth=10, SurfaceZ=12.7 },
            new VCarveOptions { IncludeInteriorFill=false, DetectCurves=false, CalculateSweptArea=false,
                ClearedStock=new(Sweeps(6.35,-2),[6.35],Rectangle(-12,-12,12,12)) }, BoostVoronoiProvider.CreateDefault());
        double expectedDepth=6.35+1-Math.Sqrt(1-.2*.2);
        var points=CutContactsAtYZero(plan,"clearing");
        Assert.True(points.Any(p=>Math.Abs(p.Y)<.5 && Math.Abs(p.X+.4)<.005 && Math.Abs(12.7-p.Z-expectedDepth)<.002),
            "Actual F3 cutting-chord contacts at Y=0: "+string.Join("; ",points.Take(15)));
    }

    [Fact]
    public void RefusedShoulderContactAddsNoCutAndNeverCreditsADeclaredFloor()
    {
        var field=Field(Rectangle(-.5,-8,.5,8), []);
        var source=new TaggedToolpath(new Point3D[] {new(-.498,-3,-12.7),new(-.498,3,-12.7)},
            Array.Empty<ToolpathSpan>(),0,"clearing",0,detectCurves:false);
        Assert.Empty(field.Apply([source],new HashSet<TaggedToolpath> { source }));
    }

    [Theory]
    [InlineData(12.7)]
    [InlineData(8)]
    public void CylinderUsesExecutedOutsideClearanceToFinishAStripNarrowerThanItsDiameter(double priorDepth)
    {
        var cylinder=new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear,1,1,20)],20);
        var plan=VCarveToolpaths.Generate([new(Rectangle(-.6,-8,.6,8))],new ToolGeometry(1) { VCarveProfile=cylinder },
            new DepthSchedule(10,5) { StartDepth=10, SurfaceZ=12.7 },
            new VCarveOptions { IncludeInteriorFill=false,DetectCurves=false,CalculateSweptArea=false,
                ClearedStock=new(Sweeps(priorDepth,-2),[priorDepth],Rectangle(-12,-12,12,12)) },BoostVoronoiProvider.CreateDefault());
        Assert.Contains(CutContactsAtYZero(plan,"clearing"),
            p=>Math.Abs(p.Y)<.5 && Math.Abs(p.X+.4)<.005 && Math.Abs(p.Z-(12.7-Math.Min(10,priorDepth)))<.003);
    }

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

    // A complete original straight edge has the same contact when represented
    // by one chord or collinear subdivisions. Evaluate the actual F3 CUTTING
    // locus at the original section; neither arbitrary vertex density nor a
    // cleared-travel point establishes the physical contact requirement.
    private static Point3D[] CutContactsAtYZero(ToolpathPlan plan,string category)
    {
        var result=new List<Point3D>();
        foreach(var path in plan.Toolpaths.Where(p=>p.Category==category))
        foreach(var span in path.Spans.Where(s=>s.Kind is ToolpathSpanKind.Cut or ToolpathSpanKind.Ramp))
        for(int i=span.StartIndex+1;i<=span.EndIndex;i++)
        {
            Point3D Posted(Point3D p)=>new(ToolpathCommandRounding.Round((float)p.X),
                ToolpathCommandRounding.Round((float)p.Y),ToolpathCommandRounding.Round((float)p.Z));
            var a=Posted(path.Points[i-1]);var b=Posted(path.Points[i]);
            if(Math.Min(a.Y,b.Y)>0||Math.Max(a.Y,b.Y)<0)continue;
            if(a.Y==b.Y){result.Add(a);result.Add(b);continue;}
            double t=-a.Y/(b.Y-a.Y);
            result.Add(new(a.X+t*(b.X-a.X),0,a.Z+t*(b.Z-a.Z)));
        }
        return result.ToArray();
    }

    private static VCarveRoughingChord[] Sweeps(double depth, int lastX = 8)
        => Enumerable.Range(0, (lastX + 8) / 2 + 1).Select(i => new VCarveRoughingChord(
            new(-8 + i * 2, -10, -depth), new(-8 + i * 2, 10, -depth), 2, 2)).ToArray();

    private static PointD[] Rectangle(double left, double bottom, double right, double top)
        => [new(left, bottom), new(right, bottom), new(right, top), new(left, top)];
}
