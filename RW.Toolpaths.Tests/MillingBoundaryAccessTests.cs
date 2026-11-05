using Clipper2Lib;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class MillingBoundaryAccessTests
{
    [Fact]
    public void AlreadyOpenFinishWallsReuseThePreparedRoughingFieldInAnIndependentScope()
    {
        var voidHole=Box(4,5,5,10);
        var regions=new[]{new MillingRegion(Box(0,0,20,20),[voidHole,Box(9,7,11,13)])};
        var access=new MillingBoundaryAccess([new MillingRegion(voidHole)],Box(-5,-5,25,25));
        var tool=new ToolGeometry(6){BottomRadius=0,TopRadius=6,ConeLength=6,TipAngleRadians=Math.PI/2};
        var preparation=VCarveToolpaths.Prepare(regions,tool,new(3,1),
            new VCarveOptions{SmoothVCarving=true,SmoothRampAngleDegrees=30,BoundaryAccess=access},
            BoostVoronoiProvider.CreateDefault());
        var analysis=preparation.AnalyzePocketPlateau(3,30,outsideAccess:access,rampOutsideEdges:false);
        Assert.NotSame(preparation,analysis.Preparation);
        Assert.False(analysis.Preparation.HasCompletedFinish);
        var pairs=preparation.Regions.Zip(analysis.Preparation.Regions).ToArray();
        Assert.Contains(pairs,p=>p.First.OriginalMedialPaths.Count>0
            &&ReferenceEquals(p.First.OriginalMedialPaths,p.Second.OriginalMedialPaths));
        Assert.All(pairs,p=>Assert.Equal(p.First.SmoothedMedialPaths.SelectMany(r=>r.Points),
            p.Second.SmoothedMedialPaths.SelectMany(r=>r.Points)));
    }

    [Fact]
    public void SelectedVoidContinuesExistingCornerRampAcrossItsBoundary()
    {
        IReadOnlyList<PointD> triangle = [new(0,0), new(20,0), new(0,20)];
        var voidHole = Box(3.5,3.5,4.5,4.5);
        var tool = new ToolGeometry(6) { BottomRadius=0, TopRadius=6, ConeLength=6, TipAngleRadians=Math.PI/2 };
        var depth = new DepthSchedule(4,4);
        var options = new VCarveOptions { SmoothVCarving=true, SmoothRampAngleDegrees=30,
            RampOutsideEdges=false, DetectCurves=false, CalculateSweptArea=false,
            BoundaryAccess=new([new MillingRegion(voidHole)],Box(-10,-10,30,30)) };
        var source = new[] { new MillingRegion(triangle, [voidHole]) };
        var preparation = VCarveToolpaths.Prepare(source,tool,depth,options,BoostVoronoiProvider.CreateDefault());
        // The existing ramp from the acute corner must pass continuously
        // through the selected internal edge. Its independent analytical
        // depth on x=y is tan(30 degrees)*x, capped at four mm: the
        // complete straight edges own the crest, rather than the corner point.
        var rampPoints = preparation.Regions.SelectMany(r=>r.SmoothedMedialPaths)
            .SelectMany(p=>p.Points).Where(p=>p.X>=3.5&&p.X<=4.5&&Math.Abs(p.X-p.Y)<1e-8).ToArray();
        Assert.NotEmpty(rampPoints);
        Assert.All(rampPoints,p=>{
            double expected=Math.Min(4,Math.Tan(Math.PI/6)*p.X);
            Assert.InRange(-p.Z,expected-VCarvePreparation.GeometricErrorBound,expected+1e-10);
        });
        var plan = VCarveToolpaths.Generate(source,tool,depth,options with {Preparation=preparation},BoostVoronoiProvider.CreateDefault());
        var cuts = plan.Toolpaths.SelectMany(p=>p.Points).Where(p=>p.X>=3.5&&p.X<=4.5
            &&p.Y>=3.5&&p.Y<=4.5&&p.Z<0).ToArray();
        Assert.NotEmpty(cuts);
        Assert.Contains(cuts,p=>-p.Z>2);
        Assert.All(cuts,p=>Assert.InRange(-p.Z,0,4));
        // Outside selection permits overlap; it does not remove the original
        // Void hole from the authored Background cutting target.
        Assert.Single(source[0].Holes);
        Assert.False(options.RampOutsideEdges);
    }

    [Fact]
    public void ApprovedClearanceInterfaceAlsoStopsAnchoringAnOutsideRamp()
    {
        var regions = new[] { new MillingRegion(Box(5,0,15,10)) };
        var tool = new ToolGeometry(6) { BottomRadius=0, TopRadius=6, ConeLength=6, TipAngleRadians=Math.PI/2 };
        var preparation = VCarveToolpaths.Prepare(regions,tool,new DepthSchedule(3,1),
            new VCarveOptions { SmoothVCarving=true, SmoothRampAngleDegrees=30 },BoostVoronoiProvider.CreateDefault());
        var access = new MillingBoundaryAccess(new[] {new MillingRegion(Box(0,0,5,10))},Box(0,0,20,20));
        var original = preparation.AnalyzePocketPlateau(3,30,rampOutsideEdges:true);
        var open = preparation.AnalyzePocketPlateau(3,30,outsideAccess:access,rampOutsideEdges:false);
        Assert.True(new PreparedRegionDistance(original.ProtectedBoundary).SignedDistance(new(5.5,.5))>0);
        Assert.True(new PreparedRegionDistance(open.ProtectedBoundary).SignedDistance(new(5.5,.5))<0);
    }

    [Theory]
    [InlineData(false,0)] [InlineData(true,0)]
    [InlineData(false,2)] [InlineData(true,2)]
    public void VCarveGenerationHonoursOutsideRampPolicy(bool rampOutside,double restWidth)
    {
        var tool = new ToolGeometry(2) { BottomRadius=0, TopRadius=2, ConeLength=2, TipAngleRadians=Math.PI/2 };
        var plan = VCarveToolpaths.Generate(new[] { new MillingRegion(Box(0,0,8,8)) },tool,new DepthSchedule(1,1),
            new VCarveOptions { SmoothVCarving=true, SmoothRampAngleDegrees=30, RampOutsideEdges=rampOutside,
                IncludeInteriorFill=false,RoughingRestWidth=restWidth,EdgeFillWidth=restWidth,
                BoundaryAccess=new(Array.Empty<MillingRegion>(),Box(0,0,8,8)), DetectCurves=false, CalculateSweptArea=false },
            BoostVoronoiProvider.CreateDefault());
        var points=plan.Toolpaths.SelectMany(p=>p.Points).Where(p=>p.Z<0).ToArray();
        if (!rampOutside)
        {
            // With no retained wall, outside permission creates no carving job.
            Assert.Empty(points);
            return;
        }
        Assert.NotEmpty(points);
        Assert.DoesNotContain(points,p=>p.X<0 || p.X>8 || p.Y<0 || p.Y>8);
        Assert.DoesNotContain(points,p=>p.X<.01 && p.Y<.01 && p.Z<-.999);
    }

    [Theory]
    [InlineData(0)] [InlineData(.3)]
    public void OutsideRampPolicyRemovesOnlyOpenAnchorsAndKeepsTheOriginalGraph(double finishAllowance)
    {
        var regions = new[] { new MillingRegion(Box(0,0,15,30)), new MillingRegion(Box(20,5,30,15)) };
        var tool = new ToolGeometry(6) { BottomRadius=0, TopRadius=6, ConeLength=6, TipAngleRadians=Math.PI/2 };
        var depth = new DepthSchedule(3,1);
        var preparation = VCarveToolpaths.Prepare(regions,tool,depth,
            new VCarveOptions { SmoothVCarving=true, SmoothRampAngleDegrees=30, StockToLeave=finishAllowance }, BoostVoronoiProvider.CreateDefault());
        var boundary = new MillingBoundaryAccess(Array.Empty<MillingRegion>(),Box(0,0,35,35));
        var closed = preparation.AnalyzePocketPlateau(3,30,rampOutsideEdges:true);
        var open = preparation.AnalyzePocketPlateau(3,30,outsideAccess:boundary,rampOutsideEdges:false);
        var closedMask = new PreparedRegionDistance(closed.ProtectedBoundary);
        var openMask = new PreparedRegionDistance(open.ProtectedBoundary);
        Assert.True(closedMask.SignedDistance(new(1,1))>0);
        Assert.True(openMask.SignedDistance(new(1,1))<0);
        Assert.True(closedMask.SignedDistance(new(21,6))>0);
        Assert.True(openMask.SignedDistance(new(21,6))>0);
        Assert.Equal(preparation.OriginalGeometryFingerprint,open.Preparation.OriginalGeometryFingerprint);
        Assert.Equal(preparation.Regions.SelectMany(r=>r.OriginalMedialPaths).SelectMany(p=>p.Points).Select(p=>(p.X,p.Y)),
            open.Preparation.Regions.SelectMany(r=>r.OriginalMedialPaths).SelectMany(p=>p.Points).Select(p=>(p.X,p.Y)));
        Assert.Equal(closed.ProtectedArea,preparation.AnalyzePocketPlateau(3,30,rampOutsideEdges:true).ProtectedArea);
        Assert.True(open.ProtectedArea<closed.ProtectedArea);
    }

    [Fact]
    public void InternalVoidRampChoiceKeepsForegroundWallsAndOriginalCuttingTarget()
    {
        var voidHole = Box(8, 7, 9, 13);
        var foreground = Box(22, 7, 23, 13);
        var regions = new[] { new MillingRegion(Box(0, 0, 32, 20), new[] { voidHole, foreground }) };
        var tool = new ToolGeometry(6) { BottomRadius = 0, TopRadius = 6, ConeLength = 6, TipAngleRadians = Math.PI / 2 };
        var depth = new DepthSchedule(3, 1);
        var access = new MillingBoundaryAccess([new MillingRegion(voidHole)], Box(-5, -5, 37, 25));
        var options = new VCarveOptions { SmoothVCarving = true, SmoothRampAngleDegrees = 30, BoundaryAccess = access };
        var noRamp = VCarveToolpaths.Prepare(regions, tool, depth, options, BoostVoronoiProvider.CreateDefault());
        var ramp = VCarveToolpaths.Prepare(regions, tool, depth, options with { RampOutsideEdges = true }, BoostVoronoiProvider.CreateDefault());
        var openMask = new PreparedRegionDistance(noRamp.AnalyzePocketPlateau(3, 30).ProtectedBoundary);
        var rampMask = new PreparedRegionDistance(ramp.AnalyzePocketPlateau(3, 30).ProtectedBoundary);
        Assert.True(openMask.SignedDistance(new(9.5, 7.5)) < 0);
        Assert.True(rampMask.SignedDistance(new(9.5, 7.5)) > 0);
        var physicalAccess = new PreparedRegionDistance(ramp.OriginalRegions.SelectMany(r => r.AsRingGroup()));
        Assert.True(physicalAccess.SignedDistance(new(8.5, 10)) > 0);
        Assert.True(physicalAccess.SignedDistance(new(22.5, 10)) < 0);
        Assert.Equal(noRamp.OriginalGeometryFingerprint, ramp.OriginalGeometryFingerprint);
        Assert.Equal(2, Assert.Single(regions).Holes.Count);
    }

    [Fact]
    public void ExplicitOutsideRampsRestoreAnchorsOnAnOpenVCarveSurface()
    {
        var regions = new[] { new MillingRegion(Box(0,0,20,20)) };
        var tool = new ToolGeometry(6) { BottomRadius=0, TopRadius=6, ConeLength=6, TipAngleRadians=Math.PI/2 };
        var depth = new DepthSchedule(3,1);
        var options = new VCarveOptions { SmoothVCarving=true, SmoothRampAngleDegrees=30,
            BoundaryAccess=new(Array.Empty<MillingRegion>(),Box(0,0,20,20)) };
        var noRamp = VCarveToolpaths.Prepare(regions,tool,depth,options,BoostVoronoiProvider.CreateDefault());
        var ramp = VCarveToolpaths.Prepare(regions,tool,depth,options with { RampOutsideEdges=true },BoostVoronoiProvider.CreateDefault());
        Assert.True(noRamp.AnalyzePocketPlateau(3,30).ProtectedArea<.001);
        Assert.True(ramp.AnalyzePocketPlateau(3,30).ProtectedArea>1);
        Assert.NotEqual(noRamp.Fingerprint,ramp.Fingerprint);
    }

    [Theory]
    [InlineData(false,0)] [InlineData(true,0)]
    [InlineData(false,.4)] [InlineData(true,.4)]
    [InlineData(false,-.25)] [InlineData(true,-.25)]
    public void OpenPocketKeepsStraightTargetBorderAndCompleteReachableInterior(bool scanline,double allowance)
    {
        var target = new[] { new MillingRegion(Box(0,0,20,12), new[] { Box(7,4,13,8) }) };
        var options = new PocketOptions { BoundaryAccess = Access(), StepOver = .4, StockToLeave=allowance,
            CalculateSweptArea = false, DetectCurves = false };
        var tool = ToolGeometry.Flat(1);
        var depth = new DepthSchedule(1,1);
        var plan = scanline ? ScanlinePocketToolpaths.Generate(target,tool,depth,options)
            : PocketToolpaths.Generate(target,tool,depth,options);
        var chords = plan.Toolpaths.SelectMany(path => Enumerable.Range(1,path.Points.Count-1)
            .Select(i => (A:path.Points[i-1], B:path.Points[i]))).ToArray();
        // The original straight edge guides the first front, even with a nearby island.
        Assert.Contains(chords, c => Math.Abs(c.A.Y)<.003 && Math.Abs(c.B.Y)<.003
            && Math.Min(c.A.X,c.B.X)<1 && Math.Max(c.A.X,c.B.X)>19);
        var island = new PreparedRegionDistance(new[] { (IReadOnlyList<PointD>)Box(7,4,13,8) });
        int checkedSites = 0;
        for(double y=.25;y<12;y+=.5)
        for(double x=.25;x<20;x+=.5)
        {
            var point = new PointD(x,y);
            if(island.SignedDistance(point)>-1.05-Math.Max(0,allowance)) continue;
            Assert.True(Covers(plan,point,1,1), $"Reachable original pocket omitted at ({x},{y}); scanline={scanline}");
            checkedSites++;
        }
        Assert.True(checkedSites>500);
        Assert.DoesNotContain(plan.Toolpaths.SelectMany(p=>p.Points),p=>p.Z<0
            && (p.X < 0 || p.X > 20 || p.Y < 0 || p.Y > 12));
    }

    [Fact]
    public void PhysicalAirPermissionCannotAuthorizeACentreOutsideThePlatform()
    {
        var target=new[]{new MillingRegion(Box(0,0,20,12))};
        var tool=ToolGeometry.Flat(1);var depth=new DepthSchedule(1,1);
        var access=Access().Resolve(target,tool,depth,false,30,0,default);
        var outside=new TaggedToolpath(new[]{new Point3D(-.1,3,-1),new Point3D(-.1,9,-1)},0,"pocket",0);
        var error=Assert.Throws<InvalidOperationException>(()=>access.Verify(new[]{outside},tool,depth,0,default));
        Assert.Contains("centre leaves the platform",error.Message);
    }

    [Theory]
    [InlineData(false)] [InlineData(true)]
    public void OpenPlatformStillHonoursANarrowerCallerClip(bool scanline)
    {
        var target=new[]{new MillingRegion(Box(0,0,20,12))};
        var options=new PocketOptions {BoundaryAccess=Access(),ClipBoundary=Box(2,2,18,10),
            CalculateSweptArea=false,DetectCurves=false,StepOver=.4};
        var tool=ToolGeometry.Flat(1);var depth=new DepthSchedule(1,1);
        var plan=scanline?ScanlinePocketToolpaths.Generate(target,tool,depth,options)
            :PocketToolpaths.Generate(target,tool,depth,options);
        Assert.NotEmpty(plan.Toolpaths);
        Assert.All(plan.Toolpaths.SelectMany(p=>p.Points),p=>{
            Assert.InRange(p.X,2,18);Assert.InRange(p.Y,2,10);
        });
    }

    [Fact]
    public void RemoteClearanceExtentCannotReshapePocketFronts()
    {
        var target = new[] { new MillingRegion(Box(8,8,20,20)) };
        var tool = ToolGeometry.Flat(1);
        var depth = new DepthSchedule(1,1);
        ToolpathPlan Generate(double extent) => PocketToolpaths.Generate(target,tool,depth,
            new PocketOptions { BoundaryAccess = new(new[] { new MillingRegion(Box(8-extent,8-extent,20+extent,20+extent)) },
                Box(0,0,40,40)), StepOver=.4, CalculateSweptArea=false, DetectCurves=false });
        var nearby = Generate(3);
        var remote = Generate(7);
        Assert.NotEmpty(nearby.Toolpaths);
        Assert.Equal(nearby.Toolpaths.SelectMany(p=>p.Points),remote.Toolpaths.SelectMany(p=>p.Points));
    }

    [Fact]
    public void ThinFrameDoesNotRequireOffsettingTheRemoteApprovedInterior()
    {
        var target=new[]{new MillingRegion(Box(0,0,100,100),new[]{Box(1,1,99,99)})};
        var permissions=new MillingBoundaryAccess(new[]{new MillingRegion(Box(1,1,99,99))},Box(0,0,100,100));
        var tool=ToolGeometry.Flat(1);var depth=new DepthSchedule(1,1);
        var domain=permissions.Resolve(target,tool,depth,false,30,0,default);
        var guide=domain.PocketRegions(1,0,default);
        Assert.DoesNotContain(guide.SelectMany(r=>r.Outer),p=>p.x < -1.001 || p.x > 101.001 || p.y < -1.001 || p.y > 101.001);
        var plan=PocketToolpaths.Generate(target,tool,depth,new PocketOptions {BoundaryAccess=permissions,
            StepOver=.5,CalculateSweptArea=false,DetectCurves=false},default);
        foreach(var p in new[]{new PointD(.5,30),new(99.5,30),new(50,.5),new(50,99.5)})
            Assert.True(Covers(plan,p,1,1),$"Frame coverage lost at {p}");
        Assert.DoesNotContain(plan.Toolpaths.SelectMany(p=>p.Points),p=>p.Z<0&&p.X>40&&p.X<60&&p.Y>40&&p.Y<60);
    }
    private static List<PointD> Box(double x,double y,double right,double top) =>
        new() { new(x,y),new(right,y),new(right,top),new(x,top) };
    private static MillingBoundaryAccess Access(params MillingRegion[] clearance) => new(clearance,Box(0,0,20,12));
    private static double Distance(PointD p, Point3D a, Point3D b)
    {
        double dx=b.X-a.X,dy=b.Y-a.Y,d=dx*dx+dy*dy;
        double t=d==0?0:Math.Clamp(((p.x-a.X)*dx+(p.y-a.Y)*dy)/d,0,1);
        return Math.Sqrt(Math.Pow(p.x-a.X-t*dx,2)+Math.Pow(p.y-a.Y-t*dy,2));
    }
    private static bool Covers(ToolpathPlan plan,PointD point,double radius,double depth) => plan.Toolpaths.Any(path=>
        Enumerable.Range(1,path.Points.Count-1).Any(i=>path.Points[i-1].Z<=-depth+.002
            && path.Points[i].Z<=-depth+.002 && Distance(point,path.Points[i-1],path.Points[i])<=radius+.002));

    [Theory]
    [InlineData(false)] [InlineData(true)]
    public void ExteriorOverhangKeepsTheCentreOnThePlatform(bool scanline)
    {
        var target=new[]{new MillingRegion(Box(0,0,4,12))};
        var tool=ToolGeometry.Flat(3); var schedule=new DepthSchedule(1,1);
        var options=new PocketOptions {BoundaryAccess=Access(),CalculateSweptArea=false,DetectCurves=false,StepOver=.5};
        var plan=scanline?ScanlinePocketToolpaths.Generate(target,tool,schedule,options):PocketToolpaths.Generate(target,tool,schedule,options);
        Assert.NotEmpty(plan.Toolpaths);
        Assert.True(Covers(plan,new(2,6),3,1));
        Assert.Contains(plan.Toolpaths.SelectMany(p=>p.Points),p=>p.X<.01 && p.Z<0);
        Assert.All(plan.Toolpaths.SelectMany(p=>p.Points),p=>Assert.InRange(p.X,0,20));
        // A real wall still starts at x=4 within the stock. Analytic middle-strip check.
        Assert.All(plan.Toolpaths.SelectMany(p=>p.Points).Where(p=>p.Y>=3 && p.Y<=9 && p.Z<0),
            p=>Assert.True(p.X+3<=4.002,$"Protected wall gouged at {p}"));
        var unreachable=new[]{new MillingRegion(Box(0,0,2,12))};
        var rejected=scanline?ScanlinePocketToolpaths.Generate(unreachable,tool,schedule,options)
            :PocketToolpaths.Generate(unreachable,tool,schedule,options);
        Assert.Empty(rejected.Toolpaths);
    }

    [Fact]
    public void ApprovedAdjacentRegionOpensTunnelButDisconnectedRegionCannotBridgeProtectedMaterial()
    {
        var targets=new[]{new MillingRegion(Box(5,0,6,12))};
        var tool=ToolGeometry.Flat(3);var depth=new DepthSchedule(1,1);
        var closed=Access(new MillingRegion(Box(0,0,4,12))).Resolve(targets,tool,depth,false,30,0,default);
        var opened=Access(new MillingRegion(Box(0,0,5,12))).Resolve(targets,tool,depth,false,30,0,default);
        var closedDistance=new PreparedRegionDistance(closed.Regions.SelectMany(r=>r.AsRingGroup()));
        var openDistance=new PreparedRegionDistance(opened.Regions.SelectMany(r=>r.AsRingGroup()));
        Assert.True(closedDistance.SignedDistance(new(3,6))<3);
        Assert.True(openDistance.SignedDistance(new(3,6))>=3-.0011);
        Assert.NotEqual(closed.Signature,opened.Signature);
    }

    [Theory]
    [InlineData(false,0d)] [InlineData(true,0d)]
    [InlineData(false,.4)] [InlineData(true,-.2)]
    public void ActualTunnelSweepsUseApprovedAirAndRespectTheAuthoredProtectedWallAllowance(bool scanline,double allowance)
    {
        var target=new[]{new MillingRegion(Box(5,0,6,12))};var tool=ToolGeometry.Flat(3);var depth=new DepthSchedule(1,.5);
        var options=new PocketOptions {BoundaryAccess=Access(new MillingRegion(Box(0,0,5,12))),
            StockToLeave=allowance,CalculateSweptArea=false,DetectCurves=false,StepOver=.5};
        var plan=scanline?ScanlinePocketToolpaths.Generate(target,tool,depth,options):PocketToolpaths.Generate(target,tool,depth,options);
        Assert.True(Covers(plan,new(5.5,6),3,1));
        Assert.All(plan.Toolpaths.SelectMany(p=>p.Points).Where(p=>p.Y>=3 && p.Y<=9 && p.Z<0),
            p=>Assert.True(p.X+3<=6-allowance+.002,$"Protected wall allowance changed at {p}"));
        Assert.DoesNotContain(plan.Toolpaths.SelectMany(p=>p.Points),p=>p.X < 1.99);
    }

    [Fact]
    public void APointContactDoesNotCreateCutterWidthClearance()
    {
        var targets=new[]{new MillingRegion(Box(8,4,9,8))};
        var resolved=Access(new MillingRegion(Box(2,0,8,4))).Resolve(targets,ToolGeometry.Flat(1),new(1,1),false,30,0,default);
        var distance=new PreparedRegionDistance(resolved.Regions.SelectMany(r=>r.AsRingGroup()));
        Assert.True(distance.SignedDistance(new(8.5,6))<1);
    }

    [Fact]
    public void PermissionFingerprintBindsCoordinatesEvenWhenTopologyAndVertexCountsMatch()
    {
        var air=Access(new MillingRegion(Box(0,0,20,12)));
        var a=air.Resolve(new[]{new MillingRegion(Box(5,0,6,12))},ToolGeometry.Flat(1),new(1,1),false,30,0,default);
        var b=air.Resolve(new[]{new MillingRegion(Box(7,0,8,12))},ToolGeometry.Flat(1),new(1,1),false,30,0,default);
        Assert.NotEqual(a.Signature,b.Signature);
    }

    [Fact]
    public void ParallelPlateauLeafUnionsRetainTheCompleteOrderedIntegerMask()
    {
        var original=Enumerable.Range(0,700).Select(i=>new TaggedToolpath(new Point3D[]{
            new(i%25*.7,i/25*.7,-.3),new(i%25*.7+.7,i/25*.7+.3,-1.5)},0,"final-carve",null)).ToArray();
        var limited=original.Select(p=>new TaggedToolpath(p.Points.Select(v=>v with{Z=Math.Max(v.Z,-.5)}).ToArray(),0,"final-carve",null)).ToArray();
        IReadOnlyList<PointD>[] boundary=[Box(-5,-5,25,25)];
        var profile=new VCarveCutterProfile([new(CarvingCutterSectionKind.Hemisphere,0,1,1),
            new(CarvingCutterSectionKind.Linear,1,2,10)],11);
        var literal=SmoothRoughingPlateau.Build(original,boundary,2,1,default,limited,profile,boundary,
            parallelLeafUnions:false,reuseTargetContact:false,indexDiscDominance:false);
        var parallel=SmoothRoughingPlateau.Build(original,boundary,2,1,default,limited,profile,boundary,
            parallelLeafUnions:true);
        Assert.Equal(literal.Count,parallel.Count);
        for(int i=0;i<literal.Count;i++)Assert.Equal(literal[i],parallel[i]);
    }

    [Fact]
    public void PlateauExclusionIsRestrictedToCuttingDemandRatherThanDeclaredAir()
    {
        var original=new[]{new TaggedToolpath(new[]{new Point3D(0,0,-3),new Point3D(6,0,-3)},0,"final-carve",null)};
        var limited=new[]{new TaggedToolpath(new[]{new Point3D(0,0,-1),new Point3D(6,0,-1)},0,"final-carve",null)};
        var target=new[]{(IReadOnlyList<PointD>)Box(2,-.5,4,.5)};
        var mask=SmoothRoughingPlateau.Build(original,new[]{(IReadOnlyList<PointD>)Box(-10,-10,20,20)},
            3,1,default,limited,target:target);
        var distance=new PreparedRegionDistance(mask);
        Assert.True(distance.SignedDistance(new(3,0))>0);
        Assert.True(distance.SignedDistance(new(0,0))<0);
        Assert.InRange(Math.Abs(mask.Sum(r=>Clipper.Area(new PathD(r)))),1.999,2.001);
    }

    [Fact]
    public void ClearanceHoleStaysProtectedAndDoesNotBecomeExtraDemand()
    {
        var target=new[]{new MillingRegion(Box(5,0,6,12))};
        var clearance=new MillingRegion(Box(0,0,5,12),new[]{(IReadOnlyList<PointD>)Box(4.2,4,4.6,8)});
        var prepared=Access(clearance).Resolve(target,ToolGeometry.Flat(.3),new(1,1),false,30,0,default);
        var distance=new PreparedRegionDistance(prepared.Regions.SelectMany(r=>r.AsRingGroup()));
        Assert.True(distance.SignedDistance(new(4.4,6))<0);
        Assert.True(distance.SignedDistance(new(3.8,6))>0);
        var air=new TaggedToolpath(new[]{new Point3D(.3,2,-1),new Point3D(.3,10,-1)},0,"clearing",0);
        Assert.Empty(prepared.Trim(new[]{air},_=>.3,default));
        var crossing=new TaggedToolpath(new[]{new Point3D(0,6,-1),new Point3D(8,6,-1)},0,"clearing",0);
        var clipped=prepared.Trim(new[]{crossing},_=>.3,default);
        Assert.All(clipped.SelectMany(p=>p.Points),p=>Assert.InRange(p.X,4.699,6.301));
    }

    [Fact]
    public void OpenInterfaceBandDoesNotInventTheDiagonalClosingStroke()
    {
        IReadOnlyList<PointD>[] open=[new PointD[]{new(0,0),new(0,10),new(10,10)}];
        IReadOnlyList<PointD>[] target=[Box(-2,-2,12,12)];
        var band=MillingBoundaryAccess.Resolved.BuildOpenInterfaceBand(open,target,.5,default);
        var distance=new PreparedRegionDistance(band);
        Assert.True(distance.SignedDistance(new(0,5))>0);
        Assert.True(distance.SignedDistance(new(5,10))>0);
        Assert.True(distance.SignedDistance(new(5,5))<0);
        Assert.True(distance.SignedDistance(new(-.2,0))>0);
        Assert.True(distance.SignedDistance(new(10.2,10))>0);
        IReadOnlyList<PointD>[] closed=[new PointD[]{new(0,0),new(0,10),new(10,10),new(10,0),new(0,0)}];
        var loop=new PreparedRegionDistance(MillingBoundaryAccess.Resolved.BuildOpenInterfaceBand(closed,target,.5,default));
        Assert.True(loop.SignedDistance(new(5,5))<0);
        Assert.True(loop.SignedDistance(new(10.2,5))>0);
    }

    [Fact]
    public void ConstructionBoundaryCannotLimitDepthOrGradeInsideTheTarget()
    {
        var target=new[]{new MillingRegion(Box(0,0,20,12))};
        var prepared=Access().Resolve(target,ToolGeometry.Flat(3),new(6,2),true,10,0,default);
        Assert.False(prepared.OpenEdgeBandPrepared);
        Assert.False(prepared.CuttingRegionsPrepared);
        prepared.RetainedRampEdges(target[0].AsRingGroup().ToArray(),default);
        Assert.False(prepared.OpenEdgeBandPrepared);
        Assert.False(prepared.CuttingRegionsPrepared);
        var distance=new PreparedRegionDistance(prepared.Regions.SelectMany(r=>r.AsRingGroup()));
        Assert.True(prepared.CuttingRegionsPrepared);
        Assert.True(distance.SignedDistance(new(0,0))>3+6/Math.Tan(Math.PI/18));
        Assert.NotEmpty(prepared.OpenEdgeTarget);
        Assert.True(prepared.OpenEdgeBandPrepared);
    }

    [Fact]
    public void SmoothRoughingAnalysisDoesNotInventProtectionAtTheOpenPartEdge()
    {
        var target=new[]{new MillingRegion(Box(0,0,20,12))};
        var tool=new ToolGeometry(1) {VCarveProfile=new VCarveCutterProfile(new[]{
            new CarvingCutterSection(CarvingCutterSectionKind.Linear,1,1,3)},1)};
        var analysis=PocketToolpaths.AnalyzeSmoothRoughing(target,tool,new(1,.5),
            new PocketOptions {BoundaryAccess=Access(),SmoothRoughing=true,SmoothRampAngleDegrees=30},
            BoostVoronoiProvider.CreateDefault());
        var protection=new PreparedRegionDistance(analysis.ProtectedBoundary);
        Assert.True(protection.SignedDistance(new(0,6))<0);
        Assert.True(protection.SignedDistance(new(10,6))<0);
    }

    [Fact]
    public void StandalonePocketPlateauCannotProtectApprovedAirOutsideItsTarget()
    {
        var target=new[]{new MillingRegion(Box(5,0,6,12))};
        var tool=new ToolGeometry(1) {VCarveProfile=new VCarveCutterProfile(new[]{
            new CarvingCutterSection(CarvingCutterSectionKind.Linear,1,1,3)},2)};
        var depth=new DepthSchedule(2,1);
        var analysis=PocketToolpaths.AnalyzeSmoothRoughing(target,tool,depth,
            new PocketOptions {BoundaryAccess=Access(new MillingRegion(Box(0,0,5,12))),
                SmoothRoughing=true,SmoothRampAngleDegrees=30},BoostVoronoiProvider.CreateDefault());
        Assert.NotEmpty(analysis.Basins);
        var demand=target.SelectMany(r=>r.AsRingGroup()).ToArray();
        Assert.Empty(PolygonOps.Difference(analysis.ProtectedBoundary,demand));
        var aggregate=analysis.Preparation.AnalyzePocketPlateau(2,30);
        Assert.Empty(PolygonOps.Difference(aggregate.ProtectedBoundary,demand));
        // The retained right wall now owns a continuous ramp. Approved air
        // on the left cannot erase that foreground crest inside the target.
        foreach (var mask in new[] { aggregate.ProtectedBoundary, analysis.ProtectedBoundary })
        {
            Assert.True(PolygonOps.Contains(mask,5.9,6));
            Assert.False(PolygonOps.Contains(mask,4.5,6));
        }

        // A separate approved-air component with no possible cutter graph also
        // contributes no protection through the inconclusive-graph branch.
        var disconnected=PocketToolpaths.AnalyzeSmoothRoughing(new[]{new MillingRegion(Box(0,0,4,12))},tool,depth,
            new PocketOptions {BoundaryAccess=Access(new MillingRegion(Box(10,4,10.1,4.1))),
                SmoothRoughing=true,SmoothRampAngleDegrees=30},BoostVoronoiProvider.CreateDefault());
        var distance=new PreparedRegionDistance(disconnected.ProtectedBoundary);
        Assert.True(distance.SignedDistance(new(10.05,4.05))<0);
    }

    [Theory]
    [InlineData(false,false,0)] [InlineData(false,true,0)]
    [InlineData(true,false,0)] [InlineData(true,true,0)]
    [InlineData(false,true,12.7)] [InlineData(true,true,12.7)]
    public void VCarveExteriorDoesNotBecomeAWallAndProtectedWallStillLimitsThePhysicalCutter(bool smooth,bool rounded,double surface)
    {
        var profile = rounded ? new VCarveCutterProfile(new[] {
            new CarvingCutterSection(CarvingCutterSectionKind.Hemisphere,0,.1,.1),
            new CarvingCutterSection(CarvingCutterSectionKind.Linear,.1,1,3.6)},1)
            : new VCarveCutterProfile(new[] {new CarvingCutterSection(CarvingCutterSectionKind.Linear,0,1,4)},1);
        var tool = new ToolGeometry(1) {VCarveProfile=profile};
        var target=new[]{new MillingRegion(Box(0,0,.5,12))};
        var depth=new DepthSchedule(1,.5) {SurfaceZ=surface};
        var options=new VCarveOptions {BoundaryAccess=Access(),SmoothVCarving=smooth,SmoothRampAngleDegrees=45,
            IncludeInteriorFill=false,EdgeFillWidth=0,StepOver=.8,CalculateSweptArea=false,DetectCurves=false};
        var plan=VCarveToolpaths.Generate(target,tool,depth,options,BoostVoronoiProvider.CreateDefault());
        Assert.NotEmpty(plan.Toolpaths);
        var samples=plan.Toolpaths.SelectMany(path=>Enumerable.Range(1,path.Points.Count-1).SelectMany(i=>
            new[]{path.Points[i-1],path.Points[i],new Point3D((path.Points[i-1].X+path.Points[i].X)/2,
                (path.Points[i-1].Y+path.Points[i].Y)/2,(path.Points[i-1].Z+path.Points[i].Z)/2)})).ToArray();
        if (!smooth)
            Assert.Contains(samples,p=>p.X<.01 && p.Y>3 && p.Y<9 && p.Z<surface-.25);
        else
        {
            // Zero fill now keeps the COMPLETE retained-wall slope. Its rows
            // may reach the open left edge without making that edge a crest.
            // A false exterior crest would instead raise their depth to zero.
            if(!rounded) Assert.Contains(samples,p=>p.X<.01 && p.Y>3 && p.Y<9 && p.Z<surface-.45);
            Assert.Contains(samples,p=>p.Y>3 && p.Y<9 && p.Z<surface-.05);
        }
        Assert.All(samples,p=> { Assert.InRange(p.X,0,20); Assert.InRange(p.Y,0,12); });
        // Independent planar footprint check on the literal physical radius at the top plane.
        Assert.All(samples.Where(p=>p.Y>2 && p.Y<10 && p.Z<surface),p=>
            Assert.True(p.X+profile.PhysicalRadiusAtDepth(surface-p.Z)<=.502,$"Protected wall gouged at {p}"));
    }
}
