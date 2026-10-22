using System.Reflection;
using Clipper2Lib;
using RW.Toolpaths;
using RW.Toolpaths.Milling;
using Xunit;

namespace RW.Toolpaths.Tests;

public sealed class VCarveMaterialLayerGoalTests
{
    static VCarveCutterProfile Profile()=>new(new[]{
        new CarvingCutterSection(CarvingCutterSectionKind.Hemisphere,0,1.5875,1.5875),
        new CarvingCutterSection(CarvingCutterSectionKind.Linear,1.5875,3,20),
        new CarvingCutterSection(CarvingCutterSectionKind.Linear,3,3,90.4)},21.5875);
    static List<TaggedToolpath> Path(params Point3D[] p)=>new(){new(p,Array.Empty<ToolpathSpan>(),7,"final-carve",3,19,detectCurves:false)};
    static Paths64 Native(IEnumerable<IReadOnlyList<PointD>> rings)=>new(rings.Select(r=>new Path64(r.Select(p=>new Point64((long)Math.Round(p.x*PathUtils.Scale),(long)Math.Round(p.y*PathUtils.Scale))))));
    [Fact]
    public void AnalysisPlaneAndActualProfileAgreeWithoutRepostingOrOrdinalChanges()
    {
        var p=Profile();var physical=Path(new(0,0,-.004),new(1,0,-.004));double plane=VCarveMaterialLayerGoal.AnalysisSurface(true);
        var analysis=VCarveMaterialLayerGoal.ForCertificate(physical,plane);
        Assert.Same(physical,VCarveMaterialLayerGoal.ForCertificate(physical,0));
        Assert.Equal(-.004,physical[0].Points[0].Z);Assert.Equal(-.003,analysis[0].Points[0].Z);
        Assert.Equal(physical[0].RegionIndex,analysis[0].RegionIndex);Assert.Equal(physical[0].Category,analysis[0].Category);
        Assert.Equal(physical[0].DepthPassIndex,analysis[0].DepthPassIndex);Assert.Equal(physical[0].VisitIdentity,analysis[0].VisitIdentity);
        Assert.Equal(physical[0].Points.Count,analysis[0].Points.Count);
        var atPlane=VCarvePhysicalSweep.Build(physical,p,plane,.001,upperBound:true);
        var translated=VCarvePhysicalSweep.Build(analysis,p,0,.001,upperBound:true);
        Assert.NotEmpty(atPlane);Assert.Empty(Clipper.Difference(Native(atPlane),Native(translated),FillRule.NonZero));
        Assert.Empty(Clipper.Difference(Native(translated),Native(atPlane),FillRule.NonZero));
        // Non-lattice diagnostic coordinates must be translated literally.
        // Reposting this analysis view would change the supplied geometry.
        var unrounded=Path(new(1.0001,2.0002,-.0019),new(1.2345,2.3456,-.0029));
        var moved=VCarveMaterialLayerGoal.ForCertificate(unrounded,plane);
        Assert.Equal(unrounded[0].Points[0].Z+.001,moved[0].Points[0].Z);
        Assert.Equal(unrounded[0].Points[0].X,moved[0].Points[0].X);
        Assert.Equal(unrounded[0].Points[0].Y,moved[0].Points[0].Y);
        using var cancel=new CancellationTokenSource();cancel.Cancel();
        Assert.Throws<OperationCanceledException>(()=>VCarveMaterialLayerGoal.ForCertificate(physical,plane,cancel.Token));
    }
    [Fact]
    public void AuthenticShallowRampHasNoLayerDemandAndUnsafeOriginalWallStillRefuses()
    {
        var profile=Profile();
        // Actual third-job winning ramp at unresolved(233.7661683,53.5564995).
        // Its complete cutter stays above surface here; its maximum tip depth
        // is one Z quantum, so it contributes no penetrating layer footprint.
        var actual=Path(new(233.696,53.543,0),new(233.934,53.445,-.001));
        Assert.NotEmpty(VCarvePhysicalSweep.Build(actual,profile,0,.001,upperBound:true));
        Assert.Empty(VCarvePhysicalSweep.Build(actual,profile,-.001,.001,upperBound:true));
        Assert.Empty(VCarvePhysicalSweep.Build(VCarveMaterialLayerGoal.ForCertificate(actual,-.001),profile,0,.001,upperBound:true));
        // Layer analysis must not become physical wall proof. A1um-deep
        // original command1um from a wall is unsafe for this rounded tip.
        var unsafePhysical=Path(new(.999,0,-.001),new(.999,.1,-.001));
        var translated=VCarveMaterialLayerGoal.ForCertificate(unsafePhysical,-.001);
        Assert.Equal(0,translated[0].Points[0].Z);Assert.Equal(-.001,unsafePhysical[0].Points[0].Z);
        IReadOnlyList<IReadOnlyList<PointD>> artwork=new[]{(IReadOnlyList<PointD>)new List<PointD>{new(-1,-1),new(1,-1),new(1,1),new(-1,1)}};
        Type wall=typeof(VCarveSmoothing).GetNestedType("CommandClearance",BindingFlags.NonPublic)!;
        var ctor=wall.GetConstructor(BindingFlags.NonPublic|BindingFlags.Instance,null,new[]{typeof(IReadOnlyList<IReadOnlyList<PointD>>),typeof(VCarveCutterProfile),typeof(double),typeof(CancellationToken),typeof(Action<string>),typeof(IReadOnlyList<IReadOnlyList<PointD>>),typeof(long),typeof(bool),typeof(int)},null)!;
        object owner=ctor.Invoke(new object?[]{artwork,profile,0d,CancellationToken.None,null,artwork,1_000_000L,true,65536});
        var proof=(Func<Point3D,Point3D,bool>)wall.GetMethod("IsLegalCommandedChord",BindingFlags.NonPublic|BindingFlags.Instance)!
            .CreateDelegate(typeof(Func<Point3D,Point3D,bool>),owner);
        Assert.False(proof(unsafePhysical[0].Points[0],unsafePhysical[0].Points[1]));
    }
}
