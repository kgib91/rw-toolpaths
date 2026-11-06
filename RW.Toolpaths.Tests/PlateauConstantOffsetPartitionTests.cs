using Clipper2Lib;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

[Collection("Outline prefetch lifetime")]
public sealed class PlateauConstantOffsetPartitionTests
{
    static PointD[] Rectangle(double width,double height)=>[new(0,0),new(width,0),new(width,height),new(0,height)];
    static readonly VCarveOptions Options=new(){SmoothVCarving=true,SmoothRampAngleDegrees=45,
        StepOver=.4,IncludeInteriorFill=false,EdgeFillWidth=2,ExcludeDiscSweep=true,DiscRadius=1,DetectCurves=false};
    static VCarveCutterProfile Profile()=>new([new(CarvingCutterSectionKind.Hemisphere,0,1,1),
        new(CarvingCutterSectionKind.Linear,1,3,9)],10);

    [Theory]
    [InlineData(512)]
    [InlineData(1024)]
    [InlineData(2048)]
    public void CapturedPolicyAndAppliedOptionsIgnoreLaterAmbientChanges(int cap)
    {
        int? before=SmoothRoughingPlateau.ConstantOffsetLeafVerticesOverride.Value;
        try
        {
            SmoothRoughingPlateau.ConstantOffsetLeafVerticesOverride.Value=cap;
            var policy=VCarveGenerationPolicy.Capture();
            var applied=policy.ApplyPlateauPolicy(Options);
            SmoothRoughingPlateau.ConstantOffsetLeafVerticesOverride.Value=cap==512?2048:512;
            Assert.Equal(cap,policy.PlateauConstantOffsetLeafVertices);
            Assert.Equal(cap,SmoothRoughingPlateau.CaptureConstantOffsetLeafVertices(applied));
            Assert.Equal(cap==2048?string.Empty:$"|plateau-constant-offset-leaf-vertices-v1:{cap}",
                policy.PlateauConstantOffsetCacheSignature);
            Assert.EndsWith(policy.PlateauConstantOffsetCacheSignature,policy.CacheSignature());
            Assert.NotSame(Options,applied);
        }
        finally{SmoothRoughingPlateau.ConstantOffsetLeafVerticesOverride.Value=before;}
    }

    [Fact]
    public void ProductionDefaultHasVersionedIdentityAndUnsupportedTrialRefuses()
    {
        int? before=SmoothRoughingPlateau.ConstantOffsetLeafVerticesOverride.Value;
        try
        {
            SmoothRoughingPlateau.ConstantOffsetLeafVerticesOverride.Value=null;
            var selected=VCarveGenerationPolicy.Capture();
            Assert.Equal(512,selected.PlateauConstantOffsetLeafVertices);
            Assert.Equal("|plateau-constant-offset-leaf-vertices-v1:512",selected.PlateauConstantOffsetCacheSignature);
            SmoothRoughingPlateau.ConstantOffsetLeafVerticesOverride.Value=2048;
            Assert.Equal(string.Empty,VCarveGenerationPolicy.Capture().PlateauConstantOffsetCacheSignature);
            SmoothRoughingPlateau.ConstantOffsetLeafVerticesOverride.Value=1023;
            Assert.Throws<ArgumentOutOfRangeException>(()=>VCarveGenerationPolicy.Capture());
        }
        finally{SmoothRoughingPlateau.ConstantOffsetLeafVerticesOverride.Value=before;}
    }

    [Fact]
    public void OmittedAndExplicitProductionDefaultShareIdentityWhileLegacyMaskStaysSeparate()
    {
        int? before=SmoothRoughingPlateau.ConstantOffsetLeafVerticesOverride.Value;
        try
        {
            SmoothRoughingPlateau.ConstantOffsetLeafVerticesOverride.Value=null;
            var omitted=VCarveGenerationPolicy.Capture();
            var request=VCarveRampSurface.CaptureRestFillRequest([Rectangle(30,30)],Options,2,1,default);
            var key=VCarvePreparation.RestFillKey.Capture(request,default);Assert.NotNull(key);
            MillingRegion[] regions=[new(Rectangle(30,30))];
            var tool=new ToolGeometry(3){VCarveProfile=Profile()};var depth=new DepthSchedule(2,2);
            var omittedFingerprint=VCarvePreparation.ComputeFingerprint(regions,tool,depth,omitted.ApplyPlateauPolicy(Options));
            SmoothRoughingPlateau.ConstantOffsetLeafVerticesOverride.Value=512;
            var explicitDefault=VCarveGenerationPolicy.Capture();
            Assert.Equal(omitted.CacheSignature(),explicitDefault.CacheSignature());
            Assert.Equal(omittedFingerprint,VCarvePreparation.ComputeFingerprint(regions,tool,depth,
                explicitDefault.ApplyPlateauPolicy(Options)));
            var selectedRequest=VCarveRampSurface.CaptureRestFillRequest([Rectangle(30,30)],Options,2,1,default);
            Assert.True(key!.Same(VCarvePreparation.RestFillKey.Capture(selectedRequest,default)!));
            SmoothRoughingPlateau.ConstantOffsetLeafVerticesOverride.Value=2048;
            var legacy=VCarveGenerationPolicy.Capture();
            Assert.NotEqual(omitted.CacheSignature(),legacy.CacheSignature());
            Assert.NotEqual(omittedFingerprint,VCarvePreparation.ComputeFingerprint(regions,tool,depth,
                legacy.ApplyPlateauPolicy(Options)));
            var legacyRequest=VCarveRampSurface.CaptureRestFillRequest([Rectangle(30,30)],Options,2,1,default);
            Assert.False(key.Same(VCarvePreparation.RestFillKey.Capture(legacyRequest,default)!));
        }
        finally{SmoothRoughingPlateau.ConstantOffsetLeafVerticesOverride.Value=before;}
    }

    [Theory]
    [InlineData(512,1)]
    [InlineData(1024,3)]
    [InlineData(2048,7)]
    public void PartitionKeepsEveryOriginalStrokeInOrderWithoutSplittingOrMutatingIt(int cap,int expectedFirstCount)
    {
        var strokes=Enumerable.Range(0,16).Select(row=>new Path64(Enumerable.Range(0,257)
            .Select(i=>new Point64(i,row)))).ToArray();
        var snapshots=strokes.Select(p=>p.ToArray()).ToArray();
        Assert.Equal(expectedFirstCount,SmoothRoughingPlateau.ConstantOffsetLeafEnd(strokes,0,128,cap));
        var published=new List<Path64>();
        for(int first=0;first<strokes.Length;)
        {
            int end=SmoothRoughingPlateau.ConstantOffsetLeafEnd(strokes,first,128,cap);
            Assert.InRange(strokes.Skip(first).Take(end-first).Sum(p=>p.Count),1,cap);
            published.AddRange(strokes.Skip(first).Take(end-first));first=end;
        }
        Assert.Equal(strokes.Length,published.Count);
        for(int i=0;i<strokes.Length;i++)
        {Assert.Same(strokes[i],published[i]);Assert.Equal(snapshots[i],strokes[i]);}
        // Existing non-chained strokes may exceed the cap: retain one complete
        // input rather than inventing new endpoints or refusing valid geometry.
        Path64[] oversized=[new(Enumerable.Range(0,cap+1).Select(i=>new Point64(i,0))),strokes[0]];
        Assert.Equal(1,SmoothRoughingPlateau.ConstantOffsetLeafEnd(oversized,0,128,cap));
    }

    [Fact]
    public void RestFillKeyBindsCapturedBatchingPolicyIndependentlyOfAmbientSelector()
    {
        int? before=SmoothRoughingPlateau.ConstantOffsetLeafVerticesOverride.Value;
        try
        {
            SmoothRoughingPlateau.ConstantOffsetLeafVerticesOverride.Value=512;
            var request=VCarveRampSurface.CaptureRestFillRequest([Rectangle(30,30)],Options,2,1,default);
            var key=VCarvePreparation.RestFillKey.Capture(request,default);Assert.NotNull(key);
            SmoothRoughingPlateau.ConstantOffsetLeafVerticesOverride.Value=2048;
            Assert.True(key!.Same(VCarvePreparation.RestFillKey.Capture(request,default)!));
            foreach(int different in new[]{1024,2048})
                Assert.False(key.Same(VCarvePreparation.RestFillKey.Capture(request with
                    {PlateauConstantOffsetLeafVertices=different},default)!));
        }
        finally{SmoothRoughingPlateau.ConstantOffsetLeafVerticesOverride.Value=before;}
    }

    [Fact]
    public async Task RetainedPreparationAndOutlineCacheUseTheirCapturedPolicy()
    {
        int? before=SmoothRoughingPlateau.ConstantOffsetLeafVerticesOverride.Value;
        try
        {
            SmoothRoughingPlateau.ConstantOffsetLeafVerticesOverride.Value=512;
            var captured=VCarveGenerationPolicy.Capture().ApplyPlateauPolicy(Options);
            MillingRegion[] regions=[new(Rectangle(30,30))];
            var tool=new ToolGeometry(3){VCarveProfile=Profile()};var depth=new DepthSchedule(2,2);
            var owner=VCarveToolpaths.Prepare(regions,tool,depth,captured,BoostVoronoiProvider.CreateDefault());
            var request=VCarveRampSurface.PrepareInitialOutline(owner.Regions[0].Boundary,captured,Profile(),2,default);
            SmoothRoughingPlateau.ConstantOffsetLeafVerticesOverride.Value=2048;
            Assert.True(owner.Matches(regions,tool,depth,captured));
            Assert.False(owner.Matches(regions,tool,depth,Options));
            using var lease=owner.PrefetchInitialOutline(default,0);Assert.NotNull(lease);await lease!.Completion;
            var log=new List<string>();
            owner.InitialOutlineRings(request with{PlateauConstantOffsetLeafVertices=1024},default,log.Add);
            Assert.Contains(log,l=>l.Contains("stage=outline-prefetch event=miss"));
            Assert.Equal(1,owner.InitialOutlineCacheState.Entries);
            log.Clear();owner.InitialOutlineRings(request,default,log.Add);
            Assert.Contains(log,l=>l.Contains("stage=outline-prefetch event=hit"));
            Assert.Equal(0,owner.InitialOutlineCacheState.Entries);
        }
        finally{SmoothRoughingPlateau.ConstantOffsetLeafVerticesOverride.Value=before;}
    }
}
