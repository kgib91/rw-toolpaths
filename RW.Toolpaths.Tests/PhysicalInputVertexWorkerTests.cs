using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

// Regression for the retired experiment selector, not validation of pooling.
public sealed class PhysicalInputVertexWorkerTests
{
    private static readonly VCarveCutterProfile Profile=new([
        new(CarvingCutterSectionKind.Hemisphere,0,.8,.8),
        new(CarvingCutterSectionKind.Linear,.8,2.5,5)],4);
    private static TaggedToolpath[] Source()=>Enumerable.Range(0,8).SelectMany(row=>new[]{
        new TaggedToolpath(Enumerable.Range(0,32).Select(i=>new Point3D(i*.025,row*.3,12.7-.3)).ToArray(),
            [],0,"retired-pool-control",0,detectCurves:false),
        new TaggedToolpath(new Point3D[]{new(0,row*.3,12.6),new(.5,row*.3+.03,12.2),new(1,row*.3,12.35)},
            [],0,"retired-pool-control",0,detectCurves:false)}).ToArray();

    [Theory]
    [InlineData(false)] [InlineData(true)]
    public void ExplicitRetiredModeRefusesBeforeDispatchOrOpaqueNomination(bool nomination)
    {
        var old=VCarvePhysicalSweep.UseInputVertexPool.Value;int calls=0;
        try
        {
            VCarvePhysicalSweep.UseInputVertexPool.Value=true;
            using var capture=VCarvePhysicalSweepTrace.BeginCapture(0);
            var error=Assert.Throws<NotSupportedException>(()=>
            {
                if(nomination)VCarvePhysicalSweep.BuildNomination(Source(),Profile,12.7,.004,paths=>
                    {calls++;return Clipper.Union(paths,FillRule.NonZero);});
                else VCarvePhysicalSweep.BuildWithParallelLeafUnions(Source(),Profile,12.7,.004,upperBound:true);
            });
            Assert.Contains("withdrawn",error.Message);Assert.Equal(0,calls);Assert.Null(capture.Completed);
        }
        finally{VCarvePhysicalSweep.UseInputVertexPool.Value=old;}
    }

    [Theory]
    [InlineData(false,false)] [InlineData(false,true)]
    [InlineData(true,false)] [InlineData(true,true)]
    public void DefaultAndExplicitOffKeepTheWholeLiteralSweepAndOnlyZeroArchiveReceipts(bool upper,bool parallel)
    {
        var source=Source();var snapshot=source.Select(p=>p.Points.ToArray()).ToArray();
        var old=VCarvePhysicalSweep.UseInputVertexPool.Value;
        try
        {
            List<List<PointD>> Build()=>parallel
                ?VCarvePhysicalSweep.BuildWithParallelLeafUnions(source,Profile,12.7,.004,upperBound:upper)
                :VCarvePhysicalSweep.Build(source,Profile,12.7,.004,upperBound:upper);
            VCarvePhysicalSweep.UseInputVertexPool.Value=null;var expected=PathUtils.ToClipper(Build());
            VCarvePhysicalSweep.UseInputVertexPool.Value=false;
            using var capture=VCarvePhysicalSweepTrace.BeginCapture(0);
            var actual=PathUtils.ToClipper(Build());Assert.Equal(expected.Count,actual.Count);
            for(int i=0;i<expected.Count;i++)Assert.Equal(expected[i].ToArray(),actual[i].ToArray());
            for(int i=0;i<source.Length;i++)Assert.Equal(snapshot[i],source[i].Points);
            if(upper)
            {
                Assert.NotNull(capture.Completed);Assert.False(capture.Completed!.Settings.UseInputVertexPool);
                var receipt=Assert.IsType<VCarvePhysicalSweepTrace.InputVertexPoolWork>(capture.Completed.Backend!.InputVertices);
                Assert.Equal(new VCarvePhysicalSweepTrace.InputVertexPoolWork(0,0,0,0,0,0,0,0,0,true),receipt);
            }
        }
        finally{VCarvePhysicalSweep.UseInputVertexPool.Value=old;}
    }
}
