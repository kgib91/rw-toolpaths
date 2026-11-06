using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class ToolpathMotionCostMemoTests
{
    [Theory]
    [InlineData(0)]
    [InlineData(12.837999809265137)]
    public void RepeatedCandidateBodiesKeepLiteralModalFeedAndSummation(double surface)
    {
        var settings=new MillingMotionSettings(1000.7,299.8,5000.25,surface+8,surface);
        var literal=new ToolpathMotionCost(settings,surface);
        var memo=new ToolpathMotionCost(settings,surface,4096);
        var random=new Random(7231);
        var body=Enumerable.Range(0,128).Select(i=>new Point3D(i*.017,
            Math.Sin(i*.3)*.4,surface-2+Math.Cos(i*.17)*.3)).ToArray();
        var kinds=new[]{ToolpathSpanKind.Cut,ToolpathSpanKind.Ramp,ToolpathSpanKind.Link,
            ToolpathSpanKind.ClearedTravel,ToolpathSpanKind.TabLift};
        for(int trial=0;trial<80;trial++)
        {
            // The same body follows different descending/vertical/above-stock
            // prefixes. Modal feed must be replayed even on a geometry hit.
            var prefix=new Point3D[]{new(-1,0,surface+1),new(-1,.00000000001,surface-1),
                new(-random.NextDouble(),0,surface-3+random.NextDouble()*3)};
            var points=prefix.Concat(body).ToArray();
            var spans=Enumerable.Range(0,points.Length-1)
                .Select(i=>new ToolpathSpan(i,i+1,kinds[(i+trial)%kinds.Length])).ToArray();
            var path=new TaggedToolpath(points,spans,0,"candidate",0,detectCurves:false);
            Assert.Equal(BitConverter.DoubleToInt64Bits(OriginalVisit(settings,surface,path)),
                BitConverter.DoubleToInt64Bits(memo.VisitSeconds(path)));
            Assert.Equal(BitConverter.DoubleToInt64Bits(literal.VisitSeconds(path)),
                BitConverter.DoubleToInt64Bits(memo.VisitSeconds(path)));
            Assert.Equal(BitConverter.DoubleToInt64Bits(literal.CompleteVisitSeconds(prefix[0] with{Z=surface+5},path)),
                BitConverter.DoubleToInt64Bits(memo.CompleteVisitSeconds(prefix[0] with{Z=surface+5},path)));
        }
        Assert.True(memo.ChordGeometryHits>10000);
        Assert.InRange(memo.ChordGeometryEntries,1,4096);
    }

    [Fact]
    public void BoundedMemoEvictsWithoutChangingRatesOrDirection()
    {
        var settings=new MillingMotionSettings(999.9,123.45,4000,15,10);
        var literal=new ToolpathMotionCost(settings,10);
        var memo=new ToolpathMotionCost(settings,10,8);
        for(int lap=0;lap<3;lap++)
        for(int i=0;i<40;i++)
        {
            var a=new Point3D(i*.0012,0,10-i*.001);
            var b=new Point3D((i+1)*.0012,.0000000001,10-(i+1)*.001);
            foreach(var kind in new[]{ToolpathSpanKind.Cut,ToolpathSpanKind.ClearedTravel,ToolpathSpanKind.TabLift})
            {
                Assert.Equal(literal.ChordSeconds(a,b,kind),memo.ChordSeconds(a,b,kind));
                Assert.Equal(literal.ChordSeconds(b,a,kind),memo.ChordSeconds(b,a,kind));
            }
        }
        Assert.Equal(8,memo.ChordGeometryEntries);
        Assert.True(memo.ChordGeometryEvictions>0);
    }

    // Retained pre-memo algorithm, including per-visit modal feed state. This
    // oracle is independent of the production PrepareChord refactoring.
    private static double OriginalVisit(MillingMotionSettings settings,double surface,TaggedToolpath path)
    {
        float top=(float)(double.IsFinite(settings.MaterialSurfaceZ)?settings.MaterialSurfaceZ:surface);
        var limits=new MillingFeedPolicy(settings.FeedRate,settings.PlungeRate,settings.TravelRate);
        var feed=new MillingFeedPolicy(settings.FeedRate,settings.PlungeRate,settings.TravelRate);
        var kinds=ToolpathSpanKinds.Expand(path);double seconds=0;
        for(int i=1;i<path.Points.Count;i++)
        {
            var a=path.Points[i-1];var b=path.Points[i];var kind=kinds[i-1];
            float ax=(float)a.X,ay=(float)a.Y,az=(float)a.Z;
            float bx=(float)b.X,by=(float)b.Y,bz=(float)b.Z;
            double dx=bx-ax,dy=by-ay,dz=bz-az;
            double distance=Math.Sqrt(Math.Pow((double)bx-ax,2)+Math.Pow((double)by-ay,2)+Math.Pow((double)bz-az,2));
            bool vertical=ToolpathCommandRounding.IsVerticalOnly((float)dx,(float)dz,(float)dy);
            static double Q(float v)=>ToolpathCommandRounding.Round(v);
            double limit=Math.Min(limits.Limit(dx,dz,dy),limits.Limit(
                vertical?0:Q(bx)-Q(ax),Q(bz-top)-Q(az-top),vertical?0:Q(by)-Q(ay)));
            double cutting=feed.Command(limit,kind==ToolpathSpanKind.ClearedTravel);
            double rate=kind==ToolpathSpanKind.ClearedTravel?Math.Max(settings.FeedRate,settings.TravelRate)
                :kind!=ToolpathSpanKind.TabLift&&(vertical&&(dz>0||bz>=top)
                    ||!vertical&&Math.Sqrt(dx*dx+dy*dy)>.001&&Math.Min(az,bz)>=top)
                    ?settings.TravelRate:cutting;
            seconds+=distance*60/rate;
        }
        return seconds;
    }
}
