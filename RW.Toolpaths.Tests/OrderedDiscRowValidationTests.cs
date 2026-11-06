using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class OrderedDiscRowValidationTests
{
    [Theory]
    [InlineData(8)] [InlineData(128)] [InlineData(1024)]
    public void RemovedPassKeepsLiteralGeneralHullsAndEveryExistingRowMemoCounter(int count)
    {
        using var literal=new VCarveOrderedDiscHull(literalFinalRowValidation:true);
        using var optimized=new VCarveOrderedDiscHull();
        var circle=Circle(count);var random=new Random(52213+count);
        var prior=(Center:new Point3D(0,0,0),Radius:.5);
        for(int i=0;i<96;i++)
        {
            var a=i%3==0?prior:(new Point3D(random.NextDouble()*20-10,random.NextDouble()*20-10,0),random.NextDouble()*2);
            var b=(new Point3D(a.Item1.X+.1,a.Item1.Y-.2,0),i%5==0?a.Item2:random.NextDouble()*2);
            Exact(literal,optimized,circle,a,b);prior=b;
        }
        Assert.True(optimized.DiscRowCacheHits>0);Assert.True(optimized.FinalRowValidationComparisonsAvoided>0);
        EqualCounters(literal,optimized);
    }

    [Fact]
    public void DuplicateRowsTieCollapseInvalidAndExtremeIntegerCoordinatesRetainLiteralOrdering()
    {
        using var literal=new VCarveOrderedDiscHull(true);using var optimized=new VCarveOrderedDiscHull();
        var circle=Circle(128);
        foreach(var row in new[]{
            (new Point3D(.5/PathUtils.Scale,-.5/PathUtils.Scale,0),1e-12),
            (new Point3D(0,0,0),0d),(new Point3D(-0d,0,0),1d),
            (new Point3D(1e15,-1e15,0),2d),(new Point3D(-1e15,1e15,0),2d),
            (new Point3D(0,0,0),-1d),(new Point3D(double.NaN,0,0),1d),
            (new Point3D(0,0,0),double.PositiveInfinity)})
        {Exact(literal,optimized,circle,row,row);Exact(literal,optimized,circle,row,row);}
        Assert.True(optimized.MaximumTieGroup>2);Assert.True(optimized.SortFallbacks>0);
        EqualCounters(literal,optimized);
    }

    [Fact]
    public void OriginalFullSortFallbackAndPointInvalidationRemainLiteral()
    {
        using var literal=new VCarveOrderedDiscHull(true);using var optimized=new VCarveOrderedDiscHull();
        var circle=Circle(32);var a=new Point3D(1,2,0);var b=new Point3D(3,4,0);
        Exact(literal,optimized,circle,(a,1),(b,2));
        // Same count with a different circle order exercises the original
        // complete X-order check and its existing general-sort fallback.
        var changed=circle.Reverse().Select(p=>(p.X*.3,p.Y*1.7)).ToArray();
        Exact(literal,optimized,changed,(a,1),(b,2));
        foreach(var owner in new[]{literal,optimized})
        {owner.BeginCircle(changed);owner.AddPoint(new(9,10));owner.AddDisc(b,2);}
        Assert.Equal(literal.FinishHull().ToArray(),optimized.FinishHull().ToArray());
        Exact(literal,optimized,changed,(a,1),(b,2));EqualCounters(literal,optimized);
        Assert.True(optimized.SortFallbacks>0);
    }

    [Fact]
    public void MemoHitsDoNotPretendToAvoidAValidationThatTheLiteralOwnerWouldNotRun()
    {
        using var literal=new VCarveOrderedDiscHull(true);using var optimized=new VCarveOrderedDiscHull();
        var circle=Circle(64);var a=new Point3D(1,2,0);var b=new Point3D(3,4,0);
        Exact(literal,optimized,circle,(a,1),(b,2));
        long comparisons=literal.FinalRowValidationComparisons,avoided=optimized.FinalRowValidationComparisonsAvoided;
        Exact(literal,optimized,circle,(a,1),(b,2));
        Assert.Equal(comparisons,literal.FinalRowValidationComparisons);
        Assert.Equal(avoided,optimized.FinalRowValidationComparisonsAvoided);EqualCounters(literal,optimized);
    }

    private static void Exact(VCarveOrderedDiscHull literal,VCarveOrderedDiscHull optimized,
        (double X,double Y)[] circle,params (Point3D Center,double Radius)[] rows)
    {
        var raw=new List<Point64>();
        foreach(var owner in new[]{literal,optimized})
        {owner.BeginCircle(circle);foreach(var row in rows)owner.AddDisc(row.Center,row.Radius);}
        foreach(var row in rows)foreach(var p in circle)raw.Add(new(
            (long)Math.Round((row.Center.X+row.Radius*p.X)*PathUtils.Scale),
            (long)Math.Round((row.Center.Y+row.Radius*p.Y)*PathUtils.Scale)));
        var expected=Oracle(raw).ToArray();
        Assert.Equal(expected,literal.FinishHull().ToArray());Assert.Equal(expected,optimized.FinishHull().ToArray());
        EqualCounters(literal,optimized);
    }
    private static void EqualCounters(VCarveOrderedDiscHull literal,VCarveOrderedDiscHull optimized)
    {
        Assert.Equal(literal.Calls,optimized.Calls);Assert.Equal(literal.Vertices,optimized.Vertices);
        Assert.Equal(literal.TieGroups,optimized.TieGroups);Assert.Equal(literal.TiePairs,optimized.TiePairs);
        Assert.Equal(literal.PairSwaps,optimized.PairSwaps);Assert.Equal(literal.SortFallbacks,optimized.SortFallbacks);
        Assert.Equal(literal.DuplicatesSkipped,optimized.DuplicatesSkipped);Assert.Equal(literal.MaximumTieGroup,optimized.MaximumTieGroup);
        Assert.Equal(literal.DiscRowCacheHits,optimized.DiscRowCacheHits);Assert.Equal(literal.DiscRowCacheMisses,optimized.DiscRowCacheMisses);
        Assert.Equal(literal.DiscRowCacheRefusals,optimized.DiscRowCacheRefusals);Assert.Equal(literal.DiscRowBufferSwaps,optimized.DiscRowBufferSwaps);
        Assert.Equal(literal.DiscVerticesAvoided,optimized.DiscVerticesAvoided);
        Assert.Equal(0,optimized.FinalRowValidationComparisons);Assert.Equal(0,literal.FinalRowValidationComparisonsAvoided);
        Assert.Equal(literal.FinalRowValidationComparisons,optimized.FinalRowValidationComparisonsAvoided);
    }
    private static (double X,double Y)[] Circle(int count)=>Enumerable.Range(0,count)
        .Select(i=>(Math.Cos(2*Math.PI*i/count),Math.Sin(2*Math.PI*i/count))).ToArray();
    private static Path64 Oracle(List<Point64> points)
    {
        points.Sort((a,b)=>a.X==b.X?a.Y.CompareTo(b.Y):a.X.CompareTo(b.X));var unique=points.Distinct().ToArray();
        if(unique.Length<=2)return new(unique);
        static Int128 Cross(Point64 a,Point64 b,Point64 c)=>(Int128)(b.X-a.X)*(c.Y-a.Y)-(Int128)(b.Y-a.Y)*(c.X-a.X);
        var hull=new Path64();foreach(var p in unique)
        {while(hull.Count>=2&&Cross(hull[^2],hull[^1],p)<=0)hull.RemoveAt(hull.Count-1);hull.Add(p);}
        int lower=hull.Count;for(int i=unique.Length-2;i>=0;i--)
        {var p=unique[i];while(hull.Count>lower&&Cross(hull[^2],hull[^1],p)<=0)hull.RemoveAt(hull.Count-1);hull.Add(p);}
        hull.RemoveAt(hull.Count-1);return hull;
    }
}
