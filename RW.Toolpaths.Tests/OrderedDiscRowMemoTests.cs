using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class OrderedDiscRowMemoTests
{
    [Fact]
    public void AdjacentAndRepeatedEndpointDiscsReuseOnlyExactSortedRows()
    {
        var circle=Circle(128);using var hull=new VCarveOrderedDiscHull();
        var a=new Point3D(1,2,0);var b=new Point3D(2,3,1);var c=new Point3D(4,5,2);
        Exact(hull,circle,(a,.4),(b,.7));
        Assert.Equal(0,hull.DiscRowCacheHits);Assert.Equal(2,hull.DiscRowCacheMisses);
        Exact(hull,circle,(b,.7),(c,.9));
        Assert.Equal(1,hull.DiscRowCacheHits);Assert.Equal(1,hull.DiscRowBufferSwaps);
        Exact(hull,circle,(b with{Z=-999},.7),(c with{Z=999},.9));
        Assert.Equal(3,hull.DiscRowCacheHits);Assert.Equal(3*circle.Length,hull.DiscVerticesAvoided);
        Assert.Equal(6*circle.Length,hull.Vertices);
    }

    [Fact]
    public void SecondEndpointCannotSwapAwayTheCurrentFirstEndpoint()
    {
        var circle=Circle(32);using var hull=new VCarveOrderedDiscHull();
        var a=new Point3D(0,0,0);var b=new Point3D(5,2,0);var c=new Point3D(-3,4,0);
        Exact(hull,circle,(a,1),(b,2));
        Exact(hull,circle,(a,1),(a,1));
        Assert.Equal(1,hull.DiscRowCacheHits); // ordinal one must compute its own row
        Assert.Equal(0,hull.DiscRowBufferSwaps);
        Exact(hull,circle,(c,.5),(a,1));
        Assert.Equal(2,hull.DiscRowCacheHits);
    }

    [Fact]
    public void KeysRetainEveryIeeeBitAndCircleArrayIdentity()
    {
        var circle=Circle(32);using var hull=new VCarveOrderedDiscHull();
        var a=new Point3D(1,2,0);
        Exact(hull,circle,(a,1));
        foreach(var row in new[]{(a with{X=Math.BitIncrement(a.X)},1d),
            (a with{Y=Math.BitIncrement(a.Y)},1d),(a,Math.BitIncrement(1d)),
            (new Point3D(+0d,0,0),1d),(new Point3D(-0d,0,0),1d)})Exact(hull,circle,row);
        Assert.Equal(0,hull.DiscRowCacheHits);
        Exact(hull,circle.ToArray(),(new Point3D(-0d,0,0),1));
        var custom=Enumerable.Range(0,circle.Length).Select(i=>(X:circle[i].X*.4,Y:circle[i].Y*1.7)).Reverse().ToArray();
        Exact(hull,custom,(new Point3D(-0d,0,0),1));
        Assert.Equal(0,hull.DiscRowCacheHits);
    }

    [Fact]
    public void PointWritesInvalidateOnlyTheirRowAndReturnedHullsRemainDetached()
    {
        var circle=Circle(64);using var hull=new VCarveOrderedDiscHull();
        var a=new Point3D(1,2,0);var b=new Point3D(5,6,0);
        Exact(hull,circle,(a,1),(b,2));
        hull.BeginCircle(circle);hull.AddPoint(new(7,8));hull.AddDisc(b,2);
        var expected=Oracle(new[]{new Point64(7,8)}.Concat(Vertices(circle,b,2)).ToList());
        Assert.Equal(expected.ToArray(),hull.FinishHull().ToArray());
        Assert.Equal(1,hull.DiscRowCacheHits);
        Exact(hull,circle,(a,1),(b,2));
        Assert.Equal(2,hull.DiscRowCacheHits); // point row cannot retain old a's key
        var returned=hull.FinishHull();returned[0]=new(123,456);
        Exact(hull,circle,(a,1),(b,2));
        hull.BeginCircle(circle);hull.AddDisc(a,1);hull.AddPoint(new(9,10));_ = hull.FinishHull();
        long hits=hull.DiscRowCacheHits;
        Exact(hull,circle,(a,1),(b,2));
        Assert.Equal(hits+1,hull.DiscRowCacheHits);
    }

    [Fact]
    public void InvalidArithmeticNeverPublishesOrReusesAMemoRow()
    {
        var circle=Circle(16);using var hull=new VCarveOrderedDiscHull();
        var badRows=new[]{(new Point3D(0,0,0),-1d),(new Point3D(double.NaN,0,0),1d),
            (new Point3D(0,double.PositiveInfinity,0),1d),(new Point3D(0,0,0),double.NaN),
            (new Point3D(0,0,0),double.PositiveInfinity)};
        foreach(var row in badRows){Exact(hull,circle,row);Exact(hull,circle,row);}
        var invalidCircle=new[]{(X:double.NaN,Y:0d),(X:0d,Y:double.NegativeInfinity)};
        Exact(hull,invalidCircle,(new(0,0,0),1));Exact(hull,invalidCircle,(new(0,0,0),1));
        Assert.Equal(0,hull.DiscRowCacheHits);Assert.Equal(0,hull.DiscVerticesAvoided);
        Assert.Equal(12,hull.DiscRowCacheRefusals);
        Exact(hull,circle,(new(1e15,-1e15,0),2));
        Exact(hull,circle,(new(1e15,-1e15,0),2)); // finite extreme cast behavior remains literal
        Assert.Equal(1,hull.DiscRowCacheHits);
    }

    [Theory]
    [InlineData(8)]
    [InlineData(128)]
    [InlineData(1024)]
    public void RandomEndpointChainsAndQuantizedTiesMatchTheLiteralGeneralHull(int count)
    {
        var random=new Random(87231+count);var circle=Circle(count);
        using var hull=new VCarveOrderedDiscHull();
        var prior=(Center:new Point3D(.5/PathUtils.Scale,-.5/PathUtils.Scale,0),Radius:.000001);
        for(int sample=0;sample<160;sample++)
        {
            var a=sample%3==0?prior:(new Point3D(random.NextDouble()*20-10,random.NextDouble()*20-10,0),Math.Pow(10,random.NextDouble()*7-6));
            var b=(new Point3D(a.Item1.X+random.NextDouble(),a.Item1.Y-random.NextDouble(),0),
                sample%7==0?a.Item2:Math.Pow(10,random.NextDouble()*7-6));
            Exact(hull,circle,a,b);prior=b;
        }
        Assert.True(hull.DiscRowCacheHits>0);Assert.True(hull.DiscVerticesAvoided>0);
    }

    private static void Exact(VCarveOrderedDiscHull hull,(double X,double Y)[] circle,
        params (Point3D Center,double Radius)[] rows)
    {
        hull.BeginCircle(circle);var raw=new List<Point64>();
        foreach(var row in rows){hull.AddDisc(row.Center,row.Radius);raw.AddRange(Vertices(circle,row.Center,row.Radius));}
        Assert.Equal(Oracle(raw).ToArray(),hull.FinishHull().ToArray());
    }
    private static IEnumerable<Point64> Vertices((double X,double Y)[] circle,Point3D center,double radius)
    {
        foreach(var unit in circle)yield return new((long)Math.Round((center.X+radius*unit.X)*PathUtils.Scale),
            (long)Math.Round((center.Y+radius*unit.Y)*PathUtils.Scale));
    }
    private static (double X,double Y)[] Circle(int count)=>Enumerable.Range(0,count)
        .Select(i=>(Math.Cos(2*Math.PI*i/count),Math.Sin(2*Math.PI*i/count))).ToArray();
    // Literal independent pre-row-cache general point-cloud construction.
    private static Path64 Oracle(List<Point64> points)
    {
        points.Sort((a,b)=>a.X==b.X?a.Y.CompareTo(b.Y):a.X.CompareTo(b.X));var unique=points.Distinct().ToArray();
        if(unique.Length<=2)return new(unique);
        static Int128 Cross(Point64 a,Point64 b,Point64 c)=>(Int128)(b.X-a.X)*(c.Y-a.Y)-(Int128)(b.Y-a.Y)*(c.X-a.X);
        var hull=new Path64();foreach(var point in unique)
        {while(hull.Count>=2&&Cross(hull[^2],hull[^1],point)<=0)hull.RemoveAt(hull.Count-1);hull.Add(point);}
        int lower=hull.Count;
        for(int i=unique.Length-2;i>=0;i--)
        {var point=unique[i];while(hull.Count>lower&&Cross(hull[^2],hull[^1],point)<=0)hull.RemoveAt(hull.Count-1);hull.Add(point);}
        hull.RemoveAt(hull.Count-1);return hull;
    }
}
