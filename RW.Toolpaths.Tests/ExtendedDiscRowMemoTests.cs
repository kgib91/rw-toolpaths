using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class ExtendedDiscRowMemoTests
{
    [Fact]
    public void NonAdjacentExactRowsReuseOnlyOwnedImmutableCopiesAndKeepEveryHullVertex()
    {
        var circle=Circle(128);using var literal=new VCarveOrderedDiscHull();using var extended=new VCarveOrderedDiscHull(retainedDiscRows:32);
        var rows=Enumerable.Range(0,16).Select(i=>(new Point3D(i,i*.1,0),.5+i*.01)).ToArray();
        foreach(var row in rows)Exact(literal,extended,circle,row);
        long hits=extended.ExtendedRowHits;
        foreach(var row in rows)Exact(literal,extended,circle,row);
        Assert.Equal(hits+rows.Length,extended.ExtendedRowHits);Assert.Equal(0,literal.ExtendedRowEntries);
        Assert.Equal(32,extended.MaximumRetainedRows);Assert.Equal(16,extended.ExtendedRowEntries);
        Assert.InRange(extended.ExtendedRowPeakBytes,1,VCarveOrderedDiscHull.MaximumExtendedRowBytes);
        extended.BeginCircle(circle);extended.AddDisc(rows[0].Item1,rows[0].Item2);
        var detached=extended.FinishHull();detached[0]=new(123,456);
        Exact(literal,extended,circle,rows[0]);
    }

    [Fact]
    public void FifoEvictionRecyclesBackingAndDoesNotTurnOldRowsIntoNewEvidence()
    {
        var circle=Circle(32);using var literal=new VCarveOrderedDiscHull();using var extended=new VCarveOrderedDiscHull(retainedDiscRows:4);
        for(int i=0;i<40;i++)Exact(literal,extended,circle,(new(i,i*.1,0),1));
        Assert.Equal(2,extended.ExtendedRowEntries);Assert.Equal(38,extended.ExtendedRowEvictions);
        Assert.Equal(38,extended.ExtendedRowBufferReuses);long hits=extended.ExtendedRowHits;
        Exact(literal,extended,circle,(new(0,0,0),1));Assert.Equal(hits,extended.ExtendedRowHits);
        Exact(literal,extended,circle,(new(39,39*.1,0),1));
        Assert.Equal(hits+1,extended.ExtendedRowHits);
    }

    [Fact]
    public void SnapshotByteBudgetRefusesLargeRowsAndDisposeDropsTheInventory()
    {
        var circle=Circle(65536);var extended=new VCarveOrderedDiscHull(retainedDiscRows:32);
        using var literal=new VCarveOrderedDiscHull();
        Exact(literal,extended,circle,(new(0,0,0),1));
        Assert.Equal(1,extended.ExtendedRowBudgetRefusals);Assert.Equal(0,extended.ExtendedRowEntries);
        Assert.Equal(0,extended.ExtendedRowBytes);extended.Dispose();Assert.Equal(0,extended.ExtendedRowBytes);
    }

    [Fact]
    public void KeysRetainCircleIdentitySignedZeroRadiusBitsAndPointWritesDoNotAliasSnapshots()
    {
        var circle=Circle(64);using var literal=new VCarveOrderedDiscHull();using var extended=new VCarveOrderedDiscHull(retainedDiscRows:32);
        foreach(var row in new[]{(new Point3D(0,0,0),1d),(new Point3D(-0d,0,0),1d),
            (new Point3D(1,2,0),Math.BitIncrement(1d)),(new Point3D(1,2,0),-1d),
            (new Point3D(double.NaN,0,0),1d),(new Point3D(1e15,-1e15,0),2d)})Exact(literal,extended,circle,row);
        long hits=extended.ExtendedRowHits;Exact(literal,extended,circle.ToArray(),(new(0,0,0),1));Assert.Equal(hits,extended.ExtendedRowHits);
        foreach(var owner in new[]{literal,extended}){owner.BeginCircle(circle);owner.AddPoint(new(7,8));owner.AddDisc(new(2,3,0),1);}
        Assert.Equal(literal.FinishHull().ToArray(),extended.FinishHull().ToArray());
        Exact(literal,extended,circle,(new(0,0,0),1));Assert.True(extended.ExtendedRowHits>hits);
    }

    private static void Exact(VCarveOrderedDiscHull literal,VCarveOrderedDiscHull extended,
        (double X,double Y)[] circle,params (Point3D Center,double Radius)[] rows)
    {
        foreach(var owner in new[]{literal,extended}){owner.BeginCircle(circle);foreach(var row in rows)owner.AddDisc(row.Center,row.Radius);}
        Assert.Equal(literal.FinishHull().ToArray(),extended.FinishHull().ToArray());
        Assert.InRange(extended.ExtendedRowBytes,0,VCarveOrderedDiscHull.MaximumExtendedRowBytes);
        Assert.InRange(extended.ExtendedRowEntries,0,extended.MaximumRetainedRows-2);
    }
    private static (double X,double Y)[] Circle(int count)=>Enumerable.Range(0,count)
        .Select(i=>(Math.Cos(2*Math.PI*i/count),Math.Sin(2*Math.PI*i/count))).ToArray();
}
