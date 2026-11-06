using System.Collections;
using System.Reflection;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class IncrementalStockStorageTests
{
    private static VCarveCutterProfile Profile=>new([new(CarvingCutterSectionKind.Linear,0,3,6)],5);
    private static CutterSweepStock Stock(bool literal,CancellationToken token=default,bool incremental=true,
        Func<Point3D,Point3D>? command=null)=>new(Profile,command??(p=>p),0,token,
            mergeIncrementally:incremental,literalIncrementalStorage:literal);

    [Fact]
    public void EveryCommittedRunAndTreeBitMatchesTheLiteralCascadeAcrossMixedAdmissions()
    {
        var oldCalls=new List<Point3D>();var newCalls=new List<Point3D>();
        var literal=Stock(true,command:p=>{oldCalls.Add(p);return p;});
        var compact=Stock(false,command:p=>{newCalls.Add(p);return p;});
        int[] counts=[1,2,1,4,1,2,1,8];
        for(int batch=0;batch<80;batch++)
        {
            double y=(batch%11)*5;
            foreach(var stock in new[]{literal,compact})
            {
                for(int i=0;i<counts[batch%counts.Length];i++)
                    stock.RecordExecuted(new(i,y,-3-(batch%3)*.25),new(i+.75,y,-3.1));
                stock.AddProtected([Cut(new(-.5,y,-3),new(0,y,-3),new(.5,y,-3))]);
                stock.Seal();
            }
            Assert.Equal(Snapshot(literal),Snapshot(compact));
            Assert.Equal(literal.Chords,compact.Chords);
            Assert.Equal(literal.MergeCount,compact.MergeCount);
            Assert.Equal(literal.MaximumBlocks,compact.MaximumBlocks);
            Assert.Equal(literal.ReindexedChords,compact.ReindexedChords);
            Assert.Equal(literal.PartitionVisits,compact.PartitionVisits);
            foreach(double z in new[]{-1d,-2.9,-3.5})
            foreach(double offset in new[]{0d,.01,1.5})
            {
                var a=new Point3D(0,y+offset,z);var b=new Point3D(.5,y+offset,z);
                Assert.Equal(literal.Contains(a,b,.01,-.01),compact.Contains(a,b,.01,-.01));
                Assert.Equal(literal.ContainsTravel(a,b,.1),compact.ContainsTravel(a,b,.1));
                Assert.Equal(literal.CoversFloorTarget(a,.01),compact.CoversFloorTarget(a,.01));
            }
            Assert.Equal(literal.Queries,compact.Queries);Assert.Equal(literal.Accepted,compact.Accepted);
            Assert.Equal(literal.UnionAccepted,compact.UnionAccepted);
            Assert.Equal(literal.PriorWholeChordTrials,compact.PriorWholeChordTrials);
            Assert.Equal(literal.PriorWholeChordProofs,compact.PriorWholeChordProofs);
            Assert.Equal(literal.FloorTargetIndexHits,compact.FloorTargetIndexHits);
            Assert.Equal(literal.FloorTargetChordFallbacks,compact.FloorTargetChordFallbacks);
            Assert.False(compact.Contains(new(0,1000,-1),new(1,1000,-1),0));
            Assert.False(literal.Contains(new(0,1000,-1),new(1,1000,-1),0));
        }
        Assert.Equal(oldCalls,newCalls);
        Assert.Equal(0,ListCount(compact,"all"));Assert.Equal(literal.Chords,ListCount(literal,"all"));
        Assert.Equal(compact.Chords,compact.IncrementalAllEntriesAvoided);
        Assert.True(compact.IncrementalChordCopyRowsAvoided>0);
        Assert.True(compact.IncrementalFloorCopyRowsAvoided>0);
    }

    [Fact]
    public void SignedZerosAndAdjacentDepthBitsSurviveRepeatedRunCopies()
    {
        var a=Stock(true);var b=Stock(false);
        double negativeZero=BitConverter.Int64BitsToDouble(long.MinValue);
        foreach(double depth in new[]{-3d,Math.BitIncrement(-3d),Math.BitDecrement(-3d),-3d})
        foreach(var stock in new[]{a,b})
        {stock.RecordExecuted(new(negativeZero,0,depth),new(0,negativeZero,depth));stock.Seal();}
        Assert.Equal(Snapshot(a),Snapshot(b));
        Assert.Contains(long.MinValue,Snapshot(b));
    }

    [Fact]
    public void EightSingleRowAdmissionsHaveTheExactAvoidedCopyReceipt()
    {
        var literal=Stock(true);var compact=Stock(false);
        for(int i=0;i<8;i++)
        foreach(var stock in new[]{literal,compact})
        {stock.AddProtected([Cut(new(i*4,0,-3),new(i*4+1,0,-3))]);stock.Seal();}
        Assert.Equal(Snapshot(literal),Snapshot(compact));
        Assert.Equal(8,compact.Chords);Assert.Equal(7,compact.MergeCount);
        Assert.Equal(8,compact.IncrementalAllEntriesAvoided);
        Assert.Equal(12,compact.IncrementalChordCopyRowsAvoided);
        Assert.Equal(12,compact.IncrementalFloorCopyRowsAvoided);
        Assert.Equal(14,compact.IncrementalTemporaryArraysAvoided);
        Assert.Equal(0,literal.IncrementalTemporaryArraysAvoided);
    }

    [Fact]
    public void CancelledMergeKeepsTheOriginalAdmissionAndCheckpointState()
    {
        using var cancellation=new CancellationTokenSource();
        var literal=Stock(true,cancellation.Token);var compact=Stock(false,cancellation.Token);
        foreach(var stock in new[]{literal,compact})
        {
            stock.RecordExecuted(new(0,0,-3),new(1,0,-3));stock.Seal();
            stock.RecordExecuted(new(2,0,-3),new(3,0,-3));
        }
        cancellation.Cancel();
        Assert.Throws<OperationCanceledException>(()=>literal.Seal());
        Assert.Throws<OperationCanceledException>(()=>compact.Seal());
        Assert.Equal(Snapshot(literal),Snapshot(compact));
        Assert.Equal(2,literal.Chords);Assert.Equal(2,compact.Chords);
        Assert.Equal(1,ListCount(literal,"pending"));Assert.Equal(1,ListCount(compact,"pending"));
        Assert.Equal(literal.MergeCount,compact.MergeCount);Assert.Equal(literal.ReindexedChords,compact.ReindexedChords);
    }

    [Fact]
    public void FailedCommandAdmissionPreservesTheLiteralCallbackAndCommittedPrefix()
    {
        var oldCalls=new List<Point3D>();var newCalls=new List<Point3D>();
        Point3D Command(Point3D p,List<Point3D> calls)
        {calls.Add(p);if(p.X==3)throw new InvalidOperationException("command failure");return p;}
        var literal=Stock(true,command:p=>Command(p,oldCalls));
        var compact=Stock(false,command:p=>Command(p,newCalls));
        var source=Cut(new(0,0,-3),new(1,0,-3),new(2,0,-3),new(3,0,-3),new(4,0,-3));
        Assert.Throws<InvalidOperationException>(()=>literal.AddProtected([source]));
        Assert.Throws<InvalidOperationException>(()=>compact.AddProtected([source]));
        Assert.Equal(oldCalls,newCalls);Assert.Equal(literal.Chords,compact.Chords);
        literal.Seal();compact.Seal();Assert.Equal(Snapshot(literal),Snapshot(compact));
    }

    [Fact]
    public void NonincrementalOwnerRetainsTheLiteralWholePrefixInventory()
    {
        var a=Stock(true,incremental:false);var b=Stock(false,incremental:false);
        for(int i=0;i<24;i++)
        foreach(var stock in new[]{a,b})
        {stock.RecordExecuted(new(i,0,-3),new(i+.75,0,-3));stock.Seal();}
        Assert.Equal(Snapshot(a),Snapshot(b));Assert.Equal(a.ReindexedChords,b.ReindexedChords);
        Assert.True(a.MergeCount>0);Assert.Equal(24,ListCount(b,"all"));
        Assert.Equal(0,b.IncrementalAllEntriesAvoided);Assert.Equal(0,b.IncrementalChordCopyRowsAvoided);
    }

    [Fact]
    public void CountOnlyAdmissionRetainsTheExactTwoMillionChordLimit()
    {
        var stock=Stock(false);
        Field(stock,"committedChordCount").SetValue(stock,1_999_999);
        stock.RecordExecuted(new(0,0,-3),new(1,0,-3));
        stock.RecordExecuted(new(1,0,-3),new(2,0,-3));
        Assert.Equal(2_000_000,stock.Chords);Assert.Equal(1,ListCount(stock,"pending"));
        int calls=0;
        var protectedStock=Stock(false,command:p=>{calls++;return p;});
        Field(protectedStock,"committedChordCount").SetValue(protectedStock,1_999_999);
        protectedStock.AddProtected([Cut(new(0,0,-3),new(1,0,-3),new(2,0,-3))]);
        Assert.Equal(2_000_000,protectedStock.Chords);Assert.Equal(2,calls);
        Assert.Equal(1,ListCount(protectedStock,"pending"));
    }

    [Fact]
    public void WarmedManyRunConstructionAllocatesLessWithoutChangingItsFinalTree()
    {
        static CutterSweepStock Build(bool literal,int batches)
        {
            var stock=Stock(literal);
            for(int batch=0;batch<batches;batch++)
            {
                for(int i=0;i<4;i++)stock.RecordExecuted(new(i,batch,-3),new(i+.5,batch,-3));
                stock.Seal();
            }
            return stock;
        }
        Build(true,8);Build(false,8);
        long start=GC.GetAllocatedBytesForCurrentThread();var old=Build(true,512);
        long literalBytes=GC.GetAllocatedBytesForCurrentThread()-start;
        start=GC.GetAllocatedBytesForCurrentThread();var compact=Build(false,512);
        long compactBytes=GC.GetAllocatedBytesForCurrentThread()-start;
        Assert.Equal(Snapshot(old),Snapshot(compact));Assert.True(compactBytes<literalBytes);
        Console.WriteLine($"[Stock.Storage.Test] literalBytes={literalBytes} compactBytes={compactBytes} "
            +$"{compact.IncrementalStorageDiagnostics} exactFinalTree=True");
    }

    private static TaggedToolpath Cut(params Point3D[] p)=>new(p,[new(0,p.Length-1,ToolpathSpanKind.Cut)],0,"test",0,detectCurves:false);
    private static FieldInfo Field(object owner,string name)=>owner.GetType().GetField(name,BindingFlags.Instance|BindingFlags.NonPublic|BindingFlags.Public)!;
    private static object Member(object owner,string name)=>owner.GetType().GetProperty(name,BindingFlags.Instance|BindingFlags.NonPublic|BindingFlags.Public)?.GetValue(owner)
        ??Field(owner,name).GetValue(owner)!;
    private static int ListCount(object owner,string name)=>((ICollection)Field(owner,name).GetValue(owner)!).Count;
    private static long[] Snapshot(CutterSweepStock stock)
    {
        var bits=new List<long>();
        void Point(Point3D p){bits.Add(BitConverter.DoubleToInt64Bits(p.X));bits.Add(BitConverter.DoubleToInt64Bits(p.Y));bits.Add(BitConverter.DoubleToInt64Bits(p.Z));}
        void Node(object node)
        {
            bits.Add((int)Member(node,"First"));bits.Add((int)Member(node,"Count"));
            foreach(string name in new[]{"MinX","MinY","MaxX","MaxY","Depth","Radius"})bits.Add(BitConverter.DoubleToInt64Bits((double)Member(node,name)));
            var left=Field(node,"Left").GetValue(node);bits.Add(left is null?0:1);
            if(left is not null){Node(left);Node(Field(node,"Right").GetValue(node)!);}
        }
        foreach(var block in (IEnumerable)Field(stock,"blocks").GetValue(stock)!)
        {
            var rows=(Array)Member(block,"Chords");bits.Add(rows.Length);
            foreach(var row in rows){Point((Point3D)Member(row!,"A"));Point((Point3D)Member(row!,"B"));}
            var sites=(Point3D[])Member(block,"FloorSites");bits.Add(sites.Length);foreach(var p in sites)Point(p);
            Node(Member(block,"Root"));
        }
        return bits.ToArray();
    }
}
