using System.Runtime.CompilerServices;
using Clipper2Lib;
using HashCode = Clipper2Lib.HashCode;

namespace RW.Toolpaths.Tests;

public sealed class ClipperPoint64EqualityTests
{
    private static readonly Point64[] Values =
    [new(0,0),new(-1,1),new(1,-1),new(long.MinValue,long.MaxValue),
        new(long.MaxValue,long.MinValue),new(long.MinValue,long.MinValue),new(long.MaxValue,long.MaxValue),new(17,17)];

    [Fact]
    public void TypedObjectOperatorsAndGenericContainersKeepTheLiteralXyContract()
    {
        Assert.True(typeof(IEquatable<Point64>).IsAssignableFrom(typeof(Point64)));
        foreach(var left in Values)
        foreach(var right in Values.Append(left))
        {
            bool expected=left.X==right.X&&left.Y==right.Y;
            Assert.Equal(expected,left==right);Assert.Equal(!expected,left!=right);
            Assert.Equal(expected,left.Equals(right));Assert.Equal(expected,left.Equals((object)right));
            Assert.Equal(expected,((IEquatable<Point64>)left).Equals(right));
            Assert.Equal(expected,EqualityComparer<Point64>.Default.Equals(left,right));
            Assert.Equal(HashCode.Combine(left.X,left.Y),left.GetHashCode());
            if(expected)Assert.Equal(left.GetHashCode(),right.GetHashCode());
        }
        foreach(var value in Values)
        {
            Assert.False(value.Equals(null));Assert.False(value.Equals((object)new PointD(value.X,value.Y)));
            Assert.False(value.Equals((object)(value.X,value.Y)));
        }
        var duplicates=Values.Concat(Values.Reverse()).Concat([new Point64(0,0)]).ToArray();
        Assert.Equal(Values,duplicates.Distinct());
        var set=new HashSet<Point64>(duplicates);Assert.Equal(Values.Length,set.Count);
        var map=Values.Select((p,i)=>(p,i)).ToDictionary(pair=>pair.p,pair=>pair.i);
        for(int i=0;i<Values.Length;i++)
        {
            Assert.True(set.Contains(new Point64(Values[i])));
            Assert.Equal(i,map[new Point64(Values[i])]);
            Assert.Equal(i,new List<Point64>(Values).IndexOf(new Point64(Values[i])));
        }
    }

    [Fact]
    public void OptionalZDataKeepsTheExistingXyOnlyEqualityAndHash()
    {
        var original=new Point64(-7,11);object boxed=original;
        // This also exercises the Z build without giving Z any new equality
        // meaning: the existing operators and hash have always used XY only.
        typeof(Point64).GetField("Z")?.SetValue(boxed,long.MinValue);
        var changed=(Point64)boxed;
        Assert.True(original==changed);Assert.True(original.Equals(changed));
        Assert.True(original.Equals((object)changed));Assert.Equal(original.GetHashCode(),changed.GetHashCode());
        Assert.Single(new HashSet<Point64>{original,changed});
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static int CountMatches(Point64[] values,int repetitions)
    {
        int result=0;var comparer=EqualityComparer<Point64>.Default;
        for(int repeat=0;repeat<repetitions;repeat++)
        for(int i=0;i<values.Length;i++)
            if(comparer.Equals(values[i],values[(i+repeat)%values.Length]))result++;
        return result;
    }

    private readonly struct LegacyPoint
    {
        internal readonly Point64 Value;
        internal LegacyPoint(Point64 value)=>Value=value;
        // Literal former Point64 contract, intentionally without IEquatable.
        [MethodImpl(MethodImplOptions.NoInlining)]
        public override bool Equals(object? other)=>other is LegacyPoint point&&Value==point.Value;
        public override int GetHashCode()=>HashCode.Combine(Value.X,Value.Y);
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static int CountLegacyMatches(LegacyPoint[] values,int repetitions)
    {
        int result=0;var comparer=EqualityComparer<LegacyPoint>.Default;
        for(int repeat=0;repeat<repetitions;repeat++)
        for(int i=0;i<values.Length;i++)
            if(comparer.Equals(values[i],values[(i+repeat)%values.Length]))result++;
        return result;
    }

    [Fact]
    public void WarmedDiscHullComparerPathDoesNotAllocateBoxes()
    {
        // FinishHull uses this exact generic comparer for every merged disc
        // vertex. Inputs and comparer are warmed before the measured loop.
        var values=Values.ToArray();var legacy=values.Select(p=>new LegacyPoint(p)).ToArray();
        for(int warm=0;warm<32;warm++){CountMatches(values,64);CountLegacyMatches(legacy,64);}
        const int repetitions=16_384;
        long before=GC.GetAllocatedBytesForCurrentThread();int matches=CountMatches(values,repetitions);
        long typedBytes=GC.GetAllocatedBytesForCurrentThread()-before;
        before=GC.GetAllocatedBytesForCurrentThread();int oldMatches=CountLegacyMatches(legacy,repetitions);
        long legacyBytes=GC.GetAllocatedBytesForCurrentThread()-before;
        Assert.Equal(oldMatches,matches);Assert.True(matches>0);Assert.Equal(0,typedBytes);
        Assert.True(legacyBytes>=16L*values.Length*repetitions,$"Legacy boxing control allocated only {legacyBytes} bytes.");
        GC.KeepAlive(values);GC.KeepAlive(legacy);
    }
}
