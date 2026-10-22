using RW.Toolpaths.Geometry;

namespace RW.Toolpaths.Tests;

public sealed class SpatialMedianPartitionTests
{
    [Fact]
    public void ExactMedianAndCompleteInventoryMatchSortingForAdversarialInputs()
    {
        var random=new Random(3341);
        foreach(int length in new[]{2,3,8,19,127,1024,8193})
        foreach(int mode in Enumerable.Range(0,6))
        {
            var input=Enumerable.Range(0,length).Select(i=>mode switch{
                0=>i,1=>length-i,2=>0,3=>i%3,4=>Math.Min(i,length-i),_=>random.Next(-length,length)}).ToArray();
            var expected=input.Order().ToArray();var actual=input.ToArray();
            SpatialMedianPartition.Partition(actual,0,length,(a,b)=>a.CompareTo(b),default);
            int mid=length/2;
            Assert.Equal(expected[mid],actual[mid]);
            Assert.All(actual.Take(mid),v=>Assert.True(v<=actual[mid]));
            Assert.All(actual.Skip(mid+1),v=>Assert.True(v>=actual[mid]));
            Assert.Equal(expected,actual.Order());
        }
        var subrange=new[]{-99,8,2,3,5,1,7,88};
        SpatialMedianPartition.Partition(subrange,1,6,(a,b)=>a.CompareTo(b),default);
        Assert.Equal(-99,subrange[0]);Assert.Equal(88,subrange[^1]);Assert.Equal(5,subrange[4]);
        using var canceled=new CancellationTokenSource();canceled.Cancel();
        Assert.Throws<OperationCanceledException>(()=>SpatialMedianPartition.Partition(new[]{2,1},0,2,
            (a,b)=>a.CompareTo(b),canceled.Token));
    }
}
