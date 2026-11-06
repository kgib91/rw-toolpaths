using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class OrderedUnionSlidingWindowTests
{
    [Fact]
    public void OrderedReadsRefillTheWindowWhileALaterOriginalLeafIsStillRunning()
    {
        using var slowStarted=new ManualResetEventSlim();
        using var releaseSlow=new ManualResetEventSlim();
        using var replacementStarted=new ManualResetEventSlim();
        using var queue=new VCarveOrderedUnionQueue(default,2,input=>{
            if(input[0][0].X==100)
            {
                slowStarted.Set();
                if(!releaseSlow.Wait(TimeSpan.FromSeconds(10)))throw new TimeoutException("slow original leaf");
            }
            else if(input[0][0].X==200)replacementStarted.Set();
            return Clipper.Union(input,FillRule.NonZero);
        },maximumBytes:1024);
        var input=Enumerable.Range(0,3).Select(i=>new Paths64{Box(i*100),Reverse(Box(i*100+10,10,20))}).ToArray();
        var first=queue.Submit(input[0]);var second=queue.Submit(input[1]);
        var published=new List<Paths64>();
        try
        {
            Assert.True(slowStarted.Wait(TimeSpan.FromSeconds(10)));
            published.Add(queue.ReadCompleted(first));
            var third=queue.Submit(input[2]);
            // A whole-wave Drain would wait for the slow second leaf and could
            // never submit this replacement before releaseSlow was signaled.
            Assert.True(replacementStarted.Wait(TimeSpan.FromSeconds(10)));
            Assert.False(releaseSlow.IsSet);
            releaseSlow.Set();
            published.Add(queue.ReadCompleted(second));
            published.Add(queue.ReadCompleted(third));
            queue.Complete();
            for(int i=0;i<input.Length;i++)Exact(Clipper.Union(input[i],FillRule.NonZero),published[i]);
            Assert.Equal(3,queue.CompletedReads);
            Assert.InRange(queue.PeakInFlight,1,2);
            Assert.InRange(queue.PeakRetainedInputBytes,1,1024);
            Assert.True(queue.OwnedWorkersJoined);
        }
        finally { releaseSlow.Set(); }
    }

    [Fact]
    public void CompletedReadsRetainTheOversizedLiteralFallback()
    {
        using var queue=new VCarveOrderedUnionQueue(default,2,maximumBytes:1024);
        var path=Box(0);path.Capacity=256;
        var input=new Paths64{path};
        var slot=queue.Submit(input);
        Exact(Clipper.Union(input,FillRule.NonZero),queue.ReadCompleted(slot));
        queue.Complete();
        Assert.Equal(1,queue.LiteralFallbacks);
        Assert.Equal(1,queue.CompletedReads);
        Assert.Equal(0,queue.Submitted);
        Assert.True(queue.OwnedWorkersJoined);
    }

    [Fact]
    public void CancelledCompletedReadPublishesNoResultAndDisposalJoinsEveryWorker()
    {
        using var cancellation=new CancellationTokenSource();
        using var started=new ManualResetEventSlim();
        using var release=new ManualResetEventSlim();
        var queue=new VCarveOrderedUnionQueue(cancellation.Token,2,input=>{
            started.Set();
            if(!release.Wait(TimeSpan.FromSeconds(10)))throw new TimeoutException("cancelled leaf control");
            return Clipper.Union(input,FillRule.NonZero);
        });
        try
        {
            var slot=queue.Submit(new Paths64{Box(0)});
            Assert.True(started.Wait(TimeSpan.FromSeconds(10)));
            cancellation.Cancel();
            Assert.ThrowsAny<OperationCanceledException>(()=>queue.ReadCompleted(slot));
            Assert.Equal(0,queue.CompletedReads);
        }
        finally { release.Set();queue.Dispose(); }
        Assert.True(queue.OwnedWorkersJoined);
    }

    [Fact]
    public void FailedCompletedReadPreservesTheOriginalErrorAndJoinsBeforeExit()
    {
        var expected=new FormatException("original leaf failed");
        var queue=new VCarveOrderedUnionQueue(default,2,_=>throw expected);
        try
        {
            var slot=queue.Submit(new Paths64{Box(0)});
            Assert.Same(expected,Assert.Throws<FormatException>(()=>queue.ReadCompleted(slot)));
            Assert.Same(expected,Assert.Throws<FormatException>(()=>queue.Complete()));
            Assert.Equal(0,queue.CompletedReads);
        }
        finally { queue.Dispose(); }
        Assert.True(queue.OwnedWorkersJoined);
    }

    private static Path64 Box(long left,long bottom=0,long size=40)
        => new(){new(left,bottom),new(left+size,bottom),new(left+size,bottom+size),new(left,bottom+size)};
    private static Path64 Reverse(Path64 path){path.Reverse();return path;}
    private static void Exact(Paths64 expected,Paths64 actual)
    {
        Assert.Equal(expected.Count,actual.Count);
        for(int i=0;i<expected.Count;i++)Assert.Equal(expected[i],actual[i]);
    }
}
