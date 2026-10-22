using System.Reflection;
using Clipper2Lib;

namespace RW.Toolpaths.Tests;

public sealed class ClipperScanlineQueueTests
{
    private static readonly MethodInfo Insert = typeof(ClipperBase).GetMethod("InsertScanline", BindingFlags.NonPublic | BindingFlags.Instance)!;
    private static readonly MethodInfo Pop = typeof(ClipperBase).GetMethod("PopScanline", BindingFlags.NonPublic | BindingFlags.Instance)!;
    private static readonly MethodInfo Reset = typeof(ClipperBase).GetMethod("Reset", BindingFlags.NonPublic | BindingFlags.Instance)!;

    [Fact]
    public void MixedEventsHaveTheExactDistinctDescendingSequenceAndSupportReuse()
    {
        var engine = new Clipper64();
        var expected = new SortedSet<long>();
        var random = new Random(70317);
        long current = long.MaxValue;
        for (int round = 0; round < 2; round++)
        {
            Reset.Invoke(engine, null);
            current = long.MaxValue;
            foreach (long value in new[] { long.MinValue, long.MaxValue, long.MaxValue, 0L }) Add(value);
            for (int eventNumber = 0; eventNumber < 20_000; eventNumber++)
            {
                // Real events are at or below the last consumed scanline.
                Add(Math.Min(current, random.NextInt64(-1_000_000, 1_000_001)));
                if (eventNumber % 5 == 0) Consume();
            }
            while (expected.Count > 0) Consume();
            object[] arguments = [0L];
            Assert.False((bool)Pop.Invoke(engine, arguments)!);
            engine.Clear();
        }
        void Add(long y) { Insert.Invoke(engine, new object[] { y }); expected.Add(y); }
        void Consume()
        {
            object[] arguments = [0L];
            Assert.True((bool)Pop.Invoke(engine, arguments)!);
            current = (long)arguments[0];
            Assert.Equal(expected.Max, current);
            expected.Remove(current);
        }
    }

    [Fact]
    public void ResetMinimaAndInsertedEventsUseTheSameOrdering()
    {
        var engine = new Clipper64();
        for (int i = 0; i < 1000; i++)
            engine.AddSubject(new Path64 { new(i * 4, 0), new(i * 4 + 2, 0),
                new(i * 4 + 2, i + 10), new(i * 4, i + 10) });
        for (int execution = 0; execution < 2; execution++)
        {
            Reset.Invoke(engine, null);
            Insert.Invoke(engine, new object[] { 2000L });
            Insert.Invoke(engine, new object[] { 10L });
            var actual = new List<long>();
            object[] arguments = [0L];
            while ((bool)Pop.Invoke(engine, arguments)!) actual.Add((long)arguments[0]);
            Assert.Equal(new[] { 2000L }.Concat(Enumerable.Range(10, 1000).Reverse().Select(i => (long)i)), actual);
        }
    }
}
