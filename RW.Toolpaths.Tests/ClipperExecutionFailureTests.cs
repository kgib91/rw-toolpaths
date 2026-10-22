using System.Collections;
using System.Reflection;
using Clipper2Lib;

namespace RW.Toolpaths.Tests;

public sealed class ClipperExecutionFailureTests
{
    private static readonly Type Guard = typeof(Clipper).Assembly.GetType("Clipper2Lib.ClipperExecution", true)!;
    private static readonly MethodInfo IntegerGuard = Guard.GetMethod("RequireSucceeded", BindingFlags.NonPublic | BindingFlags.Static,
        null, new[] { typeof(bool), typeof(Paths64), typeof(PolyTree64), typeof(ClipType), typeof(string) }, null)!;
    private static readonly MethodInfo DoubleGuard = Guard.GetMethod("RequireSucceeded", BindingFlags.NonPublic | BindingFlags.Static,
        null, new[] { typeof(bool), typeof(PathsD), typeof(PolyTreeD), typeof(ClipType), typeof(string) }, null)!;

    [Theory]
    [InlineData("BooleanOp", false, false)]
    [InlineData("BooleanOp", false, true)]
    [InlineData("BooleanOp", true, false)]
    [InlineData("BooleanOp", true, true)]
    [InlineData("Offset cleanup", false, false)]
    [InlineData("Offset cleanup", false, true)]
    [InlineData("Offset cleanup", true, false)]
    [InlineData("Offset cleanup", true, true)]
    public void RefusedIntegerClosedResultsNeverSupplyEmptyOrPartialGeometry(string context, bool useTree, bool partial)
    {
        var paths = partial ? SignedSource() : new Paths64();
        var tree = new PolyTree64();
        if (partial && useTree)
        {
            var outer = tree.AddChild(SignedSource()[0]);
            outer.AddChild(SignedSource()[1]);
        }
        // Offset tree cleanup also owns an intermediate flat path buffer.
        Paths64? suppliedPaths = !useTree || context == "Offset cleanup" ? paths : null;
        PolyTree64? suppliedTree = useTree ? tree : null;
        var operation = context == "BooleanOp" ? ClipType.Difference : ClipType.Union;

        var invocation = Assert.Throws<TargetInvocationException>(() => IntegerGuard.Invoke(null,
            new object?[] { false, suppliedPaths, suppliedTree, operation, context }));
        var failure = Assert.IsType<InvalidOperationException>(invocation.InnerException);
        Assert.Contains(context, failure.Message);
        Assert.Contains(operation.ToString(), failure.Message);
        Assert.Contains("partial closed output was discarded", failure.Message);
        if (suppliedPaths is not null) Assert.Empty(suppliedPaths);
        if (suppliedTree is not null) Assert.Equal(0, suppliedTree.Count);
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void RefusedDoubleBooleanResultsDiscardEmptyAndPartialOutputs(bool useTree, bool partial)
    {
        var paths = partial ? Clipper.ScalePathsD(SignedSource(), .00000001) : new PathsD();
        var tree = new PolyTreeD();
        if (partial && useTree) ((PolyPathD)tree.AddChild(paths[0])).AddChild(paths[1]);
        var invocation = Assert.Throws<TargetInvocationException>(() => DoubleGuard.Invoke(null,
            new object?[] { false, useTree ? null : paths, useTree ? tree : null, ClipType.Difference, "BooleanOp" }));
        Assert.IsType<InvalidOperationException>(invocation.InnerException);
        if (useTree) Assert.Equal(0, tree.Count); else Assert.Empty(paths);
    }

    [Fact]
    public void SuccessfulCheckRetainsExactOrderedPathsAndNestedTree()
    {
        var paths = SignedSource();
        var tree = new PolyTree64();
        tree.AddChild(paths[0]).AddChild(paths[1]).AddChild(paths[2]);
        var beforePaths = Capture(paths);
        var beforeTree = Capture(tree);
        var firstPath = paths[0];
        var firstChild = tree.Child(0);

        IntegerGuard.Invoke(null, new object?[] { true, paths, tree, ClipType.Union, "Offset cleanup" });

        Assert.Same(firstPath, paths[0]);
        Assert.Same(firstChild, tree.Child(0));
        Assert.Equal(beforePaths, Capture(paths));
        Assert.Equal(beforeTree, Capture(tree));
    }

    [Theory]
    [InlineData(ClipType.Intersection, FillRule.NonZero)]
    [InlineData(ClipType.Union, FillRule.NonZero)]
    [InlineData(ClipType.Difference, FillRule.NonZero)]
    [InlineData(ClipType.Xor, FillRule.NonZero)]
    [InlineData(ClipType.Intersection, FillRule.EvenOdd)]
    [InlineData(ClipType.Union, FillRule.EvenOdd)]
    [InlineData(ClipType.Difference, FillRule.EvenOdd)]
    [InlineData(ClipType.Xor, FillRule.EvenOdd)]
    public void SuccessfulIntegerBooleanWrappersMatchDirectOrderedExecution(ClipType operation, FillRule rule)
    {
        var source = SignedSource();
        var clip = new Paths64 { Rectangle(-150, -30, 30, 120), Rectangle(90, -100, 110, 100) };
        var direct = new Clipper64();
        direct.AddSubject(source); direct.AddClip(clip);
        var expected = new Paths64();
        Assert.True(direct.Execute(operation, rule, expected));
        Assert.Equal(Capture(expected), Capture(Clipper.BooleanOp(operation, source, clip, rule)));

        var directTree = new Clipper64();
        directTree.AddSubject(source); directTree.AddClip(clip);
        var expectedTree = new PolyTree64();
        Assert.True(directTree.Execute(operation, rule, expectedTree));
        var actualTree = new PolyTree64();
        Clipper.BooleanOp(operation, source, clip, actualTree, rule);
        Assert.Equal(Capture(expectedTree), Capture(actualTree));
    }

    [Theory]
    [InlineData(ClipType.Difference)]
    [InlineData(ClipType.Xor)]
    public void SuccessfulPrecisionEightBooleanWrappersRetainOrderedDoubleBits(ClipType operation)
    {
        var source = Clipper.ScalePathsD(SignedSource(), .00000001);
        // Both signs and half-grid input expose the existing rounding policy.
        source[0][0] = new PointD(-.000001005, -.000000995);
        var clip = new PathsD { new PathD { new(-.000001505, -.000000305), new(.000000305, -.000000305),
            new(.000000305, .000001205), new(-.000001505, .000001205) } };
        var direct = new ClipperD(8); direct.AddSubject(source); direct.AddClip(clip);
        var expected = new PathsD();
        Assert.True(direct.Execute(operation, FillRule.NonZero, expected));
        Assert.Equal(Capture(expected), Capture(Clipper.BooleanOp(operation, source, clip, FillRule.NonZero, 8)));

        var directTree = new ClipperD(8); directTree.AddSubject(source); directTree.AddClip(clip);
        var expectedTree = new PolyTreeD();
        Assert.True(directTree.Execute(operation, FillRule.NonZero, expectedTree));
        var actualTree = new PolyTreeD();
        Clipper.BooleanOp(operation, source, clip, actualTree, FillRule.NonZero, 8);
        Assert.Equal(Capture(expectedTree), Capture(actualTree));
    }

    [Theory]
    [InlineData(-7d, JoinType.Miter)]
    [InlineData(7d, JoinType.Miter)]
    [InlineData(-7d, JoinType.Round)]
    [InlineData(7d, JoinType.Round)]
    public void SuccessfulOffsetCleanupMatchesEstablishedOrderedUnion(double delta, JoinType join)
    {
        var source = SignedSource();
        var reference = PrepareUnchangedOffsetCleanup(source, delta, join);
        var expected = new Paths64();
        Assert.True(reference.Execute(ClipType.Union, FillRule.Positive, expected));
        var offset = NewOffset(); offset.AddPaths(source, join, EndType.Polygon);
        var actual = new Paths64(); offset.Execute(delta, actual);
        Assert.Equal(Capture(expected), Capture(actual));

        var referenceTree = PrepareUnchangedOffsetCleanup(source, delta, join);
        var expectedTree = new PolyTree64();
        Assert.True(referenceTree.Execute(ClipType.Union, FillRule.Positive, expectedTree));
        var treeOffset = NewOffset(); treeOffset.AddPaths(source, join, EndType.Polygon);
        var actualTree = new PolyTree64(); treeOffset.Execute(delta, actualTree);
        Assert.Equal(Capture(expectedTree), Capture(actualTree));
    }

    // Replays the pre-existing raw construction and cleanup Union directly.
    // Only the new result check is omitted; all geometry/order stays unchanged.
    private static Clipper64 PrepareUnchangedOffsetCleanup(Paths64 source, double delta, JoinType join)
    {
        var offset = NewOffset(); offset.AddPaths(source, join, EndType.Polygon);
        const BindingFlags flags = BindingFlags.Instance | BindingFlags.NonPublic;
        typeof(ClipperOffset).GetField("_delta", flags)!.SetValue(offset, delta);
        typeof(ClipperOffset).GetField("_mitLimSqr", flags)!.SetValue(offset, 2 / (offset.MiterLimit * offset.MiterLimit));
        var construct = typeof(ClipperOffset).GetMethod("DoGroupOffset", flags)!;
        foreach (var group in (IEnumerable)typeof(ClipperOffset).GetField("_groupList", flags)!.GetValue(offset)!)
            construct.Invoke(offset, new[] { group });
        var raw = (Paths64)typeof(ClipperOffset).GetField("_solution", flags)!.GetValue(offset)!;
        bool reversed = (bool)typeof(ClipperOffset).GetMethod("CheckPathsReversed", flags)!.Invoke(offset, null)!;
        Assert.False(reversed);
        var cleanup = new Clipper64 { PreserveCollinear = offset.PreserveCollinear,
            ReverseSolution = offset.ReverseSolution != reversed };
        cleanup.AddSubject(raw);
        return cleanup;
    }

    private static ClipperOffset NewOffset() => new(2, .25, preserveCollinear: true, reverseSolution: true);
    private static Paths64 SignedSource()
    {
        var hole = Rectangle(-40, -40, 40, 40); hole.Reverse();
        return new Paths64 { Rectangle(-100, -100, 100, 100), hole, Rectangle(-10, -10, 10, 10),
            Rectangle(120, -50, 160, 50) };
    }
    private static Path64 Rectangle(long x0, long y0, long x1, long y1)
        => new() { new(x0, y0), new(x1, y0), new(x1, y1), new(x0, y1) };

    private static long[] Capture(Paths64 paths)
    {
        var values = new List<long> { paths.Count };
        foreach (var path in paths) { values.Add(path.Count); foreach (var point in path) { values.Add(point.X); values.Add(point.Y); } }
        return values.ToArray();
    }
    private static long[] Capture(PathsD paths)
    {
        var values = new List<long> { paths.Count };
        foreach (var path in paths) { values.Add(path.Count); foreach (var point in path)
            { values.Add(BitConverter.DoubleToInt64Bits(point.x)); values.Add(BitConverter.DoubleToInt64Bits(point.y)); } }
        return values.ToArray();
    }
    private static long[] Capture(PolyPath64 tree)
    {
        var values = new List<long>();
        void Append(PolyPath64 node)
        {
            values.Add(node.Polygon is null ? -1 : node.Polygon.Count);
            if (node.Polygon is not null) foreach (var p in node.Polygon) { values.Add(p.X); values.Add(p.Y); }
            values.Add(node.Count); for (int i = 0; i < node.Count; i++) Append(node.Child(i));
        }
        Append(tree); return values.ToArray();
    }
    private static long[] Capture(PolyPathD tree)
    {
        var values = new List<long>();
        void Append(PolyPathD node)
        {
            values.Add(node.Polygon is null ? -1 : node.Polygon.Count);
            if (node.Polygon is not null) foreach (var p in node.Polygon)
                { values.Add(BitConverter.DoubleToInt64Bits(p.x)); values.Add(BitConverter.DoubleToInt64Bits(p.y)); }
            values.Add(node.Count); for (int i = 0; i < node.Count; i++) Append(node[i]);
        }
        Append(tree); return values.ToArray();
    }
}
