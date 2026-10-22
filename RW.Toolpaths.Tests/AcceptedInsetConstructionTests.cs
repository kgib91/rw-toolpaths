using System.Reflection;
using Clipper2Lib;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class AcceptedInsetConstructionTests
{
    [Fact]
    public void AcceptedFirstFrontSupportsEveryLaterTrialIncludingBacktracking()
    {
        var source = DenseRectangle(0, 0, 60, 90);
        var seeded = PolygonOps.PrepareBoundaryOffset(source, null, .001, 0,
            sourceConstructionSweepRadius: 3);
        var first = seeded.WithTolerances(1 / PathUtils.Scale, 0);
        var accepted = first.Execute(-3);
        Assert.True(seeded.TrySeedAcceptedFirstInset(accepted, -3, CancellationToken.None));
        foreach (double inset in new[] { 9d, 6d, 8d })
        {
            var candidate = seeded.Execute(-inset, compactFront: false);
            Assert.NotEmpty(candidate);
            foreach (var contour in candidate)
            foreach (var point in contour)
                Assert.True(Math.Min(Math.Min(point.x, 60 - point.x), Math.Min(point.y, 90 - point.y))
                    >= inset - 4 / PathUtils.Scale);
        }
        Assert.Equal(3, seeded.DenseInsetReuseCount);
        Assert.Equal(0, seeded.DenseSourceConstructionCount);
        var reference = PolygonOps.PrepareBoundaryOffset(source, null, .001, 0,
            sourceConstructionSweepRadius: 3);
        reference.Execute(-9, compactFront: false);
        reference.Execute(-6, compactFront: false);
        Assert.Equal(2, reference.DenseSourceConstructionCount);
        Assert.Equal(0, reference.DenseInsetReuseCount);
    }

    [Fact]
    public void SeedMustProveCompleteFilledContainmentIncludingEnclosedSourceHoles()
    {
        var source = DenseRectangle(0, 0, 60, 90);
        var hole = Rectangle(20, 30, 40, 60);
        var prepared = PolygonOps.PrepareBoundaryOffset(source, new[] { hole }, .001, 0,
            sourceConstructionSweepRadius: 3);
        // All four outer command chords are clear of the walls, but the filled
        // candidate wrongly credits the protected hole as source material.
        Assert.False(prepared.TrySeedAcceptedFirstInset(new List<List<PointD>> { Rectangle(3, 3, 57, 87) }, -3, CancellationToken.None));
        Assert.False(prepared.AcceptedInsetSeeded);
        Assert.False(prepared.TrySeedAcceptedFirstInset(new List<List<PointD>> { Rectangle(103, 103, 157, 187) }, -3, CancellationToken.None));
        Assert.False(prepared.AcceptedInsetSeeded);
    }

    [Fact]
    public void SeedMustProveOriginalWallClearanceBeforeUsingConstructionCache()
    {
        var prepared = PolygonOps.PrepareBoundaryOffset(DenseRectangle(0, 0, 60, 90), null, .001, 0,
            sourceConstructionSweepRadius: 3);
        Assert.False(prepared.TrySeedAcceptedFirstInset(new List<List<PointD>> { Rectangle(1, 1, 59, 89) }, -3, CancellationToken.None));
        Assert.False(prepared.AcceptedInsetSeeded);
        Assert.NotEmpty(prepared.Execute(-6));
        Assert.Equal(1, prepared.DenseSourceConstructionCount);
    }

    [Fact]
    public void UnprovedSeededAdvanceRepeatsEstablishedConstructionSearch()
    {
        var source = new MillingRegion(Rectangle(0, 0, 60, 90));
        var options = new RingOffsetOptions(3, 6) { PhysicalSweepRadius = 3, MinimumRingArea = 0 };
        Func<double, List<List<PointD>>> established = inset => new() { Rectangle(inset, inset, 60 - inset, 90 - inset) };
        Func<double, List<List<PointD>>> unproved = _ => new() { Rectangle(29, 44, 31, 46) };
        var method = typeof(RingOffsetEngine).GetMethod("SelectCoherentFront", BindingFlags.Static | BindingFlags.NonPublic)!;
        object Search(Func<double, List<List<PointD>>> offset, Func<double, List<List<PointD>>>? fallback)
            => method.Invoke(null, new object?[] { source, 0, 0, established(3), 3d, 9d, offset(9),
                options, offset, 0d, CancellationToken.None, null, null, null, null, fallback })!;
        var expected = Search(established, null);
        int fallbackCalls = 0;
        var recovered = Search(unproved, inset => { fallbackCalls++; return established(inset); });
        Assert.True(fallbackCalls > 0);
        Type type = recovered.GetType();
        Assert.Equal(type.GetProperty("Distance")!.GetValue(expected), type.GetProperty("Distance")!.GetValue(recovered));
        string[] Commands(object value) => ((List<List<PointD>>)type.GetProperty("Polygons")!.GetValue(value)!)
            .Select(path => string.Join(";", path.Select(point => $"{point.x:R},{point.y:R}"))).ToArray();
        Assert.Equal(Commands(expected), Commands(recovered));
    }

    private static List<PointD> DenseRectangle(double x0, double y0, double x1, double y1)
    {
        var corners = Rectangle(x0, y0, x1, y1);
        return Enumerable.Range(0, 4).SelectMany(edge => Enumerable.Range(0, 2048).Select(index =>
        {
            var a = corners[edge]; var b = corners[(edge + 1) % 4];
            double t = index / 2048d;
            return new PointD(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t);
        })).ToList();
    }

    private static List<PointD> Rectangle(double x0, double y0, double x1, double y1)
        => new() { new(x0, y0), new(x1, y0), new(x1, y1), new(x0, y1) };
}
