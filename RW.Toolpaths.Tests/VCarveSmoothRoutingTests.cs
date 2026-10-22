using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class VCarveSmoothRoutingTests
{
    [Theory]
    [InlineData(0)]
    [InlineData(4)]
    public void SmoothVisitsCarryCompleteSurfaceRampsEvenWithAuthoredStartingFloor(double startingDepth)
    {
        const double surface = 12.7;
        var sources = new[]
        {
            Path([new(5, 5, surface - 1), new(15, 5, surface - 2)]),
            Path([new(15, 5.1, surface - 2), new(5, 5.1, surface - 1)])
        };
        var diagnostics = new List<string>();
        var result = VCarvePathRouter.Route(sources,
            [new PointD[] { new(0, 0), new(20, 0), new(20, 10), new(0, 10) }],
            new ToolGeometry(3) { BottomRadius = 0, TopRadius = 3, ConeLength = 20, TipAngleRadians = 2 * Math.Atan(.15) },
            new DepthSchedule(4, 2) { SurfaceZ = surface, StartDepth = startingDepth },
            new VCarveOptions
            {
                SmoothVCarving = true, SmoothRampAngleDegrees = 15,
                IncludeInteriorFill = true, Ramp = RampSettings.Plunge, DetectCurves = false
            }, 0, null, diagnostic: diagnostics.Add);
        Assert.Equal(sources.Length, result.Count);
        Assert.All(result, path =>
        {
            Assert.Equal(surface, path.Points[0].Z);
            Assert.Contains(path.Spans, span => span.Kind == ToolpathSpanKind.Ramp);
            Assert.DoesNotContain(path.Spans, span => span.Kind == ToolpathSpanKind.Link);
            Assert.All(path.Points, point => Assert.Contains(sources.SelectMany(source => source.Points),
                source => source.X == point.X && source.Y == point.Y));
            for (int i = 1; i < path.Points.Count; i++)
            {
                var a = path.Points[i - 1]; var b = path.Points[i];
                double length = Math.Sqrt(Math.Pow(a.X - b.X, 2) + Math.Pow(a.Y - b.Y, 2));
                Assert.True(Math.Abs(a.Z - b.Z) <= length * Math.Tan(Math.PI / 12) + 1e-10,
                    $"Grade violation: {a}->{b}");
            }
        });
        Assert.Contains(diagnostics, message => message.Contains("smooth=True") && message.Contains("event=complete"));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CertifiedEntryKeepsSourceVerticesAcrossRepeatedTraversals(bool closed)
    {
        var source = new List<Point3D> { new(.0031, .0027, -1), new(.2037, .0027, -1) };
        if (closed)
        {
            source.Add(new(.2037, .2032, -1));
            source.Add(source[0]);
        }
        double angle = Math.PI / 12, grade = Math.Tan(angle);
        var result = RampPlanner.PlanPathEntry(source, 0,
            new RampSettings(angle) { Strategy = RampStrategy.Linear },
            CancellationToken.None, diagnostic: null, landAtSourceVertex: true,
            preserveClosedExit: false);
        Assert.Equal(0, result.Points[0].Z);
        Assert.True(result.Points.Count > source.Count * 2);
        Assert.All(result.Points, point => Assert.Contains(source,
            candidate => candidate.X == point.X && candidate.Y == point.Y));
        for (int i = 1; i < result.Points.Count; i++)
        {
            var a = result.Points[i - 1]; var b = result.Points[i];
            double length = Math.Sqrt(Math.Pow(a.X - b.X, 2) + Math.Pow(a.Y - b.Y, 2));
            Assert.True(Math.Abs(a.Z - b.Z) <= length * grade + 1e-10);
            Assert.True(a.Z >= -1 && b.Z >= -1);
        }
        if (closed) RampEntryStockTests.AssertFullDirectedSourceChords(source, result);
        else Assert.Equal(source, result.Points.Skip(result.Points.Count - source.Count));
    }

    private static TaggedToolpath Path(IReadOnlyList<Point3D> points)
        => new(points, Array.Empty<ToolpathSpan>(), 0, "clearing", 0, detectCurves: false);
}
