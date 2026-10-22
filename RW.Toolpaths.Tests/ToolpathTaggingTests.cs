using Clipper2Lib;
using RW.Toolpaths;
using RW.Toolpaths.Milling;
using Xunit;

namespace RW.Toolpaths.Tests;

public class ToolpathTaggingTests
{
    private sealed class EmptyMedialAxisProvider : IMedialAxisProvider
    {
        public IReadOnlyList<MedialSegment> ConstructMedialAxis(
            IReadOnlyList<PointD> boundary,
            IReadOnlyList<IReadOnlyList<PointD>> holes,
            double tolerance,
            double maxRadius,
            double filteringAngle = 3 * Math.PI / 4,
            bool useBigIntegers = true)
            => Array.Empty<MedialSegment>();
    }

    [Fact]
    public void GenerateClearingPassesTagged_AssignsRegionAndDepthPassTags()
    {
        var region = new List<IReadOnlyList<PointD>>
        {
            new List<PointD>
            {
                new(0, 0),
                new(2, 0),
                new(2, 1),
                new(0, 1),
                new(0, 0),
            }
        };

        var tagged = MedialAxisToolpaths.GenerateClearingPassesTagged(
            boundary: region,
            startDepth: 0.0,
            endDepth: 0.25,
            radianTipAngle: Math.PI / 3.0,
            depthPerPass: 0.1,
            stepOver: 0.4,
            regionIndex: 7);

        Assert.NotEmpty(tagged);
        Assert.All(tagged, t => Assert.Equal("clearing", t.Category));
        Assert.All(tagged, t => Assert.Equal(7, t.RegionIndex));
        Assert.All(tagged, t => Assert.True(t.DepthPassIndex.HasValue));

        var depthPasses = tagged
            .Select(t => t.DepthPassIndex!.Value)
            .ToList();

        Assert.Equal(depthPasses.OrderBy(i => i).ToList(), depthPasses);
        Assert.All(depthPasses, i => Assert.True(i >= 0));
    }

    [Fact]
    public void GenerateClearingPasses_RemainsGeometryCompatibleWithTaggedProjection()
    {
        var region = new List<IReadOnlyList<PointD>>
        {
            new List<PointD>
            {
                new(0, 0),
                new(1, 0),
                new(1, 1),
                new(0, 1),
                new(0, 0),
            }
        };

        var untagged = MedialAxisToolpaths.GenerateClearingPasses(
            boundary: region,
            startDepth: 0.0,
            endDepth: 0.2,
            radianTipAngle: Math.PI / 3.0,
            depthPerPass: 0.1,
            stepOver: 0.4);

        var taggedProjected = MedialAxisToolpaths.GenerateClearingPassesTagged(
            boundary: region,
            startDepth: 0.0,
            endDepth: 0.2,
            radianTipAngle: Math.PI / 3.0,
            depthPerPass: 0.1,
            stepOver: 0.4)
            .Select(t => t.Points.ToList())
            .ToList();

        Assert.Equal(untagged.Count, taggedProjected.Count);
        for (int i = 0; i < untagged.Count; i++)
        {
            Assert.Equal(untagged[i].Count, taggedProjected[i].Count);
            for (int j = 0; j < untagged[i].Count; j++)
            {
                Assert.Equal(untagged[i][j], taggedProjected[i][j]);
            }
        }
    }

    [Fact]
    public void GenerateClearingPasses_WithoutInteriorFill_EmitsOnlyDepthLayerPerimeters()
    {
        var region = new List<IReadOnlyList<PointD>>
        {
            new List<PointD>
            {
                new(0, 0),
                new(2, 0),
                new(2, 1),
                new(0, 1),
                new(0, 0),
            }
        };

        var fullFill = MedialAxisToolpaths.GenerateClearingPasses(
            boundary: region,
            startDepth: 0.0,
            endDepth: 0.2,
            radianTipAngle: Math.PI / 3.0,
            depthPerPass: 0.1,
            stepOver: 0.4);

        var edgeOnly = MedialAxisToolpaths.GenerateClearingPasses(
            boundary: region,
            startDepth: 0.0,
            endDepth: 0.2,
            radianTipAngle: Math.PI / 3.0,
            depthPerPass: 0.1,
            stepOver: 0.4,
            includeInteriorFill: false);

        Assert.Equal(2, edgeOnly.Count);
        Assert.All(edgeOnly, path => Assert.True(path.Count >= 4));
        Assert.True(fullFill.Count > edgeOnly.Count);
    }

    [Fact]
    public void GenerateClearingPasses_WithoutInteriorFill_DoesNotExceedNearestEdgeDepth()
    {
        var region = new List<IReadOnlyList<PointD>>
        {
            new List<PointD>
            {
                new(0, 0),
                new(10, 0),
                new(10, 10),
                new(0, 10),
                new(0, 0),
            }
        };

        var paths = MedialAxisToolpaths.GenerateClearingPasses(
            boundary: region,
            startDepth: 0.0,
            endDepth: 2.0,
            radianTipAngle: Math.PI / 2.0,
            depthPerPass: 1.0,
            stepOver: 0.4,
            includeInteriorFill: false);

        Assert.NotEmpty(paths);
        foreach (var point in paths.SelectMany(path => path))
        {
            double nearestEdge = Math.Min(
                Math.Min(point.X, 10.0 - point.X),
                Math.Min(point.Y, 10.0 - point.Y));

            Assert.True(-point.Z <= nearestEdge + 1e-6,
                $"Clearing point ({point.X}, {point.Y}, {point.Z}) exceeds its nearest-edge depth limit {nearestEdge}.");
        }
    }

    [Fact]
    public void GenerateVCarveTaggedForRegions_AssignsSequentialRegionIndices()
    {
        var provider = new EmptyMedialAxisProvider();

        var regions = new List<IReadOnlyList<IReadOnlyList<PointD>>>
        {
            new List<IReadOnlyList<PointD>>
            {
                new List<PointD>
                {
                    new(0, 0),
                    new(1, 0),
                    new(1, 1),
                    new(0, 1),
                    new(0, 0),
                }
            },
            new List<IReadOnlyList<PointD>>
            {
                new List<PointD>
                {
                    new(3, 0),
                    new(4, 0),
                    new(4, 1),
                    new(3, 1),
                    new(3, 0),
                }
            }
        };

        var tagged = MedialAxisToolpaths.GenerateVCarveTaggedForRegions(
            provider,
            regions,
            startDepth: 0.0,
            endDepth: 0.2,
            radianTipAngle: Math.PI / 3.0,
            depthPerPass: 0.1,
            stepOver: 0.4,
            tolerance: 0.03);

        Assert.NotEmpty(tagged);

        var regionIndices = tagged
            .Select(t => t.RegionIndex)
            .Distinct()
            .OrderBy(i => i)
            .ToList();

        Assert.Equal(new List<int> { 0, 1 }, regionIndices);
    }

    [Fact]
    public void VCarveStrategy_UsesSurfaceRelativeDepthAndReportsSweptArea()
    {
        var plan = VCarveToolpaths.Generate(
            new[]
            {
                new MillingRegion(new List<PointD>
                {
                    new(0, 0),
                    new(20, 0),
                    new(20, 20),
                    new(0, 20),
                }),
            },
            new ToolGeometry(2)
            {
                TipAngleRadians = Math.PI / 2.0,
                BottomRadius = 0,
                TopRadius = 10,
                ConeLength = 10,
            },
            new DepthSchedule(4, 2) { SurfaceZ = 10 },
            new VCarveOptions { IncludeInteriorFill = false },
            new EmptyMedialAxisProvider());

        Assert.NotEmpty(plan.Toolpaths);
        Assert.NotEmpty(plan.SweptArea);
        Assert.All(plan.Toolpaths.SelectMany(path => path.Points), point =>
        {
            Assert.InRange(point.Z, 6 - 1e-6, 10 + 1e-6);
        });
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void VCarveSweptAreaOptOutPreservesCuttingGeometryAndVisits(bool includeInteriorFill)
    {
        var regions = CurveDetectionRegions();
        var tool = new ToolGeometry(0.3)
        {
            TipAngleRadians = Math.PI / 2, BottomRadius = 0,
            TopRadius = 1, ConeLength = 1
        };
        var depth = new DepthSchedule(0.25, 0.1) { SurfaceZ = 2 };
        var options = new VCarveOptions
        {
            IncludeInteriorFill = includeInteriorFill, DetectCurves = false
        };
        Assert.True(options.CalculateSweptArea);
        var accounted = VCarveToolpaths.Generate(regions, tool, depth, options,
            BoostVoronoiProvider.CreateDefault());
        var physicalOnly = VCarveToolpaths.Generate(regions, tool, depth,
            options with { CalculateSweptArea = false }, BoostVoronoiProvider.CreateDefault());

        Assert.NotEmpty(accounted.SweptArea);
        Assert.Empty(physicalOnly.SweptArea);
        Assert.Equal(accounted.Diagnostics, physicalOnly.Diagnostics);
        Assert.Equal(accounted.Toolpaths.Count, physicalOnly.Toolpaths.Count);
        for (int index = 0; index < accounted.Toolpaths.Count; index++)
        {
            var expected = accounted.Toolpaths[index];
            var actual = physicalOnly.Toolpaths[index];
            Assert.Equal(expected.Points, actual.Points);
            Assert.Equal(expected.Spans, actual.Spans);
            Assert.Equal(expected.RegionIndex, actual.RegionIndex);
            Assert.Equal(expected.DepthPassIndex, actual.DepthPassIndex);
            Assert.Equal(expected.VisitIdentity, actual.VisitIdentity);
            Assert.Equal(expected.Category, actual.Category);
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void VCarveCurveDetectionOptOutPreservesGeometryAndRouting(bool includeInteriorFill)
    {
        var regions = CurveDetectionRegions();
        var tool = new ToolGeometry(0.3)
        {
            TipAngleRadians = Math.PI / 2,
            BottomRadius = 0,
            TopRadius = 1,
            ConeLength = 1
        };
        var depth = new DepthSchedule(0.25, 0.1) { SurfaceZ = 2 };
        var options = new VCarveOptions { IncludeInteriorFill = includeInteriorFill };
        Assert.True(options.DetectCurves);

        var fitted = VCarveToolpaths.Generate(regions, tool, depth, options,
            BoostVoronoiProvider.CreateDefault());
        var linear = VCarveToolpaths.Generate(regions, tool, depth,
            options with { DetectCurves = false }, BoostVoronoiProvider.CreateDefault());

        Assert.NotEmpty(fitted.Toolpaths);
        Assert.Contains(fitted.Toolpaths, path => path.Category == "clearing");
        Assert.Contains(fitted.Toolpaths, path => path.Category == "final-carve");
        Assert.Equal(3, fitted.Diagnostics.DepthPassCount);
        Assert.Equal(2, fitted.Diagnostics.IslandCount);
        Assert.Equal(fitted.Diagnostics, linear.Diagnostics);
        Assert.Equal(fitted.Toolpaths.Count, linear.Toolpaths.Count);
        for (int index = 0; index < fitted.Toolpaths.Count; index++)
        {
            var expected = fitted.Toolpaths[index];
            var actual = linear.Toolpaths[index];
            Assert.Equal(expected.Points, actual.Points);
            Assert.Equal(expected.Spans, actual.Spans);
            Assert.Equal(expected.RegionIndex, actual.RegionIndex);
            Assert.Equal(expected.Category, actual.Category);
            Assert.Equal(expected.DepthPassIndex, actual.DepthPassIndex);
            Assert.Equal(expected.VisitIdentity, actual.VisitIdentity);
            Assert.True(expected.DetectCurves);
            Assert.False(actual.DetectCurves);
            Assert.Equal(actual.Points.Count - 1, actual.Moves.Count);
            for (int moveIndex = 0; moveIndex < actual.Moves.Count; moveIndex++)
            {
                Assert.Equal(ToolMoveKind.Line, actual.Moves[moveIndex].Kind);
                Assert.Equal(actual.Points[moveIndex], actual.Moves[moveIndex].Start);
                Assert.Equal(actual.Points[moveIndex + 1], actual.Moves[moveIndex].End);
            }
        }
        Assert.Equal(fitted.SweptArea.Count, linear.SweptArea.Count);
        for (int index = 0; index < fitted.SweptArea.Count; index++)
            Assert.Equal(fitted.SweptArea[index], linear.SweptArea[index]);
    }

    [Fact]
    public void VCarveTaggedCurveDetectionOptOutReachesClearingAndFinalPasses()
    {
        var boundary = new List<IReadOnlyList<PointD>> { CurveDetectionRegions()[0].Outer };
        var paths = MedialAxisToolpaths.GenerateVCarveTagged(
            BoostVoronoiProvider.CreateDefault(), boundary,
            startDepth: 0, endDepth: 0.25, radianTipAngle: Math.PI / 2, depthPerPass: 0.1,
            includeInteriorFill: false, detectCurves: false);

        Assert.Contains(paths, path => path.Category == "clearing");
        Assert.Contains(paths, path => path.Category == "final-carve");
        Assert.All(paths, path =>
        {
            Assert.False(path.DetectCurves);
            Assert.Equal(path.Points.Count - 1, path.Moves.Count);
            Assert.All(path.Moves, move => Assert.Equal(ToolMoveKind.Line, move.Kind));
        });
    }

    private static List<MillingRegion> CurveDetectionRegions()
    {
        var circle = Enumerable.Range(0, 128)
            .Select(index =>
            {
                double angle = index * 2 * Math.PI / 128;
                return new PointD(3 + 0.6 * Math.Cos(angle), 1 + 0.6 * Math.Sin(angle));
            })
            .ToList();
        return new List<MillingRegion>
        {
            new(new List<PointD>
            {
                new(0, 0), new(2, 0), new(2, 0.9), new(0.9, 0.9), new(0.9, 2), new(0, 2)
            }),
            new(circle)
        };
    }
}
