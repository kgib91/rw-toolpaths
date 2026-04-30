using Clipper2Lib;
using RW.Toolpaths;
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
}
