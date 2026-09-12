using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public class CarvingSweptAreaTests
{
    [Fact]
    public void OmittingOptionalAreaPreservesEveryRoughingMoveAndDepth()
    {
        const int size = 33;
        var heights = Enumerable.Range(0, size * size).Select(i =>
        {
            double dx = i % size - 16, dy = i / size - 16;
            return (float)(0.1 + 0.7 * Math.Exp(-(dx * dx + dy * dy) / 30));
        }).ToArray();
        var surface = new HeightmapSurface(new HeightmapGrid(size, size, heights),
            new HeightmapPlacement(50, 40, 0, 0), 10, 6);
        var tool = new ToolGeometry(2);
        var options = new CarvingOptions { StepOver = 0.4, SurfaceAllowance = 0.2 };
        var withArea = CarvingRoughingToolpaths.Generate(surface, tool, 2, options);
        var pathsOnly = CarvingRoughingToolpaths.Generate(surface, tool, 2,
            options with { CalculateSweptArea = false });

        Assert.NotEmpty(withArea.SweptArea);
        Assert.Empty(pathsOnly.SweptArea);
        Assert.Equal(withArea.Diagnostics, pathsOnly.Diagnostics);
        Assert.Equal(withArea.Toolpaths.Count, pathsOnly.Toolpaths.Count);
        for (int i = 0; i < withArea.Toolpaths.Count; i++)
        {
            Assert.Equal(withArea.Toolpaths[i].Points, pathsOnly.Toolpaths[i].Points);
            Assert.Equal(withArea.Toolpaths[i].Spans, pathsOnly.Toolpaths[i].Spans);
            Assert.Equal(withArea.Toolpaths[i].DepthPassIndex, pathsOnly.Toolpaths[i].DepthPassIndex);
            Assert.Equal(withArea.Toolpaths[i].VisitIdentity, pathsOnly.Toolpaths[i].VisitIdentity);
        }
    }
}
