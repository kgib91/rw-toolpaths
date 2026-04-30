using RW.Toolpaths;

namespace RW.Toolpaths.Tests;

public class MedialAxisSegmentEmitterTests
{
    [Fact]
    public void TryCreateOutputSegments_LinearPointSegmentEdge_IsEmitted()
    {
        var edge = new MedialAxisEdgeClassifier.EdgeData(
            Index: 0,
            TwinIndex: -1,
            IsPrimary: true,
            IsSecondary: false,
            IsInfinite: false,
            PrevIndex: -1,
            NextIndex: -1,
            Vertex0: new MedialAxisEdgeClassifier.EdgePoint(-1, 0),
            Vertex1: new MedialAxisEdgeClassifier.EdgePoint(1, 0),
            Cell: new MedialAxisEdgeClassifier.CellSite(
                ContainsPoint: true,
                Point: new MedialAxisEdgeClassifier.EdgePoint(0, 0),
                Segment: null),
            TwinCell: new MedialAxisEdgeClassifier.CellSite(
                ContainsPoint: false,
                Point: null,
                Segment: new MedialAxisEdgeClassifier.SegmentSite(
                    new MedialAxisEdgeClassifier.EdgePoint(-1, 1),
                    new MedialAxisEdgeClassifier.EdgePoint(1, 1))))
        {
            IsLinear = true,
            IsCurved = false
        };

        var output = new List<MedialSegment>();

        bool emitted = MedialAxisSegmentEmitter.TryCreateOutputSegments(
            edge,
            output,
            noParabola: false,
            showSites: false,
            tolerance: 0.03,
            method: MedialAxisSegmentEmitter.ParabolaDiscretizationMethod.CentralAngle,
            maxRadius: 1.0);

        Assert.True(emitted);
        Assert.Single(output);

        var segment = output[0];
        Assert.Equal(-1.0, segment.Point0.X, 10);
        Assert.Equal(0.0, segment.Point0.Y, 10);
        Assert.Equal(1.0, segment.Point0.Radius, 10);

        Assert.Equal(1.0, segment.Point1.X, 10);
        Assert.Equal(0.0, segment.Point1.Y, 10);
        Assert.Equal(1.0, segment.Point1.Radius, 10);
    }
}
