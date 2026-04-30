using Clipper2Lib;

namespace RW.Toolpaths;

internal static class MedialAxisSegmentEmitter
{
    private const int MaxSubdivisionIterations = 20000;

    internal enum ParabolaDiscretizationMethod
    {
        Error2D = 0,
        Radius = 1,
        CentralAngle = 2
    }

    internal sealed record class SamplePoint(double X, double Y, double Radius = 0);

    internal static bool TryCreateOutputSegments(
        MedialAxisEdgeClassifier.EdgeData edge,
        List<MedialSegment> output,
        bool noParabola = false,
        bool showSites = false,
        double tolerance = 0.1,
        ParabolaDiscretizationMethod method = ParabolaDiscretizationMethod.Error2D,
        double maxRadius = -1)
    {
        var samples = new List<SamplePoint>
        {
            new(edge.Vertex0!.X, edge.Vertex0.Y),
            new(edge.Vertex1!.X, edge.Vertex1.Y)
        };

        if (edge.IsCurved)
        {
            var pointSite = edge.Cell.ContainsPoint
                ? edge.Cell.Point!
                : edge.TwinCell.Point!;
            var segmentSite = edge.Cell.ContainsPoint
                ? edge.TwinCell.Segment!
                : edge.Cell.Segment!;

            if (!noParabola)
            {
                switch (method)
                {
                    case ParabolaDiscretizationMethod.Radius:
                        DiscretizeRadius(pointSite, segmentSite, tolerance, samples);
                        break;
                    case ParabolaDiscretizationMethod.Error2D:
                        DiscretizeError2D(pointSite, segmentSite, tolerance, samples);
                        break;
                    case ParabolaDiscretizationMethod.CentralAngle:
                        DiscretizeCentralAngle(pointSite, segmentSite, tolerance, samples);
                        break;
                }
            }

            for (int i = 0; i < samples.Count; i++)
            {
                samples[i] = samples[i] with { Radius = DistanceToPoint(pointSite, samples[i]) };
            }
        }
        else if (edge.Cell.ContainsPoint && edge.TwinCell.ContainsPoint)
        {
            var pointSite = edge.Cell.Point!;

            // Point-point Voronoi edges are linear. Keep endpoint-only emission
            // and assign radius at sampled points; recursive subdivision here
            // can fail to converge on long spans and truncate valid connectors.

            for (int i = 0; i < samples.Count; i++)
            {
                samples[i] = samples[i] with { Radius = DistanceToPoint(pointSite, samples[i]) };
            }
        }
        else
        {
            if (edge.Cell.ContainsPoint || edge.TwinCell.ContainsPoint)
            {
                // Boost can produce linear edges between a point site and a
                // segment site in degenerate configurations (for example at
                // orthogonal junctions). Dropping these edges creates visible
                // breaks in the medial graph, so keep them and assign radius
                // from the point site (equal to distance-to-segment on the
                // emitted bisector endpoints).
                var pointSite = edge.Cell.ContainsPoint
                    ? edge.Cell.Point!
                    : edge.TwinCell.Point!;

                for (int i = 0; i < samples.Count; i++)
                {
                    samples[i] = samples[i] with { Radius = DistanceToPoint(pointSite, samples[i]) };
                }
            }
            else
            {
                var segmentSite = edge.Cell.Segment!;
                samples[0] = samples[0] with { Radius = DistanceToSegment(samples[0], segmentSite) };
                samples[1] = samples[1] with { Radius = DistanceToSegment(samples[1], segmentSite) };
            }
        }

        var previous = samples[0];
        for (int i = 1; i < samples.Count; i++)
        {
            var current = samples[i];
            output.Add(new MedialSegment(
                new MedialPoint(previous.X, previous.Y, previous.Radius),
                new MedialPoint(current.X, current.Y, current.Radius)));
            previous = current;
        }

        return true;
    }

    private static void DiscretizeError2D(
        MedialAxisEdgeClassifier.EdgePoint point,
        MedialAxisEdgeClassifier.SegmentSite segment,
        double tolerance,
        List<SamplePoint> samples)
    {
        double dx = segment.High.X - segment.Low.X;
        double dy = segment.High.Y - segment.Low.Y;
        double lengthSquared = dx * dx + dy * dy;
        double startProjection = lengthSquared * GetPointProjection(samples[0], segment);
        double endProjection = lengthSquared * GetPointProjection(samples[1], segment);
        double pointDx = point.X - segment.Low.X;
        double pointDy = point.Y - segment.Low.Y;
        double projectedPoint = dx * pointDx + dy * pointDy;
        double pointDistance = dx * pointDy - dy * pointDx;

        if (pointDistance == 0)
        {
            return;
        }

        var end = samples[^1];
        samples.RemoveAt(samples.Count - 1);
        var queue = new List<double> { endProjection };
        double currentX = startProjection;
        double currentY = ParabolaY(currentX, projectedPoint, pointDistance);
        double toleranceSquaredScaled = tolerance * tolerance * lengthSquared;

        int iterations = 0;
        while (queue.Count > 0)
        {
            if (++iterations > MaxSubdivisionIterations)
            {
                Console.Error.WriteLine("[medial-axis] DiscretizeError2D iteration limit reached; truncating edge discretization.");
                break;
            }

            double targetX = queue[0];
            if (Math.Abs(targetX - currentX) < 1e-12)
            {
                queue.RemoveAt(0);
                continue;
            }

            double targetY = ParabolaY(targetX, projectedPoint, pointDistance);
            double midX = (targetY - currentY) / (targetX - currentX) * pointDistance + projectedPoint;
            if (double.IsNaN(midX) || double.IsInfinity(midX))
            {
                queue.RemoveAt(0);
                continue;
            }

            double midY = ParabolaY(midX, projectedPoint, pointDistance);
            double areaTwice = (targetY - currentY) * (midX - currentX) - (targetX - currentX) * (midY - currentY);
            areaTwice = areaTwice * areaTwice / (
                (targetY - currentY) * (targetY - currentY) +
                (targetX - currentX) * (targetX - currentX));

            if (areaTwice <= toleranceSquaredScaled)
            {
                queue.RemoveAt(0);
                double worldX = (dx * targetX - dy * targetY) / lengthSquared + segment.Low.X;
                double worldY = (dx * targetY + dy * targetX) / lengthSquared + segment.Low.Y;
                samples.Add(new SamplePoint(worldX, worldY));
                currentX = targetX;
                currentY = targetY;
            }
            else
            {
                queue.Insert(0, midX);
            }
        }

        samples[^1] = end;
    }

    private static void DiscretizeRadius(
        MedialAxisEdgeClassifier.EdgePoint point,
        MedialAxisEdgeClassifier.SegmentSite segment,
        double tolerance,
        List<SamplePoint> samples)
    {
        double dx = segment.High.X - segment.Low.X;
        double dy = segment.High.Y - segment.Low.Y;
        double lengthSquared = dx * dx + dy * dy;
        double startProjection = lengthSquared * GetPointProjection(samples[0], segment);
        double endProjection = lengthSquared * GetPointProjection(samples[1], segment);
        double pointDx = point.X - segment.Low.X;
        double pointDy = point.Y - segment.Low.Y;
        double projectedPoint = dx * pointDx + dy * pointDy;
        double pointDistance = dx * pointDy - dy * pointDx;

        if (pointDistance == 0)
        {
            return;
        }

        bool reverse = startProjection > endProjection;
        double minProjection = Math.Min(startProjection, endProjection);
        double maxProjection = Math.Max(startProjection, endProjection);
        double step = tolerance * Math.Sqrt(lengthSquared);
        var projected = new List<(double X, double Y)>();

        if (minProjection <= projectedPoint && projectedPoint <= maxProjection)
        {
            for (double x = projectedPoint, y = pointDistance / 2; minProjection < x; )
            {
                projected.Insert(0, (x, y));
                y += step;
                x = ParabolaXLow(y, projectedPoint, pointDistance);
            }

            bool first = true;
            for (double x = projectedPoint, y = pointDistance / 2; x < maxProjection; )
            {
                if (!first)
                {
                    projected.Add((x, y));
                }

                y += step;
                x = ParabolaXHigh(y, projectedPoint, pointDistance);
                first = false;
            }
        }
        else if (projectedPoint < minProjection)
        {
            bool first = true;
            for (double x = minProjection, y = ParabolaY(x, projectedPoint, pointDistance); x < maxProjection; )
            {
                if (!first)
                {
                    projected.Add((x, y));
                }

                y += step;
                x = ParabolaXHigh(y, projectedPoint, pointDistance);
                first = false;
            }
        }
        else if (maxProjection < projectedPoint)
        {
            bool first = true;
            for (double x = minProjection, y = ParabolaY(x, projectedPoint, pointDistance);
                 x < maxProjection;
                 )
            {
                if (!first)
                {
                    projected.Add((x, y));
                }

                y -= step;
                if (y < pointDistance / 2)
                {
                    break;
                }

                x = ParabolaXLow(y, projectedPoint, pointDistance);
                first = false;
            }
        }

        var end = samples[^1];
        samples.RemoveAt(samples.Count - 1);

        if (reverse)
        {
            projected.Reverse();
        }

        foreach (var sample in projected)
        {
            double worldX = (dx * sample.X - dy * sample.Y) / lengthSquared + segment.Low.X;
            double worldY = (dx * sample.Y + dy * sample.X) / lengthSquared + segment.Low.Y;
            samples.Add(new SamplePoint(worldX, worldY));
        }

        samples.Add(end);
    }

    private static void DiscretizeCentralAngle(
        MedialAxisEdgeClassifier.EdgePoint point,
        MedialAxisEdgeClassifier.SegmentSite segment,
        double tolerance,
        List<SamplePoint> samples)
    {
        double dx = segment.High.X - segment.Low.X;
        double dy = segment.High.Y - segment.Low.Y;
        double lengthSquared = dx * dx + dy * dy;
        double startProjection = lengthSquared * GetPointProjection(samples[0], segment);
        double endProjection = lengthSquared * GetPointProjection(samples[1], segment);
        double pointDx = point.X - segment.Low.X;
        double pointDy = point.Y - segment.Low.Y;
        double projectedPoint = dx * pointDx + dy * pointDy;
        double pointDistance = dx * pointDy - dy * pointDx;

        if (pointDistance == 0)
        {
            return;
        }

        var end = samples[^1];
        samples.RemoveAt(samples.Count - 1);
        var queue = new List<double> { endProjection };
        double currentX = startProjection;
        double currentY = ParabolaY(currentX, projectedPoint, pointDistance);
        double currentBaseX = projectedPoint;
        double currentBaseY = pointDistance;

        int iterations = 0;
        while (queue.Count > 0)
        {
            if (++iterations > MaxSubdivisionIterations)
            {
                Console.Error.WriteLine("[medial-axis] DiscretizeCentralAngle iteration limit reached; truncating edge discretization.");
                break;
            }

            double targetX = queue[0];
            if (Math.Abs(targetX - currentX) < 1e-12)
            {
                queue.RemoveAt(0);
                continue;
            }

            double targetY = ParabolaY(targetX, projectedPoint, pointDistance);
            double midX = (targetY - currentY) / (targetX - currentX) * pointDistance + projectedPoint;
            if (double.IsNaN(midX) || double.IsInfinity(midX))
            {
                queue.RemoveAt(0);
                continue;
            }

            double midY = ParabolaY(midX, projectedPoint, pointDistance);
            double vx1 = targetX - currentBaseX;
            double vy1 = targetY - currentBaseY;
            double vx0 = currentX - currentBaseX;
            double vy0 = currentY - currentBaseY;
            double denom = Math.Sqrt((vx1 * vx1 + vy1 * vy1) * (vx0 * vx0 + vy0 * vy0));
            double angle = 0;
            if (denom != 0)
            {
                angle = Math.Acos(Math.Clamp((vx1 * vx0 + vy1 * vy0) / denom, -1.0, 1.0));
            }

            if (angle <= tolerance)
            {
                queue.RemoveAt(0);
                double worldX = (dx * targetX - dy * targetY) / lengthSquared + segment.Low.X;
                double worldY = (dx * targetY + dy * targetX) / lengthSquared + segment.Low.Y;
                samples.Add(new SamplePoint(worldX, worldY));
                currentX = targetX;
                currentY = targetY;
            }
            else
            {
                queue.Insert(0, midX);
            }
        }

        samples[^1] = end;
    }

    private static void DiscretizePointPoint(
        MedialAxisEdgeClassifier.EdgePoint point,
        double tolerance,
        List<SamplePoint> samples)
    {
        static double Distance(double x0, double y0, double x1, double y1)
        {
            double dx = x0 - x1;
            double dy = y0 - y1;
            return Math.Sqrt(dx * dx + dy * dy);
        }

        var end = samples[^1];
        samples.RemoveAt(samples.Count - 1);
        var queue = new List<SamplePoint> { end };
        double lastX = samples[0].X;
        double lastY = samples[0].Y;

        int iterations = 0;
        while (queue.Count > 0)
        {
            if (++iterations > MaxSubdivisionIterations)
            {
                Console.Error.WriteLine("[medial-axis] DiscretizePointPoint iteration limit reached; truncating edge discretization.");
                break;
            }

            var target = queue[0];
            double midX = (lastX + target.X) / 2;
            double midY = (lastY + target.Y) / 2;
            if (Math.Abs(midX - lastX) < 1e-12 && Math.Abs(midY - lastY) < 1e-12)
            {
                queue.RemoveAt(0);
                continue;
            }

            double startDistance = Distance(lastX, lastY, point.X, point.Y);
            double midDistance = Distance(midX, midY, point.X, point.Y);
            double endDistance = Distance(target.X, target.Y, point.X, point.Y);

            if (Math.Abs(startDistance - endDistance) <= tolerance && Math.Abs(startDistance - midDistance) <= tolerance)
            {
                queue.RemoveAt(0);
                samples.Add(new SamplePoint(target.X, target.Y));
                lastX = target.X;
                lastY = target.Y;
            }
            else
            {
                queue.Insert(0, new SamplePoint(midX, midY));
            }
        }

        samples.Add(end);
    }

    private static double ParabolaY(double x, double pointProjection, double pointDistance) =>
        ((x - pointProjection) * (x - pointProjection) + pointDistance * pointDistance) / (pointDistance + pointDistance);

    private static double ParabolaXLow(double y, double pointProjection, double pointDistance)
    {
        double value = 2 * pointDistance * y - pointDistance * pointDistance;
        return pointProjection - Math.Sqrt(value);
    }

    private static double ParabolaXHigh(double y, double pointProjection, double pointDistance)
    {
        double value = 2 * pointDistance * y - pointDistance * pointDistance;
        return pointProjection + Math.Sqrt(value);
    }

    private static double GetPointProjection(SamplePoint point, MedialAxisEdgeClassifier.SegmentSite segment)
    {
        double dx = segment.High.X - segment.Low.X;
        double dy = segment.High.Y - segment.Low.Y;
        double relativeX = point.X - segment.Low.X;
        double relativeY = point.Y - segment.Low.Y;
        double lengthSquared = dx * dx + dy * dy;
        return (dx * relativeX + dy * relativeY) / lengthSquared;
    }

    private static double DistanceToPoint(MedialAxisEdgeClassifier.EdgePoint point, SamplePoint sample) =>
        Hypot(point.X - sample.X, point.Y - sample.Y);

    private static double DistanceToSegment(SamplePoint sample, MedialAxisEdgeClassifier.SegmentSite segment)
    {
        double x0 = segment.Low.X;
        double y0 = segment.Low.Y;
        double x1 = segment.High.X;
        double y1 = segment.High.Y;
        return Math.Abs((y1 - y0) * sample.X - (x1 - x0) * sample.Y + x1 * y0 - y1 * x0)
            / Hypot(y0 - y1, x0 - x1);
    }

    private static double Hypot(double x, double y) => Math.Sqrt(x * x + y * y);
}

