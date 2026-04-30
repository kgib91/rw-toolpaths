using Clipper2Lib;

namespace RW.Toolpaths;

/// <summary>
/// Helical ramp lead-in helpers.
///
/// Port of the <c>rampset.loopIn</c> (and <c>rampset.zigZagIn</c>) functions from
/// </summary>
public static class RampUtils
{
    // --- loopIn ---------------------------------------------------------------
    //
    //   n = closed ring of XY points  (the innermost ring)
    //   t = target depth (e.g. -0.1 in)
    //   e = entry depth  (e.g.  0.0 in, the safe Z)
    //   i = rampingAngle in radians
    //
    // The function walks BACKWARDS through the ring, accumulating depth from
    // `t` upward until it reaches `e`.  The returned points are in forward
    // (entry -> depth) order, so they are prepended to the ring toolpath.

    /// <summary>
    /// Generates the helical ramp lead-in points for the innermost pocket ring.
    ///
    /// The ramp spirals down along the ring from <paramref name="entryZ"/>
    /// to <paramref name="depth"/>, following the ring contour in reverse so
    /// that it arrives at the ring's start point ready to mill forward.
    ///
    /// Port of <c>rampset.loopIn(n, t, e, i)</c>.
    /// </summary>
    /// <param name="ring">
    ///   The closed polygon ring to ramp into, in workspace XY coordinates.
    ///   Must contain at least 2 points.
    /// </param>
    /// <param name="depth">
    ///   Target milling depth (negative, e.g. <c>-0.1</c>).
    ///   Mirrors parameter <c>t</c>.
    /// </param>
    /// <param name="entryZ">
    ///   Z height at which the tool enters the ramp (e.g. <c>0.0</c>).
    ///   Mirrors parameter <c>e</c>.
    /// </param>
    /// <param name="rampingAngle">
    ///   Ramp angle in radians (> 0).
    ///   Mirrors parameter <c>i</c>.
    /// </param>
    /// <returns>
    ///   Ordered list of XYZ ramp points from entry height down to
    ///   <paramref name="depth"/>.  Concatenate this before the ring points.
    /// </returns>
    public static List<Point3D> LoopIn(
        IList<PointD> ring,
        double depth,
        double entryZ,
        double rampingAngle)
    {
        if (ring.Count < 2)
            throw new ArgumentException("Ring must have at least 2 points.", nameof(ring));

        double tanAngle = Math.Tan(rampingAngle);
        if (tanAngle == 0)
            throw new ArgumentException(
                "Cannot generate ramp for angle 0; use a non-zero rampingAngle.", nameof(rampingAngle));

        var rampPoints = new List<Point3D>();

        // Current position starts at ring[0] at the target depth
        double ox = ring[0].x, oy = ring[0].y, oz = depth;
        rampPoints.Add(new Point3D(ox, oy, oz));

        // Walk backward through the ring, accumulating z upward toward entryZ
        int l = ring.Count - 2;  // start one before the last point (ring is closed so [last] == [0])
        int maxSteps = Math.Max(8, ring.Count * 8);
        int steps = 0;
        while (oz < entryZ)
        {
            if (steps++ >= maxSteps)
            {
                // Degenerate rings with repeated points can stall z progression.
                rampPoints.Add(new Point3D(ox, oy, entryZ));
                break;
            }

            if (l == 0) l = ring.Count - 1; // wrap

            double cx = ring[l].x, cy = ring[l].y;
            double cz = oz + tanAngle * Math.Sqrt((cx - ox) * (cx - ox) + (cy - oy) * (cy - oy));

            if (cz > entryZ)
            {
                // Clamp: interpolate to the exact entry Z
                double frac = (entryZ - oz) / (cz - oz);
                cx = ox + frac * (cx - ox);
                cy = oy + frac * (cy - oy);
                cz = entryZ;
            }

            ox = cx; oy = cy; oz = cz;
            rampPoints.Add(new Point3D(ox, oy, oz));
            l--;
        }

        rampPoints.Reverse();
        return rampPoints;
    }

    // --- zigZagIn -------------------------------------------------------------
    //
    //   Used for open-path profiles; generates a zig-zag ramp along the path.
    //   Included here for completeness; pocket fill uses loopIn.

    /// <summary>
    /// Generates a zig-zag ramp along an open path.
    ///
    /// Port of <c>rampset.zigZagIn(n, t, e, i)</c>.
    /// </summary>
    /// <param name="path">Open path points.</param>
    /// <param name="entryZ">Z at the start of the ramp (e.g. 0.0).</param>
    /// <param name="depth">Target depth (negative).</param>
    /// <param name="rampingAngle">Ramp angle in radians (> 0).</param>
    public static List<Point3D> ZigZagIn(
        IList<PointD> path,
        double entryZ,
        double depth,
        double rampingAngle)
    {
        double ramp     = entryZ - depth;   // total depth change (positive)
        double tanAngle = Math.Tan(rampingAngle);

        var forward  = new List<Point3D> { new(path[0].x, path[0].y, entryZ) };
        var backward = new List<Point3D> { new(path[0].x, path[0].y, depth) };

        double accumulated   = 0;
        double totalDistance = 0;
        var    prev          = path[0];

        foreach (var cur in path.Skip(1))
        {
            double seg   = Math.Sqrt((cur.x - prev.x) * (cur.x - prev.x) + (cur.y - prev.y) * (cur.y - prev.y));
            double delta = tanAngle * seg;

            if (accumulated + delta > ramp / 2.0)
            {
                double frac = (ramp / 2.0 - accumulated) / delta;
                double mx   = prev.x + frac * (cur.x - prev.x);
                double my   = prev.y + frac * (cur.y - prev.y);
                forward.Add(new Point3D(mx, my, depth + ramp / 2.0));
                accumulated = ramp / 2.0;
                break;
            }

            accumulated  += delta;
            totalDistance += seg;
            forward.Add(new Point3D(cur.x, cur.y, entryZ - accumulated));
            backward.Insert(0, new Point3D(cur.x, cur.y, depth + accumulated));
            prev = cur;
        }

        if (accumulated < ramp / 2.0)
        {
            // Didn't reach half-way; rescale
            forward.RemoveAt(forward.Count - 1);
            backward[0] = backward[0] with { Z = depth + ramp / 2.0 };
            double ox = backward[0].X, oy = backward[0].Y, g = 0;
            for (int i = 1; i < backward.Count; i++)
            {
                g += Math.Sqrt((backward[i].X - ox) * (backward[i].X - ox) + (backward[i].Y - oy) * (backward[i].Y - oy)) / totalDistance * (ramp / 2.0);
                forward[^i]  = forward[^i]  with { Z = depth + ramp / 2.0 + g };
                backward[i]  = backward[i]  with { Z = depth + ramp / 2.0 - g };
                ox = backward[i].X; oy = backward[i].Y;
            }
        }

        var result = new List<Point3D>(forward.Count + backward.Count);
        result.AddRange(forward);
        result.AddRange(backward);
        return result;
    }
}


