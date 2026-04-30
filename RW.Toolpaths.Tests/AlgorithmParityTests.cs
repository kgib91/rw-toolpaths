// --- AlgorithmParityTests.cs -------------------------------------------------
//
// Tests the C# port of each JS algorithm function against known exact outputs
// derived from running the equivalent JS code in test_toolpaths.js.
//
// Matching Node.js test: test_toolpaths.js (workspace root)
// Run: node test_toolpaths.js   -> prints JSON
// Run: dotnet test              -> must produce identical numeric results
//
// Coverage:
//   x_  -> TrimSegment
//   __  -> JoinSegments
//   iC  -> JoinIntoPolylines
//   gh  -> GenerateSimplifiedLayer (morphological pass)
//   rC  -> GenerateFlatAreaFill
// -----------------------------------------------------------------------------

using Clipper2Lib;
using RW.Toolpaths;
using Xunit;
using Xunit.Abstractions;

namespace RW.Toolpaths.Tests;

// helper for comparing floating-point values with tolerance
file static class Tol
{
    public const double Eps = 1e-9;   // strict for known-exact values
    public const double Loose = 1e-5; // for clipped/interpolated values

    public static void Equal(double expected, double actual, double tolerance = Eps,
        string? label = null)
    {
        Assert.True(Math.Abs(expected - actual) <= tolerance,
            $"{label ?? "value"}: expected {expected} Â± {tolerance}, got {actual}");
    }
}

// --- TrimSegment (x_) --------------------------------------------------------
public class TrimSegmentTests
{
    // Helper: build a MedialSegment from (x0,y0,z0) -> (x1,y1,z1)
    // Z is stored in the Radius field per the port convention.
    private static MedialSegment Seg(double x0, double y0, double z0,
                                     double x1, double y1, double z1)
        => new(new MedialPoint(x0, y0, z0), new MedialPoint(x1, y1, z1));

    // -- JS:  n.point0.z >= t && n.point1.z >= t  -> return n unchanged
    [Fact]
    public void BothAbove_ReturnsSameSegment()
    {
        var seg   = Seg(0, 0, -0.1, 1, 0, -0.2);
        var flat  = new List<(PointD, PointD)>();
        var result = MedialAxisToolpaths.TrimSegment(seg, -0.5, flat);

        Assert.NotNull(result);
        Assert.Equal(seg, result.Value);
        Assert.Empty(flat);
    }

    // -- JS:  n.point0.z <= t && n.point1.z <= t  -> push flat stroke, return null
    [Fact]
    public void BothBelow_ReturnsNull_AddsFlatStroke()
    {
        var seg  = Seg(0, 0, -0.6, 1, 0, -0.8);
        var flat = new List<(PointD, PointD)>();
        var result = MedialAxisToolpaths.TrimSegment(seg, -0.5, flat);

        Assert.Null(result);
        Assert.Single(flat);

        var (a, b) = flat[0];
        Tol.Equal(0.0, a.x); Tol.Equal(0.0, a.y);
        Tol.Equal(1.0, b.x); Tol.Equal(0.0, b.y);
    }

    // -- JS:  point0 above, point1 below -> lerp, keep upper part, push lower
    //
    //  Segment: (0,0,z=-0.2) -> (1,0,z=-0.8)  cutDepth=-0.5
    //  Crossing fraction: (âˆ’0.5 âˆ’ (âˆ’0.2)) / (âˆ’0.8 âˆ’ (âˆ’0.2)) = âˆ’0.3/âˆ’0.6 = 0.5
    //  Clip x = 0 + 0.5*(1âˆ’0) = 0.5,  y = 0
    [Fact]
    public void StradBase_P0Above_P1Below()
    {
        var seg  = Seg(0, 0, -0.2, 1, 0, -0.8);
        var flat = new List<(PointD, PointD)>();
        var result = MedialAxisToolpaths.TrimSegment(seg, -0.5, flat);

        Assert.NotNull(result);
        // Kept segment: (0,0,-0.2) -> (0.5,0,-0.5)
        Tol.Equal(0.0, result!.Value.Point0.X, label: "kept.P0.X");
        Tol.Equal(0.0, result.Value.Point0.Y,  label: "kept.P0.Y");
        Tol.Equal(-0.2, result.Value.Point0.Radius, label: "kept.P0.Z");
        Tol.Equal(0.5, result.Value.Point1.X,  label: "kept.P1.X");
        Tol.Equal(0.0, result.Value.Point1.Y,  label: "kept.P1.Y");
        Tol.Equal(-0.5, result.Value.Point1.Radius, label: "kept.P1.Z");

        // Flat stroke: (0.5,0) -> (1,0)
        Assert.Single(flat);
        var (a, b) = flat[0];
        Tol.Equal(0.5, a.x, label: "flat.A.x");
        Tol.Equal(0.0, a.y, label: "flat.A.y");
        Tol.Equal(1.0, b.x, label: "flat.B.x");
        Tol.Equal(0.0, b.y, label: "flat.B.y");
    }

    // -- JS:  point0 below, point1 above -> lerp, keep upper part, push lower
    //
    //  Segment: (0,0,z=-0.8) -> (1,0,z=-0.2)  cutDepth=-0.5
    //  JS path: z0<t && z1>t
    //    frac = (t - z1) / (z0 - z1) = (âˆ’0.5âˆ’(âˆ’0.2)) / (âˆ’0.8âˆ’(âˆ’0.2)) = âˆ’0.3/âˆ’0.6 = 0.5
    //    clipped.x = z1.x + frac*(z0.x - z1.x) = 1 + 0.5*(0-1) = 0.5
    //    flatStroke: (p0={0,0}) -> (clipped={0.5,0})
    //    returned segment: (clipped={0.5,0,-0.5}) -> p1=(1,0,-0.2)
    [Fact]
    public void Straddle_P0Below_P1Above()
    {
        var seg  = Seg(0, 0, -0.8, 1, 0, -0.2);
        var flat = new List<(PointD, PointD)>();
        var result = MedialAxisToolpaths.TrimSegment(seg, -0.5, flat);

        Assert.NotNull(result);
        // Kept segment: (0.5,0,-0.5) -> (1,0,-0.2)
        Tol.Equal(0.5,  result!.Value.Point0.X,      label: "kept.P0.X");
        Tol.Equal(0.0,  result.Value.Point0.Y,        label: "kept.P0.Y");
        Tol.Equal(-0.5, result.Value.Point0.Radius,   label: "kept.P0.Z");
        Tol.Equal(1.0,  result.Value.Point1.X,        label: "kept.P1.X");
        Tol.Equal(0.0,  result.Value.Point1.Y,        label: "kept.P1.Y");
        Tol.Equal(-0.2, result.Value.Point1.Radius,   label: "kept.P1.Z");

        // Flat stroke: (0,0) -> (0.5,0)
        Assert.Single(flat);
        var (a, b) = flat[0];
        Tol.Equal(0.0, a.x, label: "flat.A.x");
        Tol.Equal(0.0, a.y, label: "flat.A.y");
        Tol.Equal(0.5, b.x, label: "flat.B.x");
        Tol.Equal(0.0, b.y, label: "flat.B.y");
    }

    // -- Exactly on the cut plane -> JS: z0 >= t && z1 >= t is TRUE -> returned unchanged
    //    (The >= check fires before <= check, so on-plane is treated as "above", not "below")
    [Fact]
    public void ExactlyAtCutDepth_TreatedAsAbove()
    {
        var seg  = Seg(0, 0, -0.5, 1, 0, -0.5);
        var flat = new List<(PointD, PointD)>();
        var result = MedialAxisToolpaths.TrimSegment(seg, -0.5, flat);

        // JS: n.point0.z >= t && n.point1.z >= t -> return n (not null)
        Assert.NotNull(result);
        Assert.Equal(seg, result.Value);
        Assert.Empty(flat);
    }
}

// --- JoinSegments (__) -------------------------------------------------------
public class JoinSegmentsTests
{
    private static (PointD, PointD) Pair(double x0, double y0, double x1, double y1)
        => (new PointD(x0, y0), new PointD(x1, y1));

    // -- Single pair -> single 2-point polyline
    [Fact]
    public void SinglePair_OnePolyline()
    {
        var pairs = new[] { Pair(0, 0, 1, 0) };
        var result = MedialAxisToolpaths.JoinSegments(pairs);

        Assert.Single(result);
        Assert.Equal(2, result[0].Count);
        Tol.Equal(0.0, result[0][0].x); Tol.Equal(0.0, result[0][0].y);
        Tol.Equal(1.0, result[0][1].x); Tol.Equal(0.0, result[0][1].y);
    }

    // -- Two chained pairs -> single 3-point polyline
    //    (0,0)->(1,0)  +  (1,0)->(2,0) share the (1,0) point
    [Fact]
    public void TwoChainedPairs_OnePolyline()
    {
        var pairs = new[] { Pair(0, 0, 1, 0), Pair(1, 0, 2, 0) };
        var result = MedialAxisToolpaths.JoinSegments(pairs);

        Assert.Single(result);
        Assert.Equal(3, result[0].Count);
        Tol.Equal(0.0, result[0][0].x);
        Tol.Equal(1.0, result[0][1].x);
        Tol.Equal(2.0, result[0][2].x);
    }

    // -- Two disconnected pairs -> two polylines
    [Fact]
    public void TwoDisconnectedPairs_TwoPolylines()
    {
        var pairs = new[] { Pair(0, 0, 1, 0), Pair(5, 5, 6, 5) };
        var result = MedialAxisToolpaths.JoinSegments(pairs);

        Assert.Equal(2, result.Count);
        Assert.Equal(2, result[0].Count);
        Assert.Equal(2, result[1].Count);
    }

    // -- Chain of three -> single 4-point polyline
    [Fact]
    public void ThreeChained_OnePolyline()
    {
        var pairs = new[]
        {
            Pair(0, 0, 1, 0),
            Pair(1, 0, 2, 0),
            Pair(2, 0, 3, 0),
        };
        var result = MedialAxisToolpaths.JoinSegments(pairs);

        Assert.Single(result);
        Assert.Equal(4, result[0].Count);
    }

    // -- Points within tolerance (< 1e-5) must join
    [Fact]
    public void WithinTolerance_Joins()
    {
        double delta = 5e-6; // < rn = 1e-5
        var pairs = new[] { Pair(0, 0, 1, 0), Pair(1 + delta, 0, 2, 0) };
        var result = MedialAxisToolpaths.JoinSegments(pairs);

        Assert.Single(result);  // joined
    }

    // -- Points just outside tolerance (>= 1e-5) must NOT join
    [Fact]
    public void OutsideTolerance_DoesNotJoin()
    {
        double delta = 2e-5; // > rn = 1e-5
        var pairs = new[] { Pair(0, 0, 1, 0), Pair(1 + delta, 0, 2, 0) };
        var result = MedialAxisToolpaths.JoinSegments(pairs);

        Assert.Equal(2, result.Count);  // not joined
    }

    // -- Empty input -> empty output
    [Fact]
    public void Empty_ReturnsEmpty()
    {
        var result = MedialAxisToolpaths.JoinSegments(Array.Empty<(PointD, PointD)>());
        Assert.Empty(result);
    }
}

// --- JoinIntoPolylines (iC) --------------------------------------------------
public class JoinIntoPolylinesTests
{
    private static MedialSegment Seg(double x0, double y0, double z0,
                                     double x1, double y1, double z1)
        => new(new MedialPoint(x0, y0, z0), new MedialPoint(x1, y1, z1));

    // -- Empty input -> null (JS: return null)
    [Fact]
    public void Empty_ReturnsNull()
    {
        var result = MedialAxisToolpaths.JoinIntoPolylines(Array.Empty<MedialSegment>());
        Assert.Null(result);
    }

    // -- Single segment -> one 2-point polyline
    [Fact]
    public void SingleSegment_OnePolyline()
    {
        var result = MedialAxisToolpaths.JoinIntoPolylines(new[]
        {
            Seg(0, 0, -0.1, 1, 0, -0.2),
        });

        Assert.NotNull(result);
        Assert.Single(result!);
        Assert.Equal(2, result[0].Count);
    }

    // -- Two connected segments -> one 3-point polyline
    [Fact]
    public void TwoConnected_OnePolyline()
    {
        var result = MedialAxisToolpaths.JoinIntoPolylines(new[]
        {
            Seg(0, 0, -0.1, 1, 0, -0.2),
            Seg(1, 0, -0.2, 2, 0, -0.3),
        });

        Assert.NotNull(result);
        Assert.Single(result!);
        Assert.Equal(3, result![0].Count);
    }

    // -- Two disconnected segments -> two polylines
    [Fact]
    public void TwoDisconnected_TwoPolylines()
    {
        var result = MedialAxisToolpaths.JoinIntoPolylines(new[]
        {
            Seg(0, 0, -0.1, 1, 0, -0.2),
            Seg(5, 5, -0.1, 6, 5, -0.2),
        });

        Assert.NotNull(result);
        Assert.Equal(2, result!.Count);
    }

    // -- Connection requires matching Z too (JS uses 3-component closeTo)
    [Fact]
    public void ZMismatch_DoesNotConnect()
    {
        // endpoint of seg1: z=-0.2; start of seg2: z=-0.9 (far from -0.2)
        var result = MedialAxisToolpaths.JoinIntoPolylines(new[]
        {
            Seg(0, 0, -0.1, 1, 0, -0.2),
            Seg(1, 0, -0.9, 2, 0, -1.0),
        });

        Assert.NotNull(result);
        Assert.Equal(2, result!.Count);
    }

    // -- Within tolerance on all axes -> connects
    [Fact]
    public void WithinTolerance_AllAxes_Connects()
    {
        double d = 5e-6;
        var result = MedialAxisToolpaths.JoinIntoPolylines(new[]
        {
            Seg(0, 0, -0.1, 1, 0, -0.2),
            Seg(1+d, d, -0.2+d, 2, 0, -0.3),
        });

        Assert.NotNull(result);
        Assert.Single(result!); // joined
    }

    // -- Point values preserved
    [Fact]
    public void PointValuesPreserved()
    {
        var result = MedialAxisToolpaths.JoinIntoPolylines(new[]
        {
            Seg(1.5, 2.5, -0.25, 3.5, 4.5, -0.75),
        });

        Assert.NotNull(result);
        var poly = result![0];
        Tol.Equal(1.5,  poly[0].X, label: "P0.X");
        Tol.Equal(2.5,  poly[0].Y, label: "P0.Y");
        Tol.Equal(-0.25, poly[0].Z, label: "P0.Z");
        Tol.Equal(3.5,  poly[1].X, label: "P1.X");
        Tol.Equal(4.5,  poly[1].Y, label: "P1.Y");
        Tol.Equal(-0.75, poly[1].Z, label: "P1.Z");
    }
}


// --- GenerateSimplifiedLayer (gh = GM) ---------------------------------------
public class GenerateSimplifiedLayerTests
{
    // Build a simple axis-aligned rectangle (workspace coords, inches)
    // 1" Ã— 0.5" centred at (0,0)
    private static List<List<PointD>> MakeRect(double w = 1.0, double h = 0.5)
    {
        double hw = w / 2, hh = h / 2;
        return new List<List<PointD>>
        {
            new List<PointD>
            {
                new(-hw, -hh), new( hw, -hh),
                new( hw,  hh), new(-hw,  hh), new(-hw, -hh),
            }
        };
    }

    // -- Non-null result for a valid rectangle at initial scale p_=4096
    [Fact]
    public void ValidRect_ReturnsNonNull()
    {
        var result = MedialAxisToolpaths.GenerateSimplifiedLayer(MakeRect(), 4096);
        Assert.NotNull(result);
        Assert.NotEmpty(result!);
    }

    // -- Morphological pass must NOT destroy a 1"Ã—0.5" rectangle.
    //    The contract+expand+contract uses delta = (16/32768)*scale = 2 Clipper units
    //    at scale=4096.  That is ~0.000488" â€” much smaller than the rectangle.
    [Fact]
    public void MorphologicalPass_PreservesLargeShape()
    {
        var result = MedialAxisToolpaths.GenerateSimplifiedLayer(MakeRect(), 4096);

        Assert.NotNull(result);
        // Should still have at least one ring with at least 3 vertices
        Assert.True(result!.Any(ring => ring.Count >= 3),
            "Morphological smoothing must not destroy large polygons");
    }

    // -- Area should be approximately preserved (within a small fraction)
    [Fact]
    public void MorphologicalPass_ApproximatesOriginalArea()
    {
        var input  = MakeRect();
        var result = MedialAxisToolpaths.GenerateSimplifiedLayer(input, 4096);

        Assert.NotNull(result);

        // Signed area of input rectangle (in workspace coords, should be 0.5 inÂ²)
        double inputArea = Math.Abs(Clipper2Lib.Clipper.Area(
            PathUtils.ToClipper(input.Select(r => r.Select(p => p)))[0])) /
            (PathUtils.Scale * PathUtils.Scale);

        // Signed area of output â€” sum all rings
        double outputArea = result!.Sum(ring =>
        {
            var path = PathUtils.ToClipper(new[] { ring.Select(p => p) })[0];
            return Math.Abs(Clipper2Lib.Clipper.Area(path));
        }) / (PathUtils.Scale * PathUtils.Scale);

        // Allow 1% area difference from morphological pass
        Assert.True(Math.Abs(inputArea - outputArea) / inputArea < 0.01,
            $"Area changed too much: input={inputArea:F6}, output={outputArea:F6}");
    }

    // -- Scale doubles (4096 -> 8192) -> morphDelta doubles too, shape still survives
    [Fact]
    public void DoubledScale_ShapeStillSurvives()
    {
        var result = MedialAxisToolpaths.GenerateSimplifiedLayer(MakeRect(), 8192);
        Assert.NotNull(result);
        Assert.True(result!.Any(ring => ring.Count >= 3));
    }

    // -- Tiny degenerate input (smaller than OFFSET_DISTANCE) -> null or empty
    //    A 1e-4" Ã— 1e-4" square at scale=4096: after contracting by 0.000488"
    //    the shape disappears entirely -> null/empty is correct JS behaviour (.clean())
    [Fact]
    public void TinyShape_ReturnsNullOrEmpty()
    {
        var tiny = new List<List<PointD>>
        {
            new List<PointD>
            {
                new(-5e-5, -5e-5), new( 5e-5, -5e-5),
                new( 5e-5,  5e-5), new(-5e-5,  5e-5), new(-5e-5, -5e-5),
            }
        };

        var result = MedialAxisToolpaths.GenerateSimplifiedLayer(tiny, 4096);
        // Either null or all rings cleaned away
        Assert.True(result is null || result.All(r => r.Count < 3),
            "Tiny shape should be erased by morphological smoothing");
    }
}

// --- GenerateFlatAreaFill (rC) ------------------------------------------------
// --- OffsetFill tree-insertion (InsertIntoTree) -------------------------------
public class OffsetFillTreeTests
{
    // When a smaller ring is inserted and a root ring already exists that
    // contains it, the root must NOT be orphaned from the roots list.
    // Bug: the original code called roots.RemoveAt(i) before InsertUnder,
    // which silently discarded the outer ring and left roots empty after one
    // iteration â€” causing all subsequent offset rings to be added as isolated
    // roots rather than children of the outer boundary.
    [Fact]
    public void BuildOffsetTree_OuterRootSurvivesAfterInnerInsert()
    {
        // A 1" Ã— 1" square centred at (0,0) â€” simple convex polygon.
        double h = 0.5;
        var outer = new List<PointD>
        {
            new(-h, -h), new( h, -h), new( h,  h), new(-h,  h), new(-h, -h)
        };

        var tree = OffsetFill.BuildOffsetTree(
            new[] { outer },
            delta: -0.05,       // inward offset = 0.05 workspace units
            maxIterations: 5);  // 5 rings

        // The root (outermost ring) must still be in the returned list.
        // Before the fix, roots would typically be empty or contain only the
        // very last offset ring, losing the outer boundary entirely.
        Assert.Single(tree);

        // The tree must have a chain of children reaching depth 5 (or the
        // polygon collapses earlier â€” either way depth > 0 is expected for a
        // square with 0.05" step and 0.5" half-width).
        var node = tree[0];
        int depth = 0;
        while (node.Children.Count > 0)
        {
            node = node.Children[0];
            depth++;
        }
        Assert.True(depth > 0, "Expected at least one inward offset ring as child of the root");
    }

    [Fact]
    public void BuildPathTree_NestedRingDoesNotOrphanParent()
    {
        // Outer ring: 2" Ã— 2" square
        var outer = new List<PointD>
        {
            new(-1,-1), new(1,-1), new(1,1), new(-1,1), new(-1,-1)
        };
        // Inner ring: 0.5" Ã— 0.5" square (the "hole")
        var inner = new List<PointD>
        {
            new(-0.25,-0.25), new(0.25,-0.25), new(0.25,0.25), new(-0.25,0.25), new(-0.25,-0.25)
        };

        // Build tree with outer first, then inner (the bug triggers when outer
        // is already a root and the inner ring causes roots.RemoveAt).
        var roots = OffsetFill.BuildPathTree(new IEnumerable<PointD>[] { outer, inner });

        // Outer ring must remain as the single root.
        Assert.Single(roots);

        // Inner ring must be a child of the outer root.
        Assert.Single(roots[0].Children);
    }

        [Fact]
        public void InsertIntoTree_AbsorbsInnerRootAndNestsInsideOuterRoot()
        {
            // Regression for JS Tg loop-continue parity.
            // Scenario: three concentric rings: R1 > R2 > R3 (outer > middle > inner).
            // After inserting R1 and R3 into roots (so roots = [R1, R3]),
            // inserting R2 should:
            //   - Condition A fires for R3 (R3 is inside R2) -> R3 becomes R2.children
            //   - Loop continues (NOT early-exit)
            //   - Condition B fires for R1 (R2 is inside R1) -> R2 inserted under R1
            // Final: roots=[R1], R1.children=[R2], R2.children=[R3]
            //
            // The old nested-loop implementation exited after condition A and added R2
            // as a separate root, breaking the nesting.

            // R1: 2" square
            var r1 = new List<PointD>
            {
                new(-1,-1), new(1,-1), new(1,1), new(-1,1), new(-1,-1)
            };
            // R2: 1" square (inside R1)
            var r2 = new List<PointD>
            {
                new(-0.5,-0.5), new(0.5,-0.5), new(0.5,0.5), new(-0.5,0.5), new(-0.5,-0.5)
            };
            // R3: 0.4" square (inside R2)
            var r3 = new List<PointD>
            {
                new(-0.2,-0.2), new(0.2,-0.2), new(0.2,0.2), new(-0.2,0.2), new(-0.2,-0.2)
            };

            // Insert R1 first, then R3, so roots = [R1, R3] before inserting R2.
            // R2 must absorb R3 (condition A) AND be placed inside R1 (condition B).
            var roots = OffsetFill.BuildPathTree(new IEnumerable<PointD>[] { r1, r3, r2 });

            Assert.Single(roots);                              // only R1 at root level
            Assert.Single(roots[0].Children);                  // R2 is R1's only child
            Assert.Single(roots[0].Children[0].Children);      // R3 is R2's only child
        }

        [Fact]
        public void GenerateSimple_ProducesToolpathsForAllLetterRings()
    {
        // Build a simplified "E" shape (a rectangle â€” pocket fill should work).
        // The real issue was that letters E, A, L produced zero toolpaths.
        var rect = new List<PointD>
        {
            new(0,0), new(80,0), new(80,180), new(0,180), new(0,0)
        };

        var result = OffsetFill.Generate(
            new[] { rect.AsEnumerable() },
            depth: -0.1,
            zTop: 0.0,
            stepOver: 5.0,   // large step in same pixel units
            maxIterations: 20);

        // Should produce at least the outer boundary ring + at least one inward ring.
        Assert.True(result.Count >= 2,
            $"Expected at least 2 toolpath polylines, got {result.Count}");
    }
}

public class GenerateFlatAreaFillTests
{
    // -- No flat area -> all strokes pass through unchanged
    [Fact]
    public void NoFlatArea_AllStrokesSurvive()
    {
        // A single horizontal polyline at y=0, x from 0 to 3 (3 segments)
        var strokes = new List<List<PointD>>
        {
            new List<PointD> { new(0,0), new(1,0), new(2,0), new(3,0) }
        };

        var segments = MedialAxisToolpaths
            .GenerateFlatAreaFill(strokes, Array.Empty<List<PointD>>(), -0.5)
            .ToList();

        Assert.Equal(3, segments.Count);
        Assert.All(segments, s =>
        {
            Tol.Equal(-0.5, s.Point0.Radius, label: "endZ p0");
            Tol.Equal(-0.5, s.Point1.Radius, label: "endZ p1");
        });
    }

    // -- Stroke entirely inside flat area -> clipped away (OUTSIDE = nothing)
    //    Flat area = 2"Ã—2" square (-1..1 Ã— -1..1)
    //    Stroke = (âˆ’0.5,0) -> (0.5,0)  entirely inside
    [Fact]
    public void StrokeInsideFlatArea_IsClippedAway()
    {
        var flatArea = new List<List<PointD>>
        {
            new List<PointD>
            {
                new(-1,-1), new(1,-1), new(1,1), new(-1,1), new(-1,-1)
            }
        };
        var strokes = new List<List<PointD>>
        {
            new List<PointD> { new(-0.5, 0), new(0.5, 0) }
        };

        var segments = MedialAxisToolpaths
            .GenerateFlatAreaFill(strokes, flatArea, -0.5)
            .ToList();

        Assert.Empty(segments);
    }

    // -- Stroke entirely outside flat area -> passes through unchanged
    //    Flat area = (-0.5..0.5)Ã—(-0.5..0.5) box
    //    Stroke = (2,0) -> (3,0) entirely outside
    [Fact]
    public void StrokeOutsideFlatArea_Survives()
    {
        var flatArea = new List<List<PointD>>
        {
            new List<PointD>
            {
                new(-0.5,-0.5), new(0.5,-0.5), new(0.5,0.5), new(-0.5,0.5), new(-0.5,-0.5)
            }
        };
        var strokes = new List<List<PointD>>
        {
            new List<PointD> { new(2, 0), new(3, 0) }
        };

        var segments = MedialAxisToolpaths
            .GenerateFlatAreaFill(strokes, flatArea, -0.5)
            .ToList();

        Assert.Single(segments);
        Tol.Equal(2.0, segments[0].Point0.X, Tol.Loose);
        Tol.Equal(3.0, segments[0].Point1.X, Tol.Loose);
    }

    // -- Stroke straddles flat area boundary -> only outer portion survives
    //    Flat area = (-0.5..0.5)Ã—(-0.5..0.5)
    //    Stroke = (âˆ’1,0) -> (1,0) enters flat area at x=âˆ’0.5, exits at x=0.5
    //    Surviving portions: (âˆ’1,0)->(âˆ’0.5,0) and (0.5,0)->(1,0)
    [Fact]
    public void StrokeStraddles_OnlyOuterPortionSurvives()
    {
        var flatArea = new List<List<PointD>>
        {
            new List<PointD>
            {
                new(-0.5,-0.5), new(0.5,-0.5), new(0.5,0.5), new(-0.5,0.5), new(-0.5,-0.5)
            }
        };
        var strokes = new List<List<PointD>>
        {
            new List<PointD> { new(-1, 0), new(1, 0) }
        };

        var segments = MedialAxisToolpaths
            .GenerateFlatAreaFill(strokes, flatArea, -0.5)
            .ToList();

        // Should get two outer segments (left and right of the box)
        Assert.Equal(2, segments.Count);

        double totalLength = segments.Sum(s =>
            Math.Sqrt(Math.Pow(s.Point1.X - s.Point0.X, 2) +
                      Math.Pow(s.Point1.Y - s.Point0.Y, 2)));

        Tol.Equal(1.0, totalLength, 1e-4, "total outer length");
    }
}

// --- PathUtils parity helpers ------------------------------------------------
public class PathUtilsParityTests
{
    private static List<Point3D> P((double x, double y) a, (double x, double y) b)
        => new()
        {
            new Point3D(a.x, a.y, -0.1),
            new Point3D(b.x, b.y, -0.1),
        };

    private static Path64 CcwSquare()
        => new()
        {
            new Point64(0, 0),
            new Point64(10, 0),
            new Point64(10, 10),
            new Point64(0, 10),
            new Point64(0, 0),
        };

    private static Path64 CwSquare()
        => new()
        {
            new Point64(0, 0),
            new Point64(0, 10),
            new Point64(10, 10),
            new Point64(10, 0),
            new Point64(0, 0),
        };

    [Fact]
    public void NearestNeighborSort_PreservesFirstPath_AsInJs()
    {
        var p0 = P((100, 100), (101, 100));
        var p1 = P((102, 100), (103, 100));
        var p2 = P((0, 0), (1, 0));

        var paths = new List<List<Point3D>> { p0, p1, p2 };

        PathUtils.NearestNeighborSort(paths);

        // JS keeps index 0 fixed and only optimizes positions t+1..end.
        Assert.Same(p0, paths[0]);
    }

    [Fact]
    public void NearestNeighborSort_UsesCurrentEndToChooseNext_AsInJs()
    {
        var p0 = P((0, 0), (10, 0));
        var p1 = P((100, 0), (101, 0));
        var p2 = P((11, 0), (12, 0));
        var p3 = P((13, 0), (14, 0));

        var paths = new List<List<Point3D>> { p0, p1, p2, p3 };

        PathUtils.NearestNeighborSort(paths);

        // From end(p0)=(10,0), p2 start=(11,0) is nearest so it must be moved
        // into slot 1 (swap with current slot-1 path p1).
        Assert.Same(p2, paths[1]);

        // Next step uses end(p2)=(12,0), so p3 should come before p1.
        Assert.Same(p3, paths[2]);
        Assert.Same(p1, paths[3]);
    }

    [Fact]
    public void OrientPath_Climb_ForcesCounterClockwise_AsInJsOlWg()
    {
        var oriented = PathUtils.OrientPath(CwSquare(), "climb");
        Assert.True(Clipper.Area(oriented) > 0, "climb should force CCW winding");
    }

    [Fact]
    public void OrientPath_Conventional_ForcesClockwise_AsInJsOlWg()
    {
        var oriented = PathUtils.OrientPath(CcwSquare(), "conventional");
        Assert.True(Clipper.Area(oriented) < 0, "conventional should force CW winding");
    }

    [Fact]
    public void OrientPath_DefaultOrNull_LeavesWindingUnchanged_AsInJsOl()
    {
        var cw = CwSquare();
        var nullDir = PathUtils.OrientPath(cw, null);
        var defaultDir = PathUtils.OrientPath(cw, "default");

        Assert.True(Clipper.Area(nullDir) < 0, "null should preserve winding");
        Assert.True(Clipper.Area(defaultDir) < 0, "default should preserve winding");
    }
}


