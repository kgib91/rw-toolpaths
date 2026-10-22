# RW Toolpaths Core

RW.Toolpaths is a .NET 8 geometry and toolpath library for pocket clearing and V-carve planning.

## Modules

- `OffsetFill`: concentric offset-ring pocket filling.
- `RampUtils`: helical and zig-zag lead-in generation.
- `PathUtils`: coordinate conversion, winding, simplification, and traversal helpers.
- `MedialAxisToolpaths`: V-carve planning pipeline with trimming and flat-zone fill.
- `BoostVoronoiProvider`: native Voronoi-backed medial-axis provider.
- `Milling.PocketToolpaths`: compensated offset pockets with full-stepover strip completion.
- `Milling.PocketContinuationRouter`: proved ring/strip connections within an existing pocket visit schedule.

At 100% stepover, offset pockets supplement rings with safe center traces wherever
the next inset collapses. Every positive-area closed inset remains a ring, including
thin slivers whose small enclosed area understates their cutter sweep. Simplifying
a valid contour to a degenerate loop retains its original vertices. The first inset
uses at most 0.001 mm arc tolerance without RDP; inner rings retain the requested
tolerances and existing nesting/ramp strategy. This protects the wall band when
neighbouring cutter sweeps have no overlap.
The completion target is the outermost legal cutter-center fill
eroded by the physical cutter radius, minus the ring sweeps. Only those uncut
components receive medial traces; reconstructing the entire source medial graph
would create irrelevant branches and excessive entries on dense artwork. Exact
cutter-sweep coverage validates a 0.00025 mm reduction of residual medial sites;
a measurable gap retries only that region once with its original residual vertices.
Clearance alone does not certify that a reduced medial graph retains every branch.
Exact
cutter-width slots whose first inset has zero area use a source-medial fallback.
Longer strips extend into cleared material when the complete extension fits;
redundant corner branches are removed by swept coverage at 0.001 mm geometry
accuracy. Nearby fragments join through independently proved bridges no longer
than one cutter diameter, preserving every retained trace chord.
Inside Out cuts the strips before rings at each depth; Outside In cuts them after.
With spiral chaining enabled, strips and rings connect at the cutting depth when
the complete cutter footprint fits the source and allowance. Closed rings choose
a safe entry point; open traces choose a safe endpoint without closing the trace.
Connections at an already cleared floor retain the next pass's entry ramp. Explicit
lift policies and discrete-loop mode keep their authored behavior.
Holes, radial stock allowance, clipping and starting-depth floors retain their
usual contracts. An authored ring cap remains a partial-clearing budget.

This full-stepover completion uses the existing native Boost Voronoi runtime,
already copied by the project for supported built RIDs. Consumers must deploy that
runtime alongside the assembly. Lower-stepover ring pocketing retains its existing
managed path. `[Pocket.Strips]` records residual components, rejected chords,
redundant traces, joined fragments, bridge rejections and remaining residual area.
`result=retry-original-residual` records a lost medial branch and both boundary
point counts; the final region log records the retry's remaining coverage.
`[Pocket.Strips.Emit]` also records recovered connections, remaining entry moves
and rejected continuation candidates.
Clearance is proved against the original source and protected holes for complete
chords, including added connections. Boost inputs discard consecutive duplicate
rounded vertices so a zero-length boundary segment cannot become a point site.
Consumers must preserve certified cutting chords or independently prove replacement
chords and their coverage. The Shared full-stepover pocket consumer retains emitted
geometry, matching its existing V-carve policy. Coverage audits derive reachable
material from the original source, rather than trusting only the first inset's core.

## V-Carve Usage

Dense smooth V-carving can use the owned CUDA coverage-witness kernel. The
Windows x64 backend loads the NVIDIA driver and an installed NVRTC compiler,
uploads the actual commanded cutter intervals and their BVH once, and searches
up to 4096 boxes per batch. Every proposed cutting pose is checked again using
the original CPU physical cutter profile. GPU misses merely request more
subdivision; target exterior checks, final coverage tolerance, wall/floor/grade
guards, cancellation and bounded work retain their existing authority.

`RW_TOOLPATHS_VCARVE_GPU=1` enables this measured experimental backend;
the default and `RW_TOOLPATHS_VCARVE_GPU=0` select the CPU reference. Missing CUDA devices,
NVRTC, unsupported inputs or runtime failures report a reason and fall back to
the reference search. `[VCarve.Stage]` certificate diagnostics include device
selection, uploaded bytes, preparation time, kernel/transfer time, nomination
queries, CPU-validated witnesses, subdivisions and fallback counts. The
60-second acceptance target covers both cold V-carving operations in the saved
`test-designs` project; kernel timing alone does not establish that target.

GPU opt-in also selects guarded consolidation of sampled first-hemisphere
**lower** sweep pieces within one affine commanded chord. This avoids redundant
internal unions while retaining inscribed physical witnesses, original numerical
reserves and the unchanged upper coverage target. Unsupported coordinates,
engagement, profile joins and unreserved tips retain the original ordered pieces;
diagnostics report each refusal separately.

`RW_TOOLPATHS_VCARVE_GPU_FP32=1` optionally uses single-precision GPU nominations
in ordinary domains. Original inputs retain their double ABI, and local XY values
are translated around the query before conversion. Every positive still passes
the unchanged CPU physical predicate; approximate misses carry no negative
authority. Unsupported inputs or FP32 compiler failures use the FP64 nomination
module. Device logs identify the arithmetic mode, refusals and fallback counts.
`RW_TOOLPATHS_VCARVE_GPU_TRIANGLES=1` selects the optional exact signed triangle
coverage domain with GPU enabled. Exact original-grid admission and oriented
boundary-chain equality retain every signed correction triangle. Three literal
CPU vertex witnesses on the same eligible convex physical sweep, exact finite
original-edge gap capsules, or the original disc search prove coverage. Exact
dyadic partitions retain unresolved stock; preparation refusals select boxes.
Logs record preparation, triangle/optional/boundary witnesses, subdivisions and
remaining work. This surface proof grants no new full-depth stock authority.
`RW_TOOLPATHS_VCARVE_GPU_BOUNDARY=1` additionally nominates original finite
boundary edges through the owned CUDA sampler in triangle mode. Complete
immutable integer BVH validation precedes upload. Only the original exact
dyadic capsule gives coverage credit; device misses keep disc/subdivision work
and bounded refusals retain CPU nominations. Winding and device distances have
no cutting authority. Logs report queries, accepted capsules and refusals.
All GPU modes remain experimental; the complete60s target is still outstanding.

```csharp
using RW.Toolpaths;
using Clipper2Lib;

var provider = BoostVoronoiProvider.CreateDefault();

var region = new List<IReadOnlyList<PointD>>
{
    new List<PointD>
    {
        new(0, 0),
        new(1, 0),
        new(1, 1),
        new(0, 1)
    }
};

var paths = MedialAxisToolpaths.Generate(
    provider,
    boundary: region,
    startDepth: 0.0,
    endDepth: 0.25,
    radianTipAngle: Math.PI / 3,
    depthPerPass: 0.05);

// Tagged variant for downstream routing/ordering logic.
var tagged = MedialAxisToolpaths.GenerateVCarveTagged(
    provider,
    boundary: region,
    startDepth: 0.0,
    endDepth: 0.25,
    radianTipAngle: Math.PI / 3,
    depthPerPass: 0.05,
    regionIndex: 0);

// Metadata:
//   tagged[i].RegionIndex    -> island/region id
//   tagged[i].Category       -> "clearing" or "final-carve"
//   tagged[i].DepthPassIndex -> clearing depth-layer index (null for final carve)
```

`Milling.DepthSchedule.StartDepth` describes the already cleared material floor as
a positive depth below the unchanged physical stock surface. Passes advance from
this floor by `DepthPerPass` until they reach the absolute final `Depth`. For example,
starting depth 1.25, final depth 3.75 and 1 mm per pass produce depths 2.25, 3.25 and
3.75 mm. A starting depth equal to the final depth retains one floor finishing pass.
`PassTopZ(0)` is `SurfaceZ - StartDepth`; `SurfaceZ` remains the original stock top.
Starting depths must be finite and between zero and the final depth.

Use `Milling.VCarveToolpaths.Generate` with `VCarveOptions.Ramp` for V-carve
operations starting on a cleared floor. It schedules the remaining clearing layers
and enters every disconnected retained visit along its original cutting chords
from that floor. Safe-height XY travel still uses the caller's original stock top;
the approach descends at the visit's XY to its known floor before ramping deeper.
The floor does not rebase the medial-axis geometry, cutter penetration, or boundary
clearance. Detail beginning above the floor starts at its original target height.
`RampPlanner.PlanPathEntry` limits entry descent to the requested angle and lands
at the first reachable local target contact. Variable-depth detail can therefore
contact a shallow feature first; it retains the complete original cutting trace
and never forces that feature down to the operation's nominal tier. Short loops
receive the required revolutions instead of a steeper entry. A path without any
horizontal entry length fails explicitly. Physical cutter depth truncation retains
the final available target and clamps the effective entry floor above it.

`DepthSchedule.SkipDepthPasses` remains available for legacy callers. It omits
leading nominal layers without lowering the known entry floor, preserves the
remaining layer depths and always retains the final available pass. `PassCount`
counts emitted passes and `NominalPassCount` counts the unfiltered floor-relative
schedule. Negative skip counts are rejected. A zero starting floor with zero skips
retains existing V-carve geometry and routing.

## Offset Fill Usage

```csharp
using RW.Toolpaths;
using Clipper2Lib;

var boundary = new List<List<PointD>>
{
    new() { new(0, 0), new(2, 0), new(2, 1), new(0, 1) }
};

var fill = OffsetFill.Generate(
    boundary,
    depth: -0.125,
    zTop: 0.0,
    stepOver: 0.05,
    rampingAngle: null,
    millingDirection: "climb");
```

## Coordinate Conventions

- `X`, `Y`: workspace plane (use a consistent unit system).
- `Z`: negative values represent depth into material.

## Project Map

- `RW.Toolpaths.csproj`: core library.
- `PathUtils.cs`: geometric utility primitives.
- `PathTreeNode.cs`: nesting tree node and `Point3D`.
- `OffsetFill.cs`: inward offset pocket planner.
- `RampUtils.cs`: ramp entry planners.
- `MedialAxisToolpaths.cs`: V-carve composition and trimming.
- `BoostVoronoiProvider.cs`: native Voronoi interop pipeline.

## Frontend

The companion desktop frontend is in `RW.Toolpaths.Avalonia`.

Run from repo root:

```powershell
dotnet run --project .\RW.Toolpaths.Avalonia\RW.Toolpaths.Avalonia.csproj
```
