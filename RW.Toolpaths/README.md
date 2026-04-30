# RW Toolpaths Core

RW.Toolpaths is a .NET 8 geometry and toolpath library for pocket clearing and V-carve planning.

## Modules

- `OffsetFill`: concentric offset-ring pocket filling.
- `RampUtils`: helical and zig-zag lead-in generation.
- `PathUtils`: coordinate conversion, winding, simplification, and traversal helpers.
- `MedialAxisToolpaths`: V-carve planning pipeline with trimming and flat-zone fill.
- `BoostVoronoiProvider`: native Voronoi-backed medial-axis provider.

## V-Carve Usage

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
