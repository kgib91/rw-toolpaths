# Pinned Clipper2 build

Upstream: Angus Johnson's `Clipper2_1.4.0` tag, commit
`736ddb0b53d97fd5f65dd3d9bbf8a0993eaf387c`.
Source: https://github.com/AngusJohnson/Clipper2/tree/Clipper2_1.4.0/CSharp/Clipper2Lib
The original copyright headers, Boost license (`LICENSE`) and signing key are retained.

`RW.Toolpaths` and `RwLegacy.Shared` reference this single project. It retains
`Clipper2Lib, Version=1.4.0.0, Culture=neutral, PublicKeyToken=ddeb3f68c442d6e4`.
The project disables package generation, pins version metadata and enables compiler
optimization in both Debug and Release. Unmodified upstream source was checked
against the 1.4.0 NuGet DLL before the queue modification.

Ordinary builds replace scanline sorted-list insertion with an array-backed max heap,
linear initial heap construction and duplicate removal at pop. They also skip an
intersection-list merge when the complete active-edge list is already nondecreasing
at the original computed top-X values, and skip two sorted merge runs whose boundary
is already nondecreasing. The original stable merge creates only strict inversions;
these cases create no intersection or node movement. Original top-X arithmetic,
join-left adjustment, equal-X order and all unordered merge code are retained.
Clipping predicates, intersections, integer rounding, offsets, fill rules and solution
ordering retain upstream code. Queue administration is O(N log N).

`RW_CLIPPER_ORDERED_SCANBEAM_REFERENCE=1` captures the original intersection-list
merge mode when each Clipper instance is constructed. Opt-in
`RW_CLIPPER_ORDERED_SCANBEAM_TRACE=1` reports whole-list and sorted-run skip counters;
leave it unset for timing. This does not change the public API or signing identity.
Complete phase latency and stock auditing require separate validation from kernel
measurements. Local investigation captures are disposable and are not prerequisites
for building or running the supported tests.

`dotnet build -p:ClipperDiagnostics=true` adds internal counters and opt-in captures.
Ordinary builds compile them out. Diagnostic summaries expose inserted events,
baseline shifted elements, consumed scanlines, peak queue/active edges and intersections.
`RW_CLIPPER_TRACE_SCANLINES=1` retains exact per-execution event sequences for reflection
checks. Set `RW_CLIPPER_CAPTURE_DIR`, `RW_CLIPPER_CAPTURE_LABEL`,
`RW_CLIPPER_CAPTURE_LIMIT` (default 8) and `RW_CLIPPER_CAPTURE_MIN_VERTICES`
(default 1000) to capture ordered integer inputs. Reusable-data inputs decline capture
instead of producing incomplete fixtures. Timing runs must use ordinary builds.

The maintained regression checks are in
[`ClipperScanlineQueueTests`](../../RW.Toolpaths.Tests/ClipperScanlineQueueTests.cs),
[`ClipperExecutionFailureTests`](../../RW.Toolpaths.Tests/ClipperExecutionFailureTests.cs)
and [`PolygonOpsTests`](../../RW.Toolpaths.Tests/PolygonOpsTests.cs). They cover distinct
descending scanline order and reuse, failed-result disposal, ordered Boolean wrappers
and polygon operations. Run the suite from the repository root:

```powershell
dotnet test .\RW.Toolpaths.Tests\RW.Toolpaths.Tests.csproj
```

Future upstream updates must explicitly revalidate assembly identity, public API and
ordered geometry. Performance qualification must measure the full affected workload
with diagnostics disabled.

For a completely ordered scanbeam, the original Adjust pass has also computed
the original own top-X of every non-top, non-join-left edge. The later top pass
can begin at the first top vertex or join-left barrier: every skipped prefix
assignment would reproduce that same coordinate, and there was no intervening
intersection or edge mutation. Single-edge, unordered and intersecting passes
retain the complete original top pass. The certificate resets on every beam;
all top vertices, maxima, horizontals and downstream edge mutations keep their
original code and ordering.

`RW_CLIPPER_SPARSE_TOP_PREFIX_REFERENCE=1` captures the complete prior top pass.
The existing opt-in ordered trace also reports skipped prefix assignments.
Full cold-phase latency and original-stock auditing remain separate acceptance
criteria for changes to this optimization.
