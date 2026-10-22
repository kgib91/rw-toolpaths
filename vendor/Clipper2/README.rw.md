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
Qualification is bound in `rw-legacy-win32/artifacts/roughing-30s/ordered-scanbeam-probe-14`:
five alternating captured-workload pairs preserve all complete ordered intermediate
results and decisions; the supported ClipperScaling checks preserve assembly identity,
public API, signed/open/closed geometry, repeated execution and offsets. These are
kernel measurements; complete phase latency and stock auditing are separate checks.

`dotnet build -p:ClipperDiagnostics=true` adds internal counters and opt-in captures.
Ordinary builds compile them out. Diagnostic summaries expose inserted events,
baseline shifted elements, consumed scanlines, peak queue/active edges and intersections.
`RW_CLIPPER_TRACE_SCANLINES=1` retains exact per-execution event sequences for reflection
checks. Set `RW_CLIPPER_CAPTURE_DIR`, `RW_CLIPPER_CAPTURE_LABEL`,
`RW_CLIPPER_CAPTURE_LIMIT` (default 8) and `RW_CLIPPER_CAPTURE_MIN_VERTICES`
(default 1000) to capture ordered integer inputs. Reusable-data inputs decline capture
instead of producing incomplete fixtures. Timing runs must use ordinary builds.

`benchmarks/ClipperScaling` compares assembly identities, public APIs, exact ordered
closed/open outputs, repeated execution and offsets in isolated assembly load contexts.
It runs five alternating timing repetitions, disjoint/equal-Y/intersection scaling and
an isolated adversarial queue benchmark. `--diagnostic-only` checks complete executed
scanline sequences without timing. `--fixtures=directory` includes diagnostic captures.
Run it with an output JSON path followed by baseline/candidate DLL paths.

Evidence for this change is in `rw-legacy-win32/artifacts/toolpath-scaling`, including
the unmodified-source check, frozen baseline runtime, physical-sweep and reference-opening
captures, exact event comparisons and counters-disabled measurements. Future upstream
updates must explicitly revalidate identity, API and ordered geometry.

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
Qualification is frozen in
`rw-legacy-win32/artifacts/roughing-30s/sparse-top-prefix-25-qualified-source-images.zip`:
2,816 adversarial cases and 2,470 complete captured workload comparisons pass;
ClipperScaling passes 6,528 ordered closed/open/repeated executions and 36 offsets
with unchanged identity/API. Six real dense Boolean operands omit 5,371,953
redundant assignments and retain complete ordered native outputs. Five alternating
kernel pairs have material timing variance (median 19.834 -> 15.723 seconds), so
full cold-phase latency and original-stock auditing remain separate acceptance.
