using Clipper2Lib;
using RW.Toolpaths.Milling.Certification;

namespace RW.Toolpaths.Milling;

internal static partial class VCarveCoverageCertificate
{
    /// <summary>
    /// Holds one successful exact decomposition for one generation owner.
    /// Calls are serialized. The caller must not mutate its input during a call;
    /// later input mutations cannot alter the saved boundary or triangles.
    /// Reuse supplies only the exact domain: physical proof, quotas and
    /// cancellation remain the responsibility of every certificate invocation.
    /// </summary>
    internal sealed class PreparedTriangleDomain
    {
        private readonly object _gate = new();
        private Paths64? _originalGrid;
        private bool _useDelaunay;
        private IReadOnlyList<SignedTriangle>? _triangles;
        private ExactSignedBoundaryDecomposition.Diagnostics? _diagnostics;

        internal bool TryCreateOrReuse(Paths64 grid, bool useDelaunay,
            CancellationToken token, Action? checkpoint,
            out IReadOnlyList<SignedTriangle> triangles,
            out ExactSignedBoundaryDecomposition.Diagnostics diagnostics, out bool reused)
        {
            ArgumentNullException.ThrowIfNull(grid);
            Check(token, checkpoint);
            lock (_gate)
            {
                Check(token, checkpoint);
                reused = false;
                if (_originalGrid is not null && _useDelaunay == useDelaunay
                    && SameGrid(_originalGrid, grid, token, checkpoint))
                {
                    Check(token, checkpoint);
                    triangles = _triangles!;
                    diagnostics = CopyDiagnostics(_diagnostics!);
                    reused = true;
                    return true;
                }

                // Keep the key separate from the helper's working boundary.
                // A successful helper may never replace the original key with
                // a mutated or conditioned proposal.
                Paths64 original = CopyGrid(grid, token, checkpoint);
                Paths64 working = CopyGrid(original, token, checkpoint);
                bool created = ExactSignedBoundaryDecomposition.TryCreate(working, token,
                    checkpoint, out SignedTriangle[] proposed, out var proof, useDelaunay: useDelaunay);
                diagnostics = CopyDiagnostics(proof);
                triangles = Array.Empty<SignedTriangle>();
                if (!created) return false;
                if (!proof.Created || proof.FinalResidualEdges != 0
                    || !SameGrid(original, working, token, checkpoint))
                {
                    diagnostics.Created = false;
                    diagnostics.Reason = "prepared-domain-incomplete-or-input-mutated";
                    return false;
                }

                var immutableTriangles = new SignedTriangle[proposed.Length];
                for (int index = 0; index < immutableTriangles.Length; index++)
                {
                    if ((index & 1023) == 0) Check(token, checkpoint);
                    immutableTriangles[index] = proposed[index];
                }
                IReadOnlyList<SignedTriangle> readOnly = Array.AsReadOnly(immutableTriangles);
                var savedProof = CopyDiagnostics(proof);
                Check(token, checkpoint);
                _originalGrid = original;
                _useDelaunay = useDelaunay;
                _triangles = readOnly;
                _diagnostics = savedProof;
                triangles = readOnly;
                return true;
            }
        }

        private static Paths64 CopyGrid(Paths64 source, CancellationToken token, Action? checkpoint)
        {
            var copy = new Paths64(source.Count);
            foreach (Path64 ring in source)
            {
                Check(token, checkpoint);
                if (ring is null) throw new ArgumentException("Target grid contains a null ring.", nameof(source));
                var copiedRing = new Path64(ring.Count);
                for (int index = 0; index < ring.Count; index++)
                {
                    if ((index & 1023) == 0) Check(token, checkpoint);
                    copiedRing.Add(ring[index]);
                }
                copy.Add(copiedRing);
            }
            Check(token, checkpoint);
            return copy;
        }

        private static bool SameGrid(Paths64 saved, Paths64 source, CancellationToken token, Action? checkpoint)
        {
            Check(token, checkpoint);
            if (saved.Count != source.Count) return false;
            for (int ringIndex = 0; ringIndex < saved.Count; ringIndex++)
            {
                Check(token, checkpoint);
                Path64 savedRing = saved[ringIndex], currentRing = source[ringIndex];
                if (currentRing is null || savedRing.Count != currentRing.Count) return false;
                for (int pointIndex = 0; pointIndex < savedRing.Count; pointIndex++)
                {
                    if ((pointIndex & 1023) == 0) Check(token, checkpoint);
                    Point64 a = savedRing[pointIndex], b = currentRing[pointIndex];
                    if (a.X != b.X || a.Y != b.Y) return false;
                }
            }
            Check(token, checkpoint);
            return true;
        }

        private static void Check(CancellationToken token, Action? checkpoint)
        {
            token.ThrowIfCancellationRequested();
            checkpoint?.Invoke();
            token.ThrowIfCancellationRequested();
        }

        private static ExactSignedBoundaryDecomposition.Diagnostics CopyDiagnostics(
            ExactSignedBoundaryDecomposition.Diagnostics source) => new()
        {
            Created = source.Created,
            UntrustedIntersectionRepairSkipped = source.UntrustedIntersectionRepairSkipped,
            Reason = source.Reason, InputPoints = source.InputPoints,
            MainTriangles = source.MainTriangles, CorrectionTriangles = source.CorrectionTriangles,
            ResidualEdges = source.ResidualEdges, FinalResidualEdges = source.FinalResidualEdges,
            TriangulationMilliseconds = source.TriangulationMilliseconds,
            ChainMilliseconds = source.ChainMilliseconds, TotalMilliseconds = source.TotalMilliseconds
        };
    }
}
