using System.Diagnostics;
using Clipper2Lib;

namespace RW.Toolpaths.Geometry;

public static partial class PolygonOps
{
    // Complete native snapshots only. This admits no editing approximation or
    // replacement site. Positive filled-hole dilation is an initial ORIGINAL
    // construction proposal; the existing executor owns all final validation.
    private sealed class OriginalHoleErosionSource
    {
        internal readonly Path64 Outer;
        internal readonly Path64[] PositiveHoles;
        internal readonly int Points;
        private OriginalHoleErosionSource(Path64 outer, Path64[] holes, int points)
        { Outer = outer; PositiveHoles = holes; Points = points; }

        internal static OriginalHoleErosionSource? Capture(Paths64 source,
            CancellationToken token, out string reason)
        {
            reason = "source-admission";
            long points = source.Sum(p => (long)p.Count);
            if (points <= 8192 || points > 1_000_000 || points > int.MaxValue)
            { reason = "source-count-budget"; return null; }
            foreach (var path in source)
            foreach (var point in path)
                if (!Bounded(point)) { reason = "native-coordinate-range"; return null; }
            token.ThrowIfCancellationRequested();
            var tree = new PolyTree64(); var clip = new Clipper64(); clip.AddSubject(source);
            if (!clip.Execute(ClipType.Union, FillRule.NonZero, tree) || tree.Count != 1
                || tree[0].IsHole || tree[0].Count < 32 || tree[0].Count > 2048
                || Enumerable.Range(0, tree[0].Count).Any(i => !tree[0][i].IsHole || tree[0][i].Count != 0))
            { reason = "canonical-hierarchy-or-hole-count"; return null; }
            var outer = tree[0].Polygon!;
            var allContours = new[] { outer }.Concat(Enumerable.Range(0, tree[0].Count)
                .Select(i => tree[0][i].Polygon!)).ToArray();
            if (allContours.Sum(p => (long)p.Count) > 1_000_000
                || allContours.Any(p => p.Count < 3 || p.Any(q => !Bounded(q))))
            { reason = "canonical-count-or-coordinate-range"; return null; }
            if (Area(outer) <= 0 || !Convex(outer)) { reason = "nonconvex-or-uncertain-outer"; return null; }
            var negatives = Enumerable.Range(0, tree[0].Count).Select(i => tree[0][i].Polygon!).ToArray();
            if (negatives.Any(p => Area(p) >= 0 || !double.IsFinite(Clipper.Area(p)) || Clipper.Area(p) >= 0))
            { reason = "hole-orientation"; return null; }
            var contours = new[] { outer }.Concat(negatives).ToArray();
            var seen = new Dictionary<NativeSegmentArrangement.Point, (int Ring, int Edge)>();
            var segments = new List<NativeSegmentArrangement.Segment>((int)points);
            var owners = new List<(int Ring, int Edge)>((int)points);
            for (int ring = 0; ring < contours.Length; ring++)
            for (int edge = 0; edge < contours[ring].Count; edge++)
            {
                if ((edge & 255) == 0) token.ThrowIfCancellationRequested();
                var a = contours[ring][edge]; var b = contours[ring][(edge + 1) % contours[ring].Count];
                if (!Bounded(a) || !Bounded(b)) { reason = "canonical-coordinate-range"; return null; }
                var na = new NativeSegmentArrangement.Point((int)a.X, (int)a.Y);
                var nb = new NativeSegmentArrangement.Point((int)b.X, (int)b.Y);
                if (na == nb || !seen.TryAdd(na, (ring, edge)))
                { reason = "touching-or-repeated-canonical-knot"; return null; }
                segments.Add(new(na, nb)); owners.Add((ring, edge));
            }
            // The tree supplies only hierarchy. Retain the complete canonical
            // source's directed native cycles, allowing a different start knot.
            // A Boolean that changed a knot or finite edge is not this proposal.
            if (source.Count != contours.Length) { reason = "original-chain-count-changed"; return null; }
            var matched = new bool[contours.Length];
            foreach (var path in source)
            {
                token.ThrowIfCancellationRequested();
                if (path.Count == 0 || !seen.TryGetValue(new((int)path[0].X, (int)path[0].Y), out var start)
                    || matched[start.Ring] || path.Count != contours[start.Ring].Count)
                { reason = "original-chain-changed"; return null; }
                var contour = contours[start.Ring];
                for (int i = 0; i < path.Count; i++)
                {
                    if ((i & 255) == 0) token.ThrowIfCancellationRequested();
                    var expected = contour[(start.Edge + i) % contour.Count];
                    if (path[i].X != expected.X || path[i].Y != expected.Y)
                    { reason = "original-chain-changed"; return null; }
                }
                matched[start.Ring] = true;
            }
            NativeSegmentArrangement.Result contacts;
            try { contacts = NativeSegmentArrangement.Prepare(segments, token); }
            catch (NativeSegmentArrangement.UnsupportedContactException failure)
            { reason = "contact-" + failure.Reason; return null; }
            catch (InvalidOperationException failure)
            {
                Console.WriteLine($"[PolygonOps.FirstHoleErosionAdmission] admitted=False "
                    + $"reason=contact-certificate-refusal detail={failure.Message} originalChainChanged=False");
                reason = "contact-certificate-refusal"; return null;
            }
            int adjacentCollinearKnots = 0, ordinaryEndpointKnots = 0;
            if (contacts.Atoms is { } atoms)
            {
                // At every introduced contact knot collect ALL original owners,
                // including an unsplit endpoint meeting another segment's interior.
                var knots = new Dictionary<NativeSegmentArrangement.Point, HashSet<int>>();
                foreach (var atom in atoms)
                {
                    token.ThrowIfCancellationRequested();
                    var original = segments[atom.OriginalOrdinal];
                    foreach (var point in new[] { atom.Segment.A, atom.Segment.B })
                        if (point != original.A && point != original.B) knots.TryAdd(point, new());
                }
                if (knots.Count == 0) { reason = "unsupported-duplicate-contact"; return null; }
                foreach (var atom in atoms)
                {
                    token.ThrowIfCancellationRequested();
                    foreach (var point in new[] { atom.Segment.A, atom.Segment.B })
                    if (knots.TryGetValue(point, out var parents))
                    {
                        parents.Add(atom.OriginalOrdinal);
                        foreach (int ordinal in atom.AdditionalOriginalOrdinals) parents.Add(ordinal);
                    }
                }
                var contourCounts = contours.Select(p => p.Count).ToArray();
                foreach (var contact in knots)
                {
                    var parents = contact.Value.ToArray();
                    int pattern = AdjacentOverlapJunction(contact.Key, parents, segments, owners, contourCounts);
                    if (pattern == 0)
                    {
                        Console.WriteLine($"[PolygonOps.FirstHoleErosionContact] admitted=False "
                            + $"knot=({contact.Key.X},{contact.Key.Y}) parents={parents.Length} "
                            + $"owners={string.Join(";", parents.Take(8).Select(p => $"{p}:r{owners[p].Ring}/e{owners[p].Edge}"))} "
                            + "reason=not-one-adjacent-collinear-overlap-with-ordinary-endpoint-owner originalChainChanged=False");
                        reason = "unsupported-overlap-junction"; return null;
                    }
                    adjacentCollinearKnots++;
                    if (pattern == 2) ordinaryEndpointKnots++;
                }
            }
            token.ThrowIfCancellationRequested();
            var positives = negatives.Select(p => { var copy = new Path64(p); copy.Reverse(); return copy; }).ToArray();
            Console.WriteLine($"[PolygonOps.FirstHoleErosionAdmission] admitted=True originalPoints={points} "
                + $"holes={positives.Length} adjacentCollinearKnots={adjacentCollinearKnots} "
                + $"ordinaryEndpointKnots={ordinaryEndpointKnots} "
                + $"contactBuildMs={contacts.BuildMilliseconds:F2} contactQueryMs={contacts.QueryMilliseconds:F2} "
                + $"contactCertificateMs={contacts.CertificateMilliseconds:F2} completeFiniteUnion=True "
                + "originalChainChanged=False originalSignedClassifier=unchanged noNestedPositiveIslands=True");
            reason = "admitted";
            return new(new Path64(outer), positives, segments.Count);
        }

        private static bool Bounded(Point64 p) => p.X > -(1L << 30) && p.X < (1L << 30)
            && p.Y > -(1L << 30) && p.Y < (1L << 30);
        private static Int128 Area(Path64 path)
        {
            Int128 area = 0;
            for (int i = 0; i < path.Count; i++)
            { var a = path[i]; var b = path[(i + 1) % path.Count]; area += (Int128)a.X * b.Y - (Int128)a.Y * b.X; }
            return area;
        }
        private static bool Convex(Path64 path)
        {
            if (path.Count < 3) return false;
            for (int i = 0; i < path.Count; i++)
            {
                var a = path[i]; var b = path[(i + 1) % path.Count]; var c = path[(i + 2) % path.Count];
                if (((Int128)b.X - a.X) * ((Int128)c.Y - b.Y) - ((Int128)b.Y - a.Y) * ((Int128)c.X - b.X) < 0) return false;
            }
            return true;
        }
        private static Int128 Cross(NativeSegmentArrangement.Point a, NativeSegmentArrangement.Point b,
            NativeSegmentArrangement.Point c) => ((Int128)b.X - a.X) * ((Int128)c.Y - a.Y)
                - ((Int128)b.Y - a.Y) * ((Int128)c.X - a.X);

        // Return 1 for a single adjacent collinear reversal, 2 when its split
        // knot also has the ordinary preceding/following endpoint owner. This
        // is a certificate over ALL normalized atom owners, not a contact merge.
        internal static int AdjacentOverlapJunction(NativeSegmentArrangement.Point knot,
            IReadOnlyList<int> parents, IReadOnlyList<NativeSegmentArrangement.Segment> segments,
            IReadOnlyList<(int Ring, int Edge)> owners, IReadOnlyList<int> contourCounts)
        {
            if (parents.Count is < 2 or > 3) return 0;
            int ring = owners[parents[0]].Ring;
            if (parents.Any(p => owners[p].Ring != ring)) return 0;
            int count = contourCounts[ring];
            bool Reversal(NativeSegmentArrangement.Segment a, NativeSegmentArrangement.Segment b)
                => a.B == b.A && Cross(a.A, a.B, b.A) == 0 && Cross(a.A, a.B, b.B) == 0
                    && ((Int128)a.B.X - a.A.X) * ((Int128)b.B.X - b.A.X)
                        + ((Int128)a.B.Y - a.A.Y) * ((Int128)b.B.Y - b.A.Y) < 0;
            bool Interior(NativeSegmentArrangement.Segment segment)
            {
                if (Cross(segment.A, segment.B, knot) != 0) return false;
                Int128 dx = (Int128)segment.B.X - segment.A.X, dy = (Int128)segment.B.Y - segment.A.Y;
                Int128 projection = dx * ((Int128)knot.X - segment.A.X) + dy * ((Int128)knot.Y - segment.A.Y);
                return projection > 0 && projection < dx * dx + dy * dy;
            }
            foreach (int firstOrdinal in parents)
            {
                int firstEdge = owners[firstOrdinal].Edge;
                int secondOrdinal = -1, thirdOrdinal = -1;
                foreach (int ordinal in parents)
                {
                    if (owners[ordinal].Edge == (firstEdge + 1) % count) secondOrdinal = ordinal;
                    if (owners[ordinal].Edge == (firstEdge + 2) % count) thirdOrdinal = ordinal;
                }
                if (secondOrdinal < 0) continue;
                var first = segments[firstOrdinal]; var second = segments[secondOrdinal];
                if (parents.Count == 2)
                {
                    if (Reversal(first, second) && (second.B == knot && Interior(first)
                        || first.A == knot && Interior(second))) return 1;
                    continue;
                }
                if (thirdOrdinal < 0) continue;
                var third = segments[thirdOrdinal];
                // Forward: long→short reversal, then the ordinary next edge.
                // Reverse: ordinary prior edge, then short→long reversal.
                // The ordinary owner must depart the common line; a third
                // overlapping collinear site is a different contact pattern.
                if (Reversal(first, second) && second.B == knot && third.A == knot && Interior(first)
                    && Cross(first.A, first.B, third.B) != 0) return 2;
                if (Reversal(second, third) && first.B == knot && second.A == knot && Interior(third)
                    && Cross(third.A, third.B, first.A) != 0) return 2;
            }
            return 0;
        }
    }

    private static bool TryBuildOriginalHoleErosion(PreparedBoundarySource geometry,
        double scaledDelta, double arc, CancellationToken token, out Paths64 result)
    {
        result = new Paths64(); token.ThrowIfCancellationRequested();
        bool Refuse(string reason)
        {
            token.ThrowIfCancellationRequested();
            Console.WriteLine($"[PolygonOps.FirstHoleErosion] proposed=False reason={reason} "
                + "fallback=complete-original-boundary-band stockAuthority=False");
            return false;
        }
        if (!double.IsFinite(scaledDelta) || scaledDelta >= -4 || -scaledDelta > 64 * PathUtils.Scale
            || !double.IsFinite(arc) || arc < 1 / PathUtils.Scale) return Refuse("inset-or-arc-range");
        long started = Stopwatch.GetTimestamp();
        var source = geometry.OriginalHoleSource(token, out string reason);
        if (source is null) return Refuse(reason);
        double actualArc = Math.Min(arc, .01);
        double guard = Math.Max(4, 2 * actualArc * PathUtils.Scale) + 4;
        Paths64 Offset(Path64 path, double delta)
        {
            token.ThrowIfCancellationRequested(); var offset = CreateOffsetter(actualArc);
            offset.AddPath(new Path64(path), JoinType.Round, EndType.Polygon);
            var paths = new Paths64(); offset.Execute(delta, paths); token.ThrowIfCancellationRequested(); return paths;
        }
        var outer = Offset(source.Outer, scaledDelta - guard);
        if (outer.Count == 0) return Refuse("empty-eroded-outer");
        long constructionStarted = Stopwatch.GetTimestamp();
        var dilated = new Paths64[source.PositiveHoles.Length];
        Parallel.For(0, dilated.Length, new ParallelOptions { CancellationToken = token,
            MaxDegreeOfParallelism = GeometryParallelism.WorkerLimit },
            i => dilated[i] = Offset(source.PositiveHoles[i], -scaledDelta + guard));
        var operands = new Paths64(); foreach (var paths in dilated) operands.AddRange(paths);
        if (operands.Sum(p => (long)p.Count) > 8_000_000) return Refuse("dilation-point-budget");
        // Negative child output from a positive dilation bounds a safe cut island.
        // Retain ALL signed pieces through this union and the final difference.
        var expanded = Clipper.Union(operands, FillRule.NonZero); token.ThrowIfCancellationRequested();
        var candidate = Clipper.Difference(outer, expanded, FillRule.NonZero); token.ThrowIfCancellationRequested();
        double constructionMs = ElapsedMilliseconds(constructionStarted);
        if (candidate.Count == 0) return Refuse("empty-candidate");
        bool Bounded(Paths64 paths)
        {
            long points = 0;
            foreach (var path in paths)
            foreach (var p in path)
            {
                if ((++points & 255) == 0) token.ThrowIfCancellationRequested();
                if (p.X <= -(1L << 30) || p.X >= (1L << 30)
                    || p.Y <= -(1L << 30) || p.Y >= (1L << 30)) return false;
            }
            return points <= 8_000_000;
        }
        if (!Bounded(outer) || !Bounded(expanded) || !Bounded(candidate))
            return Refuse("constructed-count-or-native-coordinate-range");
        long filledStarted = Stopwatch.GetTimestamp();
        // Difference already applies the complete NonZero subject fill; a second
        // full union of the original positive hole subjects is unnecessary.
        bool filled = Clipper.Difference(new Paths64(source.PositiveHoles), expanded, FillRule.NonZero).Count == 0;
        token.ThrowIfCancellationRequested(); double filledMs = ElapsedMilliseconds(filledStarted);
        long capsuleStarted = Stopwatch.GetTimestamp(); var expandedBoundary = new BoundarySegmentIndex(expanded);
        long chords = 0; bool capsules = filled;
        int rejectedHole = -1, rejectedEdge = -1;
        if (capsules)
        for (int holeIndex = 0; holeIndex < source.PositiveHoles.Length; holeIndex++)
        {
            var hole = source.PositiveHoles[holeIndex];
            for (int i = 0; i < hole.Count; i++)
            {
                if ((++chords & 255) == 0) token.ThrowIfCancellationRequested();
                // ContainsCapsule adds its existing four-native-quantum guard.
                if (expandedBoundary.ContainsCapsule(hole[i], hole[(i + 1) % hole.Count], -scaledDelta)) continue;
                capsules = false; rejectedHole = holeIndex; rejectedEdge = i; break;
            }
            if (!capsules) break;
        }
        token.ThrowIfCancellationRequested();
        Console.WriteLine($"[PolygonOps.FirstHoleErosion] proposed={capsules} originalPoints={source.Points} "
            + $"holes={source.PositiveHoles.Length} candidatePoints={candidate.Sum(p => p.Count)} "
            + $"signedDilationPieces={operands.Count} completeHoleFilledSubset={filled} "
            + $"completeFiniteCapsules={capsules} capsuleChords={chords} radius={-scaledDelta / PathUtils.Scale:R} "
            + $"certificateReason={(capsules ? "complete" : filled ? "finite-capsule-refusal" : "filled-subset-refusal")} "
            + $"rejectedHole={rejectedHole} rejectedEdge={rejectedEdge} "
            + $"guard={guard / PathUtils.Scale:R} constructionMs={constructionMs:F2} filledProofMs={filledMs:F2} "
            + $"capsuleProofMs={ElapsedMilliseconds(capsuleStarted):F2} totalMs={ElapsedMilliseconds(started):F2} "
            + "firstOriginalOnly=True originalChainChanged=False allSignedPiecesRetained=True "
            + "inversionRepair=unchanged firstMutualCleanup=unchanged stockAuthority=False "
            + "refusal=complete-original-boundary-band");
        if (!capsules) return false;
        result = candidate; return true;
    }
}
