namespace RW.Toolpaths.Milling.Certification;

/// <summary>Zero-added-tolerance positive containment of the complete stock
/// volume swept by one actual posted chord in one actual retained endpoint.
/// The caller must separately establish executed cursor/cutting provenance
/// and that the mandatory endpoint remains in the final command snapshot.</summary>
internal static class VCarveEndpointVolumeDominance
{
    internal const double ArithmeticReserve = 1e-8;
    internal const double WorldCoordinateLimit = 10000;
    internal const double PhysicalLimit = 100;
    internal const double MinimumSectionHeight = 1e-6;
    internal const double MinimumSphereEvaluationDepth = .0005;
    private const double DepthDomainReserve = 2e-11;

    internal sealed record PreparedProfile(VCarveCutterProfile Profile, double RepresentedHeight,
        double FirstHemisphereRadius, bool HeightSumExact, bool Supported, string Reason);

    internal readonly record struct Proof(bool Proven, string Mode, string Reason,
        double SourceDepth, double MandatoryDepth, double MinimumMandatoryAxialHeight,
        double MaximumAxisDistance, double SourceRadius, double MinimumMandatoryRadius,
        double RequiredRadius, double UnreservedMargin, int RadiusEvaluations)
    {
        internal string Describe() => $"stage=endpoint-volume-dominance proven={Proven} mode={Mode} reason='{Reason}' "
            + $"sourceDepth={SourceDepth:R} mandatoryDepth={MandatoryDepth:R} minimumMandatoryHeight={MinimumMandatoryAxialHeight:R} "
            + $"maximumAxisDistance={MaximumAxisDistance:R} sourceRadius={SourceRadius:R} minimumMandatoryRadius={MinimumMandatoryRadius:R} "
            + $"requiredRadius={RequiredRadius:R} unreservedMargin={UnreservedMargin:R} radiusEvaluations={RadiusEvaluations} "
            + $"arithmeticReserve={ArithmeticReserve:R} addedTolerance=0 callerOwnsPostedCursorAndRetainedEndpoint=True";
    }

    private readonly record struct Posted(long X, long Y, long Z);

    internal static PreparedProfile Prepare(VCarveCutterProfile profile)
    {
        ArgumentNullException.ThrowIfNull(profile);
        PreparedProfile Refused(string reason) => new(profile, 0, 0, false, false, reason);
        if (profile.Sections.Count is < 1 or > 32 || !double.IsFinite(profile.MaximumDepth)
            || profile.MaximumDepth <= 0 || profile.MaximumDepth > PhysicalLimit) return Refused("profile-depth-or-count-domain");
        double height = 0, previousRadius = profile.Sections[0].BottomRadius, sphereRadius = 0;
        bool exactSum = true;
        for (int index = 0; index < profile.Sections.Count; index++)
        {
            CarvingCutterSection section = profile.Sections[index];
            if (!FiniteBounded(section.BottomRadius, PhysicalLimit) || !FiniteBounded(section.TopRadius, PhysicalLimit)
                || section.BottomRadius < 0 || section.TopRadius <= 0
                || !FiniteBounded(section.Height, PhysicalLimit) || section.Height < MinimumSectionHeight)
                return Refused("profile-dimension-or-microscopic-section");
            if (section.BottomRadius != previousRadius) return Refused("profile-not-continuous");
            if (section.TopRadius < section.BottomRadius) return Refused("profile-not-monotone");
            if (section.Kind == CarvingCutterSectionKind.Hemisphere)
            {
                if (index != 0 || section.BottomRadius != 0 || section.Height != section.TopRadius)
                    return Refused("profile-not-first-full-hemisphere");
                sphereRadius = section.TopRadius;
            }
            else if (section.Kind != CarvingCutterSectionKind.Linear
                || (section.TopRadius - section.BottomRadius) / section.Height > PhysicalLimit)
                return Refused("profile-kind-or-linear-slope");
            double end = height + section.Height;
            exactSum &= AdditionError(height, section.Height, end) == 0;
            if (!(end > height) || end - height < MinimumSectionHeight / 2)
                return Refused("profile-aliased-section-join");
            if (end > PhysicalLimit || end == PhysicalLimit && !exactSum)
                return Refused("profile-total-height-domain");
            height = end; previousRadius = section.TopRadius;
        }
        if (profile.MaximumDepth > height) return Refused("profile-depth-exceeds-representation");
        return new(profile, height, sphereRadius, exactSum, true, "continuous-monotone-represented-profile");
    }

    internal static bool TryProve(Point3D postedRayA, Point3D postedRayB, Point3D postedMandatoryEndpoint,
        VCarveCutterProfile profile, double stockSurfaceZ, out Proof proof,
        CancellationToken cancellationToken = default, Action<string>? diagnostic = null)
    {
        cancellationToken.ThrowIfCancellationRequested();
        return TryProve(postedRayA, postedRayB, postedMandatoryEndpoint, Prepare(profile), stockSurfaceZ,
            out proof, cancellationToken, diagnostic);
    }

    internal static bool TryProve(Point3D postedRayA, Point3D postedRayB, Point3D postedMandatoryEndpoint,
        PreparedProfile prepared, double stockSurfaceZ, out Proof proof,
        CancellationToken cancellationToken = default, Action<string>? diagnostic = null)
    {
        ArgumentNullException.ThrowIfNull(prepared);
        cancellationToken.ThrowIfCancellationRequested();
        proof = new(false, "refused", "not-started", double.NaN, double.NaN, double.NaN,
            double.NaN, double.NaN, double.NaN, double.NaN, double.NaN, 0);
        if (!prepared.Supported) return Refuse(ref proof, prepared.Reason, diagnostic);
        if (!FiniteBounded(stockSurfaceZ, WorldCoordinateLimit)
            || !TryPosted(postedRayA, out Posted a) || !TryPosted(postedRayB, out Posted b)
            || !TryPosted(postedMandatoryEndpoint, out Posted m))
            return Refuse(ref proof, "not-ordinary-complete-posted-coordinates", diagnostic);
        double d = Math.Max(0, Math.Max(stockSurfaceZ - postedRayA.Z, stockSurfaceZ - postedRayB.Z));
        double depth = stockSurfaceZ - postedMandatoryEndpoint.Z;
        double minimumHeight = depth - d;
        proof = proof with { SourceDepth = d, MandatoryDepth = depth, MinimumMandatoryAxialHeight = minimumHeight };
        if (!(d > DepthDomainReserve) || !(depth > DepthDomainReserve))
            return Refuse(ref proof, "no-unambiguous-stock-engagement", diagnostic);
        if (m.Z > Math.Min(a.Z, b.Z) || minimumHeight < 0)
            return Refuse(ref proof, "mandatory-endpoint-shallower", diagnostic);
        if (!FiniteBounded(d, PhysicalLimit) || !FiniteBounded(depth, PhysicalLimit)
            || !Represented(prepared, stockSurfaceZ, postedMandatoryEndpoint.Z, m.Z, depth))
            return Refuse(ref proof, "outside-or-ambiguous-represented-height", diagnostic);

        // Actual posted integer equality has no XY uncertainty. The source axis
        // is constant and the mandatory tip is no higher. At every stock height
        // its local physical radius is at least the source radius, even for a
        // cylinder. No radial arithmetic, margin, or tolerance is needed.
        if (a.X == b.X && a.X == m.X && a.Y == b.Y && a.Y == m.Y)
        {
            proof = proof with { Proven = true, Mode = "exact-same-axis", Reason = "same-axis-monotone-depth-dominance",
                MaximumAxisDistance = 0, UnreservedMargin = 0 };
            diagnostic?.Invoke(proof.Describe()); return true;
        }

        if (NearSphereTip(d, prepared.FirstHemisphereRadius) || NearSphereTip(minimumHeight, prepared.FirstHemisphereRadius))
            return Refuse(ref proof, "near-tip-radius-evaluation-domain", diagnostic);
        double separation = Math.Max(Distance(postedRayA, postedMandatoryEndpoint), Distance(postedRayB, postedMandatoryEndpoint));
        double sourceRadius = prepared.Profile.PhysicalRadiusAtDepth(d);
        double mandatoryRadius = prepared.Profile.PhysicalRadiusAtDepth(minimumHeight);
        double required = separation + sourceRadius + ArithmeticReserve;
        proof = proof with { MaximumAxisDistance = separation, SourceRadius = sourceRadius,
            MinimumMandatoryRadius = mandatoryRadius, RequiredRadius = required,
            UnreservedMargin = mandatoryRadius - (separation + sourceRadius), RadiusEvaluations = 2 };
        if (!FiniteBounded(sourceRadius, PhysicalLimit + DepthDomainReserve) || sourceRadius < 0
            || !FiniteBounded(mandatoryRadius, PhysicalLimit + DepthDomainReserve) || mandatoryRadius < 0)
            return Refuse(ref proof, "literal-physical-radius-domain", diagnostic);

        // For every possible source stock z in [surface-d,surface], every source
        // physical slice radius<=r(d), while the retained endpoint radius>=r(D-d).
        // Norm convexity bounds every interpolated source axis separation by its
        // two endpoint distances. This is complete 3D stock containment, with
        // no credit for points above the stock surface or below the source tip.
        // F3 decode error at world |XYZ|<=1e4 is <1e-12 per axis; height sums have
        // <=32 terms, each<=100. First-sphere evaluated positive depths>=.0005
        // give derivative<317 (R<=100); linear slopes<=100. Together with depth,
        // norm, profile and sqrt arithmetic, worst radius/distance error<2e-9.
        // The fixed 1e-8 reserve strictly shrinks evidence, never enlarges stock
        // tolerance. Exact same-axis proofs avoid these radius evaluations.
        if (!(required <= mandatoryRadius)) return Refuse(ref proof, "insufficient-strict-volume-margin", diagnostic);
        proof = proof with { Proven = true, Mode = "strict-radial-margin", Reason = "whole-stock-height-endpoint-dominance" };
        diagnostic?.Invoke(proof.Describe()); return true;
    }

    private static bool Represented(PreparedProfile prepared, double surface, double mandatoryZ, long postedZ, double depth)
    {
        if (depth <= prepared.RepresentedHeight - DepthDomainReserve) return true;
        // Admit exact boundary equality only when the physical height sum and
        // depth subtraction are exact, and F3 Z is an exactly binary-represented
        // eighth-millimetre value. Otherwise an aliased represented top refuses.
        return depth <= prepared.RepresentedHeight && prepared.HeightSumExact && postedZ % 125 == 0
            && AdditionError(surface, -mandatoryZ, depth) == 0;
    }
    private static bool NearSphereTip(double depth, double radius)
        => radius > 0 && depth > 0 && depth < radius && depth < MinimumSphereEvaluationDepth;
    private static bool TryPosted(Point3D point, out Posted posted)
    {
        posted = default;
        if (!Coordinate(point.X, out long x) || !Coordinate(point.Y, out long y) || !Coordinate(point.Z, out long z)) return false;
        posted = new(x, y, z); return true;
    }
    private static bool Coordinate(double value, out long integer)
    {
        integer = 0;
        if (!FiniteBounded(value, WorldCoordinateLimit)) return false;
        integer = (long)Math.Round(value * 1000, MidpointRounding.ToEven);
        return integer / 1000d == value;
    }
    private static bool FiniteBounded(double value, double maximum) => double.IsFinite(value) && Math.Abs(value) <= maximum;
    private static double Distance(Point3D a, Point3D b)
    { double x = a.X - b.X, y = a.Y - b.Y; return Math.Sqrt(x * x + y * y); }
    private static double AdditionError(double a, double b, double sum)
    { double part = sum - a; return (a - (sum - part)) + (b - part); }
    private static bool Refuse(ref Proof proof, string reason, Action<string>? diagnostic)
    {
        proof = proof with { Proven = false, Reason = reason };
        diagnostic?.Invoke(proof.Describe()); return false;
    }
}
