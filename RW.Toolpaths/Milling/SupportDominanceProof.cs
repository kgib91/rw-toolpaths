using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Milling;

// A proof about the EXISTING rounded binary64 instruction sequence, not a
// replacement kernel. Each later term is strictly above either the rounded
// initial grade*distance or the actual first-sphere tail. Neither certificate
// can change the subsequent literal Math.Min.
internal static class SupportDominanceProof
{
    private readonly record struct I(double Lo, double Hi)
    {
        internal bool Valid => NormalOrZero(Lo) && NormalOrZero(Hi) && Lo <= Hi;
        internal static I Point(double value) => new(value, value);
        internal static I Bad => new(double.NaN, double.NaN);
    }
    private const double MinimumNormal = 2.2250738585072014e-308;
    private static bool NormalOrZero(double value) => double.IsFinite(value)
        && (value == 0 || Math.Abs(value) >= MinimumNormal);
    private static bool Expanded(double value, bool exactZero, out double lo, out double hi)
    {
        lo = hi = double.NaN;
        if (!NormalOrZero(value) || (value == 0 && !exactZero)) return false;
        if (value == 0) { lo = hi = 0; return true; }
        lo = Math.BitDecrement(value); hi = Math.BitIncrement(value);
        return NormalOrZero(lo) && NormalOrZero(hi);
    }
    private static I Add(I a, I b)
    {
        if (!a.Valid || !b.Valid
            || !Expanded(a.Lo + b.Lo, a.Lo == -b.Lo, out double lo, out _)
            || !Expanded(a.Hi + b.Hi, a.Hi == -b.Hi, out _, out double hi)) return I.Bad;
        return new(lo, hi);
    }
    private static I Neg(I a) => a.Valid ? new(-a.Hi, -a.Lo) : I.Bad;
    private static I Sub(I a, I b) => Add(a, Neg(b));
    private static I Mul(I a, I b)
    {
        if (!a.Valid || !b.Valid) return I.Bad;
        double lo = double.PositiveInfinity, hi = double.NegativeInfinity;
        if (!Product(a.Lo, b.Lo) || !Product(a.Lo, b.Hi)
            || !Product(a.Hi, b.Lo) || !Product(a.Hi, b.Hi)) return I.Bad;
        return new(lo, hi);
        bool Product(double x, double y)
        {
            if (!Expanded(x * y, x == 0 || y == 0, out double low, out double high)) return false;
            lo = Math.Min(lo, low); hi = Math.Max(hi, high); return true;
        }
    }

    internal const double MaximumDistance = 1_000_000, MaximumGrade = 4;
    internal const double MaximumSectionDimension = 1_000_000;
    internal const int MaximumSections = 64;
    internal readonly record struct Section(int Ordinal, double LiteralHeight,
        double MaximumRadius, double RoundedProductUpper, double ExactGapLower);
    internal sealed record Owner(Section[] Sections, double RoundoffBound,
        long RetainedBytesBound);

    // Represented ranges: subtraction <=2^20; grade*subtraction and initial
    // grade*distance <2^22; height<=2^26; bottom<2^23; each sum<2^27.
    // A full ULP at each enclosing power-of-two bounds each RN error (also
    // subnormal results). Propagate subtraction by grade<=4. These exact dyadic
    // constants give 4*2^-32 +2^-30 +2*2^-25 +2^-30, with no fitted epsilon.
    internal static readonly double UniformRoundoffBound =
        3 * Math.ScaleB(1, -30) + 2 * Math.ScaleB(1, -25);

    internal static bool TryCreate(VCarveCutterProfile profile, double grade,
        out Owner? owner, out string reason)
    {
        owner = null; reason = "unsupported-domain-or-profile";
        if (!double.IsFinite(grade) || grade <= 0 || grade > MaximumGrade
            || profile.Sections.Count is < 2 or > MaximumSections) return false;
        var first = profile.Sections[0];
        if (first.Kind != CarvingCutterSectionKind.Hemisphere || first.BottomRadius != 0
            || first.Height != first.TopRadius || first.TopRadius <= 0) return false;
        double firstRadial=first.TopRadius*grade/Math.Sqrt(1+grade*grade);
        double firstBottom=firstRadial*firstRadial/(first.TopRadius+
            Math.Sqrt(Math.Max(0,first.TopRadius*first.TopRadius-firstRadial*firstRadial)));
        var firstOffset=Sub(I.Point(firstBottom),Mul(I.Point(grade),I.Point(firstRadial)));
        bool usedFirstTail=false;
        var proofs = new Section[profile.Sections.Count - 1];
        double height = 0;
        for (int i = 0; i < profile.Sections.Count; i++)
        {
            var section = profile.Sections[i];
            if (!double.IsFinite(section.Height) || section.Height <= 0
                || section.Height > MaximumSectionDimension
                || !double.IsFinite(section.BottomRadius) || section.BottomRadius < 0
                || section.BottomRadius > MaximumSectionDimension
                || !double.IsFinite(section.TopRadius) || section.TopRadius < 0
                || section.TopRadius > MaximumSectionDimension
                || !double.IsFinite(height) || height < 0 || height > Math.ScaleB(1, 26)) return false;
            if (i > 0)
            {
                if (section.Kind != CarvingCutterSectionKind.Linear) return false;
                // Bind the SAME rounded slope and branch as SupportDistance.
                // A steep flank/cylinder always selects its bottom radius;
                // its unused top radius cannot make this later term win.
                double slope=section.TopRadius>section.BottomRadius
                    ?section.Height/(section.TopRadius-section.BottomRadius):double.PositiveInfinity;
                double maximumRadius = slope < grade
                    ? Math.Max(section.BottomRadius, section.TopRadius) : section.BottomRadius;
                var product = Mul(I.Point(grade), I.Point(maximumRadius));
                var gap = Sub(I.Point(height), product);
                if (!product.Valid || !gap.Valid)
                { reason = "later-section-can-win-after-original-rounding"; return false; }
                if(gap.Lo<=UniformRoundoffBound)
                {
                    // The later branch runs only at distance>=BottomRadius.
                    // Once that is beyond the actual first-sphere transition,
                    // compare with B0+m*(distance-r0), retaining correlation in
                    // distance. At grades above one this is substantially below
                    // the initial m*distance used by the earlier certificate.
                    // Pay BOTH literal evaluation sequences' roundoff bounds.
                    if(!firstOffset.Valid||section.BottomRadius<firstRadial
                        ||!(gap=Sub(gap,firstOffset)).Valid||gap.Lo<=2*UniformRoundoffBound)
                    { reason = "later-section-can-win-after-original-rounding"; return false; }
                    usedFirstTail=true;
                }
                proofs[i - 1] = new(i, height, maximumRadius, product.Hi, gap.Lo);
            }
            // Bind exactly the ORIGINAL binary64 axial accumulator and order.
            height += section.Height;
        }
        owner = new(proofs, (usedFirstTail?2:1)*UniformRoundoffBound, 96L + 40L * proofs.Length);
        reason = usedFirstTail?"all-later-rounded-terms-strictly-above-original-first-sphere-tail"
            :"all-later-rounded-terms-strictly-above-initial-grade-distance";
        return true;
    }
}
