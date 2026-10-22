using System.Numerics;
using RW.Toolpaths;

namespace RW.Toolpaths.Milling;

// Source-only lower construction. The caller still owns literal active spans,
// original Emit/ordered hull/grid/union/post-Boolean reserves and all stock guards.
internal sealed class TaperedRadiusRun
{
    internal const int MaximumChords = 256, MinimumChords = 3;
    const double MaximumXY = 10000, MaximumZ = 100, MaximumDepth = 100, MaximumRadius = 100;
    // Inherited proven 50-interpolation depth allowance; not a new geometry tolerance.
    const double OriginalDepthRoundoff = 5e-12;
    const double MinimumLinearHeight = 1e-6;
    const double MinimumInsetRadius = 8 / PathUtils.Scale;

    internal enum Refusal { None, Profile, Coordinate, Depth, Section, Inactive, ZeroXY, Short, Parameter, Tip, Loss, Arithmetic, SourceChanged }
    internal sealed class Counters
    {
        internal long Requests, Accepted, SourcePoints, FitPoints, CandidateFits, SphereBulges, EnclosureArithmeticCalls, TokenBarriers, SqrtEnclosureChecks;
        internal readonly long[] Refusals = new long[Enum.GetValues<Refusal>().Length];
        internal double MaximumAcceptedLoss, MaximumAcceptedReduction, MaximumAcceptedRadiusSpread;
        internal double LastRejectedLoss, LastRejectedCenterError, LastRejectedReduction;
    }
    internal readonly record struct Certificate(int First, int Last, int Section, double Radius0, double Radius1,
        double Reduction, double EndpointReductionUpper, double MaximumCenterError, double MaximumLoss,
        double MaximumSphereBulge, double RadiusSpread, double ArcLower, double ArcUpper);
    internal readonly record struct Interval(double Lo, double Hi);
    readonly record struct Site(double X, double Y, double Z, Interval Depth, Interval Radius, Interval Arc, double Bulge);
    sealed class ArithmeticRefusal : Exception { }

    // One workspace per Build, reused for every bounded attempt. No per-run lists.
    readonly Site[] _sites = new Site[MaximumChords + 1];
    readonly double[] _parameters = new double[MaximumChords + 1], _errors = new double[MaximumChords + 1];
    readonly Interval[] _starts;
    readonly VCarveCutterProfile _profile;
    readonly double _surface;
    readonly bool _eligible;
    int _count, _section, _sourceCount;
    internal Counters Metrics { get; } = new();
    internal Refusal LastRefusal { get; private set; }

    internal TaperedRadiusRun(VCarveCutterProfile profile, double surface, CancellationToken token)
    {
        ArgumentNullException.ThrowIfNull(profile);_profile = profile;_surface = surface;
        Barrier(token);_starts = new Interval[65];_starts[0] = Point(0);
        if (profile.Sections.Count is < 1 or > 64 || !FiniteBound(surface, MaximumZ)) return;
        double previous = profile.Sections[0].BottomRadius;
        try
        {
            for (int i = 0; i < profile.Sections.Count; i++)
            {
                Barrier(token);var shape = profile.Sections[i];
                if (!FiniteBound(shape.Height, MaximumDepth) || !(shape.Height > 0)
                    || !FiniteBound(shape.BottomRadius, MaximumRadius) || !FiniteBound(shape.TopRadius, MaximumRadius)
                    || shape.BottomRadius < 0 || shape.TopRadius < shape.BottomRadius || shape.BottomRadius != previous
                    || (shape.Kind == CarvingCutterSectionKind.Hemisphere && (i != 0 || shape.BottomRadius != 0 || shape.Height != shape.TopRadius))
                    || (shape.Kind == CarvingCutterSectionKind.Linear && shape.Height < MinimumLinearHeight)
                    || (shape.Kind != CarvingCutterSectionKind.Hemisphere && shape.Kind != CarvingCutterSectionKind.Linear)) return;
                _starts[i + 1] = Add(_starts[i], Point(shape.Height));
                if (!(_starts[i + 1].Lo > _starts[i].Hi)) return;
                previous = shape.TopRadius;
            }
            _eligible = true;
        }
        catch (ArithmeticRefusal) { }
        Barrier(token);
    }

    internal bool Eligible => _eligible;
    void Barrier(CancellationToken token) { Metrics.TokenBarriers++; token.ThrowIfCancellationRequested(); }
    static bool FiniteBound(double value, double bound) => double.IsFinite(value) && Math.Abs(value) <= bound;
    static Interval Point(double value) => new(value, value);
    Interval Enclose(double lower, double upper)
    {
        if (!double.IsFinite(lower) || !double.IsFinite(upper) || lower > upper) throw new ArithmeticRefusal();
        return new(lower, upper);
    }
    Interval Add(Interval a, Interval b) { Metrics.EnclosureArithmeticCalls++;return Enclose(Math.BitDecrement(a.Lo + b.Lo), Math.BitIncrement(a.Hi + b.Hi)); }
    Interval Sub(Interval a, Interval b) { Metrics.EnclosureArithmeticCalls++;return Enclose(Math.BitDecrement(a.Lo - b.Hi), Math.BitIncrement(a.Hi - b.Lo)); }
    Interval Mul(Interval a, Interval b)
    {
        Metrics.EnclosureArithmeticCalls++;
        double aa = a.Lo * b.Lo, ab = a.Lo * b.Hi, ba = a.Hi * b.Lo, bb = a.Hi * b.Hi;
        return Enclose(Math.BitDecrement(Math.Min(Math.Min(aa, ab), Math.Min(ba, bb))), Math.BitIncrement(Math.Max(Math.Max(aa, ab), Math.Max(ba, bb))));
    }
    Interval Div(Interval a, Interval b)
    {
        Metrics.EnclosureArithmeticCalls++;if (!(b.Lo > 0)) throw new ArithmeticRefusal();
        double aa = a.Lo / b.Lo, ab = a.Lo / b.Hi, ba = a.Hi / b.Lo, bb = a.Hi / b.Hi;
        return Enclose(Math.BitDecrement(Math.Min(Math.Min(aa, ab), Math.Min(ba, bb))), Math.BitIncrement(Math.Max(Math.Max(aa, ab), Math.Max(ba, bb))));
    }
    Interval Square(Interval a)
    {
        Metrics.EnclosureArithmeticCalls++;double low = a.Lo <= 0 && a.Hi >= 0 ? 0 : Math.Min(Math.Abs(a.Lo), Math.Abs(a.Hi));
        double high = Math.Max(Math.Abs(a.Lo), Math.Abs(a.Hi));
        return Enclose(Math.Max(0, Math.BitDecrement(low * low)), Math.BitIncrement(high * high));
    }
    Interval Sqrt(Interval a)
    {
        Metrics.EnclosureArithmeticCalls++;if (a.Hi < 0) throw new ArithmeticRefusal();
        double lowerInput = Math.Max(0, a.Lo);
        double lower = Math.Max(0, Math.BitDecrement(Math.Sqrt(lowerInput))), upper = Math.BitIncrement(Math.Sqrt(a.Hi));
        Metrics.SqrtEnclosureChecks += 2;
        if (!double.IsFinite(lower) || !double.IsFinite(upper) || CompareSquare(lower, lowerInput) > 0 || CompareSquare(upper, a.Hi) < 0)
            throw new ArithmeticRefusal();
        return Enclose(lower, upper);
    }
    internal static int CompareSquare(double root, double radicand)
    {
        if (root == 0) return radicand == 0 ? 0 : -1;
        if (radicand == 0) return 1;
        Dyadic(root, out ulong rm, out int re); Dyadic(radicand, out ulong xm, out int xe);
        UInt128 left = (UInt128)rm * rm, right = xm;
        int le = 2 * re, leftTop = BitLength(left) + le, rightTop = BitLength(right) + xe;
        if (leftTop != rightTop) return leftTop.CompareTo(rightTop);
        // Equal top exponents imply that this alignment fits the 106-bit
        // square significand. It never truncates an overflowed integer.
        if (le > xe) left <<= le - xe; else if (xe > le) right <<= xe - le;
        return left.CompareTo(right);
    }
    private static int BitLength(UInt128 value)
    {
        ulong high = (ulong)(value >> 64);
        return high == 0 ? 64 - BitOperations.LeadingZeroCount((ulong)value)
            : 128 - BitOperations.LeadingZeroCount(high);
    }
    private static void Dyadic(double value, out ulong mantissa, out int exponent)
    {
        ulong bits = BitConverter.DoubleToUInt64Bits(value), raw = (bits >> 52) & 2047;
        mantissa = (bits & ((1UL << 52) - 1)) | (raw == 0 ? 0 : 1UL << 52);
        exponent = raw == 0 ? -1074 : (int)raw - 1075;
    }

    internal Interval CheckedSqrt(Interval a) => Sqrt(a);

    Interval Lerp(double a, double b, double t)
    {
        if (t == 0) return Point(a);if (t == 1) return Point(b);
        return Add(Mul(Sub(Point(1), Point(t)), Point(a)), Mul(Point(t), Point(b)));
    }
    Interval Distance(Interval dx, Interval dy) => Sqrt(Add(Square(dx), Square(dy)));

    bool Read(Point3D p, out Interval depth, out Interval radius, out int section)
    {
        Metrics.SourcePoints++;depth = radius = default;section = -1;
        if (!FiniteBound(p.X, MaximumXY) || !FiniteBound(p.Y, MaximumXY) || !FiniteBound(p.Z, MaximumZ))
        { LastRefusal = Refusal.Coordinate;return false; }
        depth = Add(Sub(Point(_surface), Point(p.Z)), new(-OriginalDepthRoundoff, OriginalDepthRoundoff));
        if (!(depth.Lo > 0) || depth.Hi > MaximumDepth) { LastRefusal = Refusal.Depth;return false; }
        for (int i = 0; i < _profile.Sections.Count; i++)
        {
            if (!(depth.Lo > _starts[i].Hi && depth.Hi < _starts[i + 1].Lo)) continue;
            var shape = _profile.Sections[i];depth = Sub(depth, _starts[i]);
            radius = shape.Kind == CarvingCutterSectionKind.Hemisphere
                ? Sqrt(Mul(depth, Sub(Point(2 * shape.TopRadius), depth)))
                : shape.TopRadius == shape.BottomRadius ? Point(shape.BottomRadius)
                : Add(Point(shape.BottomRadius), Mul(Sub(Point(shape.TopRadius), Point(shape.BottomRadius)), Div(depth, Point(shape.Height))));
            if (!(radius.Lo >= MinimumInsetRadius) || radius.Hi > MaximumRadius)
            { LastRefusal = Refusal.Tip;return false; }
            section = i;return true;
        }
        LastRefusal = Refusal.Section;return false;
    }

    double SphereBulge(Site a, Site b)
    {
        if (_profile.Sections[_section].Kind != CarvingCutterSectionKind.Hemisphere || a.Z == b.Z) return 0;
        Metrics.SphereBulges++;
        Site low = a.Z > b.Z ? a : b, high = a.Z > b.Z ? b : a;
        var radialDifference = Sub(high.Radius, low.Radius);
        radialDifference = Enclose(Math.Max(0, radialDifference.Lo), radialDifference.Hi);
        Interval slope = Div(radialDifference, Sub(high.Depth, low.Depth));
        // Exact concave secant maximum: R/(sqrt(1+m*m)+m) - r_a + m*h_a.
        // Interval evaluation encloses the real maximum without selecting a rounded stationary pose.
        var support = Div(Point(_profile.Sections[_section].TopRadius), Add(Sqrt(Add(Point(1), Square(slope))), slope));
        return Math.Max(0, Add(Sub(support, low.Radius), Mul(slope, low.Depth)).Hi);
    }

    bool Append(IReadOnlyList<Point3D> points, int first, Func<int, bool> active, CancellationToken token)
    {
        int index = first + _count;Barrier(token);
        if (_count != 0)
        {
            bool allowed = active(index);Barrier(token);
            if (!allowed) { LastRefusal = Refusal.Inactive;return false; }
        }
        if (points.Count != _sourceCount || index >= points.Count) { LastRefusal = Refusal.SourceChanged;return false; }
        var p = points[index];if (!Read(p, out var depth, out var radius, out int section)) return false;
        if (_count == 0) { _section = section;_sites[_count++] = new(p.X, p.Y, p.Z, depth, radius, Point(0), 0);return true; }
        if (section != _section) { LastRefusal = Refusal.Section;return false; }
        var prior = _sites[_count - 1];
        if (p.X == prior.X && p.Y == prior.Y) { LastRefusal = Refusal.ZeroXY;return false; }
        var length = Distance(Sub(Point(p.X), Point(prior.X)), Sub(Point(p.Y), Point(prior.Y)));
        if (!(length.Lo > 0)) { LastRefusal = Refusal.ZeroXY;return false; }
        var next = new Site(p.X, p.Y, p.Z, depth, radius, Add(prior.Arc, length), 0);
        next = next with { Bulge = SphereBulge(prior, next) };_sites[_count++] = next;return true;
    }

    bool Fit(int first, int length, double budget, CancellationToken token, out Certificate certificate)
    {
        certificate = default;Metrics.CandidateFits++;var a = _sites[0];var b = _sites[length];
        if (a.X == b.X && a.Y == b.Y) { LastRefusal = Refusal.ZeroXY;return false; }
        double reduction = 0, maximumBulge = 0, maximumCenterError = 0;
        double minimumRadius = double.PositiveInfinity, maximumRadius = 0;
        for (int i = 0; i <= length; i++)
        {
            Barrier(token);Metrics.FitPoints++;var p = _sites[i];double t;
            if (i == 0) t = 0;else if (i == length) t = 1;
            else
            {
                var arcParameter = Div(p.Arc, b.Arc);
                double low = Math.Max(0, arcParameter.Lo), high = Math.Min(1, arcParameter.Hi);
                t = low + (high - low) / 2;
                if (!(t > _parameters[i - 1] && t < 1 && low <= t && t <= high))
                { LastRefusal = Refusal.Parameter;return false; }
            }
            _parameters[i] = t;
            double error = i == 0 || i == length ? 0
                : Distance(Sub(Point(p.X), Lerp(a.X, b.X, t)), Sub(Point(p.Y), Lerp(a.Y, b.Y, t))).Hi;
            _errors[i] = error;
            maximumCenterError = Math.Max(maximumCenterError, error);
            minimumRadius = Math.Min(minimumRadius, p.Radius.Lo);maximumRadius = Math.Max(maximumRadius, p.Radius.Hi);
            reduction = Math.Max(reduction, Sub(Add(Point(error), Lerp(a.Radius.Lo, b.Radius.Lo, t)), Point(p.Radius.Lo)).Hi);
            maximumBulge = Math.Max(maximumBulge, p.Bulge);
        }
        reduction = Math.Max(0, reduction);
        double r0 = Sub(Point(a.Radius.Lo), Point(reduction)).Lo, r1 = Sub(Point(b.Radius.Lo), Point(reduction)).Lo;
        if (!(r0 >= MinimumInsetRadius && r1 >= MinimumInsetRadius)) { LastRefusal = Refusal.Tip;return false; }
        double maximumLoss = 0, previousLoss = 0;
        for (int i = 0; i <= length; i++)
        {
            Barrier(token);var p = _sites[i];
            double loss = Add(Point(_errors[i]), Sub(Point(p.Radius.Hi), Lerp(r0, r1, _parameters[i]))).Hi;
            if (i != 0) maximumLoss = Math.Max(maximumLoss, Add(Point(Math.Max(previousLoss, loss)), Point(p.Bulge)).Hi);
            previousLoss = loss;
            if (!(maximumLoss <= budget))
            { Metrics.LastRejectedLoss = maximumLoss;Metrics.LastRejectedCenterError = maximumCenterError;Metrics.LastRejectedReduction = reduction;LastRefusal = Refusal.Loss;return false; }
        }
        double endpointReduction = Math.Max(Sub(Point(a.Radius.Lo), Point(r0)).Hi, Sub(Point(b.Radius.Lo), Point(r1)).Hi);
        certificate = new(first, first + length, _section, r0, r1, reduction, endpointReduction, maximumCenterError,
            maximumLoss, maximumBulge, Sub(Point(maximumRadius), Point(minimumRadius)).Hi, b.Arc.Lo, b.Arc.Hi);
        LastRefusal = Refusal.None;return true;
    }

    internal bool TryFind(IReadOnlyList<Point3D> points, int first, Func<int, bool> active, double interpolationBudget,
        CancellationToken token, out Certificate certificate)
    {
        ArgumentNullException.ThrowIfNull(points);ArgumentNullException.ThrowIfNull(active);Barrier(token);
        Metrics.Requests++;certificate = default;_count = 0;_sourceCount = points.Count;LastRefusal = Refusal.None;
        if (!_eligible) { LastRefusal = Refusal.Profile;return Refused(token); }
        if (first < 0 || first >= points.Count - MinimumChords || !double.IsFinite(interpolationBudget) || !(interpolationBudget > 0))
        { LastRefusal = Refusal.Short;return Refused(token); }
        bool accepted = false;int maximum = Math.Min(MaximumChords, points.Count - first - 1), goal = MinimumChords;
        try
        {
            if (!Append(points, first, active, token)) return Refused(token);
            while (true)
            {
                bool stopped = false;
                while (_count <= goal) if (!Append(points, first, active, token)) { stopped = true;break; }
                int length = _count - 1;
                if (length < MinimumChords) return Refused(token);
                Refusal boundary = LastRefusal;
                if (!Fit(first, length, interpolationBudget, token, out var proposed)) break;
                certificate = proposed;accepted = true;
                if (stopped || length == maximum) { LastRefusal = boundary;break; }
                goal = Math.Min(maximum, 2 * goal);
            }
        }
        catch (ArithmeticRefusal) { LastRefusal = Refusal.Arithmetic; }
        Barrier(token);
        if (!accepted) return Refused(token);
        // A callback cannot substitute a different path after the bound captured its sites.
        for (int i = 0; i <= certificate.Last - first; i++)
        {
            if (points.Count != _sourceCount) { LastRefusal = Refusal.SourceChanged;certificate = default;return Refused(token); }
            Barrier(token);var p = points[first + i];var captured = _sites[i];
            if (BitConverter.DoubleToInt64Bits(p.X) != BitConverter.DoubleToInt64Bits(captured.X)
                || BitConverter.DoubleToInt64Bits(p.Y) != BitConverter.DoubleToInt64Bits(captured.Y)
                || BitConverter.DoubleToInt64Bits(p.Z) != BitConverter.DoubleToInt64Bits(captured.Z))
            { LastRefusal = Refusal.SourceChanged;certificate = default;return Refused(token); }
        }
        Metrics.Accepted++;Metrics.MaximumAcceptedLoss = Math.Max(Metrics.MaximumAcceptedLoss, certificate.MaximumLoss);
        Metrics.MaximumAcceptedReduction = Math.Max(Metrics.MaximumAcceptedReduction, certificate.Reduction);
        Metrics.MaximumAcceptedRadiusSpread = Math.Max(Metrics.MaximumAcceptedRadiusSpread, certificate.RadiusSpread);
        if (LastRefusal != Refusal.None) Metrics.Refusals[(int)LastRefusal]++;
        Barrier(token);return true;
    }
    bool Refused(CancellationToken token) { Metrics.Refusals[(int)LastRefusal]++;Barrier(token);return false; }

    // Test-only copied parameters of the last accepted prefix; never used for emission.
    internal double[] CopyParameters(Certificate certificate)
    {
        var result = new double[certificate.Last - certificate.First + 1];
        // A later refused doubling attempt may have overwritten the scratch parameters.
        var last = _sites[result.Length - 1];
        for (int i = 0; i < result.Length; i++)
        {
            if (i == 0) result[i] = 0;else if (i == result.Length - 1) result[i] = 1;
            else {var t = Div(_sites[i].Arc, last.Arc);double low = Math.Max(0, t.Lo), high = Math.Min(1, t.Hi);result[i] = low + (high - low) / 2;}
        }
        return result;
    }
}
