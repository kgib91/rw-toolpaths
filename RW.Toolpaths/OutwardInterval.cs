namespace RW.Toolpaths;

/// <summary>Outward-rounded enclosures of elementary binary64 operations.
/// Finite-input uncertainty is retained through every operation; division
/// across zero and overflow remain unresolved and cannot certify geometry.</summary>
internal readonly record struct OutwardInterval(double Low, double High)
{
    internal bool IsFinite => double.IsFinite(Low) && double.IsFinite(High) && Low <= High;
    internal double Midpoint => Low + (High - Low) / 2;
    internal static OutwardInterval Exact(double value) => new(value, value);
    internal static OutwardInterval Integer(Int128 value)
    {
        if (value == 0) return Exact(0);
        double rounded = (double)value;
        return new(Math.BitDecrement(rounded), Math.BitIncrement(rounded));
    }
    public static OutwardInterval operator +(OutwardInterval a, OutwardInterval b)
        => new(Math.BitDecrement(a.Low + b.Low), Math.BitIncrement(a.High + b.High));
    public static OutwardInterval operator -(OutwardInterval a, OutwardInterval b)
        => new(Math.BitDecrement(a.Low - b.High), Math.BitIncrement(a.High - b.Low));
    public static OutwardInterval operator *(OutwardInterval a, OutwardInterval b)
    {
        if (a.Low == 0 && a.High == 0 || b.Low == 0 && b.High == 0) return Exact(0);
        double p0 = a.Low * b.Low, p1 = a.Low * b.High, p2 = a.High * b.Low, p3 = a.High * b.High;
        return new(Math.BitDecrement(Math.Min(Math.Min(p0, p1), Math.Min(p2, p3))),
            Math.BitIncrement(Math.Max(Math.Max(p0, p1), Math.Max(p2, p3))));
    }
    public static OutwardInterval operator /(OutwardInterval a, OutwardInterval b)
    {
        if (b.Low <= 0 && b.High >= 0) return new(double.NegativeInfinity, double.PositiveInfinity);
        double p0 = a.Low / b.Low, p1 = a.Low / b.High, p2 = a.High / b.Low, p3 = a.High / b.High;
        return new(Math.BitDecrement(Math.Min(Math.Min(p0, p1), Math.Min(p2, p3))),
            Math.BitIncrement(Math.Max(Math.Max(p0, p1), Math.Max(p2, p3))));
    }
    internal OutwardInterval Square()
    {
        double maximum = Math.Max(Math.Abs(Low), Math.Abs(High));
        double minimum = Low <= 0 && High >= 0 ? 0 : Math.Min(Math.Abs(Low), Math.Abs(High));
        return new(minimum == 0 ? 0 : Math.Max(0, Math.BitDecrement(minimum * minimum)),
            Math.BitIncrement(maximum * maximum));
    }
    internal double SqrtUpper() => High < 0 ? double.NaN : Math.BitIncrement(Math.Sqrt(High));
}
