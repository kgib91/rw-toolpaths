using Clipper2Lib;
using System.Numerics;

namespace RW.Toolpaths.Milling.Certification;

internal sealed class TriangulationBudgetException : Exception { }

/// <summary>Exact predicates used by the private untrusted decomposition aid.</summary>
internal static class InternalClipper
{
    internal static int InCircleSign(Point64 a, Point64 b, Point64 c, Point64 origin)
    {
        Int128 ax = (Int128)a.X - origin.X, ay = (Int128)a.Y - origin.Y;
        Int128 bx = (Int128)b.X - origin.X, by = (Int128)b.Y - origin.Y;
        Int128 cx = (Int128)c.X - origin.X, cy = (Int128)c.Y - origin.Y;
        try
        {
            Int128 SquareSum(Int128 x, Int128 y) => CheckedInt128.Add(CheckedInt128.Multiply(x, x), CheckedInt128.Multiply(y, y));
            Int128 Cross(Int128 x, Int128 y, Int128 u, Int128 v)
                => CheckedInt128.Subtract(CheckedInt128.Multiply(x, v), CheckedInt128.Multiply(y, u));
            Int128 value = CheckedInt128.Add(CheckedInt128.Subtract(
                CheckedInt128.Multiply(SquareSum(ax, ay), Cross(bx, by, cx, cy)),
                CheckedInt128.Multiply(SquareSum(bx, by), Cross(ax, ay, cx, cy))),
                CheckedInt128.Multiply(SquareSum(cx, cy), Cross(ax, ay, bx, by)));
            return value < 0 ? -1 : value > 0 ? 1 : 0;
        }
        catch (OverflowException)
        {
            BigInteger x = (BigInteger)ax, y = (BigInteger)ay, u = (BigInteger)bx, v = (BigInteger)by,
                p = (BigInteger)cx, q = (BigInteger)cy;
            BigInteger value = (x * x + y * y) * (u * q - v * p)
                - (u * u + v * v) * (x * q - y * p) + (p * p + q * q) * (x * v - y * u);
            return value.Sign;
        }
    }

    internal static int CrossProductSign(Point64 first, Point64 second, Point64 third)
    {
        try
        {
            Int128 value = CheckedInt128.Subtract(
                CheckedInt128.Multiply((Int128)second.X - first.X, (Int128)third.Y - first.Y),
                CheckedInt128.Multiply((Int128)second.Y - first.Y, (Int128)third.X - first.X));
            return value < 0 ? -1 : value > 0 ? 1 : 0;
        }
        catch (OverflowException)
        {
            BigInteger value = ((BigInteger)second.X - first.X) * ((BigInteger)third.Y - first.Y)
                - ((BigInteger)second.Y - first.Y) * ((BigInteger)third.X - first.X);
            return value.Sign;
        }
    }
}

/// <summary>C#10-compatible checked Int128 arithmetic; overflow never wraps into proof.</summary>
internal static class CheckedInt128
{
    internal static Int128 Multiply(Int128 first, Int128 second)
    {
        if (first == 0 || second == 0) return 0;
        // Two signed 64-bit operands have a product in [-2^126, 2^126].
        // This range proof avoids division in the common exact capsule path;
        // larger dyadic operands retain the checked general path below.
        if (first >= long.MinValue && first <= long.MaxValue
            && second >= long.MinValue && second <= long.MaxValue)
            return first * second;
        Int128 value = first * second;
        if (value / first != second) throw new OverflowException();
        return value;
    }

    internal static Int128 Add(Int128 first, Int128 second)
    {
        Int128 value = first + second;
        if ((first > 0 && second > 0 && value < 0) || (first < 0 && second < 0 && value >= 0))
            throw new OverflowException();
        return value;
    }

    internal static Int128 Subtract(Int128 first, Int128 second)
    {
        Int128 value = first - second;
        if ((first >= 0 && second < 0 && value < 0) || (first < 0 && second > 0 && value >= 0))
            throw new OverflowException();
        return value;
    }
}
