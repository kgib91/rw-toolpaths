namespace RW.Toolpaths.Milling;

internal static class VCarveCommandRounding
{
    internal const double CommandIncrement = ToolpathCommandRounding.CommandIncrement;
    // The accepted binary32 significand has at most 24 bits. Multiplication
    // by 1000 (125 * 2^3) needs at most 31 significant binary bits and is
    // exact in binary64. The bounded result is an integer <= 1e9, so the
    // final division rounds the same exact decimal rational as Parse(F3).
    // Fixed-point formatting uses nearest/even, including negative zero.
    internal static double Round(float value)
        => ToolpathCommandRounding.Round(value);

    internal static double Literal(float value)
        => ToolpathCommandRounding.Literal(value);
}
