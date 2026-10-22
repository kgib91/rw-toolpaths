using System.Globalization;

namespace RW.Toolpaths;

/// <summary>The binary32-to-F3 millimetre command used by machining geometry,
/// feed planning and machine output. Finite ordinary coordinates need no strings.</summary>
public static class ToolpathCommandRounding
{
    public const double CommandIncrement=.001;
    // Command shape is independent of chord length. A small horizontal step
    // can cross an F3 cell and must never disappear when a ramp is subdivided.
    public static bool IsVerticalOnly(float dx,float dy,float dz)
        =>float.IsFinite(dy)&&dy!=0&&dx==0&&dz==0;
    public static double Round(float value)
        => float.IsFinite(value)&&Math.Abs(value)<=1_000_000f
            ? Math.Round((double)value*1000,MidpointRounding.ToEven)/1000d
            : Literal(value);
    internal static double Literal(float value)
        => double.Parse(value.ToString("F3",CultureInfo.InvariantCulture),CultureInfo.InvariantCulture);
}
