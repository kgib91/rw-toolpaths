namespace RW.Toolpaths.Milling;

/// <summary>Shared spacing for an exact authored terminal front.</summary>
internal static class ConcentricBandSchedule
{
    internal static double FitPitch(double additionalInset,double maximumPitch,int maximumIntervals=1000)
    {
        double intervals=Math.Ceiling(additionalInset/maximumPitch);
        return intervals<=maximumIntervals?additionalInset/Math.Max(1,intervals):maximumPitch;
    }
}
