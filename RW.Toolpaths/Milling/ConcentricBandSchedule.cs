namespace RW.Toolpaths.Milling;

/// <summary>Shared spacing for an exact authored terminal front.</summary>
internal static class ConcentricBandSchedule
{
    internal readonly record struct RampBand(double Grade,double SurfacePitch,double Pitch,double First,double Reach,int Levels);
    internal static RampBand Ramp(VCarveCutterProfile profile,double depth,VCarveOptions options)
    {
        double grade=Math.Tan(options.SmoothRampAngleDegrees*Math.PI/180);
        double diameter=2*profile.PhysicalRadiusAtDepth(depth);
        double surface=profile.RoundedTip?profile.NoseLimitedPitch(diameter,options.StepOver):.05*diameter;
        double pitch=surface/Math.Sqrt(1+grade*grade);
        double first=profile.PhysicalRadiusAtDepth(ToolpathCommandRounding.CommandIncrement)+2*ToolpathCommandRounding.CommandIncrement;
        double reach=profile.RampFillWidth(depth,options.SmoothRampAngleDegrees);
        if(reach>first)pitch=FitPitch(reach-first,pitch,RingOffsetEngine.AbsoluteRingLimit-1);
        int levels=reach>=first&&pitch>0&&double.IsFinite((reach-first)/pitch)
            ? (int)Math.Min(RingOffsetEngine.AbsoluteRingLimit,Math.Floor((reach-first)/pitch+1e-10)+1):0;
        return new(grade,surface,pitch,first,reach,levels);
    }

    internal static double FitPitch(double additionalInset,double maximumPitch,int maximumIntervals=1000)
    {
        double intervals=Math.Ceiling(additionalInset/maximumPitch);
        return intervals<=maximumIntervals?additionalInset/Math.Max(1,intervals):maximumPitch;
    }
}
