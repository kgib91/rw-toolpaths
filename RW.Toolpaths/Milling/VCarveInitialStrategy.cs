namespace RW.Toolpaths.Milling;

// One transient operation request. This selects NOM construction only; every
// original floor/artwork/grade and command audit remains owned by generation.
public sealed class VCarveInitialStrategy
{
    public const string Version = "bounded-smooth-rest-finishing-v3";
    private static readonly string[] Names = {
        "RW_TOOLPATHS_VCARVE_RAW_INITIAL_INSET_NOMINATION",
        "RW_TOOLPATHS_VCARVE_INITIAL_FRONT_CONDITIONING",
        "RW_TOOLPATHS_VCARVE_INITIAL_STOCK_COHORT",
        "RW_TOOLPATHS_VCARVE_SUPPLEMENTAL_INITIAL_BAND",
        "RW_TOOLPATHS_VCARVE_PARALLEL_SWEEP_UNIONS" };
    private readonly bool _smooth, _interior;
    private readonly double _angle, _width, _depth, _stepOver, _restWidth;
    private readonly VCarveCutterProfile? _profile;
    public bool RawInitialInset => (Mask & 1) != 0;
    public bool InitialFrontConditioning => (Mask & 2) != 0;
    public bool InitialStockCohort => (Mask & 4) != 0;
    public bool SupplementalInitialBand => (Mask & 8) != 0;
    public bool ParallelSweepUnions => (Mask & 16) != 0;
    public int Mask { get; }
    public bool AutomaticEligible { get; }
    public string Origin { get; }
    public string CacheSignature => Mask == 0 ? string.Empty : $"|{Version}:{Mask:X2}";
    private VCarveInitialStrategy(VCarveOptions options,VCarveCutterProfile? profile,double depth,
        int mask,bool eligible,string origin)
    {
        _smooth=options.SmoothVCarving;_interior=options.IncludeInteriorFill;
        _angle=options.SmoothRampAngleDegrees;_width=options.EdgeFillWidth;_depth=depth;_profile=profile;_stepOver=options.StepOver;
        _restWidth=options.RoughingRestWidth;
        Mask=mask;AutomaticEligible=eligible;Origin=origin;
    }
    public static VCarveInitialStrategy Capture(VCarveOptions options,VCarveCutterProfile? profile,double effectiveDepth)
    {
        ArgumentNullException.ThrowIfNull(options);
        bool eligible=options.SmoothVCarving&&!options.IncludeInteriorFill
            &&double.IsFinite(options.SmoothRampAngleDegrees)&&options.SmoothRampAngleDegrees>0&&options.SmoothRampAngleDegrees<90
            &&double.IsFinite(options.StepOver)&&options.StepOver>0&&options.StepOver<=1
            &&double.IsFinite(options.EdgeFillWidth)&&options.EdgeFillWidth>=0
            &&double.IsFinite(options.RoughingRestWidth)&&options.RoughingRestWidth>=0
            &&Math.Max(options.EdgeFillWidth,options.RoughingRestWidth)>0&&Math.Max(options.EdgeFillWidth,options.RoughingRestWidth)<=100
            &&profile is not null&&profile.Sections.Count is>0 and<=64
            &&double.IsFinite(effectiveDepth)&&effectiveDepth>1e-6&&effectiveDepth<=profile.MaximumDepth
            &&double.IsFinite(profile.RadiusAtDepth(effectiveDepth))&&profile.RadiusAtDepth(effectiveDepth)>0
            &&profile.RadiusAtDepth(effectiveDepth)<=100;
        int explicitOnes=0;bool original=false,invalid=false;
        for(int i=0;i<Names.Length;i++)
        {
            string? value=Environment.GetEnvironmentVariable(Names[i]);
            if(value=="0")original=true;
            else if(value=="1")explicitOnes|=1<<i;
            else if(!string.IsNullOrEmpty(value))invalid=true;
        }
        // Explicit zero is atomic: never ship a half-enabled automatic policy.
        int mask=!options.SmoothVCarving||original||invalid?0:eligible?31:explicitOnes;
        string origin=original?"explicit-zero-original":invalid?"invalid-environment-original":eligible?
            explicitOnes==0?"automatic-eligible":"automatic-eligible-with-literal-one":
            explicitOnes!=0?"literal-one-diagnostic":"ineligible-original";
        return new(options,profile,effectiveDepth,mask,eligible,origin);
    }
    internal bool Matches(VCarveOptions options,VCarveCutterProfile profile,double depth)
    {
        if(_smooth!=options.SmoothVCarving||_interior!=options.IncludeInteriorFill
            ||Bits(_angle)!=Bits(options.SmoothRampAngleDegrees)||Bits(_width)!=Bits(options.EdgeFillWidth)
            ||Bits(_depth)!=Bits(depth)||Bits(_stepOver)!=Bits(options.StepOver)||Bits(_restWidth)!=Bits(options.RoughingRestWidth)||_profile is null||Bits(_profile.MaximumDepth)!=Bits(profile.MaximumDepth)
            ||_profile.Sections.Count!=profile.Sections.Count)return false;
        for(int i=0;i<profile.Sections.Count;i++)
        {
            var a=_profile.Sections[i];var b=profile.Sections[i];
            if(a.Kind!=b.Kind||Bits(a.BottomRadius)!=Bits(b.BottomRadius)||Bits(a.TopRadius)!=Bits(b.TopRadius)
                ||Bits(a.Height)!=Bits(b.Height))return false;
        }
        return true;
    }
    private static long Bits(double x)=>BitConverter.DoubleToInt64Bits(x);
}
