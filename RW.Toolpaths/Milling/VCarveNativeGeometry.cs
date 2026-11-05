namespace RW.Toolpaths.Milling;

public static partial class VCarveNativeCurvePlanning
{
    /// <summary>Conservative bounds on a native primitive's physical path length.</summary>
    public static (double Lower,double Upper) MeasureLength(ToolMove move)=>VCarveNativeCurveMath.Length(move);
    /// <summary>Algebraic coordinate translation; never samples or retessellates.</summary>
    public static ToolMove TranslateZ(ToolMove move,double offset)=>VCarveNativeCurveMath.TranslateZ(move,offset);
}
