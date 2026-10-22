namespace RW.Toolpaths.Milling;

// One explicit analysis-plane policy. Original physical command points,
// artwork/channel/floor/grade guards and returned/posting paths remain at0.
internal static class VCarveMaterialLayerGoal
{
    internal const string SelectionEnvironment="RW_TOOLPATHS_VCARVE_MATERIAL_LAYER_GOAL";
    internal const double LayerDepthMm=.001;
    internal static double AnalysisSurface(bool enabled)=>enabled?-LayerDepthMm:0;

    // Input is ALREADY decoded physical F3 geometry. Q expects stock-relative
    // zero-plane coordinates, so translate its view without F3 re-rounding.
    // Path ordinals, points, XY, tags and visit identities remain aligned.
    internal static List<TaggedToolpath> ForCertificate(List<TaggedToolpath> physicalCommands,
        double analysisSurface,CancellationToken token=default)
    {
        token.ThrowIfCancellationRequested();
        if(analysisSurface==0)return physicalCommands;
        if(analysisSurface!=-LayerDepthMm)throw new ArgumentOutOfRangeException(nameof(analysisSurface));
        var result=new List<TaggedToolpath>(physicalCommands.Count);
        foreach(var path in physicalCommands)
        {
            token.ThrowIfCancellationRequested();var points=new Point3D[path.Points.Count];
            for(int i=0;i<points.Length;i++)
            {
                if((i&255)==0)token.ThrowIfCancellationRequested();var p=path.Points[i];
                if(!double.IsFinite(p.X)||!double.IsFinite(p.Y)||!double.IsFinite(p.Z))
                    throw new InvalidOperationException("Nonfinite physical point cannot become a material-goal Q command.");
                points[i]=new(p.X,p.Y,p.Z-analysisSurface);
            }
            result.Add(new(points,Array.Empty<ToolpathSpan>(),path.RegionIndex,path.Category,path.DepthPassIndex,
                path.VisitIdentity,detectCurves:false));
        }
        token.ThrowIfCancellationRequested();return result;
    }
}
