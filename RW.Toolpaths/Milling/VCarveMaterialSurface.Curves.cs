namespace RW.Toolpaths.Milling;

internal sealed partial class VCarveMaterialSurface
{
    private VCarveNativeContainment[]? nativeLayerGuards;
    private bool LayerCurveLegal(ToolMove move)
    {
        // Each immutable FreeRegion is already an inner proof of prior posted
        // stock removal union the target, clipped to the physical Part. Its
        // extrusion from layer top bounds every deeper cross-section because
        // the physical cutter radius is nondecreasing upward from the tip.
        nativeLayerGuards ??= layers.Select(l=>new VCarveNativeContainment(l.FreeRegion,profile,token)).ToArray();
        for(int i=0;i<layers.Count;i++)
        {
            var relative=VCarveNativeCurveMath.TranslateZ(move,layers[i].TopDepth);
            if(!VCarveNativeCutterProof.Certified(relative,nativeLayerGuards[i],profile,out _,0,1))return false;
        }
        return true;
    }
    internal bool IsLegalNativeChord(Point3D a,Point3D b)=>Legal(a,b,postingReserve:true);
    internal bool IsLegalNativeCurve(ToolMove move)
    {
        if(move.Kind==ToolMoveKind.Line)return IsLegalNativeChord(move.Start,move.End);
        if(move.Kind!=ToolMoveKind.CubicBezier)return false;
        var range=VCarveNativeCurveMath.ZRange(move);
        if(!double.IsFinite(range.Min+range.Max)||range.Min < -maximumDepth)return false;
        bool CellEnvelope(ToolMove piece)
        {
            if(Math.Min(Math.Min(piece.Start.Z,piece.End.Z),Math.Min(piece.Control1.Z,piece.Control2.Z))>=0)return true;
            var d0=VCarveNativeCurveMath.Second(piece,0);var d1=VCarveNativeCurveMath.Second(piece,1);
            double xy=Math.Max(Math.Sqrt(d0.X*d0.X+d0.Y*d0.Y),Math.Sqrt(d1.X*d1.X+d1.Y*d1.Y))/8;
            double z=Math.Max(Math.Abs(d0.Z),Math.Abs(d1.Z))/8;
            // The entire cubic centre path lies inside this deepened chord's
            // XY tube. The radially nondecreasing complete cutter makes its
            // inflated sweep an outer enclosure of the actual curve sweep.
            // The existing immutable-stock cell engine proves every touched
            // cell, retaining its posting shell, Part and prior-stock guards.
            var a=piece.Start with{Z=piece.Start.Z-z};var b=piece.End with{Z=piece.End.Z-z};
            return projectedStock.ProveNativeEnvelope(a,b,xy);
        }
        return VCarveNativeIntervalProof.Certified(move,LayerCurveLegal,CellEnvelope,token);
    }
}
