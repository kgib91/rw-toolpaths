namespace RW.Toolpaths.Milling;

// Closed child intervals share their split endpoint. Both children must earn
// a complete sweep certificate; poses or omitted parameter gaps grant nothing.
internal static class VCarveNativeIntervalProof
{
    internal static bool Certified(ToolMove move,Func<ToolMove,bool> layer,Func<ToolMove,bool> cells,
        CancellationToken token=default,int maximumDepth=14,int maximumIntervals=65536)
    {
        if(maximumDepth<0||maximumIntervals<1)throw new ArgumentOutOfRangeException();
        int visited=0;
        bool Visit(ToolMove piece,int depth)
        {
            token.ThrowIfCancellationRequested();if(++visited>maximumIntervals)return false;
            if(layer(piece)||cells(piece))return true;
            if(depth>=maximumDepth||piece.Kind!=ToolMoveKind.CubicBezier)return false;
            var halves=VCarveNativeCurveMath.Split(piece);
            return Visit(halves.Left,depth+1)&&Visit(halves.Right,depth+1);
        }
        return Visit(move,0);
    }
}
