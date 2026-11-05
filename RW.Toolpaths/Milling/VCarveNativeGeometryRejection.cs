namespace RW.Toolpaths.Milling;

// Rejection only: every cubic lies in the union of the convex hulls of its
// de Casteljau subcurves. A source vertex farther than the allowance from ALL
// their enclosing boxes cannot have any continuous correspondence to the curve.
// Passing this filter grants no fit, ordering, length or cutter certificate.
internal static class VCarveNativeGeometryRejection
{
    private readonly record struct Box(Point3D Low,Point3D High)
    {
        internal double DistanceSquared(Point3D p)
        {
            double x=Math.Max(0,Math.Max(Low.X-p.X,p.X-High.X));
            double y=Math.Max(0,Math.Max(Low.Y-p.Y,p.Y-High.Y));
            double z=Math.Max(0,Math.Max(Low.Z-p.Z,p.Z-High.Z));
            return x*x+y*y+z*z;
        }
    }

    internal static bool Excluded(ToolMove move,IReadOnlyList<Point3D> source,int first,int last,double allowance)
    {
        Point3D[] controls=[move.Start,move.Control1,move.Control2,move.End];
        double scale=controls.Max(p=>Math.Max(Math.Abs(p.X),Math.Max(Math.Abs(p.Y),Math.Abs(p.Z))));
        if(!double.IsFinite(scale)||!double.IsFinite(allowance)||allowance<0)return false;
        // Outward padding is deliberately much larger than the rounding error
        // of four midpoint splits; it can only retain extra candidates.
        double padding=1e-9*Math.Max(1,scale),squared=Math.Pow(allowance+padding,2);
        var boxes=new List<Box>(16);
        void Cover(Point3D a,Point3D b,Point3D c,Point3D d,int level)
        {
            if(level==4)
            {
                boxes.Add(new(new(Math.Min(Math.Min(a.X,b.X),Math.Min(c.X,d.X)),Math.Min(Math.Min(a.Y,b.Y),Math.Min(c.Y,d.Y)),Math.Min(Math.Min(a.Z,b.Z),Math.Min(c.Z,d.Z))),
                    new(Math.Max(Math.Max(a.X,b.X),Math.Max(c.X,d.X)),Math.Max(Math.Max(a.Y,b.Y),Math.Max(c.Y,d.Y)),Math.Max(Math.Max(a.Z,b.Z),Math.Max(c.Z,d.Z)))));
                return;
            }
            var ab=VCarveNativeCurveMath.Lerp(a,b,.5);var bc=VCarveNativeCurveMath.Lerp(b,c,.5);var cd=VCarveNativeCurveMath.Lerp(c,d,.5);
            var abc=VCarveNativeCurveMath.Lerp(ab,bc,.5);var bcd=VCarveNativeCurveMath.Lerp(bc,cd,.5);var mid=VCarveNativeCurveMath.Lerp(abc,bcd,.5);
            Cover(a,ab,abc,mid,level+1);Cover(mid,bcd,cd,d,level+1);
        }
        Cover(controls[0],controls[1],controls[2],controls[3],0);
        for(int i=first;i<=last;i++)
        {
            bool possible=false;
            foreach(var box in boxes)if(box.DistanceSquared(source[i])<=squared){possible=true;break;}
            if(!possible)return true;
        }
        return false;
    }
}
