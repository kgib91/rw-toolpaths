using System.Runtime.CompilerServices;
using Clipper2Lib;

namespace RW.Toolpaths.Milling;

// Nomination-free geometry work. The caller has already performed every
// physical radius/reserve/subdivision decision in its original arithmetic.
internal sealed class VCarvePhysicalHullRecipe
{
    internal const int MaximumCirclePoints=8192;
    internal const long MaximumWorkerScratchBytes=1024*1024;
    internal sealed class Circle
    {
        internal readonly (double X,double Y)[] Points;
        internal readonly bool Safe;
        // PhysicalSweep owns these arrays and never mutates them. No caller's
        // mutable path geometry or proof/query owner is retained here.
        internal Circle((double X,double Y)[] points)
        {
            Points=points;
            Safe=points.Length is >=8 and <=MaximumCirclePoints&&(points.Length&(points.Length-1))==0
                &&points.All(p=>double.IsFinite(p.X)&&double.IsFinite(p.Y)&&Math.Abs(p.X)<=1&&Math.Abs(p.Y)<=1);
        }
    }
    internal readonly record struct Row(Point3D Center,double Radius,Point64 Point,bool IsPoint)
    {
        internal static Row Disc(Point3D center,double radius)=>new(center,radius,default,false);
        internal static Row Vertex(Point64 point)=>new(default,0,point,true);
    }
    private readonly Circle circle;
    private readonly Row first,second;
    private readonly int rows;
    private readonly bool certified;
    internal int MaximumPoints=>rows==0?0:Points(first)+(rows==2?Points(second):0);
    internal long ExpandedBytes=>64L+(long)Unsafe.SizeOf<Point64>()*(MaximumPoints+1);
    internal VCarvePhysicalHullRecipe(Circle circle,Row first,Row second,int rows)
    {this.circle=circle;this.first=first;this.second=second;this.rows=rows;certified=Certify();}

    internal bool CertifiedNondegenerate=>certified;
    private bool Certify()
    {
        if(rows is <1 or >2||!circle.Safe||!Safe(first)||rows==2&&!Safe(second))return false;
        return Certificate(first)||rows==2&&Certificate(second);
    }
    private bool Safe(Row row)=>row.IsPoint
        ?Math.Abs((double)row.Point.X)<=1L<<50&&Math.Abs((double)row.Point.Y)<=1L<<50
        :double.IsFinite(row.Center.X)&&double.IsFinite(row.Center.Y)&&double.IsFinite(row.Radius)
            &&Math.Abs(row.Center.X)<=10_000&&Math.Abs(row.Center.Y)<=10_000&&row.Radius is >=0 and <=100;
    private bool Certificate(Row row)
    {
        if(row.IsPoint)return false;
        // These are three ACTUAL input vertices, using the exact AddDisc
        // multiply/add/scale/round expression. All input coordinates above are
        // bounded, so both old long subtraction and Int128 products are exact.
        Point64 Point(int i)
        {
            var unit=circle.Points[i];
            return new((long)Math.Round((row.Center.X+row.Radius*unit.X)*PathUtils.Scale),
                (long)Math.Round((row.Center.Y+row.Radius*unit.Y)*PathUtils.Scale));
        }
        var a=Point(0);var b=Point(circle.Points.Length/4);var c=Point(circle.Points.Length/2);
        return (Int128)(b.X-a.X)*(c.Y-a.Y)-(Int128)(b.Y-a.Y)*(c.X-a.X)!=0;
    }
    private int Points(Row row)=>row.IsPoint?1:circle.Points.Length;
    internal Path64 Materialize(VCarveOrderedDiscHull hull)
    {
        if(rows is <0 or >2)throw new InvalidOperationException("A physical hull has at most two rows.");
        hull.BeginCircle(circle.Points);
        if(rows>0)Add(first);if(rows>1)Add(second);
        return hull.FinishHull();
        void Add(Row row)
        {if(row.IsPoint)hull.AddPoint(row.Point);else hull.AddDisc(row.Center,row.Radius);}
    }
}

internal sealed class VCarvePhysicalHullBatch
{
    private readonly VCarvePhysicalHullRecipe[] recipes;
    internal int Count=>recipes.Length;
    internal long EstimatedBytes {get;}
    internal VCarvePhysicalHullBatch(IReadOnlyList<VCarvePhysicalHullRecipe> source)
    {
        if(source.Count is <1 or >32)throw new ArgumentOutOfRangeException(nameof(source));
        recipes=source.ToArray();
        if(recipes.Any(r=>!r.CertifiedNondegenerate))throw new ArgumentException("Deferred hulls require actual noncollinear integer input vertices.",nameof(source));
        // Charge recipe objects/array, the worst-case expanded Path64 buffers,
        // and one active worker's bounded retained row/order scratch. Persistent
        // worker high-water scratch has a separate fixed worker-count bound.
        EstimatedBytes=checked(VCarvePhysicalHullRecipe.MaximumWorkerScratchBytes+64+8L*recipes.Length
            +recipes.Sum(r=>192L+r.ExpandedBytes));
    }
    internal Paths64 Materialize(VCarveOrderedDiscHull hull,CancellationToken token)
    {
        var result=new Paths64(recipes.Length);
        foreach(var recipe in recipes)
        {
            token.ThrowIfCancellationRequested();var path=recipe.Materialize(hull);
            if(path.Count<3)throw new InvalidOperationException("Certified deferred input failed the literal hull nondegeneracy invariant.");
            result.Add(path);
        }
        token.ThrowIfCancellationRequested();return result;
    }
}

internal static class VCarvePhysicalHullMetrics
{
    internal static void Merge(VCarveOrderedDiscHull target,VCarveOrderedDiscHull source)
    {
        target.Calls+=source.Calls;target.Vertices+=source.Vertices;
        target.TieGroups+=source.TieGroups;target.TiePairs+=source.TiePairs;target.PairSwaps+=source.PairSwaps;
        target.SortFallbacks+=source.SortFallbacks;target.DuplicatesSkipped+=source.DuplicatesSkipped;
        target.DiscRowCacheHits+=source.DiscRowCacheHits;target.DiscRowCacheMisses+=source.DiscRowCacheMisses;
        target.DiscRowCacheRefusals+=source.DiscRowCacheRefusals;target.DiscRowBufferSwaps+=source.DiscRowBufferSwaps;
        target.DiscVerticesAvoided+=source.DiscVerticesAvoided;
        target.FinalRowValidationComparisons+=source.FinalRowValidationComparisons;
        target.FinalRowValidationComparisonsAvoided+=source.FinalRowValidationComparisonsAvoided;
        target.HullCapacityHints+=source.HullCapacityHints;target.HullCapacityResizes+=source.HullCapacityResizes;
        target.ExtendedRowHits+=source.ExtendedRowHits;target.ExtendedRowMisses+=source.ExtendedRowMisses;
        target.ExtendedRowEvictions+=source.ExtendedRowEvictions;target.ExtendedRowBudgetRefusals+=source.ExtendedRowBudgetRefusals;
        target.ExtendedRowBufferReuses+=source.ExtendedRowBufferReuses;
        target.ExtendedRowPeakBytes=Math.Max(target.ExtendedRowPeakBytes,source.ExtendedRowPeakBytes);
        target.MaximumTieGroup=Math.Max(target.MaximumTieGroup,source.MaximumTieGroup);
        target.CircleOrderBuilds+=source.CircleOrderBuilds;
    }
}
