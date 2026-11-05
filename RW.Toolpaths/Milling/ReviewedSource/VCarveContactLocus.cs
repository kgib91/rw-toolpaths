using Clipper2Lib;
using RW.Toolpaths;
using RW.Toolpaths.Milling;
namespace RW.Toolpaths.Milling.ReviewedSource;

// ISOLATED prototype. Boundary contact nominates; complete stock authority
// certifies. No point/height sample grants clearance, coverage or routing.
internal static class VCarveContactLocus
{
    internal sealed record Site(PointD Point,double StockDepthLower,double StockDepthUpper,int Ring,int Edge);
    internal sealed record Pair(int SourceStation,Site Left,Site Right);
    internal sealed record Candidate(Point3D Nomination,Point3D CertifiedPose,int SourceStation,double NominalDepth,double WorstContactGap,double TotalPositiveContactGap);
    internal sealed record Result(Pair[] Sites,Candidate[] Candidates,int CircleIntersections,int PoseTrials,Candidate[] Evaluated,bool SearchIncomplete);
    internal sealed record Options(int MaximumPoseTrials,int FamilyRefinementSteps=3,int LocalRefinementSteps=8);
    internal static Pair[] ContactSites(Point3D[] branch,IReadOnlyList<IReadOnlyList<PointD>> boundary,Func<double,double,(double Lower,double Upper)> residualDepth)
    {
        var pairs=new List<Pair>();
        for(int station=0;station<branch.Length;station++){
            var p=branch[station];var a=branch[Math.Max(0,station-1)];var z=branch[Math.Min(branch.Length-1,station+1)];double tx=z.X-a.X,ty=z.Y-a.Y;
            if(tx*tx+ty*ty<1e-20)continue;
            (PointD P,int Ring,int Edge,double Distance)? left=null,right=null;
            for(int ri=0;ri<boundary.Count;ri++)for(int i=0;i<boundary[ri].Count;i++){
                var q=boundary[ri][i];var r=boundary[ri][(i+1)%boundary[ri].Count];double dx=r.x-q.x,dy=r.y-q.y,den=dx*dx+dy*dy;if(den==0)continue;
                double u=Math.Clamp(((p.X-q.x)*dx+(p.Y-q.y)*dy)/den,0,1);var foot=new PointD(q.x+u*dx,q.y+u*dy);double distance=Math.Sqrt(Math.Pow(foot.x-p.X,2)+Math.Pow(foot.y-p.Y,2));double side=tx*(foot.y-p.Y)-ty*(foot.x-p.X);
                if(side>1e-12&&(left is null||distance<left.Value.Distance))left=(foot,ri,i,distance);
                if(side< -1e-12&&(right is null||distance<right.Value.Distance))right=(foot,ri,i,distance);
            }
            if(left is null||right is null)continue;
            Site Make((PointD P,int Ring,int Edge,double Distance) s){var depth=residualDepth(s.P.x,s.P.y);if(!double.IsFinite(depth.Lower+depth.Upper)||depth.Lower<0||depth.Upper<depth.Lower)throw new InvalidOperationException("Invalid actual-stock bounds");return new(s.P,depth.Lower,depth.Upper,s.Ring,s.Edge);}
            pairs.Add(new(station,Make(left.Value),Make(right.Value)));
        }
        return pairs.ToArray();
    }
    internal static Result Construct(Point3D[] branch,IReadOnlyList<IReadOnlyList<PointD>> boundary,
        VCarveCutterProfile cutter,double maximumDepth,
        Func<double,double,(double Lower,double Upper)> residualDepth,
        Func<Point3D,Point3D> solveCertifiedDepth,Func<PointD,bool> allowedCenter,Options options,Point3D startPort,Point3D endPort)
    {
        if(branch.Length<2||maximumDepth<=0||options.MaximumPoseTrials<1)throw new ArgumentException("Invalid contact-locus input");
        var pairs=ContactSites(branch,boundary,residualDepth).ToList();
        var nominations=new List<(Point3D P,int Station,double D)>();var families=new List<(Pair Pair,double Low,double High,double Sign)>();
        foreach(var pair in pairs){
            var q=pair.Left.Point;var r=pair.Right.Point;double dx=r.x-q.x,dy=r.y-q.y,distance=Math.Sqrt(dx*dx+dy*dy);if(distance<=1e-12)continue;
            double lo=Math.Max(pair.Left.StockDepthUpper,pair.Right.StockDepthUpper),hi=maximumDepth;if(lo>hi)continue;
            double Radius(double d,Site s)=>cutter.PhysicalRadiusAtDepth(Math.Max(0,d-s.StockDepthUpper));
            double ContactSum(double d)=>Radius(d,pair.Left)+Radius(d,pair.Right);
            var knots=new List<double>{lo,hi};double sectionTop=0;
            foreach(var section in cutter.Sections){sectionTop+=section.Height;foreach(double stockTop in new[]{pair.Left.StockDepthUpper,pair.Right.StockDepthUpper}){double d=stockTop+sectionTop;if(d>lo&&d<hi)knots.Add(d);}}
            var intervals=knots.Distinct().Order().ToArray();var depths=intervals.ToList();
            // Circle feasibility changes at BOTH external and internal tangency,
            // and at physical profile-section transitions. Whole-depth fractions
            // alone miss the rounded-nose unequal-stock contact family.
            foreach(var test in new Func<double,double>[] {d=>ContactSum(d)-distance,d=>Math.Abs(Radius(d,pair.Left)-Radius(d,pair.Right))-distance})
            for(int i=1;i<intervals.Length;i++){
                double l=intervals[i-1],h=intervals[i],fl=test(l),fh=test(h);if(fl==0)depths.Add(l);if(fh==0)depths.Add(h);if(fl*fh>=0)continue;
                for(int j=0;j<60;j++){double m=(l+h)/2,fm=test(m);if(Math.Sign(fm)==Math.Sign(fl)){l=m;fl=fm;}else h=m;}depths.Add((l+h)/2);
            }
            var critical=depths.Distinct().Order().ToArray();for(int i=1;i<critical.Length;i++)depths.Add((critical[i-1]+critical[i])/2);
            for(int i=1;i<critical.Length;i++)foreach(double sign in new[]{-1d,1d})families.Add((pair,critical[i-1],critical[i],sign));
            foreach(double depth in depths.Distinct()){
                double ra=Radius(depth,pair.Left),rb=Radius(depth,pair.Right);if(ra+rb<distance-1e-10||Math.Abs(ra-rb)>distance+1e-10)continue;
                double along=(ra*ra-rb*rb+distance*distance)/(2*distance),height=Math.Sqrt(Math.Max(0,ra*ra-along*along));
                foreach(double sign in height<1e-10?new[]{1d}:new[]{-1d,1d}){
                    var p=new Point3D(q.x+along*dx/distance-sign*height*dy/distance,q.y+along*dy/distance+sign*height*dx/distance,-depth);
                    if(allowedCenter(new(p.X,p.Y)))nominations.Add((p,pair.SourceStation,depth));
                }
            }
        }
        // Only actual stock within this operation's depth scope needs contact.
        // Unique sites prevent duplicate source samples from weighting quality.
        // Finite analytic nominations ranked before expensive full-stock solves.
        // This search cap is explicit diagnostic cost, never a safety fallback.
        (double Worst,double Total) LocalScore(Point3D p,int station){var pair=pairs.Single(p=>p.SourceStation==station);var gaps=new[]{pair.Left,pair.Right}.Select(s=>{double h=-p.Z-s.StockDepthUpper;return h<0?double.PositiveInfinity:Math.Sqrt(Math.Pow(p.X-s.Point.x,2)+Math.Pow(p.Y-s.Point.y,2))-cutter.PhysicalRadiusAtDepth(h);}).ToArray();return(gaps.Max(),gaps.Sum(g=>Math.Max(0,g)));}
        var candidates=new List<Candidate>();int trials=0;
        var catalog=nominations.DistinctBy(n=>(n.Station,Math.Round(n.P.X,9),Math.Round(n.P.Y,9))).OrderBy(n=>n.Station).ToArray();
        var evaluated=new Dictionary<(int Station,double X,double Y),Candidate>();bool incomplete=false;
        Candidate? Evaluate(Point3D p,int station,double depth){var key=(station,Math.Round(p.X,9),Math.Round(p.Y,9));if(evaluated.TryGetValue(key,out var known))return known;if(trials>=options.MaximumPoseTrials){incomplete=true;return null;}var pose=solveCertifiedDepth(p);trials++;var score=LocalScore(pose,station);var c=new Candidate(p,pose,station,depth,score.Worst,score.Total);evaluated.Add(key,c);candidates.Add(c);return c;}
        // The projected intended source is a genuine contact-family nomination,
        // not an automatic output or a fallback. The same full-footprint depth
        // solver certifies each pose; native and removal proofs remain required.
        var sourcePoses=pairs.ToDictionary(p=>p.SourceStation,p=>solveCertifiedDepth(branch[p.SourceStation]));
        foreach(var pair in pairs)Evaluate(sourcePoses[pair.SourceStation],pair.SourceStation,-sourcePoses[pair.SourceStation].Z);
        // Evaluate the finite physical-profile catalog, then trace it globally.
        // Independent local choices can jump between different contact arms.
        foreach(var n in catalog)Evaluate(n.P,n.Station,n.D);
        foreach(var family in families){
            Point3D? At(double d){var q=family.Pair.Left.Point;var r=family.Pair.Right.Point;double dx=r.x-q.x,dy=r.y-q.y,l=Math.Sqrt(dx*dx+dy*dy);double a=cutter.PhysicalRadiusAtDepth(Math.Max(0,d-family.Pair.Left.StockDepthUpper)),b=cutter.PhysicalRadiusAtDepth(Math.Max(0,d-family.Pair.Right.StockDepthUpper));if(a+b<l||Math.Abs(a-b)>l)return null;double along=(a*a-b*b+l*l)/(2*l),h=Math.Sqrt(Math.Max(0,a*a-along*along));var p=new Point3D(q.x+along*dx/l-family.Sign*h*dy/l,q.y+along*dy/l+family.Sign*h*dx/l,-d);return allowedCenter(new(p.X,p.Y))?p:null;}
            double Value(double d){var p=At(d);if(p is null)return double.PositiveInfinity;return Evaluate(p.Value,family.Pair.SourceStation,d)?.WorstContactGap??double.PositiveInfinity;}
            // Golden-section values nominate only. No unimodality/optimality or
            // safety assertion is inferred; final curve/coverage proofs decide.
            double low=family.Low,high=family.High,ratio=(Math.Sqrt(5)-1)/2,u=high-ratio*(high-low),v=low+ratio*(high-low),fu=Value(u),fv=Value(v);
            for(int step=0;step<options.FamilyRefinementSteps;step++){if(fu<=fv){high=v;v=u;fv=fu;u=high-ratio*(high-low);fu=Value(u);}else{low=u;u=v;fu=fv;v=low+ratio*(high-low);fv=Value(v);}}
        }
        // Boundary-height circle pairs NOMINATE a locus, but actual protected
        // stock under the entire footprint can be the active constraint. Refine
        // XY against the complete certified-depth oracle, not a tip heightmap.
        foreach(var pair in pairs){
            var seed=candidates.Where(c=>c.SourceStation==pair.SourceStation&&double.IsFinite(c.WorstContactGap)).OrderBy(c=>c.WorstContactGap).ThenBy(c=>c.TotalPositiveContactGap).FirstOrDefault();if(seed is null)continue;
            double dx=pair.Right.Point.x-pair.Left.Point.x,dy=pair.Right.Point.y-pair.Left.Point.y,size=Math.Sqrt(dx*dx+dy*dy);if(size==0)continue;dx/=size;dy/=size;double stepSize=size/2,reach=cutter.PhysicalRadiusAtDepth(maximumDepth);
            double Value(PointD p){if(!allowedCenter(p)||Math.Sqrt(Math.Pow(p.x-pair.Left.Point.x,2)+Math.Pow(p.y-pair.Left.Point.y,2))>reach||Math.Sqrt(Math.Pow(p.x-pair.Right.Point.x,2)+Math.Pow(p.y-pair.Right.Point.y,2))>reach)return double.PositiveInfinity;return Evaluate(new(p.x,p.y,-maximumDepth),pair.SourceStation,maximumDepth)?.WorstContactGap??double.PositiveInfinity;}
            var origin=new PointD(seed.CertifiedPose.X,seed.CertifiedPose.Y);var simplex=new[]{(P:origin,F:Value(origin)),(P:new PointD(origin.x+dx*stepSize,origin.y+dy*stepSize),F:0d),(P:new PointD(origin.x-dy*stepSize,origin.y+dx*stepSize),F:0d)};simplex[1]=(simplex[1].P,Value(simplex[1].P));simplex[2]=(simplex[2].P,Value(simplex[2].P));
            PointD From(PointD center,PointD point,double factor)=>new(center.x+(point.x-center.x)*factor,center.y+(point.y-center.y)*factor);
            for(int iteration=0;iteration<options.LocalRefinementSteps;iteration++){
                simplex=simplex.OrderBy(s=>s.F).ToArray();var center=new PointD((simplex[0].P.x+simplex[1].P.x)/2,(simplex[0].P.y+simplex[1].P.y)/2);var reflected=From(center,simplex[2].P,-1);double fr=Value(reflected);
                if(fr<simplex[0].F){var expanded=From(center,reflected,2);double fe=Value(expanded);simplex[2]=fe<fr?(expanded,fe):(reflected,fr);}
                else if(fr<simplex[1].F)simplex[2]=(reflected,fr);
                else{var contracted=From(center,fr<simplex[2].F?reflected:simplex[2].P,.5);double fc=Value(contracted);if(fc<Math.Min(fr,simplex[2].F))simplex[2]=(contracted,fc);else for(int i=1;i<3;i++){var p=From(simplex[0].P,simplex[i].P,.5);simplex[i]=(p,Value(p));}}
            }
        }
        if(incomplete)return new(pairs.ToArray(),[],nominations.Count,trials,candidates.ToArray(),true);
        var chosen=ChooseCoherent(startPort,endPort,candidates.ToArray(),sourcePoses);
        return new(pairs.ToArray(),chosen,nominations.Count,trials,candidates.ToArray(),false);
    }
    // Best achievable worst contact remains the first objective. Among equally
    // good contact families, preserve the intended projected source before
    // minimizing route length. Shortest contact routing alone can move an arm
    // away from material it must still remove. These are nominations: continuous
    // native fit, real stock and independent removal certification still decide.
    internal static Candidate[] ChooseCoherent(Point3D start,Point3D end,IReadOnlyList<Candidate> catalog,
        IReadOnlyDictionary<int,Point3D>? sourcePoses=null)
    {
        var groups=catalog.Where(c=>double.IsFinite(c.WorstContactGap)).GroupBy(c=>c.SourceStation).OrderBy(g=>g.Key).ToArray();
        if(groups.Length==0)return [];
        double worst=groups.Max(g=>g.Min(c=>Math.Max(0,c.WorstContactGap)));
        var allowed=groups.Select(g=>g.Where(c=>Math.Max(0,c.WorstContactGap)<=worst).ToArray()).ToArray();
        double Displacement(Candidate c)=>sourcePoses is null?0:Math.Pow(VCarveNativeCurveMath.Distance(sourcePoses[c.SourceStation],c.CertifiedPose),2);
        var states=new[]{(Displacement:0d,Length:0d,Integral:0d,Path:Array.Empty<Candidate>())};
        foreach(var group in allowed)
        {
            double bestDisplacement=states.Min(s=>s.Displacement);
            var predecessors=states.Where(s=>s.Displacement==bestDisplacement).ToArray();
            states=group.Select(c=>
            {
                var prior=predecessors.OrderBy(s=>s.Length+VCarveNativeCurveMath.Distance(s.Path.Length==0?start:s.Path[^1].CertifiedPose,c.CertifiedPose)).ThenBy(s=>s.Integral).First();
                return(Displacement:prior.Displacement+Displacement(c),
                    Length:prior.Length+VCarveNativeCurveMath.Distance(prior.Path.Length==0?start:prior.Path[^1].CertifiedPose,c.CertifiedPose),
                    Integral:prior.Integral+Math.Max(0,c.TotalPositiveContactGap),Path:prior.Path.Append(c).ToArray());
            }).ToArray();
        }
        return states.OrderBy(s=>s.Displacement).ThenBy(s=>s.Length+VCarveNativeCurveMath.Distance(s.Path[^1].CertifiedPose,end)).ThenBy(s=>s.Integral).First().Path;
    }

}
