namespace RW.Toolpaths.Milling.ReviewedSource;

// Free native joins own no source sample or source timing. Nominate their
// ordered geometric positions on the complete source; final continuous
// certificates still cover every source chord and every native primitive.
internal static class VCarveContactSourceCorrespondence
{
    internal sealed record Mapping(Point3D[] Reference,double[] SourceParameters,int[] Indices);
    internal static Mapping Nominate(IReadOnlyList<ToolMove> moves,Point3D[] source,double[] parameter)
    {
        var joins=new List<(double T,Point3D Point,int Join)>();double prior=0;
        for(int join=1;join<moves.Count;join++)
        {
            var target=moves[join].Start;double best=double.PositiveInfinity,chosen=prior;Point3D point=source[0];
            for(int i=0;i<source.Length-1;i++)
            {
                if(parameter[i+1]<prior||parameter[i+1]<=parameter[i])continue;
                var delta=VCarveNativeCurveMath.Sub(source[i+1],source[i]);double square=VCarveNativeCurveMath.Dot(delta,delta);
                double lower=Math.Max(0,(prior-parameter[i])/(parameter[i+1]-parameter[i]));
                double u=square==0?lower:Math.Clamp(VCarveNativeCurveMath.Dot(VCarveNativeCurveMath.Sub(target,source[i]),delta)/square,lower,1);
                var candidate=VCarveNativeCurveMath.Lerp(source[i],source[i+1],u);double distance=VCarveNativeCurveMath.Distance(candidate,target);
                if(distance<best){best=distance;chosen=parameter[i]+u*(parameter[i+1]-parameter[i]);point=candidate;}
            }
            joins.Add((chosen,point,join));prior=chosen;
        }
        var events=source.Select((p,i)=>(T:parameter[i],Point:p,Join:-1)).Concat(joins).OrderBy(e=>e.T).ThenBy(e=>e.Join);
        var reference=new List<Point3D>();var times=new List<double>();var indices=new int[moves.Count+1];
        foreach(var e in events)
        {
            if(reference.Count==0||reference[^1]!=e.Point){reference.Add(e.Point);times.Add(e.T);}
            if(e.Join>0)indices[e.Join]=reference.Count-1;
        }
        indices[^1]=reference.Count-1;return new(reference.ToArray(),times.ToArray(),indices);
    }
}
