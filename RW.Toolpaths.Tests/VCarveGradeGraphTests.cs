using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class VCarveGradeGraphTests
{
    [Fact]
    public void ActiveSeedsMatchIndependentBellmanFordClosureForBothDepthFields()
    {
        var random=new Random(83117);
        for(int trial=0;trial<80;trial++)
        {
            const int count=48;
            var points=Enumerable.Range(0,count).Select(i=>new Point3D(i%8,i/8,-(trial%2==0?6.35:8*random.NextDouble()))).ToArray();
            points[0]=points[0] with{Z=0};
            var pairs=Enumerable.Range(1,count-1).Select(i=>(A:i-1,B:i)).Concat(
                Enumerable.Range(0,60).Select(_=>(A:random.Next(count),B:random.Next(count)))).ToArray();
            TaggedToolpath Path(Point3D a,Point3D b)=>new([a,b],Array.Empty<ToolpathSpan>(),0,"final-carve",null,detectCurves:false);
            var source=pairs.Select(p=>Path(points[p.A],points[p.B])).ToArray();
            var upper=source.Select(p=>Path(p.Points[0] with{Z=p.Points[0].Z-.008},p.Points[1] with{Z=p.Points[1].Z-.008})).ToArray();
            double grade=.05+random.NextDouble();
            var result=VCarveSmoothing.LimitGraphPair(source,upper,grade,default);
            for(int field=0;field<2;field++)
            {
                double[] depths=points.Select(p=>-p.Z+(field==0?0:.008)).ToArray();
                bool changed=true;int rounds=0;
                while(changed)
                {
                    Assert.True(rounds++<1000);changed=false;
                    foreach(var edge in pairs)
                    {
                        var a=points[edge.A];var b=points[edge.B];
                        double cost=grade*Math.Sqrt((a.X-b.X)*(a.X-b.X)+(a.Y-b.Y)*(a.Y-b.Y));
                        Relax(edge.A,edge.B);Relax(edge.B,edge.A);
                        void Relax(int from,int to)
                        {
                            double next=depths[from]+cost;
                            if(next>depths[from]&&next-depths[from]>cost)next=Math.BitDecrement(next);
                            if(next>=depths[to])return;depths[to]=next;changed=true;
                        }
                    }
                }
                var actual=field==0?result.Limited:result.Upper;
                for(int i=0;i<pairs.Length;i++)
                {
                    Assert.Equal(-depths[pairs[i].A],actual[i].Points[0].Z);
                    Assert.Equal(-depths[pairs[i].B],actual[i].Points[1].Z);
                }
            }
        }
    }
}
