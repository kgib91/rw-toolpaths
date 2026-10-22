namespace RW.Toolpaths.Milling;

/// <summary>Candidate index for the original descending-radius disc walk.
/// Only an already retained original disc may satisfy the unchanged scalar
/// containment predicate. Tree bounds never authorize removing a primitive.</summary>
internal sealed class PlateauDiscDominance
{
    private readonly record struct Node(double Left,double Bottom,double Right,double Top,
        double Radius,int FirstOrdinal,int First,int Count,int Low,int High);
    private readonly (double X,double Y,double Radius)[] discs;
    private readonly int[] order;
    private readonly bool[] retained;
    private readonly Node[] nodes;
    private readonly CancellationToken token;
    internal long Queries,NodeVisits,Candidates;

    internal PlateauDiscDominance((double X,double Y,double Radius)[] discs,CancellationToken token)
    {
        this.discs=discs;this.token=token;
        order=Enumerable.Range(0,discs.Length).ToArray();retained=new bool[discs.Length];
        var tree=new List<Node>();
        if(discs.Length>0)Build(0,discs.Length);
        nodes=tree.ToArray();
        int Build(int first,int count)
        {
            token.ThrowIfCancellationRequested();
            double left=double.PositiveInfinity,bottom=left,right=double.NegativeInfinity,top=right,radius=0;
            int ordinal=int.MaxValue;
            for(int i=first;i<first+count;i++)
            {
                var d=discs[order[i]];
                left=Math.Min(left,d.X);bottom=Math.Min(bottom,d.Y);right=Math.Max(right,d.X);top=Math.Max(top,d.Y);
                radius=Math.Max(radius,d.Radius);ordinal=Math.Min(ordinal,order[i]);
            }
            int index=tree.Count;tree.Add(default);int low=-1,high=-1;
            if(count>12)
            {
                bool x=right-left>=top-bottom;
                Array.Sort(order,first,count,Comparer<int>.Create((a,b)=>x?discs[a].X.CompareTo(discs[b].X):discs[a].Y.CompareTo(discs[b].Y)));
                int half=count/2;low=Build(first,half);high=Build(first+half,count-half);
            }
            tree[index]=new(left,bottom,right,top,radius,ordinal,first,count,low,high);
            return index;
        }
    }

    internal void Keep(int ordinal)=>retained[ordinal]=true;

    internal bool IsCovered(int ordinal)
    {
        token.ThrowIfCancellationRequested();Queries++;
        if(nodes.Length==0)return false;
        var disc=discs[ordinal];
        return Visit(0);
        bool Visit(int index)
        {
            var node=nodes[index];NodeVisits++;
            if(node.FirstOrdinal>=ordinal||node.Radius-disc.Radius<=1e-10)return false;
            // Deliberately expand the broad phase. Only the original literal
            // predicate below decides containment, including near tangencies.
            double slack=1e-8*Math.Max(1,Math.Max(Math.Max(Math.Abs(node.Left),Math.Abs(node.Right)),
                Math.Max(Math.Abs(node.Bottom),Math.Abs(node.Top)))+node.Radius+disc.Radius);
            double dx=Math.Max(0,Math.Max(node.Left-disc.X,disc.X-node.Right)-slack);
            double dy=Math.Max(0,Math.Max(node.Bottom-disc.Y,disc.Y-node.Top)-slack);
            double gap=node.Radius-disc.Radius+slack;
            if(dx*dx+dy*dy>gap*gap)return false;
            if(node.Low>=0)return Visit(node.Low)||Visit(node.High);
            for(int i=node.First;i<node.First+node.Count;i++)
            {
                int candidate=order[i];if(candidate>=ordinal||!retained[candidate])continue;
                Candidates++;var other=discs[candidate];
                if(other.Radius-disc.Radius>1e-10
                    &&Math.Pow(other.X-disc.X,2)+Math.Pow(other.Y-disc.Y,2)
                        <=Math.Pow(other.Radius-disc.Radius-1e-10,2))return true;
            }
            return false;
        }
    }
}
