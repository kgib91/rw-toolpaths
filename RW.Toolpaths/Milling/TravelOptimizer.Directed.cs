namespace RW.Toolpaths.Milling;

public static partial class TravelOptimizer
{
    // Exact open tour for a small set of already routed, indivisible chains.
    // Unlike boundary-distance 2-opt, costs are directed and include real entry
    // and exit poses. Dependencies restrict ordering; they grant no clearance.
    internal const int MaximumDirectedTourVisits=16;
    internal static int[] OrderDirected(double[] transfer,double[] incoming,double[] outgoing,
        int[] prerequisites,CancellationToken token)
    {
        token.ThrowIfCancellationRequested();int count=incoming.Length;
        if(count>MaximumDirectedTourVisits||transfer.Length!=count*count||outgoing.Length!=count||prerequisites.Length!=count)
            throw new ArgumentException("A bounded square directed-tour cost matrix is required.");
        if(transfer.Concat(incoming).Concat(outgoing).Any(c=>!double.IsFinite(c)||c<0))
            throw new ArgumentException("Directed-tour costs must be finite and nonnegative.");
        int states=1<<count,full=states-1;
        for(int i=0;i<count;i++)if((prerequisites[i]&~full)!=0||(prerequisites[i]&(1<<i))!=0)
            throw new ArgumentException("Invalid directed-tour prerequisite mask.");
        if(count==0)return [];
        var distance=new double[states*count];Array.Fill(distance,double.PositiveInfinity);
        var previous=new short[distance.Length];Array.Fill(previous,(short)-1);
        for(int i=0;i<count;i++)if(prerequisites[i]==0)distance[(1<<i)*count+i]=incoming[i];
        for(int mask=1;mask<states;mask++){
            if((mask&63)==0)token.ThrowIfCancellationRequested();
            for(int last=0;last<count;last++){
                double before=distance[mask*count+last];if(!double.IsFinite(before))continue;
                for(int next=0;next<count;next++){
                    int bit=1<<next;if((mask&bit)!=0||(prerequisites[next]&mask)!=prerequisites[next])continue;
                    int index=(mask|bit)*count+next;double after=before+transfer[last*count+next];
                    if(after<distance[index]-1e-12){distance[index]=after;previous[index]=(short)last;}
                }
            }
        }
        int end=-1;double best=double.PositiveInfinity;
        for(int i=0;i<count;i++){double score=distance[full*count+i]+outgoing[i];if(score<best){best=score;end=i;}}
        if(end<0)throw new InvalidOperationException("Directed-tour prerequisites contain a cycle.");
        var result=new int[count];int remaining=full;
        for(int at=count-1;at>=0;at--){result[at]=end;int prior=previous[remaining*count+end];remaining^=1<<end;end=prior;}
        return result;
    }
}
