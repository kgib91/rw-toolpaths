namespace RW.Toolpaths.Geometry;

/// <summary>Exact balanced spatial partition without sorting either subtree.</summary>
internal static class SpatialMedianPartition
{
    internal static long Partition<T>(T[] values,int first,int count,Comparison<T> compare,CancellationToken token)
    {
        int lo=first,hi=first+count-1,median=first+count/2;
        long visits=0,remaining=3L*count;
        while(lo<hi)
        {
            token.ThrowIfCancellationRequested();
            int length=hi-lo+1;visits+=length;remaining-=length;
            if(remaining<0)
            {
                // Preserve the original exact sort for adversarial partitions.
                Array.Sort(values,lo,length,Comparer<T>.Create(compare));return visits;
            }
            int mid=lo+(hi-lo)/2;
            if(compare(values[mid],values[lo])<0)Swap(mid,lo);
            if(compare(values[hi],values[lo])<0)Swap(hi,lo);
            if(compare(values[hi],values[mid])<0)Swap(hi,mid);
            T pivot=values[mid];int less=lo,at=lo,greater=hi,processed=0;
            while(at<=greater)
            {
                if((++processed&1023)==0)token.ThrowIfCancellationRequested();
                int order=compare(values[at],pivot);
                if(order<0)Swap(less++,at++);
                else if(order>0)Swap(at,greater--);
                else at++;
            }
            if(median<less)hi=less-1;
            else if(median>greater)lo=greater+1;
            else return visits;
        }
        return visits;
        void Swap(int a,int b)=>(values[a],values[b])=(values[b],values[a]);
    }
}
