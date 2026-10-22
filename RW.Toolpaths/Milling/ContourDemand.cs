namespace RW.Toolpaths.Milling;

/// <summary>Keep short cleared gaps on their source contour instead of
/// manufacturing separate excursions. This only adds nominations; original
/// physical protection still owns every resulting cutting chord.</summary>
internal static class ContourDemand
{
    internal static int BridgeGaps(IReadOnlyList<Point3D> points,bool[] required,
        bool closed,double departureAndReturnLength)
    {
        if(points.Count!=required.Length)throw new ArgumentException("Contour demand must match its vertices.");
        if(!double.IsFinite(departureAndReturnLength)||departureAndReturnLength<0)
            throw new ArgumentOutOfRangeException(nameof(departureAndReturnLength));
        int first=Array.FindIndex(required,p=>p),n=points.Count,bridged=0;
        if(first<0||n<3)return 0;
        int previous=first;
        for(int offset=1;offset<(closed?n+1:n-first);offset++)
        {
            int current=(first+offset)%n;
            if(!required[current])continue;
            int distance=(current-previous+n)%n;
            if(distance>1)
            {
                double length=0;
                for(int step=1;step<=distance&&length<=departureAndReturnLength;step++)
                {
                    var a=points[(previous+step-1)%n];var b=points[(previous+step)%n];
                    length+=Math.Sqrt(Math.Pow(b.X-a.X,2)+Math.Pow(b.Y-a.Y,2)+Math.Pow(b.Z-a.Z,2));
                }
                if(length<=departureAndReturnLength)
                {
                    for(int step=1;step<distance;step++)required[(previous+step)%n]=true;
                    bridged++;
                }
            }
            previous=current;
        }
        return bridged;
    }
}
