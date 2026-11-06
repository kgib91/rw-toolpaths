using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class VCarveRingSeamSelectionTests
{
    [Fact]
    public void BoundedSelectionMatchesLiteralNominationAndStableOrdering()
    {
        var random=new Random(194207);
        for(int sample=0;sample<120;sample++)
        {
            double scale=(sample%3) switch{0=>1e-8,1=>1, _=>1e6};
            var paths=Enumerable.Range(0,4).Select(owner=>
            {
                var points=new List<Point3D>();
                for(int i=0;i<70;i++)
                {
                    // Repeated vertices, vertical-only chords and coincident
                    // planar points with different Z retain their source ports.
                    var previous=points.LastOrDefault();
                    points.Add(i>0&&i%7==0?previous:i>0&&i%11==0?previous with{Z=previous.Z+scale}:
                        new Point3D(random.Next(-9,10)*scale,random.Next(-9,10)*scale,random.Next(-3,4)*scale));
                }
                if(sample%2==0)points.Add(points[0]);
                return Path(points);
            }).ToArray();
            int[] owners=sample%2==0?[3,1,3,0,2,1]:[0,1,2,3];
            var from=new Point3D(random.Next(-10,11)*scale,random.Next(-10,11)*scale,scale);
            double reach=sample%4==0?double.PositiveInfinity:random.Next(0,20)*scale;
            var literal=Literal(paths,from,owners,reach);
            var seams=new VCarveRingSeams(paths,default);
            Assert.Equal(literal,seams.Nearest(from,owners,reach).ToArray());
            foreach(int limit in new[]{0,1,2,7,24,48,49,200})
                Assert.Equal(literal.Take(limit).ToArray(),seams.Nearest(from,owners,reach,limit).ToArray());
        }
    }

    [Fact]
    public void EqualDistanceCandidatesPreserveOriginalParameterAndInsertionTies()
    {
        var paths=new[]{
            Path([new(-1,1,0),new(1,1,2),new(1,-1,0),new(-1,-1,2),new(-1,1,0)]),
            Path([new(-1,1,2),new(1,1,0),new(1,-1,2),new(-1,-1,0),new(-1,1,2)]),
            Path([new(0,1,0),new(0,1,3),new(0,-1,0),new(0,1,0)])};
        int[] owners=[2,1,0,2,0];var from=new Point3D(0,0,0);
        var literal=Literal(paths,from,owners,double.PositiveInfinity);
        var seams=new VCarveRingSeams(paths,default);
        for(int limit=1;limit<=literal.Length+1;limit++)
            Assert.Equal(literal.Take(limit).ToArray(),seams.Nearest(from,owners,double.PositiveInfinity,limit).ToArray());
        Assert.Equal(paths[0].Points[0],new Point3D(-1,1,0));
    }

    [Fact]
    public void BoundedSelectionSortsOnlyTheConsumedPrefixAndStillVisitsEverySourceChord()
    {
        var points=Enumerable.Range(0,4097).Select(i=>new Point3D(i,Math.Sin(i*.1),i*.001)).ToArray();
        var paths=new[]{Path(points)};var from=new Point3D(2048,2,0);
        var seams=new VCarveRingSeams(paths,default);
        var actual=seams.Nearest(from,[0],double.PositiveInfinity,49).ToArray();
        Assert.Equal(Literal(paths,from,[0],double.PositiveInfinity).Take(49).ToArray(),actual);
        Assert.Equal(4096,seams.CandidateChordsExamined);
        Assert.Equal(49,seams.CandidatesRetainedForSort);
        Assert.Equal(seams.CandidateNominations-49,seams.CandidatesDiscardedBeforeSort);
        Assert.True(seams.CandidatesDiscardedBeforeSort>4000);
        Assert.Equal(1,seams.BoundedQueries);
    }

    [Fact]
    public void WarmContiguousBoundsKeepEveryLiteralPrefixAcrossOwnersTiesAndReachModes()
    {
        var random=new Random(663921);
        for(int sample=0;sample<60;sample++)
        {
            double scale=(sample%4) switch{0=>1e-8,1=>1,2=>1e5,_=>1e9};
            var paths=Enumerable.Range(0,4).Select(owner=>
            {
                var points=new List<Point3D>();
                for(int i=0;i<321;i++)
                {
                    double angle=i*Math.Tau/320;
                    var p=new Point3D((owner*4+Math.Cos(angle))*scale,Math.Sin(angle)*scale,(i%3)*scale);
                    points.Add(i>0&&i%11==0?points[^1]:i>0&&i%17==0?points[^1] with{Z=p.Z}:p);
                }
                return Path(points);
            }).ToArray();
            int[] owners=sample%2==0?[3,1,3,0,2,1]:[0,1,2,3];
            var seams=new VCarveRingSeams(paths,default);
            // Distinct queries warm the immutable per-path trees. Their leaves
            // must still nominate in literal source order after every prune.
            seams.Nearest(new(0,0,0),owners,double.PositiveInfinity,3).ToArray();
            seams.Nearest(new(0,0,0),owners,double.PositiveInfinity,3).ToArray();
            Assert.Equal(4,seams.IndexBuilds);
            var from=new Point3D(random.Next(-2,14)*scale,random.Next(-2,3)*scale,0);
            foreach(double reach in new[]{0,.1*scale,2*scale,double.PositiveInfinity})
            {
                var expected=Literal(paths,from,owners,reach);
                foreach(int limit in new[]{1,3,12,48,49,int.MaxValue})
                    Assert.Equal(expected.Take(limit).ToArray(),seams.Nearest(from,owners,reach,limit).ToArray());
            }
            Assert.InRange(seams.IndexRetainedBytes,1,16L<<20);
        }
    }

    [Fact]
    public void RepeatedQueriesPruneDistantRunsAfterTheExactPrefixIsEstablished()
    {
        var points=Enumerable.Range(0,4097).Select(i=>new Point3D(i,Math.Sin(i*.1),i*.001)).ToArray();
        var paths=new[]{Path(points)};var from=new Point3D(0,2,0);int[] owners=[0,0];
        var seams=new VCarveRingSeams(paths,default);
        var expected=Literal(paths,from,owners,double.PositiveInfinity).Take(13).ToArray();
        Assert.Equal(expected,seams.Nearest(from,owners,double.PositiveInfinity,13).ToArray());
        Assert.Equal(0,seams.IndexBuilds); // Duplicate owners are one warm-up query.
        long before=seams.CandidateChordsExamined;
        Assert.Equal(expected,seams.Nearest(from,owners,double.PositiveInfinity,13).ToArray());
        Assert.Equal(1,seams.IndexBuilds);
        Assert.True(seams.BoundsPrunedChords>7000);
        Assert.True(seams.CandidateChordsExamined-before<512);
        Assert.True(seams.BoundsNodeVisits>0);
        Assert.Equal(Literal(paths,from,owners,.25),seams.Nearest(from,owners,.25).ToArray());
        Assert.Equal(Literal(paths,from,owners,double.PositiveInfinity),seams.Nearest(from,owners,double.PositiveInfinity).ToArray());
        Assert.InRange(seams.IndexRetainedBytes,1,16L<<20);
    }

    [Fact]
    public void BoundsKeepLiteralNumericEdgeCasesAndFallBackOutsideTheirFiniteDomain()
    {
        foreach(double origin in new[]{0d,1e12-10000,1e300})
        {
            var points=Enumerable.Range(0,257).Select(i=>new Point3D(origin+i*.0001220703125,
                i%2==0?double.Epsilon:i%3==0?-0d:0d,i%3)).ToArray();
            var paths=new[]{Path(points)};var seams=new VCarveRingSeams(paths,default);
            foreach(var from in new[]{new Point3D(origin,0,0),new Point3D(origin+.01,double.Epsilon,0)})
            for(int query=0;query<3;query++)
            foreach(double reach in new[]{0,Math.BitIncrement(.01),double.PositiveInfinity})
                Assert.Equal(Literal(paths,from,[0,0],reach).Take(3).ToArray(),
                    seams.Nearest(from,[0,0],reach,3).ToArray());
            if(origin>1e12){Assert.Equal(0,seams.IndexBuilds);}
        }
        foreach(double unsupported in new[]{double.NaN,double.PositiveInfinity,double.NegativeInfinity})
        {
            var invalid=Enumerable.Range(0,257).Select(i=>new Point3D(i==128?unsupported:i,0,0)).ToArray();
            var nonfinitePaths=new[]{Path(invalid)};var fallback=new VCarveRingSeams(nonfinitePaths,default);
            for(int query=0;query<3;query++)
                Assert.Equal(Literal(nonfinitePaths,new(0,0,0),[0],double.PositiveInfinity).Take(12).ToArray(),
                    fallback.Nearest(new(0,0,0),[0],double.PositiveInfinity,12).ToArray());
            Assert.Equal(0,fallback.IndexBuilds);Assert.Equal(1,fallback.IndexBuildRefusals);
        }
    }

    [Fact]
    public void CancellationCannotPublishAPrefixFromAWarmPrunedIndex()
    {
        var points=Enumerable.Range(0,2049).Select(i=>new Point3D(i,0,0)).ToArray();
        using var cancellation=new CancellationTokenSource();
        var seams=new VCarveRingSeams([Path(points)],cancellation.Token);
        seams.Nearest(new(0,0,0),[0],double.PositiveInfinity,3).ToArray();
        seams.Nearest(new(0,0,0),[0],double.PositiveInfinity,3).ToArray();
        Assert.Equal(1,seams.IndexBuilds);cancellation.Cancel();
        Assert.Throws<OperationCanceledException>(()=>seams.Nearest(new(0,0,0),[0],1,3).ToArray());
    }

    [Fact]
    public void BoundedSelectionRetainsOriginalCancellationCheck()
    {
        var points=Enumerable.Range(0,2048).Select(i=>new Point3D(i,0,0)).ToArray();
        using var cancellation=new CancellationTokenSource();cancellation.Cancel();
        var seams=new VCarveRingSeams([Path(points)],cancellation.Token);
        Assert.Throws<OperationCanceledException>(()=>seams.Nearest(new(0,0,0),[0],double.PositiveInfinity,49).ToArray());
    }

    static TaggedToolpath Path(IReadOnlyList<Point3D> points)
        =>new(points,[],0,"clearing",0,detectCurves:false);

    // Independent literal oracle retained from the original full nomination
    // contract. Full source visitation, deduplication and stable ordering are
    // essential: comparing only distances would miss changed seam ownership.
    static VCarveRingSeams.Landing[] Literal(IReadOnlyList<TaggedToolpath> paths,Point3D from,
        IEnumerable<int> owners,double reach)
    {
        var found=new List<VCarveRingSeams.Landing>();var seen=new HashSet<(int,Point3D)>();
        foreach(int owner in owners)
        for(int end=1;end<paths[owner].Points.Count;end++)
        {
            var a=paths[owner].Points[end-1];var b=paths[owner].Points[end];
            double dx=b.X-a.X,dy=b.Y-a.Y,square=dx*dx+dy*dy;if(square==0)continue;
            double t=Math.Clamp(((from.X-a.X)*dx+(from.Y-a.Y)*dy)/square,0,1);
            var point=t==0?a:t==1?b:new(a.X+t*dx,a.Y+t*dy,a.Z+t*(b.Z-a.Z));
            Add(t,point);Add(0,a);Add(1,b);
            void Add(double parameter,Point3D candidate)
            {
                double distance=Math.Pow(from.X-candidate.X,2)+Math.Pow(from.Y-candidate.Y,2);
                if(distance<=reach*reach&&seen.Add((owner,candidate)))found.Add(new(owner,end,parameter,candidate,distance));
            }
        }
        return found.OrderBy(p=>p.DistanceSquared).ThenBy(p=>p.Owner).ThenBy(p=>p.End).ToArray();
    }
}
