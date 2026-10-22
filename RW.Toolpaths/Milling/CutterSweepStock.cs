using Clipper2Lib;
using RW.Toolpaths.Geometry;
using System.Diagnostics;

namespace RW.Toolpaths.Milling;

/// <summary>Whole-cutter containment in immutable, committed commanded cuts.
/// This is final-union omission evidence, never travel or fresh-stock clearance.</summary>
internal sealed class CutterSweepStock
{
    private const double Reserve=1e-7;
    private readonly record struct Chord(Point3D A,Point3D B);
    private sealed class Node
    {
        internal double MinX,MinY,MaxX,MaxY,Depth,Radius;
        internal int First,Count;
        internal Node? Left,Right;
    }
    private sealed record Block(Chord[] Chords,Node Root,PreparedPointIndex FloorIndex);
    private readonly VCarveCutterProfile profile;
    private readonly Func<Point3D,Point3D> command;
    private readonly double surface;
    private readonly CancellationToken token;
    private readonly List<Chord> all=new(),pending=new();
    private readonly List<Block> blocks=new();
    private readonly Dictionary<(long,long),Point3D> floorSites=new();
    private readonly Dictionary<(long,long),Point3D> pendingFloorSites=new();
    private double maximumDepth,floorWanted,floorTolerance;
    private bool floorNormal;
    private Point3D floorQuery;
    private Point3D floorWitness;
    private bool floorWitnessKnown;
    private Chord? lastWholeWitness;
    internal long PriorWholeChordTrials{get;private set;}
    internal long PriorWholeChordProofs{get;private set;}
    internal long PriorFloorPoseTrials{get;private set;}
    internal long PriorFloorPoseProofs{get;private set;}
    private readonly Func<Point3D,bool> floorPredicate;
    private readonly Func<double,double> floorRadius;
    private long floorQueryTicks,floorBuildTicks,stockBuildTicks;
    private readonly double[] joints;
    private readonly bool supported;
    private readonly bool useMedianPartition;
    internal long PartitionVisits{get;private set;}
    internal long ReindexedChords{get;private set;}
    internal int MergeCount{get;private set;}
    internal int MaximumBlocks{get;private set;}
    internal long Queries{get;private set;}
    internal long Accepted{get;private set;}
    internal long UnionAccepted{get;private set;}
    internal int Chords=>all.Count;
    internal long FloorTargetQueries{get;private set;}
    internal long FloorTargetsCovered{get;private set;}
    internal long FloorTargetIndexHits{get;private set;}
    internal long FloorTargetChordFallbacks{get;private set;}
    internal long FloorRingQueries{get;private set;}
    internal long FloorRingCovered{get;private set;}
    internal long FloorRingBudgetFallbacks{get;private set;}
    internal int FloorSites=>floorSites.Count;
    internal double FloorTargetMilliseconds=>1000d*floorQueryTicks/Stopwatch.Frequency;
    internal double FloorIndexBuildMilliseconds=>1000d*floorBuildTicks/Stopwatch.Frequency;
    internal double StockIndexBuildMilliseconds=>1000d*stockBuildTicks/Stopwatch.Frequency;

    internal CutterSweepStock(VCarveCutterProfile profile,Func<Point3D,Point3D> command,double surface,CancellationToken token,
        bool useMedianPartition=true)
    {
        this.profile=profile;this.command=command;this.surface=surface;this.token=token;
        this.useMedianPartition=useMedianPartition;
        floorRadius=h=>floorNormal?OffsetRadius(h-floorTolerance,floorTolerance):profile.PhysicalRadiusAtDepth(h);
        floorPredicate=p=>{
            double h=-p.Z-floorWanted;if(h<0)return false;
            double r=floorRadius(h),x=p.X-floorQuery.X,y=p.Y-floorQuery.Y;
            if(x*x+y*y+Reserve>r*r)return false;
            floorWitness=p;floorWitnessKnown=true;return true;
        };
        var depths=new List<double>();double depth=0;
        supported=profile.Sections.Count<=64;
        for(int i=0;i<profile.Sections.Count;i++)
        {
            var section=profile.Sections[i];depth+=section.Height;depths.Add(depth);
            supported&=section.Kind==CarvingCutterSectionKind.Linear
                ||i==0&&section.BottomRadius==0&&section.Height==section.TopRadius;
        }
        joints=depths.ToArray();
    }

    internal void AddProtected(IReadOnlyList<TaggedToolpath> paths)
    {
        if(!supported)return;
        foreach(var path in paths)
        {
            bool known=false;
            for(int i=1;i<path.Points.Count;i++)
            {
                if((i&1023)==0)token.ThrowIfCancellationRequested();
                if(all.Count>=2_000_000)return;
                var a=path.Points[i-1];var b=path.Points[i];
                float dx=(float)b.X-(float)a.X,dy=(float)(b.Z+surface)-(float)(a.Z+surface),dz=(float)b.Y-(float)a.Y;
                if(ToolpathCommandRounding.IsVerticalOnly(dx,dy,dz)){known=false;continue;}
                var ca=command(a);var cb=command(b);
                if(!Finite(ca)||!Finite(cb)){known=false;continue;}
                // A full XYZ endpoint is real even when the incoming planar
                // cursor was unknown. Only the following affine chord is known.
                var chord=new Chord(known?ca:cb,cb);known=true;
                if(Math.Max(-chord.A.Z,-chord.B.Z)<=0)continue;
                all.Add(chord);pending.Add(chord);
                maximumDepth=Math.Max(maximumDepth,Math.Max(-chord.A.Z,-chord.B.Z));
                AddFloorSite(chord.A);AddFloorSite(chord.B);
            }
        }
        void AddFloorSite(Point3D p)
        {
            // Keep a REAL endpoint, never a cell-centre/depth combination.
            // Losing a witness only invokes the original chord search below.
            var key=((long)Math.Round(p.X*10),(long)Math.Round(p.Y*10));
            if(floorSites.TryGetValue(key,out var old))
            {if(p.Z<old.Z){floorSites[key]=p;pendingFloorSites[key]=p;}}
            else if(floorSites.Count<2_000_000){floorSites.Add(key,p);pendingFloorSites[key]=p;}
        }
    }

    internal void Seal()
    {
        if(pending.Count==0)return;
        Chord[] rows;
        bool merge=blocks.Count>=8;
        Point3D[] floorRows;
        if(merge)
        {
            rows=all.ToArray();floorRows=floorSites.Values.ToArray();
            ReindexedChords+=blocks.Sum(b=>(long)b.Chords.Length);blocks.Clear();MergeCount++;
        }
        else{rows=pending.ToArray();floorRows=pendingFloorSites.Values.ToArray();}
        pending.Clear();
        Node Build(int first,int count)
        {
            token.ThrowIfCancellationRequested();
            var node=new Node{First=first,Count=count,MinX=double.PositiveInfinity,MinY=double.PositiveInfinity,
                MaxX=double.NegativeInfinity,MaxY=double.NegativeInfinity};
            for(int i=first;i<first+count;i++)
            {
                var c=rows[i];node.MinX=Math.Min(node.MinX,Math.Min(c.A.X,c.B.X));node.MaxX=Math.Max(node.MaxX,Math.Max(c.A.X,c.B.X));
                node.MinY=Math.Min(node.MinY,Math.Min(c.A.Y,c.B.Y));node.MaxY=Math.Max(node.MaxY,Math.Max(c.A.Y,c.B.Y));
                node.Depth=Math.Max(node.Depth,Math.Max(-c.A.Z,-c.B.Z));
            }
            node.Radius=profile.PhysicalRadiusAtDepth(node.Depth);
            if(count>12)
            {
                bool x=node.MaxX-node.MinX>=node.MaxY-node.MinY;
                int Compare(Chord a,Chord b)=>(x?a.A.X+a.B.X:a.A.Y+a.B.Y).CompareTo(x?b.A.X+b.B.X:b.A.Y+b.B.Y);
                if(useMedianPartition)PartitionVisits+=SpatialMedianPartition.Partition(rows,first,count,Compare,token);
                else Array.Sort(rows,first,count,Comparer<Chord>.Create(Compare));
                int half=count/2;node.Left=Build(first,half);node.Right=Build(first+half,count-half);
            }
            return node;
        }
        long started=Stopwatch.GetTimestamp();var root=Build(0,rows.Length);
        stockBuildTicks+=Stopwatch.GetTimestamp()-started;
        started=Stopwatch.GetTimestamp();
        // Share the immutable chord-block lifetime. Rebuilding the complete
        // endpoint index after every ring repeats work on unchanged old cuts.
        var floorIndex=new PreparedPointIndex(floorRows,token);
        pendingFloorSites.Clear();floorBuildTicks+=Stopwatch.GetTimestamp()-started;
        blocks.Add(new(rows,root,floorIndex));
        MaximumBlocks=Math.Max(MaximumBlocks,blocks.Count);
    }

    internal bool Contains(Point3D a,Point3D b,double tolerance,double radialReserve=0)
    {
        token.ThrowIfCancellationRequested();
        if(!supported||!Finite(a)||!Finite(b)||++Queries>2_000_000)return false;
        a=a with{Z=Math.Min(0,a.Z+tolerance)};b=b with{Z=Math.Min(0,b.Z+tolerance)};
        double depth=Math.Max(-a.Z,-b.Z);
        if(depth<=0)return true;
        double ra=profile.PhysicalRadiusAtDepth(-a.Z)+radialReserve,rb=profile.PhysicalRadiusAtDepth(-b.Z)+radialReserve;
        bool Near(Node n,Point3D p,double reach)
        {
            if(reach<0)return false;
            double x=Math.Max(0,Math.Max(n.MinX-p.X,p.X-n.MaxX)),y=Math.Max(0,Math.Max(n.MinY-p.Y,p.Y-n.MaxY));
            return x*x+y*y<=reach*reach;
        }
        bool Covered(Chord c,Point3D first,Point3D last)
        {
            double dx=c.B.X-c.A.X,dy=c.B.Y-c.A.Y,square=dx*dx+dy*dy;
            Point3D At(Point3D q)
            {
                double t=square>0?Math.Clamp(((q.X-c.A.X)*dx+(q.Y-c.A.Y)*dy)/square,0,1):0;
                return new(c.A.X+t*dx,c.A.Y+t*dy,c.A.Z+t*(c.B.Z-c.A.Z));
            }
            var u=At(first);var v=At(last);
            double delta=Math.Min(first.Z-u.Z,last.Z-v.Z);
            if(delta<0)return false;
            double distance=Math.Max(Math.Sqrt(Math.Pow(first.X-u.X,2)+Math.Pow(first.Y-u.Y,2)),
                Math.Sqrt(Math.Pow(last.X-v.X,2)+Math.Pow(last.Y-v.Y,2)));
            return RadialMargin(delta,Math.Max(-first.Z,-last.Z))>=distance+radialReserve+Reserve;
        }
        bool Search(Block block,Node node)
        {
            if(node.Depth<depth||!Near(node,a,node.Radius-ra)||!Near(node,b,node.Radius-rb))return false;
            if(node.Left is not null)return Search(block,node.Left)||Search(block,node.Right!);
            for(int i=node.First;i<node.First+node.Count;i++)if(Covered(block.Chords[i],a,b))
            {lastWholeWitness=block.Chords[i];return true;}
            return false;
        }
        if(lastWholeWitness is { } prior)
        {
            PriorWholeChordTrials++;
            // The actual old commanded chord is only a nominee. Re-evaluate
            // its COMPLETE affine/profile proof with this query's XYZ, axial
            // allowance and radial reserve; no previous clearance transfers.
            if(Covered(prior,a,b)){Accepted++;PriorWholeChordProofs++;return true;}
        }
        for(int i=blocks.Count-1;i>=0;i--)
            if(Search(blocks[i],blocks[i].Root)){Accepted++;return true;}
        double qx=b.X-a.X,qy=b.Y-a.Y,square=qx*qx+qy*qy;
        if(square<=1e-24)return false;
        var intervals=new List<(double Lo,double Hi)>();
        Point3D QueryAt(double t)=>t==0?a:t==1?b:new(a.X+qx*t,a.Y+qy*t,a.Z+(b.Z-a.Z)*t);
        void Gather(Block block,Node node)
        {
            double reach=node.Radius-Math.Min(ra,rb);
            if(intervals.Count>4096||node.Depth<Math.Min(-a.Z,-b.Z)||reach<0
                ||node.MaxX+reach<Math.Min(a.X,b.X)||node.MinX-reach>Math.Max(a.X,b.X)
                ||node.MaxY+reach<Math.Min(a.Y,b.Y)||node.MinY-reach>Math.Max(a.Y,b.Y))return;
            if(node.Left is not null){Gather(block,node.Left);Gather(block,node.Right!);return;}
            for(int i=node.First;i<node.First+node.Count;i++)
            {
                var c=block.Chords[i];
                double u=((c.A.X-a.X)*qx+(c.A.Y-a.Y)*qy)/square;
                double v=((c.B.X-a.X)*qx+(c.B.Y-a.Y)*qy)/square;
                double lo=Math.Max(0,Math.Min(u,v)),hi=Math.Min(1,Math.Max(u,v));
                if(hi<=lo)continue;
                if(!Covered(c,QueryAt(lo),QueryAt(hi)))continue;
                // Old endpoints have physical cutter caps too. Restricting
                // coverage to their exact orthogonal projections leaves false
                // parameter gaps at otherwise covered, rounded command joins.
                // Nominate a bounded extension, then certify the ENTIRE extended
                // interval with the same affine whole-profile predicate.
                double delta=Math.Min(-c.A.Z,-c.B.Z)-depth;
                double margin=delta<0?0:RadialMargin(delta,depth)-radialReserve-Reserve;
                if(margin>0)
                {
                    double extension=margin/(2*Math.Sqrt(square));
                    double extendedLo=Math.Max(0,lo-extension),extendedHi=Math.Min(1,hi+extension);
                    if(Covered(c,QueryAt(extendedLo),QueryAt(extendedHi)))
                    {lo=extendedLo;hi=extendedHi;}
                }
                intervals.Add((lo,hi));
            }
        }
        // A long new chord can span several old commanded chords. Every
        // accepted interval has its OWN whole-profile affine certificate;
        // their exact parameter union must cover [0,1] without filling gaps.
        foreach(var block in blocks)Gather(block,block.Root);
        if(intervals.Count>4096)return false;
        intervals.Sort((x,y)=>x.Lo!=y.Lo?x.Lo.CompareTo(y.Lo):x.Hi.CompareTo(y.Hi));
        double end=0;
        foreach(var interval in intervals)
        {
            if(interval.Lo>end)return false;
            end=Math.Max(end,interval.Hi);
            if(end>=1){Accepted++;UnionAccepted++;return true;}
        }
        return false;
    }

    /// <summary>One already-cut floor target, not whole-cutter containment.
    /// Used only to discover floor-fill demand. It cannot authorize omission of
    /// a contact front, connecting motion, or any swept-volume stock credit.</summary>
    internal bool CoversFloorTarget(Point3D point,double tolerance,bool normalDistance=false)
    {
        long started=Stopwatch.GetTimestamp();
        try{return CoversFloorTargetCore(point,tolerance,normalDistance);}
        finally{floorQueryTicks+=Stopwatch.GetTimestamp()-started;}
    }

    private bool CoversFloorTargetCore(Point3D point,double tolerance,bool normalDistance)
    {
        token.ThrowIfCancellationRequested();FloorTargetQueries++;
        if(!supported||!Finite(point))return false;
        double wanted=Math.Max(0,-point.Z-tolerance);
        if(wanted<=0)return true;
        floorQuery=point;floorWanted=wanted;floorTolerance=tolerance;floorNormal=normalDistance;
        if(floorWitnessKnown)
        {
            PriorFloorPoseTrials++;
            // A real earlier pose remains real as committed stock grows. The
            // original predicate rechecks its full current point/radius demand.
            if(floorPredicate(floorWitness)){FloorTargetsCovered++;PriorFloorPoseProofs++;return true;}
        }
        if(maximumDepth>=wanted)
        for(int i=blocks.Count-1;i>=0;i--)
            if(blocks[i].FloorIndex.AnyWithin(point.X,point.Y,
                floorRadius(maximumDepth-wanted),-wanted,token,floorPredicate,floorRadius))
            {FloorTargetIndexHits++;FloorTargetsCovered++;return true;}
        FloorTargetChordFallbacks++;
        bool Search(Block block,Node n)
        {
            if(n.Depth<wanted)return false;
            double reach=floorRadius(n.Depth-wanted);
            double dx=Math.Max(0,Math.Max(n.MinX-point.X,point.X-n.MaxX));
            double dy=Math.Max(0,Math.Max(n.MinY-point.Y,point.Y-n.MaxY));
            if(dx*dx+dy*dy>reach*reach)return false;
            if(n.Left is not null)return Search(block,n.Left)||Search(block,n.Right!);
            for(int i=n.First;i<n.First+n.Count;i++)
            {
                var c=block.Chords[i];double vx=c.B.X-c.A.X,vy=c.B.Y-c.A.Y,l2=vx*vx+vy*vy;
                double t=l2>0?Math.Clamp(((point.X-c.A.X)*vx+(point.Y-c.A.Y)*vy)/l2,0,1):0;
                bool At(double at)
                {
                    double h=-(c.A.Z+(c.B.Z-c.A.Z)*at)-wanted;
                    if(h<0)return false;
                    double r=floorRadius(h);
                    double x=point.X-c.A.X-vx*at,y=point.Y-c.A.Y-vy*at;
                    if(x*x+y*y+Reserve>r*r)return false;
                    floorWitness=new(c.A.X+vx*at,c.A.Y+vy*at,c.A.Z+(c.B.Z-c.A.Z)*at);floorWitnessKnown=true;return true;
                }
                // Each accepted witness is a real pose of this literal chord;
                // no interpolation of unrelated endpoint depths is permitted.
                if(At(t)||At(0)||At(1))return true;
            }
            return false;
        }
        for(int i=blocks.Count-1;i>=0;i--)
            if(Search(blocks[i],blocks[i].Root)){FloorTargetsCovered++;return true;}
        return false;
    }

    /// <summary>Continuous floor-contact circumference in the union of real
    /// earlier cutter poses. Each witness certifies its complete angular arc;
    /// matching sampled endpoints never fills an unexplained gap.</summary>
    internal bool CoversFloorContactRing(Point3D centre,double radius,double tolerance)
    {
        FloorRingQueries++;
        token.ThrowIfCancellationRequested();
        if(!supported||!Finite(centre)||!double.IsFinite(radius)||radius<0)return false;
        double wanted=Math.Max(0,-centre.Z-tolerance);
        if(wanted<=0){FloorRingCovered++;return true;}
        if(radius<=1e-9)return CoversFloorTarget(centre,tolerance,true);
        const double tau=2*Math.PI;
        double at=0;
        for(int proof=0;proof<64;proof++)
        {
            var point=new Point3D(centre.X+radius*Math.Cos(at),centre.Y+radius*Math.Sin(at),centre.Z);
            if(!CoversFloorTarget(point,tolerance,true))return false;
            var old=floorWitness;
            double oldRadius=OffsetRadius(-old.Z+centre.Z,tolerance);
            // Point admission has a larger reserve than its certified arc.
            // At an arc end the same pose cannot mask the next needed witness.
            double oldSquare=oldRadius*oldRadius-Reserve/2;
            double x=old.X-centre.X,y=old.Y-centre.Y,distance=Math.Sqrt(x*x+y*y);
            if(oldSquare>=(distance+radius)*(distance+radius)){FloorRingCovered++;return true;}
            if(distance<=1e-12)return false;
            double cosine=(radius*radius+distance*distance-oldSquare)/(2*radius*distance);
            if(cosine>=1)return false;
            double half=Math.Acos(Math.Clamp(cosine,-1,1)),middle=Math.Atan2(y,x);
            middle+=tau*Math.Round((at-middle)/tau);
            double lo=middle-half,hi=middle+half;
            if(lo>at||hi<=at+1e-12)return false;
            if(hi>=tau){FloorRingCovered++;return true;}
            at=hi;
        }
        FloorRingBudgetFallbacks++;return false;
    }

    /// <summary>Cross-section of the physical cutter dilated by a sphere.
    /// A witness is at most clearance away in Euclidean distance, rather than
    /// only along Z. Sections and their endpoint caps are considered exactly.</summary>
    internal double OffsetRadius(double height,double clearance)
    {
        if(clearance<=0)return height>=0?profile.PhysicalRadiusAtDepth(height):0;
        if(height < -clearance)return 0;
        double answer=0,bottomHeight=0;
        foreach(var section in profile.Sections)
        {
            double topHeight=bottomHeight+section.Height;
            if(section.Kind==CarvingCutterSectionKind.Hemisphere)
            {
                double r=section.TopRadius;
                if(height<=topHeight)
                {
                    double delta=height-topHeight;
                    answer=Math.Max(answer,Math.Sqrt(Math.Max(0,(r+clearance)*(r+clearance)-delta*delta)));
                }
                else if(height-topHeight<=clearance)
                    answer=Math.Max(answer,r+Math.Sqrt(Math.Max(0,clearance*clearance-Math.Pow(height-topHeight,2))));
            }
            else
            {
                double slope=(section.TopRadius-section.BottomRadius)/section.Height;
                double normal=Math.Sqrt(1+slope*slope);
                double at=height+clearance*slope/normal;
                if(at>=bottomHeight&&at<=topHeight)
                    answer=Math.Max(answer,section.BottomRadius+slope*(height-bottomHeight)+clearance*normal);
                else
                {
                    at=Math.Clamp(at,bottomHeight,topHeight);double delta=height-at;
                    if(Math.Abs(delta)<=clearance)
                        answer=Math.Max(answer,section.BottomRadius+slope*(at-bottomHeight)
                            +Math.Sqrt(Math.Max(0,clearance*clearance-delta*delta)));
                }
            }
            bottomHeight=topHeight;
        }
        // The profile's final radius continues up the shank, as in the
        // existing physical radius owner used for every stock-height section.
        if(height>=bottomHeight)answer=Math.Max(answer,profile.PhysicalRadiusAtDepth(height)+clearance);
        return answer;
    }

    // Minimum R(h+delta)-R(h) over the ENTIRE query cutter below stock.
    // Linear/linear pieces attain minima at ends. The one remaining case,
    // linear above a spherical nose, also tests its analytic stationary point.
    internal double RadialMargin(double delta,double depth)
    {
        if(!supported||delta<0||depth<0||depth+delta>joints[^1])return double.NegativeInfinity;
        double answer=double.PositiveInfinity;
        void Test(double h)
        {if(h>=0&&h<=depth)answer=Math.Min(answer,profile.PhysicalRadiusAtDepth(h+delta)-profile.PhysicalRadiusAtDepth(h));}
        Test(0);Test(depth);
        foreach(double h in joints){Test(h);Test(h-delta);}
        if(profile.RoundedTip)
        {
            double radius=profile.Sections[0].TopRadius;
            for(int i=1;i<profile.Sections.Count;i++)
            {
                var section=profile.Sections[i];double slope=(section.TopRadius-section.BottomRadius)/section.Height;
                double h=radius*(1-slope/Math.Sqrt(1+slope*slope));
                if(h<=radius&&h+delta>=joints[i-1]&&h+delta<=joints[i])Test(h);
            }
        }
        return answer;
    }

    private static bool Finite(Point3D p)=>double.IsFinite(p.X+p.Y+p.Z)&&Math.Abs(p.X)<=10000&&Math.Abs(p.Y)<=10000&&Math.Abs(p.Z)<=100;
}
