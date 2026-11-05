using System.Diagnostics;
using Clipper2Lib;
using RW.Toolpaths.Geometry;

namespace RW.Toolpaths.Milling;

/// <summary>Construct a wall's aligned offset family before routing. Original
/// protected work remains the coverage authority; neighbours grant omission
/// evidence only, never executed-stock or travel permission.</summary>
internal static class VCarveWallContours
{
    internal sealed record Review(int Wall,IReadOnlyList<TaggedToolpath> Original,
        IReadOnlyList<TaggedToolpath> Rows,IReadOnlyList<TaggedToolpath> Neighbours,
        IReadOnlyList<IReadOnlyList<PointD>> Boundary,ToolGeometry Tool,DepthSchedule Depth,VCarveOptions Options,
        Func<Point3D,Point3D,bool> Guard,int Supports,int RequiredIntervals,int AttachedSupports);
    internal static readonly AsyncLocal<Func<int,IReadOnlyList<PointD>,bool>?> Selection=new();
    internal static readonly AsyncLocal<Action<Review>?> Observer=new();
    internal static readonly AsyncLocal<bool?> AttachJunctionsOverride=new();
    internal static readonly AsyncLocal<double?> NominationPitchScale=new();
    internal static readonly AsyncLocal<bool?> UnionProofOverride=new();
    internal sealed record Context(IReadOnlyList<TaggedToolpath> Source,IReadOnlyList<IReadOnlyList<PointD>> Boundary,
        ToolGeometry Tool,DepthSchedule Depth,VCarveOptions Options,
        Func<IReadOnlyList<TaggedToolpath>,List<TaggedToolpath>> Protect,Func<Point3D,Point3D,bool> Guard);
    internal static readonly AsyncLocal<Action<Context>?> ContextObserver=new();

    internal static List<TaggedToolpath> Replace(IReadOnlyList<TaggedToolpath> source,
        IReadOnlyList<IReadOnlyList<PointD>> boundary,ToolGeometry tool,DepthSchedule depth,VCarveOptions options,
        Func<IReadOnlyList<TaggedToolpath>,List<TaggedToolpath>> protect,Func<Point3D,Point3D,bool> guard,
        CancellationToken token,Action<string>? log)
    {
        var select=Selection.Value;if(select is null||boundary.Count<2)return source.ToList();
        var profile=VCarvePhysicalSweep.ProfileFor(tool,depth.Depth);
        var band=ConcentricBandSchedule.Ramp(profile,depth.Depth,options);
        double grade=band.Grade,first=band.First,reach=band.Reach,pitch=band.Pitch;
        double diameter=2*profile.PhysicalRadiusAtDepth(depth.Depth);int originalLevels=band.Levels,levels=originalLevels;
        double scale=NominationPitchScale.Value??1;
        if(!double.IsFinite(scale)||scale<1||scale>2)throw new ArgumentOutOfRangeException(nameof(NominationPitchScale));
        if(scale>1&&reach>first){pitch=ConcentricBandSchedule.FitPitch(reach-first,pitch*scale,RingOffsetEngine.AbsoluteRingLimit-1);
            levels=checked((int)Math.Floor((reach-first)/pitch+1e-10))+1;}
        if(reach<=first)return source.ToList();
        if(levels<2||levels>RingOffsetEngine.AbsoluteRingLimit)throw new InvalidOperationException("Wall contour inventory is outside the shared bound.");
        bool Slope(TaggedToolpath p)=>p.ConcentricSource is {} row&&row.Plan.Levels[row.Ring]<originalLevels;
        var unchanged=source.Where(p=>!Slope(p)).ToList();
        var owned=VCarveWallSources.Assign(source.Where(Slope).ToArray(),token,log);
        var current=owned.ToList();double allowance=options.PathToleranceMm/Math.Sqrt(2);
        Point3D Post(Point3D p)=>new(ToolpathCommandRounding.Round((float)p.X),ToolpathCommandRounding.Round((float)p.Y),
            ToolpathCommandRounding.Round((float)(p.Z+depth.SurfaceZ)-(float)depth.SurfaceZ));

        foreach(int wall in Enumerable.Range(1,boundary.Count-1)){
            token.ThrowIfCancellationRequested();if(!select(wall,boundary[wall]))continue;
            long started=Stopwatch.GetTimestamp();
            var original=current.Where(p=>p.ConcentricSource?.Wall==wall).ToArray();
            if(original.Length==0)continue;
            var outline=boundary[wall].ToArray();if(PolygonOps.SignedArea(outline)<0)Array.Reverse(outline);
            var otherIndex=new PolygonOps.BoundarySegmentIndex(PathUtils.ToClipper(boundary.Where((_,i)=>i!=wall)));
            var crest=outline.OrderByDescending(p=>otherIndex.DistanceSquared(new Point64((long)Math.Round(p.x*PathUtils.Scale),(long)Math.Round(p.y*PathUtils.Scale))))
                .ThenBy(p=>p.x).ThenBy(p=>p.y).First();
            var nominal=new List<TaggedToolpath>();var rowLevels=new List<int>();var parents=new List<int>();int previous=-1;
            int extraNominationLoops=0;double extraNominationLength=0;
            for(int level=0;level<levels;level++){
                int firstAtLevel=nominal.Count;
                // Dilation of one connected retained island has one exterior.
                // Conservative offset repair can also return microscopic debris
                // cycles. They are not additional walls to machine. Nominate
                // the exterior; the original-removal proof below owns every
                // omitted junction or genuine enclosed feature.
                var loops=PolygonOps.Offset([outline],first+level*pitch,.001,0)
                    .OrderByDescending(p=>Math.Abs(PolygonOps.SignedArea(p))).ToArray();
                foreach(var extra in loops.Skip(1)){
                    extraNominationLoops++;for(int i=0;i<extra.Count;i++){
                        var a=extra[i];var b=extra[(i+1)%extra.Count];
                        extraNominationLength+=Math.Sqrt(Math.Pow(a.x-b.x,2)+Math.Pow(a.y-b.y,2));}}
                foreach(var loop in loops.Take(1)){
                    var points=loop.Append(loop[0]).Select(p=>new Point3D(p.x,p.y,-depth.Depth)).ToArray();
                    if(points.Length<4)continue;
                    nominal.Add(new(points,[],original[0].RegionIndex,"clearing",null,detectCurves:false));
                    rowLevels.Add(level);parents.Add(previous);
                }
                if(nominal.Count>firstAtLevel)previous=firstAtLevel;
                if(nominal.Sum(p=>(long)p.Points.Count)>1_000_000)throw new InvalidOperationException("Wall contour nomination exceeds the existing point bound.");
            }
            var plan=new VCarveRingOrder.Plan(parents,insideOut:options.ProfileDirection==VCarveProfileDirection.BottomUp,levels:rowLevels,walls:boundary);
            var input=nominal.Select((p,i)=>p with{ConcentricSource=new(plan,i,Wall:wall)}).ToArray();
            var protectedRows=protect(input);
            if(options.AccessTarget is {} access){
                var inserted=new List<int>();protectedRows=access.Trim(protectedRows,profile.PhysicalRadiusAtDepth,token,retainContactingChords:true,interpolatedFragments:inserted);
                if(inserted.Count>0){var repaired=protect(inserted.Select(i=>protectedRows[i]).ToArray());
                    if(repaired.Count!=inserted.Count)throw new InvalidOperationException("Wall contour clipping changed physical-protection ordinals.");
                    for(int i=0;i<inserted.Count;i++)protectedRows[inserted[i]]=repaired[i];}
            }
            protectedRows=protectedRows.Select(p=>SurfaceToolpathSimplifier.Simplify(p,options.OutputChordToleranceMm,token,guard)).ToList();
            // The seam seed is geometry-derived and shared across the complete
            // family. It is not a saved fixture seam or execution permission.
            protectedRows=protectedRows.Select(p=>VCarveContinuationAdapter.Rebase(p,
                ToolpathVisitOrder.EntryIndex(p,new(crest.x,crest.y,0),token))).ToList();
            var classified=VCarveWallSources.Assign(protectedRows,token,log);
            var rows=classified.Where(p=>p.ConcentricSource?.Wall==wall).ToList();
            var neighbours=current.Where(p=>p.ConcentricSource?.Wall!=wall).Concat(unchanged).ToArray();
            double radius=2*profile.PhysicalRadiusAtDepth(depth.Depth)+options.PathToleranceMm;
            var allPoints=original.Concat(rows).SelectMany(p=>p.Points).ToArray();
            double minX=allPoints.Min(p=>p.X)-radius,maxX=allPoints.Max(p=>p.X)+radius,
                minY=allPoints.Min(p=>p.Y)-radius,maxY=allPoints.Max(p=>p.Y)+radius;
            bool Nearby(TaggedToolpath p)=>p.Points.Count>0&&p.Points.Min(q=>q.X)<=maxX&&p.Points.Max(q=>q.X)>=minX
                &&p.Points.Min(q=>q.Y)<=maxY&&p.Points.Max(q=>q.Y)>=minY;
            var near=neighbours.Where(Nearby).ToArray();
            var nativeStock=new CutterSweepStock(profile,p=>p,depth.SurfaceZ,token);
            var postedStock=new CutterSweepStock(profile,p=>p,depth.SurfaceZ,token);
            void Record(TaggedToolpath p){
                var cursor=Post(p.Points[0]);
                for(int i=1;i<p.Points.Count;i++){
                    var a=p.Points[i-1];var b=p.Points[i];var next=Post(b);
                    bool vertical=ToolpathCommandRounding.IsVerticalOnly((float)b.X-(float)a.X,(float)(b.Z+depth.SurfaceZ)-(float)(a.Z+depth.SurfaceZ),(float)b.Y-(float)a.Y);
                    if(vertical)next=new(cursor.X,cursor.Y,next.Z);
                    else{nativeStock.RecordExecuted(a,b);postedStock.RecordExecuted(cursor,next);}
                    cursor=next;
                }
            }
            foreach(var p in near.Concat(rows))Record(p);nativeStock.Seal();postedStock.Seal();
            VCarveRemovalUnion? unionNative=null,unionPosted=null;
            TaggedToolpath Actual(TaggedToolpath p,bool posted){
                var points=new List<Point3D>();Point3D cursor=default;
                for(int i=0;i<p.Points.Count;i++){
                    var q=p.Points[i];var next=posted?Post(q):new Point3D((float)q.X,(float)q.Y,(float)(q.Z+depth.SurfaceZ)-(float)depth.SurfaceZ);
                    if(i>0){var prev=p.Points[i-1];if(ToolpathCommandRounding.IsVerticalOnly((float)q.X-(float)prev.X,(float)(q.Z+depth.SurfaceZ)-(float)(prev.Z+depth.SurfaceZ),(float)q.Y-(float)prev.Y))next=new(cursor.X,cursor.Y,next.Z);}
                    points.Add(next);cursor=next;
                }
                return new(points.ToArray(),p.Spans,p.RegionIndex,p.Category,p.DepthPassIndex,detectCurves:false);
            }
            bool Union(Point3D a,Point3D b,bool posted){
                if(UnionProofOverride.Value!=true)return false;
                VCarveRemovalUnion stock;
                if(posted)stock=unionPosted??=new(profile,near.Concat(rows).Select(p=>Actual(p,true)).ToArray(),.001,token);
                else stock=unionNative??=new(profile,near.Concat(rows).Select(p=>Actual(p,false)).ToArray(),.001,token);
                int n=Math.Max(1,(int)Math.Ceiling(Math.Abs(a.Z-b.Z)/.01));
                Point3D At(double t)=>new(a.X+t*(b.X-a.X),a.Y+t*(b.Y-a.Y),a.Z+t*(b.Z-a.Z));
                for(int j=0;j<n;j++)if(!stock.Contains(j==0?a:At((double)j/n),j+1==n?b:At((double)(j+1)/n),allowance,allowance))return false;
                return true;
            }
            bool Covered(Point3D a,Point3D b)=>
                (nativeStock.Contains(a,b,allowance,-allowance)||Union(a,b,false))
                &&(postedStock.Contains(Post(a),Post(b),allowance,-allowance)||Union(Post(a),Post(b),true));
            int supports=0,required=0;var requiredSupports=new List<TaggedToolpath>();
            foreach(var p in original){
                List<Point3D>? run=null;
                void Flush(){if(run is {Count:>1}){
                    int level=p.ConcentricSource!.Plan.Levels[p.ConcentricSource.Ring];
                    int row=Enumerable.Range(0,rowLevels.Count).OrderBy(i=>Math.Abs(rowLevels[i]*pitch-level*band.Pitch)).First();
                    var points=run.ToArray();requiredSupports.Add(new(points,[],p.RegionIndex,"clearing",null,detectCurves:false){ConcentricSource=new(plan,row,Wall:wall)});
                    supports++;}run=null;}
                for(int i=1;i<p.Points.Count;i++){
                    token.ThrowIfCancellationRequested();required++;var a=p.Points[i-1];var b=p.Points[i];
                    bool covered=Covered(a,b);
                    if(covered){Flush();continue;}
                    run??=new(){a};run.Add(b);
                }
                Flush();
            }
            // Supports remain literal original work. They restore junctions;
            // they are not a second copied wall or future travel stock.
            foreach(var p in requiredSupports)Record(p);
            nativeStock.Seal();postedStock.Seal();int missing=0;
            foreach(var p in original)for(int i=1;i<p.Points.Count;i++)
                if(!Covered(p.Points[i-1],p.Points[i]))missing++;
            if(missing>0)throw new InvalidOperationException($"Constructed wall {wall} lost {missing} original removal intervals.");
            int attached=0;
            if(AttachJunctionsOverride.Value!=false){
                double step=depth.DepthPerPass>0?depth.DepthPerPass:depth.Depth;
                bool Allows(Point3D a,Point3D b){
                    double engaged=2*profile.PhysicalRadiusAtDepth(Math.Max(0,-Math.Min(a.Z,b.Z)));
                    double radial=profile.RoundedTip?profile.NoseLimitedPitch(engaged,options.StepOver):engaged*options.StepOver;
                    return ToolpathEngagementLimits.Allows(a,b,radial,step,grade,Post)&&guard(a,b);
                }
                rows=ToolpathSourceTraversal.JunctionDetours(rows,requiredSupports,diameter*options.StepOver,
                    (a,b)=>a.ConcentricSource is {} x&&b.ConcentricSource is {} y&&ReferenceEquals(x.Plan,y.Plan)&&x.Ring==y.Ring,
                    Allows,token,log,out attached);
            }else rows.AddRange(requiredSupports);
            log?.Invoke($"stage=wall-contour-construction event=certified wall={wall} originalRuns={original.Length} nominalRows={nominal.Count} ownedRuns={rows.Count} supports={supports} attachedSupports={attached} "
                +$"extraNominationLoops={extraNominationLoops} extraNominationLengthMm={extraNominationLength:R} requiredIntervals={required} missing={missing} levels={levels} originalLevels={originalLevels} nominationScale={scale:R} unionProof={UnionProofOverride.Value==true} unionCells={unionNative?.Cells??0}/{unionPosted?.Cells??0} unionBudgetRefusals={unionNative?.BudgetRefusals??0}/{unionPosted?.BudgetRefusals??0} firstInset={first:R} pitch={pitch:R} nativeAndF3Removal=True "
                +$"protectionOwner=original-production-floor-artwork-channel-grade neighbourRemovalOnly=True travelCredit=False elapsedMs={Stopwatch.GetElapsedTime(started).TotalMilliseconds:F1}");
            Observer.Value?.Invoke(new(wall,original,rows,near,boundary,tool,depth,options,guard,supports,required,attached));
            current.RemoveAll(p=>p.ConcentricSource?.Wall==wall);current.AddRange(rows);
        }
        // Routing follows each constructed strip. Wall-wide level batching was
        // independently rejected; it is not the authority for these families.
        return unchanged.Concat(current).Select(p=>p.ConcentricSource is {} row?p with{ConcentricSource=row with{Wall=-1},ContinuationSourceOrdinal=null}:p).ToList();
    }
}
