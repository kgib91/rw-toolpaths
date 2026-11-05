using Clipper2Lib;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class VCarveDiscSweepTests
{
    private static IReadOnlyList<IReadOnlyList<PointD>> Square(double size) =>
        [[new(0,0),new(size,0),new(size,size),new(0,size)]];

    [Theory]
    [InlineData(5)]
    [InlineData(10)] // Exactly fitting disc: the legal centre set collapses.
    public void SquareSubtractsTheDiscSweepAndLeavesCornersRatherThanAnEdgeBand(double radius)
    {
        var logs=new List<string>();
        var mask=new VCarveDiscSweepExclusion(Square(20),radius,default,logs.Add,"test-square");
        double remainder=400-Math.Abs(mask.SweptRegion.Sum(PolygonOps.SignedArea));
        Assert.InRange(remainder,(4-Math.PI)*radius*radius-.03,(4-Math.PI)*radius*radius+.04);
        var corners=mask.Fill(Square(20),mask.SweptRegion,.5,20,default,logs.Add);
        Assert.NotEmpty(corners);
        Assert.All(corners,r=>Assert.True(r.IsClosed));
        Assert.All(corners,r=>Assert.All(r.Points,p=>Assert.True(p.x<radius||p.x>20-radius||p.y<radius||p.y>20-radius)));
        Assert.Contains(logs,s=>s.Contains("radiusNotDiameter=True")&&s.Contains("stockCredit=False"));
    }

    [Fact]
    public void DiscTooLargeToFitExcludesNothingAndRetainsTheWholeFillRegion()
    {
        var mask=new VCarveDiscSweepExclusion(Square(4),3,default,null,"test-narrow");
        Assert.Empty(mask.SweptRegion);
        var fill=mask.Fill(Square(4),Array.Empty<IReadOnlyList<PointD>>(),.5,4,default,null);
        Assert.True(fill.Count>=4);
        Assert.All(fill,r=>Assert.True(r.IsClosed));
    }

    [Fact]
    public void IslandsRemainExcludedFromTheVirtualSweptRegion()
    {
        IReadOnlyList<IReadOnlyList<PointD>> domain=[Square(20)[0],
            new PointD[]{new(8,8),new(8,12),new(12,12),new(12,8)}];
        var mask=new VCarveDiscSweepExclusion(domain,2,default,null,"test-island");
        Assert.Empty(PolygonOps.Intersect(mask.SweptRegion,
            [new PointD[]{new(8.1,8.1),new(11.9,8.1),new(11.9,11.9),new(8.1,11.9)}]));
    }

    [Fact]
    public void SmoothExclusionRetainsEverySlopeRingAndFiltersOnlyTheExistingFloorBand()
    {
        var profile=new VCarveCutterProfile([
            new(CarvingCutterSectionKind.Hemisphere,0,.5,.5),
            new(CarvingCutterSectionKind.Linear,.5,1.5,10)],10.5);
        var boundary=Square(40);
        var options=new VCarveOptions {SmoothVCarving=true,SmoothRampAngleDegrees=30,
            IncludeInteriorFill=false,EdgeFillWidth=2,StepOver=.4};
        double front=profile.RampFillWidth(2,30);
        var original=VCarveRampSurface.Contours(boundary,options,profile,2,0,default,null);
        var roughing=RectangleSweep(Math.Ceiling((front+5)*1000)/1000,40-Math.Ceiling((front+5)*1000)/1000,5,2);
        var filtered=VCarveRampSurface.Contours(boundary,options with {ExcludeDiscSweep=true,DiscRadius=5,RoughingRestChords=roughing},profile,2,0,default,null);
        double Distance(Point3D p)=>Math.Min(Math.Min(p.X,40-p.X),Math.Min(p.Y,40-p.Y));
        bool Slope(TaggedToolpath path)=>path.Points.Min(Distance)<=front+1/PathUtils.Scale;
        Assert.All(original.Where(Slope),p=>Assert.Contains(filtered,q=>p.Points.SequenceEqual(q.Points)));
        var floor=filtered.Where(p=>!Slope(p)).ToArray();
        Assert.NotEmpty(floor);
        Assert.All(filtered,p=>Assert.Equal(p.Points[0],p.Points[^1]));
        Assert.All(floor,p=>Assert.All(p.Points,v=>Assert.InRange(Distance(v)-front,0,2.003)));
        // Off is byte-for-byte source geometry, regardless of the hidden radius.
        var off=VCarveRampSurface.Contours(boundary,options with {DiscRadius=100},profile,2,0,default,null);
        Assert.Equal(original.Select(p=>p.Points.ToArray()),off.Select(p=>p.Points.ToArray()),new PointsComparer());
        var noFill=options with {EdgeFillWidth=0};
        Assert.Equal(VCarveRampSurface.Contours(boundary,noFill,profile,2,0,default,null).Select(p=>p.Points.ToArray()),
            VCarveRampSurface.Contours(boundary,noFill with {ExcludeDiscSweep=true,DiscRadius=5},profile,2,0,default,null)
                .Select(p=>p.Points.ToArray()),new PointsComparer());
    }

    [Fact]
    public void OrdinaryFillKeepsMandatoryPerimeterAndSubtractsOnlyOptionalRows()
    {
        List<TaggedToolpath> Generate(double radius)=>MedialAxisToolpaths.GenerateClearingPassesTagged(
            Square(40),0,1,Math.PI/2,1,.4,includeInteriorFill:false,detectCurves:false,
            edgeFillWidth:8,excludedDiscRadius:radius,roughingChords:RectangleSweep(5,35,5,1));
        var original=Generate(0); var filtered=Generate(5);
        Assert.NotEmpty(filtered);
        Assert.Equal(original[0].Points,filtered[0].Points);
        Assert.True(filtered.Sum(p=>Length(p.Points))<original.Sum(p=>Length(p.Points))*.4);
        Assert.All(filtered,p=>Assert.Equal(p.Points[0],p.Points[^1]));
        Assert.All(filtered,p=>Assert.All(p.Points,v=>Assert.True(Math.Min(Math.Min(v.X,40-v.X),Math.Min(v.Y,40-v.Y))<=8.001)));
    }

    [Fact]
    public void ExactSleeveCanExcludeADiscWithoutChangingItsFillLimitOrSlope()
    {
        using var resource=typeof(VCarveDiscSweepTests).Assembly.GetManifestResourceStream(
            "RW.Toolpaths.Tests.Fixtures.vcarve-sleeve-pocket.json.gz")!;
        using var gzip=new System.IO.Compression.GZipStream(resource,System.IO.Compression.CompressionMode.Decompress);
        using var document=System.Text.Json.JsonDocument.Parse(gzip);
        var root=document.RootElement;
        var boundary=root.GetProperty("boundary").EnumerateArray().Select(r=>(IReadOnlyList<PointD>)r.EnumerateArray()
            .Select(p=>new PointD(p[0].GetDouble(),p[1].GetDouble())).ToArray()).ToArray();
        var profile=new VCarveCutterProfile([
            new(CarvingCutterSectionKind.Hemisphere,0,1.5875,1.5875),
            new(CarvingCutterSectionKind.Linear,1.5875,3,20)],21.5875);
        double limit=root.GetProperty("fillLimit").GetDouble();
        var options=new VCarveOptions {SmoothVCarving=true,SmoothRampAngleDegrees=50,StepOver=.45,
            IncludeInteriorFill=false,EdgeFillWidth=limit,RoughingRestChords=SleeveStock()};
        var original=VCarveRampSurface.Contours(boundary,options,profile,6.35,0,default,null);
        var logs=new List<string>();
        var filtered=VCarveRampSurface.Contours(boundary,options with {ExcludeDiscSweep=true,DiscRadius=3.175},profile,6.35,0,default,logs.Add);
        Assert.All(filtered,p=>Assert.Equal(p.Points[0],p.Points[^1]));
        Assert.True(Nearest(filtered,new(142.7752095833334,164.34456854281316))<.86,"The unroughed lower lobe must retain complete cleanup.");
        Assert.True(Nearest(filtered,new(149.7,153.3))>3,"The actual roughed circle must remain a hole in the fill region.");
        Assert.Contains(filtered,p=>p.Points.Min(v=>v.Y)<146&&p.Points.Max(v=>v.Y)<151&&p.Points.Count>8);
        Assert.Contains(filtered,p=>p.Points.Min(v=>v.Y)>157&&p.Points.Max(v=>v.Y)>170);
        Assert.Contains(logs,s=>s.Contains("stage=rest-fill event=contoured")&&s.Contains("closedLoops=True"));
        Assert.Contains(logs,s=>s.Contains("reference=shared-material-plateau")&&s.Contains("radius=3.175"));
        Assert.Contains(logs,s=>s.Contains("nearestEdgeLimit=6.068546"));
        var tooLarge=VCarveRampSurface.Contours(boundary,options with {ExcludeDiscSweep=true,DiscRadius=limit},profile,6.35,0,default,null);
        Assert.All(tooLarge,p=>Assert.Equal(p.Points[0],p.Points[^1]));
        Assert.True(Nearest(tooLarge,new(142.7752095833334,164.34456854281316))<.86);
        if (Environment.GetEnvironmentVariable("RW_POST_SLOPE_FILL_CAPTURE") is { } directory)
        {
            Directory.CreateDirectory(directory);
            File.WriteAllText(Path.Combine(directory,"rest-exact-sleeve.json"),System.Text.Json.JsonSerializer.Serialize(new {
                fillLimit=limit,discRadius=3.175,floorInset=profile.RampFillWidth(6.35,50),
                boundary=boundary.Select(r=>r.Select(p=>new[]{p.x,p.y})),
                original=original.Select(p=>p.Points.Select(v=>new[]{v.X,v.Y,v.Z})),
                filtered=filtered.Select(p=>p.Points.Select(v=>new[]{v.X,v.Y,v.Z})),logs}));
        }
    }

    [Fact]
    public void FootPocketExcludesRealRoughingEvenWhenTheFinisherCentreFrontCannotFitTheDisc()
    {
        // Original project: space right of the foot, source crop 35..100,260..330.
        // The actual 3.175 mm rougher fits; the supported centre front's maximum
        // clearance is only 2.804 mm. Disc fit must use the shared material floor.
        using var resource=typeof(VCarveDiscSweepTests).Assembly.GetManifestResourceStream(
            "RW.Toolpaths.Tests.Fixtures.vcarve-foot-pocket.json.gz")!;
        using var gzip=new System.IO.Compression.GZipStream(resource,System.IO.Compression.CompressionMode.Decompress);
        using var document=System.Text.Json.JsonDocument.Parse(gzip);
        var root=document.RootElement;
        var boundary=root.GetProperty("boundary").EnumerateArray().Select(r=>(IReadOnlyList<PointD>)r.EnumerateArray()
            .Select(p=>new PointD(p[0].GetDouble(),p[1].GetDouble())).ToArray()).ToArray();
        var chords=root.GetProperty("chords").EnumerateArray().Select(p=>new VCarveRoughingChord(
            new(p[0].GetDouble(),p[1].GetDouble(),p[2].GetDouble()),
            new(p[3].GetDouble(),p[4].GetDouble(),p[5].GetDouble()),p[6].GetDouble(),p[6].GetDouble())).ToArray();
        var profile=new VCarveCutterProfile([
            new(CarvingCutterSectionKind.Hemisphere,0,1.5875,1.5875),
            new(CarvingCutterSectionKind.Linear,1.5875,3,20)],21.5875);
        var options=new VCarveOptions {SmoothVCarving=true,SmoothRampAngleDegrees=50,StepOver=.45,
            IncludeInteriorFill=false,EdgeFillWidth=root.GetProperty("fillLimit").GetDouble(),
            ExcludeDiscSweep=true,DiscRadius=3.175,RoughingRestChords=chords};
        var slopes=VCarveRampSurface.Contours(boundary,options with {EdgeFillWidth=0},profile,6.35,0,default,null);
        var logs=new List<string>();
        var filtered=VCarveRampSurface.Contours(boundary,options,profile,6.35,0,default,logs.Add);
        Assert.All(slopes,p=>Assert.Contains(filtered,q=>p.Points.SequenceEqual(q.Points)));
        Assert.All(filtered,p=>Assert.Equal(p.Points[0],p.Points[^1]));
        var cleanup=filtered.Where(p=>!slopes.Any(q=>p.Points.SequenceEqual(q.Points))).ToArray();
        var actual=new VCarvePostedFloorSweep(chords).AtDepth(6.35,default,null);
        var actualIndex=new PolygonOps.BoundarySegmentIndex(PathUtils.ToClipper(actual));
        var witness=new PointD(63.5,300);
        var site=new Point64((long)(witness.x*PathUtils.Scale),(long)(witness.y*PathUtils.Scale));
        Assert.True(actualIndex.IsInside(site));
        Assert.True(Math.Sqrt(actualIndex.DistanceSquared(site))/PathUtils.Scale>2.5);
        Assert.True(Nearest(cleanup,witness)>2.5,"Optional fill must not enter the already roughed foot pocket.");
        Assert.NotEmpty(cleanup);
        // No evidence still keeps the same real floor work; fit alone grants no removal.
        var unknown=VCarveRampSurface.Contours(boundary,options with {RoughingRestChords=[]},profile,6.35,0,default,null);
        Assert.True(Nearest(unknown,witness)<.86);
    }

    [Fact]
    public void RestAreaLoopsStillPassNativeAndPostedPhysicalGuards()
    {
        var profile=new VCarveCutterProfile([
            new(CarvingCutterSectionKind.Hemisphere,0,.5,.5),
            new(CarvingCutterSectionKind.Linear,.5,1.5,10)],10.5);
        var options=new VCarveOptions {SmoothVCarving=true,SmoothRampAngleDegrees=30,StepOver=.4,
            IncludeInteriorFill=false,EdgeFillWidth=2,ExcludeDiscSweep=true,DiscRadius=3,
            CalculateSweptArea=false,DetectCurves=false};
        var paths=VCarveSmoothing.Apply([],Square(20),new ToolGeometry(1.5){VCarveProfile=profile},
            new(2,1){SurfaceZ=12.7},options,default,null);
        Assert.NotEmpty(paths);
        foreach(var path in VCarveSmoothing.CommandPaths(paths,12.7))
        foreach(var p in path.Points)
        {
            double distance=Math.Min(Math.Min(p.X,20-p.X),Math.Min(p.Y,20-p.Y));
            Assert.InRange(-p.Z,0,Math.Min(2,distance*Math.Tan(Math.PI/6))+.003);
            Assert.True(distance+.003>=profile.PhysicalRadiusAtDepth(-p.Z));
        }
    }

    [Fact]
    public void MissingOrShallowRoughingEvidenceCannotExcludeDeepFloorMaterial()
    {
        var mask=new VCarveDiscSweepExclusion(Square(20),5,default,null,"no-stock");
        var stock=new VCarvePostedFloorSweep([new(new(10,10,-1),new(10,10,-1),5,5)]);
        var cleared=stock.AtDepth(2,default,null);
        Assert.Empty(cleared);
        var rows=mask.Fill(Square(20),cleared,1,20,default,null);
        Assert.Contains(rows,r=>r.Points.All(p=>p.x>=8&&p.x<=12&&p.y>=8&&p.y<=12));
        Assert.All(rows,r=>Assert.True(r.IsClosed));
    }

    [Fact]
    public void PostedFloorEvidenceIncludesBothDeepAndShallowerCompletedRoughing()
    {
        var stock=new VCarvePostedFloorSweep([
            new(new(5,5,-12.7),new(5,5,-12.7),1,1),
            new(new(10,10,-6.35),new(10,10,-6.35),1,1),
            new(new(15,15,-1),new(15,15,-1),1,1)]);
        var floor=stock.AtDepth(6.35,default,null);
        var index=new PolygonOps.BoundarySegmentIndex(PathUtils.ToClipper(floor));
        Point64 At(double v)=>new((long)(v*PathUtils.Scale),(long)(v*PathUtils.Scale));
        Assert.True(index.IsInside(At(5)));
        Assert.True(index.IsInside(At(10)));
        Assert.False(index.IsInside(At(15)));
        Assert.Same(floor,stock.AtDepth(6.35,default,null));
        var deep=new PolygonOps.BoundarySegmentIndex(PathUtils.ToClipper(stock.AtDepth(12.7,default,null)));
        Assert.True(deep.IsInside(At(5)));
        Assert.False(deep.IsInside(At(10)));
    }

    internal static VCarveRoughingChord[] SleeveStock()
    {
        using var resource=typeof(VCarveDiscSweepTests).Assembly.GetManifestResourceStream(
            "RW.Toolpaths.Tests.Fixtures.vcarve-sleeve-roughing.json.gz")!;
        using var gzip=new System.IO.Compression.GZipStream(resource,System.IO.Compression.CompressionMode.Decompress);
        using var doc=System.Text.Json.JsonDocument.Parse(gzip);
        return doc.RootElement.EnumerateArray().Select(p=>new VCarveRoughingChord(
            new(p[0].GetDouble(),p[1].GetDouble(),p[2].GetDouble()),
            new(p[3].GetDouble(),p[4].GetDouble(),p[5].GetDouble()),p[6].GetDouble(),p[6].GetDouble())).ToArray();
    }

    private static VCarveRoughingChord[] RectangleSweep(double low,double high,double radius,double depth)
    {
        Point3D[] p=[new(low,low,-depth),new(high,low,-depth),new(high,high,-depth),new(low,high,-depth)];
        return Enumerable.Range(0,4).Select(i=>new VCarveRoughingChord(p[i],p[(i+1)%4],radius,radius)).ToArray();
    }

    private static double Nearest(IEnumerable<TaggedToolpath> paths,PointD site)=>paths.SelectMany(p=>p.Points.Zip(p.Points.Skip(1),(a,b)=> {
        double dx=b.X-a.X,dy=b.Y-a.Y;
        double t=dx*dx+dy*dy==0?0:Math.Clamp(((site.x-a.X)*dx+(site.y-a.Y)*dy)/(dx*dx+dy*dy),0,1);
        return Math.Sqrt(Math.Pow(site.x-a.X-t*dx,2)+Math.Pow(site.y-a.Y-t*dy,2));
    })).DefaultIfEmpty(double.PositiveInfinity).Min();

    private static double Length(IReadOnlyList<Point3D> p)=>Enumerable.Range(1,p.Count-1)
        .Sum(i=>Math.Sqrt(Math.Pow(p[i].X-p[i-1].X,2)+Math.Pow(p[i].Y-p[i-1].Y,2)));
    private sealed class PointsComparer : IEqualityComparer<Point3D[]>
    {
        public bool Equals(Point3D[]? a,Point3D[]? b)=>a is not null&&b is not null&&a.SequenceEqual(b);
        public int GetHashCode(Point3D[] p)=>p.Length;
    }
}
