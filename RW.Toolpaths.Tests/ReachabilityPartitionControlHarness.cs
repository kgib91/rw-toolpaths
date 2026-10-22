using System.Collections;
using System.Diagnostics;
using System.Numerics;
using System.Reflection;
using System.Text.Json;
using Clipper2Lib;
using RW.Toolpaths.Milling.Certification;
using Cert = RW.Toolpaths.Milling.Certification.SignedBoundaryCapsuleCertificate;

namespace RW.Toolpaths.Tests;

// Independent controls, not a production admission path. Every capsule oracle
// below uses BigInteger, and every winding check uses literal signed paths and
// rational query points. The supported controls exercise the live private aid.
internal static class ReachabilityPartitionControlHarness
{
    private const BindingFlags All = BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance | BindingFlags.Static;
    private static readonly List<object> Results = new();
    private static readonly List<string> Failures = new();
    private static readonly Stopwatch Total = Stopwatch.StartNew();
    private static readonly CancellationTokenSource Lifetime = new(TimeSpan.FromSeconds(45));
    private readonly record struct Q(BigInteger X, BigInteger Y, BigInteger D);
    private sealed record Partition(bool Complete, Cert.Diagnostics Proof, Paths64 Hulls, List<Rect64> Windows);

    internal static int Run(string[] args)
    {
        if (args.Length != 1 || File.Exists(args[0])) throw new ArgumentException("Supply one fresh result file.");
        Environment.SetEnvironmentVariable("RW_ARTIFACT_TWO_CAPSULE_PARTITION", null);
        Environment.SetEnvironmentVariable("RW_ARTIFACT_SUBTICK_SITE_NEIGHBORS", null);
        Environment.SetEnvironmentVariable("RW_ARTIFACT_UNRESOLVED_HULL_SIZE_TICKS", null);
        RunCapsuleControls();
        RunChainControls();
        RunPartitionControls();
        RunRefusalControls();
        RunOwnershipAndRadiusContractControls();
        RunRegisteredChoiceControls();
        RunPhysicalControls();
        File.WriteAllText(args[0], JsonSerializer.Serialize(new {
            status = Failures.Count == 0 ? "controls-pass" : "authority-contract-controls-failed",
            acceptance = false, productionIntegrationAuthorized = false,
            frozenArchiveSha256 = "6e6c61111b34e7010ac72992c9f0821b937d4b17626d5079dd5c85bcbc8493c6",
            passed = Results.Count - Failures.Count, failed = Failures.Count,
            elapsedSeconds = Total.Elapsed.TotalSeconds, controls = Results, failures = Failures
        }, new JsonSerializerOptions { WriteIndented = true }));
        Console.WriteLine($"CONTROL TOTAL passed={Results.Count-Failures.Count} failed={Failures.Count} seconds={Total.Elapsed.TotalSeconds:F3}");
        return Failures.Count == 0 ? 0 : 2;
    }

    private static void Checkpoint()
    {
        Lifetime.Token.ThrowIfCancellationRequested();
        if (Total.Elapsed > TimeSpan.FromSeconds(40)) throw new TimeoutException("Whole control run deadline.");
    }
    private static void Control(string name, Func<object?> test)
    {
        var clock = Stopwatch.StartNew();
        try {
            Checkpoint(); object? evidence = test();
            Results.Add(new { name, passed = true, milliseconds = clock.Elapsed.TotalMilliseconds, evidence });
            Console.WriteLine($"PASS {name} ({clock.Elapsed.TotalMilliseconds:F2} ms)");
        } catch (Exception e) {
            while (e is TargetInvocationException { InnerException: not null }) e = e.InnerException!;
            Failures.Add(name);
            Results.Add(new { name, passed = false, milliseconds = clock.Elapsed.TotalMilliseconds, error = e.ToString() });
            Console.WriteLine($"FAIL {name}: {e.GetType().Name}: {e.Message}");
        }
    }
    private static void Require(bool condition, string reason) { if (!condition) throw new InvalidOperationException(reason); }
    private static Path64 Box(long x0, long y0, long x1, long y1, bool negative = false)
    {
        Path64 p = new() { new(x0,y0),new(x1,y0),new(x1,y1),new(x0,y1) };
        if (negative) p.Reverse(); return p;
    }
    private static Paths64 DifferenceChain(Paths64 source, Paths64 center)
        => new(source.Select(p => new Path64(p)).Concat(center.Select(p => new Path64(p.AsEnumerable().Reverse()))));
    private static IEnumerable<(Point64 Start, Point64 End)> Sites(Paths64 source)
    {
        foreach (var p in source) for (int i=0;i<p.Count;i++) if(p[i]!=p[(i+1)%p.Count]) yield return(p[i],p[(i+1)%p.Count]);
    }
    private static bool CapsuleOracle(Q p, (Point64 Start,Point64 End) site, long radius, int bits)
    {
        BigInteger dx=(BigInteger)site.End.X-site.Start.X,dy=(BigInteger)site.End.Y-site.Start.Y;
        BigInteger wx=p.X-site.Start.X*p.D,wy=p.Y-site.Start.Y*p.D;
        BigInteger length=dx*dx+dy*dy,projection=wx*dx+wy*dy;
        BigInteger factor=BigInteger.One<<(2*bits),limit=(BigInteger)radius*radius*p.D*p.D;
        if(length==0||projection<=0)return(wx*wx+wy*wy)*factor<=limit;
        if(projection>=length*p.D){wx=p.X-site.End.X*p.D;wy=p.Y-site.End.Y*p.D;return(wx*wx+wy*wy)*factor<=limit;}
        return (wx*dy-wy*dx)*(wx*dy-wy*dx)*factor<=limit*length;
    }
    private static int Winding(Paths64 paths,Q p)
    {
        int winding=0;
        foreach(var path in paths)for(int i=0;i<path.Count;i++){
            Point64 a=path[i],b=path[(i+1)%path.Count];
            BigInteger cross=((BigInteger)b.X-a.X)*(p.Y-a.Y*p.D)-((BigInteger)b.Y-a.Y)*(p.X-a.X*p.D);
            if(a.Y*p.D<=p.Y){if(b.Y*p.D>p.Y&&cross>0)winding++;}
            else if(b.Y*p.D<=p.Y&&cross<0)winding--;
        }
        return winding;
    }
    private static bool InclusiveConvex(Path64 path,Q p)
    {
        foreach(var (a,b) in Sites(new(){path})){
            BigInteger cross=((BigInteger)b.X-a.X)*(p.Y-a.Y*p.D)-((BigInteger)b.Y-a.Y)*(p.X-a.X*p.D);
            if(cross<0)return false;
        }
        return true;
    }
    private static (Point64 Start,Point64 End) Nearest(Paths64 source,double x,double y)
    {
        var candidates=Sites(source).ToArray();
        return candidates.MinBy(s=>{
            double dx=s.End.X-s.Start.X,dy=s.End.Y-s.Start.Y,l=dx*dx+dy*dy;
            double t=l==0?0:Math.Clamp(((x-s.Start.X)*dx+(y-s.Start.Y)*dy)/l,0,1);
            double vx=x-s.Start.X-t*dx,vy=y-s.Start.Y-t*dy;return vx*vx+vy*vy;
        });
    }
    private static Partition Prove(Paths64 source,Paths64 center,long radius,int bits=0,int depth=4,int tasks=100_000,Paths64? aid=null,int windows=4096)
    {
        var partition=new Cert.PartitionPolicy { MaximumDepth=depth,SmallHullSizeTicks=0,MaximumHulls=windows };
        bool result=Cert.TryCertify(DifferenceChain(source,center),p=>Nearest(source,p.X,p.Y),radius,Lifetime.Token,out var proof,
            maximumTriangleTasks:tasks,useDelaunay:false,workCheckpoint:Checkpoint,originalBoundary:source,
            radiusFractionBits:bits,fractionalOriginalSiteChoice:(x,y)=>Nearest(source,x,y),
            partition:partition,triangulationAidBoundary:aid);
        var rectangles=partition.Enclosures.Select(p=>new Rect64(p.Min(v=>v.X),p.Min(v=>v.Y),p.Max(v=>v.X),p.Max(v=>v.Y))).ToList();
        return new(result,proof,partition.Enclosures,rectangles);
    }
    private static object Evidence(Partition p) => new {
        complete=p.Complete,p.Proof.Reason,p.Proof.MainTriangles,p.Proof.CorrectionTriangles,p.Proof.FinalResidualEdges,
        p.Proof.TriangleTasks,p.Proof.CertifiedPieces,p.Proof.BigIntegerFallbacks,
        p.Proof.SupportOutsideEnclosuresCertified,hulls=p.Hulls.Count,windows=p.Windows.Count,
        p.Proof.UnresolvedPieces,p.Proof.MaximumPendingTasks
    };
    private static bool CopiedCapsule(Int128 x,Int128 y,int shift,(Point64 Start,Point64 End) site,long radius,int bits,out int fallbacks)
    {
        var type=typeof(Cert);var dyadic=type.GetNestedType("DyadicPoint",All)!;
        object point=Activator.CreateInstance(dyadic,All,null,new object[]{x,y},null)!;
        var d=new Cert.Diagnostics();
        bool actual=(bool)type.GetMethod("CapsuleWithDyadicRadius",All)!.Invoke(null,new object[]{point,site,radius,shift,d,bits})!;
        fallbacks=d.BigIntegerFallbacks;return actual;
    }
    private static void RunCapsuleControls()
    {
        var line=(new Point64(-3,0),new Point64(3,0));
        var cases=new List<(string,Int128,Int128,int,(Point64,Point64),long,int)> {
            ("finite interior tangent",0,2,0,line,2,0),
            ("one dyadic quantum outside tangent",0,131073,16,line,2,0),
            ("finite end cap rejects supporting line witness",5,1,0,line,2,0),
            ("finite end cap tangent",5,0,0,line,2,0),
            ("stationary endpoint disc",1,1,0,(new(0,0),new(0,0)),2,0),
            ("negative fractional projection",-7,3,1,line,2,0),
            ("fractional radius exact tangent",0,3,16,line,3,16),
            ("fractional radius outside quantum",0,4,16,line,3,16),
            ("near bound signed diagonal",(Int128)((1L<<30)-3)*65536,(Int128)((1L<<30)-2)*65536,16,
                (new(-(1L<<30)+1,-(1L<<30)+1),new((1L<<30)-2,(1L<<30)-2)),2,16),
            ("large dyadic shift fallback",(Int128)1<<77,(Int128)1<<76,48,
                (new(-(1L<<30)+2,-(1L<<30)+2),new((1L<<30)-2,(1L<<30)-2)),10,16)
        };
        foreach(var c in cases)Control("capsule: "+c.Item1,()=>{
            var q=new Q((BigInteger)c.Item2,(BigInteger)c.Item3,BigInteger.One<<c.Item4);
            bool expected=CapsuleOracle(q,c.Item5,c.Item6,c.Item7);
            bool actual=CopiedCapsule(c.Item2,c.Item3,c.Item4,c.Item5,c.Item6,c.Item7,out int fallback);
            Require(actual==expected,"Frozen finite capsule disagrees with independent BigInteger oracle.");
            return new{expected,actual,fallback};
        });
        Control("capsule: near-limit exhaustive small dyadic oracle",()=>{
            int tested=0,fallbacks=0;var site=(new Point64(-(1L<<30)+1,17),new Point64((1L<<30)-1,23));
            foreach(int shift in new[]{0,16,32,48})for(int x=-2;x<=2;x++)for(int y=-2;y<=2;y++){
                Int128 qx=(Int128)x<<shift,qy=((Int128)20<<shift)+y;
                bool a=CopiedCapsule(qx,qy,shift,site,2*65536,16,out int f);fallbacks+=f;
                bool b=CapsuleOracle(new((BigInteger)qx,(BigInteger)qy,BigInteger.One<<shift),site,2*65536,16);
                Require(a==b,$"Oracle mismatch at {shift}/{x}/{y}");tested++;
            }
            Require(fallbacks>0,"BigInteger path was not exercised.");return new{tested,fallbacks};
        });
    }

    private static void RunChainControls()
    {
        Control("chain: exact collinear splitting cancels",()=>{
            var boundary=new Paths64{Box(0,0,12,12)};
            var triangles=new Paths64 {new(){new(0,0),new(12,0),new(12,12)},new(){new(0,0),new(12,12),new(0,12)}};
            boundary[0].Insert(1,new(6,0));
            var r=Residual(boundary,triangles);Require(r.Count==0,"Equivalent subdivided boundary did not cancel.");return new{edges=r.Count};
        });
        Control("chain: omitted disconnected contour requires correction",()=>{
            var source=new Paths64{Box(0,0,12,12),Box(30,0,33,3)};
            var proposed=new Paths64{new(){new(0,0),new(12,0),new(12,12)},new(){new(0,0),new(12,12),new(0,12)}};
            return CompleteCorrections(source,proposed,4);
        });
        Control("chain: swallowed negative protected hole requires corrections",()=>{
            var source=new Paths64{Box(0,0,12,12),Box(4,4,8,8,true)};
            var proposed=new Paths64{new(){new(0,0),new(12,0),new(12,12)},new(){new(0,0),new(12,12),new(0,12)}};
            return CompleteCorrections(source,proposed,4);
        });
        Control("chain: rounded triangulator crossing requires signed correction",()=>{
            var source=new Paths64{new(){new(0,0),new(11,0),new(10,10),new(0,11)}};
            var proposed=new Paths64{new(){new(0,0),new(11,0),new(11,11)},new(){new(0,0),new(11,11),new(0,11)}};
            return CompleteCorrections(source,proposed,3);
        });
        Control("chain: opposite winding duplicates have zero support",()=>{
            var p=Box(-3,-4,5,8);var boundary=new Paths64{p,new(p.AsEnumerable().Reverse())};
            Require(Residual(boundary,new()).Count==0,"Opposite exact rings did not cancel.");return null;
        });
        Control("chain: unbalanced directed correction refuses",()=>{
            var residual=new List<(Point64 Start,Point64 End)>{(new(0,0),new(10,0)),(new(10,0),new(10,10))};
            var list=TriangleList(new());object?[] args={residual,list,Lifetime.Token,(Action)Checkpoint,0};
            bool accepted=(bool)typeof(Cert).GetMethod("AppendSignedCorrections",All)!.Invoke(null,args)!;
            Require(!accepted,"An open chain was accepted as filled support.");return new{accepted};
        });
    }
    private static IList TriangleList(Paths64 paths)
    {
        var t=typeof(Cert).GetNestedType("Triangle",All)!;var list=(IList)Activator.CreateInstance(typeof(List<>).MakeGenericType(t))!;
        foreach(var p in paths)list.Add(Activator.CreateInstance(t,All,null,new object[]{p[0],p[1],p[2]},null));return list;
    }
    private static List<(Point64 Start,Point64 End)> Residual(Paths64 boundary,Paths64 paths)
        => (List<(Point64 Start,Point64 End)>)typeof(Cert).GetMethod("Residual",All)!.Invoke(null,new object[]{boundary,TriangleList(paths),Lifetime.Token,(Action)Checkpoint})!;
    private static object CompleteCorrections(Paths64 boundary,Paths64 paths,int minimumEdges)
    {
        var triangles=TriangleList(paths);
        var r=(List<(Point64 Start,Point64 End)>)typeof(Cert).GetMethod("Residual",All)!.Invoke(null,new object[]{boundary,triangles,Lifetime.Token,(Action)Checkpoint})!;
        Require(r.Count>=minimumEdges,"Counterexample did not leave a residual chain.");
        object?[] args={r,triangles,Lifetime.Token,(Action)Checkpoint,0};
        bool closed=(bool)typeof(Cert).GetMethod("AppendSignedCorrections",All)!.Invoke(null,args)!;
        var final=(List<(Point64 Start,Point64 End)>)typeof(Cert).GetMethod("Residual",All)!.Invoke(null,new object[]{boundary,triangles,Lifetime.Token,(Action)Checkpoint})!;
        Require(closed&&final.Count==0&&(int)args[4]!>0,"Every signed correction was not retained.");
        return new{initialEdges=r.Count,corrections=args[4],finalEdges=final.Count};
    }

    private static void RunPartitionControls()
    {
        Control("partition: complete narrow outer boundary band",()=>{
            var p=Prove(new(){Box(0,0,100,100)},new(){Box(2,2,98,98)},5,depth:8);
            Require(p.Complete&&p.Proof.FinalResidualEdges==0,"Narrow complete support did not certify.");return Evidence(p);
        });
        Control("partition: protected hole band and outer band",()=>{
            var source=new Paths64{Box(0,0,100,100),Box(40,40,60,60,true)};
            var center=new Paths64{Box(2,2,98,98),Box(38,38,62,62,true)};
            var p=Prove(source,center,5,depth:8);
            Require(p.Complete&&p.Proof.FinalResidualEdges==0,"Signed hole support did not certify.");return Evidence(p);
        });
        Control("partition: legal interior must survive in every outward hull",()=>{
            var source=new Paths64{Box(0,0,100,100)};var center=new Paths64{Box(15,15,85,85)};
            var p=Prove(source,center,10,depth:3);
            Require(!p.Complete&&p.Proof.SupportOutsideEnclosuresCertified&&p.Proof.FinalResidualEdges==0,"Incomplete center was accepted or partition chain missing.");
            var legal=new[]{new Q(25,50,2),new Q(175,50,2),new Q(50,25,2),new Q(50,175,2)};
            foreach(var q in legal){Require(Winding(source,q)!=0&&Winding(center,q)==0,"Test point is not omitted original material.");
                Require(!Sites(source).Any(s=>CapsuleOracle(q,s,10,0)),"Witness is not beyond every strict original capsule.");
                Require(p.Hulls.Any(h=>InclusiveConvex(h,q)),"An entire unresolved legal point disappeared.");}
            return Evidence(p);
        });
        Control("partition: omitted disconnected thin positive component survives",()=>{
            var source=new Paths64{Box(0,0,40,40),Box(100,0,103,40)};var center=new Paths64{Box(3,3,37,37)};
            var p=Prove(source,center,1,depth:2);var q=new Q(203,40,2);
            Require(!p.Complete&&p.Proof.SupportOutsideEnclosuresCertified&&p.Proof.FinalResidualEdges==0,"Disconnected support missing.");
            Require(!Sites(source).Any(s=>CapsuleOracle(q,s,1,0))&&p.Hulls.Any(h=>InclusiveConvex(h,q)),"Thin legal island was dropped.");return Evidence(p);
        });
        Control("partition: swallowed protected hole remains negative support",()=>{
            var source=new Paths64{Box(0,0,100,100),Box(40,40,60,60,true)};var center=new Paths64{Box(2,2,98,98)};
            var p=Prove(source,center,5,depth:2);var q=new Q(50,50,1);
            Require(Winding(DifferenceChain(source,center),q)<0,"Hole counterexample lost negative winding.");
            Require(!p.Complete&&((p.Proof.SupportOutsideEnclosuresCertified&&p.Hulls.Any(h=>InclusiveConvex(h,q)))
                ||(!p.Proof.SupportOutsideEnclosuresCertified&&p.Proof.Reason.StartsWith("untrusted-triangulator-"))),
                "Protected negative support earned acceptance or incomplete partition credit: "+JsonSerializer.Serialize(Evidence(p)));return Evidence(p);
        });
        Control("partition: swallowed protected hole with alternate aid retains corrections",()=>{
            var source=new Paths64{Box(0,0,100,100),Box(40,40,60,60,true)};var center=new Paths64{Box(2,2,98,98)};
            var p=Prove(source,center,5,depth:0,aid:new(){Box(0,0,100,100)});
            Require(!p.Complete&&p.Proof.SupportOutsideEnclosuresCertified&&p.Proof.FinalResidualEdges==0&&p.Proof.CorrectionTriangles>0,
                "Alternate aid ignored the protected negative correction: "+JsonSerializer.Serialize(Evidence(p)));
            Require(p.Hulls.Any(h=>InclusiveConvex(h,new(50,50,1))),"Negative hole correction did not survive in retained H.");return Evidence(p);
        });
        Control("partition: untrusted aid deletes disconnected source",()=>{
            var source=new Paths64{Box(0,0,40,40),Box(100,0,103,40)};var center=new Paths64{Box(3,3,37,37)};
            var p=Prove(source,center,1,depth:0,aid:new(){Box(0,0,40,40)});
            Require(!p.Complete&&p.Proof.FinalResidualEdges==0&&p.Proof.CorrectionTriangles>0,"Omitted aid component earned authority.");
            Require(p.Hulls.Any(h=>InclusiveConvex(h,new(203,40,2))),"Signed correction hull does not enclose missing component.");return Evidence(p);
        });
        Control("partition: untrusted aid adds exterior material",()=>{
            var source=new Paths64{Box(-10,-10,10,10)};var center=new Paths64{Box(-8,-8,8,8)};
            var p=Prove(source,center,3,depth:0,aid:new(){Box(-10,-10,10,10),Box(50,50,60,60)});
            Require(p.Proof.FinalResidualEdges==0&&p.Proof.CorrectionTriangles>0,"Exterior aid was not canceled with full signed corrections.");
            Require(!p.Complete&&p.Hulls.Any(h=>InclusiveConvex(h,new(55,55,1))),"Exterior signed support was ignored instead of conservatively retained.");return Evidence(p);
        });
        Control("partition: negative sub-tick outward hull floor ceiling",()=>{
            var source=new Paths64{Box(-10,-10,10,10)};var center=new Paths64{Box(-2,-2,2,2)};
            var p=Prove(source,center,1,depth:4);
            Require(p.Proof.SupportOutsideEnclosuresCertified&&p.Proof.FinalResidualEdges==0,"Signed tiny partition did not close.");
            foreach(var q in new[]{new Q(-3,-3,2),new Q(-15,-15,2),new Q(15,15,2)})
                if(Winding(DifferenceChain(source,center),q)!=0&&!Sites(source).Any(s=>CapsuleOracle(q,s,1,0)))
                    Require(p.Hulls.Any(h=>InclusiveConvex(h,q)),"Negative fractional hull rounded inward.");return Evidence(p);
        });
    }

    private static void RunRefusalControls()
    {
        Control("refusal: zero work budget cannot partially certify",()=>{
            var p=Prove(new(){Box(0,0,100,100)},new(){Box(20,20,80,80)},1,tasks:0);
            Require(!p.Complete&&!p.Proof.SupportOutsideEnclosuresCertified&&p.Proof.Reason=="capsule-work-budget","Zero budget accepted partial support.");return Evidence(p);
        });
        Control("refusal: zero window budget cannot omit unresolved support",()=>{
            var p=Prove(new(){Box(0,0,100,100)},new(){Box(20,20,80,80)},1,depth:0,windows:0);
            Require(!p.Complete&&!p.Proof.SupportOutsideEnclosuresCertified&&p.Proof.Reason=="original-registry-or-partition-bound","Unrecorded hull gained partition authority.");return Evidence(p);
        });
        Control("refusal: near threshold depth limit does not bridge",()=>{
            bool accepted=Cert.TryCertify(new(){new(){new(0,2),new(1,2),new(0,3)}},_ => (new(0,0),new(10,0)),1,Lifetime.Token,out var p,
                maximumTriangleTasks:100_000,workCheckpoint:Checkpoint,originalBoundary:new(){Box(0,0,10,10)});
            Require(!accepted&&!p.Certified&&p.Reason=="capsule-subdivision-unresolved","Depth exhaustion accepted unreachable support.");return new{p.Reason,p.TriangleTasks,p.MaximumPendingTasks};
        });
        foreach(long coordinate in new[]{1L<<30,long.MinValue})Control("refusal: source range "+coordinate,()=>{
            bool accepted=Cert.TryCertify(new(){new(){new(coordinate,0),new(0,1),new(1,0)}},_ => (new(0,0),new(1,0)),1,Lifetime.Token,out var p);
            Require(!accepted&&p.Reason=="input-or-integer-bound","Out-of-range source accepted.");return new{p.Reason};
        });
        Control("refusal: malformed proposed triangles",()=>{
            Require(!Cert.IsBoundedProposedTriangulation(new(){Box(0,0,10,10)}),"A quadrilateral accepted as a triangle.");
            Require(!Cert.IsBoundedProposedTriangulation(new(){new(){new(0,0),new(1L<<30,0),new(0,1)}}),"Untrusted vertices escaped exact range guard.");return null;
        });
        Control("refusal: cancellation propagates before triangulation",()=>{
            using var cancelled=new CancellationTokenSource();cancelled.Cancel();
            try{Cert.TryCertify(new(){Box(0,0,20,20)},_ => (new(0,0),new(20,0)),2,cancelled.Token,out _);throw new InvalidOperationException("Cancellation was swallowed.");}
            catch(OperationCanceledException){return new{propagated=true};}
        });
        Control("refusal: shared deadline propagates",()=>{
            try{Cert.TryCertify(new(){Box(0,0,20,20)},_ => (new(0,0),new(20,0)),2,Lifetime.Token,out _,workCheckpoint:()=>throw new TimeoutException("shared-budget"));throw new InvalidOperationException("Deadline was swallowed.");}
            catch(TimeoutException e){Require(e.Message=="shared-budget","Wrong deadline cause.");return new{propagated=true};}
        });
        Control("refusal: triangulator work checkpoint runs during allocation",()=>{
            int checks=0;using var stop=new CancellationTokenSource();
            var path=new Path64();for(int i=0;i<200;i++){path.Add(new(i,0));path.Add(new(i,1));}
            try{var d=new Delaunay(false,stop.Token,100_000,()=>{if(++checks==8)throw new TimeoutException("allocation-stop");});d.Execute(new(){path},out _);throw new InvalidOperationException("Allocation checkpoint was not reached.");}
            catch(TimeoutException e){Require(e.Message=="allocation-stop","Unexpected timeout.");return new{checks};}
        });
        Control("refusal: valid large polygon cancels before completing node allocation",()=>{
            int checks=0;using var stop=new CancellationTokenSource();
            var path=new Path64(Enumerable.Range(0,2048).Select(i=>new Point64(
                (long)Math.Round(10000*Math.Cos(i*2*Math.PI/2048)),
                (long)Math.Round(10000*Math.Sin(i*2*Math.PI/2048)))));
            Require(path.Distinct().Count()==path.Count&&Clipper.Area(path)>0,"Large valid fixture lost positive simple geometry.");
            var d=new Delaunay(false,stop.Token,100_000,()=>{if(++checks==8){stop.Cancel();stop.Token.ThrowIfCancellationRequested();}});
            Paths64 published=new();
            try{d.Execute(new(){path},out published);throw new InvalidOperationException("Allocation cancellation was not reached.");}
            catch(OperationCanceledException){
                var vertices=(ICollection)typeof(Delaunay).GetField("allVertices",All)!.GetValue(d)!;
                Require(checks==8&&vertices.Count<path.Count&&published.Count==0,
                    "Cancellation happened after full allocation or published geometry.");
                return new{checks,allocatedVertices=vertices.Count,inputVertices=path.Count,published=published.Count};
            }
        });
    }

    private static void RunOwnershipAndRadiusContractControls()
    {
        Control("authority refusal: callback capsule must belong to literal original source",()=>{
            var original=new Paths64{Box(0,0,100,100)};
            bool accepted=Cert.TryCertify(new(){new(){new(40,40),new(41,40),new(40,41)}},_ => (new(40,40),new(41,40)),2,Lifetime.Token,out var p,
                originalBoundary:original,maximumTriangleTasks:100_000,workCheckpoint:Checkpoint);
            Require(!accepted,"Frozen wheel accepted a manufactured interior finite site. A registered original-site callback is a required admission premise.");return new{p.Reason};
        });
        Control("authority refusal: unsupported radius fraction shift",()=>{
            var original=new Paths64{Box(0,0,10,10)};
            bool accepted=Cert.TryCertify(new(){new(){new(0,1),new(1,1),new(0,2)}},_ => (new(0,0),new(10,0)),3,Lifetime.Token,out var p,
                originalBoundary:original,radiusFractionBits:64,maximumTriangleTasks:100_000,workCheckpoint:Checkpoint);
            Require(!accepted,"Int128 denominator shift wraps at 128; unsupported radius fractions earned whole support acceptance.");return new{accepted,p.Reason};
        });
        Control("authority: registered finite owner adapter rejects manufactured capsule",()=>{
            var source=new Paths64{Box(0,0,100,100)};var registry=Sites(source).ToHashSet();
            var proposed=(new Point64(40,40),new Point64(41,40));
            Require(!registry.Contains(proposed)&&!registry.Contains((proposed.Item2,proposed.Item1)),"Fake owner accidentally registered.");
            var stats=new OriginalHullComplement.Statistics();
            bool accepted=OriginalHullComplement.TryTrim(Box(40,40,41,41),new[]{new OriginalHullComplement.Site(proposed.Item1,proposed.Item2)},
                s=>registry.Contains((s.Start,s.End))||registry.Contains((s.End,s.Start)),2,0,Lifetime.Token,Checkpoint,stats,out var retained);
            Require(!accepted&&retained.Count==1,"Complement trimming accepted unregistered original owner.");return new{stats.LastReason};
        });
        Control("authority: native strict radius is below literal net eligibility",()=>{
            var native=typeof(RW.Toolpaths.Geometry.PolygonOps).Assembly;
            var gate=native.GetType("RW.Toolpaths.Milling.Certification.ExactFirstCoverageGate",true)!;
            var method=gate.GetMethod("StrictSourceRadiusForTraceNative",All)!;
            var proofs=new List<object>();
            foreach(double first in new[]{1d,4.762499928474426,100d}){
                long radius=(long)method.Invoke(null,new object[]{first})!;
                double support=radius/65536d,net=first*32768-4;
                Require(support<net-Math.ScaleB(1d,-14)&&support>net-.001,"Strict dyadic radius exceeded native net allowance or lost arbitrary stock.");
                proofs.Add(new{first,radius,support,net});
            }
            return proofs;
        });
    }

    private static void RunPhysicalControls()
    {
        const long s=32768;
        Control("physical: swallowed protected hole rejected despite safe boundary chords",()=>{
            var centers=new Paths64{Box(20*s,20*s,80*s,80*s)};
            var outer=new Paths64{Box(0,0,100*s,100*s),Box(45*s,45*s,55*s,55*s,true)};
            bool valid=PotentialCenterFootprint.Certify(centers,Array.Empty<Path64>(),outer,2,Lifetime.Token,Checkpoint,out int chords,out int enclosed);
            Require(!valid&&enclosed>0&&chords==4,"Boundary-only proof swallowed an enclosed protected hole.");return new{chords,enclosed};
        });
        Control("physical: all disconnected unresolved hulls are obligations",()=>{
            var center=new Paths64{Box(10*s,10*s,20*s,20*s)};
            var hulls=new[]{Box(60*s,10*s,70*s,20*s)};
            var footprint=new Paths64{Box(0,0,30*s,30*s)};
            bool valid=PotentialCenterFootprint.Certify(center,hulls,footprint,2,Lifetime.Token,Checkpoint,out int chords,out _);
            Require(!valid&&chords>4,"A disconnected unresolved hull was omitted from physical proof.");return new{chords};
        });
        Control("physical: negative center hole remains excluded",()=>{
            var center=new Paths64{Box(10*s,10*s,90*s,90*s),Box(40*s,40*s,60*s,60*s,true)};
            var footprint=new Paths64{Box(0,0,100*s,100*s),Box(43*s,43*s,57*s,57*s,true)};
            bool valid=PotentialCenterFootprint.Certify(center,Array.Empty<Path64>(),footprint,2,Lifetime.Token,Checkpoint,out int chords,out int enclosed);
            Require(valid&&chords==8&&enclosed==0,"Signed center negative hole was filled or misclassified.");return new{chords,enclosed};
        });
        Control("physical: hull tangency to swallowed hole fails closed",()=>{
            var hull=Box(40*s,40*s,60*s,60*s);
            var footprint=new Paths64{Box(0,0,100*s,100*s),Box(40*s,40*s,45*s,45*s,true)};
            Require(!PotentialCenterFootprint.Certify(new(),new[]{hull},footprint,1,Lifetime.Token,Checkpoint,out _,out _),"Tangent protected hole admitted.");return null;
        });
    }

    private static void RunRegisteredChoiceControls()
    {
        var original=new Paths64{Box(0,0,10,10)};
        var near=new Paths64{new(){new(1,1),new(2,1),new(1,2)}};
        Control("owner: reversed literal finite edge is accepted",()=>{
            bool accepted=Cert.TryCertify(near,_ => (new(10,0),new(0,0)),3,Lifetime.Token,out var p,originalBoundary:original);
            Require(accepted&&p.Certified,"Reversed literal edge lost authorized membership.");return new{p.Reason};
        });
        Control("owner: stationary literal original endpoint is accepted",()=>{
            bool accepted=Cert.TryCertify(near,_ => (new(0,0),new(0,0)),3,Lifetime.Token,out var p,originalBoundary:original);
            Require(accepted&&p.Certified,"Literal original point owner lost authority.");return new{p.Reason};
        });
        Control("owner refusal: manufactured stationary interior site",()=>{
            bool accepted=Cert.TryCertify(near,_ => (new(1,1),new(1,1)),3,Lifetime.Token,out var p,originalBoundary:original);
            Require(!accepted&&p.Reason=="unregistered-original-site-choice","Manufactured stationary site accepted.");return new{p.Reason};
        });
        Control("owner refusal: boundary subsegment is not a literal complete edge",()=>{
            bool accepted=Cert.TryCertify(near,_ => (new(0,0),new(5,0)),3,Lifetime.Token,out var p,originalBoundary:original);
            Require(!accepted&&p.Reason=="unregistered-original-site-choice","Nonliteral edge bypassed exact provenance.");return new{p.Reason};
        });
        Control("owner refusal: fractional callback cannot bypass registry",()=>{
            bool accepted=Cert.TryCertify(near,_ => (new(0,0),new(10,0)),3,Lifetime.Token,out var p,originalBoundary:original,
                fractionalOriginalSiteChoice:(_,_) => (new(1,1),new(2,1)));
            Require(!accepted&&p.Reason=="unregistered-original-site-choice","Fractional callback manufactured a finite source site.");return new{p.Reason};
        });
        Control("owner refusal: missing literal original boundary",()=>{
            bool accepted=Cert.TryCertify(near,_ => (new(1,1),new(2,1)),3,Lifetime.Token,out var p);
            Require(!accepted&&p.Reason=="missing-original-site-registry","Missing original registry granted authority.");return new{p.Reason};
        });
        Control("owner: inherited source choices remain registered",()=>{
            var triangle=new Paths64{new(){new(0,0),new(10,0),new(0,10)}};
            bool accepted=Cert.TryCertify(triangle,p=>Nearest(original,p.X,p.Y),6,Lifetime.Token,out var p,originalBoundary:original,
                fractionalOriginalSiteChoice:(x,y)=>Nearest(original,x,y),workCheckpoint:Checkpoint);
            Require(accepted&&p.Certified&&p.InheritedSiteTrials>0&&p.InheritedSiteCertificates>0,"Inherited finite-site path not exercised or refused.");
            return new{p.Reason,p.InheritedSiteTrials,p.InheritedSiteCertificates};
        });
        Control("owner refusal: partition cannot nominate an unregistered capsule",()=>{
            var partition = new Cert.PartitionPolicy { MaximumDepth=0 };
            bool accepted=Cert.TryCertify(near,_ => (new(1,1),new(2,1)),3,Lifetime.Token,out var p,originalBoundary:original,partition:partition);
            Require(!accepted&&p.Reason=="unregistered-original-site-choice"&&!p.SupportOutsideEnclosuresCertified,
                "Unregistered site earned partial source authority.");
            return new{p.Reason};
        });
        foreach(int bits in new[]{-1,25,64,128})Control("radius refusal: unsupported fraction bits "+bits,()=>{
            bool accepted=Cert.TryCertify(near,_ => (new(0,0),new(10,0)),3,Lifetime.Token,out var p,originalBoundary:original,radiusFractionBits:bits);
            Require(!accepted&&p.Reason=="input-or-integer-bound"&&p.MainTriangles==0,"Fraction bound was not enforced before triangulation.");return new{bits,p.Reason};
        });
        Control("radius: upper permitted dyadic bits remain exact",()=>{
            var tiny=new Paths64{new(){new(0,0),new(1,0),new(0,1)}};
            bool accepted=Cert.TryCertify(tiny,_ => (new(0,0),new(10,0)),3L<<24,Lifetime.Token,out var p,originalBoundary:original,radiusFractionBits:24);
            Require(accepted&&p.Certified,"Valid 24-bit radius was rejected.");return new{p.Reason};
        });
    }
}
