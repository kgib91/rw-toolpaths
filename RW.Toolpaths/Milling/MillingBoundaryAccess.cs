using Clipper2Lib;
using RW.Toolpaths.Geometry;
using System.Diagnostics;
using System.Runtime.ExceptionServices;

namespace RW.Toolpaths.Milling;

/// <summary>
/// Operator-approved empty space through the complete requested depth. These regions are
/// clearance, never extra clearing demand. Everything else inside PartBoundary is protected.
/// Coordinates use the same world frame as the operation's target regions.
/// </summary>
public sealed record MillingBoundaryAccess(
    IReadOnlyList<MillingRegion> ClearanceRegions, IReadOnlyList<PointD> PartBoundary)
{
    /// <summary>Optional jointly evaluated target-plus-clearance fill. Source-curve adapters
    /// supply this to avoid seams caused by independently approximating shared interfaces.</summary>
    public IReadOnlyList<MillingRegion>? AllowedRegions { get; init; }
    /// <summary>Optional jointly evaluated retained fill for complemented (Outside) operations.</summary>
    public IReadOnlyList<MillingRegion>? ProtectedRegions { get; init; }
    public string? DiagnosticContext { get; init; }
    internal Resolved Resolve(IReadOnlyList<MillingRegion> targets, ToolGeometry tool,
        DepthSchedule depth, bool smooth, double angle, double allowance, CancellationToken token)
        => new(targets, this, tool, depth, smooth, angle, allowance, token);

    internal sealed class Resolved
    {
        internal IReadOnlyList<MillingRegion> Regions => GetCommandDomain(default).Regions;
        internal IReadOnlyList<IReadOnlyList<PointD>> Target { get; }
        internal IReadOnlyList<IReadOnlyList<PointD>> OpenEdgeTarget => GetOpenEdges(default).Boundary;
        internal IReadOnlyList<PointD> PartBoundary { get; }
        internal string Signature { get; }
        private readonly PolygonOps.BoundarySegmentIndex _target;
        private sealed record OpenEdges(IReadOnlyList<IReadOnlyList<PointD>> Boundary,PolygonOps.BoundarySegmentIndex Index);
        private OpenEdges? _openEdges;
        private readonly object _openEdgesLock=new();
        private sealed record CommandDomain(IReadOnlyList<IReadOnlyList<PointD>> Boundary,IReadOnlyList<MillingRegion> Regions);
        private CommandDomain? _commandDomain;
        private readonly object _commandDomainLock=new();
        internal bool CuttingRegionsPrepared=>Volatile.Read(ref _commandDomain) is not null;
        private readonly double _contactReach;
        internal bool OpenEdgeBandPrepared => Volatile.Read(ref _openEdges) is not null;
        internal sealed record PocketPreparation(IReadOnlyList<MillingRegion> Regions,
            IReadOnlyList<IReadOnlyList<PointD>> EmptyBoundary);
        private readonly IReadOnlyList<IReadOnlyList<PointD>> _physicalBoundary;
        private readonly IReadOnlyList<IReadOnlyList<PointD>> _protectedBoundary;
        private readonly PolygonOps.BoundarySegmentIndex _protectedMaterial;
        private readonly PolygonOps.BoundarySegmentIndex _part;
        private readonly PolygonOps.BoundarySegmentIndex _centres;
        private readonly IReadOnlyList<IReadOnlyList<PointD>> _centreBoundary;
        private readonly string? _context;
        private IReadOnlyList<IReadOnlyList<PointD>>? _rampInterior;
        private IReadOnlyList<IReadOnlyList<PointD>>? _rampWallBand;
        private long _rampWallClipOperations;
        internal long RampWallClipOperations=>Interlocked.Read(ref _rampWallClipOperations);
        private readonly (double MinX,double MinY,double MaxX,double MaxY) _windowBounds;
        private readonly (long MinX,long MinY,long MaxX,long MaxY)? _rectangularPart;

        internal PocketPreparation PreparePocket(double radius,double allowance,double emptyReserve,
            CancellationToken token,int maximumConcurrency=3)
        {
            if(maximumConcurrency<1||maximumConcurrency>3)throw new ArgumentOutOfRangeException(nameof(maximumConcurrency));
            return PreparePocketRecipes(t=>{GetCommandDomain(t);},t=>PocketRegions(radius,allowance,t),
                t=>CertifiedEmptyBoundary(emptyReserve,t),Math.Min(maximumConcurrency,GeometryParallelism.WorkerLimit),
                token,message=>Console.WriteLine(message),_context);
        }

        // These recipes read one immutable resolved authority. Only CommandDomain
        // publishes a memo, under its existing lock; each Boolean/offset owns its
        // own kernel. Join every admitted helper before returning or throwing.
        internal static PocketPreparation PreparePocketRecipes(Action<CancellationToken> commandDomain,
            Func<CancellationToken,IReadOnlyList<MillingRegion>> pocketRegions,
            Func<CancellationToken,IReadOnlyList<IReadOnlyList<PointD>>> emptyBoundary,
            int maximumConcurrency,CancellationToken token,Action<string>? diagnostic=null,string? context=null)
        {
            if(maximumConcurrency<1||maximumConcurrency>3)throw new ArgumentOutOfRangeException(nameof(maximumConcurrency));
            token.ThrowIfCancellationRequested();long started=Stopwatch.GetTimestamp();
            using var cancellation=CancellationTokenSource.CreateLinkedTokenSource(token);
            var workToken=cancellation.Token;long commandTicks=0,pocketTicks=0,emptyTicks=0;
            IReadOnlyList<MillingRegion>? regions=null;IReadOnlyList<IReadOnlyList<PointD>>? empty=null;
            bool completed=false;ExceptionDispatchInfo? failure=null;
            void Command(){long s=Stopwatch.GetTimestamp();try{commandDomain(workToken);}finally{commandTicks+=Stopwatch.GetTimestamp()-s;}}
            void Empty(){long s=Stopwatch.GetTimestamp();try{empty=emptyBoundary(workToken);}finally{emptyTicks+=Stopwatch.GetTimestamp()-s;}}
            void Helper(Action recipe)
            {
                using var nested=GeometryParallelism.EnterWorker();
                try{workToken.ThrowIfCancellationRequested();recipe();}
                catch{cancellation.Cancel();throw;}
            }
            // Two slots run command+empty in one helper. Three slots give each
            // independent recipe a helper; caller retains the original guide.
            var helpers=new List<Task>(2);
            try
            {
                try
                {
                    if(maximumConcurrency>1)helpers.Add(Task.Run(()=>Helper(CommandAndMaybeEmpty)));
                    if(maximumConcurrency==3)helpers.Add(Task.Run(()=>Helper(Empty)));
                }
                catch(Exception error){cancellation.Cancel();failure=ExceptionDispatchInfo.Capture(error);}
                using(var nested=GeometryParallelism.EnterWorker())
                try
                {
                    workToken.ThrowIfCancellationRequested();
                    if(maximumConcurrency==1)Command();
                    long s=Stopwatch.GetTimestamp();
                    try{regions=pocketRegions(workToken);}finally{pocketTicks+=Stopwatch.GetTimestamp()-s;}
                    if(maximumConcurrency==1)Empty();
                }
                catch(Exception error){cancellation.Cancel();failure??=ExceptionDispatchInfo.Capture(error);}
                try{Task.WhenAll(helpers).GetAwaiter().GetResult();}
                catch(Exception error)
                {
                    cancellation.Cancel();
                    // A helper fault can cancel the caller. Preserve the actual
                    // fault rather than reporting only its cooperative symptom.
                    if(failure is null||failure.SourceException is OperationCanceledException
                        &&error is not OperationCanceledException)failure=ExceptionDispatchInfo.Capture(error);
                }
                failure?.Throw();token.ThrowIfCancellationRequested();completed=true;
                return new(regions!,empty!);
            }
            finally
            {
                double Ms(long ticks)=>ticks*1000.0/Stopwatch.Frequency;
                diagnostic?.Invoke($"[Milling.OutsideEdges.PocketSetup] operation={context} completed={completed} "
                    +$"workers={maximumConcurrency} commandDomainMs={Ms(commandTicks):F3} pocketGuideMs={Ms(pocketTicks):F3} "
                    +$"emptyBoundaryMs={Ms(emptyTicks):F3} totalMs={Stopwatch.GetElapsedTime(started).TotalMilliseconds:F3} "
                    +$"helpersJoined={helpers.All(t=>t.IsCompleted)} originalRecipes=True nestedGeometryWorkers=1 "
                    +"timing=overlapping-owned-wall-guide-includes-command-domain-wait authority=unchanged-resolved-domain");
            }
            void CommandAndMaybeEmpty(){Command();if(maximumConcurrency==2){workToken.ThrowIfCancellationRequested();Empty();}}
        }

        /// <summary>
        /// Pocket fronts follow the cutting target, with a local allowance for cutter overrun.
        /// The unrestricted access domain remains the physical wall/air authority; its remote
        /// clearance contours must never become the source of concentric pocket offsets.
        /// </summary>
        internal IReadOnlyList<MillingRegion> PocketRegions(double radius, double allowance, CancellationToken token)
        {
            token.ThrowIfCancellationRequested();
            long guideStarted = Stopwatch.GetTimestamp();
            // Internal approved clearance can support a cutter contacting the target from
            // either side. At the platform edge, compensate a radius-wide apron so the
            // outermost centreline stays on the platform, while the cutter can overhang.
            double expansion = 2 * radius + Math.Max(0, allowance);
            var guide = PolygonOps.OffsetPocketGuide(Target, expansion, .0001, token);
            double expansionMilliseconds = Stopwatch.GetElapsedTime(guideStarted).TotalMilliseconds;
            var platform = PolygonOps.Offset(new[] { PartBoundary }, radius + allowance - .001, .0001, 0);
            guide = PolygonOps.Intersect(guide,platform);
            var local = PolygonOps.Intersect(guide, Fill(GetRegions(token)));
            var result = PolygonOps.DifferenceToComponents(local, Array.Empty<IReadOnlyList<PointD>>())
                .Select(c => new MillingRegion(c.Outer,c.Holes)).ToArray();
            token.ThrowIfCancellationRequested();
            Console.WriteLine($"[Milling.OutsideEdges.PocketGuide] operation={_context} targetContours={Target.Count} "
                + $"physicalAccessRegions={Regions.Count} guideRegions={result.Length} expansion={expansion:R} "
                + $"platformCentreOverrun=0 cutterOverhang={radius:R} guideArea={Math.Abs(local.Sum(r=>Clipper.Area(new PathD(r)))):R} "
                + $"expansionMs={expansionMilliseconds:F1} totalMs={Stopwatch.GetElapsedTime(guideStarted).TotalMilliseconds:F1} "
                + $"source=original-cutting-target remoteClearanceShapesFronts=False physicalWallsUnchanged=True");
            return result;
        }

        internal IReadOnlyList<IReadOnlyList<PointD>> CertifiedEmptyBoundary(double reserve,CancellationToken token)
        {
            token.ThrowIfCancellationRequested();
            // The joint allowed fill is authoritative. Demand conservatively
            // contains the cutting target, so subtracting it leaves only approved
            // clearance / Part exterior, never uncut target material or islands.
            var air=PolygonOps.Difference(_physicalBoundary,Target);
            var lower=PolygonOps.Offset(air,-reserve,.0001,0);
            token.ThrowIfCancellationRequested();
            Console.WriteLine($"[Milling.OutsideEdges.EmptySpace] operation={_context} contours={lower.Count} "
                + $"reserve={reserve:R} authority=explicit-operator-permission targetExcluded=True executedStroke=False");
            return lower.Select(p=>(IReadOnlyList<PointD>)p).ToArray();
        }

        internal void Verify(IReadOnlyList<TaggedToolpath> paths, ToolGeometry tool, DepthSchedule depth,
            double allowance, CancellationToken token)
        {
            long started=Stopwatch.GetTimestamp();
            // Ordinary pockets use a conservative engaged disk for wall checks.
            // This bound is never used as evidence of already-cleared stock.
            double envelopeDepth=PocketContinuationRouter.RequiredPostedCylinderHeight(depth);
            double envelopeRadius=tool.PhysicalProfile?.MaxRadiusBelow(envelopeDepth) ?? tool.Radius;
            var profile=tool.VCarveProfile ?? (tool.ConeLength is > 0 ? VCarvePhysicalSweep.ProfileFor(tool,depth.Depth)
                : new VCarveCutterProfile(new[]{new CarvingCutterSection(CarvingCutterSectionKind.Linear,
                    envelopeRadius,envelopeRadius,envelopeDepth)},envelopeDepth));
            VCarveSmoothing.CommandClearance? guard=null;
            bool cylinder=profile.Sections.All(section=>section.Kind==CarvingCutterSectionKind.Linear
                &&section.BottomRadius==profile.TipRadius&&section.TopRadius==profile.TipRadius)
                &&profile.TipRadius+allowance>0;
            var cylinderBoundary=cylinder?new PolygonOps.BoundarySegmentIndex(PathUtils.ToClipper(_physicalBoundary)):null;
            long chords=0,cylinderProofs=0;double fallbackPreparationMs=0;
            bool Legal(Point3D a,Point3D b)
            {
                var ca=new Point3D(VCarveCommandRounding.Round((float)a.X),VCarveCommandRounding.Round((float)a.Y),
                    VCarveCommandRounding.Round((float)(depth.SurfaceZ+a.Z)-(float)depth.SurfaceZ));
                var cb=new Point3D(VCarveCommandRounding.Round((float)b.X),VCarveCommandRounding.Round((float)b.Y),
                    VCarveCommandRounding.Round((float)(depth.SurfaceZ+b.Z)-(float)depth.SurfaceZ));
                if(cylinderBoundary!=null&&-Math.Min(ca.Z,cb.Z)<=profile.MaximumDepth
                    &&cylinderBoundary.ContainsCapsule(Native(ca),Native(cb),(profile.TipRadius+allowance)*PathUtils.Scale))
                {cylinderProofs++;return true;}
                if(guard==null)
                {
                    long prepare=Stopwatch.GetTimestamp();
                    var walls=PolygonOps.Offset(_physicalBoundary,-allowance,.0001,0);
                    guard=new VCarveSmoothing.CommandClearance(walls,profile,depth.SurfaceZ,token,null);
                    fallbackPreparationMs=Stopwatch.GetElapsedTime(prepare).TotalMilliseconds;
                }
                return guard.IsLegalCommandedChord(a,b);
            }
            foreach(var path in paths)
            for(int i=1;i<path.Points.Count;i++)
            {
                token.ThrowIfCancellationRequested();
                var a=path.Points[i-1] with { Z=path.Points[i-1].Z-depth.SurfaceZ };
                var b=path.Points[i] with { Z=path.Points[i].Z-depth.SurfaceZ };
                if(Math.Min(a.Z,b.Z)>=0) continue;
                if(!CentreChord(a,b) || !CentreChord(new((float)a.X,(float)a.Y,a.Z),new((float)b.X,(float)b.Y,b.Z))
                    || !CentreChord(new(VCarveCommandRounding.Round((float)a.X),VCarveCommandRounding.Round((float)a.Y),a.Z),
                        new(VCarveCommandRounding.Round((float)b.X),VCarveCommandRounding.Round((float)b.Y),b.Z)))
                {
                    Console.Error.WriteLine($"[Milling.OutsideEdges.Verify] operation={_context} result=failed reason=centre-outside-platform "
                        + $"from={a} to={b} fingerprint={Signature}");
                    throw new InvalidOperationException("Outside-edge tool centre leaves the platform: "+a+" -> "+b);
                }
                if(!Legal(a,b))
                {
                    Console.Error.WriteLine($"[Milling.OutsideEdges.Verify] operation={_context} result=failed fingerprint={Signature} "
                        + $"category={path.Category} pass={path.DepthPassIndex} from={a} to={b} allowance={allowance:R} "
                        + $"coordinateSpace=float-world-F3 partialOutput=False");
                    throw new InvalidOperationException("Outside-edge cutter sweep intersects protected material: "+a+" -> "+b);
                }
                chords++;
            }
            Console.WriteLine($"[Milling.OutsideEdges.Verify] operation={_context} result=passed fingerprint={Signature} chords={chords} "
                + $"elapsedMs={Stopwatch.GetElapsedTime(started).TotalMilliseconds:F1} rectangularPart={_rectangularPart.HasValue} "
                + $"strictCylinderProofs={cylinderProofs} fallbackChords={chords-cylinderProofs} fallbackPreparationMs={fallbackPreparationMs:F1} "
                + $"coordinateSpace=float-world-F3 fullPhysicalProfile=True platformCentreOverrun=0");
        }

        internal bool CentreChord(Point3D a,Point3D b)
        {
            if(_rectangularPart is {} box)
            {
                var first=Native(a);var last=Native(b);
                return first.X>=box.MinX&&first.X<=box.MaxX&&first.Y>=box.MinY&&first.Y<=box.MaxY
                    &&last.X>=box.MinX&&last.X<=box.MaxX&&last.Y>=box.MinY&&last.Y<=box.MaxY;
            }
            var intervals = _part.ContactIntervals(Native(a),Native(b),0,default,out _);
            return intervals.Count==1 && intervals[0].Start==0 && intervals[0].End==1;
        }

        internal Resolved(IReadOnlyList<MillingRegion> targets, MillingBoundaryAccess access,
            ToolGeometry tool, DepthSchedule depth, bool smooth, double angle, double allowance, CancellationToken token)
        {
            token.ThrowIfCancellationRequested();
            _context=access.DiagnosticContext;
            if (access.PartBoundary.Count < 3 || access.PartBoundary.Any(p => !double.IsFinite(p.x) || !double.IsFinite(p.y))
                || Math.Abs(Clipper.Area(new PathD(access.PartBoundary))) <= 0)
                throw new ArgumentException("Outside-edge milling requires a finite, nonempty Part boundary.");
            PartBoundary=access.PartBoundary.ToArray();
            var partVertices=PartBoundary.Select(p=>new Point64((long)Math.Round(p.x*PathUtils.Scale),(long)Math.Round(p.y*PathUtils.Scale))).ToList();
            if(partVertices.Count>1&&partVertices[0]==partVertices[^1])partVertices.RemoveAt(partVertices.Count-1);
            if(partVertices.Count==4&&partVertices.Distinct().Count()==4
                &&partVertices.Select(p=>p.X).Distinct().Count()==2&&partVertices.Select(p=>p.Y).Distinct().Count()==2
                &&Enumerable.Range(0,4).All(i=>partVertices[i].X==partVertices[(i+1)%4].X||partVertices[i].Y==partVertices[(i+1)%4].Y))
                _rectangularPart=(partVertices.Min(p=>p.X),partVertices.Min(p=>p.Y),partVertices.Max(p=>p.X),partVertices.Max(p=>p.Y));
            _part=new(PathUtils.ToClipper(new[]{PartBoundary}));
            _centreBoundary=PolygonOps.Offset(new[]{PartBoundary},-.001,.0001,0);
            _centres=new(PathUtils.ToClipper(_centreBoundary));
            Target = PolygonOps.Intersect(Fill(targets), new[] { access.PartBoundary });
            _target = new(PathUtils.ToClipper(Target));
            var allowed = access.AllowedRegions is { } joint ? Fill(joint)
                : PolygonOps.Union(Fill(targets).Concat(Fill(access.ClearanceRegions)));
            var protectedMaterial = access.ProtectedRegions is { } retained
                ? PolygonOps.Intersect(Fill(retained),new[]{access.PartBoundary})
                : PolygonOps.Difference(new[] { access.PartBoundary }, allowed);
            _protectedBoundary=protectedMaterial;
            _protectedMaterial = new(PathUtils.ToClipper(protectedMaterial));
            var points = Target.SelectMany(r => r).ToArray();
            if (points.Length == 0)
            {
                _physicalBoundary = Array.Empty<IReadOnlyList<PointD>>();
                _commandDomain=new(Array.Empty<IReadOnlyList<PointD>>(),Array.Empty<MillingRegion>());
                _openEdges=new(Array.Empty<IReadOnlyList<PointD>>(),new(new())); Signature = "empty"; return;
            }
            // The construction boundary is farther away than both physical cutter reach
            // and the complete possible grade influence. It cannot anchor a ramp at demand.
            double contactReach = tool.VCarveProfile?.PhysicalRadiusAtDepth(
                Math.Min(depth.Depth, tool.VCarveProfile.MaximumDepth)) ?? tool.Radius;
            double reach = Math.Max(tool.Radius,contactReach);
            double gradeRun = smooth ? depth.Depth / Math.Tan(angle * Math.PI / 180) : 0;
            double pad = 2 * reach + Math.Abs(allowance) + gradeRun + 1;
            if (!double.IsFinite(pad) || pad <= 0) throw new ArgumentException("Outside-edge planning extent is invalid.");
            var window = new List<PointD> { new(points.Min(p=>p.x)-pad,points.Min(p=>p.y)-pad),
                new(points.Max(p=>p.x)+pad,points.Min(p=>p.y)-pad), new(points.Max(p=>p.x)+pad,points.Max(p=>p.y)+pad),
                new(points.Min(p=>p.x)-pad,points.Max(p=>p.y)+pad) };
            var domain = PolygonOps.Difference(new[] { window }, protectedMaterial);
            _windowBounds=(window[0].x,window[0].y,window[2].x,window[2].y);
            _physicalBoundary=domain;
            // Reserve the float/F3 planar displacement before nominations, rather
            // than asking the final physical proof to accept a rounded corner gouge.
            _contactReach=contactReach;
            // Bind the complete immutable recipe. Roughing ramp-wall analysis
            // does not consume supplemental open-edge cuts, so do not buffer
            // their potentially huge interface band until that consumer asks.
            Signature = Convert.ToHexString(System.Security.Cryptography.SHA256.HashData(
                System.Text.Json.JsonSerializer.SerializeToUtf8Bytes(new { Target, domain,commandMargin=.001,contactReach,
                    openEdgePolicy="target-interface-contact-band-v1",PartBoundary, centrePolicy="inside-platform-v1" },
                    new System.Text.Json.JsonSerializerOptions { IncludeFields=true })));
            Console.WriteLine($"[Milling.OutsideEdges] operation={_context} targetRegions={targets.Count} clearanceRegions={access.ClearanceRegions.Count} "
                + $"protectedContours={protectedMaterial.Count} accessRegions=deferred openInterfaces=deferred "
                + $"targetArea={Math.Abs(Target.Sum(r=>Clipper.Area(new PathD(r)))):R} guardMargin={pad:R} "
                + $"fingerprint={Signature} clearanceIsDemand=False");
            token.ThrowIfCancellationRequested();
        }

        internal IReadOnlyList<MillingRegion> GetRegions(CancellationToken token)=>GetCommandDomain(token).Regions;

        private CommandDomain GetCommandDomain(CancellationToken token)
        {
            token.ThrowIfCancellationRequested();
            if(Volatile.Read(ref _commandDomain) is { } ready)return ready;
            lock(_commandDomainLock)
            {
                token.ThrowIfCancellationRequested();
                if(_commandDomain is { } existing)return existing;
                long started=Stopwatch.GetTimestamp();
                var boundary=PolygonOps.Offset(_physicalBoundary,-.001,.0001,0);
                var regions=PolygonOps.DifferenceToComponents(boundary,Array.Empty<IReadOnlyList<PointD>>())
                    .Select(c=>new MillingRegion(c.Outer,c.Holes)).ToArray();
                token.ThrowIfCancellationRequested();
                var prepared=new CommandDomain(boundary,regions);Volatile.Write(ref _commandDomain,prepared);
                Console.WriteLine($"[Milling.OutsideEdges.CommandDomain] operation={_context} regions={regions.Length} "
                    +$"points={boundary.Sum(r=>(long)r.Count)} elapsedMs={Stopwatch.GetElapsedTime(started).TotalMilliseconds:F1} "
                    +$"construction=unchanged-reserved-physical-domain publication=complete consumer=cutting-regions");
                return prepared;
            }
        }

        private OpenEdges GetOpenEdges(CancellationToken token)
        {
            token.ThrowIfCancellationRequested();
            if(Volatile.Read(ref _openEdges) is { } ready)return ready;
            lock(_openEdgesLock)
            {
                token.ThrowIfCancellationRequested();
                if(_openEdges is { } existing)return existing;
                long started=Stopwatch.GetTimestamp();
                // Same original clipping, buffer and intersection as eager
                // construction. No cancellation token is retained in the owner;
                // an interrupted build publishes nothing and a new caller retries.
                var contours=Target.Select(r=>(IReadOnlyList<PointD>)r.Concat(new[]{r[0]}).ToArray());
                var open=CheckedOpenClip.Require(contours,GetCommandDomain(token).Boundary,ClipType.Intersection,token,"outside-edge-interfaces");
                var boundary=BuildOpenInterfaceBand(open,Target,_contactReach,token);
                token.ThrowIfCancellationRequested();
                var prepared=new OpenEdges(boundary,new(PathUtils.ToClipper(boundary)));
                token.ThrowIfCancellationRequested();
                Volatile.Write(ref _openEdges,prepared);
                Console.WriteLine($"[Milling.OutsideEdges.OpenBand] operation={_context} interfaces={open.Count} "
                    +$"contours={boundary.Count} points={boundary.Sum(r=>(long)r.Count)} elapsedMs={Stopwatch.GetElapsedTime(started).TotalMilliseconds:F1} "
                    +$"source=original-interface-recipe publication=complete consumer=open-edge-cuts implicitClosingChords=False");
                return prepared;
            }
        }

        internal static IReadOnlyList<IReadOnlyList<PointD>> BuildOpenInterfaceBand(
            IReadOnlyList<IReadOnlyList<PointD>> interfaces,IReadOnlyList<IReadOnlyList<PointD>> target,
            double reach,CancellationToken token)
        {
            // Contact clipping returns OPEN fragments. Joined buffering adds an
            // unauthored end-to-start chord, inventing diagonal cutting demand.
            // The established bounded round-capped builder preserves exactly
            // the supplied finite strokes and checks cancellation between chunks.
            var buffered=PolygonOps.BufferOpenCenterlines(interfaces,reach,.0001,token);
            token.ThrowIfCancellationRequested();
            return PolygonOps.Intersect(buffered,target);
        }

        private static List<List<PointD>> Fill(IReadOnlyList<MillingRegion> regions) => PolygonOps.Union(
            regions.SelectMany(r => PolygonOps.Difference(new[] { r.Outer }, r.Holes)));

        internal IReadOnlyList<(PointD A, PointD B)> RetainedRampEdges(
            IReadOnlyList<IReadOnlyList<PointD>> boundary, CancellationToken token)
            => RetainedRampEdges(boundary,token,proveAllRetained:false,out _);

        internal IReadOnlyList<(PointD A, PointD B)> RetainedRampEdges(
            IReadOnlyList<IReadOnlyList<PointD>> boundary, CancellationToken token,out bool allRetained)
            => RetainedRampEdges(boundary,token,proveAllRetained:true,out allRetained);

        private readonly record struct RampWallClipRecipe(IReadOnlyList<IReadOnlyList<PointD>> Closed,
            IReadOnlyList<IReadOnlyList<PointD>> Clip,bool WindowInterior,ClipType RetainedOperation,ClipType ProofOperation,
            double WallRadius,double BandReserve);

        // The original retained-edge and complementary-empty proofs share one
        // literal recipe. Keeping it here binds both consumers to the same
        // window test, reserves, Boolean operations and lazy geometry owners.
        private RampWallClipRecipe PrepareRampWallClip(IReadOnlyList<IReadOnlyList<PointD>> boundary,CancellationToken token)
        {
            token.ThrowIfCancellationRequested();
            double wallRadius=2*VCarvePreparation.GeometricErrorBound;
            const double bandArc=.0001;
            double bandReserve=4*bandArc+32/PathUtils.Scale;
            bool windowInterior=boundary.SelectMany(p=>p).All(p=>
                p.x>_windowBounds.MinX+wallRadius+bandReserve&&p.x<_windowBounds.MaxX-wallRadius-bandReserve
                &&p.y>_windowBounds.MinY+wallRadius+bandReserve&&p.y<_windowBounds.MaxY-wallRadius-bandReserve);
            var closed = boundary.Where(r => r.Count > 1)
                .Select(r => (IReadOnlyList<PointD>)r.Concat(new[] { r[0] }).ToArray()).ToArray();
            if(windowInterior)
            {
                // In the guarded window, complement(erosion(window - P,r))
                // equals dilation(P,r). Retain an OUTER wall band, reserving
                // offset sagitta and integer construction. This can retain extra
                // protected-wall anchors; it cannot remove an existing one.
                var band=Volatile.Read(ref _rampWallBand);
                if(band==null)
                {
                    IReadOnlyList<IReadOnlyList<PointD>> prepared=PolygonOps.Buffer(_protectedBoundary,
                        wallRadius+bandReserve,bandArc);
                    band=Interlocked.CompareExchange(ref _rampWallBand,prepared,null)??prepared;
                }
                return new(closed,band,true,ClipType.Intersection,ClipType.Difference,wallRadius,bandReserve);
            }
            // Keep original finite wall segments. Only portions strictly inside
            // approved access lose their ramp anchor; protected walls stay exact.
            var interior = Volatile.Read(ref _rampInterior);
            if (interior == null)
            {
                IReadOnlyList<IReadOnlyList<PointD>> prepared = PolygonOps.Offset(_physicalBoundary,
                    -2 * VCarvePreparation.GeometricErrorBound, .0001, 0);
                interior = Interlocked.CompareExchange(ref _rampInterior, prepared, null) ?? prepared;
            }
            return new(closed,interior,false,ClipType.Difference,ClipType.Intersection,wallRadius,bandReserve);
        }

        // Owned analytical reuse needs only the SAME complementary-empty
        // certificate. A positive avoids constructing a discarded retained
        // intersection; it does not nominate walls or grant cutting access.
        internal bool RampWallsPreserved(IReadOnlyList<IReadOnlyList<PointD>> boundary,CancellationToken token)
        {
            long started=Stopwatch.GetTimestamp();var recipe=PrepareRampWallClip(boundary,token);
            bool result=ClipRampWalls(recipe,true,token).Count==0;
            token.ThrowIfCancellationRequested();
            Console.WriteLine($"[Milling.OutsideEdges.RampWalls] operation={_context} mode={(recipe.WindowInterior?"protected-wall-band":"eroded-domain")} "
                +$"allRetained={result} unchangedWallProofOnly=True retainedEdgesMaterialized=False openClipOperations=1 "
                +$"elapsedMs={Stopwatch.GetElapsedTime(started).TotalMilliseconds:F1} sameOriginalComplementProof=True");
            token.ThrowIfCancellationRequested();return result;
        }

        private List<List<PointD>> ClipRampWalls(RampWallClipRecipe recipe,bool proof,CancellationToken token)
        {
            token.ThrowIfCancellationRequested();Interlocked.Increment(ref _rampWallClipOperations);
            return CheckedOpenClip.Require(recipe.Closed,recipe.Clip,proof?recipe.ProofOperation:recipe.RetainedOperation,token,
                proof?"outside-ramp-unchanged-wall-proof":recipe.WindowInterior?"outside-ramp-protected-wall-band":"outside-ramp-walls");
        }

        private IReadOnlyList<(PointD A, PointD B)> RetainedRampEdges(
            IReadOnlyList<IReadOnlyList<PointD>> boundary, CancellationToken token,bool proveAllRetained,out bool allRetained)
        {
            long started=Stopwatch.GetTimestamp();var recipe=PrepareRampWallClip(boundary,token);
            var walls=ClipRampWalls(recipe,false,token);
            // Generic and out-parameter consumers keep their original retained
            // clip first, then the requested complementary proof, unchanged.
            allRetained=proveAllRetained&&ClipRampWalls(recipe,true,token).Count==0;
            var result=walls.SelectMany(path => Enumerable.Range(1, path.Count - 1)
                .Select(i => (path[i - 1], path[i]))).ToArray();
            Console.WriteLine($"[Milling.OutsideEdges.RampWalls] operation={_context} mode={(recipe.WindowInterior?"protected-wall-band":"eroded-domain")} "
                +$"edges={result.Length} radius={recipe.WallRadius:R} outerReserve={recipe.BandReserve:R} elapsedMs={Stopwatch.GetElapsedTime(started).TotalMilliseconds:F1} "
                +$"windowInteriorProved={recipe.WindowInterior} protectedAnchorsConservative=True "
                +$"unchangedWallProofRequested={proveAllRetained} openClipOperations={(proveAllRetained?2:1)}");
            return result;
        }

        internal List<TaggedToolpath> NominateOpenEdges(VCarveCutterProfile profile,DepthSchedule depth,
            double stepOver,double allowance,CancellationToken token)
        {
            var result=new List<TaggedToolpath>();
            if(GetOpenEdges(token).Boundary.Count==0) return result;
            // Nominate the platform-edge centreline just inside the posting reserve;
            // clipping an exact edge after nomination would discard that whole contour.
            var source=PathUtils.ToClipper(PolygonOps.Intersect(Target,_centreBoundary));
            long literalSourcePoints=source.Sum(p=>(long)p.Count);
            if(Acceleration.RawInitialClearingFrontConditioner.TryCondition(source,token,
                message=>Console.WriteLine($"[Milling.OutsideEdges.Nomination] operation={_context} {message}"),
                out var conditioned,out _))source=conditioned!;
            Console.WriteLine($"[Milling.OutsideEdges.Nomination] operation={_context} event=source "
                +$"literalPoints={literalSourcePoints} candidatePoints={source.Sum(p=>(long)p.Count)} "
                +$"conditioner=existing-raw-front originalTargetAndPhysicalWallsUnchanged=True physicalProofRequired=True");
            for(int pass=0;pass<depth.PassCount;pass++)
            {
                double penetration=depth.SurfaceZ-depth.PassBottomZ(pass);
                double radius=profile.PhysicalRadiusAtDepth(penetration);
                double diameter=2*radius;
                double pitch=profile.RoundedTip?profile.NoseLimitedPitch(diameter,stepOver)
                    : diameter*(pass==depth.PassCount-1?.05:stepOver);
                if(!(pitch>0)) continue;
                int steps=Math.Max(1,(int)Math.Ceiling(radius/pitch));
                if(steps>RingOffsetEngine.AbsoluteRingLimit)
                    throw new InvalidOperationException("Outside-edge finishing exceeds the existing ring limit; increase stepover.");
                for(int level=-steps;level<=steps;level++)
                {
                    token.ThrowIfCancellationRequested();
                    var offset=new ClipperOffset(2,.0001*PathUtils.Scale);
                    offset.AddPaths(source,JoinType.Round,EndType.Polygon);
                    var contours=new Paths64(); offset.Execute(level*radius/steps*PathUtils.Scale,contours);
                    var candidates=PathUtils.FromClipper(contours).Where(r=>r.Count>1).Select(r=>
                        new TaggedToolpath(r.Concat(new[]{r[0]}).Select(p=>new Point3D(p.x,p.y,-penetration)).ToArray(),
                            Array.Empty<ToolpathSpan>(),0,"clearing",depth.FirstPassIndex+pass,detectCurves:false));
                    foreach(var path in Trim(candidates,profile.PhysicalRadiusAtDepth,token,openEdgesOnly:true))
                    {
                        var points=new List<Point3D>();
                        void Flush()
                        {
                            if(points.Count>1) result.Add(new(points.ToArray(),Array.Empty<ToolpathSpan>(),0,"clearing",
                                depth.FirstPassIndex+pass,detectCurves:false));
                            points.Clear();
                        }
                        for(int i=1;i<path.Points.Count;i++)
                        {
                            foreach(var run in LegalRuns(new(path.Points[i-1].X,path.Points[i-1].Y),
                                new(path.Points[i].X,path.Points[i].Y),radius,allowance,token))
                            {
                                var a=new Point3D(run.A.x,run.A.y,-penetration);var b=new Point3D(run.B.x,run.B.y,-penetration);
                                if(points.Count>0&&points[^1]!=a) Flush();
                                if(points.Count==0) points.Add(a);points.Add(b);
                            }
                        }
                        Flush();
                    }
                }
            }
            Console.WriteLine($"[Milling.OutsideEdges.Nominations] operation={_context} paths={result.Count} "
                + $"passes={depth.PassCount} origin=opened-target-interfaces clearanceFillRequested=False physicalProofRequired=True");
            return result;
        }

        internal IEnumerable<(PointD A,PointD B)> LegalRuns(PointD a,PointD b,double radius,double allowance,CancellationToken token)
        {
            var forbidden=_protectedMaterial.ContactIntervals(new Point64((long)Math.Round(a.x*PathUtils.Scale),(long)Math.Round(a.y*PathUtils.Scale)),
                new Point64((long)Math.Round(b.x*PathUtils.Scale),(long)Math.Round(b.y*PathUtils.Scale)),
                (Math.Max(0,radius+allowance)+.001)*PathUtils.Scale,token,out _);
            PointD At(double t)=>new(a.x+(b.x-a.x)*t,a.y+(b.y-a.y)*t);
            double cursor=0;
            foreach(var (first,last) in forbidden)
            {
                if(first>cursor+1e-10) yield return(At(cursor),At(first));
                cursor=Math.Max(cursor,last);
            }
            if(cursor<1-1e-10) yield return(At(cursor),At(1));
        }

        internal List<OffsetRing> TrimRings(IReadOnlyList<OffsetRing> rings, double radius, CancellationToken token)
        {
            var result = new List<OffsetRing>();
            foreach (var ring in rings)
            {
                var xy = ring.IsClosed ? ring.Points.Concat(new[] { ring.Points[0] }) : ring.Points;
                var source = new TaggedToolpath(xy.Select(p=>new Point3D(p.x,p.y,0)).ToArray(),
                    Array.Empty<ToolpathSpan>(), ring.RegionIndex, "pocket", null, detectCurves:false);
                foreach (var fragment in Trim(new[] { source }, _ => radius, token))
                {
                    bool closed = fragment.Points[0] == fragment.Points[^1];
                    var points = (closed ? fragment.Points.Take(fragment.Points.Count-1) : fragment.Points)
                        .Select(p=>new PointD(p.X,p.Y)).ToList();
                    result.Add(RingOffsetEngine.CreateRing(points,ring.Level,ring.RegionIndex,ring.IsHole,closed));
                }
            }
            return result;
        }

        /// <summary>Trim proposals in XYZ while retaining their directed heights and span kinds.
        /// The indexed filled-contact query includes holes and intervening protected gaps.</summary>
        internal List<TaggedToolpath> Trim(IEnumerable<TaggedToolpath> paths, Func<double,double> radiusAtDepth,
            CancellationToken token, bool openEdgesOnly = false, double surface = 0,
            PolygonOps.BoundarySegmentIndex? clipIndex = null, bool restrictCentres = true,
            bool retainContactingChords=false,ICollection<int>? interpolatedFragments=null)
            =>TrimCore(paths,radiusAtDepth,token,openEdgesOnly,surface,clipIndex,restrictCentres,
                retainContactingChords,interpolatedFragments,false,null);

        internal static readonly AsyncLocal<bool?> ContourInteriorFirstEnabled=new();
        // The caller supplies the immutable physical profile, not an opaque
        // radius callback. Keep generic Trim and its callback order literal.
        internal List<TaggedToolpath> TrimOwnedContours(IEnumerable<TaggedToolpath> paths,VCarveCutterProfile profile,
            CancellationToken token,Action<string>? diagnostic=null)
        {
            ArgumentNullException.ThrowIfNull(profile);token.ThrowIfCancellationRequested();
            return TrimCore(paths,profile.PhysicalRadiusAtDepth,token,false,0,null,true,false,null,
                ContourInteriorFirstEnabled.Value is not false,diagnostic);
        }

        // Completed protected source and flat-pocket callers own their radius
        // recipes. They may prefer the SAME existing FullContact certificate;
        // generic radius delegates retain literal callback/query ordering above.
        internal List<TaggedToolpath> TrimOwnedPhysicalProfile(IEnumerable<TaggedToolpath> paths,VCarveCutterProfile profile,
            CancellationToken token,double surface=0,bool retainContactingChords=false,
            ICollection<int>? interpolatedFragments=null,Action<string>? diagnostic=null)
        {
            ArgumentNullException.ThrowIfNull(profile);token.ThrowIfCancellationRequested();
            return TrimCore(paths,profile.PhysicalRadiusAtDepth,token,false,surface,null,true,
                retainContactingChords,interpolatedFragments,ContourInteriorFirstEnabled.Value is not false,diagnostic,
                "owned-profile-trim","immutable-physical-profile");
        }

        internal List<TaggedToolpath> TrimOwnedFlatRadius(IEnumerable<TaggedToolpath> paths,double radius,
            CancellationToken token,double surface=0,Action<string>? diagnostic=null)
        {
            token.ThrowIfCancellationRequested();
            return TrimCore(paths,_=>radius,token,false,surface,null,true,false,null,
                ContourInteriorFirstEnabled.Value is not false,diagnostic,"owned-flat-trim","fixed-flat-radius");
        }

        private List<TaggedToolpath> TrimCore(IEnumerable<TaggedToolpath> paths,Func<double,double> radiusAtDepth,
            CancellationToken token,bool openEdgesOnly,double surface,PolygonOps.BoundarySegmentIndex? clipIndex,
            bool restrictCentres,bool retainContactingChords,ICollection<int>? interpolatedFragments,
            bool interiorFirst,Action<string>? diagnostic,string stage="owned-contour-trim",string radiusOwner="immutable-physical-profile")
        {
            long started=Stopwatch.GetTimestamp(),contactQueries=0,interiorHits=0;
            var index = clipIndex ?? (openEdgesOnly ? GetOpenEdges(token).Index : _target);
            var output = new List<TaggedToolpath>();
            long wholeContactChords=0,unchangedChords=0,insertedFragments=0;
            foreach (var path in paths)
            {
                var points = new List<Point3D>(); var spans = new List<ToolpathSpan>();
                bool interpolated=false;
                void Flush()
                {
                    if (points.Count > 1)
                    {
                        output.Add(new(points.ToArray(),spans.ToArray(),path.RegionIndex,
                            path.Category,path.DepthPassIndex,path.VisitIdentity,detectCurves:false)
                            {ConcentricSource=path.ConcentricSource});
                        if(interpolated){insertedFragments++;interpolatedFragments?.Add(output.Count-1);}
                    }
                    points.Clear(); spans.Clear();
                    interpolated=false;
                }
                int spanIndex = 0;
                for (int i=1;i<path.Points.Count;i++)
                {
                    token.ThrowIfCancellationRequested();
                    var a=path.Points[i-1]; var b=path.Points[i];
                    while (spanIndex+1<path.Spans.Count && path.Spans[spanIndex].EndIndex<i) spanIndex++;
                    var kind = path.Spans.Count==0 ? ToolpathSpanKind.Cut : path.Spans[spanIndex].Kind;
                    double radius = radiusAtDepth(Math.Max(0,surface-Math.Min(a.Z,b.Z)));
                    contactQueries++;bool interiorHit=false;
                    var contact = interiorFirst
                        ?index.ContactIntervalsInteriorFirst(Native(a),Native(b),Math.Max(0,radius)*PathUtils.Scale,token,out _,out interiorHit)
                        :index.ContactIntervals(Native(a),Native(b),Math.Max(0,radius)*PathUtils.Scale,token,out _);
                    if(interiorHit)interiorHits++;
                    var centres = restrictCentres ? _centres.ContactIntervals(Native(a),Native(b),0,token,out _) : null;
                    // A protected source chord is the physical proof unit.
                    // Cutting it at a demand-contact parameter creates new F3
                    // XY endpoints, whose rounding need not stay on its posted
                    // source chord. Keep the COMPLETE original chord when it
                    // contacts demand; it already fits approved clearance.
                    // Platform centre clipping remains mandatory and reports
                    // every interpolated fragment for fresh original protection.
                    bool hasContact=retainContactingChords&&contact.Any(c=>c.End-c.Start>=1e-12);
                    IEnumerable<(double Start,double End)> contacting=hasContact
                        ?new[]{(0d,1d)}:contact.Select(c=>(c.Start,c.End));
                    if(hasContact)wholeContactChords++;
                    var intervals = centres is null ? contacting.ToArray()
                        : contacting.SelectMany(c=>centres.Select(p=>(Start:Math.Max(c.Start,p.Start),End:Math.Min(c.End,p.End))))
                            .Where(p=>p.End>p.Start).ToArray();
                    if(intervals.Length==0) { Flush(); continue; }
                    foreach (var (first,last) in intervals)
                    {
                        if(last-first<1e-12) continue;
                        var start=Lerp(a,b,first); var end=Lerp(a,b,last);
                        if(points.Count>0 && points[^1]!=start) Flush();
                        bool inserted=first>0&&first<1||last>0&&last<1;
                        interpolated|=inserted;
                        if(!inserted)unchangedChords++;
                        if(points.Count==0) points.Add(start);
                        int from=points.Count-1; points.Add(end);
                        if(spans.Count>0 && spans[^1].Kind==kind) spans[^1]=spans[^1] with {EndIndex=points.Count-1};
                        else spans.Add(new(from,points.Count-1,kind));
                        if(last<1-1e-12) Flush();
                    }
                }
                Flush();
            }
            if(retainContactingChords)Console.WriteLine($"[Milling.OutsideEdges.PostedSourceTrim] operation={_context} "
                +$"wholeContactChords={wholeContactChords} unchangedChords={unchangedChords} interpolatedFragments={insertedFragments} "
                +$"demandUnchanged=True sourceChordGeometryRetained=True platformCentreClip=True newVerticesRequireFreshProtection=True");
            diagnostic?.Invoke($"stage={stage} event=complete insideFirstEnabled={interiorFirst} contactQueries={contactQueries} "
                +$"interiorShortcutHits={interiorHits} nearestBoundaryQueriesAvoided={interiorHits} outputPaths={output.Count} "
                +$"elapsedMs={Stopwatch.GetElapsedTime(started).TotalMilliseconds:F3} originalIntegerMidpointAndExtent=True "
                +$"originalFullContactPositiveBranch=True centreClippingUnchanged=True originalPathAndFragmentOrder=True radiusOwner={radiusOwner} "
                +"opaqueRadiusCallbacks=False parallelWorkers=1");
            return output;
        }
        private static Point64 Native(Point3D p)=>new((long)Math.Round(p.X*PathUtils.Scale),(long)Math.Round(p.Y*PathUtils.Scale));
        private static Point3D Lerp(Point3D a,Point3D b,double t)=>t==0?a:t==1?b:new(a.X+(b.X-a.X)*t,a.Y+(b.Y-a.Y)*t,a.Z+(b.Z-a.Z)*t);
    }
}
