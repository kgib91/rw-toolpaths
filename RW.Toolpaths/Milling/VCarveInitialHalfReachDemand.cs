using System.Diagnostics;

namespace RW.Toolpaths.Milling;

// This is a NOM-demand proxy, not a stock, coverage, or machining certificate.
// Only a literal original-protected F3 pose may nominate existing surface coverage.
internal static class VCarveInitialHalfReachDemand
{
    private const double GradientStep = .001, NominalInteriorMargin = .001;
    private const int MaximumQueryVisits = 100_000, LeafSize = 8;
    private const double MaximumNominationSeconds = 5;
    private readonly record struct Chord(Point3D A, Point3D B, double Radius);
    private struct Node
    {
        internal double X0,Y0,X1,Y1;
        internal int First,Count,Left,Right;
    }
    private sealed class Counts
    {
        internal long HalfPaths,InputPoints,RetainedPoints,KeptChords,DroppedChords,OutputRuns,
            Targets,Covered,Uncovered,Unknown,Distances,NodeVisits,ChordTests,RadiusQueries,QueryRefusals,BudgetFallbacks;
        internal double MaximumProjection,BuildMs;
        internal string SceneReason="not-built";
    }
    private sealed class Scene
    {
        private readonly Chord[] _chords;
        private readonly List<Node> _nodes;
        private readonly int[] _stack = new int[64];
        private readonly VCarveCutterProfile _profile;
        private readonly CancellationToken _token;
        private readonly Counts _counts;
        internal int ChordCount => _chords.Length;
        internal int NodeCount => _nodes.Count;
        internal long RetainedBytes => 56L*_chords.Length+48L*_nodes.Capacity+256+128;
        internal Scene(Chord[] chords,VCarveCutterProfile profile,CancellationToken token,Counts counts)
        {
            _chords=chords;_profile=profile;_token=token;_counts=counts;
            _nodes=new(Math.Max(1,2*((chords.Length+3)/4)));
            Build(0,chords.Length);
        }
        private int Build(int first,int count)
        {
            _token.ThrowIfCancellationRequested();
            var node=new Node {First=first,Count=count,Left=-1,Right=-1,X0=double.PositiveInfinity,Y0=double.PositiveInfinity,X1=double.NegativeInfinity,Y1=double.NegativeInfinity};
            for(int i=first;i<first+count;i++)
            {
                if((i&255)==0)_token.ThrowIfCancellationRequested();var s=_chords[i];
                node.X0=Math.Min(node.X0,Math.Min(s.A.X,s.B.X)-s.Radius);node.X1=Math.Max(node.X1,Math.Max(s.A.X,s.B.X)+s.Radius);
                node.Y0=Math.Min(node.Y0,Math.Min(s.A.Y,s.B.Y)-s.Radius);node.Y1=Math.Max(node.Y1,Math.Max(s.A.Y,s.B.Y)+s.Radius);
            }
            int ordinal=_nodes.Count;_nodes.Add(node);
            if(count>LeafSize)
            {
                bool x=node.X1-node.X0>=node.Y1-node.Y0;
                Array.Sort(_chords,first,count,Comparer<Chord>.Create((a,b)=>
                    (x?a.A.X+a.B.X:a.A.Y+a.B.Y).CompareTo(x?b.A.X+b.B.X:b.A.Y+b.B.Y)));
                int half=count/2;node.Left=Build(first,half);node.Right=Build(first+half,count-half);_nodes[ordinal]=node;
            }
            return ordinal;
        }
        internal bool TryCovered(double x,double y,out bool unknown)
        {
            unknown=false;int top=0,visits=0;_stack[top++]=0;
            while(top>0)
            {
                if((visits&255)==0)_token.ThrowIfCancellationRequested();
                if(++visits>MaximumQueryVisits){unknown=true;_counts.QueryRefusals++;return false;}
                _counts.NodeVisits++;var node=_nodes[_stack[--top]];
                if(x<node.X0||x>node.X1||y<node.Y0||y>node.Y1)continue;
                if(node.Left>=0)
                {
                    if(top+2>_stack.Length){unknown=true;_counts.QueryRefusals++;return false;}
                    _stack[top++]=node.Right;_stack[top++]=node.Left;continue;
                }
                for(int i=node.First;i<node.First+node.Count;i++)
                {
                    _counts.ChordTests++;var s=_chords[i];double dx=s.B.X-s.A.X,dy=s.B.Y-s.A.Y,square=dx*dx+dy*dy;
                    double t=square>0?Math.Clamp(((x-s.A.X)*dx+(y-s.A.Y)*dy)/square,0,1):0;
                    double qx=s.A.X+t*dx,qy=s.A.Y+t*dy,depth=-(s.A.Z+t*(s.B.Z-s.A.Z));
                    if(!double.IsFinite(t)||!double.IsFinite(depth)){unknown=true;continue;}
                    if(depth<=0)continue;
                    _counts.RadiusQueries++;double radius=_profile.PhysicalRadiusAtDepth(depth);
                    double d=Math.Sqrt((x-qx)*(x-qx)+(y-qy)*(y-qy));
                    if(!double.IsFinite(radius)||!double.IsFinite(d)){unknown=true;continue;}
                    if(d<radius-NominalInteriorMargin)return true;
                }
            }
            return false;
        }
    }
    internal static List<TaggedToolpath> SelectHalfCore(IReadOnlyList<TaggedToolpath> source,
        IReadOnlyList<TaggedToolpath> originalProtectedPaths,VCarveCutterProfile profile,double fullDepth,double authoredWidth,
        Func<Point3D,Point3D> actualCommandPoint,Func<double,double,double> artworkDistance,
        CancellationToken token,long pointBudget,Action<string>? diagnostic=null)
    {
        ArgumentNullException.ThrowIfNull(source);ArgumentNullException.ThrowIfNull(originalProtectedPaths);
        ArgumentNullException.ThrowIfNull(profile);ArgumentNullException.ThrowIfNull(actualCommandPoint);ArgumentNullException.ThrowIfNull(artworkDistance);
        token.ThrowIfCancellationRequested();long started=Stopwatch.GetTimestamp();var c=new Counts();
        long input=source.Sum(p=>(long)p.Points.Count),remaining=input,outputPoints=0;
        double fullRadius=double.IsFinite(fullDepth)&&fullDepth>0?profile.PhysicalRadiusAtDepth(fullDepth):double.NaN;
        double reach=authoredWidth+fullRadius;Scene? scene=null;
        bool eligible=pointBudget>0&&input<=pointBudget&&profile.Sections.Count is>0 and<=64
            &&double.IsFinite(fullDepth)&&fullDepth>0&&fullDepth<=profile.MaximumDepth
            &&double.IsFinite(authoredWidth)&&authoredWidth>0&&authoredWidth<=100
            &&double.IsFinite(fullRadius)&&fullRadius>0&&fullRadius<=100&&double.IsFinite(reach);
        if(eligible)
        {
            long buildStarted=Stopwatch.GetTimestamp();long segments=0;
            foreach(var path in originalProtectedPaths)
            {token.ThrowIfCancellationRequested();if(IsSupplement(path.Category))continue;segments+=Math.Max(0,path.Points.Count-1);}
            if(segments is>0 and<=int.MaxValue&&segments<=pointBudget)
            {
                var chords=new Chord[(int)segments];int n=0;bool valid=true;
                foreach(var path in originalProtectedPaths)
                {
                    token.ThrowIfCancellationRequested();if(IsSupplement(path.Category))continue;
                    if(path.Spans.Count!=0){valid=false;break;}
                    for(int i=1;i<path.Points.Count;i++)
                    {
                        if((i&255)==0)token.ThrowIfCancellationRequested();
                        Point3D a=actualCommandPoint(path.Points[i-1]),b=actualCommandPoint(path.Points[i]);
                        if(!Finite(a)||!Finite(b)){valid=false;break;}
                        double radius=Math.Max(profile.PhysicalRadiusAtDepth(-a.Z),profile.PhysicalRadiusAtDepth(-b.Z));
                        if(!double.IsFinite(radius)||radius<0||radius>100){valid=false;break;}
                        chords[n++]=new(a,b,radius);
                    }
                    if(!valid)break;
                }
                if(valid&&n==chords.Length){scene=new(chords,profile,token,c);c.SceneReason="complete-original-protected-actual-F3-chords";}
                else c.SceneReason="invalid-original-finite-chord-domain";
            }
            else c.SceneReason="original-chord-point-budget-or-empty";
            c.BuildMs=Stopwatch.GetElapsedTime(buildStarted).TotalMilliseconds;
        }
        else c.SceneReason="request-point-profile-domain";
        var result=new List<TaggedToolpath>(source.Count);
        foreach(var path in source)
        {
            token.ThrowIfCancellationRequested();remaining-=path.Points.Count;
            if(path.Category!=VCarveInitialReachSelection.HalfCoreMarkerCategory){result.Add(path);outputPoints+=path.Points.Count;continue;}
            c.HalfPaths++;c.InputPoints+=path.Points.Count;
            if(scene is null||path.Points.Count<2||path.Spans.Count!=0){KeepLiteral();continue;}
            bool closed=path.Points.Count>=4&&path.Points[0]==path.Points[^1];int vertices=closed?path.Points.Count-1:path.Points.Count;
            var demand=new bool[vertices];
            for(int i=0;i<vertices;i++)demand[i]=Demand(path.Points[i]);
            int edges=closed?vertices:vertices-1;var keep=new bool[edges];int kept=0;
            for(int i=0;i<edges;i++){keep[i]=demand[i]||demand[(i+1)%vertices];if(keep[i])kept++;}
            c.KeptChords+=kept;c.DroppedChords+=edges-kept;
            if(kept==edges){KeepLiteral();continue;}
            if(kept==0)continue;
            var runs=new List<List<Point3D>>();List<Point3D>? run=null;
            int first=closed?Array.FindIndex(keep,value=>!value)+1:0;
            for(int k=0;k<edges;k++)
            {
                token.ThrowIfCancellationRequested();int i=closed?(first+k)%edges:k;
                if(keep[i]){run??=new(){path.Points[i]};run.Add(path.Points[(i+1)%vertices]);}
                else if(run is not null){runs.Add(run);run=null;}
            }
            if(run is not null)runs.Add(run);
            long proposed=runs.Sum(p=>(long)p.Count);
            if(outputPoints+proposed+remaining>pointBudget){c.BudgetFallbacks++;KeepLiteral();continue;}
            foreach(var points in runs)result.Add(path with {Points=points,Category="clearing",Spans=Array.Empty<ToolpathSpan>()});
            outputPoints+=proposed;c.RetainedPoints+=proposed;c.OutputRuns+=runs.Count;
            void KeepLiteral(){result.Add(path with {Category="clearing"});outputPoints+=path.Points.Count;c.RetainedPoints+=path.Points.Count;c.OutputRuns++;}
        }
        token.ThrowIfCancellationRequested();
        diagnostic?.Invoke($"stage=initial-half-reach-demand event=complete sceneReason='{c.SceneReason}' originalChords={scene?.ChordCount??0} bvhNodes={scene?.NodeCount??0} retainedSceneBytes={scene?.RetainedBytes??0} buildMs={c.BuildMs:R} " +
            $"halfPaths={c.HalfPaths} inputHalfPoints={c.InputPoints} retainedHalfPoints={c.RetainedPoints} outputRuns={c.OutputRuns} keptChords={c.KeptChords} droppedChords={c.DroppedChords} " +
            $"targets={c.Targets} alreadyCovered={c.Covered} uncoveredNominations={c.Uncovered} unknownKeep={c.Unknown} artworkDistanceQueries={c.Distances} nodeVisits={c.NodeVisits} chordTests={c.ChordTests} radiusQueries={c.RadiusQueries} queryRefusals={c.QueryRefusals} outputBudgetFallbacks={c.BudgetFallbacks} " +
            $"fullDepth={fullDepth:R} authoredWidth={authoredWidth:R} fullRadius={fullRadius:R} farEdgeReach={reach:R} maximumTargetProjection={c.MaximumProjection:R} nominalInteriorMargin={NominalInteriorMargin:R} maximumNominationSeconds={MaximumNominationSeconds:R} " +
            $"elapsedMs={Stopwatch.GetElapsedTime(started).TotalMilliseconds:F3} globalPointBudget={pointBudget} " +
            "halfMarkerNormalizedOnly=True otherSupplementPathsUnchanged=True originalProtectedExcludesSupplementMarkers=True chosenXYParameterPoseOnly=True " +
            "queryMissMeans=uncovered-or-unknown-NOM-not-physical-impossibility stockCredit=False coverageCertificate=False freshOriginalFloorArtworkGradeAndF3AuditsRequired=True");
        token.ThrowIfCancellationRequested();return result;

        bool Demand(Point3D p)
        {
            token.ThrowIfCancellationRequested();c.Targets++;
            if(!Finite(p)||Stopwatch.GetElapsedTime(started).TotalSeconds>MaximumNominationSeconds){c.Unknown++;return true;}
            c.Distances+=5;double d=artworkDistance(p.X,p.Y);
            double xm=artworkDistance(p.X-GradientStep,p.Y),xp=artworkDistance(p.X+GradientStep,p.Y),ym=artworkDistance(p.X,p.Y-GradientStep),yp=artworkDistance(p.X,p.Y+GradientStep);
            double gx=(xp-xm)/(2*GradientStep),gy=(yp-ym)/(2*GradientStep),norm=Math.Sqrt(gx*gx+gy*gy),shift=reach-d;
            if(!double.IsFinite(d)||d<0||!double.IsFinite(xm)||!double.IsFinite(xp)||!double.IsFinite(ym)||!double.IsFinite(yp)||!double.IsFinite(norm)||norm<.25||norm>2||!double.IsFinite(shift)||shift<0||shift>fullRadius){c.Unknown++;return true;}
            double x=p.X+gx/norm*shift,y=p.Y+gy/norm*shift;c.MaximumProjection=Math.Max(c.MaximumProjection,shift);
            if(!double.IsFinite(x)||!double.IsFinite(y)||Math.Abs(x)>1e6||Math.Abs(y)>1e6){c.Unknown++;return true;}
            c.Distances++;double targetDistance=artworkDistance(x,y);
            if(!double.IsFinite(targetDistance)||targetDistance<d){c.Unknown++;return true;}
            bool covered=scene!.TryCovered(x,y,out bool unknown);
            if(covered){c.Covered++;return false;}
            if(unknown)c.Unknown++;else c.Uncovered++;return true;
        }
    }
    private static bool Finite(Point3D p)=>double.IsFinite(p.X)&&double.IsFinite(p.Y)&&double.IsFinite(p.Z)&&Math.Abs(p.X)<=1e6&&Math.Abs(p.Y)<=1e6;
    private static bool IsSupplement(string category)=>category==VCarveInitialReachSelection.HalfCoreMarkerCategory
        ||category==VCarveInitialReachSelection.QuarterOuterMarkerCategory||category=="clearing-eighth-core-nomination"||category=="clearing-inner-reach-nomination";
}
