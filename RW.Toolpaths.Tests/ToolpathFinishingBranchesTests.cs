using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class ToolpathFinishingBranchesTests
{
    private static TaggedToolpath Source(int owner,string category,params Point3D[] points)
        =>new(points,[],0,category,category=="clearing"?0:null,detectCurves:false){ContinuationSourceOrdinal=owner};
    private static TaggedToolpath Enter(TaggedToolpath source,double top)
    {
        var entry=RampPlanner.PlanPathEntry(source.Points,top,RampSettings.FromRatio(2),default,null,true,true);
        return source with{Points=entry.Points,Spans=entry.Spans};
    }
    private static (TaggedToolpath[] Required,List<TaggedToolpath> Entered) Example(double surface=0)
    {
        var required=new[]{Source(0,"clearing",new(0,0,surface-1),new(10,0,surface-1),new(10,10,surface-1),
                new(0,10,surface-1),new(0,0,surface-1)),
            Source(1,"final-carve",new(5,.03,surface-1),new(5,3,surface-.5),new(5,.03,surface-1))};
        return(required,required.Select(p=>Enter(p,surface+1)).ToList());
    }
    private static List<TaggedToolpath> Route(TaggedToolpath[] source,List<TaggedToolpath> entered,double surface=0,
        Func<Point3D,Point3D,bool>? guard=null,List<string>? log=null)
        =>ToolpathSourceTraversal.FinishingBranches(source,entered,entered,.1,.5,guard??((_,_)=>true),_=>surface,
            new MillingMotionSettings(1000,500,6000,surface+5),surface,null,default,log is null?null:log.Add);

    [Theory]
    [InlineData(0)]
    [InlineData(12.7)]
    public void GappedVariableHeightBranchRunsDuringOneCarrierCycle(double surface)
    {
        var (source,entered)=Example(surface);var logs=new List<string>();
        var result=Assert.Single(Route(source,entered,surface,log:logs));
        Assert.Equal(entered[0].Points[0],result.Points[0]);
        int landing=entered[0].Spans.First(s=>s.Kind==ToolpathSpanKind.Cut).StartIndex;
        Assert.Equal(entered[0].Points[landing],result.Points[landing]);
        Assert.Equal(result.Points[landing],result.Points[^1]);
        Assert.Contains(source[1].Points[1],result.Points);
        Assert.Equal(0,result.ContinuationSourceOrdinal);
        Assert.Contains(logs,l=>l.Contains("event=complete")&&l.Contains("accepted=True")&&l.Contains("sourceOwnership=stable-ordinal"));
    }

    [Fact]
    public void MissingStockProofRetainsOriginalIndependentEntries()
    {
        var (source,entered)=Example();
        Assert.Same(entered,Route(source,entered,guard:(_,_)=>false));
    }

    [Fact]
    public void MissingStableOwnershipDoesNotInferItFromCoincidentCoordinates()
    {
        var (source,entered)=Example();entered[1]=entered[1] with{ContinuationSourceOrdinal=null};
        var logs=new List<string>();Assert.Same(entered,Route(source,entered,log:logs));
        Assert.Contains(logs,l=>l.Contains("source-ownership-unavailable"));
    }

    [Fact]
    public void DifferentCarrierPassesCannotBeCollapsedIntoOneFinalLayer()
    {
        var (source,entered)=Example();
        source=[source[0],source[0] with{ContinuationSourceOrdinal=2,DepthPassIndex=1},source[1]];
        var logs=new List<string>();Assert.Same(entered,Route(source,entered,log:logs));
        Assert.Contains(logs,l=>l.Contains("carrier-or-pass-contract"));
    }

    [Fact]
    public void PostedGradeRefusalKeepsAnApparentlyValidNativeGapDisconnected()
    {
        var required=new[]{Source(0,"clearing",new(0,0,-1.00049),new(10,0,-1.00049),new(10,10,-1.00049),
                new(0,10,-1.00049),new(0,0,-1.00049)),
            Source(1,"final-carve",new(5,.0004,-1.00051),new(5,3,-.5),new(5,.0004,-1.00051))};
        var entered=required.Select(p=>Enter(p,1)).ToList();
        Assert.Same(entered,Route(required,entered));
    }

    [Fact]
    public void EntryLinksAndTabsKeepTheirOriginalExecutionContract()
    {
        var (source,entered)=Example();
        entered[1]=entered[1] with{Spans=[new(0,entered[1].Points.Count-1,ToolpathSpanKind.Link)]};
        var logs=new List<string>();Assert.Same(entered,Route(source,entered,log:logs));
        Assert.Contains(logs,l=>l.Contains("entered-motion-kinds"));
    }

    [Theory]
    [InlineData(.5,2)]
    [InlineData(1,1)]
    public void VCarveRouterOnlyAttachesTheAlreadyClearedFinalLayer(double startingDepth,int visits)
    {
        var (required,_)=Example();
        var result=VCarvePathRouter.Route(required,[],new ToolGeometry(1),new DepthSchedule(1,.25){StartDepth=startingDepth},
            new VCarveOptions{IncludeInteriorFill=false,DetectCurves=false,Motion=new(1000,500,6000,5)},0,null,
            continuationCutGuard:(_,_)=>true,materialEntryHeight:_=>0);
        Assert.Equal(visits,result.Count);
    }
}
