using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class PocketAdaptiveNegativeWitnessTests
{
    [Theory]
    [InlineData(.25,.5,false,false)]
    [InlineData(.75,1.25,false,true)]
    [InlineData(.125,.5,true,false)]
    [InlineData(.375,.5,true,true)]
    [InlineData(.0625,.25,false,false)]
    [InlineData(.9375,.25,true,true)]
    public void NarrowProtectedGapsMissedByTheOriginalThreeSitesRetainTheCompleteReferenceRefusal(
        double fraction,double radius,bool rotate,bool reverse)
    {
        var (stock,from,to)=Gap(fraction,radius,rotate,reverse);
        var adaptiveDiagnostics=new PocketContinuationRouter.CoverageDiagnostics();
        var originalDiagnostics=new PocketContinuationRouter.CoverageDiagnostics();
        var adaptive=Proof(stock,radius,adaptiveDiagnostics,29);
        var original=Proof(stock,radius,originalDiagnostics,0);
        var complete=Reference(stock,radius);
        Assert.False(complete.IsTravelSafe(from,to,radius));
        Assert.False(original.IsTravelSafe(from,to,radius));
        Assert.False(adaptive.IsTravelSafe(from,to,radius));
        Assert.Equal(1,originalDiagnostics.WholeFallbacks);
        Assert.Equal(0,originalDiagnostics.NegativeWitnesses);
        Assert.Equal(1,adaptiveDiagnostics.AdditionalNegativeWitnessRefusals);
        Assert.Equal(1,adaptiveDiagnostics.NegativeWitnesses);
        Assert.Equal(0,adaptiveDiagnostics.WholeFallbacks);
        Assert.InRange(adaptiveDiagnostics.AdditionalNegativeSites,1,29);
        Assert.InRange(adaptiveDiagnostics.AdditionalNegativeQueries,1,203);
        // The diagnostic validation path independently builds the complete
        // original morphology and would throw on any incorrect refusal.
        Assert.Equal(0,adaptiveDiagnostics.LocalProofMismatches);
    }

    [Fact]
    public void ExhaustedAdditionalSiteBudgetRetainsTheOriginalWholeFallback()
    {
        var (stock,from,to)=Gap(.3,.5,false,false);
        var diagnostics=new PocketContinuationRouter.CoverageDiagnostics();
        var adaptive=Proof(stock,.5,diagnostics,1);
        Assert.False(adaptive.IsTravelSafe(from,to,.5));
        Assert.False(Reference(stock,.5).IsTravelSafe(from,to,.5));
        Assert.Equal(1,diagnostics.AdditionalNegativeSites);
        Assert.Equal(0,diagnostics.AdditionalNegativeWitnessRefusals);
        Assert.Equal(1,diagnostics.AdditionalNegativeLimitFallbacks);
        Assert.Equal(1,diagnostics.WholeFallbacks);
    }

    [Theory]
    [InlineData(29)]
    [InlineData(125)]
    public void UncertainPositiveCapsuleStillUsesTheUnchangedCompleteMorphology(int additional)
    {
        const double radius=.5,halfWidth=radius-.0028;
        IReadOnlyList<PointD>[] air=[Rect(-4,-halfWidth,20,halfWidth)];
        var stock=new PocketContinuationRouter.ClearedStock(radius,default,knownEmpty:air,knownEmptyFloor:-1);
        var diagnostics=new PocketContinuationRouter.CoverageDiagnostics();
        var adaptive=Proof(stock,radius,diagnostics,additional);
        Point3D from=new(0,0,-1),to=new(16,0,-1);
        Assert.True(Reference(stock,radius).IsTravelSafe(from,to,radius));
        Assert.True(adaptive.IsTravelSafe(from,to,radius));
        Assert.Equal(additional,diagnostics.AdditionalNegativeSites);
        Assert.Equal(7*additional,diagnostics.AdditionalNegativeQueries);
        Assert.Equal(0,diagnostics.NegativeWitnesses);
        Assert.Equal(1,diagnostics.AdditionalNegativeLimitFallbacks);
        Assert.Equal(1,diagnostics.WholeFallbacks);
        Assert.Equal(1,diagnostics.WholeFallbackAccepted);
        Assert.Equal(1,diagnostics.WholeFallbackAfterWitnessLimitAccepted);
    }

    [Fact]
    public void ExtendedWitnessBudgetFindsAThinGapBeyondTheOriginalThirtyTwoSites()
    {
        const double radius=.5,halfWidth=radius-.0028,gap=.25;
        // The uncertain outer capsule traces keep the best-first search broad.
        // Original sites lie on half-mm positions; the gap needs t=1/64,
        // after the original 32-site prefix, before it can witness a refusal.
        IReadOnlyList<PointD>[] air=[Rect(-4,-halfWidth,gap-.03,halfWidth),
            Rect(gap+.03,-halfWidth,20,halfWidth)];
        var stock=new PocketContinuationRouter.ClearedStock(radius,default,knownEmpty:air,knownEmptyFloor:-1);
        Point3D from=new(0,0,-1),to=new(16,0,-1);
        var originalDiagnostics=new PocketContinuationRouter.CoverageDiagnostics();
        var extendedDiagnostics=new PocketContinuationRouter.CoverageDiagnostics();
        Assert.False(Reference(stock,radius).IsTravelSafe(from,to,radius));
        Assert.False(Proof(stock,radius,originalDiagnostics,29).IsTravelSafe(from,to,radius));
        Assert.False(Proof(stock,radius,extendedDiagnostics,125).IsTravelSafe(from,to,radius));
        Assert.Equal(29,originalDiagnostics.AdditionalNegativeSites);
        Assert.Equal(0,originalDiagnostics.AdditionalNegativeWitnessRefusals);
        Assert.Equal(1,originalDiagnostics.WholeFallbacks);
        Assert.Equal(1,originalDiagnostics.WholeFallbackRefused);
        Assert.Equal(1,originalDiagnostics.WholeFallbackAfterWitnessLimitRefused);
        Assert.InRange(extendedDiagnostics.AdditionalNegativeSites,30,125);
        Assert.Equal(1,extendedDiagnostics.AdditionalNegativeWitnessRefusals);
        Assert.Equal(0,extendedDiagnostics.WholeFallbacks);
        Assert.Equal(128,extendedDiagnostics.MaximumNegativeWitnessSites);
    }

    [Fact]
    public void SuccessfulUndilatedProofNeverRunsTheOptionalAdditionalSearch()
    {
        var stock=new PocketContinuationRouter.ClearedStock(.5,default);
        stock.Record([new(0,0,-1),new(30,0,-1)]);
        IReadOnlyList<PointD>[] following=[[new(40,0),new(50,0)]];
        var diagnostics=new PocketContinuationRouter.CoverageDiagnostics();
        var proof=new PocketContinuationRouter.TemporalClearance(LinkClearance.Unrestricted,stock,-1,following,.5,
            default,diagnostics,validateIndexedCoverage:true,useLocalTileCache:false);
        for(int i=0;i<10;i++)
        {
            double x=i%2==0?1+i:40.5+i/2;
            Assert.True(proof.IsTravelSafe(new(x,0,-1),new(x+.25,0,-1),.5));
        }
        Assert.True(diagnostics.UndilatedProofs>0);
        Assert.Equal(0,diagnostics.AdditionalNegativeSearches);
        Assert.Equal(0,diagnostics.AdditionalNegativeSites);
        Assert.Equal(0,diagnostics.AdditionalNegativeSeedQueries);
        Assert.Equal(0,diagnostics.WholeFallbacks);
    }

    [Fact]
    public void OptionalWitnessesRemainBoundToTheCapturedPrefixAndFloor()
    {
        var (stock,from,to)=Gap(.25,.5,false,false);
        var original=Proof(stock,.5,new(),29);
        Assert.False(original.IsTravelSafe(from,to,.5));
        stock.Record([from,to]);
        stock.AtFloor(-1);
        Assert.False(original.IsTravelSafe(from,to,.5));
        Assert.True(Proof(stock,.5,new(),29).IsTravelSafe(from,to,.5));
        var deeper=new PocketContinuationRouter.TemporalClearance(LinkClearance.Unrestricted,stock,-2,
            Array.Empty<IReadOnlyList<PointD>>(),.5,default,validateIndexedCoverage:true);
        Assert.False(deeper.IsTravelSafe(new(from.X,from.Y,-2),new(to.X,to.Y,-2),.5));
    }

    [Fact]
    public void AdditionalWitnessBoundIsExplicitAndLibraryDefaultRemainsThirtyTwoTotalSites()
    {
        var (stock,_,_)=Gap(.25,.5,false,false);
        Assert.Throws<ArgumentOutOfRangeException>(()=>Proof(stock,.5,new(),-1));
        Assert.Throws<ArgumentOutOfRangeException>(()=>Proof(stock,.5,new(),126));
        Assert.Equal(29,new PocketOptions().FlatContinuationAdditionalWitnessSites);
        var diagnostics=new PocketContinuationRouter.CoverageDiagnostics();
        _=new PocketContinuationRouter.TemporalClearance(LinkClearance.Unrestricted,stock,-1,
            Array.Empty<IReadOnlyList<PointD>>(),.5,default,diagnostics);
        Assert.Equal(32,diagnostics.MaximumNegativeWitnessSites);
    }

    private static PocketContinuationRouter.TemporalClearance Proof(PocketContinuationRouter.ClearedStock stock,
        double radius,PocketContinuationRouter.CoverageDiagnostics diagnostics,int additional)
        => new(LinkClearance.Unrestricted,stock,-1,Array.Empty<IReadOnlyList<PointD>>(),radius,default,
            diagnostics,validateIndexedCoverage:true,maximumAdditionalWitnessSites:additional);

    private static PocketContinuationRouter.TemporalClearance Reference(PocketContinuationRouter.ClearedStock stock,double radius)
        => new(LinkClearance.Unrestricted,stock,-1,Array.Empty<IReadOnlyList<PointD>>(),radius,default,
            useIndexedCoverage:false,referenceDilation:true);

    private static (PocketContinuationRouter.ClearedStock Stock,Point3D From,Point3D To) Gap(
        double fraction,double radius,bool rotate,bool reverse)
    {
        var hole=Rect(16*fraction-.03,-3,16*fraction+.03,3);Array.Reverse(hole);
        IReadOnlyList<PointD>[] air=[Rect(-4,-4,20,4),hole];
        if(rotate)air=air.Select(path=>(IReadOnlyList<PointD>)path.Select(p=>new PointD(-p.y,p.x)).ToArray()).ToArray();
        var stock=new PocketContinuationRouter.ClearedStock(radius,default,knownEmpty:air,knownEmptyFloor:-1);
        Point3D from=new(0,0,-1),to=rotate?new(0,16,-1):new(16,0,-1);
        return reverse?(stock,to,from):(stock,from,to);
    }

    private static PointD[] Rect(double left,double bottom,double right,double top)
        => [new(left,bottom),new(right,bottom),new(right,top),new(left,top)];
}
