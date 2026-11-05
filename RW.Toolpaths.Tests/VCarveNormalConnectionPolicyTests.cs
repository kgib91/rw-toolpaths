using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class VCarveNormalConnectionPolicyTests
{
    [Theory]
    [InlineData(null,true)]
    [InlineData(false,false)]
    public void NormalSmoothGenerationWithMachineRatesSelectsTheCostBasedRouterUnlessAnIsolatedControlIsRequested(bool? control,bool expected)
    {
        IReadOnlyList<PointD> outer=[new(0,0),new(20,0),new(20,14),new(0,14)];
        IReadOnlyList<PointD> island=[new(7,4),new(7,10),new(13,10),new(13,4)];
        var profile=new VCarveCutterProfile([new(CarvingCutterSectionKind.Hemisphere,0,.5,.5),new(CarvingCutterSectionKind.Linear,.5,1.5,5)],5);
        var previous=VCarvePathRouter.Observer.Value;var requests=new List<VCarvePathRouter.Request>();
        var previousControl=VCarvePathRouter.ConnectionOptimizationOverride.Value;
        try{
            VCarvePathRouter.ConnectionOptimizationOverride.Value=control;
            VCarvePathRouter.Observer.Value=r=>requests.Add(r);
            var result=VCarveToolpaths.Generate([new MillingRegion(outer,[island])],new ToolGeometry(1.5){VCarveProfile=profile},
                new(1,.5){SurfaceZ=10},new VCarveOptions{SmoothVCarving=true,IncludeInteriorFill=false,StepOver=.45,
                    Motion=new(1000,500,5000,15,10),CalculateSweptArea=false},BoostVoronoiProvider.CreateDefault());
            Assert.NotEmpty(result.Toolpaths);Assert.NotEmpty(requests);
            Assert.All(requests,r=>{Assert.Equal(expected,r.Options.OptimizeContourConnections);Assert.False(r.Options.PreserveContourWallIdentity);Assert.NotNull(r.CutGuard);});
        }finally{VCarvePathRouter.Observer.Value=previous;VCarvePathRouter.ConnectionOptimizationOverride.Value=previousControl;}
    }
}
