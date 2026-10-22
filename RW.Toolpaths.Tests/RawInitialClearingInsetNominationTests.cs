using System.Reflection;
using System.Runtime.ExceptionServices;
using Clipper2Lib;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;
using RW.Toolpaths.Milling.Acceleration;
using Xunit;

namespace RW.Toolpaths.Tests;

[Collection("Pocket construction policy controls")]
public sealed class RawInitialClearingInsetNominationTests
{
    private static IReadOnlyList<PointD> Square(double x,double size)
        =>new PointD[]{new(x,0),new(x+size,0),new(x+size,size),new(x,size)};
    private static IReadOnlyList<IReadOnlyList<PointD>> Boundary(double size=4)
        =>new[]{Square(0,size)};
    private static RawInitialClearingInsetNomination Capability()
        =>RawInitialClearingInsetNomination.CaptureForSmoothGeneration(true,15,true)!;
    private static VCarveCutterProfile Profile()
        =>new(new[]{new CarvingCutterSection(CarvingCutterSectionKind.Linear,.1,.2,1)},.5);
    private static List<TaggedToolpath> Core(IReadOnlyList<IReadOnlyList<PointD>> boundary,
        RawInitialClearingInsetNomination? capability,MedialAxisToolpaths.CertifiedInsetReuse cache,
        List<string> logs,CancellationToken token=default)
    {
        var method=typeof(MedialAxisToolpaths).GetMethod("GenerateClearingPassesTaggedCore",BindingFlags.Static|BindingFlags.NonPublic)!;
        var values=new Dictionary<string,object?>
        {
            ["boundary"]=boundary,["startDepth"]=0d,["endDepth"]=.5d,["radianTipAngle"]=Math.PI/2,
            ["depthPerPass"]=.5d,["stepOver"]=.4d,["regionIndex"]=0,
            ["bottomRadiusOverride"]=.1d,["topRadiusOverride"]=.2d,["coneLengthOverride"]=1d,
            ["includeInteriorFill"]=false,["detectCurves"]=false,["diagnostic"]=(Action<string>)logs.Add,
            ["cutterProfile"]=Profile(),["cancellationToken"]=token,["insetReuse"]=cache,
            ["initialInsetNomination"]=capability
        };
        var args=method.GetParameters().Select(p=>values.TryGetValue(p.Name!,out var v)?v:
            p.HasDefaultValue?p.DefaultValue:throw new InvalidDataException("unexpected-required-core-parameter:"+p.Name)).ToArray();
        try{return (List<TaggedToolpath>)method.Invoke(null,args)!;}
        catch(TargetInvocationException e)when(e.InnerException is not null)
        {ExceptionDispatchInfo.Capture(e.InnerException).Throw();throw;}
    }
    private static void EqualPaths(IReadOnlyList<TaggedToolpath> a,IReadOnlyList<TaggedToolpath> b)
    {
        Assert.Equal(a.Count,b.Count);
        for(int i=0;i<a.Count;i++)
        {Assert.Equal(a[i].Points,b[i].Points);Assert.Equal(a[i].Category,b[i].Category);Assert.Equal(a[i].DepthPassIndex,b[i].DepthPassIndex);}
    }

    [Fact]
    public void CapabilityRequiresSmoothOwnerAndValidatedAngle()
    {
        Assert.NotNull(Capability());
        Assert.Null(RawInitialClearingInsetNomination.CaptureForSmoothGeneration(false,15,true));
        Assert.Null(RawInitialClearingInsetNomination.CaptureForSmoothGeneration(true,15,false));
        foreach(double angle in new[]{0d,90d,-1d,double.NaN,double.PositiveInfinity})
            Assert.Null(RawInitialClearingInsetNomination.CaptureForSmoothGeneration(true,angle,true));
    }

    [Fact]
    public void FullOriginalSourceUsesLiteralReserveAndRetainsTinyIslandAsNomination()
    {
        var logs=new List<string>();var source=new[]{Square(0,4),Square(5,.01)};
        Assert.True(Capability().TryNominate(source,.001,CancellationToken.None,logs.Add,out var rings,out _));
        Assert.NotNull(rings);Assert.Equal(2,rings!.Count);
        Assert.Contains(rings,r=>r.All(p=>p.x>5));
        double q=1/PathUtils.Scale;float f=5.01f;
        double reserve=Math.Max(8*q,Math.Sqrt(2)*(MathF.BitIncrement(f)-f)/2+Math.Sqrt(2)*.0005+6*q);
        Assert.Contains(logs,s=>s.Contains("constructionBits="+BitConverter.DoubleToInt64Bits(.001+reserve))
            &&s.Contains("sourcePoints=8")&&s.Contains("sourceReduction=False")&&s.Contains("nativeArcTicks=1")
            &&s.Contains("physicalCredit=False")&&s.Contains("CcoverageCredit=False")&&s.Contains("stockCredit=False"));
    }

    [Fact]
    public void EligiblePrivateCoreBypassesCertifiedCacheAndKeepsCaptureAcrossEnvironmentChange()
    {
        var cap=Capability();var logs=new List<string>();using var cache=new MedialAxisToolpaths.CertifiedInsetReuse();
        string? before=Environment.GetEnvironmentVariable(RawInitialClearingInsetNomination.EnvironmentName);
        try
        {
            Environment.SetEnvironmentVariable(RawInitialClearingInsetNomination.EnvironmentName,"0");
            var paths=Core(Boundary(),cap,cache,logs);Assert.NotEmpty(paths);
            Assert.Contains(logs,s=>s.Contains("stage=raw-initial-clearing-inset")&&s.Contains("status=nominated"));
            Assert.Equal(0,cache.PreparedCount);Assert.Equal(0,cache.ReuseCount);
        }
        finally{Environment.SetEnvironmentVariable(RawInitialClearingInsetNomination.EnvironmentName,before);}
    }

    [Fact]
    public void UnsupportedEmptyNominationFallsBackToCompleteOriginalCertifiedContract()
    {
        var logs=new List<string>();using var cache=new MedialAxisToolpaths.CertifiedInsetReuse();
        var actual=Core(Boundary(.01),Capability(),cache,logs);
        Assert.Empty(actual);Assert.Equal(1,cache.PreparedCount);
        Assert.Contains(logs,s=>s.Contains("raw-initial-empty-or-degenerate-NOM-needs-original-fallback"));
        using var originalCache=new MedialAxisToolpaths.CertifiedInsetReuse();
        EqualPaths(Core(Boundary(.01),null,originalCache,new()),actual);
    }

    [Fact]
    public void PublicStandaloneClearingIsLiteralEvenWhenEnvironmentRequestsRawMode()
    {
        string? before=Environment.GetEnvironmentVariable(RawInitialClearingInsetNomination.EnvironmentName);
        try
        {
            List<TaggedToolpath> Run(string mode,List<string> logs)
            {
                Environment.SetEnvironmentVariable(RawInitialClearingInsetNomination.EnvironmentName,mode);
                return MedialAxisToolpaths.GenerateClearingPassesTagged(Boundary(),0,.5,Math.PI/2,.5,
                    includeInteriorFill:false,detectCurves:false,diagnostic:logs.Add,cutterProfile:Profile());
            }
            var offLogs=new List<string>();var onLogs=new List<string>();
            EqualPaths(Run("0",offLogs),Run("1",onLogs));
            Assert.DoesNotContain(onLogs,s=>s.Contains("stage=raw-initial-clearing-inset"));
            Assert.DoesNotContain(typeof(MedialAxisToolpaths).GetMethods(BindingFlags.Public|BindingFlags.Static)
                .SelectMany(m=>m.GetParameters()),p=>p.ParameterType==typeof(RawInitialClearingInsetNomination));
        }
        finally{Environment.SetEnvironmentVariable(RawInitialClearingInsetNomination.EnvironmentName,before);}
    }

    [Fact]
    public void OriginalMedialCertifiedCacheRemainsLiteralAndCancellationCannotReturnNomination()
    {
        using var cache=new MedialAxisToolpaths.CertifiedInsetReuse();var source=Boundary();
        var first=cache.Contract(source,.15);var second=cache.Contract(source,.15);
        Assert.Equal(1,cache.PreparedCount);Assert.Equal(1,cache.ReuseCount);
        Assert.Equal(first.Count,second.Count);for(int i=0;i<first.Count;i++)Assert.Equal(first[i],second[i]);
        using var canceled=new CancellationTokenSource();canceled.Cancel();
        Assert.Throws<OperationCanceledException>(()=>Capability().TryNominate(source,.15,canceled.Token,null,out _,out _));
        using var cancelInDiagnostic=new CancellationTokenSource();
        Assert.Throws<OperationCanceledException>(()=>Capability().TryNominate(source,.15,cancelInDiagnostic.Token,
            _=>cancelInDiagnostic.Cancel(),out _,out _));
    }
}
