using System.Reflection;
using System.Runtime.ExceptionServices;
using Clipper2Lib;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;
using RW.Toolpaths.Milling.Acceleration;
using Xunit;

namespace RW.Toolpaths.Tests;

[Collection("Pocket construction policy controls")]
public sealed class RawInitialClearingBandNominationTests
{
    private static IReadOnlyList<IReadOnlyList<PointD>> Boundary()
        =>new[]{(IReadOnlyList<PointD>)new PointD[]{new(0,0),new(4,0),new(4,4),new(0,4)}};
    private static RawInitialClearingInsetNomination Capability()
        =>RawInitialClearingInsetNomination.CaptureForSmoothGeneration(true,15,true)!;
    private static List<TaggedToolpath> Core(double edgeWidth,RawInitialClearingInsetNomination? cap,List<string> logs)
    {
        var method=typeof(MedialAxisToolpaths).GetMethod("GenerateClearingPassesTaggedCore",BindingFlags.Static|BindingFlags.NonPublic)!;
        var values=new Dictionary<string,object?>
        {
            ["boundary"]=Boundary(),["startDepth"]=0d,["endDepth"]=.5d,["radianTipAngle"]=Math.PI/2,
            ["depthPerPass"]=.5d,["stepOver"]=.4d,["regionIndex"]=0,
            ["bottomRadiusOverride"]=.1d,["topRadiusOverride"]=.2d,["coneLengthOverride"]=1d,
            ["includeInteriorFill"]=false,["detectCurves"]=false,["diagnostic"]=(Action<string>)logs.Add,
            ["cutterProfile"]=new VCarveCutterProfile(new[]{new CarvingCutterSection(CarvingCutterSectionKind.Linear,.1,.2,1)},.5),
            ["edgeFillWidth"]=edgeWidth,["initialInsetNomination"]=cap
        };
        var args=method.GetParameters().Select(p=>values.TryGetValue(p.Name!,out var v)?v:
            p.HasDefaultValue?p.DefaultValue:throw new InvalidDataException("unexpected-required-core-parameter:"+p.Name)).ToArray();
        try{return (List<TaggedToolpath>)method.Invoke(null,args)!;}
        catch(TargetInvocationException e)when(e.InnerException is not null)
        {ExceptionDispatchInfo.Capture(e.InnerException).Throw();throw;}
    }

    [Fact]
    public void RawBandUsesExactAuthoredWidthWithoutFirstInsetReserveOrSourceReduction()
    {
        var logs=new List<string>();
        Assert.True(Capability().TryNominateAuthoredBand(Boundary(),.5,.15,CancellationToken.None,logs.Add,out var candidate,out _));
        var ring=Assert.Single(candidate!);
        // Independent axis-aligned finite-wall geometry: literal half-unit
        // erosion, not the first-inset compensated radius or an old path match.
        Assert.Equal(.5,ring.Min(p=>p.x));Assert.Equal(.5,ring.Min(p=>p.y));
        Assert.Equal(3.5,ring.Max(p=>p.x));Assert.Equal(3.5,ring.Max(p=>p.y));
        Assert.Contains(logs,s=>s.Contains("widthBits="+BitConverter.DoubleToInt64Bits(.5))&&s.Contains("addedWidthReserve=0")
            &&s.Contains("sourcePoints=4")&&s.Contains("sourceReduction=False")&&s.Contains("physicalCredit=False"));
    }

    [Fact]
    public void EligiblePrivateBandSharesTerminalCoreAndKeepsAuthoredReach()
    {
        var logs=new List<string>();var paths=Core(.25,Capability(),logs);Assert.NotEmpty(paths);
        Assert.Contains(logs,s=>s.Contains("stage=raw-initial-clearing-band")&&s.Contains("status=nominated"));
        Assert.Contains(logs,s=>s.Contains("stage=clearing-band event=prepared")&&s.Contains("rawBandNominated=True")
            &&s.Contains("terminalSharesSameCore=True")&&s.Contains("physicalCredit=False"));
        Assert.Contains(logs,s=>s.Contains("terminalCoreContour=True"));
        Assert.Contains(paths.SelectMany(p=>p.Points),p=>p.X==.25||p.Y==.25);
        Assert.Null(RawInitialClearingInsetNomination.CaptureForSmoothGeneration(false,15,true));
        Assert.DoesNotContain(typeof(MedialAxisToolpaths).GetMethods(BindingFlags.Public|BindingFlags.Static)
            .SelectMany(m=>m.GetParameters()),p=>p.ParameterType==typeof(RawInitialClearingInsetNomination));
    }

    [Fact]
    public void UnsupportedEmptyBandUsesWholeOldBandAndDefaultCapabilityRemainsLiteral()
    {
        var logs=new List<string>();Assert.False(Capability().TryNominateAuthoredBand(Boundary(),3,.15,
            CancellationToken.None,logs.Add,out var candidate,out string reason));
        Assert.Null(candidate);Assert.Contains("whole-band-fallback",reason);
        var fallbackLogs=new List<string>();Core(3,Capability(),fallbackLogs);
        Assert.Contains(fallbackLogs,s=>s.Contains("stage=clearing-band event=prepared")&&s.Contains("rawBandNominated=False")
            &&s.Contains("canonicalCoreOwner=shared-prepared-offset"));
        var defaultLogs=new List<string>();Core(.25,null,defaultLogs);
        Assert.DoesNotContain(defaultLogs,s=>s.Contains("stage=raw-initial-clearing-band"));
        Assert.Contains(defaultLogs,s=>s.Contains("rawBandNominated=False"));
        Assert.False(Capability().TryNominateAuthoredBand(Boundary(),.15,.15,CancellationToken.None,null,out _,out _));
    }

    [Fact]
    public void RawBandCannotReturnAfterPreCancellationOrDiagnosticCancellation()
    {
        using var before=new CancellationTokenSource();before.Cancel();
        Assert.Throws<OperationCanceledException>(()=>Capability().TryNominateAuthoredBand(Boundary(),.5,.15,before.Token,null,out _,out _));
        using var callback=new CancellationTokenSource();
        Assert.Throws<OperationCanceledException>(()=>Capability().TryNominateAuthoredBand(Boundary(),.5,.15,callback.Token,
            _=>callback.Cancel(),out _,out _));
    }
}
