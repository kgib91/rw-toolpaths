using Clipper2Lib;
using RW.Toolpaths;
using RW.Toolpaths.Milling;
using Xunit;
public sealed class DepthLimitNativeFitTests
{
 static readonly VCarveCutterProfile Tool=new([new(CarvingCutterSectionKind.Linear,0,1,10)],2);
 static VCarveNativeContainment Guard(bool legal)=>new([new PointD[]{new(-5,-5),new(5,-5),new(5,5),new(-5,5)}],Tool,stockChord:(_,_)=>legal,stockCurve:_=>legal);
 static Point3D[] Source=>[new(0,0,-2),new(-.13,.02,-2),new(.3,.01,-2),new(.5,0,-2)];
 [Fact] public void DerivedCapSubdivisionDirectionDoesNotBlockCertifiedWholeLine(){
  var fit=VCarveNativeDepthLimitFit.Line(Source,Guard(true),Tool,new(.3,.3));
  Assert.NotNull(fit);Assert.True(fit.DepthLimitContour);Assert.Equal(ToolMoveKind.Line,fit.Move.Kind);
  Assert.Equal(Source[0],fit.Move.Start);Assert.Equal(Source[^1],fit.Move.End);Assert.InRange(fit.Move.MaxError,.13,.14);
 }
 [Fact] public void SameCandidateStillRequiresFullCutterAndConfiguredFit(){
  Assert.Null(VCarveNativeDepthLimitFit.Line(Source,Guard(false),Tool,new(.3,.3)));
  Assert.Null(VCarveNativeDepthLimitFit.Line(Source,Guard(true),Tool,new(.03,.3)));
 }
 [Fact] public void DepthVariationAndSubminimumAreNotAutomaticallyAccepted(){
  var changed=Source;changed[1]=changed[1] with{Z=-1.9};
  Assert.Null(VCarveNativeDepthLimitFit.Line(changed,Guard(true),Tool,new(.3,.3)));
  Assert.Null(VCarveNativeDepthLimitFit.Line([new(0,0,-2),new(.1,0,-2)],Guard(true),Tool,new(.3,.3)));
 }
}
