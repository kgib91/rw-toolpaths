using System.Runtime.CompilerServices;
using Clipper2Lib;
using RW.Toolpaths.Milling;
using Xunit.Abstractions;

namespace RW.Toolpaths.Tests;

public sealed class VCarveProfileTraversalTests(ITestOutputHelper output)
{
    private static VCarveCutterProfile Profile()=>new([
        new(CarvingCutterSectionKind.Hemisphere,0,.79375,.79375),
        new(CarvingCutterSectionKind.Linear,.79375,3,2.20625),
        new(CarvingCutterSectionKind.Linear,3,3,20)],12.7);
    // Original public-wrapper traversal, with every expression unchanged.
    private static double LiteralDepthAtRadius(VCarveCutterProfile profile,double clearance)
    {
        if(double.IsNaN(clearance))throw new ArgumentOutOfRangeException(nameof(clearance));
        if(clearance<profile.TipRadius)return 0;
        if(clearance>=profile.PhysicalProfile.MaximumRadius)return profile.MaximumDepth;
        double height=0;
        foreach(var section in profile.Sections)
        {
            if(height>=profile.MaximumDepth)break;
            double activeHeight=Math.Min(section.Height,profile.MaximumDepth-height);
            double topRadius=section.Kind==CarvingCutterSectionKind.Hemisphere
                ?Math.Sqrt(activeHeight*(2*section.TopRadius-activeHeight))
                :section.BottomRadius+(section.TopRadius-section.BottomRadius)*activeHeight/section.Height;
            if(clearance>=topRadius){height+=activeHeight;continue;}
            double localDepth;
            if(section.Kind==CarvingCutterSectionKind.Hemisphere)
            {
                double radius=section.TopRadius;
                localDepth=clearance*clearance/(radius+Math.Sqrt(Math.Max(0,radius*radius-clearance*clearance)));
            }
            else localDepth=(clearance-section.BottomRadius)*section.Height/(section.TopRadius-section.BottomRadius);
            return Math.Clamp(height+localDepth,0,profile.MaximumDepth);
        }
        return profile.MaximumDepth;
    }

    [Fact]
    public void SpanTraversalKeepsEveryDepthBitAtSectionBoundariesPlateausAndPartialDepthCaps()
    {
        var profiles=new[]{Profile(),new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear,1,1,4)],2.5),
            new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear,0,3,3),new(CarvingCutterSectionKind.Linear,3,3,5)],7.2),
            new VCarveCutterProfile([new(CarvingCutterSectionKind.Hemisphere,0,1,1),new(CarvingCutterSectionKind.Linear,1,2,4)],2.5)};
        foreach(var profile in profiles)
        {
            var values=new List<double>{double.NegativeInfinity,-0d,0,double.PositiveInfinity};
            foreach(double r in profile.Sections.SelectMany(s=>new[]{s.BottomRadius,s.TopRadius}).Append(profile.PhysicalProfile.MaximumRadius))
                values.AddRange([Math.BitDecrement(r),r,Math.BitIncrement(r)]);
            for(int i=0;i<=100;i++)values.Add(profile.PhysicalProfile.MaximumRadius*i/100d);
            foreach(double clearance in values)
                Assert.Equal(BitConverter.DoubleToInt64Bits(LiteralDepthAtRadius(profile,clearance)),
                    BitConverter.DoubleToInt64Bits(profile.DepthAtRadius(clearance)));
            Assert.Equal("clearance",Assert.Throws<ArgumentOutOfRangeException>(()=>profile.DepthAtRadius(double.NaN)).ParamName);
        }
    }

    [Fact]
    public void RepresentedHeightCachesTheOriginalSelectorSumWithoutReassociationOrMutableAliases()
    {
        var heights=new[]{1e16}.Concat(Enumerable.Repeat(1d,17)).ToArray();
        var sections=heights.Select(h=>new CarvingCutterSection(CarvingCutterSectionKind.Linear,1,1,h)).ToArray();
        var profile=new VCarveCutterProfile(sections,1);
        double original=profile.Sections.Sum(s=>s.Height),sequential=0;foreach(double h in heights)sequential+=h;
        output.WriteLine($"literalSelectorSum={original:R} sequentialSum={sequential:R} runtime={Environment.Version}");
        Assert.Equal(BitConverter.DoubleToInt64Bits(original),BitConverter.DoubleToInt64Bits(profile.RepresentedHeight));
        sections[0]=new(CarvingCutterSectionKind.Linear,9,9,1);
        Assert.Equal(1e16,profile.Sections[0].Height);Assert.Equal(1,profile.Sections[0].BottomRadius);
        Assert.Equal(BitConverter.DoubleToInt64Bits(original),BitConverter.DoubleToInt64Bits(profile.RepresentedHeight));
        Assert.Throws<NotSupportedException>(()=>((IList<CarvingCutterSection>)profile.Sections)[0]=sections[0]);
        foreach(double depth in new[]{Math.BitDecrement(original),original,Math.BitIncrement(original)})
            Assert.Equal(depth<=profile.Sections.Sum(s=>s.Height),depth<=profile.RepresentedHeight);
    }

    [Fact]
    public void NativeEnvelopeKeepsTheExactCompleteProfileHeightAdmissionBoundary()
    {
        var profile=new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear,1,1,2),new(CarvingCutterSectionKind.Linear,1,2,3)],1);
        PointD[] Box(double r)=>[new(-r,-r),new(r,-r),new(r,r),new(-r,r)];
        var stock=new VCarveProjectedStock([],[Box(20)],Box(30),profile,profile.MaximumDepth,default);
        double represented=profile.Sections.Sum(s=>s.Height);
        foreach(double depth in new[]{1d,Math.BitDecrement(represented),represented,Math.BitIncrement(represented),double.PositiveInfinity,double.NaN})
        {
            var pose=new Point3D(0,0,-depth);
            Assert.Equal(double.IsFinite(depth)&&depth<=represented,stock.ProveNativeEnvelope(pose,pose,0));
        }
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static double RepeatDepth(VCarveCutterProfile profile,int count,bool literal)
    {
        double result=0;for(int i=0;i<count;i++)result+=literal?LiteralDepthAtRadius(profile,1.2):profile.DepthAtRadius(1.2);
        return result;
    }
    [MethodImpl(MethodImplOptions.NoInlining)]
    private static double RepeatHeight(VCarveCutterProfile profile,int count,bool literal)
    {
        double result=0;for(int i=0;i<count;i++)result+=literal?profile.Sections.Sum(s=>s.Height):profile.RepresentedHeight;
        return result;
    }
    [Fact]
    public void WarmedDepthAndHeightQueriesAvoidPublicWrapperEnumerators()
    {
        var profile=Profile();for(int i=0;i<32;i++){RepeatDepth(profile,64,true);RepeatDepth(profile,64,false);RepeatHeight(profile,64,true);RepeatHeight(profile,64,false);}
        const int count=16_384;
        long before=GC.GetAllocatedBytesForCurrentThread();double oldDepth=RepeatDepth(profile,count,true);long oldDepthBytes=GC.GetAllocatedBytesForCurrentThread()-before;
        before=GC.GetAllocatedBytesForCurrentThread();double depth=RepeatDepth(profile,count,false);long depthBytes=GC.GetAllocatedBytesForCurrentThread()-before;
        before=GC.GetAllocatedBytesForCurrentThread();double oldHeight=RepeatHeight(profile,count,true);long oldHeightBytes=GC.GetAllocatedBytesForCurrentThread()-before;
        before=GC.GetAllocatedBytesForCurrentThread();double height=RepeatHeight(profile,count,false);long heightBytes=GC.GetAllocatedBytesForCurrentThread()-before;
        output.WriteLine($"queries={count} literalDepthBytes={oldDepthBytes} spanDepthBytes={depthBytes} literalHeightBytes={oldHeightBytes} cachedHeightBytes={heightBytes}");
        Assert.Equal(BitConverter.DoubleToInt64Bits(oldDepth),BitConverter.DoubleToInt64Bits(depth));
        Assert.Equal(BitConverter.DoubleToInt64Bits(oldHeight),BitConverter.DoubleToInt64Bits(height));
        Assert.Equal(0,depthBytes);Assert.Equal(0,heightBytes);Assert.True(oldDepthBytes>=count*16L);Assert.True(oldHeightBytes>=count*16L);
    }
}
