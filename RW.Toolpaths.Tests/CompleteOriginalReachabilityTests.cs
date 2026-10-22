using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling.Certification;
using Clipper2Lib;
using System.Text.Json;

namespace RW.Toolpaths.Tests;

[Collection("Local original medial diagnostic controls")]
public sealed class CompleteOriginalReachabilityTests
{
    [Fact]
    public void IndependentSignedPartitionFiniteCapsuleOwnerAndPhysicalObligationsPass()
    {
        string result=Path.Combine(Path.GetTempPath(),"rw-reachability-control-"+Guid.NewGuid().ToString("N")+".json");
        int exit=ReachabilityPartitionControlHarness.Run(new[]{result});
        if(exit!=0)
        {
            string causes="Result JSON was not produced.";
            if(File.Exists(result))
            {
                using var document=JsonDocument.Parse(File.ReadAllBytes(result));
                causes=string.Join(Environment.NewLine,document.RootElement.GetProperty("controls").EnumerateArray()
                    .Where(c=>!c.GetProperty("passed").GetBoolean())
                    .Select(c=>c.GetProperty("name").GetString()+": "+c.GetProperty("error").GetString()));
            }
            Assert.True(exit==0,$"Independent reachability controls failed; preserved evidence: {result}{Environment.NewLine}{causes}");
        }
        if(File.Exists(result))File.Delete(result);
    }

    [Fact]
    public void FractionalNominationUsesCompleteOriginalIndexWithoutChangingIntegerWitness()
    {
        var source=new Paths64{new Path64{new(-10,0),new(10,0),new(10,10),new(-10,10)}};
        var index=new PolygonOps.BoundarySegmentIndex(source);
        var before=index.NearestSegmentForCertificate(new Point64(0,1));
        var choice=new PreparedFractionalSiteChoice(index);
        var site=choice.Choose(-.125,.125);
        Assert.Equal((new Point64(-10,0),new Point64(10,0)),site);
        Assert.Equal(site,choice.Choose(-.125,.125));Assert.Equal(1,choice.MemoHits);
        Assert.Equal(before,index.NearestSegmentForCertificate(new Point64(0,1)));
    }

    [Theory]
    [InlineData(ClipType.Intersection,false)]
    [InlineData(ClipType.Intersection,true)]
    [InlineData(ClipType.Union,false)]
    [InlineData(ClipType.Union,true)]
    [InlineData(ClipType.Difference,false)]
    [InlineData(ClipType.Difference,true)]
    public void RefusedAuthoritativeBooleanNeverPublishesEmptyOrPartialCoverage(ClipType operation,bool partial)
    {
        var subjects=PathUtils.FromClipper(new Paths64{Rectangle(0,0,100,100)});
        var clips=PathUtils.FromClipper(new Paths64{Rectangle(20,20,80,80)});
        int executions=0;
        Assert.False(ExactFirstCoverageGate.TryCheckedWorldBoolean(operation,subjects,
            operation==ClipType.Union?null:clips,out var output,(engine,actualOperation,fill,native)=>{
                executions++;
                Assert.Equal(operation,actualOperation);Assert.Equal(FillRule.NonZero,fill);
                if(partial)native.Add(Rectangle(0,0,10,10));
                return false;
            }));
        Assert.Equal(1,executions);
        Assert.Empty(output); // Neither refusal can grant finished-stock evidence.
    }

    [Theory]
    [InlineData(ClipType.Intersection)]
    [InlineData(ClipType.Union)]
    [InlineData(ClipType.Difference)]
    public void CheckedBooleanPreservesOriginalSignedOperandsAndOrderedGeometry(ClipType operation)
    {
        var hole=Rectangle(30,30,70,70);hole.Reverse();
        var subjects=PathUtils.FromClipper(new Paths64{Rectangle(0,0,100,100),hole,Rectangle(200,0,220,20)});
        var clips=PathUtils.FromClipper(new Paths64{Rectangle(40,-10,210,110)});
        var expected=operation switch {
            ClipType.Intersection=>PolygonOps.Intersect(subjects,clips),
            ClipType.Difference=>PolygonOps.Difference(subjects,clips),
            _=>PolygonOps.Union(subjects),
        };
        Assert.True(ExactFirstCoverageGate.TryCheckedWorldBoolean(operation,subjects,
            operation==ClipType.Union?null:clips,out var actual));
        Assert.Equal(expected.Count,actual.Count);
        for(int i=0;i<expected.Count;i++)
            Assert.Equal(expected[i].Select(p=>(p.x,p.y)),actual[i].Select(p=>(p.x,p.y)));
    }

    private static Path64 Rectangle(long left,long top,long right,long bottom)
        =>new(){new(left,top),new(right,top),new(right,bottom),new(left,bottom)};
}
