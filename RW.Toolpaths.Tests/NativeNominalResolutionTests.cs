using RW.Toolpaths.Milling.ReviewedSource;

namespace RW.Toolpaths.Tests;

public sealed class NativeNominalResolutionTests
{
    [Fact] public void CapturedMicroscopicTerminalTaskIsResolvedBeforeContactCanStretchItsDepth()
    {
        // Original Region2 medial:7223. The retained cap passes through its root.
        var nominal=new ReviewedVCarveConstructor.Result(2,
            [new(31.405,390.39,-4.606),new(31.418,390.368,-4.562),new(31.396,390.386,-4.605),new(31.416,390.395,-4.6)],
            [new(0,1,"medial:7223"),new(2,0,"cap:118"),new(0,3,"cap:118")],1,0,0,0,0);
        var early=ReviewedVCarveDetailResolution.Apply(nominal,.3);
        Assert.Single(early.Accounting.Branches.Where(b=>b.Omitted));
        Assert.DoesNotContain(early.Source.Edges,e=>e.Owner=="medial:7223");
        Assert.Equal(nominal.Edges.Where(e=>e.Owner.StartsWith("cap:")),early.Source.Edges);
        Assert.Equal(nominal.Nodes[0],early.Source.Nodes[0]);
        var task=early.Accounting.Branches.Single(b=>b.Omitted);
        Assert.InRange(task.PlanarLengthMm,.025,.026);Assert.InRange(task.PoseLengthMm,.050,.052);

        // Actual projected root/tip from the saved corrected-tool source. A late
        // resolution decision sees1.78mm rather than the original0.051mm task.
        var expanded=nominal.Nodes.ToArray();expanded[0]=expanded[0] with{Z= -12.7};
        expanded[1]=expanded[1] with{Z=1.7769338116643993-12.699999809265137};
        var late=ReviewedVCarveDetailResolution.Apply(nominal with{Nodes=expanded},.3);
        Assert.Contains(late.Source.Edges,e=>e.Owner=="medial:7223");
        Assert.True(late.Accounting.Branches.Single(b=>b.Edges.Contains(0)).PoseLengthMm>1.7);
    }
}
