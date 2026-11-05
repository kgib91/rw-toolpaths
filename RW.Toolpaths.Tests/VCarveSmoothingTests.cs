using Clipper2Lib;
using RW.Toolpaths.Geometry;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths.Tests;

public sealed class VCarveSmoothingTests
{
    [Theory]
    [InlineData(.5773502691896257)]
    [InlineData(1.19175359259421)]
    [InlineData(1.7320508075688772)]
    [InlineData(4)]
    public void SteepLaterFlanksUseTheirActualSelectedRadiusForSupportDominance(double grade)
    {
        var profile=new VCarveCutterProfile([new(CarvingCutterSectionKind.Hemisphere,0,1.5875,1.5875),
            new(CarvingCutterSectionKind.Linear,1.5875,3,20),new(CarvingCutterSectionKind.Linear,3,3,90.4)],111.9875);
        Assert.True(SupportDominanceProof.TryCreate(profile,grade,out _,out _));
        var field=new VCarveSmoothing.FloorField([Path(new(0,0,0),new(1,0,0))],profile,grade,10);
        var literal=new VCarveSmoothing.FloorField([Path(new(0,0,0),new(1,0,0))],profile,grade,10,supportConstantsEnabled:false);
        double transition=1.5875*grade/Math.Sqrt(1+grade*grade);
        foreach(double distance in Enumerable.Range(0,10001).Select(i=>i/500d).Concat(
            new[]{Math.BitDecrement(transition),transition,Math.BitIncrement(transition),1_000_000d,1_000_001d}))
        {
            double best=grade*distance,height=0;
            foreach(var section in profile.Sections)
            {
                if(section.BottomRadius>distance)break;
                if(section.Kind==CarvingCutterSectionKind.Hemisphere)
                {
                    double r=Math.Min(distance,section.TopRadius*grade/Math.Sqrt(1+grade*grade));
                    double bottom=r*r/(section.TopRadius+Math.Sqrt(Math.Max(0,section.TopRadius*section.TopRadius-r*r)));
                    best=Math.Min(best,height+bottom+grade*(distance-r));
                }
                else
                {
                    double slope=section.TopRadius>section.BottomRadius
                        ?section.Height/(section.TopRadius-section.BottomRadius):double.PositiveInfinity;
                    double r=slope<grade?Math.Min(distance,section.TopRadius):section.BottomRadius;
                    double bottom=r<=section.BottomRadius?0:(r-section.BottomRadius)*slope;
                    best=Math.Min(best,height+bottom+grade*(distance-r));
                }
                height+=section.Height;
            }
            Assert.Equal(Math.Max(0,best),field.SupportDistance(distance));
            Assert.Equal(literal.SupportDistance(distance),field.SupportDistance(distance));
        }
        var shallow=new VCarveCutterProfile([new(CarvingCutterSectionKind.Hemisphere,0,1,1),
            new(CarvingCutterSectionKind.Linear,1,4,.2)],1.2);
        Assert.False(SupportDominanceProof.TryCreate(shallow,grade,out _,out _));
    }

    [Theory]
    [InlineData(1,.1)] [InlineData(7,.1)] [InlineData(7,4)] [InlineData(19,4)]
    public void FloorSourcePoseNominationsMatchIndependentContinuousMinimum(double x,double y)
    {
        var profile=new VCarveCutterProfile([new(CarvingCutterSectionKind.Hemisphere,0,1,1),
            new(CarvingCutterSectionKind.Linear,1,1,20)],21);
        var field=new VCarveSmoothing.FloorField([Path(new(0,0,-1),new(20,0,-7))],profile,.5,20);
        double expected=double.PositiveInfinity;
        for(int i=0;i<=100000;i++)
        {
            double u=20d*i/100000, distance=Math.Sqrt((u-x)*(u-x)+y*y);
            double radial=Math.Min(distance,1/Math.Sqrt(5));
            double support=1-Math.Sqrt(1-radial*radial)+.5*(distance-radial);
            expected=Math.Min(expected,1+.3*u+support);
        }
        Assert.InRange(field.Depth(x,y),expected-.00025,expected+1e-8);
        Assert.Contains("floorSourcePoseTrials=",field.QueryScratchDiagnostics);
    }

    private static TaggedToolpath Path(params Point3D[] points)
        => new(points, Array.Empty<ToolpathSpan>(), 0, "final-carve", null, detectCurves: false);

    [Theory]
    [InlineData(.1)]
    [InlineData(2)]
    [InlineData(10)]
    public void FloorDemandUsesThePhysicalNoseContactRatherThanTipHeight(double x)
    {
        var profile=new VCarveCutterProfile(new[]{new CarvingCutterSection(CarvingCutterSectionKind.Hemisphere,0,1,1),
            new CarvingCutterSection(CarvingCutterSectionKind.Linear,1,1.2,10)},4);
        var field=new VCarveSmoothing.FloorField([Path(new(0,0,-1),new(0,10,-1))],profile,.5,4);
        var pose=new Point3D(x,5,-field.CappedDepth(4,x,5));
        var contact=field.NominateFloorContact(pose);
        if(x==10){Assert.Equal(pose,contact);return;}
        double radial=Math.Min(x,1/Math.Sqrt(5));
        Assert.Equal(x-radial,contact.X,6);Assert.Equal(5,contact.Y);
        // The ramp surface is 1+.5*x, while its permitted tip envelope
        // includes the spherical support correction. These are distinct.
        Assert.InRange(-contact.Z,1+.5*contact.X-.003,1+.5*contact.X);
        Assert.Equal(-pose.Z-profile.PhysicalProfile.Bottom(radial),-contact.Z,8);
    }

    [Fact]
    public void FlatBottomFloorContactUsesTheSameSharedProfileSupportOwner()
    {
        var profile=new VCarveCutterProfile([new(CarvingCutterSectionKind.Linear,1,2,4)],4);
        var field=new VCarveSmoothing.FloorField([Path(new(0,0,-1),new(0,10,-1))],profile,.5,4);
        var pose=new Point3D(2,5,-field.CappedDepth(4,2,5));
        var contact=field.NominateFloorContact(pose);
        Assert.Equal(1,contact.X,8);Assert.Equal(pose.Z,contact.Z,8);
        Assert.InRange(-contact.Z,1.497,1.5);
    }

    [Fact]
    public void AnAlreadyCutTipDoesNotEraseAnUncutRampContact()
    {
        var profile=new VCarveCutterProfile([new(CarvingCutterSectionKind.Hemisphere,0,1,1),
            new(CarvingCutterSectionKind.Linear,1,1.2,10)],4);
        var field=new VCarveSmoothing.FloorField([Path(new(0,0,-1),new(0,10,-1))],profile,.5,4);
        var pose=new Point3D(2,5,-field.CappedDepth(4,2,5));
        double oldDepth=-pose.Z+profile.PhysicalProfile.Bottom(.5)-.05;
        var stock=new CutterSweepStock(profile,p=>p,0,default);
        stock.AddProtected([Path(new(2.5,0,-oldDepth),new(2.5,5,-oldDepth),new(2.5,10,-oldDepth))]);stock.Seal();
        Assert.True(stock.CoversFloorTarget(pose,.07));
        Assert.False(stock.CoversFloorTarget(field.NominateFloorContact(pose),.1));
    }

    [Theory]
    [InlineData(0)]
    [InlineData(12.7)]
    [InlineData(-12.7)]
    [InlineData(3000)]
    public void FractionalRoundedTipCompensationUsesItsActualF3Engagement(double surface)
    {
        const double requested = .003375271;
        double depth = VCarveSmoothing.QuantizeCorrectionDepth(requested, surface);
        double emitted = -double.Parse(((float)(surface - depth) - (float)surface)
            .ToString("F3", System.Globalization.CultureInfo.InvariantCulture), System.Globalization.CultureInfo.InvariantCulture);
        double nominalRadius = Math.Sqrt(requested * (2 * 1.5875 - requested));
        double insetRadius = Math.Sqrt(depth * (2 * 1.5875 - depth));
        double cuttingRadius = Math.Sqrt(emitted * (2 * 1.5875 - emitted));
        Assert.Equal(.003, depth);
        Assert.Equal(insetRadius, cuttingRadius);
        Assert.True(nominalRadius - cuttingRadius > .005);
        Assert.Equal(.001, VCarveSmoothing.QuantizeCorrectionDepth(depth / 2, surface));
    }

    [Fact]
    public void CorrectionNeighborhoodRetainsEveryRoundedReachAndTheInteriorHole()
    {
        const double reach = .06433826408401311;
        IReadOnlyList<IReadOnlyList<PointD>> error = new[]
        {
            new PointD[] { new(301.41172, 449.97673), new(302.41172, 449.97673),
                new(302.41172, 450.97673), new(301.41172, 450.97673) },
            new PointD[] { new(301.7, 450.2), new(301.7, 450.7), new(302.2, 450.7), new(302.2, 450.2) },
        };
        var neighborhood = VCarveSmoothing.CorrectionNeighborhood(error, reach);
        Assert.False(PolygonOps.Contains(neighborhood, 301.95, 450.45));
        foreach (PointD vertex in error.SelectMany(ring => ring))
        for (int sample = 0; sample < 720; sample++)
        {
            double angle = sample * Math.PI / 360;
            Assert.True(PolygonOps.Contains(neighborhood, vertex.x + reach * Math.Cos(angle),
                vertex.y + reach * Math.Sin(angle)), $"Candidate reach discarded at vertex={vertex}, sample={sample}.");
        }
    }

    [Fact]
    public void ResidualMeasurementRetainsNarrowLobesBeyondTheWidthCriterion()
    {
        IReadOnlyList<IReadOnlyList<PointD>> missing = new[]
        {
            new PointD[] { new(0, 0), new(2, 0), new(2, .0083), new(.0083, .0083),
                new(.0083, 2), new(0, 2) },
        };
        var centers = VCarveSmoothing.ResidualCentersUpper(missing, .004);
        Assert.True(PolygonOps.Contains(centers, 1, .00415));
        Assert.True(PolygonOps.Contains(centers, .00415, 1));
        Assert.False(PolygonOps.Contains(centers, 1, 1));
    }

    [Fact]
    public void RawCorrectionCandidatesRetainAFeasibleSubMillimetreCenterTrace()
    {
        const double radius = .0563382640840131;
        IReadOnlyList<IReadOnlyList<PointD>> boundary = new[]
        {
            new PointD[] { new(301, 449), new(302, 449), new(302, 449 + 2 * radius + .0008),
                new(301, 449 + 2 * radius + .0008) },
        };
        Assert.Empty(PolygonOps.CutterInset(boundary, radius));
        var candidates = VCarveSmoothing.CorrectionCenters(boundary, radius);
        Assert.True(PolygonOps.Contains(candidates, 301.5, 449 + radius + .0004));
        Assert.Empty(PolygonOps.Difference(candidates, boundary));
    }

    [Fact]
    public void TerminalRampCropRetainsTheWholeDiscOfARelevantDistantCenter()
    {
        IReadOnlyList<IReadOnlyList<PointD>> boundary = new[]
        { new PointD[] { new(0, 0), new(20, 0), new(20, 20), new(0, 20) } };
        var error = new PointD[] { new(10, 10), new(10.01, 10), new(10.01, 10.01), new(10, 10.01) };
        var cropper = new VCarveSmoothing.CorrectionBoundary(boundary);
        var ordinary = cropper.Crop(error, 1);
        var terminal = cropper.Crop(error, 1, additionalCenterReach: 1);
        Assert.False(PolygonOps.Contains(ordinary, 12.8, 10));
        for (int sample = 0; sample < 360; sample++)
        {
            double angle = sample * Math.PI / 180;
            Assert.True(PolygonOps.Contains(terminal, 11.8 + Math.Cos(angle), 10 + Math.Sin(angle)));
        }
    }

    [Fact]
    public void ResidualTargetsCoverTheWholeConcaveFrontierAndItsHole()
    {
        IReadOnlyList<IReadOnlyList<PointD>> rings = new[]
        {
            new PointD[] { new(0, 0), new(.03, 0), new(.03, .01), new(.02, .01), new(.02, .02),
                new(.03, .02), new(.03, .03), new(0, .03) },
            new PointD[] { new(.006, .006), new(.006, .02), new(.012, .02), new(.012, .006) },
        };
        var targets = VCarveSmoothing.ResidualFrontierTargets(rings, .004).ToArray();
        Assert.All(rings.SelectMany(ring => ring), vertex => Assert.Contains(vertex, targets));
        foreach (var ring in rings)
        for (int i = 0; i < ring.Count; i++)
        for (int step = 0; step <= 100; step++)
        {
            PointD a = ring[i], b = ring[(i + 1) % ring.Count];
            double x = a.x + (b.x - a.x) * step / 100, y = a.y + (b.y - a.y) * step / 100;
            double nearest = targets.Min(point => Math.Sqrt((point.x - x) * (point.x - x) + (point.y - y) * (point.y - y)));
            Assert.True(nearest <= .002 + 1e-12, $"Unrepresented residual frontier near ({x:R},{y:R}); nearest target {nearest:R} mm.");
        }
    }

    [Fact]
    public void SavedNarrowChannelFindsLegalF3CentersBeyondTheFirstTwoGridSteps()
    {
        var profile = new VCarveCutterProfile(new[]
        {
            new CarvingCutterSection(CarvingCutterSectionKind.Hemisphere, 0, 1.5875, 1.5875),
            new CarvingCutterSection(CarvingCutterSectionKind.Linear, 1.5875, 3, 20),
        }, 21.5875);
        var tool = new ToolGeometry(3) { VCarveProfile = profile, BottomRadius = 1.5875,
            TopRadius = 3, ConeLength = 20, TipAngleRadians = 2 * Math.Atan((3 - 1.5875) / 20) };
        double surface = (double)12.7f;
        var witnesses = new List<string>();
        var previousTrace = MillingGeometryTrace.Observer.Value;
        MillingGeometryTrace.Observer.Value = (stage, batch, paths) => {
            if (stage is "smoothing-input" or "protect-grade" or "stock-selected" or "source-circuits" or "routed-depth-passes")
                foreach (var group in paths.GroupBy(p => p.Category))
                    witnesses.Add($"stage={stage} batch={batch} category={group.Key} discDistance={DiscDistance(stage == "routed-depth-passes" ? group.ToArray() : VCarveSmoothing.CommandPaths(group.ToArray(), surface), stage == "routed-depth-passes" ? surface : 0):R}");
        };
        ToolpathPlan result;
        try { result = VCarveToolpaths.Generate(new[] { new MillingRegion(VCarveSavedNarrowChannelFixture.Boundary) },
            tool, new DepthSchedule(6.35, 2) { SurfaceZ = surface, StartDepth = 6.35 },
            new VCarveOptions { SmoothVCarving = true, IncludeInteriorFill = false, EdgeFillWidth = 3.175,
                StepOver = .4, DetectCurves = false, CalculateSweptArea = false }, BoostVoronoiProvider.CreateDefault()); }
        finally { MillingGeometryTrace.Observer.Value = previousTrace; }
        double distance = DiscDistance(result.Toolpaths, surface);
        Assert.True(distance <= .004, $"Saved end-channel witness remains {distance:R} mm from the continuous commanded cutter sweep.\n"
            + string.Join('\n', witnesses));

        double DiscDistance(IReadOnlyList<TaggedToolpath> paths, double surfaceZ)
        {
            const double witnessX = 304.5856925421331, witnessY = 450.02134666330767;
            double distance = double.PositiveInfinity;
            static double F3(float value) => double.Parse(value.ToString("F3", System.Globalization.CultureInfo.InvariantCulture),
                System.Globalization.CultureInfo.InvariantCulture);
            foreach (var path in paths)
            for (int i = 1; i < path.Points.Count; i++)
            {
                Point3D Command(Point3D p) => new(F3((float)p.X), F3((float)p.Y), F3((float)p.Z - (float)surfaceZ));
                var a = Command(path.Points[i - 1]); var b = Command(path.Points[i]);
                double DistanceAt(double t)
                {
                    double depth = -a.Z - t * (b.Z - a.Z);
                    if (depth <= 0) return double.PositiveInfinity;
                    double dx = a.X + t * (b.X - a.X) - witnessX, dy = a.Y + t * (b.Y - a.Y) - witnessY;
                    return Math.Sqrt(dx * dx + dy * dy) - profile.PhysicalRadiusAtDepth(depth);
                }
                // Real poses along the complete posted cut remain witnesses even
                // when the contour family no longer puts an endpoint beside it.
                double lo = 0, hi = 1;
                for (int step = 0; step < 40; step++)
                {
                    double left = hi - (hi - lo) * .6180339887498949, right = lo + (hi - lo) * .6180339887498949;
                    if (DistanceAt(left) <= DistanceAt(right)) hi = right; else lo = left;
                }
                distance = Math.Min(distance, Math.Min(Math.Min(DistanceAt(0), DistanceAt(1)), DistanceAt((lo + hi) / 2)));
            }
            return distance;
        }
    }

    [Fact]
    public void ReferenceOpeningIncludesExactRoundedCornerReachAndPreservesHoles()
    {
        IReadOnlyList<IReadOnlyList<PointD>> boundary = new[]
        {
            new PointD[] { new(0, 0), new(20, 0), new(20, 10), new(0, 10) },
            new PointD[] { new(8, 3), new(8, 7), new(12, 7), new(12, 3) },
        };
        var reference = VCarveSmoothing.ReachableReferenceUpper(boundary, 1);
        for (int sample = 1; sample < 100; sample++)
        {
            double angle = sample * Math.PI / 200;
            double x = 1 - Math.Cos(angle), y = 1 - Math.Sin(angle);
            Assert.True(PolygonOps.Contains(reference, x, y),
                $"Exact corner reach was discarded at ({x:R},{y:R}).");
        }
        Assert.False(PolygonOps.Contains(reference, .1, .1));
        Assert.False(PolygonOps.Contains(reference, 10, 5));
        Assert.True(PolygonOps.Contains(reference, 7.999, 5));
        Assert.True(PolygonOps.Contains(reference, 5, 5));
        Assert.Empty(PolygonOps.Difference(reference, boundary));
    }

    [Fact]
    public void ReferenceOpeningDropsUnreachableNarrowComponentWithoutDroppingItsNeighbor()
    {
        IReadOnlyList<IReadOnlyList<PointD>> boundary = new[]
        {
            new PointD[] { new(0, 0), new(10, 0), new(10, 2), new(0, 2) },
            new PointD[] { new(12, 0), new(22, 0), new(22, .15), new(12, .15) },
        };
        var reference = VCarveSmoothing.ReachableReferenceUpper(boundary, .1);
        Assert.True(PolygonOps.Contains(reference, 5, 1));
        Assert.False(PolygonOps.Contains(reference, 15, .075));
        double zeroRadiusArea = Math.Abs(Clipper.Area(new PathsD(
            VCarveSmoothing.ReachableReferenceUpper(boundary, 0).Select(ring => new PathD(ring)))));
        Assert.InRange(Math.Abs(zeroRadiusArea - 21.5), 0, 10 / PathUtils.Scale);
    }

    [Fact]
    public void LocalCorrectionBoundaryPreservesEveryRelevantCenterAndTheProtectedHole()
    {
        IReadOnlyList<IReadOnlyList<PointD>> boundary = new[]
        {
            new PointD[] { new(0, 0), new(20, 0), new(20, 12), new(0, 12) },
            new PointD[] { new(8, 3), new(8, 9), new(12, 9), new(12, 3) },
            new PointD[] { new(30, 0), new(40, 0), new(40, 12), new(30, 12) },
        };
        var error = new PointD[] { new(6, 5), new(6.5, 5), new(6.5, 5.5), new(6, 5.5) };
        var cropper = new VCarveSmoothing.CorrectionBoundary(boundary);
        var local = cropper.Crop(error, 1);
        Assert.True(local.SelectMany(ring => ring).Max(point => point.x) < 10);
        Assert.Empty(PolygonOps.Difference(local, boundary));
        Assert.False(PolygonOps.Contains(local, 8.1, 5));
        var fullCenters = PolygonOps.CutterInset(boundary, 1);
        var localCenters = PolygonOps.CutterInset(local, 1);
        var neighborhood = PolygonOps.Offset(new[] { error }, 1 + VCarveSmoothing.CoverageTolerance,
            arcTolerance: .00075, simplifyTolerance: 0);
        var expected = PolygonOps.Intersect(fullCenters, neighborhood);
        var actual = PolygonOps.Intersect(localCenters, neighborhood);
        double difference = Math.Abs(Clipper.Area(new PathsD(PolygonOps.Difference(expected, actual).Select(ring => new PathD(ring)))))
            + Math.Abs(Clipper.Area(new PathsD(PolygonOps.Difference(actual, expected).Select(ring => new PathD(ring)))));
        Assert.InRange(difference, 0, .0001);
    }

    [Fact]
    public void TiledCorrectionErrorRetainsEverySeamAndProtectedHole()
    {
        var error = new List<List<PointD>>
        {
            new() { new(0, 0), new(128, 0), new(128, 64), new(0, 64) },
            new() { new(40, 20), new(40, 44), new(88, 44), new(88, 20) },
        };
        var tiles = VCarveSmoothing.PartitionCorrectionComponents(error);
        Assert.True(tiles.Count > 4);
        var union = PolygonOps.Union(tiles.SelectMany(component => new[] { component.Outer }.Concat(component.Holes)));
        Assert.Empty(PolygonOps.Difference(error, union));
        Assert.Empty(PolygonOps.Difference(union, error));
        Assert.False(PolygonOps.Contains(union, 64, 32));
        foreach (var tile in tiles)
        {
            Assert.InRange(tile.Outer.Max(point => point.x) - tile.Outer.Min(point => point.x), 0, 32);
            Assert.InRange(tile.Outer.Max(point => point.y) - tile.Outer.Min(point => point.y), 0, 32);
        }
        // Exact seam points belong to one or both closed tiles; a surrounding
        // band must survive their union without a dropped cutting channel.
        IReadOnlyList<IReadOnlyList<PointD>> seamBand = new[]
        { new PointD[] { new(31.99, 1), new(32.01, 1), new(32.01, 19), new(31.99, 19) } };
        Assert.Empty(PolygonOps.Difference(seamBand, union));
    }

    [Fact]
    public void JointReachabilityExcludesAClippedGougeAndRetainsALegitimateNarrowChannel()
    {
        IReadOnlyList<IReadOnlyList<PointD>> artwork = new[]
        { new PointD[] { new(-5, -5), new(5, -5), new(5, 5), new(-5, 5) } };
        var oldCutterDisc = Enumerable.Range(0, 256).Select(i =>
            new PointD(5.6 + Math.Cos(2 * Math.PI * i / 256), Math.Sin(2 * Math.PI * i / 256))).ToList();
        IReadOnlyList<IReadOnlyList<PointD>> oldChannel = new[]
        {
            (IReadOnlyList<PointD>)oldCutterDisc,
            new PointD[] { new(-1.025, -3), new(1.025, -3), new(1.025, 3), new(-1.025, 3) },
        };
        var allowed = PolygonOps.Intersect(oldChannel, artwork);
        var wrongOrder = PolygonOps.Intersect(oldChannel, VCarveSmoothing.ReachableReferenceUpper(artwork, 1));
        var joint = VCarveSmoothing.ReachableReferenceUpper(allowed, 1);
        Assert.True(PolygonOps.Contains(wrongOrder, 4.9, 0));
        // The clipped disc is only .4 mm wide, so no complete diameter-2 cutter
        // fits the intersection even though either independent region allows it.
        Assert.False(PolygonOps.Contains(joint, 4.9, 0));
        Assert.True(PolygonOps.Contains(joint, 0, 0));
        Assert.True(PolygonOps.Contains(joint, 1.02, 0));
        Assert.Empty(PolygonOps.Difference(joint, allowed));
    }

    [Fact]
    public void SymmetricGraphLimitUsesBothDirectionsAndPreservesBranchRevisits()
    {
        var input = new[]
        {
            Path(new(0, 0, 0), new(2, 0, -5), new(4, 0, -5), new(2, 0, -5), new(2, 2, -.2)),
            Path(new(4, 0, -5), new(6, 0, 0)),
        };
        var result = VCarveSmoothing.LimitGraph(input, .25);
        Assert.Equal(.25, VCarveSmoothing.MaximumGrade(result), 10);
        Assert.Equal(-.5, result[0].Points[1].Z, 10);
        Assert.Equal(result[0].Points[1], result[0].Points[3]);
        Assert.Equal(result[0].Points[2], result[1].Points[0]);
        Assert.Equal(input.SelectMany(path => path.Points).Select(point => (point.X, point.Y)),
            result.SelectMany(path => path.Points).Select(point => (point.X, point.Y)));
        var reversed = VCarveSmoothing.LimitGraph(input.Select(path =>
            Path(path.Points.Reverse().ToArray())).ToArray(), .25);
        Assert.Equal(result.SelectMany(path => path.Points), reversed.SelectMany(path => path.Points.Reverse()));
    }

    [Fact]
    public void AlreadyGradualMedialDetailKeepsItsExactGeometry()
    {
        var path = Path(new(1, 1, 0), new(3, 1, -.02), new(5, 1, -.035));
        var source = new[] { path };
        var result = VCarveSmoothing.Apply(source,
            new[] { (IReadOnlyList<PointD>)new PointD[] { new(0, 0), new(6, 0), new(6, 2), new(0, 2) } },
            new ToolGeometry(3) { BottomRadius = 0, TopRadius = 3, ConeLength = 10 }, new DepthSchedule(3, 1),
            new VCarveOptions { SmoothVCarving = true }, CancellationToken.None, null);
        Assert.Contains(result, p => p.Category == "clearing");
        var actual = Assert.Single(result.Where(p => p.Category == "final-carve")).Points;
        Assert.Equal(path.Points[0], actual[0]);
        Assert.Equal(path.Points[^1], actual[^1]);
        foreach (var point in actual)
        {
            var a = point.X <= 3 ? path.Points[0] : path.Points[1];
            var b = point.X <= 3 ? path.Points[1] : path.Points[2];
            double expected = a.Z + (b.Z - a.Z) * (point.X - a.X) / (b.X - a.X);
            Assert.Equal(1, point.Y);
            Assert.Equal(expected, point.Z, 12);
        }
    }

    [Theory]
    [InlineData(1)]
    [InlineData(15)]
    [InlineData(75)]
    public void ContinuousPhysicalFieldProtectsFullBallAndTaperAtEveryAngle(double degrees)
    {
        var profile = new VCarveCutterProfile(new[]
        {
            new CarvingCutterSection(CarvingCutterSectionKind.Hemisphere, 0, 1, 1),
            new CarvingCutterSection(CarvingCutterSectionKind.Linear, 1, 3, 8),
        }, 9);
        var physical = new CarvingCutterProfile(profile.Sections, 9);
        double grade = Math.Tan(degrees * Math.PI / 180);
        var field = new VCarveSmoothing.FloorField(new[] { Path(new(0, 0, -.4), new(10, 0, -.4)) },
            profile, grade, 9);
        Assert.Equal(0, field.SupportDistance(0));
        for (int radialStep = 0; radialStep <= 40; radialStep++)
        {
            double y = radialStep * .1;
            double depth = field.Depth(5, y);
            for (int cutterStep = 0; cutterStep <= 120; cutterStep++)
            {
                double r = cutterStep / 40d;
                for (int direction = -1; direction <= 1; direction += 2)
                {
                    double floor = .4 + grade * Math.Abs(y + direction * r);
                    Assert.True(depth - physical.Bottom(r) <= floor + 1e-9,
                        $"degrees={degrees}, y={y}, radius={r}, tip={depth}, floor={floor}");
                }
            }
        }
    }

    [Fact]
    public void ContinuousGraphFieldFindsInteriorAnchorWithoutDependingOnSubdivision()
    {
        var profile = new VCarveCutterProfile(new[]
        { new CarvingCutterSection(CarvingCutterSectionKind.Linear, 0, 5, 10) }, 10);
        var a = new VCarveSmoothing.FloorField(new[] { Path(new(0, 0, -.2), new(10, 0, -2.2)) }, profile, .25, 10);
        var b = new VCarveSmoothing.FloorField(new[] { Path(new(0, 0, -.2), new(4, 0, -1), new(10, 0, -2.2)) }, profile, .25, 10);
        for (int i = 0; i <= 20; i++)
            Assert.InRange(Math.Abs(a.Depth(i * .5, .7) - b.Depth(i * .5, .7)), 0, .000201);
    }

    [Fact]
    public void LongMaximumGradeSegmentKeepsDepthAndDistanceCorrelated()
    {
        var profile = new VCarveCutterProfile(new[]
        { new CarvingCutterSection(CarvingCutterSectionKind.Hemisphere, 0, 1, 1),
          new CarvingCutterSection(CarvingCutterSectionKind.Linear, 1, 30, 99) }, 100);
        var field = new VCarveSmoothing.FloorField(new[] { Path(new(0, 0, -.1), new(100, 0, -25.1)) }, profile, .25, 100);
        double depth = field.Depth(90, .2);
        var support = new CarvingCutterProfile(profile.Sections, profile.MaximumDepth).Support(.25);
        Assert.InRange(depth, 22.6 - support - .000201, 22.61);
    }

    [Theory]
    [InlineData(15)]
    [InlineData(75)]
    public void RoundedSupportCurvatureBoundCertifiesInteriorChordMinima(double degrees)
    {
        var profile = new VCarveCutterProfile(new[]
        { new CarvingCutterSection(CarvingCutterSectionKind.Hemisphere, 0, 1, 1),
          new CarvingCutterSection(CarvingCutterSectionKind.Linear, 1, 3, 8) }, 9);
        double grade = Math.Tan(degrees * Math.PI / 180);
        var field = new VCarveSmoothing.FloorField(new[] { Path(new(0, 0, -.4), new(0, 0, -.4)) }, profile, grade, 9);
        Assert.True(double.IsFinite(field.CurvatureBound));
        for (int chord = 0; chord < 20; chord++)
        {
            double ax = -.7 + chord * .09, ay = .2, bx = ax + .31, by = ay + .23;
            double fa = field.Depth(ax, ay), fb = field.Depth(bx, by);
            double lengthSquared = (bx - ax) * (bx - ax) + (by - ay) * (by - ay);
            for (int sample = 0; sample <= 100; sample++)
            {
                double t = sample / 100d, x = ax + (bx - ax) * t, y = ay + (by - ay) * t;
                double lower = fa + (fb - fa) * t - field.CurvatureBound * lengthSquared * t * (1 - t) / 2;
                double exact = Math.Min(9, .4 + field.SupportDistance(Math.Sqrt(x * x + y * y)));
                Assert.True(lower <= exact + 1e-10,
                    $"angle={degrees}, chord={chord}, t={t}, lower={lower:R}, exact={exact:R}");
            }
        }
        var shallowFlank = new VCarveCutterProfile(new[]
        { new CarvingCutterSection(CarvingCutterSectionKind.Hemisphere, 0, 1, 1),
          new CarvingCutterSection(CarvingCutterSectionKind.Linear, 1, 10, .1) }, 1.1);
        var general = new VCarveSmoothing.FloorField(new[] { Path(new(0, 0, 0), new(1, 0, 0)) }, shallowFlank, grade, 1.1);
        Assert.Equal(double.PositiveInfinity, general.CurvatureBound);
    }

    [Fact]
    public void FiniteFlatTipProtectsTheAnchorAcrossItsWholeBottomDisc()
    {
        var profile = new VCarveCutterProfile(new[]
        { new CarvingCutterSection(CarvingCutterSectionKind.Linear, 1, 3, 10) }, 10);
        var field = new VCarveSmoothing.FloorField(new[] { Path(new(0, 0, -.4), new(5, 0, -.4)) }, profile, .25, 10);
        Assert.Equal(0, field.SupportDistance(.5));
        Assert.Equal(.4, field.Depth(2, .5), 10);
        Assert.Equal(.25, field.SupportDistance(2), 10);
    }

    [Fact]
    public void ZeroFillConstrainsChangedMedialAndRetainsTheCompleteSlope()
    {
        var tool = new ToolGeometry(5)
        { BottomRadius = 0, TopRadius = 5, ConeLength = 10, TipAngleRadians = 2 * Math.Atan(.5) };
        IReadOnlyList<IReadOnlyList<PointD>> boundary = new[]
        { new PointD[] { new(0, 0), new(10, 0), new(10, 4), new(0, 4) } };
        var original = new[] { Path(new(1, 2, -.2), new(4, 2, -3), new(8, 2, -3)) };
        var logs = new List<string>();
        var result = VCarveSmoothing.Apply(original, boundary, tool, new DepthSchedule(3, 1),
            new VCarveOptions { SmoothVCarving = true, SmoothRampAngleDegrees = 15,
                IncludeInteriorFill = false, DetectCurves = false }, CancellationToken.None, logs.Add);
        Assert.True(VCarveSmoothing.MaximumGrade(result) <= Math.Tan(Math.PI / 12) + 1e-8);
        Assert.Contains(result, path => path.Category == "clearing");
        Assert.Contains(logs, message => message.Contains("stage=fill-limit event=complete")
            && message.Contains("slopeAdditive=True"));
        Assert.True(result.Single(p=>p.Category=="final-carve").Points.Min(point => point.Z) > -3);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(90)]
    [InlineData(double.NaN)]
    public void SmoothAngleValidationRejectsNonphysicalGrades(double degrees)
    {
        Assert.Throws<ArgumentOutOfRangeException>(() => VCarveToolpaths.Generate(Array.Empty<MillingRegion>(),
            new ToolGeometry(1), new DepthSchedule(1, 1),
            new VCarveOptions { SmoothVCarving = true, SmoothRampAngleDegrees = degrees }, new EmptyProvider()));
    }

    [Fact]
    public void RoundedWideningChannelConvergesWithNearZeroCornerAnchors()
    {
        var profile = new VCarveCutterProfile(new[]
        {
            new CarvingCutterSection(CarvingCutterSectionKind.Hemisphere, 0, 1.5875, 1.5875),
            new CarvingCutterSection(CarvingCutterSectionKind.Linear, 1.5875, 3, 20),
        }, 3);
        var tool = new ToolGeometry(3) { VCarveProfile = profile };
        IReadOnlyList<IReadOnlyList<PointD>> boundary = new[]
        { new PointD[] { new(0, 0), new(16, 0), new(16, 5), new(0, 1.5) } };
        var raw = MedialAxisToolpaths.GenerateVCarveTagged(BoostVoronoiProvider.CreateDefault(),
            boundary, 0, 3, 0, 1.5, includeInteriorFill: false, detectCurves: false, cutterProfile: profile);
        var logs = new List<string>();
        List<TaggedToolpath> result;
        try
        {
            result = VCarveSmoothing.Apply(raw, boundary, tool, new DepthSchedule(3, 1.5),
                new VCarveOptions { SmoothVCarving = true, IncludeInteriorFill = true, DetectCurves = false },
                CancellationToken.None, logs.Add);
        }
        catch (Exception error) { throw new InvalidOperationException(string.Join(Environment.NewLine, logs), error); }
        Assert.Contains(logs, line => line.Contains("widthVerified=True"));
        Assert.True(VCarveSmoothing.MaximumGrade(result) <= Math.Tan(Math.PI / 6) + 1e-8);
    }

    private sealed class EmptyProvider : IMedialAxisProvider
    {
        public IReadOnlyList<MedialSegment> ConstructMedialAxis(IReadOnlyList<PointD> boundary,
            IReadOnlyList<IReadOnlyList<PointD>> holes, double tolerance, double maxRadius,
            double filteringAngle = 3 * Math.PI / 4, bool useBigIntegers = true) => Array.Empty<MedialSegment>();
    }
}
