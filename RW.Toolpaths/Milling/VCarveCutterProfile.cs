namespace RW.Toolpaths.Milling;

/// <summary>
/// A radially nondecreasing physical cutting profile, measured from the actual tip.
/// V-carving uses its surface footprint and inverse to turn boundary clearance into
/// penetration depth. The section representation is shared with heightmap carving.
/// </summary>
public sealed class VCarveCutterProfile
{
    private readonly CarvingCutterProfile _physical;

    public IReadOnlyList<CarvingCutterSection> Sections { get; }
    public double MaximumDepth { get; }
    internal double RepresentedHeight { get; }
    public double TipRadius { get; }
    public bool RoundedTip { get; }
    internal double FirstCylinderHeight { get; } = double.PositiveInfinity;

    /// <summary>Nose diameter for clearing pitch; zero for a pointed conical tip.</summary>
    public double ClearingDiameter { get; }

    internal double NoseLimitedPitch(double engagedDiameter,double stepOver)
        =>stepOver*Math.Min(ClearingDiameter,engagedDiameter);

    /// <summary>Centreline inset from the retained crest to the ramp's full-depth
    /// floor, including support from the complete physical cutter. Depth is
    /// stock-relative and limited by this profile's supported cutting depth.</summary>
    public double RampFillWidth(double depth, double rampAngleDegrees)
    {
        if (!double.IsFinite(depth) || depth < 0)
            throw new ArgumentOutOfRangeException(nameof(depth));
        if (!double.IsFinite(rampAngleDegrees) || rampAngleDegrees <= 0 || rampAngleDegrees >= 90)
            throw new ArgumentOutOfRangeException(nameof(rampAngleDegrees));
        if (depth == 0) return 0;
        double effectiveDepth = Math.Min(depth, MaximumDepth);
        double grade = Math.Tan(rampAngleDegrees * Math.PI / 180);
        return effectiveDepth / grade + Math.Max(TipRadius, _physical.SupportUpTo(grade, effectiveDepth) / grade);
    }

    public VCarveCutterProfile(IEnumerable<CarvingCutterSection> sections, double maximumDepth)
    {
        ArgumentNullException.ThrowIfNull(sections);
        var snapshot = sections.ToArray();
        _physical = new CarvingCutterProfile(snapshot, maximumDepth);
        // Retain this exact selector-Sum result. Native envelope queries use
        // the same immutable complete profile height, without re-enumerating
        // the public read-only wrapper or reassociating its floating sum.
        RepresentedHeight = snapshot.Sum(section => section.Height);
        if (maximumDepth > RepresentedHeight)
            throw new ArgumentException("V-carving depth exceeds the represented cutting profile.", nameof(maximumDepth));

        double height = 0, previousRadius = snapshot[0].BottomRadius;
        foreach (var section in snapshot)
        {
            // The section beginning exactly at the contact plane participates
            // in MaxRadiusBelow too. An upward shoulder there cannot be ignored.
            if (height > maximumDepth) break;
            if (section.BottomRadius != previousRadius || section.TopRadius < section.BottomRadius)
                throw new ArgumentException("V-carving requires continuous, radially nondecreasing cutting sections.", nameof(sections));
            if (section.Kind == CarvingCutterSectionKind.Linear && section.BottomRadius == section.TopRadius
                && double.IsPositiveInfinity(FirstCylinderHeight))
                FirstCylinderHeight = height;
            previousRadius = section.TopRadius;
            height += section.Height;
        }

        Sections = Array.AsReadOnly(snapshot);
        MaximumDepth = maximumDepth;
        TipRadius = snapshot[0].BottomRadius;
        RoundedTip = snapshot[0].Kind == CarvingCutterSectionKind.Hemisphere;
        ClearingDiameter = 2 * (RoundedTip ? snapshot[0].TopRadius : TipRadius);
    }

    /// <summary>Largest cutter radius contacting stock at this penetration depth.</summary>
    public double RadiusAtDepth(double depth)
    {
        if (double.IsNaN(depth)) throw new ArgumentOutOfRangeException(nameof(depth));
        return depth >= MaximumDepth ? _physical.MaximumRadius
            : _physical.MaxRadiusBelow(Math.Max(0, depth));
    }

    // Output rounding can command a tip slightly beyond the nominal operation
    // cap. Its actual physical footprint must not inherit that nominal clamp.
    internal double PhysicalRadiusAtDepth(double depth) => _physical.MaxRadiusBelow(Math.Max(0, depth));

    internal CarvingCutterProfile PhysicalProfile => _physical;

    /// <summary>
    /// Deepest penetration fitting a boundary clearance. A cylindrical plateau can
    /// reach its full axial length once its radius fits, including a straight ball mill.
    /// </summary>
    public double DepthAtRadius(double clearance)
    {
        if (double.IsNaN(clearance)) throw new ArgumentOutOfRangeException(nameof(clearance));
        if (clearance < TipRadius) return 0;
        if (clearance >= _physical.MaximumRadius) return MaximumDepth;

        double height = 0;
        foreach (var section in _physical.Sections)
        {
            if (height >= MaximumDepth) break;
            double activeHeight = Math.Min(section.Height, MaximumDepth - height);
            double topRadius = section.Kind == CarvingCutterSectionKind.Hemisphere
                ? Math.Sqrt(activeHeight * (2 * section.TopRadius - activeHeight))
                : section.BottomRadius + (section.TopRadius - section.BottomRadius) * activeHeight / section.Height;
            // Equality may continue through a cylindrical section. Its first contact
            // height is not the deepest height that fits this same boundary clearance.
            if (clearance >= topRadius)
            {
                height += activeHeight;
                continue;
            }

            double localDepth;
            if (section.Kind == CarvingCutterSectionKind.Hemisphere)
            {
                double radius = section.TopRadius;
                // Rationalized sagitta retains shallow detail near the spherical tip.
                localDepth = clearance * clearance
                    / (radius + Math.Sqrt(Math.Max(0, radius * radius - clearance * clearance)));
            }
            else
            {
                localDepth = (clearance - section.BottomRadius) * section.Height
                    / (section.TopRadius - section.BottomRadius);
            }
            return Math.Clamp(height + localDepth, 0, MaximumDepth);
        }
        return MaximumDepth;
    }
}
