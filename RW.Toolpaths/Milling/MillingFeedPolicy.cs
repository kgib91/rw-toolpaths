namespace RW.Toolpaths.Milling;

/// <summary>Common coordinated cutting feed and modal output resolution. Geometry
/// producers, timeline timing and posts use the same downward component limit.</summary>
public sealed class MillingFeedPolicy
{
    public const double DefaultTravelRate = 5000;
    public const double CommandIncrement = 1;
    private readonly double feed, plunge, travel;
    private double commanded = double.NaN;
    public int ResolutionHeldCommands { get; private set; }

    public MillingFeedPolicy(double feed, double plunge, double travel = DefaultTravelRate)
    {
        if (!double.IsFinite(feed) || feed <= 0 || !double.IsFinite(plunge) || plunge <= 0
            || !double.IsFinite(travel) || travel <= 0)
            throw new InvalidOperationException("Cutting, plunge and travel feeds must be finite and greater than zero.");
        this.feed = feed; this.plunge = plunge; this.travel = travel;
    }

    public double MaximumDescentAngle => Math.Asin(Math.Min(1, plunge / feed));

    /// <param name="vertical">Vertical displacement, positive upwards.</param>
    public double Limit(double x, double vertical, double y)
    {
        double length = Math.Sqrt(x * x + vertical * vertical + y * y);
        if (vertical >= -1e-8 || length <= 1e-8) return feed;
        return x == 0 && y == 0 ? Math.Min(feed, plunge) : Math.Min(feed, plunge * length / -vertical);
    }

    public double Command(double limit, bool clearedTravel = false)
    {
        double desired = clearedTravel ? Math.Max(feed, travel) : Math.Floor(limit);
        if (desired < 1)
            throw new InvalidOperationException("Milling feed is below the machine output resolution of 1 mm/min.");
        if (!clearedTravel && !double.IsNaN(commanded) && desired > commanded && desired <= commanded + CommandIncrement)
            ResolutionHeldCommands++;
        else commanded = desired;
        return commanded;
    }
}
