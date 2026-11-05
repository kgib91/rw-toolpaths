namespace RW.Toolpaths.Milling;

/// <summary>Alternative source-knot proposals, not alternate source geometry.
/// Every selected curve still earns the ordinary complete certificates.</summary>
public sealed record VCarveNativeNominationFamily(string Name,
    IReadOnlyList<IReadOnlyList<int>> SelectedSourceOrdinals);
