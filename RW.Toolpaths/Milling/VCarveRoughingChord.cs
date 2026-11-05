using RW.Toolpaths.Geometry;

namespace RW.Toolpaths.Milling;

/// <summary>An actual prior cutter chord used by stock and clearance queries.</summary>
public readonly record struct VCarveRoughingChord(Point3D A, Point3D B, double TipRadius,
    double ClearedColumnRadius = 0, double StockSurfaceRadius = 0);
