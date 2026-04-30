using System.Runtime.InteropServices;

namespace RW.Toolpaths;

/// <summary>
/// Raw P/Invoke bindings for the native <c>boostvoronoi</c> shared library.
/// All structs are blittable; arrays are pinned by the CLR for the call duration.
/// </summary>
internal static class BoostVoronoiInterop
{
    private const string LibName = "boostvoronoi";

    /// <summary>
    /// Mirrors the <c>boost::polygon SOURCE_CATEGORY_*</c> enum bits returned
    /// in <see cref="BvCell.SourceCategory"/>.
    /// </summary>
    internal static class SourceCategory
    {
        internal const int SinglePoint       = 0x0;
        internal const int SegmentStartPoint = 0x1;  // low endpoint  (y-then-x min)
        internal const int SegmentEndPoint   = 0x2;  // high endpoint (y-then-x max)
        internal const int InitialSegment    = 0x4;
        internal const int ReverseSegment    = 0x8;
    }

    // -- Structs must match the C layout exactly (all int32_t / double) --------

    [StructLayout(LayoutKind.Sequential)]
    internal struct BvEdge
    {
        public int TwinIndex;
        public int PrevIndex;
        public int NextIndex;
        public int Vertex0Index;   // -1 = infinite end
        public int Vertex1Index;   // -1 = infinite end
        public int CellIndex;
        public int TwinCellIndex;
        public int IsPrimary;      // 1 or 0
        public int IsLinear;
        public int IsCurved;
        public int IsInfinite;
    }

    [StructLayout(LayoutKind.Sequential)]
    internal struct BvVertex
    {
        public double X;
        public double Y;
    }

    [StructLayout(LayoutKind.Sequential)]
    internal struct BvCell
    {
        public int SourceIndex;
        public int SourceCategory;
        public int ContainsPoint;  // 1 = point site, 0 = segment site
    }

    // -- Functions -------------------------------------------------------------

    [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
    internal static extern IntPtr bv_construct(
        int[] x0, int[] y0, int[] x1, int[] y1, int count);

    [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
    internal static extern void bv_destroy(IntPtr diagram);

    [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
    internal static extern int bv_edge_count(IntPtr diagram);

    [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
    internal static extern int bv_vertex_count(IntPtr diagram);

    [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
    internal static extern int bv_cell_count(IntPtr diagram);

    [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
    internal static extern void bv_get_edges(
        IntPtr diagram, [In, Out] BvEdge[] outBuf, int count);

    [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
    internal static extern void bv_get_vertices(
        IntPtr diagram, [In, Out] BvVertex[] outBuf, int count);

    [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
    internal static extern void bv_get_cells(
        IntPtr diagram, [In, Out] BvCell[] outBuf, int count);
}

