using System.Runtime.InteropServices;
using Microsoft.Win32.SafeHandles;
using Clipper2Lib;

namespace RW.Toolpaths.Milling;

/// <summary>Pinned native polygon kernel for nominal medial preparation only.</summary>
internal static class NativeMedialPolygonKernel
{
    private const string Library = "clipper2medial";
    internal static Paths64 Execute(Paths64 paths, bool offset, FillRule fillRule, double distance,
        CancellationToken token = default)
    {
        token.ThrowIfCancellationRequested();
        if (c2m_abi() != 1 || Marshal.PtrToStringAnsi(c2m_version()) != "1.5.4")
            throw new InvalidOperationException("The nominal polygon kernel ABI/version does not match its construction policy.");
        var lengths = paths.Select(r => r.Count).ToArray();
        var input = new long[checked(lengths.Sum() * 2)]; int index = 0;
        foreach (var ring in paths)
            foreach (var p in ring) { input[index++] = p.X; input[index++] = p.Y; }
        using var result = new Result(c2m_execute(input, lengths, paths.Count, offset ? 1 : 0, (int)fillRule, distance));
        if (result.IsInvalid) throw new InvalidOperationException($"Native nominal polygon construction failed: operation={(offset ? "offset" : "union")} rings={paths.Count} points={lengths.Sum()} fillRule={fillRule} distance={distance:R} kernel=1.5.4. Incomplete geometry cannot be used.");
        token.ThrowIfCancellationRequested();
        int count = c2m_path_count(result); var output = new Paths64(count);
        for (int i = 0; i < count; i++)
        {
            token.ThrowIfCancellationRequested(); int points = c2m_point_count(result, i);
            if (points < 0) throw new InvalidOperationException("Native nominal polygon result is invalid.");
            var xy = new long[checked(points * 2)];
            if (c2m_copy_path(result, i, xy, points) != points) throw new InvalidOperationException("Native nominal polygon copy was incomplete.");
            var ring = new Path64(points);
            for (int j = 0; j < points; j++) ring.Add(new Point64(xy[2 * j], xy[2 * j + 1]));
            output.Add(ring);
        }
        return output;
    }
    private sealed class Result : SafeHandleZeroOrMinusOneIsInvalid
    {
        internal Result(IntPtr value) : base(true) => SetHandle(value);
        protected override bool ReleaseHandle() { c2m_destroy(handle); return true; }
    }
    [DllImport(Library, CallingConvention = CallingConvention.Cdecl)] private static extern int c2m_abi();
    [DllImport(Library, CallingConvention = CallingConvention.Cdecl)] private static extern IntPtr c2m_version();
    [DllImport(Library, CallingConvention = CallingConvention.Cdecl)] private static extern IntPtr c2m_execute(long[] xy, int[] lengths, int count, int operation, int fillRule, double distance);
    [DllImport(Library, CallingConvention = CallingConvention.Cdecl)] private static extern int c2m_path_count(Result result);
    [DllImport(Library, CallingConvention = CallingConvention.Cdecl)] private static extern int c2m_point_count(Result result, int index);
    [DllImport(Library, CallingConvention = CallingConvention.Cdecl)] private static extern int c2m_copy_path(Result result, int index, [Out] long[] xy, int capacity);
    [DllImport(Library, CallingConvention = CallingConvention.Cdecl)] private static extern void c2m_destroy(IntPtr result);
}
