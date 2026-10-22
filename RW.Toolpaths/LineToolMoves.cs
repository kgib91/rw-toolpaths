using System.Collections;

namespace RW.Toolpaths;

/// <summary>Line metadata derived on demand from the owned completed point trace.</summary>
internal sealed class LineToolMoves : IReadOnlyList<ToolMove>
{
    private readonly IReadOnlyList<Point3D> _points;
    internal LineToolMoves(IReadOnlyList<Point3D> points) => _points = points;
    public int Count => Math.Max(0, _points.Count - 1);
    public ToolMove this[int index]
    {
        get
        {
            if ((uint)index >= (uint)Count) throw new ArgumentOutOfRangeException(nameof(index));
            return ToolMove.Line(_points[index], _points[index + 1]);
        }
    }
    public IEnumerator<ToolMove> GetEnumerator()
    {
        for (int i = 1; i < _points.Count; i++) yield return ToolMove.Line(_points[i - 1], _points[i]);
    }
    IEnumerator IEnumerable.GetEnumerator() => GetEnumerator();
}
