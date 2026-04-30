using System.Numerics;
using Avalonia;
using Avalonia.Controls;
using Avalonia.Input;
using Avalonia.Media;
using RW.Toolpaths;

namespace RW.Toolpaths.Avalonia;

public sealed class GeometryPreviewControl : Control
{
    private static readonly SolidColorBrush PreviewBackgroundBrush = new(Color.FromRgb(250, 250, 250));

    private readonly List<List<Point>> _contours = new();
    private readonly List<List<Point3D>> _clearing = new();
    private readonly List<List<Point3D>> _vcarve = new();
    private readonly List<(Point A, Point B)> _medial = new();
    private readonly Dictionary<int, Pen> _vcarvePenCache = new();
    private readonly List<double> _clearingAverageDepths = new();

    private bool _isOrbiting;
    private bool _isPanning2D;
    private bool _isPanning3D;
    private Point _lastMouse;
    private float _orbitYaw = -0.95f;
    private float _orbitPitch = 0.78f;
    private float _orbitZoom = 1.0f;
    private Vector3 _orbitPanOffset = Vector3.Zero;
    private double _panX;
    private double _panY;
    private double _zoom2D = 1.0;
    private bool _hasBounds;
    private double _minX;
    private double _minY;
    private double _maxX;
    private double _maxY;
    private double _minVCarveZ = -0.01;
    private int _vcarveSegmentCount;
    private int _renderCounter;

    public bool PerspectiveEnabled { get; set; }
    public bool SegmentDebugColors { get; set; }
    public string DebugInfo { get; set; } = string.Empty;

    public GeometryPreviewControl()
    {
        ClipToBounds = true;

        PointerPressed += OnPointerPressed;
        PointerMoved += OnPointerMoved;
        PointerReleased += OnPointerReleased;
        PointerWheelChanged += OnPointerWheelChanged;
    }

    public void ResetView()
    {
        _orbitYaw = -0.95f;
        _orbitPitch = 0.78f;
        _orbitZoom = 1.0f;
        _orbitPanOffset = Vector3.Zero;
        _panX = 0;
        _panY = 0;
        _zoom2D = 1.0;
        InvalidateVisual();
    }

    public void SetGeometry(
        List<List<Point>> contours,
        List<List<Point3D>> clearing,
        List<List<Point3D>> vcarve,
        List<(Point A, Point B)> medial,
        string debugInfo)
    {
        _contours.Clear();
        _clearing.Clear();
        _vcarve.Clear();
        _medial.Clear();

        _contours.AddRange(contours);
        _clearing.AddRange(clearing);
        _vcarve.AddRange(vcarve);
        _medial.AddRange(medial);
        RecomputeCaches();

        DebugInfo = debugInfo;
        InvalidateVisual();
    }

    public override void Render(DrawingContext context)
    {
        long t0 = PerfLog.Start();
        base.Render(context);

        var viewport = new Rect(Bounds.Size);
        if (viewport.Width <= 0 || viewport.Height <= 0)
            return;

        using var clip = context.PushClip(viewport);
        context.FillRectangle(PreviewBackgroundBrush, viewport);

        if (!_hasBounds)
        {
            DrawText(context, "No geometry to preview", new Point(20, 20), Colors.DimGray);
            return;
        }

        if (PerspectiveEnabled)
            DrawPerspectivePreview(context, viewport);
        else
            DrawTopDownPreview(context, viewport);

        var mode = PerspectiveEnabled
            ? "Perspective Orbit 3D (left-drag=orbit, right-drag=pan, wheel=zoom)"
            : "Top-Down Ortho 2D (right-drag=pan, wheel=zoom)";
        DrawText(
            context,
            $"View: {mode} | Contours: {_contours.Count} | Clearing: {_clearing.Count} | VCarve Paths: {_vcarve.Count} | VCarve Segs: {_vcarveSegmentCount} | {DebugInfo}",
            new Point(12, 10),
            Color.FromArgb(220, 30, 30, 30));

        if (PerfLog.IsEnabled)
        {
            _renderCounter++;
            if ((_renderCounter % 30) == 0)
                PerfLog.Stop("GeometryPreviewControl.Render", t0, $"perspective={PerspectiveEnabled} contours={_contours.Count} clearing={_clearing.Count} vcarve={_vcarve.Count}");
        }
    }

    private void RecomputeCaches()
    {
        long t0 = PerfLog.Start();
        _hasBounds = false;
        _minVCarveZ = -0.01;
        _vcarveSegmentCount = 0;
        _clearingAverageDepths.Clear();

        void Acc(double x, double y)
        {
            if (!_hasBounds)
            {
                _minX = _maxX = x;
                _minY = _maxY = y;
                _hasBounds = true;
                return;
            }

            if (x < _minX) _minX = x;
            if (x > _maxX) _maxX = x;
            if (y < _minY) _minY = y;
            if (y > _maxY) _maxY = y;
        }

        foreach (var contour in _contours)
            foreach (var p in contour)
                Acc(p.X, p.Y);

        foreach (var path in _clearing)
        {
            double zSum = 0;
            int zCount = 0;
            foreach (var p in path)
            {
                Acc(p.X, p.Y);
                zSum += p.Z;
                zCount++;
            }

            _clearingAverageDepths.Add(zCount == 0 ? 0.0 : zSum / zCount);
        }

        foreach (var path in _vcarve)
        {
            _vcarveSegmentCount += Math.Max(0, path.Count - 1);
            foreach (var p in path)
            {
                Acc(p.X, p.Y);
                if (p.Z < _minVCarveZ)
                    _minVCarveZ = p.Z;
            }
        }

        foreach (var seg in _medial)
        {
            Acc(seg.A.X, seg.A.Y);
            Acc(seg.B.X, seg.B.Y);
        }

        _vcarvePenCache.Clear();
        PerfLog.Stop(
            "GeometryPreviewControl.RecomputeCaches",
            t0,
            $"contours={_contours.Count} clearing={_clearing.Count} vcarve={_vcarve.Count} segs={_vcarveSegmentCount}");
    }

    private void DrawTopDownPreview(DrawingContext context, Rect viewport)
    {
        var (scale, offsetX, offsetY) = BuildTransform(viewport, 24);
        var center = new Point(viewport.X + viewport.Width * 0.5, viewport.Y + viewport.Height * 0.5);

        Point Tx(Point p)
        {
            var baseX = offsetX + p.X * scale;
            var baseY = offsetY + p.Y * scale;
            return new Point(
                center.X + (baseX - center.X) * _zoom2D + _panX,
                center.Y + (baseY - center.Y) * _zoom2D + _panY);
        }

        var contourPen = new Pen(new SolidColorBrush(Color.FromArgb(235, 25, 25, 25)), 1.8);
        foreach (var contour in _contours)
        {
            if (contour.Count < 2)
                continue;

            var prev = Tx(contour[0]);
            for (int i = 1; i < contour.Count; i++)
            {
                var next = Tx(contour[i]);
                context.DrawLine(contourPen, prev, next);
                prev = next;
            }
        }

        for (int pathIndex = 0; pathIndex < _clearing.Count; pathIndex++)
        {
            var path = _clearing[pathIndex];
            double avgDepth = pathIndex < _clearingAverageDepths.Count ? _clearingAverageDepths[pathIndex] : 0.0;
            var color = ClearingColorForDepth(avgDepth);
            var pen = new Pen(new SolidColorBrush(color), 1.3);
            if (path.Count < 2)
                continue;

            var first = Tx(new Point(path[0].X, path[0].Y));
            var prev = first;
            for (int i = 1; i < path.Count; i++)
            {
                var next = Tx(new Point(path[i].X, path[i].Y));
                context.DrawLine(pen, prev, next);
                prev = next;
            }

            if (path.Count > 2)
                context.DrawLine(pen, prev, first);
        }

        var minVZ = MinVCarveZ();
        int moveIdx = 0;
        foreach (var path in _vcarve)
        {
            if (path.Count < 2) continue;
            for (int i = 1; i < path.Count; i++)
            {
                var a2d = Tx(new Point(path[i - 1].X, path[i - 1].Y));
                var b2d = Tx(new Point(path[i].X, path[i].Y));
                var segZ = (path[i - 1].Z + path[i].Z) * 0.5;
                var pen = SegmentDebugColors
                    ? new Pen(new SolidColorBrush(DebugSegmentColor(moveIdx)), 1.7)
                    : GetVCarvePen(segZ, minVZ, 1.7);
                context.DrawLine(pen, a2d, b2d);
                moveIdx++;
            }
        }

        var maPen = new Pen(new SolidColorBrush(Color.FromArgb(190, 120, 35, 190)), 1.1);
        foreach (var seg in _medial)
            context.DrawLine(maPen, Tx(seg.A), Tx(seg.B));
    }

    private void DrawPerspectivePreview(DrawingContext context, Rect viewport)
    {
        var paddedViewport = viewport.Deflate(34);
        if (paddedViewport.Width <= 1 || paddedViewport.Height <= 1)
            return;

        bool hasWorld = false;
        var min = default(Vector3);
        var max = default(Vector3);

        void AccWorld(float x, float y, float z)
        {
            var p = ToWorld(x, y, z);
            if (!hasWorld)
            {
                min = p;
                max = p;
                hasWorld = true;
                return;
            }

            min = Vector3.Min(min, p);
            max = Vector3.Max(max, p);
        }

        foreach (var contour in _contours)
            foreach (var p in contour)
                AccWorld((float)p.X, (float)p.Y, 0f);

        foreach (var path in _clearing)
            foreach (var p in path)
                AccWorld((float)p.X, (float)p.Y, (float)p.Z);

        foreach (var path in _vcarve)
            foreach (var p in path)
                AccWorld((float)p.X, (float)p.Y, (float)p.Z);

        foreach (var seg in _medial)
        {
            AccWorld((float)seg.A.X, (float)seg.A.Y, 0f);
            AccWorld((float)seg.B.X, (float)seg.B.Y, 0f);
        }

        if (!hasWorld)
            return;

        var center = (min + max) * 0.5f + _orbitPanOffset;
        var extents = Vector3.Max(max - min, new Vector3(0.05f, 0.05f, 0.05f));
        var radius = Math.Max(0.05f, extents.Length() * 0.7f);
        var distance = radius * 2.4f * _orbitZoom;

        var dir = new Vector3(
            MathF.Cos(_orbitPitch) * MathF.Cos(_orbitYaw),
            MathF.Cos(_orbitPitch) * MathF.Sin(_orbitYaw),
            MathF.Sin(_orbitPitch));
        var cameraPos = center + dir * distance;

        var upWorld = MathF.Abs(dir.Z) > 0.98f ? Vector3.UnitY : Vector3.UnitZ;
        var forward = Vector3.Normalize(center - cameraPos);
        var right = Vector3.Normalize(Vector3.Cross(forward, upWorld));
        var up = Vector3.Normalize(Vector3.Cross(right, forward));

        var aspect = Math.Max(0.1f, (float)(paddedViewport.Width / paddedViewport.Height));
        const float fovDeg = 50f;
        var focal = 1f / MathF.Tan(0.5f * fovDeg * MathF.PI / 180f);

        var contourPen = new Pen(new SolidColorBrush(Color.FromArgb(230, 30, 30, 30)), 1.6);
        foreach (var contour in _contours)
        {
            for (int i = 1; i < contour.Count; i++)
            {
                var a = ToWorld((float)contour[i - 1].X, (float)contour[i - 1].Y, 0f);
                var b = ToWorld((float)contour[i].X, (float)contour[i].Y, 0f);
                if (TryProjectSegment(a, b, cameraPos, forward, right, up, focal, aspect, paddedViewport, out var a2d, out var b2d))
                    context.DrawLine(contourPen, a2d, b2d);
            }
        }

        for (int pathIndex = 0; pathIndex < _clearing.Count; pathIndex++)
        {
            var path = _clearing[pathIndex];
            double avgDepth = pathIndex < _clearingAverageDepths.Count ? _clearingAverageDepths[pathIndex] : 0.0;
            var pen = new Pen(new SolidColorBrush(ClearingColorForDepth(avgDepth)), 1.4);
            for (int i = 1; i < path.Count; i++)
            {
                var a = ToWorld((float)path[i - 1].X, (float)path[i - 1].Y, (float)path[i - 1].Z);
                var b = ToWorld((float)path[i].X, (float)path[i].Y, (float)path[i].Z);
                if (TryProjectSegment(a, b, cameraPos, forward, right, up, focal, aspect, paddedViewport, out var a2d, out var b2d))
                    context.DrawLine(pen, a2d, b2d);
            }

            // Clearing paths are concentric rings; close them in 3D just like 2D preview.
            if (path.Count > 2)
            {
                var first = path[0];
                var last = path[^1];
                if (first.X != last.X || first.Y != last.Y || first.Z != last.Z)
                {
                    var a = ToWorld((float)last.X, (float)last.Y, (float)last.Z);
                    var b = ToWorld((float)first.X, (float)first.Y, (float)first.Z);
                    if (TryProjectSegment(a, b, cameraPos, forward, right, up, focal, aspect, paddedViewport, out var a2d, out var b2d))
                        context.DrawLine(pen, a2d, b2d);
                }
            }
        }

        double minZ = MinVCarveZ();
        int moveIdx = 0;
        foreach (var path in _vcarve)
        {
            for (int i = 1; i < path.Count; i++)
            {
                var a0 = path[i - 1];
                var b0 = path[i];
                var a = ToWorld((float)a0.X, (float)a0.Y, (float)a0.Z);
                var b = ToWorld((float)b0.X, (float)b0.Y, (float)b0.Z);
                if (!TryProjectSegment(a, b, cameraPos, forward, right, up, focal, aspect, paddedViewport, out var a2d, out var b2d))
                    continue;
                var segZ = (a0.Z + b0.Z) * 0.5;
                var pen = SegmentDebugColors
                    ? new Pen(new SolidColorBrush(DebugSegmentColor(moveIdx)), 1.8)
                    : GetVCarvePen(segZ, minZ, 1.8);
                context.DrawLine(pen, a2d, b2d);
                moveIdx++;
            }
        }

        var maPen = new Pen(new SolidColorBrush(Color.FromArgb(190, 120, 35, 190)), 1.1);
        foreach (var seg in _medial)
        {
            var a = ToWorld((float)seg.A.X, (float)seg.A.Y, 0f);
            var b = ToWorld((float)seg.B.X, (float)seg.B.Y, 0f);
            if (TryProjectSegment(a, b, cameraPos, forward, right, up, focal, aspect, paddedViewport, out var a2d, out var b2d))
                context.DrawLine(maPen, a2d, b2d);
        }
    }

    private static Vector3 ToWorld(float x, float y, float z) => new(x, -y, z);

    private const float NearClip = 0.02f;
    private const float MaxProjectionRadiusFactor = 8.0f;

    private static bool TryProjectPoint(
        Vector3 point,
        Vector3 cameraPos,
        Vector3 forward,
        Vector3 right,
        Vector3 up,
        float focal,
        float aspect,
        Rect viewport,
        out Point projected)
    {
        var local = point - cameraPos;
        var z = Vector3.Dot(local, forward);
        if (z <= NearClip)
        {
            projected = default;
            return false;
        }

        var x = Vector3.Dot(local, right) * (focal / z);
        var y = Vector3.Dot(local, up) * (focal / z);

        // Reject points that project absurdly far due to near-plane grazing.
        if (Math.Abs(x) > MaxProjectionRadiusFactor * aspect || Math.Abs(y) > MaxProjectionRadiusFactor)
        {
            projected = default;
            return false;
        }

        projected = new Point(
            viewport.X + viewport.Width * 0.5 + (x / aspect) * viewport.Width * 0.5,
            viewport.Y + viewport.Height * 0.5 - y * viewport.Height * 0.5);

        if (!double.IsFinite(projected.X) || !double.IsFinite(projected.Y))
        {
            projected = default;
            return false;
        }

        return true;
    }

    private static bool TryProjectSegment(
        Vector3 a,
        Vector3 b,
        Vector3 cameraPos,
        Vector3 forward,
        Vector3 right,
        Vector3 up,
        float focal,
        float aspect,
        Rect viewport,
        out Point aProjected,
        out Point bProjected)
    {
        var aLocal = a - cameraPos;
        var bLocal = b - cameraPos;
        float za = Vector3.Dot(aLocal, forward);
        float zb = Vector3.Dot(bLocal, forward);

        if (za <= NearClip && zb <= NearClip)
        {
            aProjected = default;
            bProjected = default;
            return false;
        }

        // Clip segment against near plane to avoid projection spikes.
        if (za <= NearClip || zb <= NearClip)
        {
            float denom = zb - za;
            if (Math.Abs(denom) < 1e-9f)
            {
                aProjected = default;
                bProjected = default;
                return false;
            }

            float t = (NearClip - za) / denom;
            t = Math.Clamp(t, 0f, 1f);
            var clipped = a + (b - a) * t;

            if (za <= NearClip)
            {
                a = clipped;
                za = NearClip;
            }
            else
            {
                b = clipped;
                zb = NearClip;
            }
        }

        bool aOk = TryProjectPoint(a, cameraPos, forward, right, up, focal, aspect, viewport, out aProjected);
        bool bOk = TryProjectPoint(b, cameraPos, forward, right, up, focal, aspect, viewport, out bProjected);
        if (!aOk || !bOk)
            return false;

        // Final guard for extremely long screen-space segments.
        double dx = bProjected.X - aProjected.X;
        double dy = bProjected.Y - aProjected.Y;
        double maxLen = Math.Max(viewport.Width, viewport.Height) * 5.0;
        if ((dx * dx + dy * dy) > (maxLen * maxLen))
            return false;

        return true;
    }

    private void OnPointerPressed(object? sender, PointerPressedEventArgs e)
    {
        var props = e.GetCurrentPoint(this).Properties;
        if (PerspectiveEnabled)
        {
            if (props.IsLeftButtonPressed)
            {
                _isOrbiting = true;
            }
            else if (props.IsRightButtonPressed)
            {
                _isPanning3D = true;
            }
            else
                return;
        }
        else
        {
            if (!props.IsRightButtonPressed)
                return;

            _isPanning2D = true;
        }

        _lastMouse = e.GetPosition(this);
        e.Pointer.Capture(this);
    }

    private void OnPointerMoved(object? sender, PointerEventArgs e)
    {
        var pos = e.GetPosition(this);
        var dx = (float)(pos.X - _lastMouse.X);
        var dy = (float)(pos.Y - _lastMouse.Y);
        _lastMouse = pos;

        if (PerspectiveEnabled)
        {
            if (_isOrbiting)
            {
                _orbitYaw -= dx * 0.01f;
                _orbitPitch = Math.Clamp(_orbitPitch - dy * 0.01f, -1.45f, 1.45f);
            }
            else if (_isPanning3D)
            {
                var dir = new Vector3(
                    MathF.Cos(_orbitPitch) * MathF.Cos(_orbitYaw),
                    MathF.Cos(_orbitPitch) * MathF.Sin(_orbitYaw),
                    MathF.Sin(_orbitPitch));
                var forward = Vector3.Normalize(-dir);
                var upWorld = MathF.Abs(dir.Z) > 0.98f ? Vector3.UnitY : Vector3.UnitZ;
                var right = Vector3.Normalize(Vector3.Cross(forward, upWorld));
                var up = Vector3.Normalize(Vector3.Cross(right, forward));

                float sceneSpan = (float)Math.Max(0.05, Math.Max(_maxX - _minX, _maxY - _minY));
                float pxToWorld = sceneSpan * _orbitZoom / (float)Math.Max(80.0, Bounds.Height);
                _orbitPanOffset += (-dx * pxToWorld) * right + (dy * pxToWorld) * up;
            }
            else
            {
                return;
            }

            InvalidateVisual();
            return;
        }

        if (!_isPanning2D)
            return;

        _panX += dx;
        _panY += dy;
        InvalidateVisual();
    }

    private void OnPointerReleased(object? sender, PointerReleasedEventArgs e)
    {
        if (!_isOrbiting && !_isPanning2D && !_isPanning3D)
            return;

        _isOrbiting = false;
        _isPanning2D = false;
        _isPanning3D = false;
        e.Pointer.Capture(null);
    }

    private void OnPointerWheelChanged(object? sender, PointerWheelEventArgs e)
    {
        var wheel = Math.Clamp(e.Delta.Y, -6, 6);
        var zoomStep = Math.Exp(wheel * 0.11);

        if (PerspectiveEnabled)
        {
            _orbitZoom = Math.Clamp(_orbitZoom / (float)zoomStep, 0.08f, 10.0f);
            InvalidateVisual();
            return;
        }

        var viewport = new Rect(0, 0, Bounds.Width, Bounds.Height);
        if (!_hasBounds || viewport.Width <= 1 || viewport.Height <= 1)
            return;

        var mouse = e.GetPosition(this);
        var center = new Point(viewport.X + viewport.Width * 0.5, viewport.Y + viewport.Height * 0.5);
        var oldZoom = _zoom2D;
        var newZoom = Math.Clamp(oldZoom * zoomStep, 0.2, 80.0);
        if (Math.Abs(newZoom - oldZoom) < 1e-9)
            return;

        var localX = mouse.X - center.X - _panX;
        var localY = mouse.Y - center.Y - _panY;
        _panX -= localX * (newZoom / oldZoom - 1.0);
        _panY -= localY * (newZoom / oldZoom - 1.0);
        _zoom2D = newZoom;
        InvalidateVisual();
    }

    private (double scale, double offsetX, double offsetY) BuildTransform(Rect viewport, double padding)
    {
        double width = Math.Max(1e-9, _maxX - _minX);
        double height = Math.Max(1e-9, _maxY - _minY);

        double sx = (viewport.Width - 2 * padding) / width;
        double sy = (viewport.Height - 2 * padding) / height;
        double scale = Math.Max(0.01, Math.Min(sx, sy));

        double drawW = width * scale;
        double drawH = height * scale;
        double offsetX = (viewport.Width - drawW) * 0.5 - _minX * scale;
        double offsetY = (viewport.Height - drawH) * 0.5 - _minY * scale;

        return (scale, offsetX, offsetY);
    }

    private static void DrawPolyline(DrawingContext context, IReadOnlyList<Point> points, Pen pen, bool closeLoop)
    {
        if (points.Count < 2)
            return;

        for (int i = 1; i < points.Count; i++)
            context.DrawLine(pen, points[i - 1], points[i]);

        if (closeLoop && points.Count > 2)
            context.DrawLine(pen, points[^1], points[0]);
    }

    private static void DrawText(DrawingContext context, string text, Point origin, Color color)
    {
        var formatted = new FormattedText(
            text,
            System.Globalization.CultureInfo.InvariantCulture,
            FlowDirection.LeftToRight,
            Typeface.Default,
            12,
            new SolidColorBrush(color));
        context.DrawText(formatted, origin);
    }

    private double MinVCarveZ() => _minVCarveZ;

    private Pen GetVCarvePen(double z, double minZ, double thickness)
    {
        var color = VCarveColorForDepth(z, minZ);
        int key = (thickness > 1.75 ? 1 : 0)
                  | (color.A << 8)
                  | (color.R << 16)
                  | (color.G << 24)
                  ^ color.B;

        if (_vcarvePenCache.TryGetValue(key, out var cached))
            return cached;

        var pen = new Pen(new SolidColorBrush(color), thickness);
        _vcarvePenCache[key] = pen;
        return pen;
    }

    private static Color ClearingColorForDepth(double z)
    {
        double t = Math.Clamp(-z / 0.3, 0.0, 1.0);
        byte r = (byte)(80 + (1.0 - t) * 80);
        byte g = (byte)(130 + (1.0 - t) * 60);
        byte b = (byte)(220 + t * 20);
        return Color.FromArgb(220, r, g, b);
    }

    private static Color VCarveColorForDepth(double z, double minZ)
    {
        double denom = minZ >= 0 ? -0.25 : minZ;
        double t = Math.Clamp(z / denom, 0.0, 1.0);
        byte alpha = (byte)(40 + 190 * t);
        byte red = (byte)(140 + 80 * t);
        byte green = (byte)(25 + 25 * (1.0 - t));
        byte blue = (byte)(25 + 25 * (1.0 - t));
        return Color.FromArgb(alpha, red, green, blue);
    }

    private static Color DebugSegmentColor(int idx)
    {
        double h = (idx * 137.508) % 360.0;
        const double c = 0.85, v = 0.90;
        double x = c * (1.0 - Math.Abs((h / 60.0) % 2.0 - 1.0));
        double m = v - c;
        double r, g, b;
        if      (h <  60) { r = c; g = x; b = 0; }
        else if (h < 120) { r = x; g = c; b = 0; }
        else if (h < 180) { r = 0; g = c; b = x; }
        else if (h < 240) { r = 0; g = x; b = c; }
        else if (h < 300) { r = x; g = 0; b = c; }
        else              { r = c; g = 0; b = x; }
        return Color.FromArgb(220, (byte)((r + m) * 255), (byte)((g + m) * 255), (byte)((b + m) * 255));
    }
}

