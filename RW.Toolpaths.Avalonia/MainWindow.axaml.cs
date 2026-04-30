using System.Globalization;
using System.Diagnostics;
using Avalonia;
using Avalonia.Controls;
using Avalonia.Threading;
using Clipper2Lib;
using RW.Toolpaths;

namespace RW.Toolpaths.Avalonia;

public partial class MainWindow : Window
{
    private const double PxToInch = 1.0 / 96.0;
    private const double MmPerInch = 25.4;
    private const double MinToleranceMm = MmPerInch / 32.0;
    private static readonly TimeSpan RegionSolveTimeout = TimeSpan.FromSeconds(8);

    private readonly TextBox _textInput;
    private readonly ComboBox _fontCombo;
    private readonly NumericUpDown _fontSizeInput;
    private readonly TextBox _derivedVBitAngleText;
    private readonly NumericUpDown _endDepthInput;
    private readonly NumericUpDown _depthPerPassInput;
    private readonly NumericUpDown _startDepthInput;
    private readonly NumericUpDown _toleranceInput;
    private readonly NumericUpDown _stepOverInput;
    private readonly NumericUpDown _bottomRadiusInput;
    private readonly NumericUpDown _topRadiusInput;
    private readonly NumericUpDown _coneLengthInput;
    private readonly CheckBox _perspectiveToggle;
    private readonly CheckBox _segmentDebugToggle;
    private readonly Button _resetViewButton;
    private readonly Button _generateButton;
    private readonly TextBlock _statusText;
    private readonly GeometryPreviewControl _preview;

    private readonly DispatcherTimer _debounceTimer;
    private readonly Lazy<IMedialAxisProvider> _medialAxisProvider =
        new(() => BoostVoronoiProvider.CreateDefault());

    private CancellationTokenSource? _generateCts;
    private int _generationVersion;

    private const int RegionCacheCapacity = 256;
    private readonly Dictionary<string, (List<List<Point3D>> Clearing, List<List<Point3D>> VCarve)> _regionCache = new();

    public MainWindow()
    {
        InitializeComponent();

        _textInput = FindRequired<TextBox>("InputTextBox");
        _fontCombo = FindRequired<ComboBox>("FontCombo");
        _fontSizeInput = FindRequired<NumericUpDown>("FontSizeInput");
        _derivedVBitAngleText = FindRequired<TextBox>("DerivedVBitAngleText");
        _endDepthInput = FindRequired<NumericUpDown>("EndDepthInput");
        _depthPerPassInput = FindRequired<NumericUpDown>("DepthPerPassInput");
        _startDepthInput = FindRequired<NumericUpDown>("StartDepthInput");
        _toleranceInput = FindRequired<NumericUpDown>("ToleranceInput");
        _stepOverInput = FindRequired<NumericUpDown>("StepOverInput");
        _bottomRadiusInput = FindRequired<NumericUpDown>("BottomRadiusInput");
        _topRadiusInput = FindRequired<NumericUpDown>("TopRadiusInput");
        _coneLengthInput = FindRequired<NumericUpDown>("ConeLengthInput");
        _perspectiveToggle = FindRequired<CheckBox>("PerspectiveToggle");
        _segmentDebugToggle = FindRequired<CheckBox>("SegmentDebugToggle");
        _resetViewButton = FindRequired<Button>("ResetViewButton");
        _generateButton = FindRequired<Button>("GenerateButton");
        _statusText = FindRequired<TextBlock>("StatusText");
        _preview = FindRequired<GeometryPreviewControl>("PreviewControl");

        if (OperatingSystem.IsWindows())
        {
            PopulateFonts();
        }

        _debounceTimer = new DispatcherTimer { Interval = TimeSpan.FromMilliseconds(220) };
        _debounceTimer.Tick += (_, _) =>
        {
            _debounceTimer.Stop();
            StartGenerate();
        };

        _generateButton.Click += (_, _) => StartGenerate();
        _resetViewButton.Click += (_, _) => _preview.ResetView();

        _perspectiveToggle.IsChecked = false;
        _perspectiveToggle.IsCheckedChanged += (_, _) =>
        {
            _preview.PerspectiveEnabled = _perspectiveToggle.IsChecked == true;
            _preview.InvalidateVisual();
        };

        _segmentDebugToggle.IsChecked = false;
        _segmentDebugToggle.IsCheckedChanged += (_, _) =>
        {
            _preview.SegmentDebugColors = _segmentDebugToggle.IsChecked == true;
            _preview.InvalidateVisual();
        };

        _textInput.PropertyChanged += (_, e) =>
        {
            if (e.Property.Name == nameof(TextBox.Text))
                QueueGenerate();
        };
        _fontCombo.SelectionChanged += (_, _) => QueueGenerate();
        _fontSizeInput.PropertyChanged += OnNumericChanged;
        _endDepthInput.PropertyChanged += OnNumericChanged;
        _depthPerPassInput.PropertyChanged += OnNumericChanged;
        _startDepthInput.PropertyChanged += OnNumericChanged;
        _toleranceInput.PropertyChanged += OnNumericChanged;
        _stepOverInput.PropertyChanged += OnNumericChanged;
        _bottomRadiusInput.PropertyChanged += OnNumericChanged;
        _topRadiusInput.PropertyChanged += OnNumericChanged;
        _coneLengthInput.PropertyChanged += OnNumericChanged;

        Opened += (_, _) => StartGenerate();
        Closing += (_, _) =>
        {
            _generateCts?.Cancel();
            _generateCts?.Dispose();
            _debounceTimer.Stop();
        };
    }

    private void OnNumericChanged(object? sender, AvaloniaPropertyChangedEventArgs e)
    {
        if (e.Property.Name == nameof(NumericUpDown.Value))
        {
            UpdateDerivedAngleDisplay();
            QueueGenerate();
        }
    }

    private void QueueGenerate()
    {
        _debounceTimer.Stop();
        _debounceTimer.Start();
    }

    private void UpdateDerivedAngleDisplay()
    {
        double bottomRadiusMm = Math.Max(0.0, (double)(_bottomRadiusInput.Value ?? 0m));
        double topRadiusMm = Math.Max(bottomRadiusMm, (double)(_topRadiusInput.Value ?? 6.35m));
        double coneLengthMm = Math.Max(0.01, (double)(_coneLengthInput.Value ?? 6.35m));
        double coneLength = coneLengthMm / MmPerInch;
        double bottomRadius = bottomRadiusMm / MmPerInch;
        double topRadius = topRadiusMm / MmPerInch;
        double slope = Math.Max(1e-9, (topRadius - bottomRadius) / coneLength);
        double angleDeg = 2.0 * Math.Atan(slope) * 180.0 / Math.PI;
        _derivedVBitAngleText.Text = angleDeg.ToString("0.###", CultureInfo.InvariantCulture);
    }
    [System.Runtime.Versioning.SupportedOSPlatform("windows")]    private void PopulateFonts()
    {
        List<string> fonts;
        if (OperatingSystem.IsWindows())
        {
            fonts = System.Drawing.FontFamily.Families
                .Select(f => f.Name)
                .OrderBy(n => n)
                .ToList();
        }
        else
        {
            fonts = new List<string> { "Arial" };
        }

        if (fonts.Count == 0)
            fonts.Add("Arial");

        _fontCombo.ItemsSource = fonts;
        _fontCombo.SelectedItem = fonts.Contains("Abel") ? "Abel" : fonts.Contains("Arial") ? "Arial" : fonts[0];
        UpdateDerivedAngleDisplay();
    }

    private T FindRequired<T>(string name) where T : Control
        => this.FindControl<T>(name)
           ?? throw new InvalidOperationException($"Missing required control '{name}'.");

    private void StartGenerate()
    {
        _generateCts?.Cancel();
        _generateCts?.Dispose();

        _generateCts = new CancellationTokenSource();
        int generationVersion = ++_generationVersion;
        _ = GenerateAsync(generationVersion, _generateCts.Token);
    }

    private async Task GenerateAsync(int generationVersion, CancellationToken cancellationToken)
    {
        if (!OperatingSystem.IsWindows())
        {
            await Dispatcher.UIThread.InvokeAsync(() =>
            {
                _statusText.Text = "Font glyph extraction is only supported on Windows.";
            });
            return;
        }
        var swTotal = Stopwatch.StartNew();

        try
        {
            Console.Error.WriteLine($"[ui] gen#{generationVersion} start");
            await Dispatcher.UIThread.InvokeAsync(() => _statusText.Text = "Generating...");

            var text = _textInput.Text ?? string.Empty;
            if (string.IsNullOrWhiteSpace(text))
            {
                await Dispatcher.UIThread.InvokeAsync(() =>
                {
                    _preview.SetGeometry(new(), new(), new(), new(), "empty");
                    _statusText.Text = "Enter text to generate.";
                });
                return;
            }

            string font = _fontCombo.SelectedItem?.ToString() ?? "Arial";
            float emSize = (float)(_fontSizeInput.Value ?? 180m);

            double endDepth = (double)(_endDepthInput.Value ?? 9m) / MmPerInch;
            double depthPerPass = (double)(_depthPerPassInput.Value ?? 3m) / MmPerInch;
            double startDepth = (double)(_startDepthInput.Value ?? 0m) / MmPerInch;
            double toleranceMm = (double)(_toleranceInput.Value ?? (decimal)MinToleranceMm);
            if (toleranceMm < MinToleranceMm)
                toleranceMm = MinToleranceMm;
            double tolerance = toleranceMm / MmPerInch;
            double stepOver = (double)(_stepOverInput.Value ?? 0.4m);
            double bottomRadiusMm = Math.Max(0.0, (double)(_bottomRadiusInput.Value ?? 0m));
            double topRadiusMm = Math.Max(bottomRadiusMm, (double)(_topRadiusInput.Value ?? 6.35m));
            double coneLengthMm = Math.Max(0.01, (double)(_coneLengthInput.Value ?? 6.35m));
            double bottomRadius = bottomRadiusMm / MmPerInch;
            double topRadius = topRadiusMm / MmPerInch;
            double coneLength = coneLengthMm / MmPerInch;
            double slope = Math.Max(1e-9, (topRadius - bottomRadius) / coneLength);
            double radianTipAngle = 2.0 * Math.Atan(slope);

            await Dispatcher.UIThread.InvokeAsync(UpdateDerivedAngleDisplay);

            cancellationToken.ThrowIfCancellationRequested();

            var contourResult = await Task.Run(() =>
            {
                var swContours = Stopwatch.StartNew();
                cancellationToken.ThrowIfCancellationRequested();

#pragma warning disable CA1416
                var rawContours = FontGlyphExtractor.ExtractContours(text, font, emSize);
#pragma warning restore CA1416
                var rawContoursInches = rawContours
                    .Select(c => c.Select(p => new Point(p.X * PxToInch, p.Y * PxToInch)).ToList())
                    .Where(c => c.Count >= 3)
                    .ToList();

                bool usedFallbackContours = false;

                if (rawContoursInches.Count == 0)
                {
                    rawContoursInches = BuildDebugFallbackContours(text, emSize);
                    usedFallbackContours = true;
                }

                var canonicalRings = PathUtils.CanonicalizeRings(
                    rawContoursInches.Select(c => c.Select(p => new PointD(p.X, p.Y))));

                var contoursInches = canonicalRings
                    .Select(r => r.Select(p => new Point(p.x, p.y)).ToList())
                    .Where(c => c.Count >= 3)
                    .ToList();

                if (contoursInches.Count == 0)
                {
                    contoursInches = rawContoursInches;
                    canonicalRings = rawContoursInches
                        .Select(c => c.Select(p => new PointD(p.X, p.Y)).ToList())
                        .ToList();
                }

                var rings = canonicalRings
                    .Select(c => c.ToList())
                    .Where(r => r.Count >= 3)
                    .ToList();

                Console.Error.WriteLine($"[ui] gen#{generationVersion} contours done in {swContours.ElapsedMilliseconds}ms (rings={rings.Count})");
                return (contoursInches: contoursInches, rings: rings, usedFallbackContours: usedFallbackContours);
            }, cancellationToken);

            if (generationVersion != _generationVersion || cancellationToken.IsCancellationRequested)
                return;

            await Dispatcher.UIThread.InvokeAsync(() =>
            {
                string contourDebug = contourResult.usedFallbackContours ? "contours-only|fallback-contours" : "contours-only";
                _preview.SetGeometry(contourResult.contoursInches, new(), new(), new(), contourDebug);
                _statusText.Text = string.Create(
                    CultureInfo.InvariantCulture,
                    $"Contours: {contourResult.contoursInches.Count} | Generating toolpaths...");
            });

            // Build the region tree on a thread-pool thread (fast but not trivial).
            var regions = await Task.Run(() =>
            {
                var sw = Stopwatch.StartNew();
                var r = BuildMedialAxisRegions(contourResult.rings);
                Console.Error.WriteLine($"[ui] gen#{generationVersion} region build done in {sw.ElapsedMilliseconds}ms (regions={r.Count})");
                return r;
            }, cancellationToken);

            if (generationVersion != _generationVersion || cancellationToken.IsCancellationRequested)
                return;

            var allClearing = new List<List<Point3D>>();
            var allVCarve   = new List<List<Point3D>>();
            int allVCarveSegmentCount = 0;
            var swToolpaths = Stopwatch.StartNew();

            for (int i = 0; i < regions.Count; i++)
            {
                cancellationToken.ThrowIfCancellationRequested();
                if (generationVersion != _generationVersion) return;

                var region   = regions[i];
                var cacheKey = ComputeRegionCacheKey(region, startDepth, endDepth, radianTipAngle, depthPerPass, stepOver, tolerance, bottomRadius, topRadius, coneLength);

                List<List<Point3D>> regionClearing;
                List<List<Point3D>> regionVCarve;

                if (_regionCache.TryGetValue(cacheKey, out var cached))
                {
                    regionClearing = cached.Clearing;
                    regionVCarve   = cached.VCarve;
                    Console.Error.WriteLine($"[ui] gen#{generationVersion} region {i + 1}/{regions.Count} cache hit");
                }
                else
                {
                    var swRegion     = Stopwatch.StartNew();
                    var regionCapture = region;
                    try
                    {
                        var (clearing, vcarve) = await Task.Run(
                            () => MedialAxisToolpaths.GenerateVCarveComponents(
                                _medialAxisProvider.Value,
                                regionCapture,
                                startDepth,
                                endDepth,
                                radianTipAngle,
                                depthPerPass,
                                stepOver,
                                tolerance,
                                bottomRadiusOverride: bottomRadius,
                                topRadiusOverride: topRadius,
                                coneLengthOverride: coneLength),
                            cancellationToken)
                            .WaitAsync(RegionSolveTimeout, cancellationToken);

                        regionClearing = clearing;
                        regionVCarve   = vcarve;

                        if (_regionCache.Count >= RegionCacheCapacity)
                            _regionCache.Clear();
                        _regionCache[cacheKey] = (regionClearing, regionVCarve);

                        Console.Error.WriteLine($"[ui] gen#{generationVersion} region {i + 1}/{regions.Count} done in {swRegion.ElapsedMilliseconds}ms (clearing={regionClearing.Count}, vcarve={regionVCarve.Count})");
                    }
                    catch (TimeoutException)
                    {
                        Console.Error.WriteLine($"[ui] gen#{generationVersion} region {i + 1}/{regions.Count} timed out; skipping.");
                        continue;
                    }
                    catch (OperationCanceledException) { throw; }
                    catch (Exception ex)
                    {
                        Console.Error.WriteLine($"[ui] gen#{generationVersion} region {i + 1}/{regions.Count} failed: {ex.GetBaseException()}");
                        continue;
                    }
                }

                allClearing.AddRange(regionClearing);
                allVCarve.AddRange(regionVCarve);
                for (int p = 0; p < regionVCarve.Count; p++)
                    allVCarveSegmentCount += Math.Max(0, regionVCarve[p].Count - 1);

                if (generationVersion != _generationVersion) return;

                int regionsDone  = i + 1;
                int regionsTotal = regions.Count;
                bool isFinal     = regionsDone == regionsTotal;
                bool shouldPublishPartial = isFinal || regionsDone == 1 || (regionsDone % 3 == 0);

                if (!shouldPublishPartial)
                    continue;

                // Stream partial results periodically to avoid repeated large snapshots.
                var clearingSnap = new List<List<Point3D>>(allClearing);
                var vcarveSnap   = new List<List<Point3D>>(allVCarve);

                await Dispatcher.UIThread.InvokeAsync(() =>
                {
                    if (generationVersion != _generationVersion) return;

                    string partialDebug = $"regions={regionsDone}/{regionsTotal}"
                        + (contourResult.usedFallbackContours ? "|fallback-contours" : "");
                    _preview.SetGeometry(contourResult.contoursInches, clearingSnap, vcarveSnap, new(), partialDebug);

                    string suffix = isFinal ? string.Empty : $" | Island {regionsDone}/{regionsTotal}...";
                    _statusText.Text = string.Create(
                        CultureInfo.InvariantCulture,
                        $"Contours: {contourResult.contoursInches.Count} | Clearing: {clearingSnap.Count} | VCarve Paths: {vcarveSnap.Count} | VCarve Segs: {allVCarveSegmentCount} | {partialDebug}{suffix}");
                });
            }

            Console.Error.WriteLine($"[ui] gen#{generationVersion} complete in {swTotal.ElapsedMilliseconds}ms (clearing={allClearing.Count}, vcarve={allVCarve.Count}, total={swToolpaths.ElapsedMilliseconds}ms)");
        }
        catch (OperationCanceledException)
        {
            // superseded generation request
            Console.Error.WriteLine($"[ui] gen#{generationVersion} canceled after {swTotal.ElapsedMilliseconds}ms");
        }
        catch (Exception ex)
        {
            if (generationVersion != _generationVersion)
                return;

            Console.Error.WriteLine($"[ui] gen#{generationVersion} fatal error after {swTotal.ElapsedMilliseconds}ms: {ex}");

            await Dispatcher.UIThread.InvokeAsync(() =>
            {
                _statusText.Text = "Error: " + ex.Message;
                _preview.SetGeometry(new(), new(), new(), new(), "generation-error");
            });
        }
    }

    private static List<List<Point>> BuildDebugFallbackContours(string text, float emSize)
    {
        var contours = new List<List<Point>>();
        double w = Math.Max(8.0, emSize * 0.55) * PxToInch;
        double h = Math.Max(8.0, emSize) * PxToInch;
        double gap = Math.Max(2.0, emSize * 0.08) * PxToInch;

        double x = 0.0;
        foreach (char ch in text)
        {
            if (char.IsWhiteSpace(ch))
            {
                x += w * 0.5;
                continue;
            }

            contours.Add(new List<Point>
            {
                new(x, 0),
                new(x + w, 0),
                new(x + w, h),
                new(x, h),
                new(x, 0),
            });

            x += w + gap;
        }

        return contours;
    }

    private (List<(Point A, Point B)> Segments, string Debug) GenerateRawMedialAxisPreviewSegments(
        List<List<PointD>> rings,
        double depthPerPass,
        double radianTipAngle,
        double tolerance,
        double coneLength)
    {
        var regions = BuildMedialAxisRegions(rings);

        var output = new List<(Point A, Point B)>();
        int regionCount = 0;
        int regionErrors = 0;
        var regionSegments = new List<int>();

        foreach (var region in regions)
        {
            regionCount++;

            try
            {
                var segments = MedialAxisToolpaths.GenerateRawMedialAxisSegments(
                    _medialAxisProvider.Value,
                    region,
                    radianTipAngle,
                    depthPerPass,
                    tolerance,
                    maxRadiusOverride: null,
                    coneLengthOverride: coneLength);

                regionSegments.Add(segments.Count);
                foreach (var seg in segments)
                {
                    output.Add((
                        new Point(seg.Point0.X, seg.Point0.Y),
                        new Point(seg.Point1.X, seg.Point1.Y)));
                }
            }
            catch
            {
                regionErrors++;
                regionSegments.Add(-1);
            }
        }

        string segSummary = string.Join(",", regionSegments.Take(8));
        if (regionSegments.Count > 8) segSummary += ",...";

        string debug = $"regions={regionCount};errors={regionErrors};seg={segSummary}";
        return (output, debug);
    }

    private static string ComputeRegionCacheKey(
        IReadOnlyList<IReadOnlyList<PointD>> region,
        double startDepth, double endDepth, double radianTipAngle,
        double depthPerPass, double stepOver, double tolerance,
        double bottomRadius, double topRadius, double coneLength)
    {
        // Deterministic geometry hash (O(n) in ring point count).
        long h = 17;
        foreach (var ring in region)
        {
            h = h * 31 + ring.Count;
            foreach (var p in ring)
            {
                h = h * 1_000_003 + BitConverter.DoubleToInt64Bits(p.x);
                h = h * 1_000_003 + BitConverter.DoubleToInt64Bits(p.y);
            }
        }
        return $"{h}:{startDepth:R}:{endDepth:R}:{radianTipAngle:R}:{depthPerPass:R}:{stepOver:R}:{tolerance:R}:{bottomRadius:R}:{topRadius:R}:{coneLength:R}";
    }

    private static List<IReadOnlyList<IReadOnlyList<PointD>>> BuildMedialAxisRegions(List<List<PointD>> rings)
    {
        long t0 = PerfLog.Start();
        var canonicalRings = PathUtils.CanonicalizeRings(rings);
        var roots = OffsetFill.BuildPathTree(canonicalRings);
        var regions = new List<IReadOnlyList<IReadOnlyList<PointD>>>();

        static List<PointD> EnsureWinding(List<PointD> ring, bool ccw)
        {
            var area = Clipper.Area(PathUtils.ToClipper(new[] { ring })[0]);
            bool isCcw = area > 0;
            if (isCcw == ccw) return ring;

            var copy = new List<PointD>(ring);
            copy.Reverse();
            return copy;
        }

        static void Walk(PathTreeNode node, List<IReadOnlyList<IReadOnlyList<PointD>>> output)
        {
            var region = new List<IReadOnlyList<PointD>>
            {
                EnsureWinding(node.Points, ccw: true)
            };

            foreach (var hole in node.Children)
                region.Add(EnsureWinding(hole.Points, ccw: false));

            output.Add(region);

            foreach (var hole in node.Children)
            foreach (var nestedOuter in hole.Children)
                Walk(nestedOuter, output);
        }

        foreach (var root in roots)
            Walk(root, regions);

        PerfLog.Stop(
            "MainWindow.BuildMedialAxisRegions",
            t0,
            $"rings={canonicalRings.Count} roots={roots.Count} regions={regions.Count}");

        return regions;
    }
}

