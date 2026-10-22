#if CLIPPER_DIAGNOSTICS
using System;
using System.Collections.Generic;
using System.IO;
using System.Text;
using System.Threading;

namespace Clipper2Lib
{
  // Deliberately internal and compiled out of ordinary builds. Reflection-based
  // validation can read each exact sequence without extending the public API.
  internal sealed class ClipperDiagnostics
  {
    internal long InsertedEvents, ShiftedElements, ConsumedScanlines, Intersections;
    internal int PeakQueueSize, PeakActiveEdges;
    internal readonly List<long> Scanlines = new List<long>();
    private static long _executions, _inserted, _shifted, _consumed, _intersections;
    private static int _captures;
    private readonly string? _directory = Environment.GetEnvironmentVariable("RW_CLIPPER_CAPTURE_DIR");
    private readonly string _label = Environment.GetEnvironmentVariable("RW_CLIPPER_CAPTURE_LABEL") ?? "clipper";
    private readonly int _limit = Number("RW_CLIPPER_CAPTURE_LIMIT", 8);
    private readonly int _minimum = Number("RW_CLIPPER_CAPTURE_MIN_VERTICES", 1000);
    private readonly bool _trace = Environment.GetEnvironmentVariable("RW_CLIPPER_TRACE_SCANLINES") == "1";
    private readonly Paths64 _subject = new Paths64(), _clip = new Paths64(), _open = new Paths64();
    private bool _captureSupported = true;

    private static int Number(string name, int fallback)
      => int.TryParse(Environment.GetEnvironmentVariable(name), out int value) ? Math.Max(0, value) : fallback;
    internal void Reset(int minima)
    {
      InsertedEvents = minima; ShiftedElements = ConsumedScanlines = Intersections = 0;
      PeakQueueSize = minima; PeakActiveEdges = 0; Scanlines.Clear();
    }
    internal void Insert(int size, long shifts = 0)
    {
      InsertedEvents++; ShiftedElements += shifts; PeakQueueSize = Math.Max(PeakQueueSize, size);
    }
    internal void Pop(long y)
    {
      ConsumedScanlines++;
      if (_trace) Scanlines.Add(y);
    }
    internal void Active(Active? edge)
    {
      int count = 0;
      while (edge != null) { count++; edge = edge.nextInAEL; }
      PeakActiveEdges = Math.Max(PeakActiveEdges, count);
    }
    internal void Input(Paths64 paths, PathType type, bool open)
    {
      if (string.IsNullOrWhiteSpace(_directory) || Volatile.Read(ref _captures) >= _limit) return;
      Paths64 target = open ? _open : type == PathType.Subject ? _subject : _clip;
      foreach (Path64 path in paths) target.Add(new Path64(path));
    }
    internal void ReusableInput() { _captureSupported = false; }
    internal void ClearInputs() { _subject.Clear(); _clip.Clear(); _open.Clear(); _captureSupported = true; }
    internal void Report(ClipType operation, FillRule fill, bool preserve, bool reverse)
    {
      long n = Interlocked.Increment(ref _executions);
      Interlocked.Add(ref _inserted, InsertedEvents); Interlocked.Add(ref _shifted, ShiftedElements);
      Interlocked.Add(ref _consumed, ConsumedScanlines); Interlocked.Add(ref _intersections, Intersections);
      if (n <= 8 || (n & (n - 1)) == 0)
        Console.WriteLine("[Clipper.ScanlineQueue] executions=" + n + " insertedEvents=" + Interlocked.Read(ref _inserted)
          + " shiftedElements=" + Interlocked.Read(ref _shifted) + " consumedScanlines=" + Interlocked.Read(ref _consumed)
          + " intersections=" + Interlocked.Read(ref _intersections) + " lastPeakQueue=" + PeakQueueSize
          + " lastPeakActiveEdges=" + PeakActiveEdges + " diagnosticBuild=True");
      if (!_captureSupported || string.IsNullOrWhiteSpace(_directory) || Volatile.Read(ref _captures) >= _limit) return;
      long vertices = 0;
      foreach (Paths64 source in new[] { _subject, _clip, _open }) foreach (Path64 path in source) vertices += path.Count;
      if (vertices < _minimum) return;
      int ordinal = Interlocked.Increment(ref _captures);
      if (ordinal > _limit) return;
      Directory.CreateDirectory(_directory!);
      var json = new StringBuilder("{\"operation\":\"").Append(operation).Append("\",\"fill\":\"").Append(fill)
        .Append("\",\"preserveCollinear\":").Append(preserve ? "true" : "false")
        .Append(",\"reverse\":").Append(reverse ? "true" : "false").Append(",\"subject\":");
      WritePaths(json, _subject); json.Append(",\"clip\":"); WritePaths(json, _clip);
      json.Append(",\"open\":"); WritePaths(json, _open); json.Append('}');
      // Label is a filename component only, never a caller-controlled path.
      string safeLabel = string.Join("_", _label.Split(Path.GetInvalidFileNameChars()));
      string filename = Path.Combine(_directory!, safeLabel + "-" + ordinal.ToString("D3") + ".json");
      File.WriteAllText(filename, json.ToString());
      Console.WriteLine("[Clipper.ScanlineCapture] path='" + filename + "' vertices=" + vertices + " operation=" + operation);
    }
    private static void WritePaths(StringBuilder json, Paths64 paths)
    {
      json.Append('[');
      for (int i = 0; i < paths.Count; i++)
      {
        if (i > 0) json.Append(','); json.Append('[');
        for (int j = 0; j < paths[i].Count; j++)
        {
          if (j > 0) json.Append(',');
          json.Append('[').Append(paths[i][j].X).Append(',').Append(paths[i][j].Y).Append(']');
        }
        json.Append(']');
      }
      json.Append(']');
    }
  }
}
#endif
