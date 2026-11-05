using System.Diagnostics;
using System.Globalization;
using System.Runtime.CompilerServices;
using RW.Diagnostics;

namespace RW.Toolpaths;

public static class PerfLog
{
    private static readonly bool TimingAllowed = ResolveEnabled();
    private static readonly Lazy<string?> TimingFilePath = new(ResolveFilePath);
    private static readonly object FileLock = new();

    public static bool IsEnabled => TimingAllowed && DiagnosticLog.IsEnabled("perf");

    public static long Start() => IsEnabled ? Stopwatch.GetTimestamp() : 0L;

    public static void Stop(string area, long startTimestamp,
        [InterpolatedStringHandlerArgument("startTimestamp")] ref TimingDetailsHandler details)
    {
        if (details.Enabled) Stop(area, startTimestamp, details.GetText());
    }

    public static void Stop(string area, long startTimestamp, string? details = null)
    {
        if (!IsEnabled || startTimestamp == 0L)
            return;

        double elapsedMs = (Stopwatch.GetTimestamp() - startTimestamp) * 1000.0 / Stopwatch.Frequency;
        string message;
        if (string.IsNullOrWhiteSpace(details))
        {
            message = string.Create(
                CultureInfo.InvariantCulture,
                $"[perf] {area} {elapsedMs:F2}ms");
        }
        else
        {
            message = string.Create(
                CultureInfo.InvariantCulture,
                $"[perf] {area} {elapsedMs:F2}ms {details}");
        }

        Console.Error.WriteLine(message);

        if (!string.IsNullOrWhiteSpace(TimingFilePath.Value))
        {
            lock (FileLock)
            {
                File.AppendAllText(TimingFilePath.Value!, message + Environment.NewLine);
            }
        }
    }

    private static bool ResolveEnabled()
    {
        string? env = Environment.GetEnvironmentVariable("RW_TOOLPATHS_TIMING");
        if (!string.IsNullOrWhiteSpace(env))
        {
            return env != "0" &&
                   !string.Equals(env, "false", StringComparison.OrdinalIgnoreCase) &&
                   !string.Equals(env, "off", StringComparison.OrdinalIgnoreCase);
        }

        return true;
    }

    private static string? ResolveFilePath()
    {
        string? path = Environment.GetEnvironmentVariable("RW_TOOLPATHS_TIMING_FILE");
        if (string.IsNullOrWhiteSpace(path))
            return null;

        try
        {
            string fullPath = Path.GetFullPath(path);
            string? dir = Path.GetDirectoryName(fullPath);
            if (!string.IsNullOrWhiteSpace(dir))
                Directory.CreateDirectory(dir);

            File.AppendAllText(fullPath, string.Empty);
            return fullPath;
        }
        catch
        {
            return null;
        }
    }

    // Disabled timing never evaluates interpolated detail expressions at the call site.
    [InterpolatedStringHandler]
    public ref struct TimingDetailsHandler
    {
        private DefaultInterpolatedStringHandler _builder;
        internal bool Enabled;

        public TimingDetailsHandler(int literalLength, int formattedCount, long startTimestamp, out bool shouldAppend)
        {
            Enabled = shouldAppend = startTimestamp != 0L && IsEnabled;
            _builder = Enabled ? new DefaultInterpolatedStringHandler(literalLength, formattedCount) : default;
        }

        public void AppendLiteral(string value) => _builder.AppendLiteral(value);
        public void AppendFormatted<T>(T value) => _builder.AppendFormatted(value);
        public void AppendFormatted<T>(T value, string? format) => _builder.AppendFormatted(value, format);
        public void AppendFormatted<T>(T value, int alignment, string? format = null) => _builder.AppendFormatted(value, alignment, format);
        internal string GetText() => _builder.ToStringAndClear();
    }
}
