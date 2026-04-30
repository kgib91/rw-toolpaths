using System.Diagnostics;
using System.Globalization;

namespace RW.Toolpaths;

public static class PerfLog
{
    private static readonly bool TimingEnabled = ResolveEnabled();
    private static readonly string? TimingFilePath = ResolveFilePath();
    private static readonly object FileLock = new();

    public static bool IsEnabled => TimingEnabled;

    public static long Start() => TimingEnabled ? Stopwatch.GetTimestamp() : 0L;

    public static void Stop(string area, long startTimestamp, string? details = null)
    {
        if (!TimingEnabled || startTimestamp == 0L)
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

        if (!string.IsNullOrWhiteSpace(TimingFilePath))
        {
            lock (FileLock)
            {
                File.AppendAllText(TimingFilePath!, message + Environment.NewLine);
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

#if DEBUG
        return true;
#else
        return false;
#endif
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
}
