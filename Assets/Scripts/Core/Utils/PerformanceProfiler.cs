using System;
using System.Diagnostics;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Performance profiler with conditional compilation support.
/// All profiling code is removed in Release builds.
/// Usage: 
///   using (ProfilerScope.Begin("MyOperation")) { ... }
///   or
///   ProfilerScope.BeginSample("MyOperation");
///   // ... code ...
///   ProfilerScope.EndSample();
/// </summary>
public static class PerformanceProfiler
{
    private static readonly Dictionary<string, ProfileData> _profiles = new Dictionary<string, ProfileData>();
    private static readonly Stack<SampleScope> _scopeStack = new Stack<SampleScope>();
    
    private class ProfileData
    {
        public double TotalMs;
        public int SampleCount;
        public double LastMs;
        public double MinMs = double.MaxValue;
        public double MaxMs;
        
        public double AverageMs => SampleCount > 0 ? TotalMs / SampleCount : 0;
    }
    
    private struct SampleScope
    {
        public string Name;
        public Stopwatch Stopwatch;
    }

    [Conditional("ENABLE_PROFILING")]
    public static void BeginSample(string name)
    {
        var scope = new SampleScope
        {
            Name = name,
            Stopwatch = Stopwatch.StartNew()
        };
        _scopeStack.Push(scope);
    }

    [Conditional("ENABLE_PROFILING")]
    public static void EndSample()
    {
        if (_scopeStack.Count == 0)
        {
            UnityEngine.Debug.LogWarning("PerformanceProfiler: EndSample called without matching BeginSample");
            return;
        }

        var scope = _scopeStack.Pop();
        scope.Stopwatch.Stop();
        
        double elapsedMs = scope.Stopwatch.Elapsed.TotalMilliseconds;
        
        if (!_profiles.TryGetValue(scope.Name, out var data))
        {
            data = new ProfileData();
            _profiles[scope.Name] = data;
        }
        
        data.TotalMs += elapsedMs;
        data.SampleCount++;
        data.LastMs = elapsedMs;
        data.MinMs = Math.Min(data.MinMs, elapsedMs);
        data.MaxMs = Math.Max(data.MaxMs, elapsedMs);
    }

    [Conditional("ENABLE_PROFILING")]
    public static void Reset()
    {
        _profiles.Clear();
        _scopeStack.Clear();
    }

    [Conditional("ENABLE_PROFILING")]
    public static void Reset(string name)
    {
        _profiles.Remove(name);
    }

    public static double GetLastMs(string name)
    {
#if ENABLE_PROFILING
        return _profiles.TryGetValue(name, out var data) ? data.LastMs : 0;
#else
        return 0;
#endif
    }

    public static double GetAverageMs(string name)
    {
#if ENABLE_PROFILING
        return _profiles.TryGetValue(name, out var data) ? data.AverageMs : 0;
#else
        return 0;
#endif
    }

    public static double GetMinMs(string name)
    {
#if ENABLE_PROFILING
        return _profiles.TryGetValue(name, out var data) ? data.MinMs : 0;
#else
        return 0;
#endif
    }

    public static double GetMaxMs(string name)
    {
#if ENABLE_PROFILING
        return _profiles.TryGetValue(name, out var data) ? data.MaxMs : 0;
#else
        return 0;
#endif
    }

    public static int GetSampleCount(string name)
    {
#if ENABLE_PROFILING
        return _profiles.TryGetValue(name, out var data) ? data.SampleCount : 0;
#else
        return 0;
#endif
    }

    [Conditional("ENABLE_PROFILING")]
    public static void LogResults()
    {
        if (_profiles.Count == 0)
        {
            UnityEngine.Debug.Log("PerformanceProfiler: No samples recorded");
            return;
        }

        UnityEngine.Debug.Log("=== Performance Profiler Results ===");
        foreach (var kvp in _profiles)
        {
            var data = kvp.Value;
            UnityEngine.Debug.Log($"{kvp.Key}: Last={data.LastMs:F3}ms, Avg={data.AverageMs:F3}ms, Min={data.MinMs:F3}ms, Max={data.MaxMs:F3}ms, Samples={data.SampleCount}");
        }
    }

    /// <summary>
    /// RAII-style profiling scope. Use with 'using' statement.
    /// </summary>
    public struct ProfilerScope : IDisposable
    {
        private readonly string _name;
        private readonly bool _enabled;

        private ProfilerScope(string name, bool enabled)
        {
            _name = name;
            _enabled = enabled;
#if ENABLE_PROFILING
            if (_enabled)
            {
                BeginSample(name);
            }
#endif
        }

        public static ProfilerScope Begin(string name)
        {
#if ENABLE_PROFILING
            return new ProfilerScope(name, true);
#else
            return new ProfilerScope(name, false);
#endif
        }

        public void Dispose()
        {
#if ENABLE_PROFILING
            if (_enabled)
            {
                EndSample();
            }
#endif
        }
    }
}
