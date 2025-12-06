using System.Collections.Generic;
using UnityEngine;
using System.Diagnostics;
using Debug = UnityEngine.Debug;

public class SIMDBenchmark : MonoBehaviour
{
    public int UnitCount = 100000;
    public int Iterations = 100;
    public float Radius = 10f;

    private List<Vector2> _points;
    private Vector2 _target;
    private List<int> _resultsBuffer = new List<int>();

    void Start()
    {
        _points = new List<Vector2>(UnitCount);
        for (int i = 0; i < UnitCount; i++)
        {
            _points.Add(new Vector2(Random.Range(-100f, 100f), Random.Range(-100f, 100f)));
        }
        _target = Vector2.zero;

        Debug.Log($"<color=yellow>Starting SIMD Benchmark with {UnitCount} units, {Iterations} iterations.</color>");
        
        RunBenchmarks();
    }

    [ContextMenu("Run Benchmarks")]
    public void RunBenchmarks()
    {
        BenchmarkFindNearest();
        BenchmarkFindInRadius();
        BenchmarkCalculateBounds();
        BenchmarkSum();
        BenchmarkTranslate();
        BenchmarkBatchDot();
    }

    private void BenchmarkSum()
    {
        // Warmup
        SIMDAlgorithms.SumLegacy(_points);
        SIMDAlgorithms.SumBurst(_points);

        Stopwatch sw = new Stopwatch();

        sw.Restart();
        for (int i = 0; i < Iterations; i++) SIMDAlgorithms.SumLegacy(_points);
        sw.Stop();
        double legacyTime = sw.Elapsed.TotalMilliseconds;

        sw.Restart();
        for (int i = 0; i < Iterations; i++) SIMDAlgorithms.SumBurst(_points);
        sw.Stop();
        double burstTime = sw.Elapsed.TotalMilliseconds;

        Debug.Log($"<b>Sum</b>: Legacy: {legacyTime:F2}ms | Burst: {burstTime:F2}ms | Speedup: <color=green>{legacyTime/burstTime:F2}x</color>");
    }

    private void BenchmarkTranslate()
    {
        // Clone points for fair comparison since Translate modifies in-place
        List<Vector2> pointsLegacy = new List<Vector2>(_points);
        List<Vector2> pointsBurst = new List<Vector2>(_points);
        Vector2 translation = new Vector2(1, 1);

        // Warmup
        SIMDAlgorithms.TranslateLegacy(pointsLegacy, translation);
        SIMDAlgorithms.TranslateBurst(pointsBurst, translation);

        // Reset
        pointsLegacy = new List<Vector2>(_points);
        pointsBurst = new List<Vector2>(_points);

        Stopwatch sw = new Stopwatch();

        sw.Restart();
        for (int i = 0; i < Iterations; i++) 
        {
            // We should ideally reset points every time, but that dominates benchmark.
            // Just translate back and forth? Or just translate.
            // Translating 100 times is fine.
            SIMDAlgorithms.TranslateLegacy(pointsLegacy, translation);
        }
        sw.Stop();
        double legacyTime = sw.Elapsed.TotalMilliseconds;

        sw.Restart();
        for (int i = 0; i < Iterations; i++) 
        {
            SIMDAlgorithms.TranslateBurst(pointsBurst, translation);
        }
        sw.Stop();
        double burstTime = sw.Elapsed.TotalMilliseconds;

        Debug.Log($"<b>Translate</b>: Legacy: {legacyTime:F2}ms | Burst: {burstTime:F2}ms | Speedup: <color=green>{legacyTime/burstTime:F2}x</color>");
    }

    private void BenchmarkBatchDot()
    {
        List<float> results = new List<float>(_points.Count);
        Vector2 target = new Vector2(1, 0);

        // Warmup
        SIMDAlgorithms.BatchDotLegacy(_points, target, results);
        SIMDAlgorithms.BatchDotBurst(_points, target, results);

        Stopwatch sw = new Stopwatch();

        sw.Restart();
        for (int i = 0; i < Iterations; i++) SIMDAlgorithms.BatchDotLegacy(_points, target, results);
        sw.Stop();
        double legacyTime = sw.Elapsed.TotalMilliseconds;

        sw.Restart();
        for (int i = 0; i < Iterations; i++) SIMDAlgorithms.BatchDotBurst(_points, target, results);
        sw.Stop();
        double burstTime = sw.Elapsed.TotalMilliseconds;

        Debug.Log($"<b>BatchDot</b>: Legacy: {legacyTime:F2}ms | Burst: {burstTime:F2}ms | Speedup: <color=green>{legacyTime/burstTime:F2}x</color>");
    }
    private void BenchmarkFindNearest()
    {
        // Warmup
        SIMDAlgorithms.FindNearestLegacy(_points, _target);
        SIMDAlgorithms.FindNearestBurst(_points, _target);
        SIMDAlgorithms.FindNearestBurstSerial(_points, _target);

        Stopwatch sw = new Stopwatch();

        sw.Restart();
        for (int i = 0; i < Iterations; i++)
        {
            SIMDAlgorithms.FindNearestLegacy(_points, _target);
        }
        sw.Stop();
        double legacyTime = sw.Elapsed.TotalMilliseconds;

        sw.Restart();
        for (int i = 0; i < Iterations; i++)
        {
            SIMDAlgorithms.FindNearestBurst(_points, _target);
        }
        sw.Stop();
        double burstTime = sw.Elapsed.TotalMilliseconds;

        sw.Restart();
        for (int i = 0; i < Iterations; i++)
        {
            SIMDAlgorithms.FindNearestBurstSerial(_points, _target);
        }
        sw.Stop();
        double serialTime = sw.Elapsed.TotalMilliseconds;

        Debug.Log($"<b>FindNearest</b>: Legacy: {legacyTime:F2}ms | Burst: {burstTime:F2}ms | Serial: {serialTime:F2}ms | Speedup: <color=green>{legacyTime/burstTime:F2}x</color> (Serial: {legacyTime/serialTime:F2}x)");
    }

    private void BenchmarkFindInRadius()
    {
        // Warmup
        SIMDAlgorithms.FindInRadiusLegacy(_points, _target, Radius, _resultsBuffer);
        SIMDAlgorithms.FindInRadiusBurst(_points, _target, Radius, _resultsBuffer);

        Stopwatch sw = new Stopwatch();

        sw.Restart();
        for (int i = 0; i < Iterations; i++)
        {
            SIMDAlgorithms.FindInRadiusLegacy(_points, _target, Radius, _resultsBuffer);
        }
        sw.Stop();
        double legacyTime = sw.Elapsed.TotalMilliseconds;

        sw.Restart();
        for (int i = 0; i < Iterations; i++)
        {
            SIMDAlgorithms.FindInRadiusBurst(_points, _target, Radius, _resultsBuffer);
        }
        sw.Stop();
        double burstTime = sw.Elapsed.TotalMilliseconds;

        Debug.Log($"<b>FindInRadius</b>: Legacy: {legacyTime:F2}ms | Burst: {burstTime:F2}ms | Speedup: <color=green>{legacyTime/burstTime:F2}x</color>");
    }

    private void BenchmarkCalculateBounds()
    {
        // Warmup
        SIMDAlgorithms.CalculateBoundsLegacy(_points);
        SIMDAlgorithms.CalculateBoundsBurst(_points);

        Stopwatch sw = new Stopwatch();

        sw.Restart();
        for (int i = 0; i < Iterations; i++)
        {
            SIMDAlgorithms.CalculateBoundsLegacy(_points);
        }
        sw.Stop();
        double legacyTime = sw.Elapsed.TotalMilliseconds;

        sw.Restart();
        for (int i = 0; i < Iterations; i++)
        {
            SIMDAlgorithms.CalculateBoundsBurst(_points);
        }
        sw.Stop();
        double burstTime = sw.Elapsed.TotalMilliseconds;

        Debug.Log($"<b>CalculateBounds</b>: Legacy: {legacyTime:F2}ms | Burst: {burstTime:F2}ms | Speedup: <color=green>{legacyTime/burstTime:F2}x</color>");
    }
}
