using UnityEngine;
using System.Diagnostics;
using System.Collections;

public abstract class ComparisonDemoBase : MonoBehaviour
{
    [Header("Base Settings")]
    public bool UseSIMD = true;
    public int AgentCount = 0; // Updated by subclasses

    // Performance Metrics
    protected float ExecutionTimeMs;
    protected Stopwatch Stopwatch = new Stopwatch();

    protected virtual void OnGUI()
    {
        GUILayout.BeginArea(new Rect(10, 10, 300, 400));
        GUILayout.Box("Performance Comparison");

        bool newSIMD = GUILayout.Toggle(UseSIMD, "Use SIMD Optimization");
        if (newSIMD != UseSIMD)
        {
            UseSIMD = newSIMD;
            OnSIMDChanged(UseSIMD);
        }

        GUILayout.Space(10);
        GUILayout.Label($"Agent Count: {AgentCount}");
        GUILayout.Label($"Execution Time: {ExecutionTimeMs:F4} ms");
        GUILayout.Label($"FPS: {1.0f / Time.smoothDeltaTime:F1}");

        GUILayout.Space(10);
        OnGUIDisplayExtra();

        GUILayout.EndArea();
    }

    protected virtual void OnSIMDChanged(bool useSIMD)
    {
        // To be implemented by subclasses if they need to switch systems
    }

    protected virtual void OnGUIDisplayExtra()
    {
        // To be implemented by subclasses
    }
}
