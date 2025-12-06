using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;

/// <summary>
/// Simple linear scan spatial index for line segments.
/// O(N) query time, useful for small obstacle counts or as a baseline.
/// </summary>
public class SegmentLinearIndex : ISegmentSpatialIndex
{
    private List<RVOObstacle> _obstacles;

    public void Build(List<RVOObstacle> obstacles)
    {
        // Reference copy is fine as we don't modify obstacles, only sort a query result
        _obstacles = obstacles;
    }

    public void QueryNearest(Vector2 position, List<RVOObstacle> results)
    {
        if (_obstacles == null) return;

        // Copy all (Naive implementation)
        results.Clear();
        results.AddRange(_obstacles);

        // Sort by distance to position
        float2 pos = new float2(position.x, position.y);
        results.Sort((a, b) =>
        {
            float distA = RVOMath.distSqPointLineSegment(a.Point1, a.Point2, pos);
            float distB = RVOMath.distSqPointLineSegment(b.Point1, b.Point2, pos);
            return distA.CompareTo(distB);
        });
    }
}
