using Unity.Collections;
using Unity.Mathematics;
using System.Collections.Generic;

public class SIMDLinearSegmentIndex : ISIMDSegmentSpatialIndex
{
    private NativeArray<SIMDRVO.ObstacleData> _obstacles;

    public void Build(NativeArray<SIMDRVO.ObstacleData> obstacles)
    {
        if (_obstacles.IsCreated) _obstacles.Dispose();
        _obstacles = new NativeArray<SIMDRVO.ObstacleData>(obstacles.Length, Allocator.Persistent);
        NativeArray<SIMDRVO.ObstacleData>.Copy(obstacles, _obstacles, obstacles.Length);
    }

    public NativeArray<SIMDRVO.ObstacleData> GetObstacles()
    {
        return _obstacles;
    }

    public void QueryNearest(float2 position, float range, List<int> results)
    {
        if (!_obstacles.IsCreated) return;

        float rangeSq = range * range;
        for (int i = 0; i < _obstacles.Length; i++)
        {
            float distSq = SIMDRVO.distSqPointLineSegment(_obstacles[i].Point1, _obstacles[i].Point2, position);
            if (distSq < rangeSq)
            {
                results.Add(i);
            }
        }

        results.Sort((a, b) =>
        {
            float distA = SIMDRVO.distSqPointLineSegment(_obstacles[a].Point1, _obstacles[a].Point2, position);
            float distB = SIMDRVO.distSqPointLineSegment(_obstacles[b].Point1, _obstacles[b].Point2, position);
            return distA.CompareTo(distB);
        });
    }

    public void Dispose()
    {
        if (_obstacles.IsCreated) _obstacles.Dispose();
    }
}
