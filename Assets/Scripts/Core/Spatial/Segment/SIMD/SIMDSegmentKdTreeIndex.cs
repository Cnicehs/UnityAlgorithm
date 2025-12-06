using System;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using System.Collections.Generic;

[BurstCompile]
public unsafe class SIMDSegmentKdTreeIndex : ISIMDSegmentSpatialIndex
{
    private NativeArray<SIMDRVO.ObstacleData> _treeObstacles;

    public NativeArray<SIMDRVO.ObstacleData> GetObstacles() => _treeObstacles;

    public void Build(NativeArray<SIMDRVO.ObstacleData> obstacles)
    {
        if (_treeObstacles.IsCreated) _treeObstacles.Dispose();
        
        // Just copy for now, effectively degrading to Linear until a robust Tree is reimplemented
        _treeObstacles = new NativeArray<SIMDRVO.ObstacleData>(obstacles.Length, Allocator.Persistent);
        NativeArray<SIMDRVO.ObstacleData>.Copy(obstacles, _treeObstacles, obstacles.Length);
    }

    public void QueryNearest(Unity.Mathematics.float2 position, float range, List<int> results)
    {
        if (!_treeObstacles.IsCreated) return;

        float rangeSq = range * range;
        for (int i = 0; i < _treeObstacles.Length; i++)
        {
            float distSq = SIMDRVO.distSqPointLineSegment(_treeObstacles[i].Point1, _treeObstacles[i].Point2, position);
            if (distSq < rangeSq)
            {
                results.Add(i);
            }
        }

        results.Sort((a, b) =>
        {
            float distA = SIMDRVO.distSqPointLineSegment(_treeObstacles[a].Point1, _treeObstacles[a].Point2, position);
            float distB = SIMDRVO.distSqPointLineSegment(_treeObstacles[b].Point1, _treeObstacles[b].Point2, position);
            return distA.CompareTo(distB);
        });
    }

    public void Dispose()
    {
        if (_treeObstacles.IsCreated) _treeObstacles.Dispose();
    }
}
