using Unity.Collections;
using Unity.Mathematics;
using System.Collections.Generic;

public interface ISIMDSegmentSpatialIndex
{
    void Build(NativeArray<SIMDRVO.ObstacleData> obstacles);
    NativeArray<SIMDRVO.ObstacleData> GetObstacles();
    void QueryNearest(float2 position, float range, List<int> results);
    void Dispose();
}
