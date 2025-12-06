using Unity.Collections;
using Unity.Mathematics;

public interface ISIMDSegmentSpatialIndex
{
    void Build(NativeArray<SIMDRVO.ObstacleData> obstacles);
    void Dispose();
}
