using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;
using UnityEngine;

[UpdateInGroup(SystemGroup.FixedUpdate)]
public class RVOSimulationSystem : ISystem
{
    private NativeArray<int> _neighborIndices;
    private NativeArray<int> _neighborCounts;
    private NativeArray<int> _neighborOffsets;
    private NativeArray<float2> _newVelocities;
    private NativeArray<SIMDRVO.ObstacleData> _nativeObstacles;
    private SIMDSegmentKdTreeIndex _segmentSpatialIndex;
    
    private int _maxNeighbors = 10;
    private int _capacity = 0;

    public void Initialize()
    {
        _segmentSpatialIndex = new SIMDSegmentKdTreeIndex();
    }

    public void Shutdown()
    {
        if (_neighborIndices.IsCreated) _neighborIndices.Dispose();
        if (_neighborCounts.IsCreated) _neighborCounts.Dispose();
        if (_neighborOffsets.IsCreated) _neighborOffsets.Dispose();
        if (_newVelocities.IsCreated) _newVelocities.Dispose();
        if (_nativeObstacles.IsCreated) _nativeObstacles.Dispose();
        _segmentSpatialIndex.Dispose();
    }

    public unsafe void Update(float dt)
    {
        int agentCount = EntityManager.Instance.Count;
        if (agentCount == 0) return;

        EnsureCapacity(agentCount);

        // 1. Update Spatial Index (Positions) if needed?
        // Actually SpatialIndexManager handles unit positions for neighbor queries.
        // We assume SpatialIndexManager is updated by PresentationSystem or manually.
        // But wait, pathfinding updates positions too?
        // Let's assume SpatialIndexManager is up-to-date with CURRENT positions.

        // 2. Neighbor Query
        var spatialIndex = SpatialIndexManager.Instance.GetCurrentIndex();
        
        // Prepare pointers for EntityManager data
        PositionComponent* positionsPtr = (PositionComponent*)EntityManager.Instance.GetArray<PositionComponent>().AsNativeArray().GetUnsafePtr();
        VelocityComponent* velocitiesPtr = (VelocityComponent*)EntityManager.Instance.GetArray<VelocityComponent>().AsNativeArray().GetUnsafePtr();
        RadiusComponent* radiiPtr = (RadiusComponent*)EntityManager.Instance.GetArray<RadiusComponent>().AsNativeArray().GetUnsafePtr();
        AgentParameters* agentParamsPtr = (AgentParameters*)EntityManager.Instance.GetArray<AgentParameters>().AsNativeArray().GetUnsafePtr();
        MovementState* movementStatesPtr = (MovementState*)EntityManager.Instance.GetArray<MovementState>().AsNativeArray().GetUnsafePtr();
        
        // Extract PrefVelocities from MovementState
        NativeArray<float2> prefVelocities = new NativeArray<float2>(agentCount, Allocator.TempJob);
        for(int i=0; i<agentCount; ++i) prefVelocities[i] = movementStatesPtr[i].PreferredVelocity;

        // Extract pointers to raw data for SIMDRVO (which expects float2*, float* etc)
        // Since PositionComponent is just struct { float2 Value; }, we can cast PositionComponent* to float2* ?
        // Yes, if layout is sequential and no padding. struct PositionComponent { float2 Value; } is same size/layout as float2.
        // But we need to be careful.
        // Let's use stride or just cast.
        // Burst allows casting pointer types.
        
        float2* rawPosPtr = (float2*)positionsPtr;
        float2* rawVelPtr = (float2*)velocitiesPtr;
        float* rawRadiiPtr = (float*)radiiPtr;

        // Perform Neighbor Query
        if (spatialIndex is SIMDQuadTreeIndex quadTree)
        {
            // REFACTOR: Use a parallel job for neighbor query if possible.
            // For MVP of this System, let's use a simple loop.
            
            for (int i = 0; i < agentCount; i++)
            {
                float2 pos = rawPosPtr[i];
                float dist = agentParamsPtr[i].NeighborDist;
                int maxN = agentParamsPtr[i].MaxNeighbors;
                
                // Use temp list
                System.Collections.Generic.List<int> neighbors = new System.Collections.Generic.List<int>(); 
                SpatialIndexManager.Instance.GetNeighborsInRadius(new Vector2(pos.x, pos.y), dist, neighbors);
                
                // Sort
                neighbors.Sort((a, b) =>
                {
                    float d1 = math.distancesq(pos, rawPosPtr[a]);
                    float d2 = math.distancesq(pos, rawPosPtr[b]);
                    return d1.CompareTo(d2);
                });
                
                int count = 0;
                int offset = i * _maxNeighbors;
                
                for(int j=0; j<neighbors.Count && count < maxN; ++j)
                {
                    int idx = neighbors[j];
                    if (idx != i)
                    {
                        _neighborIndices[offset + count] = idx;
                        count++;
                    }
                }
                
                _neighborCounts[i] = count;
                _neighborOffsets[i] = offset;
            }
        }
        else
        {
             // Fallback same as above
             for (int i = 0; i < agentCount; i++)
            {
                float2 pos = rawPosPtr[i];
                float dist = agentParamsPtr[i].NeighborDist;
                int maxN = agentParamsPtr[i].MaxNeighbors;
                
                System.Collections.Generic.List<int> neighbors = new System.Collections.Generic.List<int>(); 
                SpatialIndexManager.Instance.GetNeighborsInRadius(new Vector2(pos.x, pos.y), dist, neighbors);
                
                neighbors.Sort((a, b) =>
                {
                    float d1 = math.distancesq(pos, rawPosPtr[a]);
                    float d2 = math.distancesq(pos, rawPosPtr[b]);
                    return d1.CompareTo(d2);
                });
                
                int count = 0;
                int offset = i * _maxNeighbors;
                
                for(int j=0; j<neighbors.Count && count < maxN; ++j)
                {
                    int idx = neighbors[j];
                    if (idx != i)
                    {
                        _neighborIndices[offset + count] = idx;
                        count++;
                    }
                }
                
                _neighborCounts[i] = count;
                _neighborOffsets[i] = offset;
            }
        }

        // 3. Compute RVO
        SIMDRVO.ObstacleData* obstPtr = null;
        int obstCount = 0;
        SIMDRVO.ObstacleTreeNode* nodePtr = null;
        int rootIdx = -1;

        if (_nativeObstacles.IsCreated)
        {
             var treeObs = _segmentSpatialIndex.GetTreeObstacles();
             var treeNodes = _segmentSpatialIndex.GetTreeNodes();
             
             if (treeObs.IsCreated && treeNodes.IsCreated)
             {
                 obstPtr = (SIMDRVO.ObstacleData*)treeObs.GetUnsafePtr();
                 obstCount = _segmentSpatialIndex.GetTreeObstacleCount();
                 nodePtr = (SIMDRVO.ObstacleTreeNode*)treeNodes.GetUnsafePtr();
                 rootIdx = _segmentSpatialIndex.GetRootNodeIndex();
             }
             else
             {
                 obstPtr = (SIMDRVO.ObstacleData*)_nativeObstacles.GetUnsafePtr();
                 obstCount = _nativeObstacles.Length;
             }
        }

        SIMDRVO.ComputeRVOVelocities(
            rawPosPtr,
            rawVelPtr,
            rawRadiiPtr,
            agentParamsPtr,
            (float2*)prefVelocities.GetUnsafePtr(),
            (int*)_neighborIndices.GetUnsafePtr(),
            (int*)_neighborCounts.GetUnsafePtr(),
            (int*)_neighborOffsets.GetUnsafePtr(),
            obstPtr,
            obstCount,
            nodePtr,
            rootIdx,
            (float2*)_newVelocities.GetUnsafePtr(),
            agentCount,
            dt
        );

        // 4. Update Velocities and Positions
        for (int i = 0; i < agentCount; i++)
        {
            float2 v = _newVelocities[i];
            
            // Set Velocity
            VelocityComponent velComp = velocitiesPtr[i];
            velComp.Value = v;
            velocitiesPtr[i] = velComp;
            
            // Update Position
            PositionComponent posComp = positionsPtr[i];
            posComp.Value += v * dt;
            positionsPtr[i] = posComp;
        }

        prefVelocities.Dispose();
        
        // Sync Spatial Index (Optimization: Batch update)
        System.Collections.Generic.List<Vector2> posList = new System.Collections.Generic.List<Vector2>(agentCount);
        for(int i=0; i<agentCount; ++i)
        {
            float2 p = positionsPtr[i].Value;
            posList.Add(new Vector2(p.x, p.y));
        }
        SpatialIndexManager.Instance.UpdatePositions(posList);
    }

    private void EnsureCapacity(int count)
    {
        if (count > _capacity)
        {
            _capacity = math.max(count, _capacity * 2);
            if (_capacity < 128) _capacity = 128;

            if (_neighborIndices.IsCreated) _neighborIndices.Dispose();
            if (_neighborCounts.IsCreated) _neighborCounts.Dispose();
            if (_neighborOffsets.IsCreated) _neighborOffsets.Dispose();
            if (_newVelocities.IsCreated) _newVelocities.Dispose();

            _neighborIndices = new NativeArray<int>(_capacity * _maxNeighbors, Allocator.Persistent);
            _neighborCounts = new NativeArray<int>(_capacity, Allocator.Persistent);
            _neighborOffsets = new NativeArray<int>(_capacity, Allocator.Persistent);
            _newVelocities = new NativeArray<float2>(_capacity, Allocator.Persistent);
        }
    }

    // Call this to update obstacles from main thread
    public void UpdateObstacles(System.Collections.Generic.List<RVOObstacle> obstacles)
    {
        if (_nativeObstacles.IsCreated) _nativeObstacles.Dispose();
        
        // Convert
        _nativeObstacles = new NativeArray<SIMDRVO.ObstacleData>(obstacles.Count, Allocator.Persistent);
        
        for (int i = 0; i < obstacles.Count; i++)
        {
            var src = obstacles[i];
            var dest = new SIMDRVO.ObstacleData
            {
                Point1 = src.Point1,
                Point2 = src.Point2,
                Direction = src.Direction,
                IsConvex = src.IsConvex,
                NextObstacleIdx = -1,
                PrevObstacleIdx = -1
            };
            if (src.NextObstacle != null) dest.NextObstacleIdx = obstacles.IndexOf(src.NextObstacle);
            if (src.PrevObstacle != null) dest.PrevObstacleIdx = obstacles.IndexOf(src.PrevObstacle);
            _nativeObstacles[i] = dest;
        }

        _segmentSpatialIndex.Build(_nativeObstacles);
    }
}
