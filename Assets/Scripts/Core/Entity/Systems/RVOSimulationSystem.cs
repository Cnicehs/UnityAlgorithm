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
    
    // Aligned buffers for SIMD RVO
    private NativeArray<float2> _alignedPositions;
    private NativeArray<float2> _alignedVelocities;
    private NativeArray<float> _alignedRadii;
    private NativeArray<AgentParameters> _alignedParams;
    private NativeArray<float2> _alignedPrefVelocities;
    private NativeArray<int> _alignedEntityIds; // To scatter back
    
    private System.Collections.Generic.Dictionary<int, int> _entityToAlignedMap = new System.Collections.Generic.Dictionary<int, int>();

    private int _maxNeighbors = 10;
    private int _capacity = 0;

    public void Initialize()
    {
        _segmentSpatialIndex = new SIMDSegmentKdTreeIndex();
    }

    public void Shutdown()
    {
        DisposeArrays();
        _segmentSpatialIndex.Dispose();
    }

    private void DisposeArrays()
    {
        if (_neighborIndices.IsCreated) _neighborIndices.Dispose();
        if (_neighborCounts.IsCreated) _neighborCounts.Dispose();
        if (_neighborOffsets.IsCreated) _neighborOffsets.Dispose();
        if (_newVelocities.IsCreated) _newVelocities.Dispose();
        if (_nativeObstacles.IsCreated) _nativeObstacles.Dispose();
        
        if (_alignedPositions.IsCreated) _alignedPositions.Dispose();
        if (_alignedVelocities.IsCreated) _alignedVelocities.Dispose();
        if (_alignedRadii.IsCreated) _alignedRadii.Dispose();
        if (_alignedParams.IsCreated) _alignedParams.Dispose();
        if (_alignedPrefVelocities.IsCreated) _alignedPrefVelocities.Dispose();
        if (_alignedEntityIds.IsCreated) _alignedEntityIds.Dispose();
    }

    public unsafe void Update(float dt)
    {
        var entityManager = EntityManager.Instance;
        var agentParamsArray = entityManager.GetArray<AgentParameters>();
        
        int agentCount = agentParamsArray.Count;
        if (agentCount == 0) return;

        EnsureCapacity(agentCount);

        // Gather Phase: Collect data into aligned arrays
        // We iterate AgentParameters as the source of truth for "Active Agents"
        var positions = entityManager.GetArray<PositionComponent>();
        var velocities = entityManager.GetArray<VelocityComponent>();
        var radii = entityManager.GetArray<RadiusComponent>();
        var movementStates = entityManager.GetArray<MovementState>();

        int validCount = 0;
        for (int i = 0; i < agentCount; i++)
        {
            // AgentParameters is dense, but we need corresponding other components
            int entityId = agentParamsArray.GetEntityIdFromDenseIndex(i);
            
            // Check if entity has all required components
            if (!positions.Has(entityId) || !velocities.Has(entityId) || 
                !radii.Has(entityId) || !movementStates.Has(entityId))
            {
                continue;
            }

            // Copy to aligned buffers
            _alignedPositions[validCount] = positions.GetRef(entityId).Value;
            _alignedVelocities[validCount] = velocities.GetRef(entityId).Value;
            _alignedRadii[validCount] = radii.GetRef(entityId).Value;
            _alignedParams[validCount] = agentParamsArray.GetDenseRef(i);
            _alignedPrefVelocities[validCount] = movementStates.GetRef(entityId).PreferredVelocity;
            _alignedEntityIds[validCount] = entityId;
            
            validCount++;
        }

        if (validCount == 0) return;

        // 2. Neighbor Query
        // Note: SpatialIndexManager likely uses old positions or needs update.
        // Assuming SpatialIndexManager is updated externally or we update it here?
        // The original code updated it at the end.
        // Let's perform neighbor query using current positions (alignedPositions)
        
        // Update Spatial Index with CURRENT positions for accurate query
        // But SpatialIndexManager takes a list of Vector2... expensive conversion.
        // Let's assume SpatialIndexManager was updated last frame or by another system.
        // Actually, original code updated it at the END of Update.
        
        // Fallback: Using brute force or existing index?
        // Let's use the Gathered positions for distance checks (which is what matters for sorting)
        // But GetNeighborsInRadius uses the SpatialIndexManager's internal structure.
        
        for (int i = 0; i < validCount; i++)
        {
            float2 pos = _alignedPositions[i];
            float dist = _alignedParams[i].NeighborDist;
            int maxN = _alignedParams[i].MaxNeighbors;
            
            System.Collections.Generic.List<int> neighbors = new System.Collections.Generic.List<int>(); 
            SpatialIndexManager.Instance.GetNeighborsInRadius(new Vector2(pos.x, pos.y), dist, neighbors);
            
            // Sort neighbors by distance to current agent
            // Note: 'neighbors' contains IDs from SpatialIndexManager. 
            // Are those Entity IDs? Yes, likely.
            // But our aligned array is indexed 0..validCount.
            // We need to map EntityID -> AlignedIndex for RVO logic?
            // RVO logic needs neighbor's Position and Velocity.
            // If neighbor 'K' (EntityID) is found, we need its current Position/Velocity.
            // But we only have them in _alignedXXX arrays or via EntityManager lookup.
            // Lookup via EntityManager is slow inside inner loop.
            // Lookup via _alignedXXX requires searching _alignedEntityIds? Slow.
            
            // CRITICAL ISSUE:
            // RVO requires random access to neighbor data.
            // If we use Gather/Scatter, the indices passed to RVO must be indices into the Gathered arrays (0..validCount).
            // But SpatialIndexManager returns Entity IDs (or whatever it stores).
            // If SpatialIndexManager stores Entity IDs, we need to map EntityID -> AlignedIndex.
            
            // Solution: 
            // 1. Build a map EntityID -> AlignedIndex during Gather.
            //    We can use a NativeHashMap<int, int> map.
            
            // 2. Or pass pointers to ComponentArrays to RVO?
            //    But they are unaligned (SparseSet).
            //    Can SIMDRVO handle indirect access? No, it takes float2* posPtr.
            //    It expects posPtr[neighborIdx] to be valid.
            
            // Backtrack:
            // If we cannot guarantee alignment, and RVO requires dense indices 0..N,
            // Then we MUST have a mapping.
            // But SpatialIndex returns EntityIDs.
            // If we change SpatialIndex to store AlignedIndices, we need to rebuild it every frame.
            // Rebuilding Spatial Index every frame is common for dynamic agents.
            
            // Let's assume we rebuild Spatial Index here using _alignedPositions?
            // The original code used SpatialIndexManager which might be persistent.
            
            // Alternative:
            // Map EntityID -> AlignedIndex.
        }
        
        // Optimization:
        // Use a lookup map.
        // Since NativeHashMap might not be available, we use a pooled Dictionary.
        _entityToAlignedMap.Clear();
        for(int i=0; i<validCount; ++i) _entityToAlignedMap[_alignedEntityIds[i]] = i;

        for (int i = 0; i < validCount; i++)
        {
            float2 pos = _alignedPositions[i];
            float dist = _alignedParams[i].NeighborDist;
            int maxN = _alignedParams[i].MaxNeighbors;
            
            System.Collections.Generic.List<int> neighbors = new System.Collections.Generic.List<int>(); 
            SpatialIndexManager.Instance.GetNeighborsInRadius(new Vector2(pos.x, pos.y), dist, neighbors);
            
            // Filter and Map neighbors to Aligned Indices
            var validNeighbors = new System.Collections.Generic.List<int>();
            for(int k=0; k<neighbors.Count; ++k)
            {
                if (_entityToAlignedMap.TryGetValue(neighbors[k], out int alignedIdx))
                {
                    validNeighbors.Add(alignedIdx);
                }
            }
            
            validNeighbors.Sort((a, b) =>
            {
                float d1 = math.distancesq(pos, _alignedPositions[a]);
                float d2 = math.distancesq(pos, _alignedPositions[b]);
                return d1.CompareTo(d2);
            });
            
            int count = 0;
            int offset = i * _maxNeighbors;
            
            for(int j=0; j<validNeighbors.Count && count < maxN; ++j)
            {
                int idx = validNeighbors[j];
                if (idx != i)
                {
                    _neighborIndices[offset + count] = idx;
                    count++;
                }
            }
            
            _neighborCounts[i] = count;
            _neighborOffsets[i] = offset;
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
            (float2*)_alignedPositions.GetUnsafePtr(),
            (float2*)_alignedVelocities.GetUnsafePtr(),
            (float*)_alignedRadii.GetUnsafePtr(),
            (AgentParameters*)_alignedParams.GetUnsafePtr(),
            (float2*)_alignedPrefVelocities.GetUnsafePtr(),
            (int*)_neighborIndices.GetUnsafePtr(),
            (int*)_neighborCounts.GetUnsafePtr(),
            (int*)_neighborOffsets.GetUnsafePtr(),
            obstPtr,
            obstCount,
            nodePtr,
            rootIdx,
            (float2*)_newVelocities.GetUnsafePtr(),
            validCount,
            dt
        );

        // 4. Scatter Phase: Update original components
        for (int i = 0; i < validCount; i++)
        {
            int entityId = _alignedEntityIds[i];
            float2 v = _newVelocities[i];
            
            // Set Velocity
            ref var velComp = ref velocities.GetRef(entityId);
            velComp.Value = v;
            
            // Update Position
            ref var posComp = ref positions.GetRef(entityId);
            posComp.Value += v * dt;
        }

        // Sync Spatial Index
        // We use the updated positions from _alignedPositions? 
        // No, we just updated the components. 
        // SpatialIndexManager typically needs a List<Vector2>.
        // We can rebuild it from our updated data.
        
        System.Collections.Generic.List<Vector2> posList = new System.Collections.Generic.List<Vector2>(validCount);
        for(int i=0; i<validCount; ++i)
        {
            // Re-calculate pos since we modified it in scatter loop?
            // Or just read from component.
            // Scatter loop updated component.
            int entityId = _alignedEntityIds[i];
            float2 p = positions.GetRef(entityId).Value;
            posList.Add(new Vector2(p.x, p.y));
        }
        
        // Note: This pushes positions to SpatialIndexManager.
        // SpatialIndexManager expects positions for ALL entities?
        // Or just the ones we simulated?
        // If we have static entities, they might be missing here if they don't have MovementState.
        // But SpatialIndexManager likely manages "Units".
        // This part implies SpatialIndexManager is tightly coupled to this System.
        SpatialIndexManager.Instance.UpdatePositions(posList);
    }

    private void EnsureCapacity(int count)
    {
        if (count > _capacity)
        {
            _capacity = math.max(count, _capacity * 2);
            if (_capacity < 128) _capacity = 128;

            DisposeArrays();

            _neighborIndices = new NativeArray<int>(_capacity * _maxNeighbors, Allocator.Persistent);
            _neighborCounts = new NativeArray<int>(_capacity, Allocator.Persistent);
            _neighborOffsets = new NativeArray<int>(_capacity, Allocator.Persistent);
            _newVelocities = new NativeArray<float2>(_capacity, Allocator.Persistent);
            
            _alignedPositions = new NativeArray<float2>(_capacity, Allocator.Persistent);
            _alignedVelocities = new NativeArray<float2>(_capacity, Allocator.Persistent);
            _alignedRadii = new NativeArray<float>(_capacity, Allocator.Persistent);
            _alignedParams = new NativeArray<AgentParameters>(_capacity, Allocator.Persistent);
            _alignedPrefVelocities = new NativeArray<float2>(_capacity, Allocator.Persistent);
            _alignedEntityIds = new NativeArray<int>(_capacity, Allocator.Persistent);
        }
    }

    public void UpdateObstacles(System.Collections.Generic.List<RVOObstacle> obstacles)
    {
        if (_nativeObstacles.IsCreated) _nativeObstacles.Dispose();
        
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
