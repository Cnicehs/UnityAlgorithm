using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;
using UnityEngine;

[UpdateInGroup(SystemGroup.FixedUpdate)]
public class RVOSimulationSystem : ISystem
{
    public bool Enabled { get; set; } = true;
    public float LastExecutionTime { get; private set; }

    private NativeArray<int> _neighborIndices;
    private NativeArray<int> _neighborCounts;
    private NativeArray<int> _neighborOffsets;

    private NativeArray<int> _obstacleNeighborIndices;
    private NativeArray<int> _obstacleNeighborCounts;
    private NativeArray<int> _obstacleNeighborOffsets;

    private NativeArray<float2> _newVelocities;
    private NativeArray<SIMDRVO.ObstacleData> _nativeObstacles;
    private SIMDLinearSegmentIndex _segmentSpatialIndex;

    // Aligned buffers for SIMD RVO
    private NativeArray<float2> _alignedPositions;
    private NativeArray<float2> _alignedVelocities;
    private NativeArray<float> _alignedRadii;
    private NativeArray<AgentParameters> _alignedParams;
    private NativeArray<float2> _alignedPrefVelocities;
    private NativeArray<int> _alignedEntityIds; // To scatter back

    private System.Collections.Generic.Dictionary<int, int> _entityToAlignedMap = new System.Collections.Generic.Dictionary<int, int>();
    // NEW: Track which EntityID is at which index in the SpatialIndex
    private System.Collections.Generic.List<int> _spatialIndexEntityIds = new System.Collections.Generic.List<int>();

    private int _maxNeighbors = 10;
    private int _maxObstacleNeighbors = 32;
    private int _capacity = 0;

    public void Initialize()
    {
        _segmentSpatialIndex = new SIMDLinearSegmentIndex();
    }

public void Shutdown()
{
    DisposeAgentArrays();
    if (_nativeObstacles.IsCreated) _nativeObstacles.Dispose();
    _segmentSpatialIndex.Dispose();
}

private void DisposeAgentArrays()
{
    if (_neighborIndices.IsCreated) _neighborIndices.Dispose();
    if (_neighborCounts.IsCreated) _neighborCounts.Dispose();
    if (_neighborOffsets.IsCreated) _neighborOffsets.Dispose();
    if (_obstacleNeighborIndices.IsCreated) _obstacleNeighborIndices.Dispose();
    if (_obstacleNeighborCounts.IsCreated) _obstacleNeighborCounts.Dispose();
    if (_obstacleNeighborOffsets.IsCreated) _obstacleNeighborOffsets.Dispose();
    if (_newVelocities.IsCreated) _newVelocities.Dispose();
    // Do NOT dispose _nativeObstacles here as it is independent of agent capacity

    if (_alignedPositions.IsCreated) _alignedPositions.Dispose();
    if (_alignedVelocities.IsCreated) _alignedVelocities.Dispose();
    if (_alignedRadii.IsCreated) _alignedRadii.Dispose();
    if (_alignedParams.IsCreated) _alignedParams.Dispose();
    if (_alignedPrefVelocities.IsCreated) _alignedPrefVelocities.Dispose();
    if (_alignedEntityIds.IsCreated) _alignedEntityIds.Dispose();
}

    public unsafe void Update(float dt)
    {
        if (!Enabled) return;

        var stopwatch = System.Diagnostics.Stopwatch.StartNew();

        var entityManager = EntityManager.Instance;
        var agentParamsArray = entityManager.GetArray<AgentParameters>();

        int agentCount = agentParamsArray.Count;
        if (agentCount == 0) 
        {
            stopwatch.Stop();
            LastExecutionTime = (float)stopwatch.Elapsed.TotalMilliseconds;
            return;
        }

        EnsureCapacity(agentCount);

        // Gather Phase: Collect data into aligned arrays
        var positions = entityManager.GetArray<PositionComponent>();
        var velocities = entityManager.GetArray<VelocityComponent>();
        var radii = entityManager.GetArray<RadiusComponent>();
        var movementStates = entityManager.GetArray<MovementState>();

        int validCount = 0;
        for (int i = 0; i < agentCount; i++)
        {
            int entityId = agentParamsArray.GetEntityIdFromDenseIndex(i);

            if (!positions.Has(entityId) || !velocities.Has(entityId) ||
                !radii.Has(entityId) || !movementStates.Has(entityId))
            {
                continue;
            }

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
        _entityToAlignedMap.Clear();
        for (int i = 0; i < validCount; ++i) _entityToAlignedMap[_alignedEntityIds[i]] = i;

        for (int i = 0; i < validCount; i++)
        {
            float2 pos = _alignedPositions[i];
            float dist = _alignedParams[i].NeighborDist;
            int maxN = _alignedParams[i].MaxNeighbors;

            System.Collections.Generic.List<int> neighbors = new System.Collections.Generic.List<int>();
            SpatialIndexManager.Instance.GetNeighborsInRadius(new Vector2(pos.x, pos.y), dist, neighbors);

            var validNeighbors = new System.Collections.Generic.List<int>();
            for (int k = 0; k < neighbors.Count; ++k)
            {
                int spatialIndex = neighbors[k];
                if (spatialIndex >= 0 && spatialIndex < _spatialIndexEntityIds.Count)
                {
                    int neighborEntityId = _spatialIndexEntityIds[spatialIndex];
                    if (_entityToAlignedMap.TryGetValue(neighborEntityId, out int alignedIdx))
                    {
                        validNeighbors.Add(alignedIdx);
                    }
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

            for (int j = 0; j < validNeighbors.Count && count < maxN; ++j)
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

            // Obstacle Neighbors
            if (_nativeObstacles.IsCreated)
            {
                float range = _alignedParams[i].TimeHorizonObst * _alignedParams[i].MaxSpeed + _alignedRadii[i];
                var obstacleNeighbors = new System.Collections.Generic.List<int>();
                _segmentSpatialIndex.QueryNearest(new float2(pos.x, pos.y), range, obstacleNeighbors);

                int oCount = 0;
                int oOffset = i * _maxObstacleNeighbors;
                int maxObs = math.min(_maxObstacleNeighbors, _obstacleNeighborIndices.Length - oOffset);

                for (int j = 0; j < obstacleNeighbors.Count && oCount < maxObs; ++j)
                {
                    _obstacleNeighborIndices[oOffset + oCount] = obstacleNeighbors[j];
                    oCount++;
                }

                _obstacleNeighborCounts[i] = oCount;
                _obstacleNeighborOffsets[i] = oOffset;
            }
            else
            {
                _obstacleNeighborCounts[i] = 0;
            }
        }

        // Debug: Check first agent's obstacle count
        if (validCount > 0 && UnityEngine.Time.frameCount % 60 == 0)
        {
           int obsCount = _obstacleNeighborCounts[0];
           int obsTotal = _nativeObstacles.IsCreated ? _nativeObstacles.Length : -1;
           
           UnityEngine.Debug.Log($"[RVOSimulationSystem] Agent 0 Pos: {_alignedPositions[0]}, ObsNeighbors: {obsCount}, TotalObs: {obsTotal}");
           if (obsCount > 0) {
               int offset = _obstacleNeighborOffsets[0];
               int firstObsIdx = _obstacleNeighborIndices[offset];
               if (_nativeObstacles.IsCreated && firstObsIdx < _nativeObstacles.Length) {
                   var obs = _nativeObstacles[firstObsIdx];
                   UnityEngine.Debug.Log($"[RVOSimulationSystem] Neighbor Obs 0: {obs.Point1} -> {obs.Point2}");
               }
           }
        }

        // 3. Compute RVO (Using new Brute Force Signature)
        SIMDRVO.ObstacleData* obstPtr = null;
        int obstCount = 0;

        if (_nativeObstacles.IsCreated)
        {
            // Use raw obstacles directly (Tree logic removed)
            // If _segmentSpatialIndex is used, we assume it just holds the array or we use _nativeObstacles directly.
            // Here we use _nativeObstacles.
            obstPtr = (SIMDRVO.ObstacleData*)_nativeObstacles.GetUnsafePtr();
            obstCount = _nativeObstacles.Length;
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
            (int*)_obstacleNeighborIndices.GetUnsafePtr(),
            (int*)_obstacleNeighborCounts.GetUnsafePtr(),
            (int*)_obstacleNeighborOffsets.GetUnsafePtr(),
            obstPtr,
            obstCount,
            (float2*)_newVelocities.GetUnsafePtr(),
            validCount,
            dt);

        // 4. Scatter Phase
        for (int i = 0; i < validCount; i++)
        {
            int entityId = _alignedEntityIds[i];
            float2 v = _newVelocities[i];

            ref var velComp = ref velocities.GetRef(entityId);
            velComp.Value = v;

            ref var posComp = ref positions.GetRef(entityId);

            posComp.Value += v * dt;
        }

        // Sync Spatial Index
        System.Collections.Generic.List<Vector2> posList = new System.Collections.Generic.List<Vector2>(validCount);
        _spatialIndexEntityIds.Clear();
        if (_spatialIndexEntityIds.Capacity < validCount) _spatialIndexEntityIds.Capacity = validCount;

        for (int i = 0; i < validCount; ++i)
        {
            int entityId = _alignedEntityIds[i];
            float2 p = positions.GetRef(entityId).Value;
            posList.Add(new Vector2(p.x, p.y));
            _spatialIndexEntityIds.Add(entityId);
        }

        SpatialIndexManager.Instance.UpdatePositions(posList);

        stopwatch.Stop();
        LastExecutionTime = (float)stopwatch.Elapsed.TotalMilliseconds;
    }

    private void EnsureCapacity(int count)
    {
    if (count > _capacity)
    {
        _capacity = math.max(count, _capacity * 2);
        if (_capacity < 128) _capacity = 128;

        DisposeAgentArrays();

        _neighborIndices = new NativeArray<int>(_capacity * _maxNeighbors, Allocator.Persistent);
            _neighborCounts = new NativeArray<int>(_capacity, Allocator.Persistent);
            _neighborOffsets = new NativeArray<int>(_capacity, Allocator.Persistent);

            _obstacleNeighborIndices = new NativeArray<int>(_capacity * _maxObstacleNeighbors, Allocator.Persistent);
            _obstacleNeighborCounts = new NativeArray<int>(_capacity, Allocator.Persistent);
            _obstacleNeighborOffsets = new NativeArray<int>(_capacity, Allocator.Persistent);

            _newVelocities = new NativeArray<float2>(_capacity, Allocator.Persistent);

            // Do NOT dispose _nativeObstacles here! They are managed by UpdateObstacles and are independent of agent count.
            // if (_nativeObstacles.IsCreated) _nativeObstacles.Dispose();

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
        // 1. Process Obstacles (Link and Calculate Convexity)
        // Ensure obstacles are effectively linked. The source RVOObstacle objects might already be linked if they came from a clean source, 
        // but UnifiedComparisonDemo creates them fresh and might not link them.
        // It's safer to re-link them here to be sure, similar to SIMDRVOSimulator.
        float toleranceSq = 0.0001f;
        for (int i = 0; i < obstacles.Count; i++)
        {
            RVOObstacle current = obstacles[i];

            // Find NextObstacle
            current.NextObstacle = null;
            for (int j = 0; j < obstacles.Count; j++)
            {
                if (i == j) continue;
                if (math.distancesq(current.Point2, obstacles[j].Point1) < toleranceSq)
                {
                    current.NextObstacle = obstacles[j];
                    break;
                }
            }

            // Find PrevObstacle
            current.PrevObstacle = null;
            for (int j = 0; j < obstacles.Count; j++)
            {
                if (i == j) continue;
                if (math.distancesq(obstacles[j].Point2, current.Point1) < toleranceSq)
                {
                    current.PrevObstacle = obstacles[j];
                    break;
                }
            }

            // Calculate Convexity
            if (current.NextObstacle != null)
            {
                float2 nextDir = current.NextObstacle.Direction;
                float detValue = RVOMath.det(current.Direction, nextDir);
                current.IsConvex = detValue >= 0.0f;
            }
            else
            {
                current.IsConvex = true;
            }
        }

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
            // Resolve indices
            if (src.NextObstacle != null)
            {
                int nextIdx = obstacles.IndexOf(src.NextObstacle);
                if (nextIdx != -1) dest.NextObstacleIdx = nextIdx;
            }
            if (src.PrevObstacle != null)
            {
                int prevIdx = obstacles.IndexOf(src.PrevObstacle);
                if (prevIdx != -1) dest.PrevObstacleIdx = prevIdx;
            }
            _nativeObstacles[i] = dest;
        }

        // _segmentSpatialIndex might be used for other queries or debugging, so keep it synced
        _segmentSpatialIndex.Build(_nativeObstacles);
    }

    // Debug Helper
    public void GetDebugObstacleNeighbors(int entityIndex, System.Collections.Generic.List<int> results)
    {
        results.Clear();
        if (!_obstacleNeighborCounts.IsCreated || entityIndex < 0 || entityIndex >= _obstacleNeighborCounts.Length) return;

        int count = _obstacleNeighborCounts[entityIndex];
        int offset = _obstacleNeighborOffsets[entityIndex];

        for (int i = 0; i < count; ++i)
        {
            results.Add(_obstacleNeighborIndices[offset + i]);
        }
    }
}
