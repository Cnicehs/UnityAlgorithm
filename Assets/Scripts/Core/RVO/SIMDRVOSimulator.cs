using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;
using Unity.Collections;
using Unity.Burst;
using Unity.Collections.LowLevel.Unsafe;
using System.Runtime.InteropServices;

public class SIMDRVOSimulator
{
    private static SIMDRVOSimulator _instance;
    public static SIMDRVOSimulator Instance => _instance ??= new SIMDRVOSimulator();

    public float TimeStep = 0.25f;
    public float NeighborDist = 10.0f;
    public int MaxNeighbors = 10;
    public float TimeHorizon = 2.0f;
    public float TimeHorizonObst = 2.0f;
    public float Radius = 0.5f;
    public float MaxSpeed = 2.0f;

    public float NeighborQueryTimeMs => (float)PerformanceProfiler.GetLastMs("SIMDRVO.NeighborQuery");
    public float RVOComputeTimeMs => (float)PerformanceProfiler.GetLastMs("SIMDRVO.Compute");
    public float TotalStepTimeMs => (float)PerformanceProfiler.GetLastMs("SIMDRVO.Step");

    private NativeArray<SIMDRVO.AgentData> _agents;
    private NativeArray<float2> _newVelocities;
    private int _agentCount;

    private NativeArray<int> _neighborIndices;
    private NativeArray<int> _neighborCounts;
    private NativeArray<int> _neighborOffsets;

    private NativeArray<SIMDRVO.ObstacleData> _nativeObstacles;
    private int _obstacleCount;

    private List<Vector2> _positionCache = new List<Vector2>();

    private SIMDRVOSimulator()
    {
    }

    public void Initialize(int count)
    {
        if (_agents.IsCreated) _agents.Dispose();
        if (_newVelocities.IsCreated) _newVelocities.Dispose();
        if (_neighborIndices.IsCreated) _neighborIndices.Dispose();
        if (_neighborCounts.IsCreated) _neighborCounts.Dispose();
        if (_neighborOffsets.IsCreated) _neighborOffsets.Dispose();
        if (_nativeObstacles.IsCreated) _nativeObstacles.Dispose();

        _agents = new NativeArray<SIMDRVO.AgentData>(count, Allocator.Persistent);
        _newVelocities = new NativeArray<float2>(count, Allocator.Persistent);
        
        // Allocate with fixed stride per agent
        int totalNeighborSlots = count * MaxNeighbors;
        _neighborIndices = new NativeArray<int>(totalNeighborSlots, Allocator.Persistent);
        _neighborCounts = new NativeArray<int>(count, Allocator.Persistent);
        _neighborOffsets = new NativeArray<int>(count, Allocator.Persistent);
        
        _agentCount = count;
    }

    public void Reallocate(int count)
    {
        if (_agentCount == count) return;
        Initialize(count);
    }

    public void UpdateAgentData(int index, Vector2 position, Vector2 velocity, Vector2 prefVelocity)
    {
        if (index >= 0 && index < _agentCount)
        {
            var agent = _agents[index];
            agent.Position = position;
            agent.Velocity = velocity;
            agent.PrefVelocity = prefVelocity;
            agent.Radius = Radius;
            agent.MaxSpeed = MaxSpeed;
            agent.NeighborDist = NeighborDist;
            agent.MaxNeighbors = MaxNeighbors;
            agent.TimeHorizon = TimeHorizon;
            agent.TimeHorizonObst = TimeHorizonObst;
            _agents[index] = agent;
        }
    }

    public Vector2 GetNewVelocity(int index)
    {
        if (index >= 0 && index < _agentCount) return _newVelocities[index];
        return Vector2.zero;
    }

    public Vector2 GetAgentPosition(int index)
    {
        if (index >= 0 && index < _agentCount) return _agents[index].Position;
        return Vector2.zero;
    }

    public void Dispose()
    {
        if (_agents.IsCreated) _agents.Dispose();
        if (_newVelocities.IsCreated) _newVelocities.Dispose();
        if (_neighborIndices.IsCreated) _neighborIndices.Dispose();
        if (_neighborCounts.IsCreated) _neighborCounts.Dispose();
        if (_neighborOffsets.IsCreated) _neighborOffsets.Dispose();
        if (_nativeObstacles.IsCreated) _nativeObstacles.Dispose();
    }

    public void Shutdown()
    {
        Dispose();
    }

    public void UpdateObstacles(List<RVOObstacle> obstacles)
    {
        if (_nativeObstacles.IsCreated) _nativeObstacles.Dispose();
        
        _obstacleCount = obstacles.Count;
        if (_obstacleCount == 0) return;

        // 1. Process Obstacles (Link and Calculate Convexity)
        // This logic is copied from RVOSimulator.ProcessObstacles to ensure SIMD version receives valid data.
        // It modifies the input list objects directly.
        float toleranceSq = 0.0001f;
        for (int i = 0; i < _obstacleCount; i++)
        {
            RVOObstacle current = obstacles[i];

            // Find NextObstacle
            current.NextObstacle = null;
            for (int j = 0; j < _obstacleCount; j++)
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
            for (int j = 0; j < _obstacleCount; j++)
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

        // 2. Convert to NativeArray
        _nativeObstacles = new NativeArray<SIMDRVO.ObstacleData>(_obstacleCount, Allocator.Persistent);

        for (int i = 0; i < _obstacleCount; i++)
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
    }

    private List<int> _tempNeighbors = new List<int>();

    public unsafe void Step(float dt)
    {
        using (PerformanceProfiler.ProfilerScope.Begin("SIMDRVO.Step"))
        {
            // 1. Build Neighbor Lists (Main Thread)
            using (PerformanceProfiler.ProfilerScope.Begin("SIMDRVO.NeighborQuery"))
            {
                // Optimization: If using SIMDQuadTreeIndex, use its Batch Query (Burst)
                // This avoids C# overhead of looping 500+ agents, allocating lists, and sorting in managed code.
                var spatialIndex = SpatialIndexManager.Instance.GetCurrentIndex();
                if (spatialIndex is SIMDQuadTreeIndex quadTree)
                {
                    // Initialize offsets (Must be done BEFORE query as QueryNeighborsBatch reads them)
                    // Using fixed stride: i * MaxNeighbors
                    for(int i=0; i<_agentCount; i++) _neighborOffsets[i] = i * MaxNeighbors;

                    quadTree.QueryNeighborsBatch(
                        _agents, 
                        _neighborIndices, 
                        _neighborCounts, 
                        _neighborOffsets, 
                        MaxNeighbors
                    );
                }
                else
                {
                    // Fallback to Managed implementation
                    for (int i = 0; i < _agentCount; i++)
                    {
                        var agent = _agents[i];
                        _tempNeighbors.Clear();
                        SpatialIndexManager.Instance.GetNeighborsInRadius(agent.Position, agent.NeighborDist, _tempNeighbors);

                        // Sort neighbors by distance to ensure we pick the closest ones
                        _tempNeighbors.Sort((a, b) =>
                        {
                            float distA = math.distancesq(agent.Position, _agents[a].Position);
                            float distB = math.distancesq(agent.Position, _agents[b].Position);
                            return distA.CompareTo(distB);
                        });

                        // Calculate offset for this agent's neighbor data
                        int offset = i * MaxNeighbors;
                        int count = 0;
                        
                        // Bounds check: ensure we don't exceed array capacity
                        int maxNeighborsForThisAgent = math.min(MaxNeighbors, _neighborIndices.Length - offset);
                        
                        for (int j = 0; j < _tempNeighbors.Count && count < maxNeighborsForThisAgent; j++)
                        {
                            int neighborIdx = _tempNeighbors[j];
                            if (neighborIdx != i && neighborIdx < _agentCount)
                            {
                                _neighborIndices[offset + count] = neighborIdx;
                                count++;
                            }
                        }
                        
                        _neighborCounts[i] = count;
                        _neighborOffsets[i] = offset;
                    }
                }
            }

            // 2. Compute RVO velocities using Burst (Single-threaded, SIMD-accelerated)
            using (PerformanceProfiler.ProfilerScope.Begin("SIMDRVO.Compute"))
            {
                SIMDRVO.ComputeRVOVelocities(
                    (SIMDRVO.AgentData*)_agents.GetUnsafePtr(),
                    (int*)_neighborIndices.GetUnsafePtr(),
                    (int*)_neighborCounts.GetUnsafePtr(),
                    (int*)_neighborOffsets.GetUnsafePtr(),
                    (SIMDRVO.ObstacleData*)(_nativeObstacles.IsCreated ? _nativeObstacles.GetUnsafePtr() : null),
                    _nativeObstacles.IsCreated ? _obstacleCount : 0,
                    (float2*)_newVelocities.GetUnsafePtr(),
                    _agentCount,
                    dt
                );
            }

            // 3. Update Agents (Velocity & Position)
            using (PerformanceProfiler.ProfilerScope.Begin("SIMDRVO.Update"))
            {
                // Sync to managed list for SpatialIndexManager
                if (_positionCache.Capacity < _agentCount) _positionCache.Capacity = _agentCount;
                _positionCache.Clear();

                for (int i = 0; i < _agentCount; i++)
                {
                    var agent = _agents[i];
                    agent.Velocity = _newVelocities[i];
                    agent.Position += agent.Velocity * dt;
                    _agents[i] = agent; // Write back to NativeArray
                    
                    _positionCache.Add(new Vector2(agent.Position.x, agent.Position.y));
                }
                
                // Update Spatial Index with new positions
                SpatialIndexManager.Instance.UpdatePositions(_positionCache);
            }
        }
    }
}
