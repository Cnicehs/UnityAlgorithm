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

        _agents = new NativeArray<SIMDRVO.AgentData>(count, Allocator.Persistent);
        _newVelocities = new NativeArray<float2>(count, Allocator.Persistent);
        
        // Allocate with fixed stride per agent
        int totalNeighborSlots = count * MaxNeighbors;
        _neighborIndices = new NativeArray<int>(totalNeighborSlots, Allocator.Persistent);
        _neighborCounts = new NativeArray<int>(count, Allocator.Persistent);
        _neighborOffsets = new NativeArray<int>(count, Allocator.Persistent);
        
        _agentCount = count;
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
            _agents[index] = agent;
        }
    }

    public Vector2 GetNewVelocity(int index)
    {
        if (index >= 0 && index < _agentCount) return _newVelocities[index];
        return Vector2.zero;
    }

    public void Dispose()
    {
        if (_agents.IsCreated) _agents.Dispose();
        if (_newVelocities.IsCreated) _newVelocities.Dispose();
        if (_neighborIndices.IsCreated) _neighborIndices.Dispose();
        if (_neighborCounts.IsCreated) _neighborCounts.Dispose();
        if (_neighborOffsets.IsCreated) _neighborOffsets.Dispose();
    }

    public void Shutdown()
    {
        Dispose();
    }

    private List<int> _tempNeighbors = new List<int>();

    public unsafe void Step(float dt)
    {
        using (PerformanceProfiler.ProfilerScope.Begin("SIMDRVO.Step"))
        {
            // 1. Build Neighbor Lists (Main Thread)
            using (PerformanceProfiler.ProfilerScope.Begin("SIMDRVO.NeighborQuery"))
            {
                for (int i = 0; i < _agentCount; i++)
                {
                    var agent = _agents[i];
                    _tempNeighbors.Clear();
                    SpatialIndexManager.Instance.GetNeighborsInRadius(agent.Position, agent.NeighborDist, _tempNeighbors);

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

            // 2. Compute RVO velocities using Burst (Single-threaded, SIMD-accelerated)
            using (PerformanceProfiler.ProfilerScope.Begin("SIMDRVO.Compute"))
            {
                SIMDRVO.ComputeRVOVelocities(
                    (SIMDRVO.AgentData*)_agents.GetUnsafePtr(),
                    (int*)_neighborIndices.GetUnsafePtr(),
                    (int*)_neighborCounts.GetUnsafePtr(),
                    (int*)_neighborOffsets.GetUnsafePtr(),
                    (float2*)_newVelocities.GetUnsafePtr(),
                    _agentCount,
                    dt
                );
            }
        }
    }
}
