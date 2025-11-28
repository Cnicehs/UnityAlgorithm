using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;
using Unity.Collections;
using Unity.Jobs;
using Unity.Burst;
using Unity.Collections.LowLevel.Unsafe;

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

    private NativeArray<SIMDRVO.AgentData> _agents;
    private NativeArray<float2> _newVelocities;
    private int _agentCount;

    private NativeArray<int> _neighborIndices;
    private NativeArray<int> _neighborCounts;
    private NativeArray<int> _neighborOffsets;
    private int _maxNeighborCountTotal;

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
        
        // Allocate neighbor buffers. Assume max neighbors * count for safety, or resize dynamically.
        // For 10k agents * 10 neighbors = 100k ints. Safe to allocate.
        _maxNeighborCountTotal = count * MaxNeighbors;
        _neighborIndices = new NativeArray<int>(_maxNeighborCountTotal, Allocator.Persistent);
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

    public void Step(float dt)
    {
        // 1. Build Neighbor Lists
        int currentOffset = 0;
        for (int i = 0; i < _agentCount; i++)
        {
            var agent = _agents[i];
            _tempNeighbors.Clear();
            SpatialIndexManager.Instance.GetNeighborsInRadius(agent.Position, agent.NeighborDist, _tempNeighbors);

            int count = 0;
            for (int j = 0; j < _tempNeighbors.Count; j++)
            {
                if (count >= MaxNeighbors) break;
                int neighborIdx = _tempNeighbors[j];
                if (neighborIdx != i)
                {
                    if (currentOffset + count < _neighborIndices.Length)
                    {
                        _neighborIndices[currentOffset + count] = neighborIdx;
                        count++;
                    }
                }
            }
            
            _neighborCounts[i] = count;
            _neighborOffsets[i] = currentOffset;
            currentOffset += MaxNeighbors; 
        }

        // 2. Schedule RVO Job
        var rvoJob = new RVOJob
        {
            Agents = _agents,
            NeighborIndices = _neighborIndices,
            NeighborCounts = _neighborCounts,
            NeighborOffsets = _neighborOffsets,
            TimeStep = dt,
            NewVelocities = _newVelocities
        };

        JobHandle handle = rvoJob.Schedule(_agentCount, 64);
        handle.Complete();
    }
}

[BurstCompile]
public struct RVOJob : IJobParallelFor
{
    [ReadOnly] public NativeArray<SIMDRVO.AgentData> Agents;
    [ReadOnly] public NativeArray<int> NeighborIndices; // Flattened
    [ReadOnly] public NativeArray<int> NeighborCounts;  // Count per agent
    [ReadOnly] public NativeArray<int> NeighborOffsets; // Offset per agent
    [ReadOnly] public float TimeStep;

    public NativeArray<float2> NewVelocities;

    public unsafe void Execute(int index)
    {
        var agent = Agents[index];
        int count = NeighborCounts[index];
        int offset = NeighborOffsets[index];

        // Stack alloc for ORCA lines
        // Max neighbors + obstacles. 
        const int MAX_LINES = 128;
        ORCALine* orcaLines = stackalloc ORCALine[MAX_LINES];
        int lineCount = 0;

        float invTimeHorizon = 1.0f / agent.TimeHorizon;

        for (int i = 0; i < count; ++i)
        {
            int neighborIdx = NeighborIndices[offset + i];
            if (neighborIdx == index) continue;

            var other = Agents[neighborIdx];

            float2 relativePosition = other.Position - agent.Position;
            float2 relativeVelocity = agent.Velocity - other.Velocity;
            float distSq = math.lengthsq(relativePosition);
            float combinedRadius = agent.Radius + other.Radius;
            float combinedRadiusSq = combinedRadius * combinedRadius;

            ORCALine line;
            float2 u;

            if (distSq > combinedRadiusSq)
            {
                float2 w = relativeVelocity - invTimeHorizon * relativePosition;
                float wLengthSq = math.lengthsq(w);
                float dotProduct1 = math.dot(w, relativePosition);

                if (dotProduct1 < 0.0f && dotProduct1 * dotProduct1 > combinedRadiusSq * wLengthSq)
                {
                    float wLength = math.sqrt(wLengthSq);
                    float2 unitW = w / wLength;

                    line.Direction = new float2(unitW.y, -unitW.x);
                    u = (combinedRadius * invTimeHorizon - wLength) * unitW;
                }
                else
                {
                    float leg = math.sqrt(distSq - combinedRadiusSq);
                    if (SIMDRVO.det(relativePosition, w) > 0.0f)
                    {
                        line.Direction = new float2(relativePosition.x * leg - relativePosition.y * combinedRadius, relativePosition.x * combinedRadius + relativePosition.y * leg) / distSq;
                    }
                    else
                    {
                        line.Direction = -new float2(relativePosition.x * leg + relativePosition.y * combinedRadius, -relativePosition.x * combinedRadius + relativePosition.y * leg) / distSq;
                    }

                    float dotProduct2 = math.dot(relativeVelocity, line.Direction);
                    u = dotProduct2 * line.Direction - relativeVelocity;
                }
            }
            else
            {
                float invTimeStep = 1.0f / TimeStep;
                float2 w = relativeVelocity - invTimeStep * relativePosition;
                float wLength = math.length(w);
                float2 unitW = w / wLength;

                line.Direction = new float2(unitW.y, -unitW.x);
                u = (combinedRadius * invTimeStep - wLength) * unitW;
            }

            line.Point = agent.Velocity + 0.5f * u;
            
            if (lineCount < MAX_LINES)
            {
                orcaLines[lineCount++] = line;
            }
        }

        float2 newVel = float2.zero; // Start with zero or current velocity, but linearProgram2 overwrites it if successful.
        // RVO2 usually optimizes towards PrefVelocity.
        // But linearProgram2 initializes result.
        
        int lineFail = SIMDRVO.linearProgram2(orcaLines, lineCount, agent.MaxSpeed, agent.PrefVelocity, false, ref newVel);

        if (lineFail < lineCount)
        {
            SIMDRVO.linearProgram3(orcaLines, lineCount, 0, lineFail, agent.MaxSpeed, ref newVel);
        }

        NewVelocities[index] = newVel;
    }
}
