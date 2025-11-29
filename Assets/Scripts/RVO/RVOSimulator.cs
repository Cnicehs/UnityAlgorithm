using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;

public class RVOSimulator
{
    private static RVOSimulator _instance;
    public static RVOSimulator Instance => _instance ??= new RVOSimulator();

    public float TimeStep = 0.25f;
    public float NeighborDist = 10.0f;
    public int MaxNeighbors = 10;
    public float TimeHorizon = 2.0f;
    public float TimeHorizonObst = 2.0f;
    public float Radius = 0.5f;
    public float MaxSpeed = 2.0f;

    public float NeighborQueryTimeMs => (float)PerformanceProfiler.GetLastMs("RVO.NeighborQuery");
    public float RVOComputeTimeMs => (float)PerformanceProfiler.GetLastMs("RVO.Compute");
    public float PositionUpdateTimeMs => (float)PerformanceProfiler.GetLastMs("RVO.PositionUpdate");
    public float TotalStepTimeMs => (float)PerformanceProfiler.GetLastMs("RVO.Step");

    private const float ObstacleLinkEpsilon = 0.05f;
    private const float PenetrationPadding = 0.001f;

    private List<RVOAgent> _agents = new List<RVOAgent>();
    private List<ORCALine> _orcaLines = new List<ORCALine>();
    private List<int> _neighborIndices = new List<int>();
    private List<RVOAgent> _neighbors = new List<RVOAgent>();
    private List<Vector2> _cachedPositions = new List<Vector2>();
    private bool _obstaclesDirty = false;
    private bool _spatialIndexDirty = true;

    private RVOSimulator()
    {
    }

    public void AddAgent(Vector3 position)
    {
        RVOAgent agent = new RVOAgent(_agents.Count, new float2(position.x, position.z));
        agent.Radius = Radius;
        agent.MaxSpeed = MaxSpeed;
        agent.NeighborDist = NeighborDist;
        agent.MaxNeighbors = MaxNeighbors;
        agent.TimeHorizon = TimeHorizon;
        agent.TimeHorizonObst = TimeHorizonObst;
        _agents.Add(agent);
        _spatialIndexDirty = true;
    }

    public bool RemoveAgent(int agentIndex)
    {
        if (agentIndex < 0 || agentIndex >= _agents.Count)
        {
            return false;
        }

        _agents.RemoveAt(agentIndex);
        for (int i = agentIndex; i < _agents.Count; i++)
        {
            _agents[i].ID = i;
        }
        _spatialIndexDirty = true;
        return true;
    }

    public void ClearAgents()
    {
        _agents.Clear();
        _spatialIndexDirty = true;
    }

    public void SetAgentPrefVelocity(int i, Vector3 prefVel)
    {
        if (i >= 0 && i < _agents.Count)
        {
            _agents[i].PrefVelocity = new float2(prefVel.x, prefVel.z);
        }
    }

    public Vector3 GetAgentPosition(int i)
    {
        if (i >= 0 && i < _agents.Count)
        {
            float2 pos = _agents[i].Position;
            return new Vector3(pos.x, 0, pos.y);
        }
        return Vector3.zero;
    }

    public int GetAgentCount()
    {
        return _agents.Count;
    }

    public Vector3 GetAgentVelocity(int i)
    {
        if (i >= 0 && i < _agents.Count)
        {
            float2 vel = _agents[i].Velocity;
            return new Vector3(vel.x, 0, vel.y);
        }
        return Vector3.zero;
    }

    private List<RVOObstacle> _obstacles = new List<RVOObstacle>();

    public List<RVOObstacle> GetObstacles()
    {
        return _obstacles;
    }

    public void AddObstacle(Vector3 p1, Vector3 p2)
    {
        _obstacles.Add(new RVOObstacle(new float2(p1.x, p1.z), new float2(p2.x, p2.z)));
        _obstaclesDirty = true;
    }

    public void ClearObstacles()
    {
        _obstacles.Clear();
        _obstaclesDirty = true;
    }

    public bool RemoveObstacle(int obstacleIndex)
    {
        if (obstacleIndex < 0 || obstacleIndex >= _obstacles.Count)
        {
            return false;
        }

        _obstacles.RemoveAt(obstacleIndex);
        _obstaclesDirty = true;
        return true;
    }

    public void ProcessObstacles()
    {
        if (_obstacles.Count == 0)
        {
            _obstaclesDirty = false;
            return;
        }

        for (int i = 0; i < _obstacles.Count; i++)
        {
            _obstacles[i].NextObstacle = null;
            _obstacles[i].PrevObstacle = null;
        }

        Dictionary<Vector2Int, List<RVOObstacle>> startLookup = new Dictionary<Vector2Int, List<RVOObstacle>>(_obstacles.Count);
        float linkThresholdSq = ObstacleLinkEpsilon * ObstacleLinkEpsilon;

        for (int i = 0; i < _obstacles.Count; i++)
        {
            Vector2Int key = QuantizePoint(_obstacles[i].Point1);
            if (!startLookup.TryGetValue(key, out List<RVOObstacle> list))
            {
                list = new List<RVOObstacle>();
                startLookup[key] = list;
            }
            list.Add(_obstacles[i]);
        }

        for (int i = 0; i < _obstacles.Count; i++)
        {
            RVOObstacle obstacle = _obstacles[i];
            Vector2Int endKey = QuantizePoint(obstacle.Point2);
            if (!startLookup.TryGetValue(endKey, out List<RVOObstacle> candidates))
            {
                continue;
            }

            RVOObstacle best = null;
            float bestDistSq = float.MaxValue;
            for (int j = 0; j < candidates.Count; j++)
            {
                RVOObstacle candidate = candidates[j];
                if (candidate == obstacle)
                {
                    continue;
                }

                float distSq = math.lengthsq(obstacle.Point2 - candidate.Point1);
                if (distSq <= linkThresholdSq && distSq < bestDistSq)
                {
                    best = candidate;
                    bestDistSq = distSq;
                }
            }

            if (best != null)
            {
                obstacle.NextObstacle = best;
                best.PrevObstacle = obstacle;
            }
        }

        // 2. Calculate Convexity
        for (int i = 0; i < _obstacles.Count; ++i)
        {
            RVOObstacle obst = _obstacles[i];
            RVOObstacle prev = obst.PrevObstacle;
            RVOObstacle next = obst.NextObstacle;

            if (prev != null && next != null)
            {
                obst.IsConvex = RVOMath.LeftOf(prev.Point1, obst.Point1, next.Point1) >= 0.0f;
            }
            else
            {
                obst.IsConvex = true;
            }
        }

        _obstaclesDirty = false;
    }

    public void Step(float dt)
    {
        using (PerformanceProfiler.ProfilerScope.Begin("RVO.Step"))
        {
            // Process obstacles if they have been modified
            if (_obstaclesDirty)
            {
                ProcessObstacles();
            }

            // 1. RVO Computation Phase
            using (PerformanceProfiler.ProfilerScope.Begin("RVO.Compute"))
            {
                for (int i = 0; i < _agents.Count; i++)
                {
                    RVOAgent agent = _agents[i];

                    _neighborIndices.Clear();
                    SpatialIndexManager.Instance.GetNeighborsInRadius(agent.Position, agent.NeighborDist, _neighborIndices);

                    _neighborIndices.Sort((a, b) =>
                    {
                        float distA = math.distancesq(agent.Position, _agents[a].Position);
                        float distB = math.distancesq(agent.Position, _agents[b].Position);
                        return distA.CompareTo(distB);
                    });

                    _neighbors.Clear();
                    for (int j = 0; j < _neighborIndices.Count; j++)
                    {
                        int idx = _neighborIndices[j];
                        if (idx != i && idx < _agents.Count)
                        {
                            _neighbors.Add(_agents[idx]);
                        }
                        if (_neighbors.Count >= agent.MaxNeighbors) break;
                    }

                    // Construct ORCA Lines
                    _orcaLines.Clear();

                    // Add Obstacle Lines FIRST
                    int orcaCountBefore = _orcaLines.Count;
                    RVOMath.ConstructObstacleORCALines(agent, _obstacles, dt, _orcaLines);
                    int obstacleORCACount = _orcaLines.Count - orcaCountBefore;

                    // Add Agent Lines
                    RVOMath.ConstructORCALines(agent, _neighbors, dt, _orcaLines);

                    // Debug logging for first agent
                    if (i == 0 && Time.frameCount % 60 == 0)
                    {
                        Debug.Log($"[RVO Agent 0] Neighbors: {_neighbors.Count}, Obstacle ORCA Lines: {obstacleORCACount}, Total ORCA Lines: {_orcaLines.Count}, Obstacles in sim: {_obstacles.Count}");
                        Debug.Log($"[RVO Agent 0] Position: {agent.Position}, PrefVel: {agent.PrefVelocity}");
                    }

                    // Linear Programming
                    float2 newVel = agent.NewVelocity;
                    int lineFail = RVOMath.linearProgram2(_orcaLines, agent.MaxSpeed, agent.PrefVelocity, false, ref newVel);

                    if (lineFail < _orcaLines.Count)
                    {
                        // Pass obstacleORCACount so collision constraints (point=(0,0)) are treated as HARD constraints
                        RVOMath.linearProgram3(_orcaLines, obstacleORCACount, lineFail, agent.MaxSpeed, ref newVel);
                    }

                    agent.NewVelocity = newVel;

                    if (i == 0 && Time.frameCount % 60 == 0)
                    {
                        Debug.Log($"[RVO Agent 0] NewVel: {newVel}");
                    }
                }
            }

            // 3. Position Update Phase
            using (PerformanceProfiler.ProfilerScope.Begin("RVO.PositionUpdate"))
            {
                for (int i = 0; i < _agents.Count; i++)
                {
                    _agents[i].Velocity = _agents[i].NewVelocity;
                    _agents[i].Position += _agents[i].Velocity * dt;
                }

                // 4. Deep Penetration Separation
                SeparateOverlappingAgents();

                // 5. Update Spatial Index
                // We need to sync the spatial index with new positions for the next frame's queries.
                if (_cachedPositions.Capacity < _agents.Count)
                {
                    _cachedPositions.Capacity = _agents.Count;
                }
                _cachedPositions.Clear();
                for (int i = 0; i < _agents.Count; i++)
                {
                    float2 pos = _agents[i].Position;
                    _cachedPositions.Add(new Vector2(pos.x, pos.y));
                }
                SpatialIndexManager.Instance.UpdatePositions(_cachedPositions);
            }
        }
    }

    private Vector2Int QuantizePoint(float2 point)
    {
        float quantizationScale = 1.0f / ObstacleLinkEpsilon;
        return new Vector2Int(
            Mathf.RoundToInt(point.x * quantizationScale),
            Mathf.RoundToInt(point.y * quantizationScale)
        );
    }

    private void SeparateOverlappingAgents()
    {
        for (int i = 0; i < _agents.Count; i++)
        {
            RVOAgent agent1 = _agents[i];

            for (int j = i + 1; j < _agents.Count; j++)
            {
                RVOAgent agent2 = _agents[j];

                float2 diff = agent2.Position - agent1.Position;
                float distSq = math.lengthsq(diff);
                float minDist = agent1.Radius + agent2.Radius + PenetrationPadding;
                float minDistSq = minDist * minDist;

                if (distSq < minDistSq && distSq > 0.0001f)
                {
                    float dist = math.sqrt(distSq);
                    float2 dir = diff / dist;
                    float overlap = minDist - dist;
                    float2 separation = dir * (overlap * 0.5f);

                    agent1.Position -= separation;
                    agent2.Position += separation;
                }
            }
        }
    }
}
