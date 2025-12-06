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

    private const float PenetrationPadding = 0.001f;

    private List<RVOAgent> _agents = new List<RVOAgent>();
    private List<ORCALine> _orcaLines = new List<ORCALine>();
    private List<int> _neighborIndices = new List<int>();
    private List<RVOAgent> _neighbors = new List<RVOAgent>();
    private List<RVOObstacle> _neighborObstacles = new List<RVOObstacle>(); // Sorted obstacles
    private List<Vector2> _cachedPositions = new List<Vector2>();
    private bool _obstaclesDirty = false;

    private ISegmentSpatialIndex _obstacleIndex = new SegmentKdTreeIndex();

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
        return true;
    }

    public void ClearAgents()
    {
        _agents.Clear();
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

    public RVOObstacle AddObstacle(Vector3 p1, Vector3 p2)
    {
        RVOObstacle obstacle = new RVOObstacle(new float2(p1.x, p1.z), new float2(p2.x, p2.z));
        _obstacles.Add(obstacle);
        _obstaclesDirty = true;
        return obstacle;
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

    public bool RemoveObstacle(RVOObstacle obstacle)
    {
        if (obstacle == null)
        {
            return false;
        }

        bool removed = _obstacles.Remove(obstacle);
        if (removed)
        {
            _obstaclesDirty = true;
        }
        return removed;
    }

    public void ProcessObstacles()
    {
        // Link obstacles together in a chain
        // This is important for the leg-based velocity obstacle calculations
        // Link obstacles together in a chain based on geometric connectivity
        // This handles multiple disjoint polygons and out-of-order addition
        float toleranceSq = 0.0001f;

        Debug.Log("=== ProcessObstacles Start ===");
        for (int i = 0; i < _obstacles.Count; i++)
        {
            RVOObstacle current = _obstacles[i];

            Debug.Log($"Edge{i}: {current.Point1} → {current.Point2}, Dir: {current.Direction}");

            // Find NextObstacle: starts where current ends
            current.NextObstacle = null;
            for (int j = 0; j < _obstacles.Count; j++)
            {
                if (i == j) continue;
                if (math.distancesq(current.Point2, _obstacles[j].Point1) < toleranceSq)
                {
                    current.NextObstacle = _obstacles[j];
                    Debug.Log($"  Next = Edge{j}");
                    break;
                }
            }

            // Find PrevObstacle: ends where current starts
            current.PrevObstacle = null;
            for (int j = 0; j < _obstacles.Count; j++)
            {
                if (i == j) continue;
                if (math.distancesq(_obstacles[j].Point2, current.Point1) < toleranceSq)
                {
                    current.PrevObstacle = _obstacles[j];
                    Debug.Log($"  Prev = Edge{j}");
                    break;
                }
            }

            // Update convexity based on angle with next obstacle
            if (current.NextObstacle != null)
            {
                // An obstacle vertex is convex if turning left from current to next
                // det > 0 means counter-clockwise turn (convex in CCW obstacle)
                float2 nextDir = current.NextObstacle.Direction;
                float detValue = RVOMath.det(current.Direction, nextDir);
                current.IsConvex = detValue >= 0.0f;
                Debug.Log($"  IsConvex = {current.IsConvex} (det={detValue})");
            }
            else
            {
                // If no next obstacle (e.g. open chain), assume convex
                current.IsConvex = true;
                Debug.Log($"  IsConvex = true (no next)");
            }
        }
        Debug.Log("=== ProcessObstacles End ===");

        // Rebuild Spatial Index
        _obstacleIndex.Build(_obstacles);

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

                    // 1. Query Agent Neighbors (Sorted)
                    _neighborIndices.Clear();
                    // Query MaxNeighbors + 1 to account for self-inclusion
                    SpatialIndexManager.Instance.QueryKNearestSorted(agent.Position, agent.MaxNeighbors + 1, agent.NeighborDist, _neighborIndices);

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

                    // 2. Query Obstacle Neighbors (Sorted)
                    // This sorting is required for "Already Covered" optimization in RVOMath to work correctly.
                    _obstacleIndex.QueryNearest(agent.Position, _neighborObstacles);

                    // Construct ORCA Lines
                    _orcaLines.Clear();

                    // Add Obstacle Lines FIRST (using sorted list)
                    int orcaCountBefore = _orcaLines.Count;
                    RVOMath.ConstructObstacleORCALines(agent, _neighborObstacles, dt, _orcaLines);
                    int obstacleORCACount = _orcaLines.Count - orcaCountBefore;

                    // Add Agent Lines
                    RVOMath.ConstructORCALines(agent, _neighbors, dt, _orcaLines);

                    if (i == 0) // Debug Agent 0
                    {
                        Debug.Log($"[RVO] Agent 0: Obstacle ORCA={obstacleORCACount}, Agent ORCA={_orcaLines.Count - obstacleORCACount}, Total={_orcaLines.Count}");
                    }

                    // Linear Programming
                    float2 newVel = agent.NewVelocity;
                    int lineFail = RVOMath.linearProgram2(_orcaLines, agent.MaxSpeed, agent.PrefVelocity, false, ref newVel);

                    if (lineFail < _orcaLines.Count)
                    {
                        // Pass obstacleORCACount so collision constraints (point=(0,0)) are treated as HARD constraints
                        RVOMath.linearProgram3(_orcaLines, obstacleORCACount, lineFail, agent.MaxSpeed, ref newVel);
                    }

                    if (i == 0) // Debug Agent 0
                    {
                        Debug.Log($"[RVO] Agent 0: PrefVel={agent.PrefVelocity}, NewVel={newVel}, lineFail={lineFail}");
                    }

                    agent.NewVelocity = newVel;

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

                // 4. Update Spatial Index
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

}
