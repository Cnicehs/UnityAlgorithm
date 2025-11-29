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

    private List<RVOAgent> _agents = new List<RVOAgent>();
    private List<ORCALine> _orcaLines = new List<ORCALine>();
    private List<int> _neighborIndices = new List<int>();
    private List<RVOAgent> _neighbors = new List<RVOAgent>();

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

    public void AddObstacle(Vector3 p1, Vector3 p2)
    {
        _obstacles.Add(new RVOObstacle(new float2(p1.x, p1.z), new float2(p2.x, p2.z)));
    }

    public void ClearObstacles()
    {
        _obstacles.Clear();
    }

    public void ProcessObstacles()
    {
        // 1. Link obstacles
        for (int i = 0; i < _obstacles.Count; ++i)
        {
            for (int j = 0; j < _obstacles.Count; ++j)
            {
                if (i == j) continue;

                if (math.lengthsq(_obstacles[i].Point2 - _obstacles[j].Point1) < 0.01f)
                {
                    _obstacles[i].NextObstacle = _obstacles[j];
                    _obstacles[j].PrevObstacle = _obstacles[i];
                }
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
                // If not linked, treat as convex (or handle appropriately)
                obst.IsConvex = true;
            }
        }
    }

    public void Step(float dt)
    {
        using (PerformanceProfiler.ProfilerScope.Begin("RVO.Step"))
        {
            // 1. Neighbor Query Phase
            using (PerformanceProfiler.ProfilerScope.Begin("RVO.NeighborQuery"))
            {
                for (int i = 0; i < _agents.Count; i++)
                {
                    RVOAgent agent = _agents[i];

                    _neighborIndices.Clear();
                    SpatialIndexManager.Instance.GetNeighborsInRadius(agent.Position, agent.NeighborDist, _neighborIndices);

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
                }
            }

            // 2. RVO Computation Phase
            using (PerformanceProfiler.ProfilerScope.Begin("RVO.Compute"))
            {
                for (int i = 0; i < _agents.Count; i++)
                {
                    RVOAgent agent = _agents[i];

                    // Re-query neighbors (Redundant? The previous block didn't store them per agent)
                    // Optimization: We should store neighbors or do it in one pass.
                    // For now, let's just re-query or fix the logic.
                    // The previous block was just profiling? No, it was populating _neighbors but not using it?
                    // Ah, the previous block was iterating but overwriting _neighbors.
                    // Let's do the query inside the compute loop to be correct.

                    _neighborIndices.Clear();
                    SpatialIndexManager.Instance.GetNeighborsInRadius(agent.Position, agent.NeighborDist, _neighborIndices);

                    // Sort neighbors by distance to ensure we pick the closest ones
                    // Optimization: Use a custom comparer or struct to avoid repeated distance calcs?
                    // For now, just sort the indices based on distance.
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

                // 4. Update Spatial Index
                // We need to sync the spatial index with new positions for the next frame's queries.
                // Optimization: Avoid creating a new list every frame if possible.
                // But SpatialIndexManager.UpdatePositions takes List<Vector2>.
                List<Vector2> allPositions = new List<Vector2>(_agents.Count);
                for (int i = 0; i < _agents.Count; i++)
                {
                    allPositions.Add(_agents[i].Position);
                }
                SpatialIndexManager.Instance.UpdatePositions(allPositions);
            }
        }
    }
}
