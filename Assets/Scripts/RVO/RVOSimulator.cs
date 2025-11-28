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
    public float Radius = 0.5f;
    public float MaxSpeed = 2.0f;

    private List<RVOAgent> _agents = new List<RVOAgent>();
    private List<ORCALine> _orcaLines = new List<ORCALine>();
    private List<int> _neighborIndices = new List<int>();
    private List<RVOAgent> _neighbors = new List<RVOAgent>();

    private RVOSimulator()
    {
    }

    public void AddAgent(Vector2 position)
    {
        RVOAgent agent = new RVOAgent(_agents.Count, position);
        agent.Radius = Radius;
        agent.MaxSpeed = MaxSpeed;
        agent.NeighborDist = NeighborDist;
        agent.MaxNeighbors = MaxNeighbors;
        agent.TimeHorizon = TimeHorizon;
        _agents.Add(agent);
    }

    public void ClearAgents()
    {
        _agents.Clear();
    }

    public void SetAgentPrefVelocity(int i, Vector2 prefVel)
    {
        if (i >= 0 && i < _agents.Count)
        {
            _agents[i].PrefVelocity = prefVel;
        }
    }

    public Vector2 GetAgentPosition(int i)
    {
        if (i >= 0 && i < _agents.Count) return _agents[i].Position;
        return Vector2.zero;
    }

    public int GetAgentCount()
    {
        return _agents.Count;
    }

    public void Step(float dt)
    {
        // 1. Update Spatial Index (Handled by SpatialIndexManager)
        // We assume SpatialIndexManager has updated the index with current agent positions.
        
        // 2. Compute New Velocities
        for (int i = 0; i < _agents.Count; i++)
        {
            RVOAgent agent = _agents[i];
            
            // Find Neighbors
            _neighborIndices.Clear();
            SpatialIndexManager.Instance.GetNeighborsInRadius(agent.Position, agent.NeighborDist, _neighborIndices);

            _neighbors.Clear();
            for (int j = 0; j < _neighborIndices.Count; j++)
            {
                int idx = _neighborIndices[j];
                if (idx != i) // Don't collide with self
                {
                    // Note: This assumes agent indices match spatial index indices.
                    // This is true if we manage them in sync.
                    if (idx < _agents.Count)
                    {
                        _neighbors.Add(_agents[idx]);
                    }
                }
                if (_neighbors.Count >= agent.MaxNeighbors) break;
            }

            // Construct ORCA Lines
            RVOMath.ConstructORCALines(agent, _neighbors, dt, _orcaLines);

            // Linear Programming
            float2 newVel = agent.NewVelocity;
            int lineFail = RVOMath.linearProgram2(_orcaLines, agent.MaxSpeed, agent.PrefVelocity, false, ref newVel);

            if (lineFail < _orcaLines.Count)
            {
                RVOMath.linearProgram3(_orcaLines, 0, lineFail, agent.MaxSpeed, ref newVel);
            }

            agent.NewVelocity = newVel;
        }

        // 3. Update Positions
        for (int i = 0; i < _agents.Count; i++)
        {
            _agents[i].Velocity = _agents[i].NewVelocity;
            _agents[i].Position += _agents[i].Velocity * dt;
        }
    }
}
