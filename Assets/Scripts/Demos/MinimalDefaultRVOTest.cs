using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;
using System.Text;

public class MinimalDefaultRVOTest : MonoBehaviour
{
    private RVOSimulator _sim;

    // Test parameters
    [Range(1, 500)]
    public int AgentCount = 10;
    public float TimeStep = 0.25f;
    public float SpawnRadius = 10.0f;
    public float MaxSpeed = 0.2f;

    // Gizmo Settings
    public bool ShowAgents = true;
    public bool ShowVelocity = true;
    public bool ShowObstacles = true;

    void Start()
    {
        InitializeDefault();
        Debug.Log($"MinimalDefaultRVOTest Initialized with {AgentCount} agents.");
    }

    private void InitializeDefault()
    {
        // Initialize SpatialIndexManager (Shared between both implementations usually, but good to init)
        SpatialIndexManager.Instance.Initialize(AgentCount, 100);

        _sim = RVOSimulator.Instance;
        _sim.ClearAgents();
        _sim.ClearObstacles();
        
        _sim.TimeStep = TimeStep;
        _sim.Radius = 0.5f;
        _sim.MaxSpeed = MaxSpeed;
        _sim.TimeHorizon = 2.0f;
        _sim.TimeHorizonObst = 2.0f;
        _sim.NeighborDist = 10.0f;
        _sim.MaxNeighbors = 10;

        // 1. Central Box
        AddBoxObstacle(new Vector3(0, 0, 0), new Vector3(4, 0, 4));

        // 2. Random blocks
        AddBoxObstacle(new Vector3(-6, 0, 4), new Vector3(2, 0, 2));
        AddBoxObstacle(new Vector3(6, 0, -4), new Vector3(2, 0, 2));

        _sim.ProcessObstacles();

        // Agents in circle
        for (int i = 0; i < AgentCount; i++)
        {
            float angle = (i / (float)AgentCount) * Mathf.PI * 2;
            Vector3 spawnPos = new Vector3(Mathf.Cos(angle) * SpawnRadius, 0, Mathf.Sin(angle) * SpawnRadius);
            _sim.AddAgent(spawnPos);
             // Ensure agent params are set (though AddAgent uses defaults from Simulator, we want to match our params)
             // Accessing internal agent list isn't directly exposed by AddAgent return, but we know index matches order.
             // Actually RVOSimulator.AddAgent uses the current Simulator params (Radius, etc) as defaults.
        }
    }

    private void AddBoxObstacle(Vector3 center, Vector3 size)
    {
        Vector3 p1 = center + new Vector3(-size.x, 0, -size.z) * 0.5f;
        Vector3 p2 = center + new Vector3(size.x, 0, -size.z) * 0.5f;
        Vector3 p3 = center + new Vector3(size.x, 0, size.z) * 0.5f;
        Vector3 p4 = center + new Vector3(-size.x, 0, size.z) * 0.5f;

        _sim.AddObstacle(p1, p2);
        _sim.AddObstacle(p2, p3);
        _sim.AddObstacle(p3, p4);
        _sim.AddObstacle(p4, p1);
    }

    void Update()
    {
        if (_sim == null) return;

        // Update Simulator params in case they changed in Inspector
        _sim.TimeStep = TimeStep;
        _sim.MaxSpeed = MaxSpeed; // Global max speed, though agents might have their own

        float maxVel = 0f;

        for (int i = 0; i < _sim.GetAgentCount(); i++)
        {
            Vector3 pos = _sim.GetAgentPosition(i);
            float angle = (i / (float)AgentCount) * Mathf.PI * 2;
            Vector3 spawnPos = new Vector3(Mathf.Cos(angle) * SpawnRadius, 0, Mathf.Sin(angle) * SpawnRadius);
            Vector3 goal = -spawnPos;

            Vector3 dir = (goal - pos).normalized * MaxSpeed;
            if (Vector3.Distance(pos, goal) < 1.0f) dir = Vector3.zero;

            _sim.SetAgentPrefVelocity(i, dir);
            
            maxVel = Mathf.Max(maxVel, _sim.GetAgentVelocity(i).magnitude);
        }

        _sim.Step(Time.deltaTime);

        if (Time.frameCount % 60 == 0)
        {
             // Debug.Log($"[MinimalDefault] Max Vel: {maxVel:F2}");
        }
    }

    void OnDrawGizmos()
    {
        if (_sim == null) return;
        if (!Application.isPlaying) return;

        // Draw Obstacles
        if (ShowObstacles)
        {
            Gizmos.color = Color.white;
            var obstacles = _sim.GetObstacles();
            foreach (var obs in obstacles)
            {
                // RVOObstacle uses float2
                Gizmos.DrawLine(new Vector3(obs.Point1.x, 0, obs.Point1.y), new Vector3(obs.Point2.x, 0, obs.Point2.y));
            }
        }

        // Draw Agents
        if (ShowAgents)
        {
            for (int i = 0; i < _sim.GetAgentCount(); i++)
            {
                Vector3 pos = _sim.GetAgentPosition(i);
                Gizmos.color = Color.red;
                Gizmos.DrawSphere(pos, 0.2f); // Default Radius 0.5 might be too big for visual check if overlapping
                
                if (ShowVelocity)
                {
                    Vector3 vel = _sim.GetAgentVelocity(i);
                    Gizmos.DrawLine(pos, pos + vel);
                }
            }
        }
    }
    
    void OnDestroy()
    {
        _sim?.ClearAgents();
        _sim?.ClearObstacles();
    }
}
