using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;
using Unity.Collections;
using System.Text;

public class MinimalSIMDRVOTest : MonoBehaviour
{
    private SIMDRVOSimulator _simSIMD;
    
    // Test parameters
    [Range(1, 500)]
    public int AgentCount = 10; // Reduced default to check if overcrowding is the issue
    public float TimeStep = 0.25f;
    public float SpawnRadius = 10.0f;
    public float MaxSpeed = 0.2f;

    // Gizmo Settings
    public bool ShowAgents = true;
    public bool ShowVelocity = true;
    public bool ShowORCALines = false; // Only for debug agent
    public int DebugAgentIndex = 0;

    void Start()
    {
        InitializeSIMD();
        Debug.Log($"MinimalSIMDRVOTest Initialized with {AgentCount} agents.");
    }

    private void InitializeSIMD()
    {
        // Initialize SpatialIndexManager
        SpatialIndexManager.Instance.Initialize(AgentCount, 100); // 100x100 world
        
        _simSIMD = SIMDRVOSimulator.Instance;
        _simSIMD.Initialize(AgentCount);
        _simSIMD.TimeStep = TimeStep;
        
        _simSIMD.Radius = 0.5f;
        _simSIMD.MaxSpeed = MaxSpeed;
        _simSIMD.TimeHorizon = 2.0f;
        _simSIMD.TimeHorizonObst = 2.0f;
        _simSIMD.NeighborDist = 10.0f;
        _simSIMD.MaxNeighbors = 10;
        
        var obstacles = new List<RVOObstacle>();
        
        // 1. Central Box
        AddBoxObstacleSIMD(obstacles, new Vector2(0, 0), new Vector2(4, 4));
        
        // 2. Random blocks
        AddBoxObstacleSIMD(obstacles, new Vector2(-6, 4), new Vector2(2, 2));
        AddBoxObstacleSIMD(obstacles, new Vector2(6, -4), new Vector2(2, 2));
        
        // 3. Walls
        // obstacles.Add(new RVOObstacle(new float2(-15, -10), new float2(15, -10)));
        // obstacles.Add(new RVOObstacle(new float2(15, -10), new float2(15, 10)));
        // obstacles.Add(new RVOObstacle(new float2(15, 10), new float2(-15, 10)));
        // obstacles.Add(new RVOObstacle(new float2(-15, 10), new float2(-15, -10)));

        _simSIMD.UpdateObstacles(obstacles);
        
        // Agents in circle
        for (int i = 0; i < AgentCount; i++)
        {
            float angle = (i / (float)AgentCount) * Mathf.PI * 2;
            Vector2 spawnPos = new Vector2(Mathf.Cos(angle), Mathf.Sin(angle)) * SpawnRadius;
            Vector2 goal = -spawnPos;
            Vector2 dir = (goal - spawnPos).normalized * _simSIMD.MaxSpeed;
            
            _simSIMD.UpdateAgentData(i, spawnPos, Vector2.zero, dir);
        }
    }
    
    private void AddBoxObstacleSIMD(List<RVOObstacle> list, Vector2 center, Vector2 size)
    {
        Vector2 p1 = center + new Vector2(-size.x, -size.y) * 0.5f;
        Vector2 p2 = center + new Vector2(size.x, -size.y) * 0.5f;
        Vector2 p3 = center + new Vector2(size.x, size.y) * 0.5f;
        Vector2 p4 = center + new Vector2(-size.x, size.y) * 0.5f;
        
        list.Add(new RVOObstacle(new float2(p1.x, p1.y), new float2(p2.x, p2.y)));
        list.Add(new RVOObstacle(new float2(p2.x, p2.y), new float2(p3.x, p3.y)));
        list.Add(new RVOObstacle(new float2(p3.x, p3.y), new float2(p4.x, p4.y)));
        list.Add(new RVOObstacle(new float2(p4.x, p4.y), new float2(p1.x, p1.y)));
    }

    void Update()
    {
        if (_simSIMD == null) return;

        _simSIMD.Step(Time.deltaTime);
        
        float maxVel = 0f;
        int maxObs = 0;

        for (int i = 0; i < AgentCount; i++)
        {
            Vector2 posS = _simSIMD.GetAgentPosition(i);
            float angle = (i / (float)AgentCount) * Mathf.PI * 2;
            Vector2 spawnPos = new Vector2(Mathf.Cos(angle), Mathf.Sin(angle)) * SpawnRadius;
            Vector2 goal = -spawnPos;
            
            Vector2 dirS = (goal - posS).normalized * _simSIMD.MaxSpeed;
            if (Vector2.Distance(posS, goal) < 1.0f) dirS = Vector2.zero;
            _simSIMD.UpdateAgentData(i, posS, _simSIMD.GetNewVelocity(i), dirS);
            
            maxVel = Mathf.Max(maxVel, _simSIMD.GetNewVelocity(i).magnitude);
            maxObs = Mathf.Max(maxObs, _simSIMD.GetObstacleNeighborCount(i));
        }
        
        if (Time.frameCount % 60 == 0)
        {
            Debug.Log($"[MinimalSIMD] Max Vel: {maxVel:F2}, Max Obstacles: {maxObs}");
        }
    }

    void OnDrawGizmos()
    {
        if (_simSIMD == null) return;
        if (!Application.isPlaying) return;

        // Draw Obstacles
        Gizmos.color = Color.white;
        var obstacles = _simSIMD.GetObstacles();
        if (obstacles.IsCreated)
        {
            for (int i = 0; i < obstacles.Length; i++)
            {
                var obs = obstacles[i];
                Gizmos.DrawLine(new Vector3(obs.Point1.x, 0, obs.Point1.y), new Vector3(obs.Point2.x, 0, obs.Point2.y));
            }
        }

        // Draw Agents
        for (int i = 0; i < AgentCount; i++)
        {
            if (ShowAgents)
            {
                Vector2 posS = _simSIMD.GetAgentPosition(i);
                Gizmos.color = Color.red;
                Gizmos.DrawSphere(new Vector3(posS.x, 0, posS.y), 0.2f);
                
                if (ShowVelocity)
                {
                    Vector2 velS = _simSIMD.GetNewVelocity(i);
                    Gizmos.DrawLine(new Vector3(posS.x, 0, posS.y), new Vector3(posS.x + velS.x, 0, posS.y + velS.y));
                }
            }
        }

        // Debug ORCA Lines for one agent
        if (ShowORCALines && DebugAgentIndex >= 0 && DebugAgentIndex < AgentCount)
        {
            DrawDebugORCA(DebugAgentIndex);
        }
    }
    
    private void DrawDebugORCA(int agentIdx)
    {
        // Reconstruct RVO logic to visualize lines
        var agents = _simSIMD.GetAgents();
        var obstacles = _simSIMD.GetObstacles();
        var obstacleIndices = _simSIMD.GetObstacleNeighborIndices();
        int count = _simSIMD.GetObstacleNeighborCount(agentIdx);
        int offset = _simSIMD.GetObstacleNeighborOffset(agentIdx);
        
        var agent = agents[agentIdx];
        
        Gizmos.color = Color.yellow;
        Gizmos.DrawWireSphere(new Vector3(agent.Position.x, 0, agent.Position.y), agent.Radius);
        
        // Visualize Obstacle Neighbors
        for(int i=0; i<count; ++i)
        {
            int idx = obstacleIndices[offset + i];
            var obs = obstacles[idx];
            Gizmos.color = Color.cyan;
            Gizmos.DrawLine(new Vector3(obs.Point1.x, 0, obs.Point1.y), new Vector3(obs.Point2.x, 0, obs.Point2.y));
        }
        
        // Re-run single agent logic to get lines (Simplified version for Gizmos)
        // We can't easily execute the full Burst logic here, but we can visualize constraints if we assume logic is correct.
        // Or we can duplicate the critical logic:
        
        // ... (Debugging logic would go here, but for now let's just visualize inputs to confirm correct data is passed)
    }
    
    void OnDestroy()
    {
        _simSIMD?.Shutdown();
        SpatialIndexManager.Instance.Shutdown();
    }
}
