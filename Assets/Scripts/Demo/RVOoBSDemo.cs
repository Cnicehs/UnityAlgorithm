using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;
using Random = UnityEngine.Random;

public class RVOoBSDemo : MonoBehaviour
{
    public int AgentCount = 10;
    public float SpawnRadius = 5f;
    public float TargetDistance = 20f;
    public float AgentSpeed = 5f;
    public float AgentRadius = 0.5f;
    public float TimeHorizonObst = 0.5f; // REDUCED from 2.0 - closer horizon = stronger obstacle avoidance

    private List<GameObject> _agents = new List<GameObject>();
    private List<Vector3> _targets = new List<Vector3>();
    private GameObject _obstacleVisual;

    void Start()
    {
        Debug.Log("RVOoBSDemo Start");

        // 1. Setup RVO Simulator
        RVOSimulator.Instance.ClearAgents();
        RVOSimulator.Instance.ClearObstacles();

        RVOSimulator.Instance.NeighborDist = 10.0f;
        RVOSimulator.Instance.MaxNeighbors = 10;
        RVOSimulator.Instance.TimeHorizon = 2.0f;
        RVOSimulator.Instance.TimeHorizonObst = TimeHorizonObst;
        RVOSimulator.Instance.Radius = AgentRadius;
        RVOSimulator.Instance.MaxSpeed = AgentSpeed;

        Debug.Log($"RVO Settings: TimeHorizonObst={TimeHorizonObst}, Radius={AgentRadius}, MaxSpeed={AgentSpeed}");

        // 2. Create Obstacle (Simple Wall) - VISUAL
        float wallWidth = 20f;
        float wallThickness = 2f;
        float halfW = wallWidth * 0.5f;
        float halfT = wallThickness * 0.5f;

        Vector2 pTL = new Vector2(-halfW, halfT);
        Vector2 pTR = new Vector2(halfW, halfT);
        Vector2 pBR = new Vector2(halfW, -halfT);
        Vector2 pBL = new Vector2(-halfW, -halfT);

        // Create visual obstacle
        _obstacleVisual = GameObject.CreatePrimitive(PrimitiveType.Cube);
        _obstacleVisual.name = "Wall";
        _obstacleVisual.transform.position = new Vector3(0, 0, 0);
        _obstacleVisual.transform.localScale = new Vector3(wallWidth, 2, wallThickness);
        _obstacleVisual.GetComponent<Renderer>().material.color = Color.red;
        Destroy(_obstacleVisual.GetComponent<Collider>()); // Remove physics collider

        // Add to RVO simulator - COUNTER-CLOCKWISE order (RVO2 requirement)
        // CCW ensures -obstacle.Direction creates correct blocking constraints
        RVOSimulator.Instance.AddObstacle(new Vector3(pTL.x, 0, pTL.y), new Vector3(pBL.x, 0, pBL.y)); // Left
        RVOSimulator.Instance.AddObstacle(new Vector3(pBL.x, 0, pBL.y), new Vector3(pBR.x, 0, pBR.y)); // Bottom
        RVOSimulator.Instance.AddObstacle(new Vector3(pBR.x, 0, pBR.y), new Vector3(pTR.x, 0, pTR.y)); // Right
        RVOSimulator.Instance.AddObstacle(new Vector3(pTR.x, 0, pTR.y), new Vector3(pTL.x, 0, pTL.y)); // Top

        // 2.1 Create Concave "L-Shape" Obstacle at X=15
        // Defines a solid L-shaped block.
        // Bounding Box: [15, 25] x [0, 10]
        // Empty Pocket (Concave area): [20, 25] x [5, 10] (Top-Right quadrant is empty)
        // Vertices (CCW):
        Vector3[] lShape = new Vector3[] {
            new Vector3(15, 0, 0),  // Bottom-Left
            new Vector3(25, 0, 0),  // Bottom-Right
            new Vector3(25, 0, 5),  // Right-Mid
            new Vector3(20, 0, 5),  // Inner Corner (Reflex Vertex)
            new Vector3(20, 0, 10), // Top-Mid
            new Vector3(15, 0, 10)  // Top-Left
        };

        for (int i = 0; i < lShape.Length; i++)
        {
            Vector3 p1 = lShape[i];
            Vector3 p2 = lShape[(i + 1) % lShape.Length];
            RVOSimulator.Instance.AddObstacle(p1, p2);
        }

        RVOSimulator.Instance.ProcessObstacles();

        Debug.Log($"Obstacles added: {RVOSimulator.Instance.GetObstacles().Count}");

        // 3. Initialize Spatial Index (CRITICAL for Agent Avoidance)
        SpatialIndexManager.Instance.StructureType = SpatialStructureType.SIMDQuadTree;
        SpatialIndexManager.Instance.Initialize(AgentCount, 100f); // Size 100

        Debug.Log("SpatialIndexManager initialized");

        // 4. Spawn Agents
        GameObject template = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
        Destroy(template.GetComponent<Collider>());

        for (int i = 0; i < AgentCount; i++)
        {
            // Spawn at top (Random X, +10) and move downward to -10 (through the wall)
            Vector3 startPos = new Vector3(Random.Range(-5f, 5f), 0, 10f);
            Vector3 targetPos = new Vector3(startPos.x, 0, -10f); // Move downward through the wall

            GameObject go = Instantiate(template, startPos, Quaternion.identity);
            go.name = $"Agent_{i}";
            go.transform.localScale = new Vector3(AgentRadius * 2, 1, AgentRadius * 2);
            go.GetComponent<Renderer>().material.color = Color.green;

            _agents.Add(go);
            _targets.Add(targetPos);

            RVOSimulator.Instance.AddAgent(startPos);
        }

        // Spawn extra agents for L-Shape test
        for (int i = 0; i < 5; i++)
        {
            // Spawn inside the "Empty Pocket" of the L-Shape (Top-Right)
            // X in [20, 25], Z in [5, 10]
            Vector3 startPos = new Vector3(22f + Random.Range(-1f, 1f), 0, 8f); 
            
            // Target at Bottom-Left (10, 0), requires navigating around the Reflex Corner at (20, 5)
            Vector3 targetPos = new Vector3(10f, 0, 0f); 

            GameObject go = Instantiate(template, startPos, Quaternion.identity);
            go.name = $"Agent_L_{i}";
            go.transform.localScale = new Vector3(AgentRadius * 2, 1, AgentRadius * 2);
            go.GetComponent<Renderer>().material.color = Color.blue;

            _agents.Add(go);
            _targets.Add(targetPos);

            RVOSimulator.Instance.AddAgent(startPos);
        }

        Destroy(template);

        Debug.Log($"Spawned {AgentCount} agents");
        Debug.Log($"RVO Agent Count: {RVOSimulator.Instance.GetAgentCount()}");
    }

    void FixedUpdate()
    {
        float dt = Time.fixedDeltaTime;

        // Update Pref Velocities
        for (int i = 0; i < _agents.Count; i++)
        {
            Vector3 pos = RVOSimulator.Instance.GetAgentPosition(i);
            Vector3 target = _targets[i];
            Vector3 dir = (target - pos).normalized;

            // Simple direct movement
            RVOSimulator.Instance.SetAgentPrefVelocity(i, dir * AgentSpeed);
        }

        // Step Simulation
        RVOSimulator.Instance.Step(dt);

        // Update Transforms
        for (int i = 0; i < _agents.Count; i++)
        {
            Vector3 pos = RVOSimulator.Instance.GetAgentPosition(i);
            _agents[i].transform.position = pos;

            // Check if reached target
            if (Vector3.Distance(pos, _targets[i]) < 1.0f)
            {
                // Reverse target
                if (_targets[i].z > 0) _targets[i] = new Vector3(_targets[i].x, 0, -10f);
                else _targets[i] = new Vector3(_targets[i].x, 0, 10f);
            }
        }
    }

    void OnGUI()
    {
        GUILayout.BeginArea(new Rect(10, 10, 300, 200));
        GUILayout.Label("RVO Obstacle Demo");
        GUILayout.Label($"Agents: {RVOSimulator.Instance.GetAgentCount()}");
        GUILayout.Label($"Obstacles: {RVOSimulator.Instance.GetObstacles().Count}");

        if (_agents.Count > 0)
        {
            Vector3 agent0Pos = RVOSimulator.Instance.GetAgentPosition(0);
            Vector3 agent0Vel = RVOSimulator.Instance.GetAgentVelocity(0);
            GUILayout.Label($"Agent 0 Pos: ({agent0Pos.x:F2}, {agent0Pos.z:F2})");
            GUILayout.Label($"Agent 0 Vel: ({agent0Vel.x:F2}, {agent0Vel.z:F2})");
        }

        GUILayout.EndArea();
    }

    void OnDrawGizmos()
    {
        // Draw Obstacles
        Gizmos.color = Color.red;
        var obstacles = RVOSimulator.Instance.GetObstacles();
        if (obstacles != null)
        {
            foreach (var obs in obstacles)
            {
                Vector3 p1 = new Vector3(obs.Point1.x, 0, obs.Point1.y);
                Vector3 p2 = new Vector3(obs.Point2.x, 0, obs.Point2.y);
                Gizmos.DrawLine(p1, p2);

                // Normal
                Vector3 mid = (p1 + p2) * 0.5f;
                Vector3 norm = new Vector3(obs.Normal.x, 0, obs.Normal.y);
                Gizmos.color = Color.yellow;
                Gizmos.DrawLine(mid, mid + norm * 0.5f);
                Gizmos.color = Color.red;
            }
        }

        // Draw Targets
        Gizmos.color = Color.cyan;
        foreach (var t in _targets)
        {
            Gizmos.DrawWireSphere(t, 0.5f);
        }
    }
}
