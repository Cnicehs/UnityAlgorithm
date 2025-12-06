using System.Collections.Generic;
using UnityEngine;
using RVO;
using Vector2 = UnityEngine.Vector2; // Ensure we use UnityEngine.Vector2
using Random = UnityEngine.Random;

public class RVO2Demo : MonoBehaviour
{
    public int AgentCount = 10;
    public float SpawnRadius = 5f;
    public float TargetDistance = 20f;
    public float AgentSpeed = 5f;
    public float AgentRadius = 0.5f;
    public float TimeHorizonObst = 0.5f;

    // Simulator is a singleton in this version
    
    private List<GameObject> _agentVisuals = new List<GameObject>();
    private List<Vector3> _targets = new List<Vector3>();
    private GameObject _obstacleVisual;
    private List<int> _agentIds = new List<int>();
    private List<int> _obstacleIds = new List<int>(); // The API returns int for obstacle too?
    // addObstacle returns "The number of the first vertex of the obstacle"

    void Start()
    {
        Debug.Log("RVO2Demo Start");

        // 1. Setup RVO Simulator
        Simulator.Instance.Clear();
        
        float neighborDist = 10.0f;
        int maxNeighbors = 10;
        float timeHorizon = 2.0f;
        
        Simulator.Instance.setTimeStep(Time.fixedDeltaTime);
        Simulator.Instance.setAgentDefaults(neighborDist, maxNeighbors, timeHorizon, TimeHorizonObst, AgentRadius, AgentSpeed, new RVO.Vector2(0, 0));

        Debug.Log($"RVO Settings: TimeHorizonObst={TimeHorizonObst}, Radius={AgentRadius}, MaxSpeed={AgentSpeed}");

        // 2. Create Obstacle (Simple Wall) - VISUAL
        float wallWidth = 20f;
        float wallThickness = 2f;
        float halfW = wallWidth * 0.5f;
        float halfT = wallThickness * 0.5f;

        _obstacleVisual = GameObject.CreatePrimitive(PrimitiveType.Cube);
        _obstacleVisual.name = "Wall";
        _obstacleVisual.transform.position = new Vector3(0, 0, 0);
        _obstacleVisual.transform.localScale = new Vector3(wallWidth, 2, wallThickness);
        _obstacleVisual.GetComponent<Renderer>().material.color = Color.red;
        Destroy(_obstacleVisual.GetComponent<Collider>());

        // RVO2 Obstacle - Counter-Clockwise
        // Mapping X -> x, Z -> y
        RVO.Vector2 pTL = new RVO.Vector2(-halfW, halfT); 
        RVO.Vector2 pTR = new RVO.Vector2(halfW, halfT); 
        RVO.Vector2 pBR = new RVO.Vector2(halfW, -halfT); 
        RVO.Vector2 pBL = new RVO.Vector2(-halfW, -halfT); 

        IList<RVO.Vector2> obstacleVertices = new List<RVO.Vector2>
        {
            pTL,
            pBL,
            pBR,
            pTR
        };

        int firstVertexId = Simulator.Instance.addObstacle(obstacleVertices);
        // We can store just the first vertex ID to track it if needed, or just iterate vertices later.
        // Simulator has getNumObstacleVertices()

        Simulator.Instance.processObstacles();

        Debug.Log($"Obstacles added. First Vertex ID: {firstVertexId}");

        // 3. Spawn Agents
        GameObject template = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
        Destroy(template.GetComponent<Collider>());

        for (int i = 0; i < AgentCount; i++)
        {
            Vector3 startPos = new Vector3(Random.Range(-5f, 5f), 0, -10f);
            Vector3 targetPos = new Vector3(startPos.x, 0, 10f);

            GameObject go = Instantiate(template, startPos, Quaternion.identity);
            go.name = $"Agent_{i}";
            go.transform.localScale = new Vector3(AgentRadius * 2, 1, AgentRadius * 2);
            go.GetComponent<Renderer>().material.color = Color.green;

            _agentVisuals.Add(go);
            _targets.Add(targetPos);

            int agentId = Simulator.Instance.addAgent(new RVO.Vector2(startPos.x, startPos.z));
            _agentIds.Add(agentId);
        }

        Destroy(template);

        Simulator.Instance.SetNumWorkers(4); // Use multi-threading if available

        Debug.Log($"Spawned {AgentCount} agents");
    }

    void FixedUpdate()
    {
        float dt = Time.fixedDeltaTime;
        Simulator.Instance.setTimeStep(dt);

        // Update Pref Velocities & Check Targets
        for (int i = 0; i < _agentIds.Count; i++)
        {
            int agentId = _agentIds[i];
            RVO.Vector2 pos2D = Simulator.Instance.getAgentPosition(agentId);
            Vector3 pos = new Vector3(pos2D.x_, 0, pos2D.y_);

            // Update Visuals
            _agentVisuals[i].transform.position = pos;

            Vector3 target = _targets[i];
            
            if (Vector3.Distance(pos, target) < 1.0f)
            {
                if (target.z > 0) _targets[i] = new Vector3(target.x, 0, -10f);
                else _targets[i] = new Vector3(target.x, 0, 10f);
                
                target = _targets[i];
            }

            Vector3 dir = (target - pos).normalized;
            RVO.Vector2 prefVel = new RVO.Vector2(dir.x, dir.z) * AgentSpeed;

            Simulator.Instance.setAgentPrefVelocity(agentId, prefVel);
        }

        // Step Simulation
        Simulator.Instance.doStep();
    }

    void OnDestroy()
    {
        Simulator.Instance.Clear();
    }

    void OnGUI()
    {
        GUILayout.BeginArea(new Rect(10, 10, 300, 200));
        GUILayout.Label("RVO2-Unity Demo");
        GUILayout.Label($"Agents: {Simulator.Instance.getNumAgents()}");
        GUILayout.Label($"Workers: {Simulator.Instance.GetNumWorkers()}");

        if (_agentIds.Count > 0)
        {
            int id0 = _agentIds[0];
            RVO.Vector2 agent0Pos = Simulator.Instance.getAgentPosition(id0);
            RVO.Vector2 agent0Vel = Simulator.Instance.getAgentVelocity(id0);
            GUILayout.Label($"Agent 0 Pos: ({agent0Pos.x_:F2}, {agent0Pos.y_:F2})");
            GUILayout.Label($"Agent 0 Vel: ({agent0Vel.x_:F2}, {agent0Vel.y_:F2})");
        }
        GUILayout.EndArea();
    }

    void OnDrawGizmos()
    {
        // Draw Obstacles
        Gizmos.color = Color.red;
        
        // This version of RVO stores obstacles as vertices.
        // We can iterate through vertices.
        // Simulator has internal obstacles_ list but it's internal.
        // But it exposes getObstacleVertex(i) and getNextObstacleVertexNo(i).
        // getNumObstacleVertices().
        
        int numVertices = Simulator.Instance.getNumObstacleVertices();
        // Since we cannot access internal obstacles, and there's no way to know which vertices form a loop unless we trace them.
        // But we can iterate all vertices and draw line to next.
        
        for (int i = 0; i < numVertices; i++)
        {
            RVO.Vector2 p1 = Simulator.Instance.getObstacleVertex(i);
            int nextIdx = Simulator.Instance.getNextObstacleVertexNo(i);
            RVO.Vector2 p2 = Simulator.Instance.getObstacleVertex(nextIdx);
            
            Gizmos.DrawLine(new Vector3(p1.x_, 0, p1.y_), new Vector3(p2.x_, 0, p2.y_));
        }

        // Draw Targets
        Gizmos.color = Color.cyan;
        foreach (var t in _targets)
        {
            Gizmos.DrawWireSphere(t, 0.5f);
        }
    }
}
