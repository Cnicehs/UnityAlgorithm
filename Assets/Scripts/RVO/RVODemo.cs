using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;
using Random = UnityEngine.Random;

public class RVODemo : MonoBehaviour
{
    public bool UseSIMD = true;
    public int AgentCount = 100;
    public float WorldSize = 50f;
    public float AgentRadius = 0.5f;
    public float AgentSpeed = 2.0f;
    public bool ShowDebugGizmos = false;
    public SpatialStructureType SpatialIndexType = SpatialStructureType.SIMDKDTree;
    [Range(1, 200)]
    public int GridResolution = 40;
    
    private List<GameObject> _agentVisuals = new List<GameObject>();
    private List<Vector2> _positions = new List<Vector2>();
    private int _debugAgentIndex = 0;
    private List<int> _debugNeighbors = new List<int>();

    void Start()
    {
        // Initialize Managers
        SpatialIndexManager.Instance.StructureType = SpatialIndexType;
        SpatialIndexManager.Instance.GridResolution = GridResolution;
        SpatialIndexManager.Instance.Initialize(AgentCount, WorldSize);
        
        RVOSimulator.Instance.Radius = AgentRadius;
        RVOSimulator.Instance.MaxSpeed = AgentSpeed;
        RVOSimulator.Instance.NeighborDist = AgentRadius * 4; // Increase neighbor detection range

        SIMDRVOSimulator.Instance.Radius = AgentRadius;
        SIMDRVOSimulator.Instance.MaxSpeed = AgentSpeed;
        SIMDRVOSimulator.Instance.NeighborDist = AgentRadius * 4;

        SpawnAgents();
    }

    void SpawnAgents()
    {
        foreach (var go in _agentVisuals) Destroy(go);
        _agentVisuals.Clear();
        _positions.Clear();
        RVOSimulator.Instance.ClearAgents();

        GameObject template = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
        Destroy(template.GetComponent<Collider>());
        template.transform.localScale = new Vector3(AgentRadius * 2, 0.1f, AgentRadius * 2);

        for (int i = 0; i < AgentCount; i++)
        {
            // Circle arrangement
            float angle = (float)i / AgentCount * 2 * Mathf.PI;
            Vector2 pos = new Vector2(Mathf.Cos(angle), Mathf.Sin(angle)) * (WorldSize * 0.4f);
            Vector2 target = -pos; // Move to opposite side

            GameObject go = Instantiate(template, new Vector3(pos.x, 0, pos.y), Quaternion.identity);
            go.name = $"Agent_{i}";
            go.transform.SetParent(transform);
            _agentVisuals.Add(go);
            _positions.Add(pos);

            // Standard Setup
            RVOSimulator.Instance.AddAgent(pos);
            RVOSimulator.Instance.SetAgentPrefVelocity(i, (target - pos).normalized * AgentSpeed);
        }

        // SIMD Setup
        SIMDRVOSimulator.Instance.Initialize(AgentCount);
        for (int i = 0; i < AgentCount; i++)
        {
            Vector2 pos = _positions[i];
            Vector2 target = -pos;
            SIMDRVOSimulator.Instance.UpdateAgentData(i, pos, Vector2.zero, (target - pos).normalized * AgentSpeed);
        }

        Destroy(template);
    }

    void Update()
    {
        // Sync Inspector changes to Manager
        if (SpatialIndexManager.Instance.StructureType != SpatialIndexType)
        {
            SpatialIndexManager.Instance.StructureType = SpatialIndexType;
        }
        if (SpatialIndexManager.Instance.GridResolution != GridResolution)
        {
            SpatialIndexManager.Instance.GridResolution = GridResolution;
        }

        float dt = Time.deltaTime;

        if (UseSIMD)
        {
            // Update Spatial Index with current positions BEFORE RVO step
            SpatialIndexManager.Instance.UpdatePositions(_positions);
            
            SIMDRVOSimulator.Instance.Step(dt);
            for (int i = 0; i < AgentCount; i++)
            {
                Vector2 vel = SIMDRVOSimulator.Instance.GetNewVelocity(i);
                Vector2 pos = _positions[i] + vel * dt;
                _positions[i] = pos;
                
                // Update SIMD data for next frame
                // Recalculate pref velocity (simple target seeking)
                Vector2 startPos = new Vector2(Mathf.Cos((float)i / AgentCount * 2 * Mathf.PI), Mathf.Sin((float)i / AgentCount * 2 * Mathf.PI)) * (WorldSize * 0.4f);
                Vector2 goal = -startPos;
                
                Vector2 toGoal = goal - pos;
                float dist = toGoal.magnitude;
                Vector2 prefVel = dist > 1.0f ? toGoal.normalized * AgentSpeed : Vector2.zero;

                SIMDRVOSimulator.Instance.UpdateAgentData(i, pos, vel, prefVel);
                
                // Visuals
                _agentVisuals[i].transform.position = new Vector3(pos.x, 0, pos.y);
            }
        }
        else
        {
            // Sync positions from RVOSimulator to _positions list for spatial index
            for (int i = 0; i < AgentCount; i++)
            {
                _positions[i] = RVOSimulator.Instance.GetAgentPosition(i);
            }
            
            // Update Spatial Index with current positions BEFORE RVO step
            SpatialIndexManager.Instance.UpdatePositions(_positions);
            
            // Update Standard Pref Velocities
            for (int i = 0; i < AgentCount; i++)
            {
                Vector2 pos = RVOSimulator.Instance.GetAgentPosition(i);
                Vector2 startPos = new Vector2(Mathf.Cos((float)i / AgentCount * 2 * Mathf.PI), Mathf.Sin((float)i / AgentCount * 2 * Mathf.PI)) * (WorldSize * 0.4f);
                Vector2 goal = -startPos;
                Vector2 toGoal = goal - pos;
                float dist = toGoal.magnitude;
                Vector2 prefVel = dist > 1.0f ? toGoal.normalized * AgentSpeed : Vector2.zero;
                RVOSimulator.Instance.SetAgentPrefVelocity(i, prefVel);
            }

            RVOSimulator.Instance.Step(dt);

            for (int i = 0; i < AgentCount; i++)
            {
                Vector2 pos = RVOSimulator.Instance.GetAgentPosition(i);
                _positions[i] = pos; // Sync positions for next frame
                _agentVisuals[i].transform.position = new Vector3(pos.x, 0, pos.y);
            }
        }
        
        // Debug: Query neighbors for first agent
        if (ShowDebugGizmos && _positions.Count > 0)
        {
            _debugNeighbors.Clear();
            SpatialIndexManager.Instance.GetNeighborsInRadius(_positions[_debugAgentIndex], AgentRadius * 4, _debugNeighbors);
        }
    }

    private void OnDrawGizmos()
    {
        if (!ShowDebugGizmos || _positions.Count == 0) return;
        
        // Draw debug agent in green
        Gizmos.color = Color.green;
        Vector3 debugPos = new Vector3(_positions[_debugAgentIndex].x, 0.5f, _positions[_debugAgentIndex].y);
        Gizmos.DrawWireSphere(debugPos, AgentRadius * 1.5f);
        
        // Draw neighbor detection radius
        Gizmos.color = Color.yellow;
        Gizmos.DrawWireSphere(debugPos, AgentRadius * 4);
        
        // Draw neighbors in red
        Gizmos.color = Color.red;
        foreach (int idx in _debugNeighbors)
        {
            if (idx != _debugAgentIndex && idx < _positions.Count)
            {
                Vector3 neighborPos = new Vector3(_positions[idx].x, 0.5f, _positions[idx].y);
                Gizmos.DrawLine(debugPos, neighborPos);
                Gizmos.DrawWireSphere(neighborPos, AgentRadius);
            }
        }
    }

    private void OnGUI()
    {
        GUILayout.BeginArea(new Rect(10, 10, 450, 400));
        GUILayout.Label($"RVO Demo - {(UseSIMD ? "SIMD" : "Standard")} Mode");
        GUILayout.Label($"Agents: {AgentCount}");
        
        GUILayout.BeginHorizontal();
        GUILayout.Label($"Spatial Index: {SpatialIndexManager.Instance.StructureType}", GUILayout.Width(250));
        if (GUILayout.Button("Change", GUILayout.Width(60)))
        {
            var types = (SpatialStructureType[])System.Enum.GetValues(typeof(SpatialStructureType));
            int current = System.Array.IndexOf(types, SpatialIndexManager.Instance.StructureType);
            int next = (current + 1) % types.Length;
            SpatialIndexManager.Instance.StructureType = types[next];
            SpatialIndexType = types[next];
        }
        GUILayout.EndHorizontal();

        if (SpatialIndexManager.Instance.StructureType == SpatialStructureType.Grid ||
            SpatialIndexManager.Instance.StructureType == SpatialStructureType.SIMDHashGrid)
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label($"Grid Res: {SpatialIndexManager.Instance.GridResolution}", GUILayout.Width(100));
            float newRes = GUILayout.HorizontalSlider(SpatialIndexManager.Instance.GridResolution, 10, 200);
            if ((int)newRes != SpatialIndexManager.Instance.GridResolution)
            {
                SpatialIndexManager.Instance.GridResolution = (int)newRes;
                GridResolution = (int)newRes;
            }
            GUILayout.EndHorizontal();
        }

        GUILayout.Label($"FPS: {1.0f/Time.smoothDeltaTime:F1}");
        
        GUILayout.Space(10);
        GUILayout.Label("=== Spatial Index Performance ===");
        GUILayout.Label($"Build Time: {SpatialIndexManager.Instance.BuildTimeMs:F3} ms");
        GUILayout.Label($"Building: {(SpatialIndexManager.Instance.IsBuilding ? "Yes" : "No")}");
        
        GUILayout.Space(10);
        if (UseSIMD)
        {
            GUILayout.Label("=== SIMD RVO Performance ===");
            GUILayout.Label($"Neighbor Query: {SIMDRVOSimulator.Instance.NeighborQueryTimeMs:F3} ms");
            GUILayout.Label($"RVO Compute (Burst): {SIMDRVOSimulator.Instance.RVOComputeTimeMs:F3} ms");
            GUILayout.Label($"Total Step: {SIMDRVOSimulator.Instance.TotalStepTimeMs:F3} ms");
        }
        else
        {
            GUILayout.Label("=== Standard RVO Performance ===");
            GUILayout.Label($"Neighbor Query: {RVOSimulator.Instance.NeighborQueryTimeMs:F3} ms");
            GUILayout.Label($"RVO Compute: {RVOSimulator.Instance.RVOComputeTimeMs:F3} ms");
            GUILayout.Label($"Position Update: {RVOSimulator.Instance.PositionUpdateTimeMs:F3} ms");
            GUILayout.Label($"Total Step: {RVOSimulator.Instance.TotalStepTimeMs:F3} ms");
        }
        
        if (ShowDebugGizmos)
        {
            GUILayout.Space(10);
            GUILayout.Label($"Debug Agent: {_debugAgentIndex}");
            GUILayout.Label($"Neighbors: {_debugNeighbors.Count}");
        }
        GUILayout.EndArea();
    }

    private void OnDestroy()
    {
        SIMDRVOSimulator.Instance.Shutdown();
        SpatialIndexManager.Instance.Shutdown();
    }
}
