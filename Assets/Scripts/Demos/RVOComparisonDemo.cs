using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;
using Random = UnityEngine.Random;

public class RVOComparisonDemo : ComparisonDemoBase
{
    [Header("RVO Settings")]
    public float WorldSize = 50f;
    public float AgentRadius = 0.5f;
    public float AgentSpeed = 2.0f;
    public int SpawnCount = 100;
    
    [Header("Debug")]
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
        AgentCount = SpawnCount;
        
        // Initialize Managers
        SpatialIndexManager.Instance.StructureType = SpatialIndexType;
        SpatialIndexManager.Instance.GridResolution = GridResolution;
        SpatialIndexManager.Instance.Initialize(AgentCount, WorldSize);
        
        RVOSimulator.Instance.Radius = AgentRadius;
        RVOSimulator.Instance.MaxSpeed = AgentSpeed;
        RVOSimulator.Instance.NeighborDist = AgentRadius * 4; 

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
    
    protected override void OnSIMDChanged(bool useSIMD)
    {
        // When switching, we can try to sync state, but for simplicity we rely on the continuous updates loop
        // to pick up from where it was (roughly).
        // Or re-spawn.
    }

    void Update()
    {
        float dt = Time.deltaTime;

        if (UseSIMD)
        {
            // Update Spatial Index with current positions BEFORE RVO step
            SpatialIndexManager.Instance.UpdatePositions(_positions);
            
            SIMDRVOSimulator.Instance.Step(dt);
            ExecutionTimeMs = SIMDRVOSimulator.Instance.TotalStepTimeMs;

            for (int i = 0; i < AgentCount; i++)
            {
                Vector2 vel = SIMDRVOSimulator.Instance.GetNewVelocity(i);
                Vector2 pos = _positions[i] + vel * dt;
                _positions[i] = pos;
                
                // Recalculate pref velocity
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
            for (int i = 0; i < AgentCount; i++)
            {
                _positions[i] = RVOSimulator.Instance.GetAgentPosition(i);
            }
            SpatialIndexManager.Instance.UpdatePositions(_positions);
            
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
            ExecutionTimeMs = RVOSimulator.Instance.TotalStepTimeMs;

            for (int i = 0; i < AgentCount; i++)
            {
                Vector2 pos = RVOSimulator.Instance.GetAgentPosition(i);
                _positions[i] = pos; 
                _agentVisuals[i].transform.position = new Vector3(pos.x, 0, pos.y);
            }
        }
        
        // Debug...
        if (ShowDebugGizmos && _positions.Count > 0)
        {
            _debugNeighbors.Clear();
            SpatialIndexManager.Instance.GetNeighborsInRadius(_positions[_debugAgentIndex], AgentRadius * 4, _debugNeighbors);
        }
    }

    protected override void OnGUIDisplayExtra()
    {
        if (UseSIMD)
        {
            GUILayout.Label($"Neighbor Query: {SIMDRVOSimulator.Instance.NeighborQueryTimeMs:F3} ms");
            GUILayout.Label($"RVO Compute: {SIMDRVOSimulator.Instance.RVOComputeTimeMs:F3} ms");
        }
        else
        {
            GUILayout.Label($"Neighbor Query: {RVOSimulator.Instance.NeighborQueryTimeMs:F3} ms");
            GUILayout.Label($"RVO Compute: {RVOSimulator.Instance.RVOComputeTimeMs:F3} ms");
            GUILayout.Label($"Pos Update: {RVOSimulator.Instance.PositionUpdateTimeMs:F3} ms");
        }
        GUILayout.Label($"Spatial Build: {SpatialIndexManager.Instance.BuildTimeMs:F3} ms");
    }

    private void OnDrawGizmos()
    {
       if (!ShowDebugGizmos || _positions.Count == 0) return;
        
        Gizmos.color = Color.green;
        Vector3 debugPos = new Vector3(_positions[_debugAgentIndex].x, 0.5f, _positions[_debugAgentIndex].y);
        Gizmos.DrawWireSphere(debugPos, AgentRadius * 1.5f);
        
        Gizmos.color = Color.yellow;
        Gizmos.DrawWireSphere(debugPos, AgentRadius * 4);
        
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

    private void OnDestroy()
    {
        SIMDRVOSimulator.Instance.Shutdown();
        SpatialIndexManager.Instance.Shutdown();
    }
}
