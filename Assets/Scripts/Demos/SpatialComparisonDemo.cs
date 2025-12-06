using System.Collections.Generic;
using UnityEngine;
using Cysharp.Threading.Tasks;
using System.Diagnostics;
using Debug = UnityEngine.Debug;
using System;
using Random = UnityEngine.Random;

public class SpatialComparisonDemo : ComparisonDemoBase
{
    [Header("Spatial Settings")]
    public float WorldSize = 100f;
    public float UnitSpeed = 5f;
    public int SpawnCount = 5000;
    
    [Header("Query Settings")]
    public bool PerformQueries = true;
    public int QueryK = 5;
    public bool ShowGizmos = true;

    private List<Unit> _units = new List<Unit>();
    private List<Vector2> _positions = new List<Vector2>();
    
    // Query results for visualization (Hero)
    private List<int> _heroQueryResults = new List<int>();

    void Start()
    {
        AgentCount = SpawnCount;
        SpawnUnits();
        
        // Initialize SpatialIndexManager
        SpatialIndexManager.Instance.Initialize(AgentCount, WorldSize);
        UpdateStructureType();
    }

    void Update()
    {
        // 1. Update Unit Positions
        MoveUnits();
        
        // 2. Update Spatial Index
        // We pass the current positions to the manager
        SpatialIndexManager.Instance.UpdatePositions(_positions);
        
        // 3. Perform Queries
        if (PerformQueries && _units.Count > 0)
        {
            Stopwatch.Restart();
            PerformQuery();
            Stopwatch.Stop();
            ExecutionTimeMs = (float)Stopwatch.Elapsed.TotalMilliseconds;
        }
    }

    protected override void OnSIMDChanged(bool useSIMD)
    {
        UpdateStructureType();
    }

    private void UpdateStructureType()
    {
        if (UseSIMD)
        {
            // Defaulting to QuadTree for comparison, could add a dropdown for others
            SpatialIndexManager.Instance.StructureType = SpatialStructureType.SIMDQuadTree;
        }
        else
        {
            SpatialIndexManager.Instance.StructureType = SpatialStructureType.QuadTree;
        }
    }

    private void SpawnUnits()
    {
        foreach (var unit in _units)
        {
            if (unit != null) Destroy(unit.gameObject);
        }
        _units.Clear();
        _positions.Clear();

        GameObject template = GameObject.CreatePrimitive(PrimitiveType.Quad);
        Destroy(template.GetComponent<Collider>());
        
        for (int i = 0; i < AgentCount; i++)
        {
            Vector2 pos = new Vector2(Random.Range(-WorldSize/2, WorldSize/2), Random.Range(-WorldSize/2, WorldSize/2));
            GameObject go = Instantiate(template, pos, Quaternion.identity);
            go.transform.SetParent(transform);
            go.name = $"Unit_{i}";
            
            Unit unit = go.AddComponent<Unit>();
            unit.Position = pos;
            unit.Velocity = Random.insideUnitCircle.normalized * UnitSpeed;
            unit.Bounds = new Rect(-WorldSize/2, -WorldSize/2, WorldSize, WorldSize);
            
            _units.Add(unit);
            _positions.Add(pos);
        }
        
        Destroy(template);
    }

    private void MoveUnits()
    {
        float dt = Time.deltaTime;
        for (int i = 0; i < _units.Count; i++)
        {
            _units[i].Move(dt);
            _positions[i] = _units[i].Position;
            _units[i].transform.position = _units[i].Position;
        }
    }

    private void PerformQuery()
    {
        _heroQueryResults.Clear();
        Vector2 heroPos = _positions[0];
        
        SpatialIndexManager.Instance.QueryKNearest(heroPos, QueryK, _heroQueryResults);
    }

    protected override void OnGUIDisplayExtra()
    {
        GUILayout.Label($"Structure: {SpatialIndexManager.Instance.StructureType}");
        GUILayout.Label($"Multi-Threading: {(SpatialIndexManager.Instance.UseMultiThreading ? "ON" : "OFF")}");
        GUILayout.Label($"Build Time: {SpatialIndexManager.Instance.BuildTimeMs:F2} ms {(SpatialIndexManager.Instance.IsBuilding ? "(Building...)" : "")}");
        GUILayout.Label($"Query Time (K={QueryK}): {ExecutionTimeMs:F2} ms");
    }

    private void OnDrawGizmos()
    {
        if (!ShowGizmos || _units.Count == 0 || _heroQueryResults == null) return;

        // Draw Hero
        Gizmos.color = Color.green;
        if (_positions.Count > 0)
        {
            Vector3 heroPos = _positions[0];
            Gizmos.DrawWireSphere(heroPos, 1f);

            // Draw Lines to Neighbors
            Gizmos.color = Color.red;
            foreach (int idx in _heroQueryResults)
            {
                if (idx < _positions.Count)
                {
                    Gizmos.DrawLine(heroPos, _positions[idx]);
                    Gizmos.DrawWireSphere(_positions[idx], 0.5f);
                }
            }
        }
    }
}
