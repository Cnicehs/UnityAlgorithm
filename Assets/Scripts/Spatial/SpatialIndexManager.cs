using System.Collections.Generic;
using UnityEngine;
using System.Diagnostics;
using Debug = UnityEngine.Debug;

public enum SpatialStructureType
{
    Grid,
    KDTree
}

public class SpatialIndexManager : MonoBehaviour
{
    [Header("Settings")]
    public SpatialStructureType StructureType = SpatialStructureType.Grid;
    public int UnitCount = 10000;
    public float WorldSize = 100f;
    public float UnitSpeed = 5f;
    
    [Header("Grid Settings")]
    public int GridResolution = 40; // Optimized: ~6 units per cell for 10k units
    
    [Header("Query Settings")]
    public bool PerformQueries = true;
    public int QueryK = 5;
    public float QueryRadius = 10f;
    public bool ShowGizmos = true;

    [Header("Performance")]
    public float BuildTimeMs;
    public float QueryTimeMs;

    private List<Unit> _units = new List<Unit>();
    private List<Vector2> _positions = new List<Vector2>();
    private ISpatialIndex _spatialIndex;
    private Stopwatch _stopwatch = new Stopwatch();

    // Query results for visualization (just for the first unit as 'Hero')
    private List<int> _heroQueryResults = new List<int>();

    void Start()
    {
        SpawnUnits();
    }

    void Update()
    {
        // 1. Update Unit Positions
        MoveUnits();

        // 2. Rebuild Index
        RebuildIndex();

        // 3. Perform Queries
        if (PerformQueries && _units.Count > 0)
        {
            PerformDemoQueries();
        }
    }

    private void SpawnUnits()
    {
        // Clear existing
        foreach (var unit in _units)
        {
            Destroy(unit.gameObject);
        }
        _units.Clear();
        _positions.Clear();

        // Create template if needed, or just primitives
        GameObject template = GameObject.CreatePrimitive(PrimitiveType.Quad);
        Destroy(template.GetComponent<Collider>()); // Remove collider for performance
        
        for (int i = 0; i < UnitCount; i++)
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

    private void RebuildIndex()
    {
        _stopwatch.Restart();

        if (StructureType == SpatialStructureType.Grid)
        {
            if (_spatialIndex is not SpatialGridIndex)
            {
                // Create new grid
                // Origin is bottom-left
                Vector2 origin = new Vector2(-WorldSize/2, -WorldSize/2);
                float cellSize = WorldSize / GridResolution;
                _spatialIndex = new SpatialGridIndex(GridResolution, GridResolution, cellSize, origin);
            }
        }
        else
        {
            if (_spatialIndex is not KDTreeIndex)
            {
                _spatialIndex = new KDTreeIndex(_units.Count);
            }
        }

        _spatialIndex.Build(_positions);

        _stopwatch.Stop();
        BuildTimeMs = (float)_stopwatch.Elapsed.TotalMilliseconds;
    }

    private void PerformDemoQueries()
    {
        _stopwatch.Restart();

        // Query for the first unit (Hero)
        _heroQueryResults.Clear();
        _spatialIndex.QueryKNearest(_positions[0], QueryK, _heroQueryResults);

        _stopwatch.Stop();
        QueryTimeMs = (float)_stopwatch.Elapsed.TotalMilliseconds;
    }

    private void OnGUI()
    {
        GUILayout.BeginArea(new Rect(10, 10, 300, 200));
        GUILayout.Label($"Units: {UnitCount}");
        GUILayout.Label($"Structure: {StructureType}");
        GUILayout.Label($"Build Time: {BuildTimeMs:F2} ms");
        GUILayout.Label($"Query Time (1 unit): {QueryTimeMs:F4} ms");
        GUILayout.Label($"FPS: {1.0f/Time.deltaTime:F1}");
        GUILayout.EndArea();
    }

    private void OnDrawGizmos()
    {
        if (!ShowGizmos || _units.Count == 0 || _heroQueryResults.Count == 0) return;

        // Draw Hero
        Gizmos.color = Color.green;
        Vector3 heroPos = _units[0].Position;
        Gizmos.DrawWireSphere(heroPos, 1f);

        // Draw Lines to Neighbors
        Gizmos.color = Color.red;
        foreach (int idx in _heroQueryResults)
        {
            Gizmos.DrawLine(heroPos, _units[idx].Position);
            Gizmos.DrawWireSphere(_units[idx].Position, 0.5f);
        }
    }
}
