using System.Collections.Generic;
using UnityEngine;
using Cysharp.Threading.Tasks;
using System.Diagnostics;
using Debug = UnityEngine.Debug;

public enum SpatialStructureType
{
    Grid,
    KDTree,
    BVH,
    QuadTree,
    SIMDHashGrid
}

public class SpatialIndexManager : MonoBehaviour
{
    [Header("Settings")]
    public SpatialStructureType StructureType = SpatialStructureType.Grid;
    public int UnitCount = 10000;
    public float WorldSize = 100f;
    public float UnitSpeed = 5f;
    public bool UseMultiThreading = true; // Toggle for multi-threading mode
    
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
    public bool IsBuilding;
    public bool IsQuerying;

    private List<Unit> _units = new List<Unit>();
    
    // Double buffering for thread safety
    // _mainThreadPositions: Updated by Update() for rendering/logic
    // _asyncPositions: Copied from _mainThreadPositions for background tasks
    private List<Vector2> _mainThreadPositions = new List<Vector2>();
    private List<Vector2> _asyncPositions = new List<Vector2>();
    
    private ISpatialIndex _spatialIndex;
    
    // Query results for visualization (Hero)
    private List<int> _heroQueryResults = new List<int>();

    void Start()
    {
        SpawnUnits();
        // Start async build loop (independent background task)
        BuildLoop().Forget();
    }

    void Update()
    {
        // 1. Update Unit Positions (Main Thread)
        MoveUnits();
        
        // 2. Perform Queries (Main Thread - fast, no threading needed)
        if (PerformQueries && _spatialIndex != null && _units.Count > 0)
        {
            PerformQuery();
        }
    }

    private void SpawnUnits()
    {
        foreach (var unit in _units)
        {
            if (unit != null) Destroy(unit.gameObject);
        }
        _units.Clear();
        _mainThreadPositions.Clear();

        GameObject template = GameObject.CreatePrimitive(PrimitiveType.Quad);
        Destroy(template.GetComponent<Collider>());
        
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
            _mainThreadPositions.Add(pos);
        }
        
        Destroy(template);
        
        // Initialize async buffer
        _asyncPositions = new List<Vector2>(_mainThreadPositions);
    }

    private void MoveUnits()
    {
        float dt = Time.deltaTime;
        for (int i = 0; i < _units.Count; i++)
        {
            _units[i].Move(dt);
            _mainThreadPositions[i] = _units[i].Position;
            _units[i].transform.position = _units[i].Position;
        }
    }

    private ISpatialIndex _indexA;
    private ISpatialIndex _indexB;
    private bool _usingIndexA = true; // Tracks which index is currently being built/used
    private SpatialStructureType _lastStructureType;
    private int _lastUnitCount;

    private void EnsureIndicesCreated()
    {
        // Recreate indices if settings changed or not created yet
        if (_indexA == null || _indexB == null || 
            _lastStructureType != StructureType || _lastUnitCount != UnitCount)
        {
            _lastStructureType = StructureType;
            _lastUnitCount = UnitCount;
            
            _indexA = CreateNewIndex();
            _indexB = CreateNewIndex();
            
            // Force GC collection after large allocation change to clean up old indices
            System.GC.Collect();
        }
    }

    private ISpatialIndex CreateNewIndex()
    {
        switch (StructureType)
        {
            case SpatialStructureType.Grid:
                return new SpatialGridIndex(GridResolution, GridResolution, WorldSize / GridResolution, new Vector2(-WorldSize/2, -WorldSize/2));
            case SpatialStructureType.KDTree:
                return new KDTreeIndex(UnitCount);
            case SpatialStructureType.BVH:
                return new BVHIndex(UnitCount);
            case SpatialStructureType.QuadTree:
                return new QuadTreeIndex(UnitCount, new Rect(-WorldSize/2, -WorldSize/2, WorldSize, WorldSize));
            case SpatialStructureType.SIMDHashGrid:
                return new SIMDHashGridIndex(GridResolution, GridResolution, WorldSize / GridResolution, new Vector2(-WorldSize/2, -WorldSize/2), UnitCount);
            default:
                return new KDTreeIndex(UnitCount);
        }
    }

    private async UniTaskVoid BuildLoop()
    {
        while (this != null)
        {
            // Wait for next frame
            await UniTask.Yield(PlayerLoopTiming.Update);
            
            if (_units.Count == 0) continue;

            // Ensure indices exist and match current settings
            EnsureIndicesCreated();

            // Copy positions to async buffer (main thread)
            if (_asyncPositions.Capacity < _mainThreadPositions.Count)
            {
                _asyncPositions.Capacity = _mainThreadPositions.Count;
            }
            _asyncPositions.Clear();
            _asyncPositions.AddRange(_mainThreadPositions);

            // Build index
            IsBuilding = true;
            
            // Pick the "back buffer" index to build
            // If _spatialIndex is currently _indexA, we build _indexB, and vice versa.
            // Actually, let's just track which one is "active" for query, and build the other.
            ISpatialIndex indexToBuild = _usingIndexA ? _indexB : _indexA;
            
            float buildTime = 0;
            
            if (UseMultiThreading)
            {
                // Multi-threaded build (background thread)
                await UniTask.RunOnThreadPool(async () =>
                {
                    Stopwatch sw = Stopwatch.StartNew();
                    await indexToBuild.BuildAsync(_asyncPositions);
                    sw.Stop();
                    buildTime = (float)sw.Elapsed.TotalMilliseconds;
                });
            }
            else
            {
                // Single-threaded build (main thread)
                Stopwatch sw = Stopwatch.StartNew();
                await indexToBuild.BuildAsync(_asyncPositions);
                sw.Stop();
                buildTime = (float)sw.Elapsed.TotalMilliseconds;
            }
            
            BuildTimeMs = buildTime;
            IsBuilding = false;

            // Swap buffers
            _usingIndexA = !_usingIndexA;
            _spatialIndex = indexToBuild;
        }
    }

    private void PerformQuery()
    {
        if (_spatialIndex == null) return;

        Stopwatch sw = Stopwatch.StartNew();
        
        _heroQueryResults.Clear();
        Vector2 heroPos = _mainThreadPositions[0];
        
        _spatialIndex.QueryKNearest(heroPos, QueryK, _heroQueryResults);
        
        sw.Stop();
        QueryTimeMs = (float)sw.Elapsed.TotalMilliseconds;
    }


    private void OnGUI()
    {
        GUILayout.BeginArea(new Rect(10, 10, 300, 250));
        GUILayout.Label($"Units: {UnitCount}");
        GUILayout.Label($"Structure: {StructureType}");
        GUILayout.Label($"Multi-Threading: {(UseMultiThreading ? "ON" : "OFF")}");
        GUILayout.Label($"Build Time: {BuildTimeMs:F2} ms {(IsBuilding ? "(Building...)" : "")}");
        GUILayout.Label($"Query Time (Hero): {QueryTimeMs:F4} ms {(IsQuerying ? "(Querying...)" : "")}");
        GUILayout.Label($"FPS: {1.0f/Time.smoothDeltaTime:F1}");
        GUILayout.EndArea();
    }

    private void OnDrawGizmos()
    {
        if (!ShowGizmos || _units.Count == 0 || _heroQueryResults == null) return;

        // Draw Hero
        Gizmos.color = Color.green;
        // Use main thread positions for rendering
        if (_mainThreadPositions.Count > 0)
        {
            Vector3 heroPos = _mainThreadPositions[0];
            Gizmos.DrawWireSphere(heroPos, 1f);

            // Draw Lines to Neighbors
            Gizmos.color = Color.red;
            foreach (int idx in _heroQueryResults)
            {
                if (idx < _mainThreadPositions.Count)
                {
                    Gizmos.DrawLine(heroPos, _mainThreadPositions[idx]);
                    Gizmos.DrawWireSphere(_mainThreadPositions[idx], 0.5f);
                }
            }
        }
    }
}
