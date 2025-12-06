using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using System.Diagnostics;
using Random = UnityEngine.Random;
using Debug = UnityEngine.Debug;

public class CombinedComparisonDemo : ComparisonDemoBase
{
    [Header("Simulation Settings")]
    public float MapSize = 50f;
    public float AgentSpeed = 5f;
    public float AgentRadius = 0.5f;
    public int SpawnCount = 200;

    [Header("Map Generation")]
    public int ObstacleCount = 50;
    public Vector2 ObstacleSizeMin = new Vector2(2, 2);
    public Vector2 ObstacleSizeMax = new Vector2(8, 8);
    public float CellSize = 1.0f;

    [Header("Pathfinding")]
    public float PathUpdateInterval = 1.0f;

    private GridMap _gridMap;
    private Unity.Collections.NativeArray<bool> _nativeObstacles;
    private List<RVOObstacle> _rvoObstacles = new List<RVOObstacle>();
    private List<EnemyUnit> _agents = new List<EnemyUnit>();
    private List<Vector2> _tempPositions = new List<Vector2>();
    
    private int _mapWidth;
    private int _mapHeight;
    private Vector2 _origin;

    void Start()
    {
        AgentCount = SpawnCount;
        
        InitializeMap();
        InitializeSimulators();
        SpawnAgents();
        
        // Sync Obstacles
        SIMDRVOSimulator.Instance.UpdateObstacles(_rvoObstacles);
        RVOSimulator.Instance.ClearObstacles();
        foreach (var obs in _rvoObstacles)
        {
            // Note: RVOSimulator expects Vector3 for AddObstacle? No, RVOSimulator.AddObstacle(p1, p2) uses Vector3.
            RVOSimulator.Instance.AddObstacle(new Vector3(obs.Point1.x, 0, obs.Point1.y), new Vector3(obs.Point2.x, 0, obs.Point2.y));
        }
        RVOSimulator.Instance.ProcessObstacles();
    }

    private void InitializeMap()
    {
        _mapWidth = Mathf.CeilToInt(MapSize / CellSize);
        _mapHeight = Mathf.CeilToInt(MapSize / CellSize);
        _origin = new Vector2(-MapSize / 2, -MapSize / 2);

        _gridMap = new GridMap(_mapWidth, _mapHeight, CellSize, _origin);

        // Borders
        AddRectObstacle(new Rect(_origin.x - 5, _origin.y - 5, MapSize + 10, 5));
        AddRectObstacle(new Rect(_origin.x - 5, _origin.y + MapSize, MapSize + 10, 5));
        AddRectObstacle(new Rect(_origin.x - 5, _origin.y, 5, MapSize));
        AddRectObstacle(new Rect(_origin.x + MapSize, _origin.y, 5, MapSize));

        // Random
        for (int i = 0; i < ObstacleCount; i++)
        {
            float w = Random.Range(ObstacleSizeMin.x, ObstacleSizeMax.x);
            float h = Random.Range(ObstacleSizeMin.y, ObstacleSizeMax.y);
            float x = Random.Range(_origin.x + 2, _origin.x + MapSize - 2 - w);
            float y = Random.Range(_origin.y + 2, _origin.y + MapSize - 2 - h);
            
            AddRectObstacle(new Rect(x, y, w, h));
        }

        // Native
        if (_nativeObstacles.IsCreated) _nativeObstacles.Dispose();
        _nativeObstacles = new NativeArray<bool>(_mapWidth * _mapHeight, Allocator.Persistent);
        for (int x = 0; x < _mapWidth; x++)
        {
            for (int y = 0; y < _mapHeight; y++)
            {
                _nativeObstacles[y * _mapWidth + x] = _gridMap.IsObstacle(x, y);
            }
        }
    }

    private void AddRectObstacle(Rect rect)
    {
        // 1. Grid
        Vector2 min = new Vector2(rect.x, rect.y);
        Vector2 max = new Vector2(rect.x + rect.width, rect.y + rect.height);
        Vector2Int minGrid = _gridMap.WorldToGrid(min);
        Vector2Int maxGrid = _gridMap.WorldToGrid(max);

        for (int x = minGrid.x; x <= maxGrid.x; x++)
        {
            for (int y = minGrid.y; y <= maxGrid.y; y++)
            {
                if (_gridMap.IsValid(x, y)) _gridMap.SetObstacle(x, y, true);
            }
        }

        // 2. RVO (CCW)
        Vector2 bl = min;
        Vector2 br = new Vector2(max.x, min.y);
        Vector2 tr = max;
        Vector2 tl = new Vector2(min.x, max.y);
        
        AddRVOObstacleToList(bl, br);
        AddRVOObstacleToList(br, tr);
        AddRVOObstacleToList(tr, tl);
        AddRVOObstacleToList(tl, bl);

        // Visual
        GameObject cube = GameObject.CreatePrimitive(PrimitiveType.Cube);
        cube.transform.position = new Vector3(rect.center.x, 0.5f, rect.center.y);
        cube.transform.localScale = new Vector3(rect.width, 1, rect.height);
        cube.GetComponent<Renderer>().material.color = Color.red;
        Destroy(cube.GetComponent<Collider>());
    }

    private void AddRVOObstacleToList(Vector2 start, Vector2 end)
    {
        _rvoObstacles.Add(new RVOObstacle(new float2(start.x, start.y), new float2(end.x, end.y)));
    }

    private void InitializeSimulators()
    {
        SIMDRVOSimulator.Instance.Initialize(AgentCount);
        SIMDRVOSimulator.Instance.NeighborDist = 10.0f;
        SIMDRVOSimulator.Instance.MaxNeighbors = 10;
        SIMDRVOSimulator.Instance.TimeHorizon = 2.0f;
        SIMDRVOSimulator.Instance.Radius = AgentRadius;
        SIMDRVOSimulator.Instance.MaxSpeed = AgentSpeed;

        RVOSimulator.Instance.ClearAgents();
        RVOSimulator.Instance.NeighborDist = 10.0f;
        RVOSimulator.Instance.MaxNeighbors = 10;
        RVOSimulator.Instance.TimeHorizon = 2.0f;
        RVOSimulator.Instance.Radius = AgentRadius;
        RVOSimulator.Instance.MaxSpeed = AgentSpeed;
        
        SpatialIndexManager.Instance.Initialize(AgentCount, MapSize);
    }

    private void SpawnAgents()
    {
        GameObject template = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
        Destroy(template.GetComponent<Collider>());

        for (int i = 0; i < AgentCount; i++)
        {
            Vector2 spawnPos = GetRandomWalkablePoint();
            
            GameObject go = Instantiate(template, new Vector3(spawnPos.x, 0, spawnPos.y), Quaternion.identity);
            go.name = $"Agent_{i}";
            go.GetComponent<Renderer>().material.color = Color.green;
            go.transform.localScale = new Vector3(AgentRadius * 2, 1, AgentRadius * 2);

            EnemyUnit unit = new EnemyUnit();
            unit.GameObject = go;
            unit.RVOAgentId = i;
            unit.TargetPosition = GetRandomWalkablePoint();
            unit.NextPathUpdateTime = Random.Range(0f, PathUpdateInterval);
            
            _agents.Add(unit);
            
            // Setup both simulators to start consistent
            SIMDRVOSimulator.Instance.UpdateAgentData(i, spawnPos, Vector2.zero, Vector2.zero);
            RVOSimulator.Instance.AddAgent(new Vector3(spawnPos.x, 0, spawnPos.y));
        }

        Destroy(template);
    }

    void Update()
    {
        Stopwatch.Restart();
        
        UpdateAgentsLogic();
        
        float dt = Time.deltaTime;
        
        // Sync Positions for Spatial Index (using previous frame positions)
        _tempPositions.Clear();
        for(int i=0; i<AgentCount; i++) 
            _tempPositions.Add(new Vector2(_agents[i].GameObject.transform.position.x, _agents[i].GameObject.transform.position.z));
        SpatialIndexManager.Instance.UpdatePositions(_tempPositions);

        if (UseSIMD)
        {
            SIMDRVOSimulator.Instance.Step(dt);
            SyncTransforms(true);
        }
        else
        {
            RVOSimulator.Instance.Step(dt);
            SyncTransforms(false);
        }
        
        Stopwatch.Stop();
        ExecutionTimeMs = (float)Stopwatch.Elapsed.TotalMilliseconds;
    }

    private void UpdateAgentsLogic()
    {
        foreach (var unit in _agents)
        {
            Vector2 pos2D = new Vector2(unit.GameObject.transform.position.x, unit.GameObject.transform.position.z);

            if (Vector2.Distance(pos2D, unit.TargetPosition) < 1.0f)
            {
                unit.TargetPosition = GetRandomWalkablePoint();
                unit.NextPathUpdateTime = Time.time; 
            }

            if (Time.time >= unit.NextPathUpdateTime)
            {
                unit.NextPathUpdateTime = Time.time + PathUpdateInterval + Random.Range(0f, 0.2f);
                UpdatePath(unit, pos2D);
            }

            Vector2 nextWaypoint = unit.TargetPosition; 
            if (unit.Path != null && unit.Path.Count > 0)
            {
                while (unit.PathIndex < unit.Path.Count)
                {
                    if (Vector2.Distance(pos2D, unit.Path[unit.PathIndex]) < 1.0f)
                        unit.PathIndex++;
                    else
                    {
                        nextWaypoint = unit.Path[unit.PathIndex];
                        break;
                    }
                }
                if (unit.PathIndex >= unit.Path.Count) nextWaypoint = unit.TargetPosition;
            }

            Vector2 dir = (nextWaypoint - pos2D).normalized;
            Vector2 prefVel = dir * AgentSpeed;
            
            if (UseSIMD)
            {
                Vector2 currentVel = SIMDRVOSimulator.Instance.GetNewVelocity(unit.RVOAgentId);
                SIMDRVOSimulator.Instance.UpdateAgentData(unit.RVOAgentId, pos2D, currentVel, prefVel);
            }
            else
            {
                RVOSimulator.Instance.SetAgentPrefVelocity(unit.RVOAgentId, new Vector3(prefVel.x, 0, prefVel.y));
            }
        }
    }

    private void UpdatePath(EnemyUnit unit, Vector2 startPos)
    {
        Vector2Int startGrid = _gridMap.WorldToGrid(startPos);
        Vector2Int endGrid = _gridMap.WorldToGrid(unit.TargetPosition);

        if (UseSIMD)
        {
            NativeArray<float2> nativePath = new NativeArray<float2>(_gridMap.Width * _gridMap.Height, Allocator.Temp);
            int pathLength = 0;

            unsafe
            {
                SIMDAStarPathfinder.FindPath(
                    new int2(startGrid.x, startGrid.y),
                    new int2(endGrid.x, endGrid.y),
                    _gridMap.Width, _gridMap.Height,
                    _nativeObstacles,
                    (float2*)nativePath.GetUnsafePtr(),
                    ref pathLength,
                    nativePath.Length,
                    new float2(_gridMap.Origin.x, _gridMap.Origin.y),
                    _gridMap.CellSize
                );
            }

            unit.Path.Clear();
            for (int k = 0; k < pathLength; k++)
                unit.Path.Add(new Vector2(nativePath[k].x, nativePath[k].y));
            nativePath.Dispose();
        }
        else
        {
            AStarPathfinder.FindPath(startPos, unit.TargetPosition, _gridMap, unit.Path);
        }
        unit.PathIndex = 0;
    }

    private void SyncTransforms(bool simd)
    {
        for (int i = 0; i < _agents.Count; i++)
        {
            Vector2 pos;
            if (simd)
                pos = SIMDRVOSimulator.Instance.GetAgentPosition(i);
            else
            {
                Vector3 p = RVOSimulator.Instance.GetAgentPosition(i);
                pos = new Vector2(p.x, p.z);
            }
            _agents[i].GameObject.transform.position = new Vector3(pos.x, 0, pos.y);
        }
    }

    private Vector2 GetRandomWalkablePoint()
    {
        int maxAttempts = 1000;
        for (int i = 0; i < maxAttempts; i++)
        {
            float x = Random.Range(_origin.x + 2, _origin.x + MapSize - 2);
            float y = Random.Range(_origin.y + 2, _origin.y + MapSize - 2);
            Vector2 target = new Vector2(x, y);
            
            // Check clearance 
            Vector2 min = target - new Vector2(AgentRadius, AgentRadius);
            Vector2 max = target + new Vector2(AgentRadius, AgentRadius);
            
            Vector2Int minGrid = _gridMap.WorldToGrid(min);
            Vector2Int maxGrid = _gridMap.WorldToGrid(max);
            
            bool valid = true;
            for (int gx = minGrid.x; gx <= maxGrid.x; gx++)
            {
                for (int gy = minGrid.y; gy <= maxGrid.y; gy++)
                {
                    if (_gridMap.IsObstacle(gx, gy)) { valid = false; break; }
                }
                if (!valid) break;
            }

            if (valid) return target;
        }
        return _origin + new Vector2(MapSize/2, MapSize/2);
    }

    protected override void OnSIMDChanged(bool useSIMD)
    {
         // Sync state when switching
         // If switching from SIMD to Default
         if (!useSIMD)
         {
             // Copy positions from SIMD to Default
             RVOSimulator.Instance.ClearAgents();
             for(int i=0; i<AgentCount; i++)
             {
                 Vector2 pos = SIMDRVOSimulator.Instance.GetAgentPosition(i);
                 Vector2 vel = SIMDRVOSimulator.Instance.GetNewVelocity(i);
                 RVOSimulator.Instance.AddAgent(new Vector3(pos.x, 0, pos.y));
                 // Velocity not easy to set in Default without agent hacking or just let it converge
             }
         }
         else
         {
             // Default to SIMD
             for(int i=0; i<AgentCount; i++)
             {
                 Vector3 p = RVOSimulator.Instance.GetAgentPosition(i);
                 Vector2 pos = new Vector2(p.x, p.z);
                 SIMDRVOSimulator.Instance.UpdateAgentData(i, pos, Vector2.zero, Vector2.zero);
             }
         }
    }

    protected override void OnGUIDisplayExtra()
    {
         GUILayout.Label($"Obstacles: {ObstacleCount}");
         GUILayout.Label($"Map Size: {MapSize}x{MapSize}");
    }

    private void OnDestroy()
    {
        if (_nativeObstacles.IsCreated) _nativeObstacles.Dispose();
        SIMDRVOSimulator.Instance.Shutdown();
        SpatialIndexManager.Instance.Shutdown();
    }
}
