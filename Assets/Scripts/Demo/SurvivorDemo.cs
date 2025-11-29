using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;
using Random = UnityEngine.Random;
using Unity.Collections.LowLevel.Unsafe;

public class SurvivorDemo : MonoBehaviour
{
    [Header("Settings")]
    public int EnemyCount = 500;
    public float SpawnRadius = 30f;
    public float WorldSize = 50f;

    [Header("Player")]
    public float PlayerSpeed = 8f;
    public float PlayerRadius = 0.5f;

    [Header("Enemies")]
    public float EnemySpeed = 3.0f;
    public float EnemyRadius = 0.5f;
    public float PathUpdateInterval = 0.5f;
    public float AttackRange = 1.5f; // Stop when close to player

    [Header("Map")]
    public float CellSize = 1.0f;
    public int MapWidth = 50;
    public int MapHeight = 50;

    [Header("Debug")]
    public bool ShowDebugGizmos = true;
    public bool UseSIMDPathfinding = true;
    public int DebugAgentIndex = 1; // Agent 0 is player, start from 1

    private GameObject _player;
    private List<EnemyUnit> _enemies = new List<EnemyUnit>();
    private GridMap _gridMap;
    private Unity.Collections.NativeArray<bool> _nativeObstacles;
    private List<RVOObstacle> _obstacles = new List<RVOObstacle>();
    private List<int> _debugNeighbors = new List<int>();



    void Start()
    {
        InitializeMap();

        // Initialize Pathfinding System EARLY
        if (GetComponent<PathfindingSystem>() == null) gameObject.AddComponent<PathfindingSystem>();
        // Note: _nativeObstacles is not created yet? Wait, InitializeMap creates obstacles in GridMap, 
        // but _nativeObstacles is created in PathfindingSystem.UpdateNativeObstacles.
        // So we can pass null or let it handle it.
        // Actually, InitializeMap fills _gridMap.
        PathfindingSystem.Instance.Initialize(_gridMap, _nativeObstacles, MapWidth, MapHeight, UseSIMDPathfinding);

        SpawnPlayer();
        SpawnEnemies();

        // Initialize RVO
        RVOSimulator.Instance.ClearAgents();
        RVOSimulator.Instance.ClearObstacles();

        // Tune RVO Global Parameters BEFORE adding agents
        // (agents copy these values when created)
        // Parameters are now managed by PathfindingSystem, but initial values for agents need to be set here
        // or we need to update agents when parameters change.
        // For now, let's set reasonable defaults here matching PathfindingSystem defaults.
        RVOSimulator.Instance.NeighborDist = 10.0f;
        RVOSimulator.Instance.MaxNeighbors = 20;
        RVOSimulator.Instance.TimeHorizon = 2.0f;
        RVOSimulator.Instance.Radius = 0.6f;
        RVOSimulator.Instance.MaxSpeed = Mathf.Max(PlayerSpeed, EnemySpeed) * 1.2f;

        // Add obstacles to RVO
        foreach (var obs in _obstacles)
        {
            RVOSimulator.Instance.AddObstacle(new Vector3(obs.Point1.x, 0, obs.Point1.y), new Vector3(obs.Point2.x, 0, obs.Point2.y));
        }

        // Add Player to RVO (as agent 0)
        RVOSimulator.Instance.AddAgent(_player.transform.position);
        RVOSimulator.Instance.SetAgentPrefVelocity(0, Vector3.zero);

        // Add Enemies to RVO
        for (int i = 0; i < _enemies.Count; i++)
        {
            RVOSimulator.Instance.AddAgent(_enemies[i].GameObject.transform.position);
            _enemies[i].RVOAgentId = i + 1; // +1 because 0 is player
        }

        // Init Spatial Index
        SpatialIndexManager.Instance.StructureType = SpatialStructureType.SIMDQuadTree; // Use robust index
        SpatialIndexManager.Instance.Initialize(EnemyCount + 1, WorldSize);
    }

    void FixedUpdate()
    {
        float dt = Time.fixedDeltaTime;

        // 1. Player Movement
        UpdatePlayer(dt);

        // 2. Pathfinding System (Pipeline) - Handles Enemy Logic and RVO Step
        if (PathfindingSystem.Instance != null)
        {
            Vector3 pPos = _player.transform.position;
            PathfindingSystem.Instance.UpdateSystem(dt, new Vector2(pPos.x, pPos.z));
        }

        // 3. Apply RVO positions to GameObjects
        ApplyPositions();

        // 4. Debug: Query neighbors for debug agent
        if (ShowDebugGizmos && DebugAgentIndex < RVOSimulator.Instance.GetAgentCount())
        {
            Vector3 debugPos3D = RVOSimulator.Instance.GetAgentPosition(DebugAgentIndex);
            float2 debugPos = new float2(debugPos3D.x, debugPos3D.z);
            _debugNeighbors.Clear();
            SpatialIndexManager.Instance.GetNeighborsInRadius(debugPos, 5.0f, _debugNeighbors);
        }
    }

    private void InitializeMap()
    {
        _gridMap = new GridMap(MapWidth, MapHeight, CellSize, new Vector2(-MapWidth * CellSize / 2, -MapHeight * CellSize / 2));

        // Create some walls
        CreateWall(new Vector2(-10, 5), new Vector2(10, 5));
        CreateWall(new Vector2(-10, -5), new Vector2(10, -5));
        CreateWall(new Vector2(15, -10), new Vector2(15, 10));
        CreateWall(new Vector2(-15, -10), new Vector2(-15, 10));

        // U-shape
        CreateWall(new Vector2(-5, 15), new Vector2(5, 15));
        CreateWall(new Vector2(-5, 15), new Vector2(-5, 20));
        CreateWall(new Vector2(5, 15), new Vector2(5, 20));

        // Initialize Native Obstacles for SIMD
        if (_nativeObstacles.IsCreated) _nativeObstacles.Dispose();
        _nativeObstacles = new Unity.Collections.NativeArray<bool>(MapWidth * MapHeight, Unity.Collections.Allocator.Persistent);
        for (int x = 0; x < MapWidth; x++)
        {
            for (int y = 0; y < MapHeight; y++)
            {
                _nativeObstacles[y * MapWidth + x] = _gridMap.IsObstacle(x, y);
            }
        }
    }

    private void CreateWall(Vector2 start, Vector2 end)
    {
        // Track grid bounds
        int minX = int.MaxValue;
        int minY = int.MaxValue;
        int maxX = int.MinValue;
        int maxY = int.MinValue;

        // Add to GridMap (Rasterize line)
        Vector2 dir = (end - start).normalized;
        float dist = Vector2.Distance(start, end);

        // Ensure we cover the start and end points exactly
        // And step small enough to hit all cells
        for (float d = 0; d <= dist; d += CellSize * 0.5f)
        {
            Vector2 p = start + dir * d;
            Vector2Int gridPos = _gridMap.WorldToGrid(p);
            _gridMap.SetObstacle(gridPos.x, gridPos.y, true);

            if (gridPos.x < minX) minX = gridPos.x;
            if (gridPos.y < minY) minY = gridPos.y;
            if (gridPos.x > maxX) maxX = gridPos.x;
            if (gridPos.y > maxY) maxY = gridPos.y;
        }

        // Also check the exact end point to be sure (floating point errors)
        {
            Vector2Int gridPos = _gridMap.WorldToGrid(end);
            _gridMap.SetObstacle(gridPos.x, gridPos.y, true);
            if (gridPos.x < minX) minX = gridPos.x;
            if (gridPos.y < minY) minY = gridPos.y;
            if (gridPos.x > maxX) maxX = gridPos.x;
            if (gridPos.y > maxY) maxY = gridPos.y;
        }

        // Add to RVO as a "Thick" Wall (Rectangle) derived from Grid Bounds
        // This ensures perfect alignment with the visual red cubes.

        // GridToWorld returns the CENTER of the cell.
        // We want the outer edges.
        // Bottom-Left of min cell: Center - 0.5*Size
        // Top-Right of max cell: Center + 0.5*Size

        Vector2 minCenter = _gridMap.GridToWorld(minX, minY);
        Vector2 maxCenter = _gridMap.GridToWorld(maxX, maxY);

        Vector2 pMin = minCenter - new Vector2(CellSize * 0.5f, CellSize * 0.5f);
        Vector2 pMax = maxCenter + new Vector2(CellSize * 0.5f, CellSize * 0.5f);

        // Add 4 segments for the rectangle
        // CLOCKWISE Order to ensure Normals point OUTWARD
        // Normal = (-Dir.y, Dir.x)
        // If Dir is (1,0) [Right], Normal is (0,1) [Up].
        // So we want Right -> Down -> Left -> Up?
        // Let's trace:
        // Top Edge: Left to Right. Dir=(1,0). Normal=(0,1) UP. Correct.
        // Right Edge: Top to Bottom. Dir=(0,-1). Normal=(1,0) RIGHT. Correct.
        // Bottom Edge: Right to Left. Dir=(-1,0). Normal=(0,-1) DOWN. Correct.
        // Left Edge: Bottom to Top. Dir=(0,1). Normal=(-1,0) LEFT. Correct.

        // So we need: Top-Left -> Top-Right -> Bottom-Right -> Bottom-Left -> Top-Left

        Vector2 pTL = new Vector2(pMin.x, pMax.y);
        Vector2 pTR = new Vector2(pMax.x, pMax.y);
        Vector2 pBR = new Vector2(pMax.x, pMin.y);
        Vector2 pBL = new Vector2(pMin.x, pMin.y);

        _obstacles.Add(new RVOObstacle(pTL, pTR)); // Top
        _obstacles.Add(new RVOObstacle(pTR, pBR)); // Right
        _obstacles.Add(new RVOObstacle(pBR, pBL)); // Bottom
        _obstacles.Add(new RVOObstacle(pBL, pTL)); // Left
    }

    private void SpawnPlayer()
    {
        GameObject template = GameObject.CreatePrimitive(PrimitiveType.Capsule);
        _player = Instantiate(template, Vector3.zero, Quaternion.identity);
        _player.name = "Player";
        _player.GetComponent<Renderer>().material.color = Color.green;
        Destroy(template);
    }

    private void SpawnEnemies()
    {
        GameObject template = GameObject.CreatePrimitive(PrimitiveType.Cube);
        Destroy(template.GetComponent<Collider>());

        for (int i = 0; i < EnemyCount; i++)
        {
            Vector2 pos = Random.insideUnitCircle.normalized * SpawnRadius;
            // Fix: Spawn on X-Z plane
            GameObject go = Instantiate(template, new Vector3(pos.x, 0, pos.y), Quaternion.identity);
            go.name = $"Enemy_{i}";
            go.GetComponent<Renderer>().material.color = Color.red;

            EnemyUnit unit = new EnemyUnit();
            unit.GameObject = go;
            unit.NextPathUpdateTime = Time.time + Random.Range(0f, PathUpdateInterval);
            _enemies.Add(unit);

            // Register with Pathfinding System
            PathfindingSystem.Instance.RegisterUnit(unit);
        }

        Destroy(template);
    }

    private void UpdatePlayer(float dt)
    {
        Vector3 input = new Vector3(Input.GetAxis("Horizontal"), 0, Input.GetAxis("Vertical"));
        Vector3 velocity = input.normalized * PlayerSpeed;

        // Set RVO pref velocity for player
        RVOSimulator.Instance.SetAgentPrefVelocity(0, velocity);

        // Also move manually? No, RVO will move it.
        // But player input should override RVO? 
        // Ideally player is an agent that pushes others.
        // For strict control, we might want to force position, but let's let RVO handle it for smooth collision.
    }

    private void UpdateEnemies(float dt)
    {
        // This logic is now handled by PathfindingSystem.UpdateSystem
        // The PathfindingSystem will call GetValidTarget and update unit paths.
        // It will also set the RVO preferred velocity for each enemy.
    }

    private void ApplyPositions()
    {
        _player.transform.position = RVOSimulator.Instance.GetAgentPosition(0);

        for (int i = 0; i < _enemies.Count; i++)
        {
            _enemies[i].GameObject.transform.position = RVOSimulator.Instance.GetAgentPosition(_enemies[i].RVOAgentId);
        }
    }

    private void OnDrawGizmos()
    {
        if (_gridMap != null)
        {
            // Draw Grid
            Gizmos.color = new Color(0.5f, 0.5f, 0.5f, 0.1f); // Faint grey
            for (int x = 0; x < _gridMap.Width; x++)
            {
                for (int y = 0; y < _gridMap.Height; y++)
                {
                    Vector2 pos = _gridMap.GridToWorld(x, y);
                    if (_gridMap.IsObstacle(x, y))
                    {
                        Gizmos.color = new Color(1, 0, 0, 0.3f); // Red obstacle
                        Gizmos.DrawCube(new Vector3(pos.x, 0, pos.y), new Vector3(_gridMap.CellSize, 0.1f, _gridMap.CellSize));
                    }
                    else
                    {
                        Gizmos.color = new Color(0.5f, 0.5f, 0.5f, 0.1f); // Walkable
                        Gizmos.DrawWireCube(new Vector3(pos.x, 0, pos.y), new Vector3(_gridMap.CellSize, 0.1f, _gridMap.CellSize));
                    }
                }
            }
        }

        if (_obstacles != null)
        {
            Gizmos.color = Color.blue;
            foreach (var obs in _obstacles)
            {
                Vector3 p1 = new Vector3(obs.Point1.x, 0, obs.Point1.y);
                Vector3 p2 = new Vector3(obs.Point2.x, 0, obs.Point2.y);
                Gizmos.DrawLine(p1, p2);

                // Draw Normal to show orientation
                Vector3 mid = (p1 + p2) * 0.5f;
                Vector3 normal = new Vector3(obs.Normal.x, 0, obs.Normal.y);
                Gizmos.DrawLine(mid, mid + normal * 0.3f);
            }
        }

        // Draw Paths for a few enemies to verify A*
        if (!ShowDebugGizmos)
        {
            Gizmos.color = Color.yellow;
            for (int i = 0; i < Mathf.Min(_enemies.Count, 10); i++)
            {
                var unit = _enemies[i];
                if (unit.Path != null && unit.Path.Count > 0)
                {
                    for (int j = 0; j < unit.Path.Count - 1; j++)
                    {
                        Gizmos.DrawLine(new Vector3(unit.Path[j].x, 0, unit.Path[j].y), new Vector3(unit.Path[j + 1].x, 0, unit.Path[j + 1].y));
                    }
                }
            }
        }

        if (ShowDebugGizmos && DebugAgentIndex < RVOSimulator.Instance.GetAgentCount())
        {
            Vector3 debugPos3D = RVOSimulator.Instance.GetAgentPosition(DebugAgentIndex);

            // Draw debug agent in green with larger sphere
            Gizmos.color = Color.green;
            Gizmos.DrawWireSphere(debugPos3D, RVOSimulator.Instance.Radius * 1.5f);

            // Draw neighbor detection radius
            Gizmos.color = Color.yellow;
            Gizmos.DrawWireSphere(debugPos3D, RVOSimulator.Instance.NeighborDist);

            // Draw neighbors in red
            Gizmos.color = Color.red;
            foreach (int idx in _debugNeighbors)
            {
                if (idx != DebugAgentIndex && idx < RVOSimulator.Instance.GetAgentCount())
                {
                    Vector3 neighborPos3D = RVOSimulator.Instance.GetAgentPosition(idx);
                    Gizmos.DrawLine(debugPos3D, neighborPos3D);
                    Gizmos.DrawWireSphere(neighborPos3D, RVOSimulator.Instance.Radius);
                }
            }

            // Draw Path for Debug Agent
            // Find the EnemyUnit corresponding to this RVO agent
            var unit = _enemies.Find(e => e.RVOAgentId == DebugAgentIndex);
            if (unit != null && unit.Path != null && unit.Path.Count > 0)
            {
                Gizmos.color = Color.magenta;
                for (int j = 0; j < unit.Path.Count - 1; j++)
                {
                    Vector3 p1 = new Vector3(unit.Path[j].x, 0, unit.Path[j].y);
                    Vector3 p2 = new Vector3(unit.Path[j + 1].x, 0, unit.Path[j + 1].y);
                    Gizmos.DrawLine(p1, p2);
                }
                // Draw line to current target
                if (unit.PathIndex < unit.Path.Count)
                {
                    Gizmos.DrawLine(debugPos3D, new Vector3(unit.Path[unit.PathIndex].x, 0, unit.Path[unit.PathIndex].y));
                }
            }

            // Draw agent radius for first 50 agents
            Gizmos.color = new Color(0, 1, 0, 0.3f);
            for (int i = 0; i < Mathf.Min(RVOSimulator.Instance.GetAgentCount(), 50); i++)
            {
                Vector3 pos = RVOSimulator.Instance.GetAgentPosition(i);
                Gizmos.DrawWireSphere(pos, RVOSimulator.Instance.Radius);
            }
        }
    }

    private void OnGUI()
    {
        GUILayout.BeginArea(new Rect(10, 10, 450, 500));
        GUILayout.Label($"Survivor Demo");
        GUILayout.Label($"Agents: {RVOSimulator.Instance.GetAgentCount()}");
        GUILayout.Label($"Enemies: {_enemies.Count}");

        GUILayout.Space(10);
        GUILayout.Label("=== Spatial Index ===");
        GUILayout.Label($"Type: {SpatialIndexManager.Instance.StructureType}");
        GUILayout.Label($"Build Time: {SpatialIndexManager.Instance.BuildTimeMs:F3} ms");
        GUILayout.Label($"Building: {(SpatialIndexManager.Instance.IsBuilding ? "Yes" : "No")}");

        GUILayout.Space(10);
        GUILayout.Label("=== RVO Performance ===");
        GUILayout.Label($"Neighbor Query: {RVOSimulator.Instance.NeighborQueryTimeMs:F3} ms");
        GUILayout.Label($"RVO Compute: {RVOSimulator.Instance.RVOComputeTimeMs:F3} ms");
        GUILayout.Label($"Position Update: {RVOSimulator.Instance.PositionUpdateTimeMs:F3} ms");
        GUILayout.Label($"Total Step: {RVOSimulator.Instance.TotalStepTimeMs:F3} ms");

        GUILayout.Space(10);
        GUILayout.Label("=== RVO Settings ===");
        GUILayout.Label($"Neighbor Dist: {RVOSimulator.Instance.NeighborDist:F1}");
        GUILayout.Label($"Max Neighbors: {RVOSimulator.Instance.MaxNeighbors}");
        GUILayout.Label($"Time Horizon: {RVOSimulator.Instance.TimeHorizon:F1}");
        GUILayout.Label($"Radius: {RVOSimulator.Instance.Radius:F2}");

        GUILayout.Space(10);
        GUILayout.Label($"FPS: {1.0f / Time.smoothDeltaTime:F1}");

        if (ShowDebugGizmos && DebugAgentIndex < RVOSimulator.Instance.GetAgentCount())
        {
            GUILayout.Space(10);
            GUILayout.Label($"=== Debug Agent {DebugAgentIndex} ===");
            GUILayout.Label($"Neighbors Found: {_debugNeighbors.Count}");
            Vector3 debugPos = RVOSimulator.Instance.GetAgentPosition(DebugAgentIndex);
            GUILayout.Label($"Position: ({debugPos.x:F1}, {debugPos.z:F1})");
        }

        GUILayout.EndArea();
    }

    // GetValidTarget moved to PathfindingSystem
    // OnDestroy handled by PathfindingSystem for native obstacles
    private void OnDestroy()
    {
        // Cleanup if needed
    }
}
