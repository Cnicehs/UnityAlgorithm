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
    public float SeparationWeight = 1.5f; // Weight for separation steering
    public float SeparationDist = 1.0f; // Distance to check for separation

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

    private class EnemyUnit
    {
        public GameObject GameObject;
        public int RVOAgentId;
        public List<Vector2> Path = new List<Vector2>();
        public int PathIndex;
        public float NextPathUpdateTime;
    }

    void Start()
    {
        InitializeMap();
        SpawnPlayer();
        SpawnEnemies();

        // Initialize RVO
        RVOSimulator.Instance.ClearAgents();
        RVOSimulator.Instance.ClearObstacles();

        // Add obstacles to RVO
        foreach (var obs in _obstacles)
        {
            RVOSimulator.Instance.AddObstacle(obs.Point1, obs.Point2);
        }

        // Add Player to RVO (as agent 0)
        RVOSimulator.Instance.AddAgent((Vector2)_player.transform.position);
        RVOSimulator.Instance.SetAgentPrefVelocity(0, Vector2.zero);

        // Add Enemies to RVO
        for (int i = 0; i < _enemies.Count; i++)
        {
            RVOSimulator.Instance.AddAgent((Vector2)_enemies[i].GameObject.transform.position);
            _enemies[i].RVOAgentId = i + 1; // +1 because 0 is player
        }

        // Init Spatial Index
        SpatialIndexManager.Instance.StructureType = SpatialStructureType.SIMDQuadTree; // Use robust index
        SpatialIndexManager.Instance.Initialize(EnemyCount + 1, WorldSize);

        // Tune RVO Global Parameters
        RVOSimulator.Instance.NeighborDist = 10.0f; // Reset to reasonable value
        RVOSimulator.Instance.MaxNeighbors = 20;
        RVOSimulator.Instance.TimeHorizon = 1.0f; // Reduced time horizon for tighter packing without overlap
        RVOSimulator.Instance.Radius = 0.6f; // Slightly larger than visual (0.5) to ensure gap
    }

    void FixedUpdate()
    {
        float dt = Time.fixedDeltaTime;

        // 1. Player Movement
        UpdatePlayer(dt);

        // 2. Enemy Logic (Pathfinding + Separation)
        UpdateEnemies(dt);

        // 3. RVO Step
        List<Vector2> allPositions = new List<Vector2>();
        for (int i = 0; i < RVOSimulator.Instance.GetAgentCount(); i++)
        {
            allPositions.Add(RVOSimulator.Instance.GetAgentPosition(i));
        }
        SpatialIndexManager.Instance.UpdatePositions(allPositions);

        RVOSimulator.Instance.Step(dt);

        // 4. Apply RVO positions to GameObjects
        ApplyPositions();

        // 5. Debug: Query neighbors for debug agent
        if (ShowDebugGizmos && DebugAgentIndex < RVOSimulator.Instance.GetAgentCount())
        {
            _debugNeighbors.Clear();
            Vector2 debugPos = RVOSimulator.Instance.GetAgentPosition(DebugAgentIndex);
            SpatialIndexManager.Instance.GetNeighborsInRadius(debugPos, RVOSimulator.Instance.NeighborDist, _debugNeighbors);
        }
    }

    void Update()
    {
        // Only handle input or rendering stuff here if needed
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

        Vector2 p1 = new Vector2(pMin.x, pMin.y); // Bottom-Left
        Vector2 p2 = new Vector2(pMax.x, pMin.y); // Bottom-Right
        Vector2 p3 = new Vector2(pMax.x, pMax.y); // Top-Right
        Vector2 p4 = new Vector2(pMin.x, pMax.y); // Top-Left

        // Add 4 segments for the rectangle
        _obstacles.Add(new RVOObstacle(p1, p2));
        _obstacles.Add(new RVOObstacle(p2, p3));
        _obstacles.Add(new RVOObstacle(p3, p4));
        _obstacles.Add(new RVOObstacle(p4, p1));
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
            GameObject go = Instantiate(template, pos, Quaternion.identity);
            go.name = $"Enemy_{i}";
            go.GetComponent<Renderer>().material.color = Color.red;

            EnemyUnit unit = new EnemyUnit();
            unit.GameObject = go;
            unit.NextPathUpdateTime = Random.Range(0f, PathUpdateInterval);
            _enemies.Add(unit);
        }

        Destroy(template);
    }

    private void UpdatePlayer(float dt)
    {
        Vector2 input = new Vector2(Input.GetAxis("Horizontal"), Input.GetAxis("Vertical"));
        Vector2 velocity = input.normalized * PlayerSpeed;

        // Set RVO pref velocity for player
        RVOSimulator.Instance.SetAgentPrefVelocity(0, velocity);

        // Also move manually? No, RVO will move it.
        // But player input should override RVO? 
        // Ideally player is an agent that pushes others.
        // For strict control, we might want to force position, but let's let RVO handle it for smooth collision.
    }

    private void UpdateEnemies(float dt)
    {
        Vector2 playerPos = RVOSimulator.Instance.GetAgentPosition(0);

        for (int i = 0; i < _enemies.Count; i++)
        {
            EnemyUnit unit = _enemies[i];

            // Update Path periodically
            if (Time.time > unit.NextPathUpdateTime)
            {
                unit.NextPathUpdateTime = Time.time + PathUpdateInterval + Random.Range(0f, 0.2f);
                Vector2 myPos = RVOSimulator.Instance.GetAgentPosition(unit.RVOAgentId);

                // Validate Target & Start
                Vector2 validTargetPos = GetValidTarget(playerPos);
                Vector2 validStartPos = GetValidTarget(myPos); // Ensure start is also valid

                // Simple A* to player
                // Optimization: Don't run A* if line of sight is clear?
                if (UseSIMDPathfinding)
                {
                    Vector2Int startGrid = _gridMap.WorldToGrid(validStartPos);
                    Vector2Int endGrid = _gridMap.WorldToGrid(validTargetPos);

                    // Allocate max possible path length (map size)
                    Unity.Collections.NativeArray<Unity.Mathematics.float2> nativePath = new Unity.Collections.NativeArray<Unity.Mathematics.float2>(_gridMap.Width * _gridMap.Height, Unity.Collections.Allocator.Temp);
                    int pathLength = 0;

                    unsafe
                    {
                        SIMDAStarPathfinder.FindPath(
                            new Unity.Mathematics.int2(startGrid.x, startGrid.y),
                            new Unity.Mathematics.int2(endGrid.x, endGrid.y),
                            _gridMap.Width,
                            _gridMap.Height,
                            _nativeObstacles,
                            (float2*)nativePath.GetUnsafePtr(),
                            ref pathLength,
                            nativePath.Length,
                            new Unity.Mathematics.float2(_gridMap.Origin.x, _gridMap.Origin.y),
                            _gridMap.CellSize
                        );
                    }

                    unit.Path.Clear();
                    for (int k = 0; k < pathLength; k++)
                    {
                        unit.Path.Add(new Vector2(nativePath[k].x, nativePath[k].y));
                    }
                    nativePath.Dispose();
                }
                else
                {
                    AStarPathfinder.FindPath(validStartPos, validTargetPos, _gridMap, unit.Path);
                }

                unit.PathIndex = 0;
            }

            // Follow Path with Separation
            Vector2 target = playerPos;
            bool hasPath = false;

            if (unit.Path != null && unit.Path.Count > 0)
            {
                if (unit.PathIndex < unit.Path.Count)
                {
                    target = unit.Path[unit.PathIndex];
                    hasPath = true;
                    Vector2 myPos = RVOSimulator.Instance.GetAgentPosition(unit.RVOAgentId);
                    if (Vector2.Distance(myPos, target) < 1.0f)
                    {
                        unit.PathIndex++;
                        if (unit.PathIndex < unit.Path.Count)
                        {
                            target = unit.Path[unit.PathIndex];
                        }
                    }
                }
            }

            Vector2 currentPos = RVOSimulator.Instance.GetAgentPosition(unit.RVOAgentId);
            float distToPlayer = Vector2.Distance(currentPos, playerPos);

            // 1. Path/Target Force
            Vector2 moveDir = Vector2.zero;
            if (distToPlayer > AttackRange)
            {
                if (hasPath)
                {
                    moveDir = (target - currentPos).normalized;
                }
                else
                {
                    // No path found? Stop.
                    // Do NOT move directly to player if pathfinding failed, 
                    // because it likely means we are blocked or target is invalid.
                    moveDir = Vector2.zero;
                }
            }

            // 2. Separation Force
            Vector2 separationForce = Vector2.zero;
            if (SeparationWeight > 0)
            {
                List<int> neighbors = new List<int>();
                SpatialIndexManager.Instance.GetNeighborsInRadius(currentPos, SeparationDist, neighbors);

                foreach (int neighborIdx in neighbors)
                {
                    if (neighborIdx == unit.RVOAgentId) continue;

                    Vector2 neighborPos = RVOSimulator.Instance.GetAgentPosition(neighborIdx);
                    Vector2 toNeighbor = currentPos - neighborPos;
                    float distSq = toNeighbor.sqrMagnitude;

                    if (distSq > 0.001f)
                    {
                        separationForce += toNeighbor.normalized / Mathf.Sqrt(distSq);
                    }
                }
            }

            // Combine
            Vector2 finalDir = (moveDir + separationForce * SeparationWeight).normalized;

            // If close to player and separation is pushing us, we might still move.
            // But if we are close enough, we should mostly stop or just separate slightly.
            if (distToPlayer <= AttackRange && moveDir == Vector2.zero)
            {
                // Only separation applies
                finalDir = separationForce.normalized;
                // Reduce speed when just separating?
                RVOSimulator.Instance.SetAgentPrefVelocity(unit.RVOAgentId, finalDir * EnemySpeed * 0.5f);
            }
            else
            {
                RVOSimulator.Instance.SetAgentPrefVelocity(unit.RVOAgentId, finalDir * EnemySpeed);
            }
        }
    }

    private void ApplyPositions()
    {
        _player.transform.position = (Vector3)RVOSimulator.Instance.GetAgentPosition(0);

        for (int i = 0; i < _enemies.Count; i++)
        {
            _enemies[i].GameObject.transform.position = (Vector3)RVOSimulator.Instance.GetAgentPosition(_enemies[i].RVOAgentId);
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
                        Gizmos.DrawCube(new Vector3(pos.x, pos.y, 0), Vector3.one * _gridMap.CellSize);
                    }
                    else
                    {
                        Gizmos.color = new Color(0.5f, 0.5f, 0.5f, 0.1f); // Walkable
                        Gizmos.DrawWireCube(new Vector3(pos.x, pos.y, 0), Vector3.one * _gridMap.CellSize);
                    }
                }
            }
        }

        if (_obstacles != null)
        {
            Gizmos.color = Color.blue;
            foreach (var obs in _obstacles)
            {
                Vector3 p1 = new Vector3(obs.Point1.x, obs.Point1.y, 0);
                Vector3 p2 = new Vector3(obs.Point2.x, obs.Point2.y, 0);
                Gizmos.DrawLine(p1, p2);

                // Draw Normal to show orientation
                Vector3 mid = (p1 + p2) * 0.5f;
                Vector3 normal = new Vector3(obs.Normal.x, obs.Normal.y, 0);
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
                        Gizmos.DrawLine(new Vector3(unit.Path[j].x, unit.Path[j + 1].y, 0), new Vector3(unit.Path[j + 1].x, unit.Path[j + 1].y, 0));
                    }
                }
            }
        }

        if (ShowDebugGizmos && DebugAgentIndex < RVOSimulator.Instance.GetAgentCount())
        {
            Vector2 debugPos = RVOSimulator.Instance.GetAgentPosition(DebugAgentIndex);
            Vector3 debugPos3D = new Vector3(debugPos.x, debugPos.y, 0);

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
                    Vector2 neighborPos = RVOSimulator.Instance.GetAgentPosition(idx);
                    Vector3 neighborPos3D = new Vector3(neighborPos.x, neighborPos.y, 0);
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
                    Vector3 p1 = new Vector3(unit.Path[j].x, unit.Path[j].y, 0);
                    Vector3 p2 = new Vector3(unit.Path[j + 1].x, unit.Path[j + 1].y, 0);
                    Gizmos.DrawLine(p1, p2);
                }
                // Draw line to current target
                if (unit.PathIndex < unit.Path.Count)
                {
                    Gizmos.DrawLine(debugPos3D, new Vector3(unit.Path[unit.PathIndex].x, unit.Path[unit.PathIndex].y, 0));
                }
            }

            // Draw agent radius for first 50 agents
            Gizmos.color = new Color(0, 1, 0, 0.3f);
            for (int i = 0; i < Mathf.Min(RVOSimulator.Instance.GetAgentCount(), 50); i++)
            {
                Vector2 pos = RVOSimulator.Instance.GetAgentPosition(i);
                Gizmos.DrawWireSphere(new Vector3(pos.x, pos.y, 0), RVOSimulator.Instance.Radius);
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
            Vector2 debugPos = RVOSimulator.Instance.GetAgentPosition(DebugAgentIndex);
            GUILayout.Label($"Position: ({debugPos.x:F1}, {debugPos.y:F1})");
        }

        GUILayout.EndArea();
    }

    private Vector2 GetValidTarget(Vector2 targetPos)
    {
        Vector2Int gridPos = _gridMap.WorldToGrid(targetPos);
        if (!_gridMap.IsObstacle(gridPos.x, gridPos.y))
        {
            return targetPos;
        }

        // Spiral search for nearest walkable
        int radius = 1;
        while (radius < 10) // Limit search
        {
            for (int x = -radius; x <= radius; x++)
            {
                for (int y = -radius; y <= radius; y++)
                {
                    if (Mathf.Abs(x) == radius || Mathf.Abs(y) == radius)
                    {
                        int checkX = gridPos.x + x;
                        int checkY = gridPos.y + y;
                        if (!_gridMap.IsObstacle(checkX, checkY))
                        {
                            return _gridMap.GridToWorld(checkX, checkY);
                        }
                    }
                }
            }
            radius++;
        }

        return targetPos; // Give up
    }

    private void OnDestroy()
    {
        if (_nativeObstacles.IsCreated) _nativeObstacles.Dispose();
    }
}
