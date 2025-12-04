using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Random = UnityEngine.Random;

public class AStarRVOCombinedDemo : MonoBehaviour
{
    [Header("Simulation Settings")]
    public int AgentCount = 200;
    public float MapSize = 50f;
    public float AgentSpeed = 5f;
    public float AgentRadius = 0.5f;

    [Header("Map Generation")]
    public int ObstacleCount = 50;
    public Vector2 ObstacleSizeMin = new Vector2(2, 2);
    public Vector2 ObstacleSizeMax = new Vector2(8, 8);
    public float CellSize = 1.0f;

    [Header("Pathfinding")]
    public float PathUpdateInterval = 1.0f; // How often to recalculate path
    public bool UseSIMD = true;

    private GridMap _gridMap;
    private Unity.Collections.NativeArray<bool> _nativeObstacles;
    private List<RVOObstacle> _rvoObstacles = new List<RVOObstacle>();
    private List<EnemyUnit> _agents = new List<EnemyUnit>();
    
    // Bounds
    private int _mapWidth;
    private int _mapHeight;
    private Vector2 _origin;

    void Start()
    {
        Debug.Log("AStarRVOCombinedDemo Start");

        // 1. Initialize SIMD RVO
        SIMDRVOSimulator.Instance.Initialize(AgentCount);
        SIMDRVOSimulator.Instance.NeighborDist = 10.0f;
        SIMDRVOSimulator.Instance.MaxNeighbors = 10;
        SIMDRVOSimulator.Instance.TimeHorizon = 2.0f;
        SIMDRVOSimulator.Instance.Radius = AgentRadius;
        SIMDRVOSimulator.Instance.MaxSpeed = AgentSpeed;

        // 2. Initialize Map (Populates _rvoObstacles)
        InitializeMap();

        // 3. Push Obstacles to SIMD Simulator
        SIMDRVOSimulator.Instance.UpdateObstacles(_rvoObstacles);

        // 4. Spawn Agents
        SpawnAgents();

        // 5. Initialize Spatial Index
        SpatialIndexManager.Instance.StructureType = SpatialStructureType.SIMDQuadTree;
        SpatialIndexManager.Instance.Initialize(AgentCount, MapSize);
    }

    void Update()
    {
        // Debug Draw Path for first agent
        if (_agents.Count > 0 && _agents[0].Path != null)
        {
            var path = _agents[0].Path;
            for (int i = 0; i < path.Count - 1; i++)
            {
                Debug.DrawLine(new Vector3(path[i].x, 0, path[i].y), new Vector3(path[i + 1].x, 0, path[i + 1].y), Color.yellow);
            }
            Debug.DrawLine(new Vector3(_agents[0].Position.x, 0, _agents[0].Position.y), new Vector3(_agents[0].TargetPosition.x, 0, _agents[0].TargetPosition.y), Color.cyan);
        }
    }

    private void OnGUI()
    {
        GUILayout.BeginArea(new Rect(10, 10, 400, 400));
        GUILayout.Label($"A* + RVO Demo ({(UseSIMD ? "SIMD A*" : "Standard A*")})");
        GUILayout.Label($"Agents: {AgentCount} | Obstacles: {ObstacleCount}");
        
        GUILayout.Space(10);
        GUILayout.Label("=== RVO Performance (SIMD) ===");
        GUILayout.Label($"Neighbor Query: {SIMDRVOSimulator.Instance.NeighborQueryTimeMs:F3} ms");
        GUILayout.Label($"RVO Compute: {SIMDRVOSimulator.Instance.RVOComputeTimeMs:F3} ms");
        GUILayout.Label($"Total Step: {SIMDRVOSimulator.Instance.TotalStepTimeMs:F3} ms");

        GUILayout.Space(10);
        GUILayout.Label("=== Spatial Index ===");
        GUILayout.Label($"Type: {SpatialIndexManager.Instance.StructureType}");
        GUILayout.Label($"Build Time: {SpatialIndexManager.Instance.BuildTimeMs:F3} ms");

        GUILayout.Space(10);
        GUILayout.Label($"FPS: {1.0f / Time.smoothDeltaTime:F1}");
        GUILayout.EndArea();
    }

    private void OnDrawGizmos()
    {
        // Draw Grid Map (to verify baking)
        if (_gridMap != null)
        {
            for (int x = 0; x < _gridMap.Width; x++)
            {
                for (int y = 0; y < _gridMap.Height; y++)
                {
                    if (_gridMap.IsObstacle(x, y))
                    {
                        Vector2 pos = _gridMap.GridToWorld(x, y);
                        Gizmos.color = new Color(1, 0, 0, 0.3f); // Semi-transparent Red
                        Gizmos.DrawCube(new Vector3(pos.x, 0, pos.y), new Vector3(_gridMap.CellSize, 0.1f, _gridMap.CellSize));
                    }
                }
            }
        }

        // Draw RVO Obstacles (from local list)
        var obstacles = _rvoObstacles;
        if (obstacles != null)
        {
            foreach (var obs in obstacles)
            {
                Vector3 p1 = new Vector3(obs.Point1.x, 0, obs.Point1.y);
                Vector3 p2 = new Vector3(obs.Point2.x, 0, obs.Point2.y);
                
                // Draw Edge
                Gizmos.color = Color.blue; // Use Blue for contrast against Red obstacles
                Gizmos.DrawLine(p1, p2);

                // Draw Normal (Left Direction)
                // RVO assumes Counter-Clockwise winding for obstacles.
                // Left side of the edge is "Interior" (Inward).
                // So this normal points INTO the obstacle.
                Vector3 mid = (p1 + p2) * 0.5f;
                Vector3 dir = (p2 - p1).normalized;
                Vector3 normal = new Vector3(-dir.z, 0, dir.x); // (-y, x) in 2D
                
                Gizmos.color = Color.yellow;
                Gizmos.DrawLine(mid, mid + normal * 0.5f);
            }
        }
    }

    void FixedUpdate()
    {
        float dt = Time.fixedDeltaTime;

        // 1. Update Agent Logic (Pathfinding & Targets)
        UpdateAgentsLogic();

        // 2. Step RVO Simulation
        SIMDRVOSimulator.Instance.Step(dt);

        // 3. Sync Transforms
        SyncTransforms();
    }

    private void InitializeMap()
    {
        _mapWidth = Mathf.CeilToInt(MapSize / CellSize);
        _mapHeight = Mathf.CeilToInt(MapSize / CellSize);
        _origin = new Vector2(-MapSize / 2, -MapSize / 2);

        _gridMap = new GridMap(_mapWidth, _mapHeight, CellSize, _origin);

        // Add Border Walls
        AddRectObstacle(new Rect(_origin.x - 5, _origin.y - 5, MapSize + 10, 5)); // Bottom
        AddRectObstacle(new Rect(_origin.x - 5, _origin.y + MapSize, MapSize + 10, 5)); // Top
        AddRectObstacle(new Rect(_origin.x - 5, _origin.y, 5, MapSize)); // Left
        AddRectObstacle(new Rect(_origin.x + MapSize, _origin.y, 5, MapSize)); // Right

        // Add Random Obstacles
        for (int i = 0; i < ObstacleCount; i++)
        {
            float w = Random.Range(ObstacleSizeMin.x, ObstacleSizeMax.x);
            float h = Random.Range(ObstacleSizeMin.y, ObstacleSizeMax.y);
            float x = Random.Range(_origin.x + 2, _origin.x + MapSize - 2 - w);
            float y = Random.Range(_origin.y + 2, _origin.y + MapSize - 2 - h);
            
            AddRectObstacle(new Rect(x, y, w, h));
        }

        // Initialize Native Obstacles for SIMD A*
        if (_nativeObstacles.IsCreated) _nativeObstacles.Dispose();
        _nativeObstacles = new Unity.Collections.NativeArray<bool>(_mapWidth * _mapHeight, Unity.Collections.Allocator.Persistent);
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
        // 1. Add to GridMap (Rasterize)
        Vector2 min = new Vector2(rect.x, rect.y);
        Vector2 max = new Vector2(rect.x + rect.width, rect.y + rect.height);
        
        Vector2Int minGrid = _gridMap.WorldToGrid(min);
        Vector2Int maxGrid = _gridMap.WorldToGrid(max);

        for (int x = minGrid.x; x <= maxGrid.x; x++)
        {
            for (int y = minGrid.y; y <= maxGrid.y; y++)
            {
                if (_gridMap.IsValid(x, y))
                {
                    _gridMap.SetObstacle(x, y, true);
                }
            }
        }

        // 2. Add to RVO (Geometry)
        // CCW: BL -> BR -> TR -> TL -> BL
        Vector2 bl = min;
        Vector2 br = new Vector2(max.x, min.y);
        Vector2 tr = max;
        Vector2 tl = new Vector2(min.x, max.y);

        // Add visual obstacle
        GameObject cube = GameObject.CreatePrimitive(PrimitiveType.Cube);
        cube.transform.position = new Vector3(rect.center.x, 0.5f, rect.center.y);
        cube.transform.localScale = new Vector3(rect.width, 1, rect.height);
        cube.GetComponent<Renderer>().material.color = Color.red;
        Destroy(cube.GetComponent<Collider>());

        // Note: In RVO AddObstacle takes (Start, End).
        // To define a polygon, we add segments in order.
        // Normals point to the LEFT of the segment.
        // We want normals pointing OUTWARD.
        // So we trace CCW.
        
        // However, RVOSimulator.AddObstacle simply adds raw edges.
        // ConstructObstacleORCALines handles convexity if they are linked.
        // RVOSimulator.ProcessObstacles() links them.
        
        // Bottom: BR -> BL? No, if we walk CCW inside?
        // Wait, for a solid obstacle, "Outward" normal is away from center.
        // Left of (BL -> BR) is UP (Inside).
        // Wait, if "Left" is "Inside", and RVO assumes CCW obstacles...
        // Then normals point INWARD?
        // Let's re-verify RVO2-Unity docs or standard.
        // Usually, RVO2 assumes obstacles are polygons where the interior is on the Left.
        // So Normals (Left) point INTO the obstacle.
        // The constraint is: Agent must be on the RIGHT side of the edge.
        // det(line, point) < 0 means Right.
        
        // Wait, in RVOMath:
        // if (det(obstacleVec, relativePosition1) < 0) continue; // Back-Face Culling
        // This implies det < 0 is "Safe" (or rather "Back/Inside" so ignore?).
        // Actually, if det > 0, it means Left.
        // If "Left" is "Inside", then det > 0 means "Inside".
        // If we want to avoid the obstacle, we should be on the Right.
        
        // So, YES, obstacles should be defined CCW.
        // BL -> BR (Right). Left is Up (In).
        // BR -> TR (Up). Left is Left (In).
        // TR -> TL (Left). Left is Down (In).
        // TL -> BL (Down). Left is Right (In).
        
        AddRVOObstacleToList(bl, br);
        AddRVOObstacleToList(br, tr);
        AddRVOObstacleToList(tr, tl);
        AddRVOObstacleToList(tl, bl);
    }

    private void AddRVOObstacleToList(Vector2 start, Vector2 end)
    {
        // Helper to match RVOSimulator logic but for local list
        RVOObstacle obstacle = new RVOObstacle(new float2(start.x, start.y), new float2(end.x, end.y));
        _rvoObstacles.Add(obstacle);
    }

    private void InitializeRVO()
    {
        // Removed - handled in Start
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
            
            // Initial update to SIMD
            SIMDRVOSimulator.Instance.UpdateAgentData(i, spawnPos, Vector2.zero, Vector2.zero);
        }

        Destroy(template);
    }

    private void UpdateAgentsLogic()
    {
        Vector2Int endGridTemp = new Vector2Int();
        Vector2Int startGridTemp = new Vector2Int();

        foreach (var unit in _agents)
        {
            Vector2 pos2D = new Vector2(unit.GameObject.transform.position.x, unit.GameObject.transform.position.z);

            // 1. Check if reached target
            if (Vector2.Distance(pos2D, unit.TargetPosition) < 1.0f)
            {
                unit.TargetPosition = GetRandomWalkablePoint();
                unit.NextPathUpdateTime = Time.time; // Force update
            }

            // 2. Update Path if needed
            if (Time.time >= unit.NextPathUpdateTime)
            {
                unit.NextPathUpdateTime = Time.time + PathUpdateInterval + Random.Range(0f, 0.2f);
                
                startGridTemp = _gridMap.WorldToGrid(pos2D);
                endGridTemp = _gridMap.WorldToGrid(unit.TargetPosition);

                if (UseSIMD)
                {
                    // Using SIMD Pathfinding (Single Query for simplicity, but could batch)
                    // Note: SIMDAStarPathfinder.FindPath is designed for batch?
                    // It takes pointers.
                    
                    NativeArray<float2> nativePath = new NativeArray<float2>(_gridMap.Width * _gridMap.Height, Allocator.Temp);
                    int pathLength = 0;

                    unsafe
                    {
                        SIMDAStarPathfinder.FindPath(
                            new int2(startGridTemp.x, startGridTemp.y),
                            new int2(endGridTemp.x, endGridTemp.y),
                            _gridMap.Width,
                            _gridMap.Height,
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
                    {
                        unit.Path.Add(new Vector2(nativePath[k].x, nativePath[k].y));
                    }
                    nativePath.Dispose();
                }
                else
                {
                    AStarPathfinder.FindPath(pos2D, unit.TargetPosition, _gridMap, unit.Path);
                }
                unit.PathIndex = 0;
            }

            // 3. Move along path
            Vector2 nextWaypoint = unit.TargetPosition; // Default to target
            
            if (unit.Path != null && unit.Path.Count > 0)
            {
                // Advance path index if close to current waypoint
                while (unit.PathIndex < unit.Path.Count)
                {
                    if (Vector2.Distance(pos2D, unit.Path[unit.PathIndex]) < 1.0f) // Waypoint radius
                    {
                        unit.PathIndex++;
                    }
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
            
            // Update SIMD data (Position/Velocity are updated by Simulator, we update PrefVel)
            // Wait, we need to pass current pos/vel back if we manipulate them?
            // Actually SIMDRVOSimulator maintains state. We just update PrefVel.
            // But UpdateAgentData overwrites everything!
            // We should read current state first?
            // SIMDRVOSimulator doesn't expose partial update.
            // But step updates internal state.
            // We need a SetAgentPrefVelocity in SIMDRVOSimulator?
            // Or just update the struct.
            // Let's assume UpdateAgentData is cheap enough or add a helper.
            // SIMDRVOSimulator.Instance.UpdateAgentData(id, pos, vel, prefVel);
            // We need current Pos/Vel from Simulator.
            
            // Better: Add SetAgentPrefVelocity to SIMDRVOSimulator.
            // For now, I will read, modify pref, write back.
            // Actually, SIMD Simulator holds state in NativeArray.
            // I should just update PrefVelocity.
            // BUT I can't partial update easily without API.
            // I'll stick to updating all, assuming I read back the simulated pos/vel first.
            // But wait, SyncTransforms reads pos.
            // So `_agents[i]` doesn't have latest pos until Sync.
            // Sync happens AFTER Step.
            
            // Update cycle:
            // 1. UpdateAgentsLogic (Decide PrefVel based on Current Pos).
            //    - Current Pos comes from GameObject (synced last frame).
            //    - We calculate PrefVel.
            //    - We need to push PrefVel to Simulator.
            // 2. Step (Simulates).
            // 3. SyncTransforms (Read Pos from Simulator).
            
            // So in UpdateAgentsLogic, I need to send PrefVel.
            // If I use UpdateAgentData, I must send current Pos/Vel too, otherwise I reset them?
            // Yes.
            // So: Vector2 currentPos = ...; Vector2 currentVel = SIMDRVOSimulator.Instance.GetNewVelocity(unit.RVOAgentId);
            // Wait, GetNewVelocity returns last frame's result.
            // Position? SIMDRVOSimulator updates internal position.
            // Does it expose it? No `GetAgentPosition`.
            // I need to add `GetAgentPosition` to SIMDRVOSimulator or assume GameObject pos is sync.
            // It is synced.
            
            Vector2 currentVel = SIMDRVOSimulator.Instance.GetNewVelocity(unit.RVOAgentId);
            SIMDRVOSimulator.Instance.UpdateAgentData(unit.RVOAgentId, pos2D, currentVel, prefVel);
        }
    }

    private void SyncTransforms()
    {
        // Need access to positions from SIMD Simulator
        // SIMDRVOSimulator doesn't expose positions publicy yet!
        // It updates internal _agents NativeArray.
        // I need to add a way to get position.
        // Assuming UpdateAgentData updates the internal state that Step uses?
        // Step uses `_agents` array.
        // UpdateAgentData writes to `_agents`.
        // Step updates `_agents` position?
        // Let's check SIMDRVOSimulator.Step.
        // It writes `newVelocities`.
        // It updates `_agents[i].Velocity` and `Position`.
        // So yes.
        // But I need to read `_agents[i].Position`.
        // SIMDRVOSimulator needs `GetAgentPosition`.
        
        // TEMPORARY FIX: Add GetAgentPosition to SIMDRVOSimulator via reflection or just assume I add it now.
        // I will add it in next step.
        // For now, I write the call assuming it exists.
        
        for (int i = 0; i < _agents.Count; i++)
        {
            Vector2 rvoPos = SIMDRVOSimulator.Instance.GetAgentPosition(i);
            _agents[i].GameObject.transform.position = new Vector3(rvoPos.x, 0, rvoPos.y);
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
            
            // Check clearance (Agent Radius)
            // We check if the agent's bounding box overlaps any obstacle cell
            Vector2 min = target - new Vector2(AgentRadius, AgentRadius);
            Vector2 max = target + new Vector2(AgentRadius, AgentRadius);
            
            Vector2Int minGrid = _gridMap.WorldToGrid(min);
            Vector2Int maxGrid = _gridMap.WorldToGrid(max);
            
            bool valid = true;
            for (int gx = minGrid.x; gx <= maxGrid.x; gx++)
            {
                for (int gy = minGrid.y; gy <= maxGrid.y; gy++)
                {
                    if (_gridMap.IsObstacle(gx, gy))
                    {
                        valid = false;
                        break;
                    }
                }
                if (!valid) break;
            }

            if (valid)
            {
                return target;
            }
        }
        Debug.LogWarning("Failed to find walkable point after " + maxAttempts + " attempts!");
        return _origin + new Vector2(MapSize/2, MapSize/2); // Fallback to center (might be invalid but better than 0,0 if origin is offset)
    }

    private void OnDestroy()
    {
        if (_nativeObstacles.IsCreated) _nativeObstacles.Dispose();
    }
}
