using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;

public class PathfindingSystem : MonoBehaviour
{
    public static PathfindingSystem Instance { get; private set; }

    [Header("Settings")]
    public bool UseSIMDPathfinding = true;
    public float PathUpdateInterval = 0.5f;

    [Header("RVO Settings")]
    public float TimeHorizon = 0.5f;
    public float NeighborDist = 10.0f;
    public int MaxNeighbors = 20;

    [Header("References")]
    public GridMap GridMap;

    private List<EnemyUnit> _units = new List<EnemyUnit>();
    private NativeArray<bool> _nativeObstacles;

    private void Awake()
    {
        if (Instance == null) Instance = this;
        else Destroy(gameObject);
    }

    public void Initialize(GridMap gridMap, NativeArray<bool> nativeObstacles, int width, int height, bool useSIMD)
    {
        GridMap = gridMap;
        // We can either use the passed nativeObstacles (if they are persistent) or create our own.
        // SurvivorDemo seems to pass them, but PathfindingSystem also has UpdateNativeObstacles.
        // Let's use our own for safety and consistency, or just copy.
        // Actually, SurvivorDemo passes _nativeObstacles which might not be initialized yet or managed there.
        // Let's stick to PathfindingSystem managing it.
        // But the error says Initialize takes 5 arguments in the call, but defined with 1.
        // Let's update the definition to match the call, but maybe ignore some if we self-manage.

        UseSIMDPathfinding = useSIMD;
        UpdateNativeObstacles();
    }

    public void RegisterUnit(EnemyUnit unit)
    {
        if (!_units.Contains(unit))
        {
            _units.Add(unit);
            // Initialize RVO agent for this unit if not already done?
            // Assuming SurvivorDemo handles RVO agent creation for now, 
            // but ideally this system should handle it.
        }
    }

    public void UnregisterUnit(EnemyUnit unit)
    {
        _units.Remove(unit);
    }

    public void UpdateSystem(float dt, Vector2 playerPos)
    {
        // Apply RVO Settings
        RVOSimulator.Instance.TimeHorizon = TimeHorizon;
        // RVOSimulator.Instance.TimeHorizonObst = TimeHorizonObst; // Assuming this exists or we use TimeHorizon for both
        RVOSimulator.Instance.NeighborDist = NeighborDist;
        RVOSimulator.Instance.MaxNeighbors = MaxNeighbors;

        // 1. Update Paths (Pipeline Step 1)
        UpdatePaths(playerPos);

        // 2. Update RVO (Pipeline Step 2)
        // Set preferred velocities based on paths
        UpdatePreferredVelocities(playerPos);

        // 3. Step RVO Simulation
        RVOSimulator.Instance.Step(dt);
    }

    private void UpdatePaths(Vector2 playerPos)
    {
        // Periodic path updates
        // For SIMD, we could batch this. For now, keep per-unit logic but centralized.

        // Validate Target once per frame?
        Vector2 validTargetPos = GetValidTarget(playerPos);
        Vector2Int endGrid = GridMap.WorldToGrid(validTargetPos);

        foreach (var unit in _units)
        {
            if (Time.time > unit.NextPathUpdateTime)
            {
                unit.NextPathUpdateTime = Time.time + PathUpdateInterval + UnityEngine.Random.Range(0f, 0.2f);
                Vector2 unitPos = RVOSimulator.Instance.GetAgentPosition(unit.RVOAgentId);
                Vector2 validStartPos = GetValidTarget(unitPos);
                Vector2Int startGrid = GridMap.WorldToGrid(validStartPos);

                if (UseSIMDPathfinding)
                {
                    // Allocate max possible path length (map size)
                    // In a real system, we would reuse this buffer or use a shared one.
                    NativeArray<float2> nativePath = new NativeArray<float2>(GridMap.Width * GridMap.Height, Allocator.Temp);
                    int pathLength = 0;

                    unsafe
                    {
                        SIMDAStarPathfinder.FindPath(
                            new int2(startGrid.x, startGrid.y),
                            new int2(endGrid.x, endGrid.y),
                            GridMap.Width,
                            GridMap.Height,
                            _nativeObstacles,
                            (float2*)nativePath.GetUnsafePtr(),
                            ref pathLength,
                            nativePath.Length,
                            new float2(GridMap.Origin.x, GridMap.Origin.y),
                            GridMap.CellSize
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
                    AStarPathfinder.FindPath(validStartPos, validTargetPos, GridMap, unit.Path);
                }

                unit.PathIndex = 0;
            }
        }
    }

    private void UpdatePreferredVelocities(Vector2 playerPos)
    {
        float attackRangeSq = 1.5f * 1.5f; // Hardcoded for now, should come from unit

        foreach (var unit in _units)
        {
            Vector2 currentPos = RVOSimulator.Instance.GetAgentPosition(unit.RVOAgentId);
            float distToPlayerSq = (playerPos - currentPos).sqrMagnitude;

            Vector2 moveDir = Vector2.zero;

            if (distToPlayerSq > attackRangeSq)
            {
                // Follow Path
                if (unit.Path != null && unit.Path.Count > 0)
                {
                    // Advance path index
                    while (unit.PathIndex < unit.Path.Count - 1)
                    {
                        float distToNode = Vector2.Distance(currentPos, unit.Path[unit.PathIndex]);
                        if (distToNode < 0.5f) // Reached node
                        {
                            unit.PathIndex++;
                        }
                        else
                        {
                            break;
                        }
                    }

                    Vector2 target = unit.Path[unit.PathIndex];
                    
                    // Add Jitter to target to prevent overlap
                    // Use agent ID to make it deterministic but different per agent
                    // Or just random? Random might cause jittering movement.
                    // Deterministic based on ID is better.
                    float jitterAmount = 0.2f;
                    float angle = (unit.AgentId * 137.5f) * Mathf.Deg2Rad; // Golden angle
                    Vector2 jitter = new Vector2(Mathf.Cos(angle), Mathf.Sin(angle)) * jitterAmount;
                    
                    Vector2 targetWithJitter = target + jitter;
                    
                    moveDir = (targetWithJitter - currentPos).normalized;
                }
            }

            // Apply to RVO
            float speed = 3.0f; // Default enemy speed
            Vector2 prefVel = moveDir * speed;
            
            // Clamp to max speed (though normalized * speed is already clamped if speed <= maxSpeed)
            // But just in case
            if (prefVel.sqrMagnitude > speed * speed)
            {
                prefVel = prefVel.normalized * speed;
            }
            
            RVOSimulator.Instance.SetAgentPrefVelocity(unit.RVOAgentId, prefVel);
        }
    }

    public Vector2 GetValidTarget(Vector2 targetPos)
    {
        Vector2Int gridPos = GridMap.WorldToGrid(targetPos);
        if (!GridMap.IsObstacle(gridPos.x, gridPos.y))
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
                        if (!GridMap.IsObstacle(checkX, checkY))
                        {
                            return GridMap.GridToWorld(checkX, checkY);
                        }
                    }
                }
            }
            radius++;
        }

        return targetPos; // Give up
    }

    public void UpdateNativeObstacles()
    {
        if (_nativeObstacles.IsCreated) _nativeObstacles.Dispose();
        _nativeObstacles = new NativeArray<bool>(GridMap.Width * GridMap.Height, Allocator.Persistent);
        for (int y = 0; y < GridMap.Height; y++)
        {
            for (int x = 0; x < GridMap.Width; x++)
            {
                _nativeObstacles[y * GridMap.Width + x] = GridMap.Obstacles[x, y];
            }
        }
    }

    private void OnDestroy()
    {
        if (_nativeObstacles.IsCreated) _nativeObstacles.Dispose();
    }
}
