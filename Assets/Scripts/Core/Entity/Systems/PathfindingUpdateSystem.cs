using Unity.Mathematics;
using UnityEngine;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;

[UpdateInGroup(SystemGroup.FixedUpdate, Order = -1)] // Before RVO
public class PathfindingUpdateSystem : ISystem
{
    public bool Enabled { get; set; } = true;
    public bool UseSIMD { get; set; } = true;
    public GridMap GridMap;
    public NativeArray<bool> NativeObstacles;

    private Dictionary<int, List<Vector2>> _entityPaths = new Dictionary<int, List<Vector2>>();
    private Dictionary<int, int> _entityPathIndices = new Dictionary<int, int>();

    // For SIMD buffer reuse (optional optimization)
    // private NativeArray<float2> _pathBuffer; 

    public void Initialize()
    {
    }

    public void Setup(GridMap gridMap, NativeArray<bool> nativeObstacles)
    {
        GridMap = gridMap;
        NativeObstacles = nativeObstacles;
    }

    public void Shutdown()
    {
        _entityPaths.Clear();
        _entityPathIndices.Clear();
    }

    public void Update(float dt)
    {
        if (!Enabled) return;
        if (GridMap == null) return;

        var entityManager = EntityManager.Instance;
        var positions = entityManager.GetArray<PositionComponent>();
        var states = entityManager.GetArray<MovementState>();
        var agentParams = entityManager.GetArray<AgentParameters>();

        int count = states.Count;
        for (int i = 0; i < count; i++)
        {
            ref var state = ref states.GetDenseRef(i);
            int entityId = states.GetEntityIdFromDenseIndex(i);

            if (!positions.Has(entityId) || !agentParams.Has(entityId)) continue;

            ref var pos = ref positions.GetRef(entityId);
            ref readonly var param = ref agentParams.GetReadOnly(entityId);

            float2 currentPos = pos.Value;
            float2 targetPos = state.TargetPosition;

            // 1. Path Planning
            if (!state.HasPath)
            {
                if (!_entityPaths.ContainsKey(entityId)) _entityPaths[entityId] = new List<Vector2>();
                var path = _entityPaths[entityId];
                path.Clear();

                if (UseSIMD)
                {
                    CalculatePathSIMD(currentPos, targetPos, path);
                }
                else
                {
                    AStarPathfinder.FindPath(new Vector2(currentPos.x, currentPos.y), new Vector2(targetPos.x, targetPos.y), GridMap, path);
                }

                _entityPathIndices[entityId] = 0;
                state.HasPath = true;
            }

            // 2. Path Following
            if (_entityPaths.TryGetValue(entityId, out var currentPath) && currentPath.Count > 0)
            {
                int pathIndex = _entityPathIndices.ContainsKey(entityId) ? _entityPathIndices[entityId] : 0;

                // Advance Waypoints
                while (pathIndex < currentPath.Count)
                {
                    float dist = math.distance(currentPos, new float2(currentPath[pathIndex].x, currentPath[pathIndex].y));
                    if (dist < 0.5f) // Waypoint reached radius
                    {
                        pathIndex++;
                    }
                    else
                    {
                        break;
                    }
                }
                _entityPathIndices[entityId] = pathIndex;

                if (pathIndex < currentPath.Count)
                {
                    // Move towards waypoint
                    float2 waypoint = new float2(currentPath[pathIndex].x, currentPath[pathIndex].y);
                    float2 dir = waypoint - currentPos;
                    float lenSq = math.lengthsq(dir);
                    if (lenSq > 0.001f)
                    {
                        state.PreferredVelocity = math.normalize(dir) * param.MaxSpeed;
                    }
                    else
                    {
                        state.PreferredVelocity = float2.zero;
                    }
                }
                else
                {
                    // End of path
                    // Move towards final target (should be same as last waypoint, but just in case)
                    float2 dir = targetPos - currentPos;
                    float lenSq = math.lengthsq(dir);
                    if (lenSq > 0.1f) // Stop radius
                    {
                        state.PreferredVelocity = math.normalize(dir) * param.MaxSpeed;
                    }
                    else
                    {
                        state.PreferredVelocity = float2.zero;
                        // Ideally we notify that we reached target, but for now we just stop.
                        // The Demo script should check distance to TargetPosition and assign new one.
                    }
                }
            }
            else
            {
                // No path or empty path, just move direct (fallback)
                float2 dir = targetPos - currentPos;
                if (math.lengthsq(dir) > 0.1f)
                {
                    state.PreferredVelocity = math.normalize(dir) * param.MaxSpeed;
                }
                else
                {
                    state.PreferredVelocity = float2.zero;
                }
            }
        }
    }

    private void CalculatePathSIMD(float2 start, float2 end, List<Vector2> resultPath)
    {
        Vector2Int startGrid = GridMap.WorldToGrid(new Vector2(start.x, start.y));
        Vector2Int endGrid = GridMap.WorldToGrid(new Vector2(end.x, end.y));

        int maxPathLen = GridMap.Width * GridMap.Height;
        NativeArray<float2> pathBuffer = new NativeArray<float2>(maxPathLen, Allocator.Temp);
        int pathLen = 0;

        unsafe
        {
            SIMDAStarPathfinder.FindPath(
               new int2(startGrid.x, startGrid.y),
               new int2(endGrid.x, endGrid.y),
               GridMap.Width, GridMap.Height,
               NativeObstacles,
               (float2*)pathBuffer.GetUnsafePtr(),
               ref pathLen,
               maxPathLen,
               new float2(GridMap.Origin.x, GridMap.Origin.y),
               GridMap.CellSize
           );
        }

        for (int k = 0; k < pathLen; k++)
        {
            resultPath.Add(new Vector2(pathBuffer[k].x, pathBuffer[k].y));
        }
        pathBuffer.Dispose();
    }

    public List<Vector2> GetPath(int entityId)
    {
        if (_entityPaths.TryGetValue(entityId, out var path)) return path;
        return null;
    }
}
