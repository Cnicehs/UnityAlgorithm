using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using System.Diagnostics;
using Random = UnityEngine.Random;

public class PathfindingComparisonDemo : ComparisonDemoBase
{
    [Header("Map Settings")]
    public int Width = 100;
    public int Height = 100;
    public float CellSize = 1.0f;
    [Range(0, 1)]
    public float ObstacleDensity = 0.2f;

    [Header("Path Settings")]
    public int PathCount = 100;

    private GridMap _gridMap;
    private NativeArray<bool> _nativeObstacles;
    
    // Store start/end pairs for fairness
    private struct PathRequest
    {
        public Vector2 Start;
        public Vector2 End;
    }
    private List<PathRequest> _requests = new List<PathRequest>();

    void Start()
    {
        AgentCount = PathCount;
        InitializeMap();
        GenerateRequests();
    }

    void OnDestroy()
    {
        if (_nativeObstacles.IsCreated) _nativeObstacles.Dispose();
    }

    private void InitializeMap()
    {
        _gridMap = new GridMap(Width, Height, CellSize, new Vector2(-Width * CellSize / 2, -Height * CellSize / 2));
        
        // Random obstacles
        int obstaclesToPlace = (int)(Width * Height * ObstacleDensity);
        for (int i = 0; i < obstaclesToPlace; i++)
        {
            int x = Random.Range(0, Width);
            int y = Random.Range(0, Height);
            _gridMap.SetObstacle(x, y, true);
        }

        // Native obstacles for SIMD
        _nativeObstacles = new NativeArray<bool>(Width * Height, Allocator.Persistent);
        for (int y = 0; y < Height; y++)
        {
            for (int x = 0; x < Width; x++)
            {
                _nativeObstacles[y * Width + x] = _gridMap.IsObstacle(x, y);
            }
        }
    }

    private void GenerateRequests()
    {
        _requests.Clear();
        for (int i = 0; i < PathCount; i++)
        {
            Vector2 start = GetRandomWalkablePos();
            Vector2 end = GetRandomWalkablePos();
            _requests.Add(new PathRequest { Start = start, End = end });
        }
    }

    private Vector2 GetRandomWalkablePos()
    {
        int attempts = 100;
        while (attempts-- > 0)
        {
            int x = Random.Range(0, Width);
            int y = Random.Range(0, Height);
            if (!_gridMap.IsObstacle(x, y))
            {
                return _gridMap.GridToWorld(x, y);
            }
        }
        return Vector2.zero;
    }

    void Update()
    {
        Stopwatch.Restart();
        
        if (UseSIMD)
        {
            RunSIMDPathfinding();
        }
        else
        {
            RunDefaultPathfinding();
        }

        Stopwatch.Stop();
        ExecutionTimeMs = (float)Stopwatch.Elapsed.TotalMilliseconds;
    }

    private void RunDefaultPathfinding()
    {
        List<Vector2> path = new List<Vector2>();
        foreach (var req in _requests)
        {
            AStarPathfinder.FindPath(req.Start, req.End, _gridMap, path);
        }
    }

    private void RunSIMDPathfinding()
    {
        int maxPathLen = Width * Height;
        // Allocator.Temp is fast and auto-cleared at end of frame (or explicitly disposed)
        NativeArray<float2> pathBuffer = new NativeArray<float2>(maxPathLen, Allocator.Temp); 
        
        unsafe
        {
            float2* ptr = (float2*)pathBuffer.GetUnsafePtr();
            
            foreach (var req in _requests)
            {
                Vector2Int startGrid = _gridMap.WorldToGrid(req.Start);
                Vector2Int endGrid = _gridMap.WorldToGrid(req.End);
                int pathLen = 0;

                SIMDAStarPathfinder.FindPath(
                    new int2(startGrid.x, startGrid.y),
                    new int2(endGrid.x, endGrid.y),
                    Width, Height,
                    _nativeObstacles,
                    ptr,
                    ref pathLen,
                    maxPathLen,
                    new float2(_gridMap.Origin.x, _gridMap.Origin.y),
                    CellSize
                );
            }
        }
        
        pathBuffer.Dispose();
    }
    
    protected override void OnGUIDisplayExtra()
    {
        GUILayout.Label($"Map Size: {Width}x{Height}");
        GUILayout.Label($"Path Requests: {PathCount}");
    }
    
    void OnDrawGizmosSelected()
    {
        if (_gridMap != null)
        {
            Gizmos.color = Color.red;
            for (int x=0; x<Width; x++)
            {
                for (int y=0; y<Height; y++)
                {
                    if (_gridMap.IsObstacle(x,y))
                    {
                        Vector2 pos = _gridMap.GridToWorld(x,y);
                        Gizmos.DrawWireCube(new Vector3(pos.x, 0, pos.y), new Vector3(CellSize, 0.1f, CellSize));
                    }
                }
            }
        }
    }
}
