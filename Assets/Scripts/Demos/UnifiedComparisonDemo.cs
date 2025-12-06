using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;
using Unity.Collections;
using System.Diagnostics;
using Random = UnityEngine.Random;

public class UnifiedComparisonDemo : MonoBehaviour
{
    public enum DemoMode { Scene, Pathfinding, RVO, PathfindingRVO }
    public enum AlgorithmMode { Default, SIMD }

    [Header("Configuration")]
    public DemoMode CurrentMode = DemoMode.PathfindingRVO;
    public AlgorithmMode CurrentAlgorithm = AlgorithmMode.SIMD;

    [Header("Map Settings")]
    public int Width = 50;
    public int Height = 50;
    public float CellSize = 1.0f;
    public int ObstacleCount = 20;
    public Vector2 ObstacleSizeMin = new Vector2(2, 2);
    public Vector2 ObstacleSizeMax = new Vector2(5, 5);

    [Header("Agent Settings")]
    [Range(0, 1000)]
    public int AgentCount = 100;
    public float AgentSpeed = 5.0f;
    public float AgentRadius = 0.5f;

    // Systems
    private PathfindingUpdateSystem _pathfindingSystem;
    private RVOSimulationSystem _rvoSystemSIMD;
    private LegacyRVOSystem _rvoSystemDefault;

    // Data
    private GridMap _gridMap;
    private NativeArray<bool> _nativeObstacles;
    private List<RVOObstacle> _rvoObstacles = new List<RVOObstacle>();
    private List<Entity> _entities = new List<Entity>();

    // Performance
    private float _lastExecutionTime;
    private Stopwatch _stopwatch = new Stopwatch();

    void Start()
    {
        InitializeMap();
        InitializeSystems();
    }

    void OnDestroy()
    {
        if (_nativeObstacles.IsCreated) _nativeObstacles.Dispose();

        // Do not shutdown global systems or EntityManager, as they persist across scene loads (if using SystemBootstrapper)
        // _pathfindingSystem?.Shutdown();
        // _rvoSystemSIMD?.Shutdown();
        
        _rvoSystemDefault?.Shutdown(); // Legacy is local

        // Destroy all created entities to clean up EntityManager state
        foreach (var e in _entities)
        {
            EntityManager.Instance.DestroyEntity(e);
        }
        _entities.Clear();

        // EntityManager.Instance.Shutdown(); 
        SpatialIndexManager.Instance.Shutdown();
    }

    private void InitializeMap()
    {
        Vector2 origin = new Vector2(-Width * CellSize / 2, -Height * CellSize / 2);
        _gridMap = new GridMap(Width, Height, CellSize, origin);

        // Borders
        AddRectObstacle(new Rect(origin.x - 5, origin.y - 5, Width * CellSize + 10, 5));
        AddRectObstacle(new Rect(origin.x - 5, origin.y + Height * CellSize, Width * CellSize + 10, 5));
        AddRectObstacle(new Rect(origin.x - 5, origin.y, 5, Height * CellSize));
        AddRectObstacle(new Rect(origin.x + Width * CellSize, origin.y, 5, Height * CellSize));

        // Random Obstacles
        for (int i = 0; i < ObstacleCount; i++)
        {
            float w = Random.Range(ObstacleSizeMin.x, ObstacleSizeMax.x);
            float h = Random.Range(ObstacleSizeMin.y, ObstacleSizeMax.y);
            float x = Random.Range(origin.x + 2, origin.x + Width * CellSize - 2 - w);
            float y = Random.Range(origin.y + 2, origin.y + Height * CellSize - 2 - h);
            AddRectObstacle(new Rect(x, y, w, h));
        }

        // Native Obstacles for SIMD
        _nativeObstacles = new NativeArray<bool>(Width * Height, Allocator.Persistent);
        for (int y = 0; y < Height; y++)
        {
            for (int x = 0; x < Width; x++)
            {
                _nativeObstacles[y * Width + x] = _gridMap.IsObstacle(x, y);
            }
        }
    }

    private void AddRectObstacle(Rect rect)
    {
        // Grid
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

        // RVO
        Vector2 bl = min;
        Vector2 br = new Vector2(max.x, min.y);
        Vector2 tr = max;
        Vector2 tl = new Vector2(min.x, max.y);

        _rvoObstacles.Add(new RVOObstacle(new float2(bl.x, bl.y), new float2(br.x, br.y)));
        _rvoObstacles.Add(new RVOObstacle(new float2(br.x, br.y), new float2(tr.x, tr.y)));
        _rvoObstacles.Add(new RVOObstacle(new float2(tr.x, tr.y), new float2(tl.x, tl.y)));
        _rvoObstacles.Add(new RVOObstacle(new float2(tl.x, tl.y), new float2(bl.x, bl.y)));

        // Visual
        GameObject cube = GameObject.CreatePrimitive(PrimitiveType.Cube);
        cube.transform.position = new Vector3(rect.center.x, 0.5f, rect.center.y);
        cube.transform.localScale = new Vector3(rect.width, 1, rect.height);
        cube.GetComponent<Renderer>().material.color = Color.gray;
        Destroy(cube.GetComponent<Collider>());
    }

    private void InitializeSystems()
    {
        // Retrieve global systems (bootstrapped)
        _pathfindingSystem = EntityManager.Instance.GetSystem<PathfindingUpdateSystem>();
        if (_pathfindingSystem == null) UnityEngine.Debug.LogError("PathfindingUpdateSystem not found! Is SystemBootstrapper running?");
        else
        {
            _pathfindingSystem.Setup(_gridMap, _nativeObstacles);
        }

        _rvoSystemSIMD = EntityManager.Instance.GetSystem<RVOSimulationSystem>();
        if (_rvoSystemSIMD == null) UnityEngine.Debug.LogError("RVOSimulationSystem not found! Is SystemBootstrapper running?");
        else
        {
            _rvoSystemSIMD.UpdateObstacles(_rvoObstacles);
        }

        // Legacy system is local (no UpdateInGroup attribute)
        _rvoSystemDefault = new LegacyRVOSystem();
        _rvoSystemDefault.Initialize();
        
        RVOSimulator.Instance.ClearObstacles();
        foreach (var obs in _rvoObstacles)
        {
            RVOSimulator.Instance.AddObstacle(new Vector3(obs.Point1.x, 0, obs.Point1.y), new Vector3(obs.Point2.x, 0, obs.Point2.y));
        }
        RVOSimulator.Instance.ProcessObstacles();

        SpatialIndexManager.Instance.Initialize(1000, Width * CellSize);
    }

    void Update()
    {
        UpdateAgentCount();
        UpdateTargets(); // Simplified "Brain" logic

        _stopwatch.Restart();

        // 1. Pathfinding
        bool runPathfinding = (CurrentMode == DemoMode.Pathfinding || CurrentMode == DemoMode.PathfindingRVO);
        if (_pathfindingSystem != null)
        {
            _pathfindingSystem.Enabled = runPathfinding;
            _pathfindingSystem.UseSIMD = (CurrentAlgorithm == AlgorithmMode.SIMD);
            // System runs automatically in PlayerLoop FixedUpdate
        }

        // 2. RVO / Movement
        bool runRVO = (CurrentMode == DemoMode.RVO || CurrentMode == DemoMode.PathfindingRVO);
        bool useSIMD = (CurrentAlgorithm == AlgorithmMode.SIMD);

        // If RVO is enabled but Pathfinding is NOT, we need to manually calculate PreferredVelocity
        // because usually PathfindingSystem does it.
        if (runRVO && !runPathfinding)
        {
            CalculateDirectPreferredVelocity();
        }

        if (_rvoSystemSIMD != null)
        {
            // Enable SIMD system only if RVO is ON and SIMD is selected
            _rvoSystemSIMD.Enabled = runRVO && useSIMD;
            // System runs automatically in PlayerLoop FixedUpdate
        }

        if (runRVO && !useSIMD)
        {
            // Manual update for Legacy
            _rvoSystemDefault.Update(Time.deltaTime);
        }

        if (!runRVO && runPathfinding)
        {
            // Pathfinding sets PreferredVelocity, but no RVO to move agents.
            // We need simple integration.
            SimpleIntegration(Time.deltaTime);
        }
        
        _stopwatch.Stop();
        // Since we delegated to FixedUpdate, this stopwatch only measures overhead + Legacy + SimpleIntegration.
        // It does NOT measure SIMD RVO or Pathfinding time accurately anymore.
        // We will sum up LastExecutionTime from systems.
        
        float totalTime = (float)_stopwatch.Elapsed.TotalMilliseconds;
        if (_pathfindingSystem != null && _pathfindingSystem.Enabled) totalTime += 0; // Can't easily measure FixedUpdate time from Update
        // Actually, we can just read LastExecutionTime from systems!
        
        _lastExecutionTime = 0;
        if (_pathfindingSystem != null && _pathfindingSystem.Enabled) 
            _lastExecutionTime += 0; // PathfindingSystem doesn't have LastExecutionTime yet. 
            // We should have added it. But assuming the user task focused on RVOSimulationSystem.
            // For now, ignore Pathfinding time or add it later if needed.
        
        if (_rvoSystemSIMD != null && _rvoSystemSIMD.Enabled)
            _lastExecutionTime += _rvoSystemSIMD.LastExecutionTime;
            
        if (runRVO && !useSIMD)
            _lastExecutionTime += totalTime; // Legacy runs here
            
        // Note: Reporting "Execution Time" when mixed with FixedUpdate and Update is tricky.
        // The UI will show last frame's FixedUpdate time + current frame's overhead.

        RenderAgents();
    }

    private void UpdateAgentCount()
    {
        int diff = AgentCount - _entities.Count;
        if (diff > 0)
        {
            for (int i = 0; i < diff; i++) SpawnAgent();
        }
        else if (diff < 0)
        {
            for (int i = 0; i < -diff; i++)
            {
                if (_entities.Count > 0)
                {
                    Entity e = _entities[_entities.Count - 1];
                    EntityManager.Instance.DestroyEntity(e);
                    _entities.RemoveAt(_entities.Count - 1);
                }
            }
        }
    }

    private void SpawnAgent()
    {
        Entity e = EntityManager.Instance.CreateEntity();
        _entities.Add(e);

        Vector2 pos = GetRandomWalkablePos();

        EntityManager.Instance.AddComponent(e, new PositionComponent { Value = new float2(pos.x, pos.y) });
        EntityManager.Instance.AddComponent(e, new VelocityComponent { Value = float2.zero });
        EntityManager.Instance.AddComponent(e, new RadiusComponent { Value = AgentRadius });
        EntityManager.Instance.AddComponent(e, new AgentParameters
        {
            MaxSpeed = AgentSpeed,
            NeighborDist = 10.0f,
            MaxNeighbors = 10,
            TimeHorizon = 2.0f,
            TimeHorizonObst = 2.0f
        });
        EntityManager.Instance.AddComponent(e, new MovementState
        {
            TargetPosition = new float2(pos.x, pos.y), // Initial target is self
            PreferredVelocity = float2.zero,
            HasPath = false
        });
    }

    private Vector2 GetRandomWalkablePos()
    {
        for (int i = 0; i < 50; i++)
        {
            float x = Random.Range(_gridMap.Origin.x + 1, _gridMap.Origin.x + _gridMap.Width * _gridMap.CellSize - 1);
            float y = Random.Range(_gridMap.Origin.y + 1, _gridMap.Origin.y + _gridMap.Height * _gridMap.CellSize - 1);
            if (!_gridMap.IsObstacle(_gridMap.WorldToGrid(new Vector2(x, y)).x, _gridMap.WorldToGrid(new Vector2(x, y)).y))
                return new Vector2(x, y);
        }
        return Vector2.zero;
    }

    private void UpdateTargets()
    {
        // Simple brain: If reached target (dist < 1), pick new random target
        var entityManager = EntityManager.Instance;
        var positions = entityManager.GetArray<PositionComponent>();
        var states = entityManager.GetArray<MovementState>();

        for (int i = 0; i < _entities.Count; i++)
        {
            Entity e = _entities[i];
            if (!positions.Has(e.Id) || !states.Has(e.Id)) continue;

            ref var pos = ref positions.GetRef(e.Id);
            ref var state = ref states.GetRef(e.Id);

            if (math.distance(pos.Value, state.TargetPosition) < 1.0f)
            {
                Vector2 newTarget = GetRandomWalkablePos();
                state.TargetPosition = new float2(newTarget.x, newTarget.y);
                state.HasPath = false; // Trigger re-path
            }
        }
    }

    private void CalculateDirectPreferredVelocity()
    {
        var entityManager = EntityManager.Instance;
        var positions = entityManager.GetArray<PositionComponent>();
        var states = entityManager.GetArray<MovementState>();
        var agentParams = entityManager.GetArray<AgentParameters>();

        for (int i = 0; i < _entities.Count; i++)
        {
            Entity e = _entities[i];
            if (!positions.Has(e.Id) || !states.Has(e.Id)) continue;

            ref var pos = ref positions.GetRef(e.Id);
            ref var state = ref states.GetRef(e.Id);
            float maxSpeed = agentParams.Has(e.Id) ? agentParams.GetRef(e.Id).MaxSpeed : AgentSpeed;

            float2 dir = state.TargetPosition - pos.Value;
            float distSq = math.lengthsq(dir);
            if (distSq > 0.001f)
            {
                state.PreferredVelocity = math.normalize(dir) * maxSpeed;
            }
            else
            {
                state.PreferredVelocity = float2.zero;
            }
        }
    }

    private void SimpleIntegration(float dt)
    {
        var entityManager = EntityManager.Instance;
        var positions = entityManager.GetArray<PositionComponent>();
        var velocities = entityManager.GetArray<VelocityComponent>();
        var states = entityManager.GetArray<MovementState>();

        for (int i = 0; i < _entities.Count; i++)
        {
            Entity e = _entities[i];
            if (!positions.Has(e.Id) || !velocities.Has(e.Id) || !states.Has(e.Id)) continue;

            ref var pos = ref positions.GetRef(e.Id);
            ref var vel = ref velocities.GetRef(e.Id);
            ref var state = ref states.GetRef(e.Id);

            vel.Value = state.PreferredVelocity;
            pos.Value += vel.Value * dt;
        }
    }

    // Rendering
    private Mesh _agentMesh;
    private Material _agentMaterial;
    private Matrix4x4[] _matrices;

    private void RenderAgents()
    {
        if (_agentMesh == null)
        {
            GameObject temp = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
            _agentMesh = temp.GetComponent<MeshFilter>().sharedMesh;
            _agentMaterial = new Material(Shader.Find("Standard"));
            _agentMaterial.enableInstancing = true;
            _agentMaterial.color = Color.blue;
            Destroy(temp);
        }

        if (_matrices == null || _matrices.Length < _entities.Count)
        {
            _matrices = new Matrix4x4[Mathf.Max(1023, _entities.Count)]; // Batch size 1023
        }

        var positions = EntityManager.Instance.GetArray<PositionComponent>();
        int batchCount = 0;

        for (int i = 0; i < _entities.Count; i++)
        {
            Entity e = _entities[i];
            if (!positions.Has(e.Id)) continue;

            float2 p = positions.GetRef(e.Id).Value;
            _matrices[batchCount] = Matrix4x4.TRS(new Vector3(p.x, 0, p.y), Quaternion.identity, new Vector3(AgentRadius * 2, 1, AgentRadius * 2));
            batchCount++;

            if (batchCount >= 1023)
            {
                Graphics.DrawMeshInstanced(_agentMesh, 0, _agentMaterial, _matrices, batchCount);
                batchCount = 0;
            }
        }
        if (batchCount > 0)
        {
            Graphics.DrawMeshInstanced(_agentMesh, 0, _agentMaterial, _matrices, batchCount);
        }
    }

    void OnGUI()
    {
        GUILayout.BeginArea(new Rect(10, 10, 300, 400));
        GUILayout.Box("Unified Comparison Demo");

        GUILayout.Label($"Agent Count: {AgentCount}");
        AgentCount = (int)GUILayout.HorizontalSlider(AgentCount, 0, 1000);

        GUILayout.Label("Mode:");
        if (GUILayout.Button($"Current: {CurrentMode}"))
        {
            CurrentMode = (DemoMode)(((int)CurrentMode + 1) % 4);
        }

        GUILayout.Label("Algorithm:");
        if (GUILayout.Button($"Current: {CurrentAlgorithm}"))
        {
            CurrentAlgorithm = (AlgorithmMode)(((int)CurrentAlgorithm + 1) % 2);
        }

        GUILayout.Space(10);
        GUILayout.Label($"RVO Time: {_lastExecutionTime:F2} ms");
        GUILayout.Label($"FPS: {1.0f / Time.smoothDeltaTime:F1}");

        GUILayout.EndArea();
    }

    void OnDrawGizmos()
    {
        // Draw Grid Boundaries
        if (_gridMap != null)
        {
            Gizmos.color = Color.white;
            Vector3 origin = new Vector3(_gridMap.Origin.x, 0, _gridMap.Origin.y);
            Vector3 size = new Vector3(_gridMap.Width * _gridMap.CellSize, 0, _gridMap.Height * _gridMap.CellSize);
            Gizmos.DrawWireCube(origin + size * 0.5f, size);
        }

        // Draw RVO Obstacles
        if (_rvoObstacles != null)
        {
            Gizmos.color = Color.red;
            foreach (var obs in _rvoObstacles)
            {
                Gizmos.DrawLine(new Vector3(obs.Point1.x, 0.5f, obs.Point1.y), new Vector3(obs.Point2.x, 0.5f, obs.Point2.y));

                Vector2 dir = (obs.Point2 - obs.Point1); 
                Vector2 normal = new Vector2(-dir.y, dir.x).normalized; 
                Vector3 mid = new Vector3((obs.Point1.x + obs.Point2.x) * 0.5f, 0.5f, (obs.Point1.y + obs.Point2.y) * 0.5f);
                Gizmos.DrawLine(mid, mid + new Vector3(normal.x, 0, normal.y) * 0.5f);
            }
        }

        // Draw Paths (optional)
        if (_pathfindingSystem != null)
        {
            Gizmos.color = Color.yellow;
            foreach (var e in _entities)
            {
                var path = _pathfindingSystem.GetPath(e.Id);
                if (path != null)
                {
                    for (int i = 0; i < path.Count - 1; ++i)
                        Gizmos.DrawLine(new Vector3(path[i].x, 0.1f, path[i].y), new Vector3(path[i + 1].x, 0.1f, path[i + 1].y));
                }
            }

            // Draw SIMD RVO Neighbor Debug
            if ((CurrentMode == DemoMode.RVO || CurrentMode == DemoMode.PathfindingRVO) && CurrentAlgorithm == AlgorithmMode.SIMD && _rvoSystemSIMD != null && _rvoObstacles != null)
            {
                Gizmos.color = Color.yellow;
                List<int> neighborIndices = new List<int>();
                // Just check first agent (index 0)
                _rvoSystemSIMD.GetDebugObstacleNeighbors(0, neighborIndices);
                foreach (int idx in neighborIndices)
                {
                    if (idx >= 0 && idx < _rvoObstacles.Count)
                    {
                        var obs = _rvoObstacles[idx];
                        Vector3 p1 = new Vector3(obs.Point1.x, 0.5f, obs.Point1.y);
                        Vector3 p2 = new Vector3(obs.Point2.x, 0.5f, obs.Point2.y);
                        Gizmos.DrawLine(p1, p2);
                        Gizmos.DrawWireSphere((p1 + p2) * 0.5f, 0.3f);
                    }
                }
            }
        }
    }
}
