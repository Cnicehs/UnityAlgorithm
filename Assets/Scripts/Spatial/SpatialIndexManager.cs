using System.Collections.Generic;
using UnityEngine;
using Cysharp.Threading.Tasks;
using System.Diagnostics;
using Debug = UnityEngine.Debug;
using System;
using Random = UnityEngine.Random;

public enum SpatialStructureType
{
    Grid,
    KDTree,
    BVH,
    QuadTree,
    SIMDHashGrid,
    SIMDKDTree,
    SIMDBVH,
    SIMDQuadTree
}

public class SpatialIndexManager
{
    private static SpatialIndexManager _instance;
    public static SpatialIndexManager Instance => _instance ??= new SpatialIndexManager();

    public SpatialStructureType StructureType = SpatialStructureType.Grid;
    public bool UseMultiThreading = true;
    
    public float BuildTimeMs { get; private set; }
    public bool IsBuilding { get; private set; }

    private int _unitCount;
private float _worldSize;
    
    // Double buffering
    private List<Vector2> _mainThreadPositions = new List<Vector2>();
    private List<Vector2> _asyncPositionsA = new List<Vector2>();
    private List<Vector2> _asyncPositionsB = new List<Vector2>();

    private ISpatialIndex _spatialIndex;
    private ISpatialIndex _indexA;
    private ISpatialIndex _indexB;
    private bool _usingIndexA = true;

    private SpatialStructureType _lastStructureType;
    private int _lastUnitCount;
    
    private bool _isRunning;

    private SpatialIndexManager()
    {
    // Private constructor
    }

    public void Initialize(int unitCount, float worldSize)
    {
        _unitCount = unitCount;
        _worldSize = worldSize;
        _isRunning = true;
        BuildLoop().Forget();
    }

    public void Shutdown()
    {
        _isRunning = false;
        _spatialIndex = null;
        (_indexA as IDisposable)?.Dispose();
        (_indexB as IDisposable)?.Dispose();
        _indexA = null;
        _indexB = null;
    }

    public void UpdatePositions(List<Vector2> positions)
    {
        _mainThreadPositions.Clear();
        _mainThreadPositions.AddRange(positions);
    }

    public void QueryKNearest(Vector2 position, int k, List<int> results)
    {
        if (_spatialIndex == null) return;
        _spatialIndex.QueryKNearest(position, k, results);
    }

    public void GetNeighborsInRadius(Vector2 position, float radius, List<int> results)
    {
        if (_spatialIndex == null) return;
        _spatialIndex.QueryRadius(position, radius, results);
    }

    private void EnsureIndicesCreated()
    {
        if (_indexA == null || _indexB == null || 
            _lastStructureType != StructureType || _lastUnitCount != _unitCount)
        {
            _lastStructureType = StructureType;
            _lastUnitCount = _unitCount;
            
            _spatialIndex = null;
            
            (_indexA as IDisposable)?.Dispose();
            (_indexB as IDisposable)?.Dispose();

            _indexA = CreateNewIndex();
_indexB = CreateNewIndex();
            
            System.GC.Collect();
        }
    }

ISpatialIndex CreateNewIndex()
    {
        switch (StructureType)
        {
            case SpatialStructureType.Grid:
                return new SpatialGridIndex(40, 40, _worldSize / 40, new Vector2(-_worldSize/2, -_worldSize/2));
            case SpatialStructureType.KDTree:
                return new KDTreeIndex(_unitCount);
            case SpatialStructureType.BVH:
                return new BVHIndex(_unitCount);
            case SpatialStructureType.QuadTree:
                return new QuadTreeIndex(_unitCount, new Rect(-_worldSize/2, -_worldSize/2, _worldSize, _worldSize));
            case SpatialStructureType.SIMDHashGrid:
                return new SIMDHashGridIndex(40, 40, _worldSize / 40, new Vector2(-_worldSize/2, -_worldSize/2), _unitCount);
            case SpatialStructureType.SIMDKDTree:
                return new SIMDKDTreeIndex(_unitCount);
            case SpatialStructureType.SIMDBVH:
                return new SIMDBVHIndex(_unitCount);
            case SpatialStructureType.SIMDQuadTree:
                return new SIMDQuadTreeIndex(_unitCount, new Rect(-_worldSize/2, -_worldSize/2, _worldSize, _worldSize));
            default:
                return new KDTreeIndex(_unitCount);
        }
    }
    private async UniTaskVoid BuildLoop()
    {
        while (_isRunning)
        {
            await UniTask.Yield(PlayerLoopTiming.Update);
            
            if (_mainThreadPositions.Count == 0) continue;

            EnsureIndicesCreated();

ISpatialIndex indexToBuild = _usingIndexA ? _indexB : _indexA;
            List<Vector2> currentAsyncPositions = _usingIndexA ? _asyncPositionsB : _asyncPositionsA;

            if (currentAsyncPositions.Capacity < _mainThreadPositions.Count)
            {
                currentAsyncPositions.Capacity = _mainThreadPositions.Count;
            }
            currentAsyncPositions.Clear();
            currentAsyncPositions.AddRange(_mainThreadPositions);

            IsBuilding = true;
            float buildTime = 0;
            
            if (UseMultiThreading)
            {
                await UniTask.RunOnThreadPool(async () =>
                {
        Stopwatch sw = Stopwatch.StartNew();
                    await indexToBuild.BuildAsync(currentAsyncPositions);
                    sw.Stop();
                    buildTime = (float)sw.Elapsed.TotalMilliseconds;
                });
            }
            else
            {
                Stopwatch sw = Stopwatch.StartNew();
                await indexToBuild.BuildAsync(currentAsyncPositions);
                sw.Stop();
                buildTime = (float)sw.Elapsed.TotalMilliseconds;
            }
            
            BuildTimeMs = buildTime;
            IsBuilding = false;

            _usingIndexA = !_usingIndexA;
_spatialIndex = indexToBuild;
        }
    }
}
