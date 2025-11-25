using System;
using System.Collections.Generic;
using UnityEngine;
using Cysharp.Threading.Tasks;
using System.Numerics;
using System.Runtime.InteropServices; // For CollectionsMarshal

using Vector2 = UnityEngine.Vector2;

public class SIMDHashGridIndex : ISpatialIndex
{
    private int _width;
    private int _height;
    private float _cellSize;
    private Vector2 _origin;
    
    // Grid
    private int[] _gridHead;
    private int[] _next;
    
    private List<Vector2> _positions; // Reference to the list
    private int _count;

    // Cache
    private List<(int index, float distSq)> _candidateCache = new List<(int index, float distSq)>();

    public SIMDHashGridIndex(int width, int height, float cellSize, Vector2 origin, int capacity)
    {
        _width = width;
        _height = height;
        _cellSize = cellSize;
        _origin = origin;
        
        _gridHead = new int[_width * _height];
        _next = new int[capacity];
    }

    public UniTask BuildAsync(List<Vector2> positions)
    {
        _positions = positions;
        _count = positions.Count;
        
        // Resize _next if needed
        if (_next.Length < _count)
        {
            _next = new int[_count];
        }
        
        // 1. Clear Grid
        Array.Fill(_gridHead, -1);

        // 2. Build Grid
        // We access List directly. 
        // Note: List indexer overhead is small compared to logic.
        // 2. Build Grid
        // We use ListUtils.AsSpan to get direct access to the underlying array for performance
        var span = _positions.AsSpan();
        
        for (int i = 0; i < _count; i++)
        {
            Vector2 p = span[i];
            int x = (int)((p.x - _origin.x) / _cellSize);
            int y = (int)((p.y - _origin.y) / _cellSize);
            
            if (x < 0) x = 0; else if (x >= _width) x = _width - 1;
            if (y < 0) y = 0; else if (y >= _height) y = _height - 1;
            
            int cellIndex = y * _width + x;
            
            _next[i] = _gridHead[cellIndex];
            _gridHead[cellIndex] = i;
        }

        return UniTask.CompletedTask;
    }

    public void QueryKNearest(Vector2 position, int k, List<int> results)
    {
        results.Clear();
        _candidateCache.Clear();
        if (_candidateCache.Capacity < k * 4) _candidateCache.Capacity = k * 4;

        int startX = Mathf.FloorToInt((position.x - _origin.x) / _cellSize);
        int startY = Mathf.FloorToInt((position.y - _origin.y) / _cellSize);
        
        startX = Mathf.Clamp(startX, 0, _width - 1);
        startY = Mathf.Clamp(startY, 0, _height - 1);

        int searchRadius = 0;
        int maxSearchRadius = Mathf.Max(_width, _height);
        
        // SIMD Setup
        Vector<float> targetX = new Vector<float>(position.x);
        Vector<float> targetY = new Vector<float>(position.y);
        int vectorSize = Vector<float>.Count;

        // Buffers for gathering (SoA on-the-fly)
        // We gather up to vectorSize elements
        Span<float> batchX = stackalloc float[vectorSize];
        Span<float> batchY = stackalloc float[vectorSize];
        Span<int> batchIndices = stackalloc int[vectorSize];

        while (searchRadius < maxSearchRadius)
        {
            int minX = Mathf.Max(0, startX - searchRadius);
            int maxX = Mathf.Min(_width - 1, startX + searchRadius);
            int minY = Mathf.Max(0, startY - searchRadius);
            int maxY = Mathf.Min(_height - 1, startY + searchRadius);

            // Ring Loop
            if (searchRadius == 0)
            {
                ProcessCellSIMD(startX, startY, targetX, targetY, vectorSize, _candidateCache, batchX, batchY, batchIndices);
            }
            else
            {
                for (int x = minX; x <= maxX; x++)
                {
                    if (minY == startY - searchRadius) ProcessCellSIMD(x, minY, targetX, targetY, vectorSize, _candidateCache, batchX, batchY, batchIndices);
                    if (maxY == startY + searchRadius && maxY != minY) ProcessCellSIMD(x, maxY, targetX, targetY, vectorSize, _candidateCache, batchX, batchY, batchIndices);
                }
                for (int y = minY + 1; y <= maxY - 1; y++)
                {
                    if (minX == startX - searchRadius) ProcessCellSIMD(minX, y, targetX, targetY, vectorSize, _candidateCache, batchX, batchY, batchIndices);
                    if (maxX == startX + searchRadius && maxX != minX) ProcessCellSIMD(maxX, y, targetX, targetY, vectorSize, _candidateCache, batchX, batchY, batchIndices);
                }
            }

            if (_candidateCache.Count >= k)
            {
                _candidateCache.Sort((a, b) => a.distSq.CompareTo(b.distSq));
                float kthDistSq = _candidateCache[k - 1].distSq;
                
                float distToNextRingSq = GetDistanceToRectSq(position, 
                    (startX - (searchRadius + 1)) * _cellSize + _origin.x,
                    (startY - (searchRadius + 1)) * _cellSize + _origin.y,
                    (startX + (searchRadius + 1) + 1) * _cellSize + _origin.x,
                    (startY + (searchRadius + 1) + 1) * _cellSize + _origin.y
                );

                if (kthDistSq <= distToNextRingSq) break;
            }
            searchRadius++;
        }

        _candidateCache.Sort((a, b) => a.distSq.CompareTo(b.distSq));
        int count = Mathf.Min(k, _candidateCache.Count);
        for (int i = 0; i < count; i++) results.Add(_candidateCache[i].index);
    }

    private void ProcessCellSIMD(int x, int y, Vector<float> targetX, Vector<float> targetY, int vectorSize, List<(int, float)> candidates, Span<float> batchX, Span<float> batchY, Span<int> batchIndices)
    {
        int cellIndex = y * _width + x;
        int current = _gridHead[cellIndex];
        
        int batchCount = 0;
        
        var posSpan = _positions.AsSpan();

        while (current != -1)
        {
            // Gather
            Vector2 p = posSpan[current];
            batchX[batchCount] = p.x;
            batchY[batchCount] = p.y;
            batchIndices[batchCount] = current;
            batchCount++;
            
            if (batchCount == vectorSize)
            {
                ProcessBatch(batchX, batchY, batchIndices, batchCount, targetX, targetY, candidates);
                batchCount = 0;
            }
            
            current = _next[current];
        }
        
        if (batchCount > 0)
        {
            ProcessBatch(batchX, batchY, batchIndices, batchCount, targetX, targetY, candidates);
        }
    }

    private void ProcessBatch(Span<float> px, Span<float> py, Span<int> indices, int count, Vector<float> targetX, Vector<float> targetY, List<(int, float)> candidates)
    {
        // Load into vectors
        Vector<float> vx = new Vector<float>(px);
        Vector<float> vy = new Vector<float>(py);
        
        // Calc dist sq
        Vector<float> dx = vx - targetX;
        Vector<float> dy = vy - targetY;
        Vector<float> dSq = dx * dx + dy * dy;
        
        // Extract results
        for (int i = 0; i < count; i++)
        {
            candidates.Add((indices[i], dSq[i]));
        }
    }

    public void QueryRadius(Vector2 position, float radius, List<int> results)
    {
        results.Clear();
        float rSq = radius * radius;
        Vector<float> rSqVec = new Vector<float>(rSq);
        Vector<float> targetX = new Vector<float>(position.x);
        Vector<float> targetY = new Vector<float>(position.y);
        int vectorSize = Vector<float>.Count;

        int minX = Mathf.FloorToInt((position.x - radius - _origin.x) / _cellSize);
        int maxX = Mathf.FloorToInt((position.x + radius - _origin.x) / _cellSize);
        int minY = Mathf.FloorToInt((position.y - radius - _origin.y) / _cellSize);
        int maxY = Mathf.FloorToInt((position.y + radius - _origin.y) / _cellSize);

        minX = Mathf.Clamp(minX, 0, _width - 1);
        maxX = Mathf.Clamp(maxX, 0, _width - 1);
        minY = Mathf.Clamp(minY, 0, _height - 1);
        maxY = Mathf.Clamp(maxY, 0, _height - 1);

        Span<float> batchX = stackalloc float[vectorSize];
        Span<float> batchY = stackalloc float[vectorSize];
        Span<int> batchIndices = stackalloc int[vectorSize];

        for (int y = minY; y <= maxY; y++)
        {
            for (int x = minX; x <= maxX; x++)
            {
                int cellIndex = y * _width + x;
                int current = _gridHead[cellIndex];
                
                int batchCount = 0;
                var posSpan = _positions.AsSpan();

                while (current != -1)
                {
                    Vector2 p = posSpan[current];
                    batchX[batchCount] = p.x;
                    batchY[batchCount] = p.y;
                    batchIndices[batchCount] = current;
                    batchCount++;

                    if (batchCount == vectorSize)
                    {
                        ProcessBatchRadius(batchX, batchY, batchIndices, batchCount, targetX, targetY, rSqVec, results);
                        batchCount = 0;
                    }
                    current = _next[current];
                }
                if (batchCount > 0)
                {
                    ProcessBatchRadius(batchX, batchY, batchIndices, batchCount, targetX, targetY, rSqVec, results);
                }
            }
        }
    }

    private void ProcessBatchRadius(Span<float> px, Span<float> py, Span<int> indices, int count, Vector<float> targetX, Vector<float> targetY, Vector<float> rSqVec, List<int> results)
    {
        Vector<float> vx = new Vector<float>(px);
        Vector<float> vy = new Vector<float>(py);
        
        Vector<float> dx = vx - targetX;
        Vector<float> dy = vy - targetY;
        Vector<float> dSq = dx * dx + dy * dy;
        
        // Compare
        Vector<int> mask = Vector.LessThanOrEqual(dSq, rSqVec);
        
        for (int i = 0; i < count; i++)
        {
            if (mask[i] != 0)
            {
                results.Add(indices[i]);
            }
        }
    }

    private float GetDistanceToRectSq(Vector2 p, float minX, float minY, float maxX, float maxY)
    {
        float dx = Mathf.Max(minX - p.x, 0, p.x - maxX);
        float dy = Mathf.Max(minY - p.y, 0, p.y - maxY);
        return dx * dx + dy * dy;
    }
}
