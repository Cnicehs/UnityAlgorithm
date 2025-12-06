using System.Collections.Generic;
using UnityEngine;
using Cysharp.Threading.Tasks;

public class SpatialGridIndex : ISpatialIndex
{
    private int _width;
    private int _height;
    private float _cellSize;
    private Vector2 _origin;
    
    // Flattened grid: _grid[y * _width + x] contains the head index of the linked list
    private int[] _gridHead;
    
    // Linked list for elements: _next[i] points to the next element index in the same cell
    private int[] _next;
    
    // Store positions for distance calculations
    private List<Vector2> _positions;

    public SpatialGridIndex(int width, int height, float cellSize, Vector2 origin)
    {
        _width = width;
        _height = height;
        _cellSize = cellSize;
        _origin = origin;
        
        _gridHead = new int[_width * _height];
        // _next will be resized in Build if needed
        _next = new int[0];
    }

    public UniTask BuildAsync(List<Vector2> positions)
    {
        // Grid build is fast and hard to parallelize (write conflicts).
        // We run it synchronously on the calling thread.
        
        _positions = positions;
        int count = positions.Count;
        
        // Resize _next array if needed
        if (_next.Length < count)
        {
            _next = new int[count];
        }

        // Clear grid heads
        System.Array.Fill(_gridHead, -1);

        // Populate grid
        for (int i = 0; i < count; i++)
        {
            Vector2 pos = positions[i];
            int x = Mathf.FloorToInt((pos.x - _origin.x) / _cellSize);
            int y = Mathf.FloorToInt((pos.y - _origin.y) / _cellSize);

            // Clamp to grid bounds
            if (x < 0) x = 0;
            if (x >= _width) x = _width - 1;
            if (y < 0) y = 0;
            if (y >= _height) y = _height - 1;

            int cellIndex = y * _width + x;
            
            // Insert at head of linked list for this cell
            _next[i] = _gridHead[cellIndex];
            _gridHead[cellIndex] = i;
        }
        
        return UniTask.CompletedTask;
    }

    // Cache for query candidates
    private List<Candidate> _candidateCache = new List<Candidate>();

    public void QueryKNearest(Vector2 position, int k, List<int> results)
    {
        results.Clear();
        
        _candidateCache.Clear();
        if (_candidateCache.Capacity < k * 4) _candidateCache.Capacity = k * 4;
        var candidates = _candidateCache;

        int startX = Mathf.FloorToInt((position.x - _origin.x) / _cellSize);
        int startY = Mathf.FloorToInt((position.y - _origin.y) / _cellSize);
        
        startX = Mathf.Clamp(startX, 0, _width - 1);
        startY = Mathf.Clamp(startY, 0, _height - 1);

        float worstDistSq = float.MaxValue;
        int searchRadius = 0;
        int maxSearchRadius = Mathf.Max(_width, _height);

        while (searchRadius <= maxSearchRadius)
        {
            int minX = Mathf.Max(0, startX - searchRadius);
            int maxX = Mathf.Min(_width - 1, startX + searchRadius);
            int minY = Mathf.Max(0, startY - searchRadius);
            int maxY = Mathf.Min(_height - 1, startY + searchRadius);

            // Ring Loop
            if (searchRadius == 0)
            {
                AddCandidatesFromCellUnsorted(startX, startY, position, k, candidates, ref worstDistSq);
            }
            else
            {
                for (int x = minX; x <= maxX; x++)
                {
                    if (minY == startY - searchRadius) AddCandidatesFromCellUnsorted(x, minY, position, k, candidates, ref worstDistSq);
                    if (maxY == startY + searchRadius && maxY != minY) AddCandidatesFromCellUnsorted(x, maxY, position, k, candidates, ref worstDistSq);
                }
                for (int y = minY + 1; y <= maxY - 1; y++)
                {
                    if (minX == startX - searchRadius) AddCandidatesFromCellUnsorted(minX, y, position, k, candidates, ref worstDistSq);
                    if (maxX == startX + searchRadius && maxX != minX) AddCandidatesFromCellUnsorted(maxX, y, position, k, candidates, ref worstDistSq);
                }
            }

            if (candidates.Count >= k)
            {
                // Distance to the inner edge of the next ring
                float nextRingDistSq = GetDistanceToRectSq(position, 
                    (startX - (searchRadius + 1)) * _cellSize + _origin.x,
                    (startY - (searchRadius + 1)) * _cellSize + _origin.y,
                    (startX + (searchRadius + 1) + 1) * _cellSize + _origin.x,
                    (startY + (searchRadius + 1) + 1) * _cellSize + _origin.y
                );

                if (worstDistSq <= nextRingDistSq)
                {
                    break;
                }
            }
            
            searchRadius++;
        }

        // Return unsorted results (up to k)
        int count = Mathf.Min(k, candidates.Count);
        for (int i = 0; i < count; i++)
        {
            results.Add(candidates[i].Index);
        }
    }
    
    private void AddCandidatesFromCellUnsorted(int x, int y, Vector2 position, int k, List<Candidate> candidates, ref float worstDistSq)
    {
        int cellIndex = y * _width + x;
        int current = _gridHead[cellIndex];
        while (current != -1)
        {
            float dSq = (_positions[current] - position).sqrMagnitude;
            
            if (candidates.Count < k)
            {
                candidates.Add(new Candidate { Index = current, DistSq = dSq });
                if (dSq > worstDistSq || worstDistSq == float.MaxValue) worstDistSq = dSq;
                if (candidates.Count == k)
                {
                    worstDistSq = 0;
                    for (int i = 0; i < candidates.Count; i++)
                        if (candidates[i].DistSq > worstDistSq) worstDistSq = candidates[i].DistSq;
                }
            }
            else if (dSq < worstDistSq)
            {
                int worstIdx = 0;
                for (int i = 1; i < candidates.Count; i++)
                    if (candidates[i].DistSq > candidates[worstIdx].DistSq) worstIdx = i;
                candidates[worstIdx] = new Candidate { Index = current, DistSq = dSq };
                worstDistSq = 0;
                for (int i = 0; i < candidates.Count; i++)
                    if (candidates[i].DistSq > worstDistSq) worstDistSq = candidates[i].DistSq;
            }
            
            current = _next[current];
        }
    }

    public void QueryKNearestSorted(Vector2 position, int k, float radius, List<int> results)
    {
        results.Clear();
        
        _candidateCache.Clear();
        if (_candidateCache.Capacity < k * 4)
        {
            _candidateCache.Capacity = k * 4;
        }
        var candidates = _candidateCache; 

        int startX = Mathf.FloorToInt((position.x - _origin.x) / _cellSize);
        int startY = Mathf.FloorToInt((position.y - _origin.y) / _cellSize);
        
        startX = Mathf.Clamp(startX, 0, _width - 1);
        startY = Mathf.Clamp(startY, 0, _height - 1);

        float radiusSq = radius * radius;
        int searchRadius = 0;
        int maxSearchRadius = Mathf.Max(_width, _height);
        
        // Limit search by radius
        if (radius != float.MaxValue)
        {
            int limitRadius = Mathf.CeilToInt(radius / _cellSize);
            maxSearchRadius = Mathf.Min(maxSearchRadius, limitRadius);
        }

        while (searchRadius <= maxSearchRadius)
        {
            int minX = Mathf.Max(0, startX - searchRadius);
            int maxX = Mathf.Min(_width - 1, startX + searchRadius);
            int minY = Mathf.Max(0, startY - searchRadius);
            int maxY = Mathf.Min(_height - 1, startY + searchRadius);

            // Optimized Ring Loop: Only iterate the edges
            if (searchRadius == 0)
            {
                AddCandidatesFromCell(startX, startY, position, radiusSq, candidates);
            }
            else
            {
                // Top and Bottom rows
                for (int x = minX; x <= maxX; x++)
                {
                    if (minY == startY - searchRadius) AddCandidatesFromCell(x, minY, position, radiusSq, candidates);
                    if (maxY == startY + searchRadius && maxY != minY) AddCandidatesFromCell(x, maxY, position, radiusSq, candidates);
                }
                // Left and Right columns (excluding corners already done)
                for (int y = minY + 1; y <= maxY - 1; y++)
                {
                    if (minX == startX - searchRadius) AddCandidatesFromCell(minX, y, position, radiusSq, candidates);
                    if (maxX == startX + searchRadius && maxX != minX) AddCandidatesFromCell(maxX, y, position, radiusSq, candidates);
                }
            }

            if (candidates.Count >= k)
            {
                // Sort candidates by distance
                candidates.Sort((a, b) => a.DistSq.CompareTo(b.DistSq));
                
                // Distance to K-th candidate
                float kthDistSq = candidates[k - 1].DistSq;
                
                // Distance to the inner edge of the next ring
                float nextRingDistSq = GetDistanceToRectSq(position, 
                    (startX - (searchRadius + 1)) * _cellSize + _origin.x,
                    (startY - (searchRadius + 1)) * _cellSize + _origin.y,
                    (startX + (searchRadius + 1) + 1) * _cellSize + _origin.x,
                    (startY + (searchRadius + 1) + 1) * _cellSize + _origin.y
                );

                if (kthDistSq <= nextRingDistSq)
                {
                    break;
                }
            }
            
            searchRadius++;
        }

        // Final sort
        candidates.Sort((a, b) => a.DistSq.CompareTo(b.DistSq));

        int count = Mathf.Min(k, candidates.Count);
        for (int i = 0; i < count; i++)
        {
            results.Add(candidates[i].Index);
        }
    }

    private void AddCandidatesFromCell(int x, int y, Vector2 position, float radiusSq, List<Candidate> candidates)
    {
        int cellIndex = y * _width + x;
        int current = _gridHead[cellIndex];
        while (current != -1)
        {
            float dSq = (_positions[current] - position).sqrMagnitude;
            if (dSq <= radiusSq)
            {
                candidates.Add(new Candidate { Index = current, DistSq = dSq });
            }
            current = _next[current];
        }
    }

    private struct Candidate
    {
        public int Index;
        public float DistSq;
    }

    public void QueryRadius(Vector2 position, float radius, List<int> results)
    {
        results.Clear();
        float radiusSq = radius * radius;

        int minX = Mathf.FloorToInt((position.x - radius - _origin.x) / _cellSize);
        int maxX = Mathf.FloorToInt((position.x + radius - _origin.x) / _cellSize);
        int minY = Mathf.FloorToInt((position.y - radius - _origin.y) / _cellSize);
        int maxY = Mathf.FloorToInt((position.y + radius - _origin.y) / _cellSize);

        minX = Mathf.Clamp(minX, 0, _width - 1);
        maxX = Mathf.Clamp(maxX, 0, _width - 1);
        minY = Mathf.Clamp(minY, 0, _height - 1);
        maxY = Mathf.Clamp(maxY, 0, _height - 1);

        for (int y = minY; y <= maxY; y++)
        {
            for (int x = minX; x <= maxX; x++)
            {
                int cellIndex = y * _width + x;
                int current = _gridHead[cellIndex];
                while (current != -1)
                {
                    if ((_positions[current] - position).sqrMagnitude <= radiusSq)
                    {
                        results.Add(current);
                    }
                    current = _next[current];
                }
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
