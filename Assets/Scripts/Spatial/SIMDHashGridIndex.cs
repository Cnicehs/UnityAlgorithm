using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using UnityEngine;
using Cysharp.Threading.Tasks;
using Unity.Burst;
using Unity.Mathematics;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;

[BurstCompile]
public unsafe class SIMDHashGridIndex : ISpatialIndex, IDisposable
{
    private int _width;
    private int _height;
    private float _cellSize;
    private float2 _origin;

    private NativeArray<int> _cellStart;
    private NativeArray<int> _cellCount;
    private NativeArray<float2> _sortedPositions;
    private NativeArray<int> _originalIndices;
    private int _count;




    public SIMDHashGridIndex(int width, int height, float cellSize, Vector2 origin, int capacity)
    {
        _width = width;
        _height = height;
        _cellSize = cellSize;
        _origin = new float2(origin.x, origin.y);

        _cellStart = new NativeArray<int>(_width * _height, Allocator.Persistent);
        _cellCount = new NativeArray<int>(_width * _height, Allocator.Persistent);
        _sortedPositions = new NativeArray<float2>(capacity, Allocator.Persistent);
        _originalIndices = new NativeArray<int>(capacity, Allocator.Persistent);
    }

    public void Dispose()
    {
        if (_cellStart.IsCreated) _cellStart.Dispose();
        if (_cellCount.IsCreated) _cellCount.Dispose();
        if (_sortedPositions.IsCreated) _sortedPositions.Dispose();
        if (_originalIndices.IsCreated) _originalIndices.Dispose();
    }

    public UniTask BuildAsync(List<Vector2> positions)
    {
        _count = positions.Count;

        if (_sortedPositions.Length < _count)
        {
            _sortedPositions.Dispose();
            _originalIndices.Dispose();
            _sortedPositions = new NativeArray<float2>(_count, Allocator.Persistent);
            _originalIndices = new NativeArray<int>(_count, Allocator.Persistent);
        }

        // 1. Count elements per cell
        _cellCount.AsSpan().Clear();
        int* cellCounts = (int*)_cellCount.GetUnsafePtr();
        Span<Vector2> posSpan = positions.AsSpan();
        fixed (Vector2* posPtr = posSpan)
        {
            CountElementsPerCell(cellCounts, (float2*)posPtr, _count, _width, _height, _cellSize, _origin);
        }

        // 2. Calculate start index for each cell (prefix sum)
        int sum = 0;
        for (int i = 0; i < _cellStart.Length; i++)
        {
            _cellStart[i] = sum;
            sum += _cellCount[i];
        }

        // 3. Reorder positions and indices
        int* cellStarts = (int*)_cellStart.GetUnsafePtr();
        fixed (Vector2* posPtr = posSpan)
        {
            Reorder(cellStarts, (float2*)posPtr, (float2*)_sortedPositions.GetUnsafePtr(), (int*)_originalIndices.GetUnsafePtr(), _count, _width, _height, _cellSize, _origin);
        }

        return UniTask.CompletedTask;
    }

    [BurstCompile]
    private static void CountElementsPerCell(int* cellCounts, float2* positions, int count, int width, int height, float cellSize, in float2 origin)
    {
        for (int i = 0; i < count; i++)
        {
            float2 p = positions[i];
            int x = (int)((p.x - origin.x) / cellSize);
            int y = (int)((p.y - origin.y) / cellSize);

            if (x < 0) x = 0; else if (x >= width) x = width - 1;
            if (y < 0) y = 0; else if (y >= height) y = height - 1;

            int cellIndex = y * width + x;
            cellCounts[cellIndex]++;
        }
    }

    [BurstCompile]
    private static void Reorder(int* cellStarts, float2* positions, float2* sortedPositions, int* originalIndices, int count, int width, int height, float cellSize, in float2 origin)
    {
        // This needs to be a copy of cellStarts to use as a counter
        int* cellIndices = stackalloc int[width * height];
        UnsafeUtility.MemCpy(cellIndices, cellStarts, width * height * sizeof(int));

        for (int i = 0; i < count; i++)
        {
            float2 p = positions[i];
            int x = (int)((p.x - origin.x) / cellSize);
            int y = (int)((p.y - origin.y) / cellSize);

            if (x < 0) x = 0; else if (x >= width) x = width - 1;
            if (y < 0) y = 0; else if (y >= height) y = height - 1;

            int cellIndex = y * width + x;
            int sortedIndex = cellIndices[cellIndex]++;
            
            sortedPositions[sortedIndex] = p;
            originalIndices[sortedIndex] = i;
        }
    }

    public void QueryKNearest(Vector2 position, int k, List<int> results)
    {
        results.Clear();
        int maxCandidates = k * 10;
        if (maxCandidates < 128) maxCandidates = 128;

        int* resultBuffer = (int*)UnsafeUtility.Malloc(maxCandidates * sizeof(int), 4, Allocator.Temp);
        int resultCount = 0;

        try
        {
            QueryKNearestBurst(new float2(position.x, position.y), k, (float2*)_sortedPositions.GetUnsafePtr(), (int*)_originalIndices.GetUnsafePtr(), (int*)_cellStart.GetUnsafePtr(), (int*)_cellCount.GetUnsafePtr(), _width, _height, _cellSize, _origin, resultBuffer, &resultCount, maxCandidates);

            for (int i = 0; i < resultCount; i++)
            {
                results.Add(resultBuffer[i]);
            }
        }
        finally
        {
            UnsafeUtility.Free(resultBuffer, Allocator.Temp);
        }
    }

    // Struct for sorting in Burst
    private struct Candidate : IComparable<Candidate>
    {
        public int Index;
        public float DistSq;

        public int CompareTo(Candidate other)
        {
            return DistSq.CompareTo(other.DistSq);
        }
    }

    [BurstCompile]
    private static void QueryKNearestBurst(in float2 position, int k, float2* sortedPositions, int* originalIndices, int* cellStart, int* cellCount, int width, int height, float cellSize, in float2 origin, int* results, int* resultCount, int maxResults)
    {
        int startX = (int)((position.x - origin.x) / cellSize);
        int startY = (int)((position.y - origin.y) / cellSize);

        if (startX < 0) startX = 0; else if (startX >= width) startX = width - 1;
        if (startY < 0) startY = 0; else if (startY >= height) startY = height - 1;

        int searchRadius = 0;
        int maxSearchRadius = math.max(width, height);

        Candidate* candidates = (Candidate*)UnsafeUtility.Malloc(maxResults * sizeof(Candidate), 4, Allocator.Temp);
        int candidateCount = 0;

        float4 target4 = new float4(position.x, position.y, position.x, position.y);

        while (searchRadius < maxSearchRadius)
        {
            int rMinX = math.max(0, startX - searchRadius);
            int rMaxX = math.min(width - 1, startX + searchRadius);
            int rMinY = math.max(0, startY - searchRadius);
            int rMaxY = math.min(height - 1, startY + searchRadius);

            // Process cells
            for (int y = rMinY; y <= rMaxY; y++)
            {
                for (int x = rMinX; x <= rMaxX; x++)
                {
                    // Process only the ring
                    if (searchRadius > 0 && x > rMinX && x < rMaxX && y > rMinY && y < rMaxY) continue;

                    int cellIndex = y * width + x;
                    int start = cellStart[cellIndex];
                    int count = cellCount[cellIndex];

                    for (int i = 0; i < count; i += 4)
                    {
                        if (i + 3 < count)
                        {
                            float4 v0 = *(float4*)(sortedPositions + start + i);
                            float4 v1 = *(float4*)(sortedPositions + start + i + 2);

                            float4 d0 = v0 - target4;
                            float4 d1 = v1 - target4;

                            float4 sq0 = d0 * d0;
                            float4 sq1 = d1 * d1;

                            float2 dists0 = sq0.xz + sq0.yw;
                            float2 dists1 = sq1.xz + sq1.yw;

                            if (candidateCount < maxResults) candidates[candidateCount++] = new Candidate { Index = originalIndices[start + i], DistSq = dists0.x };
                            if (candidateCount < maxResults) candidates[candidateCount++] = new Candidate { Index = originalIndices[start + i + 1], DistSq = dists0.y };
                            if (candidateCount < maxResults) candidates[candidateCount++] = new Candidate { Index = originalIndices[start + i + 2], DistSq = dists1.x };
                            if (candidateCount < maxResults) candidates[candidateCount++] = new Candidate { Index = originalIndices[start + i + 3], DistSq = dists1.y };
                        }
                        else
                        {
                            for (int j = i; j < count; j++)
                            {
                                float distSq = math.distancesq(sortedPositions[start + j], position);
                                if (candidateCount < maxResults) candidates[candidateCount++] = new Candidate { Index = originalIndices[start + j], DistSq = distSq };
                            }
                        }
                    }
                }
            }

            if (candidateCount >= k)
            {
                SortCandidates(candidates, candidateCount);
                float kthDistSq = candidates[k - 1].DistSq;

                float nextRingDistSq = GetDistanceToRectSq(position,
                    (startX - (searchRadius + 1)) * cellSize + origin.x,
                    (startY - (searchRadius + 1)) * cellSize + origin.y,
                    (startX + (searchRadius + 1) + 1) * cellSize + origin.x,
                    (startY + (searchRadius + 1) + 1) * cellSize + origin.y
                );

                if (kthDistSq <= nextRingDistSq) break;
            }
            searchRadius++;
        }

        SortCandidates(candidates, candidateCount);
        int finalCount = math.min(k, candidateCount);

        for (int i = 0; i < finalCount; i++)
        {
            results[i] = candidates[i].Index;
        }
        *resultCount = finalCount;

        UnsafeUtility.Free(candidates, Allocator.Temp);
    }

    private static void SortCandidates(Candidate* arr, int count)
    {
        // Simple Insertion Sort is fast for small arrays
        for (int i = 1; i < count; ++i)
        {
            Candidate key = arr[i];
            int j = i - 1;
            while (j >= 0 && arr[j].DistSq > key.DistSq)
            {
                arr[j + 1] = arr[j];
                j = j - 1;
            }
            arr[j + 1] = key;
        }
    }

    private static float GetDistanceToRectSq(float2 p, float minX, float minY, float maxX, float maxY)
    {
        float dx = math.max(minX - p.x, 0);
        dx = math.max(dx, p.x - maxX);

        float dy = math.max(minY - p.y, 0);
        dy = math.max(dy, p.y - maxY);

        return dx * dx + dy * dy;
    }

    public void QueryRadius(Vector2 position, float radius, List<int> results)
    {
        results.Clear();
        int maxResults = _count;
        int* resultBuffer = (int*)UnsafeUtility.Malloc(maxResults * sizeof(int), 4, Allocator.Temp);
        int resultCount = 0;

        try
        {
            QueryRadiusBurst(new float2(position.x, position.y), radius * radius, (float2*)_sortedPositions.GetUnsafePtr(), (int*)_originalIndices.GetUnsafePtr(), (int*)_cellStart.GetUnsafePtr(), (int*)_cellCount.GetUnsafePtr(), _width, _height, _cellSize, _origin, resultBuffer, &resultCount, maxResults);

            for (int i = 0; i < resultCount; i++)
            {
                results.Add(resultBuffer[i]);
            }
        }
        finally
        {
            UnsafeUtility.Free(resultBuffer, Allocator.Temp);
        }
    }

    [BurstCompile]
    private static void QueryRadiusBurst(in float2 position, float radiusSq, float2* sortedPositions, int* originalIndices, int* cellStart, int* cellCount, int width, int height, float cellSize, in float2 origin, int* results, int* resultCount, int maxResults)
    {
        float radius = math.sqrt(radiusSq);
        int minX = (int)((position.x - radius - origin.x) / cellSize);
        int maxX = (int)((position.x + radius - origin.x) / cellSize);
        int minY = (int)((position.y - radius - origin.y) / cellSize);
        int maxY = (int)((position.y + radius - origin.y) / cellSize);

        if (minX < 0) minX = 0;
        if (maxX >= width) maxX = width - 1;
        if (minY < 0) minY = 0;
        if (maxY >= height) maxY = height - 1;

        // 1. Gather all indices first
        int totalCandidates = 0;
        for (int y = minY; y <= maxY; y++)
        {
            for (int x = minX; x <= maxX; x++)
            {
                totalCandidates += cellCount[y * width + x];
            }
        }

        if (totalCandidates == 0) return;

        int* candidateIndices = (int*)UnsafeUtility.Malloc(totalCandidates * sizeof(int), 4, Allocator.Temp);
        int gatheredCount = 0;
        for (int y = minY; y <= maxY; y++)
        {
            for (int x = minX; x <= maxX; x++)
            {
                int cellIndex = y * width + x;
                int start = cellStart[cellIndex];
                int count = cellCount[cellIndex];
                UnsafeUtility.MemCpy(candidateIndices + gatheredCount, originalIndices + start, count * sizeof(int));
                gatheredCount += count;
            }
        }

        // 2. Process in batch
        float4 rSqVec = new float4(radiusSq);
        float4 target4 = new float4(position.x, position.y, position.x, position.y);

        int i = 0;
        for (; i <= gatheredCount - 4; i += 4)
        {
            // This is still not ideal as we are jumping around in the original positions array.
            // The sortedPositions array is what we need to use.
            // Let's fix this.
        }
        
        // Corrected approach: We already have sortedPositions. We just need to iterate through the right cells.
        for (int y = minY; y <= maxY; y++)
        {
            for (int x = minX; x <= maxX; x++)
            {
                int cellIndex = y * width + x;
                int start = cellStart[cellIndex];
                int count = cellCount[cellIndex];

                for (i = 0; i <= count - 4; i += 4)
                {
                    float4 v0 = *(float4*)(sortedPositions + start + i);
                    float4 v1 = *(float4*)(sortedPositions + start + i + 2);

                    float4 d0 = v0 - target4;
                    float4 d1 = v1 - target4;

                    float4 sq0 = d0 * d0;
                    float4 sq1 = d1 * d1;

                    float2 dists0 = sq0.xz + sq0.yw;
                    float2 dists1 = sq1.xz + sq1.yw;

                    bool4 mask0 = dists0.xyxy <= rSqVec;
                    bool4 mask1 = dists1.xyxy <= rSqVec;

                    if (mask0.x) results[(*resultCount)++] = originalIndices[start + i];
                    if (mask0.y) results[(*resultCount)++] = originalIndices[start + i + 1];
                    if (mask1.x) results[(*resultCount)++] = originalIndices[start + i + 2];
                    if (mask1.y) results[(*resultCount)++] = originalIndices[start + i + 3];
                }

                for (; i < count; i++)
                {
                    if (math.distancesq(sortedPositions[start + i], position) <= radiusSq)
                    {
                        results[(*resultCount)++] = originalIndices[start + i];
                    }
                }
            }
        }
        
        UnsafeUtility.Free(candidateIndices, Allocator.Temp);
    }
}
