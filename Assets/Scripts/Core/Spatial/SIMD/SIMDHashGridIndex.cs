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

    [BurstCompile]
    private static void GetShuffleMask(int mask, int* result)
    {
        switch (mask)
        {
            case 0: *(int4*)result = new int4(-1, -1, -1, -1); break;
            case 1: *(int4*)result = new int4(0, -1, -1, -1); break;
            case 2: *(int4*)result = new int4(1, -1, -1, -1); break;
            case 3: *(int4*)result = new int4(0, 1, -1, -1); break;
            case 4: *(int4*)result = new int4(2, -1, -1, -1); break;
            case 5: *(int4*)result = new int4(0, 2, -1, -1); break;
            case 6: *(int4*)result = new int4(1, 2, -1, -1); break;
            case 7: *(int4*)result = new int4(0, 1, 2, -1); break;
            case 8: *(int4*)result = new int4(3, -1, -1, -1); break;
            case 9: *(int4*)result = new int4(0, 3, -1, -1); break;
            case 10: *(int4*)result = new int4(1, 3, -1, -1); break;
            case 11: *(int4*)result = new int4(0, 1, 3, -1); break;
            case 12: *(int4*)result = new int4(2, 3, -1, -1); break;
            case 13: *(int4*)result = new int4(0, 2, 3, -1); break;
            case 14: *(int4*)result = new int4(1, 2, 3, -1); break;
            case 15: *(int4*)result = new int4(0, 1, 2, 3); break;
            default: *(int4*)result = new int4(-1, -1, -1, -1); break;
        }
    }

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
    private static void CountElementsPerCell(int* cellCounts, float2* positions, int count, int width, int height, float cellSize,in float2 origin)
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
    private static void Reorder(int* cellStarts, float2* positions, float2* sortedPositions, int* originalIndices, int count, int width, int height, float cellSize,in float2 origin)
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
            fixed (float2* originPtr = &_origin)
            {
                QueryKNearestBurst(new float2(position.x, position.y), k, float.MaxValue, (float2*)_sortedPositions.GetUnsafePtr(), (int*)_originalIndices.GetUnsafePtr(), (int*)_cellStart.GetUnsafePtr(), (int*)_cellCount.GetUnsafePtr(), _width, _height, _cellSize, originPtr, resultBuffer, &resultCount, maxCandidates, false);
            }

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

    public void QueryKNearestSorted(Vector2 position, int k, float radius, List<int> results)
    {
        results.Clear();
        int maxCandidates = k * 10;
        if (maxCandidates < 128) maxCandidates = 128;

        int* resultBuffer = (int*)UnsafeUtility.Malloc(maxCandidates * sizeof(int), 4, Allocator.Temp);
        int resultCount = 0;

        try
        {
            fixed (float2* originPtr = &_origin)
            {
                QueryKNearestBurst(new float2(position.x, position.y), k, radius * radius, (float2*)_sortedPositions.GetUnsafePtr(), (int*)_originalIndices.GetUnsafePtr(), (int*)_cellStart.GetUnsafePtr(), (int*)_cellCount.GetUnsafePtr(), _width, _height, _cellSize, originPtr, resultBuffer, &resultCount, maxCandidates, true);
            }

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
    private static void QueryKNearestBurst(in float2 position, int k, float radiusSq, float2* sortedPositions, int* originalIndices, int* cellStart, int* cellCount, int width, int height, float cellSize, [NoAlias] float2* originPtr, int* results, int* resultCount, int maxResults, bool sortResults)
    {
        float2 origin = *originPtr;
        int startX = (int)((position.x - origin.x) / cellSize);
        int startY = (int)((position.y - origin.y) / cellSize);

        if (startX < 0) startX = 0; else if (startX >= width) startX = width - 1;
        if (startY < 0) startY = 0; else if (startY >= height) startY = height - 1;

        int searchRadius = 0;
        int maxSearchRadius = math.max(width, height);
        
        if (radiusSq != float.MaxValue)
        {
            int limit = (int)(math.sqrt(radiusSq) / cellSize) + 1;
            maxSearchRadius = math.min(maxSearchRadius, limit);
        }

        Candidate* candidates = (Candidate*)UnsafeUtility.Malloc(maxResults * sizeof(Candidate), 4, Allocator.Temp);
        int candidateCount = 0;

        float4 target4 = new float4(position.x, position.y, position.x, position.y);

        while (searchRadius <= maxSearchRadius)
        {
            int rMinX = math.max(0, startX - searchRadius);
            int rMaxX = math.min(width - 1, startX + searchRadius);
            int rMinY = math.max(0, startY - searchRadius);
            int rMaxY = math.min(height - 1, startY + searchRadius);

            for (int y = rMinY; y <= rMaxY; y++)
            {
                for (int x = rMinX; x <= rMaxX; x++)
                {
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
                            
                            if (dists0.x <= radiusSq) if (candidateCount < maxResults) candidates[candidateCount++] = new Candidate { Index = originalIndices[start + i], DistSq = dists0.x };
                            if (dists0.y <= radiusSq) if (candidateCount < maxResults) candidates[candidateCount++] = new Candidate { Index = originalIndices[start + i + 1], DistSq = dists0.y };
                            if (dists1.x <= radiusSq) if (candidateCount < maxResults) candidates[candidateCount++] = new Candidate { Index = originalIndices[start + i + 2], DistSq = dists1.x };
                            if (dists1.y <= radiusSq) if (candidateCount < maxResults) candidates[candidateCount++] = new Candidate { Index = originalIndices[start + i + 3], DistSq = dists1.y };
                        }
                        else
                        {
                            for (int j = i; j < count; j++)
                            {
                                float distSq = math.distancesq(sortedPositions[start + j], position);
                                if (distSq <= radiusSq)
                                {
                                    if (candidateCount < maxResults) candidates[candidateCount++] = new Candidate { Index = originalIndices[start + j], DistSq = distSq };
                                }
                            }
                        }
                    }
                }
            }

            if (candidateCount >= k)
            {
                // Need to check if we can terminate early
                // First find K-th smallest without full sort
                float kthDistSq = FindKthSmallest(candidates, candidateCount, k - 1);

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

        // Sort only if requested
        if (sortResults)
        {
            SortCandidates(candidates, candidateCount);
        }
        
        int finalCount = math.min(k, candidateCount);
        for (int i = 0; i < finalCount; i++)
        {
            results[i] = candidates[i].Index;
        }
        *resultCount = finalCount;

        UnsafeUtility.Free(candidates, Allocator.Temp);
    }
    
    // Quick select to find K-th smallest element
    private static float FindKthSmallest(Candidate* arr, int count, int k)
    {
        if (count <= k) return float.MaxValue;
        
        // Simple approach: partial sort for small K
        // Just find the k-th element by scanning
        float kth = float.MaxValue;
        int found = 0;
        for (int i = 0; i < count && found <= k; i++)
        {
            float val = arr[i].DistSq;
            int rank = 0;
            for (int j = 0; j < count; j++)
            {
                if (arr[j].DistSq < val || (arr[j].DistSq == val && j < i)) rank++;
            }
            if (rank == k) return val;
        }
        return kth;
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
            fixed (float2* originPtr = &_origin)
            {
                float2 pos = new float2(position.x, position.y);
                QueryRadiusBurst(pos, radius * radius, (float2*)_sortedPositions.GetUnsafePtr(), (int*)_originalIndices.GetUnsafePtr(), (int*)_cellStart.GetUnsafePtr(), (int*)_cellCount.GetUnsafePtr(), _width, _height, _cellSize, originPtr, resultBuffer, &resultCount, maxResults);
            }

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
    private static void QueryRadiusBurst(in float2 position, float radiusSq, float2* sortedPositions, int* originalIndices, int* cellStart, int* cellCount, int width, int height, float cellSize, [NoAlias] float2* originPtr, int* results, int* resultCount, int maxResults)
    {
        float2 origin = *originPtr;
        float radius = math.sqrt(radiusSq);
        int minX = (int)((position.x - radius - origin.x) / cellSize);
        int maxX = (int)((position.x + radius - origin.x) / cellSize);
        int minY = (int)((position.y - radius - origin.y) / cellSize);
        int maxY = (int)((position.y + radius - origin.y) / cellSize);

        if (minX < 0) minX = 0;
        if (maxX >= width) maxX = width - 1;
        if (minY < 0) minY = 0;
        if (maxY >= height) maxY = height - 1;

        float4 rSqVec = new float4(radiusSq);
        float4 target4 = new float4(position.x, position.y, position.x, position.y);

        for (int y = minY; y <= maxY; y++)
        {
            for (int x = minX; x <= maxX; x++)
            {
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

                        bool4 mask = new bool4(dists0.x <= radiusSq, dists0.y <= radiusSq, dists1.x <= radiusSq, dists1.y <= radiusSq);
                        int maskInt = math.bitmask(mask);
                        
                        if (maskInt > 0)
                        {
                            int4 indices = new int4(originalIndices[start + i], originalIndices[start + i + 1], originalIndices[start + i + 2], originalIndices[start + i + 3]);
                            
                            int4 shuffleMask;
                            GetShuffleMask(maskInt, (int*)&shuffleMask);
                            
                            int* indicesPtr = (int*)&indices;
                            int* shufflePtr = (int*)&shuffleMask;
                            
                            int4 compressed;
                            int* compressedPtr = (int*)&compressed;
                            
                            compressedPtr[0] = indicesPtr[shufflePtr[0]];
                            compressedPtr[1] = indicesPtr[shufflePtr[1]];
                            compressedPtr[2] = indicesPtr[shufflePtr[2]];
                            compressedPtr[3] = indicesPtr[shufflePtr[3]];
                            
                            int validCount = math.countbits(maskInt);
                            *(int4*)(results + *resultCount) = compressed;
                            *resultCount += validCount;
                        }
                    }
                    else
                    {
                        for (int j = i; j < count; j++)
                        {
                            if (math.distancesq(sortedPositions[start + j], position) <= radiusSq)
                            {
                                results[(*resultCount)++] = originalIndices[start + j];
                            }
                        }
                    }
                }
            }
        }
    }
}
