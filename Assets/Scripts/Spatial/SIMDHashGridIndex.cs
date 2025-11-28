using System;
using System.Collections.Generic;
using UnityEngine;
using Cysharp.Threading.Tasks;
using Unity.Burst;
using Unity.Mathematics;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;

public unsafe class SIMDHashGridIndex : ISpatialIndex, IDisposable
{
    private int _width;
    private int _height;
    private float _cellSize;
    private float2 _origin;
    
    // Grid - using native arrays for Burst compatibility if we wanted, 
    // but user asked to use native C# structures where possible, 
    // BUT to use Burst we need to pass pointers. 
    // So we will keep int[] but pin them or get pointers when calling Burst.
    private int[] _gridHead;
    private int[] _next;
    
    private List<Vector2> _positions; // Reference to the list
    private int _count;

    // Cache
    private List<(int index, float distSq)> _candidateCache = new List<(int index, float distSq)>();

    // Function Pointers
    private static readonly SharedStatic<FunctionPointer<BuildGridDelegate>> _buildGridFP = SharedStatic<FunctionPointer<BuildGridDelegate>>.GetOrCreate<SIMDHashGridIndex, FunctionPointer<BuildGridDelegate>>();
    private static readonly SharedStatic<FunctionPointer<QueryKNearestDelegate>> _queryKNearestFP = SharedStatic<FunctionPointer<QueryKNearestDelegate>>.GetOrCreate<SIMDHashGridIndex, FunctionPointer<QueryKNearestDelegate>>();
    private static readonly SharedStatic<FunctionPointer<QueryRadiusDelegate>> _queryRadiusFP = SharedStatic<FunctionPointer<QueryRadiusDelegate>>.GetOrCreate<SIMDHashGridIndex, FunctionPointer<QueryRadiusDelegate>>();

    // Delegates
    private delegate void BuildGridDelegate(int* gridHead, int* next, float2* positions, int count, int width, int height, float cellSize, float2 origin);
    private delegate void QueryKNearestDelegate(float2 position, int k, float2* positions, int* gridHead, int* next, int width, int height, float cellSize, float2 origin, int* results, int* resultCount, int maxResults);
    private delegate void QueryRadiusDelegate(float2 position, float radiusSq, float2* positions, int* gridHead, int* next, int width, int height, float cellSize, float2 origin, int* results, int* resultCount, int maxResults);

    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.SubsystemRegistration)]
    private static void Init()
    {
        if (!_buildGridFP.Data.IsCreated) _buildGridFP.Data = BurstCompiler.CompileFunctionPointer<BuildGridDelegate>(BuildGridBurst);
        if (!_queryKNearestFP.Data.IsCreated) _queryKNearestFP.Data = BurstCompiler.CompileFunctionPointer<QueryKNearestDelegate>(QueryKNearestBurst);
        if (!_queryRadiusFP.Data.IsCreated) _queryRadiusFP.Data = BurstCompiler.CompileFunctionPointer<QueryRadiusDelegate>(QueryRadiusBurst);
    }

    public SIMDHashGridIndex(int width, int height, float cellSize, Vector2 origin, int capacity)
    {
        _width = width;
        _height = height;
        _cellSize = cellSize;
        _origin = new float2(origin.x, origin.y);
        
        _gridHead = new int[_width * _height];
        _next = new int[capacity];

        // Ensure init if not already (constructor might run before subsystem reg in editor sometimes?)
        if (!_buildGridFP.Data.IsCreated) Init();
    }

    public void Dispose()
    {
        // Nothing unmanaged to dispose, arrays are managed
    }

    public UniTask BuildAsync(List<Vector2> positions)
    {
        _positions = positions;
        _count = positions.Count;
        
        if (_next.Length < _count)
        {
            _next = new int[_count];
        }
        
        // 1. Clear Grid
        Array.Fill(_gridHead, -1);

        // 2. Build Grid using Burst
        // Get pointers
        fixed (int* gridHeadPtr = _gridHead)
        fixed (int* nextPtr = _next)
        {
            // Access List<T> internal array via ListUtils
            Span<Vector2> posSpan = _positions.AsSpan();
            fixed (Vector2* posPtr = posSpan)
            {
                _buildGridFP.Data.Invoke(gridHeadPtr, nextPtr, (float2*)posPtr, _count, _width, _height, _cellSize, _origin);
            }
        }

        return UniTask.CompletedTask;
    }

    [BurstCompile]
    private static void BuildGridBurst(int* gridHead, int* next, float2* positions, int count, int width, int height, float cellSize, float2 origin)
    {
        for (int i = 0; i < count; i++)
        {
            float2 p = positions[i];
            int x = (int)((p.x - origin.x) / cellSize);
            int y = (int)((p.y - origin.y) / cellSize);
            
            if (x < 0) x = 0; else if (x >= width) x = width - 1;
            if (y < 0) y = 0; else if (y >= height) y = height - 1;
            
            int cellIndex = y * width + x;
            
            next[i] = gridHead[cellIndex];
            gridHead[cellIndex] = i;
        }
    }

    public void QueryKNearest(Vector2 position, int k, List<int> results)
    {
        results.Clear();
        
        // We need a temporary buffer for results from Burst
        // Since we can't easily pass List<int> to Burst to Add()
        // We'll use a stackalloc buffer if k is small, or a NativeArray if large.
        // Assuming k is relatively small for KNN.
        
        // However, KNN with rings is complex to fully burstify in one go without a scratchpad.
        // The user wants "SIMD version", "Burst compiled".
        // A full KNN search in Burst requires managing the candidate list.
        
        // Let's allocate a temporary buffer for candidates.
        // Max candidates? Hard to know.
        // But wait, the previous implementation gathered candidates then sorted.
        // We can do the same in Burst.
        
        int maxCandidates = k * 10; // Heuristic
        if (maxCandidates < 128) maxCandidates = 128;
        
        // We'll use a native array for the results to pass to burst
        // Or just stackalloc if we are on the main thread and k is small.
        // But k is dynamic.
        
        // Let's use a pooled array or just a temporary NativeArray.
        // Since we are not using Job System, we can just use UnsafeUtility.Malloc
        
        int* resultBuffer = (int*)UnsafeUtility.Malloc(maxCandidates * sizeof(int), 4, Allocator.Temp);
        int resultCount = 0;

        try
        {
            fixed (int* gridHeadPtr = _gridHead)
            fixed (int* nextPtr = _next)
            {
                Span<Vector2> posSpan = _positions.AsSpan();
                fixed (Vector2* posPtr = posSpan)
                {
                    _queryKNearestFP.Data.Invoke(new float2(position.x, position.y), k, (float2*)posPtr, gridHeadPtr, nextPtr, _width, _height, _cellSize, _origin, resultBuffer, &resultCount, maxCandidates);
                }
            }

            // Copy back
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
    private static void QueryKNearestBurst(float2 position, int k, float2* positions, int* gridHead, int* next, int width, int height, float cellSize, float2 origin, int* results, int* resultCount, int maxResults)
    {
        // We need a local cache for candidates
        // Since we can't allocate managed objects, we use stackalloc or UnsafeUtility
        // But we are inside a function pointer, so we should be careful with allocations.
        // We can use a fixed size buffer on stack if small, or just iterate.
        
        // Implementation of the ring search in Burst
        
        int startX = (int)((position.x - origin.x) / cellSize);
        int startY = (int)((position.y - origin.y) / cellSize);
        
        if (startX < 0) startX = 0; else if (startX >= width) startX = width - 1;
        if (startY < 0) startY = 0; else if (startY >= height) startY = height - 1;

        int searchRadius = 0;
        int maxSearchRadius = math.max(width, height);
        
        // We need to store candidates (index, distSq)
        // We can't use List. We'll use a raw pointer buffer.
        // We'll reuse the 'results' buffer for indices, but we need dists too.
        // Actually, let's just find the best K.
        // A simple MinHeap or just gathering and sorting at the end.
        // For simplicity and SIMD-friendliness (batching), gathering is good.
        
        // We need a scratch buffer for candidates. 
        // Let's assume maxResults is enough for our candidates.
        
        // Allocate temporary memory for candidates (Index, DistSq)
        // We can't use Malloc in Burst easily without passing an allocator? 
        // Actually Allocator.Temp works in Burst if on main thread? 
        // No, "Burst does not support managed objects". 
        // UnsafeUtility.Malloc IS supported.
        
        Candidate* candidates = (Candidate*)UnsafeUtility.Malloc(maxResults * sizeof(Candidate), 4, Allocator.Temp);
        int candidateCount = 0;
        
        // SIMD setup
        float2 target = position;
        
        while (searchRadius < maxSearchRadius)
        {
            int minX = math.max(0, startX - searchRadius);
            int maxX = math.min(width - 1, startX + searchRadius);
            int minY = math.max(0, startY - searchRadius);
            int maxY = math.min(height - 1, startY + searchRadius);

            // Ring Loop
            if (searchRadius == 0)
            {
                ProcessCell(startX, startY, target, positions, gridHead, next, width, candidates, &candidateCount, maxResults);
            }
            else
            {
                for (int x = minX; x <= maxX; x++)
                {
                    if (minY == startY - searchRadius) ProcessCell(x, minY, target, positions, gridHead, next, width, candidates, &candidateCount, maxResults);
                    if (maxY == startY + searchRadius && maxY != minY) ProcessCell(x, maxY, target, positions, gridHead, next, width, candidates, &candidateCount, maxResults);
                }
                for (int y = minY + 1; y <= maxY - 1; y++)
                {
                    if (minX == startX - searchRadius) ProcessCell(minX, y, target, positions, gridHead, next, width, candidates, &candidateCount, maxResults);
                    if (maxX == startX + searchRadius && maxX != minX) ProcessCell(maxX, y, target, positions, gridHead, next, width, candidates, &candidateCount, maxResults);
                }
            }

            if (candidateCount >= k)
            {
                // Sort candidates
                // Simple bubble sort or insertion sort for small N, or QuickSort
                // Since we are in Burst, we can write a simple sort.
                SortCandidates(candidates, candidateCount);
                
                float kthDistSq = candidates[k - 1].DistSq;
                
                // Dist to next ring
                float distToNextRingSq = GetDistanceToRectSq(position, 
                    (startX - (searchRadius + 1)) * cellSize + origin.x,
                    (startY - (searchRadius + 1)) * cellSize + origin.y,
                    (startX + (searchRadius + 1) + 1) * cellSize + origin.x,
                    (startY + (searchRadius + 1) + 1) * cellSize + origin.y
                );

                if (kthDistSq <= distToNextRingSq) break;
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

    private static void ProcessCell(int x, int y, float2 target, float2* positions, int* gridHead, int* next, int width, Candidate* candidates, int* candidateCount, int maxCandidates)
    {
        int cellIndex = y * width + x;
        int current = gridHead[cellIndex];
        
        // SIMD Batching
        // We can process 4 at a time using float4
        // But we are reading from linked list (random access), so gather is needed.
        // Since we are in Burst, the compiler might auto-vectorize if we write it simply.
        // But manual SIMD with linked list is tricky because of the dependency chain.
        // We can gather into a buffer.
        
        int batchSize = 4;
        float2* batchPos = stackalloc float2[batchSize];
        int* batchIndices = stackalloc int[batchSize];
        int count = 0;

        while (current != -1)
        {
            if (*candidateCount >= maxCandidates) break; // Safety break

            batchPos[count] = positions[current];
            batchIndices[count] = current;
            count++;

            if (count == batchSize)
            {
                ProcessBatch(batchPos, batchIndices, count, target, candidates, candidateCount, maxCandidates);
                count = 0;
            }

            current = next[current];
        }

        if (count > 0)
        {
            ProcessBatch(batchPos, batchIndices, count, target, candidates, candidateCount, maxCandidates);
        }
    }

    private static void ProcessBatch(float2* batchPos, int* batchIndices, int count, float2 target, Candidate* candidates, int* candidateCount, int maxCandidates)
    {
        // Manual SIMD using float4
        // We have up to 4 points.
        // x1, y1, x2, y2, ...
        
        // Load into float4s
        // x: x1, x2, x3, x4
        // y: y1, y2, y3, y4
        
        float4 px = new float4(
            count > 0 ? batchPos[0].x : 0,
            count > 1 ? batchPos[1].x : 0,
            count > 2 ? batchPos[2].x : 0,
            count > 3 ? batchPos[3].x : 0
        );
        
        float4 py = new float4(
            count > 0 ? batchPos[0].y : 0,
            count > 1 ? batchPos[1].y : 0,
            count > 2 ? batchPos[2].y : 0,
            count > 3 ? batchPos[3].y : 0
        );
        
        float4 tx = new float4(target.x);
        float4 ty = new float4(target.y);
        
        float4 dx = px - tx;
        float4 dy = py - ty;
        float4 dSq = dx * dx + dy * dy;
        
        for (int i = 0; i < count; i++)
        {
            if (*candidateCount < maxCandidates)
            {
                candidates[*candidateCount] = new Candidate { Index = batchIndices[i], DistSq = dSq[i] };
                (*candidateCount)++;
            }
        }
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
        
        // Similar strategy: Allocate temp buffer
        int maxResults = _count; // Worst case all
        // But allocating _count might be too big for stack.
        // Use UnsafeUtility.Malloc(Allocator.Temp)
        
        int* resultBuffer = (int*)UnsafeUtility.Malloc(maxResults * sizeof(int), 4, Allocator.Temp);
        int resultCount = 0;

        try
        {
            fixed (int* gridHeadPtr = _gridHead)
            fixed (int* nextPtr = _next)
            {
                Span<Vector2> posSpan = _positions.AsSpan();
                fixed (Vector2* posPtr = posSpan)
                {
                    _queryRadiusFP.Data.Invoke(new float2(position.x, position.y), radius * radius, (float2*)posPtr, gridHeadPtr, nextPtr, _width, _height, _cellSize, _origin, resultBuffer, &resultCount, maxResults);
                }
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
    private static void QueryRadiusBurst(float2 position, float radiusSq, float2* positions, int* gridHead, int* next, int width, int height, float cellSize, float2 origin, int* results, int* resultCount, int maxResults)
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
        
        float2 target = position;
        float4 rSqVec = new float4(radiusSq);
        float4 tx = new float4(target.x);
        float4 ty = new float4(target.y);

        int batchSize = 4;
        float2* batchPos = stackalloc float2[batchSize];
        int* batchIndices = stackalloc int[batchSize];
        
        int count = 0;

        for (int y = minY; y <= maxY; y++)
        {
            for (int x = minX; x <= maxX; x++)
            {
                int cellIndex = y * width + x;
                int current = gridHead[cellIndex];
                
                while (current != -1)
                {
                    batchPos[count] = positions[current];
                    batchIndices[count] = current;
                    count++;

                    if (count == batchSize)
                    {
                        // Process Batch
                        float4 px = new float4(batchPos[0].x, batchPos[1].x, batchPos[2].x, batchPos[3].x);
                        float4 py = new float4(batchPos[0].y, batchPos[1].y, batchPos[2].y, batchPos[3].y);
                        
                        float4 dx = px - tx;
                        float4 dy = py - ty;
                        float4 dSq = dx * dx + dy * dy;
                        
                        bool4 mask = dSq <= rSqVec;
                        
                        if (mask.x) results[(*resultCount)++] = batchIndices[0];
                        if (mask.y) results[(*resultCount)++] = batchIndices[1];
                        if (mask.z) results[(*resultCount)++] = batchIndices[2];
                        if (mask.w) results[(*resultCount)++] = batchIndices[3];
                        
                        count = 0;
                    }
                    
                    current = next[current];
                }
            }
        }
        
        // Leftovers
        if (count > 0)
        {
            for (int i = 0; i < count; i++)
            {
                float dx = batchPos[i].x - target.x;
                float dy = batchPos[i].y - target.y;
                if (dx*dx + dy*dy <= radiusSq)
                {
                    results[(*resultCount)++] = batchIndices[i];
                }
            }
        }
    }
}
