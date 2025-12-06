using System;
using System.Collections.Generic;
using UnityEngine;
using Cysharp.Threading.Tasks;
using Unity.Burst;
using Unity.Mathematics;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;

[BurstCompile]
public unsafe class SIMDKDTreeIndex : ISpatialIndex, IDisposable
{
    private struct Node
    {
        public int Axis; // 0: X, 1: Y
        public int UnitIndex;
        public int Left;
        public int Right;
        public float SplitValue;
    }

    private NativeArray<Node> _nodes; // Using NativeArray for easier management, can get pointer
    private NativeArray<int> _indicesBuffer; // Reusable buffer
    private int _rootIndex;
    
    private List<Vector2> _positions;
    private int _count;



    public SIMDKDTreeIndex(int capacity)
    {
        _nodes = new NativeArray<Node>(capacity, Allocator.Persistent);
        _indicesBuffer = new NativeArray<int>(capacity, Allocator.Persistent);
    }

    public void Dispose()
    {
        if (_nodes.IsCreated) _nodes.Dispose();
        if (_indicesBuffer.IsCreated) _indicesBuffer.Dispose();
    }

    public UniTask BuildAsync(List<Vector2> positions)
    {
        _positions = positions;
        _count = positions.Count;

        if (_nodes.Length < _count)
        {
            _nodes.Dispose();
            _indicesBuffer.Dispose();
            _nodes = new NativeArray<Node>(_count, Allocator.Persistent);
            _indicesBuffer = new NativeArray<int>(_count, Allocator.Persistent);
        }

        Span<Vector2> posSpan = _positions.AsSpan();
        fixed (Vector2* posPtr = posSpan)
        {
            _rootIndex = BuildBurst((Node*)_nodes.GetUnsafePtr(), (int*)_indicesBuffer.GetUnsafePtr(), (float2*)posPtr, _count);
        }

        return UniTask.CompletedTask;
    }

    private struct BuildJob
    {
        public int Start;
        public int Length;
        public int Depth;
        public int ParentIndex;
        public bool IsLeft;
    }

    [BurstCompile]
    private static int BuildBurst(Node* nodes, int* indices, float2* positions, int count)
    {
        for (int i = 0; i < count; i++) indices[i] = i;

        // Iterative Build
        // We need a stack.
        // Stack size? Log2(N) * 2 roughly. For 1M units, depth ~20. Stack 64 is plenty.
        // But we need to allocate it.
        
        BuildJob* stack = stackalloc BuildJob[64];
        int stackCount = 0;

        // Root
        int rootIndex = -1;
        
        // We can't return rootIndex directly from iterative if we don't know it yet.
        // But we know the first processed node will be root? 
        // No, median is root.
        
        // Let's do the first step manually to get root.
        int mid = count / 2;
        QuickSelect(indices, positions, 0, count, mid, 0);
        rootIndex = indices[mid];
        
        nodes[rootIndex].Axis = 0;
        nodes[rootIndex].UnitIndex = rootIndex;
        nodes[rootIndex].SplitValue = positions[rootIndex].x;
        nodes[rootIndex].Left = -1;
        nodes[rootIndex].Right = -1;

        // Push children
        if (stackCount < 64) stack[stackCount++] = new BuildJob { Start = mid + 1, Length = count - mid - 1, Depth = 1, ParentIndex = rootIndex, IsLeft = false };
        if (stackCount < 64) stack[stackCount++] = new BuildJob { Start = 0, Length = mid, Depth = 1, ParentIndex = rootIndex, IsLeft = true };

        while (stackCount > 0)
        {
            BuildJob job = stack[--stackCount];
            
            if (job.Length <= 0)
            {
                if (job.IsLeft) nodes[job.ParentIndex].Left = -1;
                else nodes[job.ParentIndex].Right = -1;
                continue;
            }

            int axis = job.Depth % 2;
            int jobMid = job.Length / 2;
            int medianIndex = job.Start + jobMid;

            QuickSelect(indices, positions, job.Start, job.Length, jobMid, axis);

            int nodeIndex = indices[medianIndex];
            float2 pos = positions[nodeIndex];

            nodes[nodeIndex].Axis = axis;
            nodes[nodeIndex].UnitIndex = nodeIndex;
            nodes[nodeIndex].SplitValue = (axis == 0) ? pos.x : pos.y;
            nodes[nodeIndex].Left = -1;
            nodes[nodeIndex].Right = -1;

            if (job.IsLeft) nodes[job.ParentIndex].Left = nodeIndex;
            else nodes[job.ParentIndex].Right = nodeIndex;

            if (stackCount < 64) stack[stackCount++] = new BuildJob { Start = job.Start + jobMid + 1, Length = job.Length - jobMid - 1, Depth = job.Depth + 1, ParentIndex = nodeIndex, IsLeft = false };
            if (stackCount < 64) stack[stackCount++] = new BuildJob { Start = job.Start, Length = jobMid, Depth = job.Depth + 1, ParentIndex = nodeIndex, IsLeft = true };
        }

        return rootIndex;
    }

    private static void QuickSelect(int* indices, float2* positions, int start, int length, int k, int axis)
    {
        int left = start;
        int right = start + length - 1;
        int kAbs = start + k;

        while (left < right)
        {
            int pivotIndex = Partition(indices, positions, left, right, axis);
            if (pivotIndex == kAbs) return;
            else if (pivotIndex < kAbs) left = pivotIndex + 1;
            else right = pivotIndex - 1;
        }
    }

    private static int Partition(int* indices, float2* positions, int left, int right, int axis)
    {
        int pivotIndex = (left + right) / 2;
        int pivotValIdx = indices[pivotIndex];
        float pivotVal = (axis == 0) ? positions[pivotValIdx].x : positions[pivotValIdx].y;

        int temp = indices[pivotIndex]; indices[pivotIndex] = indices[right]; indices[right] = temp; // Swap pivot to end

        int storeIndex = left;
        for (int i = left; i < right; i++)
        {
            int idx = indices[i];
            float val = (axis == 0) ? positions[idx].x : positions[idx].y;
            if (val < pivotVal)
            {
                temp = indices[storeIndex]; indices[storeIndex] = indices[i]; indices[i] = temp;
                storeIndex++;
            }
        }
        
        temp = indices[storeIndex]; indices[storeIndex] = indices[right]; indices[right] = temp;
        return storeIndex;
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
            Span<Vector2> posSpan = _positions.AsSpan();
            fixed (Vector2* posPtr = posSpan)
            {
                QueryKNearestBurst(new float2(position.x, position.y), k, (Node*)_nodes.GetUnsafePtr(), (float2*)posPtr, _rootIndex, resultBuffer, &resultCount, maxCandidates);
            }

            for (int i = 0; i < resultCount; i++) results.Add(resultBuffer[i]);
        }
        finally
        {
            UnsafeUtility.Free(resultBuffer, Allocator.Temp);
        }
    }

    private struct SearchJob
    {
        public int NodeIndex;
        public float MinDistSq;
    }

    private struct Candidate : IComparable<Candidate>
    {
        public int Index;
        public float DistSq;
        public int CompareTo(Candidate other) => DistSq.CompareTo(other.DistSq);
    }

    [BurstCompile]
    private static void QueryKNearestBurst(in float2 position, int k, Node* nodes, float2* positions, int rootIndex, int* results, int* resultCount, int maxResults)
    {
        if (rootIndex == -1) return;

        Candidate* candidates = (Candidate*)UnsafeUtility.Malloc(maxResults * sizeof(Candidate), 4, Allocator.Temp);
        int candidateCount = 0;

        SearchJob* stack = stackalloc SearchJob[64];
        int stackCount = 0;

        stack[stackCount++] = new SearchJob { NodeIndex = rootIndex, MinDistSq = 0 };

        // Optimization: Track worst distance to prune nodes early
        float worstDistSq = float.MaxValue;

        while (stackCount > 0)
        {
            SearchJob job = stack[--stackCount];
            int nodeIndex = job.NodeIndex;

            if (nodeIndex == -1) continue;

            // Optimization: Pruning
            if (candidateCount >= k && job.MinDistSq >= worstDistSq) continue;

            int unitIdx = nodes[nodeIndex].UnitIndex;
            float distSq = math.distancesq(positions[unitIdx], position);

            if (candidateCount < k || distSq < worstDistSq)
            {
                AddCandidate(unitIdx, distSq, k, candidates, &candidateCount);
                if (candidateCount >= k)
                {
                    // Update worstDistSq
                    worstDistSq = 0;
                    for (int i = 0; i < candidateCount; ++i) if (candidates[i].DistSq > worstDistSq) worstDistSq = candidates[i].DistSq;
                }
            }

            int axis = nodes[nodeIndex].Axis;
            float diff = (axis == 0) ? (position.x - positions[unitIdx].x) : (position.y - positions[unitIdx].y);
            
            int first = (diff < 0) ? nodes[nodeIndex].Left : nodes[nodeIndex].Right;
            int second = (diff < 0) ? nodes[nodeIndex].Right : nodes[nodeIndex].Left;
            
            float diffSq = diff * diff;

            if (second != -1)
            {
                if (stackCount < 64) stack[stackCount++] = new SearchJob { NodeIndex = second, MinDistSq = diffSq };
            }
            if (first != -1)
            {
                if (stackCount < 64) stack[stackCount++] = new SearchJob { NodeIndex = first, MinDistSq = 0 };
            }
        }

        // Sort and copy
        // Simple sort
        for (int i = 1; i < candidateCount; ++i)
        {
            Candidate key = candidates[i];
            int j = i - 1;
            while (j >= 0 && candidates[j].DistSq > key.DistSq)
            {
                candidates[j + 1] = candidates[j];
                j--;
            }
            candidates[j + 1] = key;
        }

        int finalCount = math.min(k, candidateCount);
        for (int i = 0; i < finalCount; i++) results[i] = candidates[i].Index;
        *resultCount = finalCount;

        UnsafeUtility.Free(candidates, Allocator.Temp);
    }

    private static void AddCandidate(int index, float distSq, int k, Candidate* candidates, int* count)
    {
        // Insert sorted
        int insertPos = 0;
        while (insertPos < *count && candidates[insertPos].DistSq < distSq) insertPos++;

        if (insertPos < k)
        {
            // Shift
            int end = math.min(*count, k - 1);
            for (int i = end; i > insertPos; i--)
            {
                candidates[i] = candidates[i - 1];
            }
            candidates[insertPos] = new Candidate { Index = index, DistSq = distSq };
            if (*count < k) (*count)++;
        }
    }

    public void QueryRadius(Vector2 position, float radius, List<int> results)
    {
        results.Clear();
        int maxResults = _count; 
        int* resultBuffer = (int*)UnsafeUtility.Malloc(maxResults * sizeof(int), 4, Allocator.Temp);
        int resultCount = 0;

        try
        {
            Span<Vector2> posSpan = _positions.AsSpan();
            fixed (Vector2* posPtr = posSpan)
            {
                QueryRadiusBurst(new float2(position.x, position.y), radius * radius, (Node*)_nodes.GetUnsafePtr(), (float2*)posPtr, _rootIndex, resultBuffer, &resultCount, maxResults);
            }
            for (int i = 0; i < resultCount; i++) results.Add(resultBuffer[i]);
        }
        finally
        {
            UnsafeUtility.Free(resultBuffer, Allocator.Temp);
        }
    }

    [BurstCompile]
    private static void QueryRadiusBurst(in float2 position, float radiusSq, Node* nodes, float2* positions, int rootIndex, int* results, int* resultCount, int maxResults)
    {
        if (rootIndex == -1) return;

        SearchJob* stack = stackalloc SearchJob[64];
        int stackCount = 0;

        stack[stackCount++] = new SearchJob { NodeIndex = rootIndex, MinDistSq = 0 };

        while (stackCount > 0)
        {
            SearchJob job = stack[--stackCount];
            int nodeIndex = job.NodeIndex;

            if (nodeIndex == -1) continue;
            if (job.MinDistSq > radiusSq) continue;

            int unitIdx = nodes[nodeIndex].UnitIndex;
            float distSq = math.distancesq(positions[unitIdx], position);

            if (distSq <= radiusSq)
            {
                results[(*resultCount)++] = unitIdx;
            }

            int axis = nodes[nodeIndex].Axis;
            float diff = (axis == 0) ? (position.x - positions[unitIdx].x) : (position.y - positions[unitIdx].y);
            
            int first = (diff < 0) ? nodes[nodeIndex].Left : nodes[nodeIndex].Right;
            int second = (diff < 0) ? nodes[nodeIndex].Right : nodes[nodeIndex].Left;
            
            float diffSq = diff * diff;

            if (second != -1 && diffSq <= radiusSq)
            {
                if (stackCount < 64) stack[stackCount++] = new SearchJob { NodeIndex = second, MinDistSq = diffSq };
            }
            if (first != -1)
            {
                if (stackCount < 64) stack[stackCount++] = new SearchJob { NodeIndex = first, MinDistSq = 0 };
            }
        }
    }
}
