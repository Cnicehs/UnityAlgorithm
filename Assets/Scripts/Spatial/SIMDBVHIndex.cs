using System;
using System.Collections.Generic;
using UnityEngine;
using Cysharp.Threading.Tasks;
using Unity.Burst;
using Unity.Mathematics;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;

public unsafe class SIMDBVHIndex : ISpatialIndex, IDisposable
{
    private struct Node
    {
        public int Left;
        public int Right;
        public int UnitIndex; // -1 if internal
        public float MinX, MinY, MaxX, MaxY;
    }

    private NativeArray<Node> _nodes;
    private NativeArray<int> _indicesBuffer;
    private int _rootIndex;
    private int _nodeCount;
    
    private List<Vector2> _positions;
    private int _count;

    private static readonly SharedStatic<FunctionPointer<BuildDelegate>> _buildFP = SharedStatic<FunctionPointer<BuildDelegate>>.GetOrCreate<SIMDBVHIndex, FunctionPointer<BuildDelegate>>();
    private static readonly SharedStatic<FunctionPointer<QueryKNearestDelegate>> _queryKNearestFP = SharedStatic<FunctionPointer<QueryKNearestDelegate>>.GetOrCreate<SIMDBVHIndex, FunctionPointer<QueryKNearestDelegate>>();
    private static readonly SharedStatic<FunctionPointer<QueryRadiusDelegate>> _queryRadiusFP = SharedStatic<FunctionPointer<QueryRadiusDelegate>>.GetOrCreate<SIMDBVHIndex, FunctionPointer<QueryRadiusDelegate>>();

    private delegate int BuildDelegate(Node* nodes, int* indices, float2* positions, int count, int* nodeCount);
    private delegate void QueryKNearestDelegate(float2 position, int k, Node* nodes, float2* positions, int rootIndex, int* results, int* resultCount, int maxResults);
    private delegate void QueryRadiusDelegate(float2 position, float radiusSq, Node* nodes, float2* positions, int rootIndex, int* results, int* resultCount, int maxResults);

    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.SubsystemRegistration)]
    private static void Init()
    {
        if (!_buildFP.Data.IsCreated) _buildFP.Data = BurstCompiler.CompileFunctionPointer<BuildDelegate>(BuildBurst);
        if (!_queryKNearestFP.Data.IsCreated) _queryKNearestFP.Data = BurstCompiler.CompileFunctionPointer<QueryKNearestDelegate>(QueryKNearestBurst);
        if (!_queryRadiusFP.Data.IsCreated) _queryRadiusFP.Data = BurstCompiler.CompileFunctionPointer<QueryRadiusDelegate>(QueryRadiusBurst);
    }

    public SIMDBVHIndex(int capacity)
    {
        _nodes = new NativeArray<Node>(capacity * 2, Allocator.Persistent);
        _indicesBuffer = new NativeArray<int>(capacity, Allocator.Persistent);
        if (!_buildFP.Data.IsCreated) Init();
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

        if (_indicesBuffer.Length < _count)
        {
            _nodes.Dispose();
            _indicesBuffer.Dispose();
            _nodes = new NativeArray<Node>(_count * 2, Allocator.Persistent);
            _indicesBuffer = new NativeArray<int>(_count, Allocator.Persistent);
        }

        Span<Vector2> posSpan = _positions.AsSpan();
        fixed (Vector2* posPtr = posSpan)
        {
            fixed (int* nodeCountPtr = &_nodeCount)
            {
                _rootIndex = _buildFP.Data.Invoke((Node*)_nodes.GetUnsafePtr(), (int*)_indicesBuffer.GetUnsafePtr(), (float2*)posPtr, _count, nodeCountPtr);
            }
        }

        return UniTask.CompletedTask;
    }

    private struct BuildJob
    {
        public int NodeIndex;
        public int Start;
        public int Length;
        public int Depth;
    }

    [BurstCompile]
    private static int BuildBurst(Node* nodes, int* indices, float2* positions, int count, int* nodeCount)
    {
        for (int i = 0; i < count; i++) indices[i] = i;
        *nodeCount = 0;

        if (count == 0) return -1;

        int rootIndex = (*nodeCount)++;
        
        BuildJob* stack = stackalloc BuildJob[64];
        int stackCount = 0;

        stack[stackCount++] = new BuildJob { NodeIndex = rootIndex, Start = 0, Length = count, Depth = 0 };

        while (stackCount > 0)
        {
            BuildJob job = stack[--stackCount];
            int nodeIdx = job.NodeIndex;
            int start = job.Start;
            int length = job.Length;

            // Calc AABB
            float minX = float.MaxValue, minY = float.MaxValue;
            float maxX = float.MinValue, maxY = float.MinValue;

            for (int i = 0; i < length; i++)
            {
                float2 p = positions[indices[start + i]];
                if (p.x < minX) minX = p.x;
                if (p.x > maxX) maxX = p.x;
                if (p.y < minY) minY = p.y;
                if (p.y > maxY) maxY = p.y;
            }

            nodes[nodeIdx].MinX = minX;
            nodes[nodeIdx].MinY = minY;
            nodes[nodeIdx].MaxX = maxX;
            nodes[nodeIdx].MaxY = maxY;

            if (length == 1)
            {
                nodes[nodeIdx].UnitIndex = indices[start];
                nodes[nodeIdx].Left = -1;
                nodes[nodeIdx].Right = -1;
                continue;
            }

            nodes[nodeIdx].UnitIndex = -1;

            // Split
            float sizeX = maxX - minX;
            float sizeY = maxY - minY;
            int axis = (sizeX > sizeY) ? 0 : 1;
            int mid = length / 2;

            QuickSelect(indices, positions, start, length, mid, axis);

            int leftChild = (*nodeCount)++;
            int rightChild = (*nodeCount)++;

            nodes[nodeIdx].Left = leftChild;
            nodes[nodeIdx].Right = rightChild;

            if (stackCount < 64) stack[stackCount++] = new BuildJob { NodeIndex = rightChild, Start = start + mid, Length = length - mid, Depth = job.Depth + 1 };
            if (stackCount < 64) stack[stackCount++] = new BuildJob { NodeIndex = leftChild, Start = start, Length = mid, Depth = job.Depth + 1 };
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
        int pivotIdx = indices[pivotIndex];
        float pivotVal = (axis == 0) ? positions[pivotIdx].x : positions[pivotIdx].y;

        int temp = indices[pivotIndex]; indices[pivotIndex] = indices[right]; indices[right] = temp;
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
                _queryKNearestFP.Data.Invoke(new float2(position.x, position.y), k, (Node*)_nodes.GetUnsafePtr(), (float2*)posPtr, _rootIndex, resultBuffer, &resultCount, maxCandidates);
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
    private static void QueryKNearestBurst(float2 position, int k, Node* nodes, float2* positions, int rootIndex, int* results, int* resultCount, int maxResults)
    {
        if (rootIndex == -1) return;

        Candidate* candidates = (Candidate*)UnsafeUtility.Malloc(maxResults * sizeof(Candidate), 4, Allocator.Temp);
        int candidateCount = 0;

        SearchJob* stack = stackalloc SearchJob[64];
        int stackCount = 0;

        stack[stackCount++] = new SearchJob { NodeIndex = rootIndex, MinDistSq = 0 };

        while (stackCount > 0)
        {
            SearchJob job = stack[--stackCount];
            int nodeIdx = job.NodeIndex;

            if (nodeIdx == -1) continue;

            if (candidateCount >= k)
            {
                // Find worst
                float worstDistSq = 0;
                for (int i = 0; i < candidateCount; ++i) if (candidates[i].DistSq > worstDistSq) worstDistSq = candidates[i].DistSq;
                if (job.MinDistSq >= worstDistSq) continue;
            }

            if (nodes[nodeIdx].UnitIndex != -1)
            {
                float dSq = math.distancesq(positions[nodes[nodeIdx].UnitIndex], position);
                AddCandidate(nodes[nodeIdx].UnitIndex, dSq, k, candidates, &candidateCount);
                continue;
            }

            int left = nodes[nodeIdx].Left;
            int right = nodes[nodeIdx].Right;

            float distLeft = DistToAABB(position, nodes, left);
            float distRight = DistToAABB(position, nodes, right);

            if (distLeft < distRight)
            {
                if (right != -1) if (stackCount < 64) stack[stackCount++] = new SearchJob { NodeIndex = right, MinDistSq = distRight };
                if (left != -1) if (stackCount < 64) stack[stackCount++] = new SearchJob { NodeIndex = left, MinDistSq = distLeft };
            }
            else
            {
                if (left != -1) if (stackCount < 64) stack[stackCount++] = new SearchJob { NodeIndex = left, MinDistSq = distLeft };
                if (right != -1) if (stackCount < 64) stack[stackCount++] = new SearchJob { NodeIndex = right, MinDistSq = distRight };
            }
        }

        // Sort
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
        int insertPos = 0;
        while (insertPos < *count && candidates[insertPos].DistSq < distSq) insertPos++;

        if (insertPos < k)
        {
            int end = math.min(*count, k - 1);
            for (int i = end; i > insertPos; i--) candidates[i] = candidates[i - 1];
            candidates[insertPos] = new Candidate { Index = index, DistSq = distSq };
            if (*count < k) (*count)++;
        }
    }

    private static float DistToAABB(float2 p, Node* nodes, int nodeIdx)
    {
        if (nodeIdx == -1) return float.MaxValue;
        float dx = math.max(nodes[nodeIdx].MinX - p.x, 0);
        dx = math.max(dx, p.x - nodes[nodeIdx].MaxX);
        float dy = math.max(nodes[nodeIdx].MinY - p.y, 0);
        dy = math.max(dy, p.y - nodes[nodeIdx].MaxY);
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
            Span<Vector2> posSpan = _positions.AsSpan();
            fixed (Vector2* posPtr = posSpan)
            {
                _queryRadiusFP.Data.Invoke(new float2(position.x, position.y), radius * radius, (Node*)_nodes.GetUnsafePtr(), (float2*)posPtr, _rootIndex, resultBuffer, &resultCount, maxResults);
            }
            for (int i = 0; i < resultCount; i++) results.Add(resultBuffer[i]);
        }
        finally
        {
            UnsafeUtility.Free(resultBuffer, Allocator.Temp);
        }
    }

    [BurstCompile]
    private static void QueryRadiusBurst(float2 position, float radiusSq, Node* nodes, float2* positions, int rootIndex, int* results, int* resultCount, int maxResults)
    {
        if (rootIndex == -1) return;

        SearchJob* stack = stackalloc SearchJob[64];
        int stackCount = 0;

        stack[stackCount++] = new SearchJob { NodeIndex = rootIndex, MinDistSq = 0 };

        while (stackCount > 0)
        {
            SearchJob job = stack[--stackCount];
            int nodeIdx = job.NodeIndex;

            if (nodeIdx == -1) continue;
            if (job.MinDistSq > radiusSq) continue;

            if (nodes[nodeIdx].UnitIndex != -1)
            {
                if (math.distancesq(positions[nodes[nodeIdx].UnitIndex], position) <= radiusSq)
                {
                    results[(*resultCount)++] = nodes[nodeIdx].UnitIndex;
                }
                continue;
            }

            int left = nodes[nodeIdx].Left;
            int right = nodes[nodeIdx].Right;

            float distLeft = DistToAABB(position, nodes, left);
            float distRight = DistToAABB(position, nodes, right);

            if (right != -1 && distRight <= radiusSq) if (stackCount < 64) stack[stackCount++] = new SearchJob { NodeIndex = right, MinDistSq = distRight };
            if (left != -1 && distLeft <= radiusSq) if (stackCount < 64) stack[stackCount++] = new SearchJob { NodeIndex = left, MinDistSq = distLeft };
        }
    }
}
