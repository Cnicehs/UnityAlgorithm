using System;
using System.Collections.Generic;
using UnityEngine;
using Cysharp.Threading.Tasks;
using Unity.Burst;
using Unity.Mathematics;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;

public unsafe class SIMDQuadTreeIndex : ISpatialIndex, IDisposable
{
    private struct Node
    {
        public int Child0; // BL
        public int Child1; // BR
        public int Child2; // TL
        public int Child3; // TR
        public int FirstUnit;
        public int Count;
        public float X, Y, W, H;
    }

    private NativeArray<Node> _nodes;
    private NativeArray<int> _linkedUnits;
    private NativeArray<int> _unitIndices;
    private int _nodeCount;
    private int _rootIndex;
    
    private List<Vector2> _positions;
    private int _count;
    private Rect _bounds;

    private static readonly SharedStatic<FunctionPointer<BuildDelegate>> _buildFP = SharedStatic<FunctionPointer<BuildDelegate>>.GetOrCreate<SIMDQuadTreeIndex, FunctionPointer<BuildDelegate>>();
    private static readonly SharedStatic<FunctionPointer<QueryKNearestDelegate>> _queryKNearestFP = SharedStatic<FunctionPointer<QueryKNearestDelegate>>.GetOrCreate<SIMDQuadTreeIndex, FunctionPointer<QueryKNearestDelegate>>();
    private static readonly SharedStatic<FunctionPointer<QueryRadiusDelegate>> _queryRadiusFP = SharedStatic<FunctionPointer<QueryRadiusDelegate>>.GetOrCreate<SIMDQuadTreeIndex, FunctionPointer<QueryRadiusDelegate>>();

    private delegate void BuildDelegate(Node* nodes, int* linkedUnits, int* unitIndices, float2* positions, int count, int* nodeCount, float4 bounds, int maxDepth, int bucketSize);
    private delegate void QueryKNearestDelegate(float2 position, int k, Node* nodes, int* linkedUnits, int* unitIndices, float2* positions, int rootIndex, int* results, int* resultCount, int maxResults);
    private delegate void QueryRadiusDelegate(float2 position, float radiusSq, Node* nodes, int* linkedUnits, int* unitIndices, float2* positions, int rootIndex, int* results, int* resultCount, int maxResults);

    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.SubsystemRegistration)]
    private static void Init()
    {
        if (!_buildFP.Data.IsCreated) _buildFP.Data = BurstCompiler.CompileFunctionPointer<BuildDelegate>(BuildBurst);
        if (!_queryKNearestFP.Data.IsCreated) _queryKNearestFP.Data = BurstCompiler.CompileFunctionPointer<QueryKNearestDelegate>(QueryKNearestBurst);
        if (!_queryRadiusFP.Data.IsCreated) _queryRadiusFP.Data = BurstCompiler.CompileFunctionPointer<QueryRadiusDelegate>(QueryRadiusBurst);
    }

    public SIMDQuadTreeIndex(int capacity, Rect bounds)
    {
        _nodes = new NativeArray<Node>(capacity * 2, Allocator.Persistent);
        _linkedUnits = new NativeArray<int>(capacity, Allocator.Persistent);
        _unitIndices = new NativeArray<int>(capacity, Allocator.Persistent);
        _bounds = bounds;
        if (!_buildFP.Data.IsCreated) Init();
    }

    public void Dispose()
    {
        if (_nodes.IsCreated) _nodes.Dispose();
        if (_linkedUnits.IsCreated) _linkedUnits.Dispose();
        if (_unitIndices.IsCreated) _unitIndices.Dispose();
    }

    public UniTask BuildAsync(List<Vector2> positions)
    {
        _positions = positions;
        _count = positions.Count;

        if (_nodes.Length < _count * 2)
        {
            _nodes.Dispose();
            _linkedUnits.Dispose();
            _unitIndices.Dispose();
            _nodes = new NativeArray<Node>(_count * 2, Allocator.Persistent);
            _linkedUnits = new NativeArray<int>(_count, Allocator.Persistent);
            _unitIndices = new NativeArray<int>(_count, Allocator.Persistent);
        }

        Span<Vector2> posSpan = _positions.AsSpan();
        fixed (Vector2* posPtr = posSpan)
        {
            fixed (int* nodeCountPtr = &_nodeCount)
            {
                float4 b = new float4(_bounds.x, _bounds.y, _bounds.width, _bounds.height);
                _buildFP.Data.Invoke((Node*)_nodes.GetUnsafePtr(), (int*)_linkedUnits.GetUnsafePtr(), (int*)_unitIndices.GetUnsafePtr(), (float2*)posPtr, _count, nodeCountPtr, b, 8, 16);
            }
        }
        _rootIndex = 0;

        return UniTask.CompletedTask;
    }

    [BurstCompile]
    private static void BuildBurst(Node* nodes, int* linkedUnits, int* unitIndices, float2* positions, int count, int* nodeCount, float4 bounds, int maxDepth, int bucketSize)
    {
        *nodeCount = 0;
        int rootIndex = (*nodeCount)++;
        nodes[rootIndex] = new Node 
        { 
            X = bounds.x, Y = bounds.y, W = bounds.z, H = bounds.w,
            Child0 = -1, Child1 = -1, Child2 = -1, Child3 = -1,
            FirstUnit = -1, Count = 0 
        };

        for (int i = 0; i < count; i++)
        {
            Insert(i, positions[i], nodes, linkedUnits, unitIndices, nodeCount, maxDepth, bucketSize, positions);
        }
    }

    private static void Insert(int unitIdx, float2 pos, Node* nodes, int* linkedUnits, int* unitIndices, int* nodeCount, int maxDepth, int bucketSize, float2* positions)
    {
        int current = 0; // Root
        int depth = 0;

        while (true)
        {
            if (nodes[current].Child0 == -1)
            {
                if (nodes[current].Count < bucketSize || depth >= maxDepth)
                {
                    unitIndices[unitIdx] = unitIdx;
                    linkedUnits[unitIdx] = nodes[current].FirstUnit;
                    nodes[current].FirstUnit = unitIdx;
                    nodes[current].Count++;
                    return;
                }
                else
                {
                    Split(current, nodes, linkedUnits, unitIndices, positions, nodeCount);

                }
            }

            // Find child
            float midX = nodes[current].X + nodes[current].W * 0.5f;
            float midY = nodes[current].Y + nodes[current].H * 0.5f;
            
            int childIdx = 0;
            if (pos.x > midX) childIdx += 1;
            if (pos.y > midY) childIdx += 2;

            int nextNode = -1;
            switch (childIdx)
            {
                case 0: nextNode = nodes[current].Child0; break;
                case 1: nextNode = nodes[current].Child1; break;
                case 2: nextNode = nodes[current].Child2; break;
                case 3: nextNode = nodes[current].Child3; break;
            }
            
            current = nextNode;
            depth++;
        }
    }



    private static void Split(int nodeIdx, Node* nodes, int* linkedUnits, int* unitIndices, float2* positions, int* nodeCount)
    {
        float x = nodes[nodeIdx].X;
        float y = nodes[nodeIdx].Y;
        float w = nodes[nodeIdx].W * 0.5f;
        float h = nodes[nodeIdx].H * 0.5f;

        int c0 = (*nodeCount)++;
        int c1 = (*nodeCount)++;
        int c2 = (*nodeCount)++;
        int c3 = (*nodeCount)++;

        nodes[c0] = new Node { X = x, Y = y, W = w, H = h, Child0=-1, Child1=-1, Child2=-1, Child3=-1, FirstUnit=-1 };
        nodes[c1] = new Node { X = x+w, Y = y, W = w, H = h, Child0=-1, Child1=-1, Child2=-1, Child3=-1, FirstUnit=-1 };
        nodes[c2] = new Node { X = x, Y = y+h, W = w, H = h, Child0=-1, Child1=-1, Child2=-1, Child3=-1, FirstUnit=-1 };
        nodes[c3] = new Node { X = x+w, Y = y+h, W = w, H = h, Child0=-1, Child1=-1, Child2=-1, Child3=-1, FirstUnit=-1 };

        nodes[nodeIdx].Child0 = c0;
        nodes[nodeIdx].Child1 = c1;
        nodes[nodeIdx].Child2 = c2;
        nodes[nodeIdx].Child3 = c3;

        int currUnit = nodes[nodeIdx].FirstUnit;
        while (currUnit != -1)
        {
            int nextUnit = linkedUnits[currUnit];
            float2 pos = positions[currUnit]; // We need positions here!
            
            int child = 0;
            if (pos.x > x + w) child += 1;
            if (pos.y > y + h) child += 2;

            int targetNode = -1;
            switch (child)
            {
                case 0: targetNode = c0; break;
                case 1: targetNode = c1; break;
                case 2: targetNode = c2; break;
                case 3: targetNode = c3; break;
            }

            linkedUnits[currUnit] = nodes[targetNode].FirstUnit;
            nodes[targetNode].FirstUnit = currUnit;
            nodes[targetNode].Count++;

            currUnit = nextUnit;
        }

        nodes[nodeIdx].FirstUnit = -1;
        nodes[nodeIdx].Count = 0;
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
                _queryKNearestFP.Data.Invoke(new float2(position.x, position.y), k, (Node*)_nodes.GetUnsafePtr(), (int*)_linkedUnits.GetUnsafePtr(), (int*)_unitIndices.GetUnsafePtr(), (float2*)posPtr, _rootIndex, resultBuffer, &resultCount, maxCandidates);
            }
            for (int i = 0; i < resultCount; i++) results.Add(resultBuffer[i]);
        }
        finally
        {
            UnsafeUtility.Free(resultBuffer, Allocator.Temp);
        }
    }

    private struct Candidate : IComparable<Candidate>
    {
        public int Index;
        public float DistSq;
        public int CompareTo(Candidate other) => DistSq.CompareTo(other.DistSq);
    }

    [BurstCompile]
    private static void QueryKNearestBurst(float2 position, int k, Node* nodes, int* linkedUnits, int* unitIndices, float2* positions, int rootIndex, int* results, int* resultCount, int maxResults)
    {
        Candidate* candidates = (Candidate*)UnsafeUtility.Malloc(maxResults * sizeof(Candidate), 4, Allocator.Temp);
        int candidateCount = 0;

        int* stack = stackalloc int[64];
        int stackCount = 0;

        stack[stackCount++] = rootIndex;

        while (stackCount > 0)
        {
            int nodeIdx = stack[--stackCount];

            float distToNode = DistToRect(position, nodes[nodeIdx]);
            if (candidateCount >= k)
            {
                // Find worst
                float worstDistSq = 0;
                for (int i = 0; i < candidateCount; ++i) if (candidates[i].DistSq > worstDistSq) worstDistSq = candidates[i].DistSq;
                if (distToNode * distToNode >= worstDistSq) continue;
            }

            if (nodes[nodeIdx].Child0 == -1)
            {
                int curr = nodes[nodeIdx].FirstUnit;
                while (curr != -1)
                {
                    float dSq = math.distancesq(positions[curr], position);
                    AddCandidate(curr, dSq, k, candidates, &candidateCount);
                    curr = linkedUnits[curr];
                }
                continue;
            }

            // Push children
            if (stackCount < 64) stack[stackCount++] = nodes[nodeIdx].Child0;
            if (stackCount < 64) stack[stackCount++] = nodes[nodeIdx].Child1;
            if (stackCount < 64) stack[stackCount++] = nodes[nodeIdx].Child2;
            if (stackCount < 64) stack[stackCount++] = nodes[nodeIdx].Child3;
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

    private static float DistToRect(float2 p, Node n)
    {
        float dx = math.max(n.X - p.x, 0);
        dx = math.max(dx, p.x - (n.X + n.W));
        float dy = math.max(n.Y - p.y, 0);
        dy = math.max(dy, p.y - (n.Y + n.H));
        return math.sqrt(dx * dx + dy * dy);
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
                _queryRadiusFP.Data.Invoke(new float2(position.x, position.y), radius * radius, (Node*)_nodes.GetUnsafePtr(), (int*)_linkedUnits.GetUnsafePtr(), (int*)_unitIndices.GetUnsafePtr(), (float2*)posPtr, _rootIndex, resultBuffer, &resultCount, maxResults);
            }
            for (int i = 0; i < resultCount; i++) results.Add(resultBuffer[i]);
        }
        finally
        {
            UnsafeUtility.Free(resultBuffer, Allocator.Temp);
        }
    }

    [BurstCompile]
    private static void QueryRadiusBurst(float2 position, float radiusSq, Node* nodes, int* linkedUnits, int* unitIndices, float2* positions, int rootIndex, int* results, int* resultCount, int maxResults)
    {
        float radius = math.sqrt(radiusSq);
        int* stack = stackalloc int[64];
        int stackCount = 0;

        stack[stackCount++] = rootIndex;

        while (stackCount > 0)
        {
            int nodeIdx = stack[--stackCount];

            float dist = DistToRect(position, nodes[nodeIdx]);
            if (dist > radius) continue;

            if (nodes[nodeIdx].Child0 == -1)
            {
                int curr = nodes[nodeIdx].FirstUnit;
                while (curr != -1)
                {
                    if (math.distancesq(positions[curr], position) <= radiusSq)
                    {
                        results[(*resultCount)++] = curr;
                    }
                    curr = linkedUnits[curr];
                }
                continue;
            }

            if (stackCount < 64) stack[stackCount++] = nodes[nodeIdx].Child0;
            if (stackCount < 64) stack[stackCount++] = nodes[nodeIdx].Child1;
            if (stackCount < 64) stack[stackCount++] = nodes[nodeIdx].Child2;
            if (stackCount < 64) stack[stackCount++] = nodes[nodeIdx].Child3;
        }
    }
}
