using System;
using System.Collections.Generic;
using UnityEngine;
using Cysharp.Threading.Tasks;
using Unity.Burst;
using Unity.Mathematics;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;

[BurstCompile]
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




    public SIMDQuadTreeIndex(int capacity, Rect bounds)
    {
        _nodes = new NativeArray<Node>(capacity * 2, Allocator.Persistent);
        _linkedUnits = new NativeArray<int>(capacity, Allocator.Persistent);
        _unitIndices = new NativeArray<int>(capacity, Allocator.Persistent);
        _bounds = bounds;
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
                BuildBurst((Node*)_nodes.GetUnsafePtr(), (int*)_linkedUnits.GetUnsafePtr(), (int*)_unitIndices.GetUnsafePtr(), (float2*)posPtr, _count, nodeCountPtr, b, 8, 16);
            }
        }
        _rootIndex = 0;

        return UniTask.CompletedTask;
    }

    [BurstCompile]
    private static void BuildBurst(Node* nodes, int* linkedUnits, int* unitIndices, float2* positions, int count, int* nodeCount, in float4 bounds, int maxDepth, int bucketSize)
    {
        *nodeCount = 0;
        int rootIndex = (*nodeCount)++;
        nodes[rootIndex] = new Node
        {
            X = bounds.x,
            Y = bounds.y,
            W = bounds.z,
            H = bounds.w,
            Child0 = -1,
            Child1 = -1,
            Child2 = -1,
            Child3 = -1,
            FirstUnit = -1,
            Count = 0
        };

        for (int i = 0; i < count; i++)
        {
            Insert(i, positions[i], nodes, linkedUnits, unitIndices, nodeCount, maxDepth, bucketSize, positions);
        }
    }

    private static void Insert(int unitIdx, in float2 pos, Node* nodes, int* linkedUnits, int* unitIndices, int* nodeCount, int maxDepth, int bucketSize, float2* positions)
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

        nodes[c0] = new Node { X = x, Y = y, W = w, H = h, Child0 = -1, Child1 = -1, Child2 = -1, Child3 = -1, FirstUnit = -1 };
        nodes[c1] = new Node { X = x + w, Y = y, W = w, H = h, Child0 = -1, Child1 = -1, Child2 = -1, Child3 = -1, FirstUnit = -1 };
        nodes[c2] = new Node { X = x, Y = y + h, W = w, H = h, Child0 = -1, Child1 = -1, Child2 = -1, Child3 = -1, FirstUnit = -1 };
        nodes[c3] = new Node { X = x + w, Y = y + h, W = w, H = h, Child0 = -1, Child1 = -1, Child2 = -1, Child3 = -1, FirstUnit = -1 };

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



    public void QueryNeighborsBatch(NativeArray<SIMDRVO.AgentData> agents, NativeArray<int> outIndices, NativeArray<int> outCounts, NativeArray<int> outOffsets, int maxNeighbors)
    {
        // Pointers for Burst
        Span<Vector2> posSpan = _positions.AsSpan();
        fixed (Vector2* posPtr = posSpan)
        {
            QueryNeighborsBatchBurst(
                (SIMDRVO.AgentData*)agents.GetUnsafePtr(), 
                agents.Length, 
                (int*)outIndices.GetUnsafePtr(), 
                (int*)outCounts.GetUnsafePtr(), 
                (int*)outOffsets.GetUnsafePtr(), 
                maxNeighbors,
                (Node*)_nodes.GetUnsafePtr(), 
                (int*)_linkedUnits.GetUnsafePtr(), 
                (int*)_unitIndices.GetUnsafePtr(), 
                (float2*)posPtr, 
                _rootIndex
            );
        }
    }

    [BurstCompile]
    private static void QueryNeighborsBatchBurst(
        SIMDRVO.AgentData* agents, int agentCount, 
        int* outIndices, int* outCounts, int* outOffsets, int maxNeighbors,
        Node* nodes, int* linkedUnits, int* unitIndices, float2* positions, int rootIndex)
    {
        for (int i = 0; i < agentCount; i++)
        {
            float2 pos = agents[i].Position;
            // Use large radius or neighborDist? QueryKNearestBurst doesn't filter radius yet.
            // But if we want RVO behavior, we might need radius.
            // However, RVO usually processes K nearest.
            // Let's call QueryKNearestBurst with default infinite radius behavior.
            // Or better, update QueryKNearestBurst to support radius.
            
            // For now, reuse QueryKNearestBurst (which now supports radiusSq).
            // Pass float.MaxValue as radiusSq if we only care about K.
            // But RVO has NeighborDist.
            // agents[i].NeighborDist * agents[i].NeighborDist
            
            int* resultPtr = outIndices + outOffsets[i];
            int tempCount = 0;
            
            // Stackalloc small buffer
            int* tempResults = stackalloc int[128];
            
            // float radiusSq = agents[i].NeighborDist * agents[i].NeighborDist; // If we want to filter
            // But QueryKNearestBurst below needs update to accept radiusSq.
            
            // Let's pass MaxValue for now to maintain previous behavior, or implement filtering.
            QueryKNearestBurst(pos, maxNeighbors, float.MaxValue, nodes, linkedUnits, unitIndices, positions, rootIndex, tempResults, &tempCount, 128);
            
            for(int j=0; j<tempCount; j++)
            {
                resultPtr[j] = tempResults[j];
            }
            outCounts[i] = tempCount;
        }
    }

    public void QueryKNearest(Vector2 position, int k, List<int> results)
    {
        QueryKNearestSorted(position, k, float.MaxValue, results);
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
            Span<Vector2> posSpan = _positions.AsSpan();
            fixed (Vector2* posPtr = posSpan)
            {
                QueryKNearestBurst(new float2(position.x, position.y), k, radius * radius, (Node*)_nodes.GetUnsafePtr(), (int*)_linkedUnits.GetUnsafePtr(), (int*)_unitIndices.GetUnsafePtr(), (float2*)posPtr, _rootIndex, resultBuffer, &resultCount, maxCandidates);
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
    private static void QueryKNearestBurst(in float2 position, int k, float radiusSq, Node* nodes, int* linkedUnits, int* unitIndices, float2* positions, int rootIndex, int* results, int* resultCount, int maxResults)
    {
        Candidate* candidates = (Candidate*)UnsafeUtility.Malloc(maxResults * sizeof(Candidate), 4, Allocator.Temp);
        int candidateCount = 0;

        int* stack = stackalloc int[64];
        int stackCount = 0;

        stack[stackCount++] = rootIndex;

        // Optimization: Track worst distance to prune nodes early
        float worstDistSq = radiusSq; // Start with radius as cutoff

        while (stackCount > 0)
        {
            int nodeIdx = stack[--stackCount];

            // Optimization: Check distance to node bounds before processing
            float distToNode = DistToRect(position, nodes[nodeIdx]);
            if (distToNode * distToNode > worstDistSq) continue; // Use strict greater for radius pruning

            if (nodes[nodeIdx].Child0 == -1)
            {
                int curr = nodes[nodeIdx].FirstUnit;
                while (curr != -1)
                {
                    float dSq = math.distancesq(positions[curr], position);
                    if (dSq <= radiusSq) // Radius check
                    {
                        if (candidateCount < k || dSq < worstDistSq)
                        {
                            AddCandidate(curr, dSq, k, candidates, &candidateCount);
                            if (candidateCount >= k)
                            {
                                // Update worstDistSq based on K-th candidate
                                // Find worst in current set (which is kept sorted by AddCandidate? No, AddCandidate inserts sorted.)
                                // AddCandidate keeps sorted array. So worst is last.
                                worstDistSq = candidates[candidateCount - 1].DistSq;
                            }
                        }
                    }
                    curr = linkedUnits[curr];
                }
                continue;
            }

            int c0 = nodes[nodeIdx].Child0;
            int c1 = nodes[nodeIdx].Child1;
            int c2 = nodes[nodeIdx].Child2;
            int c3 = nodes[nodeIdx].Child3;
            
            if (stackCount < 60) 
            {
                stack[stackCount++] = c0;
                stack[stackCount++] = c1;
                stack[stackCount++] = c2;
                stack[stackCount++] = c3;
            }
        }

        // Sort? AddCandidate keeps them sorted.
        // But verifying sort is cheap here.
        // Actually AddCandidate implementation:
        // "insertPos... for loop shift..." -> Yes, it maintains sorted order.
        
        int finalCount = math.min(k, candidateCount);
        for (int i = 0; i < finalCount; i++) results[i] = candidates[i].Index;
        *resultCount = finalCount;

        UnsafeUtility.Free(candidates, Allocator.Temp);
    }

    private static void AddCandidate(int index, float distSq, int k, Candidate* candidates, int* count)
    {
        int insertPos = 0;
        // Linear scan
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
                QueryRadiusBurst(new float2(position.x, position.y), radius * radius, (Node*)_nodes.GetUnsafePtr(), (int*)_linkedUnits.GetUnsafePtr(), (int*)_unitIndices.GetUnsafePtr(), (float2*)posPtr, _rootIndex, resultBuffer, &resultCount, maxResults);
            }
            for (int i = 0; i < resultCount; i++) results.Add(resultBuffer[i]);
        }
        finally
        {
            UnsafeUtility.Free(resultBuffer, Allocator.Temp);
        }
    }

    [BurstCompile]
    private static void QueryRadiusBurst(in float2 position, float radiusSq, Node* nodes, int* linkedUnits, int* unitIndices, float2* positions, int rootIndex, int* results, int* resultCount, int maxResults)
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
