using System.Collections.Generic;
using UnityEngine;
using System;

public class KDTreeIndex : ISpatialIndex
{
    private struct Node
    {
        public int Axis; // 0 for X, 1 for Y
        public int UnitIndex;
        public int Left;
        public int Right;
        public float SplitValue;
        public float MinX, MinY, MaxX, MaxY; // Bounding box for pruning
    }

    private Node[] _nodes;
    private int _rootIndex;
    private List<Vector2> _positions;
    private int[] _indicesBuffer; // Reusable buffer for building

    public KDTreeIndex(int capacity)
    {
        _nodes = new Node[capacity];
        _indicesBuffer = new int[capacity];
    }

    public void Build(List<Vector2> positions)
    {
        _positions = positions;
        int count = positions.Count;

        if (_nodes.Length < count)
        {
            _nodes = new Node[count];
            _indicesBuffer = new int[count];
        }

        for (int i = 0; i < count; i++)
        {
            _indicesBuffer[i] = i;
        }

        _rootIndex = BuildRecursive(0, count - 0, 0);
    }

    private int BuildRecursive(int start, int length, int depth)
    {
        if (length <= 0) return -1;

        int axis = depth % 2;
        int mid = length / 2;
        int medianIndex = start + mid;

        // Quick select to find median
        QuickSelect(start, length, mid, axis);

        int nodeIndex = _indicesBuffer[medianIndex];
        Vector2 pos = _positions[nodeIndex];

        _nodes[nodeIndex].Axis = axis;
        _nodes[nodeIndex].UnitIndex = nodeIndex;
        _nodes[nodeIndex].SplitValue = (axis == 0) ? pos.x : pos.y;

        // Calculate bounds for this node (could be optimized by passing down bounds)
        // For simple KDTree, we might not need full bounds stored if we just use plane splitting logic.
        // But storing bounds helps with "Ball-Rectangle" intersection tests for KNN.
        // For now, let's stick to standard KDTree traversal without explicit bounds storage per node to save memory,
        // relying on the recursion stack to track current bounds.
        
        _nodes[nodeIndex].Left = BuildRecursive(start, mid, depth + 1);
        _nodes[nodeIndex].Right = BuildRecursive(start + mid + 1, length - mid - 1, depth + 1);

        return nodeIndex;
    }

    private void QuickSelect(int start, int length, int k, int axis)
    {
        int left = start;
        int right = start + length - 1;
        int kAbs = start + k;

        while (left < right)
        {
            int pivotIndex = Partition(left, right, axis);
            if (pivotIndex == kAbs)
                return;
            else if (pivotIndex < kAbs)
                left = pivotIndex + 1;
            else
                right = pivotIndex - 1;
        }
    }

    private int Partition(int left, int right, int axis)
    {
        int pivotIndex = (left + right) / 2;
        int pivotValueIndex = _indicesBuffer[pivotIndex];
        float pivotValue = (axis == 0) ? _positions[pivotValueIndex].x : _positions[pivotValueIndex].y;
        
        Swap(pivotIndex, right);
        int storeIndex = left;

        for (int i = left; i < right; i++)
        {
            int idx = _indicesBuffer[i];
            float val = (axis == 0) ? _positions[idx].x : _positions[idx].y;
            if (val < pivotValue)
            {
                Swap(storeIndex, i);
                storeIndex++;
            }
        }
        Swap(storeIndex, right);
        return storeIndex;
    }

    private void Swap(int i, int j)
    {
        int temp = _indicesBuffer[i];
        _indicesBuffer[i] = _indicesBuffer[j];
        _indicesBuffer[j] = temp;
    }

    public void QueryKNearest(Vector2 position, int k, List<int> results)
    {
        results.Clear();
        // Use a max-heap to store the K nearest neighbors found so far.
        // Since we want the K *nearest*, we keep the K smallest distances.
        // A max-heap of size K allows us to quickly check if a new point is closer than the furthest of our current best K.
        // If it is, we pop the max and insert the new one.
        
        // C# PriorityQueue is available in .NET 6 / Unity 2021.2+.
        // Assuming older Unity or "Pure C#", let's implement a simple custom MaxHeap or just use a List and Sort (slow for inserts).
        // For K=1 (common), simple variable. For K small, List is fine.
        
        // Let's use a custom simple MaxHeap for performance.
        // Actually, for simplicity in this demo, let's use a List and Sort, but optimize:
        // Only sort when full.
        
        // Better: Maintain a list of (index, distSq).
        
        List<(int index, float distSq)> best = new List<(int index, float distSq)>(k + 1);
        
        SearchKNN(_rootIndex, position, k, best);
        
        // Extract indices
        for (int i = 0; i < best.Count; i++)
        {
            results.Add(best[i].index);
        }
    }

    private void SearchKNN(int nodeIndex, Vector2 target, int k, List<(int index, float distSq)> best)
    {
        if (nodeIndex == -1) return;

        int unitIdx = _nodes[nodeIndex].UnitIndex;
        float distSq = (_positions[unitIdx] - target).sqrMagnitude;

        // Try to add to best
        AddCandidate(unitIdx, distSq, k, best);

        // Decide which side to search first
        int axis = _nodes[nodeIndex].Axis;
        float diff = (axis == 0) ? (target.x - _positions[unitIdx].x) : (target.y - _positions[unitIdx].y);
        
        int first = (diff < 0) ? _nodes[nodeIndex].Left : _nodes[nodeIndex].Right;
        int second = (diff < 0) ? _nodes[nodeIndex].Right : _nodes[nodeIndex].Left;

        SearchKNN(first, target, k, best);

        // Pruning: Do we need to search the other side?
        // Only if the distance to the splitting plane is less than the distance to the worst candidate in 'best'.
        // If 'best' is not full, we must search.
        
        bool shouldSearchSecond = true;
        if (best.Count == k)
        {
            // 'best' is full, check if plane is within reach of the worst candidate
            float worstDistSq = best[best.Count - 1].distSq; // List is sorted ascending? No, we need to manage that.
            // If we use a MaxHeap, worst is at top.
            // With List, let's assume we keep it sorted or find max.
            // Let's keep it sorted ascending for simplicity of "nearest".
            // So worst is at the end.
            
            if (diff * diff >= worstDistSq)
            {
                shouldSearchSecond = false;
            }
        }

        if (shouldSearchSecond)
        {
            SearchKNN(second, target, k, best);
        }
    }

    private void AddCandidate(int index, float distSq, int k, List<(int index, float distSq)> best)
    {
        // Insert into sorted list (ascending distance)
        int insertPos = 0;
        while (insertPos < best.Count && best[insertPos].distSq < distSq)
        {
            insertPos++;
        }

        if (insertPos < k)
        {
            best.Insert(insertPos, (index, distSq));
            if (best.Count > k)
            {
                best.RemoveAt(best.Count - 1);
            }
        }
    }

    public void QueryRadius(Vector2 position, float radius, List<int> results)
    {
        results.Clear();
        SearchRadius(_rootIndex, position, radius * radius, results);
    }

    private void SearchRadius(int nodeIndex, Vector2 target, float radiusSq, List<int> results)
    {
        if (nodeIndex == -1) return;

        int unitIdx = _nodes[nodeIndex].UnitIndex;
        float distSq = (_positions[unitIdx] - target).sqrMagnitude;

        if (distSq <= radiusSq)
        {
            results.Add(unitIdx);
        }

        int axis = _nodes[nodeIndex].Axis;
        float diff = (axis == 0) ? (target.x - _positions[unitIdx].x) : (target.y - _positions[unitIdx].y);

        int first = (diff < 0) ? _nodes[nodeIndex].Left : _nodes[nodeIndex].Right;
        int second = (diff < 0) ? _nodes[nodeIndex].Right : _nodes[nodeIndex].Left;

        SearchRadius(first, target, radiusSq, results);

        if (diff * diff <= radiusSq)
        {
            SearchRadius(second, target, radiusSq, results);
        }
    }
}
