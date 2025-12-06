using System.Collections.Generic;
using UnityEngine;
using Cysharp.Threading.Tasks;
using System.Threading.Tasks;

public class BVHIndex : ISpatialIndex
{
    private struct Node
    {
        public int Left;
        public int Right;
        public int UnitIndex; // -1 if internal node
        public float MinX, MinY, MaxX, MaxY; // AABB
    }

    private Node[] _nodes;
    private int _rootIndex;
    private List<Vector2> _positions;
    private int[] _indicesBuffer;
    private int _nodeCount;

    // Cache for query candidates
    private List<(int index, float distSq)> _candidateCache = new List<(int index, float distSq)>();

    // Stack for iterative build/query
    private struct BuildJob
    {
        public int NodeIndex;
        public int Start;
        public int Length;
        public int Depth;
    }
    
    private struct SearchJob
    {
        public int NodeIndex;
        public float MinDistSq;
    }
    private Stack<SearchJob> _searchStack = new Stack<SearchJob>(64);

    public BVHIndex(int capacity)
    {
        // BVH can have up to 2*N - 1 nodes
        _nodes = new Node[capacity * 2];
        _indicesBuffer = new int[capacity];
    }

    public UniTask BuildAsync(List<Vector2> positions)
    {
        _positions = positions;
        int count = positions.Count;

        if (_indicesBuffer.Length < count)
        {
            _indicesBuffer = new int[count];
            _nodes = new Node[count * 2];
        }

        for (int i = 0; i < count; i++)
        {
            _indicesBuffer[i] = i;
        }

        _nodeCount = 0;
        _rootIndex = BuildIterative(0, count);

        return UniTask.CompletedTask;
    }

    private int BuildIterative(int start, int length)
    {
        if (length == 0) return -1;

        // Create root
        int rootIndex = _nodeCount++;
        
        var stack = new Stack<BuildJob>();
        stack.Push(new BuildJob { NodeIndex = rootIndex, Start = start, Length = length, Depth = 0 });

        while (stack.Count > 0)
        {
            var job = stack.Pop();
            int nodeIdx = job.NodeIndex;
            int jobStart = job.Start;
            int jobLength = job.Length;

            // Calculate AABB for this node
            float minX = float.MaxValue, minY = float.MaxValue;
            float maxX = float.MinValue, maxY = float.MinValue;

            for (int i = 0; i < jobLength; i++)
            {
                Vector2 pos = _positions[_indicesBuffer[jobStart + i]];
                if (pos.x < minX) minX = pos.x;
                if (pos.x > maxX) maxX = pos.x;
                if (pos.y < minY) minY = pos.y;
                if (pos.y > maxY) maxY = pos.y;
            }

            _nodes[nodeIdx].MinX = minX;
            _nodes[nodeIdx].MinY = minY;
            _nodes[nodeIdx].MaxX = maxX;
            _nodes[nodeIdx].MaxY = maxY;

            // Leaf node condition
            if (jobLength == 1)
            {
                _nodes[nodeIdx].UnitIndex = _indicesBuffer[jobStart];
                _nodes[nodeIdx].Left = -1;
                _nodes[nodeIdx].Right = -1;
                continue;
            }

            _nodes[nodeIdx].UnitIndex = -1;

            // Split strategy: Longest Axis Median
            float sizeX = maxX - minX;
            float sizeY = maxY - minY;
            int axis = (sizeX > sizeY) ? 0 : 1;

            int mid = jobLength / 2;
            
            // QuickSelect to partition
            QuickSelect(jobStart, jobLength, mid, axis);

            // Allocate children
            int leftChildIdx = _nodeCount++;
            int rightChildIdx = _nodeCount++;

            _nodes[nodeIdx].Left = leftChildIdx;
            _nodes[nodeIdx].Right = rightChildIdx;

            // Push children to stack
            stack.Push(new BuildJob { NodeIndex = rightChildIdx, Start = jobStart + mid, Length = jobLength - mid, Depth = job.Depth + 1 });
            stack.Push(new BuildJob { NodeIndex = leftChildIdx, Start = jobStart, Length = mid, Depth = job.Depth + 1 });
        }

        return rootIndex;
    }

    private void QuickSelect(int start, int length, int k, int axis)
    {
        int left = start;
        int right = start + length - 1;
        int kAbs = start + k;

        while (left < right)
        {
            int pivotIndex = Partition(left, right, axis);
            if (pivotIndex == kAbs) return;
            else if (pivotIndex < kAbs) left = pivotIndex + 1;
            else right = pivotIndex - 1;
        }
    }

    private int Partition(int left, int right, int axis)
    {
        int pivotIndex = (left + right) / 2;
        int pivotIdx = _indicesBuffer[pivotIndex];
        float pivotVal = (axis == 0) ? _positions[pivotIdx].x : _positions[pivotIdx].y;

        Swap(pivotIndex, right);
        int storeIndex = left;

        for (int i = left; i < right; i++)
        {
            int idx = _indicesBuffer[i];
            float val = (axis == 0) ? _positions[idx].x : _positions[idx].y;
            if (val < pivotVal)
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
        _candidateCache.Clear();
        if (_candidateCache.Capacity < k + 1) _candidateCache.Capacity = k + 1;

        // Use unsorted search - collect K nearest without maintaining sort order
        SearchKNNUnsorted(_rootIndex, position, k, _candidateCache);
        
        for (int i = 0; i < _candidateCache.Count; i++) results.Add(_candidateCache[i].index);
    }
    
    private void SearchKNNUnsorted(int rootIndex, Vector2 target, int k, List<(int index, float distSq)> best)
    {
        if (rootIndex == -1) return;
        
        _searchStack.Clear();
        _searchStack.Push(new SearchJob { NodeIndex = rootIndex, MinDistSq = 0 });
        
        float worstDistSq = float.MaxValue;

        while (_searchStack.Count > 0)
        {
            var job = _searchStack.Pop();
            int nodeIdx = job.NodeIndex;
            if (nodeIdx == -1) continue;

            // Pruning
            if (best.Count == k && job.MinDistSq > worstDistSq) continue;

            // Leaf check
            if (_nodes[nodeIdx].UnitIndex != -1)
            {
                float dSq = (_positions[_nodes[nodeIdx].UnitIndex] - target).sqrMagnitude;
                AddCandidateUnsorted(_nodes[nodeIdx].UnitIndex, dSq, k, best, ref worstDistSq);
                continue;
            }

            // Internal node: check children
            int left = _nodes[nodeIdx].Left;
            int right = _nodes[nodeIdx].Right;

            float distLeft = DistToAABB(target, left);
            float distRight = DistToAABB(target, right);

            // Visit closest first
            if (distLeft < distRight)
            {
                if (right != -1) _searchStack.Push(new SearchJob { NodeIndex = right, MinDistSq = distRight });
                if (left != -1) _searchStack.Push(new SearchJob { NodeIndex = left, MinDistSq = distLeft });
            }
            else
            {
                if (left != -1) _searchStack.Push(new SearchJob { NodeIndex = left, MinDistSq = distLeft });
                if (right != -1) _searchStack.Push(new SearchJob { NodeIndex = right, MinDistSq = distRight });
            }
        }
    }
    
    private void AddCandidateUnsorted(int index, float distSq, int k, List<(int index, float distSq)> best, ref float worstDistSq)
    {
        if (best.Count < k)
        {
            best.Add((index, distSq));
            if (distSq > worstDistSq || worstDistSq == float.MaxValue) worstDistSq = distSq;
            if (best.Count == k)
            {
                worstDistSq = 0;
                for (int i = 0; i < best.Count; i++)
                    if (best[i].distSq > worstDistSq) worstDistSq = best[i].distSq;
            }
        }
        else if (distSq < worstDistSq)
        {
            int worstIdx = 0;
            for (int i = 1; i < best.Count; i++)
                if (best[i].distSq > best[worstIdx].distSq) worstIdx = i;
            best[worstIdx] = (index, distSq);
            worstDistSq = 0;
            for (int i = 0; i < best.Count; i++)
                if (best[i].distSq > worstDistSq) worstDistSq = best[i].distSq;
        }
    }

    public void QueryKNearestSorted(Vector2 position, int k, float radius, List<int> results)
    {
        results.Clear();
        _candidateCache.Clear();
        if (_candidateCache.Capacity < k + 1) _candidateCache.Capacity = k + 1;

        _searchStack.Clear();
        _searchStack.Push(new SearchJob { NodeIndex = _rootIndex, MinDistSq = 0 });
        
        float radiusSq = radius * radius;

        while (_searchStack.Count > 0)
        {
            var job = _searchStack.Pop();
            int nodeIdx = job.NodeIndex;
            if (nodeIdx == -1) continue;

            // Pruning
            float cutoffSq = radiusSq;
            if (_candidateCache.Count == k)
            {
                if (_candidateCache[_candidateCache.Count - 1].distSq < cutoffSq)
                {
                    cutoffSq = _candidateCache[_candidateCache.Count - 1].distSq;
                }
            }
            
            if (job.MinDistSq > cutoffSq) continue;

            // Leaf check
            if (_nodes[nodeIdx].UnitIndex != -1)
            {
                float dSq = (_positions[_nodes[nodeIdx].UnitIndex] - position).sqrMagnitude;
                if (dSq <= radiusSq)
                {
                    AddCandidate(_nodes[nodeIdx].UnitIndex, dSq, k, _candidateCache);
                }
                continue;
            }

            // Internal node: check children
            int left = _nodes[nodeIdx].Left;
            int right = _nodes[nodeIdx].Right;

            float distLeft = DistToAABB(position, left);
            float distRight = DistToAABB(position, right);

            // Visit closest first
            if (distLeft < distRight)
            {
                if (right != -1) _searchStack.Push(new SearchJob { NodeIndex = right, MinDistSq = distRight });
                if (left != -1) _searchStack.Push(new SearchJob { NodeIndex = left, MinDistSq = distLeft });
            }
            else
            {
                if (left != -1) _searchStack.Push(new SearchJob { NodeIndex = left, MinDistSq = distLeft });
                if (right != -1) _searchStack.Push(new SearchJob { NodeIndex = right, MinDistSq = distRight });
            }
        }

        for (int i = 0; i < _candidateCache.Count; i++) results.Add(_candidateCache[i].index);
    }

    private float DistToAABB(Vector2 p, int nodeIdx)
    {
        if (nodeIdx == -1) return float.MaxValue;
        float dx = Mathf.Max(_nodes[nodeIdx].MinX - p.x, 0, p.x - _nodes[nodeIdx].MaxX);
        float dy = Mathf.Max(_nodes[nodeIdx].MinY - p.y, 0, p.y - _nodes[nodeIdx].MaxY);
        return dx * dx + dy * dy;
    }

    private void AddCandidate(int index, float distSq, int k, List<(int index, float distSq)> best)
    {
        int insertPos = 0;
        while (insertPos < best.Count && best[insertPos].distSq < distSq) insertPos++;

        if (insertPos < k)
        {
            best.Insert(insertPos, (index, distSq));
            if (best.Count > k) best.RemoveAt(best.Count - 1);
        }
    }

    public void QueryRadius(Vector2 position, float radius, List<int> results)
    {
        results.Clear();
        float rSq = radius * radius;
        
        _searchStack.Clear();
        _searchStack.Push(new SearchJob { NodeIndex = _rootIndex, MinDistSq = 0 });

        while (_searchStack.Count > 0)
        {
            var job = _searchStack.Pop();
            int nodeIdx = job.NodeIndex;
            if (nodeIdx == -1) continue;

            if (job.MinDistSq > rSq) continue;

            if (_nodes[nodeIdx].UnitIndex != -1)
            {
                if ((_positions[_nodes[nodeIdx].UnitIndex] - position).sqrMagnitude <= rSq)
                {
                    results.Add(_nodes[nodeIdx].UnitIndex);
                }
                continue;
            }

            int left = _nodes[nodeIdx].Left;
            int right = _nodes[nodeIdx].Right;

            float distLeft = DistToAABB(position, left);
            float distRight = DistToAABB(position, right);

            if (right != -1 && distRight <= rSq) _searchStack.Push(new SearchJob { NodeIndex = right, MinDistSq = distRight });
            if (left != -1 && distLeft <= rSq) _searchStack.Push(new SearchJob { NodeIndex = left, MinDistSq = distLeft });
        }
    }
}
