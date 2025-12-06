using System.Collections.Generic;
using UnityEngine;
using Cysharp.Threading.Tasks;
using System.Threading.Tasks;

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

    // Threshold for switching to parallel build
    private const int ParallelThreshold = 4096;
    // Max depth for parallelism to avoid over-threading (2^4 = 16 tasks)
    private const int MaxParallelDepth = 4;

    public KDTreeIndex(int capacity)
    {
        _nodes = new Node[capacity];
        _indicesBuffer = new int[capacity];
    }

    public UniTask BuildAsync(List<Vector2> positions)
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

        // Run synchronously (caller handles threading)
        _rootIndex = BuildRecursive(0, count, 0);
        
        return UniTask.CompletedTask;
    }

    // Iterative Build
    private struct BuildJob
    {
        public int Start;
        public int Length;
        public int Depth;
        public int ParentIndex; // Index of the parent node in _nodes array
        public bool IsLeft;     // Is this the left child of the parent?
    }

    private int BuildRecursive(int start, int length, int depth)
    {
        if (length <= 0) return -1;

        int axis = depth % 2;
        int mid = length / 2;
        int medianIndex = start + mid;

        // Partition
        QuickSelect(start, length, mid, axis);

        int nodeIndex = _indicesBuffer[medianIndex];
        Vector2 pos = _positions[nodeIndex];

        _nodes[nodeIndex].Axis = axis;
        _nodes[nodeIndex].UnitIndex = nodeIndex;
        _nodes[nodeIndex].SplitValue = (axis == 0) ? pos.x : pos.y;

        // Parallel Fork-Join
        if (depth < MaxParallelDepth && length >= ParallelThreshold)
        {
            int leftResult = -1;
            int rightResult = -1;

            Parallel.Invoke(
                () => leftResult = BuildRecursive(start, mid, depth + 1),
                () => rightResult = BuildRecursive(start + mid + 1, length - mid - 1, depth + 1)
            );

            _nodes[nodeIndex].Left = leftResult;
            _nodes[nodeIndex].Right = rightResult;
        }
        else
        {
            BuildIterative(start, mid, depth + 1, nodeIndex, true);
            BuildIterative(start + mid + 1, length - mid - 1, depth + 1, nodeIndex, false);
        }

        return nodeIndex;
    }

    private void BuildIterative(int rootStart, int rootLength, int rootDepth, int rootParent, bool rootIsLeft)
    {
        var stack = new Stack<BuildJob>();
        stack.Push(new BuildJob 
        { 
            Start = rootStart, 
            Length = rootLength, 
            Depth = rootDepth, 
            ParentIndex = rootParent, 
            IsLeft = rootIsLeft 
        });

        while (stack.Count > 0)
        {
            var job = stack.Pop();
            
            if (job.Length <= 0)
            {
                if (job.IsLeft) _nodes[job.ParentIndex].Left = -1;
                else _nodes[job.ParentIndex].Right = -1;
                continue;
            }

            int axis = job.Depth % 2;
            int mid = job.Length / 2;
            int medianIndex = job.Start + mid;

            QuickSelect(job.Start, job.Length, mid, axis);

            int nodeIndex = _indicesBuffer[medianIndex];
            Vector2 pos = _positions[nodeIndex];

            _nodes[nodeIndex].Axis = axis;
            _nodes[nodeIndex].UnitIndex = nodeIndex;
            _nodes[nodeIndex].SplitValue = (axis == 0) ? pos.x : pos.y;

            if (job.IsLeft) _nodes[job.ParentIndex].Left = nodeIndex;
            else _nodes[job.ParentIndex].Right = nodeIndex;

            stack.Push(new BuildJob 
            { 
                Start = job.Start + mid + 1, 
                Length = job.Length - mid - 1, 
                Depth = job.Depth + 1, 
                ParentIndex = nodeIndex, 
                IsLeft = false 
            });

            stack.Push(new BuildJob 
            { 
                Start = job.Start, 
                Length = mid, 
                Depth = job.Depth + 1, 
                ParentIndex = nodeIndex, 
                IsLeft = true 
            });
        }
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

    private List<(int index, float distSq)> _candidateCache = new List<(int index, float distSq)>();
    
    private struct SearchJob
    {
        public int NodeIndex;
        public float MinDistSq; 
    }
    private Stack<SearchJob> _searchStack = new Stack<SearchJob>(64);

    public void QueryKNearest(Vector2 position, int k, List<int> results)
    {
        results.Clear();
        
        _candidateCache.Clear();
        if (_candidateCache.Capacity < k + 1)
        {
            _candidateCache.Capacity = k + 1;
        }
        
        // Use unsorted search - collect K nearest without maintaining sort order
        SearchKNNUnsorted(_rootIndex, position, k, _candidateCache);
        
        for (int i = 0; i < _candidateCache.Count; i++)
        {
            results.Add(_candidateCache[i].index);
        }
    }
    
    private void SearchKNNUnsorted(int rootIndex, Vector2 target, int k, List<(int index, float distSq)> best)
    {
        if (rootIndex == -1) return;

        _searchStack.Clear();
        _searchStack.Push(new SearchJob { NodeIndex = rootIndex, MinDistSq = 0f });
        
        float worstDistSq = float.MaxValue;

        while (_searchStack.Count > 0)
        {
            var job = _searchStack.Pop();
            int nodeIndex = job.NodeIndex;
            
            if (nodeIndex == -1) continue;

            // Pruning: if this branch can't improve our worst candidate, skip
            if (best.Count == k && job.MinDistSq > worstDistSq)
            {
                continue;
            }

            int unitIdx = _nodes[nodeIndex].UnitIndex;
            float distSq = (_positions[unitIdx] - target).sqrMagnitude;

            // Add candidate using max-heap pattern (unsorted)
            if (best.Count < k)
            {
                best.Add((unitIdx, distSq));
                // Update worst distance
                if (distSq > worstDistSq || worstDistSq == float.MaxValue)
                {
                    worstDistSq = distSq;
                }
                // Find new worst if we just filled the list
                if (best.Count == k)
                {
                    worstDistSq = 0;
                    for (int i = 0; i < best.Count; i++)
                    {
                        if (best[i].distSq > worstDistSq) worstDistSq = best[i].distSq;
                    }
                }
            }
            else if (distSq < worstDistSq)
            {
                // Replace the worst candidate
                int worstIdx = 0;
                for (int i = 1; i < best.Count; i++)
                {
                    if (best[i].distSq > best[worstIdx].distSq) worstIdx = i;
                }
                best[worstIdx] = (unitIdx, distSq);
                // Find new worst
                worstDistSq = 0;
                for (int i = 0; i < best.Count; i++)
                {
                    if (best[i].distSq > worstDistSq) worstDistSq = best[i].distSq;
                }
            }

            // Decide which side to search first
            int axis = _nodes[nodeIndex].Axis;
            float diff = (axis == 0) ? (target.x - _positions[unitIdx].x) : (target.y - _positions[unitIdx].y);
            
            int first = (diff < 0) ? _nodes[nodeIndex].Left : _nodes[nodeIndex].Right;
            int second = (diff < 0) ? _nodes[nodeIndex].Right : _nodes[nodeIndex].Left;
            
            float diffSq = diff * diff;

            if (second != -1)
            {
                _searchStack.Push(new SearchJob { NodeIndex = second, MinDistSq = diffSq });
            }

            if (first != -1)
            {
                _searchStack.Push(new SearchJob { NodeIndex = first, MinDistSq = 0f });
            }
        }
    }

    public void QueryKNearestSorted(Vector2 position, int k, float radius, List<int> results)
    {
        results.Clear();
        
        _candidateCache.Clear();
        if (_candidateCache.Capacity < k + 1)
        {
            _candidateCache.Capacity = k + 1;
        }
        
        SearchKNNIterative(_rootIndex, position, k, radius, _candidateCache);
        
        for (int i = 0; i < _candidateCache.Count; i++)
        {
            results.Add(_candidateCache[i].index);
        }
    }

    private void SearchKNNIterative(int rootIndex, Vector2 target, int k, float radius, List<(int index, float distSq)> best)
    {
        if (rootIndex == -1) return;

        float radiusSq = radius * radius;

        _searchStack.Clear();
        _searchStack.Push(new SearchJob { NodeIndex = rootIndex, MinDistSq = 0f });

        while (_searchStack.Count > 0)
        {
            var job = _searchStack.Pop();
            int nodeIndex = job.NodeIndex;
            
            if (nodeIndex == -1) continue;

            // Pruning
            float currentWorstDistSq = (best.Count == k) ? best[best.Count - 1].distSq : radiusSq;
            
            if (job.MinDistSq > currentWorstDistSq)
            {
                continue;
            }

            int unitIdx = _nodes[nodeIndex].UnitIndex;
            float distSq = (_positions[unitIdx] - target).sqrMagnitude;

            // Try to add to best
            if (distSq <= radiusSq)
            {
                AddCandidate(unitIdx, distSq, k, best);
            }

            // Decide which side to search first
            int axis = _nodes[nodeIndex].Axis;
            float diff = (axis == 0) ? (target.x - _positions[unitIdx].x) : (target.y - _positions[unitIdx].y);
            
            int first = (diff < 0) ? _nodes[nodeIndex].Left : _nodes[nodeIndex].Right;
            int second = (diff < 0) ? _nodes[nodeIndex].Right : _nodes[nodeIndex].Left;
            
            float diffSq = diff * diff;

            if (second != -1)
            {
                _searchStack.Push(new SearchJob { NodeIndex = second, MinDistSq = diffSq });
            }

            if (first != -1)
            {
                _searchStack.Push(new SearchJob { NodeIndex = first, MinDistSq = 0f });
            }
        }
    }

    private void AddCandidate(int index, float distSq, int k, List<(int index, float distSq)> best)
    {
        int insertPos = 0;
        // Linear scan (fast for small K)
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
        float radiusSq = radius * radius;
        
        _searchStack.Clear();
        _searchStack.Push(new SearchJob { NodeIndex = _rootIndex, MinDistSq = 0f });

        while (_searchStack.Count > 0)
        {
            var job = _searchStack.Pop();
            int nodeIndex = job.NodeIndex;
            
            if (nodeIndex == -1) continue;

            if (job.MinDistSq > radiusSq) continue;

            int unitIdx = _nodes[nodeIndex].UnitIndex;
            float distSq = (_positions[unitIdx] - position).sqrMagnitude;

            if (distSq <= radiusSq)
            {
                results.Add(unitIdx);
            }

            int axis = _nodes[nodeIndex].Axis;
            float diff = (axis == 0) ? (position.x - _positions[unitIdx].x) : (position.y - _positions[unitIdx].y);
            
            int first = (diff < 0) ? _nodes[nodeIndex].Left : _nodes[nodeIndex].Right;
            int second = (diff < 0) ? _nodes[nodeIndex].Right : _nodes[nodeIndex].Left;
            
            float diffSq = diff * diff;

            if (second != -1 && diffSq <= radiusSq)
            {
                _searchStack.Push(new SearchJob { NodeIndex = second, MinDistSq = diffSq });
            }

            if (first != -1)
            {
                _searchStack.Push(new SearchJob { NodeIndex = first, MinDistSq = 0f });
            }
        }
    }
}
