using System.Collections.Generic;
using UnityEngine;
using Cysharp.Threading.Tasks;

public class QuadTreeIndex : ISpatialIndex
{
    private struct Node
    {
        public int Child0; // TopLeft
        public int Child1; // TopRight
        public int Child2; // BottomLeft
        public int Child3; // BottomRight
        public int FirstUnit; // Index into _linkedUnits
        public int Count;
        public float X, Y, W, H; // Bounds
    }

    private Node[] _nodes;
    private int _nodeCount;
    private int _rootIndex;
    
    // Linked list for units in leaves
    private int[] _linkedUnits; // Points to next unit in the same leaf
    private int[] _unitIndices; // The actual unit index
    private int _linkCount;

    private List<Vector2> _positions;
    private Rect _bounds;
    private int _maxDepth = 8;
    private int _bucketSize = 16;

    // Cache for query
    private List<(int index, float distSq)> _candidateCache = new List<(int index, float distSq)>();
    private Stack<int> _searchStack = new Stack<int>(64);

    public QuadTreeIndex(int capacity, Rect bounds)
    {
        // QuadTree nodes count is hard to predict exactly, but for N units, N nodes is safe upper bound if bucket > 1
        _nodes = new Node[capacity * 2]; 
        _linkedUnits = new int[capacity];
        _unitIndices = new int[capacity];
        _bounds = bounds;
    }

    public UniTask BuildAsync(List<Vector2> positions)
    {
        _positions = positions;
        int count = positions.Count;

        if (_nodes.Length < count * 2) _nodes = new Node[count * 2];
        if (_linkedUnits.Length < count) 
        {
            _linkedUnits = new int[count];
            _unitIndices = new int[count];
        }

        _nodeCount = 0;
        _linkCount = 0;

        // Create Root
        _rootIndex = _nodeCount++;
        _nodes[_rootIndex] = new Node 
        { 
            X = _bounds.x, Y = _bounds.y, W = _bounds.width, H = _bounds.height,
            Child0 = -1, Child1 = -1, Child2 = -1, Child3 = -1,
            FirstUnit = -1, Count = 0 
        };

        // Insert all units
        for (int i = 0; i < count; i++)
        {
            Insert(i, positions[i]);
        }

        return UniTask.CompletedTask;
    }

    private void Insert(int unitIdx, Vector2 pos)
    {
        int current = _rootIndex;
        int depth = 0;

        while (true)
        {
            // If leaf
            if (_nodes[current].Child0 == -1)
            {
                // If bucket not full or max depth reached, add here
                if (_nodes[current].Count < _bucketSize || depth >= _maxDepth)
                {
                    // Add to linked list
                    _unitIndices[unitIdx] = unitIdx; // Store the unit index
                    _linkedUnits[unitIdx] = _nodes[current].FirstUnit;
                    _nodes[current].FirstUnit = unitIdx;
                    _nodes[current].Count++;
                    return;
                }
                else
                {
                    // Split
                    Split(current);
                    // Continue loop to insert into children
                }
            }

            // Find child
            float midX = _nodes[current].X + _nodes[current].W * 0.5f;
            float midY = _nodes[current].Y + _nodes[current].H * 0.5f;
            
            int childIdx = 0;
            if (pos.x > midX) childIdx += 1;
            if (pos.y > midY) childIdx += 2; // 0:TL, 1:TR, 2:BL, 3:BR (Y-up? No, standard 2D: usually 0=BL. Let's stick to logic)
            // Let's define:
            // 0: Left-Bottom, 1: Right-Bottom, 2: Left-Top, 3: Right-Top
            // Wait, standard QuadTree usually:
            // 0: NW, 1: NE, 2: SW, 3: SE
            // Let's use:
            // 0: minX, minY (BL)
            // 1: maxX, minY (BR)
            // 2: minX, maxY (TL)
            // 3: maxX, maxY (TR)
            
            // Logic above:
            // if x > mid -> 1
            // if y > mid -> 2
            // 0: <midX, <midY (BL)
            // 1: >midX, <midY (BR)
            // 2: <midX, >midY (TL)
            // 3: >midX, >midY (TR)
            
            int nextNode = -1;
            switch (childIdx)
            {
                case 0: nextNode = _nodes[current].Child0; break;
                case 1: nextNode = _nodes[current].Child1; break;
                case 2: nextNode = _nodes[current].Child2; break;
                case 3: nextNode = _nodes[current].Child3; break;
            }
            
            current = nextNode;
            depth++;
        }
    }

    private void Split(int nodeIdx)
    {
        float x = _nodes[nodeIdx].X;
        float y = _nodes[nodeIdx].Y;
        float w = _nodes[nodeIdx].W * 0.5f;
        float h = _nodes[nodeIdx].H * 0.5f;

        int c0 = _nodeCount++;
        int c1 = _nodeCount++;
        int c2 = _nodeCount++;
        int c3 = _nodeCount++;

        _nodes[c0] = new Node { X = x, Y = y, W = w, H = h, Child0=-1, Child1=-1, Child2=-1, Child3=-1, FirstUnit=-1 };
        _nodes[c1] = new Node { X = x+w, Y = y, W = w, H = h, Child0=-1, Child1=-1, Child2=-1, Child3=-1, FirstUnit=-1 };
        _nodes[c2] = new Node { X = x, Y = y+h, W = w, H = h, Child0=-1, Child1=-1, Child2=-1, Child3=-1, FirstUnit=-1 };
        _nodes[c3] = new Node { X = x+w, Y = y+h, W = w, H = h, Child0=-1, Child1=-1, Child2=-1, Child3=-1, FirstUnit=-1 };

        _nodes[nodeIdx].Child0 = c0;
        _nodes[nodeIdx].Child1 = c1;
        _nodes[nodeIdx].Child2 = c2;
        _nodes[nodeIdx].Child3 = c3;

        // Redistribute units
        int currUnit = _nodes[nodeIdx].FirstUnit;
        while (currUnit != -1)
        {
            int nextUnit = _linkedUnits[currUnit];
            Vector2 pos = _positions[currUnit];
            
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

            // Add to child list
            _linkedUnits[currUnit] = _nodes[targetNode].FirstUnit;
            _nodes[targetNode].FirstUnit = currUnit;
            _nodes[targetNode].Count++;

            currUnit = nextUnit;
        }

        _nodes[nodeIdx].FirstUnit = -1;
        _nodes[nodeIdx].Count = 0;
    }

    public void QueryKNearest(Vector2 position, int k, List<int> results)
    {
        results.Clear();
        _candidateCache.Clear();
        if (_candidateCache.Capacity < k + 1) _candidateCache.Capacity = k + 1;

        _searchStack.Clear();
        _searchStack.Push(_rootIndex);

        while (_searchStack.Count > 0)
        {
            int nodeIdx = _searchStack.Pop();
            
            // Pruning: Dist to rect
            float distToNode = DistToRect(position, _nodes[nodeIdx]);
            if (_candidateCache.Count == k)
            {
                if (distToNode * distToNode >= _candidateCache[_candidateCache.Count - 1].distSq) continue;
            }

            // If leaf
            if (_nodes[nodeIdx].Child0 == -1)
            {
                int curr = _nodes[nodeIdx].FirstUnit;
                while (curr != -1)
                {
                    float dSq = (_positions[curr] - position).sqrMagnitude;
                    AddCandidate(curr, dSq, k, _candidateCache);
                    curr = _linkedUnits[curr];
                }
                continue;
            }

            // Internal: Push children
            // Order by distance? Simple optimization: push all
            // Better: Sort children by distance
            
            // For simplicity, just push all. 
            // To optimize, we should push furthest first so closest is popped first.
            
            // Calculate distances
            float d0 = DistToRect(position, _nodes[_nodes[nodeIdx].Child0]);
            float d1 = DistToRect(position, _nodes[_nodes[nodeIdx].Child1]);
            float d2 = DistToRect(position, _nodes[_nodes[nodeIdx].Child2]);
            float d3 = DistToRect(position, _nodes[_nodes[nodeIdx].Child3]);
            
            // Sort indices (0,1,2,3) by distance descending
            // Simple manual sort or just push
            _searchStack.Push(_nodes[nodeIdx].Child0);
            _searchStack.Push(_nodes[nodeIdx].Child1);
            _searchStack.Push(_nodes[nodeIdx].Child2);
            _searchStack.Push(_nodes[nodeIdx].Child3);
        }

        for (int i = 0; i < _candidateCache.Count; i++) results.Add(_candidateCache[i].index);
    }

    private float DistToRect(Vector2 p, Node n)
    {
        float dx = Mathf.Max(n.X - p.x, 0, p.x - (n.X + n.W));
        float dy = Mathf.Max(n.Y - p.y, 0, p.y - (n.Y + n.H));
        return Mathf.Sqrt(dx * dx + dy * dy);
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
        _searchStack.Push(_rootIndex);

        while (_searchStack.Count > 0)
        {
            int nodeIdx = _searchStack.Pop();
            
            float dist = DistToRect(position, _nodes[nodeIdx]);
            if (dist > radius) continue;

            if (_nodes[nodeIdx].Child0 == -1)
            {
                int curr = _nodes[nodeIdx].FirstUnit;
                while (curr != -1)
                {
                    if ((_positions[curr] - position).sqrMagnitude <= rSq)
                    {
                        results.Add(curr);
                    }
                    curr = _linkedUnits[curr];
                }
                continue;
            }

            _searchStack.Push(_nodes[nodeIdx].Child0);
            _searchStack.Push(_nodes[nodeIdx].Child1);
            _searchStack.Push(_nodes[nodeIdx].Child2);
            _searchStack.Push(_nodes[nodeIdx].Child3);
        }
    }
}
