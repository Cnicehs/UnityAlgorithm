using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

[BurstCompile]
public struct SIMDAStarPathfinder
{
    private struct Node : System.IComparable<Node>
    {
        public int Index;
        public float F;

        public int CompareTo(Node other)
        {
            return F.CompareTo(other.F);
        }
    }

    // MinHeap implementation for NativeArray
    private struct MinHeap
    {
        private NativeArray<Node> _items;
        public int Count;

        public MinHeap(int capacity, Allocator allocator)
        {
            _items = new NativeArray<Node>(capacity, allocator);
            Count = 0;
        }

        public void Dispose()
        {
            if (_items.IsCreated) _items.Dispose();
        }

        public void Push(Node node)
        {
            if (Count >= _items.Length) return; // Should resize, but for now fixed size
            _items[Count] = node;
            BubbleUp(Count);
            Count++;
        }

        public Node Pop()
        {
            Node result = _items[0];
            Count--;
            _items[0] = _items[Count];
            if (Count > 0)
            {
                BubbleDown(0);
            }
            return result;
        }

        private void BubbleUp(int index)
        {
            while (index > 0)
            {
                int parentIndex = (index - 1) / 2;
                if (_items[index].F >= _items[parentIndex].F) break;

                Swap(index, parentIndex);
                index = parentIndex;
            }
        }

        private void BubbleDown(int index)
        {
            int lastIndex = Count - 1;
            while (true)
            {
                int leftChild = index * 2 + 1;
                if (leftChild > lastIndex) break;

                int rightChild = leftChild + 1;
                int smallest = leftChild;

                if (rightChild <= lastIndex && _items[rightChild].F < _items[leftChild].F)
                {
                    smallest = rightChild;
                }

                if (_items[index].F <= _items[smallest].F) break;

                Swap(index, smallest);
                index = smallest;
            }
        }

        private void Swap(int a, int b)
        {
            Node temp = _items[a];
            _items[a] = _items[b];
            _items[b] = temp;
        }
    }

    [BurstCompile]
    public static unsafe void FindPath(in int2 startGrid, in int2 endGrid, int width, int height, [ReadOnly] in NativeArray<bool> obstacles, float2* path, ref int pathLength, int maxPathLength, in float2 origin, float cellSize)
    {
        pathLength = 0;

        if (startGrid.x < 0 || startGrid.x >= width || startGrid.y < 0 || startGrid.y >= height ||
            endGrid.x < 0 || endGrid.x >= width || endGrid.y < 0 || endGrid.y >= height)
        {
            return;
        }

        int endIndex = endGrid.y * width + endGrid.x;
        if (obstacles[endIndex])
        {
            return;
        }

        int startIndex = startGrid.y * width + startGrid.x;

        // Data structures
        int mapSize = width * height;
        NativeArray<float> gScore = new NativeArray<float>(mapSize, Allocator.Temp);
        NativeArray<int> cameFrom = new NativeArray<int>(mapSize, Allocator.Temp);
        NativeArray<bool> inOpenSet = new NativeArray<bool>(mapSize, Allocator.Temp); // Optimization to check containment

        // Initialize
        for (int i = 0; i < mapSize; i++)
        {
            gScore[i] = float.MaxValue;
            cameFrom[i] = -1;
        }

        // Heap capacity: mapSize is safe upper bound
        MinHeap openSet = new MinHeap(mapSize, Allocator.Temp);

        gScore[startIndex] = 0;
        openSet.Push(new Node { Index = startIndex, F = Heuristic(startGrid, endGrid) });
        inOpenSet[startIndex] = true;

        // Neighbor offsets (8 directions)
        var offsets = stackalloc int2[]
        {
            new int2(0, 1), new int2(1, 0), new int2(0, -1), new int2(-1, 0),
            new int2(1, 1), new int2(1, -1), new int2(-1, -1), new int2(-1, 1)
        };

        while (openSet.Count > 0)
        {
            Node current = openSet.Pop();
            int currentIndex = current.Index;
            inOpenSet[currentIndex] = false;

            if (currentIndex == endIndex)
            {
                // Reconstruct path
                ReconstructPath(cameFrom, endIndex, width, origin, cellSize, path, ref pathLength, maxPathLength);
                break;
            }

            int2 currentPos = new int2(currentIndex % width, currentIndex / width);

            for (int i = 0; i < 8; i++)
            {
                int2 offset = offsets[i];
                int2 neighborPos = currentPos + offset;

                if (neighborPos.x < 0 || neighborPos.x >= width || neighborPos.y < 0 || neighborPos.y >= height) continue;

                int neighborIndex = neighborPos.y * width + neighborPos.x;
                if (obstacles[neighborIndex]) continue;

                // Corner cutting check
                if (offset.x != 0 && offset.y != 0)
                {
                    int idx1 = currentPos.y * width + (currentPos.x + offset.x);
                    int idx2 = (currentPos.y + offset.y) * width + currentPos.x;
                    if (obstacles[idx1] || obstacles[idx2]) continue;
                }

                float moveCost = (offset.x != 0 && offset.y != 0) ? 1.41421356f : 1.0f;
                float tentativeG = gScore[currentIndex] + moveCost;

                if (tentativeG < gScore[neighborIndex])
                {
                    cameFrom[neighborIndex] = currentIndex;
                    gScore[neighborIndex] = tentativeG;
                    float f = tentativeG + Heuristic(neighborPos, endGrid);

                    openSet.Push(new Node { Index = neighborIndex, F = f });
                }
            }
        }

        gScore.Dispose();
        cameFrom.Dispose();
        inOpenSet.Dispose();
        openSet.Dispose();
    }

    private static float Heuristic(in int2 a, in int2 b)
    {
        return math.distance(new float2(a.x, a.y), new float2(b.x, b.y));
    }

    private static unsafe void ReconstructPath(in NativeArray<int> cameFrom, int currentIndex, int width, in float2 origin, float cellSize, float2* path, ref int pathLength, int maxPathLength)
    {
        pathLength = 0;
        while (currentIndex != -1)
        {
            if (pathLength >= maxPathLength) break; // Safety check

            int x = currentIndex % width;
            int y = currentIndex / width;
            float2 worldPos = new float2(x * cellSize + cellSize * 0.5f, y * cellSize + cellSize * 0.5f) + origin;
            path[pathLength] = worldPos;
            pathLength++;

            currentIndex = cameFrom[currentIndex];
        }
        // Path is reversed (End -> Start). 
        // We can reverse it here.
        for (int i = 0; i < pathLength / 2; i++)
        {
            float2 temp = path[i];
            path[i] = path[pathLength - 1 - i];
            path[pathLength - 1 - i] = temp;
        }
    }
}
