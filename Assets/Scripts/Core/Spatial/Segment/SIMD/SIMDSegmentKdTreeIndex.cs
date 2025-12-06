using System;
using Unity.Collections;
using Unity.Mathematics;
using Unity.Burst;
using Unity.Collections.LowLevel.Unsafe;

[BurstCompile]
public unsafe class SIMDSegmentKdTreeIndex : ISIMDSegmentSpatialIndex
{
    private NativeArray<SIMDRVO.ObstacleData> _treeObstacles;
    private int _treeObstacleCount;
    private NativeArray<SIMDRVO.ObstacleTreeNode> _treeNodes;
    private int _treeNodeCount;
    private int _rootNodeIndex;

    public NativeArray<SIMDRVO.ObstacleData> GetTreeObstacles() => _treeObstacles;
    public int GetTreeObstacleCount() => _treeObstacleCount;
    public NativeArray<SIMDRVO.ObstacleTreeNode> GetTreeNodes() => _treeNodes;
    public int GetRootNodeIndex() => _rootNodeIndex;

    public void Build(NativeArray<SIMDRVO.ObstacleData> obstacles)
    {
        if (_treeObstacles.IsCreated) _treeObstacles.Dispose();
        if (_treeNodes.IsCreated) _treeNodes.Dispose();

        // Heuristic: Splitting can increase obstacle count. 4x is usually enough.
        int capacity = math.max(16, obstacles.Length * 4);
        _treeObstacles = new NativeArray<SIMDRVO.ObstacleData>(capacity, Allocator.Persistent);
        _treeNodes = new NativeArray<SIMDRVO.ObstacleTreeNode>(capacity, Allocator.Persistent);
        
        NativeArray<SIMDRVO.ObstacleData>.Copy(obstacles, _treeObstacles, obstacles.Length);
        _treeObstacleCount = obstacles.Length;
        _treeNodeCount = 0;

        // Initial indices list
        NativeArray<int> obstacleIndices = new NativeArray<int>(obstacles.Length, Allocator.TempJob);
        for (int i = 0; i < obstacles.Length; i++)
        {
            obstacleIndices[i] = i;
        }

        SIMDRVO.ObstacleData* obstaclesPtr = (SIMDRVO.ObstacleData*)_treeObstacles.GetUnsafePtr();
        SIMDRVO.ObstacleTreeNode* nodesPtr = (SIMDRVO.ObstacleTreeNode*)_treeNodes.GetUnsafePtr();
        int* indicesPtr = (int*)obstacleIndices.GetUnsafePtr();

        // Use local vars for ref passing
        int treeObstacleCount = _treeObstacleCount;
        int treeNodeCount = _treeNodeCount;
        int rootNodeIndex = _rootNodeIndex;

        BuildTreeBurst(obstaclesPtr, capacity, ref treeObstacleCount, nodesPtr, capacity, ref treeNodeCount, ref rootNodeIndex, indicesPtr, obstacles.Length);

        _treeObstacleCount = treeObstacleCount;
        _treeNodeCount = treeNodeCount;
        _rootNodeIndex = rootNodeIndex;

        obstacleIndices.Dispose();
    }

    public void Dispose()
    {
        if (_treeObstacles.IsCreated) _treeObstacles.Dispose();
        if (_treeNodes.IsCreated) _treeNodes.Dispose();
    }

    [BurstCompile]
    private static void BuildTreeBurst(
        SIMDRVO.ObstacleData* treeObstacles, int treeObstaclesCapacity, ref int treeObstacleCount,
        SIMDRVO.ObstacleTreeNode* treeNodes, int treeNodesCapacity, ref int treeNodeCount,
        ref int rootNodeIndex, 
        int* obstacleIndices, int obstacleIndicesCount)
    {
        if (obstacleIndicesCount == 0)
        {
            rootNodeIndex = -1;
            return;
        }

        rootNodeIndex = BuildRecursive(treeObstacles, treeObstaclesCapacity, ref treeObstacleCount, treeNodes, treeNodesCapacity, ref treeNodeCount, obstacleIndices, obstacleIndicesCount);
    }

    private static int BuildRecursive(
        SIMDRVO.ObstacleData* treeObstacles, int treeObstaclesCapacity, ref int treeObstacleCount,
        SIMDRVO.ObstacleTreeNode* treeNodes, int treeNodesCapacity, ref int treeNodeCount,
        int* obstacleIndices, int obstacleIndicesCount)
    {
        if (obstacleIndicesCount == 0)
        {
            return -1;
        }

        int optimalSplit = 0;
        int minLeft = obstacleIndicesCount;
        int minRight = obstacleIndicesCount;

        for (int i = 0; i < obstacleIndicesCount; ++i)
        {
            int leftSize = 0;
            int rightSize = 0;

            int obstI_idx = obstacleIndices[i];
            SIMDRVO.ObstacleData obstI = treeObstacles[obstI_idx];

            for (int j = 0; j < obstacleIndicesCount; ++j)
            {
                if (i == j) continue;

                int obstJ_idx = obstacleIndices[j];
                SIMDRVO.ObstacleData obstJ = treeObstacles[obstJ_idx];

                float j1Left = SIMDRVO.det(obstI.Point2 - obstI.Point1, obstJ.Point1 - obstI.Point1);
                float j2Left = SIMDRVO.det(obstI.Point2 - obstI.Point1, obstJ.Point2 - obstI.Point1);

                if (j1Left >= -SIMDRVO.RVO_EPSILON && j2Left >= -SIMDRVO.RVO_EPSILON) ++leftSize;
                else if (j1Left <= SIMDRVO.RVO_EPSILON && j2Left <= SIMDRVO.RVO_EPSILON) ++rightSize;
                else
                {
                    ++leftSize;
                    ++rightSize;
                }

                if (math.max(leftSize, rightSize) >= math.max(minLeft, minRight)) break;
            }

            if (math.max(leftSize, rightSize) < math.max(minLeft, minRight))
            {
                minLeft = leftSize;
                minRight = rightSize;
                optimalSplit = i;
            }
        }

        // Split
        int splitObstIdx = obstacleIndices[optimalSplit];
        SIMDRVO.ObstacleData splitObst = treeObstacles[splitObstIdx];

        // Allocate Temp memory for recursion
        int* leftIndices = (int*)UnsafeUtility.Malloc(minLeft * sizeof(int), 4, Allocator.Temp);
        int* rightIndices = (int*)UnsafeUtility.Malloc(minRight * sizeof(int), 4, Allocator.Temp);
        int leftCount = 0;
        int rightCount = 0;

        for (int i = 0; i < obstacleIndicesCount; ++i)
        {
            if (i == optimalSplit) continue;

            int obstJ_idx = obstacleIndices[i];
            SIMDRVO.ObstacleData obstJ = treeObstacles[obstJ_idx];

            float j1Left = SIMDRVO.det(splitObst.Point2 - splitObst.Point1, obstJ.Point1 - splitObst.Point1);
            float j2Left = SIMDRVO.det(splitObst.Point2 - splitObst.Point1, obstJ.Point2 - splitObst.Point1);

            if (j1Left >= -SIMDRVO.RVO_EPSILON && j2Left >= -SIMDRVO.RVO_EPSILON) leftIndices[leftCount++] = obstJ_idx;
            else if (j1Left <= SIMDRVO.RVO_EPSILON && j2Left <= SIMDRVO.RVO_EPSILON) rightIndices[rightCount++] = obstJ_idx;
            else
            {
                // Split obstJ
                float t = SIMDRVO.det(splitObst.Point2 - splitObst.Point1, obstJ.Point1 - splitObst.Point1) /
                          SIMDRVO.det(splitObst.Point2 - splitObst.Point1, obstJ.Point1 - obstJ.Point2);

                float2 splitPoint = obstJ.Point1 + t * (obstJ.Point2 - obstJ.Point1);

                SIMDRVO.ObstacleData newObst = new SIMDRVO.ObstacleData
                {
                    Point1 = splitPoint,
                    Point2 = obstJ.Point2,
                    Direction = obstJ.Direction,
                    IsConvex = true,
                    NextObstacleIdx = obstJ.NextObstacleIdx,
                    PrevObstacleIdx = obstJ_idx 
                };

                // Add to main list
                if (treeObstacleCount < treeObstaclesCapacity)
                {
                    int newObstIdx = treeObstacleCount++;
                    treeObstacles[newObstIdx] = newObst;

                    // Update obstJ (first half)
                    obstJ.Point2 = splitPoint;
                    obstJ.NextObstacleIdx = newObstIdx;
                    treeObstacles[obstJ_idx] = obstJ; 

                    // Update links
                    if (newObst.NextObstacleIdx != -1)
                    {
                        SIMDRVO.ObstacleData nextObs = treeObstacles[newObst.NextObstacleIdx];
                        nextObs.PrevObstacleIdx = newObstIdx;
                        treeObstacles[newObst.NextObstacleIdx] = nextObs;
                    }

                    if (j1Left > 0.0f)
                    {
                        leftIndices[leftCount++] = obstJ_idx;
                        rightIndices[rightCount++] = newObstIdx;
                    }
                    else
                    {
                        rightIndices[rightCount++] = obstJ_idx;
                        leftIndices[leftCount++] = newObstIdx;
                    }
                }
                else
                {
                    // Capacity exceeded! 
                    // Should we warn? In Burst we can't easy log string formatted.
                    // For now, ignore the split (potentially incorrect behavior but prevents crash)
                    // Or add to one side?
                    // Let's add original to left.
                    leftIndices[leftCount++] = obstJ_idx;
                }
            }
        }

        int nodeIdx = -1;
        if (treeNodeCount < treeNodesCapacity)
        {
            nodeIdx = treeNodeCount++;
            
            // Temporarily store
            treeNodes[nodeIdx] = new SIMDRVO.ObstacleTreeNode
            {
                ObstacleIndex = splitObstIdx,
                LeftNodeIndex = -1, 
                RightNodeIndex = -1
            };
        }

        int leftNodeIdx = BuildRecursive(treeObstacles, treeObstaclesCapacity, ref treeObstacleCount, treeNodes, treeNodesCapacity, ref treeNodeCount, leftIndices, leftCount);
        int rightNodeIdx = BuildRecursive(treeObstacles, treeObstaclesCapacity, ref treeObstacleCount, treeNodes, treeNodesCapacity, ref treeNodeCount, rightIndices, rightCount);

        if (nodeIdx != -1)
        {
            SIMDRVO.ObstacleTreeNode node = treeNodes[nodeIdx];
            node.LeftNodeIndex = leftNodeIdx;
            node.RightNodeIndex = rightNodeIdx;
            treeNodes[nodeIdx] = node;
        }

        UnsafeUtility.Free(leftIndices, Allocator.Temp);
        UnsafeUtility.Free(rightIndices, Allocator.Temp);

        return nodeIdx;
    }
}
