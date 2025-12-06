using System;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;

/// <summary>
/// KD-Tree based spatial index for line segments (RVO obstacles).
/// Uses RVO2's segment-splitting algorithm for optimal partitioning.
/// </summary>
public class SegmentKdTreeIndex : ISegmentSpatialIndex
{
    private class ObstacleTreeNode
    {
        public RVOObstacle Obstacle;
        public ObstacleTreeNode Left;
        public ObstacleTreeNode Right;
    }

    private ObstacleTreeNode _root;
    private List<RVOObstacle> _treeObstacles; // Valid obstacles including splits

    public void Build(List<RVOObstacle> obstacles)
    {
        // 1. We need a working copy of obstacles because RVO KdTree algorithm SPLITS them.
        // We shouldn't modify the original source list from Simulator.
        // So we clone the list structure.
        // However, Next/Prev links are references.
        // If we split, we update links. This might mess up the original obstacles if they share references?
        // Yes. Standard RVO2 builds the tree ONCE and uses it.
        // If we rebuild every frame (dynamic?), splitting repeatedly is bad.
        // But obstacles are usually static.
        // In this implementation, we will assume Build is called rarely (initially).
        // We will create a local list of obstacles to manipulate.
        // But to avoid breaking original references, we might need deep clone?
        // That's too heavy.
        // Let's assume we can modify the list logic or work with a separate set.
        // Actually, RVO2-Unity Simulator holds the list.
        // If we use this Index, maybe we should let it manage the "Tree Obstacles".

        // For simplicity and robustness given the user's prompt to "reuse",
        // I will implement a simpler KdTree that DOES NOT split, or a BVH.
        // But user asked "How to store segments... compare nearest?".
        // RVO2 Splitting KdTree is the "Correct" way for RVO.
        // Let's implement it but be careful.
        // We will make a shallow copy list. If we split, we add to this list.
        // But we won't modify the original 'obstacles' list object, but we might touch the Next/Prev of items in it.
        // This is a side effect.
        // If RVOSimulator keeps using its list, and we update links, it might be okay?
        // Actually, if we split "Edge A" into "Edge A1" and "Edge A2", "Edge A" object is modified to be "A1", and "A2" is new.
        // This is destructive.

        // ALTERNATIVE: Use a BVH (Bounding Volume Hierarchy) for segments.
        // It does NOT split geometry.
        // It is safer for dynamic updates or external management.
        // RVO2 uses KdTree because it's efficient for 2D ray casting style queries.
        // Given constraints, I will implement a BVH for Segments.
        // It fits "Storage for different things" and "Compare nearest".

        // WAIT, the user explicitly mentioned "Like KdTree... reuse it".
        // I will stick to RVO2 KdTree logic but try to avoid destructive split if possible, OR accept it.
        // If I accept it, I must tell RVOSimulator to use the "Tree Obstacles" instead of its own list?
        // But `Query` returns neighbors.

        // Let's implement the RVO2 KdTree logic (splitting) because it provides the best culling for RVO.
        // I will create a method to clone the obstacles first so we don't destroy the original scene data.

        _treeObstacles = new List<RVOObstacle>();
        // Clone
        Dictionary<RVOObstacle, RVOObstacle> map = new Dictionary<RVOObstacle, RVOObstacle>();
        foreach (var obs in obstacles)
        {
            var clone = new RVOObstacle(obs.Point1, obs.Point2);
            clone.Direction = obs.Direction;
            clone.IsConvex = obs.IsConvex;
            clone.Id = obs.Id;
            map[obs] = clone;
            _treeObstacles.Add(clone);
        }
        // Fix links
        foreach (var obs in obstacles)
        {
            if (obs.NextObstacle != null && map.ContainsKey(obs.NextObstacle)) map[obs].NextObstacle = map[obs.NextObstacle];
            if (obs.PrevObstacle != null && map.ContainsKey(obs.PrevObstacle)) map[obs].PrevObstacle = map[obs.PrevObstacle];
        }

        _root = BuildRecursive(_treeObstacles);
    }

    private ObstacleTreeNode BuildRecursive(IList<RVOObstacle> obstacles)
    {
        if (obstacles.Count == 0) return null;

        ObstacleTreeNode node = new ObstacleTreeNode();

        int optimalSplit = 0;
        int minLeft = obstacles.Count;
        int minRight = obstacles.Count;

        // Find optimal split
        for (int i = 0; i < obstacles.Count; ++i)
        {
            int leftSize = 0;
            int rightSize = 0;

            RVOObstacle obstI = obstacles[i];

            for (int j = 0; j < obstacles.Count; ++j)
            {
                if (i == j) continue;

                RVOObstacle obstJ = obstacles[j];

                // Check if J is Left/Right of I
                float j1Left = RVOMath.det(obstI.Point2 - obstI.Point1, obstJ.Point1 - obstI.Point1);
                float j2Left = RVOMath.det(obstI.Point2 - obstI.Point1, obstJ.Point2 - obstI.Point1);

                if (j1Left >= -0.0001f && j2Left >= -0.0001f) ++leftSize;
                else if (j1Left <= 0.0001f && j2Left <= 0.0001f) ++rightSize;
                else
                {
                    ++leftSize;
                    ++rightSize;
                }

                if (Mathf.Max(leftSize, rightSize) >= Mathf.Max(minLeft, minRight)) break;
            }

            if (Mathf.Max(leftSize, rightSize) < Mathf.Max(minLeft, minRight))
            {
                minLeft = leftSize;
                minRight = rightSize;
                optimalSplit = i;
            }
        }

        // Split
        var leftObstacles = new List<RVOObstacle>(minLeft);
        var rightObstacles = new List<RVOObstacle>(minRight);

        RVOObstacle splitObst = obstacles[optimalSplit];

        for (int j = 0; j < obstacles.Count; ++j)
        {
            if (optimalSplit == j) continue;

            RVOObstacle obstJ = obstacles[j];

            float j1Left = RVOMath.det(splitObst.Point2 - splitObst.Point1, obstJ.Point1 - splitObst.Point1);
            float j2Left = RVOMath.det(splitObst.Point2 - splitObst.Point1, obstJ.Point2 - splitObst.Point1);

            if (j1Left >= -0.0001f && j2Left >= -0.0001f) leftObstacles.Add(obstJ);
            else if (j1Left <= 0.0001f && j2Left <= 0.0001f) rightObstacles.Add(obstJ);
            else
            {
                // Split obstJ
                float t = RVOMath.det(splitObst.Point2 - splitObst.Point1, obstJ.Point1 - splitObst.Point1) /
                          RVOMath.det(splitObst.Point2 - splitObst.Point1, obstJ.Point1 - obstJ.Point2);

                float2 splitPoint = obstJ.Point1 + t * (obstJ.Point2 - obstJ.Point1);

                RVOObstacle newObst = new RVOObstacle(splitPoint, obstJ.Point2);
                newObst.Direction = obstJ.Direction;
                newObst.IsConvex = true;
                newObst.NextObstacle = obstJ.NextObstacle;
                newObst.PrevObstacle = obstJ;

                if (obstJ.NextObstacle != null) obstJ.NextObstacle.PrevObstacle = newObst;

                obstJ.Point2 = splitPoint;
                obstJ.NextObstacle = newObst;

                if (j1Left > 0.0f)
                {
                    leftObstacles.Add(obstJ);
                    rightObstacles.Add(newObst);
                }
                else
                {
                    rightObstacles.Add(obstJ);
                    leftObstacles.Add(newObst);
                }
            }
        }

        node.Obstacle = splitObst;
        node.Left = BuildRecursive(leftObstacles);
        node.Right = BuildRecursive(rightObstacles);

        return node;
    }

    public void QueryNearest(Vector2 position, List<RVOObstacle> results)
    {
        if (_root == null) return;

        float2 pos = new float2(position.x, position.y);
        results.Clear();

        // RVO2 uses a Range, but here we want "sorted results"
        // And usually we iterate recursively with a rangeSq
        // We will collect all within a large range, then sort?
        // Or implement the recursive insertion sort?
        // RVO2-Unity Agent.insertObstacleNeighbor does insertion sort.
        // We can pass the list and let the query fill it.
        // But QueryRecursive needs a rangeSq to prune.
        // Let's use a dynamic rangeSq (start infinite).

        float rangeSq = float.MaxValue;
        QueryRecursive(pos, ref rangeSq, _root, results);

        // Sort results by distance (required for RVO implicit culling)
        results.Sort((a, b) =>
        {
            float distA = RVOMath.distSqPointLineSegment(a.Point1, a.Point2, pos);
            float distB = RVOMath.distSqPointLineSegment(b.Point1, b.Point2, pos);
            return distA.CompareTo(distB);
        });
    }

    private void QueryRecursive(float2 pos, ref float rangeSq, ObstacleTreeNode node, List<RVOObstacle> results)
    {
        if (node == null) return;

        RVOObstacle obstacle1 = node.Obstacle;
        RVOObstacle obstacle2 = obstacle1.NextObstacle; // Can be null if end of chain? But RVO2 structure assumes loop or valid next.
                                                        // Note: My RVOObstacle.Point2 IS the end point. RVO2 uses vertex nodes.
                                                        // In my struct, Obstacle IS the edge.
                                                        // RVO2 logic: "obstacle1" is the node. "obstacle2" is next.
                                                        // The line is obst1.point -> obst2.point.
                                                        // In my code: line is obstacle.Point1 -> obstacle.Point2.

        float2 p1 = obstacle1.Point1;
        float2 p2 = obstacle1.Point2;

        float agentLeftOfLine = RVOMath.det(p2 - p1, pos - p1);

        QueryRecursive(pos, ref rangeSq, agentLeftOfLine >= 0.0f ? node.Left : node.Right, results);

        float distSqLine = (agentLeftOfLine * agentLeftOfLine) / RVOMath.absSq(p2 - p1);

        if (distSqLine < rangeSq)
        {
            if (agentLeftOfLine < 0.0f)
            {
                // Agent is to the right (valid side)
                // Insert into sorted list
                InsertObstacleNeighbor(node.Obstacle, rangeSq, results, pos);
            }

            QueryRecursive(pos, ref rangeSq, agentLeftOfLine >= 0.0f ? node.Right : node.Left, results);
        }
    }

    private void InsertObstacleNeighbor(RVOObstacle obstacle, float rangeSq, List<RVOObstacle> results, float2 pos)
    {
        // Calculate dist
        float distSq = RVOMath.distSqPointLineSegment(obstacle.Point1, obstacle.Point2, pos);

        // Standard Insertion Sort
        // RVO2 maintains a limit (MaxNeighbors). We don't have it here, but we can assume we want all relevant?
        // Or limit to some number?
        // The interface `QueryNearest` implies we return a list.
        // RVO2 `Agent` keeps `obstacleNeighbors_` list.
        // Let's just add and sort at the end? No, `rangeSq` update depends on it?
        // RVO2 updates `rangeSq` only if `obstacleNeighbors_.Count == maxNeighbors_`.
        // If we don't have a max count, rangeSq remains whatever.

        results.Add(obstacle);

        // Optimization: We could sort at the end of Query, but RVO2 does it incrementally.
        // Given we return `List`, we can just Add.
        // BUT the recursive query logic depends on `rangeSq`?
        // Only if we limit the count. If we want ALL within radius (infinite), we don't shrink rangeSq.
        // The caller (RVOSimulator) wants sorted list.
        // So we should sort.
    }
}
