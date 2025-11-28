using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;

public class AStarPathfinder
{
    private class Node
    {
        public Vector2Int Position;
        public Node Parent;
        public float G; // Cost from start
        public float H; // Heuristic to end
        public float F => G + H;

        public Node(Vector2Int pos)
        {
            Position = pos;
        }
    }

    private static readonly Vector2Int[] Directions = new Vector2Int[]
    {
        new Vector2Int(0, 1),   // Up
        new Vector2Int(1, 0),   // Right
        new Vector2Int(0, -1),  // Down
        new Vector2Int(-1, 0),  // Left
        new Vector2Int(1, 1),   // Up-Right
        new Vector2Int(1, -1),  // Down-Right
        new Vector2Int(-1, -1), // Down-Left
        new Vector2Int(-1, 1)   // Up-Left
    };

    public static bool FindPath(Vector2 startPos, Vector2 endPos, GridMap map, List<Vector2> path)
    {
        path.Clear();

        Vector2Int startNodePos = map.WorldToGrid(startPos);
        Vector2Int endNodePos = map.WorldToGrid(endPos);

        if (!map.IsValid(startNodePos.x, startNodePos.y) || !map.IsValid(endNodePos.x, endNodePos.y))
        {
            return false;
        }

        if (map.IsObstacle(endNodePos.x, endNodePos.y))
        {
            // Find nearest walkable neighbor to end
            // Simplified: just fail for now or pick one
            return false;
        }

        Dictionary<Vector2Int, Node> openSet = new Dictionary<Vector2Int, Node>();
        HashSet<Vector2Int> closedSet = new HashSet<Vector2Int>();

        // Priority queue would be better, but for simplicity using List and sorting
        List<Node> openList = new List<Node>();

        Node startNode = new Node(startNodePos);
        startNode.G = 0;
        startNode.H = Vector2Int.Distance(startNodePos, endNodePos);

        openSet.Add(startNodePos, startNode);
        openList.Add(startNode);

        while (openList.Count > 0)
        {
            // Get node with lowest F
            openList.Sort((a, b) => a.F.CompareTo(b.F));
            Node current = openList[0];
            openList.RemoveAt(0);
            openSet.Remove(current.Position);

            closedSet.Add(current.Position);

            if (current.Position == endNodePos)
            {
                RetracePath(current, map, path);
                return true;
            }

            foreach (Vector2Int dir in Directions)
            {
                Vector2Int neighborPos = current.Position + dir;

                if (!map.IsValid(neighborPos.x, neighborPos.y) || map.IsObstacle(neighborPos.x, neighborPos.y) || closedSet.Contains(neighborPos))
                {
                    continue;
                }

                // Corner cutting check
                if (dir.x != 0 && dir.y != 0)
                {
                    if (map.IsObstacle(current.Position.x + dir.x, current.Position.y) ||
                        map.IsObstacle(current.Position.x, current.Position.y + dir.y))
                    {
                        continue;
                    }
                }

                float moveCost = (dir.x != 0 && dir.y != 0) ? 1.414f : 1.0f;
                float newG = current.G + moveCost;

                Node neighborNode;
                if (openSet.TryGetValue(neighborPos, out neighborNode))
                {
                    if (newG < neighborNode.G)
                    {
                        neighborNode.G = newG;
                        // F is computed property, no need to assign
                        neighborNode.Parent = current;
                    }
                }
                else
                {
                    neighborNode = new Node(neighborPos);
                    neighborNode.G = newG;
                    neighborNode.H = Vector2Int.Distance(neighborPos, endNodePos);
                    neighborNode.Parent = current;

                    openSet.Add(neighborPos, neighborNode);
                    openList.Add(neighborNode);
                }
            }
        }

        return false;
    }

    private static void RetracePath(Node endNode, GridMap map, List<Vector2> path)
    {
        Node current = endNode;
        while (current != null)
        {
            path.Add(map.GridToWorld(current.Position.x, current.Position.y));
            current = current.Parent;
        }
        path.Reverse();
    }
}
