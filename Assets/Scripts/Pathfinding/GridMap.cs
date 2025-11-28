using UnityEngine;
using System.Collections.Generic;

public class GridMap
{
    public int Width { get; private set; }
    public int Height { get; private set; }
    public float CellSize { get; private set; }
    public Vector2 Origin { get; private set; }

    private bool[,] _obstacles;

    public GridMap(int width, int height, float cellSize, Vector2 origin)
    {
        Width = width;
        Height = height;
        CellSize = cellSize;
        Origin = origin;
        _obstacles = new bool[width, height];
    }

    public void SetObstacle(int x, int y, bool isObstacle)
    {
        if (IsValid(x, y))
        {
            _obstacles[x, y] = isObstacle;
        }
    }

    public bool IsObstacle(int x, int y)
    {
        if (!IsValid(x, y)) return true; // Out of bounds is obstacle
        return _obstacles[x, y];
    }

    public bool IsValid(int x, int y)
    {
        return x >= 0 && x < Width && y >= 0 && y < Height;
    }

    public Vector2Int WorldToGrid(Vector2 worldPos)
    {
        int x = Mathf.FloorToInt((worldPos.x - Origin.x) / CellSize);
        int y = Mathf.FloorToInt((worldPos.y - Origin.y) / CellSize);
        return new Vector2Int(x, y);
    }

    public Vector2 GridToWorld(int x, int y)
    {
        return new Vector2(x * CellSize + CellSize * 0.5f, y * CellSize + CellSize * 0.5f) + Origin;
    }
    
    public void Clear()
    {
        System.Array.Clear(_obstacles, 0, _obstacles.Length);
    }
}
