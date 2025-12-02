# RVO vs RVO2-Unity: Data Structure Comparison

## Obstacle Representation

### RVO2-Unity (Official)
```csharp
// File: Obstacle.cs
internal class Obstacle
{
    internal Obstacle next_;        // Next obstacle vertex
    internal Obstacle previous_;    // Previous obstacle vertex
    internal Vector2 direction_;    // Unit direction to next vertex
    internal Vector2 point_;        // THIS vertex position
    internal int id_;
    internal bool convex_;          // Is THIS vertex convex?
}
```

**Key Points:**
- Each `Obstacle` object represents a **VERTEX** (not an edge)
- `point_` is the position of THIS vertex
- `next_` points to the next vertex in the chain
- `direction_` is the unit vector pointing from THIS vertex to the NEXT vertex
- An edge is formed between `obstacle.point_` and `obstacle.next_.point_`

**Example (Rectangle):**
```
Obstacle[0]: point=(TL), next=Obstacle[1], direction=(BL-TL normalized)
Obstacle[1]: point=(BL), next=Obstacle[2], direction=(BR-BL normalized)
Obstacle[2]: point=(BR), next=Obstacle[3], direction=(TR-BR normalized)
Obstacle[3]: point=(TR), next=Obstacle[0], direction=(TL-TR normalized)
```

### Custom RVO
```csharp
// File: RVOObstacle.cs
public class RVOObstacle
{
    public float2 Point1;            // Edge start vertex
    public float2 Point2;            // Edge end vertex
    public float2 Direction;         // Unit direction from Point1 to Point2
    public float2 Normal;            // Perpendicular to Direction
    public int Id;
    public bool IsConvex;            // Is Point1 convex?
    public RVOObstacle NextObstacle; // Next edge
    public RVOObstacle PrevObstacle; // Previous edge
}
```

**Key Points:**
- Each `RVOObstacle` represents an **EDGE** (not a vertex)
- `Point1` is the start of THIS edge
- `Point2` is the end of THIS edge
- `Direction` is the unit vector from Point1 to Point2
- `IsConvex` refers to the convexity at Point1
- `NextObstacle.Point1` should equal THIS `Point2` (if properly linked)

**Example (Same Rectangle):**
```
RVOObstacle[0]: Point1=(TL), Point2=(BL), Direction=(BL-TL normalized)
RVOObstacle[1]: Point1=(BL), Point2=(BR), Direction=(BR-BL normalized)
RVOObstacle[2]: Point1=(BR), Point2=(TR), Direction=(TR-BR normalized)
RVOObstacle[3]: Point1=(TR), Point2=(TL), Direction=(TL-TR normalized)
```

### Critical Difference

**RVO2-Unity Processing:**
```csharp
Obstacle obstacle1 = obstacleNeighbors_[i].Value;
Obstacle obstacle2 = obstacle1.next_;

Vector2 vertex1 = obstacle1.point_;  // First vertex
Vector2 vertex2 = obstacle2.point_;  // Second vertex (from next object)
// Edge is from vertex1 to vertex2
```

**Custom RVO Should Process:**
```csharp
RVOObstacle obstacle = obstacles[i];

float2 vertex1 = obstacle.Point1;  // First vertex
float2 vertex2 = obstacle.Point2;  // Second vertex (from same object)
// Edge is from vertex1 to vertex2
```

---

## Agent Representation

### RVO2-Unity
```csharp
internal class Agent
{
    internal Vector2 position_;
    internal Vector2 velocity_;
    internal Vector2 prefVelocity_;
    internal float radius_;
    internal float timeHorizon_;
    internal float timeHorizonObst_;
    internal IList<KeyValuePair<float, Obstacle>> obstacleNeighbors_;
    internal IList<Line> orcaLines_;
}
```

### Custom RVO
```csharp
public class RVOAgent
{
    public int ID;
    public float2 Position;
    public float2 Velocity;
    public float2 PrefVelocity;
    public float Radius;
    public float TimeHorizon;
    public float TimeHorizonObst;
    public float2 NewVelocity;
}
```

**Difference:** RVO2-Unity stores neighbors in agent, custom RVO computes neighbors on-demand.

---

## ORCA Line Representation

### RVO2-Unity
```csharp
internal struct Line
{
    internal Vector2 direction;  // Line direction (unit vector)
    internal Vector2 point;      // A point on the line
}
```

### Custom RVO
```csharp
public struct ORCALine
{
    public float2 Direction;  // Line direction (unit vector)
    public float2 Point;      // A point on the line
}
```

**Difference:** None - structurally identical.

---

## Obstacle Storage in Simulator

### RVO2-Unity
```csharp
// File: Simulator.cs
private IList<Obstacle> obstacles_ = new List<Obstacle>();

public int addObstacle(IList<Vector2> vertices)
{
    // Creates one Obstacle per vertex
    for (int i = 0; i < vertices.Count; ++i)
    {
        Obstacle obstacle = new Obstacle();
        obstacle.point_ = vertices[i];
        obstacles_.Add(obstacle);
    }
    
    // Links them together
    for (int i = 0; i < numObstVertices; ++i)
    {
        obstacles_[i].next_ = obstacles_[(i + 1) % numObstVertices];
        obstacles_[i].previous_ = obstacles_[(i - 1 + numObstVertices) % numObstVertices];
        obstacles_[i].direction_ = normalize(obstacles_[i].next_.point_ - obstacles_[i].point_);
    }
}
```

### Custom RVO
```csharp
// File: RVOSimulator.cs
private List<RVOObstacle> _obstacles = new List<RVOObstacle>();

public RVOObstacle AddObstacle(Vector3 p1, Vector3 p2)
{
    RVOObstacle obstacle = new RVOObstacle(new float2(p1.x, p1.z), new float2(p2.x, p2.z));
    _obstacles.Add(obstacle);
    return obstacle;
}

public void ProcessObstacles()
{
    // Links edges together
    for (int i = 0; i < _obstacles.Count; i++)
    {
        current.NextObstacle = _obstacles[(i + 1) % _obstacles.Count];
        current.PrevObstacle = _obstacles[(i - 1 + _obstacles.Count) % _obstacles.Count];
        current.IsConvex = det(current.Direction, nextDir) >= 0.0f;
    }
}
```

**Critical Difference:**
- RVO2-Unity: Adds N vertices → creates N Obstacle objects (one per vertex)
- Custom RVO: Adds N edges → creates N RVOObstacle objects (one per edge)

---

## Convexity Definition

### RVO2-Unity
```csharp
// Convexity is computed for each VERTEX
// A vertex is convex if the angle turns left (CCW)
float determinant = det(direction_, next_.direction_);
convex_ = determinant >= 0.0f;
```
- `direction_` points from THIS vertex to NEXT vertex
- `next_.direction_` points from NEXT vertex to NEXT-NEXT vertex
- Convexity is about the turn at the vertex

### Custom RVO
```csharp
// Convexity is computed for each EDGE's starting vertex
float2 nextDir = current.NextObstacle.Direction;
current.IsConvex = RVOMath.det(current.Direction, nextDir) >= 0.0f;
```
- `current.Direction` is the direction of the current edge
- `nextDir` is the direction of the next edge
- This is actually the same logic, just stored differently

**Conclusion:** Convexity logic is equivalent.

---

## Summary of Key Structural Differences

| Aspect | RVO2-Unity | Custom RVO |
|--------|------------|------------|
| Obstacle Unit | Vertex | Edge |
| Obstacle Count | N vertices | N edges |
| Edge Definition | obstacle1.point_ → obstacle1.next_.point_ | obstacle.Point1 → obstacle.Point2 |
| Iteration | Loop through vertices | Loop through edges |
| Convexity Storage | On vertex object | On edge object (for start vertex) |
| Neighbor Search | KD-Tree based | Spatial index based |
