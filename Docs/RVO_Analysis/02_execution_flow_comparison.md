# RVO vs RVO2-Unity: Execution Flow Comparison

## Overall Simulation Step

### RVO2-Unity Flow
```
1. Simulator.doStep()
2.   → Agent.computeNeighbors()
3.     → KdTree.computeObstacleNeighbors()  // Find nearby obstacles
4.     → KdTree.computeAgentNeighbors()     // Find nearby agents
5.   → Agent.computeNewVelocity()
6.     → Process obstacle neighbors → create obstacle ORCA lines
7.     → Process agent neighbors → create agent ORCA lines
8.     → linearProgram2() → linearProgram3() → solve for new velocity
9.   → Agent.update()  // Apply velocity, update position
```

### Custom RVO Flow
```
1. RVOSimulator.Step()
2.   → For each agent:
3.     → SpatialIndexManager.GetNeighborsInRadius()  // Find nearby agents
4.     → RVOMath.ConstructObstacleORCALines()  // Process ALL obstacles
5.     → RVOMath.ConstructORCALines()  // Process agent neighbors
6.     → RVOMath.linearProgram2() → linearProgram3() → solve
7.   → For each agent:
8.     → Apply velocity, update position
9.   → SpatialIndexManager.UpdatePositions()
```

## Key Difference: Obstacle Neighbor Discovery

### RVO2-Unity: Pre-filtered via KD-Tree
```csharp
// Agent.computeNeighbors()
obstacleNeighbors_.Clear();
float rangeSq = RVOMath.sqr(timeHorizonObst_ * maxSpeed_ + radius_);
Simulator.Instance.kdTree_.computeObstacleNeighbors(this, rangeSq);

// Later in computeNewVelocity():
for (int i = 0; i < obstacleNeighbors_.Count; ++i)  // Only nearby obstacles!
{
    Obstacle obstacle1 = obstacleNeighbors_[i].Value;
    // ... process this obstacle
}
```
- **Result:** Only processes obstacles within range
- **Count:** Typically 1-4 obstacles for a simple wall

### Custom RVO: Processes ALL Obstacles
```csharp
// RVOMath.ConstructObstacleORCALines()
for (int i = 0; i < obstacles.Count; ++i)  // ALL obstacles!
{
    RVOObstacle obstacle1 = obstacles[i];
    
    // Manual range check
    float distSqToLine = distSqPointLineSegment(vertex1, vertex2, agent.Position);
    if (distSqToLine > rangeSq)
    {
        continue;  // Skip far obstacles
    }
    // ... process this obstacle
}
```
- **Result:** Loops through all obstacles, filters manually
- **Count:** Checks all 4 edges of the wall

**Impact:** Should be equivalent if range check logic is identical.

---

## Obstacle ORCA Line Construction

### RVO2-Unity: Agent.computeNewVelocity() [Lines 81-339]

```csharp
for (int i = 0; i < obstacleNeighbors_.Count; ++i)
{
    Obstacle obstacle1 = obstacleNeighbors_[i].Value;  // VERTEX
    Obstacle obstacle2 = obstacle1.next_;              // NEXT VERTEX
    
    Vector2 relativePosition1 = obstacle1.point_ - position_;
    Vector2 relativePosition2 = obstacle2.point_ - position_;
    
    // Already covered check
    bool alreadyCovered = false;
    for (int j = 0; j < orcaLines_.Count; ++j) { ... }
    if (alreadyCovered) continue;
    
    // Collision detection
    float distSq1 = absSq(relativePosition1);
    float distSq2 = absSq(relativePosition2);
    float radiusSq = sqr(radius_);
    
    Vector2 obstacleVector = obstacle2.point_ - obstacle1.point_;
    float s = (-relativePosition1 * obstacleVector) / absSq(obstacleVector);
    float distSqLine = absSq(-relativePosition1 - s * obstacleVector);
    
    // [Collision cases: left vertex, right vertex, segment]
    // [Oblique view cases]
    // [Usual case with leg calculations]
    // [Foreign leg detection]
    // [Velocity projection]
}
```

### Custom RVO: RVOMath.ConstructObstacleORCALines()

```csharp
for (int i = 0; i < obstacles.Count; ++i)
{
    RVOObstacle obstacle1 = obstacles[i];  // EDGE
    
    float2 vertex1 = obstacle1.Point1;  // Edge start
    float2 vertex2 = obstacle1.Point2;  // Edge end
    
    // Range filter
    float distSqToLine = distSqPointLineSegment(vertex1, vertex2, agent.Position);
    if (distSqToLine > rangeSq) continue;
    
    float2 relativePosition1 = vertex1 - agent.Position;
    float2 relativePosition2 = vertex2 - agent.Position;
    
    // Already covered check
    bool alreadyCovered = false;
    for (int j = 0; j < orcaLines.Count; ++j) { ... }
    if (alreadyCovered) continue;
    
    // Collision detection
    float distSq1 = absSq(relativePosition1);
    float distSq2 = absSq(relativePosition2);
    float radiusSq = agent.Radius * agent.Radius;
    
    float2 obstacleVector = vertex2 - vertex1;
    float s = dot(-relativePosition1, obstacleVector) / absSq(obstacleVector);
    float distSqLine = absSq(-relativePosition1 - s * obstacleVector);
    
    // [Same collision cases]
    // [Same oblique view cases]
    // [Same usual case with leg calculations]
    // [Foreign leg detection - needs adjustment]
    // [Velocity projection]
}
```

---

## Critical Logic Differences in Obstacle Processing

### 1. Neighbor Access Pattern

**RVO2-Unity:**
```csharp
Obstacle obstacle1 = obstacleNeighbors_[i].Value;  // Get vertex from neighbor list
Obstacle obstacle2 = obstacle1.next_;              // Get next vertex via pointer
```

**Custom RVO:**
```csharp
RVOObstacle obstacle1 = obstacles[i];  // Get edge from full list
// vertex1 and vertex2 are BOTH in obstacle1
```

### 2. Right Vertex Convexity Check

**RVO2-Unity (Line 148):**
```csharp
// For right vertex collision
if (obstacle2.convex_ && det(relativePosition2, obstacle2.direction_) >= 0.0f)
{
    // obstacle2 is the NEXT vertex
    // obstacle2.direction_ points from vertex2 to vertex3
    // Check if agent is on the "outside" of vertex2
}
```

**Custom RVO:**
```csharp
// For right vertex collision (Point2 of current edge)
RVOObstacle nextObstacle = obstacle1.NextObstacle;
bool isConvex = (nextObstacle != null) ? nextObstacle.IsConvex : true;
float2 nextDirection = (nextObstacle != null) ? nextObstacle.Direction : obstacle1.Direction;

if (isConvex && det(relativePosition2, nextDirection) >= 0.0f)
{
    // nextObstacle.IsConvex refers to the convexity at nextObstacle.Point1
    // But we need convexity at current vertex2!
    // ⚠️ POTENTIAL BUG: vertex2 is the END of current edge
    // Its convexity should be checked differently
}
```

**ISSUE IDENTIFIED:** The convexity check for the right vertex is wrong!

---

## Problem Analysis

### RVO2-Unity Convexity Model

```
obstacle1: [point1] --direction1--> [obstacle2.point]
obstacle2: [point2] --direction2--> [obstacle3.point]
              ↑
        obstacle2.convex_
        (refers to angle at point2)
```

### Custom RVO Convexity Model

```
obstacle1: [Point1] --Direction--> [Point2]
                      ↑
              obstacle1.IsConvex
              (refers to angle at Point1)

obstacle2: [Point1] --Direction--> [Point2]
           (Same as obstacle1.Point2)
                      ↑
              obstacle2.IsConvex
              (refers to angle at obstacle2.Point1)
```

### The Mismatch

When checking right vertex (vertex2 = obstacle1.Point2):
- **Need:** Convexity at vertex2
- **RVO2-Unity:** `obstacle2.convex_` (obstacle2 IS vertex2)
- **Custom RVO:** `nextObstacle.IsConvex` (this is convexity at vertex3, NOT vertex2!)

**Fix:** For vertex2, we should check `obstacle1.NextObstacle.IsConvex` OR we need to store convexity differently!

---

## Proposed Fix

The issue is that in custom RVO:
- `obstacle1.IsConvex` tells us about convexity at `Point1`
- But we have NO direct way to know convexity at `Point2`!

We need to check the NEXT obstacle's IsConvex because:
- `obstacle1.Point2` == `nextObstacle.Point1` (they should be the same vertex)
- `nextObstacle.IsConvex` gives us convexity at that vertex

But wait - this should already be correct! Let me trace through more carefully...

Actually, the problem might be in ProcessObstacles() - how we compute IsConvex.
