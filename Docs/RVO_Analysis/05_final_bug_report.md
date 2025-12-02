# ROOT CAUSE IDENTIFIED: Obstacle Reference Bug in Foreign Leg Detection

## The Bug

In the oblique view cases, RVO2-Unity **modifies** `obstacle1` and `obstacle2` to point to the vertex where both legs emanate from. These modified references are then used in foreign leg detection.

Custom RVO does NOT modify any obstacle references, so foreign leg detection uses the WRONG obstacles.

## RVO2-Unity Behavior

```csharp
// Initial state
Obstacle obstacle1 = obstacleNeighbors_[i].Value;  // vertex1
Obstacle obstacle2 = obstacle1.next_;              // vertex2

// ... collision checks ...

// Oblique case 1: s < 0 && distSqLine <= radiusSq
if (s < 0.0f && distSqLine <= radiusSq)
{
    obstacle2 = obstacle1;  // ⚠️ BOTH NOW POINT TO vertex1
    // compute legs from vertex1
}
// Oblique case 2: s > 1 && distSqLine <= radiusSq
else if (s > 1.0f && distSqLine <= radiusSq)
{
    obstacle1 = obstacle2;  // ⚠️ BOTH NOW POINT TO vertex2
    // compute legs from vertex2
}

// Foreign leg detection
Obstacle leftNeighbor = obstacle1.previous_;   // Uses MODIFIED obstacle1
                                                // Gets correct previous for the vertex

if (obstacle1.convex_ && det(leftLegDirection, -leftNeighbor.direction_) >= 0.0f)
{
    leftLegDirection = -leftNeighbor.direction_;
    isLeftLegForeign = true;
}

if (obstacle2.convex_ && det(rightLegDirection, obstacle2.direction_) <= 0.0f)
{
    rightLegDirection = obstacle2.direction_;
    isRightLegForeign = true;
}
```

## Custom RVO Bug

```csharp
// Initial state
RVOObstacle obstacle1 = obstacles[i];  // edge from vertex1 to vertex2

float2 vertex1 = obstacle1.Point1;
float2 vertex2 = obstacle1.Point2;

// ... collision checks ...

// Oblique case 1
if (s < 0.0f && distSqLine <= radiusSq)
{
    // We compute legs from vertex1
    // But we DON'T modify obstacle1!
}
// Oblique case 2
else if (s > 1.0f && distSqLine <= radiusSq)
{
    // We compute legs from vertex2
    // But we DON'T modify obstacle1!
}

// Foreign leg detection - ALWAYS uses original obstacle1!
RVOObstacle prevObstacle = obstacle1.PrevObstacle;   // ⚠️ WRONG in oblique case 2!
RVOObstacle nextObstacle2 = obstacle1.NextObstacle;  // ⚠️ WRONG in oblique case 1!

// In oblique case 2 (legs from vertex2):
// - We should use vertex2's neighbors
// - But we're using vertex1's neighbors!
```

## Example Scenario

Rectangle obstacle:
```
TL ---(edge0)---> BL ---(edge1)---> BR ---(edge2)---> TR ---(edge3)---> TL
```

Agent position: oblique to edge1, closer to BR (vertex2 of edge1)

**RVO2-Unity:**
```
obstacle1 = vertex BL (from neighbor search)
obstacle2 = vertex BR (obstacle1.next_)

s > 1.0f && distSqLine <= radiusSq → oblique case 2

obstacle1 = obstacle2  // Now obstacle1 = BR

leftNeighbor = obstacle1.previous_ = BL
rightNeighbor = obstacle2 = BR

// Foreign leg check uses BR's neighbors (BL and TR)
// CORRECT!
```

**Custom RVO:**
```
obstacle1 = edge1 (BL → BR)

s > 1.0f && distSqLine <= radiusSq → oblique case 2

// obstacle1 is NOT modified!

prevObstacle = obstacle1.PrevObstacle = edge0 (TL → BL)
nextObstacle2 = obstacle1.NextObstacle = edge2 (BR → TR)

// Foreign leg check:
// - Left leg should check against BR's left neighbor (edge from BL)
//   But we check against edge0's direction (TL → BL)  ❌ WRONG!
// - Right leg should check against BR's right neighbor (edge to TR)  
//   We check against edge2's direction (BR → TR)  ✓ This one is correct by luck!
```

## The Fix

We need to track which VERTEX the legs emanate from, and use THAT vertex's neighbors for foreign leg detection.

### Option 1: Add vertex tracking variables

```csharp
RVOObstacle leftVertexObstacle = obstacle1;   // Obstacle whose Point1 is the left vertex
RVOObstacle rightVertexObstacle = obstacle1.NextObstacle;  // Obstacle whose Point1 is the right vertex

if (s < 0.0f && distSqLine <= radiusSq)
{
    // Both legs from vertex1
    rightVertexObstacle = obstacle1;  // Both now reference vertex1's obstacle
}
else if (s > 1.0f && distSqLine <= radiusSq)
{
    // Both legs from vertex2
    leftVertexObstacle = obstacle1.NextObstacle;  // Both now reference vertex2's obstacle
}

// Foreign leg detection
RVOObstacle leftNeighbor = leftVertexObstacle.PrevObstacle;
RVOObstacle rightVertex = rightVertexObstacle;

if (leftVertexObstacle.IsConvex && leftNeighbor != null && 
    det(leftLegDirection, -leftNeighbor.Direction) >= 0.0f)
{
    leftLegDirection = -leftNeighbor.Direction;
    isLeftLegForeign = true;
}

if (rightVertex.IsConvex && 
    det(rightLegDirection, rightVertex.Direction) <= 0.0f)
{
    rightLegDirection = rightVertex.Direction;
    isRightLegForeign = true;
}
```

### Option 2: Inline the logic for each case

Handle foreign leg detection separately in each case (usual, oblique left, oblique right).

## Impact

This bug causes incorrect foreign leg detection, which leads to:
- Wrong ORCA line directions in certain geometric configurations
- Agents can penetrate obstacles when approaching from specific angles
- Particularly affects corners and oblique approaches
