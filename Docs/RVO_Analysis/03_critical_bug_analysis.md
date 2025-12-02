# CRITICAL BUG FOUND: Convexity Calculation Error

## Root Cause

The convexity calculation in `RVOSimulator.ProcessObstacles()` is **FUNDAMENTALLY WRONG**.

### RVO2-Unity (Correct)

```csharp
// File: Simulator.cs, Line 248
obstacle.convex_ = (RVOMath.leftOf(
    vertices[(i == 0 ? vertices.Count - 1 : i - 1)],  // Previous vertex
    vertices[i],                                       // Current vertex  
    vertices[(i == vertices.Count - 1 ? 0 : i + 1)]   // Next vertex
) >= 0.0f);
```

**Uses 3-point test:**
- Takes previous vertex, current vertex, next vertex
- Calls `leftOf()` which computes the cross product
- Determines if the turn at current vertex is left (convex) or right (concave)

```csharp
// RVOMath.cs
public static float leftOf(Vector2 a, Vector2 b, Vector2 c)
{
    return det(a - c, b - a);
}
```

This is equivalent to:
```
det((prev - next), (current - prev))
```

### Custom RVO (WRONG)

```csharp
// File: RVOSimulator.cs, Line 170-171
float2 nextDir = current.NextObstacle.Direction;
current.IsConvex = RVOMath.det(current.Direction, nextDir) >= 0.0f;
```

**Uses 2-edge test:**
- Takes current edge direction and next edge direction
- Computes determinant of two directions
- This is NOT equivalent to the 3-point test!

### Why They're Different

**3-Point Test (RVO2-Unity):**
```
Given vertices: prev, curr, next
Vector1 = curr - prev
Vector2 = next - curr
det(Vector1, Vector2) >= 0 means left turn (convex)
```

**2-Edge Test (Custom RVO):**
```
Given edges: edge1, edge2
edge1.Direction = normalize(edge1.Point2 - edge1.Point1)
edge2.Direction = normalize(edge2.Point1 - edge2.Point2)
det(edge1.Direction, edge2.Direction) >= 0
```

**Problem:** The 2-edge test uses NORMALIZED directions, while the 3-point test uses unnormalized vectors!

More critically, for our edge representation:
- `current.Direction` = direction FROM Point1 TO Point2
- `nextObstacle.Direction` = direction FROM nextObstacle.Point1 TO nextObstacle.Point2

But `current.Point2` should equal `nextObstacle.Point1`!

So we're computing:
```
det(Point2 - Point1, nextPoint2 - Point2)
```

Wait, that's actually correct in terms of the vectors, but the normalization might cause issues...

No wait, let me reconsider. Let me trace through a specific example.

---

## Concrete Example: Rectangle (CCW)

Vertices in order:
```
TL = (-10, 1)
BL = (-10, -1)
BR = (10, -1)  
TR = (10, 1)
```

### RVO2-Unity Processing

**Obstacle 0 (vertex TL):**
```
previous vert = TR = (10, 1)
current vert = TL = (-10, 1)
next vert = BL = (-10, -1)

Vector1 = TL - TR = (-10, 1) - (10, 1) = (-20, 0)
Vector2 = BL - TL = (-10, -1) - (-10, 1) = (0, -2)

det(Vector1, Vector2) = det((-20, 0), (0, -2))
                      = (-20) * (-2) - 0 * 0
                      = 40 >= 0  ✓ CONVEX

direction_ = normalize(BL - TL) = normalize((0, -2)) = (0, -1)
```

**Obstacle 1 (vertex BL):**
```
previous vert = TL = (-10, 1)
current vert = BL = (-10, -1)
next vert = BR = (10, -1)

Vector1 = BL - TL = (0, -2)
Vector2 = BR - BL = (20, 0)

det(Vector1, Vector2) = det((0,  -2), (20, 0))
                      = 0 * 0 - (-2) * 20
                      = 40 >= 0  ✓ CONVEX

direction_ = normalize(BR - BL) = normalize((20, 0)) = (1, 0)
```

### Custom RVO Processing

**RVOObstacle 0 (edge TL→BL):**
```
Point1 = TL = (-10, 1)
Point2 = BL = (-10, -1)
Direction = normalize(BL - TL) = normalize((0, -2)) = (0, -1)

NextObstacle = RVOObstacle[1]
NextObstacle.Direction = normalize(BR - BL) = normalize((20, 0)) = (1, 0)

IsConvex = det(Direction, NextObstacle.Direction)
         = det((0, -1), (1, 0))
         = 0 * 0 - (-1) * 1
         = 1 >= 0  ✓ CONVEX
```

Hmm, this gives the correct result for BL vertex!

**RVOObstacle 1 (edge BL→BR):**
```
Point1 = BL = (-10, -1)
Point2 = BR = (10, -1)
Direction = normalize(BR - BL) = (1, 0)

NextObstacle = RVOObstacle[2]
NextObstacle.Direction = normalize(TR - BR) = normalize((0, 2)) = (0, 1)

IsConvex = det((1, 0), (0, 1))
         = 1 * 1 - 0 * 0
         = 1 >= 0  ✓ CONVEX
```

Also correct for BR vertex!

So the convexity calculation might actually be fine...

## Real Problem: What's Being Checked

Let me re-examine the collision detection logic more carefully.

Actually, I think I found it! Look at line 333 in RVOMath.cs (our current code):

For **right vertex collision** (vertex2 = Point2):
```csharp
RVOObstacle nextObstacle = obstacle1.NextObstacle;
bool isConvex = (nextObstacle != null) ? nextObstacle.IsConvex : true;
```

This checks `nextObstacle.IsConvex`, which is the convexity at `nextObstacle.Point1`.

**But `nextObstacle.Point1` is the SAME as `obstacle1.Point2`!**

So `nextObstacle.IsConvex` tells us about vertex2's convexity. **This is actually CORRECT!**

Wait, then what's the problem?

Let me look more care fully at when obstacles are being selected...

OH! I think I found it. The issue might be in how we're iterating obstacles vs how RVO2-Unity uses neighbor search!

RVO2-Unity uses a KD-Tree to find ONLY nearby obstacle edges, while we're checking ALL edges with manual range filtering.

But more importantly - let me check what `obstacle1` vs `obstacle2` mean in the oblique cases...
