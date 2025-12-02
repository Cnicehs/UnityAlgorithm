# Complete Obstacle Reference Mapping

## Normal Case (legs from different vertices)

### RVO2-Unity
```
obstacle1 = vertex1
obstacle2 = vertex2 = obstacle1.next_

leftLegDirection: from vertex1
rightLegDirection: from vertex2

Foreign leg detection:
- Left: obstacle1.convex_, obstacle1.previous_.direction_
- Right: obstacle2.convex_, obstacle2.direction_

Cutoff centers:
- Left: obstacle1.point_ = vertex1
- Right: obstacle2.point_ = vertex2

Cutoff line direction: -obstacle1.direction_ (edge from vertex1 to vertex2)
```

### Custom RVO
```
obstacle1 = edge (vertex1 → vertex2)
leftVertexObstacle = obstacle1 (vertex1 is at Point1)
rightVertexObstacle = obstacle1.NextObstacle (vertex2 is at NextObstacle.Point1)

leftLegDirection: from vertex1
rightLegDirection: from vertex2

Foreign leg detection:
- Left: leftVertexObstacle.IsConvex, leftVertexObstacle.PrevObstacle.Direction
- Right: rightVertexObstacle.IsConvex, rightVertexObstacle.Direction

Cutoff centers:
- Left: leftVertexObstacle.Point1 = vertex1
- Right: rightVertexObstacle.Point1 = vertex2

Cutoff line direction: -obstacle1.Direction (edge from vertex1 to vertex2)
```

**Status:** ✅ Equivalent

---

## Oblique Left Case (s < 0, both legs from vertex1)

### RVO2-Unity
```
obstacle1 = vertex1 (unchanged)
obstacle2 = obstacle1 (MODIFIED! now also points to vertex1)

leftLegDirection: from vertex1
rightLegDirection: from vertex1

Foreign leg detection:
- Left: obstacle1.convex_, obstacle1.previous_.direction_
- Right: obstacle2.convex_ (=obstacle1.convex_), obstacle2.direction_ (=obstacle1.direction_)

Cutoff centers:
- Left: obstacle1.point_ = vertex1
- Right: obstacle2.point_ = vertex1  (SAME as left!)

Cutoff line direction: -obstacle1.direction_ (edge from vertex1 to next vertex)
```

### Custom RVO  
```
obstacle1 = edge (vertex1 → vertex2) (unchanged)
leftVertexObstacle = obstacle1 (unchanged)
rightVertexObstacle = obstacle1 (MODIFIED! now same as left)

leftLegDirection: from vertex1
rightLegDirection: from vertex1

Foreign leg detection:
- Left: leftVertexObstacle.IsConvex, leftVertexObstacle.PrevObstacle.Direction
- Right: rightVertexObstacle.IsConvex (=leftVertexObstacle.IsConvex), 
         rightVertexObstacle.Direction (=leftVertexObstacle.Direction = obstacle1.Direction)

Cutoff centers:
- Left: leftVertexObstacle.Point1 = vertex1
- Right: rightVertexObstacle.Point1 = vertex1  (SAME as left!)

Cutoff line direction: -obstacle1.Direction (edge from vertex1 to vertex2)
```

**Analysis:**
- Foreign leg right uses `rightVertexObstacle.Direction` = `obstacle1.Direction` = direction from vertex1 to vertex2
- RVO2 uses `obstacle2.direction_` = `obstacle1.direction_` = direction from vertex1 to next vertex
- These are THE SAME because "next vertex after vertex1" IS vertex2!

**Status:** ✅ Equivalent

---

## Oblique Right Case (s > 1, both legs from vertex2)

### RVO2-Unity
```
obstacle1 = obstacle2 (MODIFIED! now points to vertex2)
obstacle2 = vertex2 (unchanged)

leftLegDirection: from vertex2
rightLegDirection: from vertex2

Foreign leg detection:
- Left: obstacle1.convex_ (=obstacle2.convex_), obstacle1.previous_.direction_ (=obstacle2.previous_.direction_)
- Right: obstacle2.convex_, obstacle2.direction_

Cutoff centers:
- Left: obstacle1.point_ = vertex2  (CHANGED!)
- Right: obstacle2.point_ = vertex2

Cutoff line direction: -obstacle1.direction_ (=obstacle2.direction_, edge from vertex2 to next vertex)
```

### Custom RVO
```
obstacle1 = edge (vertex1 → vertex2) (unchanged)
leftVertexObstacle = obstacle1.NextObstacle (MODIFIED! vertex2's edge)
rightVertexObstacle = obstacle1.NextObstacle (MODIFIED! same)

leftLegDirection: from vertex2
rightLegDirection: from vertex2

Foreign leg detection:
- Left: leftVertexObstacle.IsConvex (nextObstacle.IsConvex = vertex2 convexity), 
        leftVertexObstacle.PrevObstacle.Direction (= obstacle1.Direction, edge from vertex1 to vertex2)
- Right: rightVertexObstacle.IsConvex, rightVertexObstacle.Direction (edge from vertex2 to vertex3)

Cutoff centers:
- Left: leftVertexObstacle.Point1 = nextObstacle.Point1 = vertex2
- Right: rightVertexObstacle.Point1 = nextObstacle.Point1 = vertex2

Cutoff line direction: -obstacle1.Direction (edge from vertex1 to vertex2)
```

**Analysis - Cutoff Line Direction:**
- RVO2 uses: -obstacle1.direction_ where obstacle1 now = vertex2, so direction is vertex2→vertex3
- Custom uses: -obstacle1.Direction where obstacle1 is still the original edge, so direction is vertex1→vertex2

**⚠️ MISMATCH!** In oblique right case, cutoff line direction is WRONG!

---

## The Problem

In oblique right case, RVO2-Unity modifies `obstacle1 = obstacle2`, so `obstacle1.direction_` becomes the direction FROM vertex2 TO vertex3.

But in our implementation, we always use the original `obstacle1.Direction`, which is FROM vertex1 TO vertex2.

## The Fix

We need to track which edge contains the cutoff line. The cutoff line should be parallel to the edge that connects the two leg-emanation vertices.

In oblique right case, both legs come from vertex2, so we should use the edge that STARTS at vertex2.

```csharp
// Track the edge that defines the cutoff line
// In normal case: the original edge (vertex1 to vertex2)
// In oblique left: the original edge (starts at vertex1)
// In oblique right: the next edge (starts at vertex2)
RVOObstacle cutoffEdge = obstacle1;

if (s > 1.0f && distSqLine <= radiusSq)
{
    // Oblique right
    cutoffEdge = obstacle1.NextObstacle;
    leftVertexObstacle = cutoffEdge;
    rightVertexObstacle = cutoffEdge;
}

// Later, when projecting on cutoff line:
line.Direction = -cutoffEdge.Direction;
```
