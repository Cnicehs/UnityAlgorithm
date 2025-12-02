# Convexity Calculation Debug

## Obstacle Setup (from RVOoBSDemo.cs)

```
Rectangle: width=20, thickness=2
pTL = (-10, 1)   pTR = (10, 1)
pBL = (-10, -1)  pBR = (10, -1)
```

### Edges Added (CCW order):
```
Edge0: TL→BL = (-10,1) → (-10,-1)   Direction = (0, -2) normalized = (0, -1)
Edge1: BL→BR = (-10,-1) → (10,-1)   Direction = (20, 0) normalized = (1, 0)  
Edge2: BR→TR = (10,-1) → (10,1)     Direction = (0, 2) normalized = (0, 1)
Edge3: TR→TL = (10,1) → (-10,1)     Direction = (-20, 0) normalized = (-1, 0)
```

## Convexity Calculation in `ProcessObstacles`

```csharp
current.IsConvex = det(current.Direction, nextDir) >= 0.0f;
```

### Edge0 (Left edge):
- `current.Direction` = (0, -1)
- `nextDir` = Edge1.Direction = (1, 0)
- `det((0,-1), (1,0))` = 0×0 - (-1)×1 = 1 **≥ 0 → CONVEX ✓**

### Edge1 (Bottom edge):
- `current.Direction` = (1, 0)
- `nextDir` = Edge2.Direction = (0, 1)
- `det((1,0), (0,1))` = 1×1 - 0×0 = 1 **≥ 0 → CONVEX ✓**

### Edge2 (Right edge):
- `current.Direction` = (0, 1)
- `nextDir` = Edge3.Direction = (-1, 0)
- `det((0,1), (-1,0))` = 0×0 - 1×(-1) = 1 **≥ 0 → CONVEX ✓**

### Edge3 (Top edge):
- `current.Direction` = (-1, 0)
- `nextDir` = Edge0.Direction = (0, -1)
- `det((-1,0), (0,-1))` = (-1)×(-1) - 0×0 = 1 **≥ 0 → CONVEX ✓**

**All vertices are CONVEX!** This is correct for a CCW rectangle.

## But wait... IsConvex is stored PER EDGE, not PER VERTEX!

In our system:
- `Edge0.IsConvex` = convexity at Edge0.Point1 = TL
- `Edge1.IsConvex` = convexity at Edge1.Point1 = BL
- `Edge2.IsConvex` = convexity at Edge2.Point1 = BR
- `Edge3.IsConvex` = convexity at Edge3.Point1 = TR

But in `ConstructObstacleORCALines`, when we check `nextObstacle.IsConvex` for the right vertex collision:

```csharp
// obstacle1 = Edge1 (BL→BR)
// vertex2 = BR = obstacle1.Point2 = nextObstacle.Point1
// We check: nextObstacle.IsConvex

// nextObstacle = Edge2 (BR→TR)
// Edge2.IsConvex = convexity at BR ✓ CORRECT!
```

This looks correct too...

## Let me check the actual obstacle linking in ProcessObstacles

After geometric linking:
- Edge0 (TL→BL): Next=?, Prev=?

Wait, the geometric linking might be finding the WRONG next/prev!

Let me trace it...
