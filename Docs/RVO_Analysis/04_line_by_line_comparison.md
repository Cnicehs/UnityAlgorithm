# Line-by-Line Code Comparison: Obstacle ORCA Construction

## Collision Cases Comparison

### Left Vertex Collision (s < 0.0f && distSq1 <= radiusSq)

#### RVO2-Unity (Lines 130-141)
```csharp
if (s < 0.0f && distSq1 <= radiusSq)
{
    /* Collision with left vertex. Ignore if non-convex. */
    if (obstacle1.convex_)
    {
        line.point = new Vector2(0.0f, 0.0f);
        line.direction = RVOMath.normalize(new Vector2(-relativePosition1.y(), relativePosition1.x()));
        orcaLines_.Add(line);
    }
    
    continue;
}
```

**Logic:**
- Check if obstacle1 (the LEFT vertex) is convex
- If convex, add perpendicular constraint line
- `point = (0,0)` means this is a HARD constraint (collision)
- Direction is perpendicular to relativePosition1

#### Custom RVO (Lines 321-329)
```csharp
if (s < 0.0f && distSq1 <= radiusSq)
{
    if (obstacle1.IsConvex)
    {
        line.Point = new float2(0.0f, 0.0f);
        line.Direction = normalize(new float2(-relativePosition1.y, relativePosition1.x));
        orcaLines.Add(line);
    }
    continue;
}
```

**Status:** ✅ IDENTICAL LOGIC

---

### Right Vertex Collision (s > 1.0f && distSq2 <= radiusSq)

#### RVO2-Unity (Lines 142-156)
```csharp
else if (s > 1.0f && distSq2 <= radiusSq)
{
    /*
     * Collision with right vertex. Ignore if non-convex or if
     * it will be taken care of by neighboring obstacle.
     */
    if (obstacle2.convex_ && RVOMath.det(relativePosition2, obstacle2.direction_) >= 0.0f)
    {
        line.point = new Vector2(0.0f, 0.0f);
        line.direction = RVOMath.normalize(new Vector2(-relativePosition2.y(), relativePosition2.x()));
        orcaLines_.Add(line);
    }
    
    continue;
}
```

**Logic:**
1. Check if obstacle2 (the RIGHT vertex) is convex
2. **ADDITIONAL CHECK:** `det(relativePosition2, obstacle2.direction_) >= 0.0f`
3. Only add line if BOTH conditions are true

**What does the det() check mean?**
- `relativePosition2` is vector from agent to vertex2
- `obstacle2.direction_` is direction FROM vertex2 TO vertex3
- `det(relativePosition2, obstacle2.direction_)` checks if agent is on the "left" side of the edge starting at vertex2
- This ensures we only add the constraint if the agent is approaching from the correct side!

#### Custom RVO (Lines 330-342) - ⚠️ MISSING CHECK!
```csharp
else if (s > 1.0f && distSq2 <= radiusSq)
{
    // For the right vertex, we need to check the next obstacle's convexity
    RVOObstacle nextObstacle = obstacle1.NextObstacle;
    bool isConvex = (nextObstacle != null) ? nextObstacle.IsConvex : true;
    float2 nextDirection = (nextObstacle != null) ? nextObstacle.Direction : obstacle1.Direction;

    if (isConvex && det(relativePosition2, nextDirection) >= 0.0f)
    {
        line.Point = new float2(0.0f, 0.0f);
        line.Direction = normalize(new float2(-relativePosition2.y, relativePosition2.x));
        orcaLines.Add(line);
    }
    continue;
}
```

**Wait, we DO have the det() check!**

Let me verify:
- `nextObstacle` is the obstacle AFTER current
- `nextObstacle.Direction` is direction from `nextObstacle.Point1` to `nextObstacle.Point2`
- But `nextObstacle.Point1` should equal `obstacle1.Point2` (which is vertex2)
- So `nextObstacle.Direction` is direction FROM vertex2 TO vertex3

**This should be equivalent!**

But wait... let me check if those points are actually equal...

---

## Problem Identified: Point Continuity

In RVO2-Unity:
- `obstacle1.point_` = vertex1
- `obstacle2 = obstacle1.next_`
- `obstacle2.point_` = vertex2
- `obstacle2.direction_` = normalized(vertex3 - vertex2)

In Custom RVO:
- `obstacle1.Point1` = vertex1
- `obstacle1.Point2` = vertex2
- `nextObstacle = obstacle1.NextObstacle`
- `nextObstacle.Point1` = ??? 
- `nextObstacle.Direction` = normalized(nextObstacle.Point2 - nextObstacle.Point1)

**CRITICAL QUESTION:** Is `nextObstacle.Point1` equal to `obstacle1.Point2`?

Let's check how obstacles are added in the demo:

```csharp
// RVOoBSDemo.cs, lines 57-60
RVOSimulator.Instance.AddObstacle(new Vector3(pTL.x, 0, pTL.y), new Vector3(pBL.x, 0, pBL.y)); // Left: TL→BL
RVOSimulator.Instance.AddObstacle(new Vector3(pBL.x, 0, pBL.y), new Vector3(pBR.x, 0, pBR.y)); // Bottom: BL→BR
RVOSimulator.Instance.AddObstacle(new Vector3(pBR.x, 0, pBR.y), new Vector3(pTR.x, 0, pTR.y)); // Right: BR→TR
RVOSimulator.Instance.AddObstacle(new Vector3(pTR.x, 0, pTR.y), new Vector3(pTL.x, 0, pTL.y)); // Top: TR→TL
```

So:
- Obstacle[0]: Point1=TL, Point2=BL
- Obstacle[1]: Point1=BL, Point2=BR  ✅ Obstacle[1].Point1 == Obstacle[0].Point2
- Obstacle[2]: Point1=BR, Point2=TR  ✅ Obstacle[2].Point1 == Obstacle[1].Point2
- Obstacle[3]: Point1=TR, Point2=TL  ✅ Obstacle[3].Point1 == Obstacle[2].Point2

**Points ARE continuous!**

So the logic should be equivalent...

---

## Segment Collision (s >= 0.0f && s <= 1.0f && distSqLine <= radiusSq)

#### RVO2-Unity (Lines 157-165)
```csharp
else if (s >= 0.0f && s <= 1.0f && distSqLine <= radiusSq)
{
    /* Collision with obstacle segment. */
    line.point = new Vector2(0.0f, 0.0f);
    line.direction = -obstacle1.direction_;
    orcaLines_.Add(line);
    
    continue;
}
```

#### Custom RVO (Lines 343-349)
```csharp
else if (s >= 0.0f && s <= 1.0f && distSqLine <= radiusSq)
{
    line.Point = new float2(0.0f, 0.0f);
    line.Direction = -obstacle1.Direction;
    orcaLines.Add(line);
    continue;
}
```

**Status:** ✅ IDENTICAL LOGIC

---

## No-Collision: Oblique View Cases

### Case 1: s < 0.0f && distSqLine <= radiusSq

#### RVO2-Unity (Lines 175-192)
```csharp
if (s < 0.0f && distSqLine <= radiusSq)
{
    /*
     * Obstacle viewed obliquely so that left vertex
     * defines velocity obstacle.
     */
    if (!obstacle1.convex_)
    {
        /* Ignore obstacle. */
        continue;
    }

    obstacle2 = obstacle1;

    float leg1 = RVOMath.sqrt(distSq1 - radiusSq);
    leftLegDirection = new Vector2(relativePosition1.x() * leg1 - relativePosition1.y() * radius_, relativePosition1.x() * radius_ + relativePosition1.y() * leg1) / distSq1;
    rightLegDirection = new Vector2(relativePosition1.x() * leg1 + relativePosition1.y() * radius_, -relativePosition1.x() * radius_ + relativePosition1.y() * leg1) / distSq1;
}
```

**Key:** Sets `obstacle2 = obstacle1` - both legs emanate from vertex1!

#### Custom RVO (Lines 353-363)
```csharp
if (s < 0.0f && distSqLine <= radiusSq)
{
    // Obstacle viewed obliquely so that left vertex defines velocity obstacle
    if (!obstacle1.IsConvex)
    {
        continue; // Ignore obstacle
    }

    // Both legs emanate from vertex1
    float leg1 = math.sqrt(distSq1 - radiusSq);
    leftLegDirection = new float2(relativePosition1.x * leg1 - relativePosition1.y * agent.Radius, 
                                  relativePosition1.x * agent.Radius + relativePosition1.y * leg1) / distSq1;
    rightLegDirection = new float2(relativePosition1.x * leg1 + relativePosition1.y * agent.Radius, 
                                   -relativePosition1.x * agent.Radius + relativePosition1.y * leg1) / distSq1;
}
```

**Status:** ✅ EQUIVALENT (we don't modify obstacle2, but we don't use it later either)

### Case 2: s > 1.0f && distSqLine <= radiusSq

#### RVO2-Unity (Lines 193-210)
```csharp
else if (s > 1.0f && distSqLine <= radiusSq)
{
    /*
     * Obstacle viewed obliquely so that
     * right vertex defines velocity obstacle.
     */
    if (!obstacle2.convex_)
    {
        /* Ignore obstacle. */
        continue;
    }

    obstacle1 = obstacle2;

    float leg2 = RVOMath.sqrt(distSq2 - radiusSq);
    leftLegDirection = new Vector2(relativePosition2.x() * leg2 - relativePosition2.y() * radius_, relativePosition2.x() * radius_ + relativePosition2.y() * leg2) / distSq2;
    rightLegDirection = new Vector2(relativePosition2.x() * leg2 + relativePosition2.y() * radius_, -relativePosition2.x() * radius_ + relativePosition2.y() * leg2) / distSq2;
}
```

**Key:** Sets `obstacle1 = obstacle2` - both legs emanate from vertex2!

#### Custom RVO (Lines 364-379)
```csharp
else if (s > 1.0f && distSqLine <= radiusSq)
{
    // Obstacle viewed obliquely so that right vertex defines velocity obstacle
    RVOObstacle nextObstacle = obstacle1.NextObstacle;
    bool isConvex = (nextObstacle != null) ? nextObstacle.IsConvex : true;
    
    if (!isConvex)
    {
        continue; // Ignore obstacle
    }

    // Both legs emanate from vertex2
    float leg2 = math.sqrt(distSq2 - radiusSq);
    leftLegDirection = new float2(relativePosition2.x * leg2 - relativePosition2.y * agent.Radius, 
                                  relativePosition2.x * agent.Radius + relativePosition2.y * leg2) / distSq2;
    rightLegDirection = new float2(relativePosition2.x * leg2 + relativePosition2.y * agent.Radius, 
                                   -relativePosition2.x * agent.Radius + relativePosition2.y * leg2) / distSq2;
}
```

**Status:** ✅ EQUIVALENT

---

## Foreign Leg Detection - THIS IS WHERE THE BUG IS!

After computing leg directions, RVO2-Unity checks for "foreign legs":

### RVO2-Unity (Lines 243-260)
```csharp
Obstacle leftNeighbor = obstacle1.previous_;

bool isLeftLegForeign = false;
bool isRightLegForeign = false;

if (obstacle1.convex_ && RVOMath.det(leftLegDirection, -leftNeighbor.direction_) >= 0.0f)
{
    /* Left leg points into obstacle. */
    leftLegDirection = -leftNeighbor.direction_;
    isLeftLegForeign = true;
}

if (obstacle2.convex_ && RVOMath.det(rightLegDirection, obstacle2.direction_) <= 0.0f)
{
    /* Right leg points into obstacle. */
    rightLegDirection = obstacle2.direction_;
    isRightLegForeign = true;
}
```

**Note:** After the oblique cases, `obstacle1` and `obstacle2` may have been MODIFIED!
- If oblique left: `obstacle2 = obstacle1` (both refer to left vertex)
- If oblique right: `obstacle1 = obstacle2` (both refer to right vertex)
- Otherwise: `obstacle1` = left vertex, `obstacle2` = right vertex

### Custom RVO (Lines 424-436)
```csharp
RVOObstacle prevObstacle = obstacle1.PrevObstacle;
RVOObstacle nextObstacle2 = obstacle1.NextObstacle;

bool isLeftLegForeign = false;
bool isRightLegForeign = false;

if (obstacle1.IsConvex && prevObstacle != null && det(leftLegDirection, -prevObstacle.Direction) >= 0.0f)
{
    // Left leg points into obstacle
    leftLegDirection = -prevObstacle.Direction;
    isLeftLegForeign = true;
}

if (nextObstacle2 != null && nextObstacle2.IsConvex && det(rightLegDirection, nextObstacle2.Direction) <= 0.0f)
{
    // Right leg points into obstacle
    rightLegDirection = nextObstacle2.Direction;
    isRightLegForeign = true;
}
```

**CRITICAL DIFFERENCE:** We use `obstacle1.PrevObstacle` and `obstacle1.NextObstacle`, but `obstacle1` is the ORIGINAL obstacle from the loop, NOT the potentially modified one!

In RVO2-Unity:
- After oblique cases, `obstacle1` and `obstacle2` may point to the SAME vertex
- `leftNeighbor = obstacle1.previous_` gets the CORRECT previous neighbor for the vertex represented by (modified) `obstacle1`
- Similarly for `obstacle2`

In Custom RVO:
- We always use the ORIGINAL `obstacle1` from the loop
- After oblique cases, we should be using different obstacles!

**THIS IS THE BUG!**
