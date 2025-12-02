# Second Bug Found: Cutoff Center Calculation

## The Problem

After fixing the foreign leg detection, cutoff centers are still calculated using the ORIGINAL `vertex1` and `vertex2`, but they should use the vertices from `leftVertexObstacle` and `rightVertexObstacle`!

## RVO2-Unity Behavior

```csharp
// After oblique cases, obstacle1 and obstacle2 may have been modified

// Compute cut-off centers using MODIFIED obstacles
Vector2 leftCutOff = invTimeHorizonObst * (obstacle1.point_ - position_);
Vector2 rightCutOff = invTimeHorizonObst * (obstacle2.point_ - position_);
```

**Key:** Uses `obstacle1.point_` and `obstacle2.point_`, which refer to the vertices where the legs actually emanate from!

## Custom RVO Bug

```csharp
// At loop start:
float2 vertex1 = obstacle1.Point1;
float2 vertex2 = obstacle1.Point2;

// ... oblique cases may set leftVertexObstacle and rightVertexObstacle ...

// But cutoff calculation uses ORIGINAL vertices!
float2 leftCutOff = invTimeHorizonObst * (vertex1 - agent.Position);  // ❌ WRONG
float2 rightCutOff = invTimeHorizonObst * (vertex2 - agent.Position);  // ❌ WRONG
```

## Example: Oblique Right Case

Agent approaching from the right of edge1 (BL→BR):

**RVO2-Unity:**
```
obstacle1 = BL (vertex)
obstacle2 = BR (vertex)

s > 1.0f && distSqLine <= radiusSq → oblique right

obstacle1 = obstacle2  // NOW obstacle1 = BR

// Both legs from BR
leftCutOff = invTimeHorizonObst * (BR - agentPos)   ✓ CORRECT
rightCutOff = invTimeHorizonObst * (BR - agentPos)  ✓ CORRECT
```

**Custom RVO:**
```
vertex1 = BL
vertex2 = BR

s > 1.0f && distSqLine <= radiusSq → oblique right

leftVertexObstacle = nextObstacle (BR's obstacle)
rightVertexObstacle = nextObstacle (BR's obstacle)

// But cutoff uses original vertices!
leftCutOff = invTimeHorizonObst * (BL - agentPos)   ❌ SHOULD BE BR
rightCutOff = invTimeHorizonObst * (BR - agentPos)  ✓ This one correct
```

**Impact:** Wrong leftCutOff causes incorrect velocity projection!

## The Fix

Cutoff centers should use the actual leg emanation vertices:

```csharp
// Compute cut-off centers using the correct vertices
float2 leftCutOff = invTimeHorizonObst * (leftVertexObstacle.Point1 - agent.Position);
float2 rightCutOff = invTimeHorizonObst * (rightVertexObstacle.Point1 - agent.Position);
```

This ensures both cutoff centers are at the vertices where the legs actually emanate from.
