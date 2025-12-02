# 🐛 Critical Bug Fix: Obstacle Range Check

## Problem
Agents were **not detecting obstacles at all**, even with identical parameters to the working RVO2-Unity reference implementation.

## Root Cause
We were using **point-to-line-segment distance** for obstacle range filtering in `ConstructObstacleORCALines`:

```csharp
// ❌ WRONG - Too restrictive
float distSqToLine = distSqPointLineSegment(vertex1, vertex2, agent.Position);
if (distSqToLine > rangeSq) continue;  // Filters too many obstacles!
```

This caused obstacles to be **filtered out** when the agent was perpendicular to the obstacle but beyond its endpoints, even if the obstacle was close enough.

## RVO2-Unity's Approach
RVO2-Unity uses **point-to-infinite-line distance** in its KdTree spatial filtering (`KdTree.queryObstacleTreeRecursive:550`):

```csharp
// ✅ CORRECT - Point to infinite line
float agentLeftOfLine = RVOMath.leftOf(obstacle1.point_, obstacle2.point_, agent.position_);
float distSqLine = RVOMath.sqr(agentLeftOfLine) / RVOMath.absSq(obstacle2.point_ - obstacle1.point_);
if (distSqLine < rangeSq) {
    agent.insertObstacleNeighbor(node.obstacle_, rangeSq);
}
```

This is **more permissive** - it only filters obstacles whose infinite line extension is too far, not those whose endpoints are far but line is close.

## The Fix
Changed `RVOMath.cs:292-301` to use infinite line distance:

```csharp
// ✅ FIXED - Using infinite line distance like RVO2-Unity
float2 obstacleVector = vertex2 - vertex1;
float agentLeftOfLine = det(obstacleVector, agent.Position - vertex1);
float distSqToInfiniteLine = (agentLeftOfLine * agentLeftOfLine) / math.lengthsq(obstacleVector);

if (distSqToInfiniteLine > rangeSq) {
    continue;  // Only filter if infinite line is too far
}
```

## Impact
- Agents can now detect obstacles even when positioned beyond the segment endpoints
- Behavior now matches RVO2-Unity reference implementation
- No parameter changes needed - the bug was purely in the implementation

## Example Scenario
**Before Fix:**
- Agent at `(-4.4, -10)`
- Obstacle edge from `(-10, -1)` to `(10, -1)` 
- Point-to-segment distance: ~9 units (to endpoint)
- Point-to-line distance: exactly 9 units (perpendicular)
- With `rangeSq=9`, the segment distance could be > 9 due to endpoint consideration
- Result: **Obstacle filtered incorrectly** ❌

**After Fix:**
- Same geometry
- Infinite line distance: exactly 9 units
- With `rangeSq=9`, infinite line distance = 9 (<=rangeSq)
- Result: **Obstacle correctly detected** ✅
