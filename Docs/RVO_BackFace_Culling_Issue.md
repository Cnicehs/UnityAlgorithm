# RVO Obstacle Penetration Issue: Back-Face Culling

## Problem Description
Agents were observed penetrating static obstacles (specifically a box) when approaching from the top (`y=10` to `y=-10`), but not from the bottom.

### Symptoms
- Agents moving downwards would pass through the top edge of the obstacle.
- Agents moving upwards would correctly avoid the bottom edge.
- Debug logs showed that the Linear Programming step was failing (`lineFail=0`) for the top-to-bottom case, meaning the very first constraint added was considered impossible to satisfy.

### Root Cause
The issue was caused by the agent processing **all** edges of the obstacle, including "back-facing" edges (edges pointing away from the agent).

1. **Scenario**: Agent is above the box.
2. **Back-Facing Edge**: The bottom edge of the box is "back-facing" relative to the agent.
3. **Invalid Constraint**: The agent is technically "inside" the half-plane defined by the bottom edge (since the box interior is "up" relative to the bottom edge). This generated a constraint requiring the agent to be "inside" the obstacle relative to that edge.
4. **LP Failure**: This "inside" constraint, combined with the agent's position and velocity, resulted in a constraint line that was often outside the maximum speed circle (impossible to satisfy).
5. **Consequence**: `linearProgram2` fails immediately when it encounters an impossible constraint. This failure caused the valid constraint from the "front-facing" top edge (which might have been processed later or overridden by the failure logic) to be ignored.

## Solution: Back-Face Culling
We implemented a **Back-Face Culling** check in `RVOMath.ConstructObstacleORCALines`.

### Logic
An edge is considered "back-facing" if the agent is to the **Left** of the edge vector (assuming Counter-Clockwise winding where "Left" is "Inside" the obstacle).

We use the determinant of the edge vector and the vector from the agent to the vertex to determine this:
- `obstacleVec = vertex2 - vertex1`
- `relativePosition1 = vertex1 - agent.Position` (Vector from Agent to Vertex)
- `det(obstacleVec, relativePosition1)`

If `det < 0`, the agent is to the "Left" (Inside/Back) of the edge vector. These edges are skipped.

### Code Implementation
```csharp
// In RVOMath.cs

float2 obstacleVec = vertex2 - vertex1;

// Back-face culling: If the agent is to the left of the edge (inside the half-plane defined by the edge),
// we skip it. RVO assumes CCW winding, so "Left" is "Inside".
// det(obstacleVec, relativePosition1) < 0 means Agent is Left (Back-facing).
if (det(obstacleVec, relativePosition1) < 0)
{
    continue;
}
```

## Result
With this fix, only "front-facing" edges generate constraints. The impossible constraints from back edges are discarded, allowing the Linear Programming solver to successfully find a valid velocity that respects the actual collision boundaries (the front-facing edges).
