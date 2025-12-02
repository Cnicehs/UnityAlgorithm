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

## Solutions and RVO2-Unity Mechanism

We explored two solutions to resolve this issue. The core problem is preventing the "back-facing" edges from generating invalid "inside" constraints.

### Solution 1: Explicit Back-Face Culling (Initial Approach)

Directly check if the edge is back-facing using the determinant, and skip it if so.

**Implementation:**
```csharp
float2 obstacleVec = vertex2 - vertex1;
// det < 0 means Agent is to the Left (Inside/Back) of the edge
if (det(obstacleVec, relativePosition1) < 0) {
    continue; // Skip back-facing edge
}
```

**Pros:**
- Simple and robust.
- Works regardless of obstacle processing order (no sorting needed).
- Efficient (avoids processing invalid edges entirely).

**Cons:**
- Differs from the reference RVO2-Unity implementation.

### Solution 2: Sorting + Implicit Culling (RVO2-Unity Approach)

This is the solution we finally adopted to align with the standard RVO2 library. RVO2-Unity does not use explicit culling; instead, it relies on processing order.

**RVO2-Unity Mechanism:**
1.  **Strict Distance Sorting**: RVO2-Unity sorts all obstacle neighbors by distance (closest to farthest) during the neighbor query phase.
2.  **"Already Covered" Check**: It checks if a new obstacle's Velocity Obstacle (VO) is already covered by existing constraints.

**Why it works:**
- **Sorting** ensures the **Front-Facing Edge** (Closest) is processed *before* the **Back-Facing Edge** (Farthest).
- The Front-Face generates a valid constraint first.
- When the algorithm reaches the Back-Face, it is geometrically "behind" the Front-Face. Its VO is fully covered by the Front-Face's constraint.
- The `alreadyCovered` check detects this redundancy and **skips** the Back-Face.

**Our Implementation (Matching RVO2):**
In `RVOMath.cs`, we implemented:
1.  **Sorting**: We sort the `obstacles` list by distance (`distSqPointLineSegment`) before iteration.
2.  **Restored `alreadyCovered`**: We restored the check that was previously removed.

```csharp
// 1. Sort by distance
obstacles.Sort((a, b) => distSq(a).CompareTo(distSq(b)));

// 2. Iterate and check coverage
for (each obstacle) {
   if (IsAlreadyCovered(obstacle)) continue;
   // ... process ...
}
```

### Conclusion on Sorting Issues
This issue highlights a critical dependency on **Sorting** in the RVO algorithm.
- **For Obstacles**: Sorting is required for the `alreadyCovered` optimization to work correctly as an implicit Back-Face Culler. Without sorting, we might process Back-Face first -> LP Failure.
- **For Agents**: (Note for review) Sorting is also typically required for Neighbor Selection (picking the K closest neighbors). If neighbors are not sorted, we might pick distant agents and ignore close ones, leading to collisions. We must ensure all Neighbor Queries return sorted results or are sorted before use.
