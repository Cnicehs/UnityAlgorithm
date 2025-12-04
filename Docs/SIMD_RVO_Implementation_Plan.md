# SIMD RVO Implementation Plan

## Objective
Accelerate RVO2 performance by porting the entire calculation pipeline (including Obstacle Avoidance) to **Burst-compiled SIMD code**, maximizing single-threaded performance as requested.

## Current State Analysis
- **Standard RVO (`RVOSimulator`)**: Fully featured (Agents + Obstacles) but slow due to managed code overhead, GC, and lack of SIMD.
- **SIMD RVO (`SIMDRVOSimulator`)**: Fast but incomplete. Currently **ignores obstacles** (only handles Agent-Agent avoidance).
- **Bottlenecks**:
    - Neighbor Query (Fixed via `SIMDQuadTreeIndex.QueryNeighborsBatch`).
    - RVO Compute: Currently sequential C# in Standard, or incomplete in SIMD.

## Implementation Steps

### 1. Data Structure Design (`SIMDRVO.cs`)
Define unmanaged structs to represent RVO data for Burst compatibility.

```csharp
public struct ObstacleData
{
    public float2 Point1;
    public float2 Point2;
    public float2 Direction;
    public int NextObstacleIdx; // Index in the NativeArray
    public int PrevObstacleIdx; // Index in the NativeArray
    public bool IsConvex;
}
```

### 2. SIMD Library Expansion (`SIMDRVO.cs`)
Port the core `ConstructObstacleORCALines` logic from `RVOMath` to `SIMDRVO` (unsafe/pointer-based).

**Key Algorithms to Port:**
- **Obstacle Sorting**: Implement a fast sort (e.g., Insertion Sort or QuickSort) for obstacles relative to agent position inside the Burst job.
    - *Why inside?* Sorting depends on `dist(Agent, Obstacle)`, which is unique per agent.
- **Already Covered Check**: Port the logic to skip obstacles hidden behind others.
- **Linear Constraints**: Ensure `linearProgram2` accepts lines from both obstacles and agents.

**New Function Signature:**
```csharp
public static void ComputeRVOVelocities(
    ..., 
    ObstacleData* obstacles, int obstacleCount, // NEW: Obstacle Data
    ...
)
```

### 3. Simulator Integration (`SIMDRVOSimulator.cs`)
Update the simulator to manage the lifecycle of obstacle data.

- **Data Management**:
    - Maintain `NativeArray<ObstacleData> _nativeObstacles`.
    - Rebuild this array whenever obstacles change (`ProcessObstacles`).
    - Map object references (`NextObstacle`) to indices (`NextObstacleIdx`).
- **Execution Loop**:
    - Pass `_nativeObstacles` to `SIMDRVO.ComputeRVOVelocities`.

### 4. Integration with Demo
- Switch `AStarRVOCombinedDemo` to use `SIMDRVOSimulator` instead of `RVOSimulator`.
- Ensure `SIMDRVOSimulator` exposes APIs to Add/Remove obstacles matching the Standard simulator.

## Performance Considerations
- **Sorting Overhead**: Sorting 50-100 obstacles per agent for 1000 agents is intensive ($10^5$ operations). 
    - *Optimization*: Spatial Hashing for obstacles? RVO2 uses a KdTree for obstacles. We can implement a static `SIMDObstacleGrid` or simply use brute-force sort for reasonable obstacle counts (<100). For high counts, we *must* implement a spatial structure for obstacles in SIMD.
- **Memory**: Pre-allocate `ORCALine` buffers (stackalloc in Burst is fast but limited size). Max lines constraint needs careful handling.

## Execution Plan
1.  **Define `ObstacleData`** in `SIMDRVO.cs`.
2.  **Implement `ConstructObstacleLines`** in `SIMDRVO.cs` (porting logic from `RVOMath`).
3.  **Update `SIMDRVOSimulator`** to build and pass `NativeArray<ObstacleData>`.
4.  **Verify** using `AStarRVOCombinedDemo` (switch to SIMD).
