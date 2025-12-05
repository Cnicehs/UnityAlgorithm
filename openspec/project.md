# Project Context

## Purpose
This project is a high-performance Unity implementation of navigation and collision avoidance algorithms tailored for MOBA (Multiplayer Online Battle Arena) games. It focuses on integrating global pathfinding (A*) with local avoidance (RVO2/ORCA), optimized using SIMD and efficient spatial indexing data structures. The goal is to support a large number of agents with minimal performance overhead.

## Tech Stack
- **Engine:** Unity 6000.1.0f1
- **Language:** C#
- **Core Libraries:**
  - `Unity.Mathematics` (for SIMD `float2` and optimized math)
  - `Unity.Collections` (implied for native collections)
  - `Unity.Burst` (target optimization)

## Project Conventions

### Code Style
- **Naming:** Standard C# PascalCase for methods/classes, camelCase for local variables.
- **Math Types:** Prefer `Unity.Mathematics.float2` over `UnityEngine.Vector2` for SIMD compatibility in core algorithms.
- **Algorithmic Purity:** Core algorithms (e.g., `RVOMath`) should be stateless `static` classes containing pure functions. State is managed by simulator classes (`RVOSimulator`).
- **Separation of Concerns:** Clear separation between:
  - Data (`RVOAgent`, `RVOObstacle`)
  - Logic/Simulation (`RVOSimulator`, `AStarPathfinder`)
  - Spatial Indexing (`SpatialIndexManager`, `QuadTreeIndex`)
  - Visualization/Demo (`RVO2Demo`)

### Architecture Patterns
- **Simulation Loop:** Explicit `Step(dt)` phase:
  1. **Plan:** Calculate new velocities for all agents (RVO/ORCA).
  2. **Move:** Apply velocities to update positions.
  3. **Index:** Update spatial structures with new positions.
- **Spatial Indexing:** Abstracted via `ISpatialIndex` interface, allowing hot-swapping of implementations (QuadTree, KdTree, Grid, etc.).
- **Data-Oriented Design:** Preference for Structs and NativeArrays where possible to leverage CPU cache and SIMD (gradual migration from OOP).

### Testing Strategy
- **Visual Verification:** Use Demo scenes in `Assets/Scenes/` and scripts in `Assets/Scripts/Demo/` to visually verify behavior (e.g., `RVO2Demo`, `AStarRVOCombinedDemo`).
- **Benchmarking:** Use scripts in `Assets/Scripts/Performance/` to measure execution time and throughput.

### Git Workflow
- Standard feature-branch workflow.
- Commit messages should be descriptive.

## Domain Context
- **RVO (Reciprocal Velocity Obstacles):** A decentralized collision avoidance algorithm. Agents observe neighbors and obstacles to construct ORCA (Optimal Reciprocal Collision Avoidance) lines, solving a linear programming problem to find the optimal collision-free velocity.
- **Pathfinding:** A* algorithm on a GridMap for global route planning.
- **Spatial Partitioning:** Critical for performance. Reduces $O(N^2)$ neighbor queries to $O(N \log N)$ or $O(N)$.

## Important Constraints
- **Performance:** The simulation must handle hundreds of agents at 60 FPS.
- **Memory Management:** Zero Garbage Collection (GC) allocations allowed inside the main simulation loop (`Step`). Reuse collections and objects.
- **Determinism:** Logic should be sequential and predictable within the frame update.

## External Dependencies
- Unity standard packages (Mathematics, Burst, Collections).
