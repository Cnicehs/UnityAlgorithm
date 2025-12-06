# Change: Refine Library Structure

## Why
The initial refactor moved scripts into modules but did not separate "Default" (Reference/OOP) and "SIMD" (Optimized) implementations clearly. Additionally, `IObstacleSpatialIndex` is currently in `Core/Spatial` but is tightly coupled to RVO logic, making the dependency graph unclear.

## What Changes
- **Sub-module Structure**:
  - `Assets/Scripts/Core/{Pathfinding, RVO, Spatial}` will each have `Default` and `SIMD` subdirectories.
  - Scripts will be moved to their respective subdirectories based on their implementation type.
- **Relocation of RVO Dependencies**:
  - `IObstacleSpatialIndex` and its implementations (`ObstacleKdTreeIndex`, etc.) will move from `Core/Spatial` to `Core/RVO/Spatial` (or just `Core/RVO` if simpler).
  - Rationale: Only RVO uses obstacle indexing in this specific way (edges, convexity).

## Impact
- **Breaking**: File locations change again. Meta files must be moved to preserve references.
- **Code**: Namespaces might be good to add now, but for this step, just physical structure.
