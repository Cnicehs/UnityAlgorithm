# Change: Create Unified Comparison Demo

## Why
Currently, there are multiple separate demo scripts for testing different aspects (Pathfinding, RVO). We need a single, flexible demo to compare performance and correctness across different algorithms (Default vs SIMD) and modes (Pathfinding, RVO, Combined), specifically using the new Entity system.

## What Changes
- Create `UnifiedComparisonDemo` to manage simulation loops and rendering.
- Update `PathfindingUpdateSystem` to support actual A* pathfinding (SIMD and Default).
- Create `LegacyRVOSystem` to bridge the Entity system with the object-based `RVOSimulator` for baseline comparison.
- Implement switching logic for algorithms and modes.
- Add OnGUI metrics and Gizmos visualization.

## Impact
- **Added Files**:
    - `Assets/Scripts/Demos/UnifiedComparisonDemo.cs`
    - `Assets/Scripts/Core/Entity/Systems/LegacyRVOSystem.cs`
- **Modified Files**:
    - `Assets/Scripts/Core/Entity/Systems/PathfindingUpdateSystem.cs`
- **Affected Specs**:
    - `demos` (New capability)
