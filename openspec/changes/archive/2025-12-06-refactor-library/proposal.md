# Change: Refactor to Algorithm Library

## Why
The current codebase has scripts loosely organized in `Assets/Scripts`. To evolve this project into a reusable, general-purpose algorithm library for Unity, we need a modular structure. This separation will allow easier maintenance, clearer dependencies, and the ability to compare "Default" (OOP/Reference) vs "SIMD" (Optimized) implementations side-by-side.

## What Changes
- **Directory Structure Refactor**:
  - `Assets/Scripts/RVO2-Unity` (Legacy) -> `Assets/Scripts/Legacy/RVO2-Unity`
  - `Assets/Scripts/{RVO, Pathfinding, Spatial}` -> `Assets/Scripts/Core/{RVO, Pathfinding, Spatial}`
  - `Assets/Scripts/Demo` -> `Assets/Scripts/Demos`
- **Module Standardization**:
  - Each Core module (RVO, Pathfinding, Spatial) will contain both Default and SIMD implementations.
- **Demo Consolidation**:
  - Consolidate demos into 4 main categories: Spatial, Pathfinding, RVO, and Combined.
  - Add `OnGUI` performance metrics to all comparison demos.

## Impact
- **Breaking**: Moving scripts will break existing Scenes (`sampleScene.unity`, etc.) unless GUIDs are preserved (Meta files moved with scripts). I will attempt to move `.meta` files to preserve references, but some manual scene fixup might be needed.
- **Affected Systems**: All subsystems.
