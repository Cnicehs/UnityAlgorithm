## 1. Legacy Code Management
- [x] 1.1 Create `Assets/Scripts/Legacy` directory.
- [x] 1.2 Move `Assets/Scripts/RVO2-Unity` to `Assets/Scripts/Legacy/RVO2-Unity` (including meta files).
- [x] 1.3 Add `README.md` to `Assets/Scripts/Legacy/RVO2-Unity` explaining its purpose as reference code.

## 2. Core Module Refactoring
- [x] 2.1 Create `Assets/Scripts/Core` directory.
- [x] 2.2 Create `Assets/Scripts/Core/RVO`, `Assets/Scripts/Core/Pathfinding`, `Assets/Scripts/Core/Spatial`.
- [x] 2.3 Move existing RVO scripts to `Core/RVO` (including `SIMD` variants and meta files).
- [x] 2.4 Move existing Pathfinding scripts to `Core/Pathfinding` (including `SIMD` variants and meta files).
- [x] 2.5 Move existing Spatial scripts to `Core/Spatial` (including `SIMD` variants and meta files).
- [x] 2.6 Move `Assets/Scripts/Utils` to `Assets/Scripts/Core/Utils`.
- [x] 2.7 Verify all scripts are accounted for.

## 3. Demo Consolidation
- [x] 3.1 Create `Assets/Scripts/Demos` directory (rename from `Assets/Scripts/Demo`).
- [x] 3.2 Implement `SpatialComparisonDemo` with OnGUI metrics.
- [x] 3.3 Implement `PathfindingComparisonDemo` with OnGUI metrics.
- [x] 3.4 Implement `RVOComparisonDemo` with OnGUI metrics (update `RVO2Demo` logic).
- [x] 3.5 Implement `CombinedComparisonDemo` with OnGUI metrics.
