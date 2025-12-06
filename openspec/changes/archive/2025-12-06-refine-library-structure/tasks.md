## 1. Directory Structure Refinement
- [x] 1.1 Create `Assets/Scripts/Core/Pathfinding/Default` and `Assets/Scripts/Core/Pathfinding/SIMD`.
- [x] 1.2 Move Pathfinding scripts to respective folders.
- [x] 1.3 Create `Assets/Scripts/Core/RVO/Default` and `Assets/Scripts/Core/RVO/SIMD`.
- [x] 1.4 Move RVO scripts to respective folders.
- [x] 1.5 Create `Assets/Scripts/Core/Spatial/Default` and `Assets/Scripts/Core/Spatial/SIMD`.
- [x] 1.6 Move Spatial scripts to respective folders.

## 2. Refactor Obstacle Spatial Index
- [x] 2.1 Create `Assets/Scripts/Core/RVO/Spatial`.
- [x] 2.2 Move `IObstacleSpatialIndex.cs` and obstacle implementations (`ObstacleKdTreeIndex.cs`, `SimpleObstacleIndex.cs`) from `Core/Spatial` to `Core/RVO/Spatial`.
