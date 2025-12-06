# Change: Refine RVO Spatial Integration

## Why
The current RVO implementation performs redundant sorting of neighbors, and the `RVOMath` library is shared between Default and SIMD without clear separation. Additionally, RVO should leverage `Core/Spatial` more effectively by offloading neighbor sorting to the spatial index.

## What Changes
- **Directory Structure**:
  - Move `RVOMath.cs` to `Assets/Scripts/Core/RVO/Default/`.
- **Spatial Interface**:
  - Add `QueryKNearestSorted(Vector2 position, int k, float radius, List<int> results)` to `ISpatialIndex`.
  - This method returns up to `k` neighbors within `radius`, sorted by distance (nearest first).
- **RVO Optimization**:
  - Update `RVOSimulator` (Default) to use `QueryKNearestSorted` instead of `QueryRadius` + manual sort.
- **Implementations**:
  - Update all `ISpatialIndex` implementations (`KDTree`, `QuadTree`, `Grid`, etc.) to support the new method.

## Impact
- **Performance**: Improved performance for RVO by potentially using optimized spatial query sorting (e.g. KDTree traversal order) instead of full sort.
- **Code**: Interface change requires updating all spatial indices.
