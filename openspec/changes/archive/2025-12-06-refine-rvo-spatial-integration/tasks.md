## 1. Directory Structure
- [x] 1.1 Move `Assets/Scripts/Core/RVO/RVOMath.cs` to `Assets/Scripts/Core/RVO/Default/RVOMath.cs`.

## 2. Spatial Interface Update
- [x] 2.1 Update `ISpatialIndex.cs` to include `QueryKNearestSorted`.
- [x] 2.2 Update `SpatialIndexManager.cs` to expose `QueryKNearestSorted`.

## 3. Spatial Implementations Update
- [x] 3.1 Update `KDTreeIndex.cs` (Default).
- [x] 3.2 Update `QuadTreeIndex.cs` (Default).
- [x] 3.3 Update `BVHIndex.cs` (Default).
- [x] 3.4 Update `SpatialGridIndex.cs` (Default).
- [x] 3.5 Update `SIMDQuadTreeIndex.cs` (SIMD).
- [x] 3.6 Update `SIMDKDTreeIndex.cs` (SIMD).
- [x] 3.7 Update `SIMDBVHIndex.cs` (SIMD).
- [x] 3.8 Update `SIMDHashGridIndex.cs` (SIMD).

## 4. RVO Integration
- [x] 4.1 Update `RVOSimulator.cs` to use `SpatialIndexManager.Instance.QueryKNearestSorted` instead of manual sort.
