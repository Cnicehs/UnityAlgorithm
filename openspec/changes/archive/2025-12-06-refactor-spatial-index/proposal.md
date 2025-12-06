# Refactor Spatial Index: Unify RVO Obstacle Index and Fix QueryKNearest Performance

## Why

This proposal addresses two issues in the `Core/Spatial` subsystem:

1. **RVO Obstacle Spatial Index Duplication**: The RVO module has its own `IObstacleSpatialIndex` interface and implementations (`ObstacleKdTreeIndex`, `SimpleObstacleIndex`) that were stored separately in `Core/RVO/Spatial/`, creating architectural fragmentation.

2. **QueryKNearest Performance**: All 8 implementations of `ISpatialIndex` (4 Default + 4 SIMD) incorrectly delegate `QueryKNearest()` to `QueryKNearestSorted()`, causing unnecessary sorting overhead when callers don't need sorted results.

## What Changes

### Part 1: Fix QueryKNearest to Avoid Unnecessary Sorting

Implement proper unsorted KNN search in all 8 implementations:

**For tree-based structures (KDTree, BVH, QuadTree):**
- Use a max-heap pattern instead of sorted list for candidate management
- Skip the final sorting step
- Return results in arbitrary order

**For grid-based structures (SpatialGrid, HashGrid):**
- Expand search rings until K candidates are found
- Don't sort candidates before returning

### Part 2: Reorganize RVO Obstacle Index (Option B: Keep Separate)

Move RVO obstacle spatial index files to `Core/Spatial/Segment/`:
- `IObstacleSpatialIndex` → `ISegmentSpatialIndex`
- `ObstacleKdTreeIndex` → `SegmentKdTreeIndex`
- `SimpleObstacleIndex` → `SegmentLinearIndex`

This preserves RVO's segment-splitting logic without forcing point-based algorithms to handle segments.

## Files Changed

### QueryKNearest Fix (8 files)
- `Core/Spatial/Default/KDTreeIndex.cs`
- `Core/Spatial/Default/BVHIndex.cs`
- `Core/Spatial/Default/QuadTreeIndex.cs`
- `Core/Spatial/Default/SpatialGridIndex.cs`
- `Core/Spatial/SIMD/SIMDKDTreeIndex.cs`
- `Core/Spatial/SIMD/SIMDBVHIndex.cs`
- `Core/Spatial/SIMD/SIMDQuadTreeIndex.cs`
- `Core/Spatial/SIMD/SIMDHashGridIndex.cs`

### RVO Obstacle Reorganization
- Created `Core/Spatial/Segment/` directory
- Moved and renamed 3 files from `Core/RVO/Spatial/`
- Updated `RVOSimulator.cs` to reference new names
