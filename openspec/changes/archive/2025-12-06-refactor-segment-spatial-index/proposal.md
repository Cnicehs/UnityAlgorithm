# Change: Refactor Segment Spatial Index

## Why
The current `Core/Spatial/Segment` implementation only supports Managed objects (`RVOObstacle`), which prevents SIMD optimizations.
RVO has a SIMD implementation (`SIMDRVO`) but it currently relies on linear scanning of obstacles or managing its own native arrays, lacking a proper spatial index for O(log N) queries.
To fully utilize SIMD performance and maintain architectural consistency, we need to split Segment Index into Default (Managed) and SIMD (Native/Burst) versions.

## What Changes
- **Directory Structure**:
    - Move `SegmentKdTreeIndex` and `SegmentLinearIndex` to `Default/`.
    - Create `SIMD/` directory.
- **Interfaces**:
    - Introduce `ISIMDSegmentSpatialIndex` for Burst-compatible obstacle indices.
- **New Implementations**:
    - `DefaultSegmentKdTreeIndex` (renamed).
    - `SIMDSegmentKdTreeIndex`: A Burst-compatible KD-Tree for obstacles.
- **Integration**:
    - Update `SIMDRVOSimulator` to use `SIMDSegmentKdTreeIndex` for obstacle management.
    - Update `SIMDRVO` to query the KD-Tree structure instead of linear scanning.

## Impact
- **Affected Specs**: Spatial, RVO
- **Affected Code**:
    - `Assets/Scripts/Core/Spatial/Segment/*`
    - `Assets/Scripts/Core/RVO/SIMD/SIMDRVOSimulator.cs`
    - `Assets/Scripts/Core/RVO/SIMD/SIMDRVO.cs`
- **Breaking Changes**:
    - Renaming of `SegmentKdTreeIndex` to `DefaultSegmentKdTreeIndex`.
    - `SIMDRVOSimulator` will require `SIMDSegmentKdTreeIndex` or fallback logic.
