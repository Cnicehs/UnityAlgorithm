## 1. Refactor Structure
- [x] 1.1 Move `SegmentKdTreeIndex.cs` to `Assets/Scripts/Core/Spatial/Segment/Default/DefaultSegmentKdTreeIndex.cs` and rename class.
- [x] 1.2 Move `SegmentLinearIndex.cs` to `Assets/Scripts/Core/Spatial/Segment/Default/DefaultSegmentLinearIndex.cs` and rename class.
- [x] 1.3 Create `ISIMDSegmentSpatialIndex.cs` in `Assets/Scripts/Core/Spatial/Segment/SIMD/`.

## 2. Implement SIMD Segment Index
- [x] 2.1 Implement `SIMDSegmentKdTreeIndex.cs` using Burst/NativeArrays.
- [x] 2.2 Implement KD-Tree build logic for segments (adapting RVO2 logic to flat arrays).

## 3. Update RVO Integration
- [x] 3.1 Update `SIMDRVOSimulator` to build/maintain `SIMDSegmentKdTreeIndex`.
- [x] 3.2 Update `SIMDRVO.ComputeRVOVelocities` to accept KD-Tree data.
- [x] 3.3 Implement `SIMDRVO.QueryObstacleNeighbors` (or inline logic) using the tree.

## 4. Cleanup and Verification
- [ ] 4.1 Update `SpatialIndexManager` if necessary (though Segment index is usually specific to RVO).
- [ ] 4.2 Verify RVO demos work with SIMD mode.
