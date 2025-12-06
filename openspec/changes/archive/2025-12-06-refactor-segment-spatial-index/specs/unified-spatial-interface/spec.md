## MODIFIED Requirements
### Requirement: Segment Spatial Index MUST Be Organized in Dedicated Directory
Line segment spatial indices (used for RVO obstacles) MUST be located in `Core/Spatial/Segment/`, further organized into `Default/` (Managed) and `SIMD/` (Native/Burst) subdirectories.

#### Scenario: Segment Index Found in Correct Location
- **WHEN** looking for spatial index implementations
- **THEN** Managed implementations are found in `Core/Spatial/Segment/Default/`
- **AND** SIMD implementations are found in `Core/Spatial/Segment/SIMD/`

## ADDED Requirements
### Requirement: SIMD Segment Spatial Index Support
The system MUST provide a Burst-compatible spatial index for line segments (RVO obstacles) to support high-performance SIMD RVO.

#### Scenario: SIMD RVO Uses SIMD Spatial Index
- **WHEN** `SIMDRVOSimulator` is initialized
- **THEN** it uses `SIMDSegmentKdTreeIndex` (or compatible) for obstacle queries
- **AND** queries are performed using Burst-compatible data structures (e.g. `NativeArray` based KD-Tree)
