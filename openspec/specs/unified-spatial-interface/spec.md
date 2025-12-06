# unified-spatial-interface Specification

## Purpose
TBD - created by archiving change refactor-spatial-index. Update Purpose after archive.
## Requirements
### Requirement: QueryKNearest MUST Not Sort Results

The `QueryKNearest` method MUST return results in arbitrary order to avoid unnecessary sorting overhead. Callers who need sorted results MUST use `QueryKNearestSorted` instead.

#### Scenario: QueryKNearest Returns Unsorted K-Nearest Results
**Given** a spatial index built with N points  
**When** `QueryKNearest(position, k, results)` is called  
**Then** `results` contains exactly K indices (or less if N < K)  
**And** each index corresponds to one of the K nearest points to `position`  
**And** the order of indices in `results` is NOT guaranteed to be sorted by distance  

#### Scenario: QueryKNearestSorted Returns Distance-Sorted Results
**Given** a spatial index built with N points  
**When** `QueryKNearestSorted(position, k, radius, results)` is called  
**Then** `results` contains at most K indices within the specified radius  
**And** `results` is sorted by distance to `position` (nearest first)

### Requirement: QueryKNearest and QueryKNearestSorted MUST Return Equivalent Sets

Both methods MUST return the same logical set of nearest neighbors (when radius is not limiting).

#### Scenario: Sorted Version is Superset of Unsorted
**Given** a spatial index built with N points  
**When** `QueryKNearest(position, k, unsortedResults)` is called  
**And** `QueryKNearestSorted(position, k, float.MaxValue, sortedResults)` is called  
**Then** `unsortedResults` and `sortedResults` contain the same indices (possibly in different order)

### Requirement: Segment Spatial Index MUST Be Organized in Dedicated Directory
Line segment spatial indices (used for RVO obstacles) MUST be located in `Core/Spatial/Segment/`, further organized into `Default/` (Managed) and `SIMD/` (Native/Burst) subdirectories.

#### Scenario: Segment Index Found in Correct Location
- **WHEN** looking for spatial index implementations
- **THEN** Managed implementations are found in `Core/Spatial/Segment/Default/`
- **AND** SIMD implementations are found in `Core/Spatial/Segment/SIMD/`

### Requirement: SIMD Segment Spatial Index Support
The system MUST provide a Burst-compatible spatial index for line segments (RVO obstacles) to support high-performance SIMD RVO.

#### Scenario: SIMD RVO Uses SIMD Spatial Index
- **WHEN** `SIMDRVOSimulator` is initialized
- **THEN** it uses `SIMDSegmentKdTreeIndex` (or compatible) for obstacle queries
- **AND** queries are performed using Burst-compatible data structures (e.g. `NativeArray` based KD-Tree)

