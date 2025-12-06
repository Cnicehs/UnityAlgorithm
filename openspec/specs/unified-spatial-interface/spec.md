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

Line segment spatial indices (used for RVO obstacles) MUST be located in `Core/Spatial/Segment/` to distinguish from point-based indices.

#### Scenario: Segment Index Found in Correct Location
**Given** a need to query line segments (not points)  
**When** looking for spatial index implementations  
**Then** segment-specific implementations are found in `Core/Spatial/Segment/`  
**And** point-specific implementations remain in `Core/Spatial/Default/` and `Core/Spatial/SIMD/`

