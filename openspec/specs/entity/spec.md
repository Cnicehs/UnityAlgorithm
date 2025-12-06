# entity Specification

## Purpose
TBD - created by archiving change add-entity-system. Update Purpose after archive.
## Requirements
### Requirement: Data-Oriented Component Storage
Components MUST be stored in contiguous memory (NativeArrays) to support SIMD processing and Burst jobs.

#### Scenario: Contiguous Access
- **WHEN** iterating over a component type
- **THEN** data is accessed linearly in memory
- **AND** can be passed to Burst jobs as `NativeArray` or pointers

### Requirement: Entity ID Lookup
The system MUST provide O(1) lookup from Entity ID to Component Index.
(For simple ECS, typically Entity ID == Index if sparse set is not used, or map is used. For SIMD, dense arrays are preferred. We will assume Dense Indices for now with generational ID).

#### Scenario: Lookup Performance
- **WHEN** accessing a component for a valid Entity ID
- **THEN** the retrieval is direct indexing (O(1)) without searching

### Requirement: System Integration
Existing systems (RVO, Pathfinding) MUST operate on the central component data.

#### Scenario: RVO Integration
- **WHEN** `SIMDRVOSimulator` runs
- **THEN** it reads positions and velocities directly from `EntityManager` arrays
- **AND** writes new velocities back to `EntityManager` arrays

