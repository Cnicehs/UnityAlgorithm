## ADDED Requirements

### Requirement: Implementation Separation
Implementations MUST be separated into `Default` and `SIMD` subdirectories within each module.

#### Scenario: RVO Structure
- **WHEN** inspecting `Core/RVO`
- **THEN** `Default` contains `RVOSimulator.cs`
- **AND** `SIMD` contains `SIMDRVOSimulator.cs`

### Requirement: RVO Spatial Coupling
Obstacle spatial indexing logic SHALL be located within the RVO module.

#### Scenario: Obstacle Index Location
- **WHEN** inspecting `Core/RVO/Spatial`
- **THEN** it contains `IObstacleSpatialIndex.cs`
