## ADDED Requirements

### Requirement: Unified Comparison Demo
The system SHALL provide a unified demo environment to compare different simulation algorithms and modes.

#### Scenario: Mode Switching
- **WHEN** the user selects a mode (Scene, Pathfinding, RVO, Pathfinding+RVO)
- **THEN** the demo SHALL activate the corresponding systems and deactivate others.

#### Scenario: Algorithm Switching
- **WHEN** the user switches between Default and SIMD algorithms
- **THEN** the demo SHALL switch the underlying implementation (e.g., `AStarPathfinder` vs `SIMDAStarPathfinder`, `RVOSimulator` vs `RVOSimulationSystem`) while maintaining the current state if possible.

#### Scenario: Entity Management
- **WHEN** the user adjusts the agent count
- **THEN** the demo SHALL spawn or despawn entities to match the requested count using the Entity system.

#### Scenario: Performance Visualization
- **WHEN** the simulation is running
- **THEN** the demo SHALL display execution times for key phases (Pathfinding, RVO) via OnGUI.

#### Scenario: Visual Debugging
- **WHEN** Gizmos are enabled
- **THEN** the demo SHALL draw relevant debug information (Grid, Paths, RVO Obstacles, Velocity vectors).
