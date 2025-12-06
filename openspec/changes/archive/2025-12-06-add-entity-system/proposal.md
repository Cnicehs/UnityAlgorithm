# Change: Add Attribute-Driven Entity System

## Why
The project currently manages game objects using disparate systems (`PathfindingSystem`, `RVOSimulator`, `SpatialIndexManager`). A monolithic `EntityManager` approach lacks extensibility. A system architecture based on Attributes and Unity's PlayerLoop will allow modular, automatically registered systems with precise execution order control, providing a robust foundation for scaling.

## What Changes
- **Core Entity System**:
    - `EntityManager`: Pure Data Store (Entities + Component Arrays).
    - `ISystem`: Interface for logic systems.
    - `SystemAttribute`: Attribute to define Execution Timing (Update/LateUpdate) and Order.
    - `SystemBootstrapper`: Automatically registers systems into Unity's PlayerLoop using Reflection.
- **Components**:
    - `PositionComponent`, `VelocityComponent`, `RadiusComponent`.
    - `AgentParametersComponent`.
- **Integration**:
    - Convert `SIMDRVOSimulator` logic into an `ISystem` (`RVOSimulationSystem`).
    - Convert Pathfinding logic into an `ISystem` (`PathfindingUpdateSystem`).
    - Systems read/write directly to `EntityManager` data.

## Impact
- **Affected Specs**: Entity System, RVO, Pathfinding
- **Affected Code**:
    - Refactor `EntityManager.cs` (Data only).
    - New `Assets/Scripts/Core/Entity/Systems/` directory.
    - `Assets/Scripts/Core/Systems/PathfindingSystem.cs` (Deprecate or convert to System).
