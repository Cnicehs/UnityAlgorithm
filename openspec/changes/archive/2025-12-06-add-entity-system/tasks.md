## 1. Core Entity System (Data)
- [x] 1.1 Implement `Entity` struct.
- [x] 1.2 Implement `NativeComponentArray<T>` (manual resize).
- [x] 1.3 Implement `EntityManager` (Data Store, Create/Destroy, Swap-Back).

## 2. System Architecture
- [x] 2.1 Implement `ISystem` interface.
- [x] 2.2 Implement `SystemAttribute` (Group, Order).
- [x] 2.3 Implement `SystemBootstrapper` (Scanning, PlayerLoop Integration).

## 3. Components
- [ ] 3.1 Define core components: `Position`, `Velocity`, `Radius`, `AgentParameters`, `MovementState`.

## 4. System Implementation
- [ ] 4.1 Refactor `SIMDRVO.ComputeRVOVelocities` to support SoA (Structure of Arrays).
- [ ] 4.2 Implement `RVOSimulationSystem`.
- [ ] 4.3 Implement `PathfindingUpdateSystem`.

## 5. Cleanup
- [ ] 5.1 Remove legacy `SIMDRVOSimulator` (or make it a wrapper).
- [ ] 5.2 Remove legacy `PathfindingSystem` (or make it a wrapper).
