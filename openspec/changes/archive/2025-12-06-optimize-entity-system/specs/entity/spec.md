# entity Specification Delta

## MODIFIED Requirements

### Requirement: Data-Oriented Component Storage

Components MUST be stored in contiguous memory (NativeArrays) using a Dictionary-mapped Dense Array pattern to support both SIMD processing and scalable, sparse Entity IDs.

#### Scenario: Contiguous Access
- **WHEN** iterating over a component type
- **THEN** data is accessed linearly in dense memory via `ComponentWrapper<T>`
- **AND** can be passed to Burst jobs as `NativeArray` (with wrapper layout consideration)

### Requirement: Entity ID Lookup
The system MUST provide O(1) lookup from Entity ID to Component Index using a Dictionary-based mapping to support sparse and large Entity IDs.

#### Scenario: Lookup Performance
- **WHEN** accessing a component for a valid Entity ID
- **THEN** the retrieval uses a Dictionary lookup (O(1)) followed by direct indexing

### Requirement: System Integration
Existing systems (RVO, Pathfinding) MUST operate on the central component data using safe accessors.

#### Scenario: RVO Integration
- **WHEN** `RVOSimulationSystem` runs
- **THEN** it gathers aligned data from `EntityManager` using the new API
- **AND** processes it with SIMD jobs

---

## ADDED Requirements

### Requirement: Safe Component Access
Systems MUST be able to access components via `ref`/`ref readonly` without requiring unsafe code.

#### Scenario: Zero-Copy Modification
- **WHEN** calling `GetComponent<PositionComponent>(entity)`
- **THEN** a `ref PositionComponent` is returned
- **AND** modifying it updates the storage directly without copying.

### Requirement: Complete PlayerLoop Timing
The system MUST support all Unity PlayerLoop phases to enable precise System scheduling.

#### Scenario: System Executes in EarlyUpdate
- **GIVEN** a System with `[UpdateInGroup(SystemGroup.EarlyUpdate)]`
- **WHEN** the PlayerLoop executes
- **THEN** the System runs during Unity's EarlyUpdate phase

### Requirement: System Ordering Attributes
Systems MUST support fine-grained ordering via `[UpdateBefore]` and `[UpdateAfter]` attributes.

#### Scenario: Explicit System Ordering
- **GIVEN** SystemA with `[UpdateBefore(typeof(SystemB))]`
- **AND** Both systems in the same SystemGroup
- **WHEN** the group executes
- **THEN** SystemA.Update() is called before SystemB.Update()

### Requirement: Manual ID Assignment
The system MUST allow creating Entities with specific IDs to support synchronization with external systems (e.g. Backend).

#### Scenario: Backend Sync
- **WHEN** `CreateEntity(1005)` is called
- **THEN** an Entity with ID 1005 is created
- **AND** subsequent auto-generated IDs will skip 1005 to avoid collision.
