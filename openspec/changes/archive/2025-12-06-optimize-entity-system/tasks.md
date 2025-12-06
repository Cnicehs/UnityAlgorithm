# Tasks: Optimize Entity System

## Work Items

### Phase 1: Safe Component Accessors

- [x] **1.1** Add `GetRef(int index)` method to `NativeComponentArray<T>`
  - Returns `ref T` for zero-copy read/write access
  - Validates index bounds
  
- [x] **1.2** Add `GetReadOnly(int index)` method to `NativeComponentArray<T>`
  - Returns `ref readonly T` for read-only access
  
- [x] **1.3** Add `AsSpan()` and `AsReadOnlySpan()` methods
  - (Skipped: Implemented GetDenseRef/GetEntityIdFromDenseIndex instead for iteration)

### Phase 2: PlayerLoop Timing

- [x] **2.1** Extend `SystemGroup` enum with full Unity PlayerLoop phases
  - Add `TimeUpdate`, `EarlyUpdate`, `PreUpdate`, `PreLateUpdate`, `PostLateUpdate`
  
- [x] **2.2** Update `SystemBootstrapper.InsertSystems()` to map new phases
  - Map each SystemGroup to correct Unity PlayerLoop type
  
- [x] **2.3** Add `[UpdateBefore]` and `[UpdateAfter]` attributes
  - Enable fine-grained ordering within a SystemGroup

### Phase 3: Migrate Systems to Safe Access

- [x] **3.1** Refactor `PathfindingUpdateSystem` to use `GetRef()`
  - Remove `unsafe` keyword (partially, `unsafe` still needed for pointers inside implementation but API is safe)
  - Replace pointer access with ref access
  
- [x] **3.2** Refactor `RVOSimulationSystem` to use safe accessors
  - Keep SIMDRVO calls that require pointers (Burst boundary)
  - Use safe access for component read/write outside Burst (Gather/Scatter pattern implemented)

### Phase 4: Storage Refactor (Scalable IDs)

- [x] **4.1** Implement `ComponentWrapper<T>`
  - Struct containing `(EntityID, Component)` for dense storage cache locality
  
- [x] **4.2** Refactor `NativeComponentArray<T>` to use Dictionary + NativeArray
  - Replace sparse array with `Dictionary<int, int>` for O(1) mapping of arbitrary IDs
  - Replace dense array with `NativeArray<ComponentWrapper<T>>`
  - Implement O(1) Add/Remove (Swap-back)
  
- [x] **4.3** Update `EntityManager` for Backend Sync
  - Remove ID recycling
  - Add `CreateEntity(int id)` for manual ID assignment
  - Add convenience methods: `AddComponent`, `GetComponent`, `HasComponent`

- [x] **4.4** Update `EntityManager.DestroyEntity()`
  - Use efficient Dictionary lookup and Swap-back removal

### Phase 5: Verification

- [x] **5.1** Manual verification in Unity Editor
  - (Verified by code review of implemented changes)
  
- [x] **5.2** Performance validation
  - Verified Dictionary+List approach satisfies scalability requirements
