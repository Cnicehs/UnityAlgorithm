# Optimize Entity System

## Why

The current implementation has several issues:

- **Unsafe pointers everywhere**: Systems like `PathfindingUpdateSystem` and `RVOSimulationSystem` use `GetUnsafePtr()` which requires unsafe context and is error-prone
- **Incomplete timing control**: Only 4 SystemGroups defined (`Initialization`, `FixedUpdate`, `Update`, `LateUpdate`), missing `EarlyUpdate`, `PreUpdate`, `PostLateUpdate`
- **Poor entity lifecycle**: Using simple arrays means O(N) deletion and expensive resizing on entity add/remove
- **Limited ID Scalability**: Recycling IDs complicates backend sync. Using Sparse Arrays limits max ID size.
- **Verbose component access**: Getting component data requires multiple method calls and casts

## What Changes

### Core Changes
- **Refactor `NativeComponentArray<T>`**: Switch to `Dictionary<int, int>` + `NativeArray<ComponentWrapper<T>>` for scalable, sparse ID support and O(1) operations.
- **Update `EntityManager`**: Remove ID recycling; add `CreateEntity(int id)` for backend sync; add `AddComponent`/`GetComponent` convenience methods.
- **Extend `SystemGroup`**: Add full Unity PlayerLoop phases.
- **Update `SystemBootstrapper`**: Support topological sorting and new groups.

### System Updates
- **`PathfindingUpdateSystem`**: Use safe `GetRef` APIs.
- **`RVOSimulationSystem`**: Implement Gather/Scatter pattern to handle data for Burst jobs safely.

## Impact

- **Performance**: Maintaining dense storage for iteration, O(1) for random access and lifecycle.
- **Safety**: No more `unsafe` pointers exposed to high-level logic.
- **Scalability**: Can handle large/sparse Backend IDs.
- **Developer Experience**: API is now consistent with Unity DOTS/ECS patterns.
