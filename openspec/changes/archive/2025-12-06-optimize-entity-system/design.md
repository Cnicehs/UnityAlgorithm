# Design: Optimize Entity System

## Overview

This document describes the architectural approach for optimizing the ECS system, focusing on safe component access, complete PlayerLoop timing, efficient entity lifecycle management, and scalable ID support.

## 1. Safe Component Access

### API Design

```csharp
public interface IComponentAccessor<T> where T : unmanaged
{
    /// <summary>Read-only access (in parameter semantics)</summary>
    ref readonly T GetReadOnly(int entityId);
    
    /// <summary>Read-write access (ref parameter semantics)</summary>
    ref T GetRef(int entityId);
    
    /// <summary>Dense iteration access</summary>
    ref T GetDenseRef(int denseIndex);
}
```

## 2. Complete PlayerLoop Timing

### SystemGroup

```csharp
public enum SystemGroup
{
    Initialization,
    TimeUpdate,
    EarlyUpdate,
    PreUpdate,
    FixedUpdate,
    Update,
    PreLateUpdate,
    LateUpdate,
    PostLateUpdate
}
```

## 3. Storage Architecture: Dictionary + Dense Array

### Problem with Arrays
Directly using EntityID as an array index (`sparse[entityId]`) fails when EntityIDs grow large (e.g. synced with backend DB), leading to huge memory waste.

### Solution: Dictionary Map + Packed Array
We use a `Dictionary` for the sparse mapping (EntityID -> DenseIndex) and a packed `NativeArray` for the component data.

```
┌─────────────────────────────────────────────────────────────────┐
│                        Hybrid Storage                            │
├─────────────────────────────────────────────────────────────────┤
│  Sparse Map (Dictionary<int, int>)                               │
│  EntityID ──> DenseIndex                                         │
│                                                                  │
│  Dense Array (NativeArray<ComponentWrapper<T>>)                  │
│  ┌────────────────────────┬────────────────────────┬─────┐       │
│  │ Wrapper { ID, Data }   │ Wrapper { ID, Data }   │ ... │       │
│  └────────────────────────┴────────────────────────┴─────┘       │
│           Index 0                  Index 1                       │
└─────────────────────────────────────────────────────────────────┘
```

### Component Wrapper
Storing `(EntityID, Component)` together improves cache locality when iterating densely and needing the EntityID (e.g. for lookups in other component arrays).

```csharp
public struct ComponentWrapper<T> where T : unmanaged
{
    public int EntityId;
    public T Component;
}
```

### Key Operations

| Operation      | Implementation                                | Complexity |
|----------------|-----------------------------------------------|------------|
| Add Entity     | `Dict.Add(id, count)`, `Array.Add(wrapper)`   | O(1)*      |
| Remove Entity  | Swap-back in Array, Update Dict, `Dict.Remove`| O(1)*      |
| Get Component  | `Dict.TryGetValue` -> `Array[index]`          | O(1)*      |
| Iterate All    | Iterate `Array`                               | O(N)       |

*Amortized for Dictionary/List resize.

## 4. Entity Lifecycle & IDs

### No Recycling
To support persistent backend IDs, we do **not** recycle Entity IDs. IDs are monotonically increasing.

### Manual ID Assignment
`EntityManager.CreateEntity(int id)` allows creating entities with specific IDs (e.g. from network packets), ensuring frontend/backend sync.

### Internal Counter
`_nextId` automatically skips over manually assigned IDs to prevent collisions for locally created entities.

## 5. Migration Strategy

### Phase 1: Storage Refactor
- Update `NativeComponentArray` to use `Dictionary` and `NativeArray`
- Implement `ComponentWrapper` layout

### Phase 2: API Updates
- Update `EntityManager` with `AddComponent`, `GetComponent`
- Remove ID recycling logic

### Phase 3: System Updates
- Update `PathfindingUpdateSystem` and `RVOSimulationSystem` to use new safe APIs
- Implement Gather/Scatter for RVO to handle unaligned memory for Burst
