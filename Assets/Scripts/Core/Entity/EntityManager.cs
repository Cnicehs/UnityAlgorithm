using System;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Mathematics;

public interface INativeComponentArray : IDisposable
{
    void Resize(int newCapacity);
    void Copy(int fromIndex, int toIndex);
    void RemoveLast();
}

public class EntityManager
{
    private static EntityManager _instance;
    public static EntityManager Instance => _instance ??= new EntityManager();

    // Map Entity ID -> Component Array Index
    private Dictionary<int, int> _entityToIndex = new Dictionary<int, int>();
    private List<Entity> _indexToEntity = new List<Entity>();
    
    private int _nextId = 0;

    // Component Storage (Generic)
    private Dictionary<Type, INativeComponentArray> _componentArrays = new Dictionary<Type, INativeComponentArray>();

    public int Count => _indexToEntity.Count;

    private EntityManager()
    {
        // Pre-register core components for convenience, or lazy load?
        // Lazy load in GetArray is better for extensibility.
    }

    public NativeComponentArray<T> GetArray<T>() where T : struct
    {
        Type type = typeof(T);
        if (!_componentArrays.TryGetValue(type, out var array))
        {
            var newArray = new NativeComponentArray<T>();
            // Ensure capacity matches current entities
            if (_indexToEntity.Count > 0)
            {
                newArray.Resize(math.max(128, _indexToEntity.Count));
            }
            _componentArrays[type] = newArray;
            return newArray;
        }
        return (NativeComponentArray<T>)array;
    }

    public Entity CreateEntity()
    {
        int id = _nextId++;
        int index = _indexToEntity.Count;
        int version = 1;

        Entity entity = new Entity { Id = id, Version = version };
        
        _entityToIndex[id] = index;
        _indexToEntity.Add(entity);

        // Resize all existing component arrays
        foreach (var array in _componentArrays.Values)
        {
            // Note: NativeComponentArray.Set handles resizing if accessed out of bounds,
            // but here we just added an entity.
            // We should ensure capacity if we are dense packing.
            // But NativeComponentArray handles explicit Resize/Set.
            // Let's just let Set handle it when components are added.
            // Wait, if we use SoA, index must be valid for all.
            // But we don't know what components this entity HAS.
            // In this simple dense ECS, ALL entities have ALL components (that are registered).
            // So we need to ensure space.
            // But NativeComponentArray resizes on Set.
            // If we access index 'index', it might be OOB.
            // Let's assume GetArray<T>().Set(index, default) is called by user?
            // Or we force default init for all known arrays.
        }
        // Force default init for all known arrays to keep them in sync
        foreach (var array in _componentArrays.Values)
        {
            // We can't call Set(index, default) generically easily without reflection or interface change.
            // Actually, NativeComponentArray auto-expands on Set.
            // But we need to ensure the "Count" of component arrays matches Entity Count?
            // Or we just rely on dense packing logic: index < capacity.
            // NativeComponentArray.Resize checks capacity.
            // We should resize if needed.
            // But 'INativeComponentArray' interface method Resize takes capacity.
            // We don't have 'SetDefault(index)'.
            // Let's just leave it. The arrays are sparse-ish or dense?
            // Dense means index I has valid data.
            // If we just added Entity I, data at I is uninitialized/default in NativeArray (0).
            // We just need to make sure Capacity > I.
            // Since we lazily resize in Set, it's fine.
            // BUT if we iterate using Count, we need arrays to be large enough.
            // Systems iterate 0..Count. They read arrays[i].
            // If array capacity <= i, NativeArray throws.
            // So we MUST resize all arrays to at least Count.
        }
        
        // Resize all arrays to ensure capacity covers the new index
        // We use a small buffer margin or exact?
        // Let's strictly ensure capacity >= Count.
        int requiredCapacity = index + 1;
        foreach (var array in _componentArrays.Values)
        {
            // This casts to interface. NativeComponentArray<T> implements it.
            // We can check capacity? Interface doesn't expose it.
            // Just blind resize? Resize checks current capacity inside.
            array.Resize(requiredCapacity);
        }

        return entity;
    }

    public void DestroyEntity(Entity entity)
    {
        if (!_entityToIndex.ContainsKey(entity.Id)) return;

        int indexToRemove = _entityToIndex[entity.Id];
        int lastIndex = _indexToEntity.Count - 1;

        if (indexToRemove != lastIndex)
        {
            // Swap with last
            Entity lastEntity = _indexToEntity[lastIndex];

            // Move components
            foreach (var array in _componentArrays.Values)
            {
                array.Copy(lastIndex, indexToRemove);
            }

            // Update maps
            _entityToIndex[lastEntity.Id] = indexToRemove;
            _indexToEntity[indexToRemove] = lastEntity;
        }

        // Remove last
        foreach (var array in _componentArrays.Values)
        {
            array.RemoveLast();
        }

        _entityToIndex.Remove(entity.Id);
        _indexToEntity.RemoveAt(lastIndex);
    }

    public void Shutdown()
    {
        foreach (var array in _componentArrays.Values)
        {
            array.Dispose();
        }
        _componentArrays.Clear();
        _instance = null;
    }
}
