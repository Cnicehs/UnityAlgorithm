using System;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Mathematics;

public class EntityManager
{
    private static EntityManager _instance;
    public static EntityManager Instance => _instance ??= new EntityManager();

    private int _nextId = 0;
    private HashSet<int> _activeEntityIds = new HashSet<int>();

    // Component Storage (Generic)
    private Dictionary<Type, INativeComponentArray> _componentArrays = new Dictionary<Type, INativeComponentArray>();

    public int Count => _activeEntityIds.Count;

    private EntityManager()
    {
    }

    public NativeComponentArray<T> GetArray<T>() where T : unmanaged
    {
        Type type = typeof(T);
        if (!_componentArrays.TryGetValue(type, out var array))
        {
            var newArray = new NativeComponentArray<T>();
            _componentArrays[type] = newArray;
            return newArray;
        }
        return (NativeComponentArray<T>)array;
    }

    public Entity CreateEntity()
    {
        int id = _nextId++;
        // Skip IDs that might have been manually added to avoid collision
        while (_activeEntityIds.Contains(id))
        {
            id = _nextId++;
        }

        _activeEntityIds.Add(id);
        
        return new Entity { Id = id, Version = 1 };
    }

    public Entity CreateEntity(int id)
    {
        if (_activeEntityIds.Contains(id))
        {
            // Or throw exception?
            // For safety, we just return existing or log error.
            // Returning new handle to same ID is safer than crashing.
            return new Entity { Id = id, Version = 1 };
        }
        
        _activeEntityIds.Add(id);
        // Ensure _nextId is ahead of manual ID if it's larger, to reduce collision checks later
        if (id >= _nextId) _nextId = id + 1;
        
        return new Entity { Id = id, Version = 1 };
    }

    public void DestroyEntity(Entity entity)
    {
        if (!_activeEntityIds.Contains(entity.Id)) return;

        foreach (var array in _componentArrays.Values)
        {
            array.Remove(entity.Id);
        }

        _activeEntityIds.Remove(entity.Id);
    }

    public void AddComponent<T>(Entity entity, T component) where T : unmanaged
    {
        GetArray<T>().Add(entity.Id, component);
    }

    public void RemoveComponent<T>(Entity entity) where T : unmanaged
    {
        GetArray<T>().Remove(entity.Id);
    }

    public bool HasComponent<T>(Entity entity) where T : unmanaged
    {
        return GetArray<T>().Has(entity.Id);
    }

    public ref T GetComponent<T>(Entity entity) where T : unmanaged
    {
        return ref GetArray<T>().GetRef(entity.Id);
    }

    public void Shutdown()
    {
        foreach (var array in _componentArrays.Values)
        {
            array.Dispose();
        }
        _componentArrays.Clear();
        _activeEntityIds.Clear();
        _instance = null;
    }
}
