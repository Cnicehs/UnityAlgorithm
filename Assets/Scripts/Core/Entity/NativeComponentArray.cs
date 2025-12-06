using System;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;

public interface INativeComponentArray : IDisposable
{
    void Remove(int entityId);
    bool Has(int entityId);
}

public struct ComponentWrapper<T> where T : unmanaged
{
    public int EntityId;
    public T Component;
}

public unsafe class NativeComponentArray<T> : INativeComponentArray where T : unmanaged
{
    // Sparse Map: EntityID -> DenseIndex
    // Using Dictionary to handle unlimited/sparse EntityIDs
    private Dictionary<int, int> _indexMap; 
    
    // Dense Storage: (EntityID, Component)
    // Using NativeArray with manual resize because NativeList might be missing in older Unity versions
    private NativeArray<ComponentWrapper<T>> _data;
    private int _count;
    private int _capacity;

    public int Count => _count;

    public NativeComponentArray(int initialCapacity = 128)
    {
        _capacity = initialCapacity;
        _indexMap = new Dictionary<int, int>(initialCapacity);
        _data = new NativeArray<ComponentWrapper<T>>(_capacity, Allocator.Persistent);
        _count = 0;
    }

    public void Add(int entityId, T component)
    {
        if (_indexMap.ContainsKey(entityId))
        {
            // Update existing
            int index = _indexMap[entityId];
            var ptr = (ComponentWrapper<T>*)_data.GetUnsafePtr();
            ptr[index].Component = component;
            return;
        }

        if (_count >= _capacity)
        {
            ResizeDense(math.max(_count + 1, _capacity * 2));
        }

        int newIndex = _count;
        var ptr2 = (ComponentWrapper<T>*)_data.GetUnsafePtr();
        ptr2[newIndex] = new ComponentWrapper<T> { EntityId = entityId, Component = component };
        
        _indexMap[entityId] = newIndex;
        _count++;
    }

    public void Remove(int entityId)
    {
        if (!_indexMap.TryGetValue(entityId, out int index)) return;

        int lastIndex = _count - 1;
        if (index != lastIndex)
        {
            // Swap with last
            var ptr = (ComponentWrapper<T>*)_data.GetUnsafePtr();
            ComponentWrapper<T> lastElement = ptr[lastIndex];
            
            // Move last element to 'index'
            ptr[index] = lastElement;
            
            // Update map for the moved element
            _indexMap[lastElement.EntityId] = index;
        }

        // Just decrement count, effectively removing last element
        _count--;
        _indexMap.Remove(entityId);
    }

    public bool Has(int entityId)
    {
        return _indexMap.ContainsKey(entityId);
    }

    public ref T GetRef(int entityId)
    {
        if (!_indexMap.TryGetValue(entityId, out int index))
        {
             throw new InvalidOperationException($"Entity {entityId} does not have component {typeof(T).Name}");
        }
        
        var ptr = (ComponentWrapper<T>*)_data.GetUnsafePtr();
        return ref ptr[index].Component;
    }

    public ref readonly T GetReadOnly(int entityId)
    {
        if (!_indexMap.TryGetValue(entityId, out int index))
        {
             throw new InvalidOperationException($"Entity {entityId} does not have component {typeof(T).Name}");
        }
        var ptr = (ComponentWrapper<T>*)_data.GetUnsafeReadOnlyPtr();
        return ref ptr[index].Component;
    }

    // Dense iteration API
    public ref ComponentWrapper<T> GetDenseWrapperRef(int denseIndex)
    {
        if (denseIndex >= _count) throw new IndexOutOfRangeException();
        var ptr = (ComponentWrapper<T>*)_data.GetUnsafePtr();
        return ref ptr[denseIndex];
    }
    
    // Renamed back to GetDenseRef to match system usage
    public ref T GetDenseRef(int denseIndex)
    {
        return ref GetDenseWrapperRef(denseIndex).Component;
    }

    public int GetEntityIdFromDenseIndex(int denseIndex)
    {
        return GetDenseWrapperRef(denseIndex).EntityId;
    }

    private void ResizeDense(int newCapacity)
    {
        var newData = new NativeArray<ComponentWrapper<T>>(newCapacity, Allocator.Persistent);
        NativeArray<ComponentWrapper<T>>.Copy(_data, newData, _count);
        _data.Dispose();
        _data = newData;
        _capacity = newCapacity;
    }

    public void Dispose()
    {
        if (_data.IsCreated) _data.Dispose();
        _indexMap = null;
    }
}
