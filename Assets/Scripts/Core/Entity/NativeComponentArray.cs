using System;
using Unity.Collections;
using Unity.Mathematics;

public unsafe class NativeComponentArray<T> : INativeComponentArray where T : struct
{
    private NativeArray<T> _data;
    private int _count;
    private int _capacity;

    public NativeComponentArray(int initialCapacity = 128)
    {
        _capacity = initialCapacity;
        _data = new NativeArray<T>(_capacity, Allocator.Persistent);
        _count = 0;
    }

    public void Resize(int newCapacity)
    {
        if (newCapacity <= _capacity) return;
        var newData = new NativeArray<T>(newCapacity, Allocator.Persistent);
        if (_count > 0) NativeArray<T>.Copy(_data, newData, _count);
        _data.Dispose();
        _data = newData;
        _capacity = newCapacity;
    }

    public void Set(int index, T value)
    {
        if (index >= _capacity) Resize(math.max(index + 1, _capacity * 2));
        _data[index] = value;
        if (index >= _count) _count = index + 1;
    }

    public T Get(int index)
    {
        if (index >= _count) return default;
        return _data[index];
    }
    
    public NativeArray<T> AsNativeArray() => _data; // Expose for Burst
    public int Count => _count;

    public void Copy(int fromIndex, int toIndex)
    {
        _data[toIndex] = _data[fromIndex];
    }

    public void RemoveLast()
    {
        if (_count > 0) _count--;
    }

    public void Dispose()
    {
        if (_data.IsCreated) _data.Dispose();
    }
}
