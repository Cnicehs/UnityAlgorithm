using System;
using System.Collections.Generic;
using System.Linq.Expressions;
using System.Reflection;

public static class ListUtils
{
    private static class ArrayAccessor<T>
    {
        public static readonly Func<List<T>, T[]> Getter;

        static ArrayAccessor()
        {
            var param = Expression.Parameter(typeof(List<T>), "list");
            // _items is the internal array name in Mono/CoreCLR
            var field = typeof(List<T>).GetField("_items", BindingFlags.NonPublic | BindingFlags.Instance);
            
            if (field == null)
            {
                // Fallback for different implementations if needed, though _items is standard
                throw new InvalidOperationException("Could not find _items field in List<T>");
            }

            var fieldAccess = Expression.Field(param, field);
            var lambda = Expression.Lambda<Func<List<T>, T[]>>(fieldAccess, param);
            Getter = lambda.Compile();
        }
    }

    /// <summary>
    /// Returns a Span<T> over the internal array of the List<T>.
    /// Equivalent to CollectionsMarshal.AsSpan in newer .NET versions.
    /// </summary>
    public static Span<T> AsSpan<T>(this List<T> list)
    {
        if (list == null) return default;
        T[] array = ArrayAccessor<T>.Getter(list);
        return new Span<T>(array, 0, list.Count);
    }
}
