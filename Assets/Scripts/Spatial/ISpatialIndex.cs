using System.Collections.Generic;
using UnityEngine;
using Cysharp.Threading.Tasks;

public interface ISpatialIndex
{
    /// <summary>
    /// Rebuilds the index with the given positions asynchronously.
    /// </summary>
    /// <param name="positions">List of positions to index.</param>
    UniTask BuildAsync(List<Vector2> positions);

    /// <summary>
    /// Finds the K nearest neighbors to the query position.
    /// </summary>
    /// <param name="position">Query position.</param>
    /// <param name="k">Number of neighbors to find.</param>
    /// <param name="results">List to store the indices of the nearest neighbors.</param>
    void QueryKNearest(Vector2 position, int k, List<int> results);

    /// <summary>
    /// Finds all neighbors within a given radius.
    /// </summary>
    /// <param name="position">Query position.</param>
    /// <param name="radius">Search radius.</param>
    /// <param name="results">List to store the indices of the neighbors.</param>
    void QueryRadius(Vector2 position, float radius, List<int> results);
}
