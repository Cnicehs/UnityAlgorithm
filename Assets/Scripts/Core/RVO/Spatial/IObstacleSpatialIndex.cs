using System.Collections.Generic;
using UnityEngine;

public interface IObstacleSpatialIndex
{
    /// <summary>
    /// Rebuilds the index with the given obstacles.
    /// </summary>
    void Build(List<RVOObstacle> obstacles);

    /// <summary>
    /// Queries obstacles near the position, sorted by distance.
    /// Used by RVO to ensure correct processing order (Front-to-Back) for implicit occlusion culling.
    /// </summary>
    void QueryNearest(Vector2 position, List<RVOObstacle> results);
}
