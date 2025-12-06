using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Spatial index interface for line segment elements (e.g., RVO obstacles).
/// Unlike point-based ISpatialIndex, this handles geometry with extent.
/// </summary>
public interface ISegmentSpatialIndex
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
