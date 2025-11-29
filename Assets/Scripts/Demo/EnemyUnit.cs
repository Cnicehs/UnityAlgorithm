using System.Collections.Generic;
using UnityEngine;

public class EnemyUnit
{
    public GameObject GameObject;
    public int RVOAgentId;
    public List<Vector2> Path = new List<Vector2>();
    public int PathIndex;
    public float NextPathUpdateTime;

    // IPathfindingUnit implementation (if we had an interface)
    public Vector2 Position => GameObject.transform.position;
    public Vector2 TargetPosition { get; set; }
    public float Speed => 3.0f; // EnemySpeed - hardcoded for now or injected
    public float PathRecalculateInterval => 0.5f; // PathUpdateInterval
    public List<Vector2> CurrentPath => Path;
    public int CurrentPathIndex { get => PathIndex; set => PathIndex = value; }
    public float LastPathRecalculateTime { get => NextPathUpdateTime; set => NextPathUpdateTime = value; }
    public int AgentId => RVOAgentId;
}
