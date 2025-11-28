using UnityEngine;
using Unity.Mathematics;

public class RVOAgent
{
    public int ID;
    public float2 Position;
    public float2 Velocity;
    public float2 PrefVelocity;
    public float Radius;
    public float MaxSpeed;
    public float NeighborDist;
    public int MaxNeighbors;
    public float TimeHorizon;
    public float TimeHorizonObst;

    public float2 NewVelocity;

    public RVOAgent(int id, Vector2 pos)
    {
        ID = id;
        Position = pos;
        Velocity = float2.zero;
        PrefVelocity = float2.zero;
        Radius = 0.5f;
        MaxSpeed = 2.0f;
        NeighborDist = 10.0f;
        MaxNeighbors = 10;
        TimeHorizon = 2.0f;
        TimeHorizonObst = 2.0f;
    }
}
