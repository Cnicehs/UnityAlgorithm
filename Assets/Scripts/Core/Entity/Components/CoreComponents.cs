using Unity.Mathematics;

public struct PositionComponent
{
    public float2 Value;
}

public struct VelocityComponent
{
    public float2 Value;
}

public struct RadiusComponent
{
    public float Value;
}

public struct AgentParameters
{
    public float MaxSpeed;
    public float NeighborDist;
    public int MaxNeighbors;
    public float TimeHorizon;
    public float TimeHorizonObst;
}

public struct MovementState
{
    public float2 TargetPosition;
    public float2 PreferredVelocity;
    public bool HasPath;
}
