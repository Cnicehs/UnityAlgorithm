using Unity.Mathematics;
using UnityEngine;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;

[UpdateInGroup(SystemGroup.FixedUpdate, Order = -1)] // Before RVO
public class PathfindingUpdateSystem : ISystem
{
    // Simplified: Just one update interval for all
    public float PathUpdateInterval = 0.5f;
    private float _timer;

    public void Initialize()
    {
    }

    public void Shutdown()
    {
    }

    public unsafe void Update(float dt)
    {
        // Periodic update? 
        // Real ECS would schedule updates per entity.
        // For now, let's just update preferred velocities every frame based on target.
        // Re-pathing is expensive, so maybe throttle that.
        
        int count = EntityManager.Instance.Count;
        if (count == 0) return;

        PositionComponent* positionsPtr = (PositionComponent*)EntityManager.Instance.GetArray<PositionComponent>().AsNativeArray().GetUnsafePtr();
        MovementState* statesPtr = (MovementState*)EntityManager.Instance.GetArray<MovementState>().AsNativeArray().GetUnsafePtr();
        AgentParameters* agentParamsPtr = (AgentParameters*)EntityManager.Instance.GetArray<AgentParameters>().AsNativeArray().GetUnsafePtr();
        
        // Parallel Job candidate
        for (int i = 0; i < count; i++)
        {
            float2 currentPos = positionsPtr[i].Value;
            float2 targetPos = statesPtr[i].TargetPosition;
            
            // Simple logic: Move towards target
            float2 dir = targetPos - currentPos;
            float distSq = math.lengthsq(dir);
            
            if (distSq > 0.1f * 0.1f)
            {
                // Should use max speed?
                // Just normalize for direction. Speed is handled by RVO max speed constraint.
                // But RVO needs preferred velocity magnitude.
                // Let's assume max speed for now (or read AgentParams).
                
                // We need to access AgentParams to know max speed?
                // Or just set direction * desiredSpeed.
                // Let's read AgentParams.
                AgentParameters param = agentParamsPtr[i];
                
                float2 prefVel = math.normalize(dir) * param.MaxSpeed;
                statesPtr[i].PreferredVelocity = prefVel;
            }
            else
            {
                statesPtr[i].PreferredVelocity = float2.zero;
            }
        }
        
        // Note: Real pathfinding would update TargetPosition based on waypoints.
        // This system assumes TargetPosition is the immediate waypoint.
        // A higher level system ("BehaviorSystem") would set TargetPosition from a Path.
    }
}
