using Unity.Mathematics;
using UnityEngine;
using System.Collections.Generic;
using Unity.Collections;

[UpdateInGroup(SystemGroup.FixedUpdate, Order = -1)] // Before RVO
public class PathfindingUpdateSystem : ISystem
{
    public float PathUpdateInterval = 0.5f;

    public void Initialize()
    {
    }

    public void Shutdown()
    {
    }

    public void Update(float dt)
    {
        var entityManager = EntityManager.Instance;
        var positions = entityManager.GetArray<PositionComponent>();
        var states = entityManager.GetArray<MovementState>();
        var agentParams = entityManager.GetArray<AgentParameters>();

        // Iterate over all entities that have MovementState
        // Note: In a full ECS we would have a query for (Position + Movement + Params)
        // Here we iterate one and lookup others.
        for (int i = 0; i < states.Count; i++)
        {
            ref var state = ref states.GetDenseRef(i);
            int entityId = states.GetEntityIdFromDenseIndex(i);

            if (!positions.Has(entityId) || !agentParams.Has(entityId)) continue;

            ref var pos = ref positions.GetRef(entityId);
            ref readonly var param = ref agentParams.GetReadOnly(entityId); // Use ReadOnly if possible
            
            float2 currentPos = pos.Value;
            float2 targetPos = state.TargetPosition;
            
            float2 dir = targetPos - currentPos;
            float distSq = math.lengthsq(dir);
            
            if (distSq > 0.1f * 0.1f)
            {
                float2 prefVel = math.normalize(dir) * param.MaxSpeed;
                state.PreferredVelocity = prefVel;
            }
            else
            {
                state.PreferredVelocity = float2.zero;
            }
        }
    }
}
