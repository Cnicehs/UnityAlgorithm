using Unity.Mathematics;
using UnityEngine;
using System.Collections.Generic;

public class LegacyRVOSystem : ISystem
{
    // Mapping from Entity ID to RVO Agent Index
    private Dictionary<int, int> _entityToAgentMap = new Dictionary<int, int>();
    private List<int> _activeAgents = new List<int>(); // List of Entity IDs managed

    public void Initialize()
    {
        RVOSimulator.Instance.ClearAgents();
        _entityToAgentMap.Clear();
        _activeAgents.Clear();
    }

    public void Shutdown()
    {
        RVOSimulator.Instance.ClearAgents();
        _entityToAgentMap.Clear();
        _activeAgents.Clear();
    }

    public void Update(float dt)
    {
        var entityManager = EntityManager.Instance;
        var positions = entityManager.GetArray<PositionComponent>();
        var velocities = entityManager.GetArray<VelocityComponent>();
        var radii = entityManager.GetArray<RadiusComponent>();
        var paramsArray = entityManager.GetArray<AgentParameters>();
        var movementStates = entityManager.GetArray<MovementState>();

        // 1. Sync Entities -> Simulator
        // In a real ECS, we would track changes. Here we just iterate all.
        // Also need to handle adding/removing agents.
        
        // We'll iterate through paramsArray as the "source of truth" for agents
        int count = paramsArray.Count;
        for (int i = 0; i < count; i++)
        {
            int entityId = paramsArray.GetEntityIdFromDenseIndex(i);
            
            if (!positions.Has(entityId) || !movementStates.Has(entityId)) continue;
            
            ref var pos = ref positions.GetRef(entityId);
            ref var vel = ref velocities.GetRef(entityId); // Velocity might be optional? Let's assume required.
            ref var param = ref paramsArray.GetDenseRef(i);
            ref var state = ref movementStates.GetRef(entityId);
            float radius = radii.Has(entityId) ? radii.GetRef(entityId).Value : 0.5f;

            if (!_entityToAgentMap.TryGetValue(entityId, out int agentIndex))
            {
                // New Agent - Set global defaults first so the new agent picks them up
                RVOSimulator.Instance.Radius = radius;
                RVOSimulator.Instance.MaxSpeed = param.MaxSpeed;
                RVOSimulator.Instance.NeighborDist = param.NeighborDist;
                RVOSimulator.Instance.MaxNeighbors = param.MaxNeighbors;
                RVOSimulator.Instance.TimeHorizon = param.TimeHorizon;
                // RVOSimulator.Instance.TimeHorizonObst = param.TimeHorizonObst; // Assuming this field exists based on RVOSimulator.cs

                RVOSimulator.Instance.AddAgent(new Vector3(pos.Value.x, 0, pos.Value.y));
                agentIndex = RVOSimulator.Instance.GetAgentCount() - 1;

                if (agentIndex != -1)
                {
                    _entityToAgentMap[entityId] = agentIndex;
                    _activeAgents.Add(entityId);
                }
            }
            
            // Sync Input Data
            if (agentIndex != -1)
            {
                // Update position (in case it was moved externally, e.g. teleport)
                // Note: RVOSimulator usually owns position, but if we have hybrid, we might need to force it.
                // But for pure simulation, we assume Simulator updated it last frame.
                // However, we MUST set Preferred Velocity.
                Vector2 prefVel = new Vector2(state.PreferredVelocity.x, state.PreferredVelocity.y);
                RVOSimulator.Instance.SetAgentPrefVelocity(agentIndex, new Vector3(prefVel.x, 0, prefVel.y));
                
                // If we want to support external position changes, we should check distance?
                // For now, assume Simulator is the only one moving agents during RVO mode.
            }
        }

        // 2. Step Simulation
        RVOSimulator.Instance.Step(dt);

        // 3. Sync Simulator -> Entities
        foreach (var kvp in _entityToAgentMap)
        {
            int entityId = kvp.Key;
            int agentIndex = kvp.Value;
            
            if (!positions.Has(entityId)) continue; // Entity might have been destroyed

            Vector3 p = RVOSimulator.Instance.GetAgentPosition(agentIndex);
            Vector3 v = RVOSimulator.Instance.GetAgentVelocity(agentIndex);
            
            ref var pos = ref positions.GetRef(entityId);
            pos.Value = new float2(p.x, p.z);
            
            if (velocities.Has(entityId))
            {
                ref var vel = ref velocities.GetRef(entityId);
                vel.Value = new float2(v.x, v.z);
            }
        }
    }
}
