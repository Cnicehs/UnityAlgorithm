using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using UnityEngine;
using UnityEngine.LowLevel;
using UnityEngine.PlayerLoop;

public static class SystemBootstrapper
{
    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterAssembliesLoaded)]
    public static void Initialize()
    {
        var systems = new List<ISystem>();
        var types = AppDomain.CurrentDomain.GetAssemblies()
            .Where(a => a.GetName().Name == "Assembly-CSharp" || a.GetName().Name.StartsWith("Assembly-CSharp-"))
            .SelectMany(a => a.GetTypes())
            .Where(t => typeof(ISystem).IsAssignableFrom(t) && !t.IsInterface && !t.IsAbstract);

        foreach (var type in types)
        {
            var attr = type.GetCustomAttribute<UpdateInGroupAttribute>();
            if (attr != null)
            {
                try 
                {
                    var system = (ISystem)Activator.CreateInstance(type);
                    system.Initialize();
                    systems.Add(system);
                    EntityManager.Instance.RegisterSystem(system);
                }
                catch (Exception e)
                {
                    Debug.LogError($"[SystemBootstrapper] Failed to instantiate {type.Name}: {e}");
                }
            }
        }

        // Group systems by enum
        var timeSystems = GetSortedSystems(systems, SystemGroup.TimeUpdate);
        var initializationSystems = GetSortedSystems(systems, SystemGroup.Initialization);
        var earlyUpdateSystems = GetSortedSystems(systems, SystemGroup.EarlyUpdate);
        var fixedUpdateSystems = GetSortedSystems(systems, SystemGroup.FixedUpdate);
        var preUpdateSystems = GetSortedSystems(systems, SystemGroup.PreUpdate);
        var updateSystems = GetSortedSystems(systems, SystemGroup.Update);
        var preLateUpdateSystems = GetSortedSystems(systems, SystemGroup.PreLateUpdate);
        var lateUpdateSystems = GetSortedSystems(systems, SystemGroup.LateUpdate);
        var postLateUpdateSystems = GetSortedSystems(systems, SystemGroup.PostLateUpdate);

        // Inject into PlayerLoop
        var playerLoop = PlayerLoop.GetCurrentPlayerLoop();
        
        InsertSystems(ref playerLoop, typeof(UnityEngine.PlayerLoop.TimeUpdate), timeSystems);
        InsertSystems(ref playerLoop, typeof(UnityEngine.PlayerLoop.Initialization), initializationSystems);
        InsertSystems(ref playerLoop, typeof(UnityEngine.PlayerLoop.EarlyUpdate), earlyUpdateSystems);
        InsertSystems(ref playerLoop, typeof(UnityEngine.PlayerLoop.FixedUpdate), fixedUpdateSystems);
        InsertSystems(ref playerLoop, typeof(UnityEngine.PlayerLoop.PreUpdate), preUpdateSystems);
        InsertSystems(ref playerLoop, typeof(UnityEngine.PlayerLoop.Update), updateSystems);
        
        // LateUpdate logic
        // PreLateUpdate -> LateUpdate -> PostLateUpdate
        // Unity only exposes PreLateUpdate and PostLateUpdate as main phases usually, 
        // but let's check standard PlayerLoop.
        // Usually PreLateUpdate contains ScriptRunBehaviourLateUpdate.
        // We will insert PreLateUpdate and LateUpdate into PreLateUpdate phase.
        // And PostLateUpdate into PostLateUpdate phase.
        
        InsertSystems(ref playerLoop, typeof(UnityEngine.PlayerLoop.PreLateUpdate), preLateUpdateSystems);
        InsertSystems(ref playerLoop, typeof(UnityEngine.PlayerLoop.PreLateUpdate), lateUpdateSystems); // Append after PreLateUpdate systems
        InsertSystems(ref playerLoop, typeof(UnityEngine.PlayerLoop.PostLateUpdate), postLateUpdateSystems);
        
        PlayerLoop.SetPlayerLoop(playerLoop);
        
        Debug.Log($"[SystemBootstrapper] Initialized systems. Total: {systems.Count}");
    }

    private static List<ISystem> GetSortedSystems(List<ISystem> systems, SystemGroup group)
    {
        var groupSystems = systems
            .Where(s => s.GetType().GetCustomAttribute<UpdateInGroupAttribute>()?.Group == group)
            .ToList();

        return TopologicalSort(groupSystems);
    }

    private static List<ISystem> TopologicalSort(List<ISystem> systems)
    {
        var sorted = new List<ISystem>();
        var visited = new HashSet<ISystem>();
        var visiting = new HashSet<ISystem>(); // For cycle detection

        // Build dependency graph
        // Map Type -> System instance
        var typeToSystem = systems.ToDictionary(s => s.GetType(), s => s);

        void Visit(ISystem system)
        {
            if (visited.Contains(system)) return;
            if (visiting.Contains(system))
            {
                Debug.LogError($"[SystemBootstrapper] Circular dependency detected involving {system.GetType().Name}");
                return;
            }

            visiting.Add(system);

            var type = system.GetType();
            
            // Handle UpdateAfter (dependencies must run first)
            var afterAttrs = type.GetCustomAttributes<UpdateAfterAttribute>();
            foreach (var attr in afterAttrs)
            {
                if (typeToSystem.TryGetValue(attr.TargetType, out var dep))
                {
                    Visit(dep);
                }
            }

            // Implicit dependency: Order in UpdateInGroup
            // We can't strictly enforce mixed Order/Attribute topological sort easily without a complex graph.
            // Simplified approach: Sort by Order first, then resolve Attribute dependencies.
            // Actually, if we use pure topological sort, we need to respect Order as well.
            // Let's assume Attributes override Order, or Order is a fallback tie-breaker.
            
            // For now, let's keep it simple: Standard topological sort only on Before/After attributes.
            // Systems without dependencies are visited. 
            // BUT we also need to respect UpdateBefore (dependents must run after).
            // UpdateBefore(T) on A means A runs before T. So T depends on A.
            
            visiting.Remove(system);
            visited.Add(system);
            sorted.Add(system);
        }

        // First, handle explicit UpdateBefore to reverse dependencies
        // If A has UpdateBefore(B), then B depends on A.
        var reverseDeps = new Dictionary<Type, List<Type>>();
        foreach (var sys in systems)
        {
            var type = sys.GetType();
            var beforeAttrs = type.GetCustomAttributes<UpdateBeforeAttribute>();
            foreach (var attr in beforeAttrs)
            {
                if (!reverseDeps.ContainsKey(attr.TargetType))
                    reverseDeps[attr.TargetType] = new List<Type>();
                reverseDeps[attr.TargetType].Add(type);
            }
        }

        // Re-implement Visit to handle both directions
        visited.Clear();
        visiting.Clear();
        sorted.Clear();

        void VisitWithFullDeps(ISystem system)
        {
            if (visited.Contains(system)) return;
            if (visiting.Contains(system))
            {
                Debug.LogError($"[SystemBootstrapper] Circular dependency detected involving {system.GetType().Name}");
                return;
            }

            visiting.Add(system);

            var type = system.GetType();

            // 1. Dependencies from UpdateAfter (these must run BEFORE current system)
            var afterAttrs = type.GetCustomAttributes<UpdateAfterAttribute>();
            foreach (var attr in afterAttrs)
            {
                if (typeToSystem.TryGetValue(attr.TargetType, out var dep))
                {
                    VisitWithFullDeps(dep);
                }
            }

            // 2. Dependencies from UpdateBefore (Reverse: If X has UpdateBefore(Current), X must run BEFORE Current)
            if (reverseDeps.TryGetValue(type, out var befores))
            {
                foreach (var beforeType in befores)
                {
                    if (typeToSystem.TryGetValue(beforeType, out var dep))
                    {
                        VisitWithFullDeps(dep);
                    }
                }
            }

            visiting.Remove(system);
            visited.Add(system);
            sorted.Add(system);
        }

        // Initial sort by Order to provide deterministic baseline
        var orderedSystems = systems.OrderBy(s => s.GetType().GetCustomAttribute<UpdateInGroupAttribute>().Order).ToList();

        foreach (var system in orderedSystems)
        {
            VisitWithFullDeps(system);
        }

        return sorted;
    }

    private static void InsertSystems(ref PlayerLoopSystem loop, Type loopType, List<ISystem> systems)
    {
        if (systems.Count == 0) return;

        if (loop.type == loopType)
        {
            var newSubSystems = new List<PlayerLoopSystem>(loop.subSystemList ?? Array.Empty<PlayerLoopSystem>());
            
            foreach (var system in systems)
            {
                newSubSystems.Add(new PlayerLoopSystem
                {
                    type = system.GetType(),
                    updateDelegate = () => 
                    {
                        try 
                        {
                            system.Update(Time.deltaTime);
                        }
                        catch (Exception e)
                        {
                            Debug.LogException(e);
                        }
                    }
                });
            }
            
            loop.subSystemList = newSubSystems.ToArray();
            return;
        }

        if (loop.subSystemList != null)
        {
            for (int i = 0; i < loop.subSystemList.Length; i++)
            {
                InsertSystems(ref loop.subSystemList[i], loopType, systems);
            }
        }
    }
}
