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
                }
                catch (Exception e)
                {
                    Debug.LogError($"[SystemBootstrapper] Failed to instantiate {type.Name}: {e}");
                }
            }
        }

        // Group systems by enum
        var initializationSystems = GetSortedSystems(systems, SystemGroup.Initialization);
        var fixedUpdateSystems = GetSortedSystems(systems, SystemGroup.FixedUpdate);
        var updateSystems = GetSortedSystems(systems, SystemGroup.Update);
        var lateUpdateSystems = GetSortedSystems(systems, SystemGroup.LateUpdate);

        // Inject into PlayerLoop
        var playerLoop = PlayerLoop.GetCurrentPlayerLoop();
        InsertSystems(ref playerLoop, typeof(UnityEngine.PlayerLoop.Initialization), initializationSystems);
        InsertSystems(ref playerLoop, typeof(UnityEngine.PlayerLoop.FixedUpdate), fixedUpdateSystems);
        InsertSystems(ref playerLoop, typeof(UnityEngine.PlayerLoop.Update), updateSystems);
        InsertSystems(ref playerLoop, typeof(UnityEngine.PlayerLoop.PreLateUpdate), lateUpdateSystems); // PreLateUpdate is closest to LateUpdate logic usually
        // Note: UnityEngine.PlayerLoop.PostLateUpdate is another option, but usually logic is in PreLateUpdate or Update.
        // Let's check where MonoBehaviour LateUpdate runs. It runs in PreLateUpdate.ScriptRunBehaviourLateUpdate.
        // So injecting into PreLateUpdate is correct.
        
        PlayerLoop.SetPlayerLoop(playerLoop);
        
        Debug.Log($"[SystemBootstrapper] Initialized systems. Init: {initializationSystems.Count}, Fixed: {fixedUpdateSystems.Count}, Update: {updateSystems.Count}, Late: {lateUpdateSystems.Count}");
    }

    private static List<ISystem> GetSortedSystems(List<ISystem> systems, SystemGroup group)
    {
        return systems
            .Where(s => s.GetType().GetCustomAttribute<UpdateInGroupAttribute>()?.Group == group)
            .OrderBy(s => s.GetType().GetCustomAttribute<UpdateInGroupAttribute>().Order)
            .ToList();
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
