using System;

public enum SystemGroup
{
    Initialization,
    FixedUpdate,  // Simulation
    Update,       // Presentation
    LateUpdate
}

[AttributeUsage(AttributeTargets.Class)]
public class UpdateInGroupAttribute : Attribute
{
    public SystemGroup Group;
    public int Order;

    public UpdateInGroupAttribute(SystemGroup group, int order = 0)
    {
        Group = group;
        Order = order;
    }
}
