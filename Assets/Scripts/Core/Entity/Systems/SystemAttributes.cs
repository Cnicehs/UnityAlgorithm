using System;

public enum SystemGroup
{
    Initialization,
    TimeUpdate,
    EarlyUpdate,
    PreUpdate,
    FixedUpdate,  // Simulation
    Update,       // Presentation
    PreLateUpdate,
    LateUpdate,
    PostLateUpdate
}

[AttributeUsage(AttributeTargets.Class, AllowMultiple = false, Inherited = true)]
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

[AttributeUsage(AttributeTargets.Class, AllowMultiple = true)]
public class UpdateBeforeAttribute : Attribute
{
    public Type TargetType;
    public UpdateBeforeAttribute(Type targetType) => TargetType = targetType;
}

[AttributeUsage(AttributeTargets.Class, AllowMultiple = true)]
public class UpdateAfterAttribute : Attribute
{
    public Type TargetType;
    public UpdateAfterAttribute(Type targetType) => TargetType = targetType;
}
