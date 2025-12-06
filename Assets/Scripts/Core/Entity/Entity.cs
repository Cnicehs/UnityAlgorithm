using System;

[Serializable]
public struct Entity : IEquatable<Entity>
{
    public int Id;
    public int Version;

    public static Entity Null => new Entity { Id = -1, Version = 0 };

    public bool Equals(Entity other) => Id == other.Id && Version == other.Version;
    public override bool Equals(object obj) => obj is Entity other && Equals(other);
    public override int GetHashCode() => HashCode.Combine(Id, Version);
    public static bool operator ==(Entity left, Entity right) => left.Equals(right);
    public static bool operator !=(Entity left, Entity right) => !left.Equals(right);
    public override string ToString() => $"Entity({Id}:{Version})";
}
