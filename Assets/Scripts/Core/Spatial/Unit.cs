using UnityEngine;

public class Unit : MonoBehaviour
{
    public Vector2 Position;
    public Vector2 Velocity;
    public Rect Bounds;

    public void Move(float dt)
    {
        Position += Velocity * dt;

        // Bounce off walls
        if (Position.x < Bounds.xMin || Position.x > Bounds.xMax)
        {
            Velocity.x *= -1;
            Position.x = Mathf.Clamp(Position.x, Bounds.xMin, Bounds.xMax);
        }
        if (Position.y < Bounds.yMin || Position.y > Bounds.yMax)
        {
            Velocity.y *= -1;
            Position.y = Mathf.Clamp(Position.y, Bounds.yMin, Bounds.yMax);
        }
    }
}
