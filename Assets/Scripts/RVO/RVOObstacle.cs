using Unity.Mathematics;
using UnityEngine;

public class RVOObstacle
{
    public float2 Point1;
    public float2 Point2;
    public float2 Direction; // Normalized direction from Point1 to Point2
    public float2 Normal;    // Normal vector (perpendicular to Direction)
    public int Id;
    public bool IsConvex;
    public RVOObstacle NextObstacle;
    public RVOObstacle PrevObstacle;

    public RVOObstacle(float2 p1, float2 p2)
    {
        Point1 = p1;
        Point2 = p2;
        float2 diff = p2 - p1;
        float len = math.length(diff);
        if (len > 0.0001f)
        {
            Direction = diff / len;
        }
        else
        {
            Direction = new float2(1, 0);
        }
        Normal = new float2(-Direction.y, Direction.x);
    }
}
