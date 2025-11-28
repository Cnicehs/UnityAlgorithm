using Unity.Burst;
using Unity.Mathematics;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;

[BurstCompile]
public static unsafe class SIMDRVO
{
    private const float RVO_EPSILON = 0.00001f;

    public struct AgentData
    {
        public float2 Position;
        public float2 Velocity;
        public float2 PrefVelocity;
        public float Radius;
        public float MaxSpeed;
        public float NeighborDist;
        public int MaxNeighbors;
        public float TimeHorizon;
        public float TimeHorizonObst;
    }

    public static float det(float2 v1, float2 v2)
    {
        return v1.x * v2.y - v1.y * v2.x;
    }

    public static float absSq(float2 v)
    {
        return math.dot(v, v);
    }

    public static float2 normalize(float2 v)
    {
        return math.normalize(v);
    }

    public static float distSqPointLineSegment(float2 v1, float2 v2, float2 v3)
    {
        float r = math.dot(v3 - v1, v2 - v1) / absSq(v2 - v1);
        if (r < 0.0f) return absSq(v3 - v1);
        if (r > 1.0f) return absSq(v3 - v2);
        return absSq(v3 - (v1 + r * (v2 - v1)));
    }

    public static bool linearProgram1(ORCALine* lines, int lineNo, float radius, float2 optVelocity, bool directionOpt, ref float2 result)
    {
        float dotProduct = math.dot(lines[lineNo].Point, lines[lineNo].Direction);
        float discriminant = dotProduct * dotProduct + radius * radius - absSq(lines[lineNo].Point);

        if (discriminant < 0.0f)
        {
            return false;
        }

        float sqrtDiscriminant = math.sqrt(discriminant);
        float tLeft = -dotProduct - sqrtDiscriminant;
        float tRight = -dotProduct + sqrtDiscriminant;

        for (int i = 0; i < lineNo; ++i)
        {
            float denominator = det(lines[lineNo].Direction, lines[i].Direction);
            float numerator = det(lines[i].Direction, lines[lineNo].Point - lines[i].Point);

            if (math.abs(denominator) <= RVO_EPSILON)
            {
                if (numerator < 0.0f)
                {
                    return false;
                }
                continue;
            }

            float t = numerator / denominator;

            if (denominator > 0.0f)
            {
                tRight = math.min(tRight, t);
            }
            else
            {
                tLeft = math.max(tLeft, t);
            }

            if (tLeft > tRight)
            {
                return false;
            }
        }

        if (directionOpt)
        {
            if (math.dot(optVelocity, lines[lineNo].Direction) > 0.0f)
            {
                result = lines[lineNo].Point + tRight * lines[lineNo].Direction;
            }
            else
            {
                result = lines[lineNo].Point + tLeft * lines[lineNo].Direction;
            }
        }
        else
        {
            float t = math.dot(lines[lineNo].Direction, (optVelocity - lines[lineNo].Point));
            if (t < tLeft)
            {
                result = lines[lineNo].Point + tLeft * lines[lineNo].Direction;
            }
            else if (t > tRight)
            {
                result = lines[lineNo].Point + tRight * lines[lineNo].Direction;
            }
            else
            {
                result = lines[lineNo].Point + t * lines[lineNo].Direction;
            }
        }

        return true;
    }

    public static int linearProgram2(ORCALine* lines, int lineCount, float radius, float2 optVelocity, bool directionOpt, ref float2 result)
    {
        if (directionOpt)
        {
            result = optVelocity * radius;
        }
        else if (absSq(optVelocity) > radius * radius)
        {
            result = normalize(optVelocity) * radius;
        }
        else
        {
            result = optVelocity;
        }

        for (int i = 0; i < lineCount; ++i)
        {
            if (det(lines[i].Direction, lines[i].Point - result) > 0.0f)
            {
                float2 tempResult = result;
                if (!linearProgram1(lines, i, radius, optVelocity, directionOpt, ref result))
                {
                    result = tempResult;
                    return i;
                }
            }
        }

        return lineCount;
    }

    public static void linearProgram3(ORCALine* lines, int lineCount, int numObstLines, int beginLine, float radius, ref float2 result)
    {
        float distance = 0.0f;

        for (int i = beginLine; i < lineCount; ++i)
        {
            if (det(lines[i].Direction, lines[i].Point - result) > distance)
            {
                // We need a temporary list of lines for the recursive call.
                // Since we are in Burst/Unsafe, we can stackalloc or use a pre-allocated buffer.
                // However, linearProgram2 expects a pointer to ORCALines.
                // We can copy the relevant lines to a temporary buffer.
                // Max lines is usually small (MaxNeighbors + Obstacles).
                // Let's assume a max limit for stack allocation.
                
                const int MAX_LINES = 128; // Should be enough for neighbors
                ORCALine* projLines = stackalloc ORCALine[MAX_LINES];
                int projLineCount = 0;

                for (int j = 0; j < numObstLines; ++j)
                {
                    projLines[projLineCount++] = lines[j];
                }

                for (int j = numObstLines; j < i; ++j)
                {
                    ORCALine line;
                    float determinant = det(lines[i].Direction, lines[j].Direction);
                    if (math.abs(determinant) <= RVO_EPSILON)
                    {
                        if (math.dot(lines[i].Direction, lines[j].Direction) > 0.0f)
                        {
                            continue;
                        }
                        else
                        {
                            line.Point = 0.5f * (lines[i].Point + lines[j].Point);
                        }
                    }
                    else
                    {
                        line.Point = lines[i].Point + (det(lines[j].Direction, lines[i].Point - lines[j].Point) / determinant) * lines[i].Direction;
                    }

                    line.Direction = normalize(lines[j].Direction - lines[i].Direction);
                    projLines[projLineCount++] = line;
                }

                float2 tempResult = result;
                if (linearProgram2(projLines, projLineCount, radius, new float2(-lines[i].Direction.y, lines[i].Direction.x), true, ref result) < projLineCount)
                {
                    result = tempResult;
                }

                distance = det(lines[i].Direction, lines[i].Point - result);
            }
        }
    }
}
