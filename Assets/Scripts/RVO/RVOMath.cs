using System;
using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;

public struct ORCALine
{
    public float2 Direction;
    public float2 Point;
}

public static class RVOMath
{
    private const float RVO_EPSILON = 0.00001f;

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

    public static bool linearProgram1(List<ORCALine> lines, int lineNo, float radius, float2 optVelocity, bool directionOpt, ref float2 result)
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

    public static int linearProgram2(List<ORCALine> lines, float radius, float2 optVelocity, bool directionOpt, ref float2 result)
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

        for (int i = 0; i < lines.Count; ++i)
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

        return lines.Count;
    }

    public static void linearProgram3(List<ORCALine> lines, int numObstLines, int beginLine, float radius, ref float2 result)
    {
        float distance = 0.0f;

        for (int i = beginLine; i < lines.Count; ++i)
        {
            if (det(lines[i].Direction, lines[i].Point - result) > distance)
            {
                List<ORCALine> projLines = new List<ORCALine>();
                for (int j = 0; j < numObstLines; ++j)
                {
                    projLines.Add(lines[j]);
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
                    projLines.Add(line);
                }

                float2 tempResult = result;
                if (linearProgram2(projLines, radius, new float2(-lines[i].Direction.y, lines[i].Direction.x), true, ref result) < projLines.Count)
                {
                    result = tempResult;
                }

                distance = det(lines[i].Direction, lines[i].Point - result);
            }
        }
    }

    public static void ConstructORCALines(RVOAgent agent, List<RVOAgent> neighbors, float timeStep, List<ORCALine> orcaLines)
    {
        orcaLines.Clear();
        float invTimeHorizon = 1.0f / agent.TimeHorizon;

        for (int i = 0; i < neighbors.Count; ++i)
        {
            RVOAgent other = neighbors[i];

            float2 relativePosition = other.Position - agent.Position;
            float2 relativeVelocity = agent.Velocity - other.Velocity;
            float distSq = absSq(relativePosition);
            float combinedRadius = agent.Radius + other.Radius;
            float combinedRadiusSq = combinedRadius * combinedRadius;

            ORCALine line;
            float2 u;

            if (distSq > combinedRadiusSq)
            {
                // No collision
                float2 w = relativeVelocity - invTimeHorizon * relativePosition;
                float wLengthSq = absSq(w);
                float dotProduct1 = math.dot(w, relativePosition);

                if (dotProduct1 < 0.0f && dotProduct1 * dotProduct1 > combinedRadiusSq * wLengthSq)
                {
                    // Project on cut-off circle
                    float wLength = math.sqrt(wLengthSq);
                    float2 unitW = w / wLength;

                    line.Direction = new float2(unitW.y, -unitW.x);
                    u = (combinedRadius * invTimeHorizon - wLength) * unitW;
                }
                else
                {
                    // Project on legs
                    float leg = math.sqrt(distSq - combinedRadiusSq);
                    if (det(relativePosition, w) > 0.0f)
                    {
                        // Project on left leg
                        line.Direction = new float2(relativePosition.x * leg - relativePosition.y * combinedRadius, relativePosition.x * combinedRadius + relativePosition.y * leg) / distSq;
                    }
                    else
                    {
                        // Project on right leg
                        line.Direction = -new float2(relativePosition.x * leg + relativePosition.y * combinedRadius, -relativePosition.x * combinedRadius + relativePosition.y * leg) / distSq;
                    }

                    float dotProduct2 = math.dot(relativeVelocity, line.Direction);
                    u = dotProduct2 * line.Direction - relativeVelocity;
                }
            }
            else
            {
                // Collision
                float invTimeStep = 1.0f / timeStep;
                float2 w = relativeVelocity - invTimeStep * relativePosition;
                float wLength = math.length(w);
                float2 unitW = w / wLength;

                line.Direction = new float2(unitW.y, -unitW.x);
                u = (combinedRadius * invTimeStep - wLength) * unitW;
            }

            line.Point = agent.Velocity + 0.5f * u;
            orcaLines.Add(line);
        }
    }
}
