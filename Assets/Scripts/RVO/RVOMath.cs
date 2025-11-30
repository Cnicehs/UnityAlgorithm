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
            float detValue = det(lines[i].Direction, lines[i].Point - result);

            if (detValue > 0.0f)
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

    public static float LeftOf(float2 a, float2 b, float2 c)
    {
        return det(a - c, b - a);
    }

    public static void ConstructORCALines(RVOAgent agent, List<RVOAgent> neighbors, float timeStep, List<ORCALine> orcaLines)
    {
        // DO NOT CLEAR - obstacle lines were already added!
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

    public static void ConstructObstacleORCALines(RVOAgent agent, List<RVOObstacle> obstacles, float timeStep, List<ORCALine> orcaLines)
    {
        float invTimeHorizonObst = 1.0f / agent.TimeHorizonObst;
        
        // Range check logic from RVO2-Unity to filter distant obstacles
        float rangeSq = (agent.TimeHorizonObst * agent.MaxSpeed + agent.Radius) * (agent.TimeHorizonObst * agent.MaxSpeed + agent.Radius);

        for (int i = 0; i < obstacles.Count; ++i)
        {
            RVOObstacle obst1 = obstacles[i];
            RVOObstacle obst2 = obst1.NextObstacle;

            // Handle null NextObstacle (treat as endpoint)
            float2 p2 = (obst2 != null) ? obst2.Point1 : obst1.Point2;
            
            // Filter obstacles that are too far away
            float distSqToLine = distSqPointLineSegment(obst1.Point1, p2, agent.Position);
            if (distSqToLine > rangeSq)
            {
                continue;
            }

            float2 relativePosition1 = obst1.Point1 - agent.Position;
            float2 relativePosition2 = p2 - agent.Position;

            // Check if agent is behind the obstacle line. If so, ignore it.
            // Only if convex.

            bool alreadyCovered = false;

            for (int j = 0; j < orcaLines.Count; ++j)
            {
                if (det(invTimeHorizonObst * relativePosition1 - orcaLines[j].Point, orcaLines[j].Direction) - invTimeHorizonObst * agent.Radius >= -RVO_EPSILON &&
                    det(invTimeHorizonObst * relativePosition2 - orcaLines[j].Point, orcaLines[j].Direction) - invTimeHorizonObst * agent.Radius >= -RVO_EPSILON)
                {
                    alreadyCovered = true;
                    break;
                }
            }

            if (alreadyCovered)
            {
                continue;
            }

            float distSq1 = absSq(relativePosition1);
            float distSq2 = absSq(relativePosition2);
            float radius = agent.Radius;
            float radiusSq = radius * radius;

            float2 obstacleVector = p2 - obst1.Point1;
            float s = math.dot(-relativePosition1, obstacleVector) / absSq(obstacleVector);
            float distSqLine = distSqPointLineSegment(obst1.Point1, p2, agent.Position);

            ORCALine line;

            if (s < 0.0f && distSq1 <= radiusSq)
            {
                if (obst1.IsConvex)
                {
                    float2 w = new float2(-relativePosition1.y, relativePosition1.x);
                    line.Direction = normalize(w);
                    line.Point = (radius - math.sqrt(distSq1)) * invTimeHorizonObst * line.Direction;
                    line.Point += agent.Velocity;
                    orcaLines.Add(line);
                }
                continue;
            }
            else if (s > 1.0f && distSq2 <= radiusSq)
            {
                // If obst2 is null, we can't check its convexity or direction.
                // But if it's null, it's an endpoint, so effectively convex.
                // However, we don't have a "next" direction to check against.
                // So we just treat it as a point collision if we are close to it.

                bool nextIsConvex = (obst2 != null) ? obst2.IsConvex : true;
                float2 nextDirection = (obst2 != null) ? obst2.Direction : obst1.Direction; // Fallback

                if (nextIsConvex && det(relativePosition2, nextDirection) >= 0.0f)
                {
                    float2 n = normalize(relativePosition2);
                    line.Direction = new float2(n.y, -n.x);
                    line.Point = (radius - math.sqrt(distSq2)) * invTimeHorizonObst * n;
                    line.Point += agent.Velocity;
                    orcaLines.Add(line);
                }
                continue;
            }
            else if (s >= 0.0f && s <= 1.0f && distSqLine <= radiusSq)
            {
                // Collision with line segment
                // RVO2-Unity reference: collision constraints use point=(0,0) and direction=-obstacle.Direction
                line.Point = new float2(0.0f, 0.0f);
                line.Direction = -obst1.Direction;
                orcaLines.Add(line);
                continue;
            }

            // No collision
            if (s < 0.0f)
            {
                float2 w = agent.Velocity - invTimeHorizonObst * relativePosition1;
                float w_sq = absSq(w);
                if (w_sq > radiusSq)
                {
                    float2 unit_w = math.normalize(w);
                    line.Direction = new float2(unit_w.y, -unit_w.x);
                    line.Point = (radius * invTimeHorizonObst - math.sqrt(w_sq)) * unit_w;
                    line.Point += agent.Velocity;
                    orcaLines.Add(line);
                }
            }
            else if (s > 1.0f)
            {
                float2 w = agent.Velocity - invTimeHorizonObst * relativePosition2;
                float w_sq = absSq(w);
                if (w_sq > radiusSq)
                {
                    float2 unit_w = math.normalize(w);
                    line.Direction = new float2(unit_w.y, -unit_w.x);
                    line.Point = (radius * invTimeHorizonObst - math.sqrt(w_sq)) * unit_w;
                    line.Point += agent.Velocity;
                    orcaLines.Add(line);
                }
            }
            else
            {
                float2 w = agent.Velocity - invTimeHorizonObst * (relativePosition1 - s * obstacleVector);
                float w_sq = absSq(w);
                if (w_sq > radiusSq)
                {
                    float2 unit_w = math.normalize(w);
                    line.Direction = new float2(unit_w.y, -unit_w.x);
                    line.Point = (radius * invTimeHorizonObst - math.sqrt(w_sq)) * unit_w;
                    line.Point += agent.Velocity;
                    orcaLines.Add(line);
                }
            }
        }
    }
}
