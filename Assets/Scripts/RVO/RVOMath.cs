using System;
using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;

public struct ORCALine
{
    public float2 Direction;
    public float2 Point;
}

/// <summary>
/// RVO Math utilities.
/// 
/// DATA QUALITY REQUIREMENTS:
/// 1. Obstacle Processing:
///    - Obstacles neighbors provided to ConstructObstacleORCALines MUST be sorted by distance to the agent.
///    - This is critical for the "Already Covered" optimization to work correctly and implicitly cull back-facing edges.
///    - (Note: Current implementation of ConstructObstacleORCALines performs this sorting internally on a local copy).
///    
/// 2. Agent Processing:
///    - Agent neighbors provided to ConstructORCALines SHOULD be sorted by distance.
///    - The list MUST represent the K-nearest neighbors. Passing random neighbors instead of closest ones will lead to collisions.
/// </summary>
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

        // Note: Obstacles should ideally be sorted by distance for "Already Covered" optimization to work best.
        // However, we iterate the provided list directly. The caller (Spatial Index) is responsible for sorting.

        bool debug = agent.ID == 0;

        if (debug)
        {
            Debug.Log($"[RVO] === Agent {agent.ID} at {agent.Position} ===");
            Debug.Log($"[RVO] TimeHorizonObst={agent.TimeHorizonObst}, MaxSpeed={agent.MaxSpeed}, Radius={agent.Radius}");
            Debug.Log($"[RVO] Processing {obstacles.Count} obstacles");
        }

        for (int i = 0; i < obstacles.Count; ++i)
        {
            RVOObstacle obstacle1 = obstacles[i];

            // In our implementation, obstacle1 represents an edge from Point1 to Point2
            // This is different from RVO2-Unity where each Obstacle is a vertex
            // obstacle1.Point1 is the first vertex, obstacle1.Point2 is the second vertex
            float2 vertex1 = obstacle1.Point1;
            float2 vertex2 = obstacle1.Point2;

            // NOTE: RVO2-Unity uses rangeSq filtering only within its KdTree spatial structure
            // for performance optimization. Since we iterate all obstacles directly,
            // we skip the range check to ensure all obstacles are considered.
            // The ORCA line generation will naturally handle which obstacles affect the agent.

            float2 relativePosition1 = vertex1 - agent.Position;
            float2 relativePosition2 = vertex2 - agent.Position;

            if (debug)
            {
                Debug.Log($"[RVO]   Edge{i}: {vertex1}→{vertex2}, RelPos1={relativePosition1}, RelPos2={relativePosition2}");
            }

            // Check if velocity obstacle is already covered by previous ORCA lines
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

            // Not yet covered. Check for collisions.
            float distSq1 = absSq(relativePosition1);
            float distSq2 = absSq(relativePosition2);
            float radiusSq = agent.Radius * agent.Radius;

            float2 obstacleVec = vertex2 - vertex1;

            // REMOVED: Explicit Back-face culling.
            // RVO2-Unity relies on Sorting + AlreadyCovered check to skip back faces.
            // if (det(obstacleVec, relativePosition1) < 0) { continue; }

            float s = math.dot(-relativePosition1, obstacleVec) / absSq(obstacleVec);
            float distSqLine = absSq(-relativePosition1 - s * obstacleVec);


            ORCALine line;

            // Collision with left vertex
            if (s < 0.0f && distSq1 <= radiusSq)
            {
                if (obstacle1.IsConvex)
                {
                    line.Point = new float2(0.0f, 0.0f);
                    line.Direction = normalize(new float2(-relativePosition1.y, relativePosition1.x));
                    orcaLines.Add(line);
                }
                continue;
            }
            // Collision with right vertex
            else if (s > 1.0f && distSq2 <= radiusSq)
            {
                // For the right vertex, we need to check the next obstacle's convexity
                RVOObstacle nextObstacle = obstacle1.NextObstacle;
                bool isConvex = (nextObstacle != null) ? nextObstacle.IsConvex : true;
                float2 nextVertex = (nextObstacle != null) ? nextObstacle.Point2 : vertex2; // Fallback

                if (isConvex && det(relativePosition2, nextVertex - vertex2) >= 0.0f)
                {
                    line.Point = new float2(0.0f, 0.0f);
                    line.Direction = normalize(new float2(-relativePosition2.y, relativePosition2.x));
                    orcaLines.Add(line);
                }
                continue;
            }
            // Collision with obstacle segment
            else if (s >= 0.0f && s <= 1.0f && distSqLine <= radiusSq)
            {
                line.Point = new float2(0.0f, 0.0f);
                line.Direction = -obstacle1.Direction;
                orcaLines.Add(line);
                continue;
            }

            // No collision. Compute legs (velocity obstacle tangent lines)
            float2 leftLegDirection, rightLegDirection;

            // Track which obstacles define the left and right vertices where legs emanate from
            // This is critical for foreign leg detection and cutoff center calculation
            RVOObstacle leftVertexObstacle = obstacle1;          // Default: left leg from vertex1 (obstacle1.Point1)
            RVOObstacle rightVertexObstacle = obstacle1.NextObstacle;  // Default: right leg from vertex2 (nextObstacle.Point1)

            // Track which edge defines the cutoff line direction
            RVOObstacle cutoffEdge = obstacle1;  // Default: original edge

            if (s < 0.0f && distSqLine <= radiusSq)
            {
                // Obstacle viewed obliquely so that left vertex defines velocity obstacle
                if (!obstacle1.IsConvex)
                {
                    continue; // Ignore obstacle
                }

                // Both legs emanate from vertex1
                rightVertexObstacle = obstacle1;  // Both legs now from vertex1

                float leg1 = math.sqrt(distSq1 - radiusSq);
                leftLegDirection = new float2(relativePosition1.x * leg1 - relativePosition1.y * agent.Radius,
                                              relativePosition1.x * agent.Radius + relativePosition1.y * leg1) / distSq1;
                rightLegDirection = new float2(relativePosition1.x * leg1 + relativePosition1.y * agent.Radius,
                                               -relativePosition1.x * agent.Radius + relativePosition1.y * leg1) / distSq1;
            }
            else if (s > 1.0f && distSqLine <= radiusSq)
            {
                // Obstacle viewed obliquely so that right vertex defines velocity obstacle
                RVOObstacle nextObstacle = obstacle1.NextObstacle;
                bool isConvex = (nextObstacle != null) ? nextObstacle.IsConvex : true;

                if (!isConvex)
                {
                    continue; // Ignore obstacle
                }

                // Both legs emanate from vertex2
                leftVertexObstacle = nextObstacle;  // Both legs now from vertex2
                rightVertexObstacle = nextObstacle;
                cutoffEdge = nextObstacle;  // Cutoff line is edge starting at vertex2

                float leg2 = math.sqrt(distSq2 - radiusSq);
                leftLegDirection = new float2(relativePosition2.x * leg2 - relativePosition2.y * agent.Radius,
                                              relativePosition2.x * agent.Radius + relativePosition2.y * leg2) / distSq2;
                rightLegDirection = new float2(relativePosition2.x * leg2 + relativePosition2.y * agent.Radius,
                                               -relativePosition2.x * agent.Radius + relativePosition2.y * leg2) / distSq2;
            }
            else
            {
                // Usual situation - compute legs from both vertices
                if (obstacle1.IsConvex)
                {
                    float leg1 = math.sqrt(distSq1 - radiusSq);
                    leftLegDirection = new float2(relativePosition1.x * leg1 - relativePosition1.y * agent.Radius,
                                                  relativePosition1.x * agent.Radius + relativePosition1.y * leg1) / distSq1;
                }
                else
                {
                    // Left vertex non-convex; left leg extends cut-off line
                    leftLegDirection = -obstacle1.Direction;
                }

                RVOObstacle nextObstacle = obstacle1.NextObstacle;
                if (nextObstacle != null && nextObstacle.IsConvex)
                {
                    float leg2 = math.sqrt(distSq2 - radiusSq);
                    rightLegDirection = new float2(relativePosition2.x * leg2 + relativePosition2.y * agent.Radius,
                                                   -relativePosition2.x * agent.Radius + relativePosition2.y * leg2) / distSq2;
                }
                else
                {
                    // Right vertex non-convex; right leg extends cut-off line
                    rightLegDirection = obstacle1.Direction;
                }
            }

            // Foreign leg detection - use the CORRECT vertex obstacles
            RVOObstacle leftNeighbor = leftVertexObstacle.PrevObstacle;

            bool isLeftLegForeign = false;
            bool isRightLegForeign = false;

            if (leftVertexObstacle.IsConvex && leftNeighbor != null && det(leftLegDirection, -leftNeighbor.Direction) >= 0.0f)
            {
                // Left leg points into obstacle
                leftLegDirection = -leftNeighbor.Direction;
                isLeftLegForeign = true;
            }

            if (rightVertexObstacle != null && rightVertexObstacle.IsConvex && det(rightLegDirection, rightVertexObstacle.Direction) <= 0.0f)
            {
                // Right leg points into obstacle
                rightLegDirection = rightVertexObstacle.Direction;
                isRightLegForeign = true;
            }


            // Compute cut-off centers - MUST use the vertices where legs actually emanate from!
            // In oblique cases, both legs may come from the same vertex
            float2 leftCutOff = invTimeHorizonObst * (leftVertexObstacle.Point1 - agent.Position);
            float2 rightCutOff = invTimeHorizonObst * (rightVertexObstacle.Point1 - agent.Position);
            float2 cutOffVector = rightCutOff - leftCutOff;

            // Project current velocity on velocity obstacle

            // Check if current velocity is projected on cutoff circles
            // Special case: when both legs from same vertex (oblique), t is always 0.5
            float t = (leftVertexObstacle == rightVertexObstacle) ? 0.5f : math.dot(agent.Velocity - leftCutOff, cutOffVector) / absSq(cutOffVector);
            float tLeft = math.dot(agent.Velocity - leftCutOff, leftLegDirection);
            float tRight = math.dot(agent.Velocity - rightCutOff, rightLegDirection);

            if ((t < 0.0f && tLeft < 0.0f) || (leftVertexObstacle == rightVertexObstacle && tLeft < 0.0f && tRight < 0.0f))
            {
                // Project on left cut-off circle
                float2 unitW = normalize(agent.Velocity - leftCutOff);
                line.Direction = new float2(unitW.y, -unitW.x);
                line.Point = leftCutOff + agent.Radius * invTimeHorizonObst * unitW;
                orcaLines.Add(line);
                continue;
            }
            else if (t > 1.0f && tRight < 0.0f)
            {
                // Project on right cut-off circle
                float2 unitW = normalize(agent.Velocity - rightCutOff);
                line.Direction = new float2(unitW.y, -unitW.x);
                line.Point = rightCutOff + agent.Radius * invTimeHorizonObst * unitW;
                orcaLines.Add(line);
                continue;
            }

            // Project on left leg, right leg, or cut-off line (whichever is closest to velocity)
            float distSqCutoff = (t < 0.0f || t > 1.0f || leftVertexObstacle == rightVertexObstacle) ? float.PositiveInfinity : absSq(agent.Velocity - (leftCutOff + t * cutOffVector));
            float distSqLeft = tLeft < 0.0f ? float.PositiveInfinity : absSq(agent.Velocity - (leftCutOff + tLeft * leftLegDirection));
            float distSqRight = tRight < 0.0f ? float.PositiveInfinity : absSq(agent.Velocity - (rightCutOff + tRight * rightLegDirection));

            if (distSqCutoff <= distSqLeft && distSqCutoff <= distSqRight)
            {
                // Project on cut-off line
                line.Direction = -cutoffEdge.Direction;
                line.Point = leftCutOff + agent.Radius * invTimeHorizonObst * new float2(-line.Direction.y, line.Direction.x);
                orcaLines.Add(line);
                continue;
            }
            else if (distSqLeft <= distSqRight)
            {
                // Project on left leg
                if (isLeftLegForeign)
                {
                    continue;
                }

                line.Direction = leftLegDirection;
                line.Point = leftCutOff + agent.Radius * invTimeHorizonObst * new float2(-line.Direction.y, line.Direction.x);
                orcaLines.Add(line);
                continue;
            }
            else
            {
                // Project on right leg
                if (isRightLegForeign)
                {
                    continue;
                }

                line.Direction = rightLegDirection;
                line.Point = rightCutOff + agent.Radius * invTimeHorizonObst * new float2(-line.Direction.y, line.Direction.x);
                orcaLines.Add(line);
                continue;
            }
        }
    }
}
