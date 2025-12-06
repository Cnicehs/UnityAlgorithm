using System;
using Unity.Burst;
using Unity.Mathematics;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;

/// <summary>
/// SIMD RVO implementation using Burst.
/// 
/// DATA QUALITY REQUIREMENTS:
/// 1. Neighbor Lists:
///    - The 'neighborIndices' provided to ComputeRVOVelocities MUST contain the K-nearest neighbors for each agent.
///    - While strict sorting is not required for correctness of a single linear program, ensuring neighbors are sorted
///      by distance is STRONGLY RECOMMENDED to ensure that if MaxNeighbors is hit, the closest (most critical) ones are kept.
///      (SIMDRVOSimulator now performs this sorting).
/// </summary>
[BurstCompile]
public static unsafe class SIMDRVO
{
    public const float RVO_EPSILON = 0.00001f;

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

    public struct ObstacleData
    {
        public float2 Point1;
        public float2 Point2;
        public float2 Direction;
        public int NextObstacleIdx; // Index in the NativeArray (-1 if null)
        public int PrevObstacleIdx; // Index in the NativeArray (-1 if null)
        public bool IsConvex;
    }

    public struct ObstacleTreeNode
    {
        public int ObstacleIndex;
        public int LeftNodeIndex;
        public int RightNodeIndex;
        public float MinX, MaxX, MinY, MaxY;
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

    // Helper struct for sorting obstacles
    private struct ObstacleDist : IComparable<ObstacleDist>
    {
        public int Index;
        public float DistSq;
        public int CompareTo(ObstacleDist other) => DistSq.CompareTo(other.DistSq);
    }

    [BurstCompile]
    public static unsafe void ComputeRVOVelocities(
        AgentData* agents,
        int* neighborIndices,
        int* neighborCounts,
        int* neighborOffsets,
        ObstacleData* obstacles,
        int obstacleCount,
        ObstacleTreeNode* treeNodes,
        int rootNodeIndex,
        float2* newVelocities,
        int agentCount,
        float timeStep)
    {
        const int MAX_LINES = 512;
        const int MAX_OBSTACLES = 256;

        ObstacleDist* obstacleDists = stackalloc ObstacleDist[MAX_OBSTACLES];
        ORCALine* orcaLines = stackalloc ORCALine[MAX_LINES];

        for (int agentIdx = 0; agentIdx < agentCount; agentIdx++)
        {
            AgentData agent = agents[agentIdx];
            // Extract fields for compatibility with shared logic
            float2 position = agent.Position;
            float2 velocity = agent.Velocity;
            float radius = agent.Radius;
            float maxSpeed = agent.MaxSpeed;
            float timeHorizon = agent.TimeHorizon;
            float timeHorizonObst = agent.TimeHorizonObst;
            float2 prefVelocity = agent.PrefVelocity;

            int count = neighborCounts[agentIdx];
            int offset = neighborOffsets[agentIdx];

            int lineCount = 0;
            
            float invTimeHorizonObst = 1.0f / timeHorizonObst;

            // 1. Process Obstacles
            int validObstacleCount = 0;

            if (treeNodes != null && rootNodeIndex != -1)
            {
                float rangeSq = (timeHorizonObst * maxSpeed + radius); 
                rangeSq = rangeSq * rangeSq;

                QueryObstacleTreeRecursive(
                    position,
                    rangeSq,
                    treeNodes,
                    obstacles,
                    rootNodeIndex,
                    obstacleDists,
                    ref validObstacleCount,
                    MAX_OBSTACLES
                );
            }
            else if (obstacleCount > 0)
            {
                for (int i = 0; i < obstacleCount; i++)
                {
                    float distSq = distSqPointLineSegment(obstacles[i].Point1, obstacles[i].Point2, position);
                    
                    if (validObstacleCount < MAX_OBSTACLES)
                    {
                        obstacleDists[validObstacleCount] = new ObstacleDist { Index = i, DistSq = distSq };
                        validObstacleCount++;
                    }
                }

                for (int i = 1; i < validObstacleCount; i++)
                {
                    ObstacleDist key = obstacleDists[i];
                    int j = i - 1;
                    while (j >= 0 && obstacleDists[j].DistSq > key.DistSq)
                    {
                        obstacleDists[j + 1] = obstacleDists[j];
                        j--;
                    }
                    obstacleDists[j + 1] = key;
                }
            }

            if (validObstacleCount > 0)
            {
                for (int i = 0; i < validObstacleCount; i++)
                {
                    int obstIdx = obstacleDists[i].Index;
                    ObstacleData obstacle1 = obstacles[obstIdx];
                    
                    bool alreadyCovered = false;
                    for (int j = 0; j < lineCount; ++j)
                    {
                        if (det(invTimeHorizonObst * (obstacle1.Point1 - position) - orcaLines[j].Point, orcaLines[j].Direction) - invTimeHorizonObst * radius >= -RVO_EPSILON &&
                            det(invTimeHorizonObst * (obstacle1.Point2 - position) - orcaLines[j].Point, orcaLines[j].Direction) - invTimeHorizonObst * radius >= -RVO_EPSILON)
                        {
                            alreadyCovered = true;
                            break;
                        }
                    }
                    if (alreadyCovered) continue;

                    float2 relativePosition1 = obstacle1.Point1 - position;
                    float2 relativePosition2 = obstacle1.Point2 - position;
                    
                    float distSq1 = absSq(relativePosition1);
                    float distSq2 = absSq(relativePosition2);
                    float radiusSq = radius * radius;

                    float2 obstacleVec = obstacle1.Point2 - obstacle1.Point1;
                    float s = math.dot(-relativePosition1, obstacleVec) / absSq(obstacleVec);
                    float distSqLine = absSq(-relativePosition1 - s * obstacleVec);

                    ORCALine line = default;
                    bool lineAdded = false;

                    if (s < 0.0f && distSq1 <= radiusSq)
                    {
                        if (obstacle1.IsConvex)
                        {
                            line.Point = new float2(0.0f, 0.0f);
                            line.Direction = normalize(new float2(-relativePosition1.y, relativePosition1.x));
                            lineAdded = true;
                        }
                    }
                    else if (s > 1.0f && distSq2 <= radiusSq)
                    {
                        bool nextIsConvex = true;
                        if (obstacle1.NextObstacleIdx != -1)
                        {
                            ObstacleData nextObs = obstacles[obstacle1.NextObstacleIdx];
                            nextIsConvex = nextObs.IsConvex;
                        }

                        if (nextIsConvex)
                        {
                            line.Point = new float2(0.0f, 0.0f);
                            line.Direction = normalize(new float2(-relativePosition2.y, relativePosition2.x));
                            lineAdded = true;
                        }
                    }
                    else if (s >= 0.0f && s <= 1.0f && distSqLine <= radiusSq)
                    {
                        line.Point = new float2(0.0f, 0.0f);
                        line.Direction = -obstacle1.Direction;
                        lineAdded = true;
                    }
                    else
                    {
                        float2 leftLegDirection, rightLegDirection;
                        bool isLeftLegForeign = false;
                        bool isRightLegForeign = false;

                        if (s < 0.0f && distSqLine <= radiusSq)
                        {
                            if (!obstacle1.IsConvex) continue;
                            
                            obstacle1 = obstacles[obstIdx];
                            
                            float leg1 = math.sqrt(distSq1 - radiusSq);
                            leftLegDirection = new float2(relativePosition1.x * leg1 - relativePosition1.y * radius, relativePosition1.x * radius + relativePosition1.y * leg1) / distSq1;
                            
                            float leg2 = math.sqrt(distSq2 - radiusSq);
                            rightLegDirection = new float2(relativePosition2.x * leg2 + relativePosition2.y * radius, -relativePosition2.x * radius + relativePosition2.y * leg2) / distSq2;
                        }
                        else if (s > 1.0f && distSqLine <= radiusSq)
                        {
                             float leg1 = math.sqrt(distSq1 - radiusSq);
                             leftLegDirection = new float2(relativePosition1.x * leg1 - relativePosition1.y * radius, relativePosition1.x * radius + relativePosition1.y * leg1) / distSq1;
                             float leg2 = math.sqrt(distSq2 - radiusSq);
                             rightLegDirection = new float2(relativePosition2.x * leg2 + relativePosition2.y * radius, -relativePosition2.x * radius + relativePosition2.y * leg2) / distSq2;
                        }
                        else
                        {
                            if (obstacle1.IsConvex)
                            {
                                float leg1 = math.sqrt(distSq1 - radiusSq);
                                leftLegDirection = new float2(relativePosition1.x * leg1 - relativePosition1.y * radius, relativePosition1.x * radius + relativePosition1.y * leg1) / distSq1;
                            }
                            else leftLegDirection = -obstacle1.Direction;

                            bool nextIsConvex = true;
                            if (obstacle1.NextObstacleIdx != -1) nextIsConvex = obstacles[obstacle1.NextObstacleIdx].IsConvex;
                            
                            if (nextIsConvex)
                            {
                                float leg2 = math.sqrt(distSq2 - radiusSq);
                                rightLegDirection = new float2(relativePosition2.x * leg2 + relativePosition2.y * radius, -relativePosition2.x * radius + relativePosition2.y * leg2) / distSq2;
                            }
                            else rightLegDirection = obstacle1.Direction;
                        }

                        ObstacleData leftNeighbor = default;
                        bool hasLeft = obstacle1.PrevObstacleIdx != -1;
                        if (hasLeft) leftNeighbor = obstacles[obstacle1.PrevObstacleIdx];

                        if (obstacle1.IsConvex && hasLeft && det(leftLegDirection, -leftNeighbor.Direction) >= 0.0f)
                        {
                            leftLegDirection = -leftNeighbor.Direction;
                            isLeftLegForeign = true;
                        }

                        ObstacleData rightNeighbor = default;
                        bool hasRight = obstacle1.NextObstacleIdx != -1;
                        if (hasRight) rightNeighbor = obstacles[obstacle1.NextObstacleIdx];

                        if (hasRight && rightNeighbor.IsConvex && det(rightLegDirection, rightNeighbor.Direction) <= 0.0f)
                        {
                            rightLegDirection = rightNeighbor.Direction;
                            isRightLegForeign = true;
                        }

                        float2 leftCutOff = invTimeHorizonObst * (obstacle1.Point1 - position);
                        float2 rightCutOff = invTimeHorizonObst * (obstacle1.Point2 - position);
                        float2 cutOffVector = rightCutOff - leftCutOff;

                        float t = (obstacle1.Point1.Equals(obstacle1.Point2)) ? 0.5f : math.dot(velocity - leftCutOff, cutOffVector) / absSq(cutOffVector);
                        float tLeft = math.dot(velocity - leftCutOff, leftLegDirection);
                        float tRight = math.dot(velocity - rightCutOff, rightLegDirection);

                        if ((t < 0.0f && tLeft < 0.0f) || (obstacle1.Point1.Equals(obstacle1.Point2) && tLeft < 0.0f && tRight < 0.0f))
                        {
                            float2 unitW = normalize(velocity - leftCutOff);
                            line.Direction = new float2(unitW.y, -unitW.x);
                            line.Point = leftCutOff + radius * invTimeHorizonObst * unitW;
                            lineAdded = true;
                        }
                        else if (t > 1.0f && tRight < 0.0f)
                        {
                            float2 unitW = normalize(velocity - rightCutOff);
                            line.Direction = new float2(unitW.y, -unitW.x);
                            line.Point = rightCutOff + radius * invTimeHorizonObst * unitW;
                            lineAdded = true;
                        }
                        else if (t >= 0.0f && t <= 1.0f)
                        {
                             float distSqCutoff = absSq(velocity - (leftCutOff + t * cutOffVector));
                             float distSqLeft = tLeft < 0.0f ? float.PositiveInfinity : absSq(velocity - (leftCutOff + tLeft * leftLegDirection));
                             float distSqRight = tRight < 0.0f ? float.PositiveInfinity : absSq(velocity - (rightCutOff + tRight * rightLegDirection));

                             if (distSqCutoff <= distSqLeft && distSqCutoff <= distSqRight)
                             {
                                 line.Direction = -obstacle1.Direction;
                                 line.Point = leftCutOff + radius * invTimeHorizonObst * new float2(-line.Direction.y, line.Direction.x);
                                 lineAdded = true;
                             }
                             else if (distSqLeft <= distSqRight)
                             {
                                 if (!isLeftLegForeign)
                                 {
                                     line.Direction = leftLegDirection;
                                     line.Point = leftCutOff + radius * invTimeHorizonObst * new float2(-line.Direction.y, line.Direction.x);
                                     lineAdded = true;
                                 }
                             }
                             else
                             {
                                 if (!isRightLegForeign)
                                 {
                                     line.Direction = rightLegDirection;
                                     line.Point = rightCutOff + radius * invTimeHorizonObst * new float2(-line.Direction.y, line.Direction.x);
                                     lineAdded = true;
                                 }
                             }
                        }
                    }

                    if (lineAdded && lineCount < MAX_LINES)
                    {
                        orcaLines[lineCount++] = line;
                    }
                }
            }

            // 2. Process Agents
            float invTimeHorizon = 1.0f / timeHorizon;

            for (int i = 0; i < count; ++i)
            {
                int neighborIdx = neighborIndices[offset + i];
                // Access neighbors from AoS
                AgentData other = agents[neighborIdx];
                
                float2 otherPos = other.Position;
                float2 otherVel = other.Velocity;
                float otherRadius = other.Radius;

                float2 relativePosition = otherPos - position;
                float2 relativeVelocity = velocity - otherVel;
                float distSq = math.lengthsq(relativePosition);
                float combinedRadius = radius + otherRadius;
                float combinedRadiusSq = combinedRadius * combinedRadius;

                ORCALine line;
                float2 u;

                if (distSq > combinedRadiusSq)
                {
                    float2 w = relativeVelocity - invTimeHorizon * relativePosition;
                    float wLengthSq = math.lengthsq(w);
                    float dotProduct1 = math.dot(w, relativePosition);

                    if (dotProduct1 < 0.0f && dotProduct1 * dotProduct1 > combinedRadiusSq * wLengthSq)
                    {
                        float wLength = math.sqrt(wLengthSq);
                        float2 unitW = w / wLength;

                        line.Direction = new float2(unitW.y, -unitW.x);
                        u = (combinedRadius * invTimeHorizon - wLength) * unitW;
                    }
                    else
                    {
                        float leg = math.sqrt(distSq - combinedRadiusSq);
                        if (det(relativePosition, w) > 0.0f)
                        {
                            line.Direction = new float2(relativePosition.x * leg - relativePosition.y * combinedRadius, 
                                                       relativePosition.x * combinedRadius + relativePosition.y * leg) / distSq;
                        }
                        else
                        {
                            line.Direction = -new float2(relativePosition.x * leg + relativePosition.y * combinedRadius, 
                                                        -relativePosition.x * combinedRadius + relativePosition.y * leg) / distSq;
                        }

                        float dotProduct2 = math.dot(relativeVelocity, line.Direction);
                        u = dotProduct2 * line.Direction - relativeVelocity;
                    }
                }
                else
                {
                    float invTimeStep = 1.0f / timeStep;
                    float2 w = relativeVelocity - invTimeStep * relativePosition;
                    float wLength = math.length(w);
                    float2 unitW = w / wLength;

                    line.Direction = new float2(unitW.y, -unitW.x);
                    u = (combinedRadius * invTimeStep - wLength) * unitW;
                }

                line.Point = velocity + 0.5f * u;
                
                if (lineCount < MAX_LINES)
                {
                    orcaLines[lineCount++] = line;
                }
            }

            float2 newVel = float2.zero;
            int lineFail = linearProgram2(orcaLines, lineCount, maxSpeed, prefVelocity, false, ref newVel);

            if (lineFail < lineCount)
            {
                linearProgram3(orcaLines, lineCount, 0, lineFail, maxSpeed, ref newVel);
            }

            newVelocities[agentIdx] = newVel;
        }
    }

    [BurstCompile]
    public static unsafe void ComputeRVOVelocities(
        float2* positions,
        float2* velocities,
        float* radii,
        AgentParameters* agentParams,
        float2* prefVelocities,
        int* neighborIndices,
        int* neighborCounts,
        int* neighborOffsets,
        ObstacleData* obstacles,
        int obstacleCount,
        ObstacleTreeNode* treeNodes,
        int rootNodeIndex,
        float2* newVelocities,
        int agentCount,
        float timeStep)
    {
        const int MAX_LINES = 512;
        const int MAX_OBSTACLES = 256;

        ObstacleDist* obstacleDists = stackalloc ObstacleDist[MAX_OBSTACLES];
        ORCALine* orcaLines = stackalloc ORCALine[MAX_LINES];

        for (int agentIdx = 0; agentIdx < agentCount; agentIdx++)
        {
            float2 position = positions[agentIdx];
            float2 velocity = velocities[agentIdx];
            float radius = radii[agentIdx];
            AgentParameters param = agentParams[agentIdx];
            float2 prefVelocity = prefVelocities[agentIdx];

            int count = neighborCounts[agentIdx];
            int offset = neighborOffsets[agentIdx];

            int lineCount = 0;
            
            float invTimeHorizonObst = 1.0f / param.TimeHorizonObst;

            // 1. Process Obstacles
            int validObstacleCount = 0;

            if (treeNodes != null && rootNodeIndex != -1)
            {
                float rangeSq = (param.TimeHorizonObst * param.MaxSpeed + radius); 
                rangeSq = rangeSq * rangeSq;

                QueryObstacleTreeRecursive(
                    position,
                    rangeSq,
                    treeNodes,
                    obstacles,
                    rootNodeIndex,
                    obstacleDists,
                    ref validObstacleCount,
                    MAX_OBSTACLES
                );
            }
            else if (obstacleCount > 0)
            {
                for (int i = 0; i < obstacleCount; i++)
                {
                    float distSq = distSqPointLineSegment(obstacles[i].Point1, obstacles[i].Point2, position);
                    
                    if (validObstacleCount < MAX_OBSTACLES)
                    {
                        obstacleDists[validObstacleCount] = new ObstacleDist { Index = i, DistSq = distSq };
                        validObstacleCount++;
                    }
                }

                for (int i = 1; i < validObstacleCount; i++)
                {
                    ObstacleDist key = obstacleDists[i];
                    int j = i - 1;
                    while (j >= 0 && obstacleDists[j].DistSq > key.DistSq)
                    {
                        obstacleDists[j + 1] = obstacleDists[j];
                        j--;
                    }
                    obstacleDists[j + 1] = key;
                }
            }

            if (validObstacleCount > 0)
            {
                for (int i = 0; i < validObstacleCount; i++)
                {
                    int obstIdx = obstacleDists[i].Index;
                    ObstacleData obstacle1 = obstacles[obstIdx];
                    
                    bool alreadyCovered = false;
                    for (int j = 0; j < lineCount; ++j)
                    {
                        if (det(invTimeHorizonObst * (obstacle1.Point1 - position) - orcaLines[j].Point, orcaLines[j].Direction) - invTimeHorizonObst * radius >= -RVO_EPSILON &&
                            det(invTimeHorizonObst * (obstacle1.Point2 - position) - orcaLines[j].Point, orcaLines[j].Direction) - invTimeHorizonObst * radius >= -RVO_EPSILON)
                        {
                            alreadyCovered = true;
                            break;
                        }
                    }
                    if (alreadyCovered) continue;

                    float2 relativePosition1 = obstacle1.Point1 - position;
                    float2 relativePosition2 = obstacle1.Point2 - position;
                    
                    float distSq1 = absSq(relativePosition1);
                    float distSq2 = absSq(relativePosition2);
                    float radiusSq = radius * radius;

                    float2 obstacleVec = obstacle1.Point2 - obstacle1.Point1;
                    float s = math.dot(-relativePosition1, obstacleVec) / absSq(obstacleVec);
                    float distSqLine = absSq(-relativePosition1 - s * obstacleVec);

                    ORCALine line = default;
                    bool lineAdded = false;

                    if (s < 0.0f && distSq1 <= radiusSq)
                    {
                        if (obstacle1.IsConvex)
                        {
                            line.Point = new float2(0.0f, 0.0f);
                            line.Direction = normalize(new float2(-relativePosition1.y, relativePosition1.x));
                            lineAdded = true;
                        }
                    }
                    else if (s > 1.0f && distSq2 <= radiusSq)
                    {
                        bool nextIsConvex = true;
                        if (obstacle1.NextObstacleIdx != -1)
                        {
                            ObstacleData nextObs = obstacles[obstacle1.NextObstacleIdx];
                            nextIsConvex = nextObs.IsConvex;
                        }

                        if (nextIsConvex)
                        {
                            line.Point = new float2(0.0f, 0.0f);
                            line.Direction = normalize(new float2(-relativePosition2.y, relativePosition2.x));
                            lineAdded = true;
                        }
                    }
                    else if (s >= 0.0f && s <= 1.0f && distSqLine <= radiusSq)
                    {
                        line.Point = new float2(0.0f, 0.0f);
                        line.Direction = -obstacle1.Direction;
                        lineAdded = true;
                    }
                    else
                    {
                        float2 leftLegDirection, rightLegDirection;
                        bool isLeftLegForeign = false;
                        bool isRightLegForeign = false;

                        if (s < 0.0f && distSqLine <= radiusSq)
                        {
                            if (!obstacle1.IsConvex) continue;
                            
                            obstacle1 = obstacles[obstIdx];
                            
                            float leg1 = math.sqrt(distSq1 - radiusSq);
                            leftLegDirection = new float2(relativePosition1.x * leg1 - relativePosition1.y * radius, relativePosition1.x * radius + relativePosition1.y * leg1) / distSq1;
                            
                            float leg2 = math.sqrt(distSq2 - radiusSq);
                            rightLegDirection = new float2(relativePosition2.x * leg2 + relativePosition2.y * radius, -relativePosition2.x * radius + relativePosition2.y * leg2) / distSq2;
                        }
                        else if (s > 1.0f && distSqLine <= radiusSq)
                        {
                             float leg1 = math.sqrt(distSq1 - radiusSq);
                             leftLegDirection = new float2(relativePosition1.x * leg1 - relativePosition1.y * radius, relativePosition1.x * radius + relativePosition1.y * leg1) / distSq1;
                             float leg2 = math.sqrt(distSq2 - radiusSq);
                             rightLegDirection = new float2(relativePosition2.x * leg2 + relativePosition2.y * radius, -relativePosition2.x * radius + relativePosition2.y * leg2) / distSq2;
                        }
                        else
                        {
                            if (obstacle1.IsConvex)
                            {
                                float leg1 = math.sqrt(distSq1 - radiusSq);
                                leftLegDirection = new float2(relativePosition1.x * leg1 - relativePosition1.y * radius, relativePosition1.x * radius + relativePosition1.y * leg1) / distSq1;
                            }
                            else leftLegDirection = -obstacle1.Direction;

                            bool nextIsConvex = true;
                            if (obstacle1.NextObstacleIdx != -1) nextIsConvex = obstacles[obstacle1.NextObstacleIdx].IsConvex;
                            
                            if (nextIsConvex)
                            {
                                float leg2 = math.sqrt(distSq2 - radiusSq);
                                rightLegDirection = new float2(relativePosition2.x * leg2 + relativePosition2.y * radius, -relativePosition2.x * radius + relativePosition2.y * leg2) / distSq2;
                            }
                            else rightLegDirection = obstacle1.Direction;
                        }

                        ObstacleData leftNeighbor = default;
                        bool hasLeft = obstacle1.PrevObstacleIdx != -1;
                        if (hasLeft) leftNeighbor = obstacles[obstacle1.PrevObstacleIdx];

                        if (obstacle1.IsConvex && hasLeft && det(leftLegDirection, -leftNeighbor.Direction) >= 0.0f)
                        {
                            leftLegDirection = -leftNeighbor.Direction;
                            isLeftLegForeign = true;
                        }

                        ObstacleData rightNeighbor = default;
                        bool hasRight = obstacle1.NextObstacleIdx != -1;
                        if (hasRight) rightNeighbor = obstacles[obstacle1.NextObstacleIdx];

                        if (hasRight && rightNeighbor.IsConvex && det(rightLegDirection, rightNeighbor.Direction) <= 0.0f)
                        {
                            rightLegDirection = rightNeighbor.Direction;
                            isRightLegForeign = true;
                        }

                        float2 leftCutOff = invTimeHorizonObst * (obstacle1.Point1 - position);
                        float2 rightCutOff = invTimeHorizonObst * (obstacle1.Point2 - position);
                        float2 cutOffVector = rightCutOff - leftCutOff;

                        float t = (obstacle1.Point1.Equals(obstacle1.Point2)) ? 0.5f : math.dot(velocity - leftCutOff, cutOffVector) / absSq(cutOffVector);
                        float tLeft = math.dot(velocity - leftCutOff, leftLegDirection);
                        float tRight = math.dot(velocity - rightCutOff, rightLegDirection);

                        if ((t < 0.0f && tLeft < 0.0f) || (obstacle1.Point1.Equals(obstacle1.Point2) && tLeft < 0.0f && tRight < 0.0f))
                        {
                            float2 unitW = normalize(velocity - leftCutOff);
                            line.Direction = new float2(unitW.y, -unitW.x);
                            line.Point = leftCutOff + radius * invTimeHorizonObst * unitW;
                            lineAdded = true;
                        }
                        else if (t > 1.0f && tRight < 0.0f)
                        {
                            float2 unitW = normalize(velocity - rightCutOff);
                            line.Direction = new float2(unitW.y, -unitW.x);
                            line.Point = rightCutOff + radius * invTimeHorizonObst * unitW;
                            lineAdded = true;
                        }
                        else if (t >= 0.0f && t <= 1.0f)
                        {
                             float distSqCutoff = absSq(velocity - (leftCutOff + t * cutOffVector));
                             float distSqLeft = tLeft < 0.0f ? float.PositiveInfinity : absSq(velocity - (leftCutOff + tLeft * leftLegDirection));
                             float distSqRight = tRight < 0.0f ? float.PositiveInfinity : absSq(velocity - (rightCutOff + tRight * rightLegDirection));

                             if (distSqCutoff <= distSqLeft && distSqCutoff <= distSqRight)
                             {
                                 line.Direction = -obstacle1.Direction;
                                 line.Point = leftCutOff + radius * invTimeHorizonObst * new float2(-line.Direction.y, line.Direction.x);
                                 lineAdded = true;
                             }
                             else if (distSqLeft <= distSqRight)
                             {
                                 if (!isLeftLegForeign)
                                 {
                                     line.Direction = leftLegDirection;
                                     line.Point = leftCutOff + radius * invTimeHorizonObst * new float2(-line.Direction.y, line.Direction.x);
                                     lineAdded = true;
                                 }
                             }
                             else
                             {
                                 if (!isRightLegForeign)
                                 {
                                     line.Direction = rightLegDirection;
                                     line.Point = rightCutOff + radius * invTimeHorizonObst * new float2(-line.Direction.y, line.Direction.x);
                                     lineAdded = true;
                                 }
                             }
                        }
                    }

                    if (lineAdded && lineCount < MAX_LINES)
                    {
                        orcaLines[lineCount++] = line;
                    }
                }
            }

            // 2. Process Agents
            float invTimeHorizon = 1.0f / param.TimeHorizon;

            for (int i = 0; i < count; ++i)
            {
                int neighborIdx = neighborIndices[offset + i];
                // Note: neighborIdx is global index. Access SoA arrays.
                
                float2 otherPos = positions[neighborIdx];
                float2 otherVel = velocities[neighborIdx];
                float otherRadius = radii[neighborIdx];

                float2 relativePosition = otherPos - position;
                float2 relativeVelocity = velocity - otherVel;
                float distSq = math.lengthsq(relativePosition);
                float combinedRadius = radius + otherRadius;
                float combinedRadiusSq = combinedRadius * combinedRadius;

                ORCALine line;
                float2 u;

                if (distSq > combinedRadiusSq)
                {
                    float2 w = relativeVelocity - invTimeHorizon * relativePosition;
                    float wLengthSq = math.lengthsq(w);
                    float dotProduct1 = math.dot(w, relativePosition);

                    if (dotProduct1 < 0.0f && dotProduct1 * dotProduct1 > combinedRadiusSq * wLengthSq)
                    {
                        float wLength = math.sqrt(wLengthSq);
                        float2 unitW = w / wLength;

                        line.Direction = new float2(unitW.y, -unitW.x);
                        u = (combinedRadius * invTimeHorizon - wLength) * unitW;
                    }
                    else
                    {
                        float leg = math.sqrt(distSq - combinedRadiusSq);
                        if (det(relativePosition, w) > 0.0f)
                        {
                            line.Direction = new float2(relativePosition.x * leg - relativePosition.y * combinedRadius, 
                                                       relativePosition.x * combinedRadius + relativePosition.y * leg) / distSq;
                        }
                        else
                        {
                            line.Direction = -new float2(relativePosition.x * leg + relativePosition.y * combinedRadius, 
                                                        -relativePosition.x * combinedRadius + relativePosition.y * leg) / distSq;
                        }

                        float dotProduct2 = math.dot(relativeVelocity, line.Direction);
                        u = dotProduct2 * line.Direction - relativeVelocity;
                    }
                }
                else
                {
                    float invTimeStep = 1.0f / timeStep;
                    float2 w = relativeVelocity - invTimeStep * relativePosition;
                    float wLength = math.length(w);
                    float2 unitW = w / wLength;

                    line.Direction = new float2(unitW.y, -unitW.x);
                    u = (combinedRadius * invTimeStep - wLength) * unitW;
                }

                line.Point = velocity + 0.5f * u;
                
                if (lineCount < MAX_LINES)
                {
                    orcaLines[lineCount++] = line;
                }
            }

            float2 newVel = float2.zero;
            int lineFail = linearProgram2(orcaLines, lineCount, param.MaxSpeed, prefVelocity, false, ref newVel);

            if (lineFail < lineCount)
            {
                linearProgram3(orcaLines, lineCount, 0, lineFail, param.MaxSpeed, ref newVel);
            }

            newVelocities[agentIdx] = newVel;
        }
    }

    private static void QueryObstacleTreeRecursive(
        float2 position,
        float rangeSq,
        ObstacleTreeNode* treeNodes,
        ObstacleData* treeObstacles,
        int nodeIdx,
        ObstacleDist* results,
        ref int resultCount,
        int maxResults)
    {
        if (nodeIdx == -1) return;

        ObstacleTreeNode node = treeNodes[nodeIdx];
        ObstacleData obstacle1 = treeObstacles[node.ObstacleIndex];

        float2 p1 = obstacle1.Point1;
        float2 p2 = obstacle1.Point2;

        float agentLeftOfLine = det(p2 - p1, position - p1);

        QueryObstacleTreeRecursive(position, rangeSq, treeNodes, treeObstacles,
            agentLeftOfLine >= 0.0f ? node.LeftNodeIndex : node.RightNodeIndex,
            results, ref resultCount, maxResults);

        float distSqLine = (agentLeftOfLine * agentLeftOfLine) / absSq(p2 - p1);

        if (distSqLine < rangeSq)
        {
            if (agentLeftOfLine < 0.0f)
            {
                // Insert maintaining sort
                float distSq = distSqPointLineSegment(p1, p2, position);
                InsertObstacleNeighbor(node.ObstacleIndex, distSq, results, ref resultCount, maxResults);
            }

            QueryObstacleTreeRecursive(position, rangeSq, treeNodes, treeObstacles,
                agentLeftOfLine >= 0.0f ? node.RightNodeIndex : node.LeftNodeIndex,
                results, ref resultCount, maxResults);
        }
    }

    private static void InsertObstacleNeighbor(int index, float distSq, ObstacleDist* results, ref int count, int maxResults)
    {
        if (count < maxResults)
        {
            results[count] = new ObstacleDist { Index = index, DistSq = distSq };
            count++;
        }
        else
        {
            // If full, check if new one is closer than furthest?
            // Since we want CLOSEST neighbors, and we might discard far ones.
            // obstacleDists array should keep closest.
            // But usually we just take first N.
            // If we have sort logic, we should replace the worst one if better.
            if (distSq >= results[count - 1].DistSq) return;
            
            results[count - 1] = new ObstacleDist { Index = index, DistSq = distSq };
        }

        // Bubbling up to keep sorted
        int i = count - 1;
        while (i > 0 && results[i - 1].DistSq > results[i].DistSq)
        {
            ObstacleDist temp = results[i];
            results[i] = results[i - 1];
            results[i - 1] = temp;
            i--;
        }
    }
}
