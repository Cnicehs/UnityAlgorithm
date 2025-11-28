using System;
using System.Collections.Generic;
using UnityEngine;
using Unity.Burst;
using Unity.Mathematics;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Burst.CompilerServices;

[BurstCompile]
public static unsafe class SIMDAlgorithms
{
    // -----------------------------------------------------------------------------------
    // Find Nearest
    // -----------------------------------------------------------------------------------

    public static int FindNearestLegacy(List<Vector2> points, Vector2 target)
    {
        int bestIndex = -1;
        float minDistSq = float.MaxValue;
        int count = points.Count;

        for (int i = 0; i < count; i++)
        {
            float distSq = (points[i] - target).sqrMagnitude;
            if (distSq < minDistSq)
            {
                minDistSq = distSq;
                bestIndex = i;
            }
        }
        return bestIndex;
    }

    public static int FindNearestBurst(List<Vector2> points, Vector2 target)
    {
        Span<Vector2> span = points.AsSpan();
        float2 t = target;
        fixed (Vector2* ptr = span)
        {
            return FindNearestBurst((float2*)ptr, points.Count, &t);
        }
    }

    [BurstCompile]
    private static int FindNearestBurst(float2* points, int count, float2* targetPtr)
    {
        float2 target = *targetPtr;
        float4 target4 = new float4(target.x, target.y, target.x, target.y);
        
        float4 minDists = new float4(float.MaxValue);
        int4 bestIndices = new int4(-1);
        
        int i = 0;
        int batchCount = count - (count % 4);
        
        for (; i < batchCount; i += 4)
        {
            float4 v0 = *(float4*)(points + i);
            float4 v1 = *(float4*)(points + i + 2);

            float4 d0 = v0 - target4;
            float4 d1 = v1 - target4;

            float4 sq0 = d0 * d0;
            float4 sq1 = d1 * d1;

            float2 dists0 = sq0.xz + sq0.yw;
            float2 dists1 = sq1.xz + sq1.yw;
            
            float4 currentDists = new float4(dists0, dists1);
            int4 currentIndices = new int4(i, i + 1, i + 2, i + 3);
            
            bool4 cmp = currentDists < minDists;
            minDists = math.select(minDists, currentDists, cmp);
            bestIndices = math.select(bestIndices, currentIndices, cmp);
        }
        
        // Reduce
        float minDistSq = float.MaxValue;
        int bestIndex = -1;
        
        for(int k=0; k<4; k++) {
            if (minDists[k] < minDistSq) {
                minDistSq = minDists[k];
                bestIndex = bestIndices[k];
            }
        }

        // Tail
        for (; i < count; i++)
        {
            float distSq = math.distancesq(points[i], target);
            if (distSq < minDistSq)
            {
                minDistSq = distSq;
                bestIndex = i;
            }
        }
        return bestIndex;
    }

    // -----------------------------------------------------------------------------------
    // Find Nearest Serial (Strict SIMD with Loop Carried Dependence)
    // -----------------------------------------------------------------------------------

    public static int FindNearestBurstSerial(List<Vector2> points, Vector2 target)
    {
        Span<Vector2> span = points.AsSpan();
        float2 t = target;
        fixed (Vector2* ptr = span)
        {
            return FindClosestIndex_SerialSIMD_Strict((float2*)ptr, points.Count, &t);
        }
    }

    [BurstCompile]
    private static unsafe int FindClosestIndex_SerialSIMD_Strict(float2* pointsPtr, int count, float2* targetPtr)
    {
        if (count == 0) return -1;
        float2 pointB = *targetPtr;

        // --- 初始化 SIMD 常量 ---
        var bX = new float4(pointB.x);
        var bY = new float4(pointB.y);
        // 0, 1, 2, 3 的增量
        var increment4 = new float4(0.0f, 1.0f, 2.0f, 3.0f);

        // --- 单个 SIMD 最小值追踪器：强串行依赖的根源 ---
        var mD0 = new float4(float.MaxValue); // 追踪历史最佳距离 (4个)
        var mI0 = float4.zero;                // 追踪历史最佳索引 (4个)

        int i = 0;

        // 主 SIMD 循环：每次迭代处理 4 个点
        for (; i <= count - 4; i += 4)
        {
            // ----------------------------------------------------
            // BLOCK 1: i 到 i+3 (4 points)
            // 保持与 Ultimate 版本相同的 AoS-to-SoA 加载
            var rA0 = *(float4*)(pointsPtr + i);     // Load P0, P1 -> [x0, y0, x1, y1]
            var rB0 = *(float4*)(pointsPtr + i + 2); // Load P2, P3 -> [x2, y2, x3, y3]

            var pX0 = new float4(rA0.x, rA0.z, rB0.x, rB0.z);
            var pY0 = new float4(rA0.y, rA0.w, rB0.y, rB0.w);

            // 距离计算：SIMD 高效
            var d0 = (pX0 - bX) * (pX0 - bX) + (pY0 - bY) * (pY0 - bY);
            var idx0 = new float4(i) + increment4;

            // --- 归约操作 (循环携带依赖) ---
            // 1. 比较当前计算的 d0 与历史最佳 mD0
            var mask0 = d0 < mD0;

            // 2. 更新 mD0 和 mI0：
            // 下一次迭代的 LOAD/COMPARE 指令必须等到这次迭代的 mD0/mI0 写入完成。
            mD0 = math.select(mD0, d0, mask0);
            mI0 = math.select(mI0, idx0, mask0);
        }

        // --------------------------------------------------------
        // --- 最终 SIMD 归约 (Final Vectorized Reduction) ---
        // --------------------------------------------------------
        // 只剩下 mD0 和 mI0 需要归约。

        // 阶段 1：垂直合并 (Vertical Merge) - 跳过 (因为没有 mD1/mD2/mD3)
        var finalD_Merged = mD0;
        var finalI_Merged = mI0;

        // 阶段 2：水平归约 (Horizontal Reduction)
        // 目标：从 finalD_Merged (4个元素) 中找出全局最小值和对应的索引

        // 找到最小值的索引
        int finalMinIndex;

        // 沿用 UltimateSIMD 的水平归约逻辑：
        // 比较 (x, y) vs (z, w)
        var d_Half = finalD_Merged.xy;
        var i_Half = finalI_Merged.xy;
        if (finalD_Merged.z < d_Half.x)
        {
            d_Half.x = finalD_Merged.z;
            i_Half.x = finalI_Merged.z;
        }
        if (finalD_Merged.w < d_Half.y)
        {
            d_Half.y = finalD_Merged.w;
            i_Half.y = finalI_Merged.w;
        }

        // 比较 x vs y
        if (d_Half.y < d_Half.x)
        {
            // 最终结果在 d_Half.y (以及 i_Half.y) 中
            finalMinIndex = (int)i_Half.y;
        }
        else
        {
            // 最终结果在 d_Half.x (以及 i_Half.x) 中
            finalMinIndex = (int)i_Half.x;
        }

        // --- 处理剩余的点 (Loop Remainder) ---
        // 获取归约后的最小距离
        float currentMinDist = d_Half.x < d_Half.y ? d_Half.x : d_Half.y;

        for (; i < count; i++)
        {
            // 使用 math.distancesq 简化标量计算
            float sqrDist = math.distancesq(pointsPtr[i], pointB);
            if (sqrDist < currentMinDist)
            {
                currentMinDist = sqrDist;
                finalMinIndex = i;
            }
        }
        return finalMinIndex;
    }

    // -----------------------------------------------------------------------------------
    // Find In Radius
    // -----------------------------------------------------------------------------------

    public static void FindInRadiusLegacy(List<Vector2> points, Vector2 target, float radius, List<int> results)
    {
        results.Clear();
        float radiusSq = radius * radius;
        int count = points.Count;

        for (int i = 0; i < count; i++)
        {
            if ((points[i] - target).sqrMagnitude <= radiusSq)
            {
                results.Add(i);
            }
        }
    }

    public static void FindInRadiusBurst(List<Vector2> points, Vector2 target, float radius, List<int> results)
    {
        results.Clear();
        int count = points.Count;
        
        // Allocate temp buffer for results
        // Worst case: all points are in radius
        int* resultBuffer = (int*)UnsafeUtility.Malloc(count * sizeof(int), 4, Allocator.Temp);
        int resultCount = 0;
        float2 t = target;

        try
        {
            Span<Vector2> span = points.AsSpan();
            fixed (Vector2* ptr = span)
            {
                FindInRadiusBurst((float2*)ptr, count, &t, radius * radius, resultBuffer, &resultCount, count);
            }

            // Copy back
            // Optimization: If we could access List<int> internal array, we could memcpy.
            // But List<int> might not have enough capacity.
            if (results.Capacity < resultCount) results.Capacity = resultCount;
            
            // We can use ListUtils to get Span<int> of results if we resize it first?
            // No, we can't easily resize List without adding.
            // So standard loop add is fine, or we can use AddRange if we had a collection.
            // Or we can use UnsafeUtility.MemCpy if we resize list via reflection or just loop.
            // Loop is safest.
            for (int i = 0; i < resultCount; i++)
            {
                results.Add(resultBuffer[i]);
            }
        }
        finally
        {
            UnsafeUtility.Free(resultBuffer, Allocator.Temp);
        }
    }

    [BurstCompile]
    private static void FindInRadiusBurst(float2* points, int count, float2* targetPtr, float radiusSq, int* results, int* resultCount, int maxResults)
    {
        float2 target = *targetPtr;
        int found = 0;
        int i = 0;

        // SIMD Batching
        int batchCount = count - (count % 4);
        if (batchCount > 0)
        {
            float4 target4 = new float4(target.x, target.y, target.x, target.y);
            
            for (; i < batchCount; i += 4)
            {
                float4 v0 = *(float4*)(points + i);
                float4 v1 = *(float4*)(points + i + 2);

                float4 d0 = v0 - target4;
                float4 d1 = v1 - target4;

                float4 sq0 = d0 * d0;
                float4 sq1 = d1 * d1;

                float2 dists0 = sq0.xz + sq0.yw;
                float2 dists1 = sq1.xz + sq1.yw;

                if (dists0.x <= radiusSq && found < maxResults) results[found++] = i;
                if (dists0.y <= radiusSq && found < maxResults) results[found++] = i + 1;
                if (dists1.x <= radiusSq && found < maxResults) results[found++] = i + 2;
                if (dists1.y <= radiusSq && found < maxResults) results[found++] = i + 3;
            }
        }

        for (; i < count; i++)
        {
            if (math.distancesq(points[i], target) <= radiusSq)
            {
                if (found < maxResults)
                {
                    results[found++] = i;
                }
            }
        }
        *resultCount = found;
    }

    // -----------------------------------------------------------------------------------
    // Calculate Bounds (AABB)
    // -----------------------------------------------------------------------------------

    public static Rect CalculateBoundsLegacy(List<Vector2> points)
    {
        if (points.Count == 0) return new Rect();

        float minX = float.MaxValue, minY = float.MaxValue;
        float maxX = float.MinValue, maxY = float.MinValue;

        foreach (var p in points)
        {
            if (p.x < minX) minX = p.x;
            if (p.x > maxX) maxX = p.x;
            if (p.y < minY) minY = p.y;
            if (p.y > maxY) maxY = p.y;
        }

        return new Rect(minX, minY, maxX - minX, maxY - minY);
    }

    public static Rect CalculateBoundsBurst(List<Vector2> points)
    {
        if (points.Count == 0) return new Rect();

        float4 result;
        Span<Vector2> span = points.AsSpan();
        fixed (Vector2* ptr = span)
        {
            CalculateBoundsBurst((float2*)ptr, points.Count, &result);
        }

        return new Rect(result.x, result.y, result.z - result.x, result.w - result.y);
    }

    [BurstCompile]
    private static void CalculateBoundsBurst(float2* points, int count, float4* resultPtr)
    {
        float4 minX_vec = new float4(float.MaxValue);
        float4 minY_vec = new float4(float.MaxValue);
        float4 maxX_vec = new float4(float.MinValue);
        float4 maxY_vec = new float4(float.MinValue);
        
        int i = 0;
        int batchCount = count - (count % 4);
        
        for (; i < batchCount; i += 4)
        {
            float4 v0 = *(float4*)(points + i);
            float4 v1 = *(float4*)(points + i + 2);
            
            float4 xs = new float4(v0.x, v0.z, v1.x, v1.z);
            float4 ys = new float4(v0.y, v0.w, v1.y, v1.w);
            
            minX_vec = math.min(minX_vec, xs);
            minY_vec = math.min(minY_vec, ys);
            maxX_vec = math.max(maxX_vec, xs);
            maxY_vec = math.max(maxY_vec, ys);
        }
        
        float minX = math.cmin(minX_vec);
        float minY = math.cmin(minY_vec);
        float maxX = math.cmax(maxX_vec);
        float maxY = math.cmax(maxY_vec);

        for (; i < count; i++)
        {
            float2 p = points[i];
            minX = math.min(minX, p.x);
            minY = math.min(minY, p.y);
            maxX = math.max(maxX, p.x);
            maxY = math.max(maxY, p.y);
        }

        *resultPtr = new float4(minX, minY, maxX, maxY);
    }

    // -----------------------------------------------------------------------------------
    // Aggregations
    // -----------------------------------------------------------------------------------

    public static Vector2 SumLegacy(List<Vector2> points)
    {
        Vector2 sum = Vector2.zero;
        foreach (var p in points) sum += p;
        return sum;
    }

    public static Vector2 SumBurst(List<Vector2> points)
    {
        Span<Vector2> span = points.AsSpan();
        float2 result;
        fixed (Vector2* ptr = span)
        {
            SumBurst((float2*)ptr, points.Count, &result);
        }
        return result;
    }

    [BurstCompile]
    private static void SumBurst(float2* points, int count, float2* resultPtr)
    {
        float2 sum = float2.zero;
        int i = 0;
        int batchCount = count - (count % 4);
        
        if (batchCount > 0)
        {
            float4 sum4 = float4.zero;
            for (; i < batchCount; i += 4)
            {
                float4 v0 = *(float4*)(points + i);
                float4 v1 = *(float4*)(points + i + 2);
                sum4 += v0 + v1;
            }
            // sum4 is (x0+x2+..., y0+y2+..., x1+x3+..., y1+y3+...)
            // we want (sumX, sumY)
            // sumX = sum4.x + sum4.z
            // sumY = sum4.y + sum4.w
            sum.x = sum4.x + sum4.z;
            sum.y = sum4.y + sum4.w;
        }

        for (; i < count; i++)
        {
            sum += points[i];
        }
        *resultPtr = sum;
    }

    public static Vector2 AverageLegacy(List<Vector2> points)
    {
        if (points.Count == 0) return Vector2.zero;
        return SumLegacy(points) / points.Count;
    }

    public static Vector2 AverageBurst(List<Vector2> points)
    {
        if (points.Count == 0) return Vector2.zero;
        return SumBurst(points) / points.Count;
    }

    // -----------------------------------------------------------------------------------
    // Transformations (In-Place)
    // -----------------------------------------------------------------------------------

    public static void TranslateLegacy(List<Vector2> points, Vector2 translation)
    {
        for (int i = 0; i < points.Count; i++)
        {
            points[i] += translation;
        }
    }

    public static void TranslateBurst(List<Vector2> points, Vector2 translation)
    {
        Span<Vector2> span = points.AsSpan();
        float2 t = translation;
        fixed (Vector2* ptr = span)
        {
            TranslateBurst((float2*)ptr, points.Count, &t);
        }
    }

    [BurstCompile]
    private static void TranslateBurst(float2* points, int count, float2* translationPtr)
    {
        float2 translation = *translationPtr;
        float4 t4 = new float4(translation.x, translation.y, translation.x, translation.y);
        
        int i = 0;
        int batchCount = count - (count % 4);
        
        for (; i < batchCount; i += 4)
        {
            float4 v0 = *(float4*)(points + i);
            float4 v1 = *(float4*)(points + i + 2);
            
            v0 += t4;
            v1 += t4;
            
            *(float4*)(points + i) = v0;
            *(float4*)(points + i + 2) = v1;
        }

        for (; i < count; i++)
        {
            points[i] += translation;
        }
    }

    public static void ScaleLegacy(List<Vector2> points, float scale)
    {
        for (int i = 0; i < points.Count; i++)
        {
            points[i] *= scale;
        }
    }

    public static void ScaleBurst(List<Vector2> points, float scale)
    {
        Span<Vector2> span = points.AsSpan();
        fixed (Vector2* ptr = span)
        {
            ScaleBurst((float2*)ptr, points.Count, scale);
        }
    }

    [BurstCompile]
    private static void ScaleBurst(float2* points, int count, float scale)
    {
        int i = 0;
        int batchCount = count - (count % 4);
        
        for (; i < batchCount; i += 4)
        {
            float4 v0 = *(float4*)(points + i);
            float4 v1 = *(float4*)(points + i + 2);
            
            v0 *= scale;
            v1 *= scale;
            
            *(float4*)(points + i) = v0;
            *(float4*)(points + i + 2) = v1;
        }

        for (; i < count; i++)
        {
            points[i] *= scale;
        }
    }

    // -----------------------------------------------------------------------------------
    // Batch Operations
    // -----------------------------------------------------------------------------------

    public static void BatchDotLegacy(List<Vector2> points, Vector2 target, List<float> results)
    {
        results.Clear();
        for (int i = 0; i < points.Count; i++)
        {
            results.Add(Vector2.Dot(points[i], target));
        }
    }

    public static void BatchDotBurst(List<Vector2> points, Vector2 target, List<float> results)
    {
        results.Clear();
        int count = points.Count;
        
        float* resultBuffer = (float*)UnsafeUtility.Malloc(count * sizeof(float), 4, Allocator.Temp);
        float2 t = target;
        
        try
        {
            Span<Vector2> span = points.AsSpan();
            fixed (Vector2* ptr = span)
            {
                BatchDotBurst((float2*)ptr, count, &t, resultBuffer);
            }

            for (int i = 0; i < count; i++)
            {
                results.Add(resultBuffer[i]);
            }
        }
        finally
        {
            UnsafeUtility.Free(resultBuffer, Allocator.Temp);
        }
    }

    [BurstCompile]
    private static void BatchDotBurst(float2* points, int count, float2* targetPtr, float* results)
    {
        float2 target = *targetPtr;
        float4 target4 = new float4(target.x, target.y, target.x, target.y);
        
        int i = 0;
        int batchCount = count - (count % 4);
        
        for (; i < batchCount; i += 4)
        {
            float4 v0 = *(float4*)(points + i);
            float4 v1 = *(float4*)(points + i + 2);
            
            // Dot: x1*tx + y1*ty
            // v0 * target4 = (x1*tx, y1*ty, x2*tx, y2*ty)
            float4 m0 = v0 * target4;
            float4 m1 = v1 * target4;
            
            // Horizontal add
            float2 d0 = m0.xz + m0.yw; // (dot1, dot2)
            float2 d1 = m1.xz + m1.yw; // (dot3, dot4)
            
            results[i] = d0.x;
            results[i+1] = d0.y;
            results[i+2] = d1.x;
            results[i+3] = d1.y;
        }

        for (; i < count; i++)
        {
            results[i] = math.dot(points[i], target);
        }
    }
}
