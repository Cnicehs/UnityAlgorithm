# RVO2 算法详解

## 1. 概述

RVO2 (Reciprocal Velocity Obstacles) 是一种用于多智能体导航的实时避障算法。它能让每个智能体独立计算自己的无碰撞速度，同时考虑到其他智能体和静态障碍物。此算法的核心思想是，每个智能体通过构建速度障碍物（Velocity Obstacles）来预测未来可能发生的碰撞，并选择一个最接近自己期望速度的、且能避免碰撞的速度。

本项目的实现主要围绕以下几个核心文件，它们共同构成了RVO2算法的基础：

- **`RVOAgent.cs`**: 定义了模拟中的动态智能体。每个智能体都包含位置 (`Position`)、当前速度 (`Velocity`)、期望速度 (`PrefVelocity`)、半径 (`Radius`)、最大速度 (`MaxSpeed`) 等物理属性，以及用于避障计算的参数，如邻居距离 (`NeighborDist`) 和时间视界 (`TimeHorizon`)。

- **`RVOObstacle.cs`**: 定义了单个静态障碍物。
    -   障碍物被定义为一条有向线段，由 `Point1` 和 `Point2` 构成。
    -   `Direction`: 从 `Point1` 指向 `Point2` 的单位向量，表示障碍物的朝向。
    -   `Normal`: `Direction` 的法向量，指向障碍物的“外部”（即智能体不允许进入的一侧）。在RVO2中，规定障碍物顶点需要以**逆时针**顺序定义，这样计算出的法线才能正确地指向多边形障碍物的外部。
    -   `NextObstacle` 和 `PrevObstacle`: 这两个字段用于将多个障碍物线段链接成一个连续的多边形。`RVOSimulator.ProcessObstacles()` 方法负责处理这些链接。
    -   `IsConvex`: 标记该障碍物与其相邻障碍物形成的角是否为凸角。这个属性对于正确处理凹多边形角落的避障至关重要。

- **`RVOSimulator.cs`**: 是整个模拟的驱动核心。它管理着所有的 `RVOAgent` 和 `RVOObstacle`，并在每个时间步 (`Step`) 中执行完整的RVO2计算流程，包括邻居查询、ORCA约束计算、速度求解和位置更新。

- **`RVOMath.cs`**: 包含了RVO2算法所需的所有核心数学函数。这包括ORCA线的构建 (`ConstructORCALines`, `ConstructObstacleORCALines`) 和求解最优速度的线性规划算法 (`linearProgram1`, `linearProgram2`, `linearProgram3`)。

## 2. 数据流与核心算法

RVO2 的模拟过程在 `RVOSimulator.cs` 的 `Step` 方法中实现，可以分为以下几个阶段：

### 阶段一：邻居查询

- **目的**: 为每个智能体找到其附近的其他智能体。
- **流程**:
    1. 遍历所有智能体。
    2. **范围查询**: 对每个智能体，使用空间索引 (`SpatialIndexManager`) 查询其 `NeighborDist` 范围内的所有其他智能体。
    3. **距离排序**: 计算出范围内每个邻居与当前智能体的精确距离（通常使用距离的平方以避免开销大的开方运算），并根据距离对所有邻居进行升序排序。
    4. **选择Top-N**: 从排序后的邻居列表中，选取前 `MaxNeighbors` 个作为最终的邻居列表。
  - **重要性**: 这个过程确保了每个智能体总是优先考虑离它最近、碰撞风险最高的其他智能体。在智能体密集的区域，一个智能体的邻域内可能有远超 `MaxNeighbors` 数量的其他智能体，通过距离排序并选择最近的N个，可以极大地提高避障的准确性和稳定性，避免因忽略了近处的威胁而导致碰撞。
- **数据**:
    - **输入**: 所有智能体的位置。
    - **输出**: 每个智能体的邻居列表。

### 阶段二：ORCA 约束计算

这是 RVO2 算法的核心。每个智能体为自己计算一组称为 ORCA (Optimal Reciprocal Collision Avoidance) 的约束。这些约束定义了一个“安全速度”的集合，智能体可以从中选择自己的新速度。

- **目的**: 为每个智能体计算与其他智能体和障碍物的速度约束。
- **流程**:
    1. **清空旧约束**: 为每个智能体清空上一帧的 ORCA 线列表 (`_orcaLines.Clear()`)。
    2. **计算障碍物约束**:
        - 调用 `RVOMath.ConstructObstacleORCALines`。
        - 遍历所有静态障碍物，为每个可能与之碰撞的障碍物生成一条 ORCA 线。
        - 这些线是“硬约束”，意味着智能体绝对不能穿越。
    3. **计算智能体约束**:
        - 调用 `RVOMath.ConstructORCALines`。
        - 遍历该智能体的所有邻居。
        - 为每个邻居生成一条 ORCA 线。这些线代表了为避免与该邻居碰撞，当前智能体需要调整其速度的方向和大小。
- **数据**:
    - **输入**:
        - 当前智能体的信息（位置、速度、半径）。
        - 邻居智能体列表。
        - 所有静态障碍物。
    - **输出**: 一系列 `ORCALine`，每条线都定义了一个半平面，表示“允许”的速度区域。

### 阶段三：速度求解

- **目的**: 在满足所有 ORCA 约束的前提下，为智能体选择一个最优的新速度，该速度应尽可能接近其期望速度 (`PrefVelocity`)。
- **流程**:
    1. **线性规划**: 使用 `RVOMath.linearProgram2` 和 `RVOMath.linearProgram3` 求解。
    2. `linearProgram2` 尝试找到一个满足所有约束的速度。如果期望速度本身就在安全区域内，那么它就是最优解。如果不在，`linearProgram2` 会找到边界上最接近期望速度的点。
    3. 如果 `linearProgram2` 失败（意味着期望速度严重违反了某个约束），`linearProgram3` 会介入，处理更复杂的约束冲突情况，特别是当障碍物约束和智能体约束同时存在时。
- **数据**:
    - **输入**:
        - 上一阶段计算出的 `ORCALine` 列表。
        - 智能体的最大速度 `MaxSpeed`。
        - 智能体的期望速度 `PrefVelocity`。
    - **输出**: 智能体的新速度 `NewVelocity`。

### 阶段四：位置更新

- **目的**: 根据新计算出的速度更新智能体的位置。
- **流程**:
    1. 将 `NewVelocity` 赋给 `Velocity`。
    2. `Position += Velocity * dt`。
    3. 更新空间索引中的智能体位置，为下一帧的邻居查询做准备。
- **数据**:
    - **输入**: `NewVelocity`。
    - **输出**: 更新后的 `Position`。

## 3. Bug 分析：障碍物约束被清除

在之前的版本中，存在一个严重的数据流问题，导致障碍物躲避逻辑失效。

- **问题描述**:
  在 `RVOSimulator.cs` 的 `Step` 方法中，每个智能体的 ORCA 约束计算流程如下：
  1. `_orcaLines.Clear()`
  2. `RVOMath.ConstructObstacleORCALines()`
  3. `RVOMath.ConstructORCALines()` (for agents)

  然而，在 `RVOMath.ConstructORCALines` 函数的一开始，也存在一个 `orcaLines.Clear()` 的调用。

- **根本原因**:
  数据流的设计是，先计算障碍物约束，将它们添加到 `_orcaLines` 列表中，然后再计算智能体约束，继续向该列表添加。
  然而，由于 `RVOMath.ConstructORCALines` 中多余的 `Clear` 调用，导致前面计算好的障碍物约束被全部清空了。因此，最终进行速度求解时，只有智能体之间的约束，没有任何障碍物的约束，智能体因此会直接穿过障碍物。

- **修复方案**:
  移除 `RVOMath.ConstructORCALines` 函数开头的 `orcaLines.Clear()` 语句。
  正确的逻辑应该是，`_orcaLines` 列表在每个智能体开始计算其所有约束之前（在 `RVOSimulator.cs` 中）被清空一次，然后依次填充障碍物和智能体的约束。

## 4. 时间线与数据流总结

下面是一个单帧更新中，针对**一个智能体**的数据流时间线：

1. **`RVOSimulator.Step()` 开始**:
   - `_orcaLines` 被清空。
2. **计算障碍物约束**:
   - `ConstructObstacleORCALines()` 被调用。
   - `_orcaLines` 列表中被填充了 0 条或多条来自静态障碍物的约束。
3. **计算智能体约束**:
   - `ConstructORCALines()` 被调用。
   - **(Bug 发生点)** 如果这里有 `Clear`，`_orcaLines` 会被清空，所有障碍物约束丢失。
   - `_orcaLines` 列表中继续被填充来自邻居智能体的约束。
4. **求解速度**:
   - `linearProgram2/3` 被调用。
   - **(受 Bug 影响)** 求解器只看到了智能体约束，没有看到障碍物约束。
   - 计算出 `NewVelocity`。
5. **更新位置**:
   - 智能体位置根据 `NewVelocity` 更新。

通过这个时间线，可以清晰地看到，数据（`_orcaLines`）的传递过程中任何一个环节出现问题，都会导致最终结果的错误。理解并确保数据在处理流程中的完整性和正确性，是算法正常工作的关键。
