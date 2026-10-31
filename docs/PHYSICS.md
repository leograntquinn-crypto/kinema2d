# 物理与算法原理

本文按 **一次 `World::Step(dt)` 的执行顺序** 讲解引擎中的每个算法，并标注对应的源码位置。
建议一边读一边在客户端里打开对应的调试选项观察。

---

## 0. 坐标系与单位

- 单位：**米、千克、秒**（MKS）。物体尺寸在 0.1 ~ 10 米之间时数值最稳定。
- 世界坐标 y 轴向上，角度逆时针为正（弧度）。
- 约定：**Body 的位置就是质心**。`PolygonShape::Set()` 会把顶点平移到以质心为原点的坐标系，
  这让后面所有的动力学公式都不需要处理"质心偏移"。

## 1. 数学基础：2D 叉积

> 📄 `engine/include/kinema/math/Math.h`

3D 中角速度是向量，在 2D 中它总是垂直于平面，因此用一个标量 ω 表示。2D "叉积"因此有三种形式：

| 形式 | 结果 | 物理含义 |
| --- | --- | --- |
| $a \times b = a_x b_y - a_y b_x$ | 标量 | 力矩 $\tau = r \times F$ |
| $\omega \times r = (-\omega r_y,\ \omega r_x)$ | 向量 | 旋转引起的点速度 |
| $r \times s = (s r_y,\ -s r_x)$ | 向量 | 用于构造法线 / 切线 |

刚体上一点 $p$ 的速度：

$$v_p = v + \omega \times (p - c)$$

这一行公式在接触求解、关节求解中会反复出现（代码里的 `B.v + Cross(B.w, rB)`）。

## 2. 质量属性

> 📄 `engine/src/collision/Shape.cpp` — `ComputeMass()`

- 圆：$m = \rho \pi r^2$，$I = \frac{1}{2} m r^2$
- 凸多边形：分解为三角扇，每个三角形 $(0, e_1, e_2)$ 的面积 $A = \frac{1}{2} e_1 \times e_2$，
  二阶矩积分 $\int x^2 dA = \frac{D}{12}(e_{1x}^2 + e_{1x} e_{2x} + e_{2x}^2)$。
  最后用 **平行轴定理** $I_c = I_o - m |c|^2$ 换算到质心。

静态 / 运动学物体的 $m^{-1} = I^{-1} = 0$——在所有公式里它们自然表现为"质量无穷大"，不需要特殊分支。

## 3. 宽阶段：排序扫掠

> 📄 `engine/src/collision/BroadPhase.cpp`

N 个物体两两精确检测需要 $O(N^2)$ 次。宽阶段先用 AABB 快速筛掉明显不相交的对：

1. 计算物体中心在 x、y 上的方差，选择分布更分散的轴（自适应选轴）；
2. 按 AABB 在该轴上的最小值排序；
3. 从左到右扫描：对物体 i，只需检查后面 `min(j) <= max(i)` 的物体，一旦超出立刻 `break`。

平均复杂度 $O(N \log N + K)$（K 为输出对数）。在漏斗场景中切换到 Brute Force，可以在性能面板看到宽阶段耗时上升约 3 倍。

## 4. 窄阶段：接触流形

> 📄 `engine/src/collision/Collision.cpp`

输出 **Manifold**：一个公共法线 $n$（从 A 指向 B）+ 最多 2 个接触点，每个点带穿透深度。

### 4.1 多边形 vs 多边形：SAT + 裁剪

**分离轴定理**：两个凸多边形不相交 ⟺ 存在一条分离轴。2D 中候选轴只需检查两个多边形所有边的法线。

```
对 A 的每条边法线 n_i：
    s_i = min_j dot(n_i, B 的顶点 j - A 的顶点 i)   // B 在该方向上最深的点
取 s_i 最大的边 → 若 > 0，找到分离轴，提前退出
```

对 B 做同样计算，取分离量更大的那一边作为 **参考面（reference face）**，
另一个多边形上与参考面法线最反向的边作为 **入射边（incident edge）**。

然后用参考面的两条侧边对入射边做 **Sutherland–Hodgman 裁剪**，保留位于参考面后方的点，
就得到 1~2 个接触点。

> 为什么要偏向 A（`kTol`）？两个分离量几乎相等时，如果每帧在 A、B 之间来回切换参考面，接触点会跳变，导致堆叠抖动。

### 4.2 接触特征 ID

每个接触点记录它来自"哪条边 × 哪个顶点"（`ContactFeature`）。下一帧通过 ID 匹配同一个接触点，
继承它的累积冲量——这就是 **热启动** 的基础。

## 5. 积分：半隐式欧拉

> 📄 `engine/src/dynamics/World.cpp` — `Solve()`

$$v_{n+1} = v_n + \Delta t \cdot (g + F/m)$$
$$x_{n+1} = x_n + \Delta t \cdot v_{n+1}$$

先更新速度，再用 **新** 速度更新位置（Symplectic Euler）。它和显式欧拉一样便宜，但能量长期近似守恒，
是游戏物理的标准选择。阻尼使用隐式形式 $v' = v / (1 + c \Delta t)$，任何阻尼值都不会发散。

单元测试 `World.FreeFallMatchesKinematics` 精确验证了这个积分公式。

## 6. 约束求解：顺序冲量法

> 📄 `engine/src/dynamics/Contact.cpp`

### 6.1 思路

不去求"力"，而是直接求 **冲量** $P$（力 × 时间），使速度满足约束：

- 法向：相对速度 $v_n \ge 0$（不再相互靠近），且冲量 $\lambda_n \ge 0$（只能推不能拉）
- 切向：$|\lambda_t| \le \mu \lambda_n$（库仑摩擦锥）

对每个接触点施加冲量后，B 的速度变化 $\Delta v_B = P / m_B$，角速度变化 $\Delta\omega_B = (r_B \times P) / I_B$。
让 $v_n$ 变成目标值所需的冲量大小：

$$\lambda = -\frac{v_n - b}{K},\qquad K = m_A^{-1} + m_B^{-1} + I_A^{-1}(r_A \times n)^2 + I_B^{-1}(r_B \times n)^2$$

$1/K$ 称为 **有效质量**（effective mass）。

### 6.2 累积冲量钳制（关键！）

逐个约束求解会互相干扰，所以要迭代多次（`velocityIterations`）。
钳制的必须是 **本帧累积的总冲量**，而不是每次的增量：

```cpp
float newImpulse = std::max(mp.normalImpulse + lambda, 0.0f);   // 累积值 >= 0
lambda = newImpulse - mp.normalImpulse;                          // 实际施加的增量（可以为负！）
mp.normalImpulse = newImpulse;
```

允许增量为负，迭代才能"纠正"之前施加过多的冲量，从而收敛到正确解。

### 6.3 热启动

把上一帧的累积冲量在本帧一开始直接施加上去。对于静止的堆叠，上一帧的答案几乎就是这一帧的答案，
迭代只需微调。**试验**：金字塔场景中关闭 *Warm starting*，堆叠会明显下沉甚至倒塌。

### 6.4 块求解器（Block Solver）

箱子平放时有两个接触点。逐点求解时，修正点 1 会破坏点 2，反之亦然，高塔会左右摇摆。
块求解器把两个法向约束 **联立** 为 2×2 的线性互补问题（LCP）：

$$v_n = K x + b,\quad x \ge 0,\quad v_n \ge 0,\quad x_i \cdot v_{n,i} = 0$$

二维 LCP 只有 4 种情况（两点受力 / 仅点 1 / 仅点 2 / 都不受力），逐一尝试即可得到精确解。
单元测试 `World.BoxStackIsStableAndSleeps` 中 10 层的竖直塔就是靠它保持完全竖直的。

### 6.5 位置修正：Baumgarte

数值误差会让物体逐渐穿透。在速度目标中加入一个与穿透深度成正比的"偏置速度"：

$$b = \frac{\beta}{\Delta t} \max(0,\ -\text{separation} - \text{slop})$$

- $\beta$（Baumgarte 系数，默认 0.2）：每步修正 20% 的穿透
- slop（默认 5 mm）：允许少量穿透，避免接触在"接触 / 不接触"之间闪烁

### 6.6 弹性

若碰撞前法向相对速度 $v_n < -v_{threshold}$，目标速度设为 $-e \cdot v_n$（$e$ 为恢复系数）。
阈值的作用：低速时不反弹，否则静止物体永远在微小地弹跳。

## 7. 关节

> 📄 `engine/src/joints/Joint.cpp`

所有关节都套用和接触相同的框架：`PrepareAndWarmStart()` 计算有效质量，`SolveVelocity()` 迭代施加冲量。

| 关节 | 约束 $C$ | 自由度 |
| --- | --- | --- |
| 距离关节 | $\lvert p_B - p_A \rvert - L$ | 1 |
| 旋转关节 | $p_B - p_A$（二维向量） | 2，有效质量为 2×2 矩阵 |
| 鼠标关节 | $p_B - target$ | 2，软约束 |

### 软约束（弹簧）

把刚性约束变"软"只需两个参数：频率 $f$（Hz）和阻尼比 $\zeta$。

$$\omega = 2\pi f,\quad k = m\omega^2,\quad c = 2m\zeta\omega$$
$$\gamma = \frac{1}{\Delta t(c + \Delta t k)},\qquad \beta = \Delta t k \gamma$$

冲量公式变为 $\lambda = -\frac{\dot C + \beta C + \gamma \lambda_{acc}}{K + \gamma}$。
好处是：参数与时间步长、质量无关，调出来的手感在任何帧率下都一致。软体场景就是用这个实现的。

### 马达

旋转关节的马达是一个"速度约束"：$\omega_B - \omega_A = \omega_{target}$，冲量被限制在 $\pm \tau_{max} \Delta t$ 之内。

## 8. 休眠与岛屿

> 📄 `World::UpdateSleep()`

1. 每个动态物体若线速度、角速度都低于阈值，累加"安静时间"，否则清零。
2. 用 **并查集** 把通过接触或关节相连的非静态物体合并成岛屿（静态物体不连接岛屿，否则整个世界会成为一个岛）。
3. 岛屿中 **最小** 的安静时间超过 `timeToSleep` 时，整个岛屿一起入睡。

为什么不能单个物体各自睡？想象一个堆叠：底部的箱子睡着了，上面的箱子还在动——底部箱子不再响应，
上面的箱子就会穿过去。被唤醒的物体在窄阶段碰到睡着的物体时会把它叫醒。

## 9. 固定时间步

> 📄 `app/src/editor/Simulation.cpp`

渲染帧率会波动，但物理必须用固定的 dt 才能稳定、可复现：

```cpp
accumulator += frameTime;
while (accumulator >= dt) { world.Step(dt); accumulator -= dt; }
```

每帧最多执行 8 步，防止"越慢越要多算、越多算越慢"的 **死亡螺旋**。
子步（Sub-steps）把一个固定步再细分，代价线性增加，但对高堆叠和高速物体的改善非常明显。
