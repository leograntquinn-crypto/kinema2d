# 学习路线与练习

推荐的学习方式：**先玩、再读、后改**。每个阶段都先在客户端里观察现象，再读对应源码，最后动手完成练习。
每完成一个练习，就补一个单元测试（`tests/`）—— 这是专业项目的习惯。

---

## 阶段 1：熟悉项目（1~2 天）

- [ ] 按 README 构建并运行，把 13 个场景都玩一遍
- [ ] 打开 *Debug Draw* 里的每个选项，理解它们画的是什么
- [ ] 运行 `ctest` 和 `kinema_bench`
- [ ] 阅读 [ARCHITECTURE.md](ARCHITECTURE.md)，画出你自己理解的模块关系图

**练习 1.1**：在 `scene/src/SceneLibrary.cpp` 中新增一个场景（例如"保龄球"），注册到 `GetScenes()`。

**练习 1.2**：在 `I18n.cpp` 中为你新增的场景补上中文翻译。

## 阶段 2：数学与几何（2~3 天）

阅读：`math/Math.h`、`collision/Shape.cpp`、`tests/test_math.cpp`、`tests/test_shapes.cpp`

**练习 2.1**：给 `Vec2` 增加 `Rotate(float angle)` 成员函数并写测试。

**练习 2.2**：新增 `CapsuleShape`（胶囊体 = 线段 + 半径）。需要实现：
`ComputeMass`、`ComputeAABB`、`TestPoint`、`RayCast`。（提示：质量 = 矩形 + 两个半圆）

## 阶段 3：碰撞检测（3~5 天）

阅读：[PHYSICS.md](PHYSICS.md) 第 3、4 节，`collision/Collision.cpp`、`collision/BroadPhase.cpp`

**练习 3.1**：暂停后用"单步"观察两个箱子碰撞，打开 *Contact points* 和 *Contact normals*，
对照 `CollidePolygons` 解释每个接触点是怎么来的。

**练习 3.2**：实现 **均匀网格宽阶段** `UniformGridBroadPhase`，加入 `BroadPhaseType` 枚举和设置面板，
用 `kinema_bench` 和三种算法对比。

**练习 3.3（进阶）**：实现 **动态 AABB 树**（Box2D 的 `b2DynamicTree`），支持增量更新而不是每帧重建。

**练习 3.4**：为 `CapsuleShape` 实现与圆、多边形的碰撞函数（提示：转化为"线段到点 / 线段到线段的最近距离"）。

## 阶段 4：动力学与求解器（5~7 天）

阅读：PHYSICS.md 第 5、6 节，`dynamics/World.cpp`、`dynamics/Contact.cpp`

**练习 4.1**：在金字塔场景里依次做以下实验，记录现象并解释原因：
- 速度迭代次数 1 / 4 / 10 / 30
- 关闭热启动
- Baumgarte 0 / 0.2 / 0.8
- 子步数 1 / 4

**练习 4.2**：把求解顺序改成"先法向后摩擦"，观察有什么不同，再改回来。

**练习 4.3**：实现 **NGS 位置求解器**（Box2D v2 的 `SolvePositionConstraints`），替代 Baumgarte，
比较两者在高堆叠上的表现。

**练习 4.4**：实现 **滚动摩擦**（rolling resistance），让圆在平地上最终停下来。

## 阶段 5：关节（3~5 天）

阅读：PHYSICS.md 第 7 节，`joints/Joint.cpp`

**练习 5.1**：给 `RevoluteJoint` 增加 **角度限制**（lower/upper angle），做一个"布娃娃"场景。

**练习 5.2**：实现 **棱柱关节**（PrismaticJoint，只能沿一条轴滑动），做一个"活塞"场景。

**练习 5.3**：实现 **焊接关节**（WeldJoint），用它把多个形状组合成一个复合物体。

**练习 5.4**：给小车加上 **悬挂**（用软距离关节或 WheelJoint）。

## 阶段 6：客户端（3~5 天）

阅读：ARCHITECTURE.md，`app/src/`

**练习 6.1**：实现 **撤销 / 重做**（提示：命令模式；或者每次编辑前用 `SerializeWorld` 存快照）。

**练习 6.2**：实现 **渲染插值**：用累加器剩余时间 `alpha = accumulator / dt` 在上一状态与当前状态之间插值，
让 144 Hz 显示器上的运动更平滑。

**练习 6.3**：在 Inspector 中支持 **编辑多边形顶点**（在视口中拖动顶点）。

**练习 6.4**：增加 **"录制回放"** 功能：每步保存所有物体位姿，用时间轴拖动回看。

## 阶段 7：性能（进阶）

**练习 7.1**：用 `perf`（Linux）或 Visual Studio Profiler 分析漏斗场景，找出热点。

**练习 7.2**：把 `std::unordered_map<uint64_t, Contact>` 换成更缓存友好的结构，测量收益。

**练习 7.3**：岛屿之间互相独立 —— 用多线程并行求解不同岛屿。

**练习 7.4**：实现 **连续碰撞检测（CCD）**：高速小球不应该穿过薄墙（提示：保守前进 / 时间撞击 TOI）。

---

## 推荐阅读

- Erin Catto — GDC 演讲合集：<https://box2d.org/publications/>
- Box2D 源码（v2.4 结构清晰，v3 是面向数据的现代实现）：<https://github.com/erincatto/box2d>
- Christer Ericson — *Real-Time Collision Detection*
- Glenn Fiedler — *Fix Your Timestep!*：<https://gafferongames.com/post/fix_your_timestep/>
- Dear ImGui Wiki：<https://github.com/ocornut/imgui/wiki>
