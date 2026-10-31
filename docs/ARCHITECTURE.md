# 软件架构

## 1. 分层

```
┌──────────────────────────────────────────────────────────────┐
│  app/  Kinema2D 桌面客户端                                     │
│  ┌────────────┐ ┌────────────┐ ┌──────────────┐ ┌──────────┐ │
│  │ core       │ │ editor     │ │ render       │ │ util     │ │
│  │ 窗口/主循环 │ │ 面板/工具   │ │ GL/批处理/相机 │ │ 日志/i18n │ │
│  └────────────┘ └────────────┘ └──────────────┘ └──────────┘ │
│        依赖：GLFW · OpenGL 3.3 · Dear ImGui                     │
└───────────────────────────┬──────────────────────────────────┘
                            │
┌───────────────────────────▼──────────────────────────────────┐
│  scene/  kinema_scene —— 示例场景 + .kscene 序列化              │
└───────────────────────────┬──────────────────────────────────┘
                            │
┌───────────────────────────▼──────────────────────────────────┐
│  engine/  kinema —— 物理引擎核心（只依赖 C++ 标准库）            │
│  math → collision → dynamics → joints                         │
└──────────────────────────────────────────────────────────────┘
     ▲                         ▲
     │ tests/ (GoogleTest)      │ tools/benchmark
```

**规则：依赖只能向下。** 引擎完全不知道窗口、OpenGL、ImGui 的存在，所以：

- 单元测试无需显卡即可运行（CI 服务器上也能跑）；
- 基准程序能精确测量纯物理开销；
- 引擎可以原封不动地拿去做游戏、服务器端模拟等。

## 2. 一帧发生了什么

```
Application::Run()
 └─ while (!quit)
     ├─ glfwPollEvents()                         系统事件 → ImGui IO
     ├─ ImGui::NewFrame()
     ├─ Editor::OnFrame(dt)
     │   ├─ UpdateFrameState(dt)
     │   │   ├─ Simulation::Update(dt)           固定步长累加器 → World::Step() × N
     │   │   ├─ 清理失效的选择 / 关节 ID
     │   │   └─ 相机跟随、提示消息、统计历史
     │   ├─ HandleShortcuts()                    键盘快捷键
     │   ├─ DrawMenuBar() / DrawStatusBar() / DockSpace
     │   └─ 每个 Panel::OnImGui()
     │       └─ ViewportPanel
     │           ├─ HandleInput()                鼠标 → 工具逻辑（可能修改 World）
     │           ├─ RenderScene()                World → Renderer2D → 离屏 MSAA 帧缓冲
     │           └─ AddImage() + 叠加层            帧缓冲纹理显示在面板中
     ├─ ImGui::Render() + RenderDrawData()       绘制整个界面
     └─ glfwSwapBuffers()
```

## 3. 关键设计决策

### 3.1 用 ID 而不是指针引用物体

编辑器保存的是 `BodyId` / `JointId`，每次使用时通过 `World::FindBody(id)` 查找。
物体可能随时被删除（掉出世界、被删除工具点中、加载新场景），
长期持有 `Body*` 会产生悬空指针，而 ID 查找失败只会返回 `nullptr`。

### 3.2 SolverBody：面向数据的求解器

求解器迭代 10 次 × 每个接触点，访问的数据只有速度、质量倒数、位置。
`World::Solve()` 先把它们拷贝进连续数组 `std::vector<SolverBody>`，迭代完再写回 `Body`。
这样内存访问连续，缓存友好。

### 3.3 策略模式：宽阶段

`BroadPhase` 是抽象接口，`BruteForceBroadPhase` 与 `SweepAndPruneBroadPhase` 是两种实现，
通过 `CreateBroadPhase(type)` 工厂创建，运行时可切换。想实现动态 AABB 树或均匀网格？
只需新增一个子类（见 [LEARNING_PATH.md](LEARNING_PATH.md) 的练习）。

### 3.4 形状分派：类型排序

碰撞函数只有 3 个（圆-圆、多边形-圆、多边形-多边形）。创建 `Contact` 时保证
`A.type >= B.type`，就不需要"圆-多边形"这个重复版本。

### 3.5 离屏渲染 + ImGui::Image

场景先渲染到一个 4× MSAA 帧缓冲，解析成纹理后作为图片嵌入 Viewport 面板。
这是商业引擎编辑器的标准做法：视口可以被拖动、停靠、调整大小，与 UI 完全统一。

### 3.6 批量渲染

`Renderer2D` 把所有图元（包括线）展开成三角形，追加到一个 CPU 数组，
最后一次性上传并 `glDrawArrays`。400 个物体的场景通常只需 1~2 次 draw call（见性能面板）。

### 3.7 自写 OpenGL 加载器

`app/src/render/GL.h` 用一个 X-Macro 列出需要的 ~55 个函数，展开成函数指针声明、定义和加载代码。
它展示了 OpenGL 的本质：驱动在运行时提供函数地址，我们通过 `glfwGetProcAddress` 查询。

### 3.8 国际化

`Tr("English text")` 以英文原文为键查表。窗口名统一为 `"<译文>###<稳定ID>"`，
ImGui 只用 `###` 之后的部分计算 ID，所以切换语言不会打乱停靠布局。
中文字体只为"常用 2500 字 + 所有译文中出现的字"生成字形，纹理图集保持在较小尺寸。

### 3.9 文本场景格式

`.kscene` 采用"类型 + key=value"的行格式：可以手写（见 `examples/hello.kscene`）、
可以 diff、读取时忽略未知字段（向前兼容）。解析先写入临时结构，全部成功后才修改 World。

## 4. 代码规范

- C++17，`.clang-format` 统一风格（Google 风格 + 4 空格缩进 + 120 列）
- 引擎目标开启 `-Wall -Wextra -Wpedantic -Wshadow -Wold-style-cast ...`，CI 中视警告为错误
- 命名：类型 `PascalCase`，函数 `PascalCase`，成员 `m_camelCase`，常量 `kPascalCase`
- 头文件只放声明与短小的内联函数；注释解释"为什么"，而不仅是"做什么"
