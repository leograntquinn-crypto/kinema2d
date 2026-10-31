#include "util/I18n.h"

#include <string_view>
#include <unordered_map>

namespace app {

namespace {

Language g_language = Language::English;

struct Entry {
    const char* en;
    const char* zh;
};

// 翻译表：英文原文 -> 简体中文
// 新增界面文字时，在这里补一行即可；缺失的条目会自动回退到英文。
constexpr Entry kTable[] = {
    // ---- 窗口 / 菜单 ----
    {"Viewport", "视口"},
    {"Scenes", "场景"},
    {"Outliner", "大纲"},
    {"Inspector", "属性"},
    {"Settings", "设置"},
    {"Performance", "性能"},
    {"Console", "控制台"},
    {"File", "文件"},
    {"Edit", "编辑"},
    {"Simulation", "模拟"},
    {"View", "视图"},
    {"Help", "帮助"},
    {"New Sandbox", "新建沙盒"},
    {"Open Scene...", "打开场景..."},
    {"Save Scene As...", "场景另存为..."},
    {"Open Scene", "打开场景"},
    {"Save Scene", "保存场景"},
    {"Save Screenshot", "保存截图"},
    {"Exit", "退出"},
    {"Delete Selected", "删除选中物体"},
    {"Deselect", "取消选择"},
    {"Wake All Bodies", "唤醒所有物体"},
    {"Play", "运行"},
    {"Pause", "暂停"},
    {"Step", "单步"},
    {"Reset Scene", "重置场景"},
    {"Reset Layout", "重置布局"},
    {"Reset Camera", "重置相机"},
    {"Follow Selection", "相机跟随选中物体"},
    {"Theme", "主题"},
    {"Dark", "深色"},
    {"Light", "浅色"},
    {"ImGui Demo", "ImGui 示例窗口"},
    {"Controls", "操作说明"},
    {"About Kinema2D", "关于 Kinema2D"},
    {"Close", "关闭"},
    {"Input", "输入"},
    {"Action", "功能"},
    {"File path:", "文件路径："},
    {"Open", "打开"},
    {"Save", "保存"},
    {"Cancel", "取消"},
    {"Opened", "已打开"},
    {"Saved", "已保存"},
    {"(no .kscene files in the working directory)", "（工作目录下没有 .kscene 文件）"},
    {"Saves the current state (positions and velocities) of the world.", "保存世界的当前状态（位置与速度）。"},
    {"A 2D rigid body physics engine built for learning.", "一个为学习而编写的 2D 刚体物理引擎。"},
    {"Sequential impulse solver with warm starting", "带热启动的顺序冲量求解器"},
    {"SAT + clipping contact manifolds", "分离轴定理 + 裁剪生成接触流形"},
    {"Sweep-and-prune broad phase", "排序扫掠宽阶段"},
    {"Island-based sleeping", "基于岛屿的休眠"},
    {"Distance, revolute (motor) and mouse joints", "距离关节、旋转关节（马达）、鼠标关节"},

    // ---- 操作说明 ----
    {"Play / pause", "运行 / 暂停"},
    {"Step one frame", "前进一帧"},
    {"Reset scene", "重置场景"},
    {"Select tool", "切换工具"},
    {"Delete selected body", "删除选中物体"},
    {"Toggle camera follow", "切换相机跟随"},
    {"Reset camera", "重置相机"},
    {"Save / open scene", "保存 / 打开场景"},
    {"Save screenshot", "保存截图"},
    {"Zoom at cursor", "以光标为中心缩放"},
    {"Pan camera", "平移相机"},
    {"Pan camera (Select tool)", "平移相机（选择工具）"},

    // ---- 状态 / 提示 ----
    {"Paused", "已暂停"},
    {"Running", "运行中"},
    {"PAUSED", "已暂停"},
    {"Scene", "场景"},
    {"Bodies", "物体"},
    {"awake", "活跃"},
    {"sleeping", "休眠"},
    {"Contacts", "接触"},
    {"Tool", "工具"},
    {"Scene reset", "场景已重置"},
    {"Camera follows selection", "相机跟随选中物体"},
    {"Camera follow off", "相机跟随已关闭"},
    {"Joint created", "已创建关节"},
    {"Screenshot saved", "截图已保存"},

    // ---- 工具 ----
    {"Select", "选择"},
    {"Circle", "圆形"},
    {"Box", "方块"},
    {"Polygon", "多边形"},
    {"Static", "静态"},
    {"Joint", "关节"},
    {"Explode", "爆炸"},
    {"Ray", "射线"},
    {"Delete", "删除"},
    {"Click to select. Drag to move bodies (mouse joint while running).",
     "点击选择物体；拖动移动物体（运行时通过鼠标关节拖拽）。"},
    {"Click to spawn a circle. Drag to launch it like a slingshot.", "点击生成圆形；拖动可以像弹弓一样发射。"},
    {"Click to spawn a box. Drag to launch it like a slingshot.", "点击生成方块；拖动可以像弹弓一样发射。"},
    {"Click to spawn a convex polygon. Drag to launch it.", "点击生成凸多边形；拖动发射。"},
    {"Drag a rectangle to create a static box.", "拖出一个矩形，创建静态方块。"},
    {"Drag from one body to another to connect them.", "从一个物体拖到另一个物体，用关节连接它们。"},
    {"Click to apply a radial explosion impulse.", "点击施加径向爆炸冲量。"},
    {"Drag to cast a ray and see the hit point and normal.", "拖动发射射线，查看命中点与法线。"},
    {"Click a body to delete it.", "点击物体将其删除。"},
    {"Play (Space)", "运行（空格）"},
    {"Pause (Space)", "暂停（空格）"},
    {"Step one frame (N)", "前进一帧（N）"},
    {"Reset scene (R)", "重置场景（R）"},
    {"Speed", "速度"},
    {"Follow", "跟随"},
    {"Unfollow", "取消跟随"},
    {"Mass", "质量"},

    // ---- 引擎类型名 ----
    {"Dynamic", "动态"},
    {"Kinematic", "运动学"},
    {"Distance", "距离关节"},
    {"Revolute", "旋转关节"},
    {"Mouse", "鼠标关节"},

    // ---- 场景 ----
    {"Sandbox", "沙盒"},
    {"Pyramid", "金字塔"},
    {"Wrecking Ball", "破坏球"},
    {"Newton's Cradle", "牛顿摆"},
    {"Rope Bridge", "吊桥"},
    {"Chain", "锁链"},
    {"Dominoes", "多米诺骨牌"},
    {"Funnel (400 bodies)", "漏斗（400 个物体）"},
    {"Windmill", "风车"},
    {"Car", "小车"},
    {"Materials", "材质对比"},
    {"Soft Bodies", "软体"},
    {"Mixer", "搅拌机"},
    {"Empty playground. Use the toolbar to spawn bodies.", "空白场地，使用工具栏创建物体。"},
    {"210 stacked boxes: tests solver stability, warm starting and sleeping.",
     "210 个堆叠的箱子：测试求解器稳定性、热启动与休眠。"},
    {"A heavy ball on a rigid distance joint smashes two towers.", "刚性距离关节吊着的重球撞倒两座塔。"},
    {"Elastic balls (restitution = 1) hanging on distance joints.", "用距离关节悬挂的完全弹性小球（弹性系数 = 1）。"},
    {"Planks connected with revolute joints between two pillars.", "两根柱子之间由旋转关节连接的木板。"},
    {"A long chain of revolute joints with a heavy ball at the end.", "由旋转关节组成的长链，末端挂着重球。"},
    {"Classic chain reaction - friction and rotational dynamics.", "经典连锁反应：摩擦与转动动力学。"},
    {"Many mixed shapes poured through a funnel: a broad-phase benchmark.",
     "大量形状倒入漏斗：宽阶段性能测试。"},
    {"Motorized revolute joints + collision filtering between the blades.", "带马达的旋转关节 + 叶片之间的碰撞过滤。"},
    {"Motorized wheels on rough terrain. Select the chassis and enable Follow.",
     "马达驱动的车轮行驶在崎岖地形上。选中车身并开启跟随。"},
    {"Left: restitution 0.0..1.0. Right: friction 0.0..0.5 on 15-degree ramps.",
     "左：弹性系数 0.0~1.0；右：15° 斜坡上摩擦系数 0.0~0.5。"},
    {"Particle grids linked by soft distance joints (springs).", "由软距离关节（弹簧）连接的粒子网格。"},
    {"A kinematic paddle stirring dynamic bodies.", "运动学桨叶搅动动态物体。"},

    // ---- 大纲 ----
    {"Search (#id or shape)", "搜索（#编号 或 形状）"},
    {"All", "全部"},
    {"Awake", "活跃"},
    {"bodies", "个物体"},
    {"joints", "个关节"},
    {"Body", "物体"},
    {"Type", "类型"},
    {"State", "状态"},

    // ---- 属性面板 ----
    {"No body selected.", "未选中物体。"},
    {"Click a body in the viewport (Select tool) or in the Outliner to inspect it.",
     "在视口中（选择工具）或大纲中点击物体以查看其属性。"},
    {"Focus", "聚焦"},
    {"Transform", "变换"},
    {"Position", "位置"},
    {"Angle", "角度"},
    {"Velocity", "速度"},
    {"Linear", "线速度"},
    {"Angular", "角速度"},
    {"Shape", "形状"},
    {"Radius", "半径"},
    {"Vertices", "顶点数"},
    {"Size", "尺寸"},
    {"Scale", "缩放"},
    {"Uniformly scale the polygon", "等比例缩放多边形"},
    {"Material", "材质"},
    {"Density", "密度"},
    {"Friction", "摩擦系数"},
    {"Restitution", "弹性系数"},
    {"Coulomb friction coefficient", "库仑摩擦系数"},
    {"Bounciness: 0 = no bounce, 1 = perfectly elastic", "弹性：0 = 不反弹，1 = 完全弹性"},
    {"Dynamics", "动力学"},
    {"Body type", "物体类型"},
    {"Linear damping", "线性阻尼"},
    {"Angular damping", "角阻尼"},
    {"Gravity scale", "重力倍数"},
    {"Fixed rotation", "固定旋转"},
    {"Allow sleep", "允许休眠"},
    {"Collision Filter", "碰撞过滤"},
    {"Category bits", "类别位"},
    {"Mask bits", "掩码位"},
    {"Mass Properties", "质量属性"},
    {"Inertia", "转动惯量"},
    {"Kinetic energy", "动能"},
    {"Momentum", "动量"},
    {"Sleep timer", "静止计时"},
    {"Joints", "关节"},
    {"Use the Joint tool to connect bodies.", "使用关节工具连接物体。"},
    {"Motor", "马达"},
    {"Motor speed", "马达转速"},
    {"Target relative angular velocity (rad/s)", "目标相对角速度（rad/s）"},
    {"Max torque", "最大扭矩"},
    {"Length", "长度"},
    {"Frequency", "频率"},
    {"0 = rigid rod, > 0 = spring stiffness (Hz)", "0 = 刚性杆，> 0 = 弹簧刚度（Hz）"},
    {"Damping ratio", "阻尼比"},
    {"Reaction force", "约束反力"},
    {"Delete joint", "删除关节"},
    {"Kick upward", "向上踢一脚"},
    {"Spin", "旋转"},

    // ---- 设置面板 ----
    {"World", "世界"},
    {"Gravity", "重力"},
    {"Earth", "地球"},
    {"Moon", "月球"},
    {"Zero-G", "失重"},
    {"Flip", "反转"},
    {"Remove bodies below kill plane", "删除掉出世界的物体"},
    {"Time Step", "时间步长"},
    {"Physics rate", "物理频率"},
    {"Fixed time step frequency. Higher = more accurate but more expensive.", "固定时间步频率。越高越精确，但计算量越大。"},
    {"Sub-steps", "子步数"},
    {"Each fixed step is split into N smaller steps. Greatly improves stiff stacks and fast objects.",
     "每个固定步再细分为 N 个小步。能显著改善高堆叠和高速物体的表现。"},
    {"Time scale", "时间缩放"},
    {"Solver", "求解器"},
    {"Velocity iterations", "速度迭代次数"},
    {"How many times the sequential impulse solver sweeps over all constraints per step.",
     "每一步中顺序冲量求解器遍历所有约束的次数。"},
    {"Warm starting", "热启动"},
    {"Reuse last frame's impulses as the initial guess. Turn it off and watch the pyramid sag.",
     "用上一帧的冲量作为初始猜测。关掉它，看看金字塔会怎样下沉。"},
    {"Baumgarte", "Baumgarte 系数"},
    {"Fraction of penetration corrected per step. 0 = bodies sink, too high = jitter.",
     "每步修正穿透量的比例。0 = 物体下沉，过高 = 抖动。"},
    {"Linear slop", "穿透容差"},
    {"Allowed penetration. A small slop prevents contacts from flickering.", "允许的穿透量。少量容差可以防止接触闪烁。"},
    {"Restitution threshold", "反弹速度阈值"},
    {"Broad phase", "宽阶段算法"},
    {"Brute Force O(n²)", "暴力枚举 O(n²)"},
    {"Sweep and Prune", "排序扫掠"},
    {"Compare both on the Funnel scene and watch the Performance panel.", "在漏斗场景中切换两种算法，观察性能面板。"},
    {"Allow sleeping", "允许休眠"},
    {"Islands of bodies that stay still for a while are put to sleep and skipped.",
     "静止一段时间的物体岛屿会进入休眠，不再参与计算。"},
    {"Time to sleep", "入睡时间"},
    {"Debug Draw", "调试绘制"},
    {"Shapes", "形状"},
    {"Outlines", "轮廓"},
    {"Grid", "网格"},
    {"AABBs", "包围盒"},
    {"Contact points", "接触点"},
    {"Contact normals", "接触法线"},
    {"Contact impulses", "接触冲量"},
    {"Centers of mass", "质心"},
    {"Velocities", "速度向量"},
    {"Sleep tint", "休眠变灰"},
    {"Rotation markers", "旋转标记"},
    {"Fill opacity", "填充不透明度"},
    {"Off", "关闭"},
    {"Anti-aliasing", "抗锯齿"},
    {"Tool Options", "工具参数"},
    {"New bodies", "新物体"},
    {"Circle radius", "圆半径"},
    {"Box half size", "方块半尺寸"},
    {"Polygon sides", "多边形边数"},
    {"random", "随机"},
    {"Polygon radius", "多边形半径"},
    {"Launch strength", "发射力度"},
    {"Random color", "随机颜色"},
    {"Joint tool", "关节工具"},
    {"Joint type", "关节类型"},
    {"Rod (rigid distance)", "刚性杆（距离关节）"},
    {"Spring (soft distance)", "弹簧（软距离关节）"},
    {"Hinge (revolute)", "铰链（旋转关节）"},
    {"Spring frequency", "弹簧频率"},
    {"Spring damping", "弹簧阻尼"},
    {"Connected bodies collide", "被连接物体之间碰撞"},
    {"Explosion", "爆炸"},
    {"Strength", "强度"},

    // ---- 性能面板 ----
    {"FPS", "帧率"},
    {"Frame time", "帧时间"},
    {"Awake bodies", "活跃物体"},
    {"Islands", "岛屿"},
    {"Broad-phase pairs", "宽阶段候选对"},
    {"Draw calls", "绘制调用"},
    {"Last physics step", "最近一次物理步"},
    {"Narrow phase", "窄阶段"},
    {"Integrate", "积分"},
    {"Sleep / islands", "休眠 / 岛屿"},
    {"Total", "总计"},
    {"Steps", "步数"},

    // ---- 控制台 ----
    {"Clear", "清空"},
    {"Auto-scroll", "自动滚动"},
};

const std::unordered_map<std::string_view, const char*>& Map() {
    static const std::unordered_map<std::string_view, const char*> map = [] {
        std::unordered_map<std::string_view, const char*> m;
        for (const Entry& e : kTable) m.emplace(e.en, e.zh);
        return m;
    }();
    return map;
}

}  // namespace

void SetLanguage(Language lang) { g_language = lang; }
Language GetLanguage() { return g_language; }

const char* Tr(const char* english) {
    if (g_language == Language::English || english == nullptr) return english;
    const auto& map = Map();
    const auto it = map.find(english);
    return it != map.end() ? it->second : english;
}

std::string TrId(const char* english, const char* id) { return std::string(Tr(english)) + "###" + id; }

std::vector<const char*> AllTranslations() {
    std::vector<const char*> out;
    out.reserve(sizeof(kTable) / sizeof(kTable[0]));
    for (const Entry& e : kTable) out.push_back(e.zh);
    return out;
}

}  // namespace app
