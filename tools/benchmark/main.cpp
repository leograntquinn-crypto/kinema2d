// =============================================================================
// kinema_bench：无界面的性能基准测试
//
// 用法：kinema_bench [scene-id ...] [--steps N]
// 对每个场景分别用两种宽阶段算法运行 N 步，输出各阶段平均耗时。
// 这是优化引擎时最重要的工具 —— 先测量，再优化。
// =============================================================================
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include <kinema/Kinema.h>
#include <kinema_scene/SceneLibrary.h>

using namespace kinema;

namespace {

struct Result {
    double total = 0, broad = 0, narrow = 0, solver = 0, integrate = 0, sleep = 0;
    int bodies = 0, contacts = 0;
};

Result Run(const scene::SceneInfo& info, BroadPhaseType bp, int steps) {
    WorldSettings s;
    s.broadPhase = bp;
    World world(s);
    info.build(world);

    Result r;
    Timer wall;
    for (int i = 0; i < steps; ++i) {
        world.Step(1.0f / 60.0f);
        const WorldStats& st = world.GetStats();
        r.broad += st.broadPhaseMs;
        r.narrow += st.narrowPhaseMs;
        r.solver += st.solverMs;
        r.integrate += st.integrateMs;
        r.sleep += st.sleepMs;
        r.contacts = std::max(r.contacts, st.contactCount);
    }
    r.total = wall.ElapsedMs();
    r.bodies = static_cast<int>(world.GetBodies().size());
    const double inv = 1.0 / steps;
    r.total *= inv;
    r.broad *= inv;
    r.narrow *= inv;
    r.solver *= inv;
    r.integrate *= inv;
    r.sleep *= inv;
    return r;
}

}  // namespace

int main(int argc, char** argv) {
    int steps = 600;
    std::vector<std::string> ids;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--steps" && i + 1 < argc) steps = std::max(1, std::atoi(argv[++i]));
        else if (a == "--help") {
            std::printf("Usage: kinema_bench [scene-id ...] [--steps N]\n");
            return 0;
        } else ids.push_back(a);
    }
    if (ids.empty()) ids = {"pyramid", "funnel", "windmill", "softbody", "mixer"};

    std::printf("Kinema2D benchmark  (%d steps @ 60 Hz, averages per step)\n\n", steps);
    std::printf("%-10s %-16s %7s %9s %9s %9s %9s %9s %9s\n", "scene", "broad phase", "bodies", "contacts", "total",
                "broad", "narrow", "solver", "sleep");
    std::printf("%s\n", std::string(96, '-').c_str());

    for (const std::string& id : ids) {
        const int index = scene::FindScene(id);
        if (index < 0) {
            std::fprintf(stderr, "Unknown scene: %s\n", id.c_str());
            continue;
        }
        const auto& info = scene::GetScenes()[static_cast<size_t>(index)];
        for (BroadPhaseType bp : {BroadPhaseType::BruteForce, BroadPhaseType::SweepAndPrune}) {
            const Result r = Run(info, bp, steps);
            std::printf("%-10s %-16s %7d %9d %7.3fms %7.3fms %7.3fms %7.3fms %7.3fms\n", id.c_str(), ToString(bp),
                        r.bodies, r.contacts, r.total, r.broad, r.narrow, r.solver, r.sleep);
        }
    }
    return 0;
}
