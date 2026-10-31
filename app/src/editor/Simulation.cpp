#include "editor/Simulation.h"

#include <algorithm>
#include <filesystem>

#include <kinema_scene/SceneLibrary.h>
#include <kinema_scene/SceneSerializer.h>

#include "util/Log.h"

namespace app {

Simulation::Simulation() : m_world(std::make_unique<kinema::World>()) {}

void Simulation::LoadScene(int index) {
    const auto& scenes = kinema::scene::GetScenes();
    if (index < 0 || index >= static_cast<int>(scenes.size())) return;

    // 保留用户调过的求解器设置，只把重力恢复默认
    kinema::WorldSettings settings = m_world->GetSettings();
    settings.gravity = kinema::WorldSettings{}.gravity;

    m_world->Clear();
    m_world->SetSettings(settings);
    scenes[static_cast<size_t>(index)].build(*m_world);

    m_sceneIndex = index;
    m_sceneName = scenes[static_cast<size_t>(index)].name;
    m_fileSnapshot.clear();
    ResetTime();

    Log::Info("Loaded scene '%s' (%zu bodies, %zu joints)", m_sceneName.c_str(), m_world->GetBodies().size(),
              m_world->GetJoints().size());
}

bool Simulation::LoadFile(const std::string& path, std::string* error) {
    std::string text;
    {
        std::string err;
        kinema::World probe;  // 先读到临时世界，失败时不破坏当前场景
        if (!kinema::scene::LoadWorldFromFile(probe, path, &err)) {
            if (error) *error = err;
            return false;
        }
        text = kinema::scene::SerializeWorld(probe);
    }

    kinema::scene::DeserializeWorld(*m_world, text);
    m_fileSnapshot = std::move(text);
    m_sceneIndex = -1;
    m_sceneName = std::filesystem::path(path).filename().string();
    ResetTime();
    Log::Info("Opened '%s' (%zu bodies)", path.c_str(), m_world->GetBodies().size());
    return true;
}

bool Simulation::SaveFile(const std::string& path, std::string* error) const {
    if (!kinema::scene::SaveWorldToFile(*m_world, path, error)) return false;
    Log::Info("Saved scene to '%s'", path.c_str());
    return true;
}

void Simulation::Reset() {
    if (m_sceneIndex >= 0) {
        LoadScene(m_sceneIndex);
    } else if (!m_fileSnapshot.empty()) {
        kinema::scene::DeserializeWorld(*m_world, m_fileSnapshot);
        ResetTime();
        Log::Info("Reset '%s'", m_sceneName.c_str());
    }
}

void Simulation::ResetTime() {
    m_accumulator = 0.0;
    m_simTime = 0.0;
    m_stepCount = 0;
}

void Simulation::StepOnce() {
    kinema::Timer timer;
    const float dt = 1.0f / hz;
    const int n = std::max(subSteps, 1);
    for (int i = 0; i < n; ++i) m_world->Step(dt / static_cast<float>(n));
    m_simTime += dt;
    ++m_stepCount;
    m_lastStepMs = static_cast<float>(timer.ElapsedMs());
}

int Simulation::Update(float frameDt) {
    if (paused) {
        m_accumulator = 0.0;
        return 0;
    }

    // 防止"死亡螺旋"：卡顿时不要试图一次追回太多时间
    frameDt = std::min(frameDt, 0.1f);
    m_accumulator += static_cast<double>(frameDt * timeScale);

    const double dt = 1.0 / static_cast<double>(hz);
    constexpr int kMaxStepsPerFrame = 8;
    int steps = 0;
    while (m_accumulator >= dt && steps < kMaxStepsPerFrame) {
        StepOnce();
        m_accumulator -= dt;
        ++steps;
    }
    if (steps == kMaxStepsPerFrame) m_accumulator = 0.0;  // 跟不上了，丢弃剩余时间
    return steps;
}

}  // namespace app
