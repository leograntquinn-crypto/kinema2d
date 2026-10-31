// =============================================================================
// Simulation：在 World 外面包一层"时间管理"
//
// 固定时间步长（Fixed Timestep）+ 累加器：
//   渲染帧率会波动（30~240 FPS），但物理必须用固定的 dt 推进才能稳定、可复现。
//   每帧把真实流逝的时间加进累加器，然后按固定 dt 消耗它。
//   参考：Glenn Fiedler, "Fix Your Timestep!"
//
// 另外负责：暂停/单步、子步、时间缩放、场景加载与重置。
// =============================================================================
#pragma once

#include <memory>
#include <string>

#include <kinema/Kinema.h>

namespace app {

class Simulation {
public:
    Simulation();

    [[nodiscard]] kinema::World& GetWorld() { return *m_world; }
    [[nodiscard]] const kinema::World& GetWorld() const { return *m_world; }

    // ---- 场景 ----
    void LoadScene(int index);
    bool LoadFile(const std::string& path, std::string* error);
    bool SaveFile(const std::string& path, std::string* error) const;
    /// 回到当前场景的初始状态
    void Reset();
    [[nodiscard]] int GetSceneIndex() const { return m_sceneIndex; }
    [[nodiscard]] const std::string& GetSceneName() const { return m_sceneName; }

    // ---- 时间 ----
    /// 每帧调用，frameDt 为真实流逝时间（秒）。返回本帧执行的物理步数。
    int Update(float frameDt);
    /// 执行一个固定步（包括子步）
    void StepOnce();

    bool paused = false;
    float hz = 60.0f;        // 物理频率
    int subSteps = 1;        // 每个固定步再细分几次
    float timeScale = 1.0f;  // 慢动作 / 快进

    [[nodiscard]] double GetSimTime() const { return m_simTime; }
    [[nodiscard]] uint64_t GetStepCount() const { return m_stepCount; }
    /// 最近一个固定步（含子步）的总耗时
    [[nodiscard]] float GetLastStepMs() const { return m_lastStepMs; }

private:
    void ResetTime();

    std::unique_ptr<kinema::World> m_world;
    int m_sceneIndex = -1;
    std::string m_sceneName;
    std::string m_fileSnapshot;  // 从文件加载时保存原始文本，用于 Reset

    double m_accumulator = 0.0;
    double m_simTime = 0.0;
    uint64_t m_stepCount = 0;
    float m_lastStepMs = 0.0f;
};

}  // namespace app
