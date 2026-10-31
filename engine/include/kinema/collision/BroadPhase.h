// =============================================================================
// 宽阶段（Broad Phase）
//
// 目标：在 N 个物体中快速找出"AABB 重叠"的候选对，避免 O(N²) 次窄阶段测试。
//
// 这里使用策略模式（Strategy Pattern）提供两种实现，方便在界面中切换对比性能：
//   * BruteForceBroadPhase   : O(N²)，最简单，作为正确性基准
//   * SweepAndPruneBroadPhase: 排序扫掠，平均 O(N log N)
// =============================================================================
#pragma once

#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

#include "kinema/math/AABB.h"

namespace kinema {

/// 宽阶段代理：一个 AABB + 它所属物体在 World 中的索引
struct BroadPhaseProxy {
    AABB aabb;
    uint32_t index = 0;
};

using ProxyPair = std::pair<uint32_t, uint32_t>;

enum class BroadPhaseType : uint8_t {
    BruteForce = 0,
    SweepAndPrune = 1,
};

const char* ToString(BroadPhaseType type);

class BroadPhase {
public:
    virtual ~BroadPhase() = default;

    /// 计算所有 AABB 重叠的代理对，结果写入 outPairs（每对 first < second 的顺序不保证）
    virtual void ComputePairs(const std::vector<BroadPhaseProxy>& proxies, std::vector<ProxyPair>& outPairs) = 0;

    [[nodiscard]] virtual BroadPhaseType GetType() const = 0;
};

class BruteForceBroadPhase final : public BroadPhase {
public:
    void ComputePairs(const std::vector<BroadPhaseProxy>& proxies, std::vector<ProxyPair>& outPairs) override;
    [[nodiscard]] BroadPhaseType GetType() const override { return BroadPhaseType::BruteForce; }
};

/// 排序扫掠（Sort and Sweep / Sweep and Prune）
///   1. 选择物体分布更"分散"的轴（方差更大）作为扫掠轴
///   2. 按 AABB 在该轴上的最小值排序
///   3. 从左到右扫描，只与"区间仍然重叠"的物体做完整 AABB 测试
class SweepAndPruneBroadPhase final : public BroadPhase {
public:
    void ComputePairs(const std::vector<BroadPhaseProxy>& proxies, std::vector<ProxyPair>& outPairs) override;
    [[nodiscard]] BroadPhaseType GetType() const override { return BroadPhaseType::SweepAndPrune; }

private:
    std::vector<BroadPhaseProxy> m_sorted;  // 复用内存，避免每帧分配
};

std::unique_ptr<BroadPhase> CreateBroadPhase(BroadPhaseType type);

}  // namespace kinema
