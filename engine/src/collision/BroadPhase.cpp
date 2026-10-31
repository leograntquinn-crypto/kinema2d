#include "kinema/collision/BroadPhase.h"

#include <algorithm>

namespace kinema {

const char* ToString(BroadPhaseType type) {
    switch (type) {
        case BroadPhaseType::BruteForce: return "Brute Force";
        case BroadPhaseType::SweepAndPrune: return "Sweep and Prune";
    }
    return "Unknown";
}

void BruteForceBroadPhase::ComputePairs(const std::vector<BroadPhaseProxy>& proxies,
                                        std::vector<ProxyPair>& outPairs) {
    outPairs.clear();
    const size_t n = proxies.size();
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = i + 1; j < n; ++j) {
            if (Overlaps(proxies[i].aabb, proxies[j].aabb)) outPairs.emplace_back(proxies[i].index, proxies[j].index);
        }
    }
}

void SweepAndPruneBroadPhase::ComputePairs(const std::vector<BroadPhaseProxy>& proxies,
                                           std::vector<ProxyPair>& outPairs) {
    outPairs.clear();
    if (proxies.size() < 2) return;

    // 1) 选轴：比较 x、y 方向上中心点的方差
    float sumX = 0.0f, sumY = 0.0f, sumX2 = 0.0f, sumY2 = 0.0f;
    for (const auto& p : proxies) {
        const Vec2 c = p.aabb.Center();
        sumX += c.x;
        sumY += c.y;
        sumX2 += c.x * c.x;
        sumY2 += c.y * c.y;
    }
    const float invN = 1.0f / static_cast<float>(proxies.size());
    const float varX = sumX2 * invN - (sumX * invN) * (sumX * invN);
    const float varY = sumY2 * invN - (sumY * invN) * (sumY * invN);
    const bool useX = varX >= varY;

    auto minOf = [useX](const AABB& b) { return useX ? b.lower.x : b.lower.y; };
    auto maxOf = [useX](const AABB& b) { return useX ? b.upper.x : b.upper.y; };

    // 2) 排序。由于物体每帧移动很少，上一帧的顺序几乎是有序的；
    //    这里为了代码清晰直接用 std::sort（进阶：改用插入排序利用时间相干性）。
    m_sorted = proxies;
    std::sort(m_sorted.begin(), m_sorted.end(),
              [&](const BroadPhaseProxy& a, const BroadPhaseProxy& b) { return minOf(a.aabb) < minOf(b.aabb); });

    // 3) 扫掠
    const size_t n = m_sorted.size();
    for (size_t i = 0; i < n; ++i) {
        const AABB& a = m_sorted[i].aabb;
        const float maxA = maxOf(a);
        for (size_t j = i + 1; j < n; ++j) {
            const AABB& b = m_sorted[j].aabb;
            if (minOf(b) > maxA) break;  // 后面的物体都在右边，不可能再重叠
            if (Overlaps(a, b)) outPairs.emplace_back(m_sorted[i].index, m_sorted[j].index);
        }
    }
}

std::unique_ptr<BroadPhase> CreateBroadPhase(BroadPhaseType type) {
    switch (type) {
        case BroadPhaseType::BruteForce: return std::make_unique<BruteForceBroadPhase>();
        case BroadPhaseType::SweepAndPrune: return std::make_unique<SweepAndPruneBroadPhase>();
    }
    return std::make_unique<SweepAndPruneBroadPhase>();
}

}  // namespace kinema
