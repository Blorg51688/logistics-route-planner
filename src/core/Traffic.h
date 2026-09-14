#pragma once

#include <string>
#include <vector>

#include "core/LogisticsGraph.h"

namespace logistics {

// 一次路况变化中被改动的边。
struct TrafficChange {
    std::string fromId;
    std::string toId;
    // 该边耗时相对 baseTimeMin 的增幅（0.2 表示 +20%）。
    // 每次路况刷新把 timeMin 重设为 baseTimeMin * (1 + 增幅)，**不叠加**，
    // 因此该值就是本次施加的增幅，也保证了耗时不会逐次失控增长。
    double      increaseRatio = 0.0;
};

struct TrafficReport {
    std::vector<TrafficChange> changes;   // 本次被改动的边，按选定顺序
};

// 模拟路况变化（§5.4）：
//   选取 round(ratio * 边数) 条边（ratio > 0 时至少 1 条），
//   将其耗时设为 baseTimeMin * (1 + [minRatio, maxRatio] 内的随机值)，
//   并置拥堵标记。**baseTimeMin 始终不变**——它是判断"耗时增加 ≥20%"的基准。
//
// 手写 xorshift32 保证可复现：同一种子必得完全相同的改动集合与增幅。
// 不使用 <random>：核心层不引入类库（§10.1），且确定性种子让测试不 flaky。
TrafficReport simulateTrafficChange(LogisticsGraph& graph,
                                    double ratio,
                                    double minRatio,
                                    double maxRatio,
                                    unsigned int seed);

// 是否需要触发重规划（§5.4）：
// 仅当某条被改动的边恰好是 routeNodes 中**相邻的一对**（有向，方向必须一致），
// 且其 increaseRatio >= thresholdRatio 时返回 true。
bool needsReplan(const std::vector<std::string>& routeNodes,
                 const TrafficReport& report,
                 double thresholdRatio);

} // namespace logistics
