#include "core/Traffic.h"

#include "core/Random.h"
#include "core/RoutePlanner.h"

#include <cstddef>

namespace logistics {

TrafficReport simulateTrafficChange(LogisticsGraph& graph,
                                    double ratio,
                                    double minRatio,
                                    double maxRatio,
                                    unsigned int seed,
                                    double clearProbability) {
    TrafficReport report;
    if (ratio <= 0.0) {
        return report;
    }

    const std::vector<Edge> all = graph.edges();
    const std::size_t total = all.size();
    if (total == 0) {
        return report;
    }

    // 改动条数：round(ratio * E)，ratio > 0 时至少 1 条
    long long wanted = static_cast<long long>(ratio * static_cast<double>(total) + 0.5);
    if (wanted < 1) {
        wanted = 1;
    }
    if (wanted > static_cast<long long>(total)) {
        wanted = static_cast<long long>(total);
    }
    const std::size_t count = static_cast<std::size_t>(wanted);

    // 部分 Fisher-Yates：只需前 count 个位置被打乱，走完全量是浪费
    std::vector<Edge> pool = all;
    Rng rng(seed);
    for (std::size_t i = 0; i < count; ++i) {
        const std::size_t j = static_cast<std::size_t>(
            rng.nextInt(static_cast<int>(i), static_cast<int>(total - 1)));
        const Edge tmp = pool[i];
        pool[i] = pool[j];
        pool[j] = tmp;
    }

    const double lo = minRatio < 0.0 ? 0.0 : minRatio;
    const double hi = maxRatio < lo ? lo : maxRatio;

    const double clearChance = clearProbability < 0.0 ? 0.0
                               : (clearProbability > 1.0 ? 1.0 : clearProbability);

    for (std::size_t i = 0; i < count; ++i) {
        const Edge& edge = pool[i];

        // 畅通还是拥堵：需求把路况定义为"拥堵 / 畅通"两种动态属性，两者都要能出现
        const bool becomesClear = rng.nextUnit() < clearChance;
        const double increase = becomesClear ? 0.0 : (lo + (hi - lo) * rng.nextUnit());
        const double newTime = edge.baseTimeMin * (1.0 + increase);

        if (!graph.updateEdgeTime(edge.fromId, edge.toId, newTime)) {
            continue;
        }

        TrafficChange change;
        change.fromId = edge.fromId;
        change.toId = edge.toId;
        change.increaseRatio = increase;
        change.congested = !becomesClear;
        report.changes.push_back(change);
    }

    return report;
}

bool needsReplan(const std::vector<std::string>& routeNodes,
                 const TrafficReport& report,
                 double thresholdRatio) {
    for (const TrafficChange& c : report.changes) {
        if (c.increaseRatio >= thresholdRatio
            && isEdgeOnRoute(routeNodes, c.fromId, c.toId)) {
            return true;
        }
    }
    return false;
}

} // namespace logistics
