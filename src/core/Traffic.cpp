#include "core/Traffic.h"

#include <cstddef>

namespace logistics {

namespace {

// 手写 xorshift32 伪随机数发生器。
// 核心层不引入 <random>（§10.1），且确定性种子让测试可复现。
class Rng {
public:
    explicit Rng(unsigned int seed)
        : state_(seed == 0u ? 0x9E3779B9u : seed) {}

    unsigned int nextU32() {
        unsigned int x = state_;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        state_ = x;
        return x;
    }

    // 返回 [0, 1) 内的数；取高 24 位以保证精度足够且不引入低位偏差
    double nextUnit() {
        return static_cast<double>(nextU32() >> 8) / 16777216.0;
    }

    // 返回 [low, high] 闭区间内的整数
    int nextInt(int low, int high) {
        if (high <= low) {
            return low;
        }
        const unsigned int span = static_cast<unsigned int>(high - low + 1);
        return low + static_cast<int>(nextU32() % span);
    }

private:
    unsigned int state_;
};

} // namespace

TrafficReport simulateTrafficChange(LogisticsGraph& graph,
                                    double ratio,
                                    double minRatio,
                                    double maxRatio,
                                    unsigned int seed) {
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

    for (std::size_t i = 0; i < count; ++i) {
        const Edge& edge = pool[i];
        const double increase = lo + (hi - lo) * rng.nextUnit();

        const double newTime = edge.baseTimeMin * (1.0 + increase);
        if (!graph.updateEdgeTime(edge.fromId, edge.toId, newTime)) {
            continue;
        }

        TrafficChange change;
        change.fromId = edge.fromId;
        change.toId = edge.toId;
        change.increaseRatio = increase;
        report.changes.push_back(change);
    }

    return report;
}

bool needsReplan(const std::vector<std::string>& routeNodes,
                 const TrafficReport& report,
                 double thresholdRatio) {
    for (std::size_t i = 1; i < routeNodes.size(); ++i) {
        for (const TrafficChange& c : report.changes) {
            if (c.fromId == routeNodes[i - 1] && c.toId == routeNodes[i]
                && c.increaseRatio >= thresholdRatio) {
                return true;
            }
        }
    }
    return false;
}

} // namespace logistics
