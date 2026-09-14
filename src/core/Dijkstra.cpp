#include "core/Dijkstra.h"

#include <cstddef>

#include "core/MinHeap.h"

namespace logistics {

namespace {

// 不可达标记。真实权重远小于该值。
const double kInfinity = 1e100;

// 取边在指定维度上的权重。手写分支，不引入 <algorithm> 或任何库算法。
double weightOf(const Edge& edge, WeightType type) {
    switch (type) {
        case WeightType::Distance: return edge.distanceKm;
        case WeightType::Time:     return edge.timeMin;
        case WeightType::Cost:     return edge.costYuan;
    }
    return 0.0;
}

} // namespace

PathResult shortestPath(const LogisticsGraph& graph,
                        const std::string& sourceId,
                        const std::string& targetId,
                        WeightType weight) {
    PathResult result;

    const int source = graph.indexOf(sourceId);
    const int target = graph.indexOf(targetId);
    if (source < 0 || target < 0) {
        return result;  // 端点不存在
    }

    const std::size_t count = graph.nodes().size();
    std::vector<double> dist(count, kInfinity);
    std::vector<int> previous(count, -1);
    const std::vector<std::vector<Edge>>& adjacency = graph.adjacency();

    dist[static_cast<std::size_t>(source)] = 0.0;

    MinHeap heap;
    heap.push(0.0, source);

    while (!heap.empty()) {
        double currentDist = 0.0;
        int current = -1;
        if (!heap.popMin(currentDist, current)) {
            break;
        }
        // 跳过过期条目（惰性删除的清理步骤）。
        // 注意：这是性能优化，不是正确性必需——堆总先弹出更小的 key，
        // 故节点的正确条目必定先于其旧条目被处理，旧条目算出的候选值不可能更小。
        // 经变异测试验证：去掉本守卫，全部测试结果不变。
        if (currentDist > dist[static_cast<std::size_t>(current)]) {
            continue;
        }

        for (const Edge& edge : adjacency[static_cast<std::size_t>(current)]) {
            // 节点数很小（30 量级），线性查找下标足够；不为此引入额外的索引结构
            const int next = graph.indexOf(edge.toId);
            if (next < 0) {
                continue;
            }
            const double candidate = currentDist + weightOf(edge, weight);
            if (candidate < dist[static_cast<std::size_t>(next)]) {
                dist[static_cast<std::size_t>(next)] = candidate;
                previous[static_cast<std::size_t>(next)] = current;
                heap.push(candidate, next);
            }
        }
    }

    if (dist[static_cast<std::size_t>(target)] >= kInfinity) {
        return result;  // 不可达
    }

    // 由前驱数组回溯，再反转得到起点到终点的顺序
    std::vector<std::string> reversed;
    for (int at = target; at >= 0; at = previous[static_cast<std::size_t>(at)]) {
        reversed.push_back(graph.nodes()[static_cast<std::size_t>(at)].id);
    }
    for (std::size_t i = reversed.size(); i > 0; --i) {
        result.nodes.push_back(reversed[i - 1]);
    }

    result.found = true;
    result.totalWeight = dist[static_cast<std::size_t>(target)];
    return result;
}

} // namespace logistics
