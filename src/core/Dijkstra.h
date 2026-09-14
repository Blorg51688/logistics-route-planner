#pragma once

#include <string>
#include <vector>

#include "core/LogisticsGraph.h"

namespace logistics {

// Dijkstra 可用的权重维度。
// 两种策略（最短距离 / 最低成本）共用同一份实现，只在这里切换权重来源，
// 不重复写三份最短路。
enum class WeightType {
    Distance,
    Time,
    Cost
};

struct PathResult {
    bool                     found = false;
    std::vector<std::string> nodes;        // 含起点与终点
    double                   totalWeight = 0.0;
};

// 单源最短路（求到指定目标）。
// 起点或终点不存在、或目标不可达时，found == false 且 nodes 为空。
// 起点等于终点时返回只含起点的单元素路径，权重为 0。
PathResult shortestPath(const LogisticsGraph& graph,
                        const std::string& sourceId,
                        const std::string& targetId,
                        WeightType weight);

} // namespace logistics
