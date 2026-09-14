#pragma once

#include <string>
#include <vector>

#include "core/LogisticsGraph.h"
#include "core/WeightType.h"

namespace logistics {

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
