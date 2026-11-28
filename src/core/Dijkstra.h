#pragma once

#include <functional>
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

// 边准入谓词（规格七 #63「慢车物理断路」）：最短路在**松弛处**过滤边，图本身不动。
//
// 为什么做成最短路的一个参数而不是复制一份 Dijkstra：全项目只有一份最短路实现
// （手写算法口径由 AGENTS §1 管住），「哪些边这台车能走」是**策略**，
// 由调用方以谓词给出；图与算法都不该知道车辆。空谓词 = 不过滤（既有调用逐字不变）。
using EdgeAllowed = std::function<bool(const std::string& fromId, const std::string& toId)>;

// 单源最短路（求到指定目标）。
// 起点或终点不存在、或目标不可达时，found == false 且 nodes 为空。
// 起点等于终点时返回只含起点的单元素路径，权重为 0。
PathResult shortestPath(const LogisticsGraph& graph,
                        const std::string& sourceId,
                        const std::string& targetId,
                        WeightType weight);

// 带边准入谓词的版本（allowEdge 为空等价于上一版）。被过滤掉的有向边视同不存在。
PathResult shortestPath(const LogisticsGraph& graph,
                        const std::string& sourceId,
                        const std::string& targetId,
                        WeightType weight,
                        const EdgeAllowed& allowEdge);

} // namespace logistics
