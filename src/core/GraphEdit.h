#pragma once

#include <string>

#include "core/LogisticsGraph.h"
#include "core/Random.h"

namespace logistics {

// B5 动态增删：模拟新客户 / 道路封闭，以及界面手工加边时的权重估算。
//
// 坐标到权重的比例**从图自身的既有边推算**（距离/耗时/成本与坐标差的平均比值），
// 而不是在核心里硬编码「坐标单位 → 公里」的换算，避免与数据集的尺度脱钩。

// 在图中随机位置插入一个配送点，并与**最近的既有节点**双向连通。
// 返回新节点 ID；图为空时返回空串。
std::string addRandomCustomer(LogisticsGraph& graph, Rng& rng);

// 随机封闭一条道路：删除一条有向边；若其反向边也存在则一并删除（道路封闭是双向的）。
// 返回被封闭道路的描述 "from->to"；无边可删时返回空串。
std::string closeRandomRoad(LogisticsGraph& graph, Rng& rng);

// 为给定两点构造一条权重与图中既有边口径一致的边（供界面手工加边/模拟使用）。
// 任一端点不存在，或两点坐标重合导致距离为 0 时返回 false。
bool makeSyntheticEdge(const LogisticsGraph& graph, const std::string& fromId,
                       const std::string& toId, Edge& out);

} // namespace logistics
