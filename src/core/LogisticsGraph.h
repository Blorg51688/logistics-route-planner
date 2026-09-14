#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "core/Edge.h"
#include "core/Node.h"

namespace logistics {

// 有向带权配送网络，邻接表存储。
//
// 存储方式：nodes_ 与 out_ 下标严格对齐，out_[i] 是 nodes_[i] 的全部出边。
// 因为有向图的入边分散在其他节点的出边表中，删除节点时必须扫描全部邻接表。
class LogisticsGraph {
public:
    // 节点：ID 为空或已存在时返回 false
    bool addNode(const Node& node);
    // 删除节点；同时删除其全部出边与入边。节点不存在时返回 false
    bool removeNode(const std::string& id);

    // 边：端点不存在或 (from, to) 已存在时返回 false
    bool addEdge(const Edge& edge);
    bool removeEdge(const std::string& fromId, const std::string& toId);

    // 只读查询；未命中返回 nullptr / 空表
    const Node* findNode(const std::string& id) const;
    const Edge* findEdge(const std::string& fromId, const std::string& toId) const;

    const std::vector<Node>& nodes() const;
    const std::vector<Edge>& outEdges(const std::string& id) const;
    // 全部边的拍平副本，便于遍历与输出
    std::vector<Edge> edges() const;

    std::size_t nodeCount() const;
    std::size_t edgeCount() const;

    // 邻接表文本表示，用于调试与课程要求的图表示输出
    std::string toAdjacencyListString() const;

    // 邻接表本体：下标与 nodes() 对齐，供图算法按索引高效遍历
    const std::vector<std::vector<Edge>>& adjacency() const { return out_; }
    // 按 ID 取节点下标；未命中返回 -1
    int indexOf(const std::string& id) const;

private:
    std::vector<Node>              nodes_;
    std::vector<std::vector<Edge>> out_;
};

} // namespace logistics
