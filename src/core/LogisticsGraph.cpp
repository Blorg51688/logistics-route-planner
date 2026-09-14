#include "core/LogisticsGraph.h"

#include <iomanip>
#include <sstream>

namespace logistics {

namespace {
// 查询未命中时的返回值，避免调用方拿到悬空引用
const std::vector<Edge> kNoEdges;
} // namespace

int LogisticsGraph::indexOf(const std::string& id) const {
    for (std::size_t i = 0; i < nodes_.size(); ++i) {
        if (nodes_[i].id == id) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

bool LogisticsGraph::addNode(const Node& node) {
    if (node.id.empty() || indexOf(node.id) >= 0) {
        return false;
    }
    nodes_.push_back(node);
    out_.emplace_back();
    return true;
}

bool LogisticsGraph::removeNode(const std::string& id) {
    const int idx = indexOf(id);
    if (idx < 0) {
        return false;
    }
    const std::size_t pos = static_cast<std::size_t>(idx);

    // nodes_ 与 out_ 下标对齐，必须按同一位置同步删除
    nodes_.erase(nodes_.begin() + static_cast<std::ptrdiff_t>(pos));
    out_.erase(out_.begin() + static_cast<std::ptrdiff_t>(pos));

    // 有向图中指向该节点的入边散落在其他节点的出边表里，需全部清除
    for (std::vector<Edge>& list : out_) {
        for (std::size_t i = 0; i < list.size();) {
            if (list[i].toId == id) {
                list.erase(list.begin() + static_cast<std::ptrdiff_t>(i));
            } else {
                ++i;
            }
        }
    }
    return true;
}

bool LogisticsGraph::addEdge(const Edge& edge) {
    const int from = indexOf(edge.fromId);
    if (from < 0 || indexOf(edge.toId) < 0) {
        return false;  // 端点必须存在
    }
    if (findEdge(edge.fromId, edge.toId) != nullptr) {
        return false;  // (from, to) 至多一条边
    }

    Edge stored = edge;
    if (stored.baseTimeMin <= 0.0) {
        stored.baseTimeMin = stored.timeMin;  // 未指定初始耗时时按当前值固化
    }
    out_[static_cast<std::size_t>(from)].push_back(stored);
    return true;
}

bool LogisticsGraph::removeEdge(const std::string& fromId, const std::string& toId) {
    const int from = indexOf(fromId);
    if (from < 0) {
        return false;
    }
    std::vector<Edge>& list = out_[static_cast<std::size_t>(from)];
    for (std::size_t i = 0; i < list.size(); ++i) {
        if (list[i].toId == toId) {
            list.erase(list.begin() + static_cast<std::ptrdiff_t>(i));
            return true;
        }
    }
    return false;
}

bool LogisticsGraph::updateEdgeTime(const std::string& fromId, const std::string& toId,
                                    double newTimeMin) {
    const int from = indexOf(fromId);
    if (from < 0) {
        return false;
    }
    for (Edge& e : out_[static_cast<std::size_t>(from)]) {
        if (e.toId == toId) {
            e.timeMin = newTimeMin;
            e.congested = newTimeMin > e.baseTimeMin;
            return true;
        }
    }
    return false;
}

const Node* LogisticsGraph::findNode(const std::string& id) const {    const int idx = indexOf(id);
    return idx < 0 ? nullptr : &nodes_[static_cast<std::size_t>(idx)];
}

const Edge* LogisticsGraph::findEdge(const std::string& fromId, const std::string& toId) const {
    const int from = indexOf(fromId);
    if (from < 0) {
        return nullptr;
    }
    for (const Edge& e : out_[static_cast<std::size_t>(from)]) {
        if (e.toId == toId) {
            return &e;
        }
    }
    return nullptr;
}

const std::vector<Node>& LogisticsGraph::nodes() const {
    return nodes_;
}

const std::vector<Edge>& LogisticsGraph::outEdges(const std::string& id) const {
    const int idx = indexOf(id);
    return idx < 0 ? kNoEdges : out_[static_cast<std::size_t>(idx)];
}

std::vector<Edge> LogisticsGraph::edges() const {
    std::vector<Edge> all;
    for (const std::vector<Edge>& list : out_) {
        for (const Edge& e : list) {
            all.push_back(e);
        }
    }
    return all;
}

std::size_t LogisticsGraph::nodeCount() const {
    return nodes_.size();
}

std::size_t LogisticsGraph::edgeCount() const {
    std::size_t total = 0;
    for (const std::vector<Edge>& list : out_) {
        total += list.size();
    }
    return total;
}

std::string LogisticsGraph::toAdjacencyMatrixString(WeightType weight) const {
    const int kWidth = 10;
    std::ostringstream os;

    os << std::setw(kWidth) << "ID";
    for (const Node& col : nodes_) {
        os << std::setw(kWidth) << col.id;
    }
    os << '\n';

    for (const Node& row : nodes_) {
        os << std::setw(kWidth) << row.id;
        for (const Node& col : nodes_) {
            const Edge* edge = findEdge(row.id, col.id);
            if (edge == nullptr) {
                os << std::setw(kWidth) << "-";   // 缺边占位；有向图的对角线同样是 "-"
            } else {
                os << std::setw(kWidth)
                   << pickWeight(edge->distanceKm, edge->timeMin, edge->costYuan, weight);
            }
        }
        os << '\n';
    }
    return os.str();
}

std::string LogisticsGraph::toAdjacencyListString() const {
    std::ostringstream os;
    os << "邻接表（节点 " << nodeCount() << " 个，边 " << edgeCount() << " 条）\n";
    for (const Node& n : nodes_) {
        os << n.id << '(' << toString(n.type) << ',' << n.name << "):";
        const std::vector<Edge>& list = outEdges(n.id);
        if (list.empty()) {
            os << " -";
        } else {
            for (const Edge& e : list) {
                os << " -> " << e.toId << '(' << e.distanceKm << "km,"
                   << e.timeMin << "min," << e.costYuan << "元"
                   << (e.congested ? ",拥堵" : "") << ')';
            }
        }
        os << '\n';
    }
    return os.str();
}

} // namespace logistics
