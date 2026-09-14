#pragma once

#include <cmath>
#include <string>

#include "core/LogisticsGraph.h"

// 多个测试共用的图构造夹具，避免 Node / Edge 的构造样板在各测试文件里重复。
namespace fixtures {

inline bool nearlyEqual(double a, double b) {
    return std::fabs(a - b) < 1e-9;
}

inline logistics::Node makeNode(const std::string& id, logistics::NodeType type,
                               const std::string& name) {
    logistics::Node n;
    n.id = id;
    n.type = type;
    n.name = name;
    return n;
}

inline logistics::Edge makeEdge(const std::string& from, const std::string& to,
                               double km, double minutes, double yuan) {
    logistics::Edge e;
    e.fromId = from;
    e.toId = to;
    e.distanceKm = km;
    e.timeMin = minutes;
    e.costYuan = yuan;
    return e;
}

// 双向边：真实道路通常两个方向都通行，且能让"返回仓库"必然可达
inline void addTwoWay(logistics::LogisticsGraph& g, const std::string& a, const std::string& b,
                      double km, double minutes, double yuan) {
    g.addEdge(makeEdge(a, b, km, minutes, yuan));
    g.addEdge(makeEdge(b, a, km, minutes, yuan));
}

} // namespace fixtures
