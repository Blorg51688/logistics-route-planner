#pragma once

#include <cmath>
#include <string>

#include "core/LogisticsGraph.h"
#include "core/Order.h"
#include "core/Vehicle.h"

// 多个测试共用的图构造夹具，避免 Node / Edge 的构造样板在各测试文件里重复。
namespace fixtures {

inline bool nearlyEqual(double a, double b, double tolerance = 1e-9) {
    return std::fabs(a - b) < tolerance;
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

inline logistics::Vehicle makeVehicle(const std::string& startId, double capacityKg,
                                     int departMin) {
    logistics::Vehicle v;
    v.id = "V01";
    v.startNodeId = startId;
    v.capacityKg = capacityKg;
    v.departTimeMin = departMin;
    return v;
}

// 订单夹具：全部订单统一 A→B 取送（规格一 #23）。pickup 必填，调用方需显式给出取货点。
// 注：本票（#32）只把签名与调用点迁到七列口径，各用例的期望值仍按旧语义书写，
// 重算归 #33（构造器重写）——因此 routeplanner_tests 暂时允许红。
inline logistics::Order makeOrder(const std::string& id, const std::string& pickupNodeId,
                                 const std::string& nodeId, double demandKg,
                                 int windowStartMin, int windowEndMin,
                                 bool urgent = false) {
    logistics::Order o;
    o.id = id;
    o.pickupNodeId = pickupNodeId;
    o.nodeId = nodeId;
    o.demandKg = demandKg;
    o.windowStartMin = windowStartMin;
    o.windowEndMin = windowEndMin;
    o.urgent = urgent;
    return o;
}

} // namespace fixtures
