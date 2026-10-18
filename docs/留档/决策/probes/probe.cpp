// 一次性评审探针（未入库）。编译：g++ -std=c++17 -I <repo>/src probe.cpp <core+io>.cpp
// 目的：
//  A. 正常规划下"每趟返程有没有盈余货"——检验提案前提。
//  B. 中转站是否落在"最后一站 -> 仓库"的最短路线上——检验"顺路零绕路"前提。
//  C. 当前设计下一次紧急订单的真实代价（车被迫回仓库取货）——给提案收益定上界。
#include <algorithm>
#include <cstdio>
#include <iomanip>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "core/Config.h"
#include "core/Dijkstra.h"
#include "core/LogisticsGraph.h"
#include "core/Order.h"
#include "core/RoutePlanner.h"
#include "core/Vehicle.h"
#include "core/WeightType.h"
#include "io/ConfigLoader.h"

using namespace logistics;

static const char* wname(WeightType w) {
    switch (w) {
        case WeightType::Distance: return "distance";
        case WeightType::Time:     return "time";
        case WeightType::Cost:     return "cost";
    }
    return "?";
}

static double wof(double d, double t, double c, WeightType w) {
    return pickWeight(d, t, c, w);
}

int main(int argc, char** argv) {
    const std::string path = (argc > 1) ? argv[1] : "config/default.ini";
    Config cfg;
    std::string err;
    if (!ConfigLoader::load(path, cfg, err)) {
        std::cerr << "加载失败: " << err << "\n";
        return 2;
    }
    const Vehicle& v = cfg.vehicles.front();
    const LogisticsGraph& g = cfg.graph;

    // 节点 -> 需求总量
    std::map<std::string, double> demandAt;
    for (const Order& o : cfg.orders) {
        demandAt[o.nodeId] += o.demandKg;
    }
    // 子网络 -> 中转站
    std::map<int, std::string> hubOf;
    for (const Node& n : g.nodes()) {
        if (n.type == NodeType::Transit && n.subNetworkId != 0) {
            hubOf[n.subNetworkId] = n.id;
        }
    }
    const WeightType W = WeightType::Distance;

    // ================= A. 盈余检验 + B. 顺路检验 =================
    std::cout << "===== A/B: 正常规划下每趟的载荷与回程是否顺路（权重=" << wname(W)
              << ", 载重上限=" << v.capacityKg << "kg）=====\n";
    for (WeightType w : {WeightType::Distance, WeightType::Time, WeightType::Cost}) {
        const RoutePlan p = planRoute(g, v, cfg.orders, w);
        if (p.status != PlanStatus::Ok) {
            std::cout << "[" << wname(w) << "] 规划失败: " << p.reason << "\n";
            continue;
        }
        std::cout << "\n-- 策略 " << wname(w) << ": 总距离 " << p.totalDistanceKm
                  << "km 趟数 " << p.trips.size() << "\n";
        double maxResidual = 0.0;
        for (std::size_t ti = 0; ti < p.trips.size(); ++ti) {
            const Trip& t = p.trips[ti];
            double delivered = 0.0;
            std::set<std::string> hubs;
            for (const Stop& s : t.stops) {
                delivered += demandAt[s.nodeId];
                const Node* n = g.findNode(s.nodeId);
                if (n != nullptr && hubOf.count(n->subNetworkId) != 0) {
                    hubs.insert(hubOf[n->subNetworkId]);
                }
            }
            const double residual = t.loadKg - delivered;
            maxResidual = std::max(maxResidual, residual);

            // 回程段：最后一站 -> 仓库，是否经过本簇中转站
            std::string last = t.nodes.empty() ? std::string() : t.nodes.front();
            for (std::size_t i = 0; i < t.nodes.size(); ++i) {
                if (i < t.nodeIsStop.size() && t.nodeIsStop[i]) {
                    last = t.nodes[i];
                }
            }
            std::string hubNote = "(无簇)";
            if (!hubs.empty()) {
                hubNote.clear();
                for (const std::string& h : hubs) {
                    const PathResult back = shortestPath(g, last, v.startNodeId, W);
                    bool onPath = false;
                    for (const std::string& id : back.nodes) {
                        if (id == h) {
                            onPath = true;
                        }
                    }
                    const PathResult viaHub = shortestPath(g, last, h, W);
                    const PathResult hubToW = shortestPath(g, h, v.startNodeId, W);
                    const double detour = viaHub.totalWeight + hubToW.totalWeight - back.totalWeight;
                    std::ostringstream os;
                    os << h << (onPath ? "[在回程最短路上]" : "[不在]")
                       << "绕路+" << std::fixed << std::setprecision(2) << detour << "km";
                    if (!hubNote.empty()) {
                        hubNote += " ";
                    }
                    hubNote += os.str();
                }
            }
            std::cout << "  第" << (ti + 1) << "趟 load=" << t.loadKg << "kg 实送=" << delivered
                      << "kg 盈余=" << residual << "kg 停靠" << t.stops.size()
                      << " 末站=" << last << " 回程顺路检查: " << hubNote << "\n";
        }
        std::cout << "  >> 本策略最大返程盈余 = " << maxResidual << " kg\n";
    }

    // ================= C. 紧急订单的真实代价 =================
    const RoutePlan p0 = planRoute(g, v, cfg.orders, W);
    std::cout << "\n===== C: 紧急订单代价（基准：08:00 出发的原始计划 " << p0.totalDistanceKm
              << "km / penalty " << p0.totalPenaltyMin << "）=====\n";
    const int times[] = {510, 540, 570, 600, 660};
    const char* targets[] = {"D08", "D05", "D20", "D10", "D25"};
    for (int T : times) {
        const std::size_t idx = nodeIndexAtTime(p0, T);
        const std::string pos = p0.nodes.empty() ? v.startNodeId : p0.nodes[idx];
        // 已送达 = 原始计划里到达时刻 <= T 的停靠点
        std::set<std::string> served;
        for (const Stop& s : p0.stops) {
            if (s.arrivalMin <= T) {
                served.insert(s.nodeId);
            }
        }
        std::vector<Order> remaining;
        for (const Order& o : cfg.orders) {
            if (served.count(o.nodeId) == 0) {
                remaining.push_back(o);
            }
        }
        if (remaining.empty()) {
            continue;
        }
        // 真实基线：GUI 会把"车上此刻载着的货"喂给重规划（VehicleState.onboard）。
        // 近似：当前趟内、到达时刻晚于 T 的停靠点，其货此刻还在车上。
        std::vector<OnboardItem> onboard;
        if (idx < p0.nodeTripIndex.size()) {
            const std::size_t curTrip = p0.nodeTripIndex[idx];
            for (std::size_t si = 0; si < p0.stops.size(); ++si) {
                if (p0.stops[si].arrivalMin > T) {
                    bool sameTrip = false;
                    for (std::size_t k = 0; k < p0.nodes.size(); ++k) {
                        if (p0.nodes[k] == p0.stops[si].nodeId && p0.nodeIsStop[k]
                            && p0.nodeTripIndex[k] == curTrip) {
                            sameTrip = true;
                        }
                    }
                    if (sameTrip) {
                        OnboardItem it;
                        it.nodeId = p0.stops[si].nodeId;
                        it.kg = demandAt[p0.stops[si].nodeId];
                        onboard.push_back(it);
                    }
                }
            }
        }
        const RoutePlan base = replan(g, v, remaining, pos, T, W, onboard);
        const PathResult posToW = shortestPath(g, pos, v.startNodeId, W);
        std::cout << "\n时刻 " << T / 60 << ":" << std::setw(2) << std::setfill('0') << (T % 60)
                  << std::setfill(' ') << " 车在 " << pos << "（距仓库 " << posToW.totalWeight
                  << "km）在途货 " << onboard.size() << " 项；剩余单 " << remaining.size() << " 张；不插单基线 "
                  << base.totalDistanceKm << "km/penalty " << base.totalPenaltyMin << "\n";
        for (const char* u : targets) {
            const Node* n = g.findNode(u);
            if (n == nullptr || served.count(u) != 0) {
                continue;
            }
            Order uo;
            uo.id = "U9";
            uo.nodeId = u;
            uo.demandKg = 6.0;
            uo.windowStartMin = T;
            uo.windowEndMin = T + 60;
            uo.urgent = true;
            const InsertResult r = insertUrgentOrder(g, v, remaining, uo, pos, T, W, onboard);
            const int hubSub = n->subNetworkId;
            std::string hub = hubOf.count(hubSub) ? hubOf[hubSub] : std::string();
            double ceiling = 0.0;
            std::string detail = "(无簇)";
            if (!hub.empty()) {
                const double viaW = posToW.totalWeight
                                    + shortestPath(g, v.startNodeId, u, W).totalWeight;
                const double viaH = shortestPath(g, pos, hub, W).totalWeight
                                    + shortestPath(g, hub, u, W).totalWeight;
                ceiling = viaW - viaH;
                std::ostringstream os;
                os << "经仓库 " << viaW << "km vs 经站(" << hub << ") " << viaH
                   << "km => 上界省 " << ceiling << "km";
                detail = os.str();
            }
            std::cout << "   紧急单 ->" << u << " 插单后 " << r.plan.totalDistanceKm
                      << "km/penalty " << r.plan.totalPenaltyMin << " (Δ距离 "
                      << (r.plan.totalDistanceKm - base.totalDistanceKm) << ", Δpenalty "
                      << (r.plan.totalPenaltyMin - base.totalPenaltyMin) << ") " << detail
                      << "\n";
        }
    }
    return 0;
}
