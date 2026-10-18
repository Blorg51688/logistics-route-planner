// 第四轮探针：用户方案（不借货：只用"车上盈余" + "站内存货"）的两种读法 + 直接实例化其例子。
//  F1: "盈余"的货源读法 — (读法1) 只能拿同簇后续趟尚未上车的货 / (读法2) 仓库可发"不记名缓冲货"
//      -> 决定站里最终能攒多少 kg
//  F2: 用户例子("车上盈余 + 站内库存 >= 紧急单需求 -> 顺路去站补足 -> 就地送达")的距离收益
//      对比现状(独立紧急趟、强制回仓库)
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
static const double EPS = 1e-6;

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
    const WeightType W = WeightType::Distance;

    std::map<std::string, double> demandAt;
    for (const Order& o : cfg.orders) {
        demandAt[o.nodeId] += o.demandKg;
    }
    std::map<int, std::string> hubOf;
    for (const Node& n : g.nodes()) {
        if (n.type == NodeType::Transit && n.subNetworkId != 0) {
            hubOf[n.subNetworkId] = n.id;
        }
    }

    // ============ F1：两种货源读法下，站里能攒多少 ============
    std::cout << "===== F1: \"满载出仓的盈余\"到底能攒多少？（默认数据、无紧急单）=====\n";
    std::cout << "  读法1 = 盈余只能是'本簇后续趟尚未上车的货'（模型里唯一存在的货）\n";
    std::cout << "  读法2 = 仓库可发出'不属于任何订单的缓冲货'（= 不记名库存；当时历史实现的口径）\n";
    for (WeightType w : {WeightType::Distance, WeightType::Time, WeightType::Cost}) {
        const RoutePlan p = planRoute(g, v, cfg.orders, w);
        double bank1 = 0.0, bank2 = 0.0;
        std::map<std::string, double> s1, s2;
        for (std::size_t ti = 0; ti < p.trips.size(); ++ti) {
            const Trip& t = p.trips[ti];
            const double spare = v.capacityKg - t.loadKg;
            if (spare <= EPS) {
                continue;
            }
            std::set<int> subs;
            for (const Stop& s : t.stops) {
                const Node* n = g.findNode(s.nodeId);
                if (n != nullptr && hubOf.count(n->subNetworkId) != 0) {
                    subs.insert(n->subNetworkId);
                }
            }
            if (subs.size() != 1) {
                continue;
            }
            const int sub = *subs.begin();
            const std::string hub = hubOf[sub];
            std::string last = t.nodes.empty() ? std::string() : t.nodes.front();
            for (std::size_t i = 0; i < t.nodes.size(); ++i) {
                if (i < t.nodeIsStop.size() && t.nodeIsStop[i]) {
                    last = t.nodes[i];
                }
            }
            const PathResult back = shortestPath(g, last, v.startNodeId, W);
            const PathResult lh = shortestPath(g, last, hub, W);
            const PathResult hw = shortestPath(g, hub, v.startNodeId, W);
            if (lh.totalWeight + hw.totalWeight - back.totalWeight > EPS) {
                continue;   // 回程不顺路 -> 按用户规则不寄存
            }
            double later = 0.0;
            for (std::size_t tj = ti + 1; tj < p.trips.size(); ++tj) {
                for (const Stop& s : p.trips[tj].stops) {
                    const Node* n = g.findNode(s.nodeId);
                    if (n != nullptr && n->subNetworkId == sub) {
                        later += demandAt[s.nodeId];
                    }
                }
            }
            bank1 += std::min(spare, later);
            s1[hub] += std::min(spare, later);
            bank2 += spare;
            s2[hub] += spare;
        }
        std::cout << "\n  [" << (w == WeightType::Distance ? "distance"
                              : w == WeightType::Time ? "time" : "cost")
                  << "] 读法1 站内总量 = " << bank1 << "kg  明细:";
        for (const auto& kv : s1) {
            std::cout << " " << kv.first << "=" << kv.second;
        }
        std::cout << "\n          读法2 站内总量 = " << bank2 << "kg  明细:";
        for (const auto& kv : s2) {
            std::cout << " " << kv.first << "=" << kv.second;
        }
        std::cout << "\n";
    }

    // ============ F2：用户例子的收益（簇1 目标，站点 T01） ============
    const RoutePlan p0 = planRoute(g, v, cfg.orders, W);
    std::cout << "\n===== F2: 用户例子——'车上盈余 + T01 库存'够用时，就地送达能省多少 =====\n";
    std::cout << "  现状 = 独立紧急趟（强制 车->仓库->紧急点->仓库）；站方案 = 把 (T01,紧急点) 插进在途趟\n";
    const int times[] = {510, 540, 570, 600, 660};
    const char* c1[] = {"D08", "D10", "D01", "D11", "D03"};
    double bestSave = 0.0;
    std::string bestDesc;
    for (int T : times) {
        const std::size_t idx = nodeIndexAtTime(p0, T);
        const std::string pos = p0.nodes.empty() ? v.startNodeId : p0.nodes[idx];
        if (idx >= p0.nodeTripIndex.size()) {
            continue;
        }
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
        const std::size_t curTrip = p0.nodeTripIndex[idx];
        std::vector<OnboardItem> onboard;
        for (std::size_t k = 0; k < p0.nodes.size(); ++k) {
            if (p0.nodeIsStop[k] && p0.nodeTripIndex[k] == curTrip && p0.nodeArrivalMin[k] > T) {
                OnboardItem it;
                it.nodeId = p0.nodes[k];
                it.kg = demandAt[p0.nodes[k]];
                onboard.push_back(it);
            }
        }
        const RoutePlan base = replan(g, v, remaining, pos, T, W, onboard);
        if (base.trips.empty()) {
            continue;
        }
        const Trip& t0 = base.trips.front();
        for (const char* u : c1) {
            if (served.count(u) != 0) {
                continue;
            }
            Order uo;
            uo.id = "U9";
            uo.nodeId = u;
            uo.demandKg = 6.0;
            uo.windowStartMin = T;
            uo.windowEndMin = T + 60;
            uo.urgent = true;
            const double nowCost =
                insertUrgentOrder(g, v, remaining, uo, pos, T, W, onboard).plan.totalDistanceKm
                - base.totalDistanceKm;
            // 站方案：把 (T01, U) 依次插进当前趟最便宜的相邻边之间
            double best = 1e9;
            for (std::size_t i = 0; i + 1 < t0.nodes.size(); ++i) {
                const PathResult a1 = shortestPath(g, t0.nodes[i], "T01", W);
                const PathResult a2 = shortestPath(g, "T01", u, W);
                const PathResult a3 = shortestPath(g, u, t0.nodes[i + 1], W);
                const PathResult a0 = shortestPath(g, t0.nodes[i], t0.nodes[i + 1], W);
                if (a1.found && a2.found && a3.found && a0.found) {
                    best = std::min(best,
                                    a1.totalWeight + a2.totalWeight + a3.totalWeight
                                        - a0.totalWeight);
                }
            }
            // 若当前趟本来就经过 T01，则只插 U 即可
            for (std::size_t i = 0; i + 1 < t0.nodes.size(); ++i) {
                if (t0.nodes[i] == "T01") {
                    const PathResult a2 = shortestPath(g, "T01", u, W);
                    const PathResult a3 = shortestPath(g, u, t0.nodes[i + 1], W);
                    const PathResult a0 = shortestPath(g, "T01", t0.nodes[i + 1], W);
                    if (a2.found && a3.found && a0.found) {
                        best = std::min(best, a2.totalWeight + a3.totalWeight - a0.totalWeight);
                    }
                }
            }
            if (best > 1e8) {
                continue;
            }
            const double save = nowCost - best;
            std::cout << "  时刻 " << T / 60 << ":" << std::setw(2) << std::setfill('0') << (T % 60)
                      << std::setfill(' ') << " 车在 " << pos << " 紧急单->" << u << ": 现状 +"
                      << std::fixed << std::setprecision(1) << nowCost << "km, 站方案 +" << best
                      << "km => 省 " << save << "km\n";
            if (save > bestSave) {
                bestSave = save;
                std::ostringstream os;
                os << T / 60 << ":" << (T % 60) << " 车在 " << pos << " ->" << u;
                bestDesc = os.str();
            }
        }
    }
    std::cout << "  >> 最大可省 " << bestSave << "km（" << bestDesc << "）\n";
    return 0;
}
