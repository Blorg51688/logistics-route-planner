// 第二轮评审探针：先把用户的两个"零成本"前提当作成立（承认），再量**消费端**的代价。
//  D1：若后续趟改从"中转站"取货再送客户，相对"从仓库直接送"多跑多少 km（逐客户）。
//  D2：生产端（满载出仓 + 回程顺路寄存）在默认数据上会攒出多少 kg 库存、有没有消费者。
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
    std::map<std::string, int> subOfHub;
    for (const Node& n : g.nodes()) {
        if (n.type == NodeType::Transit && n.subNetworkId != 0) {
            hubOf[n.subNetworkId] = n.id;
            subOfHub[n.id] = n.subNetworkId;
        }
    }

    // ================== D1：消费端代价 ==================
    std::cout << "===== D1: 后续趟改从站里取货 -> 送客户，相对从仓库直达客户，多跑多少 =====";
    std::cout << "\n(判据: (W->H) + (H->X) − (W->X)，单位 km；仓库 = " << v.startNodeId << ")\n";
    double worst = 0.0;
    std::string worstNode, worstHub;
    int worseCnt = 0, equalCnt = 0;
    std::map<int, std::pair<double, int>> perHub;   // sub -> (max detour, count worse)
    for (const Node& x : g.nodes()) {
        if (x.type != NodeType::Delivery) {
            continue;
        }
        const int sub = x.subNetworkId;
        if (hubOf.count(sub) == 0) {
            continue;
        }
        const std::string& h = hubOf[sub];
        const PathResult direct = shortestPath(g, v.startNodeId, x.id, W);
        const PathResult wh = shortestPath(g, v.startNodeId, h, W);
        const PathResult hx = shortestPath(g, h, x.id, W);
        if (!direct.found || !wh.found || !hx.found) {
            continue;
        }
        const double detour = wh.totalWeight + hx.totalWeight - direct.totalWeight;
        std::cout << "  " << x.id << " (簇 " << sub << ", 站点 " << h << "): 直达 "
                  << std::fixed << std::setprecision(2) << direct.totalWeight << "km, 经站 "
                  << (wh.totalWeight + hx.totalWeight) << "km, 绕路 +" << detour << "km\n";
        if (detour > EPS) {
            ++worseCnt;
            perHub[sub].first = std::max(perHub[sub].first, detour);
            perHub[sub].second += 1;
        } else {
            ++equalCnt;
        }
        if (detour > worst) {
            worst = detour;
            worstNode = x.id;
            worstHub = h;
        }
    }
    std::cout << "  >> 更差客户 " << worseCnt << " 个 / 不更差(等距) " << equalCnt
              << " 个；最差 " << worstNode << " 经 " << worstHub << " 多跑 +" << worst << "km\n";
    (void)perHub;

    // ================== D2：生产端会攒出多少库存 ==================
    std::cout << "\n===== D2: 满载出仓 + 回程顺路寄存（默认数据、无紧急订单）会攒出多少 kg =====";
    std::cout << "\n(空位 = 载重上限 − 本趟实际装载；可寄存量 = min(空位, 本簇后续趟尚未上车的货))\n";
    for (WeightType w : {WeightType::Distance, WeightType::Time, WeightType::Cost}) {
        const RoutePlan p = planRoute(g, v, cfg.orders, w);
        std::map<std::string, double> stock;
        double totalBanked = 0.0;
        for (std::size_t ti = 0; ti < p.trips.size(); ++ti) {
            const Trip& t = p.trips[ti];
            const double spare = v.capacityKg - t.loadKg;
            // 本趟服务的簇（取停靠点所属子网络）
            std::set<int> subs;
            for (const Stop& s : t.stops) {
                const Node* n = g.findNode(s.nodeId);
                if (n != nullptr && hubOf.count(n->subNetworkId) != 0) {
                    subs.insert(n->subNetworkId);
                }
            }
            if (subs.size() != 1 || spare <= EPS) {
                continue;
            }
            const int sub = *subs.begin();
            const std::string hub = hubOf[sub];
            // 回程是否经过该站且零绕路
            std::string last = t.nodes.empty() ? std::string() : t.nodes.front();
            for (std::size_t i = 0; i < t.nodes.size(); ++i) {
                if (i < t.nodeIsStop.size() && t.nodeIsStop[i]) {
                    last = t.nodes[i];
                }
            }
            const PathResult back = shortestPath(g, last, v.startNodeId, W);
            const PathResult lh = shortestPath(g, last, hub, W);
            const PathResult hw = shortestPath(g, hub, v.startNodeId, W);
            const double detour = lh.totalWeight + hw.totalWeight - back.totalWeight;
            if (detour > EPS) {
                continue;   // 不满足"顺路"，按用户规则原样带回仓库
            }
            // 本簇后续趟尚未上车的货（可用来填满本趟的货）
            double later = 0.0;
            for (std::size_t tj = ti + 1; tj < p.trips.size(); ++tj) {
                for (const Stop& s : p.trips[tj].stops) {
                    const Node* n = g.findNode(s.nodeId);
                    if (n != nullptr && n->subNetworkId == sub) {
                        later += demandAt[s.nodeId];
                    }
                }
            }
            const double banked = std::min(spare, later);
            if (banked > EPS) {
                stock[hub] += banked;
                totalBanked += banked;
                std::cout << "\n  [" << (w == WeightType::Distance ? "distance"
                                        : w == WeightType::Time ? "time" : "cost")
                          << "] 第" << (ti + 1) << "趟 空位 " << spare << "kg，本簇(簇" << sub
                          << ")后续未上车货 " << later << "kg -> 顺路寄存 " << banked << "kg 于 "
                          << hub;
            }
        }
        std::cout << "\n  ["
                  << (w == WeightType::Distance ? "distance"
                      : w == WeightType::Time ? "time" : "cost")
                  << "] 策略结束时站内累计库存 = " << totalBanked << " kg  明细:";
        for (const auto& kv : stock) {
            std::cout << " " << kv.first << "=" << kv.second << "kg";
        }
        std::cout << "\n  消费者：默认数据 25 张订单**全部不是紧急单**（"
                  << "config/default.ini [orders] is_urgent 全 0）"
                  << " -> 这些库存本轮无任何取用路径。\n";
    }
    return 0;
}
