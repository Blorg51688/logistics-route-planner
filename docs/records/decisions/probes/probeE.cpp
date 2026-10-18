// 第三轮评审探针：量用户"先看车上现成载货能否满足紧急单"这条（记作 M1）。
// M1 不需要中转站：车上有货 -> 直接去紧急单节点，代价 = 把被借货的客户推迟到后续趟。
// 本探针给 M1 的**乐观上界**（忽略被借客户的推迟代价）与"车上可用货量"的实测。
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
    const RoutePlan p0 = planRoute(g, v, cfg.orders, W);

    std::cout << "===== E: 紧急单三条处置路径的距离代价对比（权重 = distance）=====\n";
    std::cout << "  (1) 现状: 车 -> 仓库取货 -> 紧急单  (2) M1乐观上界: 车直接插到紧急单节点\n";
    const int times[] = {510, 540, 570, 600, 660};
    const char* targets[] = {"D08", "D05", "D20", "D10", "D25"};
    double maxSave = 0.0;
    std::string maxDesc;
    int cnt = 0, saveCnt = 0;
    for (int T : times) {
        const std::size_t idx = nodeIndexAtTime(p0, T);
        const std::string pos = p0.nodes.empty() ? v.startNodeId : p0.nodes[idx];
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
        if (remaining.empty() || idx >= p0.nodeTripIndex.size()) {
            continue;
        }
        const std::size_t curTrip = p0.nodeTripIndex[idx];
        // 车上现成的货：当前趟内尚未送达的停靠点
        std::vector<OnboardItem> onboard;
        for (std::size_t k = 0; k < p0.nodes.size(); ++k) {
            if (p0.nodeIsStop[k] && p0.nodeTripIndex[k] == curTrip
                && p0.nodeArrivalMin[k] > T) {
                OnboardItem it;
                it.nodeId = p0.nodes[k];
                it.kg = demandAt[p0.nodes[k]];
                onboard.push_back(it);
            }
        }
        double onboardKg = 0.0;
        double minItem = 1e9;
        for (const OnboardItem& it : onboard) {
            onboardKg += it.kg;
            minItem = std::min(minItem, it.kg);
        }
        const RoutePlan base = replan(g, v, remaining, pos, T, W, onboard);
        std::cout << "\n时刻 " << T / 60 << ":" << std::setw(2) << std::setfill('0') << (T % 60)
                  << std::setfill(' ') << " 车在 " << pos << " | 车上现成货 " << onboard.size()
                  << " 项 / 合计 " << onboardKg << "kg / 单项最小 "
                  << (onboard.empty() ? 0.0 : minItem) << "kg | 剩余单 " << remaining.size()
                  << " 张\n";
        // base 的第一趟序列（车正在跑的这一趟）
        static const Trip kEmptyTrip;
        const Trip& t0 = base.trips.empty() ? kEmptyTrip : base.trips.front();
        for (const char* u : targets) {
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
            const InsertResult r = insertUrgentOrder(g, v, remaining, uo, pos, T, W, onboard);
            const double urgCost = r.plan.totalDistanceKm - base.totalDistanceKm;
            // M1 乐观上界：把 U 插进"当前这趟"最便宜的相邻边之间
            double bestIns = 1e9;
            for (std::size_t i = 0; i + 1 < t0.nodes.size(); ++i) {
                if (t0.nodes[i] == u) {
                    bestIns = 0.0;
                    break;
                }
                const PathResult au = shortestPath(g, t0.nodes[i], u, W);
                const PathResult ub = shortestPath(g, u, t0.nodes[i + 1], W);
                const PathResult ab = shortestPath(g, t0.nodes[i], t0.nodes[i + 1], W);
                if (au.found && ub.found && ab.found) {
                    bestIns = std::min(bestIns, au.totalWeight + ub.totalWeight - ab.totalWeight);
                }
            }
            if (bestIns > 1e8) {
                continue;
            }
            ++cnt;
            const double save = urgCost - bestIns;
            if (save > 0) {
                ++saveCnt;
            }
            if (save > maxSave) {
                maxSave = save;
                std::ostringstream os;
                os << T / 60 << ":" << (T % 60) << " 车在 " << pos << " 紧急单->" << u;
                maxDesc = os.str();
            }
            std::cout << "    ->" << u << ": 现状代价 " << std::showpos << std::fixed
                      << std::setprecision(1) << urgCost << std::noshowpos
                      << "km | M1乐观上界代价 " << bestIns << "km | 上界可省 " << save << "km\n";
        }
    }
    std::cout << "\n  >> 共 " << cnt << " 组：M1乐观上界可省的 " << saveCnt
              << " 组；最大 " << maxSave << "km（" << maxDesc << "）\n";
    return 0;
}
