#include "core/RoutePlanner.h"

#include <cmath>
#include <cstddef>

namespace logistics {

namespace {

// 一个待服务的停靠点候选
struct Candidate {
    std::string nodeId;
    double      demandKg = 0.0;
    int         windowStartMin = 0;
    int         windowEndMin = 0;
    bool        urgent = false;
};

// 把订单列表折叠成停靠点候选：
// 同一配送点上的多个订单合并为一次停靠——需求量求和，窗口取并集
// [min 窗口起, max 窗口止]，只要有一个订单紧急则该停靠点按紧急处理。
std::vector<Candidate> buildCandidates(const std::vector<Order>& orders) {
    std::vector<Candidate> candidates;
    for (const Order& o : orders) {
        bool merged = false;
        for (Candidate& c : candidates) {
            if (c.nodeId != o.nodeId) {
                continue;
            }
            c.demandKg += o.demandKg;
            if (o.windowStartMin < c.windowStartMin) {
                c.windowStartMin = o.windowStartMin;
            }
            if (o.windowEndMin > c.windowEndMin) {
                c.windowEndMin = o.windowEndMin;
            }
            c.urgent = c.urgent || o.urgent;
            merged = true;
            break;
        }
        if (merged) {
            continue;
        }
        Candidate c;
        c.nodeId = o.nodeId;
        c.demandKg = o.demandKg;
        c.windowStartMin = o.windowStartMin;
        c.windowEndMin = o.windowEndMin;
        c.urgent = o.urgent;
        candidates.push_back(c);
    }
    return candidates;
}

// 沿一条最短路推进时间并累加三个维度的权重，
// 把途经节点追加到 plan.nodes（不含路径起点，起点已在前一步入列）。
// 时间推进始终使用耗时维度，与本次规划所选策略无关。
void walkPath(const LogisticsGraph& graph,
              const std::vector<std::string>& path,
              RoutePlan& plan,
              double& elapsedMin) {
    for (std::size_t i = 1; i < path.size(); ++i) {
        const Edge* edge = graph.findEdge(path[i - 1], path[i]);
        if (edge == nullptr) {
            continue;
        }
        elapsedMin += edge->timeMin;
        plan.totalDistanceKm += edge->distanceKm;
        plan.totalCostYuan += edge->costYuan;
        plan.nodes.push_back(path[i]);
    }
}

} // namespace

RoutePlan replan(const LogisticsGraph& graph,
                 const Vehicle& vehicle,
                 const std::vector<Order>& remainingOrders,
                 const std::string& currentPositionId,
                 int currentTimeMin,
                 double serviceTimeMin,
                 WeightType weight) {
    RoutePlan plan;

    if (graph.findNode(currentPositionId) == nullptr) {
        plan.status = PlanStatus::Unreachable;
        plan.reason = "出发位置不存在: " + currentPositionId;
        return plan;
    }

    std::vector<Candidate> remaining = buildCandidates(remainingOrders);

    double loadKg = 0.0;
    for (const Candidate& c : remaining) {
        loadKg += c.demandKg;
    }

    // D14：单趟在仓库一次性装载全部待配送货物，因此容量约束等价于
    // 「总需求 <= 载重上限」，在规划前一次性判定；超限时不生成任何路线。
    if (loadKg > vehicle.capacityKg) {
        plan.status = PlanStatus::OverCapacity;
        plan.reason = "总需求 " + std::to_string(loadKg) + "kg 超过载重上限 "
                      + std::to_string(vehicle.capacityKg) + "kg";
        return plan;
    }

    std::string current = currentPositionId;
    double elapsedMin = static_cast<double>(currentTimeMin);
    plan.nodes.push_back(current);

    while (!remaining.empty()) {
        // 紧急订单优先：只要还有未服务的紧急订单，候选集就收缩到紧急订单之内。
        // 该判断必须在贪心选择内部逐轮进行，而不是对订单事后排序。
        bool hasUrgent = false;
        for (const Candidate& c : remaining) {
            if (c.urgent) {
                hasUrgent = true;
                break;
            }
        }

        // 贪心：选"当前位置到它的最短路权重"最小的未服务停靠点
        int bestIndex = -1;
        double bestWeight = 0.0;
        PathResult bestPath;

        std::vector<std::string> unreachable;
        for (std::size_t i = 0; i < remaining.size(); ++i) {
            if (hasUrgent && !remaining[i].urgent) {
                continue;   // 还有紧急订单时，非紧急订单一律不参选
            }
            const PathResult path = shortestPath(graph, current, remaining[i].nodeId, weight);
            if (!path.found) {
                unreachable.push_back(remaining[i].nodeId);
                continue;
            }
            // 权重并列时按节点 ID 字典序取，保证同一输入必得同一条路线（输出确定）
            const bool tieBreak =
                bestIndex >= 0 && path.totalWeight == bestWeight
                && remaining[i].nodeId
                       < remaining[static_cast<std::size_t>(bestIndex)].nodeId;
            if (bestIndex < 0 || path.totalWeight < bestWeight || tieBreak) {
                bestIndex = static_cast<int>(i);
                bestWeight = path.totalWeight;
                bestPath = path;
            }
        }

        if (bestIndex < 0) {
            std::string names;
            for (const std::string& id : unreachable) {
                if (!names.empty()) {
                    names += ", ";
                }
                names += id;
            }
            plan.status = PlanStatus::Unreachable;
            plan.reason = "无法从 " + current + " 到达配送点: " + names;
            return plan;
        }

        walkPath(graph, bestPath.nodes, plan, elapsedMin);

        const Candidate& chosen = remaining[static_cast<std::size_t>(bestIndex)];

        Stop stop;
        stop.nodeId = chosen.nodeId;

        const double rawArrival = elapsedMin;
        const double wait = (static_cast<double>(chosen.windowStartMin) > rawArrival)
                                ? (static_cast<double>(chosen.windowStartMin) - rawArrival)
                                : 0.0;
        const double arrival = rawArrival + wait;

        stop.rawArrivalMin = static_cast<int>(std::lround(rawArrival));
        stop.waitMin = static_cast<int>(std::lround(wait));
        stop.arrivalMin = static_cast<int>(std::lround(arrival));
        stop.late = arrival > static_cast<double>(chosen.windowEndMin);
        stop.penaltyMin = stop.late
                              ? static_cast<int>(std::lround(arrival - static_cast<double>(chosen.windowEndMin)))
                              : 0;
        plan.totalPenaltyMin += stop.penaltyMin;

        elapsedMin = arrival + serviceTimeMin;
        stop.departureMin = static_cast<int>(std::lround(elapsedMin));

        loadKg -= chosen.demandKg;
        stop.remainingLoadKg = loadKg;
        plan.stops.push_back(stop);

        current = chosen.nodeId;
        remaining.erase(remaining.begin() + static_cast<std::ptrdiff_t>(bestIndex));
    }

    // 服务完所有停靠点后返回车辆起始仓库
    const PathResult back = shortestPath(graph, current, vehicle.startNodeId, weight);
    if (!back.found) {
        plan.status = PlanStatus::Unreachable;
        plan.reason = "无法从 " + current + " 返回仓库 " + vehicle.startNodeId;
        return plan;
    }
    walkPath(graph, back.nodes, plan, elapsedMin);

    plan.totalTimeMin = elapsedMin - static_cast<double>(currentTimeMin);
    return plan;
}

bool isEdgeOnRoute(const std::vector<std::string>& routeNodes,
                   const std::string& fromId,
                   const std::string& toId) {
    for (std::size_t i = 1; i < routeNodes.size(); ++i) {
        if (routeNodes[i - 1] == fromId && routeNodes[i] == toId) {
            return true;
        }
    }
    return false;
}

RoutePlan planRoute(const LogisticsGraph& graph,
                    const Vehicle& vehicle,
                    const std::vector<Order>& orders,
                    double serviceTimeMin,
                    WeightType weight) {
    return replan(graph, vehicle, orders, vehicle.startNodeId,
                  vehicle.departTimeMin, serviceTimeMin, weight);
}

InsertResult insertUrgentOrder(const LogisticsGraph& graph,
                               const Vehicle& vehicle,
                               const std::vector<Order>& remainingOrders,
                               const Order& newOrder,
                               const std::string& currentPositionId,
                               int currentTimeMin,
                               double serviceTimeMin,
                               WeightType weight) {
    InsertResult result;

    // 插入的订单一律按紧急处理，调用方传入的 urgent 不作数
    Order inserted = newOrder;
    inserted.urgent = true;

    std::vector<Order> all = remainingOrders;
    all.push_back(inserted);

    // 冲突判定：最早到达时刻始终按耗时维度衡量，与本次规划所选策略无关
    const PathResult toNew =
        shortestPath(graph, currentPositionId, inserted.nodeId, WeightType::Time);
    if (!toNew.found) {
        result.warning = "紧急订单目标不可达: " + inserted.nodeId;
    } else {
        const int earliestArrival =
            currentTimeMin + static_cast<int>(std::lround(toNew.totalWeight));
        if (inserted.windowEndMin < earliestArrival) {
            result.warning = "紧急订单 " + inserted.id + " 无法在窗口内送达（窗口止 "
                             + std::to_string(inserted.windowEndMin) + "，最早到达 "
                             + std::to_string(earliestArrival) + "）";
        }
    }

    result.plan = replan(graph, vehicle, all, currentPositionId, currentTimeMin,
                         serviceTimeMin, weight);
    return result;
}

} // namespace logistics
