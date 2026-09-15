#include "core/RoutePlanner.h"

#include <cmath>
#include <sstream>
#include <cstddef>
#include <map>

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

// 开局一趟：把起点写入序列（appendLeg 只追加路径上的后续节点，不含起点）
void beginTrip(Trip& trip, const std::string& nodeId, double elapsedMin) {
    trip.nodes.push_back(nodeId);
    trip.nodeArrivalMin.push_back(static_cast<int>(std::lround(elapsedMin)));
    trip.nodeIsStop.push_back(false);
}

// 沿 from→to 的最短路走一段，追加到某一趟。
// 时间推进始终使用耗时维度，与本次规划所选策略无关。
// endpointIsStop=true 表示这段的终点是一次配送停靠（而非途经或返程）。
bool appendLeg(const LogisticsGraph& graph,
               Trip& trip,
               double& elapsedMin,
               const std::string& from,
               const std::string& to,
               WeightType weight,
               bool endpointIsStop,
               std::string& failReason) {
    if (from == to) {
        return true;
    }
    const PathResult path = shortestPath(graph, from, to, weight);
    if (!path.found) {
        failReason = "无法从 " + from + " 到达 " + to;
        return false;
    }
    for (std::size_t i = 1; i < path.nodes.size(); ++i) {
        const Edge* edge = graph.findEdge(path.nodes[i - 1], path.nodes[i]);
        if (edge == nullptr) {
            failReason = "路径中间边缺失: " + path.nodes[i - 1] + " -> " + path.nodes[i];
            return false;
        }
        elapsedMin += edge->timeMin;
        trip.totalDistanceKm += edge->distanceKm;
        trip.totalCostYuan += edge->costYuan;
        trip.nodes.push_back(path.nodes[i]);
        trip.nodeArrivalMin.push_back(static_cast<int>(std::lround(elapsedMin)));
        trip.nodeIsStop.push_back(endpointIsStop && (i + 1 == path.nodes.size()));
    }
    return true;
}

// 一趟内的贪心串联：从 startPos 出发，若给了 pickupNode 则先到那里装货，
// 然后在 pool 内反复选"当前位置到它的最短路权重最小"的未服务点送达，
// 最后走到 endNode。pool 以值传递，调用方保留自己的副本。
//
// 贪心规则与单趟版本完全一致（含紧急订单优先与节点 ID 字典序 tie-break），
// 因此单趟调用时结果与历史行为逐位相同。
bool weave(const LogisticsGraph& graph,
           std::vector<Candidate> pool,
           const std::string& startPos,
           double& elapsedMin,
           double serviceTimeMin,
           WeightType weight,
           const std::string& pickupNode,
           const std::string& endNode,
           double loadKg,
           Trip& trip,
           std::string& failReason) {
    std::string current = startPos;
    beginTrip(trip, current, elapsedMin);

    if (!pickupNode.empty() && current != pickupNode) {
        if (!appendLeg(graph, trip, elapsedMin, current, pickupNode, weight, false, failReason)) {
            return false;
        }
        current = pickupNode;
    }

    while (!pool.empty()) {
        // 紧急订单优先：只要还有未服务的紧急订单，候选集就收缩到紧急订单之内。
        // 该判断必须在贪心选择内部逐轮进行，而不是对订单事后排序。
        bool hasUrgent = false;
        for (const Candidate& c : pool) {
            if (c.urgent) {
                hasUrgent = true;
                break;
            }
        }

        int bestIndex = -1;
        double bestWeight = 0.0;
        std::vector<std::string> unreachable;
        for (std::size_t i = 0; i < pool.size(); ++i) {
            if (hasUrgent && !pool[i].urgent) {
                continue;   // 还有紧急订单时，非紧急订单一律不参选
            }
            const PathResult path = shortestPath(graph, current, pool[i].nodeId, weight);
            if (!path.found) {
                unreachable.push_back(pool[i].nodeId);
                continue;
            }
            // 权重并列时按节点 ID 字典序取，保证同一输入必得同一条路线（输出确定）
            const bool tieBreak =
                bestIndex >= 0 && path.totalWeight == bestWeight
                && pool[i].nodeId < pool[static_cast<std::size_t>(bestIndex)].nodeId;
            if (bestIndex < 0 || path.totalWeight < bestWeight || tieBreak) {
                bestIndex = static_cast<int>(i);
                bestWeight = path.totalWeight;
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
            failReason = "无法从 " + current + " 到达配送点: " + names;
            return false;
        }

        const Candidate chosen = pool[static_cast<std::size_t>(bestIndex)];
        if (!appendLeg(graph, trip, elapsedMin, current, chosen.nodeId, weight, true, failReason)) {
            return false;
        }

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

        // 停靠节点在序列中记录的应是**实际送达时刻**（等窗口开启之后的 arrival），
        // 而不是 appendLeg 记下的未经等待的原始到达时刻。
        trip.nodeArrivalMin[trip.nodeArrivalMin.size() - 1] = stop.arrivalMin;

        elapsedMin = arrival + serviceTimeMin;
        stop.departureMin = static_cast<int>(std::lround(elapsedMin));

        loadKg -= chosen.demandKg;
        stop.remainingLoadKg = loadKg;
        trip.stops.push_back(stop);

        current = chosen.nodeId;
        pool.erase(pool.begin() + static_cast<std::ptrdiff_t>(bestIndex));
    }

    if (current != endNode) {
        if (!appendLeg(graph, trip, elapsedMin, current, endNode, weight, false, failReason)) {
            return false;
        }
    }
    trip.endNodeId = endNode;
    return true;
}

void eraseCandidateByNode(std::vector<Candidate>& pool, const std::string& nodeId) {
    for (std::size_t i = 0; i < pool.size(); ++i) {
        if (pool[i].nodeId == nodeId) {
            pool.erase(pool.begin() + static_cast<std::ptrdiff_t>(i));
            return;
        }
    }
}

double totalDemand(const std::vector<Candidate>& pool) {
    double sum = 0.0;
    for (const Candidate& c : pool) {
        sum += c.demandKg;
    }
    return sum;
}

// 由 trips 展平出兼容视图（nodes / nodeArrivalMin / nodeIsStop / stops）并汇总。
// 扁平字段只在这一处生成，因此不会与 trips 各自维护而走样。
void flatten(RoutePlan& plan, int startTimeMin, double elapsedMin) {
    plan.nodes.clear();
    plan.nodeArrivalMin.clear();
    plan.nodeIsStop.clear();
    plan.nodeTripIndex.clear();
    plan.stops.clear();
    plan.totalDistanceKm = 0.0;
    plan.totalCostYuan = 0.0;
    plan.totalPenaltyMin = 0;

    bool firstTrip = true;
    for (std::size_t tripIndex = 0; tripIndex < plan.trips.size(); ++tripIndex) {
        Trip& trip = plan.trips[tripIndex];
        // 后续趟的起点与上一趟终点是同一个节点，展平时跳过，避免出现"原地一步"
        const std::size_t begin = firstTrip ? 0 : 1;
        for (std::size_t i = begin; i < trip.nodes.size(); ++i) {
            plan.nodes.push_back(trip.nodes[i]);
            plan.nodeArrivalMin.push_back(
                i < trip.nodeArrivalMin.size() ? trip.nodeArrivalMin[i] : 0);
            plan.nodeIsStop.push_back(i < trip.nodeIsStop.size() ? trip.nodeIsStop[i] : false);
            plan.nodeTripIndex.push_back(tripIndex);
        }
        for (const Stop& s : trip.stops) {
            plan.stops.push_back(s);
            plan.totalPenaltyMin += s.penaltyMin;
        }
        plan.totalDistanceKm += trip.totalDistanceKm;
        plan.totalCostYuan += trip.totalCostYuan;
        if (!trip.nodeArrivalMin.empty()) {
            trip.totalTimeMin = static_cast<double>(trip.nodeArrivalMin.back()
                                                    - trip.nodeArrivalMin.front());
        }
        firstTrip = false;
    }

    plan.returnArrivalMin = plan.nodeArrivalMin.empty() ? startTimeMin
                                                        : plan.nodeArrivalMin.back();
    // 用**未经取整**的 elapsed 计算，与历史口径一致；
    // 若改用 returnArrivalMin（已取整）反推，会引入 0.1min 级的精度回归
    plan.totalTimeMin = elapsedMin - static_cast<double>(startTimeMin);
}

// 收集图中全部中转站，用于统一填充暂存状态（未使用的中转站终值/峰值均为 0）
void collectTransits(const LogisticsGraph& graph, std::vector<TransitStock>& out) {
    for (const Node& n : graph.nodes()) {
        if (n.type == NodeType::Transit) {
            TransitStock st;
            st.nodeId = n.id;
            out.push_back(st);
        }
    }
}

// 多趟 + 中转集散（设计 §5.6）。仅在总需求超过载重上限时进入。
RoutePlan multiTripPlan(const LogisticsGraph& graph,
                        const Vehicle& vehicle,
                        const std::vector<Candidate>& candidates,
                        const std::string& startPos,
                        int startTimeMin,
                        double serviceTimeMin,
                        WeightType weight) {
    RoutePlan plan;

    // 子网络编号 → 该子网络的中转站。配送点按 sub_network_id 归属，
    // 这是数据里的行政归属（需求原文：中转站*下属*若干配送点），不重算最近枢纽。
    std::map<int, std::string> transitBySub;
    for (const Node& n : graph.nodes()) {
        if (n.type == NodeType::Transit && n.subNetworkId != 0) {
            transitBySub[n.subNetworkId] = n.id;
        }
    }

    // 分簇：键为中转站 ID；空字符串表示"仓库簇"（不属于任何子网络的配送点）
    std::map<std::string, std::vector<Candidate>> clusters;
    for (const Candidate& c : candidates) {
        const Node* node = graph.findNode(c.nodeId);
        const int sub = (node != nullptr) ? node->subNetworkId : 0;
        std::string hub;
        const std::map<int, std::string>::const_iterator it = transitBySub.find(sub);
        if (sub != 0 && it != transitBySub.end()) {
            hub = it->second;
        }
        clusters[hub].push_back(c);
    }

    // 簇按总货量降序处理（平局按 hub 名升序），保证输出确定
    std::vector<std::string> hubOrder;
    for (std::map<std::string, std::vector<Candidate>>::const_iterator it = clusters.begin();
         it != clusters.end(); ++it) {
        hubOrder.push_back(it->first);
    }
    for (std::size_t i = 0; i + 1 < hubOrder.size(); ++i) {
        std::size_t best = i;
        for (std::size_t j = i + 1; j < hubOrder.size(); ++j) {
            const double dj = totalDemand(clusters[hubOrder[j]]);
            const double db = totalDemand(clusters[hubOrder[best]]);
            if (dj > db || (dj == db && hubOrder[j] < hubOrder[best])) {
                best = j;
            }
        }
        if (best != i) {
            const std::string tmp = hubOrder[i];
            hubOrder[i] = hubOrder[best];
            hubOrder[best] = tmp;
        }
    }

    double elapsed = static_cast<double>(startTimeMin);
    std::string current = startPos;
    std::map<std::string, double> stock;
    std::map<std::string, double> peak;

    for (std::size_t h = 0; h < hubOrder.size(); ++h) {
        const std::string hub = hubOrder[h];
        std::vector<Candidate> pool = clusters[hub];
        const double clusterTotal = totalDemand(pool);
        std::string fail;

        // 单趟装得下就直接送达，不必动用中转站（保持默认数据的简单路径）
        const bool useStation = !hub.empty() && clusterTotal > vehicle.capacityKg + 1e-9;

        if (!useStation) {
            // 不经中转站：按载重上限分批，每批一趟直接从仓库出发送达后返回。
            // （簇总货量不超过载重时，这里天然只跑一趟，与单趟路径等价。）
            while (!pool.empty()) {
                double batchLoad = 0.0;
                std::vector<Candidate> batch;
                for (const Candidate& c : pool) {
                    if (c.demandKg > vehicle.capacityKg + 1e-9) {
                        plan.status = PlanStatus::OrderExceedsCapacity;
                        std::ostringstream os;
                        os << "订单货量 " << static_cast<long long>(std::lround(c.demandKg))
                           << "kg 超过载重上限 "
                           << static_cast<long long>(std::lround(vehicle.capacityKg)) << "kg";
                        plan.reason = os.str();
                        return plan;
                    }
                    if (batchLoad + c.demandKg <= vehicle.capacityKg + 1e-9) {
                        batch.push_back(c);
                        batchLoad += c.demandKg;
                    }
                }
                if (batch.empty()) {
                    break;   // 兜底，避免死循环
                }
                Trip trip;
                if (!weave(graph, batch, current, elapsed, serviceTimeMin, weight,
                           vehicle.startNodeId, vehicle.startNodeId, batchLoad, trip, fail)) {
                    plan.status = PlanStatus::Unreachable;
                    plan.reason = fail;
                    return plan;
                }
                plan.trips.push_back(trip);
                current = vehicle.startNodeId;
                for (const Candidate& b : batch) {
                    eraseCandidateByNode(pool, b.nodeId);
                }
            }
            continue;
        }

        // 经中转站：反复"回仓库补货入库"与"取货二次配发"，直到该簇送完
        while (!pool.empty()) {
            double smallest = -1.0;
            for (const Candidate& c : pool) {
                if (smallest < 0.0 || c.demandKg < smallest) {
                    smallest = c.demandKg;
                }
            }
            // 单个订单的货量就超过载重：分多少趟都装不下，这才是真正的载重不可行
            if (smallest > vehicle.capacityKg + 1e-9) {
                plan.status = PlanStatus::OrderExceedsCapacity;
                std::ostringstream os;
                os << "订单货量 " << static_cast<long long>(std::lround(smallest))
                   << "kg 超过载重上限 "
                   << static_cast<long long>(std::lround(vehicle.capacityKg)) << "kg";
                plan.reason = os.str();
                return plan;
            }

            if (stock[hub] + 1e-9 >= smallest) {
                // 暂存够用：取货并二次配发给该簇的配送点
                double take = 0.0;
                std::vector<Candidate> batch;
                for (const Candidate& c : pool) {
                    if (take + c.demandKg <= vehicle.capacityKg + 1e-9
                        && take + c.demandKg <= stock[hub] + 1e-9) {
                        batch.push_back(c);
                        take += c.demandKg;
                    }
                }
                if (batch.empty()) {
                    continue;   // 兜底：理论上不会发生，避免死循环
                }
                Trip trip;
                if (!weave(graph, batch, current, elapsed, serviceTimeMin, weight,
                           hub, hub, take, trip, fail)) {
                    plan.status = PlanStatus::Unreachable;
                    plan.reason = fail;
                    return plan;
                }
                TransitOp op;
                op.nodeId = hub;
                op.amountKg = -take;      // 出库
                trip.transitOps.push_back(op);
                stock[hub] -= take;
                plan.trips.push_back(trip);
                current = hub;
                for (const Candidate& b : batch) {
                    eraseCandidateByNode(pool, b.nodeId);
                }
            } else {
                // 暂存不足：回仓库补货，运到中转站入库暂存。本趟不送达任何订单。
                double load = 0.0;
                for (const Candidate& c : pool) {
                    if (load + c.demandKg <= vehicle.capacityKg + 1e-9) {
                        load += c.demandKg;
                    }
                }
                Trip trip;
                beginTrip(trip, current, elapsed);
                if (!appendLeg(graph, trip, elapsed, current, vehicle.startNodeId, weight,
                               false, fail)) {
                    plan.status = PlanStatus::Unreachable;
                    plan.reason = fail;
                    return plan;
                }
                current = vehicle.startNodeId;
                if (!appendLeg(graph, trip, elapsed, current, hub, weight, false, fail)) {
                    plan.status = PlanStatus::Unreachable;
                    plan.reason = fail;
                    return plan;
                }
                current = hub;
                TransitOp op;
                op.nodeId = hub;
                op.amountKg = load;       // 入库
                trip.transitOps.push_back(op);
                trip.endNodeId = hub;
                stock[hub] += load;
                if (stock[hub] > peak[hub]) {
                    peak[hub] = stock[hub];
                }
                plan.trips.push_back(trip);
            }
        }
    }

    // 收尾：车辆从当前位置返回起始仓库
    if (current != vehicle.startNodeId) {
        Trip trip;
        std::string fail;
        beginTrip(trip, current, elapsed);
        if (!appendLeg(graph, trip, elapsed, current, vehicle.startNodeId, weight, false, fail)) {
            plan.status = PlanStatus::Unreachable;
            plan.reason = fail;
            return plan;
        }
        trip.endNodeId = vehicle.startNodeId;
        plan.trips.push_back(trip);
        current = vehicle.startNodeId;
    }

    // 暂存状态：全部中转站都记录（未使用的终值与峰值均为 0）
    collectTransits(graph, plan.transitStock);
    for (TransitStock& st : plan.transitStock) {
        st.finalKg = stock[st.nodeId];
        st.peakKg = peak[st.nodeId];
    }
    flatten(plan, startTimeMin, elapsed);
    return plan;
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

    const std::vector<Candidate> remaining = buildCandidates(remainingOrders);
    const double total = totalDemand(remaining);

    if (total > vehicle.capacityKg + 1e-9) {
        // 总需求超过载重：改走多趟 + 中转集散（D21），不再判为不可行
        return multiTripPlan(graph, vehicle, remaining, currentPositionId, currentTimeMin,
                             serviceTimeMin, weight);
    }

    // 单趟：车辆在起点已装载全部货物，直接贪心串联后回仓库（与历史行为一致）
    double elapsed = static_cast<double>(currentTimeMin);
    Trip trip;
    std::string fail;
    if (!weave(graph, remaining, currentPositionId, elapsed, serviceTimeMin, weight,
               std::string(), vehicle.startNodeId, total, trip, fail)) {
        plan.status = PlanStatus::Unreachable;
        plan.reason = fail;
        return plan;
    }
    plan.trips.push_back(trip);

    collectTransits(graph, plan.transitStock);
    flatten(plan, currentTimeMin, elapsed);
    return plan;
}

namespace {

const Candidate* findCandidate(const std::vector<Candidate>& pool, const std::string& nodeId) {
    for (const Candidate& c : pool) {
        if (c.nodeId == nodeId) {
            return &c;
        }
    }
    return nullptr;
}

// 从停靠记录生成一个 Stop（时间推进口径与 weave 完全一致）
Stop makeStop(const Candidate& chosen, double& elapsedMin, double serviceTimeMin,
              double& loadKg) {
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
        ? static_cast<int>(std::lround(arrival - static_cast<double>(chosen.windowEndMin))) : 0;
    elapsedMin = arrival + serviceTimeMin;
    stop.departureMin = static_cast<int>(std::lround(elapsedMin));
    loadKg -= chosen.demandKg;
    stop.remainingLoadKg = loadKg;
    return stop;
}

} // namespace

RoutePlan replanIncremental(const LogisticsGraph& graph,
                            const Vehicle& vehicle,
                            const std::vector<Order>& remainingOrders,
                            const RoutePlan& previous,
                            int currentTimeMin,
                            double serviceTimeMin,
                            WeightType weight,
                            const TrafficReport& report,
                            double thresholdRatio) {
    const std::string startPos =
        previous.nodes.empty() ? vehicle.startNodeId : previous.nodes.front();

    // 适用范围：单趟且序列完整。否则退回全量重算。
    if (previous.status != PlanStatus::Ok || previous.trips.size() != 1
        || previous.nodes.size() < 2
        || previous.nodes.back() != vehicle.startNodeId) {
        return replan(graph, vehicle, remainingOrders, startPos, currentTimeMin,
                      serviceTimeMin, weight);
    }

    const std::vector<Candidate> pool = buildCandidates(remainingOrders);

    // 停靠点在扁平序列中的下标 -> 切段边界 [起点, 站1, 站2, …, 末站, 终点]
    std::vector<std::size_t> bounds;
    bounds.push_back(0);
    for (std::size_t i = 0; i < previous.nodeIsStop.size(); ++i) {
        if (previous.nodeIsStop[i]) {
            bounds.push_back(i);
        }
    }
    bounds.push_back(previous.nodes.size() - 1);
    const std::size_t stopCount = bounds.size() - 2;

    // 逐段处理：只有"走过的边被路况命中"的段才重新求最短路
    std::vector<std::vector<std::string> > legPaths;
    for (std::size_t k = 0; k + 1 < bounds.size(); ++k) {
        const std::size_t b = bounds[k];
        const std::size_t e = bounds[k + 1];

        bool affected = false;
        for (std::size_t i = b + 1; i <= e && !affected; ++i) {
            for (const TrafficChange& c : report.changes) {
                if (c.increaseRatio >= thresholdRatio
                    && c.fromId == previous.nodes[i - 1] && c.toId == previous.nodes[i]) {
                    affected = true;
                    break;
                }
            }
        }

        std::vector<std::string> path;
        if (affected && previous.nodes[b] != previous.nodes[e]) {
            const PathResult r = shortestPath(graph, previous.nodes[b], previous.nodes[e], weight);
            if (!r.found) {
                // 该段已不可达：退回全量重算，由它给出统一的原因
                return replan(graph, vehicle, remainingOrders, startPos, currentTimeMin,
                              serviceTimeMin, weight);
            }
            path = r.nodes;
        } else {
            path.assign(previous.nodes.begin() + static_cast<std::ptrdiff_t>(b),
                        previous.nodes.begin() + static_cast<std::ptrdiff_t>(e) + 1);
        }
        legPaths.push_back(path);
    }

    // 沿各段重建路线（停靠顺序保持不变）
    RoutePlan plan;
    Trip trip;
    double elapsed = static_cast<double>(currentTimeMin);
    double load = 0.0;
    for (const Candidate& c : pool) {
        load += c.demandKg;
    }

    trip.nodes.push_back(startPos);
    trip.nodeArrivalMin.push_back(currentTimeMin);
    trip.nodeIsStop.push_back(false);

    for (std::size_t k = 0; k < legPaths.size(); ++k) {
        const std::vector<std::string>& path = legPaths[k];
        const bool endpointIsStop = (k < stopCount);

        for (std::size_t i = 1; i < path.size(); ++i) {
            const Edge* edge = graph.findEdge(path[i - 1], path[i]);
            if (edge == nullptr) {
                return replan(graph, vehicle, remainingOrders, startPos, currentTimeMin,
                              serviceTimeMin, weight);
            }
            elapsed += edge->timeMin;
            trip.totalDistanceKm += edge->distanceKm;
            trip.totalCostYuan += edge->costYuan;
            trip.nodes.push_back(path[i]);
            trip.nodeArrivalMin.push_back(static_cast<int>(std::lround(elapsed)));
            trip.nodeIsStop.push_back(endpointIsStop && (i + 1 == path.size()));
        }

        if (!endpointIsStop) {
            continue;
        }
        const Candidate* chosen = findCandidate(pool, path.back());
        if (chosen == nullptr) {
            return replan(graph, vehicle, remainingOrders, startPos, currentTimeMin,
                          serviceTimeMin, weight);
        }
        const Stop stop = makeStop(*chosen, elapsed, serviceTimeMin, load);
        // 停靠节点记录实际送达时刻（等窗口开启之后）
        trip.nodeArrivalMin[trip.nodeArrivalMin.size() - 1] = stop.arrivalMin;
        trip.stops.push_back(stop);
    }

    trip.endNodeId = vehicle.startNodeId;
    plan.trips.push_back(trip);
    collectTransits(graph, plan.transitStock);
    flatten(plan, currentTimeMin, elapsed);
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
