#include "core/RoutePlanner.h"

#include <cmath>
#include <sstream>
#include <cstddef>
#include <map>
#include <set>

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
           WeightType weight,
           const std::string& pickupNode,
           const std::string& endNode,
           double loadKg,
           Trip& trip,
           std::string& failReason) {
    trip.loadKg = loadKg;
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

        elapsedMin = arrival;

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

// 多趟 + 中转集散（设计 §5.6）。仅在总需求超过载重上限时进入。
// 多趟规划的实际实现：分簇 -> 按载重分批 -> 每批一趟。中转站不参与排线。
// （用于生成"直达"对照版本）。
RoutePlan multiTripPlanImpl(const LogisticsGraph& graph,
                            const Vehicle& vehicle,
                            const std::vector<Candidate>& candidates,
                            const std::string& startPos,
                            int startTimeMin,
                            WeightType weight,
                            const std::vector<OnboardItem>& onboard) {
    RoutePlan plan;

    // 子网络编号 → 该子网络的中转站。配送点按 sub_network_id 归属，
    // 这是数据里的行政归属（需求原文：中转站*下属*若干配送点），不重算最近枢纽。
    std::map<int, std::string> transitBySub;
    for (const Node& n : graph.nodes()) {
        if (n.type == NodeType::Transit && n.subNetworkId != 0) {
            transitBySub[n.subNetworkId] = n.id;
        }
    }

    // ---- 紧急订单优先：单独成趟，最先执行（E3）----
    //
    // 多趟模式下不能只靠"簇内紧急优先"：若紧急订单分属两个簇，它们仍会被
    // 各自簇的行程隔开，不满足「优先满足紧急订单」。同样地，现实中紧急单
    // 也应当立即派车，因此这里把它摘出来作为独立批次直接从仓库送达。
    std::vector<Candidate> urgentPool;
    std::vector<Candidate> normalPool;
    for (const Candidate& c : candidates) {
        if (c.urgent) {
            urgentPool.push_back(c);
        } else {
            normalPool.push_back(c);
        }
    }

    double elapsed = static_cast<double>(startTimeMin);
    std::string current = startPos;

    while (!urgentPool.empty()) {
        double batchLoad = 0.0;
        std::vector<Candidate> batch;
        for (const Candidate& c : urgentPool) {
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
            break;   // 兜底
        }
        Trip trip;
        std::string fail;
        if (!weave(graph, batch, current, elapsed, weight,
                   vehicle.startNodeId, vehicle.startNodeId, batchLoad, trip, fail)) {
            plan.status = PlanStatus::Unreachable;
            plan.reason = fail;
            return plan;
        }
        plan.trips.push_back(trip);
        current = vehicle.startNodeId;
        for (const Candidate& b : batch) {
            eraseCandidateByNode(urgentPool, b.nodeId);
        }
    }

    // ---- 车上已经载着的货，接着把它们送掉（在**紧急订单之后**）----
    //
    // 顺序：紧急订单 -> 在途货 -> 其余按簇分批。
    //   · 紧急订单必须最先（E3）；它的货不在车上，车得先回仓库取，
    //     而这正是"顺路寄存"发生的时机——在途货会被卸在回程路径上的站里。
    //   · 紧急批次之后才轮到在途货：它们已经在车上，从**当前位置**直接出发
    //     即可，不必跑一趟仓库。
    // 这一步不能省：若规划器对在途货一无所知，它会按"货物都在仓库"来排线，
    // 于是每次重规划的每一段都可能先跑回仓库——车在仓库与客户之间来回跳，
    // Debug 模式下尤其明显（每个 tick 都重规划一次）。
    if (!onboard.empty()) {
        // 必须从**剩余池**里取，不能从全量 candidates 取：
        // 紧急批次可能已经把某些客户送掉了，从全量取会把它们再送一遍
        // （表现为两趟行程完全相同，车在原地打转）。
        std::vector<Candidate> carried;
        for (const OnboardItem& item : onboard) {
            for (const std::vector<Candidate>* pool : {&urgentPool, &normalPool}) {
                for (const Candidate& c : *pool) {
                    if (c.nodeId == item.nodeId) {
                        carried.push_back(c);
                    }
                }
            }
        }
        // 同一节点可能有多张订单，这里按节点归并，避免同一趟里重复插入同一站点
        {
            std::vector<Candidate> uniq;
            for (const Candidate& c : carried) {
                bool seen = false;
                for (const Candidate& u : uniq) {
                    if (u.nodeId == c.nodeId) {
                        seen = true;
                        break;
                    }
                }
                if (!seen) {
                    uniq.push_back(c);
                }
            }
            carried.swap(uniq);
        }
        if (!carried.empty()) {
            Trip trip;
            std::string fail;
            const double load = totalDemand(carried);
            // 从**当前位置**直接出发（车上的货不需要回仓库取），终点仍是仓库
            if (weave(graph, carried, startPos, elapsed, weight,
                      startPos, vehicle.startNodeId, load, trip, fail)) {
                plan.trips.push_back(trip);
                current = trip.endNodeId;
                for (const Candidate& b : carried) {
                    eraseCandidateByNode(urgentPool, b.nodeId);
                    eraseCandidateByNode(normalPool, b.nodeId);
                }
            }
        }
    }

    // 分簇：键为中转站 ID；空字符串表示"仓库簇"（不属于任何子网络的配送点）
    std::map<std::string, std::vector<Candidate>> clusters;
    for (const Candidate& c : normalPool) {
        const Node* node = graph.findNode(c.nodeId);
        const int sub = (node != nullptr) ? node->subNetworkId : 0;
        std::string hub;
        const std::map<int, std::string>::const_iterator it = transitBySub.find(sub);
        if (sub != 0 && it != transitBySub.end()) {
            hub = it->second;
        }
        clusters[hub].push_back(c);
    }

    // 簇按总货量降序处理（平局按 hub 名升序），保证输出确定。
    // 紧急订单已在上面的独立批次里先行处理，不参与这里的簇排序。
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

    // 期初存货：站里本来就有货时，才可能把它当前置仓库用

    for (std::size_t h = 0; h < hubOrder.size(); ++h) {
        const std::string hub = hubOrder[h];
        std::vector<Candidate> pool = clusters[hub];
        const double clusterTotal = totalDemand(pool);
        std::string fail;

        // 是否动用中转站：**只有站内确实有存货时才考虑**。
        //
        // 这是用户第 9 轮定的原则："一切决策都不应该为了满足某种策略的前提条件
        // 而去实际执行更差的策略。" 实验（设计 §16 P12）证明：让每个簇都强行经站，
        // 在单车辆模型下比直达分批全面更差（198.2 vs 178.2km、13 趟 vs 6 趟）。
        // 中转站初始无存货，因此默认数据一律走下面的直达分批。
        //
        // 站内存货来自"即将带回仓库的余货顺路寄存"（前置储存点机制，见 §16 P13）。
        // 在那一机制落地之前，stock 恒为 0，此分支不会进入。
        // 中转站**不参与路由**。
        // 中转站**不参与排线**：实测让它参与全面更差（198.2km/13 趟 vs 直达
        // 178.2km/6 趟），且车一停在站里就会排出「站 -> 仓库 -> 站」的补货趟，
        // 造成车在仓库附近来回跳。它只作为节点类型与子网络标识存在。
        // （原先那条"经中转站"的实现已随本轮代码整理删除。）
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
            if (!weave(graph, batch, current, elapsed, weight,
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

    flatten(plan, startTimeMin, elapsed);
    return plan;
}

// 多趟规划入口。
//
// 三件事，按顺序：
//   ① **顺路寄存**：在途货里那些"车本次要回仓库、于是会白带一趟"的部分，
//      若它所属簇的中转站就在回程路径上，就顺手卸在站里——**零成本**，
//      只是把那部分货从"跟着车白跑"变成"站里的期初存货"。
//      安全约束（用户第 10 轮确认）：只卸在**回程路径上的站**，且**必须属于
//      该站所服务的簇**；不满足就原样带回去，绝不为了寄存而绕路。
//   ② 算两版：一版完全不许用中转站（直达），一版允许把中转站当前置仓库用。
//   ③ 取更优者（用站版 penalty 更差则一票否决），使"绝不更差"成为构造保证。
// 多趟规划入口：把所有剩余订单排成若干趟。
//
// 历史上这里做过"直达 vs 经中转站"两版取优（先把"中转站绝不使结果更差"变成
// 构造性保证）。第 13 轮实测证明经站路全面更差，中转站因此退出排线；
// 两版随之变成**参数完全相同**的两次调用——保留它只会误导读者，
// 故收敛为单次调用。若将来要重新引入"备用方案取优"，请连同判据一起加回来。
RoutePlan multiTripPlan(const LogisticsGraph& graph,
                        const Vehicle& vehicle,
                        const std::vector<Candidate>& candidates,
                        const std::string& startPos,
                        int startTimeMin,
                        WeightType weight,
                        const std::vector<OnboardItem>& onboard) {
    return multiTripPlanImpl(graph, vehicle, candidates, startPos, startTimeMin,
                             weight, onboard);
}

} // namespace

RoutePlan replan(const LogisticsGraph& graph,
                 const Vehicle& vehicle,
                 const std::vector<Order>& remainingOrders,
                 const std::string& currentPositionId,
                 int currentTimeMin,
                 WeightType weight,
                 const std::vector<OnboardItem>& onboard) {
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
                             weight, onboard);
    }

    // 单趟：车辆在起点已装载全部货物，直接贪心串联后回仓库（与历史行为一致）
    double elapsed = static_cast<double>(currentTimeMin);
    Trip trip;
    std::string fail;
    if (!weave(graph, remaining, currentPositionId, elapsed, weight,
               std::string(), vehicle.startNodeId, total, trip, fail)) {
        plan.status = PlanStatus::Unreachable;
        plan.reason = fail;
        return plan;
    }
    plan.trips.push_back(trip);

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
Stop makeStop(const Candidate& chosen, double& elapsedMin, double& loadKg) {
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
    elapsedMin = arrival;
    loadKg -= chosen.demandKg;
    stop.remainingLoadKg = loadKg;
    return stop;
}

} // namespace

RoutePlan sliceRemainder(const RoutePlan& plan, std::size_t fromNodeIndex) {
    RoutePlan remainder;
    remainder.status = PlanStatus::Ok;
    if (plan.nodes.empty() || plan.trips.empty()) {
        return remainder;
    }
    const std::size_t from = (fromNodeIndex < plan.nodes.size()) ? fromNodeIndex : 0;

    // 每一趟在扁平序列中的区间 [begin, end)
    std::vector<std::size_t> tripBegin(plan.trips.size(), 0);
    std::vector<std::size_t> tripEnd(plan.trips.size(), 0);
    for (std::size_t i = 0; i < plan.nodeTripIndex.size(); ++i) {
        const std::size_t t = plan.nodeTripIndex[i];
        if (t < plan.trips.size()) {
            if (tripEnd[t] == 0) {
                tripBegin[t] = i;
            }
            tripEnd[t] = i + 1;
        }
    }
    const std::size_t cur =
        (from < plan.nodeTripIndex.size()) ? plan.nodeTripIndex[from] : 0;

    for (std::size_t t = cur; t < plan.trips.size(); ++t) {
        const std::size_t b = (t == cur && from > tripBegin[t]) ? from : tripBegin[t];
        Trip trip;
        for (std::size_t i = b; i < tripEnd[t] && i < plan.nodes.size(); ++i) {
            trip.nodes.push_back(plan.nodes[i]);
            trip.nodeArrivalMin.push_back(plan.nodeArrivalMin[i]);
            trip.nodeIsStop.push_back(plan.nodeIsStop[i]);
        }
        if (trip.nodes.empty()) {
            continue;
        }
        trip.endNodeId = trip.nodes.back();
        trip.loadKg = plan.trips[t].loadKg;
        remainder.trips.push_back(trip);
        for (std::size_t i = 0; i < trip.nodes.size(); ++i) {
            remainder.nodes.push_back(trip.nodes[i]);
            remainder.nodeIsStop.push_back(trip.nodeIsStop[i]);
        }
    }
    return remainder;
}

RoutePlan replanIncremental(const LogisticsGraph& graph,
                            const Vehicle& vehicle,
                            const std::vector<Order>& remainingOrders,
                            const RoutePlan& previous,
                            int currentTimeMin,
                            WeightType weight,
                            const TrafficReport& report,
                            double thresholdRatio,
                            const std::vector<OnboardItem>& onboard) {
    const std::string startPos =
        previous.nodes.empty() ? vehicle.startNodeId : previous.nodes.front();

    // 适用范围：单趟且序列完整。否则退回全量重算。
    if (previous.status != PlanStatus::Ok || previous.trips.size() != 1
        || previous.nodes.size() < 2
        || previous.nodes.back() != vehicle.startNodeId) {
        return replan(graph, vehicle, remainingOrders, startPos, currentTimeMin,
                      weight, onboard);
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
                              weight, onboard);
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
                              weight, onboard);
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
                          weight, onboard);
        }
        const Stop stop = makeStop(*chosen, elapsed, load);
        // 停靠节点记录实际送达时刻（等窗口开启之后）
        trip.nodeArrivalMin[trip.nodeArrivalMin.size() - 1] = stop.arrivalMin;
        trip.stops.push_back(stop);
    }

    trip.endNodeId = vehicle.startNodeId;
    plan.trips.push_back(trip);
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
                    WeightType weight) {
    return replan(graph, vehicle, orders, vehicle.startNodeId,
                  vehicle.departTimeMin, weight);
}

InsertResult insertUrgentOrder(const LogisticsGraph& graph,
                               const Vehicle& vehicle,
                               const std::vector<Order>& remainingOrders,
                               const Order& newOrder,
                               const std::string& currentPositionId,
                               int currentTimeMin,
                               WeightType weight,
                               const std::vector<OnboardItem>& onboard) {
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
                         weight, onboard);
    return result;
}


} // namespace logistics
