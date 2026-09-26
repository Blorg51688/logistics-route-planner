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

// 多趟分批（设计 §5.6）。仅在总需求超过载重上限时进入。
// 实际实现：分簇 -> 按载重分批 -> 每批一趟。中转站不参与排线。

// ---- 中转站缓冲库存（2026-09-26 用户裁定恢复；**仅服务紧急单**）----
//
// 生产端：每次**出仓装满**，本趟订单用不到的那部分记作 trip.bufferKg
// （不属于任何订单的缓冲货）。返程若顺路经过本趟所属簇的中转站，就地把缓冲卸进站。
//
// 为什么是零成本：`appendLeg` 会把最短路上的**全部途经节点**写进 trip.nodes，
// 所以"顺路"意味着该站**本来就在序列里**——寄存不新增节点、不改时刻、不改里程。
// （本项目停靠无服务时间，故距离/耗时/成本/penalty 一个字节都不动。）

// 缓冲库存机制：把**指定的那一张**紧急单的取货点从"仓库"改到"车上缓冲 / 中转站"。
// `nodeId` 为空 = 不启用（= 候选 A：独立紧急趟先回仓库装货）。
//
// 纪律：只换**取货点**。紧急批仍然排在所有普通停靠点之前（E3 优先性不得被取优牺牲），
// 也不动本趟要送的订单货（不借货）。
struct UrgentSupply {
    std::string nodeId;   // 目标紧急单的配送点
    std::string pickup;   // 取货点；**空串 = 车上已有，不新增节点**
};

// 把站内装卸按时刻排好。规格要求 transitOps 是"展平后**按时刻**"的序列，
// 而"趟收尾时记账 + 紧急单取用时补记"两个来源天然会插乱顺序。
// 手写插入排序：本项目的 STL 口径禁止把算法主体交给标准库（std::sort 不可用）。
void sortTransitOpsByTime(std::vector<TransitOp>& ops) {
    for (std::size_t i = 1; i < ops.size(); ++i) {
        const TransitOp key = ops[i];
        std::size_t j = i;
        while (j > 0 && ops[j - 1].atMin > key.atMin) {
            ops[j] = ops[j - 1];
            --j;
        }
        ops[j] = key;
    }
}

// 求某个配送点在候选集里的（**合并后**）需求量。同一配送点上的多张订单会被
// buildCandidates 合并成同一个停靠点，所以"这一张订单的货量"≠"这个停靠点要送的货量"。
double candidateDemand(const std::vector<Candidate>& candidates, const std::string& nodeId) {
    for (const Candidate& c : candidates) {
        if (c.nodeId == nodeId) {
            return c.demandKg;
        }
    }
    return 0.0;
}

// 读取某站当前的库存（不在表里 = 0）
double stockOf(const std::map<std::string, double>& stock, const std::string& stationId) {
    const std::map<std::string, double>::const_iterator it = stock.find(stationId);
    return (it != stock.end()) ? it->second : 0.0;
}

// 记一次**站内出库**（紧急单就地取用）：扣减期末库存 + 写一条负的 TransitOp。
//
// atMin 必须取"车**真的**到过该站"的时刻——从计划自己的节点序列里实读。
// 读不到就不记账（宁可不记，也不记一条追溯不到的出库：守卫 G3 要求每一次库存变动
// 都能追溯到某趟的 nodeArrivalMin）。
bool recordStationDraw(RoutePlan& plan, const std::string& stationId,
                       const std::string& urgentNodeId, double kg) {
    if (stationId.empty() || kg <= 1e-9) {
        return false;
    }
    for (const Trip& t : plan.trips) {
        bool servesUrgent = false;   // 只认"服务了该紧急单的那一趟"
        for (const Stop& s : t.stops) {
            if (s.nodeId == urgentNodeId) {
                servesUrgent = true;
            }
        }
        if (!servesUrgent) {
            continue;
        }
        for (std::size_t k = 0; k < t.nodes.size() && k < t.nodeArrivalMin.size(); ++k) {
            if (t.nodes[k] != stationId) {
                continue;
            }
            TransitOp op;
            op.nodeId = stationId;
            op.kgDelta = -kg;
            op.atMin = t.nodeArrivalMin[k];
            plan.transitOps.push_back(op);
            for (TransitStock& st : plan.transitStock) {
                if (st.nodeId == stationId && st.finalKg + 1e-9 >= kg) {
                    st.finalKg -= kg;
                    return true;
                }
            }
            plan.transitOps.pop_back();   // 库存不够：撤回这条，宁可不记
            return false;
        }
    }
    return false;
}

// 求一个配送点所属子网络的中转站（空串 = 无站）
std::string hubOfNode(const LogisticsGraph& graph,
                      const std::map<int, std::string>& transitBySub,
                      const std::string& nodeId) {
    const Node* n = graph.findNode(nodeId);
    if (n == nullptr) {
        return std::string();
    }
    const std::map<int, std::string>::const_iterator it = transitBySub.find(n->subNetworkId);
    return (it != transitBySub.end()) ? it->second : std::string();
}

// 求一趟停靠点所属的**唯一**簇站。跨簇 / 无站 / 无停靠点 ⇒ 返回空串（不寄存）。
std::string tripHub(const LogisticsGraph& graph,
                    const std::map<int, std::string>& transitBySub,
                    const Trip& trip) {
    std::set<int> subs;
    for (const Stop& s : trip.stops) {
        const Node* n = graph.findNode(s.nodeId);
        if (n != nullptr && transitBySub.find(n->subNetworkId) != transitBySub.end()) {
            subs.insert(n->subNetworkId);
        }
    }
    if (subs.size() != 1) {
        return std::string();
    }
    const std::map<int, std::string>::const_iterator it = transitBySub.find(*subs.begin());
    return (it != transitBySub.end()) ? it->second : std::string();
}

// 顺路寄存（**纯记账**）。成立则往 trip.bankOps 记一条，并通过 out 回报。
//
// 判定**以实际序列为准**：该站必须真的出现在"最后一个停靠点之后"的回程段里。
// 只信距离等式是不够的——并列最短路时等式可能成立，而车其实没走那条路；
// 那时记一条寄存就等于"把货卸在车没去过的站"（守卫 G1② 正是守这件事）。
// 距离等式再作一次交叉核对，两边都成立才寄存。
bool bankBufferEnRoute(const LogisticsGraph& graph,
                       const std::string& hub,
                       const std::string& depotId,
                       WeightType weight,
                       Trip& trip,
                       TransitOp* out) {
    if (hub.empty() || trip.bufferKg <= 1e-9) {
        return false;
    }
    std::size_t lastStopIdx = trip.nodes.size();
    for (std::size_t i = 0; i < trip.nodeIsStop.size() && i < trip.nodes.size(); ++i) {
        if (trip.nodeIsStop[i]) {
            lastStopIdx = i;
        }
    }
    if (lastStopIdx + 1 >= trip.nodes.size()) {
        return false;   // 停靠点之后没有回程段
    }
    std::size_t hubIdx = trip.nodes.size();
    for (std::size_t i = lastStopIdx + 1; i < trip.nodes.size(); ++i) {
        if (trip.nodes[i] == hub) {
            hubIdx = i;
            break;
        }
    }
    if (hubIdx >= trip.nodes.size()) {
        return false;   // 回程没真的经过该站 ⇒ 不寄存
    }
    const std::string& lastStopNode = trip.nodes[lastStopIdx];
    const PathResult back = shortestPath(graph, lastStopNode, depotId, weight);
    const PathResult lh = shortestPath(graph, lastStopNode, hub, weight);
    const PathResult hw = shortestPath(graph, hub, depotId, weight);
    if (!back.found || !lh.found || !hw.found) {
        return false;
    }
    if (lh.totalWeight + hw.totalWeight - back.totalWeight > 1e-6) {
        return false;   // 顺路等式不成立 ⇒ 不寄存（缓冲跟车回仓库）
    }
    TransitOp op;
    op.nodeId = hub;
    op.kgDelta = trip.bufferKg;
    op.atMin = trip.nodeArrivalMin[hubIdx];
    trip.bankOps.push_back(op);
    if (out != nullptr) {
        *out = op;
    }
    return true;
}

// 装满车厢：本趟订单货量之外的空位全部装成**缓冲货**（不属于任何订单）。
// 只在"这一趟是在仓库装货出发"时调用——在途货趟与收尾趟都不是出仓，不得凭空计缓冲。
void fillBufferFromDepot(const Vehicle& vehicle, double batchLoadKg, Trip& trip) {
    const double spare = vehicle.capacityKg - batchLoadKg;
    trip.bufferKg = (spare > 0.0) ? spare : 0.0;
}

RoutePlan multiTripPlanImpl(const LogisticsGraph& graph,
                            const Vehicle& vehicle,
                            const std::vector<Candidate>& candidates,
                            const std::string& startPos,
                            int startTimeMin,
                            WeightType weight,
                            const std::vector<OnboardItem>& onboard,
                            const std::map<std::string, double>& initialStock,
                            const UrgentSupply& supply) {
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
        // 缓冲库存机制：若这一批里有"目标紧急单"，它的取货点可以被换成车上缓冲 / 中转站。
        // 只换**取货点**——批次结构（紧急在先、独立成趟）一律不动。
        std::string pickupNode = vehicle.startNodeId;
        bool supplyUsed = false;
        for (const Candidate& c : batch) {
            if (!supply.nodeId.empty() && c.nodeId == supply.nodeId) {
                pickupNode = supply.pickup;
                supplyUsed = true;
            }
        }
        if (!weave(graph, batch, current, elapsed, weight,
                   pickupNode, vehicle.startNodeId, batchLoad, trip, fail)) {
            plan.status = PlanStatus::Unreachable;
            plan.reason = fail;
            return plan;
        }
        if (supplyUsed) {
            // 用缓冲/站内存货就地满足：这一批**不是出仓**（没回仓库装货）
            // ⇒ 不计缓冲；站内取用由 insertUrgentOrder 在选定方案后记账。
            trip.bufferKg = 0.0;
        } else {
            // 出仓装满：本批订单之外的载重空位装成**缓冲货**（不属任何订单）
            fillBufferFromDepot(vehicle, batchLoad, trip);
            TransitOp banked;
            if (bankBufferEnRoute(graph, tripHub(graph, transitBySub, trip),
                                  vehicle.startNodeId, weight, trip, &banked)) {
                plan.transitOps.push_back(banked);
            }
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
    //     因此车必须回仓库取货（"顺路寄存"机制已移除，见设计 §16 P25/P30）。
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
                // **不装缓冲**：这一趟是从当前位置接着送车上已有的货，不是"出仓"。
                // 它的货早在前一趟出仓时就装过一次了，这里再算一次缓冲会**重复计数**。
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

    // 逐簇处理：下面按载重上限分批，不再有"动用中转站"的分支。

    for (std::size_t h = 0; h < hubOrder.size(); ++h) {
        const std::string hub = hubOrder[h];
        std::vector<Candidate> pool = clusters[hub];
        const double clusterTotal = totalDemand(pool);
        std::string fail;

        // 中转站不参与路由，也不参与排线：实测让它参与全面更差
        // （198.2km/13 趟 vs 直达 178.2km/6 趟），且车一停在站里就会排出
        // 「站 -> 仓库 -> 站」的补货趟，造成车在仓库附近来回跳。
        // 它只作为节点类型与子网络标识存在；原先那条"经中转站"的实现
        // 已随代码整理删除（设计 §16 P25/P30）。
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
            // 出仓装满 + 返程顺路寄存（同紧急批；簇批一定是"从仓库装货出发"）
            fillBufferFromDepot(vehicle, batchLoad, trip);
            {
                TransitOp banked;
                if (bankBufferEnRoute(graph, tripHub(graph, transitBySub, trip),
                                      vehicle.startNodeId, weight, trip, &banked)) {
                    plan.transitOps.push_back(banked);
                }
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

    // ---- 站内库存汇总：期初（来自 initialStock）+ 本次规划期间的全部寄存 ----
    // 只列"数据里确实存在的站"（transitBySub 的值，去重），保证界面与守恒断言口径一致。
    {
        std::vector<std::string> stations;
        for (std::map<int, std::string>::const_iterator it = transitBySub.begin();
             it != transitBySub.end(); ++it) {
            bool seen = false;
            for (const std::string& s : stations) {
                if (s == it->second) {
                    seen = true;
                    break;
                }
            }
            if (!seen) {
                stations.push_back(it->second);
            }
        }
        for (const std::string& id : stations) {
            TransitStock st;
            st.nodeId = id;
            const std::map<std::string, double>::const_iterator it = initialStock.find(id);
            st.initialKg = (it != initialStock.end()) ? it->second : 0.0;
            st.finalKg = st.initialKg;
            plan.transitStock.push_back(st);
        }
        // 记账：每一条 bankOps 都要落到对应站的期末库存上（守恒由守卫 G3 核）
        for (const TransitOp& op : plan.transitOps) {
            for (TransitStock& st : plan.transitStock) {
                if (st.nodeId == op.nodeId) {
                    st.finalKg += op.kgDelta;
                    break;
                }
            }
        }
    }

    sortTransitOpsByTime(plan.transitOps);
    flatten(plan, startTimeMin, elapsed);
    return plan;
}

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
                        const std::vector<OnboardItem>& onboard,
                        const std::map<std::string, double>& initialStock,
                        const UrgentSupply& supply = UrgentSupply()) {
    return multiTripPlanImpl(graph, vehicle, candidates, startPos, startTimeMin,
                             weight, onboard, initialStock, supply);
}

} // namespace

RoutePlan replan(const LogisticsGraph& graph,
                 const Vehicle& vehicle,
                 const std::vector<Order>& remainingOrders,
                 const std::string& currentPositionId,
                 int currentTimeMin,
                 WeightType weight,
                 const std::vector<OnboardItem>& onboard,
                 const std::map<std::string, double>& initialStock) {
    RoutePlan plan;

    if (graph.findNode(currentPositionId) == nullptr) {
        plan.status = PlanStatus::Unreachable;
        plan.reason = "出发位置不存在: " + currentPositionId;
        return plan;
    }

    const std::vector<Candidate> remaining = buildCandidates(remainingOrders);
    const double total = totalDemand(remaining);

    if (total > vehicle.capacityKg + 1e-9) {
        // 总需求超过载重：改走多趟分批（D21），不再判为不可行
        return multiTripPlan(graph, vehicle, remaining, currentPositionId, currentTimeMin,
                             weight, onboard, initialStock);
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
        // 缓冲货与寄存记账必须**随趟一起带走**：否则切出"剩余路线"后本趟的缓冲凭空消失，
        // 随后的紧急单取优就看不到车上还有货了。
        trip.bufferKg = plan.trips[t].bufferKg;
        trip.bankOps = plan.trips[t].bankOps;
        remainder.trips.push_back(trip);
        for (std::size_t i = 0; i < trip.nodes.size(); ++i) {
            remainder.nodes.push_back(trip.nodes[i]);
            remainder.nodeIsStop.push_back(trip.nodeIsStop[i]);
        }
    }
    // 库存快照必须带到"剩余路线"上：否则路况重规划/增量重算之后库存凭空归零，
    // 守恒断言（规格 I3 / 守卫 G3）会失效。
    remainder.transitStock = plan.transitStock;
    remainder.transitOps = plan.transitOps;
    return remainder;
}

// 对**一趟**做增量重算：把该趟按停靠点切成若干 leg，只有"走过的边被路况命中"的
// leg 才重新求最短路，其余 leg 原样复用；停靠顺序不变。
//
// 返回 false 表示这一趟无法增量（序列不全 / 某个受影响 leg 已不可达），
// 调用方应退回全量重算。startTimeMin 是本趟出发时刻。
bool replanOneTrip(const LogisticsGraph& graph,
                   const std::vector<Candidate>& pool,
                   const Trip& prevTrip,
                   const TrafficReport& report,
                   double thresholdRatio,
                   WeightType weight,
                   int startTimeMin,
                   Trip& out,
                   double& endTimeMin) {
    if (prevTrip.nodes.size() < 2 || prevTrip.nodeIsStop.size() != prevTrip.nodes.size()) {
        return false;   // 序列不完整，增量无从下手
    }

    // 切段边界：[起点, 站1, 站2, …, 末站, 终点]
    std::vector<std::size_t> bounds;
    bounds.push_back(0);
    for (std::size_t i = 0; i < prevTrip.nodeIsStop.size(); ++i) {
        if (prevTrip.nodeIsStop[i]) {
            bounds.push_back(i);
        }
    }
    bounds.push_back(prevTrip.nodes.size() - 1);
    const std::size_t stopCount = bounds.size() - 2;

    // 逐段：命中路况的段重新求最短路，其余直接复用上一版的节点序列
    std::vector<std::vector<std::string> > legPaths;
    for (std::size_t k = 0; k + 1 < bounds.size(); ++k) {
        const std::size_t b = bounds[k];
        const std::size_t e = bounds[k + 1];

        bool affected = false;
        for (std::size_t i = b + 1; i <= e && !affected; ++i) {
            for (const TrafficChange& c : report.changes) {
                if (c.increaseRatio >= thresholdRatio
                    && c.fromId == prevTrip.nodes[i - 1] && c.toId == prevTrip.nodes[i]) {
                    affected = true;
                    break;
                }
            }
        }

        std::vector<std::string> path;
        if (affected && prevTrip.nodes[b] != prevTrip.nodes[e]) {
            const PathResult r = shortestPath(graph, prevTrip.nodes[b], prevTrip.nodes[e], weight);
            if (!r.found) {
                return false;   // 该段已不可达
            }
            path = r.nodes;
        } else {
            path.assign(prevTrip.nodes.begin() + static_cast<std::ptrdiff_t>(b),
                        prevTrip.nodes.begin() + static_cast<std::ptrdiff_t>(e) + 1);
        }
        legPaths.push_back(path);
    }

    // 沿各段重建这一趟（停靠顺序不变）
    out = Trip();
    out.loadKg = prevTrip.loadKg;
    double elapsed = static_cast<double>(startTimeMin);
    double load = prevTrip.loadKg;

    out.nodes.push_back(prevTrip.nodes.front());
    out.nodeArrivalMin.push_back(startTimeMin);
    out.nodeIsStop.push_back(false);

    for (std::size_t k = 0; k < legPaths.size(); ++k) {
        const std::vector<std::string>& path = legPaths[k];
        const bool endpointIsStop = (k < stopCount);

        for (std::size_t i = 1; i < path.size(); ++i) {
            const Edge* edge = graph.findEdge(path[i - 1], path[i]);
            if (edge == nullptr) {
                return false;
            }
            elapsed += edge->timeMin;
            out.totalDistanceKm += edge->distanceKm;
            out.totalCostYuan += edge->costYuan;
            out.nodes.push_back(path[i]);
            out.nodeArrivalMin.push_back(static_cast<int>(std::lround(elapsed)));
            out.nodeIsStop.push_back(endpointIsStop && (i + 1 == path.size()));
        }

        if (!endpointIsStop) {
            continue;
        }
        const Candidate* chosen = findCandidate(pool, path.back());
        if (chosen == nullptr) {
            return false;
        }
        const Stop stop = makeStop(*chosen, elapsed, load);
        out.nodeArrivalMin[out.nodeArrivalMin.size() - 1] = stop.arrivalMin;
        out.stops.push_back(stop);
    }

    out.endNodeId = prevTrip.endNodeId;
    endTimeMin = elapsed;
    return true;
}

RoutePlan replanIncremental(const LogisticsGraph& graph,
                            const Vehicle& vehicle,
                            const std::vector<Order>& remainingOrders,
                            const RoutePlan& previous,
                            int currentTimeMin,
                            WeightType weight,
                            const TrafficReport& report,
                            double thresholdRatio,
                            const std::vector<OnboardItem>& onboard,
                            bool* usedIncremental,
                            const std::map<std::string, double>& initialStock) {
    if (usedIncremental != nullptr) {
        *usedIncremental = false;
    }
    const std::string startPos =
        previous.nodes.empty() ? vehicle.startNodeId : previous.nodes.front();

    // 适用范围：序列完整的多趟（或单趟）剩余路线。
    // **逐趟增量**：每一趟各自只重算受影响的 leg，互不影响；
    // 任何一趟增量失败（不可达/序列不全）就整体退回全量重算。
    // （此前只支持"整条剩余路线是单趟"，于是默认 6 趟方案下只有最后一趟用得上，
    //   其余一律全量——审计认为这实质等于没实现。）
    if (previous.status != PlanStatus::Ok || previous.trips.empty()
        || previous.nodes.size() < 2
        || previous.nodes.back() != vehicle.startNodeId) {
        return replan(graph, vehicle, remainingOrders, startPos, currentTimeMin,
                      weight, onboard, initialStock);
    }

    const std::vector<Candidate> pool = buildCandidates(remainingOrders);

    // 子网络 → 该簇中转站（与多趟规划同源），用于逐趟重新推导寄存
    std::map<int, std::string> transitBySub;
    for (const Node& n : graph.nodes()) {
        if (n.type == NodeType::Transit && n.subNetworkId != 0) {
            transitBySub[n.subNetworkId] = n.id;
        }
    }

    RoutePlan plan;
    double elapsed = static_cast<double>(currentTimeMin);
    for (const Trip& prevTrip : previous.trips) {
        Trip rebuilt;
        double endTime = elapsed;
        if (!replanOneTrip(graph, pool, prevTrip, report, thresholdRatio, weight,
                           static_cast<int>(std::lround(elapsed)), rebuilt, endTime)) {
            return replan(graph, vehicle, remainingOrders, startPos, currentTimeMin,
                          weight, onboard, initialStock);
        }
        // **每一趟仍要"出仓满载"**。增量重规划逐趟重建时只复制 loadKg，会**丢掉缓冲货**
        // （实测症状：一次路况重规划之后，车回仓库只装订单货就出发，站里再也攒不到货）。
        // 这里**重新推导**而不是从 prevTrip 复制：受影响的 leg 可能已被重算，顺路关系
        // 随之改变。规则与 multiTripPlanImpl 完全一致——只有"从仓库装货出发"的趟才计缓冲。
        if (!rebuilt.nodes.empty() && rebuilt.nodes.front() == vehicle.startNodeId
            && rebuilt.loadKg > 1e-9) {
            fillBufferFromDepot(vehicle, rebuilt.loadKg, rebuilt);
            TransitOp banked;
            if (bankBufferEnRoute(graph, tripHub(graph, transitBySub, rebuilt),
                                  vehicle.startNodeId, weight, rebuilt, &banked)) {
                plan.transitOps.push_back(banked);
            }
        }
        plan.trips.push_back(rebuilt);
        elapsed = endTime;
    }

    // 库存快照必须跟着走，否则界面与后续规划看到的库存凭空归零。
    // 期末值按"期初 + 本计划全部操作"重算，保证守恒（规格 I3）在这条路径上也成立。
    plan.transitStock = previous.transitStock;
    for (TransitStock& st : plan.transitStock) {
        st.finalKg = st.initialKg;
        for (const TransitOp& op : plan.transitOps) {
            if (op.nodeId == st.nodeId) {
                st.finalKg += op.kgDelta;
            }
        }
    }

    if (usedIncremental != nullptr) {
        *usedIncremental = true;
    }
    sortTransitOpsByTime(plan.transitOps);
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

std::size_t nodeIndexAtTime(const RoutePlan& plan, int timeMin) {
    if (plan.nodes.empty()) {
        return 0;   // 没有节点时下标无意义，按契约返回 0（调用方需自行判空）
    }

    // **语义 = 「半路显示所在段的起点」**：取最大的 i 使 nodeArrivalMin[i] <= timeMin。
    // 车在 i -> i+1 途中时，最后一个已到达的节点就是 i，显示 i 即段的起点。
    // 不依赖"到达时刻单调不减"这一前提（虽然它成立），全表扫描取**最后一个**满足者，
    // 这样即使上游给了非单调的序列，返回值仍严格符合上面的字面定义。
    const std::size_t n = plan.nodeArrivalMin.size() < plan.nodes.size()
                              ? plan.nodeArrivalMin.size()
                              : plan.nodes.size();
    std::size_t best = 0;
    bool found = false;
    for (std::size_t i = 0; i < n; ++i) {
        if (plan.nodeArrivalMin[i] <= timeMin) {
            best = i;
            found = true;
        }
    }
    if (!found) {
        return 0;   // 早于首节点到达时刻：车还在起点
    }
    // 下标一律 clamp 到 [0, nodes.size()-1]
    if (best >= plan.nodes.size()) {
        best = plan.nodes.size() - 1;
    }
    return best;
}

RoutePlan planRoute(const LogisticsGraph& graph,
                    const Vehicle& vehicle,
                    const std::vector<Order>& orders,
                    WeightType weight,
                    const std::map<std::string, double>& initialStock) {
    return replan(graph, vehicle, orders, vehicle.startNodeId,
                  vehicle.departTimeMin, weight, std::vector<OnboardItem>(), initialStock);
}

InsertResult insertUrgentOrder(const LogisticsGraph& graph,
                               const Vehicle& vehicle,
                               const std::vector<Order>& remainingOrders,
                               const Order& newOrder,
                               const std::string& currentPositionId,
                               int currentTimeMin,
                               WeightType weight,
                               const std::vector<OnboardItem>& onboard,
                               double carBufferKg,
                               const std::map<std::string, double>& initialStock) {
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

    // ---- 候选 A：现状（独立紧急趟**先回仓库装货**）----
    const RoutePlan planA = replan(graph, vehicle, all, currentPositionId, currentTimeMin,
                                   weight, onboard, initialStock);

    // ---- 候选 B：用「车上缓冲 + 该紧急点上属站的库存」就地满足 ----
    //
    // 可用条件（规格 §4）：
    //   avail >= d               -> 取货点 = 车上（不新增节点）
    //   否则 H 非空且库存够补足（avail + stock[H] >= d） -> 取货点 = H
    //   否则候选 B 不可用，直接退回 A（连参与取优的资格都没有）。
    // 只换**取货点**：紧急批仍独立成趟且排在所有普通停靠点之前（E3 优先性不动）。
    RoutePlan planB;
    bool        bUsable = false;
    std::string bPickup;      // 空串 = 车上已有
    std::string bStation;     // 从哪个站取（空 = 不从站取）
    double      bTakeKg = 0.0;
    double      supplyDemandKg = 0.0;   // 该紧急停靠点的（合并后）需求
    {
        std::map<int, std::string> transitBySub;
        for (const Node& n : graph.nodes()) {
            if (n.type == NodeType::Transit && n.subNetworkId != 0) {
                transitBySub[n.subNetworkId] = n.id;
            }
        }
        const std::vector<Candidate> allCandidates = buildCandidates(all);
        // **用"停靠点的合并需求"，不用"这一张订单的需求"**。
        // 规格 §4 写的是 `d = 紧急单需求`，但那默认了该配送点只有这一张紧急单；
        // 若同点还有普通订单，它们会与紧急单合并成同一个停靠点，此时按 10kg 去凑
        // 会出现"车上只有 10kg 却把这个停靠点的 70kg 送掉"——正是 P25 那类
        // "车送它没装的货"。按停靠点需求算，物理故事才自洽。
        const double d = candidateDemand(allCandidates, inserted.nodeId);
        supplyDemandKg = d;
        const std::string hub = hubOfNode(graph, transitBySub, inserted.nodeId);
        const double avail = (carBufferKg > 0.0) ? carBufferKg : 0.0;
        const double stock = stockOf(initialStock, hub);

        if (d > 1e-9 && avail >= d - 1e-9) {
            bUsable = true;                      // 车上缓冲足够，不新增节点
        } else if (d > 1e-9 && !hub.empty() && stock > 1e-9 && avail + stock >= d - 1e-9) {
            bUsable = true;
            bPickup = hub;
            bStation = hub;
            bTakeKg = d - avail;
        }

        // 只有"总需求会走多趟路径"时 B 才有意义：总量不超载时单趟路径本来就不回仓库。
        if (bUsable && totalDemand(allCandidates) > vehicle.capacityKg + 1e-9) {
            UrgentSupply supply;
            supply.nodeId = inserted.nodeId;
            supply.pickup = bPickup;
            planB = multiTripPlan(graph, vehicle, allCandidates, currentPositionId,
                                  currentTimeMin, weight, onboard, initialStock, supply);
            bUsable = (planB.status == PlanStatus::Ok);
        } else {
            bUsable = false;
        }
    }

    // ---- 取优：「绝不更差」必须是**构造保证**，不能靠"顺路所以零成本"的推理 ----
    //   penalty 更差 -> 一票否决，选 A
    //   否则目标值更小者胜；目标值相同再看趟数；再平选 A（保守）
    bool chooseB = false;
    if (bUsable) {
        const double targetA = (weight == WeightType::Distance) ? planA.totalDistanceKm
                             : (weight == WeightType::Time)     ? planA.totalTimeMin
                                                                : planA.totalCostYuan;
        const double targetB = (weight == WeightType::Distance) ? planB.totalDistanceKm
                             : (weight == WeightType::Time)     ? planB.totalTimeMin
                                                                : planB.totalCostYuan;
        if (planB.totalPenaltyMin > planA.totalPenaltyMin) {
            chooseB = false;                                          // penalty 一票否决
        } else if (targetB < targetA - 1e-9) {
            chooseB = true;
        } else if (std::fabs(targetB - targetA) <= 1e-9
                   && planB.trips.size() < planA.trips.size()) {
            chooseB = true;
        }
    }

    result.plan = chooseB ? planB : planA;

    // 守恒的**消费端**：真的从站里取用了才记账，且 atMin 实读自
    // "服务该紧急单的那一趟"的节点序列（读不到就不记——守卫 G3 要求可追溯）。
    if (chooseB) {
        bool stationDrawn = false;
        if (!bStation.empty()) {
            stationDrawn = recordStationDraw(result.plan, bStation, inserted.nodeId, bTakeKg);
        }
        if (stationDrawn) {
            result.stationUsed = bStation;
            result.stationUsedKg = bTakeKg;
        }
        // 车上缓冲的净消耗 = 该停靠点需求 − 真的从站里取到的量。
        // 把这两个数回报给调用方，界面才能把"车真的取了货"落成物理事实，不用自己猜。
        const double fromStation = stationDrawn ? bTakeKg : 0.0;
        result.carBufferUsedKg =
            (supplyDemandKg > fromStation) ? (supplyDemandKg - fromStation) : 0.0;
    }
    sortTransitOpsByTime(result.plan.transitOps);
    return result;
}


} // namespace logistics
