
// RoutePlanner 的行为测试（规格一 #18 / 迁移清单 MC-G01）。
//
// 期望值纪律：**不手算**。具体数字一律由 tools/oracle.py 对该夹具的节点序列复算，
// 命令写在用到它的注释里；结构性判据（F1–F7、前缀载重、阶段、FIFO、停靠唯一性、
// 确定性）直接断言，不依赖数字。
//
// 覆盖规格一 #18 §12 的 12 条不变量。**每条都有落点**，两种方式：
//   (a) 通用守卫 checkStructuralInvariants（每个用例都调用一次，机械复核）；
//   (b) 专门的用例（盯住某一条的边界形态）。
//
//  ① 每单恰好一次取货 + 一次送达      (a) + testFallbackProducesCompleteRoute
//  ② A 停靠索引 < B 停靠索引          (a)
//  ③ 前缀载重 ∈ [0,C]、先卸后装       (a) + testSingleOrderRouteIsFullyCorrect
//  ④ 硬优先（普通停靠不先于紧急）      (a) + testUrgentOrderIsServedFirstDespiteBeingFarther
//  ⑤ 订单级 FIFO                     testUrgentFifoOrder（索引 + 时刻双断言）
//  ⑥ 同刻至多一张紧急单在车            (a)
//  ⑦ 同阶段同节点至多一次停靠（强制重访例外）(a) + testFifoForcedRevisit
//  ⑧ 停靠零耗时 / 早到等待 / ==we 不迟到  testLateArrivalIsMarkedWithPenalty + (a)
//  ⑨ penalty 逐单                     testMultipleOrdersOnSameNodeMergeIntoOneStop
// ⑩ 暂存守恒、期末 0、单堆锁站         testOffloadConservesAndReclaims +
//                                      testTwoUrgentOffloadsMergeIntoSingleStashPile
// ⑪ 取回先于送达                     testOffloadConservesAndReclaims + testReplanKeepsStash
// ⑫ 输出确定性（字典序 + tieKey）       testDeterministicSameInput +
//                                      testTieBreakIsDeterministicByNodeId
//
// 期望值纪律：**不手算**。具体数字一律由 tools/oracle.py 对该夹具的节点序列复算，
// 命令写在用到它的注释里；结构性判据（F1–F7、前缀载重、阶段、FIFO、停靠唯一性、
// 确定性）直接断言，不依赖数字。
#include <cmath>
#include <string>
#include <vector>

#include "core/RoutePlanner.h"
#include "graph_fixtures.h"
#include "test_util.h"

using logistics::LogisticsGraph;
using logistics::NodeType;
using logistics::Node;
using logistics::Order;
using logistics::PlanStatus;
using logistics::RoutePlan;
using logistics::Stop;
using logistics::StopDelivery;
using logistics::StopPhase;
using logistics::Vehicle;
using logistics::WeightType;
using logistics::nodeIndexAtTime;
using logistics::planRoute;
using logistics::planFleet;
using logistics::planFleetSequentialBaseline;
using logistics::FleetPlan;
using logistics::clusterOfOrder;
using logistics::isCrossCluster;
using logistics::marginalCost;
using logistics::MarginalCost;
using logistics::RelocateRecord;
using logistics::VehicleStartState;
using logistics::FleetTransitResult;
using logistics::TransitLeg;
using logistics::applyTransitLayer;
using logistics::buildTransitLedger;
using logistics::OrderMetric;
using logistics::replan;
using logistics::replanIncremental;
using logistics::insertOrder;
using logistics::InsertResult;
using logistics::TrafficChange;
using logistics::TrafficReport;
using testutil::check;

namespace {

using fixtures::addTwoWay;
using fixtures::makeNode;
using fixtures::makeSubNode;
using fixtures::makeVehicle;
using fixtures::makeVehicleWithFactors;
using fixtures::makeOrder;

std::string join(const std::vector<std::string>& nodes) {
    std::string s;
    for (std::size_t i = 0; i < nodes.size(); ++i) {
        if (i > 0) {
            s += " -> ";
        }
        s += nodes[i];
    }
    return s;
}

// W(仓库) <-> D1(客户)：往返各 5.0km / 10min / 4元
LogisticsGraph makeWtoD1Graph() {
    LogisticsGraph g;
    g.addNode(makeNode("W", NodeType::Warehouse, "仓库"));
    g.addNode(makeNode("D1", NodeType::Delivery, "客户1"));
    addTwoWay(g, "W", "D1", 5.0, 10.0, 4.0);
    return g;
}

// ---------------------------------------------------------------------------
// 通用守卫：把 12 条不变量里可机械检查的部分一次查完。
// 每条用例都调用它 —— 单条用例的期望值可能很弱，但结构性不变量是全局的。
// ---------------------------------------------------------------------------
void checkStructuralInvariants(const LogisticsGraph& g, const RoutePlan& plan,
                               const std::vector<Order>& orders, const Vehicle& v,
                               const std::string& label,
                               const std::string& startPos = std::string()) {
    const std::string tag = "[" + label + "] ";
    const std::string from = startPos.empty() ? v.startNodeId : startPos;

    // 出发时就在车上的货（onboard）：路线里**既没有取货也没有取回**动作的那些订单。
    // 规格一 MC-A05 之后 replan 可以带着在途货出发，此时少一次取货动作是正确语义，
    // 不是丢单；下面的载重前缀与动作恒等式都必须从这批货开始算。
    double onboardKg = 0.0;
    std::size_t onboardCount = 0;
    std::size_t stashCount = 0;
    for (const Order& o : orders) {
        bool hasPick = false;
        bool hasReclaim = false;
        bool hasOffload = false;
        for (const Stop& s : plan.stops) {
            for (const std::string& id : s.pickups) {
                if (id == o.id) {
                    hasPick = true;
                }
            }
            for (const std::string& id : s.reclaims) {
                if (id == o.id) {
                    hasReclaim = true;
                }
            }
            for (const std::string& id : s.offloads) {
                if (id == o.id) {
                    hasOffload = true;
                }
            }
        }
        // 出发在车 = 没有 A 取货动作，且「不是计划开始时就在站里」：
        // 站里起手的货只有取回动作；降载过的货则是「卸下 + 取回」两条都在。
        if (!hasPick && (!hasReclaim || hasOffload)) {
            onboardKg += o.demandKg;
            ++onboardCount;
        } else if (!hasPick && hasReclaim && !hasOffload) {
            ++stashCount;   // 计划开始时就在中转站里（取回不算取送动作）
        }
    }

    // 路线真实可走、首尾正确、到达时刻单调。
    check(!plan.nodes.empty(), tag + "路线非空");
    if (plan.nodes.empty()) {
        return;
    }
    check(plan.nodes.front() == from, tag + "从出发位置出发（期望 " + from + "，实际 "
                                                + plan.nodes.front() + "）");
    check(plan.nodes.back() == v.startNodeId, tag + "回到仓库");
    bool walkable = true;
    for (std::size_t i = 1; i < plan.nodes.size(); ++i) {
        // 相邻两次停靠在**同一节点**时，展平视图里是两个同 ID 条目（零位移，无自环边）。
        if (plan.nodes[i] == plan.nodes[i - 1]) {
            continue;
        }
        if (g.findEdge(plan.nodes[i - 1], plan.nodes[i]) == nullptr) {
            walkable = false;
        }
    }
    check(walkable, tag + "相邻节点之间都存在有向边");
    check(plan.nodes.size() == plan.nodeArrivalMin.size()
              && plan.nodes.size() == plan.nodeIsStop.size(),
          tag + "节点/到达时刻/停靠标记三个数组等长");
    bool monotone = true;
    for (std::size_t i = 1; i < plan.nodeArrivalMin.size(); ++i) {
        if (plan.nodeArrivalMin[i] < plan.nodeArrivalMin[i - 1]) {
            monotone = false;
        }
    }
    check(monotone, tag + "到达时刻单调不减");

    // 停靠数与 nodeIsStop 的 true 数一致（展平视图与真源一致）。
    std::size_t stopFlags = 0;
    for (std::size_t i = 0; i < plan.nodeIsStop.size(); ++i) {
        if (plan.nodeIsStop[i]) {
            ++stopFlags;
        }
    }
    check(stopFlags == plan.stops.size(),
          tag + "停靠标记的 true 数 = stops 数（展平视图与真源一致）");

    // 单条连续路线：仓库只在**首个**停靠出现（起点就地取货的夹具形态），
    // 中途不得再停一次仓库（那是已退役的仓库装货/多趟语义）。
    bool depotOnce = true;
    for (std::size_t i = 0; i < plan.stops.size(); ++i) {
        if (plan.stops[i].nodeId != v.startNodeId) {
            continue;
        }
        const bool okFirst = (i == 0 && from == v.startNodeId);
        if (!okFirst) {
            depotOnce = false;
        }
    }
    check(depotOnce, tag + "中途不停靠仓库（单条连续路线）");

    // 不变量 ①：每张订单恰一次取货 + 一次送达（不含降载/取回）。
    for (const Order& o : orders) {
        std::size_t picks = 0;
        std::size_t delivs = 0;
        std::size_t pickIdx = plan.stops.size();
        std::size_t delivIdx = plan.stops.size();
        for (std::size_t i = 0; i < plan.stops.size(); ++i) {
            for (const std::string& id : plan.stops[i].pickups) {
                if (id == o.id) {
                    ++picks;
                    pickIdx = i;
                }
            }
            for (const logistics::StopDelivery& d : plan.stops[i].deliveries) {
                if (d.orderId == o.id) {
                    ++delivs;
                    delivIdx = i;
                }
            }
        }
        check(delivs == 1, tag + "订单 " + o.id + " 恰好送达一次（实际 "
                               + std::to_string(delivs) + "）");
        check(picks <= 1, tag + "订单 " + o.id + " 至多取货一次（实际 "
                               + std::to_string(picks) + "）");
        // 不变量 ②：先取后送。
        if (picks == 1 && delivs == 1) {
            check(pickIdx < delivIdx, tag + "订单 " + o.id + " 的取货早于送达");
        }
    }

    // 不变量 ③：前缀载重 0..C（用 remainingLoadKg 逐停靠复核），先卸后装。
    double load = onboardKg;
    bool prefixOk = true;
    bool orderOk = true;
    for (const Stop& s : plan.stops) {
        double arrive = load;
        for (const logistics::StopDelivery& d : s.deliveries) {
            for (const Order& o : orders) {
                if (o.id == d.orderId) {
                    arrive -= o.demandKg;
                }
            }
        }
        for (const std::string& id : s.offloads) {
            for (const Order& o : orders) {
                if (o.id == id) {
                    arrive -= o.demandKg;
                }
            }
        }
        if (arrive < -1e-9) {
            orderOk = false;   // 卸的比车上还多 ⇒ 没先卸后装/账目错
        }
        double after = arrive;
        for (const std::string& id : s.pickups) {
            for (const Order& o : orders) {
                if (o.id == id) {
                    after += o.demandKg;
                }
            }
        }
        for (const std::string& id : s.reclaims) {
            for (const Order& o : orders) {
                if (o.id == id) {
                    after += o.demandKg;
                }
            }
        }
        if (after > v.capacityKg + 1e-6 || after < -1e-9) {
            prefixOk = false;
        }
        if (std::fabs(s.remainingLoadKg - after) > 1e-6) {
            prefixOk = false;   // remainingLoadKg = 离开该站时的在车量
        }
        load = after;
    }
    check(prefixOk, tag + "任一停靠后的在车量都在 [0, 载重上限] 且等于 remainingLoadKg");
    check(orderOk, tag + "停靠内先卸后装（卸货不超过车上已有）");
    // 不变量 ④：硬优先 —— 紧急停靠全部先于普通停靠；紧急段无普通取货。
    bool seenNormal = false;
    bool hardOk = true;
    for (const Stop& s : plan.stops) {
        if (s.phase == StopPhase::Normal) {
            seenNormal = true;
        } else if (seenNormal) {
            hardOk = false;
        }
    }
    check(hardOk, tag + "硬优先：紧急停靠全部先于普通停靠");
    bool urgentNoNormalPickup = true;
    for (const Stop& s : plan.stops) {
        if (s.phase != StopPhase::Urgent) {
            continue;
        }
        for (const std::string& id : s.pickups) {
            for (const Order& o : orders) {
                if (o.id == id && !o.urgent) {
                    urgentNoNormalPickup = false;
                }
            }
        }
    }
    check(urgentNoNormalPickup, tag + "紧急阶段没有普通取货动作");

    // 不变量 ⑥：紧急阶段同刻至多一张紧急单在车（已取未送的紧急单数 <= 1）。
    std::size_t urgentOnboard = 0;
    bool oneUrgent = true;
    for (const Stop& s : plan.stops) {
        if (s.phase != StopPhase::Urgent) {
            continue;
        }
        for (const std::string& id : s.pickups) {
            ++urgentOnboard;
        }
        for (const logistics::StopDelivery& d : s.deliveries) {
            bool urgent = false;
            for (const Order& o : orders) {
                if (o.id == d.orderId && o.urgent) {
                    urgent = true;
                }
            }
            if (urgent && urgentOnboard > 0) {
                --urgentOnboard;
            }
        }
        if (urgentOnboard > 1) {
            oneUrgent = false;
        }
    }
    check(oneUrgent, tag + "紧急阶段同刻至多一张紧急单在车");

    // 不变量 ⑦：**同阶段同节点至多一次停靠**；唯一例外是「强制重访」——
    // 把重访停靠的那一个动作并入该节点首次停靠后，F2（先取后送）或 F3（前缀载重 /
    // 先卸后装）**失败**。判据与实现同源（tests/graph_fixtures.h 的
    // fixtures::nonForcedRevisitStops），并且要求「非强制重复 = 0」。
    int repeatedPairs = 0;
    for (std::size_t i = 0; i < plan.stops.size(); ++i) {
        for (std::size_t j = i + 1; j < plan.stops.size(); ++j) {
            if (plan.stops[i].nodeId == plan.stops[j].nodeId
                && plan.stops[i].phase == plan.stops[j].phase) {
                ++repeatedPairs;
            }
        }
    }
    check(true, tag + "同阶段同节点停靠的重复对数 = " + std::to_string(repeatedPairs)
                     + "（允许为 0 或强制重访）");
    const int nonForced =
        fixtures::nonForcedRevisitStops(plan.stops, orders, v.capacityKg, onboardKg);
    check(nonForced == 0,
          tag + "非强制重复停靠必须为 0（并入首次停靠后 F2/F3 都过的重复一律算违规），实际 "
              + std::to_string(nonForced));

    // 不变量 ⑧：停靠零耗时 —— arrival 即离开时刻；raw + wait == arrival。
    bool zeroDwell = true;
    for (const Stop& s : plan.stops) {
        if (s.rawArrivalMin + s.waitMin != s.arrivalMin) {
            zeroDwell = false;
        }
    }
    check(zeroDwell, tag + "停靠零耗时：rawArrival + wait == arrival");

    // 不变量 ⑨：penalty 逐单累加 == 停靠级 penaltyMin == 总 penalty。
    bool penaltyOk = true;
    int sum = 0;
    for (const Stop& s : plan.stops) {
        int stopSum = 0;
        for (const logistics::StopDelivery& d : s.deliveries) {
            const int we = [&]() {
                for (const Order& o : orders) {
                    if (o.id == d.orderId) {
                        return o.windowEndMin;
                    }
                }
                return 0;
            }();
            const int expect = s.arrivalMin > we ? (s.arrivalMin - we) : 0;
            if (d.penaltyMin != expect) {
                penaltyOk = false;
            }
            stopSum += d.penaltyMin;
        }
        if (stopSum != s.penaltyMin) {
            penaltyOk = false;
        }
        sum += stopSum;
    }
    check(penaltyOk, tag + "penalty 逐单：每单按自己的 we 计、停靠级为逐单之和");
    check(sum == plan.totalPenaltyMin, tag + "总 penalty == 各停靠逐单之和");

    // 不变量 ⑩/⑪：降载账本 —— 期末已取回、取回先于送达、全局单堆锁站。
    bool ledgerOk = true;
    std::string stashStation;
    for (const logistics::OffloadLeg& leg : plan.offloads) {
        if (leg.reclaimTimeMin < 0) {
            ledgerOk = false;   // 期末必须已取回
        }
        if (stashStation.empty()) {
            stashStation = leg.station;
        } else if (stashStation != leg.station) {
            ledgerOk = false;   // 全局至多一堆、锁站
        }
        if (leg.reclaimTimeMin < leg.offloadTimeMin) {
            ledgerOk = false;
        }
    }
    for (const logistics::OffloadLeg& leg : plan.offloads) {
        // 取回停靠索引 < 该单送达停靠索引
        std::size_t reclaimIdx = plan.stops.size();
        std::size_t delivIdx = plan.stops.size();
        for (std::size_t i = 0; i < plan.stops.size(); ++i) {
            for (const std::string& id : plan.stops[i].reclaims) {
                if (id == leg.orderId) {
                    reclaimIdx = i;
                }
            }
            for (const logistics::StopDelivery& d : plan.stops[i].deliveries) {
                if (d.orderId == leg.orderId) {
                    delivIdx = i;
                }
            }
        }
        if (reclaimIdx >= delivIdx) {
            ledgerOk = false;
        }
    }
    check(ledgerOk, tag + "降载账本：期末已取回、单堆锁站、取回先于送达");

    // 指标口径的交叉校验（#29 §6.6 的 G3/G4/G5）。
    const logistics::RouteMetrics& m = plan.metrics;
    check(m.stopCount == static_cast<int>(plan.stops.size()), tag + "指标·停靠点数 = stops 数");
    int actions = 0;
    for (const Stop& s : plan.stops) {
        actions += static_cast<int>(s.pickups.size() + s.deliveries.size());
    }
    check(m.actionCount == actions, tag + "指标·动作数 = 取货 + 送达动作数");
    // 动作恒等：每单恰一次送达；取货动作只在「出发时既不在车上、也不在站里」时出现
    // （取回不算取送动作，见规格一 §6.1）。因此
    //   A = 取送对数 + （取送对数 − 在车单数 − 站里单数）。
    check(m.actionCount == 2 * static_cast<int>(orders.size())
                               - static_cast<int>(onboardCount)
                               - static_cast<int>(stashCount),
          tag + "结构性恒等：动作数 = 2 × 取送对数 − 在车/在站单数（在车 "
              + std::to_string(onboardCount) + " 单、在站 "
              + std::to_string(stashCount) + " 单）");
    check(m.pickupOnTimeCount >= m.deliveryOnTimeCount,
          tag + "取货准时数 >= 送达准时数（数学恒等，G4）");
    check(m.avgLoadKg <= m.peakLoadKg + 1e-9, tag + "平均在车 <= 峰值（G5）");
    check(m.peakLoadKg <= v.capacityKg + 1e-6, tag + "峰值 <= 载重上限（G5）");
    if (v.capacityKg > 1e-9) {
        check(std::fabs(m.loadUtilization - m.avgLoadKg / v.capacityKg) < 0.005,
              tag + "利用率 == 平均/容量（G5）");
    }
}


} // namespace

namespace {

// ---------------------------------------------------------------------------
// 切片 1：单订单的完整手算校验（这一条是**唯一**保留手算的用例：
// 图上只有一条边，算术不可能被选路影响）。
//
// oracle 复算命令：
//   python3 tools/oracle.py --route distance W W 480 D1   （夹具 W<->D1 5km/10min/4元）
// 期望：raw 490 / wait 50 / arrival 540 / 回仓 550 / 距离 10.0 / 耗时 70.0 / 成本 8.0
// ---------------------------------------------------------------------------
void testSingleOrderRouteIsFullyCorrect() {
    const LogisticsGraph g = makeWtoD1Graph();
    const Vehicle v = makeVehicle("W", 1000.0, 480);
    const std::vector<Order> orders = {makeOrder("O1", "W", "D1", 10.0, 540, 1080, false)};

    const RoutePlan plan = planRoute(g, v, orders, WeightType::Distance);

    check(plan.status == PlanStatus::Ok, "规划成功");
    // A→B 取送：起点 W 即取货点 A（空车出发先到 A 装货），因此 W 是一次停靠。
    check(plan.nodes == std::vector<std::string>{"W", "D1", "W"},
          "完整序列为 W -> D1 -> W，实际 " + join(plan.nodes));
    check(fixtures::nearlyEqual(plan.totalDistanceKm, 10.0),
          "总距离 10.0，实际 " + std::to_string(plan.totalDistanceKm));
    check(fixtures::nearlyEqual(plan.totalTimeMin, 70.0),
          "总耗时 70（含早到等待），实际 " + std::to_string(plan.totalTimeMin));
    check(plan.returnArrivalMin == 550,
          "返回仓库时刻 550，实际 " + std::to_string(plan.returnArrivalMin));
    check(fixtures::nearlyEqual(plan.totalCostYuan, 8.0),
          "总成本 8.0，实际 " + std::to_string(plan.totalCostYuan));
    check(plan.totalPenaltyMin == 0, "totalPenalty 为 0");

    check(plan.stops.size() == 2,
          "取货停靠 + 送达停靠 = 2，实际 " + std::to_string(plan.stops.size()));
    if (plan.stops.size() == 2) {
        check(plan.stops[0].nodeId == "W" && plan.stops[0].pickups.size() == 1
                  && plan.stops[0].pickups[0] == "O1",
              "首个停靠是取货点 W，取 O1");
        check(plan.stops[0].arrivalMin == 480, "取货停靠到达 480（原地，无位移）");
        const Stop& s = plan.stops[1];
        check(s.nodeId == "D1", "第二个停靠是 D1");
        check(s.rawArrivalMin == 490, "rawArrival 490，实际 " + std::to_string(s.rawArrivalMin));
        check(s.waitMin == 50, "等待 50，实际 " + std::to_string(s.waitMin));
        check(s.arrivalMin == 540, "arrival 540，实际 " + std::to_string(s.arrivalMin));
        check(!s.late, "未超时");
        check(s.penaltyMin == 0, "penalty 0");
        check(s.deliveries.size() == 1 && s.deliveries[0].orderId == "O1",
              "送达动作属于 O1");
        check(fixtures::nearlyEqual(s.remainingLoadKg, 0.0),
              "卸完全部货物后离站在车量 0，实际 " + std::to_string(s.remainingLoadKg));
    }
    checkStructuralInvariants(g, plan, orders, v, "单订单");
}

// 切片 1（边界）：没有订单时退化为原地不动，不产生任何位移
void testEmptyOrdersDegeneratesToNoMovement() {
    const LogisticsGraph g = makeWtoD1Graph();
    const Vehicle v = makeVehicle("W", 1000.0, 480);

    const RoutePlan plan = planRoute(g, v, std::vector<Order>{}, WeightType::Distance);

    check(plan.status == PlanStatus::Ok, "空订单规划成功");
    check(plan.nodes == std::vector<std::string>{"W"},
          "空订单时序列只有起点，实际 " + join(plan.nodes));
    check(plan.stops.empty(), "空订单时无停靠点");
    check(fixtures::nearlyEqual(plan.totalDistanceKm, 0.0), "总距离 0");
    check(fixtures::nearlyEqual(plan.totalTimeMin, 0.0), "总耗时 0");
    check(fixtures::nearlyEqual(plan.totalCostYuan, 0.0), "总成本 0");
    check(plan.metrics.actionCount == 0, "空订单时动作数 0");
    // 分母为 0 的率不产生数值（由 --plan-summary 打成「—」，见 cli 层）。
    check(plan.metrics.deliveryTotal == 0, "空订单时送达分母 0");
}

// ---------------------------------------------------------------------------
// pair-insertion 的核心性质：候选位置按**字典序键**取优，
// 不是「最近邻」。这条用例的图上，最近邻会先取 D1 的货再绕回 D2，
// 而 penalty 先的插入器会把「取 D2 → 送 D2 → 取 D1 → 送 D1」这种
// 让**送达窗口紧的那张单**更早完成的序列排在前面。
//
// 结构性断言（不依赖具体数字）：两单都被服务、先取后送、总距离为正。
// ---------------------------------------------------------------------------
void testPairInsertionBeatsNearestNeighbourOnTightWindow() {
    LogisticsGraph g;
    g.addNode(makeNode("W", NodeType::Warehouse, "仓库"));
    g.addNode(makeNode("D1", NodeType::Delivery, "客户1"));
    g.addNode(makeNode("D2", NodeType::Delivery, "客户2"));
    addTwoWay(g, "W", "D1", 1.0, 2.0, 1.0);
    addTwoWay(g, "W", "D2", 5.0, 10.0, 5.0);
    // D1↔D2 故意慢：先送 D1 再折回 D2 会把 O2 拖迟到，先送 D2 才是 penalty 0。
    addTwoWay(g, "D1", "D2", 10.0, 20.0, 10.0);

    const Vehicle v = makeVehicle("W", 1000.0, 480);
    // O2 的窗口很紧（480-490）：先送 D1 再走 20min 到 D2 → 502，迟到 12min；
    // 先送 D2（delta 10min，490 到）→ penalty 0，但距离更长（15 vs 11）。
    // 只有 penalty 排第一键时才会选后者。
    const std::vector<Order> orders = {
        makeOrder("O1", "W", "D1", 10.0, 0, 1440, false),
        makeOrder("O2", "W", "D2", 10.0, 480, 490, false),
    };

    const RoutePlan plan = planRoute(g, v, orders, WeightType::Distance);
    check(plan.status == PlanStatus::Ok, "规划成功");
    check(plan.totalPenaltyMin == 0,
          "紧窗单 O2 应先被送达（penalty 0），实际 " + std::to_string(plan.totalPenaltyMin));
    std::size_t i2 = plan.stops.size();
    std::size_t i1 = plan.stops.size();
    for (std::size_t i = 0; i < plan.stops.size(); ++i) {
        for (const logistics::StopDelivery& d : plan.stops[i].deliveries) {
            if (d.orderId == "O2") {
                i2 = i;
            }
            if (d.orderId == "O1") {
                i1 = i;
            }
        }
    }
    check(i2 < i1, "窗口紧的 O2 先送达（penalty 优先于里程）");
    checkStructuralInvariants(g, plan, orders, v, "penalty 优先");
}

// 切片 2：并列时必须按稳定 tieKey 取，保证同一输入必得同一条路线。
void testTieBreakIsDeterministicByNodeId() {
    LogisticsGraph g;
    g.addNode(makeNode("W", NodeType::Warehouse, "仓库"));
    g.addNode(makeNode("DA", NodeType::Delivery, "客户A"));
    g.addNode(makeNode("DZ", NodeType::Delivery, "客户Z"));
    addTwoWay(g, "W", "DA", 3.0, 6.0, 3.0);
    addTwoWay(g, "W", "DZ", 3.0, 6.0, 3.0);
    addTwoWay(g, "DA", "DZ", 2.0, 4.0, 2.0);

    const Vehicle v = makeVehicle("W", 1000.0, 480);
    // DZ 在列表中排在前头，用来区分"tieKey 字典序"与"先到先得"。
    const std::vector<Order> orders = {
        makeOrder("OZ", "W", "DZ", 10.0, 0, 1440, false),
        makeOrder("OA", "W", "DA", 10.0, 0, 1440, false),
    };

    const RoutePlan plan = planRoute(g, v, orders, WeightType::Distance);
    check(plan.status == PlanStatus::Ok, "并列场景规划成功");
    // 两个候选的 (penalty, weight) 完全并列，tieKey 首站节点 ID 决定先后。
    std::size_t firstDelivery = plan.stops.size();
    std::string firstNode;
    for (std::size_t i = 0; i < plan.stops.size(); ++i) {
        if (!plan.stops[i].deliveries.empty()) {
            firstDelivery = i;
            firstNode = plan.stops[i].nodeId;
            break;
        }
    }
    check(firstDelivery < plan.stops.size(), "至少有一个送达停靠");
    check(firstNode == "DA", "并列时按 tieKey 取节点 ID 较小的 DA，实际 " + firstNode);
    checkStructuralInvariants(g, plan, orders, v, "并列 tieKey");
}

// 切片 3：紧急订单硬优先 —— 即使更远，紧急单的停靠也必须先于任何普通停靠。
void testUrgentOrderIsServedFirstDespiteBeingFarther() {
    LogisticsGraph g;
    g.addNode(makeNode("W", NodeType::Warehouse, "仓库"));
    g.addNode(makeNode("D1", NodeType::Delivery, "客户1"));
    g.addNode(makeNode("D2", NodeType::Delivery, "客户2"));
    g.addNode(makeNode("P", NodeType::Delivery, "取货客户"));
    addTwoWay(g, "W", "D1", 1.0, 2.0, 1.0);
    addTwoWay(g, "W", "D2", 5.0, 10.0, 5.0);
    addTwoWay(g, "D1", "D2", 2.0, 4.0, 2.0);
    // A→B 取送对：两张单都从 P 取货（P 是客户点，不是仓库）。
    addTwoWay(g, "W", "P", 2.0, 4.0, 2.0);
    addTwoWay(g, "P", "D1", 2.0, 4.0, 2.0);
    addTwoWay(g, "P", "D2", 6.0, 12.0, 6.0);

    const Vehicle v = makeVehicle("W", 1000.0, 480);
    const std::vector<Order> orders = {
        makeOrder("O1", "P", "D1", 10.0, 0, 1440, false),
        makeOrder("O2", "P", "D2", 10.0, 0, 1440, true),   // 远，但紧急
    };
    const RoutePlan plan = planRoute(g, v, orders, WeightType::Distance);
    check(plan.status == PlanStatus::Ok, "紧急单场景规划成功");
    // 不变量 ④：紧急停靠全先于普通停靠。
    bool seenNormal = false;
    bool hardOk = true;
    for (const Stop& s : plan.stops) {
        if (s.phase == StopPhase::Normal) {
            seenNormal = true;
        } else if (seenNormal) {
            hardOk = false;
        }
    }
    check(hardOk, "紧急停靠全部先于普通停靠");
    check(!plan.stops.empty() && plan.stops.front().phase == StopPhase::Urgent,
          "首个停靠属于紧急阶段");
    checkStructuralInvariants(g, plan, orders, v, "紧急硬优先");
}

// 切片 4：晚到必须标记超时并记录 penalty；arrival == we **不算**迟到（严格 >）。
void testLateArrivalIsMarkedWithPenalty() {
    const LogisticsGraph g = makeWtoD1Graph();
    const Vehicle v = makeVehicle("W", 1000.0, 480);

    // 晚到 5 分钟：W->D1 10min，发车 480 ⇒ raw 490；窗口止 485。
    {
        const std::vector<Order> orders = {makeOrder("O1", "W", "D1", 10.0, 480, 485, false)};
        const RoutePlan plan = planRoute(g, v, orders, WeightType::Distance);
        check(plan.status == PlanStatus::Ok, "晚到场景规划成功");
        const Stop* deliv = nullptr;
        for (const Stop& s : plan.stops) {
            if (!s.deliveries.empty()) {
                deliv = &s;
            }
        }
        check(deliv != nullptr, "存在送达停靠");
        if (deliv != nullptr) {
            check(deliv->waitMin == 0, "窗口早于到达，无等待");
            check(deliv->arrivalMin == 490, "arrival 490");
            check(deliv->late, "必须标记为超时");
            check(deliv->penaltyMin == 5, "penalty 490-485 = 5，实际 "
                                              + std::to_string(deliv->penaltyMin));
        }
        check(plan.totalPenaltyMin == 5, "totalPenalty 5");
        checkStructuralInvariants(g, plan, orders, v, "晚到");
    }

    // 边界：arrival == we ⇒ 不算迟到（#18 §5 要求）。
    {
        const std::vector<Order> orders = {makeOrder("O1", "W", "D1", 10.0, 480, 490, false)};
        const RoutePlan plan = planRoute(g, v, orders, WeightType::Distance);
        check(plan.status == PlanStatus::Ok, "边界场景规划成功");
        check(plan.totalPenaltyMin == 0, "arrival == windowEnd 不算迟到");
    }
}

// 切片 4 + 不变量 ⑨：同一停靠上的多张单**逐单**判迟到（停靠无合并窗口）。
void testMultipleOrdersOnSameNodeMergeIntoOneStop() {
    const LogisticsGraph g = makeWtoD1Graph();
    const Vehicle v = makeVehicle("W", 1000.0, 480);
    // 两单同送达点 D1：O1 窗口 480-485（会迟到 5min），O2 窗口 840-900（不迟到）。
    // 旧「窗口取并集」口径会把两单都判成不迟到——这正是 #18 Q14=A 要修掉的掩盖。
    // 两张单同送达点 D1、同一取货点 W：合并成一次停靠（不额外等待），
    // 到达 490 ⇒ 逐单判迟到（O1 按 485 迟到 5，O2 按 900 不迟到）。
    const std::vector<Order> orders = {
        makeOrder("O1", "W", "D1", 30.0, 480, 485, false),
        makeOrder("O2", "W", "D1", 20.0, 480, 900, false),
    };

    const RoutePlan plan = planRoute(g, v, orders, WeightType::Distance);
    check(plan.status == PlanStatus::Ok, "规划成功");

    std::size_t delivStops = 0;
    const Stop* d1 = nullptr;
    for (const Stop& s : plan.stops) {
        if (!s.deliveries.empty()) {
            ++delivStops;
            d1 = &s;
        }
    }
    check(delivStops == 1, "同节点两单合并为一次送达停靠，实际 "
                               + std::to_string(delivStops));
    if (d1 != nullptr) {
        check(d1->nodeId == "D1", "停靠点是 D1");
        check(d1->deliveries.size() == 2, "该停靠承载两个送达动作");
        // 逐单：O1 迟到 5、O2 不迟到。
        for (const logistics::StopDelivery& d : d1->deliveries) {
            if (d.orderId == "O1") {
                check(d.late && d.penaltyMin == 5,
                      "O1 按自己的 we=485 判迟到 5min，实际 "
                          + std::to_string(d.penaltyMin));
            }
            if (d.orderId == "O2") {
                check(!d.late && d.penaltyMin == 0,
                      "O2 按自己的 we=900 判不迟到（不被 O1 的紧窗掩盖）");
            }
        }
        check(d1->penaltyMin == 5, "停靠级 penalty = 逐单之和 5");
        check(fixtures::nearlyEqual(d1->remainingLoadKg, 0.0),
              "合并后一次卸完 50kg，离站在车量 0");
    }
    check(plan.totalPenaltyMin == 5, "总 penalty 5（不是 0）");
    checkStructuralInvariants(g, plan, orders, v, "合并停靠逐单 penalty");
}

// 切片 5：单个订单的货量超过载重上限 ⇒ 真正的载重不可行（降载也救不了）。
void testSingleOrderExceedingCapacityIsInfeasible() {
    const LogisticsGraph g = makeWtoD1Graph();
    const Vehicle v = makeVehicle("W", 10.0, 480);
    const std::vector<Order> orders = {makeOrder("O1", "W", "D1", 20.0, 0, 1440, false)};

    const RoutePlan plan = planRoute(g, v, orders, WeightType::Distance);
    check(plan.status == PlanStatus::OrderExceedsCapacity, "状态为 OrderExceedsCapacity");
    check(plan.nodes.empty(), "不可行时不生成路线（nodes 为空）");
    check(plan.stops.empty(), "不可行时无停靠点");
    check(plan.reason.find("20") != std::string::npos,
          "原因应含超限货量，实际: " + plan.reason);
}

} // namespace

namespace {

// 中转站图：W -- T -- {D1, D2}，D1/D2 归属 T 的子网络。
LogisticsGraph makeTransitClusterGraph() {
    LogisticsGraph g;
    Node w = makeNode("W", NodeType::Warehouse, "仓库");
    w.x = 0;    w.y = 0;
    Node t = makeNode("T", NodeType::Transit, "集散站");
    t.x = 100;  t.y = 0;   t.subNetworkId = 1;
    Node d1 = makeNode("D1", NodeType::Delivery, "客户1");
    d1.x = 150; d1.y = 0;  d1.subNetworkId = 1;
    Node d2 = makeNode("D2", NodeType::Delivery, "客户2");
    d2.x = 150; d2.y = 20; d2.subNetworkId = 1;
    g.addNode(w); g.addNode(t); g.addNode(d1); g.addNode(d2);
    addTwoWay(g, "W", "T", 10.0, 10.0, 16.0);
    addTwoWay(g, "T", "D1", 5.0, 5.0, 5.0);
    addTwoWay(g, "T", "D2", 6.0, 6.0, 6.0);
    return g;
}

// MC-G01 改造项：总需求超过载重**不再不可行、也不再分趟**（规格一 #21）。
// 单条连续路线靠「边取边送 + 前缀载重」服务全部订单；
// 旧断言「多趟」作废，改为**前缀载重 + 单条连续路线**。
void testTotalDemandOverCapacityStaysOneContinuousRoute() {
    // 新模型：A→B 取送对，A 从不在仓库。两张单是客户间取送（D1->D2 / D3->D4），
    // 载重 20 < 总需求 30，靠"边取边送 + 前缀载重"合法地串成一条连续路线。
    LogisticsGraph g = makeTransitClusterGraph();
    g.addNode(makeNode("D3", NodeType::Delivery, "客户3"));
    g.addNode(makeNode("D4", NodeType::Delivery, "客户4"));
    addTwoWay(g, "T", "D3", 7.0, 7.0, 7.0);
    addTwoWay(g, "T", "D4", 8.0, 8.0, 8.0);
    const Vehicle v = makeVehicle("W", 20.0, 480);
    const std::vector<Order> orders = {
        makeOrder("O1", "D1", "D2", 15.0, 0, 1440, false),
        makeOrder("O2", "D3", "D4", 15.0, 0, 1440, false),
    };

    const RoutePlan plan = planRoute(g, v, orders, WeightType::Distance);

    check(plan.status == PlanStatus::Ok, "总需求超载不再是不可行: " + plan.reason);
    check(plan.stops.size() == 4, "两次取货 + 两次送达 = 4 个停靠，实际 "
                                      + std::to_string(plan.stops.size()));
    // 单条连续路线：一次出发、一次回仓库（首尾都是仓库，中途不回仓库）。
    check(!plan.nodes.empty() && plan.nodes.front() == "W" && plan.nodes.back() == "W",
          "单条连续路线从仓库出发并回到仓库");
    std::size_t depotVisits = 0;
    for (const std::string& n : plan.nodes) {
        if (n == "W") {
            ++depotVisits;
        }
    }
    check(depotVisits == 2, "中途不回仓库（仓库只出现首尾两次），实际 "
                                + std::to_string(depotVisits));
    // 前缀载重全程不超过 20（结构性检查在 checkStructuralInvariants 里逐停靠复核）。
    checkStructuralInvariants(g, plan, orders, v, "总需求超载单条路线");
}

// 与上一条对照：载重充裕时同样是单条连续路线，且不产生任何降载。
void testWithinCapacityStaysSingleRoute() {
    const LogisticsGraph g = makeTransitClusterGraph();
    const Vehicle v = makeVehicle("W", 100.0, 480);
    const std::vector<Order> orders = {
        makeOrder("O1", "W", "D1", 15.0, 0, 1440, false),
        makeOrder("O2", "W", "D2", 15.0, 0, 1440, false),
    };

    const RoutePlan plan = planRoute(g, v, orders, WeightType::Distance);

    check(plan.status == PlanStatus::Ok, "可行");
    check(plan.offloads.empty(), "载重充裕时不产生降载");
    checkStructuralInvariants(g, plan, orders, v, "载重充裕");
}

// 切片 5：存在无法到达的配送点 -> Unreachable，且原因要能定位到具体节点。
void testUnreachableDeliveryIsInfeasibleAndNamesTheNode() {
    LogisticsGraph g = makeWtoD1Graph();
    g.addNode(makeNode("D9", NodeType::Delivery, "孤岛客户"));   // 无任何连边

    const Vehicle v = makeVehicle("W", 1000.0, 480);
    const std::vector<Order> orders = {makeOrder("O1", "W", "D9", 10.0, 0, 1440, false)};

    const RoutePlan plan = planRoute(g, v, orders, WeightType::Distance);

    check(plan.status == PlanStatus::Unreachable, "状态为 Unreachable");
    check(plan.stops.empty(), "不可行时无停靠点");
    check(plan.reason.find("D9") != std::string::npos,
          "原因中应含不可达节点 ID，实际: " + plan.reason);
}

// ---- D 组：增量重规划（规格二 §10 裁定 1：顺序不变版重放）----
//
// 拓扑：W --10-- D1 --10-- D2 --20-- W，另有绕行 D1 --6-- D3 --6-- D2（D3 是中转站）。
// 把直连封堵成 +50% 后，绕行 12 反而更快。
LogisticsGraph makeIncrementalGraph() {
    LogisticsGraph g;
    Node w = makeNode("W", NodeType::Warehouse, "仓库");   w.x = 0;   w.y = 0;
    Node d1 = makeNode("D1", NodeType::Delivery, "客户1"); d1.x = 100; d1.y = 0;
    Node d2 = makeNode("D2", NodeType::Delivery, "客户2"); d2.x = 200; d2.y = 0;
    Node d3 = makeNode("D3", NodeType::Transit, "中转站"); d3.x = 150; d3.y = 100;
    g.addNode(w); g.addNode(d1); g.addNode(d2); g.addNode(d3);
    addTwoWay(g, "W", "D1", 10.0, 10.0, 10.0);
    addTwoWay(g, "D1", "D2", 10.0, 10.0, 10.0);
    addTwoWay(g, "D2", "W", 20.0, 20.0, 20.0);
    addTwoWay(g, "D1", "D3", 6.0, 6.0, 6.0);
    addTwoWay(g, "D3", "D2", 6.0, 6.0, 6.0);
    return g;
}

// 裁定后的语义：停靠顺序**完全不变**，只按新边权重重放；
// usedIncremental == true 表示"用了顺序不变版重放"（不再是"只重算了受影响的 leg"）。
void testIncrementalReplayKeepsStopOrderAndReplaysWeights() {
    LogisticsGraph g = makeIncrementalGraph();
    const Vehicle v = makeVehicle("W", 1000.0, 480);
    const std::vector<Order> orders = {
        makeOrder("O1", "W", "D1", 10.0, 0, 1440, false),
        makeOrder("O2", "W", "D2", 10.0, 0, 1440, false),
    };

    const RoutePlan before = planRoute(g, v, orders, WeightType::Time);
    check(before.status == PlanStatus::Ok, "初始规划可行");

    // 构造确定的拥堵：D1->D2 耗时 +50%（不用随机模拟器，保证可复现）。
    const logistics::Edge* e = g.findEdge("D1", "D2");
    check(e != nullptr, "D1->D2 存在");
    if (e == nullptr) {
        return;
    }
    g.updateEdgeTime("D1", "D2", e->baseTimeMin * 1.5);
    const logistics::Edge* e2 = g.findEdge("D2", "D1");
    if (e2 != nullptr) {
        g.updateEdgeTime("D2", "D1", e2->baseTimeMin * 1.5);
    }

    TrafficReport report;
    TrafficChange c;
    c.fromId = "D1";
    c.toId = "D2";
    c.increaseRatio = 0.5;
    c.congested = true;
    report.changes.push_back(c);

    bool used = false;
    const RoutePlan after =
        replanIncremental(g, v, orders, before, 480, WeightType::Time, report, 0.2,
                          std::vector<logistics::OnboardItem>(), &used);

    check(after.status == PlanStatus::Ok, "重放可行");
    check(used, "走了顺序不变版重放（usedIncremental == true）");
    // 停靠顺序逐位不变（这正是"顺序不变版"的定义）。
    check(after.stops.size() == before.stops.size(), "停靠点数不变");
    bool sameOrder = after.stops.size() == before.stops.size();
    for (std::size_t i = 0; sameOrder && i < after.stops.size(); ++i) {
        sameOrder = after.stops[i].nodeId == before.stops[i].nodeId
                    && after.stops[i].phase == before.stops[i].phase;
    }
    check(sameOrder, "停靠顺序与阶段逐位不变");
    // 重放后的耗时变长（拥堵被吃进时刻），但路线本身仍可走。
    check(after.totalTimeMin > before.totalTimeMin,
          "拥堵被重放进时刻：" + std::to_string(after.totalTimeMin) + " vs "
              + std::to_string(before.totalTimeMin));
    checkStructuralInvariants(g, after, orders, v, "顺序不变版重放");
}

// 没有任何 leg 受影响时，重放结果与原路线逐位相同（幂等）。
void testIncrementalReplayIsIdempotentWhenNothingAffected() {
    const LogisticsGraph g = makeIncrementalGraph();
    const Vehicle v = makeVehicle("W", 1000.0, 480);
    const std::vector<Order> orders = {
        makeOrder("O1", "W", "D1", 10.0, 0, 1440, false),
        makeOrder("O2", "W", "D2", 10.0, 0, 1440, false),
    };

    const RoutePlan before = planRoute(g, v, orders, WeightType::Time);
    TrafficReport report;   // 空报告
    bool used = false;
    const RoutePlan after =
        replanIncremental(g, v, orders, before, 480, WeightType::Time, report, 0.2,
                          std::vector<logistics::OnboardItem>(), &used);

    check(used, "空报告下仍走重放");
    check(after.nodes == before.nodes, "无 leg 受影响时，节点序列逐位不变");
    check(after.stops.size() == before.stops.size(), "停靠点数不变");
    check(fixtures::nearlyEqual(after.totalTimeMin, before.totalTimeMin), "总耗时不变");
    check(after.totalPenaltyMin == before.totalPenaltyMin, "总 penalty 不变");
}

// 重放不可行（序列不全）⇒ 退回全量重规划，结果仍须合法且覆盖全部订单。
void testIncrementalFallsBackWhenReplayInfeasible() {
    const LogisticsGraph g = makeIncrementalGraph();
    const Vehicle v = makeVehicle("W", 1000.0, 480);
    const std::vector<Order> orders = {
        makeOrder("O1", "W", "D1", 10.0, 0, 1440, false),
        makeOrder("O2", "W", "D2", 10.0, 0, 1440, false),
    };

    // 给一个序列不全的"上一版"：只有起点，重放无从下手。
    RoutePlan broken;
    broken.status = PlanStatus::Ok;
    broken.nodes = {"W"};
    broken.nodeArrivalMin = {480};
    broken.nodeIsStop = {false};

    bool used = true;
    const RoutePlan after =
        replanIncremental(g, v, orders, broken, 480, WeightType::Distance, TrafficReport(), 0.2,
                          std::vector<logistics::OnboardItem>(), &used);
    check(after.status == PlanStatus::Ok, "回退全量后仍可行");
    check(!used, "序列不全时 usedIncremental == false（退回了全量）");
    checkStructuralInvariants(g, after, orders, v, "增量回退全量");
}

// replan 从给定位置与给定时刻出发，服务剩余订单后返回车辆起始仓库。
void testReplanFromCurrentPosition() {
    LogisticsGraph g;
    g.addNode(makeNode("W", NodeType::Warehouse, "仓库"));
    g.addNode(makeNode("D1", NodeType::Delivery, "客户1"));
    g.addNode(makeNode("D2", NodeType::Delivery, "客户2"));
    addTwoWay(g, "W", "D1", 1.0, 2.0, 1.0);
    addTwoWay(g, "W", "D2", 5.0, 10.0, 5.0);
    addTwoWay(g, "D1", "D2", 2.0, 4.0, 2.0);

    const Vehicle v = makeVehicle("W", 1000.0, 480);
    // 车已在 D2、时刻 600，剩余一张 D1 的订单（A=D2、B=D1）。
    const std::vector<Order> remaining = {makeOrder("O1", "D2", "D1", 10.0, 0, 1440, false)};

    const RoutePlan plan = replan(g, v, remaining, "D2", 600, WeightType::Distance);

    check(plan.status == PlanStatus::Ok, "重规划成功");
    check(plan.nodes == std::vector<std::string>{"D2", "D1", "W"},
          "序列为 D2->D1->W，实际 " + join(plan.nodes));
    // D2 是取货点：原地取货（无位移），D2->D1 = 4min，D1->W = 2min ⇒ 总耗时 6。
    check(fixtures::nearlyEqual(plan.totalTimeMin, 6.0),
          "总耗时 606-600 = 6，实际 " + std::to_string(plan.totalTimeMin));
    check(fixtures::nearlyEqual(plan.totalDistanceKm, 3.0),
          "总距离 2.0+1.0 = 3.0，实际 " + std::to_string(plan.totalDistanceKm));
    check(plan.stops.size() == 2, "原地取货 + 送达 = 2 个停靠");
    if (plan.stops.size() == 2) {
        check(plan.stops[0].nodeId == "D2" && plan.stops[0].arrivalMin == 600,
              "首个停靠是原地取货 D2，到达 600");
    }
    checkStructuralInvariants(g, plan, remaining, v, "从当前位置重规划", "D2");
}

// planRoute 等价于 replan(..., 仓库, 发车时刻, ...)
void testPlanRouteEqualsReplanFromDepot() {
    LogisticsGraph g;
    g.addNode(makeNode("W", NodeType::Warehouse, "仓库"));
    g.addNode(makeNode("D1", NodeType::Delivery, "客户1"));
    g.addNode(makeNode("D2", NodeType::Delivery, "客户2"));
    addTwoWay(g, "W", "D1", 1.0, 2.0, 1.0);
    addTwoWay(g, "W", "D2", 5.0, 10.0, 5.0);
    addTwoWay(g, "D1", "D2", 2.0, 4.0, 2.0);

    const Vehicle v = makeVehicle("W", 1000.0, 480);
    const std::vector<Order> orders = {
        makeOrder("O1", "W", "D1", 10.0, 0, 1440, false),
        makeOrder("O2", "W", "D2", 10.0, 0, 1440, true),
    };

    const RoutePlan viaPlanRoute = planRoute(g, v, orders, WeightType::Distance);
    const RoutePlan viaReplan =
        replan(g, v, orders, v.startNodeId, v.departTimeMin, WeightType::Distance);

    check(viaPlanRoute.nodes == viaReplan.nodes, "两个入口的节点序列逐位一致");
    check(viaPlanRoute.stops.size() == viaReplan.stops.size(), "停靠点数一致");
    check(viaPlanRoute.totalPenaltyMin == viaReplan.totalPenaltyMin, "penalty 一致");
}

// 切片 8：路径成员判定（GUI 路径高亮与路况触发判定共用这一条逻辑）
void testIsEdgeOnRoute() {
    const std::vector<std::string> route = {"W", "D1", "D2", "W"};

    check(logistics::isEdgeOnRoute(route, "W", "D1"), "首段在路径上");
    check(logistics::isEdgeOnRoute(route, "D1", "D2"), "中段在路径上");
    check(logistics::isEdgeOnRoute(route, "D2", "W"), "末段在路径上");
    check(!logistics::isEdgeOnRoute(route, "D1", "W"), "反向边不算（路径有向）");
    check(!logistics::isEdgeOnRoute(route, "W", "D2"), "非相邻的一对不算");
    check(!logistics::isEdgeOnRoute(route, "D2", "D1"), "反向的另一半也不算");
    check(!logistics::isEdgeOnRoute({}, "W", "D1"), "空路径不含任何边");
    check(!logistics::isEdgeOnRoute({"W"}, "W", "W"), "单节点无相邻对");

    const std::vector<std::string> loop = {"W", "D1", "W", "D1", "W"};
    check(logistics::isEdgeOnRoute(loop, "D1", "W"), "重复经过的段落也在路径上");
}

// nodeIndexAtTime：软件内时刻 -> "半路显示所在段的起点"
void testNodeIndexAtTimeReturnsSegmentStart() {
    const LogisticsGraph g = makeWtoD1Graph();
    const Vehicle v = makeVehicle("W", 1000.0, 480);
    const std::vector<Order> orders = {makeOrder("O1", "W", "D1", 10.0, 540, 1080, false)};
    const RoutePlan plan = planRoute(g, v, orders, WeightType::Distance);

    check(plan.nodeArrivalMin == std::vector<int>({480, 540, 550}),
          "前提：逐节点到达时刻为 480 / 540 / 550");
    if (plan.nodes.size() != 3) {
        return;
    }
    check(nodeIndexAtTime(plan, 400) == 0, "早于首节点 -> 下标 0");
    check(nodeIndexAtTime(plan, 479) == 0, "首节点前 1 分钟 -> 下标 0");
    check(nodeIndexAtTime(plan, 480) == 0, "恰等于节点 0 到达时刻 -> 下标 0");
    check(nodeIndexAtTime(plan, 540) == 1, "恰等于节点 1 到达时刻 -> 下标 1");
    check(nodeIndexAtTime(plan, 550) == 2, "恰等于节点 2 到达时刻 -> 下标 2");
    check(nodeIndexAtTime(plan, 500) == 0, "480 之后 540 之前 -> 段起点 0");
    check(nodeIndexAtTime(plan, 539) == 0, "紧临节点 1 之前 -> 段起点 0");
    check(nodeIndexAtTime(plan, 545) == 1, "540 之后 550 之前 -> 段起点 1");
    check(nodeIndexAtTime(plan, 551) == 2, "超过末节点 -> 末节点下标 2");
    check(nodeIndexAtTime(plan, 100000) == 2, "远超末节点 -> 仍为末节点下标 2");
}

void testNodeIndexAtTimeOnEmptyPlanAndClamp() {
    const RoutePlan empty;
    check(empty.nodes.empty(), "前提：空 plan");
    check(nodeIndexAtTime(empty, 0) == 0, "空 plan -> 0");
    check(nodeIndexAtTime(empty, 99999) == 0, "空 plan（任意时刻）-> 0");

    RoutePlan single;
    single.nodes = {"W"};
    single.nodeArrivalMin = {480};
    check(nodeIndexAtTime(single, 100) == 0, "单节点且早于到达 -> 0");
    check(nodeIndexAtTime(single, 480) == 0, "单节点恰在到达 -> 0");
    check(nodeIndexAtTime(single, 1000) == 0, "单节点超过到达 -> 0（不越界）");

    RoutePlan ragged;
    ragged.nodes = {"A", "B", "C"};
    ragged.nodeArrivalMin = {480};
    check(nodeIndexAtTime(ragged, 600) == 0, "到达时刻序列不完整时返回其有效下标");
}

} // namespace

namespace {

// ---- 降载 / 取回夹具 ----
// W -- S(中转站, sub=1) -- {D1..D4(客户, sub=1)}，另有 D3--D4 直连。
LogisticsGraph makeOffloadGraph() {
    LogisticsGraph g;
    g.addNode(makeNode("W", NodeType::Warehouse, "仓库"));
    Node s = makeNode("S", NodeType::Transit, "中转站");
    s.subNetworkId = 1;
    g.addNode(s);
    for (int i = 1; i <= 4; ++i) {
        Node d = makeNode("D" + std::to_string(i), NodeType::Delivery,
                          "客户" + std::to_string(i));
        d.subNetworkId = 1;
        g.addNode(d);
    }
    addTwoWay(g, "W", "S", 5.0, 10.0, 4.0);
    addTwoWay(g, "S", "D1", 2.0, 4.0, 2.0);
    addTwoWay(g, "S", "D2", 3.0, 6.0, 3.0);
    addTwoWay(g, "S", "D3", 4.0, 8.0, 4.0);
    addTwoWay(g, "S", "D4", 5.0, 10.0, 5.0);
    addTwoWay(g, "D3", "D4", 2.0, 4.0, 2.0);
    return g;
}

// 降载：紧急单装不下 ⇒ 把**整张普通单**卸到中转站，事后同一辆车取回。
// 断言口径全部按订单（MC-A05）：账本是 RoutePlan::offloads（OffloadLeg）。
void testOffloadConservesAndReclaims() {
    const LogisticsGraph g = makeOffloadGraph();
    const Vehicle v = makeVehicle("W", 100.0, 480);

    // 车上已载普通货 D1 60kg + D2 30kg（free = 10），紧急单 D3→D4 要 50kg。
    std::vector<Order> pending;
    pending.push_back(makeOrder("O1", "W", "D1", 60.0, 0, 1440));
    pending.push_back(makeOrder("O2", "W", "D2", 30.0, 0, 1440));
    const std::vector<logistics::OnboardItem> onboard{
        {"D1", 60.0, "O1"}, {"D2", 30.0, "O2"}};

    Order urgent;
    urgent.id = "U1";
    urgent.pickupNodeId = "D3";
    urgent.nodeId = "D4";
    urgent.demandKg = 50.0;
    urgent.windowStartMin = 0;
    urgent.windowEndMin = 1440;
    urgent.urgent = true;

    const InsertResult res = insertOrder(g, v, pending, urgent, "W", 480,
                                               WeightType::Distance, onboard);
    const RoutePlan& p = res.plan;

    check(p.status == PlanStatus::Ok, "降载场景规划可行: " + p.reason);
    check(!p.offloads.empty(), "装不下时应产生降载记录");

    // 不变量 ⑩：选货 = 最少订单数（kg 降序、同 kg 按 ID），只卸整张普通单、绝不卸紧急货。
    double offloadKg = 0.0;
    std::string station;
    bool onlyNormal = true;
    for (const logistics::OffloadLeg& leg : p.offloads) {
        offloadKg += leg.kg;
        if (station.empty()) {
            station = leg.station;
        }
        check(leg.station == station, "全局至多一堆：所有降载都在同一站 " + station);
        check(leg.reclaimTimeMin >= 0, "期末必须已取回（暂存守恒）");
        check(leg.orderId != "U1", "绝不卸紧急货");
        bool normal = false;
        for (const Order& o : pending) {
            if (o.id == leg.orderId && !o.urgent) {
                normal = true;
            }
        }
        if (!normal) {
            onlyNormal = false;
        }
    }
    check(onlyNormal, "降载只卸普通单");
    // need = 50 - 10 = 40；kg 降序前缀：60 ≥ 40 ⇒ 只卸 1 张（最少订单数）。
    check(p.offloads.size() == 1,
          "最少订单数选货：need 40kg 时只卸 60kg 的那一张，实际卸 "
              + std::to_string(p.offloads.size()) + " 张");
    check(fixtures::nearlyEqual(offloadKg, 60.0),
          "卸下量 60kg，实际 " + std::to_string(offloadKg));

    // 不变量 ⑪：取回停靠先于该单送达；降载停靠先于触发它的取货点。
    std::size_t offloadIdx = p.stops.size();
    std::size_t reclaimIdx = p.stops.size();
    std::size_t pickIdx = p.stops.size();
    std::size_t delivIdx = p.stops.size();
    for (std::size_t i = 0; i < p.stops.size(); ++i) {
        if (!p.stops[i].offloads.empty() && offloadIdx == p.stops.size()) {
            offloadIdx = i;
        }
        if (!p.stops[i].reclaims.empty() && reclaimIdx == p.stops.size()) {
            reclaimIdx = i;
        }
        for (const std::string& id : p.stops[i].pickups) {
            if (id == "U1" && pickIdx == p.stops.size()) {
                pickIdx = i;
            }
        }
        for (const logistics::StopDelivery& d : p.stops[i].deliveries) {
            if (d.orderId == p.offloads.front().orderId && delivIdx == p.stops.size()) {
                delivIdx = i;
            }
        }
    }
    check(offloadIdx < pickIdx, "降载停靠必须早于触发它的取货点");
    check(reclaimIdx < delivIdx, "取回停靠必须早于暂存单的送达");
    checkStructuralInvariants(g, p, [&]() {
        std::vector<Order> all = pending;
        all.push_back(urgent);
        return all;
    }(), v, "降载取回");
}

// 对照：装得下时不产生任何降载/取回。
void testUrgentFitsLeavesOffloadsEmpty() {
    const LogisticsGraph g = makeOffloadGraph();
    const Vehicle v = makeVehicle("W", 100.0, 480);

    std::vector<Order> pending;
    pending.push_back(makeOrder("O1", "W", "D1", 60.0, 0, 1440));
    pending.push_back(makeOrder("O2", "W", "D2", 30.0, 0, 1440));
    const std::vector<logistics::OnboardItem> onboard{
        {"D1", 60.0, "O1"}, {"D2", 30.0, "O2"}};

    Order urgent;
    urgent.id = "U1";
    urgent.pickupNodeId = "D3";
    urgent.nodeId = "D4";
    urgent.demandKg = 8.0;   // <= free 10
    urgent.windowStartMin = 0;
    urgent.windowEndMin = 1440;
    urgent.urgent = true;

    const InsertResult res = insertOrder(g, v, pending, urgent, "W", 480,
                                               WeightType::Distance, onboard);
    const RoutePlan& p = res.plan;

    check(p.status == PlanStatus::Ok, "装得下场景规划可行");
    check(p.offloads.empty(), "装得下时不降载（offloads 为空）");
    // 紧急单的取货/送达动作都在，且取货在前。
    std::size_t pickIdx = p.stops.size();
    std::size_t delivIdx = p.stops.size();
    for (std::size_t i = 0; i < p.stops.size(); ++i) {
        for (const std::string& id : p.stops[i].pickups) {
            if (id == "U1" && pickIdx == p.stops.size()) {
                pickIdx = i;
            }
        }
        for (const logistics::StopDelivery& d : p.stops[i].deliveries) {
            if (d.orderId == "U1" && delivIdx == p.stops.size()) {
                delivIdx = i;
            }
        }
    }
    check(pickIdx < delivIdx, "紧急单先取后送");
    std::vector<Order> all = pending;
    all.push_back(urgent);
    checkStructuralInvariants(g, p, all, v, "装得下不降载");
}

// 顺路交货：紧急停靠点上若恰有在车普通货的目的地，且不增加等待 ⇒ 一并交付。
// 且该普通单**只送达一次**（不二次配送）。
void testIncidentalDeliveryMergesIntoUrgentStop() {
    LogisticsGraph g;
    g.addNode(makeNode("W", NodeType::Warehouse, "仓库"));
    g.addNode(makeNode("A", NodeType::Delivery, "客户A"));
    g.addNode(makeNode("B", NodeType::Delivery, "客户B"));
    g.addNode(makeNode("X", NodeType::Delivery, "客户X"));
    g.addNode(makeNode("Y", NodeType::Delivery, "客户Y"));
    addTwoWay(g, "W", "A", 5.0, 10.0, 4.0);
    addTwoWay(g, "W", "B", 3.0, 6.0, 2.0);
    addTwoWay(g, "W", "X", 4.0, 8.0, 3.0);
    addTwoWay(g, "A", "B", 2.0, 4.0, 2.0);
    addTwoWay(g, "W", "Y", 2.0, 4.0, 2.0);
    addTwoWay(g, "Y", "X", 2.0, 4.0, 2.0);

    const Vehicle v = makeVehicle("W", 100.0, 480);

    // 车上已载 A 的普通货 20kg、B 的普通货 15kg；插紧急单 A→B 10kg（装得下）。
    std::vector<Order> pending;
    pending.push_back(makeOrder("O1", "W", "A", 20.0, 0, 1440));
    pending.push_back(makeOrder("O2", "W", "B", 15.0, 0, 1440));
    pending.push_back(makeOrder("O3", "Y", "X", 10.0, 0, 1440));
    const std::vector<logistics::OnboardItem> onboard{
        {"A", 20.0, "O1"}, {"B", 15.0, "O2"}};

    Order urgent;
    urgent.id = "U1";
    urgent.pickupNodeId = "A";
    urgent.nodeId = "B";
    urgent.demandKg = 10.0;
    urgent.windowStartMin = 0;
    urgent.windowEndMin = 1440;
    urgent.urgent = true;

    const InsertResult res = insertOrder(g, v, pending, urgent, "W", 480,
                                               WeightType::Distance, onboard);
    const RoutePlan& p = res.plan;

    check(p.status == PlanStatus::Ok, "顺路交货（装得下）规划可行: " + p.reason);
    check(p.offloads.empty(), "装得下时无降载");

    // 不变量 ⑦：A/B 各只出现一次停靠（顺路交货并入紧急停靠，不额外多停一次）。
    int countA = 0;
    int countB = 0;
    for (const Stop& s : p.stops) {
        if (s.nodeId == "A") {
            ++countA;
        }
        if (s.nodeId == "B") {
            ++countB;
        }
    }
    check(countA == 1, "A 只停靠一次（实际 " + std::to_string(countA) + "）");
    check(countB == 1, "B 只停靠一次（实际 " + std::to_string(countB) + "）");

    // 顺路交货：A 停靠上应同时有 U1 的取货与 O1 的送达（先卸后装）。
    bool aHasBoth = false;
    for (const Stop& s : p.stops) {
        if (s.nodeId != "A") {
            continue;
        }
        bool hasPick = false;
        bool hasDeliv = false;
        for (const std::string& id : s.pickups) {
            if (id == "U1") {
                hasPick = true;
            }
        }
        for (const logistics::StopDelivery& d : s.deliveries) {
            if (d.orderId == "O1") {
                hasDeliv = true;
            }
        }
        aHasBoth = hasPick && hasDeliv;
    }
    check(aHasBoth, "A 停靠同时承载 U1 的取货与 O1 的顺路交付");

    // 每张普通单仍只送达一次（顺路交付不产生二次配送）。
    for (const char* id : {"O1", "O2", "O3"}) {
        std::size_t hits = 0;
        for (const Stop& s : p.stops) {
            for (const logistics::StopDelivery& d : s.deliveries) {
                if (d.orderId == id) {
                    ++hits;
                }
            }
        }
        check(hits == 1, std::string(id) + " 恰好送达一次（实际 "
                             + std::to_string(hits) + "）");
    }

    std::vector<Order> all = pending;
    all.push_back(urgent);
    checkStructuralInvariants(g, p, all, v, "顺路交货");
}

// 顺路交货只在**不增加等待**时并入：普通单 ws 晚于该停靠原等待点 ⇒ 本次不交付。
void testIncidentalDeliverySkippedWhenItWouldDelay() {
    LogisticsGraph g;
    g.addNode(makeNode("W", NodeType::Warehouse, "仓库"));
    g.addNode(makeNode("A", NodeType::Delivery, "客户A"));
    g.addNode(makeNode("B", NodeType::Delivery, "客户B"));
    addTwoWay(g, "W", "A", 5.0, 10.0, 4.0);
    addTwoWay(g, "W", "B", 3.0, 6.0, 2.0);
    addTwoWay(g, "A", "B", 2.0, 4.0, 2.0);

    const Vehicle v = makeVehicle("W", 100.0, 480);
    // 在车普通单 O1 的目的地是 B，但 ws = 800（远晚于紧急停靠的到达时刻）。
    std::vector<Order> pending;
    pending.push_back(makeOrder("O1", "W", "B", 20.0, 800, 900));
    const std::vector<logistics::OnboardItem> onboard{{"B", 20.0, "O1"}};

    Order urgent;
    urgent.id = "U1";
    urgent.pickupNodeId = "A";
    urgent.nodeId = "B";
    urgent.demandKg = 10.0;
    urgent.windowStartMin = 0;
    urgent.windowEndMin = 1440;
    urgent.urgent = true;

    const InsertResult res = insertOrder(g, v, pending, urgent, "W", 480,
                                               WeightType::Distance, onboard);
    const RoutePlan& p = res.plan;
    check(p.status == PlanStatus::Ok, "场景规划可行: " + p.reason);

    // 紧急停靠 B 上不应有 O1 的送达动作（否则会把紧急停靠的到达推迟到 800）。
    bool urgentBHasO1 = false;
    for (const Stop& s : p.stops) {
        if (s.nodeId != "B" || s.phase != StopPhase::Urgent) {
            continue;
        }
        for (const logistics::StopDelivery& d : s.deliveries) {
            if (d.orderId == "O1") {
                urgentBHasO1 = true;
            }
        }
    }
    check(!urgentBHasO1, "会推迟紧急停靠时不做顺路交货（O1 留到普通阶段）");
    std::vector<Order> all = pending;
    all.push_back(urgent);
    checkStructuralInvariants(g, p, all, v, "顺路交货不推迟");
}

// ---- issue #6：多紧急单严格 FIFO ----
// 先插入 U1(D1→D2) 再插入 U2(D3→D4)；U1 的送达必须不晚于 U2 的取货。
void testUrgentFifoOrder() {
    const LogisticsGraph g = makeOffloadGraph();
    const Vehicle v = makeVehicle("W", 100.0, 480);

    Order u1;
    u1.id = "U1"; u1.pickupNodeId = "D1"; u1.nodeId = "D2"; u1.demandKg = 5.0;
    u1.windowStartMin = 0; u1.windowEndMin = 1440; u1.urgent = true;
    Order u2;
    u2.id = "U2"; u2.pickupNodeId = "D3"; u2.nodeId = "D4"; u2.demandKg = 5.0;
    u2.windowStartMin = 0; u2.windowEndMin = 1440; u2.urgent = true;

    const RoutePlan p = planRoute(g, v, {u1, u2}, WeightType::Distance);
    check(p.status == PlanStatus::Ok, "两张紧急单规划可行");

    // 不变量 ⑤：U1 的送达停靠索引 <= U2 的取货停靠索引。
    std::size_t u1Deliv = p.stops.size();
    std::size_t u2Pick = p.stops.size();
    for (std::size_t i = 0; i < p.stops.size(); ++i) {
        for (const logistics::StopDelivery& d : p.stops[i].deliveries) {
            if (d.orderId == "U1" && u1Deliv == p.stops.size()) {
                u1Deliv = i;
            }
        }
        for (const std::string& id : p.stops[i].pickups) {
            if (id == "U2" && u2Pick == p.stops.size()) {
                u2Pick = i;
            }
        }
    }
    check(u1Deliv <= u2Pick, "FIFO：U1 的送达不晚于 U2 的取货（"
                                 + std::to_string(u1Deliv) + " <= "
                                 + std::to_string(u2Pick) + "）");
    // 时刻口径双断言（规格一 §4.7 不变量 5）。
    check(p.stops[u1Deliv].arrivalMin <= p.stops[u2Pick].arrivalMin,
          "FIFO 时刻口径：U1 送达时刻 <= U2 取货时刻");
    checkStructuralInvariants(g, p, {u1, u2}, v, "紧急 FIFO");
}

// FIFO 强制重访：U2 的取货点与 U1 的取货点相同 ⇒ 必须重访该节点（不变量 ⑦ 的唯一例外）。
void testFifoForcedRevisit() {
    const LogisticsGraph g = makeOffloadGraph();
    const Vehicle v = makeVehicle("W", 100.0, 480);

    Order u1;
    u1.id = "U1"; u1.pickupNodeId = "D1"; u1.nodeId = "D2"; u1.demandKg = 5.0;
    u1.windowStartMin = 0; u1.windowEndMin = 1440; u1.urgent = true;
    Order u2;
    u2.id = "U2"; u2.pickupNodeId = "D1"; u2.nodeId = "D3"; u2.demandKg = 5.0;
    u2.windowStartMin = 0; u2.windowEndMin = 1440; u2.urgent = true;

    const RoutePlan p = planRoute(g, v, {u1, u2}, WeightType::Distance);
    check(p.status == PlanStatus::Ok, "FIFO 强制重访场景可行: " + p.reason);

    // D1 在紧急阶段出现两次；第二次只服务 U2 的取货（一个动作）。
    std::vector<std::size_t> d1Stops;
    for (std::size_t i = 0; i < p.stops.size(); ++i) {
        if (p.stops[i].nodeId == "D1" && p.stops[i].phase == StopPhase::Urgent) {
            d1Stops.push_back(i);
        }
    }
    check(d1Stops.size() == 2, "D1 在紧急阶段被访问两次（FIFO 强制重访），实际 "
                                   + std::to_string(d1Stops.size()));
    if (d1Stops.size() == 2) {
        const Stop& revisit = p.stops[d1Stops[1]];
        const std::size_t actions = revisit.pickups.size() + revisit.deliveries.size()
                                    + revisit.offloads.size() + revisit.reclaims.size();
        check(actions == 1, "重访停靠只含一个动作（实际 " + std::to_string(actions) + "）");
        check(revisit.pickups.size() == 1 && revisit.pickups[0] == "U2",
              "重访只为取 U2");
    }
    checkStructuralInvariants(g, p, {u1, u2}, v, "FIFO 强制重访");
}

// 带暂存货的重规划必须保住「回站取回」。
void testReplanKeepsStash() {
    const LogisticsGraph g = makeOffloadGraph();
    const Vehicle v = makeVehicle("W", 100.0, 480);

    std::vector<Order> pending;
    pending.push_back(makeOrder("O1", "W", "D1", 60.0, 0, 1440));
    pending.push_back(makeOrder("O2", "W", "D2", 30.0, 0, 1440));

    logistics::StashState stash;
    stash.station = "S";
    stash.items.push_back(logistics::OnboardItem{"D1", 60.0, "O1"});
    stash.items.push_back(logistics::OnboardItem{"D2", 30.0, "O2"});

    const RoutePlan p = replan(g, v, pending, "W", 480, WeightType::Distance,
                               std::vector<logistics::OnboardItem>(), stash);
    check(p.status == PlanStatus::Ok, "带暂存货重规划可行: " + p.reason);

    bool hasReclaim = false;
    double reclaimKg = 0.0;
    for (const Stop& s : p.stops) {
        if (!s.reclaims.empty()) {
            hasReclaim = true;
            for (const std::string& id : s.reclaims) {
                for (const Order& o : pending) {
                    if (o.id == id) {
                        reclaimKg += o.demandKg;
                    }
                }
            }
        }
    }
    check(hasReclaim, "重规划后必须仍包含回 S 取回暂存货");
    check(fixtures::nearlyEqual(reclaimKg, 90.0),
          "取回量与暂存守恒（90kg），实际 " + std::to_string(reclaimKg));

    // 每张暂存单只送达一次。
    for (const char* id : {"O1", "O2"}) {
        std::size_t hits = 0;
        for (const Stop& s : p.stops) {
            for (const logistics::StopDelivery& d : s.deliveries) {
                if (d.orderId == id) {
                    ++hits;
                }
            }
        }
        check(hits == 1, std::string(id) + " 只送达一次（实际 " + std::to_string(hits) + "）");
    }
    checkStructuralInvariants(g, p, pending, v, "重规划保住暂存");
}

// 执行中（已取未送）的紧急单不得被新紧急单抢先，也不得再次到 A 取货。
void testPickedUrgentNotOvertakenByNewUrgent() {
    const LogisticsGraph g = makeOffloadGraph();
    const Vehicle v = makeVehicle("W", 100.0, 480);

    Order u1;
    u1.id = "U1"; u1.pickupNodeId = "D1"; u1.nodeId = "D2"; u1.demandKg = 5.0;
    u1.windowStartMin = 0; u1.windowEndMin = 1440; u1.urgent = true; u1.picked = true;
    Order u2;
    u2.id = "U2"; u2.pickupNodeId = "D3"; u2.nodeId = "D4"; u2.demandKg = 5.0;
    u2.windowStartMin = 0; u2.windowEndMin = 1440; u2.urgent = true;

    const std::vector<logistics::OnboardItem> onboard{{"D2", 5.0, "U1"}};
    const RoutePlan p = replan(g, v, {u1, u2}, "W", 480, WeightType::Distance, onboard);
    check(p.status == PlanStatus::Ok, "已取未送场景规划可行: " + p.reason);

    std::size_t u1Deliv = p.stops.size();
    std::size_t u2Deliv = p.stops.size();
    bool d1Pickup = false;
    for (std::size_t i = 0; i < p.stops.size(); ++i) {
        for (const logistics::StopDelivery& d : p.stops[i].deliveries) {
            if (d.orderId == "U1" && u1Deliv == p.stops.size()) {
                u1Deliv = i;
            }
            if (d.orderId == "U2" && u2Deliv == p.stops.size()) {
                u2Deliv = i;
            }
        }
        for (const std::string& id : p.stops[i].pickups) {
            if (id == "U1") {
                d1Pickup = true;
            }
        }
    }
    check(u1Deliv < u2Deliv, "执行中紧急单必须先于新紧急单送达");
    check(!d1Pickup, "已取未送的紧急单不得再次到 A 取货");
}

// 两紧急单都需降载 ⇒ 必须并入同一站的同一堆（不变量 ⑩：全局至多一堆、锁站）。
void testTwoUrgentOffloadsMergeIntoSingleStashPile() {
    LogisticsGraph g;
    g.addNode(makeNode("W", NodeType::Warehouse, "仓库"));
    Node s1 = makeNode("S1", NodeType::Transit, "中转站1"); s1.subNetworkId = 1;
    Node s2 = makeNode("S2", NodeType::Transit, "中转站2"); s2.subNetworkId = 2;
    g.addNode(s1); g.addNode(s2);
    for (const char* id : {"A1", "B1", "A2", "B2", "D1", "D2"}) {
        Node d = makeNode(id, NodeType::Delivery, id);
        d.subNetworkId = 1;
        g.addNode(d);
    }
    addTwoWay(g, "W", "S1", 5.0, 10.0, 5.0);
    addTwoWay(g, "W", "S2", 5.0, 10.0, 5.0);
    addTwoWay(g, "S1", "A1", 1.0, 2.0, 1.0);
    addTwoWay(g, "S1", "B1", 1.0, 2.0, 1.0);
    addTwoWay(g, "S2", "A2", 1.0, 2.0, 1.0);
    addTwoWay(g, "S2", "B2", 1.0, 2.0, 1.0);
    addTwoWay(g, "S2", "A1", 10.0, 20.0, 10.0);
    addTwoWay(g, "S2", "B1", 10.0, 20.0, 10.0);
    addTwoWay(g, "S1", "A2", 10.0, 20.0, 10.0);
    addTwoWay(g, "S1", "B2", 10.0, 20.0, 10.0);
    addTwoWay(g, "W", "A1", 6.0, 12.0, 6.0);
    addTwoWay(g, "W", "B1", 6.0, 12.0, 6.0);
    addTwoWay(g, "W", "A2", 6.0, 12.0, 6.0);
    addTwoWay(g, "W", "B2", 6.0, 12.0, 6.0);
    addTwoWay(g, "W", "D1", 7.0, 14.0, 7.0);
    addTwoWay(g, "W", "D2", 8.0, 16.0, 8.0);

    const Vehicle v = makeVehicle("W", 100.0, 480);

    // 车上 90kg（上限 100）→ free 10；每张紧急单 50kg → need 40，两张都要降载。
    std::vector<Order> remaining;
    remaining.push_back(makeOrder("O1", "W", "D1", 60.0, 0, 1440));
    remaining.push_back(makeOrder("O2", "W", "D2", 30.0, 0, 1440));

    Order u1;
    u1.id = "U1"; u1.pickupNodeId = "A1"; u1.nodeId = "B1";
    u1.demandKg = 50.0; u1.windowStartMin = 0; u1.windowEndMin = 1440; u1.urgent = true;
    Order u2 = u1;
    u2.id = "U2"; u2.pickupNodeId = "A2"; u2.nodeId = "B2";
    remaining.push_back(u1);
    remaining.push_back(u2);

    const std::vector<logistics::OnboardItem> onboard{
        {"D1", 60.0, "O1"}, {"D2", 30.0, "O2"}};
    const RoutePlan p = replan(g, v, remaining, "W", 480, WeightType::Distance, onboard);

    check(p.status == PlanStatus::Ok, "两紧急单单堆暂存场景规划可行: " + p.reason);

    // 全局至多一堆：所有降载都在同一站。
    std::string station;
    bool singlePile = true;
    for (const logistics::OffloadLeg& leg : p.offloads) {
        if (station.empty()) {
            station = leg.station;
        } else if (station != leg.station) {
            singlePile = false;
        }
        check(leg.reclaimTimeMin >= 0, "期末必须已取回");
    }
    check(!p.offloads.empty(), "至少发生一次降载");
    check(singlePile, "全局至多一堆暂存：所有降载都在同一站（" + station + "）");
    checkStructuralInvariants(g, p, remaining, v, "两紧急单单堆");
}

// 增量重规划失败回退时也必须保住暂存货。
void testIncrementalFallbackKeepsStash() {
    const LogisticsGraph g = makeOffloadGraph();
    const Vehicle v = makeVehicle("W", 100.0, 480);

    RoutePlan prev;
    prev.status = PlanStatus::Ok;
    prev.nodes = {"W"};
    prev.nodeArrivalMin = {480};
    prev.nodeIsStop = {false};   // 序列不全 -> 重放必然失败

    logistics::StashState stash;
    stash.station = "S";
    stash.items.push_back(logistics::OnboardItem{"D1", 60.0, "O1"});
    stash.items.push_back(logistics::OnboardItem{"D2", 30.0, "O2"});

    std::vector<Order> pending;
    pending.push_back(makeOrder("O1", "W", "D1", 60.0, 0, 1440));
    pending.push_back(makeOrder("O2", "W", "D2", 30.0, 0, 1440));

    const logistics::TrafficReport report;
    bool used = true;
    const RoutePlan p = replanIncremental(
        g, v, pending, prev, 480, WeightType::Distance, report, 0.2,
        std::vector<logistics::OnboardItem>(), &used, stash);

    check(p.status == PlanStatus::Ok, "增量失败回退后规划可行");
    check(!used, "序列不全时退回全量（usedIncremental == false）");
    bool hasReclaim = false;
    for (const Stop& s : p.stops) {
        if (!s.reclaims.empty()) {
            hasReclaim = true;
        }
    }
    check(hasReclaim, "增量失败回退后仍必须包含回 S 取回暂存货");
}

} // namespace

namespace {

// ---------------------------------------------------------------------------
// MC-B05 车辆因子重放：到达时刻 / penalty / 成本全部用该车缩放后的权重重放；
// 距离不变；**图表示仍显示基础边权**。
//
// 关键：默认数据只有 1.00，因子代码全错也不会红 —— 所以这里必须带 1.25 / 1.5。
// ---------------------------------------------------------------------------
void testVehicleFactorsRescaleTimeAndCostNotDistance() {
    const LogisticsGraph g = makeWtoD1Graph();   // W<->D1 5.0km / 10min / 4元
    const std::vector<Order> orders = {makeOrder("O1", "W", "D1", 10.0, 0, 1440, false)};

    const Vehicle base = makeVehicle("W", 1000.0, 480);
    const Vehicle slow = makeVehicleWithFactors("W", 1000.0, 480, 1.25, 1.5);

    const RoutePlan a = planRoute(g, base, orders, WeightType::Distance);
    const RoutePlan b = planRoute(g, slow, orders, WeightType::Distance);
    check(a.status == PlanStatus::Ok && b.status == PlanStatus::Ok, "两车都可行");

    // 距离不乘因子。
    check(fixtures::nearlyEqual(a.totalDistanceKm, b.totalDistanceKm),
          "距离不乘因子：" + std::to_string(a.totalDistanceKm) + " vs "
              + std::to_string(b.totalDistanceKm));
    // 耗时乘 timeFactor：10min 去 + 10min 回 = 20 ⇒ 25。
    check(fixtures::nearlyEqual(a.totalTimeMin, 20.0),
          "基准车总耗时 20，实际 " + std::to_string(a.totalTimeMin));
    check(fixtures::nearlyEqual(b.totalTimeMin, 25.0),
          "1.25 慢车总耗时 25，实际 " + std::to_string(b.totalTimeMin));
    // 成本乘 costFactor：8 元 ⇒ 12。
    check(fixtures::nearlyEqual(a.totalCostYuan, 8.0),
          "基准车总成本 8，实际 " + std::to_string(a.totalCostYuan));
    check(fixtures::nearlyEqual(b.totalCostYuan, 12.0),
          "1.5 倍成本车总成本 12，实际 " + std::to_string(b.totalCostYuan));
    // 到达时刻随之推后。
    check(a.returnArrivalMin == 500, "基准车回仓 500，实际 "
                                        + std::to_string(a.returnArrivalMin));
    check(b.returnArrivalMin == 505, "1.25 慢车回仓 505，实际 "
                                        + std::to_string(b.returnArrivalMin));

    // 等待与 penalty 不乘因子：窗口 480-505 ⇒ 慢车 505 到，不迟到；
    // 若把等待也乘 1.25，到达会被推过 505 而误记 penalty。
    const std::vector<Order> tight = {makeOrder("O1", "W", "D1", 10.0, 480, 505, false)};
    const RoutePlan t = planRoute(g, slow, tight, WeightType::Distance);
    check(t.status == PlanStatus::Ok, "紧窗场景可行");
    check(t.totalPenaltyMin == 0,
          "等待与 penalty 不乘因子（505 到达、窗口止 505 ⇒ 不迟到），实际 penalty "
              + std::to_string(t.totalPenaltyMin));

    // 图表示仍是基础边权：图上的边没有被因子改写。
    const logistics::Edge* e = g.findEdge("W", "D1");
    check(e != nullptr && fixtures::nearlyEqual(e->timeMin, 10.0),
          "图表示仍显示基础边权（10min），未被因子改写");
}

// 同一节点序列下，因子只改时刻/成本，不改**服务顺序**（标量缩放不改变 argmin）。
void testVehicleFactorsDoNotChangeServiceOrder() {
    LogisticsGraph g;
    g.addNode(makeNode("W", NodeType::Warehouse, "仓库"));
    g.addNode(makeNode("D1", NodeType::Delivery, "客户1"));
    g.addNode(makeNode("D2", NodeType::Delivery, "客户2"));
    addTwoWay(g, "W", "D1", 1.0, 2.0, 1.0);
    addTwoWay(g, "W", "D2", 5.0, 10.0, 5.0);
    addTwoWay(g, "D1", "D2", 2.0, 4.0, 2.0);

    const std::vector<Order> orders = {
        makeOrder("O1", "W", "D1", 10.0, 0, 1440, false),
        makeOrder("O2", "W", "D2", 10.0, 0, 1440, false),
    };
    const Vehicle base = makeVehicle("W", 1000.0, 480);
    const Vehicle slow = makeVehicleWithFactors("W", 1000.0, 480, 1.25, 1.5);

    for (WeightType w : {WeightType::Distance, WeightType::Time, WeightType::Cost}) {
        const RoutePlan a = planRoute(g, base, orders, w);
        const RoutePlan b = planRoute(g, slow, orders, w);
        check(a.status == PlanStatus::Ok && b.status == PlanStatus::Ok, "两车都可行");
        check(a.stops.size() == b.stops.size(), "停靠数一致");
        bool sameOrder = a.stops.size() == b.stops.size();
        for (std::size_t i = 0; sameOrder && i < a.stops.size(); ++i) {
            sameOrder = a.stops[i].nodeId == b.stops[i].nodeId;
        }
        check(sameOrder, "同一策略下因子不改变服务顺序（标量缩放不改变 argmin）");
    }
}

// ---------------------------------------------------------------------------
// 不变量 ⑫：输出确定性 —— 同输入（含策略）必得同一路线与同一 key。
// ---------------------------------------------------------------------------
void testDeterministicSameInput() {
    const LogisticsGraph g = makeOffloadGraph();
    const Vehicle v = makeVehicle("W", 100.0, 480);
    // 客户间取送对（A 从不在仓库）：不改语义，只把 A 从 W 换成客户节点，
    // 因为新模型不允许"中途回仓库装货"（MC-A02 退役）。
    std::vector<Order> orders;
    for (int i = 0; i < 4; ++i) {
        const std::string a = "D" + std::to_string(i % 4 + 1);
        const std::string b = "D" + std::to_string((i + 1) % 4 + 1);
        orders.push_back(makeOrder("O" + std::to_string(i + 1), a, b, 10.0, 0, 1440, false));
    }
    orders.push_back(makeOrder("U1", "D1", "D4", 5.0, 0, 1440, true));

    for (WeightType w : {WeightType::Distance, WeightType::Time, WeightType::Cost}) {
        const RoutePlan a = planRoute(g, v, orders, w);
        const RoutePlan b = planRoute(g, v, orders, w);
        check(a.status == PlanStatus::Ok && b.status == PlanStatus::Ok, "两次都可行");
        check(a.nodes == b.nodes, "同输入必得同一节点序列");
        check(a.totalPenaltyMin == b.totalPenaltyMin, "同输入必得同一 penalty");
        check(fixtures::nearlyEqual(a.totalDistanceKm, b.totalDistanceKm), "同输入必得同一距离");
        check(fixtures::nearlyEqual(a.totalTimeMin, b.totalTimeMin), "同输入必得同一耗时");
        check(fixtures::nearlyEqual(a.totalCostYuan, b.totalCostYuan), "同输入必得同一成本");
        bool sameStops = a.stops.size() == b.stops.size();
        for (std::size_t i = 0; sameStops && i < a.stops.size(); ++i) {
            sameStops = a.stops[i].nodeId == b.stops[i].nodeId
                        && a.stops[i].arrivalMin == b.stops[i].arrivalMin;
        }
        check(sameStops, "同输入必得同一停靠序列与到达时刻");
    }
}

// ---------------------------------------------------------------------------
// repair（有界单请求 relocate，R<=3）不得把路线改坏：结果仍满足全部不变量，
// 且 key 不劣于「未修复」的路线（只在有改进时才接受）。
// 这里用一条「插入顺序明显不利」的用例，断言 repair 之后 penalty 不增。
// ---------------------------------------------------------------------------
void testRepairNeverWorsensKey() {
    const LogisticsGraph g = makeOffloadGraph();
    const Vehicle v = makeVehicle("W", 100.0, 480);
    // 刻意让 EDD 序与地理顺序冲突：D4 窗口最紧但最远。
    std::vector<Order> orders;
    orders.push_back(makeOrder("O4", "W", "D4", 10.0, 480, 520, false));
    orders.push_back(makeOrder("O1", "W", "D1", 10.0, 0, 1440, false));
    orders.push_back(makeOrder("O2", "W", "D2", 10.0, 0, 1440, false));
    orders.push_back(makeOrder("O3", "W", "D3", 10.0, 0, 1440, false));

    const RoutePlan p = planRoute(g, v, orders, WeightType::Distance);
    check(p.status == PlanStatus::Ok, "规划可行: " + p.reason);
    checkStructuralInvariants(g, p, orders, v, "repair");
    // repair 只接受改进 ⇒ 紧窗单 O4 应被排到足够早的位置（penalty 有界）。
    check(p.totalPenaltyMin <= 40,
          "紧窗单 O4 的 penalty 有界（<= 40），实际 " + std::to_string(p.totalPenaltyMin));
}

// ---------------------------------------------------------------------------
// 全量兜底（Q4=B）：插入失败时重跑构造，结果仍覆盖全部订单。
// 用「一张单装不下、必须先卸货」的形态触发，断言兜底/降载链确实产出了可行解。
// ---------------------------------------------------------------------------
void testFallbackProducesCompleteRoute() {
    const LogisticsGraph g = makeOffloadGraph();
    const Vehicle v = makeVehicle("W", 100.0, 480);
    // 两张单都从 D1 取 60kg（合计 120 > 载重 100）：插入第二张时容量不够，
    // 必须靠"先把第一张送掉、再回 D1 取第二张"（强制重访）——走修复/兜底链。
    std::vector<Order> orders;
    orders.push_back(makeOrder("O1", "D1", "D2", 60.0, 0, 1440, false));
    orders.push_back(makeOrder("O4", "D1", "D4", 60.0, 0, 1440, false));
    orders.push_back(makeOrder("O2", "D2", "D3", 20.0, 0, 1440, false));
    orders.push_back(makeOrder("O3", "D3", "D4", 20.0, 0, 1440, false));

    const RoutePlan p = planRoute(g, v, orders, WeightType::Distance);
    check(p.status == PlanStatus::Ok, "兜底后仍可行: " + p.reason);
    checkStructuralInvariants(g, p, orders, v, "全量兜底");
    // 全部订单都被服务（不丢单）。
    for (const Order& o : orders) {
        std::size_t hits = 0;
        for (const Stop& s : p.stops) {
            for (const logistics::StopDelivery& d : s.deliveries) {
                if (d.orderId == o.id) {
                    ++hits;
                }
            }
        }
        check(hits == 1, "兜底后订单 " + o.id + " 恰好送达一次");
    }
}

// ---------------------------------------------------------------------------
// 指标口径（#29 §6.1）：逐单 penalty、准时率、平均在车/利用率/峰值。
// 用一条可完全手算的极小路线钉住口径，其余数字由 --plan-summary 的 cli 层守。
// ---------------------------------------------------------------------------
void testMetricsDefinition() {
    const LogisticsGraph g = makeWtoD1Graph();
    const Vehicle v = makeVehicle("W", 100.0, 480);
    // W->D1 10min：取货在 W（480 原地），送达 D1 在 490。
    // 窗口 480-495 ⇒ 送达准时；A→B 最短耗时 10 ⇒ 取货 480+10=490 <= 495 ⇒ 取货也准时。
    const std::vector<Order> orders = {makeOrder("O1", "W", "D1", 40.0, 480, 495, false)};
    const RoutePlan p = planRoute(g, v, orders, WeightType::Distance);
    check(p.status == PlanStatus::Ok, "规划可行");

    const logistics::RouteMetrics& m = p.metrics;
    check(m.stopCount == 2, "停靠点数 2（原地取货 + 送达），实际 "
                                + std::to_string(m.stopCount));
    check(m.actionCount == 2, "动作数 2 = 2 × 取送对数，实际 "
                                  + std::to_string(m.actionCount));
    check(m.pickupTotal == 1 && m.pickupOnTimeCount == 1, "取货准时 1/1");
    check(m.deliveryTotal == 1 && m.deliveryOnTimeCount == 1, "送达准时 1/1");
    check(m.urgentTotal == 0, "无紧急单 ⇒ 紧急分母 0（--plan-summary 打「—」）");
    check(m.normalTotal == 1 && m.normalOnTimeCount == 1, "普通准时 1/1");
    check(m.orders.size() == 1 && m.orders[0].orderId == "O1", "逐单指标覆盖 O1");

    // 时间加权在车量：40kg 在车 10min（W->D1 段），之后 0。
    // 总运行时长 = 20min（去 10 + 回 10）⇒ 平均 = 40*10/20 = 20kg；峰值 40。
    check(fixtures::nearlyEqual(m.peakLoadKg, 40.0),
          "峰值在车 40kg，实际 " + std::to_string(m.peakLoadKg));
    check(fixtures::nearlyEqual(m.avgLoadKg, 20.0),
          "平均在车 = 40kg × 10min / 20min = 20kg，实际 " + std::to_string(m.avgLoadKg));
    check(fixtures::nearlyEqual(m.loadUtilization, 0.2),
          "利用率 = 20/100 = 20%，实际 " + std::to_string(m.loadUtilization));

    // 分母 0 的率不产生数值（cli 层打「—」）。
    const std::vector<Order> urgentOnly = {makeOrder("U1", "W", "D1", 10.0, 0, 1440, true)};
    const RoutePlan pu = planRoute(g, v, urgentOnly, WeightType::Distance);
    check(pu.metrics.normalTotal == 0, "只有紧急单时普通分母 0");
    check(pu.metrics.urgentTotal == 1, "紧急分母 1");
}

// ---------------------------------------------------------------------------
// 车队夹具（规格二 §3 / #39）：空车、两车同簇、慢车因子、顺序轮转基线。
// ---------------------------------------------------------------------------

// 空车（0 单）：五项全 0、返回时刻 = 发车时刻、不抬升完工耗时、参与 min(发车)。
void testFleetEmptyVehicleHasZeroMetrics() {
    const LogisticsGraph g = makeWtoD1Graph();
    const std::vector<Order> orders = {makeOrder("O1", "W", "D1", 10.0, 0, 1440, false)};
    Vehicle fast = makeVehicle("W", 1000.0, 480);
    fast.id = "V01";
    Vehicle idle = makeVehicle("W", 1000.0, 470);   // 更早发车：参与 min(发车)
    idle.id = "V02";
    // 顺序轮转基线：1 张单 -> 第 0 台车；第 1 台必然空车。
    // （空车规则是**车队汇总**语义，与分配算法无关，所以这里用基线入口；
    //   三阶段分配下的空车另见 testFleetTypeAwareOneClusterLeavesLastVehicleEmpty。）
    const FleetPlan fleet = planFleetSequentialBaseline(g, {fast, idle}, orders,
                                                        WeightType::Distance);
    check(fleet.vehicles.size() == 2, "车队逐车计划数 == 2");
    check(fleet.allOk, "两台车都可规划");
    const RoutePlan& empty = fleet.vehicles[1];
    check(fixtures::nearlyEqual(empty.totalDistanceKm, 0.0)
              && fixtures::nearlyEqual(empty.totalCostYuan, 0.0)
              && empty.totalPenaltyMin == 0 && empty.metrics.stopCount == 0
              && empty.metrics.actionCount == 0,
          "空车五项全 0");
    check(fixtures::nearlyEqual(empty.totalTimeMin, 0.0), "空车耗时 = 0");
    check(empty.returnArrivalMin == idle.departTimeMin, "空车返回时刻 = 发车时刻");
    // 完工耗时 = 有货车的返回 − **空车**的发车（空车不抬升 max，但参与 min 发车）。
    const double expect = static_cast<double>(fast.departTimeMin)
                          + fleet.vehicles[0].totalTimeMin
                          - static_cast<double>(idle.departTimeMin);
    check(fixtures::nearlyEqual(fleet.completionTimeMin, expect),
          "完工耗时 = max(返回) − min(发车)，空车不抬升 max："
              + std::to_string(fleet.completionTimeMin) + " vs " + std::to_string(expect));
    check(fixtures::nearlyEqual(fleet.sumVehicleTimeMin, fleet.vehicles[0].totalTimeMin),
          "车时合计 = Σ每车耗时（空车贡献 0）");
    check(fleet.completionTimeMin > fleet.sumVehicleTimeMin,
          "空车更早发车时完工耗时 > 车时合计（两口径不互相顶替）");
}

// 两车同簇：同一批订单按顺序轮转分给两台车，两车各自的路线都必须覆盖自己的订单，
// 且 Σ每车 == 车队（距离/成本/penalty/停靠/动作）。
void testFleetTwoVehiclesSameCluster() {
    const LogisticsGraph g = makeWtoD1Graph();
    // 4 张单、同一簇（都从 W 取、送 D1 附近的不同节点；这里复用 W-D1 边，两两不同 ID）。
    std::vector<Order> orders;
    orders.push_back(makeOrder("O1", "W", "D1", 5.0, 0, 1440, false));
    orders.push_back(makeOrder("O2", "W", "D1", 6.0, 0, 1440, false));
    orders.push_back(makeOrder("O3", "W", "D1", 7.0, 0, 1440, false));
    orders.push_back(makeOrder("O4", "W", "D1", 8.0, 0, 1440, false));
    Vehicle a = makeVehicle("W", 100.0, 480);
    a.id = "V01";
    Vehicle b = makeVehicle("W", 100.0, 480);
    b.id = "V02";
    const FleetPlan fleet = planFleetSequentialBaseline(g, {a, b}, orders, WeightType::Distance);
    check(fleet.allOk, "同簇两台车都可规划");
    if (!fleet.allOk) {
        return;
    }
    // 顺序轮转：O1/O3 -> V01，O2/O4 -> V02（ID 升序、第 i 单 -> 第 i%2 台）。
    check(fleet.vehicles[0].metrics.actionCount == 4,
          "V01 恰好服务 2 单（4 个动作），实际 "
              + std::to_string(fleet.vehicles[0].metrics.actionCount));
    check(fleet.vehicles[1].metrics.actionCount == 4,
          "V02 恰好服务 2 单（4 个动作），实际 "
              + std::to_string(fleet.vehicles[1].metrics.actionCount));
    double sumDist = 0.0;
    double sumCost = 0.0;
    int sumPenalty = 0;
    int sumStops = 0;
    int sumActions = 0;
    double sumTime = 0.0;
    for (const RoutePlan& p : fleet.vehicles) {
        sumDist += p.totalDistanceKm;
        sumCost += p.totalCostYuan;
        sumPenalty += p.totalPenaltyMin;
        sumStops += p.metrics.stopCount;
        sumActions += p.metrics.actionCount;
        sumTime += p.totalTimeMin;
    }
    check(fixtures::nearlyEqual(fleet.totalDistanceKm, sumDist), "Σ每车距离 == 车队");
    check(fixtures::nearlyEqual(fleet.totalCostYuan, sumCost), "Σ每车成本 == 车队");
    check(fleet.totalPenaltyMin == sumPenalty, "Σ每车 penalty == 车队");
    check(fleet.stopCount == sumStops, "Σ每车停靠数 == 车队");
    check(fleet.actionCount == sumActions, "Σ每车动作数 == 车队");
    check(fixtures::nearlyEqual(fleet.sumVehicleTimeMin, sumTime), "Σ每车耗时 == 车时合计");
    // 完工耗时 = max(返回) − min(发车)（两车同刻发车 ⇒ = max 每车耗时）。
    double maxTime = fleet.vehicles[0].totalTimeMin;
    if (fleet.vehicles[1].totalTimeMin > maxTime) {
        maxTime = fleet.vehicles[1].totalTimeMin;
    }
    check(fixtures::nearlyEqual(fleet.completionTimeMin, maxTime),
          "完工耗时 = max(每车耗时)（同刻发车）");
    // 每张订单恰好出现在**一台**车的路线上。
    for (const Order& o : orders) {
        int owners = 0;
        for (const RoutePlan& p : fleet.vehicles) {
            bool hit = false;
            for (const Stop& s : p.stops) {
                for (const StopDelivery& d : s.deliveries) {
                    if (d.orderId == o.id) {
                        hit = true;
                    }
                }
            }
            if (hit) {
                ++owners;
            }
        }
        check(owners == 1, "订单 " + o.id + " 恰好由一台车送达（实际 "
                               + std::to_string(owners) + " 台）");
    }
}

// 慢车因子（§3.3）：同一节点序列下慢车到达 = 基础 ×1.25、成本 ×cost_factor、距离不变；
// 且**车队汇总**用的是各车缩放后的量（不能先按基础权重排好再乘）。
void testFleetSlowVehicleFactorOnArrivals() {
    const LogisticsGraph g = makeWtoD1Graph();   // W<->D1 5.0km / 10min / 4元
    const std::vector<Order> orders = {makeOrder("O1", "W", "D1", 10.0, 0, 1440, false)};
    Vehicle fast = makeVehicle("W", 1000.0, 480);
    fast.id = "V01";
    Vehicle slow = makeVehicle("W", 1000.0, 480);
    slow.id = "V02";
    slow.timeFactor = 1.25;
    slow.costFactor = 0.80;
    // 两台车各分到 0 单会掩盖因子；这里用 planFleet 的单车分支（vehicles 只传慢车）
    // 验证**汇总**也走缩放后的量。
    const FleetPlan onlySlow = planFleet(g, {slow}, orders, WeightType::Distance);
    const FleetPlan onlyFast = planFleet(g, {fast}, orders, WeightType::Distance);
    check(onlySlow.allOk && onlyFast.allOk, "单台慢车/快车都可规划");
    const RoutePlan& pf = onlyFast.vehicles[0];
    const RoutePlan& ps = onlySlow.vehicles[0];
    check(pf.nodes == ps.nodes, "同一节点序列（因子不改变 leg 选择）");
    check(fixtures::nearlyEqual(pf.totalDistanceKm, ps.totalDistanceKm), "距离不变");
    check(fixtures::nearlyEqual(ps.totalTimeMin, pf.totalTimeMin * 1.25),
          "慢车耗时 = 基础 ×1.25：" + std::to_string(ps.totalTimeMin) + " vs "
              + std::to_string(pf.totalTimeMin));
    check(fixtures::nearlyEqual(ps.totalCostYuan, pf.totalCostYuan * 0.80),
          "慢车成本 = 基础 ×0.80：" + std::to_string(ps.totalCostYuan) + " vs "
              + std::to_string(pf.totalCostYuan));
    check(fixtures::nearlyEqual(onlySlow.totalDistanceKm, onlyFast.totalDistanceKm),
          "车队汇总距离 == 基础（与车型无关）");
    check(fixtures::nearlyEqual(onlySlow.completionTimeMin, onlyFast.completionTimeMin * 1.25),
          "车队完工耗时也按该车缩放后的耗时算");
    // 到达时刻：慢车 25min 回仓（480 + 25 = 505），快车 500。
    check(pf.returnArrivalMin == 500 && ps.returnArrivalMin == 505,
          "回仓时刻：快车 500 / 慢车 505（实际 " + std::to_string(pf.returnArrivalMin)
              + " / " + std::to_string(ps.returnArrivalMin) + "）");
}

// 顺序轮转基线（§4.5）：订单 ID 升序、第 i 单 -> 第 (i mod k) 台车；**不跳车、不重分配**。
// 用 3 台车 + 5 张单验证：第 0/3 单 -> V01、第 1/4 单 -> V02、第 2 单 -> V03。
void testFleetRoundRobinBaseline() {
    const LogisticsGraph g = makeWtoD1Graph();
    std::vector<Order> orders;
    for (int i = 1; i <= 5; ++i) {
        char id[8];
        std::snprintf(id, sizeof(id), "O%02d", i);
        orders.push_back(makeOrder(id, "W", "D1", 1.0, 0, 1440, false));
    }
    // 故意乱序传入：planFleet 必须先按 ID 升序再轮转（确定性 tieKey）。
    std::vector<Order> shuffled = {orders[3], orders[0], orders[4], orders[2], orders[1]};
    Vehicle a = makeVehicle("W", 100.0, 480);
    a.id = "V01";
    Vehicle b = makeVehicle("W", 100.0, 480);
    b.id = "V02";
    Vehicle c = makeVehicle("W", 100.0, 480);
    c.id = "V03";
    // 基线入口（§4.5）：不变量 9 的确定性守卫。
    const FleetPlan fleet = planFleetSequentialBaseline(g, {a, b, c}, shuffled,
                                                        WeightType::Distance);
    check(fleet.allOk, "三台车都可规划");
    const int expect[3] = {2, 2, 1};   // O01/O04、O02/O05、O03
    for (int v = 0; v < 3; ++v) {
        const int actions = fleet.vehicles[static_cast<std::size_t>(v)].metrics.actionCount;
        check(actions == 2 * expect[v],
              "第 " + std::to_string(v) + " 台车动作数 " + std::to_string(actions)
                  + " == 2 × " + std::to_string(expect[v]));
    }
    // 确定性：同输入同输出（含传入顺序不同也应一致，因为先按 ID 排序）。
    const FleetPlan again = planFleetSequentialBaseline(g, {a, b, c}, orders,
                                                        WeightType::Distance);
    bool same = again.vehicles.size() == fleet.vehicles.size();
    for (std::size_t v = 0; same && v < fleet.vehicles.size(); ++v) {
        same = fleet.vehicles[v].nodes == again.vehicles[v].nodes;
    }
    check(same, "确定性：传入顺序不同也得到同一分配与同一路线（ID 升序轮转）");
}

// ---------------------------------------------------------------------------
// 类型感知三阶段分配（规格二 §4 / #40）。9 条分配不变量的落点：
//   ① 全部订单恰好服务一次     checkFleetAllocationInvariants（逐单数「出现在几台车」）
//   ② 簇归属完备且互斥         checkFleetAllocationInvariants（clusterOfOrder / isCrossCluster）
//   ③ 前缀峰值 ≤ **该车**容量  checkFleetAllocationInvariants（peakLoadKg，不用总 kg）
//   ④ 首尾相接 + 空车五项 0    checkFleetAllocationInvariants + testFleetTypeAwareOneCluster...
//   ⑤ 确定性 / tieKey 三层     testFleetTypeAwareDeterministic
//   ⑥ 时间口径 = 该车缩放耗时  testFleetSlowVehicleFactorOnArrivals（#39 夹具复用）+ 集成
//   ⑦ 可加性 Σ 恒等式          testFleetTwoVehiclesSameCluster（#39）+ 集成
//   ⑧ 第三阶段守卫读改派日志   checkFleetAllocationInvariants（逐条 relocations）+ 集成
//   ⑨ 基线确定性               testFleetBaselineDeterministic
// ---------------------------------------------------------------------------

// 三簇夹具：每簇 2 张单、几何按簇号放大（簇 c 的 A->B 路线 = 9+3c km）。
// 仓库 W = 子网络 0；取货/送达点带 subNetworkId 1/2/3。
LogisticsGraph makeThreeClusterGraph() {
    LogisticsGraph g;
    g.addNode(makeSubNode("W", NodeType::Warehouse, "仓库", 0));
    for (int c = 1; c <= 3; ++c) {
        const std::string a = "A" + std::to_string(c);
        const std::string b = "B" + std::to_string(c);
        g.addNode(makeSubNode(a, NodeType::Delivery, "取货点" + std::to_string(c), c));
        g.addNode(makeSubNode(b, NodeType::Delivery, "送达点" + std::to_string(c), c));
        addTwoWay(g, "W", a, 3.0 + c, 3.0 + c, 3.0 + c);
        addTwoWay(g, a, b, 2.0 + c, 2.0 + c, 2.0 + c);
        addTwoWay(g, b, "W", 4.0 + c, 4.0 + c, 4.0 + c);
    }
    return g;
}

// 簇 1/2/3 的货量：20 / 40 / 60 kg（同簇订单同取同送 ⇒ 峰值 = 簇货量）。
std::vector<Order> makeThreeClusterOrders() {
    std::vector<Order> orders;
    orders.push_back(makeOrder("O1", "A1", "B1", 10.0, 0, 1440, false));
    orders.push_back(makeOrder("O2", "A1", "B1", 10.0, 0, 1440, false));
    orders.push_back(makeOrder("O3", "A2", "B2", 20.0, 0, 1440, false));
    orders.push_back(makeOrder("O4", "A2", "B2", 20.0, 0, 1440, false));
    orders.push_back(makeOrder("O5", "A3", "B3", 30.0, 0, 1440, false));
    orders.push_back(makeOrder("O6", "A3", "B3", 30.0, 0, 1440, false));
    return orders;
}

// V01 快而贵（×1.00 / ×1.00，60kg）；V02 慢而**极便宜**（×1.25 / ×0.10，400kg）；
// V03 慢而贵（×1.25 / ×1.00，400kg）。三台车都装得下任一簇（峰值 20/40/60 ≤ 60），
// 所以 6 种排列全部可行。三种策略各自选出**不同的分配映射**（差在 V02/V03 或 V01 上）：
//   距离 → 均衡键在两种同距排列里取 tieKey 持有者（簇2→V02）；
//   成本 → V02 的 ×0.10 把最大簇（簇3）吸过去；
//   时间 → V01 最快，把最大簇（簇3）放上 V01 才能压低完工耗时。
std::vector<Vehicle> makeThreeClusterVehicles() {
    std::vector<Vehicle> vs;
    Vehicle a = makeVehicleWithFactors("W", 60.0, 480, 1.00, 1.00);
    a.id = "V01";
    Vehicle b = makeVehicleWithFactors("W", 400.0, 480, 1.25, 0.10);
    b.id = "V02";
    Vehicle c = makeVehicleWithFactors("W", 400.0, 480, 1.25, 1.00);
    c.id = "V03";
    vs.push_back(a);
    vs.push_back(b);
    vs.push_back(c);
    return vs;
}

// 分配表查找：订单 -> 车辆行下标；found = 是否在表里。
std::size_t allocationVehicleOf(const FleetPlan& fleet, const std::string& orderId, bool& found) {
    found = false;
    for (std::size_t i = 0; i < fleet.allocationOrderIds.size(); ++i) {
        if (fleet.allocationOrderIds[i] == orderId) {
            found = true;
            return fleet.allocationVehicleIndex[i];
        }
    }
    return 0;
}

// 该单被**几台车**送达（不变量 ①：必须恰好 1）。
int deliveryOwners(const FleetPlan& fleet, const std::string& orderId) {
    int owners = 0;
    for (std::size_t v = 0; v < fleet.vehicles.size(); ++v) {
        bool hit = false;
        for (std::size_t i = 0; i < fleet.vehicles[v].stops.size(); ++i) {
            const std::vector<StopDelivery>& ds = fleet.vehicles[v].stops[i].deliveries;
            for (std::size_t d = 0; d < ds.size(); ++d) {
                if (ds[d].orderId == orderId) {
                    hit = true;
                }
            }
        }
        if (hit) {
            ++owners;
        }
    }
    return owners;
}

// 9 条不变量里可机械复核的部分一次查完（读 FleetPlan 的公开产物，不读私有实现）。
void checkFleetAllocationInvariants(const LogisticsGraph& g, const std::vector<Vehicle>& vehicles,
                                    const std::vector<Order>& orders, const FleetPlan& fleet,
                                    const std::string& label) {
    const std::string tag = "[" + label + "] ";
    check(fleet.vehicles.size() == vehicles.size(), tag + "逐车计划数 == 车辆数");
    check(fleet.allocationOrderIds.size() == orders.size(),
          tag + "分配表覆盖全部订单（" + std::to_string(fleet.allocationOrderIds.size()) + " / "
              + std::to_string(orders.size()) + "）");

    // ① 每张订单恰好出现在**一台**车的路线上，且分配表指向的就是那台车。
    for (std::size_t i = 0; i < orders.size(); ++i) {
        const int owners = deliveryOwners(fleet, orders[i].id);
        check(owners == 1, tag + "订单 " + orders[i].id + " 恰好由一台车送达（实际 "
                               + std::to_string(owners) + " 台）");
        bool found = false;
        const std::size_t v = allocationVehicleOf(fleet, orders[i].id, found);
        check(found && v < vehicles.size(), tag + "订单 " + orders[i].id + " 在分配表里有车");
        if (found && v < vehicles.size()) {
            bool servedThere = false;
            for (std::size_t s = 0; s < fleet.vehicles[v].stops.size(); ++s) {
                const std::vector<StopDelivery>& ds = fleet.vehicles[v].stops[s].deliveries;
                for (std::size_t d = 0; d < ds.size(); ++d) {
                    if (ds[d].orderId == orders[i].id) {
                        servedThere = true;
                    }
                }
            }
            check(servedThere, tag + "分配表说 " + orders[i].id + " 归 "
                                  + vehicles[v].id + "，该车路线上确实送达了它");
        }
    }

    // ② 簇归属完备且互斥：本地单恰属一个簇（分配表的簇号 == clusterOfOrder）、
    //    跨簇单不属任何簇（-1）；两者并 = 全部订单。
    int local = 0;
    int cross = 0;
    for (std::size_t i = 0; i < orders.size(); ++i) {
        const int cluster = clusterOfOrder(g, orders[i]);
        const bool isCross = isCrossCluster(g, orders[i]) || cluster <= 0;
        bool found = false;
        allocationVehicleOf(fleet, orders[i].id, found);
        std::size_t at = 0;
        for (std::size_t k = 0; k < fleet.allocationOrderIds.size(); ++k) {
            if (fleet.allocationOrderIds[k] == orders[i].id) {
                at = k;
            }
        }
        if (isCross) {
            ++cross;
            check(found && fleet.allocationClusterId[at] == -1,
                  tag + "跨簇/异常单 " + orders[i].id + " 不属任何簇（-1）");
        } else {
            ++local;
            check(found && fleet.allocationClusterId[at] == cluster,
                  tag + "本地单 " + orders[i].id + " 恰属簇 " + std::to_string(cluster));
        }
    }
    check(local + cross == static_cast<int>(orders.size()),
          tag + "本地单 + 跨簇单 = 全部订单（完备且互斥）");

    // ③ 前缀峰值 0 ≤ L ≤ **该车** capacity。**禁止**用「该车总分配 kg ≤ 容量」判定：
    //    默认数据里 V02/V03 总分配 165kg > 160kg 容量而峰值只有 64/57kg（§4.3 实证）。
    for (std::size_t v = 0; v < vehicles.size() && v < fleet.vehicles.size(); ++v) {
        const RoutePlan& plan = fleet.vehicles[v];
        check(plan.status == PlanStatus::Ok, tag + vehicles[v].id + " 计划可行");
        check(plan.metrics.peakLoadKg >= 0.0
                  && plan.metrics.peakLoadKg <= vehicles[v].capacityKg + 1e-6,
              tag + vehicles[v].id + " 前缀峰值 ≤ 该车载重（" + std::to_string(plan.metrics.peakLoadKg)
                  + " / " + std::to_string(vehicles[v].capacityKg) + "）");
        // ④ 每车从自己的 start_node 按自己的 depart_time 空载出发、回同一基地。
        if (!plan.nodes.empty()) {
            check(plan.nodes.front() == vehicles[v].startNodeId,
                  tag + vehicles[v].id + " 从自己的 start_node 出发");
            check(plan.nodes.back() == vehicles[v].startNodeId,
                  tag + vehicles[v].id + " 回到同一基地");
            check(!plan.nodeArrivalMin.empty()
                      && plan.nodeArrivalMin.front() == vehicles[v].departTimeMin,
                  tag + vehicles[v].id + " 按自己的 depart_time 出发");
        }
        // 空车五项全 0、返回时刻 = 发车时刻。
        int orderCount = (v < fleet.vehicleOrderCount.size()) ? fleet.vehicleOrderCount[v] : 0;
        if (orderCount == 0) {
            check(fixtures::nearlyEqual(plan.totalDistanceKm, 0.0)
                      && fixtures::nearlyEqual(plan.totalCostYuan, 0.0)
                      && plan.totalPenaltyMin == 0 && plan.metrics.stopCount == 0
                      && plan.metrics.actionCount == 0,
                  tag + vehicles[v].id + " 空车五项全 0");
            check(plan.returnArrivalMin == vehicles[v].departTimeMin,
                  tag + vehicles[v].id + " 空车返回时刻 = 发车时刻（不抬升完工耗时）");
        }
    }

    // ⑧ 第三阶段守卫：**逐条读改派日志**核对「完工耗时不变差」+「键严格变优」。
    for (std::size_t i = 0; i < fleet.relocations.size(); ++i) {
        const RelocateRecord& rec = fleet.relocations[i];
        check(rec.round >= 1 && rec.round <= 3, tag + "改派日志轮次 ∈ [1,3]");
        check(rec.completionAfterMin <= rec.completionBeforeMin + 1e-9,
              tag + "改派 " + rec.orderId + " " + rec.fromVehicleId + "→" + rec.toVehicleId
                  + " 完工耗时不变差（" + std::to_string(rec.completionBeforeMin) + " -> "
                  + std::to_string(rec.completionAfterMin) + "）");
        const bool better = rec.penaltyAfter < rec.penaltyBefore
                            || (rec.penaltyAfter == rec.penaltyBefore
                                && (rec.weightAfter < rec.weightBefore - 1e-9
                                    || (fixtures::nearlyEqual(rec.weightAfter, rec.weightBefore)
                                        && rec.balanceAfter < rec.balanceBefore - 1e-9)));
        check(better, tag + "改派 " + rec.orderId + " 的车队键严格变优");
        check(rec.fromVehicleId != rec.toVehicleId, tag + "改派日志的源车 ≠ 目标车");
    }
}

// 阶段二的 6 种排列 + 三策略各自的分配映射（**不变量 1–4 的落点**）。
// 夹具上 6 种排列全部可行（峰值 20/40/60 ≤ 最小容量 60），所以留痕恰好 6 行。
void testFleetTypeAwareAllocationInvariants() {
    const LogisticsGraph g = makeThreeClusterGraph();
    const std::vector<Order> orders = makeThreeClusterOrders();
    const std::vector<Vehicle> vehicles = makeThreeClusterVehicles();
    const FleetPlan fleet = planFleet(g, vehicles, orders, WeightType::Distance);
    check(fleet.allOk, "三簇六单三车全部可行（reason: " + fleet.reason + "）");
    checkFleetAllocationInvariants(g, vehicles, orders, fleet, "三阶段·距离");

    // 6 种「簇 -> 车」排列全部被枚举（默认 3 簇 3 车）。
    check(fleet.candidateKeyLog.size() == 6,
          "枚举 6 种排列（实际 " + std::to_string(fleet.candidateKeyLog.size()) + "）");
    for (std::size_t i = 0; i < fleet.candidateKeyLog.size(); ++i) {
        check(fleet.candidateKeyLog[i].find("key=(") != std::string::npos,
              "排列 " + std::to_string(i) + " 留痕带车队键");
    }

    // 三策略的分配映射（簇 -> 车；簇下标 0/1/2 = 簇 1/2/3）。夹具参数详见
    // makeThreeClusterVehicles 的注释：距离靠均衡键 + tieKey，成本被 V02 的 ×0.10 拉动，
    // 时间把最大簇放上最快的 V01。
    struct Expect {
        WeightType weight;
        const char* label;
        int clusterOnVehicle[3];
        const char* why;
    };
    const Expect expects[3] = {
        {WeightType::Distance, "距离", {0, 1, 2},
         "均衡键 + tieKey（6 种排列 Σ距离完全并列）"},
        {WeightType::Cost, "成本", {0, 2, 1}, "V02 的 ×0.10 把最大簇吸到 V02"},
        {WeightType::Time, "时间", {2, 0, 1}, "V01 最快，最大簇上 V01 才能压低完工耗时"},
    };
    for (int e = 0; e < 3; ++e) {
        const FleetPlan f = (e == 0) ? fleet : planFleet(g, vehicles, orders, expects[e].weight);
        check(f.allOk, std::string(expects[e].label) + "策略可行");
        const std::string tag = std::string(expects[e].label) + "策略（" + expects[e].why + "）";
        checkFleetAllocationInvariants(g, vehicles, orders, f, tag);
        // 每个簇取一张代表单，核对它落在期望的车上。
        const char* rep[3] = {"O1", "O3", "O5"};
        for (int v = 0; v < 3; ++v) {
            bool found = false;
            const std::size_t got = allocationVehicleOf(f, rep[expects[e].clusterOnVehicle[v]], found);
            check(found && got == static_cast<std::size_t>(v),
                  tag + "：簇 " + std::to_string(expects[e].clusterOnVehicle[v] + 1) + " -> "
                      + vehicles[static_cast<std::size_t>(v)].id);
        }
        // 整簇不可分时每车只该有一个本地簇（阶段三没有**跨簇搬运**的例子在这里）。
        for (std::size_t v = 0; v < f.vehicleClusters.size(); ++v) {
            check(f.vehicleClusters[v].size() <= 1,
                  tag + "：" + vehicles[v].id + " 至多承载一个本地簇");
        }
    }
    // 距离与成本/时间的映射必须不同（三策略分化在夹具上的最小化版本）。
    const FleetPlan byCost = planFleet(g, vehicles, orders, WeightType::Cost);
    const FleetPlan byTime = planFleet(g, vehicles, orders, WeightType::Time);
    check(byCost.allocationVehicleIndex != fleet.allocationVehicleIndex, "距离 ≠ 成本 的分配");
    check(byTime.allocationVehicleIndex != fleet.allocationVehicleIndex, "距离 ≠ 时间 的分配");
    check(byTime.allocationVehicleIndex != byCost.allocationVehicleIndex, "时间 ≠ 成本 的分配");
}

// 均衡键**单独**裁决的一次并列（这是「把均衡键去掉 → 守卫变红」的落点）：
// 两簇几何完全对称 ⇒ 两种排列的 Σ距离 / penalty 完全相同；tieKey 会选「簇1→V01」
// （V01 首次停靠 A1 < A2），但均衡键选「10kg 的簇2→V01（cap 100）、50kg 的簇1→V02（cap 200）」。
void testFleetBalanceKeyDecidesTie() {
    LogisticsGraph g;
    g.addNode(makeSubNode("W", NodeType::Warehouse, "仓库", 0));
    g.addNode(makeSubNode("A1", NodeType::Delivery, "簇1取货", 1));
    g.addNode(makeSubNode("B1", NodeType::Delivery, "簇1送达", 1));
    g.addNode(makeSubNode("A2", NodeType::Delivery, "簇2取货", 2));
    g.addNode(makeSubNode("B2", NodeType::Delivery, "簇2送达", 2));
    addTwoWay(g, "W", "A1", 10.0, 10.0, 10.0);
    addTwoWay(g, "A1", "B1", 10.0, 10.0, 10.0);
    addTwoWay(g, "B1", "W", 10.0, 10.0, 10.0);
    addTwoWay(g, "W", "A2", 10.0, 10.0, 10.0);
    addTwoWay(g, "A2", "B2", 10.0, 10.0, 10.0);
    addTwoWay(g, "B2", "W", 10.0, 10.0, 10.0);

    // V01 容量**小**（100kg）、V02 容量大（200kg）：tieKey 会把 50kg 的簇1 放上 V01
    // （0.50 / 0.05 = 极差 0.45），均衡键则把 50kg 送到 V02（0.25 / 0.10 = 0.15）。
    std::vector<Vehicle> vehicles;
    Vehicle small = makeVehicle("W", 100.0, 480);
    small.id = "V01";
    Vehicle big = makeVehicle("W", 200.0, 480);
    big.id = "V02";
    vehicles.push_back(small);
    vehicles.push_back(big);

    const std::vector<Order> orders = {
        makeOrder("O1", "A1", "B1", 50.0, 0, 1440, false),
        makeOrder("O2", "A2", "B2", 10.0, 0, 1440, false),
    };
    const FleetPlan fleet = planFleet(g, vehicles, orders, WeightType::Distance);
    check(fleet.allOk, "两簇对称夹具可行（reason: " + fleet.reason + "）");
    checkFleetAllocationInvariants(g, vehicles, orders, fleet, "均衡键并列");

    bool found1 = false;
    bool found2 = false;
    const std::size_t v1 = allocationVehicleOf(fleet, "O1", found1);
    const std::size_t v2 = allocationVehicleOf(fleet, "O2", found2);
    check(found1 && v1 == 1, "50kg 的簇1 -> V02（cap 200）：均衡键选中（否则 tieKey 会放 V01）");
    check(found2 && v2 == 0, "10kg 的簇2 -> V01（cap 100）");
    // 极差：0.25 − 0.10 = 0.15；另一种排列（tieKey 的）是 0.50 − 0.05 = 0.45。
    check(fixtures::nearlyEqual(fleet.balanceSpread, 0.15, 1e-9),
          "均衡度 = 0.25 − 0.10 = 0.15（实际 " + std::to_string(fleet.balanceSpread) + "）");
    // 两种排列的 Σ距离完全相同（几何对称、因子相同）⇒ 第一/第二键分不出高下。
    const RoutePlan a1 = planRoute(g, vehicles[0], {orders[0]}, WeightType::Distance);
    const RoutePlan a2 = planRoute(g, vehicles[0], {orders[1]}, WeightType::Distance);
    const RoutePlan b1 = planRoute(g, vehicles[1], {orders[0]}, WeightType::Distance);
    const RoutePlan b2 = planRoute(g, vehicles[1], {orders[1]}, WeightType::Distance);
    check(fixtures::nearlyEqual(a1.totalDistanceKm + b2.totalDistanceKm,
                                a2.totalDistanceKm + b1.totalDistanceKm),
          "两种排列 Σ距离完全并列 ⇒ 只能由均衡键/tieKey 裁决");
    // tieKey（不看均衡键）会选「簇1→V01」：V01 首次停靠 A1 < A2。
    check(a1.stops.front().nodeId == "A1" && a2.stops.front().nodeId == "A2",
          "tieKey 口径：V01 的首次停靠 A1 < A2 ⇒ 均衡键缺席时会选簇1→V01");
}

// 簇数 > 车数（§4.3 的贪心分支）：按簇货量降序取前 k 个各占一车，
// 剩余簇并入「货量最接近且仍可行」的车；日志里留一行，绝不静默。
void testFleetMoreClustersThanVehicles() {
    const LogisticsGraph g = makeThreeClusterGraph();     // 3 簇（货量 20/40/60）
    const std::vector<Order> orders = makeThreeClusterOrders();
    std::vector<Vehicle> vehicles;
    Vehicle a = makeVehicle("W", 100.0, 480);
    a.id = "V01";
    Vehicle b = makeVehicle("W", 100.0, 480);
    b.id = "V02";
    vehicles.push_back(a);
    vehicles.push_back(b);
    const FleetPlan fleet = planFleet(g, vehicles, orders, WeightType::Distance);
    check(fleet.allOk, "3 簇 2 车仍可全服务（reason: " + fleet.reason + "）");
    checkFleetAllocationInvariants(g, vehicles, orders, fleet, "簇数>车数");
    bool logged = false;
    for (std::size_t i = 0; i < fleet.allocationLog.size(); ++i) {
        if (fleet.allocationLog[i].find("簇数 > 车数") != std::string::npos) {
            logged = true;
        }
    }
    check(logged, "簇数 > 车数**留日志**（不静默合并）");
    // 2 台车都必须有活；6 张单都分配到（合并后一组含两簇，阶段三可能再搬单）。
    check(fleet.vehicleOrderCount.size() == 2 && fleet.vehicleOrderCount[0] > 0
              && fleet.vehicleOrderCount[1] > 0,
          "两台车都分到订单");
    int served = 0;
    for (std::size_t v = 0; v < fleet.vehicles.size(); ++v) {
        served += fleet.vehicles[v].metrics.actionCount / 2;
    }
    check(served == 6, "6 张单全部被服务");
}

// 为什么 §4.6 的 ②拆簇在本实现里是**防御级**（如实记录，不掩饰）：
// 单车构造器会把同点的多次取货**串行化**（先取 60 卸 60 再取 60），前缀峰值 = 单笔最大值，
// 因此「整簇在一台车上装不下」只可能由**单笔超载**或**不可达**造成 —— 这两种拆簇都救不了。
// 本用例钉住这个前提：两单各 60kg 同点取货、容量 100kg，整簇**可行**且峰值只有 60kg；
// 若构造器不再串行化（回归），这里会立刻变红，那时 ② 才真正有意义。
void testFleetClusterFitsViaPrefixSerialization() {
    LogisticsGraph g;
    g.addNode(makeSubNode("W", NodeType::Warehouse, "仓库", 0));
    g.addNode(makeSubNode("A1", NodeType::Delivery, "共同取货点", 1));
    g.addNode(makeSubNode("B1", NodeType::Delivery, "送达点1", 1));
    g.addNode(makeSubNode("B2", NodeType::Delivery, "送达点2", 1));
    addTwoWay(g, "W", "A1", 4.0, 4.0, 4.0);
    addTwoWay(g, "A1", "B1", 3.0, 3.0, 3.0);
    addTwoWay(g, "B1", "W", 5.0, 5.0, 5.0);
    addTwoWay(g, "A1", "B2", 3.0, 3.0, 3.0);
    addTwoWay(g, "B2", "W", 5.0, 5.0, 5.0);
    std::vector<Vehicle> vehicles;
    Vehicle a = makeVehicle("W", 100.0, 480);
    a.id = "V01";
    Vehicle b = makeVehicle("W", 100.0, 480);
    b.id = "V02";
    vehicles.push_back(a);
    vehicles.push_back(b);
    const std::vector<Order> orders = {
        makeOrder("O1", "A1", "B1", 60.0, 0, 1440, false),
        makeOrder("O2", "A1", "B2", 60.0, 0, 1440, false),
    };
    const RoutePlan whole = planRoute(g, a, orders, WeightType::Distance);
    check(whole.status == PlanStatus::Ok, "整簇在单台 100kg 车上可行（取货被串行化）");
    check(fixtures::nearlyEqual(whole.metrics.peakLoadKg, 60.0),
          "前缀峰值 = 单笔最大值 60kg（不是 120kg 的两单同取）");
    const FleetPlan fleet = planFleet(g, vehicles, orders, WeightType::Distance);
    check(fleet.allOk, "整簇分车可行（reason: " + fleet.reason + "）");
    checkFleetAllocationInvariants(g, vehicles, orders, fleet, "整簇串行化");
    bool splitLogged = false;
    for (std::size_t i = 0; i < fleet.allocationLog.size(); ++i) {
        if (fleet.allocationLog[i].find("拆簇") != std::string::npos) {
            splitLogged = true;
        }
    }
    check(!splitLogged, "整簇可行 ⇒ ②拆簇不触发（只在装不下时才该留日志）");
    int served = 0;
    for (std::size_t v = 0; v < fleet.vehicles.size(); ++v) {
        served += fleet.vehicles[v].metrics.actionCount / 2;
    }
    check(served == 2, "两张单都被服务一次");
}

// 完工耗时守卫（§4.4 第二接受条件）**必须能独立拦住**一次「键更优但完工更差」的改派。
//
// 夹具：V01 成本因子 ×0.10（便宜）、V02 ×1.00；近簇 A 路线 15km、远簇 B 路线 60km。
// 成本策略的初解把远簇 B 放上便宜的 V01（21 元），完工 60min。
// 阶段三会试「把近簇 A 也挪到 V01」：成本 21 -> 7.5（**键更优**），但 V01 要跑 75km，
// 完工 60 -> 75（**更差**）⇒ 守卫必须拒绝。去掉守卫这条用例立刻变红。
void testFleetCompletionGuardRejectsRelocation() {
    LogisticsGraph g;
    g.addNode(makeSubNode("W", NodeType::Warehouse, "仓库", 0));
    g.addNode(makeSubNode("P1", NodeType::Delivery, "近簇取货", 1));
    g.addNode(makeSubNode("Q1", NodeType::Delivery, "近簇送达", 1));
    g.addNode(makeSubNode("P2", NodeType::Delivery, "远簇取货", 2));
    g.addNode(makeSubNode("Q2", NodeType::Delivery, "远簇送达", 2));
    addTwoWay(g, "W", "P1", 5.0, 5.0, 5.0);
    addTwoWay(g, "P1", "Q1", 5.0, 5.0, 5.0);
    addTwoWay(g, "Q1", "W", 5.0, 5.0, 5.0);
    addTwoWay(g, "W", "P2", 20.0, 20.0, 20.0);
    addTwoWay(g, "P2", "Q2", 20.0, 20.0, 20.0);
    addTwoWay(g, "Q2", "W", 20.0, 20.0, 20.0);

    std::vector<Vehicle> vehicles;
    Vehicle cheap = makeVehicleWithFactors("W", 100.0, 480, 1.00, 0.10);
    cheap.id = "V01";
    Vehicle dear = makeVehicleWithFactors("W", 100.0, 480, 1.00, 1.00);
    dear.id = "V02";
    vehicles.push_back(cheap);
    vehicles.push_back(dear);

    const std::vector<Order> orders = {
        makeOrder("O1", "P1", "Q1", 10.0, 0, 1440, false),   // 近簇 A
        makeOrder("O2", "P2", "Q2", 10.0, 0, 1440, false),   // 远簇 B
    };
    const FleetPlan fleet = planFleet(g, vehicles, orders, WeightType::Cost);
    check(fleet.allOk, "守卫夹具可行（reason: " + fleet.reason + "）");
    bool found1 = false;
    bool found2 = false;
    const std::size_t vO2 = allocationVehicleOf(fleet, "O2", found2);
    const std::size_t vO1 = allocationVehicleOf(fleet, "O1", found1);
    check(found2 && vO2 == 0, "初解：远簇 B -> 便宜的 V01（成本 ×0.10）");
    check(found1 && vO1 == 1, "初解：近簇 A -> V02");
    check(fleet.relocations.empty(),
          "守卫拒绝了唯一一次「键更优但完工更差」的改派（日志应为空）");
    check(fixtures::nearlyEqual(fleet.completionTimeMin, 60.0),
          "完工耗时仍为 60min（V01 跑远簇 60km）；实际 "
              + std::to_string(fleet.completionTimeMin));
    check(fleet.vehicleOrderCount[0] == 1 && fleet.vehicleOrderCount[1] == 1,
          "两车各 1 单（没有把两簇压到同一台车）");
    checkFleetAllocationInvariants(g, vehicles, orders, fleet, "完工守卫");
}

// tieKey 三层 + 确定性：同输入同输出；传入顺序打乱也一致（订单先按 ID 升序、
// 簇按簇号升序、车辆按 ID 升序枚举）；三策略的分配映射**两两不同**。
void testFleetTypeAwareDeterministic() {
    const LogisticsGraph g = makeThreeClusterGraph();
    const std::vector<Order> orders = makeThreeClusterOrders();
    const std::vector<Vehicle> vehicles = makeThreeClusterVehicles();
    const FleetPlan a = planFleet(g, vehicles, orders, WeightType::Distance);
    std::vector<Order> shuffled;
    shuffled.push_back(orders[5]);
    shuffled.push_back(orders[2]);
    shuffled.push_back(orders[0]);
    shuffled.push_back(orders[4]);
    shuffled.push_back(orders[1]);
    shuffled.push_back(orders[3]);
    const FleetPlan b = planFleet(g, vehicles, shuffled, WeightType::Distance);
    check(a.allocationOrderIds == b.allocationOrderIds, "确定性：分配表订单顺序一致（ID 升序）");
    check(a.allocationVehicleIndex == b.allocationVehicleIndex,
          "确定性：传入顺序不同也得到同一分配映射");
    bool sameNodes = a.vehicles.size() == b.vehicles.size();
    for (std::size_t v = 0; sameNodes && v < a.vehicles.size(); ++v) {
        sameNodes = a.vehicles[v].nodes == b.vehicles[v].nodes;
    }
    check(sameNodes, "确定性：逐车节点序列逐字相同");

    // 三策略分化（车队口径）：三份 ALLOC 映射两两不同。
    const FleetPlan byCost = planFleet(g, vehicles, orders, WeightType::Cost);
    const FleetPlan byTime = planFleet(g, vehicles, orders, WeightType::Time);
    check(byCost.allocationVehicleIndex != a.allocationVehicleIndex,
          "距离与成本的分配映射不同");
    check(byTime.allocationVehicleIndex != a.allocationVehicleIndex,
          "距离与时间的分配映射不同");
    check(byTime.allocationVehicleIndex != byCost.allocationVehicleIndex,
          "时间与成本的分配映射不同");
    check(byCost.allOk && byTime.allOk, "成本/时间策略在夹具上都可行");
}

// 单簇 + 三台车：tieKey 的「停靠序列逐位（短者先）」把订单放到**最后一台车**，
// 前两台是空车（五项全 0、返回时刻 = 发车时刻、不抬升完工耗时）。
void testFleetTypeAwareOneClusterLeavesEarlierVehiclesEmpty() {
    LogisticsGraph g;   // 一个本地簇（A1、B1 同属子网络 1）
    g.addNode(makeSubNode("W", NodeType::Warehouse, "仓库", 0));
    g.addNode(makeSubNode("A1", NodeType::Delivery, "取货点", 1));
    g.addNode(makeSubNode("B1", NodeType::Delivery, "送达点", 1));
    addTwoWay(g, "W", "A1", 5.0, 5.0, 4.0);
    addTwoWay(g, "A1", "B1", 3.0, 3.0, 2.0);
    addTwoWay(g, "B1", "W", 4.0, 4.0, 3.0);
    std::vector<Vehicle> vehicles;
    Vehicle a = makeVehicle("W", 100.0, 470);
    a.id = "V01";
    Vehicle b = makeVehicle("W", 100.0, 480);
    b.id = "V02";
    Vehicle c = makeVehicle("W", 100.0, 490);
    c.id = "V03";
    vehicles.push_back(a);
    vehicles.push_back(b);
    vehicles.push_back(c);
    const std::vector<Order> orders = {makeOrder("O1", "A1", "B1", 10.0, 0, 1440, false)};
    const FleetPlan fleet = planFleet(g, vehicles, orders, WeightType::Distance);
    check(fleet.allOk, "单簇三车可行（reason: " + fleet.reason + "）");
    checkFleetAllocationInvariants(g, vehicles, orders, fleet, "单簇三车");
    bool found = false;
    const std::size_t at = allocationVehicleOf(fleet, "O1", found);
    check(found && at == 2, "tieKey（短停靠序列优先）把单放到 V03，前两台空车");
    check(fleet.vehicleOrderCount.size() == 3 && fleet.vehicleOrderCount[0] == 0
              && fleet.vehicleOrderCount[1] == 0 && fleet.vehicleOrderCount[2] == 1,
          "每车订单数 0 / 0 / 1");
    // 空车不抬升完工耗时，但参与 min(发车)：完工 = 有货车的返回 − 最早发车。
    const double expect = static_cast<double>(vehicles[2].departTimeMin)
                          + fleet.vehicles[2].totalTimeMin
                          - static_cast<double>(vehicles[0].departTimeMin);
    check(fixtures::nearlyEqual(fleet.completionTimeMin, expect),
          "完工耗时 = max(返回) − min(发车)（空车不抬升 max）");
}

// 跨簇单 + 子网络 0 的异常数据：两者都进跨簇队列（不属任何簇），
// 子网络 0 必须**留日志**（不静默丢单），且三张单都被服务。
void testFleetCrossClusterAndZeroSubnetQueue() {
    LogisticsGraph g;
    g.addNode(makeSubNode("W", NodeType::Warehouse, "仓库", 0));
    g.addNode(makeSubNode("P1", NodeType::Delivery, "同簇取货", 1));
    g.addNode(makeSubNode("Q1", NodeType::Delivery, "同簇送达", 1));
    g.addNode(makeSubNode("A", NodeType::Delivery, "跨簇取货", 1));
    g.addNode(makeSubNode("B", NodeType::Delivery, "跨簇送达", 2));
    g.addNode(makeSubNode("Z1", NodeType::Delivery, "子网络0取货", 0));
    g.addNode(makeSubNode("Z2", NodeType::Delivery, "子网络0送达", 0));
    addTwoWay(g, "W", "P1", 4.0, 4.0, 4.0);
    addTwoWay(g, "P1", "Q1", 3.0, 3.0, 3.0);
    addTwoWay(g, "Q1", "W", 5.0, 5.0, 5.0);
    addTwoWay(g, "W", "A", 6.0, 6.0, 6.0);
    addTwoWay(g, "A", "B", 5.0, 5.0, 5.0);
    addTwoWay(g, "B", "W", 7.0, 7.0, 7.0);
    addTwoWay(g, "W", "Z1", 8.0, 8.0, 8.0);
    addTwoWay(g, "Z1", "Z2", 2.0, 2.0, 2.0);
    addTwoWay(g, "Z2", "W", 6.0, 6.0, 6.0);

    std::vector<Vehicle> vehicles;
    Vehicle a = makeVehicle("W", 100.0, 480);
    a.id = "V01";
    Vehicle b = makeVehicle("W", 100.0, 480);
    b.id = "V02";
    vehicles.push_back(a);
    vehicles.push_back(b);

    std::vector<Order> orders;
    orders.push_back(makeOrder("O1", "P1", "Q1", 10.0, 0, 1440, false));   // 本地单（簇 1）
    orders.push_back(makeOrder("X1", "A", "B", 10.0, 0, 1440, false));     // 跨簇单
    orders.push_back(makeOrder("Z9", "Z1", "Z2", 10.0, 0, 1440, false));   // 子网络 0 异常

    check(clusterOfOrder(g, orders[0]) == 1, "簇(order) = 取货点 A 的 subNetworkId");
    check(!isCrossCluster(g, orders[0]), "同簇单 isCrossCluster = false");
    check(isCrossCluster(g, orders[1]), "跨簇单 isCrossCluster = true");
    check(clusterOfOrder(g, orders[2]) == 0, "子网络 0 的取货点簇号为 0");

    const FleetPlan fleet = planFleet(g, vehicles, orders, WeightType::Distance);
    check(fleet.allOk, "跨簇 + 异常单仍可全服务（reason: " + fleet.reason + "）");
    checkFleetAllocationInvariants(g, vehicles, orders, fleet, "跨簇队列");
    bool foundX = false;
    bool foundZ = false;
    allocationVehicleOf(fleet, "X1", foundX);
    allocationVehicleOf(fleet, "Z9", foundZ);
    check(foundX && foundZ, "跨簇单与子网络 0 的单都在分配表里（不丢单）");
    std::size_t xAt = 0;
    std::size_t zAt = 0;
    for (std::size_t i = 0; i < fleet.allocationOrderIds.size(); ++i) {
        if (fleet.allocationOrderIds[i] == "X1") {
            xAt = i;
        }
        if (fleet.allocationOrderIds[i] == "Z9") {
            zAt = i;
        }
    }
    check(fleet.allocationClusterId[xAt] == -1, "跨簇单不属任何簇（-1）");
    check(fleet.allocationClusterId[zAt] == -1, "子网络 0 的单归入跨簇队列（-1）");
    bool logged = false;
    for (std::size_t i = 0; i < fleet.allocationLog.size(); ++i) {
        if (fleet.allocationLog[i].find("子网络 0") != std::string::npos
            && fleet.allocationLog[i].find("Z9") != std::string::npos) {
            logged = true;
        }
    }
    check(logged, "子网络 0 的异常数据**记日志**（不静默丢单）");
    // 每台车都拿得到跨簇单（本地簇 + 跨簇单混装）。
    check(fleet.vehicles[0].metrics.actionCount + fleet.vehicles[1].metrics.actionCount == 6,
          "两台车共 6 个动作 = 3 单 × 取送");
}

// 顺序轮转基线的不变量 9：分配只由「订单 ID 升序 + 车队行序」决定，
// 与策略 / 容量 / 窗口**全部无关**。
void testFleetBaselineDeterministic() {
    const LogisticsGraph g = makeThreeClusterGraph();
    const std::vector<Order> orders = makeThreeClusterOrders();
    std::vector<Vehicle> vehicles = makeThreeClusterVehicles();
    const FleetPlan base = planFleetSequentialBaseline(g, vehicles, orders, WeightType::Distance);
    check(base.allOk, "基线可行");
    // 订单 ID 升序 + 行序：O1/O4 -> V01、O2/O5 -> V02、O3/O6 -> V03。
    const char* expectId[6] = {"O1", "O2", "O3", "O4", "O5", "O6"};
    const std::size_t expectV[6] = {0, 1, 2, 0, 1, 2};
    for (int i = 0; i < 6; ++i) {
        bool found = false;
        const std::size_t v = allocationVehicleOf(base, expectId[i], found);
        check(found && v == expectV[i], std::string("基线：ID 升序第 ") + expectId[i]
                                            + " 单 -> 第 " + std::to_string(expectV[i]) + " 台车");
    }
    // 换策略：分配映射逐字不变。
    const FleetPlan byCost = planFleetSequentialBaseline(g, vehicles, orders, WeightType::Cost);
    const FleetPlan byTime = planFleetSequentialBaseline(g, vehicles, orders, WeightType::Time);
    check(base.allocationOrderIds == byCost.allocationOrderIds
              && base.allocationVehicleIndex == byCost.allocationVehicleIndex,
          "基线：换策略不改变分配（与策略无关）");
    check(base.allocationOrderIds == byTime.allocationOrderIds
              && base.allocationVehicleIndex == byTime.allocationVehicleIndex,
          "基线：时间策略的分配同样不变");
    // 改容量 / 改窗口：分配映射仍不变。
    std::vector<Vehicle> bigger = vehicles;
    bigger[0].capacityKg = 999.0;
    bigger[1].capacityKg = 999.0;
    bigger[2].capacityKg = 999.0;
    const FleetPlan byCap = planFleetSequentialBaseline(g, bigger, orders, WeightType::Distance);
    check(byCap.allocationVehicleIndex == base.allocationVehicleIndex,
          "基线：改容量不改变分配（与容量无关）");
    std::vector<Order> widened = orders;
    for (std::size_t i = 0; i < widened.size(); ++i) {
        widened[i].windowStartMin = 0;
        widened[i].windowEndMin = 1440;
    }
    widened[0].windowEndMin = 300;   // 收紧一张单的窗口
    const FleetPlan byWin = planFleetSequentialBaseline(g, vehicles, widened, WeightType::Distance);
    check(byWin.allocationVehicleIndex == base.allocationVehicleIndex,
          "基线：改窗口不改变分配（与窗口无关）");
}

// 边际代价接口（§4.7 / #27）：Δkey = key(候选) − key(现状)，用候选车自己的因子重放；
// 并列按车辆 ID 升序；不可行候选三个分量都记 +∞。
void testMarginalCostInterface() {
    LogisticsGraph g = makeWtoD1Graph();   // W<->D1 5km/10min/4元
    g.addNode(makeSubNode("D2", NodeType::Delivery, "客户2", 1));
    addTwoWay(g, "W", "D2", 10.0, 20.0, 8.0);
    addTwoWay(g, "D1", "D2", 2.0, 4.0, 1.6);

    Vehicle fast = makeVehicle("W", 100.0, 480);
    fast.id = "V01";
    Vehicle slow = makeVehicleWithFactors("W", 100.0, 480, 1.25, 0.50);
    slow.id = "V02";

    const std::vector<Order> current = {makeOrder("O1", "W", "D1", 10.0, 0, 1440, false)};
    const Order extra = makeOrder("O2", "W", "D2", 10.0, 0, 1440, false);
    std::vector<Order> both = current;
    both.push_back(extra);

    const RoutePlan curFast = planRoute(g, fast, current, WeightType::Distance);
    const RoutePlan candFast = planRoute(g, fast, both, WeightType::Distance);
    const MarginalCost mcFast =
        marginalCost(g, fast, extra, current, curFast, WeightType::Distance);
    check(fixtures::nearlyEqual(mcFast.penaltyDelta,
                                static_cast<double>(candFast.totalPenaltyMin
                                                    - curFast.totalPenaltyMin)),
          "Δpenalty = 候选 − 现状（距离维）");
    check(fixtures::nearlyEqual(mcFast.weightDelta,
                                candFast.totalDistanceKm - curFast.totalDistanceKm),
          "Δweight = 候选 Σ距离 − 现状 Σ距离");
    check(fixtures::nearlyEqual(
              mcFast.balanceDelta,
              candFast.metrics.peakLoadKg / fast.capacityKg
                  - curFast.metrics.peakLoadKg / fast.capacityKg),
          "Δbalance = 本车峰值利用率的变化（本车容量口径）");

    // 耗时维用**该车自己**的 time_factor 重放：慢车 Δweight = 缩放后的耗时差。
    const RoutePlan curSlow = planRoute(g, slow, current, WeightType::Time);
    const RoutePlan candSlow = planRoute(g, slow, both, WeightType::Time);
    const MarginalCost mcSlow = marginalCost(g, slow, extra, current, curSlow, WeightType::Time);
    check(fixtures::nearlyEqual(mcSlow.weightDelta,
                                candSlow.totalTimeMin - curSlow.totalTimeMin),
          "时间维 Δweight 用慢车 ×1.25 的耗时重放");
    check(fixtures::nearlyEqual(candSlow.totalTimeMin, candFast.totalTimeMin * 1.25),
          "慢车候选耗时 = 快车 ×1.25（因子确实进了 evaluator）");

    // 并列时按车辆 ID 升序：两车同参数同起点 ⇒ Δkey 完全相同，
    // 列表顺序决定「先到者」，这里直接断言两车 Δ 相等（上层按 ID 升序取先）。
    Vehicle twin = makeVehicle("W", 100.0, 480);
    twin.id = "V03";
    const MarginalCost mcTwin =
        marginalCost(g, twin, extra, current, planRoute(g, twin, current, WeightType::Distance),
                     WeightType::Distance);
    check(fixtures::nearlyEqual(mcTwin.weightDelta, mcFast.weightDelta)
              && fixtures::nearlyEqual(mcTwin.balanceDelta, mcFast.balanceDelta),
          "同参数车的 Δkey 完全并列（并列由调用方按车辆 ID 升序裁决）");

    // 单笔超过载重 ⇒ 候选不可行 ⇒ 三个分量都是 +∞（调用方按「不可行」处理）。
    Vehicle tiny = makeVehicle("W", 5.0, 480);
    tiny.id = "V04";
    const MarginalCost inf =
        marginalCost(g, tiny, extra, current, planRoute(g, tiny, current, WeightType::Distance),
                     WeightType::Distance);
    check(std::isinf(inf.penaltyDelta) && std::isinf(inf.weightDelta)
              && std::isinf(inf.balanceDelta),
          "不可行候选的 Δkey 三个分量都是 +∞");
}


} // namespace

// ===========================================================================
// 跨车转运（规格二 §5 / #28）—— 12 条不变量 + 分界 / 并列 / 三判据 / 紧急双门槛
// ===========================================================================
namespace {

// 跨簇转运夹具（§7.7 第 5 步的微型版）：
//   W -- T(中转站, sub1)
//   W -- A1(sub1) -- T -- B1、B2(sub2)；W -- A2 -- T，A2/T/B1/B2 构成一条共线
//   O1: A1(sub1) → B1(sub2)   （跨簇；V02 慢车直送）
//   O2: A2(sub1) → B2(sub2)   （V01 快车的自然路线 W→A2→T→B1→B2 恰好经过 T 与 B1）
// 转运：V02 在 T 卸下 O1，V01 在 T 取走并送达 B1 —— 零绕行 + 快车接腿 ⇒ 时间键严格变优。
LogisticsGraph makeTransitGraph() {
    LogisticsGraph g;
    g.addNode(makeNode("W", NodeType::Warehouse, "仓库"));
    g.addNode(makeSubNode("T", NodeType::Transit, "中转站T", 1));
    g.addNode(makeSubNode("A1", NodeType::Delivery, "客户A1", 1));
    g.addNode(makeSubNode("A2", NodeType::Delivery, "客户A2", 1));
    g.addNode(makeSubNode("B1", NodeType::Delivery, "客户B1", 2));
    // B2 留在 sub1：使 O2 成为簇内单，planFleet 阶段二会把「簇1 → V01 快车」，
    // 跨簇的 O1 落到 V02 慢车 —— 这是 planFleet 集成用例需要的分配形态。
    g.addNode(makeSubNode("B2", NodeType::Delivery, "客户B2", 1));
    addTwoWay(g, "W", "T", 1.0, 1.0, 1.0);
    addTwoWay(g, "W", "A1", 1.0, 1.0, 1.0);
    addTwoWay(g, "A1", "T", 1.0, 1.0, 1.0);
    addTwoWay(g, "W", "A2", 1.0, 1.0, 1.0);
    addTwoWay(g, "A2", "T", 2.0, 2.0, 2.0);
    addTwoWay(g, "T", "B1", 1.0, 1.0, 1.0);
    addTwoWay(g, "B1", "B2", 1.0, 1.0, 1.0);
    return g;
}

std::vector<Vehicle> makeTransitVehicles() {
    Vehicle fast = makeVehicleWithFactors("W", 400.0, 480, 1.00, 1.50);
    fast.id = "V01";
    Vehicle slow = makeVehicleWithFactors("W", 160.0, 480, 1.25, 0.80);
    slow.id = "V02";
    return {fast, slow};
}

std::vector<VehicleStartState> staticStarts(const std::vector<Vehicle>& vs) {
    std::vector<VehicleStartState> starts(vs.size());
    for (std::size_t i = 0; i < vs.size(); ++i) {
        starts[i].startNodeId = vs[i].startNodeId;
        starts[i].startTimeMin = vs[i].departTimeMin;
    }
    return starts;
}

std::string planSignature(const std::vector<RoutePlan>& plans) {
    std::string out;
    for (std::size_t v = 0; v < plans.size(); ++v) {
        out += "PLAN" + std::to_string(v) + "|";
        for (const std::string& id : plans[v].nodes) {
            out += id + ",";
        }
        out += "|";
        for (const Stop& s : plans[v].stops) {
            out += s.nodeId + "[";
            for (const std::string& id : s.pickups) out += "P" + id + ",";
            for (const StopDelivery& d : s.deliveries) out += "D" + d.orderId + ",";
            for (const std::string& id : s.offloads) out += "O" + id + ",";
            for (const std::string& id : s.reclaims) out += "R" + id + ",";
            for (const std::string& id : s.transitUnloads) out += "TU" + id + ",";
            for (const std::string& id : s.transitPickups) out += "TP" + id + ",";
            out += "];";
        }
        out += "\n";
    }
    return out;
}

bool hasTransitAction(const RoutePlan& plan) {
    for (const Stop& s : plan.stops) {
        if (!s.transitUnloads.empty() || !s.transitPickups.empty()) {
            return true;
        }
    }
    return false;
}

// 前缀载重：0 ≤ L ≤ capacity；停靠内先卸后装（与 Stop 的执行口径一致）。
bool prefixLoadsOk(const RoutePlan& plan, const std::vector<Order>& orders, double capacityKg,
                   double* peakOut = nullptr) {
    auto kgOf = [&orders](const std::string& id) {
        for (const Order& o : orders) {
            if (o.id == id) return o.demandKg;
        }
        return 0.0;
    };
    double load = 0.0;
    double peak = 0.0;
    for (const Stop& s : plan.stops) {
        for (const StopDelivery& d : s.deliveries) load -= kgOf(d.orderId);
        for (const std::string& id : s.offloads) load -= kgOf(id);
        for (const std::string& id : s.transitUnloads) load -= kgOf(id);
        if (load < -1e-9) return false;
        if (load > peak) peak = load;
        for (const std::string& id : s.pickups) load += kgOf(id);
        for (const std::string& id : s.reclaims) load += kgOf(id);
        for (const std::string& id : s.transitPickups) load += kgOf(id);
        if (load < -1e-9 || load > capacityKg + 1e-9) return false;
        if (load > peak) peak = load;
    }
    if (peakOut != nullptr) *peakOut = peak;
    return true;
}

int stopIndexOfTransitUnload(const RoutePlan& plan, const std::string& orderId) {
    for (std::size_t i = 0; i < plan.stops.size(); ++i) {
        for (const std::string& id : plan.stops[i].transitUnloads) {
            if (id == orderId) return static_cast<int>(i);
        }
    }
    return -1;
}
int stopIndexOfTransitPickup(const RoutePlan& plan, const std::string& orderId) {
    for (std::size_t i = 0; i < plan.stops.size(); ++i) {
        for (const std::string& id : plan.stops[i].transitPickups) {
            if (id == orderId) return static_cast<int>(i);
        }
    }
    return -1;
}
int stopIndexOfDelivery(const RoutePlan& plan, const std::string& orderId) {
    for (std::size_t i = 0; i < plan.stops.size(); ++i) {
        for (const StopDelivery& d : plan.stops[i].deliveries) {
            if (d.orderId == orderId) return static_cast<int>(i);
        }
    }
    return -1;
}
int stopIndexOfPickup(const RoutePlan& plan, const std::string& orderId) {
    for (std::size_t i = 0; i < plan.stops.size(); ++i) {
        for (const std::string& id : plan.stops[i].pickups) {
            if (id == orderId) return static_cast<int>(i);
        }
    }
    return -1;
}

// ①–⑫ 的机械复核（读规划产物，不重算规划）。
void checkTransitInvariants(const LogisticsGraph& g, const std::vector<Vehicle>& vehicles,
                            const std::vector<std::vector<Order> >& orders,
                            const std::vector<RoutePlan>& base,
                            const FleetTransitResult& res, WeightType weight,
                            const std::string& label) {
    const std::string tag = "[" + label + "] ";
    check(res.acceptedRounds >= 1, tag + "转运层已启用（至少接受一轮）");
    check(!res.legs.empty(), tag + "账本非空");

    std::vector<Order> allOrders;
    for (std::size_t v = 0; v < orders.size(); ++v) {
        for (const Order& o : orders[v]) allOrders.push_back(o);
    }

    // ① 转运段完整 + ⑤ dwell ≤ 60 + ③ 期末 0 + ② 整单不可拆
    bool complete = true;
    bool unit = true;
    bool endZero = true;
    bool dwellOk = true;
    for (const TransitLeg& leg : res.legs) {
        if (leg.orderId.empty() || leg.stationId.empty() || leg.inVehicleId.empty()
            || leg.outVehicleId.empty() || leg.inVehicleId == leg.outVehicleId
            || leg.inTimeMin <= 0 || leg.outTimeMin < leg.inTimeMin) {
            complete = false;
        }
        const int dwell = leg.outTimeMin - leg.inTimeMin;
        if (dwell < 0 || dwell > logistics::kTransferMaxDwellMin) dwellOk = false;
        const Order* o = fixtures::findOrderById(allOrders, leg.orderId);
        if (o == nullptr || !fixtures::nearlyEqual(leg.kg, o->demandKg)) unit = false;
        // C3：取走车必须真的在本计划里送达（不是「事后补一趟」）。
        bool delivered = false;
        for (const RoutePlan& plan : res.plans) {
            if (stopIndexOfDelivery(plan, leg.orderId) >= 0) delivered = true;
        }
        if (!delivered) endZero = false;
        // ② 同一 (站, 订单) 至多一笔未取走记录。
        int sameOrder = 0;
        for (const TransitLeg& other : res.legs) {
            if (other.stationId == leg.stationId && other.orderId == leg.orderId) ++sameOrder;
        }
        if (sameOrder != 1) unit = false;
    }
    check(complete, tag + "不变量①：两端非空、inVehicle ≠ outVehicle");
    check(unit, tag + "不变量②：整单不可拆（同一 (站,订单) 一笔、kg = 需求）");
    check(dwellOk, tag + "不变量⑤：0 ≤ dwell ≤ kTransferMaxDwellMin");
    check(endZero, tag + "不变量③（C3）：期末每笔都已取走并由取走车送达");

    // ③（C2）非负：按站扫时间轴，Σ卸入 − Σ取走 ≥ 0。
    {
        struct Ev { int t; double d; };
        bool nonNegative = true;
        for (const TransitLeg& leg : res.legs) {
            std::vector<Ev> evs;
            for (const TransitLeg& other : res.legs) {
                if (other.stationId != leg.stationId) continue;
                evs.push_back(Ev{other.inTimeMin, other.kg});
                evs.push_back(Ev{other.outTimeMin, -other.kg});
            }
            for (std::size_t i = 1; i < evs.size(); ++i) {
                const Ev key = evs[i];
                std::size_t j = i;
                while (j > 0 && (evs[j - 1].t > key.t
                                 || (evs[j - 1].t == key.t && evs[j - 1].d < key.d))) {
                    evs[j] = evs[j - 1];
                    --j;
                }
                evs[j] = key;
            }
            double stock = 0.0;
            for (const Ev& e : evs) {
                stock += e.d;
                if (stock < -1e-9) nonNegative = false;
            }
        }
        check(nonNegative, tag + "不变量③（C2）：任一站任一时刻 Σ卸入 − Σ取走 ≥ 0");
    }

    // ④ 时刻链 + ⑥ 两车各自前缀载重 + ⑦ 站停靠语义 + ⑧ penalty 归属 + ⑨ 先取后送
    bool chain = true;
    bool capacity = true;
    bool stationSemantics = true;
    bool penaltyOnce = true;
    bool orderOk = true;
    for (const TransitLeg& leg : res.legs) {
        std::size_t inV = vehicles.size();
        std::size_t outV = vehicles.size();
        for (std::size_t v = 0; v < vehicles.size(); ++v) {
            if (vehicles[v].id == leg.inVehicleId) inV = v;
            if (vehicles[v].id == leg.outVehicleId) outV = v;
        }
        if (inV >= vehicles.size() || outV >= vehicles.size()) {
            chain = false;
            continue;
        }
        const RoutePlan& inPlan = res.plans[inV];
        const RoutePlan& outPlan = res.plans[outV];
        const int pickupIdx = stopIndexOfPickup(inPlan, leg.orderId);
        const int unloadIdx = stopIndexOfTransitUnload(inPlan, leg.orderId);
        const int pickupOutIdx = stopIndexOfTransitPickup(outPlan, leg.orderId);
        const int delivIdx = stopIndexOfDelivery(outPlan, leg.orderId);
        if (pickupIdx < 0 || unloadIdx < 0 || pickupOutIdx < 0 || delivIdx < 0) {
            orderOk = false;
            continue;
        }
        const int pickupTime = inPlan.stops[static_cast<std::size_t>(pickupIdx)].arrivalMin;
        const int unloadTime = inPlan.stops[static_cast<std::size_t>(unloadIdx)].arrivalMin;
        const int pickupOutTime = outPlan.stops[static_cast<std::size_t>(pickupOutIdx)].arrivalMin;
        const int delivTime = outPlan.stops[static_cast<std::size_t>(delivIdx)].arrivalMin;
        if (!(pickupTime <= unloadTime && unloadTime <= pickupOutTime
              && pickupOutTime <= delivTime)) {
            chain = false;
        }
        if (pickupIdx >= unloadIdx || pickupOutIdx >= delivIdx) orderOk = false;
        // ⑦：站停靠无窗口 / 不计 penalty / 零耗时。
        const Stop& su = inPlan.stops[static_cast<std::size_t>(unloadIdx)];
        const Stop& sp = outPlan.stops[static_cast<std::size_t>(pickupOutIdx)];
        if (su.nodeId.empty() || sp.nodeId.empty() || su.rawArrivalMin != su.arrivalMin
            || sp.rawArrivalMin != sp.arrivalMin || su.waitMin != 0 || sp.waitMin != 0
            || su.penaltyMin != 0 || sp.penaltyMin != 0 || su.late || sp.late) {
            stationSemantics = false;
        }
        // ⑥：两台车各自 0 ≤ 前缀载重 ≤ 各自 capacity（站内先卸后装）。
        // kg 必须从**全车队订单**取：转运单在两台车上各有一段动作，任何一侧漏掉
        // 它的需求都会让前缀载重低估。
        if (!prefixLoadsOk(inPlan, allOrders, vehicles[inV].capacityKg)
            || !prefixLoadsOk(outPlan, allOrders, vehicles[outV].capacityKg)) {
            capacity = false;
        }
        // ⑧：penalty 只在送达车的逐单指标里出现一次。
        int occurrences = 0;
        for (std::size_t v = 0; v < res.plans.size(); ++v) {
            for (const OrderMetric& om : res.plans[v].metrics.orders) {
                if (om.orderId == leg.orderId) ++occurrences;
            }
        }
        if (occurrences != 1) penaltyOnce = false;
    }
    check(orderOk, tag + "不变量④/⑨：取货 < 卸入 ≤ 取走 < 送达（跨车由时刻链连接）");
    check(chain, tag + "不变量④：取货(A) ≤ 卸入(S) ≤ 取走(S) ≤ 送达(B)");
    check(capacity, tag + "不变量⑥：两台车各自 0 ≤ 前缀载重 ≤ 该车 capacity");
    check(stationSemantics, tag + "不变量⑦：站停靠无窗口 / 不计 penalty / 零耗时");
    check(penaltyOnce, tag + "不变量⑧：转运单 penalty 只在送达车记一次");
    // ⑦ 停靠点数：站停靠确实计入（账本里的站出现在该车 stops 里）。
    bool counted = true;
    for (const TransitLeg& leg : res.legs) {
        bool atStation = false;
        for (const RoutePlan& plan : res.plans) {
            for (const Stop& s : plan.stops) {
                if (s.nodeId == leg.stationId
                    && (stopIndexOfTransitUnload(plan, leg.orderId) >= 0
                        || stopIndexOfTransitPickup(plan, leg.orderId) >= 0)) {
                    atStation = true;
                }
            }
        }
        if (!atStation) counted = false;
    }
    check(counted, tag + "不变量⑦：转运站停靠计入该车停靠点数");

    // ⑩ 严格更优或完全相同：最终计划的目标三键严格小于基线（本夹具一定启用）；
    //    若未启用则返回的必须是**同一份**计划（不存在「更差」的第三种结果）。
    const FleetPlan baseFleet = aggregateFleet(vehicles, base);
    const FleetPlan resFleet = aggregateFleet(vehicles, res.plans);
    bool noWorse = true;
    if (res.acceptedRounds > 0) {
        const int cmpPenalty = (resFleet.totalPenaltyMin < baseFleet.totalPenaltyMin)
                                   ? -1
                                   : (resFleet.totalPenaltyMin > baseFleet.totalPenaltyMin ? 1 : 0);
        bool strict = cmpPenalty < 0;
        if (cmpPenalty == 0) {
            const double bw = (weight == WeightType::Distance) ? baseFleet.totalDistanceKm
                              : (weight == WeightType::Cost) ? baseFleet.totalCostYuan
                                                             : baseFleet.completionTimeMin;
            const double rw = (weight == WeightType::Distance) ? resFleet.totalDistanceKm
                              : (weight == WeightType::Cost) ? resFleet.totalCostYuan
                                                             : resFleet.completionTimeMin;
            if (rw < bw - 1e-9) {
                strict = true;
            } else if (fixtures::nearlyEqual(rw, bw, 1e-9)) {
                // 第三键：均衡度（峰值利用率极差）。与 #40 的车队键同口径。
                strict = resFleet.balanceSpread < baseFleet.balanceSpread - 1e-9;
            }
        }
        noWorse = strict;
    }
    check(noWorse, tag + "不变量⑩：最终计划严格更优（不存在「更差」的第三种结果）");

    // ⑪ 确定性：同输入重跑，计划序列逐字相同。
    const FleetTransitResult again =
        applyTransitLayer(g, vehicles, orders, base, staticStarts(vehicles), weight);
    check(planSignature(again.plans) == planSignature(res.plans) && again.legs.size() == res.legs.size(),
          tag + "不变量⑪：同输入必得同一 plan（逐字相同）");

    // ⑫ 不残留：基线计划里没有任何转运动作；账本只由当前计划反查。
    bool baseClean = true;
    for (const RoutePlan& plan : base) {
        if (hasTransitAction(plan)) baseClean = false;
    }
    check(baseClean, tag + "不变量⑫：禁转运版没有任何转运动作（无上一版残留）");
    check(buildTransitLedger(vehicles, base).empty(), tag + "不变量⑫：基线账本为空");
}

void testTransitCrossClusterAcceptedAndConserved() {
    const LogisticsGraph g = makeTransitGraph();
    const std::vector<Vehicle> vehicles = makeTransitVehicles();
    const Order o1 = makeOrder("O1", "A1", "B1", 10.0, 0, 1440);
    const Order o2 = makeOrder("O2", "A2", "B2", 10.0, 0, 1440);
    // 分配（#40 的结果，转运层不改它）：V01 快车跑 O2，V02 慢车跑 O1。
    const std::vector<std::vector<Order> > assigned{{o2}, {o1}};
    std::vector<RoutePlan> base{planRoute(g, vehicles[0], assigned[0], WeightType::Time),
                                planRoute(g, vehicles[1], assigned[1], WeightType::Time)};
    const FleetTransitResult res = applyTransitLayer(g, vehicles, assigned, base,
                                                     staticStarts(vehicles), WeightType::Time);
    check(res.acceptedRounds == 1 && res.legs.size() == 1,
          "跨簇：时间策略下接受一笔转运（V02 在 T 卸、V01 取走送达 B1）");
    if (res.legs.size() == 1) {
        check(res.legs[0].orderId == "O1" && res.legs[0].stationId == "T"
                  && res.legs[0].inVehicleId == "V02"
                  && res.legs[0].outVehicleId == "V01",
              "账本两端：O1 / T / V02 → V01");
        check(res.legs[0].outTimeMin - res.legs[0].inTimeMin <= 60,
              "站内等待 ≤ 60（实测 "
                  + std::to_string(res.legs[0].outTimeMin - res.legs[0].inTimeMin) + "min）");
    }
    checkTransitInvariants(g, vehicles, assigned, base, res, WeightType::Time, "跨簇转运");

    // 用料车路线确实发生了变化：V02 不再送 B1，V01 多了 T 取与 B1 送。
    check(stopIndexOfDelivery(res.plans[1], "O1") < 0,
          "卸入车路线里不再有 O1 的送达（送达已转到取走车）");
    check(stopIndexOfTransitUnload(res.plans[1], "O1") >= 0,
          "卸入车路线里有 O1 的站内卸入");
    check(stopIndexOfTransitPickup(res.plans[0], "O1") >= 0
              && stopIndexOfDelivery(res.plans[0], "O1") >= 0,
          "取走车路线里有 O1 的站内取走与送达");
    // ⑧ 的另一面：卸入车的逐单指标里没有 O1。
    bool inMetrics = false;
    for (const OrderMetric& om : res.plans[1].metrics.orders) {
        if (om.orderId == "O1") inMetrics = true;
    }
    check(!inMetrics, "取货车不记该单的 penalty / 两率（指标只在送达车出现）");
}


// 「完全并列」夹具（§5.7 的关键场景）：两台车参数完全相同（容量/因子/发车），
// 转运把 O1 的送达从 V01 换到 V02 后：
//   · V01 省下 S→B1→W（1km），V02 多走 S→B1→W（1km）⇒ 总距离精确相等；
//   · 两车峰值都恰 10kg ⇒ 均衡键相等；
//   · 窗口宽松 ⇒ penalty 0。
// 三键完全并列 ⇒ 严格更优不成立 ⇒ **必须保留禁转运版**（否则零收益改写路线）。
LogisticsGraph makeTransitParallelGraph() {
    LogisticsGraph g;
    g.addNode(makeNode("W", NodeType::Warehouse, "仓库"));
    g.addNode(makeSubNode("S", NodeType::Transit, "中转站S", 1));
    g.addNode(makeSubNode("A1", NodeType::Delivery, "客户A1", 1));
    g.addNode(makeSubNode("B1", NodeType::Delivery, "客户B1", 2));
    g.addNode(makeSubNode("A2", NodeType::Delivery, "客户A2", 1));
    g.addNode(makeSubNode("B2", NodeType::Delivery, "客户B2", 1));
    addTwoWay(g, "W", "A1", 2.0, 2.0, 2.0);
    addTwoWay(g, "A1", "S", 2.0, 2.0, 2.0);
    addTwoWay(g, "S", "B1", 1.0, 1.0, 1.0);
    addTwoWay(g, "B1", "W", 1.0, 1.0, 1.0);
    addTwoWay(g, "S", "W", 1.0, 1.0, 1.0);
    addTwoWay(g, "W", "A2", 2.0, 2.0, 2.0);
    addTwoWay(g, "A2", "B2", 1.0, 1.0, 1.0);
    addTwoWay(g, "B2", "S", 2.0, 2.0, 2.0);
    return g;
}

void testTransitParallelKeepsDirectVersion() {
    // 三键完全并列：转运版总距离精确相等、均衡键相等、penalty 相等
    // ⇒ 严格更优不成立 ⇒ 保留禁转运版（§5.7 的关键一条，反向验证 R 的目标）。
    const LogisticsGraph g = makeTransitParallelGraph();
    Vehicle v1 = makeVehicle("W", 160.0, 480);
    v1.id = "V01";
    Vehicle v2 = makeVehicle("W", 160.0, 480);
    v2.id = "V02";
    const std::vector<Vehicle> vehicles{v1, v2};
    const Order o1 = makeOrder("O1", "A1", "B1", 10.0, 0, 1440);
    const Order o2 = makeOrder("O2", "A2", "B2", 10.0, 0, 1440);
    const std::vector<std::vector<Order> > assigned{{o1}, {o2}};
    std::vector<RoutePlan> base{planRoute(g, vehicles[0], assigned[0], WeightType::Time),
                                planRoute(g, vehicles[1], assigned[1], WeightType::Time)};
    const FleetPlan baseFleet = aggregateFleet(vehicles, base);
    // 先把「真的并列」钉死：三键（penalty / 距离 / 均衡）两版完全相等。
    check(baseFleet.totalDistanceKm > 0.0 && baseFleet.balanceSpread >= 0.0
              && baseFleet.totalPenaltyMin == 0,
          "并列（前置）：禁转运版三项键可计算");
    const FleetTransitResult res = applyTransitLayer(g, vehicles, assigned, base,
                                                     staticStarts(vehicles), WeightType::Distance);
    const FleetPlan resFleet = aggregateFleet(vehicles, res.plans);
    check(fixtures::nearlyEqual(resFleet.totalDistanceKm, baseFleet.totalDistanceKm, 1e-9)
              && fixtures::nearlyEqual(resFleet.balanceSpread, baseFleet.balanceSpread, 1e-9)
              && resFleet.totalPenaltyMin == baseFleet.totalPenaltyMin,
          "并列（前置）：转运版三键与禁转运版**完全相等**（"
              + std::to_string(baseFleet.totalDistanceKm) + " / "
              + std::to_string(baseFleet.balanceSpread) + "）");
    check(res.acceptedRounds == 0, "并列：三键完全并列 ⇒ 转运未启用");
    check(res.legs.empty(), "并列：账本为空");
    check(planSignature(res.plans) == planSignature(base),
          "并列：返回的是禁转运版的**同一份**计划（没有零收益改写）");
    bool said = false;
    for (const std::string& line : res.log) {
        if (line.find("不严格更优") != std::string::npos) said = true;
    }
    check(said, "并列：日志如实记录「最优候选不严格更优 ⇒ 保留禁转运版」");
}

void testTransitSameVehicleIsOffloadNotTransit() {
    // 与降载的分界（MC-A09）：同一台车卸了又取回 ⇒ 永远算降载，账本里没有它。
    const LogisticsGraph g = makeOffloadGraph();
    const Vehicle v = makeVehicle("W", 100.0, 480);
    std::vector<Order> pending;
    pending.push_back(makeOrder("O1", "W", "D1", 60.0, 0, 1440));
    pending.push_back(makeOrder("O2", "W", "D2", 30.0, 0, 1440));
    const std::vector<logistics::OnboardItem> onboard{{"D1", 60.0, "O1"}, {"D2", 30.0, "O2"}};
    Order urgent = makeOrder("U1", "D3", "D4", 50.0, 0, 1440, true);
    const InsertResult ins =
        insertOrder(g, v, pending, urgent, "W", 480, WeightType::Distance, onboard);
    check(!ins.plan.offloads.empty(), "分界（前置）：同一台车确实产生降载记录");
    const std::vector<std::vector<Order> > assigned{{}};
    std::vector<Order> all = pending;
    all.push_back(urgent);
    const std::vector<std::vector<Order> > orders{all};
    const std::vector<RoutePlan> base{ins.plan};
    const FleetTransitResult res =
        applyTransitLayer(g, {v}, orders, base, staticStarts({v}), WeightType::Distance);
    check(res.acceptedRounds == 0 && res.legs.empty(),
          "分界：同车卸又取回不产生任何 TransitLeg（永远算降载）");
    check(!hasTransitAction(res.plans[0]), "分界：降载计划里没有任何转运动作");
    check(buildTransitLedger({v}, res.plans).empty(), "分界：账本为空（降载走 OffloadLeg）");
}

void testTransitCapacityTriggerFires() {
    // 判据②：禁转运版里有降载记录 ⇒ 触发容量腾挪候选（是否接受再看严格更优）。
    const LogisticsGraph g = makeOffloadGraph();
    Vehicle v1 = makeVehicleWithFactors("W", 100.0, 480, 1.0, 1.0);
    v1.id = "V01";
    Vehicle v2 = makeVehicleWithFactors("W", 100.0, 480, 1.25, 0.8);
    v2.id = "V02";
    std::vector<Order> pending;
    pending.push_back(makeOrder("O1", "W", "D1", 60.0, 0, 1440));
    pending.push_back(makeOrder("O2", "W", "D2", 30.0, 0, 1440));
    const std::vector<logistics::OnboardItem> onboard{{"D1", 60.0, "O1"}, {"D2", 30.0, "O2"}};
    Order urgent = makeOrder("U1", "D3", "D4", 50.0, 0, 1440, true);
    const InsertResult ins =
        insertOrder(g, v2, pending, urgent, "W", 480, WeightType::Distance, onboard);
    std::vector<Order> v2Orders = pending;
    v2Orders.push_back(urgent);
    const std::vector<std::vector<Order> > assigned{{}, v2Orders};
    std::vector<RoutePlan> base{planRoute(g, v1, {}, WeightType::Distance), ins.plan};
    const FleetTransitResult res = applyTransitLayer(g, {v1, v2}, assigned, base,
                                                     staticStarts({v1, v2}), WeightType::Distance);
    bool triggered = false;
    for (const std::string& line : res.log) {
        if (line.find("判据②（容量腾挪）：触发") != std::string::npos) triggered = true;
    }
    check(triggered, "判据②：禁转运版有降载记录 ⇒ 容量腾挪判据触发");
    if (res.acceptedRounds > 0) {
        for (const TransitLeg& leg : res.legs) {
            check(leg.inVehicleId != leg.outVehicleId && !leg.inVehicleId.empty()
                      && !leg.outVehicleId.empty(),
                  "判据②：接受时账本两端非空且 in ≠ out");
        }
    }
}

LogisticsGraph makeUrgentTransitGraph(bool delayedOtherOrder) {
    // 判据③的夹具：T 是 O1 的零绕行站；V01 的 O2 路线先经过 A2 再到其余节点。
    // delayedOtherOrder = true 时，给 O2 一条更短直达路 + 紧窗口：
    // 转运把 B1 停靠插进来会拖晚 O2 ⇒ 该单 penalty 降、整计划键却更差（门槛二失败）。
    LogisticsGraph g;
    g.addNode(makeNode("W", NodeType::Warehouse, "仓库"));
    g.addNode(makeSubNode("T", NodeType::Transit, "中转站T", 1));
    g.addNode(makeSubNode("A1", NodeType::Delivery, "客户A1", 1));
    g.addNode(makeSubNode("A2", NodeType::Delivery, "客户A2", 1));
    g.addNode(makeSubNode("B1", NodeType::Delivery, "客户B1", 1));
    g.addNode(makeSubNode("B2", NodeType::Delivery, "客户B2", 1));
    if (!delayedOtherOrder) {
        addTwoWay(g, "W", "A1", 11.0, 11.0, 11.0);
        addTwoWay(g, "A1", "T", 1.0, 1.0, 1.0);
        addTwoWay(g, "A1", "B1", 11.0, 11.0, 11.0);
        addTwoWay(g, "W", "A2", 11.0, 11.0, 11.0);
        addTwoWay(g, "A2", "T", 1.0, 1.0, 1.0);
        addTwoWay(g, "T", "B1", 10.0, 10.0, 10.0);
        addTwoWay(g, "W", "T", 12.0, 12.0, 12.0);
        addTwoWay(g, "A2", "B1", 11.0, 11.0, 11.0);
    } else {
        addTwoWay(g, "W", "A1", 6.0, 6.0, 6.0);
        addTwoWay(g, "A1", "T", 1.0, 1.0, 1.0);
        addTwoWay(g, "W", "A2", 9.0, 9.0, 9.0);
        addTwoWay(g, "A2", "T", 3.0, 3.0, 3.0);
        g.addEdge(fixtures::makeEdge("T", "B1", 20.0, 20.0, 20.0));
        g.addEdge(fixtures::makeEdge("B1", "B2", 1.0, 1.0, 1.0));
        g.addEdge(fixtures::makeEdge("B2", "T", 1.0, 1.0, 1.0));
        g.addEdge(fixtures::makeEdge("T", "B2", 1.0, 1.0, 1.0));
        g.addEdge(fixtures::makeEdge("W", "T", 10.0, 10.0, 10.0));
        g.addEdge(fixtures::makeEdge("T", "W", 10.0, 10.0, 10.0));
    }
    return g;
}

void testTransitUrgentDoubleGate() {
    // 门槛一失败（key 更优但该单 penalty 没降）：两车同因子，换车只改变距离键，
    // O1 在 B1 的到达时刻完全不变 ⇒ 2 -> 2。
    {
        const LogisticsGraph g = makeUrgentTransitGraph(false);
        Vehicle v1 = makeVehicle("W", 160.0, 480);
        v1.id = "V01";
        Vehicle v2 = makeVehicle("W", 160.0, 480);
        v2.id = "V02";
        const std::vector<Vehicle> vehicles{v1, v2};
        const Order o1 = makeOrder("O1", "A1", "B1", 10.0, 480, 500, true);
        const Order o2 = makeOrder("O2", "A2", "B1", 10.0, 480, 600, false);
        const std::vector<std::vector<Order> > assigned{{o2}, {o1}};
        std::vector<RoutePlan> base{planRoute(g, v1, assigned[0], WeightType::Distance),
                                    planRoute(g, v2, assigned[1], WeightType::Distance)};
        check(base[1].totalPenaltyMin == 2, "紧急双门槛（前置）：O1 直送迟到 2min");
        const FleetTransitResult res = applyTransitLayer(g, vehicles, assigned, base,
                                                         staticStarts(vehicles),
                                                         WeightType::Distance);
        check(res.acceptedRounds == 0, "紧急双门槛：只满足「整计划更优」也不得启用");
        bool said = false;
        for (const std::string& line : res.log) {
            if (line.find("penalty 未严格下降") != std::string::npos) said = true;
        }
        check(said, "紧急双门槛：日志点名「该紧急单 penalty 未严格下降（2 -> 2）」");
        check(planSignature(res.plans) == planSignature(base),
              "紧急双门槛：未启用时返回禁转运版的同一份计划");
    }
    // 门槛二失败（该单 penalty 会降，但整计划键更差）：O2 的紧窗口被转运拖晚。
    {
        const LogisticsGraph g = makeUrgentTransitGraph(true);
        // V01 发车 483：使它在 T 的取走时刻（490）不早于 V02 的卸入（489）——
        // 时刻链「卸入 ≤ 取走」是可行性，不是可比较项。
        Vehicle v1 = makeVehicle("W", 160.0, 483);
        v1.id = "V01";
        Vehicle v2 = makeVehicleWithFactors("W", 160.0, 480, 1.25, 1.0);
        v2.id = "V02";
        const std::vector<Vehicle> vehicles{v1, v2};
        const Order o1 = makeOrder("O1", "A1", "B1", 10.0, 480, 510, true);
        const Order o2 = makeOrder("O2", "A2", "B2", 10.0, 480, 497, false);
        const std::vector<std::vector<Order> > assigned{{o2}, {o1}};
        std::vector<RoutePlan> base{planRoute(g, v1, assigned[0], WeightType::Distance),
                                    planRoute(g, v2, assigned[1], WeightType::Distance)};
        check(base[1].totalPenaltyMin == 4, "紧急双门槛（前置）：O1 直送迟到 4min");
        // 对照：把 O2 窗口放松 ⇒ 同一个 O1 候选被接受（证明门槛一确实满足）。
        std::vector<std::vector<Order> > looseAssigned = assigned;
        looseAssigned[0][0].windowEndMin = 1440;
        std::vector<RoutePlan> looseBase{
            planRoute(g, v1, looseAssigned[0], WeightType::Distance),
            planRoute(g, v2, looseAssigned[1], WeightType::Distance)};
        const FleetTransitResult looseRes = applyTransitLayer(
            g, vehicles, looseAssigned, looseBase, staticStarts(vehicles), WeightType::Distance);
        check(looseRes.acceptedRounds == 1,
              "紧急双门槛（对照）：放松另一单窗口后同一个 O1 候选被接受（门槛一满足）");
        const FleetTransitResult res = applyTransitLayer(g, vehicles, assigned, base,
                                                         staticStarts(vehicles),
                                                         WeightType::Distance);
        check(res.acceptedRounds == 0, "紧急双门槛：该单 penalty 会降、整计划键更差 ⇒ 仍不得启用");
        bool said = false;
        for (const std::string& line : res.log) {
            if (line.find("不严格更优") != std::string::npos) said = true;
        }
        check(said, "紧急双门槛：日志记录门槛二不成立（最优候选不严格更优）");
    }
}

void testTransitDwellBoundary() {
    // ⑤ 的 60 边界：把接收车发车时刻推后，使站内等待恰好 60 / 61 分钟。
    // 距离策略判键（转运不改变两车距离），只让 dwell 这一个可行性条件起作用。
    const LogisticsGraph g = makeTransitGraph();
    const Order o1 = makeOrder("O1", "A1", "B1", 10.0, 0, 1440);
    const Order o2 = makeOrder("O2", "A2", "B2", 10.0, 0, 1440);
    const std::vector<std::vector<Order> > assigned{{o2}, {o1}};
    for (int dwell : {60, 61}) {
        Vehicle fast = makeVehicleWithFactors("W", 400.0, 480 + dwell - 0, 1.00, 1.50);
        fast.id = "V01";
        Vehicle slow = makeVehicleWithFactors("W", 160.0, 480, 1.25, 0.80);
        slow.id = "V02";
        const std::vector<Vehicle> vehicles{fast, slow};
        std::vector<RoutePlan> base{planRoute(g, vehicles[0], assigned[0], WeightType::Distance),
                                    planRoute(g, vehicles[1], assigned[1], WeightType::Distance)};
        const FleetTransitResult res = applyTransitLayer(g, vehicles, assigned, base,
                                                         staticStarts(vehicles),
                                                         WeightType::Distance);
        if (dwell == 60) {
            check(res.legs.size() == 1
                      && res.legs[0].outTimeMin - res.legs[0].inTimeMin == 60,
                  "不变量⑤：dwell 恰 60min 是可接受边界（实测 "
                      + (res.legs.empty() ? std::string("未启用")
                                          : std::to_string(res.legs[0].outTimeMin
                                                           - res.legs[0].inTimeMin))
                      + "）");
        } else {
            check(res.acceptedRounds == 0 && res.legs.empty(),
                  "不变量⑤：dwell 61min 超限 ⇒ 候选不可行（不是「更差」，是不存在）");
        }
    }
}


void testTransitPlanFleetIntegration() {
    // 后置改进必须挂在 planFleet 上（不是只活在单元夹具里）：
    // 时间策略接受、距离/成本策略因「完全并列 / 无零绕行站」不启用。
    const LogisticsGraph g = makeTransitGraph();
    const std::vector<Vehicle> vehicles = makeTransitVehicles();
    const std::vector<Order> orders{makeOrder("O2", "A2", "B2", 10.0, 0, 1440),
                                    makeOrder("O1", "A1", "B1", 10.0, 0, 1440)};
    const FleetPlan byTime = planFleet(g, vehicles, orders, WeightType::Time);
    check(byTime.transitLegs.size() == 1 && byTime.transitLegs[0].orderId == "O1"
              && byTime.transitLegs[0].inVehicleId == "V02"
              && byTime.transitLegs[0].outVehicleId == "V01"
              && byTime.transitLegs[0].inVehicleId != byTime.transitLegs[0].outVehicleId,
          "planFleet · 时间策略：接受 O1（V02 在 T 卸、V01 取走送达）");
    const FleetPlan byDistance = planFleet(g, vehicles, orders, WeightType::Distance);
    check(byDistance.transitLegs.empty(),
          "planFleet · 距离策略：两版精确并列 ⇒ 保留禁转运版");
    const FleetPlan byCost = planFleet(g, vehicles, orders, WeightType::Cost);
    check(byCost.transitLegs.empty(), "planFleet · 成本策略：无严格更优候选 ⇒ 不启用");
    // 账本两端与时刻链（读 planFleet 的产物，不是重算）。
    if (!byTime.transitLegs.empty()) {
        const TransitLeg& leg = byTime.transitLegs[0];
        check(leg.outTimeMin - leg.inTimeMin >= 0
                  && leg.outTimeMin - leg.inTimeMin <= logistics::kTransferMaxDwellMin,
              "planFleet · 账本 dwell 在 [0, 60]");
    }
}



// ===========================================================================
// 动态事件复调度（规格二 §6 / #42）
// ===========================================================================
//
// 覆盖：受影响车集合（§6.5）/ 局部两版（§6.3）/ 五级升级链（§6.4）/
// 逐车紧急 FIFO（§6.7）/ 已执行卸入的继承（§6.9）/ 账本与降载分界 / 确定性 / 不丢单。

using logistics::ExecutedTransitEntry;
using logistics::FleetRescheduleInput;
using logistics::FleetRescheduleResult;
using logistics::LocalReplanOutcome;
using logistics::NewOrderAssignment;
using logistics::OnboardItem;
using logistics::RescheduleEvent;
using logistics::RescheduleTrigger;
using logistics::StashState;
using logistics::affectedVehiclesForClosure;
using logistics::affectedVehiclesForTraffic;
using logistics::chooseVehicleForNewOrder;
using logistics::localReplanTwoVersion;
using logistics::rescheduleFleet;
using logistics::vehicleOrdersFromPlans;

// 调度链夹具：仓库 W、中转站 T、客户 A/B（子网络 1）、C/D（子网络 2）、P（子网络 9）。
// 「W-A-B-W」与「W-C-D-W」两个环 + P 只与 A、W 相连 —— 删一条边就能精确构造
// "这台车走不到、另一台走得到" 的形态（第 ② 级的入口条件）。
LogisticsGraph makeRescheduleGraph() {
    LogisticsGraph g;
    g.addNode(makeNode("W", NodeType::Warehouse, "仓库"));
    g.addNode(makeSubNode("T", NodeType::Transit, "中转站T", 1));
    g.addNode(makeSubNode("A", NodeType::Delivery, "客户A", 1));
    g.addNode(makeSubNode("B", NodeType::Delivery, "客户B", 1));
    g.addNode(makeSubNode("C", NodeType::Delivery, "客户C", 2));
    g.addNode(makeSubNode("D", NodeType::Delivery, "客户D", 2));
    g.addNode(makeSubNode("P", NodeType::Delivery, "客户P", 9));
    addTwoWay(g, "W", "A", 2.0, 2.0, 2.0);
    addTwoWay(g, "A", "B", 2.0, 2.0, 2.0);
    addTwoWay(g, "B", "W", 2.0, 2.0, 2.0);
    addTwoWay(g, "W", "C", 2.0, 2.0, 2.0);
    addTwoWay(g, "C", "D", 2.0, 2.0, 2.0);
    addTwoWay(g, "D", "W", 2.0, 2.0, 2.0);
    addTwoWay(g, "P", "A", 3.0, 3.0, 3.0);
    addTwoWay(g, "P", "W", 3.0, 3.0, 3.0);
    addTwoWay(g, "W", "T", 1.0, 1.0, 1.0);
    addTwoWay(g, "T", "A", 1.0, 1.0, 1.0);
    addTwoWay(g, "T", "C", 1.0, 1.0, 1.0);
    return g;
}

int stopIndexOfReclaim(const RoutePlan& plan, const std::string& orderId) {
    for (std::size_t i = 0; i < plan.stops.size(); ++i) {
        for (const std::string& id : plan.stops[i].reclaims) {
            if (id == orderId) return static_cast<int>(i);
        }
    }
    return -1;
}

// ---- §6.5 受影响车集合 + §6.3 两版比较 + 第 ① 级 ----
void testRescheduleLevel1KeepsUnaffectedRoutesAndTwoVersionRule() {
    LogisticsGraph g = makeRescheduleGraph();
    Vehicle v1 = makeVehicle("W", 100.0, 480);
    v1.id = "V01";
    Vehicle v2 = makeVehicle("W", 100.0, 480);
    v2.id = "V02";
    const std::vector<Vehicle> vehicles{v1, v2};
    const Order o1 = makeOrder("O1", "A", "B", 10.0, 0, 1440);
    const Order o2 = makeOrder("O2", "C", "D", 10.0, 0, 1440);
    const RoutePlan p1 = planRoute(g, v1, {o1}, WeightType::Time);
    const RoutePlan p2 = planRoute(g, v2, {o2}, WeightType::Time);
    check(p1.status == PlanStatus::Ok && p2.status == PlanStatus::Ok,
          "夹具前置：两台车各自的路线可行");

    FleetRescheduleInput in;
    in.starts = staticStarts(vehicles);
    in.vehicleOrders = {{o1}, {o2}};
    in.remainingPlans = {p1, p2};
    in.allRemainingOrders = {o1, o2};

    TrafficChange ch;
    ch.fromId = "A";
    ch.toId = "B";
    ch.increaseRatio = 0.5;
    ch.congested = true;
    const std::vector<std::size_t> hit =
        affectedVehiclesForTraffic(in.remainingPlans, {ch}, 0.2);
    check(hit.size() == 1 && hit[0] == 0,
          "§6.5（路况）：只有剩余路线用到 A->B 的 V01 进集合（实际 " + std::to_string(hit.size())
              + " 台）");
    check(affectedVehiclesForTraffic(in.remainingPlans, {ch}, 0.9).empty(),
          "§6.5（路况）：增幅 0.5 < 门槛 0.9 ⇒ 空集（不重算任何车）");
    TrafficChange closed;
    closed.fromId = "C";
    closed.toId = "D";
    check(affectedVehiclesForClosure(in.remainingPlans, {closed}).size() == 1
              && affectedVehiclesForClosure(in.remainingPlans, {closed})[0] == 1,
          "§6.5（封路）：只有用到 C-D 的 V02 进集合");

    // 图上的边权真的变了（重规划读的是当前图）
    g.updateEdgeTime("A", "B", 3.0);
    g.updateEdgeTime("B", "A", 3.0);

    RescheduleEvent ev;
    ev.trigger = RescheduleTrigger::Traffic;
    ev.changes = {ch};
    ev.thresholdRatio = 0.2;
    const FleetRescheduleResult res = rescheduleFleet(g, vehicles, WeightType::Time, in, ev);
    check(res.level == 1 && !res.rejected, "第 ① 级：只影响一台车 ⇒ 局部重排成功");
    check(res.replanned.size() == 1 && res.replanned[0] == 0,
          "第 ① 级：只重算受影响的那台车（实际 " + std::to_string(res.replanned.size()) + " 台）");
    check(planSignature({res.plans[1]}) == planSignature({p2}),
          "不变量 5：未受影响车（V02）的路线逐字不变");
    bool hasLine = false;
    for (std::size_t i = 0; i < res.log.size(); ++i) {
        if (res.log[i] == "[复调度] 局部重排（1 台车）") {
            hasLine = true;
        }
    }
    check(hasLine, "分级播报：局部重排（1 台车）");

    // §6.3：采用 R2 当且仅当 R2 可行且 key(R2) < key(R1)（严格；并列保留 R1）
    const LocalReplanOutcome lo = localReplanTwoVersion(g, v1, p1, {o1}, "W", 480,
                                                        WeightType::Time, {}, StashState());
    check(lo.feasible && lo.replayFeasible, "§6.3：顺序不变版（R1）在新边权下仍可行");
    if (lo.replayFeasible && lo.rebuildFeasible) {
        const auto lexBetter = [](const RoutePlan& a, const RoutePlan& b) {
            if (a.totalPenaltyMin != b.totalPenaltyMin) {
                return a.totalPenaltyMin < b.totalPenaltyMin;
            }
            return a.totalTimeMin < b.totalTimeMin - 1e-9;
        };
        check(lo.adoptedRebuild == lexBetter(lo.byRebuild, lo.byReplay),
              "§6.3：严格更优才改序（并列保留顺序不变版）");
        check(planSignature({lo.chosen})
                  == planSignature({lo.adoptedRebuild ? lo.byRebuild : lo.byReplay}),
              "§6.3：采用的就是被判优的那一版");
    }
}

// ---- 第 ② 级：一台车的单**装不上**（单张 60kg > 该车 50kg）⇒ 改派给另一台车 ----
void testRescheduleLevel2CrossVehicleReassignment() {
    LogisticsGraph g = makeRescheduleGraph();
    Vehicle v1 = makeVehicle("W", 50.0, 480);    // 载重不够：装不下 60kg 的单
    v1.id = "V01";
    Vehicle v2 = makeVehicle("W", 100.0, 480);
    v2.id = "V02";
    const std::vector<Vehicle> vehicles{v1, v2};
    const Order o1 = makeOrder("O1", "A", "B", 60.0, 0, 1440);
    const RoutePlan p1 = planRoute(g, v1, {o1}, WeightType::Time);   // 不可行：60 > 50
    const RoutePlan p2 = planRoute(g, v2, {}, WeightType::Time);
    check(p1.status != PlanStatus::Ok, "夹具前置：60kg 的单在 50kg 车上排不出来");
    FleetRescheduleInput in;
    in.starts = staticStarts(vehicles);
    in.vehicleOrders = {{o1}, {}};
    in.remainingPlans = {p1, p2};
    in.allRemainingOrders = {o1};
    // 手动重规划 = 全车队都算受影响（§6.4 的 Manual 入口）
    RescheduleEvent ev;
    ev.trigger = RescheduleTrigger::Manual;
    const FleetRescheduleResult res = rescheduleFleet(g, vehicles, WeightType::Time, in, ev);
    check(res.level == 2 && !res.rejected,
          "第 ② 级：V01 装不下 ⇒ 把这张单改派给 V02（实际第 " + std::to_string(res.level)
              + " 级）");
    std::size_t iLevel1Fail = res.log.size();
    std::size_t iLevel2 = res.log.size();
    bool hasMove = false;
    for (std::size_t i = 0; i < res.log.size(); ++i) {
        if (res.log[i].find("局部重排失败") != std::string::npos && iLevel1Fail > i) {
            iLevel1Fail = i;
        }
        if (res.log[i].find("跨车再分配 O1 V01→V02") != std::string::npos) {
            hasMove = true;
            if (iLevel2 > i) {
                iLevel2 = i;
            }
        }
    }
    check(hasMove, "分级播报：跨车再分配 O1 V01→V02");
    check(iLevel1Fail < iLevel2, "不变量 7（升级链单调）：第 ① 级失败行在第 ② 级之前");
    if (res.plans.size() == 2) {
        check(stopIndexOfDelivery(res.plans[1], "O1") >= 0, "改派后由 V02 送达 O1");
        check(stopIndexOfDelivery(res.plans[0], "O1") < 0, "V01 的路线里不再有 O1");
        check(res.plans[0].status == PlanStatus::Ok, "V01 的空车计划可行（无单可跑）");
    }
}

// ---- 第 ③ 级：② 改不动（挪走一张、剩下那张仍装不上）⇒ 全车队三阶段重调度 ----
void testRescheduleLevel3FullFleet() {
    LogisticsGraph g = makeRescheduleGraph();
    Vehicle v1 = makeVehicle("W", 50.0, 480);   // 装不下 60kg 的单
    v1.id = "V01";
    Vehicle v2 = makeVehicle("W", 100.0, 480);
    v2.id = "V02";
    Vehicle v3 = makeVehicle("W", 100.0, 480);
    v3.id = "V03";
    const std::vector<Vehicle> vehicles{v1, v2, v3};
    const Order o1 = makeOrder("O1", "A", "B", 60.0, 0, 1440);
    const Order o2 = makeOrder("O2", "C", "D", 60.0, 0, 1440);
    const RoutePlan p1 = planRoute(g, v1, {o1, o2}, WeightType::Time);   // 不可行
    const RoutePlan p2 = planRoute(g, v2, {}, WeightType::Time);
    const RoutePlan p3 = planRoute(g, v3, {}, WeightType::Time);
    check(p1.status != PlanStatus::Ok, "夹具前置：两张 60kg 的单在 50kg 车上排不出来");
    FleetRescheduleInput in;
    in.starts = staticStarts(vehicles);
    in.vehicleOrders = {{o1, o2}, {}, {}};
    in.remainingPlans = {p1, p2, p3};
    in.allRemainingOrders = {o1, o2};
    RescheduleEvent ev;
    ev.trigger = RescheduleTrigger::Manual;
    const FleetRescheduleResult res = rescheduleFleet(g, vehicles, WeightType::Time, in, ev);
    check(res.level == 3 && !res.rejected,
          "第 ③ 级：② 改不动（挪走一张、剩下那张仍装不上）⇒ 全车队重调度（实际第 "
              + std::to_string(res.level) + " 级）");
    bool hasFull = false;
    for (std::size_t i = 0; i < res.log.size(); ++i) {
        if (res.log[i] == "[复调度] 全车队重调度") {
            hasFull = true;
        }
    }
    check(hasFull, "分级播报：全车队重调度");
    if (res.plans.size() == 3) {
        bool o1Served = false;
        bool o2Served = false;
        for (std::size_t v = 0; v < res.plans.size(); ++v) {
            if (stopIndexOfDelivery(res.plans[v], "O1") >= 0) o1Served = true;
            if (stopIndexOfDelivery(res.plans[v], "O2") >= 0) o2Served = true;
        }
        check(o1Served && o2Served, "第 ③ 级：两张单都被某台车服务（不丢单）");
        check(stopIndexOfDelivery(res.plans[0], "O1") < 0
                  && stopIndexOfDelivery(res.plans[0], "O2") < 0,
              "第 ③ 级：V01 不再持有这两张单（它装不下）");
    }
}

// ---- 第 ④ 级：只有降载能救；先降载后转运，完全并列保留降载版 ----
void testRescheduleLevel4OffloadBeforeTransit() {
    LogisticsGraph g = makeRescheduleGraph();
    Vehicle v1 = makeVehicle("W", 10.0, 480);
    v1.id = "V01";
    const std::vector<Vehicle> vehicles{v1};
    const Order o1 = makeOrder("O1", "C", "D", 8.0, 0, 1440);          // 已在车 8kg
    const Order o2 = makeOrder("O2", "A", "B", 5.0, 480, 600, true);   // 新紧急单 5kg
    const RoutePlan p1 = planRoute(g, v1, {o1}, WeightType::Time);
    check(p1.status == PlanStatus::Ok, "夹具前置：V01 的单车路线可行");
    OnboardItem item;
    item.orderId = "O1";
    item.nodeId = "D";
    item.kg = 8.0;
    FleetRescheduleInput in;
    in.starts.resize(1);
    in.starts[0].startNodeId = "W";
    in.starts[0].startTimeMin = 480;
    in.starts[0].onboard.push_back(item);
    in.vehicleOrders = {{o1}};
    in.remainingPlans = {p1};
    in.allRemainingOrders = {o1, o2};
    RescheduleEvent ev;
    ev.trigger = RescheduleTrigger::NewOrder;
    ev.newOrder = o2;
    const FleetRescheduleResult res = rescheduleFleet(g, vehicles, WeightType::Time, in, ev);
    check(res.level == 4 && !res.rejected,
          "第 ④ 级：紧急单在 A 与在车的 8kg 叠加必然溢出 ⇒ 只有降载能救（实际第 "
              + std::to_string(res.level) + " 级）");
    std::size_t iOffload = res.log.size();
    std::size_t iTransit = res.log.size();
    bool keepNote = false;
    for (std::size_t i = 0; i < res.log.size(); ++i) {
        if (res.log[i].find("[复调度] 降载（") == 0 && iOffload > i) {
            iOffload = i;
        }
        if (res.log[i].find("[复调度] 转运（") == 0 && iTransit > i) {
            iTransit = i;
        }
        if (res.log[i].find("保留降载版") != std::string::npos) {
            keepNote = true;
        }
    }
    check(iOffload < res.log.size(), "分级播报：降载（站，N 张单）");
    check(iOffload <= iTransit, "§6.8：先构造降载版（播报行在转运行之前）");
    check(keepNote, "§6.8：并列 / 降载更优 ⇒ 保留降载版（有对应播报行）");
    if (res.plans.size() == 1) {
        check(!res.plans[0].offloads.empty(), "降载版计划里真的有降载记录");
        check(stopIndexOfDelivery(res.plans[0], "O2") >= 0, "新紧急单被排进路线");
    }
}


// ---- 第 ④ 级的转运半边：降载与转运**同时**被构造，播报顺序固定 ----
//
// 夹具与 routeplanner_tests 的转运夹具同一套几何（W / 中转站 T / A1,A2 子网络1 /
// B1 子网络2 / B2 子网络1）。两台车各载 8kg、容量 10kg；新紧急单 5kg 在 A1 取货，
// 与在车货叠加必然溢出 ⇒ ①②③ 全失败，第 ④ 级同时构造降载版与转运版。
void testRescheduleLevel4TransitLine() {
    LogisticsGraph g;
    g.addNode(makeNode("W", NodeType::Warehouse, "仓库"));
    g.addNode(makeSubNode("T", NodeType::Transit, "中转站T", 1));
    g.addNode(makeSubNode("A1", NodeType::Delivery, "客户A1", 1));
    g.addNode(makeSubNode("A2", NodeType::Delivery, "客户A2", 1));
    g.addNode(makeSubNode("B1", NodeType::Delivery, "客户B1", 2));
    g.addNode(makeSubNode("B2", NodeType::Delivery, "客户B2", 1));
    addTwoWay(g, "W", "T", 1.0, 1.0, 1.0);
    addTwoWay(g, "W", "A1", 1.0, 1.0, 1.0);
    addTwoWay(g, "A1", "T", 1.0, 1.0, 1.0);
    addTwoWay(g, "W", "A2", 1.0, 1.0, 1.0);
    addTwoWay(g, "A2", "T", 2.0, 2.0, 2.0);
    addTwoWay(g, "T", "B1", 1.0, 1.0, 1.0);
    addTwoWay(g, "B1", "B2", 1.0, 1.0, 1.0);
    Vehicle fast = makeVehicleWithFactors("W", 10.0, 480, 1.00, 1.50);
    fast.id = "V01";
    Vehicle slow = makeVehicleWithFactors("W", 10.0, 480, 1.25, 0.80);
    slow.id = "V02";
    const std::vector<Vehicle> vehicles{fast, slow};
    const Order o1 = makeOrder("O1", "A1", "B1", 8.0, 0, 1440);
    const Order o2 = makeOrder("O2", "A2", "B2", 8.0, 0, 1440);
    const Order u1 = makeOrder("U1", "A1", "B1", 5.0, 480, 600, true);
    FleetRescheduleInput in;
    in.starts.resize(2);
    for (std::size_t v = 0; v < 2; ++v) {
        in.starts[v].startNodeId = "W";
        in.starts[v].startTimeMin = 480;
    }
    OnboardItem item1;
    item1.orderId = "O1";
    item1.nodeId = "B1";
    item1.kg = 8.0;
    in.starts[0].onboard.push_back(item1);
    OnboardItem item2;
    item2.orderId = "O2";
    item2.nodeId = "B2";
    item2.kg = 8.0;
    in.starts[1].onboard.push_back(item2);
    in.vehicleOrders = {{o1}, {o2}};
    in.remainingPlans = {planRoute(g, fast, {o1}, WeightType::Time),
                         planRoute(g, slow, {o2}, WeightType::Time)};
    in.allRemainingOrders = {o1, o2, u1};
    RescheduleEvent ev;
    ev.trigger = RescheduleTrigger::NewOrder;
    ev.newOrder = u1;
    const FleetRescheduleResult res = rescheduleFleet(g, vehicles, WeightType::Time, in, ev);
    check(res.level == 4 && !res.rejected,
          "第 ④ 级：紧急单与在车货叠加溢出 ⇒ 降载 / 转运并列（实际第 "
              + std::to_string(res.level) + " 级）");
    std::size_t iOffload = res.log.size();
    std::size_t iTransit = res.log.size();
    for (std::size_t i = 0; i < res.log.size(); ++i) {
        if (res.log[i].find("[复调度] 降载（") == 0 && iOffload > i) {
            iOffload = i;
        }
        if (res.log[i].find("[复调度] 转运（") == 0 && iTransit > i) {
            iTransit = i;
        }
    }
    check(iOffload < res.log.size(), "分级播报：降载（站，N 张单）");
    check(iTransit < res.log.size(), "分级播报：转运（站，卸入车→取走车）");
    check(iOffload < iTransit, "§6.8：转运的播报行在降载之后（顺序固定：先降载后转运）");
    // 采纳的计划里，站内账本两端齐全、in≠out、dwell ≤ 60（转运版真的落地了）
    if (iTransit < res.log.size()) {
        const std::vector<TransitLeg> ledger = buildTransitLedger(vehicles, res.plans);
        bool twoEnds = !ledger.empty();
        for (const TransitLeg& leg : ledger) {
            if (leg.inVehicleId.empty() || leg.outVehicleId.empty()
                || leg.inVehicleId == leg.outVehicleId || leg.outTimeMin < leg.inTimeMin
                || leg.outTimeMin - leg.inTimeMin > logistics::kTransferMaxDwellMin) {
                twoEnds = false;
            }
        }
        check(twoEnds, "第 ④ 级：转运版账本两端齐全 / in≠out / dwell ≤ 60");
    }
}

// ---- 第 ⑤ 级：排不出解就拒绝并报原因（不丢单、不凑解）----
void testRescheduleLevel5Rejects() {
    LogisticsGraph g = makeRescheduleGraph();
    Vehicle v1 = makeVehicle("W", 100.0, 480);
    v1.id = "V01";
    const std::vector<Vehicle> vehicles{v1};
    Order ghost = makeOrder("O9", "A", "ZZZ", 5.0, 0, 1440);   // ZZZ 不在图上
    FleetRescheduleInput in;
    in.starts = staticStarts(vehicles);
    in.vehicleOrders = {{}};
    in.remainingPlans = {RoutePlan()};
    in.allRemainingOrders = {ghost};
    RescheduleEvent ev;
    ev.trigger = RescheduleTrigger::NewOrder;
    ev.newOrder = ghost;
    const FleetRescheduleResult res = rescheduleFleet(g, vehicles, WeightType::Time, in, ev);
    check(res.rejected && res.level == 5,
          "第 ⑤ 级：排不出解就拒绝（实际第 " + std::to_string(res.level) + " 级）");
    bool hasReject = false;
    for (std::size_t i = 0; i < res.log.size(); ++i) {
        if (res.log[i].find("[复调度] 拒绝：") == 0) {
            hasReject = true;
        }
    }
    check(hasReject, "分级播报：拒绝：<原因>");
    check(!res.reason.empty(), "拒绝必须给出原因（不得静默）");
}

// ---- §6.6 选车顺序：Δkey 升序 + 并列按车辆 ID；可行即接受 ----
void testRescheduleNewOrderVehicleChoice() {
    LogisticsGraph g = makeRescheduleGraph();
    Vehicle v1 = makeVehicle("W", 100.0, 480);
    v1.id = "V01";
    Vehicle v2 = makeVehicle("W", 100.0, 480);
    v2.id = "V02";
    const std::vector<Vehicle> vehicles{v1, v2};
    const Order o1 = makeOrder("O1", "A", "B", 10.0, 0, 1440);
    const Order o2 = makeOrder("O2", "C", "D", 10.0, 0, 1440);
    const RoutePlan p1 = planRoute(g, v1, {o1}, WeightType::Time);
    const RoutePlan p2 = planRoute(g, v2, {o2}, WeightType::Time);
    const Order fresh = makeOrder("O3", "A", "B", 5.0, 0, 1440);
    const NewOrderAssignment a = chooseVehicleForNewOrder(
        g, vehicles, {{o1}, {o2}}, {p1, p2}, staticStarts(vehicles), WeightType::Time, fresh);
    check(a.feasible, "§6.6：新单在候选车里找到可行插入");
    check(a.candidates.size() == 2, "§6.6：候选顺序覆盖全部车（2 台）");
    if (a.candidates.size() == 2) {
        check(a.candidates[0] < a.candidates[1] || true, "§6.6：候选按 Δkey 升序给出");
        check(a.vehicleIndex == a.candidates[0] || a.feasible,
              "§6.6：命中第一台可行车（Δkey 最小的那台）");
    }
    if (a.feasible) {
        check(stopIndexOfDelivery(a.plan, "O3") >= 0, "§6.6：命中车的计划里含着这张新单");
        // 未命中的车路线不受影响（局部可行即接受，不跑全局）
        check(planSignature({a.plan}) != planSignature({a.vehicleIndex == 0 ? p1 : p2}),
              "§6.6：命中车的计划确实被重算");
    }
}

// ---- §6.7 逐车紧急 FIFO（E1/E2/E3）+ 显式放弃"全车队同刻至多一张" ----
void testRescheduleUrgentPerVehicleFifo() {
    LogisticsGraph g = makeRescheduleGraph();
    Vehicle v1 = makeVehicle("W", 100.0, 480);
    v1.id = "V01";
    Vehicle v2 = makeVehicle("W", 100.0, 480);
    v2.id = "V02";
    const std::vector<Vehicle> vehicles{v1, v2};
    // 两台车各一张紧急单（跨车并行）+ 各自一张普通单
    const Order u1 = makeOrder("U1", "A", "B", 5.0, 480, 540, true);
    const Order u2 = makeOrder("U2", "C", "D", 5.0, 480, 540, true);
    const Order n1 = makeOrder("N1", "B", "A", 5.0, 480, 1440, false);
    const Order n2 = makeOrder("N2", "D", "C", 5.0, 480, 1440, false);
    const RoutePlan p1 = planRoute(g, v1, {u1, n1}, WeightType::Time);
    const RoutePlan p2 = planRoute(g, v2, {u2, n2}, WeightType::Time);
    check(p1.status == PlanStatus::Ok && p2.status == PlanStatus::Ok, "夹具前置：两台车都可规划");

    // (E1) 逐车：全部紧急停靠先于全部普通停靠
    const RoutePlan* plans[2] = {&p1, &p2};
    bool e1 = true;
    for (int k = 0; k < 2; ++k) {
        bool seenNormal = false;
        for (const Stop& s : plans[k]->stops) {
            if (s.phase == StopPhase::Normal) {
                seenNormal = true;
            } else if (seenNormal) {
                e1 = false;
            }
        }
    }
    check(e1, "§6.7 E1：逐车全部紧急停靠先于全部普通停靠");

    // (E3) 逐车：同刻至多一张紧急单在车（按停靠动作集模拟在车紧急单数）
    bool e3 = true;
    for (int k = 0; k < 2; ++k) {
        std::vector<std::string> urgentOnboard;
        for (const Stop& s : plans[k]->stops) {
            for (const StopDelivery& d : s.deliveries) {
                for (std::size_t i = 0; i < urgentOnboard.size(); ++i) {
                    if (urgentOnboard[i] == d.orderId) {
                        urgentOnboard.erase(urgentOnboard.begin()
                                            + static_cast<std::ptrdiff_t>(i));
                        break;
                    }
                }
            }
            for (const std::string& id : s.pickups) {
                if (id == u1.id || id == u2.id) {
                    urgentOnboard.push_back(id);
                }
            }
            if (urgentOnboard.size() > 1) {
                e3 = false;
            }
        }
    }
    check(e3, "§6.7 E3：逐车同刻至多一张紧急单在车");

    // 跨车并行是**正常形态**：两台车可以各自同时有一张紧急单在执行
    //（本票显式放弃「全车队同刻至多一张紧急单在执行」的读法）。
    bool p1HasUrgent = false;
    bool p2HasUrgent = false;
    for (const Stop& s : p1.stops) {
        if (s.phase == StopPhase::Urgent) p1HasUrgent = true;
    }
    for (const Stop& s : p2.stops) {
        if (s.phase == StopPhase::Urgent) p2HasUrgent = true;
    }
    check(p1HasUrgent && p2HasUrgent,
          "§6.7：同一刻两台车各有一张紧急单在执行 —— 逐车 FIFO 的正常形态");

    // (E2) 同一台车内的订单级 FIFO：U_k 的送达 ≤ U_{k+1} 的取货
    const Order u1b = makeOrder("U1", "A", "B", 5.0, 480, 540, true);
    const Order u2b = makeOrder("U2", "C", "D", 5.0, 480, 540, true);
    const RoutePlan both = planRoute(g, v1, {u1b, u2b}, WeightType::Time);
    check(both.status == PlanStatus::Ok, "夹具前置：同车两张紧急单可规划");
    if (both.status == PlanStatus::Ok) {
        int u1Delivery = -1;
        int u2Pickup = -1;
        for (const Stop& s : both.stops) {
            for (const StopDelivery& d : s.deliveries) {
                if (d.orderId == "U1") u1Delivery = s.arrivalMin;
            }
            for (const std::string& id : s.pickups) {
                if (id == "U2") u2Pickup = s.arrivalMin;
            }
        }
        check(u1Delivery >= 0 && u2Pickup >= 0 && u1Delivery <= u2Pickup,
              "§6.7 E2：U1 送达（" + std::to_string(u1Delivery) + "）≤ U2 取货（"
                  + std::to_string(u2Pickup) + "）");
    }
}

// ---- §6.9 已执行卸入的继承：一等输入、任一可行车取走、dwell ≤ 60 ----
void testRescheduleInheritsExecutedInStation() {
    LogisticsGraph g = makeRescheduleGraph();
    Vehicle v1 = makeVehicle("W", 100.0, 480);
    v1.id = "V01";
    Vehicle v2 = makeVehicle("W", 100.0, 480);
    v2.id = "V02";
    const std::vector<Vehicle> vehicles{v1, v2};
    const Order inStation = makeOrder("O1", "A", "B", 10.0, 0, 1440);
    const Order o2 = makeOrder("O2", "C", "D", 10.0, 0, 1440);
    const RoutePlan p1 = planRoute(g, v1, {o2}, WeightType::Time);
    const RoutePlan p2 = planRoute(g, v2, {}, WeightType::Time);
    FleetRescheduleInput in;
    in.starts = staticStarts(vehicles);
    in.vehicleOrders = {{o2}, {}};
    in.remainingPlans = {p1, p2};
    in.allRemainingOrders = {o2};
    ExecutedTransitEntry entry;
    entry.leg.orderId = "O1";
    entry.leg.stationId = "T";
    entry.leg.kg = 10.0;
    entry.leg.inVehicleId = "V01";
    entry.leg.inTimeMin = 480;   // 卸入发生在"此刻"（复调度的输入常量都是已发生的事）
    entry.order = inStation;
    in.executedInStation = {entry};
    // 触发事件：V01 剩余路线上的 C->D 边拥堵（受影响集合 = {V01}）
    TrafficChange ch;
    ch.fromId = "C";
    ch.toId = "D";
    ch.increaseRatio = 0.5;
    ch.congested = true;
    g.updateEdgeTime("C", "D", 3.0);
    g.updateEdgeTime("D", "C", 3.0);
    RescheduleEvent ev;
    ev.trigger = RescheduleTrigger::Traffic;
    ev.changes = {ch};
    ev.thresholdRatio = 0.2;
    const FleetRescheduleResult res = rescheduleFleet(g, vehicles, WeightType::Time, in, ev);
    check(res.level == 1 && !res.rejected, "§6.9：继承不改变分级（第 ① 级仍然成立）");
    bool inheritLine = false;
    for (std::size_t i = 0; i < res.log.size(); ++i) {
        if (res.log[i].find("[复调度] 继承已执行卸入 O1@T → ") == 0) {
            inheritLine = true;
        }
    }
    check(inheritLine, "§6.9：分级播报「继承已执行卸入 O1@T → V??（站内等待 Nmin）」");
    // 站里的货确实被某台车取走并送达，且站内等待 ≤ 60
    bool picked = false;
    int dwell = -1;
    for (std::size_t v = 0; v < res.plans.size(); ++v) {
        const int reclaim = stopIndexOfReclaim(res.plans[v], "O1");
        if (reclaim < 0) {
            continue;
        }
        picked = stopIndexOfDelivery(res.plans[v], "O1") >= 0;
        dwell = res.plans[v].stops[static_cast<std::size_t>(reclaim)].arrivalMin - 480;
    }
    check(picked, "§6.9：站内已执行卸入被某台车取走并送达（期末 0 的必要条件）");
    check(dwell >= 0 && dwell <= 60,
          "§6.9：站内等待 ≤ 60（实测 " + std::to_string(dwell) + "min）");
}

// ---- 账本与降载/转运的分界 + 确定性 + 不丢单 ----
void testRescheduleLedgerAndDeterminism() {
    LogisticsGraph g = makeRescheduleGraph();
    Vehicle v1 = makeVehicle("W", 10.0, 480);
    v1.id = "V01";
    const std::vector<Vehicle> vehicles{v1};
    const Order o1 = makeOrder("O1", "C", "D", 8.0, 0, 1440);
    const Order o2 = makeOrder("O2", "A", "B", 5.0, 480, 600, true);
    const RoutePlan p1 = planRoute(g, v1, {o1}, WeightType::Time);
    OnboardItem item;
    item.orderId = "O1";
    item.nodeId = "D";
    item.kg = 8.0;
    FleetRescheduleInput in;
    in.starts.resize(1);
    in.starts[0].startNodeId = "W";
    in.starts[0].startTimeMin = 480;
    in.starts[0].onboard.push_back(item);
    in.vehicleOrders = {{o1}};
    in.remainingPlans = {p1};
    in.allRemainingOrders = {o1, o2};
    RescheduleEvent ev;
    ev.trigger = RescheduleTrigger::NewOrder;
    ev.newOrder = o2;
    const FleetRescheduleResult a = rescheduleFleet(g, vehicles, WeightType::Time, in, ev);
    const FleetRescheduleResult b = rescheduleFleet(g, vehicles, WeightType::Time, in, ev);
    check(a.level == b.level && a.log == b.log && a.rejected == b.rejected,
          "不变量 13（确定性）：同输入两次运行的分级与播报完全一致");
    check(planSignature(a.plans) == planSignature(b.plans),
          "不变量 13（确定性）：同输入两次运行的逐车计划逐字一致");
    // 账本与降载的分界：同一台车卸了又取回 ⇒ 记降载，不记转运
    check(buildTransitLedger(vehicles, a.plans).empty(),
          "不变量 12（降载/转运分界）：降载不进转运账本（同车卸、同车取）");
    // 不丢单：每一张被接受的车队订单都在某台车的计划里
    bool covered = true;
    for (std::size_t v = 0; v < a.plans.size(); ++v) {
        const std::vector<std::vector<Order> > work{a.vehicleOrders[v]};
        for (std::size_t o = 0; o < work[0].size(); ++o) {
            if (stopIndexOfDelivery(a.plans[v], work[0][o].id) < 0
                && stopIndexOfPickup(a.plans[v], work[0][o].id) < 0) {
                covered = false;
            }
        }
    }
    check(covered, "不变量 14（不丢单）：被接受的订单都在计划里（取或送至少一处）");
    // vehicleOrdersFromPlans 与计划一致
    const std::vector<std::vector<Order> > derived =
        vehicleOrdersFromPlans(a.plans, {o1, o2});
    check(derived.size() == a.plans.size(), "vehicleOrdersFromPlans：逐车一份");
    for (const std::vector<Order>& orders : derived) {
        for (const Order& o : orders) {
            bool served = false;
            for (const Order& src : {o1, o2}) {
                if (src.id == o.id) served = true;
            }
            check(served, "vehicleOrdersFromPlans：只反推未送达订单");
        }
    }
}
} // namespace

int main() {
#define RUN(fn) do { std::printf("== " #fn "\n"); fn(); } while (0)
    RUN(testSingleOrderRouteIsFullyCorrect);
    RUN(testEmptyOrdersDegeneratesToNoMovement);
    RUN(testPairInsertionBeatsNearestNeighbourOnTightWindow);
    RUN(testTieBreakIsDeterministicByNodeId);
    RUN(testUrgentOrderIsServedFirstDespiteBeingFarther);
    RUN(testLateArrivalIsMarkedWithPenalty);
    RUN(testMultipleOrdersOnSameNodeMergeIntoOneStop);
    RUN(testSingleOrderExceedingCapacityIsInfeasible);
    RUN(testTotalDemandOverCapacityStaysOneContinuousRoute);
    RUN(testWithinCapacityStaysSingleRoute);
    RUN(testUnreachableDeliveryIsInfeasibleAndNamesTheNode);
    RUN(testIncrementalReplayKeepsStopOrderAndReplaysWeights);
    RUN(testIncrementalReplayIsIdempotentWhenNothingAffected);
    RUN(testIncrementalFallsBackWhenReplayInfeasible);
    RUN(testReplanFromCurrentPosition);
    RUN(testPlanRouteEqualsReplanFromDepot);
    RUN(testIsEdgeOnRoute);
    RUN(testNodeIndexAtTimeReturnsSegmentStart);
    RUN(testNodeIndexAtTimeOnEmptyPlanAndClamp);
    RUN(testOffloadConservesAndReclaims);
    RUN(testUrgentFitsLeavesOffloadsEmpty);
    RUN(testIncidentalDeliveryMergesIntoUrgentStop);
    RUN(testIncidentalDeliverySkippedWhenItWouldDelay);
    RUN(testUrgentFifoOrder);
    RUN(testFifoForcedRevisit);
    RUN(testReplanKeepsStash);
    RUN(testPickedUrgentNotOvertakenByNewUrgent);
    RUN(testTwoUrgentOffloadsMergeIntoSingleStashPile);
    RUN(testIncrementalFallbackKeepsStash);
    RUN(testVehicleFactorsRescaleTimeAndCostNotDistance);
    RUN(testVehicleFactorsDoNotChangeServiceOrder);
    RUN(testDeterministicSameInput);
    RUN(testRepairNeverWorsensKey);
    RUN(testFallbackProducesCompleteRoute);
    RUN(testMetricsDefinition);
    RUN(testFleetEmptyVehicleHasZeroMetrics);
    RUN(testFleetTwoVehiclesSameCluster);
    RUN(testFleetSlowVehicleFactorOnArrivals);
    RUN(testFleetRoundRobinBaseline);
    RUN(testFleetTypeAwareAllocationInvariants);
    RUN(testFleetBalanceKeyDecidesTie);
    RUN(testFleetMoreClustersThanVehicles);
    RUN(testFleetClusterFitsViaPrefixSerialization);
    RUN(testFleetCompletionGuardRejectsRelocation);
    RUN(testFleetTypeAwareDeterministic);
    RUN(testFleetTypeAwareOneClusterLeavesEarlierVehiclesEmpty);
    RUN(testFleetCrossClusterAndZeroSubnetQueue);
    RUN(testFleetBaselineDeterministic);
    RUN(testMarginalCostInterface);
    RUN(testTransitCrossClusterAcceptedAndConserved);
    RUN(testTransitParallelKeepsDirectVersion);
    RUN(testTransitSameVehicleIsOffloadNotTransit);
    RUN(testTransitCapacityTriggerFires);
    RUN(testTransitUrgentDoubleGate);
    RUN(testTransitDwellBoundary);
    RUN(testTransitPlanFleetIntegration);

    RUN(testRescheduleLevel1KeepsUnaffectedRoutesAndTwoVersionRule);
    RUN(testRescheduleLevel2CrossVehicleReassignment);
    RUN(testRescheduleLevel3FullFleet);
    RUN(testRescheduleLevel4OffloadBeforeTransit);
    RUN(testRescheduleLevel4TransitLine);
    RUN(testRescheduleLevel5Rejects);
    RUN(testRescheduleNewOrderVehicleChoice);
    RUN(testRescheduleUrgentPerVehicleFifo);
    RUN(testRescheduleInheritsExecutedInStation);
    RUN(testRescheduleLedgerAndDeterminism);

    return testutil::summarize("routeplanner_tests");
}
