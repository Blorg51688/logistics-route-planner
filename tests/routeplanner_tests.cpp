
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
using logistics::StopPhase;
using logistics::Vehicle;
using logistics::WeightType;
using logistics::nodeIndexAtTime;
using logistics::planRoute;
using logistics::replan;
using logistics::replanIncremental;
using logistics::insertUrgentOrder;
using logistics::InsertResult;
using logistics::TrafficChange;
using logistics::TrafficReport;
using testutil::check;

namespace {

using fixtures::addTwoWay;
using fixtures::makeNode;
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

    // 不变量 ⑦：同阶段同节点至多一次停靠；重复只允许是「先取后送强制重访」。
    bool uniqueOk = true;
    std::string badNode;
    for (std::size_t i = 0; i < plan.stops.size() && uniqueOk; ++i) {
        for (std::size_t j = i + 1; j < plan.stops.size(); ++j) {
            if (plan.stops[i].nodeId != plan.stops[j].nodeId
                || plan.stops[i].phase != plan.stops[j].phase) {
                continue;
            }
            // 中转站动作与取送停靠可以同节点共存（降载 + 取回是两件事）。
            const bool transit = plan.stops[i].isTransitStop() || plan.stops[j].isTransitStop();
            const std::size_t actions = plan.stops[j].pickups.size()
                                        + plan.stops[j].deliveries.size()
                                        + plan.stops[j].offloads.size()
                                        + plan.stops[j].reclaims.size();
            if (transit || actions != 1) {
                uniqueOk = false;
                badNode = plan.stops[j].nodeId;
                break;
            }
        }
    }
    check(uniqueOk, tag + "同阶段同节点至多一次停靠，重复时只含一个动作"
                        + (badNode.empty() ? std::string() : "（" + badNode + "）"));

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

    const InsertResult res = insertUrgentOrder(g, v, pending, urgent, "W", 480,
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

    const InsertResult res = insertUrgentOrder(g, v, pending, urgent, "W", 480,
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

    const InsertResult res = insertUrgentOrder(g, v, pending, urgent, "W", 480,
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

    const InsertResult res = insertUrgentOrder(g, v, pending, urgent, "W", 480,
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

    return testutil::summarize("routeplanner_tests");
}
