// 集成测试：用真实的 config/default.ini 跑通「配置 -> 规划」全链路。
// 单元测试（routeplanner_tests）用人工小图钉住语义；
// 这里验证真实数据集确实能支撑题目要求的完整配送（B6），
// 并顺带打印两种策略的路线摘要，便于人工核对演示数据。
#include <cstdio>
#include <string>
#include <vector>

#include "core/Config.h"
#include "core/RoutePlanner.h"
#include "core/Traffic.h"
#include "graph_fixtures.h"
#include "io/ConfigLoader.h"
#include "plan_golden.h"
#include "test_util.h"

using logistics::Config;
using logistics::LogisticsGraph;
using logistics::ConfigLoader;
using logistics::PlanStatus;
using logistics::RoutePlan;
using logistics::Vehicle;
using logistics::WeightType;
using logistics::planRoute;
using logistics::planFleet;
using logistics::planFleetSequentialBaseline;
using logistics::clusterOfOrder;
using logistics::isCrossCluster;
using logistics::FleetPlan;
using logistics::RelocateRecord;
using logistics::replan;
using logistics::simulateTrafficChange;
using logistics::needsReplan;
using logistics::insertOrder;
using logistics::InsertResult;
using logistics::TrafficReport;
using testutil::check;

#ifndef DEFAULT_CONFIG_PATH
#error "缺少 DEFAULT_CONFIG_PATH 编译定义"
#endif

namespace {

// 去重后的订单目标配送点数量 = 期望的停靠点数量
std::size_t distinctOrderNodes(const Config& cfg) {
    std::vector<std::string> seen;
    for (const logistics::Order& o : cfg.orders) {
        bool found = false;
        for (const std::string& id : seen) {
            if (id == o.nodeId) {
                found = true;
                break;
            }
        }
        if (!found) {
            seen.push_back(o.nodeId);
        }
    }
    return seen.size();
}

// 单张订单的窗口跨度（分钟，逐单口径）。
//
// 旧实现按**停靠点**取同节点全部订单窗口的并集再判跨度 —— 那会把紧窗单的迟到
// 藏在同节点宽窗单里（规格一 §6.5 升级④点名要改逐单，与 #18 Q14=A 同源）。
int orderWindowSpanMinutes(const logistics::Order& o) {
    return o.windowEndMin - o.windowStartMin;
}

const logistics::Order* findOrder(const Config& cfg, const std::string& id) {
    for (const logistics::Order& o : cfg.orders) {
        if (o.id == id) {
            return &o;
        }
    }
    return nullptr;
}

// 规格一 #18 §12 的 12 条不变量在**真实数据**上的机械复核。
// 全部读 RoutePlan 产物（用户可见输出同源），不重算规划。
// 单车不变量的**逐车**版本（规格二 §3.6）：起点/容量/订单集合都取该车自己的，
// 不得再用 vehicles[0] / 全量订单一笔带过（那正是多车下会漏掉越界的地方）。
void checkPlanInvariants(const Config& cfg, const Vehicle& vehicle,
                         const std::vector<logistics::Order>& orders, const RoutePlan& plan,
                         const std::string& label) {
    const std::string tag = label + "：";

    check(plan.status == PlanStatus::Ok, tag + "可规划出可行路线（reason: " + plan.reason + "）");
    if (plan.status != PlanStatus::Ok) {
        return;
    }
    check(!plan.nodes.empty(), tag + "路线非空");
    if (plan.nodes.empty()) {
        return;
    }
    check(plan.nodes.front() == vehicle.startNodeId, tag + "从该车起始仓库出发");
    check(plan.nodes.back() == vehicle.startNodeId, tag + "返回该车起始仓库（同一基地）");

    bool walkable = true;
    for (std::size_t i = 1; i < plan.nodes.size(); ++i) {
        if (plan.nodes[i] == plan.nodes[i - 1]) {
            continue;   // 同一节点连续两次停靠 = 零位移（无自环边）
        }
        if (cfg.graph.findEdge(plan.nodes[i - 1], plan.nodes[i]) == nullptr) {
            walkable = false;
        }
    }
    check(walkable, tag + "相邻节点之间都存在有向边");
    check(plan.nodes.size() == plan.nodeArrivalMin.size()
              && plan.nodes.size() == plan.nodeIsStop.size(),
          tag + "扁平序列的节点/到达时刻/停靠标记三个数组等长");
    std::size_t stopFlags = 0;
    for (std::size_t i = 0; i < plan.nodeIsStop.size(); ++i) {
        if (plan.nodeIsStop[i]) {
            ++stopFlags;
        }
    }
    check(stopFlags == plan.stops.size(), tag + "停靠标记的 true 数 = stops 数（展平视图与真源一致）");

    // 单条连续路线：仓库只作为首个停靠出现（起点就地取货），中途不得回仓装货。
    bool depotOnce = true;
    for (std::size_t i = 0; i < plan.stops.size(); ++i) {
        if (plan.stops[i].nodeId == vehicle.startNodeId) {
            depotOnce = false;
        }
    }
    check(depotOnce, tag + "中途不停靠仓库（单条连续路线，仓库装货语义已退役）");

    // 不变量 ① ②：每张订单恰好一次送达、至多一次取货，且取货早于送达。
    for (const logistics::Order& o : orders) {
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
        check(picks <= 1, tag + "订单 " + o.id + " 至多取货一次");
        if (picks == 1 && delivs == 1) {
            check(pickIdx < delivIdx, tag + "订单 " + o.id + " 的取货早于送达");
        }
    }

    // 不变量 ③：前缀载重 ∈ [0, C]，停靠内先卸后装，remainingLoadKg = 离站在车量。
    double load = 0.0;
    bool prefixOk = true;
    bool unloadOk = true;
    for (const logistics::Stop& st : plan.stops) {
        double after = load;
        for (const logistics::StopDelivery& d : st.deliveries) {
            const logistics::Order* o = findOrder(cfg, d.orderId);
            if (o != nullptr) {
                after -= o->demandKg;
            }
        }
        for (const std::string& id : st.offloads) {
            const logistics::Order* o = findOrder(cfg, id);
            if (o != nullptr) {
                after -= o->demandKg;
            }
        }
        if (after < -1e-9) {
            unloadOk = false;
        }
        for (const std::string& id : st.pickups) {
            const logistics::Order* o = findOrder(cfg, id);
            if (o != nullptr) {
                after += o->demandKg;
            }
        }
        for (const std::string& id : st.reclaims) {
            const logistics::Order* o = findOrder(cfg, id);
            if (o != nullptr) {
                after += o->demandKg;
            }
        }
        if (after < -1e-9 || after > vehicle.capacityKg + 1e-6) {
            prefixOk = false;
        }
        if (std::fabs(st.remainingLoadKg - after) > 1e-6) {
            prefixOk = false;
        }
        load = after;
    }
    check(prefixOk, tag + "任一停靠后的在车量都在 [0, 载重上限] 且等于 remainingLoadKg");
    check(unloadOk, tag + "停靠内先卸后装（卸货不超过车上已有）");

    // 不变量 ④：硬优先。⑤ FIFO 与 ⑥ 同刻至多一张紧急单：本数据集全普通，只查结构。
    bool seenNormal = false;
    bool hardOk = true;
    for (const logistics::Stop& st : plan.stops) {
        if (st.phase == logistics::StopPhase::Normal) {
            seenNormal = true;
        } else if (seenNormal) {
            hardOk = false;
        }
    }
    check(hardOk, tag + "硬优先：紧急停靠全部先于普通停靠");

    // 不变量 ⑦（规格字面）：**同阶段同节点至多一次停靠**；唯一例外是强制重访 ——
    // 把重访停靠的那一个动作并入该节点首次停靠后，F2（先取后送）/ F5（紧急 FIFO）/
    // F3（前缀载重、先卸后装）**失败**。默认数据集要求**非强制重复为 0**。
    // 判据与实现同源：tests/graph_fixtures.h 的 fixtures::nonForcedRevisitStops。
    const int nonForced =
        fixtures::nonForcedRevisitStops(plan.stops, orders, vehicle.capacityKg, 0.0);
    check(nonForced == 0, tag + "非强制重复停靠必须为 0（并入首次停靠后 F2/F5/F3 都过的重复"
                              "一律算违规），实际 " + std::to_string(nonForced));

    // 不变量 ⑧ ⑨：停靠零耗时、逐单 penalty。
    bool zeroDwell = true;
    bool penaltyOk = true;
    int penaltySum = 0;
    for (const logistics::Stop& st : plan.stops) {
        if (st.rawArrivalMin + st.waitMin != st.arrivalMin) {
            zeroDwell = false;
        }
        int stopPenalty = 0;
        for (const logistics::StopDelivery& d : st.deliveries) {
            const logistics::Order* o = findOrder(cfg, d.orderId);
            const int we = (o == nullptr) ? 0 : o->windowEndMin;
            const int expect = st.arrivalMin > we ? (st.arrivalMin - we) : 0;
            if (d.penaltyMin != expect) {
                penaltyOk = false;
            }
            stopPenalty += d.penaltyMin;
        }
        if (stopPenalty != st.penaltyMin) {
            penaltyOk = false;
        }
        penaltySum += stopPenalty;
    }
    check(zeroDwell, tag + "停靠零耗时：rawArrival + wait == arrival");
    check(penaltyOk, tag + "penalty 逐单：每单按自己的 we 计、停靠级为逐单之和（G7）");
    check(penaltySum == plan.totalPenaltyMin, tag + "总 penalty = 各停靠逐单之和（G7）");

    // 不变量 ⑩ ⑪：降载账本。
    std::string stashStation;
    bool ledgerOk = true;
    for (const logistics::OffloadLeg& leg : plan.offloads) {
        if (leg.reclaimTimeMin < 0 || leg.reclaimTimeMin < leg.offloadTimeMin) {
            ledgerOk = false;
        }
        if (stashStation.empty()) {
            stashStation = leg.station;
        } else if (stashStation != leg.station) {
            ledgerOk = false;
        }
    }
    check(ledgerOk, tag + "降载账本：期末已取回、取回不早于卸下、全局单堆锁站");

    // 指标口径（#29 §6.6 的 G3/G4/G5）。
    const logistics::RouteMetrics& m = plan.metrics;
    check(m.stopCount == static_cast<int>(plan.stops.size()), tag + "指标·停靠点数 = stops 数");
    int actions = 0;
    for (const logistics::Stop& st : plan.stops) {
        actions += static_cast<int>(st.pickups.size() + st.deliveries.size());
    }
    check(m.actionCount == actions, tag + "指标·动作数 = 取货 + 送达动作数");
    check(m.actionCount == 2 * static_cast<int>(orders.size()),
          tag + "结构性恒等：动作数 = 2 × 取送对数（数据全为 A→B 取送对）");
    check(m.pickupOnTimeCount >= m.deliveryOnTimeCount, tag + "取货准时数 >= 送达准时数（G4）");
    check(m.avgLoadKg <= m.peakLoadKg + 1e-9, tag + "平均在车 <= 峰值（G5）");
    check(m.peakLoadKg <= vehicle.capacityKg + 1e-6, tag + "峰值 <= **该车**载重上限（G5）");
    check(std::fabs(m.loadUtilization - m.avgLoadKg / vehicle.capacityKg) < 0.005,
          tag + "利用率 == 平均/**该车**容量（G5）");
    check(m.deliveryTotal == static_cast<int>(orders.size()),
          tag + "送达准时率分母 = 该车已规划订单数（G6）");
}

// 顺序轮转基线（规格二 §4.5）：订单 ID 升序、第 i 单 -> 第 (i mod k) 台车。
// **与 C++ planFleet 的分配同口径**，但由测试自己算一遍——测试不读被测实现的分配，
// 而是把"该车应服务的订单集合"独立算出来后交给 planRoute 逐车规划，
// 这样"分配漂移"（例如少给一单）会立刻在逐车覆盖断言上暴露。
std::vector<std::vector<logistics::Order> > roundRobinAssignment(const Config& cfg) {
    std::vector<std::size_t> idx(cfg.orders.size());
    for (std::size_t i = 0; i < idx.size(); ++i) {
        idx[i] = i;
    }
    for (std::size_t i = 1; i < idx.size(); ++i) {
        const std::size_t key = idx[i];
        std::size_t j = i;
        while (j > 0 && cfg.orders[key].id < cfg.orders[idx[j - 1]].id) {
            idx[j] = idx[j - 1];
            --j;
        }
        idx[j] = key;
    }
    std::vector<std::vector<logistics::Order> > out(cfg.vehicles.size());
    for (std::size_t i = 0; i < idx.size(); ++i) {
        out[i % cfg.vehicles.size()].push_back(cfg.orders[idx[i]]);
    }
    return out;
}

// 车队级 13 字段 + 三条恒等式（规格二 §3.4 / §3.6-8）的守卫。
// 全部读 planFleet 产物（与 --plan-summary 同源），不重算规划。
void checkFleetInvariants(const Config& cfg, const FleetPlan& fleet, const std::string& label) {
    const std::string tag = label + "：";
    check(fleet.vehicles.size() == cfg.vehicles.size(), tag + "逐车计划数 == 车辆数");
    check(fleet.allOk, tag + "全部车辆都可规划（reason: " + fleet.reason + "）");

    // ① Σ 每车 == 车队（距离 / 成本 / penalty / 停靠数 / 动作数）。
    double sumDist = 0.0;
    double sumCost = 0.0;
    int sumPenalty = 0;
    int sumStops = 0;
    int sumActions = 0;
    double sumTime = 0.0;
    double maxReturn = -1.0;
    int minDepart = cfg.vehicles.empty() ? 0 : cfg.vehicles[0].departTimeMin;
    for (std::size_t v = 0; v < fleet.vehicles.size() && v < cfg.vehicles.size(); ++v) {
        const RoutePlan& p = fleet.vehicles[v];
        sumDist += p.totalDistanceKm;
        sumCost += p.totalCostYuan;
        sumPenalty += p.totalPenaltyMin;
        sumStops += p.metrics.stopCount;
        sumActions += p.metrics.actionCount;
        sumTime += p.totalTimeMin;
        const double ret = static_cast<double>(cfg.vehicles[v].departTimeMin) + p.totalTimeMin;
        if (ret > maxReturn) {
            maxReturn = ret;
        }
        if (cfg.vehicles[v].departTimeMin < minDepart) {
            minDepart = cfg.vehicles[v].departTimeMin;
        }
    }
    check(fixtures::nearlyEqual(fleet.totalDistanceKm, sumDist, 1e-6),
          tag + "Σ每车距离 == 车队总距离");
    check(fixtures::nearlyEqual(fleet.totalCostYuan, sumCost, 1e-6),
          tag + "Σ每车成本 == 车队总成本");
    check(fleet.totalPenaltyMin == sumPenalty, tag + "Σ每车 penalty == 车队总 penalty");
    check(fleet.stopCount == sumStops, tag + "Σ每车停靠数 == 车队停靠数");
    check(fleet.actionCount == sumActions, tag + "Σ每车动作数 == 车队动作数");

    // ② 完工耗时 == max(返回) − min(发车)；③ Σ每车耗时 == 车时合计。
    check(fixtures::nearlyEqual(fleet.completionTimeMin, maxReturn - minDepart, 1e-6),
          tag + "完工耗时 == max(各车返回) − min(各车发车)（实际 "
              + std::to_string(fleet.completionTimeMin) + " vs "
              + std::to_string(maxReturn - minDepart) + "）");
    check(fixtures::nearlyEqual(fleet.sumVehicleTimeMin, sumTime, 1e-6),
          tag + "Σ每车耗时 == 车时合计");

    // ④ 两条口径**不得互相顶替**：多车下完工耗时严格小于车时合计（空闲车会让它更明显）。
    if (cfg.vehicles.size() > 1) {
        check(fleet.completionTimeMin < fleet.sumVehicleTimeMin - 1e-6,
              tag + "完工耗时 ≠ 车时合计（" + std::to_string(fleet.completionTimeMin) + " vs "
                  + std::to_string(fleet.sumVehicleTimeMin) + "），二者不得互相顶替");
    }

    // ⑤ 车队级指标是 Σ/Σ：两率分母 = 全部订单数；峰值 = 各车峰值 max。
    check(fleet.metrics.pickupTotal == static_cast<int>(cfg.orders.size()),
          tag + "车队取货准时率分母 == 订单数");
    check(fleet.metrics.deliveryTotal == static_cast<int>(cfg.orders.size()),
          tag + "车队送达准时率分母 == 订单数");
    double peak = 0.0;
    for (const RoutePlan& p : fleet.vehicles) {
        if (p.metrics.peakLoadKg > peak) {
            peak = p.metrics.peakLoadKg;
        }
    }
    check(fixtures::nearlyEqual(fleet.metrics.peakLoadKg, peak, 1e-6),
          tag + "车队峰值 == max(各车峰值)");
    check(fleet.metrics.avgLoadKg <= fleet.metrics.peakLoadKg + 1e-9,
          tag + "车队平均在车 <= 车队峰值（G5）");

    // ⑤b 车队级第 12 项（载重利用率）的口径（评审修复 B）：它是**派生时间加权聚合**
    //     Σ(在车量×段时长) / Σ(该车 capacity×该车运行时长)。G5 只钉逐车「平均÷该车容量」，
    //     规格没有给车队级公式 ⇒ 这里从逐车指标反算，把口径钉成可核验（容差 0.1）。
    {
        double loadInt = 0.0;
        double capTime = 0.0;
        double timeSum = 0.0;
        for (std::size_t v = 0; v < fleet.vehicles.size() && v < cfg.vehicles.size(); ++v) {
            const double t = fleet.vehicles[v].totalTimeMin;
            if (t <= 0.0) {
                continue;
            }
            loadInt += fleet.vehicles[v].metrics.avgLoadKg * t;
            capTime += cfg.vehicles[v].capacityKg * t;
            timeSum += t;
        }
        check(timeSum > 0.0 && capTime > 0.0, tag + "逐车有正耗时（利用率反算前提）");
        if (timeSum > 0.0 && capTime > 0.0) {
            check(fixtures::nearlyEqual(fleet.metrics.avgLoadKg, loadInt / timeSum, 1e-6),
                  tag + "车队平均在车 = Σ(逐车平均×该车耗时)/Σ耗时（时间加权）");
            check(fixtures::nearlyEqual(fleet.metrics.loadUtilization * 100.0,
                                        100.0 * loadInt / capTime, plan_golden::kTolRate),
                  tag + "车队第 12 项利用率 = 时间加权 Σ(在车量×段时长)/Σ(capacity×运行时长)"
                        "（容差 0.1）");
        }
    }
}

// 分配表 -> 每车订单集合。**只读 FleetPlan 的公开分配产物**（不重算分配），
// 随后 checkPlanInvariants 用计划自身的 stops 反向核对，所以分配表与路线是双向对拍。
std::vector<std::vector<logistics::Order> > allocationOrderSets(const Config& cfg,
                                                                const FleetPlan& fleet) {
    std::vector<std::vector<logistics::Order> > out(cfg.vehicles.size());
    for (std::size_t i = 0; i < fleet.allocationOrderIds.size(); ++i) {
        const std::size_t v = fleet.allocationVehicleIndex[i];
        const logistics::Order* o = findOrder(cfg, fleet.allocationOrderIds[i]);
        if (o != nullptr && v < out.size()) {
            out[v].push_back(*o);
        }
    }
    return out;
}

FleetPlan planFleetForWeight(const Config& cfg, WeightType weight, const char* label) {
    // 默认入口 = 类型感知三阶段（规格二 §4 / #40）。
    const FleetPlan fleet = planFleet(cfg.graph, cfg.vehicles, cfg.orders, weight);
    checkFleetInvariants(cfg, fleet, label);

    // 逐车不变量（每车自己的起点/容量/订单集合）+ 逐车摘要打印。
    // 订单集合取自**分配表**；checkPlanInvariants 再用该车 stops 反向核对
    // （分配说归它的单必须真在路线上；动作数 = 2 × 该车单数）。
    const std::vector<std::vector<logistics::Order> > assigned = allocationOrderSets(cfg, fleet);
    int lateTotal = 0;
    for (std::size_t v = 0; v < fleet.vehicles.size() && v < cfg.vehicles.size(); ++v) {
        const RoutePlan& plan = fleet.vehicles[v];
        checkPlanInvariants(cfg, cfg.vehicles[v], assigned[v], plan,
                            label + std::string("·车 ") + cfg.vehicles[v].id);
        std::printf("      %s 车 %s: 距离 %.3fkm  耗时 %.3fmin  成本 %.3f元  penalty %dmin  "
                    "停靠 %d 站 动作 %d 次\n",
                    label, cfg.vehicles[v].id.c_str(), plan.totalDistanceKm,
                    plan.totalTimeMin, plan.totalCostYuan, plan.totalPenaltyMin,
                    plan.metrics.stopCount, plan.metrics.actionCount);
        for (const logistics::OrderMetric& om : plan.metrics.orders) {
            if (om.late) {
                ++lateTotal;
            }
        }
    }
    std::printf("      %s 车队: 总距离 %.3fkm  完工耗时 %.3fmin（车时合计 %.3fmin）  "
                "总成本 %.3f元  penalty %dmin  停靠 %d 站 动作 %d 次\n",
                label, fleet.totalDistanceKm, fleet.completionTimeMin,
                fleet.sumVehicleTimeMin, fleet.totalCostYuan, fleet.totalPenaltyMin,
                fleet.stopCount, fleet.actionCount);
    std::printf("        超时订单数: %d\n", lateTotal);
    return fleet;
}

// 数据质量不变量。
//
// 起因（必须记住的教训）：RoutePlanner 完成后单元测试全绿，但默认数据里藏着两个
// 真实缺陷 —— ① 巡游 1048km / 28 小时，全部订单必然超时；② 成本与距离完全线性相关，
// 最低成本策略恒等价于最短距离策略、B7"两种策略"名存实亡。
// 当时集成测试只有"非零 / 非负 / 结构正确"这类弱断言，两个缺陷是靠人眼看 printf
// 发现的 —— 也就是说同样的缺陷今天重来一遍，测试依然会全绿通过。
// 以下断言把"数据有效性"从"人眼观察"变成"机器守的契约"。
// 数据质量不变量（规格一 §6.5 升级后的①–④）。
//
// 起因（必须记住的教训）：RoutePlanner 完成后单元测试全绿，但默认数据里藏着两个
// 真实缺陷 —— ① 巡游 1048km / 28 小时；② 成本与距离完全线性相关。当时集成测试
// 只有"非零 / 非负 / 结构正确"这类弱断言，两个缺陷是靠人眼看 printf 发现的。
// 以下断言把"数据有效性"从"人眼观察"变成"机器守的契约"。

// 转运默认不启用（规格二 §5.13 / #41）：默认数据 25 单全部簇内、每簇一批不超容量、
// 无紧急单 ⇒ 三条判据**全部不触发**，转运层一行候选都不构造。
// 这是「绝不更差」的最强经验证据：默认三策略的车队口径必须与 #40 基线逐项一致。
void checkDefaultTransitNotTriggered(const Config& cfg, const FleetPlan& byDistance,
                                     const FleetPlan& byCost, const FleetPlan& byTime) {
    const FleetPlan* fleets[3] = {&byDistance, &byCost, &byTime};
    const char* labels[3] = {"距离策略", "成本策略", "时间策略"};
    for (int i = 0; i < 3; ++i) {
        check(fleets[i]->transitLegs.empty(),
              std::string(labels[i]) + "：默认数据上转运账本必须为空（三判据静态不触发）");
        check(fleets[i]->transitLog.empty(),
              std::string(labels[i]) + "：默认数据上转运层不产生任何候选日志（门控在工作）");
    }
    // 22 个车队级数字（3 × 距离/耗时/成本/penalty/停靠/动作）——**#40 基线**。
    // 口径：本工作区实现、config/default.ini 三车 25 单（不是规格一单车黄金值）。
    struct Base {
        const char* label;
        double distance;
        double completion;
        double cost;
        int penalty;
        int stops;
        int actions;
    };
    const Base kBase[3] = {
        {"距离", 139.900, 429.000, 172.970, 0, 28, 50},
        {"成本", 145.400, 449.500, 160.260, 0, 32, 50},
        {"时间", 170.400, 378.200, 243.090, 0, 30, 50},
    };
    for (int i = 0; i < 3; ++i) {
        const FleetPlan& f = *fleets[i];
        const Base& b = kBase[i];
        check(fixtures::nearlyEqual(f.totalDistanceKm, b.distance, 0.001),
              std::string(b.label) + "策略·默认数据总距离与 #40 基线一致（"
                  + std::to_string(f.totalDistanceKm) + " vs " + std::to_string(b.distance) + "）");
        check(fixtures::nearlyEqual(f.completionTimeMin, b.completion, 0.001),
              std::string(b.label) + "策略·完工耗时与 #40 基线一致");
        check(fixtures::nearlyEqual(f.totalCostYuan, b.cost, 0.001),
              std::string(b.label) + "策略·总成本与 #40 基线一致");
        check(f.totalPenaltyMin == b.penalty, std::string(b.label) + "策略·penalty 与 #40 基线一致");
        check(f.stopCount == b.stops, std::string(b.label) + "策略·停靠数与 #40 基线一致");
        check(f.actionCount == b.actions, std::string(b.label) + "策略·动作数与 #40 基线一致");
    }
    (void)cfg;
}

void checkDataQualityInvariants(const Config& cfg, const FleetPlan& byDistance,
                                const FleetPlan& byCost, const FleetPlan& byTime) {
    const FleetPlan* fleets[] = {&byDistance, &byCost, &byTime};
    const char* labels[] = {"距离策略", "成本策略", "时间策略"};

    // ① 三策略**两两**分化（3 对）：逐车路线序列或自身指标必须不同。
    const double own[3] = {byDistance.totalDistanceKm, byCost.totalCostYuan,
                           byTime.completionTimeMin};
    for (int i = 0; i < 3; ++i) {
        for (int j = i + 1; j < 3; ++j) {
            bool differ = !fixtures::nearlyEqual(own[i], own[j], 0.001);
            for (std::size_t v = 0;
                 v < fleets[i]->vehicles.size() && v < fleets[j]->vehicles.size(); ++v) {
                if (fleets[i]->vehicles[v].nodes != fleets[j]->vehicles[v].nodes) {
                    differ = true;
                }
            }
            check(differ, std::string(labels[i]) + " 与 " + labels[j]
                              + " 必须给出不同结果（三策略两两分化）");
        }
    }

    // ② 每条策略在**自身目标**上不劣于另两条（3 × 2 = 6 条）。车队口径：
    //    距离/成本是 Σ，耗时是**完工耗时**（主口径，§3.4）。
    check(byDistance.totalDistanceKm <= byCost.totalDistanceKm + 1e-6,
          "距离策略的总距离不应劣于成本策略（" + std::to_string(byDistance.totalDistanceKm)
              + " vs " + std::to_string(byCost.totalDistanceKm) + "）");
    check(byDistance.totalDistanceKm <= byTime.totalDistanceKm + 1e-6,
          "距离策略的总距离不应劣于时间策略（" + std::to_string(byDistance.totalDistanceKm)
              + " vs " + std::to_string(byTime.totalDistanceKm) + "）");
    check(byCost.totalCostYuan <= byDistance.totalCostYuan + 1e-6,
          "成本策略的总成本不应劣于距离策略（" + std::to_string(byCost.totalCostYuan)
              + " vs " + std::to_string(byDistance.totalCostYuan) + "）");
    check(byCost.totalCostYuan <= byTime.totalCostYuan + 1e-6,
          "成本策略的总成本不应劣于时间策略（" + std::to_string(byCost.totalCostYuan)
              + " vs " + std::to_string(byTime.totalCostYuan) + "）");
    check(byTime.completionTimeMin <= byDistance.completionTimeMin + 1e-6,
          "时间策略的完工耗时不应劣于距离策略（" + std::to_string(byTime.completionTimeMin)
              + " vs " + std::to_string(byDistance.completionTimeMin) + "）");
    check(byTime.completionTimeMin <= byCost.completionTimeMin + 1e-6,
          "时间策略的完工耗时不应劣于成本策略（" + std::to_string(byTime.completionTimeMin)
              + " vs " + std::to_string(byCost.completionTimeMin) + "）");

    // ③ 完工耗时 ≤ 12h **且逐车耗时 ≤ 12h**（规格二 §8.2 ③）。
    const double kWorkdayMinutes = 12.0 * 60.0;
    for (int i = 0; i < 3; ++i) {
        check(fleets[i]->completionTimeMin <= kWorkdayMinutes,
              std::string(labels[i]) + " 完工耗时须在 12 小时工作日内，实际 "
                  + std::to_string(fleets[i]->completionTimeMin));
        for (std::size_t v = 0; v < fleets[i]->vehicles.size(); ++v) {
            check(fleets[i]->vehicles[v].totalTimeMin <= kWorkdayMinutes,
                  std::string(labels[i]) + " 车 " + cfg.vehicles[v].id
                      + " 耗时须在 12 小时工作日内，实际 "
                      + std::to_string(fleets[i]->vehicles[v].totalTimeMin));
        }
    }

    // ④ 迟到必须只出现在"刻意收紧"的窗口上，且**逐单**判。阈值是 characterization。
    const std::size_t kMaxLateOrders = 8;
    const int kTightWindowMinutes = 4 * 60;
    for (int i = 0; i < 3; ++i) {
        std::size_t lateOrders = 0;
        for (const logistics::OrderMetric& om : fleets[i]->metrics.orders) {
            if (!om.late) {
                continue;
            }
            ++lateOrders;
            const logistics::Order* o = findOrder(cfg, om.orderId);
            const int span = (o == nullptr) ? 0 : orderWindowSpanMinutes(*o);
            check(span <= kTightWindowMinutes,
                  std::string(labels[i]) + "：窗口跨度充裕的订单不应超时，" + om.orderId
                      + " 跨度 " + std::to_string(span) + " 分钟");
        }
        check(lateOrders <= kMaxLateOrders,
              std::string(labels[i]) + "：迟到订单应属少数，实际 "
                  + std::to_string(lateOrders) + " / " + std::to_string(cfg.orders.size()));
    }

    // ⑤ **102 个常量 + 每车路线序列**（规格二 §7.1–§7.3）：**单一真值表**
    // tests/plan_golden.h。车队级 13×3 + 每车 7×3×3 = 102 个数值，**全部**逐项比对；
    // 第 7–9 行的节点序列逐字比对（G17）。三车是冻结口径：车辆数变了就必须重捕，
    // 所以先把「车辆数 == 真值表的车辆数」作为前置断言（防止静默跳过 63 项）。
    check(cfg.vehicles.size() == static_cast<std::size_t>(plan_golden::kVehicleCount),
          std::string("默认数据车辆数 == 真值表冻结车辆数（实际 ")
              + std::to_string(cfg.vehicles.size()) + " vs "
              + std::to_string(plan_golden::kVehicleCount) + "）");
    for (int i = 0; i < 3; ++i) {
        const logistics::RouteMetrics& m = fleets[i]->metrics;
        const plan_golden::StrategyGolden& g = plan_golden::kAll[i];
        const std::string tag = std::string(labels[i]) + "·黄金[" + g.name + "]：";

        // 车队级 13 字段（0.001 / 0.1 / 整数）。
        check(fixtures::nearlyEqual(fleets[i]->totalDistanceKm, g.distanceKm,
                                    plan_golden::kTolMetric),
              tag + "总距离 " + std::to_string(fleets[i]->totalDistanceKm) + " vs "
                  + std::to_string(g.distanceKm));
        check(fixtures::nearlyEqual(fleets[i]->completionTimeMin, g.timeMin,
                                    plan_golden::kTolMetric),
              tag + "总耗时 " + std::to_string(fleets[i]->completionTimeMin) + " vs "
                  + std::to_string(g.timeMin));
        check(fixtures::nearlyEqual(fleets[i]->totalCostYuan, g.costYuan,
                                    plan_golden::kTolMetric),
              tag + "总成本 " + std::to_string(fleets[i]->totalCostYuan) + " vs "
                  + std::to_string(g.costYuan));
        check(fleets[i]->totalPenaltyMin == g.penaltyMin, tag + "总 penalty");
        check(m.stopCount == g.stopCount, tag + "停靠点数 S");
        check(m.actionCount == g.actionCount, tag + "动作数 A");
        const double pickupRate = 100.0 * static_cast<double>(m.pickupOnTimeCount)
                                  / static_cast<double>(m.pickupTotal);
        const double deliveryRate = 100.0 * static_cast<double>(m.deliveryOnTimeCount)
                                    / static_cast<double>(m.deliveryTotal);
        const double normalRate = 100.0 * static_cast<double>(m.normalOnTimeCount)
                                  / static_cast<double>(m.normalTotal);
        check(fixtures::nearlyEqual(pickupRate, g.pickupRatePct, plan_golden::kTolRate),
              tag + "取货准时率");
        check(fixtures::nearlyEqual(deliveryRate, g.deliveryRatePct, plan_golden::kTolRate),
              tag + "送达准时率·总");
        check(fixtures::nearlyEqual(normalRate, g.normalRatePct, plan_golden::kTolRate),
              tag + "送达准时率·普通");
        // 第 9 项：紧急送达准时率。真值表用 -1 表示「分母 0 记 —」。
        if (g.urgentRatePct == plan_golden::kNoDenominatorSentinel) {
            check(m.urgentTotal == 0, tag + "无紧急单时紧急准时率分母为 0（记「—」）");
        } else {
            check(m.urgentTotal > 0
                      && fixtures::nearlyEqual(
                             100.0 * static_cast<double>(m.urgentOnTimeCount)
                                 / static_cast<double>(m.urgentTotal),
                             g.urgentRatePct, plan_golden::kTolRate),
                  tag + "送达准时率·紧急");
        }
        check(fixtures::nearlyEqual(m.avgLoadKg, g.avgLoadKg, plan_golden::kTolRate),
              tag + "平均在车货量");
        check(fixtures::nearlyEqual(m.loadUtilization * 100.0, g.utilizationPct,
                                    plan_golden::kTolRate),
              tag + "载重利用率");
        check(fixtures::nearlyEqual(m.peakLoadKg, g.peakLoadKg, plan_golden::kTolRate),
              tag + "峰值在车量");

        // 每车 7 字段 × 3 车（规格二 §7.2）：这一层是 ADR 0008 的核心——
        // V02↔V03 整条路线互换时，车队级 13 项**一个都不变**，只有这里会红。
        for (std::size_t v = 0; v < cfg.vehicles.size()
                                && v < static_cast<std::size_t>(plan_golden::kVehicleCount);
             ++v) {
            const logistics::RoutePlan& plan = fleets[i]->vehicles[v];
            const plan_golden::VehicleGolden& vg = g.vehicles[v];
            const std::string vtag = tag + "车 " + vg.id + "：";
            check(cfg.vehicles[v].id == vg.id, vtag + "车辆行序与真值表一致");
            check(fixtures::nearlyEqual(plan.totalDistanceKm, vg.distanceKm,
                                        plan_golden::kTolMetric),
                  vtag + "距离 " + std::to_string(plan.totalDistanceKm) + " vs "
                      + std::to_string(vg.distanceKm));
            check(fixtures::nearlyEqual(plan.totalTimeMin, vg.timeMin, plan_golden::kTolMetric),
                  vtag + "耗时 " + std::to_string(plan.totalTimeMin) + " vs "
                      + std::to_string(vg.timeMin));
            check(fixtures::nearlyEqual(plan.totalCostYuan, vg.costYuan,
                                        plan_golden::kTolMetric),
                  vtag + "成本 " + std::to_string(plan.totalCostYuan) + " vs "
                      + std::to_string(vg.costYuan));
            check(plan.totalPenaltyMin == vg.penaltyMin, vtag + "penalty");
            check(fixtures::nearlyEqual(plan.metrics.loadUtilization * 100.0,
                                        vg.utilizationPct, plan_golden::kTolRate),
                  vtag + "载重利用率（÷ 该车 capacity）");
            const auto perVehicleRate = [](int hit, int total, double& out) -> bool {
                if (total <= 0) {
                    return false;
                }
                out = 100.0 * static_cast<double>(hit) / static_cast<double>(total);
                return true;
            };
            double pRate = 0.0;
            double dRate = 0.0;
            const bool pHas = perVehicleRate(plan.metrics.pickupOnTimeCount,
                                             plan.metrics.pickupTotal, pRate);
            const bool dHas = perVehicleRate(plan.metrics.deliveryOnTimeCount,
                                             plan.metrics.deliveryTotal, dRate);
            check(pHas, vtag + "取货率分母 = 该车订单数（> 0）");
            check(dHas, vtag + "送达率分母 = 该车订单数（> 0）");
            if (pHas) {
                check(fixtures::nearlyEqual(pRate, vg.pickupRatePct, plan_golden::kTolRate),
                      vtag + "取货准时率");
            }
            if (dHas) {
                check(fixtures::nearlyEqual(dRate, vg.deliveryRatePct,
                                            plan_golden::kTolRate),
                      vtag + "送达准时率");
            }

            // G17：每车路线序列**逐字**（空车只有起点，与 §7.4 的路线行一致）。
            std::string route;
            if (plan.nodes.empty()) {
                route = cfg.vehicles[v].startNodeId;
            } else {
                for (std::size_t n = 0; n < plan.nodes.size(); ++n) {
                    if (n > 0) {
                        route += " ";
                    }
                    route += plan.nodes[n];
                }
            }
            check(route == g.routeNodes[v],
                  vtag + "路线序列逐字一致（G17）\n        实际: " + route
                      + "\n        真值: " + g.routeNodes[v]);
            std::printf("      %s 车 %s 路线（G17）: %s\n", labels[i], vg.id,
                        route.c_str());
        }

        // G7：总 penalty == 逐单 penalty 之和。
        int penaltySum = 0;
        for (const logistics::OrderMetric& om : m.orders) {
            penaltySum += om.penaltyMin;
        }
        check(fleets[i]->totalPenaltyMin == penaltySum, tag + "总 penalty = 逐单之和（G7）");
        check(m.deliveryTotal == static_cast<int>(cfg.orders.size())
                  && m.pickupTotal == static_cast<int>(cfg.orders.size()),
              tag + "准时率分母 = 已规划订单数");
    }
}

// 规格二 §4.9 的 9 条分配不变量（默认数据，三策略 + 顺序轮转基线）。
// 断言读的都是**规划产物**（分配表 / 逐车计划 / 改派日志 / 候选键留痕），
// 不自己重算分配；基线的独立对照由 roundRobinAssignment（测试自算）给出。
void checkAllocationInvariants(const Config& cfg, const FleetPlan* fleets[3],
                               const char* labels[3], const FleetPlan& baseDistance,
                               const FleetPlan& baseCost, const FleetPlan& baseTime) {
    // ---- ① 全部订单恰好服务一次、每张只出现在**一台**车的路线上 ----
    for (int f = 0; f < 3; ++f) {
        const FleetPlan& fleet = *fleets[f];
        const std::string tag = std::string(labels[f]) + "·分配①：";
        check(fleet.allocationOrderIds.size() == cfg.orders.size(),
              tag + "分配表覆盖全部订单（" + std::to_string(fleet.allocationOrderIds.size())
                  + " / " + std::to_string(cfg.orders.size()) + "）");
        for (std::size_t i = 0; i < cfg.orders.size(); ++i) {
            const logistics::Order& o = cfg.orders[i];
            int owners = 0;
            std::size_t ownerVehicle = cfg.vehicles.size();
            for (std::size_t v = 0; v < fleet.vehicles.size(); ++v) {
                for (std::size_t s = 0; s < fleet.vehicles[v].stops.size(); ++s) {
                    const std::vector<logistics::StopDelivery>& ds =
                        fleet.vehicles[v].stops[s].deliveries;
                    for (std::size_t d = 0; d < ds.size(); ++d) {
                        if (ds[d].orderId == o.id) {
                            ++owners;
                            ownerVehicle = v;
                        }
                    }
                }
            }
            check(owners == 1, tag + "订单 " + o.id + " 恰好由一台车送达（实际 "
                                   + std::to_string(owners) + " 台）");
            // 分配表必须与路线一致（分配说归谁，谁的路线上就得真有这单）。
            bool listed = false;
            std::size_t listedVehicle = cfg.vehicles.size();
            for (std::size_t k = 0; k < fleet.allocationOrderIds.size(); ++k) {
                if (fleet.allocationOrderIds[k] == o.id) {
                    listed = true;
                    listedVehicle = fleet.allocationVehicleIndex[k];
                }
            }
            check(listed, tag + "订单 " + o.id + " 出现在分配表里");
            if (owners == 1 && listed) {
                check(listedVehicle == ownerVehicle,
                      tag + "订单 " + o.id + " 的分配与路线一致");
            }
        }
    }

    // ---- ② 簇归属完备且互斥：本地单恰属一个簇、跨簇单不属任何簇、两者并 = 全部订单 ----
    int localCount = 0;
    int crossCount = 0;
    for (std::size_t i = 0; i < cfg.orders.size(); ++i) {
        const logistics::Order& o = cfg.orders[i];
        const int cluster = clusterOfOrder(cfg.graph, o);
        const bool cross = isCrossCluster(cfg.graph, o) || cluster <= 0;
        int matched = 0;
        for (int f = 0; f < 3; ++f) {
            const FleetPlan& fleet = *fleets[f];
            std::size_t at = fleet.allocationOrderIds.size();
            for (std::size_t k = 0; k < fleet.allocationOrderIds.size(); ++k) {
                if (fleet.allocationOrderIds[k] == o.id) {
                    at = k;
                }
            }
            const bool ok = (at < fleet.allocationOrderIds.size())
                            && (cross ? fleet.allocationClusterId[at] == -1
                                      : fleet.allocationClusterId[at] == cluster);
            check(ok, std::string(labels[f]) + "·分配②：订单 " + o.id
                          + (cross ? " 是跨簇单 ⇒ 不属任何簇（-1）"
                                   : " 恰属簇 " + std::to_string(cluster)));
            if (ok) {
                ++matched;
            }
        }
        if (matched == 3) {
            if (cross) {
                ++crossCount;
            } else {
                ++localCount;
            }
        }
    }
    check(localCount + crossCount == static_cast<int>(cfg.orders.size()),
          "分配②：本地单 + 跨簇单 = 全部订单（完备且互斥）");
    std::printf("      分配②：本地单 %d 张、跨簇单 %d 张（默认数据应为 0 跨簇）\n", localCount,
                crossCount);
    check(crossCount == 0, "分配②：默认数据 0 条跨簇单（§4.2 数据事实）");

    // ---- ③ 前缀峰值 0 ≤ L ≤ **该车**容量；并实证「总分配 kg ≤ 容量」是**非法**判据 ----
    for (int f = 0; f < 3; ++f) {
        const FleetPlan& fleet = *fleets[f];
        const std::string tag = std::string(labels[f]) + "·分配③：";
        for (std::size_t v = 0; v < cfg.vehicles.size() && v < fleet.vehicles.size(); ++v) {
            const double peak = fleet.vehicles[v].metrics.peakLoadKg;
            check(peak <= cfg.vehicles[v].capacityKg + 1e-6,
                  tag + cfg.vehicles[v].id + " 前缀峰值 ≤ 该车容量（"
                      + std::to_string(peak) + " / "
                      + std::to_string(cfg.vehicles[v].capacityKg) + "）");
        }
    }
    // 反例（§4.3 / Q5 实证）：顺序轮转基线上 V02/V03 各分到 165kg > 160kg 容量，
    // 但峰值只有 64 / 57kg，**完全可行**。任何「总 kg ≤ 容量」的判定都会误报违规。
    {
        const std::vector<std::vector<logistics::Order> > assigned =
            allocationOrderSets(cfg, baseDistance);
        int overTotal = 0;
        for (std::size_t v = 0; v < cfg.vehicles.size() && v < assigned.size(); ++v) {
            double totalKg = 0.0;
            for (std::size_t i = 0; i < assigned[v].size(); ++i) {
                totalKg += assigned[v][i].demandKg;
            }
            const double peak = baseDistance.vehicles[v].metrics.peakLoadKg;
            const double cap = cfg.vehicles[v].capacityKg;
            std::printf("      分配③（基线）%s：总分配 %.0fkg / 容量 %.0fkg，峰值 %.0fkg\n",
                        cfg.vehicles[v].id.c_str(), totalKg, cap, peak);
            if (totalKg > cap + 1e-6) {
                ++overTotal;
                check(peak <= cap + 1e-6,
                      std::string("分配③：") + cfg.vehicles[v].id
                          + " 总分配 kg > 容量（" + std::to_string(totalKg) + " > "
                          + std::to_string(cap) + "）但前缀峰值合规（"
                          + std::to_string(peak) + "）⇒ 总 kg 判据会误报");
            }
        }
        check(overTotal >= 2,
              "分配③：顺序轮转基线至少有 2 台车「总分配 kg > 该车容量」而峰值合规"
              "（§4.3 实证；禁止用总 kg 判可行性）");
    }

    // ---- ④ 每车从自己的 start_node / depart_time 空载出发、回同一基地；空车五项 0 ----
    for (int f = 0; f < 3; ++f) {
        const FleetPlan& fleet = *fleets[f];
        const std::string tag = std::string(labels[f]) + "·分配④：";
        for (std::size_t v = 0; v < cfg.vehicles.size() && v < fleet.vehicles.size(); ++v) {
            const logistics::RoutePlan& plan = fleet.vehicles[v];
            if (plan.nodes.empty()) {
                check(false, tag + cfg.vehicles[v].id + " 路线非空");
                continue;
            }
            check(plan.nodes.front() == cfg.vehicles[v].startNodeId,
                  tag + cfg.vehicles[v].id + " 从自己的 start_node 出发");
            check(plan.nodes.back() == cfg.vehicles[v].startNodeId,
                  tag + cfg.vehicles[v].id + " 回到同一基地");
            check(!plan.nodeArrivalMin.empty()
                      && plan.nodeArrivalMin.front() == cfg.vehicles[v].departTimeMin,
                  tag + cfg.vehicles[v].id + " 按自己的 depart_time 出发");
            check(plan.metrics.actionCount == 2 * fleet.vehicleOrderCount[v],
                  tag + cfg.vehicles[v].id + " 动作数 = 2 × 分配单数（空载出发、每单取送各一次）");
            if (fleet.vehicleOrderCount[v] == 0) {
                check(fixtures::nearlyEqual(plan.totalDistanceKm, 0.0)
                          && fixtures::nearlyEqual(plan.totalCostYuan, 0.0)
                          && plan.totalPenaltyMin == 0 && plan.metrics.stopCount == 0
                          && plan.metrics.actionCount == 0,
                      tag + cfg.vehicles[v].id + " 空车五项全 0");
                check(plan.returnArrivalMin == cfg.vehicles[v].departTimeMin,
                      tag + cfg.vehicles[v].id + " 空车返回时刻 = 发车时刻");
            }
        }
    }

    // ---- ⑤ 确定性：同输入同输出；tieKey 三层（车辆 ID 升序 → 停靠序列 → 枚举序）----
    {
        const FleetPlan again = planFleet(cfg.graph, cfg.vehicles, cfg.orders, WeightType::Distance);
        check(again.allocationOrderIds == fleets[0]->allocationOrderIds
                  && again.allocationVehicleIndex == fleets[0]->allocationVehicleIndex,
              "分配⑤：同输入同输出（分配表逐项相同）");
        bool sameNodes = again.vehicles.size() == fleets[0]->vehicles.size();
        for (std::size_t v = 0; sameNodes && v < again.vehicles.size(); ++v) {
            sameNodes = again.vehicles[v].nodes == fleets[0]->vehicles[v].nodes;
        }
        check(sameNodes, "分配⑤：同输入同输出（逐车节点序列逐字相同）");
        std::vector<logistics::Order> shuffled;
        for (std::size_t i = 0; i < cfg.orders.size(); ++i) {
            shuffled.push_back(cfg.orders[(i * 7 + 3) % cfg.orders.size()]);
        }
        const FleetPlan byShuffle =
            planFleet(cfg.graph, cfg.vehicles, shuffled, WeightType::Distance);
        check(byShuffle.allocationVehicleIndex == fleets[0]->allocationVehicleIndex,
              "分配⑤：打乱订单传入顺序不改变分配（ID/簇/车辆三层排序在枚举之前）");
        std::printf("      分配⑤：候选键留痕 %zu 行（默认 3 簇 3 车应为 6 行）\n",
                    fleets[0]->candidateKeyLog.size());
        check(fleets[0]->candidateKeyLog.size() == 6,
              "分配⑤：枚举 6 种「簇 -> 车」排列（默认 3 簇 3 车）");
    }

    // ---- ⑧ 第三阶段守卫：逐条读改派日志核对「完工耗时不变差」+「键严格变优」 ----
    {
        int records = 0;
        for (int f = 0; f < 3; ++f) {
            const FleetPlan& fleet = *fleets[f];
            for (std::size_t i = 0; i < fleet.relocations.size(); ++i) {
                const RelocateRecord& rec = fleet.relocations[i];
                ++records;
                check(rec.completionAfterMin <= rec.completionBeforeMin + 1e-9,
                      std::string(labels[f]) + "·分配⑧：改派 " + rec.orderId + " "
                          + rec.fromVehicleId + "→" + rec.toVehicleId + " 完工耗时不变差（"
                          + std::to_string(rec.completionBeforeMin) + " -> "
                          + std::to_string(rec.completionAfterMin) + "）");
                const bool better = rec.penaltyAfter < rec.penaltyBefore
                                    || (rec.penaltyAfter == rec.penaltyBefore
                                        && (rec.weightAfter < rec.weightBefore - 1e-9
                                            || (fixtures::nearlyEqual(rec.weightAfter,
                                                                     rec.weightBefore)
                                                && rec.balanceAfter < rec.balanceBefore - 1e-9)));
                check(better, std::string(labels[f]) + "·分配⑧：改派 " + rec.orderId
                                  + " 的车队键严格变优");
                std::printf("      分配⑧：%s 轮 %d %s %s→%s 完工 %f→%f 键 (%d,%f,%f)→(%d,%f,%f)\n",
                            labels[f], rec.round, rec.orderId.c_str(),
                            rec.fromVehicleId.c_str(), rec.toVehicleId.c_str(),
                            rec.completionBeforeMin, rec.completionAfterMin, rec.penaltyBefore,
                            rec.weightBefore, rec.balanceBefore, rec.penaltyAfter,
                            rec.weightAfter, rec.balanceAfter);
            }
        }
        // **守卫必须真的测到了东西**：默认数据至少有一条被接受的改派。
        check(records >= 1, "分配⑧：改派日志非空（守卫在这份数据上真的被消费过）");
    }

    // ---- ⑨ 基线确定性：分配只由「订单 ID 升序 + 车队行序」决定，与策略/容量/窗口无关 ----
    {
        check(baseDistance.allocationOrderIds == baseCost.allocationOrderIds
                  && baseDistance.allocationVehicleIndex == baseCost.allocationVehicleIndex,
              "分配⑨：基线换策略（距离 -> 成本）不改变分配");
        check(baseDistance.allocationOrderIds == baseTime.allocationOrderIds
                  && baseDistance.allocationVehicleIndex == baseTime.allocationVehicleIndex,
              "分配⑨：基线换策略（距离 -> 时间）不改变分配");
        // 与测试自算的顺序轮转映射逐单核对（读被测实现的分配 vs 独立算的分配）。
        const std::vector<std::vector<logistics::Order> > expected = roundRobinAssignment(cfg);
        for (std::size_t v = 0; v < expected.size(); ++v) {
            for (std::size_t i = 0; i < expected[v].size(); ++i) {
                std::size_t at = baseDistance.allocationOrderIds.size();
                for (std::size_t k = 0; k < baseDistance.allocationOrderIds.size(); ++k) {
                    if (baseDistance.allocationOrderIds[k] == expected[v][i].id) {
                        at = k;
                    }
                }
                check(at < baseDistance.allocationOrderIds.size()
                          && baseDistance.allocationVehicleIndex[at] == v,
                      "分配⑨：基线把 " + expected[v][i].id + " 分给第 " + std::to_string(v)
                          + " 台车（ID 升序轮转）");
            }
        }
        // 改容量 / 改窗口：分配不变。
        Config widened = cfg;
        for (std::size_t i = 0; i < widened.orders.size(); ++i) {
            widened.orders[i].windowStartMin = 0;
            widened.orders[i].windowEndMin = 1440;
        }
        widened.vehicles[0].capacityKg = cfg.vehicles[0].capacityKg + 100.0;
        const FleetPlan byVariant = planFleetSequentialBaseline(
            widened.graph, widened.vehicles, widened.orders, WeightType::Distance);
        check(byVariant.allocationVehicleIndex == baseDistance.allocationVehicleIndex,
              "分配⑨：改容量 / 改窗口不改变基线分配");
        // 三阶段分配与基线**必须不同**（否则 #40 没生效）。
        check(fleets[0]->allocationVehicleIndex != baseDistance.allocationVehicleIndex,
              "分配⑨：三阶段分配与顺序轮转基线不同（#40 生效）");
    }

    // 逐策略摘要打印（人工核对用）。
    for (int f = 0; f < 3; ++f) {
        const FleetPlan& fleet = *fleets[f];
        std::string alloc;
        for (std::size_t i = 0; i < fleet.allocationOrderIds.size(); ++i) {
            const std::size_t v = fleet.allocationVehicleIndex[i];
            if (!alloc.empty()) {
                alloc += ",";
            }
            alloc += fleet.allocationOrderIds[i] + "="
                     + (v < cfg.vehicles.size() ? cfg.vehicles[v].id : std::string("-"));
        }
        std::printf("      %s ALLOC %s\n", labels[f], alloc.c_str());
        std::printf("      %s 均衡度（峰值利用率极差）= %.3f；改派 %zu 次\n", labels[f],
                    fleet.balanceSpread, fleet.relocations.size());
    }
    std::printf("      基线 ALLOC（顺序轮转）逐车单数：");
    for (std::size_t v = 0; v < baseDistance.vehicleOrderCount.size(); ++v) {
        std::printf(" %s=%d", cfg.vehicles[v].id.c_str(), baseDistance.vehicleOrderCount[v]);
    }
    std::printf("\n");
}

// 载重远小于总需求时：**仍是单条连续路线**（边取边送 + 前缀载重），
// 不再分趟、也不得中途回仓库装货（规格一 MC-A01/A02 退役）。
void checkSmallCapacityStaysSingleRoute(const Config& cfg) {
    Vehicle small = cfg.vehicles[0];
    small.capacityKg = 60.0;   // 远小于总需求 516kg，但大于单张订单最大 24kg（否则是载重不可行）

    const RoutePlan plan = planRoute(cfg.graph, small, cfg.orders, WeightType::Distance);
    check(plan.status == PlanStatus::Ok,
          "载重 60kg（< 总需求 516kg）时仍是可行的单条连续路线: " + plan.reason);
    if (plan.status != PlanStatus::Ok) {
        return;
    }
    check(plan.nodes.front() == small.startNodeId && plan.nodes.back() == small.startNodeId,
          "小载重路线从仓库出发并回到仓库");
    bool walkable = true;
    for (std::size_t i = 1; i < plan.nodes.size(); ++i) {
        if (plan.nodes[i] == plan.nodes[i - 1]) {
            continue;
        }
        if (cfg.graph.findEdge(plan.nodes[i - 1], plan.nodes[i]) == nullptr) {
            walkable = false;
        }
    }
    check(walkable, "小载重路线的相邻节点之间都存在有向边");
    // 前缀载重：逐停靠按动作集复核，全程 ∈ [0, 20]。
    double load = 0.0;
    bool prefixOk = true;
    for (const logistics::Stop& st : plan.stops) {
        double after = load;
        for (const logistics::StopDelivery& d : st.deliveries) {
            const logistics::Order* o = findOrder(cfg, d.orderId);
            if (o != nullptr) {
                after -= o->demandKg;
            }
        }
        for (const std::string& id : st.offloads) {
            const logistics::Order* o = findOrder(cfg, id);
            if (o != nullptr) {
                after -= o->demandKg;
            }
        }
        if (after < -1e-9) {
            prefixOk = false;
        }
        for (const std::string& id : st.pickups) {
            const logistics::Order* o = findOrder(cfg, id);
            if (o != nullptr) {
                after += o->demandKg;
            }
        }
        for (const std::string& id : st.reclaims) {
            const logistics::Order* o = findOrder(cfg, id);
            if (o != nullptr) {
                after += o->demandKg;
            }
        }
        if (after > small.capacityKg + 1e-6 || after < -1e-9) {
            prefixOk = false;
        }
        load = after;
    }
    check(prefixOk, "小载重路线的前缀载重全程在 [0, 载重上限]");
    // 每张订单仍恰好服务一次（不得因为载重小而丢单）。
    std::size_t served = 0;
    for (const logistics::Order& o : cfg.orders) {
        std::size_t hits = 0;
        for (const logistics::Stop& st : plan.stops) {
            for (const logistics::StopDelivery& d : st.deliveries) {
                if (d.orderId == o.id) {
                    ++hits;
                }
            }
        }
        if (hits == 1) {
            ++served;
        }
    }
    check(served == cfg.orders.size(),
          "小载重下全部订单仍恰好服务一次，实际 " + std::to_string(served) + " / "
              + std::to_string(cfg.orders.size()));
    std::printf("    小载重（60kg）单条路线：%d 站，总距离 %.1fkm，penalty %dmin\n",
                plan.metrics.stopCount, plan.totalDistanceKm, plan.totalPenaltyMin);
}

void checkGraphRepresentationsOnRealData(const Config& cfg) {
    const std::string list = cfg.graph.toAdjacencyListString();
    check(list.find("W01") != std::string::npos, "邻接表包含顶点 W01");
    check(list.find("->") != std::string::npos, "邻接表包含出边");

    const std::string matrix = cfg.graph.toAdjacencyMatrixString(WeightType::Distance);
    std::size_t lines = 0;
    std::size_t tokensPerLine = 0;
    std::size_t mismatched = 0;
    std::size_t i = 0;
    while (i < matrix.size()) {
        const std::size_t nl = matrix.find('\n', i);
        const std::string line =
            (nl == std::string::npos) ? matrix.substr(i) : matrix.substr(i, nl - i);
        std::size_t tokens = 0;
        bool inToken = false;
        for (char c : line) {
            const bool space = (c == ' ' || c == '\t' || c == '\r');
            if (!space && !inToken) {
                ++tokens;
                inToken = true;
            } else if (space) {
                inToken = false;
            }
        }
        ++lines;
        if (lines == 1) {
            tokensPerLine = tokens;
        } else if (tokens != tokensPerLine) {
            ++mismatched;
        }
        if (nl == std::string::npos) {
            break;
        }
        i = nl + 1;
    }

    const std::size_t expected = cfg.graph.nodeCount() + 1;   // 角标 + 各列
    check(lines == expected, "邻接矩阵行数 = 节点数 + 1（" + std::to_string(expected)
              + "），实际 " + std::to_string(lines));
    check(mismatched == 0, "邻接矩阵各行 token 数一致");
    check(tokensPerLine == expected, "邻接矩阵每行 token 数 = 节点数 + 1（"
              + std::to_string(expected) + "），实际 " + std::to_string(tokensPerLine));
    check(matrix.find("-") != std::string::npos, "邻接矩阵含缺边占位符");

    std::printf("      B3: 邻接表 %zu 行；邻接矩阵 %zux%zu\n",
                cfg.graph.nodeCount() + 1, lines, tokensPerLine);
}

// E1 + E3 在真实数据上的端到端验证：
// 服务前 k 站后触发路况变化 -> 重规划 -> 再插入紧急订单 -> 再次重算。
void checkTrafficAndUrgentOrderOnRealData(const Config& cfg) {
    LogisticsGraph g = cfg.graph;   // 副本：路况变化会改写边耗时，不能污染原配置
    const Vehicle& v = cfg.vehicles[0];
    const RoutePlan before = planRoute(g, v, cfg.orders, WeightType::Distance);
    check(before.status == PlanStatus::Ok, "拥堵前默认数据可规划");
    if (before.status != PlanStatus::Ok || before.stops.size() < 6) {
        return;
    }

    // 记录基准耗时，稍后验证路况变化不会动它
    std::vector<double> baseTimes;
    for (const logistics::Edge& e : g.edges()) {
        baseTimes.push_back(e.baseTimeMin);
    }

    const TrafficReport report = simulateTrafficChange(
        g, cfg.general.trafficChangeRatio, cfg.general.trafficTimeIncreaseMin,
        cfg.general.trafficTimeIncreaseMax, 42u);
    // oracle: round(0.1 * 106 边) = 11
    check(report.changes.size() == 11,
          "按 traffic_change_ratio=0.1 改动 11 条边（106 边），实际 "
              + std::to_string(report.changes.size()));

    std::size_t i = 0;
    bool baseIntact = true;
    for (const logistics::Edge& e : g.edges()) {
        if (e.baseTimeMin != baseTimes[i++]) {
            baseIntact = false;
        }
    }
    check(baseIntact, "全部 72 条边的 baseTimeMin 保持不变");

    // 车已服务前 5 站，停在第 5 站；剩余订单为其余配送点
    const std::size_t served = 5;
    const std::string here = before.stops[served - 1].nodeId;
    const int nowMin = before.stops[served - 1].arrivalMin;

    // 剩余订单按**动作集**判（同一节点可能被多次停靠，按节点判会误杀）。
    std::vector<logistics::Order> remaining;
    std::size_t servedOrders = 0;
    for (const logistics::Order& o : cfg.orders) {
        bool done = false;
        for (std::size_t k = 0; k < served; ++k) {
            for (const logistics::StopDelivery& d : before.stops[k].deliveries) {
                if (d.orderId == o.id) {
                    done = true;
                }
            }
        }
        if (done) {
            ++servedOrders;
        } else {
            remaining.push_back(o);
        }
    }
    check(remaining.size() + servedOrders == cfg.orders.size(),
          "剩余订单数 + 已服务订单数 = 总订单数");

    const bool trigger = needsReplan(before.nodes, report, cfg.general.trafficTimeIncreaseMin);
    std::printf("      E1: 已服务 %zu 站，位置 %s，时刻 %d；路况改动 %zu 条边；触发重规划=%s\n",
                served, here.c_str(), nowMin, report.changes.size(), trigger ? "是" : "否");

    if (trigger) {
        const RoutePlan replanned = replan(g, v, remaining, here, nowMin,
                                           WeightType::Distance);
        check(replanned.status == PlanStatus::Ok, "路况重规划后仍可行");
        // 逐单覆盖：剩余订单每张恰好送达一次（停靠数 ≠ 订单数，合并停靠会少）。
        std::size_t covered = 0;
        for (const logistics::Order& o : remaining) {
            std::size_t hits = 0;
            for (const logistics::Stop& st : replanned.stops) {
                for (const logistics::StopDelivery& d : st.deliveries) {
                    if (d.orderId == o.id) {
                        ++hits;
                    }
                }
            }
            if (hits == 1) {
                ++covered;
            }
        }
        check(covered == remaining.size(), "重规划恰好覆盖全部剩余订单（逐单）");
        check(replanned.nodes.front() == here, "重规划从当前位置出发");
        std::printf("          重规划后：距离 %.1fkm  耗时 %.1fmin  penalty %dmin\n",
                    replanned.totalDistanceKm, replanned.totalTimeMin,
                    replanned.totalPenaltyMin);
    }

    // E3：插入一张紧急 A→B 取送单。窗口按 #24 R4 取 [事件刻, 事件刻 + 60]；
    // 超时不弃（D13），因此这里**不**断言 warning 为空，只断言：单不丢、先取后送、
    // 紧急停靠全部先于普通停靠（硬优先），且计划仍可行。
    logistics::Order urgent;
    urgent.id = "URG01";
    urgent.nodeId = remaining.front().nodeId;
    // 取送式紧急单：A 取 → B 送；A 取一个与 B 不同的客户节点。
    urgent.pickupNodeId = (urgent.nodeId == "D02") ? "D03" : "D02";
    urgent.demandKg = 5.0;
    urgent.windowStartMin = nowMin;
    urgent.windowEndMin = nowMin + 60;
    urgent.urgent = true;

    const InsertResult inserted = insertOrder(g, v, remaining, urgent, here, nowMin, WeightType::Distance);
    check(inserted.plan.status == PlanStatus::Ok, "插入紧急订单后仍可行");
    // 窗口基准 = 事件刻（而不是结算时刻）：60 分钟窗口必须原样带进计划。
    check(urgent.windowEndMin - urgent.windowStartMin == 60, "紧急单窗口 = 事件刻 + 60min");
    bool urgentPicked = false;
    bool urgentDelivered = false;
    std::size_t urgentPickIdx = inserted.plan.stops.size();
    std::size_t urgentDelivIdx = inserted.plan.stops.size();
    for (std::size_t k = 0; k < inserted.plan.stops.size(); ++k) {
        for (const std::string& id : inserted.plan.stops[k].pickups) {
            if (id == urgent.id) {
                urgentPicked = true;
                urgentPickIdx = k;
            }
        }
        for (const logistics::StopDelivery& d : inserted.plan.stops[k].deliveries) {
            if (d.orderId == urgent.id) {
                urgentDelivered = true;
                urgentDelivIdx = k;
            }
        }
    }
    check(urgentPicked && urgentDelivered, "插入的紧急订单必须进计划（取货 + 送达各一次）");
    check(urgentPickIdx < urgentDelivIdx, "紧急单先取后送");
    // 硬优先：紧急停靠全部先于普通停靠。
    bool seenNormal = false;
    bool hardOk = true;
    for (const logistics::Stop& st : inserted.plan.stops) {
        if (st.phase == logistics::StopPhase::Normal) {
            seenNormal = true;
        } else if (seenNormal) {
            hardOk = false;
        }
    }
    check(hardOk, "插入紧急单后仍保持硬优先（紧急停靠全在普通停靠之前）");
    std::printf("      E3: 插入紧急订单 %s@%s（窗口 [%d,%d]），首停靠=%s，警告=%s，距离 %.1fkm 耗时 %.1fmin\n",
                urgent.id.c_str(), urgent.nodeId.c_str(), urgent.windowStartMin,
                urgent.windowEndMin,
                inserted.plan.stops.empty() ? "(无)" : inserted.plan.stops[0].nodeId.c_str(),
                inserted.warning.empty() ? "(无)" : inserted.warning.c_str(),
                inserted.plan.totalDistanceKm, inserted.plan.totalTimeMin);
}

// 空车规则（规格二 §3.4 / §8.1-S5）：0 单的车五项全 0、返回时刻 = 发车时刻、
// **不抬升**完工耗时，但参与 min(发车时刻)。
void checkEmptyVehicleRules(const Config& cfg) {
    if (cfg.vehicles.empty()) {
        return;
    }
    // 造一台「没有分到单」的车：容量为正、但把它的订单集合清空 —— 等价于
    // 顺序轮转下「订单数 < 车数」时的空车。这里直接用 planRoute 验证单车语义，
    // 再用 planFleet 验证车队汇总不把它算进完工耗时。
    const Vehicle& base = cfg.vehicles[0];
    const RoutePlan empty = planRoute(cfg.graph, base, std::vector<logistics::Order>(),
                                      WeightType::Distance);
    check(empty.status == PlanStatus::Ok, "空车（0 单）可规划（不报不可行）");
    check(fixtures::nearlyEqual(empty.totalDistanceKm, 0.0, 1e-9)
              && fixtures::nearlyEqual(empty.totalCostYuan, 0.0, 1e-9)
              && empty.totalPenaltyMin == 0 && empty.metrics.stopCount == 0
              && empty.metrics.actionCount == 0,
          "空车五项全 0（距离/耗时/成本/penalty/停靠数）");
    check(fixtures::nearlyEqual(empty.totalTimeMin, 0.0, 1e-9),
          "空车耗时 = 返回 − 发车 = 0");
    check(empty.returnArrivalMin == base.departTimeMin,
          "空车返回时刻 = 发车时刻（不产生额外行驶）");

    // 车队层面：把一台车变成空车（订单全给第一台），完工耗时不得被它抬升。
    std::vector<Vehicle> two;
    two.push_back(cfg.vehicles[0]);
    Vehicle idle = cfg.vehicles[0];
    idle.id = "VIDLE";
    idle.departTimeMin = base.departTimeMin - 60;   // 更早发车：必须参与 min(发车)
    two.push_back(idle);
    // 只给第 0 台车订单：第 1 台（VIDLE）必然分到 0 单（顺序轮转 i mod 2，
    // 第 0 单给 0 号车；要让 1 号车空载，需要订单数 == 0 —— 故这里直接构造单车订单集
    // 交给 planFleet：订单全归 0 号车时 1 号车自然为空。
    // 只留一张单：顺序轮转下它必然归第 0 台车（i mod 2 == 0），第 1 台必然空车。
    std::vector<logistics::Order> firstOnly;
    firstOnly.push_back(cfg.orders.front());
    // 空车规则是**车队汇总**语义（与分配算法无关）：这里用顺序轮转基线，
    // 「1 张单必然归第 0 台车」才是这条用例的前提。
    const FleetPlan fleet =
        planFleetSequentialBaseline(cfg.graph, two, firstOnly, WeightType::Distance);
    check(fleet.vehicles.size() == 2, "空车车队：逐车计划数 == 2");
    if (fleet.vehicles.size() == 2) {
        check(fleet.vehicles[1].metrics.stopCount == 0,
              "第二台车确实分到 0 单（空车）");
        const double ret0 = static_cast<double>(two[0].departTimeMin)
                            + fleet.vehicles[0].totalTimeMin;
        check(fixtures::nearlyEqual(fleet.completionTimeMin,
                                    ret0 - static_cast<double>(idle.departTimeMin), 1e-6),
              "空车不抬升完工耗时，但参与 min(发车时刻)（完工 = 有货车的返回 − 空车发车）");
        check(fleet.sumVehicleTimeMin < fleet.completionTimeMin + 1e-9,
              "车时合计 >= 完工耗时（空车贡献 0，主口径仍是完工耗时）");
    }
}

// 慢车因子（规格二 §3.3 / §3.6-6）：同一节点序列下，慢车（time_factor 1.25）的
// 到达时刻 = 基础 × 1.25；**等待与 penalty 不乘因子**。
// 用最小夹具（W-D1-D2）逐项算，不依赖默认数据的路线形状。
void checkSlowVehicleFactorReplay(const Config& cfg) {
    // 夹具：W 出发，两个客户点，边权都是 10min；慢车 1.25 ⇒ 12.5min。
    logistics::LogisticsGraph g;
    logistics::Node w;
    w.id = "W";
    w.type = logistics::NodeType::Warehouse;
    w.x = 0;
    w.y = 0;
    logistics::Node a;
    a.id = "A";
    a.type = logistics::NodeType::Delivery;
    a.x = 10;
    a.y = 0;
    logistics::Node b;
    b.id = "B";
    b.type = logistics::NodeType::Delivery;
    b.x = 20;
    b.y = 0;
    g.addNode(w);
    g.addNode(a);
    g.addNode(b);
    logistics::Edge wa;
    wa.fromId = "W";
    wa.toId = "A";
    wa.distanceKm = 10.0;
    wa.timeMin = 10.0;
    wa.baseTimeMin = 10.0;
    wa.costYuan = 10.0;
    logistics::Edge aw = wa;
    aw.fromId = "A";
    aw.toId = "W";
    logistics::Edge ab;
    ab.fromId = "A";
    ab.toId = "B";
    ab.distanceKm = 10.0;
    ab.timeMin = 10.0;
    ab.baseTimeMin = 10.0;
    ab.costYuan = 10.0;
    logistics::Edge ba = ab;
    ba.fromId = "B";
    ba.toId = "A";
    g.addEdge(wa);
    g.addEdge(aw);
    g.addEdge(ab);
    g.addEdge(ba);

    logistics::Order o;
    o.id = "O1";
    o.pickupNodeId = "A";
    o.nodeId = "B";
    o.demandKg = 5.0;
    o.windowStartMin = 480;
    o.windowEndMin = 24 * 60;
    std::vector<logistics::Order> orders{o};

    Vehicle fast;
    fast.id = "VF";
    fast.startNodeId = "W";
    fast.capacityKg = 100.0;
    fast.departTimeMin = 480;
    fast.timeFactor = 1.0;
    fast.costFactor = 1.0;
    Vehicle slow = fast;
    slow.id = "VS";
    slow.timeFactor = 1.25;
    slow.costFactor = 0.80;

    const RoutePlan pf = planRoute(g, fast, orders, WeightType::Distance);
    const RoutePlan ps = planRoute(g, slow, orders, WeightType::Distance);
    check(pf.status == PlanStatus::Ok && ps.status == PlanStatus::Ok,
          "因子夹具：快车/慢车都能规划");
    if (pf.status != PlanStatus::Ok || ps.status != PlanStatus::Ok) {
        return;
    }
    // 同一节点序列（§3.3 的推导：正标量缩放不改 argmin）。
    check(pf.nodes == ps.nodes, "同一节点序列：因子不改变 leg 选择（正标量缩放）");
    check(fixtures::nearlyEqual(pf.totalDistanceKm, ps.totalDistanceKm, 1e-9),
          "距离与车型无关（两车距离相同）");
    check(fixtures::nearlyEqual(ps.totalTimeMin, pf.totalTimeMin * 1.25, 1e-6),
          "慢车到达时刻 = 基础 × 1.25（耗时 " + std::to_string(ps.totalTimeMin) + " vs "
              + std::to_string(pf.totalTimeMin) + "）");
    check(fixtures::nearlyEqual(ps.totalCostYuan, pf.totalCostYuan * 0.80, 1e-6),
          "慢车成本 = 基础 × 0.80（cost_factor）");
    // 等待不乘因子：窗口起很晚（远晚于到达）时，等待时长两车相同。
    orders[0].windowStartMin = 600;
    const RoutePlan wf = planRoute(g, fast, orders, WeightType::Distance);
    const RoutePlan ws = planRoute(g, slow, orders, WeightType::Distance);
    check(wf.status == PlanStatus::Ok && ws.status == PlanStatus::Ok,
          "等待夹具：两车都能规划");
    if (wf.status == PlanStatus::Ok && ws.status == PlanStatus::Ok) {
        int waitF = 0;
        int waitS = 0;
        for (const logistics::Stop& st : wf.stops) {
            waitF += st.waitMin;
        }
        for (const logistics::Stop& st : ws.stops) {
            waitS += st.waitMin;
        }
        check(waitF > 0, "等待夹具确实产生了早到等待（快车 " + std::to_string(waitF)
                              + "min）");
        // 慢车行驶更久 ⇒ 等待更少；「行驶 + 等待 = 窗口起 − 发车」（等待不乘因子）。
        // 送达停靠的 rawArrival 是「行驶」部分、arrival 是「加等待」部分：
        //   · raw 按该车缩放后的耗时推进（慢车 = 快车 × 1.25）；
        //   · arrival 被窗口起钳到 600（两车相同）⇒ 等待 = 600 − raw，**不乘因子**。
        const logistics::Stop* dF = nullptr;
        const logistics::Stop* dS = nullptr;
        for (const logistics::Stop& st : wf.stops) {
            if (!st.deliveries.empty()) {
                dF = &st;
            }
        }
        for (const logistics::Stop& st : ws.stops) {
            if (!st.deliveries.empty()) {
                dS = &st;
            }
        }
        check(dF != nullptr && dS != nullptr, "等待夹具：两车都有送达停靠");
        if (dF != nullptr && dS != nullptr) {
            // 缩放的是**行驶时长**，不是绝对时刻：rawS = 发车 + (rawF − 发车) × 1.25。
            const double expectS = static_cast<double>(fast.departTimeMin)
                                   + (static_cast<double>(dF->rawArrivalMin)
                                      - static_cast<double>(fast.departTimeMin)) * 1.25;
            check(fixtures::nearlyEqual(static_cast<double>(dS->rawArrivalMin), expectS, 1.0),
                  "慢车 rawArrival = 发车 + (快车行驶) × 1.25（行驶乘因子；实际 "
                      + std::to_string(dS->rawArrivalMin) + " vs 预期 "
                      + std::to_string(expectS) + "）");
            check(dF->arrivalMin == 600 && dS->arrivalMin == 600,
                  "两车的送达到达都被窗口起钳到 600（arrival 相同）");
            check(dS->waitMin == 600 - dS->rawArrivalMin
                      && dF->waitMin == 600 - dF->rawArrivalMin,
                  "等待 = 窗口起 − rawArrival，**不乘因子**（慢车 "
                      + std::to_string(dS->waitMin) + " / 快车 "
                      + std::to_string(dF->waitMin) + "）");
            check(dS->waitMin < dF->waitMin,
                  "慢车行驶更久 ⇒ 早到等待更少（" + std::to_string(dS->waitMin) + " < "
                      + std::to_string(dF->waitMin) + "）");
        }
        check(ws.totalPenaltyMin == wf.totalPenaltyMin,
              "penalty 不乘因子（两车都在窗口内，penalty 都为 0）");
    }
}

// 腿结构（规格三 §1 / #45）+ 默认数据「全簇内」的事实：
// 默认 25 单没有一张跨簇 ⇒ 每条订单恰好 1 条 customer->customer 腿、没有站间腿；
// 就绪时刻 = 该单执行车的发车时刻（08:00）。这条守卫同时钉住「本票不改默认路线」：
// 腿只是登记结构，plan.summary 的黄金值与它无关，二者互不影响。
void checkDefaultLegsAreLocalSingleLegs(const Config& cfg, const FleetPlan* fleets[3],
                                       const char* labels[3]) {
    for (int s = 0; s < 3; ++s) {
        const FleetPlan& fleet = *fleets[s];
        const std::string tag = std::string("[腿·") + labels[s] + "] ";
        check(fleet.legs.size() == cfg.orders.size(),
              tag + "腿条数 == 订单数（默认 25 单全簇内 => 25 条腿），实际 "
                  + std::to_string(fleet.legs.size()));
        int stationLegs = 0;
        int multiLegOrders = 0;
        for (std::size_t i = 0; i < fleet.legs.size(); ++i) {
            const logistics::Leg& leg = fleet.legs[i];
            if (leg.kind == logistics::LegKind::StationToStation
                || leg.kind == logistics::LegKind::CustomerToStation
                || leg.kind == logistics::LegKind::StationToCustomer) {
                ++stationLegs;
            }
            if (leg.seq != 1) {
                ++multiLegOrders;
            }
        }
        check(stationLegs == 0, tag + "默认数据没有站间/中转腿（全簇内 => 0 条），实际 "
                                    + std::to_string(stationLegs));
        check(multiLegOrders == 0, tag + "默认数据每单只有 L1（没有 L2/L3）");
        // 逐单双向核对：订单表里每张单都能在腿清单里查到**它自己的** A->B 腿。
        for (std::size_t oi = 0; oi < cfg.orders.size(); ++oi) {
            const logistics::Order& o = cfg.orders[oi];
            bool found = false;
            int count = 0;
            for (std::size_t i = 0; i < fleet.legs.size(); ++i) {
                if (fleet.legs[i].orderId != o.id) {
                    continue;
                }
                ++count;
                if (fleet.legs[i].seq == 1
                    && fleet.legs[i].kind == logistics::LegKind::CustomerToCustomer
                    && fleet.legs[i].from == o.pickupNodeId && fleet.legs[i].to == o.nodeId) {
                    found = true;
                }
            }
            check(count == 1, tag + "订单 " + o.id + " 恰好 1 条腿（实际有 "
                                    + std::to_string(count) + " 条）");
            check(found, tag + "订单 " + o.id + " 的腿就是它自己的 A -> B");
        }
        // 行序：订单 ID 升序（同单内只有 L1，无需再比腿序）。
        bool ordered = true;
        for (std::size_t i = 1; i < fleet.legs.size(); ++i) {
            if (fleet.legs[i - 1].orderId > fleet.legs[i].orderId) {
                ordered = false;
            }
        }
        check(ordered, tag + "腿清单按订单 ID 升序");
        // 就绪时刻 = 该单执行车的发车时刻（默认三车都 08:00）。
        bool readyOk = true;
        for (std::size_t i = 0; i < fleet.legs.size(); ++i) {
            const logistics::Leg& leg = fleet.legs[i];
            int depart = -1;
            for (std::size_t v = 0; v < cfg.vehicles.size(); ++v) {
                if (cfg.vehicles[v].id == leg.vehicleId) {
                    depart = cfg.vehicles[v].departTimeMin;
                }
            }
            if (depart < 0 || leg.readyTimeMin != depart) {
                readyOk = false;
            }
        }
        check(readyOk, tag + "每腿 readyTime = 该腿执行车的发车时刻");
        // 「结构已立、接力未启用」必须在这条入口也留登记（不声称已转运）。
        bool logged = false;
        for (std::size_t i = 0; i < fleet.legLog.size(); ++i) {
            if (fleet.legLog[i].find("接力未启用") != std::string::npos) {
                logged = true;
            }
        }
        check(logged, tag + "legLog 登记「结构已立、接力未启用」");
    }
}

} // namespace

int main() {
    Config cfg;
    std::string error;
    const bool ok = ConfigLoader::load(DEFAULT_CONFIG_PATH, cfg, error);
    check(ok, "默认配置加载成功（" + error + "）");
    if (!ok || cfg.vehicles.empty()) {
        return testutil::summarize("integration_tests");
    }

    std::printf("    默认数据集：节点 %zu，边 %zu，订单 %zu，车辆 %zu 台\n",
                cfg.graph.nodeCount(), cfg.graph.edgeCount(), cfg.orders.size(),
                cfg.vehicles.size());
    for (const Vehicle& v : cfg.vehicles) {
        std::printf("      车 %s：%s，%.0fkg，%02d:%02d，time×%.2f cost×%.2f\n",
                    v.id.c_str(), v.startNodeId.c_str(), v.capacityKg, v.departTimeMin / 60,
                    v.departTimeMin % 60, v.timeFactor, v.costFactor);
    }

    const FleetPlan byDistance = planFleetForWeight(cfg, WeightType::Distance, "最短距离策略");
    const FleetPlan byCost = planFleetForWeight(cfg, WeightType::Cost, "最低成本策略");
    const FleetPlan byTime = planFleetForWeight(cfg, WeightType::Time, "最短耗时策略");

    checkDataQualityInvariants(cfg, byDistance, byCost, byTime);
    checkDefaultTransitNotTriggered(cfg, byDistance, byCost, byTime);

    // ---- 规格二 §4 / #40：9 条分配不变量（三策略 + 顺序轮转基线）----
    const FleetPlan baseDistance =
        planFleetSequentialBaseline(cfg.graph, cfg.vehicles, cfg.orders, WeightType::Distance);
    const FleetPlan baseCost =
        planFleetSequentialBaseline(cfg.graph, cfg.vehicles, cfg.orders, WeightType::Cost);
    const FleetPlan baseTime =
        planFleetSequentialBaseline(cfg.graph, cfg.vehicles, cfg.orders, WeightType::Time);
    const FleetPlan* threeFleets[3] = {&byDistance, &byCost, &byTime};
    const char* threeLabels[3] = {"最短距离策略", "最低成本策略", "最短耗时策略"};
    checkAllocationInvariants(cfg, threeFleets, threeLabels, baseDistance, baseCost, baseTime);

    // 腿与就绪链（规格三 §1 / #45）：默认数据全簇内 => 每单 1 条本地腿，无站间腿。
    {
        const FleetPlan* legFleets[3] = {&byDistance, &byCost, &byTime};
        const char* legLabels[3] = {"最短距离策略", "最低成本策略", "最短耗时策略"};
        checkDefaultLegsAreLocalSingleLegs(cfg, legFleets, legLabels);
    }

    // 阶段二的 6 种排列留痕（默认 3 簇 3 车）——人工核对「各自怎么取优」。
    for (std::size_t i = 0; i < byDistance.candidateKeyLog.size(); ++i) {
        std::printf("      排列[%zu] %s\n", i, byDistance.candidateKeyLog[i].c_str());
    }

    // 车队专属规则（规格二 §3.4 / §3.6）：空车与「慢车因子」两条。
    checkEmptyVehicleRules(cfg);
    checkSlowVehicleFactorReplay(cfg);

    checkSmallCapacityStaysSingleRoute(cfg);
    checkGraphRepresentationsOnRealData(cfg);
    checkTrafficAndUrgentOrderOnRealData(cfg);

    return testutil::summarize("integration_tests");
}