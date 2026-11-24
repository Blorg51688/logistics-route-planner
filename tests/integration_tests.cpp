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
#include "test_util.h"

using logistics::Config;
using logistics::LogisticsGraph;
using logistics::ConfigLoader;
using logistics::PlanStatus;
using logistics::RoutePlan;
using logistics::Vehicle;
using logistics::WeightType;
using logistics::planRoute;
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
void checkPlanInvariants(const Config& cfg, const RoutePlan& plan, const std::string& label) {
    const std::string tag = label + "：";

    check(plan.status == PlanStatus::Ok, tag + "可规划出可行路线（reason: " + plan.reason + "）");
    if (plan.status != PlanStatus::Ok) {
        return;
    }
    check(!plan.nodes.empty(), tag + "路线非空");
    if (plan.nodes.empty()) {
        return;
    }
    check(plan.nodes.front() == cfg.vehicles[0].startNodeId, tag + "从起始仓库出发");
    check(plan.nodes.back() == cfg.vehicles[0].startNodeId, tag + "返回起始仓库");

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
        if (plan.stops[i].nodeId == cfg.vehicles[0].startNodeId) {
            depotOnce = false;
        }
    }
    check(depotOnce, tag + "中途不停靠仓库（单条连续路线，仓库装货语义已退役）");

    // 不变量 ① ②：每张订单恰好一次送达、至多一次取货，且取货早于送达。
    for (const logistics::Order& o : cfg.orders) {
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
        if (after < -1e-9 || after > cfg.vehicles[0].capacityKg + 1e-6) {
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
        fixtures::nonForcedRevisitStops(plan.stops, cfg.orders, cfg.vehicles[0].capacityKg, 0.0);
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
    check(m.actionCount == 2 * static_cast<int>(cfg.orders.size()),
          tag + "结构性恒等：动作数 = 2 × 取送对数（数据全为 A→B 取送对）");
    check(m.pickupOnTimeCount >= m.deliveryOnTimeCount, tag + "取货准时数 >= 送达准时数（G4）");
    check(m.avgLoadKg <= m.peakLoadKg + 1e-9, tag + "平均在车 <= 峰值（G5）");
    check(m.peakLoadKg <= cfg.vehicles[0].capacityKg + 1e-6, tag + "峰值 <= 载重上限（G5）");
    check(std::fabs(m.loadUtilization - m.avgLoadKg / cfg.vehicles[0].capacityKg) < 0.005,
          tag + "利用率 == 平均/容量（G5）");
    check(m.deliveryTotal == static_cast<int>(cfg.orders.size()),
          tag + "送达准时率分母 = 已规划订单数（G6）");
}

RoutePlan planForWeight(const Config& cfg, WeightType weight, const char* label) {
    const RoutePlan plan = planRoute(cfg.graph, cfg.vehicles[0], cfg.orders, weight);
    checkPlanInvariants(cfg, plan, label);

    // 逐单超时清单一并打出来（第 3 行摘要的来源），便于人工核对逐单窗口。
    std::printf("      %s: 距离 %.3fkm  耗时 %.3fmin  成本 %.3f元  penalty %dmin  停靠 %d 站 动作 %d 次\n",
                label, plan.totalDistanceKm, plan.totalTimeMin, plan.totalCostYuan,
                plan.totalPenaltyMin, plan.metrics.stopCount, plan.metrics.actionCount);
    std::printf("        超时订单:");
    bool anyLate = false;
    for (const logistics::OrderMetric& om : plan.metrics.orders) {
        if (!om.late) {
            continue;
        }
        std::printf(" %s@%s(+%dmin)", om.orderId.c_str(), om.nodeId.c_str(), om.penaltyMin);
        anyLate = true;
    }
    std::printf("%s\n", anyLate ? "" : " 无");
    return plan;
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
void checkDataQualityInvariants(const Config& cfg, const RoutePlan& byDistance,
                                const RoutePlan& byCost, const RoutePlan& byTime) {
    const RoutePlan* plans[] = {&byDistance, &byCost, &byTime};
    const char* labels[] = {"距离策略", "成本策略", "时间策略"};

    // ① 三策略**两两**分化（3 对）：节点序列或自身指标必须不同。
    const double own[3] = {byDistance.totalDistanceKm, byCost.totalCostYuan,
                           byTime.totalTimeMin};
    for (int i = 0; i < 3; ++i) {
        for (int j = i + 1; j < 3; ++j) {
            const bool differ = plans[i]->nodes != plans[j]->nodes
                                || !fixtures::nearlyEqual(own[i], own[j], 0.001);
            check(differ, std::string(labels[i]) + " 与 " + labels[j]
                              + " 必须给出不同结果（三策略两两分化）");
        }
    }

    // ② 每条策略在**自身目标**上不劣于另两条（3 × 2 = 6 条）。
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
    check(byTime.totalTimeMin <= byDistance.totalTimeMin + 1e-6,
          "时间策略的总耗时不应劣于距离策略（" + std::to_string(byTime.totalTimeMin)
              + " vs " + std::to_string(byDistance.totalTimeMin) + "）");
    check(byTime.totalTimeMin <= byCost.totalTimeMin + 1e-6,
          "时间策略的总耗时不应劣于成本策略（" + std::to_string(byTime.totalTimeMin)
              + " vs " + std::to_string(byCost.totalTimeMin) + "）");

    // ③ 单车辆必须能在工作日内跑完，否则默认数据下全部订单必然超时。
    const double kWorkdayMinutes = 12.0 * 60.0;
    for (int i = 0; i < 3; ++i) {
        check(plans[i]->totalTimeMin <= kWorkdayMinutes,
              std::string(labels[i]) + " 总耗时须在 12 小时工作日内，实际 "
                  + std::to_string(plans[i]->totalTimeMin));
    }

    // ④ 迟到必须只出现在"刻意收紧"的窗口上，且**逐单**判（旧的并集口径会把紧窗单
    //    迟到判成宽窗停靠点迟到而误红）。阈值是 characterization：实施后按实测重定。
    const std::size_t kMaxLateOrders = 8;
    const int kTightWindowMinutes = 4 * 60;
    for (int i = 0; i < 3; ++i) {
        std::size_t lateOrders = 0;
        for (const logistics::OrderMetric& om : plans[i]->metrics.orders) {
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

    // ⑤ 三策略 × 13 字段的黄金值**本票不落**（由 #35 捕获并写进 tests/plan_golden.h）。
    //    这里只保留"当前实现自洽"的占位断言，等 #35 把真值表接过来之后删掉。
    // TODO(#35): 用 tests/plan_golden.h 的单一真值表替换下面这段自洽性检查。
    for (int i = 0; i < 3; ++i) {
        const logistics::RouteMetrics& m = plans[i]->metrics;
        check(m.stopCount > 0 && m.actionCount == 2 * static_cast<int>(cfg.orders.size()),
              std::string(labels[i]) + "：停靠数与动作数自洽（#35 前占位）");
        check(m.deliveryTotal == static_cast<int>(cfg.orders.size())
                  && m.pickupTotal == static_cast<int>(cfg.orders.size()),
              std::string(labels[i]) + "：准时率分母自洽（#35 前占位）");
        check(fixtures::nearlyEqual(plans[i]->totalPenaltyMin,
                                    [&]() {
                                        int sum = 0;
                                        for (const logistics::OrderMetric& om : m.orders) {
                                            sum += om.penaltyMin;
                                        }
                                        return static_cast<double>(sum);
                                    }(),
                                    0.001),
              std::string(labels[i]) + "：总 penalty = 逐单 penalty 之和（G7 占位）");
    }
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

} // namespace

int main() {
    Config cfg;
    std::string error;
    const bool ok = ConfigLoader::load(DEFAULT_CONFIG_PATH, cfg, error);
    check(ok, "默认配置加载成功（" + error + "）");
    if (!ok || cfg.vehicles.empty()) {
        return testutil::summarize("integration_tests");
    }

    std::printf("    默认数据集：节点 %zu，边 %zu，订单 %zu，载重上限 %.0fkg\n",
                cfg.graph.nodeCount(), cfg.graph.edgeCount(), cfg.orders.size(),
                cfg.vehicles[0].capacityKg);

    const RoutePlan byDistance = planForWeight(cfg, WeightType::Distance, "最短距离策略");
    const RoutePlan byCost = planForWeight(cfg, WeightType::Cost, "最低成本策略");
    const RoutePlan byTime = planForWeight(cfg, WeightType::Time, "最短耗时策略");

    checkDataQualityInvariants(cfg, byDistance, byCost, byTime);

    checkSmallCapacityStaysSingleRoute(cfg);
    checkGraphRepresentationsOnRealData(cfg);
    checkTrafficAndUrgentOrderOnRealData(cfg);

    return testutil::summarize("integration_tests");
}
