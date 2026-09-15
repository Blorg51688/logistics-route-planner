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
using logistics::insertUrgentOrder;
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

// 合并后停靠点的窗口跨度（分钟）：同节点多订单取并集 [min 起, max 止]
int windowSpanMinutes(const Config& cfg, const std::string& nodeId) {
    int lo = 1 << 30;
    int hi = -(1 << 30);
    for (const logistics::Order& o : cfg.orders) {
        if (o.nodeId != nodeId) {
            continue;
        }
        if (o.windowStartMin < lo) {
            lo = o.windowStartMin;
        }
        if (o.windowEndMin > hi) {
            hi = o.windowEndMin;
        }
    }
    return hi > lo ? hi - lo : 0;
}

RoutePlan checkPlanCoversAllOrders(const Config& cfg, WeightType weight, const char* label) {
    const Vehicle& v = cfg.vehicles[0];
    const RoutePlan plan = planRoute(cfg.graph, v, cfg.orders, cfg.general.serviceTimeMin, weight);

    check(plan.status == PlanStatus::Ok,
          std::string(label) + "：默认数据可规划出可行路线（reason: " + plan.reason + "）");

    const std::size_t expectedStops = distinctOrderNodes(cfg);
    check(plan.stops.size() == expectedStops,
          std::string(label) + "：停靠点数 " + std::to_string(plan.stops.size())
              + " 应等于有订单的配送点数 " + std::to_string(expectedStops));

    // 每个有订单的配送点必须恰好停靠一次
    for (const logistics::Order& o : cfg.orders) {
        std::size_t hits = 0;
        for (const logistics::Stop& s : plan.stops) {
            if (s.nodeId == o.nodeId) {
                ++hits;
            }
        }
        check(hits == 1, std::string(label) + "：配送点 " + o.nodeId + " 恰好停靠一次");
    }

    check(plan.totalDistanceKm > 0.0, std::string(label) + "：总距离为正");
    check(plan.totalPenaltyMin >= 0, std::string(label) + "：总 penalty 非负");
    check(!plan.nodes.empty() && plan.nodes.front() == v.startNodeId,
          std::string(label) + "：路线从起始仓库出发");
    check(plan.nodes.back() == v.startNodeId,
          std::string(label) + "：路线返回起始仓库");

    std::printf("      %s: 距离 %.3fkm  耗时 %.3fmin  成本 %.3f元  penalty %dmin  停靠 %zu 站\n",
                label, plan.totalDistanceKm, plan.totalTimeMin, plan.totalCostYuan,
                plan.totalPenaltyMin, plan.stops.size());

    // 列出超时停靠点，便于核对 E2（时间窗硬约束 + penalty）的演示效果
    std::printf("        超时停靠点:");
    bool anyLate = false;
    for (const logistics::Stop& s : plan.stops) {
        if (s.late) {
            std::printf(" %s(到达%d,penalty%d)", s.nodeId.c_str(), s.arrivalMin, s.penaltyMin);
            anyLate = true;
        }
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
void checkDataQualityInvariants(const Config& cfg, const RoutePlan& byDistance,
                                const RoutePlan& byCost) {
    // ① 双策略必须真的分化
    check(byDistance.nodes != byCost.nodes
              || !fixtures::nearlyEqual(byDistance.totalDistanceKm, byCost.totalDistanceKm),
          "两种策略必须给出不同结果（相同则最低成本策略形同虚设）");

    // ② 各自在自身目标上不得劣于对方 —— "最优"的直接体现
    check(byDistance.totalDistanceKm <= byCost.totalDistanceKm + 1e-6,
          "距离策略的总距离不应劣于成本策略");
    check(byCost.totalCostYuan <= byDistance.totalCostYuan + 1e-6,
          "成本策略的总成本不应劣于距离策略");

    // ③ 单车辆必须能在工作日内跑完，否则默认数据下全部订单必然超时
    const double kWorkdayMinutes = 12.0 * 60.0;
    check(byDistance.totalTimeMin <= kWorkdayMinutes,
          "距离策略总耗时须在 12 小时工作日内，实际 "
              + std::to_string(byDistance.totalTimeMin));
    check(byCost.totalTimeMin <= kWorkdayMinutes,
          "成本策略总耗时须在 12 小时工作日内，实际 " + std::to_string(byCost.totalTimeMin));

    // ④ 超时必须是少数，且只能出现在"刻意收紧"的窗口上：
    //    窗口跨度充裕（> 4 小时）的停靠点绝不允许超时。
    //    这条用"窗口跨度"判定而非硬编码节点 ID，数据调整后依然成立。
    const std::size_t kMaxLateStops = 3;
    const int kTightWindowMinutes = 4 * 60;
    const RoutePlan* plans[] = {&byDistance, &byCost};
    const char* labels[] = {"距离策略", "成本策略"};
    for (int i = 0; i < 2; ++i) {
        std::size_t lateStops = 0;
        for (const logistics::Stop& s : plans[i]->stops) {
            if (!s.late) {
                continue;
            }
            ++lateStops;
            const int span = windowSpanMinutes(cfg, s.nodeId);
            check(span <= kTightWindowMinutes,
                  std::string(labels[i]) + "：窗口跨度充裕的停靠点不应超时，"
                      + s.nodeId + " 跨度 " + std::to_string(span) + " 分钟");
        }
        check(lateStops <= kMaxLateStops,
              std::string(labels[i]) + "：超时停靠点应属少数，实际 "
                  + std::to_string(lateStops) + " / " + std::to_string(plans[i]->stops.size()));
    }

    // 5) 黄金基准（characterization test）。
    //    明确性质：这四个值是从**当前行为捕获**的，不是独立推导的真值
    //    （贪心顺序来自被测实现，oracle 无法独立复现）。因此它们不用于证明正确性，
    //    只用于**检测非预期漂移**：数据或算法被无意改动时立刻报警。
    //    有意调整数据/算法时，应连同这些值一起显式更新。
    check(fixtures::nearlyEqual(byDistance.totalDistanceKm, 210.8, 0.01),
          "黄金值·距离策略总距离 210.8，实际 " + std::to_string(byDistance.totalDistanceKm));
    check(fixtures::nearlyEqual(byDistance.totalCostYuan, 273.4, 0.01),
          "黄金值·距离策略总成本 273.4，实际 " + std::to_string(byDistance.totalCostYuan));
    check(fixtures::nearlyEqual(byCost.totalDistanceKm, 247.0, 0.01),
          "黄金值·成本策略总距离 247.0，实际 " + std::to_string(byCost.totalDistanceKm));
    check(fixtures::nearlyEqual(byCost.totalCostYuan, 247.0, 0.01),
          "黄金值·成本策略总成本 247.0，实际 " + std::to_string(byCost.totalCostYuan));
}

// B3：两种图表示在真实规模（30 节点）上的形状检查
// D21：总需求超过载重上限**不再判不可行**，改为多趟 + 中转集散。
// 用真实网络 + 把载重改小来构造（默认 740 <= 800 只会走单趟）。
// 合成图只能覆盖简单拓扑，这里验证真实网络上的不变量。
void checkMultiTripAndTransitOnRealData(const Config& cfg) {
    Vehicle small = cfg.vehicles[0];
    small.capacityKg = 100.0;   // 远小于总需求 740kg

    const RoutePlan plan = planRoute(cfg.graph, small, cfg.orders,
                                     cfg.general.serviceTimeMin, WeightType::Distance);

    check(plan.status == PlanStatus::Ok, "载重远小于总需求时不再是不可行");
    check(plan.stops.size() == distinctOrderNodes(cfg),
          "全部配送点仍被服务，实际 " + std::to_string(plan.stops.size()));
    check(plan.trips.size() > 1,
          "确实分成多趟，实际 " + std::to_string(plan.trips.size()) + " 趟");

    // 不变量：任一趟的在车货量不超过载重上限
    bool loadOk = true;
    double maxOp = 0.0;
    for (const logistics::Trip& trip : plan.trips) {
        for (const logistics::Stop& s : trip.stops) {
            if (s.remainingLoadKg < -1e-9) {
                loadOk = false;
            }
        }
        for (const logistics::TransitOp& op : trip.transitOps) {
            if (std::fabs(op.amountKg) > small.capacityKg + 1e-9) {
                loadOk = false;
            }
            if (std::fabs(op.amountKg) > maxOp) {
                maxOp = std::fabs(op.amountKg);
            }
        }
    }
    check(loadOk, "任一趟的在车货量都不超过载重上限");
    check(maxOp > 0.0, "中转站确有装卸记录，单次最大装卸 " + std::to_string(maxOp) + "kg");

    // 不变量：暂存终值必须为 0；峰值 > 0 说明中转站确实参与了集散
    double finalAbs = 0.0;
    double peakSum = 0.0;
    std::size_t usedStations = 0;
    for (const logistics::TransitStock& st : plan.transitStock) {
        finalAbs += std::fabs(st.finalKg);
        peakSum += st.peakKg;
        if (st.peakKg > 1e-9) {
            ++usedStations;
        }
    }
    check(finalAbs < 1e-6, "规划结束时全部中转站暂存为 0（不留残余库存）");
    check(peakSum > 0.0, "中转站峰值暂存合计 > 0，实际 " + std::to_string(peakSum));
    check(usedStations > 0, "至少一个中转站被真正使用");

    // 不变量：扁平视图必须等于各趟的拼接
    std::size_t stopSum = 0;
    double distSum = 0.0;
    double costSum = 0.0;
    int penaltySum = 0;
    for (const logistics::Trip& trip : plan.trips) {
        stopSum += trip.stops.size();
        distSum += trip.totalDistanceKm;
        costSum += trip.totalCostYuan;
        for (const logistics::Stop& s : trip.stops) {
            penaltySum += s.penaltyMin;
        }
    }
    check(stopSum == plan.stops.size(), "停靠点数 = 各趟之和");
    check(std::fabs(distSum - plan.totalDistanceKm) < 1e-6, "总距离 = 各趟之和");
    check(std::fabs(costSum - plan.totalCostYuan) < 1e-6, "总成本 = 各趟之和");
    check(penaltySum == plan.totalPenaltyMin, "总 penalty = 各停靠点之和");
    check(plan.nodes.size() == plan.nodeArrivalMin.size()
              && plan.nodes.size() == plan.nodeIsStop.size(),
          "扁平序列的节点/到达时刻/停靠标记三个数组等长");
    check(!plan.nodes.empty() && plan.nodes.front() == cfg.vehicles[0].startNodeId
              && plan.nodes.back() == cfg.vehicles[0].startNodeId,
          "多趟路线从仓库出发并回到仓库");

    // 序列必须是图上真实可走的：相邻节点之间必须有有向边。
    // 这条断言本该一开始就有——多趟的 Trip 一度漏写起点，
    // 只查"末尾是不是仓库"恰好被蒙对，直到看图才发现。
    bool walkable = true;
    for (std::size_t i = 1; i < plan.nodes.size(); ++i) {
        if (cfg.graph.findEdge(plan.nodes[i - 1], plan.nodes[i]) == nullptr) {
            walkable = false;
        }
    }
    check(walkable, "多趟路线的相邻节点之间都存在有向边");

    bool tripsWalkable = true;
    for (std::size_t k = 0; k < plan.trips.size(); ++k) {
        const logistics::Trip& trip = plan.trips[k];
        if (trip.nodes.empty()) {
            tripsWalkable = false;
            continue;
        }
        if (k > 0 && trip.nodes.front() != plan.trips[k - 1].endNodeId) {
            tripsWalkable = false;
        }
        for (std::size_t i = 1; i < trip.nodes.size(); ++i) {
            if (cfg.graph.findEdge(trip.nodes[i - 1], trip.nodes[i]) == nullptr) {
                tripsWalkable = false;
            }
        }
    }
    check(tripsWalkable, "各趟首尾相接且每趟自身可走");
    check(!plan.trips.empty() && plan.trips.back().endNodeId == cfg.vehicles[0].startNodeId,
          "最后一趟终点是仓库");

    std::printf("    多趟场景（载重 100kg）：%zu 趟，总距离 %.1fkm，"
                "使用 %zu 个中转站，峰值合计 %.0fkg\n",
                plan.trips.size(), plan.totalDistanceKm, usedStations, peakSum);
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
    const double service = cfg.general.serviceTimeMin;

    const RoutePlan before = planRoute(g, v, cfg.orders, service, WeightType::Distance);
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
    const int nowMin = before.stops[served - 1].departureMin;

    std::vector<logistics::Order> remaining;
    for (const logistics::Order& o : cfg.orders) {
        bool done = false;
        for (std::size_t k = 0; k < served; ++k) {
            if (before.stops[k].nodeId == o.nodeId) {
                done = true;
            }
        }
        if (!done) {
            remaining.push_back(o);
        }
    }
    check(remaining.size() == cfg.orders.size() - served,
          "剩余订单数 = 总订单数 - 已服务站点数");

    const bool trigger = needsReplan(before.nodes, report, cfg.general.trafficTimeIncreaseMin);
    std::printf("      E1: 已服务 %zu 站，位置 %s，时刻 %d；路况改动 %zu 条边；触发重规划=%s\n",
                served, here.c_str(), nowMin, report.changes.size(), trigger ? "是" : "否");

    if (trigger) {
        const RoutePlan replanned = replan(g, v, remaining, here, nowMin, service,
                                           WeightType::Distance);
        check(replanned.status == PlanStatus::Ok, "路况重规划后仍可行");
        check(replanned.stops.size() == remaining.size(), "重规划恰好覆盖全部剩余订单");
        std::printf("          重规划后：距离 %.1fkm  耗时 %.1fmin  penalty %dmin\n",
                    replanned.totalDistanceKm, replanned.totalTimeMin,
                    replanned.totalPenaltyMin);
    }

    // E3：插入一个紧急订单，窗口足够宽（不触发冲突警告）
    logistics::Order urgent;
    urgent.id = "URG01";
    urgent.nodeId = remaining.front().nodeId;
    urgent.demandKg = 5.0;
    urgent.windowStartMin = 0;
    urgent.windowEndMin = 1440;
    urgent.urgent = true;

    const InsertResult inserted = insertUrgentOrder(g, v, remaining, urgent, here, nowMin,
                                                    service, WeightType::Distance);
    check(inserted.plan.status == PlanStatus::Ok, "插入紧急订单后仍可行");
    check(inserted.warning.empty(), "窗口充裕时不应冲突，实际: " + inserted.warning);
    check(!inserted.plan.stops.empty() && inserted.plan.stops[0].nodeId == urgent.nodeId,
          "插入的紧急订单应被优先服务");
    std::printf("      E3: 插入紧急订单 %s@%s，首个停靠=%s，距离 %.1fkm 耗时 %.1fmin\n",
                urgent.id.c_str(), urgent.nodeId.c_str(),
                inserted.plan.stops.empty() ? "(无)" : inserted.plan.stops[0].nodeId.c_str(),
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

    const RoutePlan byDistance =
        checkPlanCoversAllOrders(cfg, WeightType::Distance, "最短距离策略");
    const RoutePlan byCost = checkPlanCoversAllOrders(cfg, WeightType::Cost, "最低成本策略");

    checkDataQualityInvariants(cfg, byDistance, byCost);

    checkMultiTripAndTransitOnRealData(cfg);
    checkGraphRepresentationsOnRealData(cfg);
    checkTrafficAndUrgentOrderOnRealData(cfg);

    return testutil::summarize("integration_tests");
}
