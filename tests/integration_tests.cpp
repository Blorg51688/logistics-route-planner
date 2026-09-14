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

void checkPlanCoversAllOrders(const Config& cfg, WeightType weight, const char* label) {
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

    std::printf("      %s: 距离 %.1fkm  耗时 %.1fmin  成本 %.1f元  penalty %dmin  停靠 %zu 站\n",
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
    // oracle: round(0.1 * 72 边) = 7
    check(report.changes.size() == 7,
          "按 traffic_change_ratio=0.1 改动 7 条边（72 边），实际 "
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

    checkPlanCoversAllOrders(cfg, WeightType::Distance, "最短距离策略");
    checkPlanCoversAllOrders(cfg, WeightType::Cost, "最低成本策略");

    checkTrafficAndUrgentOrderOnRealData(cfg);

    return testutil::summarize("integration_tests");
}
