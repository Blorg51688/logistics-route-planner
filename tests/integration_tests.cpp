// 集成测试：用真实的 config/default.ini 跑通「配置 -> 规划」全链路。
// 单元测试（routeplanner_tests）用人工小图钉住语义；
// 这里验证真实数据集确实能支撑题目要求的完整配送（B6），
// 并顺带打印两种策略的路线摘要，便于人工核对演示数据。
#include <cstdio>
#include <string>
#include <vector>

#include "core/Config.h"
#include "core/RoutePlanner.h"
#include "io/ConfigLoader.h"
#include "test_util.h"

using logistics::Config;
using logistics::ConfigLoader;
using logistics::PlanStatus;
using logistics::RoutePlan;
using logistics::Vehicle;
using logistics::WeightType;
using logistics::planRoute;
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

    return testutil::summarize("integration_tests");
}
