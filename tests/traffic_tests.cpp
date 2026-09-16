// E1 路况模拟/重规划 与 E3 动态订单插入 的行为测试。
// 期望值均先由 tools/oracle.py 独立算出，再抄为字面量。
#include <algorithm>
#include <initializer_list>
#include <map>
#include <string>
#include <vector>

#include "core/LogisticsGraph.h"
#include "core/RoutePlanner.h"
#include "core/Traffic.h"
#include "graph_fixtures.h"
#include "test_util.h"

using logistics::LogisticsGraph;
using logistics::NodeType;
using logistics::TrafficReport;
using logistics::Vehicle;
using logistics::simulateTrafficChange;
using logistics::needsReplan;
using testutil::check;

namespace {

using fixtures::addTwoWay;
using fixtures::makeNode;
using fixtures::makeVehicle;
using fixtures::makeOrder;

// W <-> D1 <-> D2：W-D1 1.0/2.0/1.0，W-D2 5.0/10.0/5.0，D1-D2 2.0/4.0/2.0
// 共 6 条有向边
LogisticsGraph makeGreedyGraph() {
    LogisticsGraph g;
    g.addNode(makeNode("W", NodeType::Warehouse, "仓库"));
    g.addNode(makeNode("D1", NodeType::Delivery, "客户1"));
    g.addNode(makeNode("D2", NodeType::Delivery, "客户2"));
    addTwoWay(g, "W", "D1", 1.0, 2.0, 1.0);
    addTwoWay(g, "W", "D2", 5.0, 10.0, 5.0);
    addTwoWay(g, "D1", "D2", 2.0, 4.0, 2.0);
    return g;
}

std::string key(const std::string& from, const std::string& to) {
    return from + "|" + to;
}

// 快照所有边的 baseTimeMin 与 timeMin，用于验证"改的是 timeMin、不动 baseTimeMin"
struct EdgeSnapshot {
    std::map<std::string, double> base;
    std::map<std::string, double> time;
    std::map<std::string, bool> congested;
};

EdgeSnapshot snapshot(const LogisticsGraph& g) {
    EdgeSnapshot s;
    for (const logistics::Edge& e : g.edges()) {
        s.base[key(e.fromId, e.toId)] = e.baseTimeMin;
        s.time[key(e.fromId, e.toId)] = e.timeMin;
        s.congested[key(e.fromId, e.toId)] = e.congested;
    }
    return s;
}

// 切片 A：改动边数按 round(ratio*E) 计，增幅落在 [min,max]，
// 且只改 timeMin、绝不动 baseTimeMin。
void testTrafficChangeCountMagnitudeAndBasePreserved() {
    LogisticsGraph g = makeGreedyGraph();
    check(g.edgeCount() == 6, "夹具 6 条有向边");
    const EdgeSnapshot before = snapshot(g);

    // ratio 0.5 * 6 边 = 3 条（由 oracle 算出）
    const TrafficReport report = simulateTrafficChange(g, 0.5, 0.2, 0.5, 12345u, 0.0);
    check(report.changes.size() == 3,
          "改动 3 条边，实际 " + std::to_string(report.changes.size()));

    for (const logistics::TrafficChange& c : report.changes) {
        const std::string k = key(c.fromId, c.toId);
        check(before.base.count(k) == 1, "改动边确实存在于图中: " + k);
        check(c.increaseRatio >= 0.2 - 1e-9
                  && c.increaseRatio <= 0.5 + 1e-9,
              "增幅落在 [0.2, 0.5]，实际 " + std::to_string(c.increaseRatio));

        const logistics::Edge* e = g.findEdge(c.fromId, c.toId);
        check(e != nullptr, "改动后仍能查到该边");
        if (e != nullptr) {
            check(fixtures::nearlyEqual(e->baseTimeMin, before.base.at(k)),
                  "baseTimeMin 必须保持不变: " + k);
            check(fixtures::nearlyEqual(
                      e->timeMin, e->baseTimeMin * (1.0 + c.increaseRatio)),
                  "timeMin = base * (1+增幅): " + k);
            check(e->congested, "被改动的边必须置拥堵标记: " + k);
        }
    }

    // 未被选中的边必须原样不动
    std::size_t touched = 0;
    for (const logistics::Edge& e : g.edges()) {
        const std::string k = key(e.fromId, e.toId);
        if (!fixtures::nearlyEqual(e.timeMin, before.time.at(k))) {
            ++touched;
        }
    }
    check(touched == 3, "恰好 3 条边的 timeMin 发生变化，实际 " + std::to_string(touched));
}

// 切片 A：ratio = 1.0 时必须改动全部边（与随机种子无关）
void testTrafficRatioOneTouchesEveryEdge() {
    LogisticsGraph g = makeGreedyGraph();
    const TrafficReport report = simulateTrafficChange(g, 1.0, 0.2, 0.5, 99u, 0.0);

    check(report.changes.size() == 6,
          "ratio=1.0 时改动全部 6 条边，实际 " + std::to_string(report.changes.size()));

    std::size_t congested = 0;
    for (const logistics::Edge& e : g.edges()) {
        if (e.congested) {
            ++congested;
        }
    }
    check(congested == 6, "全部边都标记拥堵，实际 " + std::to_string(congested));
}

// 切片 A：同一种子必须给出完全相同的结果（否则测试必 flaky）
void testTrafficIsDeterministicForSameSeed() {
    LogisticsGraph g1 = makeGreedyGraph();
    LogisticsGraph g2 = makeGreedyGraph();

    const TrafficReport r1 = simulateTrafficChange(g1, 0.5, 0.2, 0.5, 2024u, 0.0);
    const TrafficReport r2 = simulateTrafficChange(g2, 0.5, 0.2, 0.5, 2024u, 0.0);

    check(r1.changes.size() == r2.changes.size(), "同种子改动数量一致");
    bool identical = r1.changes.size() == r2.changes.size();
    for (std::size_t i = 0; identical && i < r1.changes.size(); ++i) {
        identical = r1.changes[i].fromId == r2.changes[i].fromId
                    && r1.changes[i].toId == r2.changes[i].toId
                    && fixtures::nearlyEqual(r1.changes[i].increaseRatio,
                                             r2.changes[i].increaseRatio);
    }
    check(identical, "同种子改动内容与顺序完全一致");

    const EdgeSnapshot s1 = snapshot(g1);
    const EdgeSnapshot s2 = snapshot(g2);
    check(s1.time == s2.time, "同种子下两条图的边耗时完全一致");
}

logistics::TrafficChange changeOf(const std::string& from, const std::string& to, double ratio) {
    logistics::TrafficChange c;
    c.fromId = from;
    c.toId = to;
    c.increaseRatio = ratio;
    return c;
}

TrafficReport reportOf(std::initializer_list<logistics::TrafficChange> items) {
    TrafficReport r;
    for (const logistics::TrafficChange& c : items) {
        r.changes.push_back(c);
    }
    return r;
}


// 切片：路况变化必须也能"变畅通"。
// 需求原文把实时路况定义为动态属性"拥堵 / 畅通"，早先只实现了变拥堵。
// clearProbability = 1.0 时，被选中的边应全部恢复自由通行基准。
void testTrafficCanAlsoBecomeClear() {
    LogisticsGraph g = makeGreedyGraph();
    // 先制造一次拥堵
    simulateTrafficChange(g, 1.0, 0.4, 0.4, 5u, 0.0);
    std::size_t congestedBefore = 0;
    for (const logistics::Edge& e : g.edges()) {
        if (e.congested) {
            ++congestedBefore;
        }
    }
    check(congestedBefore == 6, "先让全部 6 条边拥堵，实际 "
              + std::to_string(congestedBefore));

    // 再全部转为畅通
    const TrafficReport report = simulateTrafficChange(g, 1.0, 0.2, 0.5, 9u, 1.0);
    check(report.changes.size() == 6, "畅通时同样改动全部 6 条边");

    bool allClear = true;
    bool ratioZero = true;
    bool timeBackToBase = true;
    for (const logistics::TrafficChange& c : report.changes) {
        if (c.congested) {
            allClear = false;
        }
        if (c.increaseRatio > 1e-9) {
            ratioZero = false;
        }
        const logistics::Edge* e = g.findEdge(c.fromId, c.toId);
        if (e == nullptr || !fixtures::nearlyEqual(e->timeMin, e->baseTimeMin)) {
            timeBackToBase = false;
        }
    }
    check(allClear, "全部标记为畅通而非拥堵");
    check(ratioZero, "畅通的增幅为 0");
    check(timeBackToBase, "畅通后耗时恢复到 baseTimeMin");

    std::size_t stillCongested = 0;
    for (const logistics::Edge& e : g.edges()) {
        if (e.congested) {
            ++stillCongested;
        }
    }
    check(stillCongested == 0, "图中不再有拥堵边，实际 " + std::to_string(stillCongested));

    // 畅通不应触发重规划（触发条件是"耗时增加 >= 20%"）
    check(!needsReplan(std::vector<std::string>{"W", "D1"}, report, 0.2),
          "畅通不构成重规划触发条件");
}

// 切片 B：仅当变化边位于当前路径上**且**增幅达到阈值时才需要重规划
void testNeedsReplanOnlyForCongestedEdgesOnTheRoute() {
    const std::vector<std::string> route = {"W", "D1", "W"};
    const double kThreshold = 0.2;

    check(needsReplan(route, reportOf({changeOf("W", "D1", 0.5)}), kThreshold),
          "路径上去程边 +50% -> 触发");
    check(needsReplan(route, reportOf({changeOf("D1", "W", 0.5)}), kThreshold),
          "路径上返程边 +50% -> 触发");
    check(!needsReplan(route, reportOf({changeOf("D1", "D2", 0.5)}), kThreshold),
          "变化边不在路径上 -> 不触发");
    check(!needsReplan({"W", "D1"}, reportOf({changeOf("D1", "W", 0.5)}), kThreshold),
          "反向边不构成路径上的一段 -> 不触发");
    check(!needsReplan(route, reportOf({changeOf("W", "D1", 0.1)}), kThreshold),
          "路径上但仅 +10% -> 不触发");
    check(needsReplan(route, reportOf({changeOf("W", "D1", 0.2)}), kThreshold),
          "恰好 +20% -> 触发（阈值为 >=）");
    check(!needsReplan(route, reportOf({}), kThreshold), "无任何变化 -> 不触发");
    check(!needsReplan({}, reportOf({changeOf("W", "D1", 0.9)}), kThreshold), "空路径 -> 不触发");
    check(needsReplan(route,
                      reportOf({changeOf("D1", "D2", 0.9), changeOf("D1", "W", 0.3)}),
                      kThreshold),
          "多变化中命中一条即触发");
}

// 切片 C：端到端 —— 拥堵使耗时上升 -> 触发 -> 重规划后路线与时刻随之改变。
// 令 minRatio == maxRatio == 0.5，所有边精确 +50%，消除随机性以便逐值断言。
void testCongestionTriggersReplanAndChangesTimes() {
    LogisticsGraph g;
    g.addNode(makeNode("W", NodeType::Warehouse, "仓库"));
    g.addNode(makeNode("D1", NodeType::Delivery, "客户1"));
    addTwoWay(g, "W", "D1", 5.0, 10.0, 4.0);

    const Vehicle v = makeVehicle("W", 1000.0, 480);
    const std::vector<logistics::Order> orders = {makeOrder("O1", "D1", 10.0, 540, 1080)};

    // 拥堵前：W->D1 耗时 10 -> raw 490，窗口 540 -> 等待 50 -> 总耗时 75
    const logistics::RoutePlan before =
        logistics::planRoute(g, v, orders, logistics::WeightType::Distance);
    check(before.status == logistics::PlanStatus::Ok, "拥堵前规划成功");
    check(fixtures::nearlyEqual(before.totalTimeMin, 70.0),
          "拥堵前总耗时 70，实际 " + std::to_string(before.totalTimeMin));
    check(before.stops.size() == 1 && before.stops[0].waitMin == 50, "拥堵前等待 50");

    // 所有边耗时 +50%（10 -> 15）
    const TrafficReport report = simulateTrafficChange(g, 1.0, 0.5, 0.5, 7u, 0.0);
    check(report.changes.size() == 2,
          "两条边都被改动，实际 " + std::to_string(report.changes.size()));

    const logistics::Edge* e = g.findEdge("W", "D1");
    check(e != nullptr && fixtures::nearlyEqual(e->timeMin, 15.0),
          "拥堵后 W->D1 耗时变为 15");
    check(e != nullptr && fixtures::nearlyEqual(e->baseTimeMin, 10.0), "基准耗时仍是 10");

    check(needsReplan(before.nodes, report, 0.2), "受影响边在路径上且 +50% -> 应触发重规划");

    // 重规划（oracle 值）：raw 495 -> 等待 45 -> arrival 540 -> departure 545
    //                       D1->W 15min -> 560，总耗时 560-480 = 80
    const logistics::RoutePlan after =
        logistics::replan(g, v, orders, "W", 480, logistics::WeightType::Distance);

    check(after.status == logistics::PlanStatus::Ok, "重规划成功");
    check(after.nodes == std::vector<std::string>{"W", "D1", "W"},
          "重规划序列不变（只有一条路可走）");
    check(after.stops.size() == 1, "1 个停靠点");
    if (after.stops.size() == 1) {
        const logistics::Stop& s = after.stops[0];
        check(s.rawArrivalMin == 495, "raw 495，实际 " + std::to_string(s.rawArrivalMin));
        check(s.waitMin == 45, "等待 45，实际 " + std::to_string(s.waitMin));
        check(s.arrivalMin == 540, "arrival 540，实际 " + std::to_string(s.arrivalMin));
    }
    check(fixtures::nearlyEqual(after.totalTimeMin, 75.0),
          "重规划后总耗时 75，实际 " + std::to_string(after.totalTimeMin));
    check(fixtures::nearlyEqual(after.totalDistanceKm, 10.0), "总距离仍为 10.0");
    check(after.totalTimeMin > before.totalTimeMin, "拥堵后总耗时必须增加");
}

// 切片 D：插入紧急订单 —— 新单优先服务，已服务订单不进入新路线。
// 图中额外放入 D0 并在剩余集合中排除它，用以验证"已服务节点不再规划"。
void testInsertUrgentOrderIsServedFirstAndServedNodesExcluded() {
    LogisticsGraph g;
    g.addNode(makeNode("W", NodeType::Warehouse, "仓库"));
    g.addNode(makeNode("D0", NodeType::Delivery, "已完成客户"));
    g.addNode(makeNode("D1", NodeType::Delivery, "客户1"));
    g.addNode(makeNode("D2", NodeType::Delivery, "客户2"));
    addTwoWay(g, "W", "D1", 1.0, 2.0, 1.0);
    addTwoWay(g, "W", "D2", 5.0, 10.0, 5.0);
    addTwoWay(g, "D1", "D2", 2.0, 4.0, 2.0);
    addTwoWay(g, "D0", "D1", 2.0, 4.0, 2.0);

    const Vehicle v = makeVehicle("W", 1000.0, 480);
    const std::vector<logistics::Order> remaining = {makeOrder("O1", "D1", 10.0, 0, 1440)};
    // 刻意传 urgent=false：按 §5.3，插入的订单必须被标记为紧急
    const logistics::Order incoming = makeOrder("O2", "D2", 10.0, 0, 1440, false);

    const logistics::InsertResult r = logistics::insertUrgentOrder(
        g, v, remaining, incoming, "W", 480, logistics::WeightType::Distance);

    check(r.warning.empty(), "窗口充裕时不应有警告，实际: " + r.warning);
    check(r.plan.status == logistics::PlanStatus::Ok, "规划成功");
    check(r.plan.stops.size() == remaining.size() + 1,
          "停靠点数 = 剩余订单数 + 新插订单数，实际 "
              + std::to_string(r.plan.stops.size()));
    check(!r.plan.stops.empty() && r.plan.stops[0].nodeId == "D2",
          "新插单即使用 urgent=false 传入也必须被优先服务，实际首个停靠 "
              + (r.plan.stops.empty() ? std::string("(无)") : r.plan.stops[0].nodeId));

    for (const logistics::Stop& s : r.plan.stops) {
        check(s.nodeId != "D0", "已服务节点 D0 不得出现在新计划的停靠点中");
    }
    for (const std::string& n : r.plan.nodes) {
        check(n != "D0", "已服务节点 D0 不得出现在完整序列中");
    }

    // oracle 值：nodes [W,D1,D2,D1,W]，总距离 6.0，总耗时 12.0
    check(r.plan.nodes == std::vector<std::string>{"W", "D1", "D2", "D1", "W"},
          "完整序列，实际 " + [&] {
              std::string s;
              for (std::size_t i = 0; i < r.plan.nodes.size(); ++i) {
                  s += (i ? " -> " : "") + r.plan.nodes[i];
              }
              return s;
          }());
    check(fixtures::nearlyEqual(r.plan.totalDistanceKm, 6.0),
          "总距离 6.0，实际 " + std::to_string(r.plan.totalDistanceKm));
    check(fixtures::nearlyEqual(r.plan.totalTimeMin, 12.0),
          "总耗时 12.0，实际 " + std::to_string(r.plan.totalTimeMin));
}

// 切片 E：新订单必然超时时给出警告，但按 D13 仍纳入规划并记录 penalty
void testInsertUrgentOrderWarnsButStillPlansWhenWindowCannotBeMet() {
    const LogisticsGraph g = makeGreedyGraph();
    const Vehicle v = makeVehicle("W", 1000.0, 480);
    const std::vector<logistics::Order> none;

    // 从 W 于 600 出发，到 D2 最早 606；窗口止 500 -> 必然超时
    const logistics::Order tight = makeOrder("O2", "D2", 10.0, 400, 500, true);
    const logistics::InsertResult bad = logistics::insertUrgentOrder(
        g, v, none, tight, "W", 600, logistics::WeightType::Distance);

    check(!bad.warning.empty(), "必然超时必须给出警告");
    check(bad.plan.status == logistics::PlanStatus::Ok,
          "D13 口径为超时不弃，仍应生成路线");
    bool planned = false;
    for (const logistics::Stop& s : bad.plan.stops) {
        if (s.nodeId == "D2") {
            planned = true;
        }
    }
    check(planned, "超时的紧急订单仍必须纳入路线");
    // oracle 值：raw 606 -> arrival 606，窗口止 500 -> penalty 106
    check(bad.plan.totalPenaltyMin == 106,
          "penalty 106，实际 " + std::to_string(bad.plan.totalPenaltyMin));
    check(fixtures::nearlyEqual(bad.plan.totalTimeMin, 12.0),
          "总耗时 12.0，实际 " + std::to_string(bad.plan.totalTimeMin));

    // 对照：窗口充裕 -> 无警告，无 penalty
    const logistics::Order wide = makeOrder("O3", "D2", 10.0, 0, 1440, true);
    const logistics::InsertResult ok = logistics::insertUrgentOrder(
        g, v, none, wide, "W", 600, logistics::WeightType::Distance);
    check(ok.warning.empty(), "窗口充裕时不应有警告，实际: " + ok.warning);
    check(ok.plan.totalPenaltyMin == 0, "窗口充裕时 penalty 为 0");
}

// 跨种子不变量：路况模拟是随机的，单一种子只覆盖一条随机路径。
// 这里遍历多个种子，验证「改动条数」「增幅范围」「baseTimeMin 不变」
// 这些不变量对**任意种子**都成立；并确认不同种子确实给出不同结果（随机性真实存在）。
// 种子集合固定，因此本测试自身是确定性的、不会 flaky。
void testTrafficInvariantsHoldAcrossSeeds() {
    const unsigned int kSeeds = 20;
    std::vector<std::string> signatures;
    bool countOk = true;
    bool rangeOk = true;
    bool baseOk = true;

    for (unsigned int seed = 1; seed <= kSeeds; ++seed) {
        LogisticsGraph g = makeGreedyGraph();
        const EdgeSnapshot before = snapshot(g);
        const TrafficReport r = simulateTrafficChange(g, 0.5, 0.2, 0.5, seed, 0.0);

        if (r.changes.size() != 3) {
            countOk = false;
        }
        for (const logistics::TrafficChange& c : r.changes) {
            if (c.increaseRatio < 0.2 - 1e-9 || c.increaseRatio > 0.5 + 1e-9) {
                rangeOk = false;
            }
        }
        // baseTimeMin 在任何种子下都不得被改动
        std::size_t i = 0;
        for (const logistics::Edge& e : g.edges()) {
            if (!fixtures::nearlyEqual(e.baseTimeMin, before.base.at(key(e.fromId, e.toId)))) {
                baseOk = false;
            }
            ++i;
        }

        // 用改动集合的排序签名判断不同种子是否给出不同结果
        std::vector<std::string> picked;
        for (const logistics::TrafficChange& c : r.changes) {
            picked.push_back(key(c.fromId, c.toId));
        }
        std::sort(picked.begin(), picked.end());
        std::string sig;
        for (const std::string& p : picked) {
            sig += p + ";";
        }
        bool seen = false;
        for (const std::string& s : signatures) {
            if (s == sig) {
                seen = true;
            }
        }
        if (!seen) {
            signatures.push_back(sig);
        }
    }

    check(countOk, "任意种子下改动条数恒为 round(0.5*6) = 3");
    check(rangeOk, "任意种子下增幅恒落在 [0.2, 0.5]");
    check(baseOk, "任意种子下 baseTimeMin 恒不被改动");
    check(signatures.size() >= 2,
          "不同种子应给出不同改动集合（证明随机性真实存在），实际 "
              + std::to_string(signatures.size()) + " 种");
}

} // namespace

int main() {
    testTrafficChangeCountMagnitudeAndBasePreserved();
    testTrafficRatioOneTouchesEveryEdge();
    testTrafficIsDeterministicForSameSeed();
    testTrafficCanAlsoBecomeClear();
    testNeedsReplanOnlyForCongestedEdgesOnTheRoute();
    testCongestionTriggersReplanAndChangesTimes();
    testInsertUrgentOrderIsServedFirstAndServedNodesExcluded();
    testInsertUrgentOrderWarnsButStillPlansWhenWindowCannotBeMet();
    testTrafficInvariantsHoldAcrossSeeds();
    return testutil::summarize("traffic_tests");
}
