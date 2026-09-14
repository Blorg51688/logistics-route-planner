// 手写 Dijkstra 的行为测试。
// 期望值全部来自手工推导的字面量，不复用实现中的任何计算。
//
// 测试用图（有向）：
//   A -> B  距离 1.0  耗时 10.0  成本 5.0
//   A -> C  距离 4.0  耗时  3.0  成本 2.0
//   B -> C  距离 1.0  耗时 10.0  成本 5.0
//   B -> D  距离 1.0  耗时  1.0  成本 5.0
//   C -> D  距离 1.0  耗时  1.0  成本 2.0
//   E 为孤岛节点，无任何入边
#include <cmath>
#include <string>
#include <vector>

#include "core/Dijkstra.h"
#include "core/LogisticsGraph.h"
#include "test_util.h"

using logistics::Edge;
using logistics::LogisticsGraph;
using logistics::Node;
using logistics::NodeType;
using logistics::PathResult;
using logistics::WeightType;
using logistics::shortestPath;
using testutil::check;

namespace {

bool nearlyEqual(double a, double b) {
    return std::fabs(a - b) < 1e-9;
}

Node makeNode(const std::string& id, NodeType type, const std::string& name) {
    Node n;
    n.id = id;
    n.type = type;
    n.name = name;
    return n;
}

Edge makeEdge(const std::string& from, const std::string& to,
              double km, double minutes, double yuan) {
    Edge e;
    e.fromId = from;
    e.toId = to;
    e.distanceKm = km;
    e.timeMin = minutes;
    e.costYuan = yuan;
    return e;
}

LogisticsGraph makeGraph() {
    LogisticsGraph g;
    g.addNode(makeNode("A", NodeType::Warehouse, "仓库A"));
    g.addNode(makeNode("B", NodeType::Delivery, "客户B"));
    g.addNode(makeNode("C", NodeType::Transit, "中转站C"));
    g.addNode(makeNode("D", NodeType::Delivery, "客户D"));
    g.addNode(makeNode("E", NodeType::Delivery, "孤岛E"));

    g.addEdge(makeEdge("A", "B", 1.0, 10.0, 5.0));
    g.addEdge(makeEdge("A", "C", 4.0, 3.0, 2.0));
    g.addEdge(makeEdge("B", "C", 1.0, 10.0, 5.0));
    g.addEdge(makeEdge("B", "D", 1.0, 1.0, 5.0));
    g.addEdge(makeEdge("C", "D", 1.0, 1.0, 2.0));
    return g;
}

std::string joinPath(const std::vector<std::string>& nodes) {
    std::string s;
    for (std::size_t i = 0; i < nodes.size(); ++i) {
        if (i > 0) {
            s += " -> ";
        }
        s += nodes[i];
    }
    return s;
}

// 切片 3：按距离权重求 A -> D 的最短路
void testShortestDistancePath() {
    const LogisticsGraph g = makeGraph();
    const PathResult r = shortestPath(g, "A", "D", WeightType::Distance);

    check(r.found, "A->D 按距离可达");
    // 手算：A->B->D = 1.0 + 1.0 = 2.0，优于 A->C->D = 5.0 与 A->B->C->D = 3.0
    check(r.nodes == std::vector<std::string>{"A", "B", "D"},
          "距离最短路为 A->B->D，实际 " + joinPath(r.nodes));
    check(nearlyEqual(r.totalWeight, 2.0),
          "总距离为 2.0，实际 " + std::to_string(r.totalWeight));
}

// 切片 4：目标不可达时必须明确报告，而不是让 INF 伪装成真实距离
void testUnreachableTarget() {
    const LogisticsGraph g = makeGraph();
    const PathResult r = shortestPath(g, "A", "E", WeightType::Distance);

    check(!r.found, "A->E 不可达，found 为 false");
    check(r.nodes.empty(), "不可达时 nodes 为空");
}

// 切片 5：起点等于终点 -> 单元素路径、权重 0
void testSourceEqualsTarget() {
    const LogisticsGraph g = makeGraph();
    const PathResult r = shortestPath(g, "B", "B", WeightType::Distance);

    check(r.found, "B->B 视为可达");
    check(r.nodes == std::vector<std::string>{"B"},
          "起终点相同返回单元素路径，实际 " + joinPath(r.nodes));
    check(nearlyEqual(r.totalWeight, 0.0),
          "权重为 0，实际 " + std::to_string(r.totalWeight));
}

// 切片 6：同一张图换权重维度应得到不同最优路径，
// 证明"最短距离"与"最低成本"共用一份实现而非各写一份。
void testWeightTypeSelectsDifferentPaths() {
    const LogisticsGraph g = makeGraph();

    const PathResult byDistance = shortestPath(g, "A", "D", WeightType::Distance);
    // 手算：A->B->D = 1.0+1.0 = 2.0（优于 A->B->C->D = 3.0）
    check(byDistance.found && byDistance.nodes == std::vector<std::string>{"A", "B", "D"},
          "距离最优路径为 A->B->D，实际 " + joinPath(byDistance.nodes));
    check(nearlyEqual(byDistance.totalWeight, 2.0), "距离最优值 2.0");

    const PathResult byTime = shortestPath(g, "A", "D", WeightType::Time);
    // 手算：A->C->D = 3.0+1.0 = 4.0（优于 A->B->D = 11.0）
    check(byTime.found && byTime.nodes == std::vector<std::string>{"A", "C", "D"},
          "耗时最优路径为 A->C->D，实际 " + joinPath(byTime.nodes));
    check(nearlyEqual(byTime.totalWeight, 4.0), "耗时最优值 4.0");

    const PathResult byCost = shortestPath(g, "A", "D", WeightType::Cost);
    // 手算：A->C->D = 2.0+2.0 = 4.0（优于 A->B->D = 10.0）
    check(byCost.found && byCost.nodes == std::vector<std::string>{"A", "C", "D"},
          "成本最优路径为 A->C->D，实际 " + joinPath(byCost.nodes));
    check(nearlyEqual(byCost.totalWeight, 4.0), "成本最优值 4.0");

    check(byDistance.nodes != byTime.nodes,
          "距离与耗时两种策略确实给出不同路径（一份实现服务两种策略）");
}

// 切片 7：同一节点被重复入堆时，结果仍取最短。
// 求 A->C 时，C 先以 4.0 入堆；随后经 A->B->C 发现 2.0，于是 C 以 2.0 再次入堆。
// 本测试断言最终取 2.0 而非旧条目的 4.0。
//
// 变异测试结论（诚实记录）：删掉实现中"跳过过期条目"的守卫后，本测试**依然通过**。
// 原因是堆总先弹出更小的 key，节点的正确条目必定先于旧条目被处理，
// 旧条目算出的候选值不可能更小。因此该守卫是性能优化，不是正确性必需。
// 本测试真正的价值，是守住"重复入堆不产生错误结果"这一行为契约。
void testRepeatedInsertionStillYieldsOptimalResult() {
    const LogisticsGraph g = makeGraph();
    const PathResult r = shortestPath(g, "A", "C", WeightType::Distance);

    check(r.found, "A->C 可达");
    // 手算：A->B->C = 1.0+1.0 = 2.0，优于直达 A->C = 4.0
    check(r.nodes == std::vector<std::string>{"A", "B", "C"},
          "应取 A->B->C，实际 " + joinPath(r.nodes));
    check(nearlyEqual(r.totalWeight, 2.0),
          "权重应为 2.0（而非旧条目 4.0），实际 " + std::to_string(r.totalWeight));

    // 同一张图上重复求解必须给出相同结果（堆状态被正确清理，无跨调用污染）
    const PathResult again = shortestPath(g, "A", "C", WeightType::Distance);
    check(again.nodes == r.nodes && nearlyEqual(again.totalWeight, r.totalWeight),
          "重复求解结果一致");
}

} // namespace

int main() {
    testShortestDistancePath();
    testUnreachableTarget();
    testSourceEqualsTarget();
    testWeightTypeSelectsDifferentPaths();
    testRepeatedInsertionStillYieldsOptimalResult();
    return testutil::summarize("dijkstra_tests");
}
