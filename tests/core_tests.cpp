// 核心层最小测试：不依赖 Qt，不引入任何测试框架。
// 覆盖三类用例：正确运行 / 错误输入 / 边界数据。
#include <cmath>
#include <cstdio>
#include <string>

#include "core/LogisticsGraph.h"
#include "test_util.h"

using logistics::Edge;
using logistics::LogisticsGraph;
using logistics::Node;
using logistics::NodeType;
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
              double km, double min, double yuan) {
    Edge e;
    e.fromId = from;
    e.toId = to;
    e.distanceKm = km;
    e.timeMin = min;
    e.costYuan = yuan;
    return e;
}

// N01(仓库) -> N02(配送点) -> N03(中转站)
LogisticsGraph makeSampleGraph() {
    LogisticsGraph g;
    check(g.addNode(makeNode("N01", NodeType::Warehouse, "中央仓库")), "addNode N01");
    check(g.addNode(makeNode("N02", NodeType::Delivery, "客户A")), "addNode N02");
    check(g.addNode(makeNode("N03", NodeType::Transit, "中转站B")), "addNode N03");
    check(g.addEdge(makeEdge("N01", "N02", 5.2, 12, 8.0)), "addEdge N01->N02");
    check(g.addEdge(makeEdge("N02", "N03", 3.0, 7, 4.5)), "addEdge N02->N03");
    return g;
}

void testNodeAddAndQuery() {
    LogisticsGraph g = makeSampleGraph();
    check(g.nodeCount() == 3, "nodeCount == 3");

    const Node* n = g.findNode("N02");
    check(n != nullptr, "findNode 命中");
    check(n != nullptr && n->type == NodeType::Delivery, "N02 类型为配送点");
    check(n != nullptr && n->name == "客户A", "N02 名称正确");
    check(g.findNode("N99") == nullptr, "findNode 未命中返回 nullptr");

    // 错误输入：ID 重复 / ID 为空
    check(!g.addNode(makeNode("N02", NodeType::Delivery, "重复")), "重复 ID 被拒绝");
    check(g.nodeCount() == 3, "重复 ID 未改变节点数");
    check(!g.addNode(makeNode("", NodeType::Delivery, "空")), "空 ID 被拒绝");
}

void testEdgeAddAndQuery() {
    LogisticsGraph g = makeSampleGraph();
    check(g.edgeCount() == 2, "edgeCount == 2");

    const Edge* e = g.findEdge("N01", "N02");
    check(e != nullptr, "findEdge 命中");
    check(e != nullptr && nearlyEqual(e->distanceKm, 5.2), "距离正确");
    check(e != nullptr && nearlyEqual(e->timeMin, 12.0), "耗时正确");
    check(e != nullptr && nearlyEqual(e->costYuan, 8.0), "成本正确");
    check(e != nullptr && nearlyEqual(e->baseTimeMin, 12.0), "baseTimeMin 自动固化为当前耗时");
    check(e != nullptr && !e->congested, "初始未拥堵");

    check(g.findEdge("N02", "N01") == nullptr, "反向边不存在（有向图）");

    // 错误输入：端点不存在 / 重复边
    check(!g.addEdge(makeEdge("N01", "N99", 1, 1, 1)), "指向不存在节点的边被拒绝");
    check(!g.addEdge(makeEdge("N99", "N01", 1, 1, 1)), "起点不存在的边被拒绝");
    check(!g.addEdge(makeEdge("N01", "N02", 9.9, 9.9, 9.9)), "重复 (from,to) 被拒绝");
    check(g.edgeCount() == 2, "非法加边未改变边数");
}

void testOutEdgesAndRemoval() {
    LogisticsGraph g = makeSampleGraph();
    check(g.outEdges("N01").size() == 1, "N01 出边数 == 1");
    check(g.outEdges("N03").empty(), "N03 无出边");
    check(g.outEdges("N99").empty(), "不存在节点的出边为空且不崩溃");

    check(g.removeEdge("N01", "N02"), "removeEdge 成功");
    check(g.edgeCount() == 1, "删边后 edgeCount == 1");
    check(!g.removeEdge("N01", "N02"), "重复删边返回 false");

    // 不变量：删除节点必须同步删除其出边与入边
    LogisticsGraph h = makeSampleGraph();
    check(h.removeNode("N02"), "removeNode N02 成功");
    check(h.nodeCount() == 2, "删点后 nodeCount == 2");
    check(h.edgeCount() == 0, "N02 的出边与入边都被删除");
    check(!h.removeNode("N02"), "重复删点返回 false");
    // 删点后下标重建：剩余节点仍可按 ID 查到，且无残留边
    check(h.findNode("N01") != nullptr, "删点后 N01 仍可查");
    check(h.findNode("N03") != nullptr, "删点后 N03 仍可查");
    check(h.outEdges("N01").empty(), "删点后 N01 无残留出边");
}

void testAdjacencyListString() {
    LogisticsGraph g = makeSampleGraph();
    const std::string s = g.toAdjacencyListString();
    check(s.find("N01") != std::string::npos, "邻接表包含 N01");
    check(s.find("warehouse") != std::string::npos, "邻接表包含节点类型文本");
    check(s.find("N01(warehouse,中央仓库): -> N02") != std::string::npos,
          "邻接表包含边 N01 -> N02");
    check(s.find("5.2") != std::string::npos, "邻接表包含距离权重");
}

void testEmptyGraphBoundary() {
    LogisticsGraph g;
    check(g.nodeCount() == 0, "空图节点数为 0");
    check(g.edgeCount() == 0, "空图边数为 0");
    check(g.edges().empty(), "空图无边可遍历");
    check(g.findNode("N01") == nullptr, "空图查询返回 nullptr");
    check(g.findEdge("N01", "N02") == nullptr, "空图查边返回 nullptr");
    check(!g.removeNode("N01"), "空图删点返回 false");
    check(!g.removeEdge("N01", "N02"), "空图删边返回 false");
    check(!g.toAdjacencyListString().empty(), "空图邻接表输出不崩溃");
}

} // namespace

int main() {
    testNodeAddAndQuery();
    testEdgeAddAndQuery();
    testOutEdgesAndRemoval();
    testAdjacencyListString();
    testEmptyGraphBoundary();

    return testutil::summarize("core_tests");
}
