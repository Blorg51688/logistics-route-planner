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
    check(!g.toAdjacencyMatrixString().empty(), "空图邻接矩阵输出不崩溃");
}

// ---- 邻接矩阵 ----
// 断言单元格的**语义**（第 i 行第 j 列是否等于边 weights[i-1][j-1]），
// 而不是空格排版 —— 否则任何对齐格式微调都会让测试变红。

std::vector<std::vector<std::string>> splitGrid(const std::string& text) {
    std::vector<std::vector<std::string>> rows;
    std::size_t start = 0;
    while (start <= text.size()) {
        const std::size_t nl = text.find('\n', start);
        const std::string line =
            (nl == std::string::npos) ? text.substr(start) : text.substr(start, nl - start);
        std::vector<std::string> cells;
        std::size_t i = 0;
        while (i < line.size()) {
            while (i < line.size() && (line[i] == ' ' || line[i] == '\t' || line[i] == '\r')) {
                ++i;
            }
            const std::size_t b = i;
            while (i < line.size() && !(line[i] == ' ' || line[i] == '\t' || line[i] == '\r')) {
                ++i;
            }
            if (i > b) {
                cells.push_back(line.substr(b, i - b));
            }
        }
        if (!cells.empty()) {
            rows.push_back(cells);
        }
        if (nl == std::string::npos) {
            break;
        }
        start = nl + 1;
    }
    return rows;
}

void testAdjacencyMatrixCellsMatchEdges() {
    const LogisticsGraph g = makeSampleGraph();   // N01 -> N02 -> N03

    const std::vector<std::vector<std::string>> grid =
        splitGrid(g.toAdjacencyMatrixString(logistics::WeightType::Distance));

    check(grid.size() == 4, "矩阵应有 1 行表头 + 3 行数据，实际 "
              + std::to_string(grid.size()));
    for (std::size_t r = 0; r < grid.size(); ++r) {
        check(grid[r].size() == 4, "第 " + std::to_string(r) + " 行应有 1 角标 + 3 列，实际 "
                  + std::to_string(grid[r].size()));
    }
    if (grid.size() != 4) {
        return;
    }

    // 表头为节点 ID，顺序与 nodes() 一致；行标同理
    check(grid[0][1] == "N01" && grid[0][2] == "N02" && grid[0][3] == "N03",
          "表头为节点 ID 顺序");
    check(grid[1][0] == "N01" && grid[2][0] == "N02" && grid[3][0] == "N03",
          "行标为节点 ID 顺序");

    // 单元格语义：grid[r][c] 对应 nodes[r-1] -> nodes[c-1]
    check(grid[1][2] == "5.2", "N01->N02 距离 5.2，实际 " + grid[1][2]);
    check(grid[2][3] == "3", "N02->N03 距离 3，实际 " + grid[2][3]);
    // 对角线（无自环）与反向边（有向图）都应为缺边占位
    check(grid[1][1] == "-", "对角 N01->N01 无自环");
    check(grid[2][2] == "-", "对角 N02->N02 无自环");
    check(grid[3][3] == "-", "对角 N03->N03 无自环");
    check(grid[2][1] == "-", "N02->N01 不存在（有向）");
    check(grid[3][1] == "-", "N03->N01 不存在");
    check(grid[1][3] == "-", "N01->N03 不存在");
}

void testAdjacencyMatrixReflectsChosenWeight() {
    const LogisticsGraph g = makeSampleGraph();
    const std::vector<std::vector<std::string>> byDistance =
        splitGrid(g.toAdjacencyMatrixString(logistics::WeightType::Distance));
    const std::vector<std::vector<std::string>> byTime =
        splitGrid(g.toAdjacencyMatrixString(logistics::WeightType::Time));
    const std::vector<std::vector<std::string>> byCost =
        splitGrid(g.toAdjacencyMatrixString(logistics::WeightType::Cost));

    check(byDistance.size() == 4 && byTime.size() == 4 && byCost.size() == 4,
          "三种权重维度都产出 4 行");
    if (byDistance.size() != 4) {
        return;
    }
    check(byDistance[1][2] == "5.2", "距离矩阵 N01->N02 为 5.2");
    check(byTime[1][2] == "12", "耗时矩阵 N01->N02 为 12，实际 " + byTime[1][2]);
    check(byCost[1][2] == "8", "成本矩阵 N01->N02 为 8，实际 " + byCost[1][2]);
}

void testEmptyGraphAdjacencyMatrix() {
    const LogisticsGraph g;
    const std::vector<std::vector<std::string>> grid =
        splitGrid(g.toAdjacencyMatrixString(logistics::WeightType::Distance));
    // 空图：只有 1 行表头（含角标），且不崩溃
    check(grid.size() <= 1, "空图矩阵不超过 1 行，实际 " + std::to_string(grid.size()));
}

} // namespace

int main() {
    testNodeAddAndQuery();
    testEdgeAddAndQuery();
    testOutEdgesAndRemoval();
    testAdjacencyListString();
    testEmptyGraphBoundary();
    testAdjacencyMatrixCellsMatchEdges();
    testAdjacencyMatrixReflectsChosenWeight();
    testEmptyGraphAdjacencyMatrix();

    return testutil::summarize("core_tests");
}
