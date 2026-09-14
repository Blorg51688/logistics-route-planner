// B5 动态增删（模拟新客户 / 道路封闭）与权重估算的行为测试。
//
// 说明：本文件是在实现之后补写的（偏离了本项目一贯的"测试先行"），
// 因此额外做了变异验证，证明这些断言确实能失败、不是空跑。
#include <cmath>
#include <string>
#include <vector>

#include "core/GraphEdit.h"
#include "core/Random.h"
#include "graph_fixtures.h"
#include "test_util.h"

using logistics::Edge;
using logistics::LogisticsGraph;
using logistics::Node;
using logistics::NodeType;
using logistics::Rng;
using logistics::addRandomCustomer;
using logistics::closeRandomRoad;
using logistics::makeSyntheticEdge;
using testutil::check;

namespace {

using fixtures::addTwoWay;
using fixtures::makeNode;

// 坐标必须有区分度：权重估算依赖坐标差，全为 (0,0) 会让距离退化为 0
LogisticsGraph makeSmallNetwork() {
    LogisticsGraph g;
    Node w = makeNode("W", NodeType::Warehouse, "仓库");
    w.x = 0;    w.y = 0;
    Node d1 = makeNode("D1", NodeType::Delivery, "客户1");
    d1.x = 100; d1.y = 0;
    Node d2 = makeNode("D2", NodeType::Delivery, "客户2");
    d2.x = 300; d2.y = 200;
    g.addNode(w);
    g.addNode(d1);
    g.addNode(d2);
    addTwoWay(g, "W", "D1", 1.0, 2.0, 1.2);
    addTwoWay(g, "W", "D2", 5.0, 10.0, 6.0);
    addTwoWay(g, "D1", "D2", 2.0, 4.0, 2.4);
    return g;
}


// 测试用的可达性判定（有向图 BFS）
bool reaches(const LogisticsGraph& g, const std::string& from, const std::string& to) {
    std::vector<std::string> seen;
    std::vector<std::string> queue;
    seen.push_back(from);
    queue.push_back(from);
    for (std::size_t head = 0; head < queue.size(); ++head) {
        const std::string cur = queue[head];
        if (cur == to) {
            return true;
        }
        for (const Edge& e : g.outEdges(cur)) {
            bool known = false;
            for (const std::string& s : seen) {
                if (s == e.toId) { known = true; break; }
            }
            if (!known) { seen.push_back(e.toId); queue.push_back(e.toId); }
        }
    }
    return false;
}

double distanceBetween(const Node& a, const Node& b) {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

std::vector<std::string> idsOf(const LogisticsGraph& g) {
    std::vector<std::string> ids;
    for (const Node& n : g.nodes()) {
        ids.push_back(n.id);
    }
    return ids;
}

// 切片 1：新客户必须插入成功、为配送点、连到**最近的既有节点**（双向），
// 且相同种子给出完全相同的结果。
//
// 注意为什么要跨多个种子：若只在单一种子上验证，"最近节点"很容易恰好等于
// 实现可能误选的某个固定节点（例如 nodes().front()），断言就会失去区分力。
// 跨 30 个种子后，任何固定选择都会在多数种子上偏离最近节点而被抓到。
void testAddRandomCustomerProperties() {
    const LogisticsGraph reference = makeSmallNetwork();
    const std::vector<std::string> beforeIds = idsOf(reference);

    const unsigned int kSeeds = 30;
    bool countOk = true;
    bool typeOk = true;
    bool nearOk = true;
    bool twoWayOk = true;
    bool weightOk = true;
    bool outEdgeOk = true;

    for (unsigned int seed = 1; seed <= kSeeds; ++seed) {
        LogisticsGraph g = makeSmallNetwork();
        Rng rng(seed);
        const std::string newId = addRandomCustomer(g, rng);

        if (newId.empty() || g.nodeCount() != reference.nodeCount() + 1) {
            countOk = false;
            continue;
        }
        const Node* created = g.findNode(newId);
        if (created == nullptr) {
            typeOk = false;
            continue;
        }
        if (created->type != NodeType::Delivery) {
            typeOk = false;
        }
        if (g.outEdges(newId).size() != 1) {
            outEdgeOk = false;
        }

        // 用结果自身暴力求"最近的既有节点"，不依赖被测代码的选择逻辑
        const Node* nearest = nullptr;
        double best = 1e18;
        for (const std::string& id : beforeIds) {
            const Node* n = g.findNode(id);
            if (n == nullptr) {
                continue;
            }
            const double d = distanceBetween(*created, *n);
            if (d < best) {
                best = d;
                nearest = n;
            }
        }
        if (nearest == nullptr) {
            nearOk = false;
            continue;
        }
        if (g.findEdge(newId, nearest->id) == nullptr
            || g.findEdge(nearest->id, newId) == nullptr) {
            twoWayOk = false;
        }
        const Edge* e = g.findEdge(newId, nearest->id);
        if (e == nullptr || e->distanceKm <= 0.0 || e->timeMin <= 0.0 || e->costYuan <= 0.0) {
            weightOk = false;
        }
    }

    check(countOk, "30 个种子下：节点数均 +1 且返回非空 ID");
    check(typeOk, "30 个种子下：新节点类型均为配送点");
    check(outEdgeOk, "30 个种子下：新客户均恰好一条出边");
    check(nearOk, "30 个种子下：均能确定最近既有节点");
    check(twoWayOk, "30 个种子下：均与**最近**节点双向连通");
    check(weightOk, "30 个种子下：新边距离/耗时/成本均为正");

    // 确定性：同种子必得相同结果
    LogisticsGraph a = makeSmallNetwork();
    LogisticsGraph b = makeSmallNetwork();
    Rng r1(2024u);
    Rng r2(2024u);
    const std::string idA = addRandomCustomer(a, r1);
    const std::string idB = addRandomCustomer(b, r2);
    const Node* na = a.findNode(idA);
    const Node* nb = b.findNode(idB);
    check(idA == idB, "同种子产生相同的节点 ID");
    check(na != nullptr && nb != nullptr && fixtures::nearlyEqual(na->x, nb->x)
              && fixtures::nearlyEqual(na->y, nb->y),
          "同种子产生相同的坐标");
}

// 切片 1 边界：空图无法插入新客户
void testAddRandomCustomerOnEmptyGraph() {
    LogisticsGraph empty;
    Rng rng(1u);
    check(addRandomCustomer(empty, rng).empty(), "空图返回空 ID");
    check(empty.nodeCount() == 0, "空图仍为空");
}

// 切片 2：道路封闭必须双向删除，且返回被封闭的道路描述
void testCloseRandomRoadRemovesBothDirections() {
    LogisticsGraph g = makeSmallNetwork();
    const std::size_t before = g.edgeCount();
    check(before == 6, "夹具 6 条有向边");

    Rng rng(7u);
    const std::string closed = closeRandomRoad(g, rng);

    check(!closed.empty(), "应返回被封闭道路的描述");
    const std::size_t sep = closed.find("->");
    check(sep != std::string::npos, "描述形如 from->to");
    if (sep == std::string::npos) {
        return;
    }
    const std::string from = closed.substr(0, sep);
    const std::string to = closed.substr(sep + 2);

    check(g.findEdge(from, to) == nullptr, "正向边已删除");
    check(g.findEdge(to, from) == nullptr, "反向边一并删除（道路封闭是双向的）");
    check(g.edgeCount() == before - 2, "边数减少 2");

    // 闭环性：反复封闭会终止，且最终拒绝继续（剩下的都是桥）
    int guard = 0;
    while (!closeRandomRoad(g, rng).empty() && guard < 100) {
        ++guard;
    }
    check(guard < 100, "反复封闭能够终止，不会死循环");
    check(g.edgeCount() > 0, "剩下的道路都是桥，应拒绝继续封闭");
}


// 切片 2：道路封闭**不得切断网络**（不封桥），否则演示会陷入无法恢复的不可行状态。
// 这是实际运行 demo 时暴露出来的问题：封闭 T02<->D15 后 D15 成为孤岛。
void testCloseRandomRoadNeverBreaksConnectivity() {
    const unsigned int kSeeds = 20;
    bool noBridgeClosed = true;
    bool reachableAfterClose = true;

    for (unsigned int seed = 1; seed <= kSeeds; ++seed) {
        LogisticsGraph g = makeSmallNetwork();
        Rng rng(seed);
        const std::string closed = closeRandomRoad(g, rng);
        if (closed.empty()) {
            continue;
        }
        const std::size_t sep = closed.find("->");
        if (sep == std::string::npos) {
            noBridgeClosed = false;
            continue;
        }
        const std::string from = closed.substr(0, sep);
        const std::string to = closed.substr(sep + 2);

        // 封闭后两端必须仍能互相到达
        if (!reaches(g, from, to) || !reaches(g, to, from)) {
            reachableAfterClose = false;
        }
        // 且所有节点必须仍与其余部分连通（不存在孤岛）
        for (const Node& n : g.nodes()) {
            bool linked = false;
            for (const Node& other : g.nodes()) {
                if (n.id != other.id && (reaches(g, n.id, other.id) || reaches(g, other.id, n.id))) {
                    linked = true;
                    break;
                }
            }
            if (!linked) {
                noBridgeClosed = false;
            }
        }
    }

    check(noBridgeClosed, "20 个种子下：封闭后不存在孤立节点");
    check(reachableAfterClose, "20 个种子下：封闭后两端仍能互相到达（未封桥）");
}

// 切片 2 边界：无边的图返回空串
void testCloseRandomRoadOnEmptyGraph() {
    LogisticsGraph g;
    g.addNode(makeNode("W", NodeType::Warehouse, "仓库"));
    Rng rng(3u);
    check(closeRandomRoad(g, rng).empty(), "无边可删时返回空串");
}

// 切片 3：手工加边时的权重估算必须与图中既有边的口径一致
void testMakeSyntheticEdgeUsesGraphScale() {
    const LogisticsGraph g = makeSmallNetwork();

    // 夹具中每条边的 cost/dist 恒为 1.2，time/dist 恒为 2.0
    Edge e;
    check(makeSyntheticEdge(g, "W", "D2", e), "端点存在时可估算");
    check(e.fromId == "W" && e.toId == "D2", "端点填入正确");
    check(e.distanceKm > 0.0, "距离为正");
    check(std::fabs(e.costYuan / e.distanceKm - 1.2) < 1e-6,
          "成本/距离比与既有边一致（1.2），实际 "
              + std::to_string(e.costYuan / e.distanceKm));
    check(std::fabs(e.timeMin / e.distanceKm - 2.0) < 1e-6,
          "耗时/距离比与既有边一致（2.0），实际 "
              + std::to_string(e.timeMin / e.distanceKm));
    check(std::fabs(e.baseTimeMin - e.timeMin) < 1e-9, "baseTimeMin 与 timeMin 一致");
}

// 切片 3 边界：端点不存在 / 坐标重合
void testMakeSyntheticEdgeBoundaries() {
    const LogisticsGraph g = makeSmallNetwork();
    Edge e;
    check(!makeSyntheticEdge(g, "W", "NOPE", e), "终点不存在时返回 false");
    check(!makeSyntheticEdge(g, "NOPE", "W", e), "起点不存在时返回 false");

    LogisticsGraph same;
    same.addNode(makeNode("A", NodeType::Delivery, "A"));
    same.addNode(makeNode("B", NodeType::Delivery, "B"));   // 坐标均为 (0,0)
    check(!makeSyntheticEdge(same, "A", "B", e), "坐标重合时返回 false（无法给出有意义权重）");
}

// Rng 的种子扩散。
//
// 这里曾经有过**真实缺陷**：直接把小整数种子当作 xorshift 状态时，前几个输出的
// 高位几乎不变——实测连续 30 个种子下 nextUnit() 的首个结果几乎相同，导致
// 「随机位置」退化到包围盒的同一个角落（诊断脚本显示 x 坐标只差 0.2）。
// 该缺陷是在"变异测试没被抓到"的追查中发现的，因此专门加这条回归守卫。
void testRngSeedDiffusion() {
    const unsigned int kSeeds = 20;
    double lowest = 1e18;
    double highest = -1e18;
    for (unsigned int seed = 1; seed <= kSeeds; ++seed) {
        Rng rng(seed);
        const double first = rng.nextUnit();
        if (first < lowest) {
            lowest = first;
        }
        if (first > highest) {
            highest = first;
        }
    }
    check(highest - lowest > 0.5,
          "连续种子的首个随机数必须有足够区分度，实际跨度 "
              + std::to_string(highest - lowest));

    // 扩散不得破坏"同种子完全可复现"这一前提
    Rng a(123u);
    Rng b(123u);
    bool same = true;
    for (int i = 0; i < 50; ++i) {
        if (!fixtures::nearlyEqual(a.nextUnit(), b.nextUnit())) {
            same = false;
        }
    }
    check(same, "同种子的 50 次连续取值完全一致");
}

} // namespace

int main() {
    testAddRandomCustomerProperties();
    testAddRandomCustomerOnEmptyGraph();
    testCloseRandomRoadRemovesBothDirections();
    testCloseRandomRoadOnEmptyGraph();
    testCloseRandomRoadNeverBreaksConnectivity();
    testMakeSyntheticEdgeUsesGraphScale();
    testMakeSyntheticEdgeBoundaries();
    testRngSeedDiffusion();
    return testutil::summarize("graphedit_tests");
}
