#include "core/GraphEdit.h"

#include <cmath>
#include <cstddef>
#include <cstdio>
#include <vector>

namespace logistics {

namespace {

// 从既有边推算的换算比例：坐标单位 -> (公里, 分钟, 元)
struct WeightScale {
    double kmPerUnit = 0.05;      // 没有边可参考时的兜底值
    double minPerKm = 1.5;
    double yuanPerKm = 1.2;
};

WeightScale estimateScale(const LogisticsGraph& graph) {
    WeightScale scale;
    double unitSum = 0.0;
    double kmSum = 0.0;
    double minSum = 0.0;
    double yuanSum = 0.0;

    for (const Edge& edge : graph.edges()) {
        const Node* from = graph.findNode(edge.fromId);
        const Node* to = graph.findNode(edge.toId);
        if (from == nullptr || to == nullptr) {
            continue;
        }
        const double dx = from->x - to->x;
        const double dy = from->y - to->y;
        const double unit = std::sqrt(dx * dx + dy * dy);
        if (unit <= 1e-9) {
            continue;
        }
        unitSum += unit;
        kmSum += edge.distanceKm;
        minSum += edge.timeMin;
        yuanSum += edge.costYuan;
    }

    if (unitSum > 1e-9 && kmSum > 1e-9) {
        scale.kmPerUnit = kmSum / unitSum;
        if (minSum > 1e-9) {
            scale.minPerKm = minSum / kmSum;
        }
        if (yuanSum > 1e-9) {
            scale.yuanPerKm = yuanSum / kmSum;
        }
    }
    return scale;
}

// 在"跳过 from<->to 这条双向路"的前提下，检查 start 能否到达 goal
bool reachesSkipping(const LogisticsGraph& graph, const std::string& start,
                     const std::string& goal, const std::string& skipFrom,
                     const std::string& skipTo) {
    std::vector<std::string> seen;
    std::vector<std::string> queue;
    seen.push_back(start);
    queue.push_back(start);

    for (std::size_t head = 0; head < queue.size(); ++head) {
        const std::string current = queue[head];
        if (current == goal) {
            return true;
        }
        for (const Edge& edge : graph.outEdges(current)) {
            const bool isSkipped = (current == skipFrom && edge.toId == skipTo)
                                   || (current == skipTo && edge.toId == skipFrom);
            if (isSkipped) {
                continue;
            }
            bool known = false;
            for (const std::string& s : seen) {
                if (s == edge.toId) {
                    known = true;
                    break;
                }
            }
            if (!known) {
                seen.push_back(edge.toId);
                queue.push_back(edge.toId);
            }
        }
    }
    return false;
}

double distanceBetween(const Node& a, const Node& b) {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

} // namespace

bool makeSyntheticEdge(const LogisticsGraph& graph, const std::string& fromId,
                       const std::string& toId, Edge& out) {
    const Node* from = graph.findNode(fromId);
    const Node* to = graph.findNode(toId);
    if (from == nullptr || to == nullptr) {
        return false;
    }
    const double unit = distanceBetween(*from, *to);
    if (unit <= 1e-9) {
        return false;   // 坐标重合，无法给出有意义的权重
    }

    const WeightScale scale = estimateScale(graph);
    const double km = unit * scale.kmPerUnit;

    out = Edge{};
    out.fromId = fromId;
    out.toId = toId;
    out.distanceKm = km;
    out.timeMin = km * scale.minPerKm;
    out.costYuan = km * scale.yuanPerKm;
    out.baseTimeMin = out.timeMin;
    return true;
}

std::string addRandomCustomer(LogisticsGraph& graph, Rng& rng) {
    if (graph.nodeCount() == 0) {
        return std::string();
    }

    // 在既有节点的包围盒内（留 10% 边距）取一个随机位置
    double minX = 1e18;
    double maxX = -1e18;
    double minY = 1e18;
    double maxY = -1e18;
    for (const Node& node : graph.nodes()) {
        // 手写比较：核心层不引入 <algorithm>（§10.1）
        if (node.x < minX) { minX = node.x; }
        if (node.x > maxX) { maxX = node.x; }
        if (node.y < minY) { minY = node.y; }
        if (node.y > maxY) { maxY = node.y; }
    }
    const double marginX = (maxX - minX) * 0.1 + 1.0;
    const double marginY = (maxY - minY) * 0.1 + 1.0;

    Node created;
    created.type = NodeType::Delivery;
    created.x = rng.nextRange(minX - marginX, maxX + marginX);
    created.y = rng.nextRange(minY - marginY, maxY + marginY);

    // 生成不重复的 ID
    for (int i = 1; i <= 1000; ++i) {
        char buf[16];
        std::snprintf(buf, sizeof(buf), "X%03d", i);
        if (graph.findNode(buf) == nullptr) {
            created.id = buf;
            break;
        }
    }
    if (created.id.empty()) {
        return std::string();
    }
    created.name = "新客户" + created.id.substr(1);

    // 连到最近的既有节点
    const Node* nearest = nullptr;
    double best = 1e18;
    for (const Node& node : graph.nodes()) {
        const double d = distanceBetween(created, node);
        if (d < best) {
            best = d;
            nearest = &node;
        }
    }
    if (nearest == nullptr) {
        return std::string();
    }
    const std::string neighborId = nearest->id;

    // 子网络编号的继承（规格一 MC-C04 / #30 缺陷）：新节点**必须**落进某个已有子网络，
    // 否则 sub_network_id 恒为 0 —— 这个字段在按子网络聚类的分配算法（ADR 0005）里
    // 是簇键，0 是一个**不存在的子网络**，新客户单会无处归簇。
    //
    // 规则（必须确定，故并列按节点 ID 取小）：
    //   ① 继承**最近既有节点**的编号（可能是中转站，也可能是配送点）；
    //   ② 最近节点是仓库（编号 0）时，回落到「最近的、编号非 0 的配送点或中转站」；
    //   ③ 整张图没有任何非 0 编号（如测试用小图）时保持 0 —— 无从继承，不臆造编号。
    created.subNetworkId = nearest->subNetworkId;
    if (created.subNetworkId == 0) {
        const Node* fallback = nullptr;
        double fallbackDist = 0.0;
        for (const Node& node : graph.nodes()) {
            if (node.type == NodeType::Warehouse || node.subNetworkId == 0) {
                continue;
            }
            const double d = distanceBetween(created, node);
            if (fallback == nullptr || d < fallbackDist - 1e-12
                || (d <= fallbackDist + 1e-12 && node.id < fallback->id)) {
                fallback = &node;
                fallbackDist = d;
            }
        }
        if (fallback != nullptr) {
            created.subNetworkId = fallback->subNetworkId;
        }
    }

    if (!graph.addNode(created)) {
        return std::string();
    }

    Edge forward;
    Edge backward;
    if (!makeSyntheticEdge(graph, created.id, neighborId, forward)
        || !makeSyntheticEdge(graph, neighborId, created.id, backward)) {
        graph.removeNode(created.id);   // 建边失败则回滚，保持图一致
        return std::string();
    }
    graph.addEdge(forward);
    graph.addEdge(backward);
    return created.id;
}

std::string closeRandomRoad(LogisticsGraph& graph, Rng& rng) {
    const std::vector<Edge> all = graph.edges();
    if (all.empty()) {
        return std::string();
    }

    // 从随机位置开始遍历候选，但**不封闭"桥"**：
    // 若某条道路封闭后其两端不再能互相到达，说明它是连通性的唯一通道，
    // 封掉会把一片区域变成孤岛，使演示陷入无法恢复的"不可行"状态。
    // 想演示"封路导致不可达"的场景，请用界面的手工增删，而不是这个模拟按钮。
    const std::size_t start = static_cast<std::size_t>(
        rng.nextInt(0, static_cast<int>(all.size()) - 1));

    for (std::size_t k = 0; k < all.size(); ++k) {
        const Edge& candidate = all[(start + k) % all.size()];
        const std::string& from = candidate.fromId;
        const std::string& to = candidate.toId;

        if (!reachesSkipping(graph, from, to, from, to)
            || !reachesSkipping(graph, to, from, from, to)) {
            continue;   // 这是桥，跳过
        }
        graph.removeEdge(from, to);
        graph.removeEdge(to, from);   // 道路封闭是双向的
        return from + "->" + to;
    }

    return std::string();   // 剩下的全是桥，拒绝封闭
}

} // namespace logistics
