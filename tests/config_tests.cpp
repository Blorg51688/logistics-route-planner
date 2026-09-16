// 配置加载测试：不依赖 Qt，不引入测试框架。
// 覆盖三类用例：正确运行 / 错误输入 / 边界数据。
#include <sstream>
#include <string>
#include <vector>

#include "core/Config.h"
#include "core/LogisticsGraph.h"
#include "io/ConfigLoader.h"
#include "test_util.h"

using logistics::Config;
using logistics::ConfigLoader;
using logistics::Edge;
using logistics::LogisticsGraph;
using logistics::NodeType;
using testutil::check;

#ifndef DEFAULT_CONFIG_PATH
#error "缺少 DEFAULT_CONFIG_PATH 编译定义"
#endif

namespace {

// 从 from 出发能否到达 to（有向图上的可达性，测试用）
bool reaches(const LogisticsGraph& g, const std::string& from, const std::string& to) {
    const int start = g.indexOf(from);
    const int goal = g.indexOf(to);
    if (start < 0 || goal < 0) {
        return false;
    }
    const std::vector<std::vector<Edge>>& adj = g.adjacency();
    std::vector<bool> visited(adj.size(), false);
    std::vector<std::size_t> queue;

    visited[static_cast<std::size_t>(start)] = true;
    queue.push_back(static_cast<std::size_t>(start));

    for (std::size_t head = 0; head < queue.size(); ++head) {
        const std::size_t cur = queue[head];
        if (static_cast<int>(cur) == goal) {
            return true;
        }
        for (const Edge& e : adj[cur]) {
            const int next = g.indexOf(e.toId);
            if (next >= 0 && !visited[static_cast<std::size_t>(next)]) {
                visited[static_cast<std::size_t>(next)] = true;
                queue.push_back(static_cast<std::size_t>(next));
            }
        }
    }
    return false;
}

double totalDemand(const Config& cfg) {
    double sum = 0.0;
    for (const logistics::Order& o : cfg.orders) {
        sum += o.demandKg;
    }
    return sum;
}

std::size_t countType(const Config& cfg, NodeType type) {
    std::size_t n = 0;
    for (const logistics::Node& node : cfg.graph.nodes()) {
        if (node.type == type) {
            ++n;
        }
    }
    return n;
}

// 最小但完整的合法配置，供错误用例做局部替换
const char* kMinimalConfig =
    "[nodes]\n"
    "W01, warehouse, 0, 0, 仓库, 0\n"
    "D01, delivery, 10, 10, 客户, 0\n"
    "[edges]\n"
    "W01, D01, 1.0, 2.0, 1.5\n"
    "[vehicles]\n"
    "V01, W01, 100, 08:00\n"
    "[orders]\n"
    "O01, D01, 10, 09:00, 18:00, 0\n";

// 用一段文本尝试加载，期望失败
void expectParseFailure(const std::string& text, const std::string& what) {
    Config cfg;
    std::string error;
    const bool ok = ConfigLoader::parse(text, cfg, error);
    check(!ok, "应加载失败: " + what);
    check(!error.empty(), "失败时应给出错误说明: " + what);
}

std::string nodeName(int index) {
    if (index == 0) {
        return "W01";
    }
    return "N" + std::to_string(index);
}

// 恰好 30 节点 / 50 边的边界配置
std::string buildScaleBoundaryConfig() {
    const int nodeCount = 30;
    const int edgeCount = 50;
    std::ostringstream os;
    os << "[nodes]\n";
    for (int i = 0; i < nodeCount; ++i) {
        if (i == 0) {
            os << "W01, warehouse, 0, 0, 仓库, 0\n";
        } else {
            os << nodeName(i) << ", delivery, " << i * 10 << ", 0, 点" << i << ", 0\n";
        }
    }
    os << "[edges]\n";
    int made = 0;
    for (int a = 0; a < nodeCount && made < edgeCount; ++a) {
        for (int b = 0; b < nodeCount && made < edgeCount; ++b) {
            if (a == b) {
                continue;
            }
            os << nodeName(a) << ", " << nodeName(b) << ", 1.0, 2.0, 1.5\n";
            ++made;
        }
    }
    os << "[vehicles]\nV01, W01, 1000, 08:00\n";
    os << "[orders]\nO01, N1, 10, 09:00, 18:00, 0\n";
    return os.str();
}

// ---- 正确运行用例 ----

void testLoadDefaultConfig() {
    Config cfg;
    std::string error;
    const bool ok = ConfigLoader::load(DEFAULT_CONFIG_PATH, cfg, error);
    check(ok, "默认配置加载成功（错误: " + error + "）");
    if (!ok) {
        return;
    }

    // 题目规模要求：节点 ≥30（≥2 仓库、≥15 配送点、≥3 中转站），边 ≥50
    check(cfg.graph.nodeCount() >= 30, "节点总数 ≥ 30");
    check(countType(cfg, NodeType::Warehouse) >= 2, "仓库 ≥ 2");
    check(countType(cfg, NodeType::Delivery) >= 15, "配送点 ≥ 15");
    check(countType(cfg, NodeType::Transit) >= 3, "中转站 ≥ 3");
    check(cfg.graph.edgeCount() >= 50, "路径总数 ≥ 50");

    // 车辆
    check(cfg.vehicles.size() == 1, "第一版恰有 1 辆车");
    if (!cfg.vehicles.empty()) {
        const logistics::Vehicle& v = cfg.vehicles[0];
        check(v.capacityKg > 0.0, "载重上限为正");
        check(v.departTimeMin == 8 * 60, "发车时刻 08:00 解析为 480 分钟");
        const logistics::Node* start = cfg.graph.findNode(v.startNodeId);
        check(start != nullptr && start->type == NodeType::Warehouse, "起始节点是仓库");
    }

    // 订单
    check(cfg.orders.size() >= 15, "订单数量满足配送点要求");
    for (const logistics::Order& o : cfg.orders) {
        check(cfg.graph.findNode(o.nodeId) != nullptr, "订单目标节点存在: " + o.id);
        check(o.windowStartMin < o.windowEndMin, "订单窗口起早于止: " + o.id);
        check(o.demandKg > 0.0, "订单货物量为正: " + o.id);
    }

    // D21（修订 D14）：总需求**可以**超过载重——车辆会多趟往返取货、
    // 把货暂存在中转站再二次配发。真正不可行的是"**单个订单**的货量就超过载重"，
    // 那样分多少趟都装不下。
    double maxOrderDemand = 0.0;
    for (const logistics::Order& o : cfg.orders) {
        if (o.demandKg > maxOrderDemand) {
            maxOrderDemand = o.demandKg;
        }
    }
    check(!cfg.vehicles.empty() && maxOrderDemand <= cfg.vehicles[0].capacityKg,
          "单个订单最大货量 " + std::to_string(maxOrderDemand) + " ≤ 载重");

    // [general] 参数
    // 注：service_time_min 已移除——要求原文从未提及服务时间（搜「服务/停留/装卸」
    // 均为 0 处），按用户要求删掉，到达时刻即离开时刻。
    check(cfg.general.trafficChangeRatio == 0.1, "traffic_change_ratio == 0.1");
    check(cfg.general.trafficTimeIncreaseMin == 0.2, "traffic_time_increase_min == 0.2");
}

void testDefaultConfigRoutingPreconditions() {
    Config cfg;
    std::string error;
    if (!ConfigLoader::load(DEFAULT_CONFIG_PATH, cfg, error)) {
        check(false, "默认配置加载成功（可达性测试前置）");
        return;
    }

    // B6 要求遍历全部待配送点后返回仓库：每个订单点都必须能从起点仓库到达、
    // 且能返回起点仓库，否则默认数据无法支撑完整路线。
    const std::string& start = cfg.vehicles[0].startNodeId;
    for (const logistics::Order& o : cfg.orders) {
        check(reaches(cfg.graph, start, o.nodeId), "从起点可达配送点: " + o.nodeId);
        check(reaches(cfg.graph, o.nodeId, start), "从配送点可返回起点: " + o.nodeId);
    }
}

void testDynamicAddRemove() {
    Config cfg;
    std::string error;
    if (!ConfigLoader::load(DEFAULT_CONFIG_PATH, cfg, error)) {
        check(false, "默认配置加载成功（增删测试前置）");
        return;
    }

    const std::size_t before = cfg.graph.nodeCount();
    logistics::Node extra;
    extra.id = "X01";
    extra.type = NodeType::Delivery;
    extra.name = "新客户";
    check(cfg.graph.addNode(extra), "动态添加节点");
    check(cfg.graph.nodeCount() == before + 1, "添加后节点数 +1");

    Edge link;
    link.fromId = "W01";
    link.toId = "X01";
    link.distanceKm = 2.0;
    link.timeMin = 3.0;
    link.costYuan = 2.4;
    check(cfg.graph.addEdge(link), "动态添加边");
    check(cfg.graph.removeNode("X01"), "动态删除节点");
    check(cfg.graph.nodeCount() == before, "删除后节点数还原");
    check(cfg.graph.findEdge("W01", "X01") == nullptr, "删除节点时同步删除其边");
}

// ---- 错误输入用例 ----

void testErrorCases() {
    const std::string base = kMinimalConfig;

    // 缺节
    expectParseFailure("[nodes]\nW01, warehouse, 0, 0, 仓, 0\n[vehicles]\n[orders]\n",
                       "缺少 [edges] 节");
    expectParseFailure("[edges]\n[vehicles]\n[orders]\n", "缺少 [nodes] 节");
    expectParseFailure(
        "[nodes]\nW01, warehouse, 0, 0, 仓, 0\n"
        "[edges]\nW01, W01, 1, 1, 1\n[orders]\n", "缺少 [vehicles] 节");
    expectParseFailure(
        "[nodes]\nW01, warehouse, 0, 0, 仓, 0\n"
        "[edges]\nW01, W01, 1, 1, 1\n[vehicles]\nV01, W01, 10, 08:00\n", "缺少 [orders] 节");

    // 字段数不符
    expectParseFailure(
        "[nodes]\nW01, warehouse, 0, 0\n[edges]\n[vehicles]\n[orders]\n", "[nodes] 字段数不符");
    expectParseFailure(
        "[nodes]\nW01, warehouse, 0, 0, 仓, 0\n[edges]\nW01, W01, 1, 1\n[vehicles]\n[orders]\n",
        "[edges] 字段数不符");

    // 未知节点类型
    expectParseFailure(
        "[nodes]\nW01, hub, 0, 0, 仓, 0\n[edges]\n[vehicles]\n[orders]\n", "未知节点类型");

    // 节点 ID 重复
    expectParseFailure(
        "[nodes]\nW01, warehouse, 0, 0, 仓, 0\nW01, delivery, 1, 1, 客, 0\n"
        "[edges]\n[vehicles]\n[orders]\n", "节点 ID 重复");

    // 边指向不存在的节点
    expectParseFailure(
        "[nodes]\nW01, warehouse, 0, 0, 仓, 0\n"
        "[edges]\nW01, NOPE, 1, 1, 1\n[vehicles]\n[orders]\n", "边端点不存在");

    // 起始节点不是仓库
    expectParseFailure(
        "[nodes]\nW01, warehouse, 0, 0, 仓, 0\nD01, delivery, 1, 1, 客, 0\n"
        "[edges]\n[vehicles]\nV01, D01, 10, 08:00\n[orders]\n", "起始节点必须为仓库");

    // 非法时间格式
    expectParseFailure(
        "[nodes]\nW01, warehouse, 0, 0, 仓, 0\n"
        "[edges]\n[vehicles]\nV01, W01, 10, 25:00\n[orders]\n", "小时超范围的发车时刻");
    expectParseFailure(
        "[nodes]\nW01, warehouse, 0, 0, 仓, 0\n"
        "[edges]\n[vehicles]\nV01, W01, 10, 8点\n[orders]\n", "非 HH:MM 的发车时刻");

    // 窗口起不早于止
    expectParseFailure(
        "[nodes]\nW01, warehouse, 0, 0, 仓, 0\nD01, delivery, 1, 1, 客, 0\n"
        "[edges]\n[vehicles]\nV01, W01, 100, 08:00\n"
        "[orders]\nO01, D01, 10, 18:00, 09:00, 0\n", "窗口起晚于止");

    // is_urgent 非 0/1
    expectParseFailure(
        "[nodes]\nW01, warehouse, 0, 0, 仓, 0\nD01, delivery, 1, 1, 客, 0\n"
        "[edges]\n[vehicles]\nV01, W01, 100, 08:00\n"
        "[orders]\nO01, D01, 10, 09:00, 18:00, 2\n", "is_urgent 取值非法");

    // 订单目标不是配送点
    expectParseFailure(
        "[nodes]\nW01, warehouse, 0, 0, 仓, 0\n"
        "[edges]\n[vehicles]\nV01, W01, 100, 08:00\n"
        "[orders]\nO01, W01, 10, 09:00, 18:00, 0\n", "订单目标不是配送点");

    // 订单货物量非正
    expectParseFailure(
        "[nodes]\nW01, warehouse, 0, 0, 仓, 0\nD01, delivery, 1, 1, 客, 0\n"
        "[edges]\n[vehicles]\nV01, W01, 100, 08:00\n"
        "[orders]\nO01, D01, 0, 09:00, 18:00, 0\n", "货物量必须为正");

    // 载重非正
    expectParseFailure(
        "[nodes]\nW01, warehouse, 0, 0, 仓, 0\n"
        "[edges]\n[vehicles]\nV01, W01, 0, 08:00\n[orders]\n", "载重必须为正");

    // 重复 section
    expectParseFailure(
        std::string(base) + "[nodes]\n", "重复 [section]");

    // 配置项出现在任何 section 之前
    expectParseFailure("key = value\n" + std::string(base), "配置项出现在节之前");

    // 数据行混入 '='
    expectParseFailure(
        "[nodes]\nW01 = warehouse\n[edges]\n[vehicles]\n[orders]\n", "数据行含 '='");

    // [section] 缺少右括号
    expectParseFailure("[nodes\nW01, warehouse, 0, 0, 仓, 0\n", "[section] 缺少右括号");

    // 文件不存在
    Config cfg;
    std::string error;
    check(!ConfigLoader::load("/nonexistent/path/nope.ini", cfg, error), "配置文件缺失时加载失败");
    check(!error.empty(), "文件缺失时给出错误说明");
}

// ---- 边界数据用例 ----

void testBoundaryCases() {
    // 恰好 30 节点 / 50 边
    {
        const std::string text = buildScaleBoundaryConfig();
        Config cfg;
        std::string error;
        const bool ok = ConfigLoader::parse(text, cfg, error);
        check(ok, "30 节点 / 50 边配置加载成功（错误: " + error + "）");
        if (ok) {
            check(cfg.graph.nodeCount() == 30, "节点数恰好 30");
            check(cfg.graph.edgeCount() == 50, "边数恰好 50");
        }
    }

    // 空订单列表：应合法（[orders] 节存在但无数据行）
    {
        Config cfg;
        std::string error;
        const bool ok = ConfigLoader::parse(
            "[nodes]\nW01, warehouse, 0, 0, 仓, 0\n"
            "[edges]\n[vehicles]\nV01, W01, 100, 08:00\n[orders]\n", cfg, error);
        check(ok, "空订单列表合法（错误: " + error + "）");
        check(ok && cfg.orders.empty(), "空订单列表解析为空");
    }

    // 载重恰好等于总需求：应合法（边界为 ≤，不是 <）
    {
        Config cfg;
        std::string error;
        const bool ok = ConfigLoader::parse(
            "[nodes]\nW01, warehouse, 0, 0, 仓, 0\nD01, delivery, 1, 1, 客, 0\n"
            "[edges]\nW01, D01, 1, 2, 1.5\n"
            "[vehicles]\nV01, W01, 10, 08:00\n"
            "[orders]\nO01, D01, 10, 09:00, 18:00, 0\n", cfg, error);
        check(ok, "载重等于总需求时合法（错误: " + error + "）");
        if (ok) {
            check(totalDemand(cfg) == cfg.vehicles[0].capacityKg, "总需求恰好等于载重");
        }
    }

    // 时间窗口卡点：到达时刻恰好等于窗口止不算超时（此处只验证配置可加载）
    {
        Config cfg;
        std::string error;
        const bool ok = ConfigLoader::parse(
            "[nodes]\nW01, warehouse, 0, 0, 仓, 0\nD01, delivery, 1, 1, 客, 0\n"
            "[edges]\nW01, D01, 1, 2, 1.5\n"
            "[vehicles]\nV01, W01, 100, 08:00\n"
            "[orders]\nO01, D01, 10, 08:00, 08:02, 0\n", cfg, error);
        check(ok, "紧时间窗口配置可加载（错误: " + error + "）");
        if (ok) {
            check(cfg.orders[0].windowStartMin == 480 && cfg.orders[0].windowEndMin == 482,
                  "08:00-08:02 解析为 480-482 分钟");
        }
    }

    // 可选 [general]：完全不提供时使用默认值
    {
        Config cfg;
        std::string error;
        const bool ok = ConfigLoader::parse(kMinimalConfig, cfg, error);
        check(ok, "无 [general] 节时使用默认值（错误: " + error + "）");
        check(ok && cfg.general.trafficChangeRatio == 0.1,
              "默认 traffic_change_ratio == 0.1");
    }
}

} // namespace

int main() {
    testLoadDefaultConfig();
    testDefaultConfigRoutingPreconditions();
    testDynamicAddRemove();
    testErrorCases();
    testBoundaryCases();
    return testutil::summarize("config_tests");
}
