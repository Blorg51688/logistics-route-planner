// 配置加载测试：不依赖 Qt，不引入测试框架。
// 覆盖三类用例：正确运行 / 错误输入 / 边界数据。
//
// 本版对应规格一（#23）的数据契约：
//   [orders]   七列 ID, pickup, delivery, demand_kg, window_start, window_end, is_urgent
//   [vehicles] 六列 ID, start_node, capacity_kg, depart_time, time_factor, cost_factor
//   [general]  事件改 event_level_*（固定表 1/3/9/27），退役键出现即带行号报错
#include <cmath>
#include <set>
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
using logistics::EventLevel;
using logistics::LogisticsGraph;
using logistics::NodeType;
using logistics::OrderSource;
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

double minCapacity(const Config& cfg) {
    double best = 0.0;
    for (std::size_t i = 0; i < cfg.vehicles.size(); ++i) {
        if (i == 0 || cfg.vehicles[i].capacityKg < best) {
            best = cfg.vehicles[i].capacityKg;
        }
    }
    return best;
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

// 最小但完整的合法配置底座，供错误用例做局部替换。
// 注意：订单已改为 A→B 取送，所以至少要两个配送点（D01 取、D02 送）。
const char* kNodesBase =
    "[nodes]\n"
    "W01, warehouse, 0, 0, 仓库, 0\n"
    "D01, delivery, 10, 10, 客户一, 0\n"
    "D02, delivery, 20, 10, 客户二, 0\n";
const char* kEdgesBase =
    "[edges]\n"
    "W01, D01, 1.0, 2.0, 1.5\n"
    "W01, D02, 1.0, 2.0, 1.5\n"
    "D01, D02, 1.0, 2.0, 1.5\n";
const char* kValidVehicleRow = "V01, W01, 100, 08:00, 1.00, 1.00";

const char* kMinimalConfig =
    "[nodes]\n"
    "W01, warehouse, 0, 0, 仓库, 0\n"
    "D01, delivery, 10, 10, 客户一, 0\n"
    "D02, delivery, 20, 10, 客户二, 0\n"
    "[edges]\n"
    "W01, D01, 1.0, 2.0, 1.5\n"
    "W01, D02, 1.0, 2.0, 1.5\n"
    "D01, D02, 1.0, 2.0, 1.5\n"
    "[vehicles]\n"
    "V01, W01, 100, 08:00, 1.00, 1.00\n"
    "[orders]\n"
    "O01, D01, D02, 10, 09:00, 18:00, 0\n";

// [orders] 数据行固定落在第 12 行：底座 1–11 行见 kNodesBase/kEdgesBase/[vehicles]。
std::string configWithOrderRows(const std::string& rows,
                                const std::string& vehicleRow = kValidVehicleRow) {
    return std::string(kNodesBase) + kEdgesBase + "[vehicles]\n" + vehicleRow +
           "\n[orders]\n" + rows;
}

// [vehicles] 数据行固定落在第 10 行。
std::string configWithVehicleRow(const std::string& row) {
    return std::string(kNodesBase) + kEdgesBase + "[vehicles]\n" + row +
           "\n[orders]\nO01, D01, D02, 10, 09:00, 18:00, 0\n";
}

// 用一段文本尝试加载，期望失败
void expectParseFailure(const std::string& text, const std::string& what) {
    Config cfg;
    std::string error;
    const bool ok = ConfigLoader::parse(text, cfg, error);
    check(!ok, "应加载失败: " + what);
    check(!error.empty(), "失败时应给出错误说明: " + what);
}

// 错误信息必须含「第 <lineNo> 行」，且 <lineNo> 与 expectLineNo 逐位相等。
// 为什么不能只查"含'第'字"：那样即使 fail() 把行号拼成常量或拼错，
// 断言依然通过——本条守卫要的是"行号与该错误在配置里实际所在行一致"。
// 行号后用完整串比较，防止 "第 2 行" 误配 "第 12 行"。
void expectParseFailureAtLine(const std::string& text, int expectLineNo,
                              const std::string& what) {
    Config cfg;
    std::string error;
    const bool ok = ConfigLoader::parse(text, cfg, error);
    check(!ok, "应加载失败: " + what);
    const std::string token = "第 " + std::to_string(expectLineNo) + " 行";
    const std::size_t at = error.find(token);
    check(at != std::string::npos,
          "错误信息应带行号 " + std::to_string(expectLineNo) + "（实际 \"" + error + "\"）: " + what);
    if (at == std::string::npos) {
        return;
    }
    // 行号必须恰好结束在这里：后面紧跟的是冒号或空格+节名，不能再接数字
    const std::size_t after = at + token.size();
    const bool boundaryOk = after < error.size() && error[after] != '0' && error[after] != '1' &&
                            error[after] != '2' && error[after] != '3' && error[after] != '4' &&
                            error[after] != '5' && error[after] != '6' && error[after] != '7' &&
                            error[after] != '8' && error[after] != '9';
    check(boundaryOk, "行号 " + std::to_string(expectLineNo) + " 后不应再跟数字（实际 \"" + error +
                          "\"）: " + what);
}

// 在"带对了行号"之外再断言具体文案：数据契约要求报错文案逐字可读，
// 只断行号会让"把原因写成别的话"也通过。
void expectParseFailureAtLineWithMessage(const std::string& text, int expectLineNo,
                                         const std::string& needle, const std::string& what) {
    expectParseFailureAtLine(text, expectLineNo, what);
    Config cfg;
    std::string error;
    if (ConfigLoader::parse(text, cfg, error)) {
        return;
    }
    check(error.find(needle) != std::string::npos,
          "错误信息应含 \"" + needle + "\"（实际 \"" + error + "\"）: " + what);
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
    os << "[vehicles]\nV01, W01, 1000, 08:00, 1.00, 1.00\n";
    os << "[orders]\nO01, N1, N2, 10, 09:00, 18:00, 0\n";
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

    // 车辆：规格三 §4 的**默认四车车队**（六列，含两因子），行序即车辆序。
    //   一台快车（400kg / 成本因子 1.50：簇间搬运贵而快）+ 三台慢车（160kg / 0.80：
    //   每台守一个簇）。**这是职责封闭的前提**：三簇三慢车 ⇒「簇 -> 慢车」排列数 = 1。
    //   订单表在规格六 #60 换成「簇内 / 跨簇参半」（13 簇内 + 12 跨簇 = 25 单 456kg）。
    check(cfg.vehicles.size() == 4, "默认车队恰有 4 辆车（V01 快车 + V02/V03/V04 慢车）");
    if (cfg.vehicles.size() == 4) {
        struct ExpectVehicle {
            const char* id;
            double      capacityKg;
            double      timeFactor;
            double      costFactor;
            const char* startNode;   // 规格四 #54：慢车就近驻场，不再全部 W01
            int         departMin;   // 规格四 #54：V03 提前发车覆盖跨城通勤
        };
        // 起始节点与发车时刻是**数据事实**，随规格七 #63 改动（慢车出生在本簇中转站）：
        //   · V01 是快车，仍从中央仓库 A（W01）出发，08:00；
        //   · V02 守簇 1，出生在**本簇中转站 T01**，08:00；
        //   · V03 守簇 2，出生在 **T02**，08:00；
        //   · V04 守簇 3，出生在 **T03**，**14:00**（簇 3 没有簇内单，它的第一件活是
        //     在 T03 接跨簇货 —— 班次跟着 L2 到站时刻走：1->3 与 2->3 两条走廊的货
        //     14:02~14:37 才到站，班次落在它前面 60min 窗口内才接得住）。
        // 为什么这些要写进测试而不是"随便哪个站都行"：它们直接决定黄金值，
        // 漂了就是真回归（每台车驻在哪片区域、什么时候出发，都是可核对的承诺）。
        // 断路的后果：出生点必须是**本簇**的站，否则慢车连第一段路都走不出去。
        const ExpectVehicle expect[4] = {{"V01", 400.0, 1.00, 1.50, "W01", 8 * 60},
                                         {"V02", 160.0, 1.25, 0.80, "T01", 8 * 60},
                                         {"V03", 160.0, 1.25, 0.80, "T02", 8 * 60},
                                         {"V04", 160.0, 1.25, 0.80, "T03", 14 * 60}};
        for (int i = 0; i < 4; ++i) {
            const logistics::Vehicle& v = cfg.vehicles[static_cast<std::size_t>(i)];
            const std::string tag = std::string("默认车辆 ") + expect[i].id + "：";
            check(v.id == expect[i].id, tag + "行序与 ID 一致");
            check(v.capacityKg == expect[i].capacityKg, tag + "载重");
            check(v.departTimeMin == expect[i].departMin, tag + "发车时刻");
            check(v.timeFactor == expect[i].timeFactor, tag + "耗时倍率");
            check(v.costFactor == expect[i].costFactor, tag + "成本倍率");
            check(v.startNodeId == expect[i].startNode, tag + "起始节点");
            const logistics::Node* start = cfg.graph.findNode(v.startNodeId);
            // 规格七 #63：快车从仓库出发，慢车从**本簇中转站**出发（客户节点一律不许当起点）。
            const bool startOk =
                start != nullptr
                && (start->type == NodeType::Warehouse || start->type == NodeType::Transit);
            check(startOk, tag + "起始节点是仓库或中转站");
            if (start != nullptr && start->type == NodeType::Transit) {
                // 慢车的出生站必须**在本簇**：T01->簇1 / T02->簇2 / T03->簇3。
                const int expectCluster = i;   // 车队表：V02=簇1、V03=簇2、V04=簇3（V01 是快车）
                check(start->subNetworkId == expectCluster,
                      tag + "出生站属于本簇（站 " + std::to_string(start->subNetworkId)
                          + " vs 期望 " + std::to_string(expectCluster) + "）");
            }
        }
        // §3.1 生成器自检在数据上的**结果**（契约层不强制，但默认数据集必须满足）：
        // 两型车真的分化；快车容量 >= 2 × 最小容量。
        bool hasSlower = false;
        bool hasCheaper = false;
        double minCap = cfg.vehicles[0].capacityKg;
        bool hasBig = false;
        for (const logistics::Vehicle& v : cfg.vehicles) {
            if (v.timeFactor > 1.0) {
                hasSlower = true;
            }
            if (v.costFactor < 1.0) {
                hasCheaper = true;
            }
            if (v.capacityKg < minCap) {
                minCap = v.capacityKg;
            }
        }
        for (const logistics::Vehicle& v : cfg.vehicles) {
            if (v.capacityKg >= 2.0 * minCap) {
                hasBig = true;
            }
        }
        check(hasSlower, "默认车队存在 time_factor > 1 的慢车");
        check(hasCheaper, "默认车队存在 cost_factor < 1 的低成本车");
        check(hasBig, "默认车队存在容量 >= 2 × 最小容量的快车（400 >= 2×160）");
    }

    // 订单：25 单 A→B 取送，七列口径
    check(cfg.orders.size() == 25, "默认数据集恰有 25 张订单");
    std::set<std::string> pickupNodes;
    std::set<std::string> deliveryNodes;
    std::set<std::string> allDeliveryNodes;
    for (const logistics::Node& node : cfg.graph.nodes()) {
        if (node.type == NodeType::Delivery) {
            allDeliveryNodes.insert(node.id);
        }
    }
    double maxOrderDemand = 0.0;
    for (const logistics::Order& o : cfg.orders) {
        const logistics::Node* pickup = cfg.graph.findNode(o.pickupNodeId);
        const logistics::Node* delivery = cfg.graph.findNode(o.nodeId);
        check(pickup != nullptr, "订单取货点存在: " + o.id);
        check(delivery != nullptr, "订单送达点存在: " + o.id);
        if (pickup != nullptr) {
            check(pickup->type == NodeType::Delivery, "取货点必须是配送点: " + o.id);
            pickupNodes.insert(o.pickupNodeId);
        }
        if (delivery != nullptr) {
            check(delivery->type == NodeType::Delivery, "送达点必须是配送点: " + o.id);
            deliveryNodes.insert(o.nodeId);
        }
        check(!o.pickupNodeId.empty(), "订单取货点必填: " + o.id);
        check(o.pickupNodeId != o.nodeId, "取货点与送达点不能相同: " + o.id);
        check(o.windowStartMin < o.windowEndMin, "订单窗口起早于止: " + o.id);
        check(o.demandKg > 0.0, "订单货物量为正: " + o.id);
        check(!o.urgent, "初始数据集全普通（is_urgent = 0）: " + o.id);
        check(o.source == OrderSource::Initial, "配置加载的订单来源为 initial: " + o.id);
        if (o.demandKg > maxOrderDemand) {
            maxOrderDemand = o.demandKg;
        }
    }
    check(deliveryNodes == allDeliveryNodes, "B 集合 = 全部 25 个配送点");
    check(deliveryNodes.size() == 25, "B 集合恰有 25 个节点");
    // 规格六 #60：集货点收敛到每簇的枢纽客户（跨簇单的 L1 要「一站取完」才能在
    // W=60min 的站内交接窗口内把整组货交到快车手上）。
    const std::set<std::string> expectedPickups = {"D01", "D08", "D14"};
    check(pickupNodes == expectedPickups, "A 集合 = {D01,D08,D14}");
    check(totalDemand(cfg) == 456.0, "默认数据集总重 456kg");

    // ---- 规格六 #60 守卫 B（本票新增）：**参半数据**的三条形状 ----
    // 反向验证：把订单表改回「全簇内」（规格五的 25 单），①②立刻变红。
    int crossOrders = 0;
    std::set<std::pair<int, int> > corridors;
    for (std::size_t i = 0; i < cfg.orders.size(); ++i) {
        const logistics::Order& o = cfg.orders[i];
        const logistics::Node* a = cfg.graph.findNode(o.pickupNodeId);
        const logistics::Node* b = cfg.graph.findNode(o.nodeId);
        if (a == nullptr || b == nullptr) {
            continue;
        }
        if (a->subNetworkId != b->subNetworkId) {
            ++crossOrders;
            int lo = a->subNetworkId < b->subNetworkId ? a->subNetworkId : b->subNetworkId;
            int hi = a->subNetworkId < b->subNetworkId ? b->subNetworkId : a->subNetworkId;
            corridors.insert(std::make_pair(lo, hi));
        }
    }
    const double crossRatio =
        static_cast<double>(crossOrders) / static_cast<double>(cfg.orders.size());
    check(crossRatio >= 0.40 && crossRatio <= 0.60,
          "数据参半①：跨簇单占比 " + std::to_string(crossRatio * 100.0) + "% 应在 [40%, 60%]"
              "（跨簇 " + std::to_string(crossOrders) + " / 总 "
              + std::to_string(cfg.orders.size()) + "）");
    std::set<std::pair<int, int> > expectedCorridors;
    expectedCorridors.insert(std::make_pair(1, 2));
    expectedCorridors.insert(std::make_pair(1, 3));
    expectedCorridors.insert(std::make_pair(2, 3));
    check(corridors == expectedCorridors,
          "数据参半②：三条走廊齐备（1-2 / 1-3 / 2-3），实际 " + std::to_string(corridors.size())
              + " 条");
    bool anyUrgent = false;
    for (std::size_t i = 0; i < cfg.orders.size(); ++i) {
        if (cfg.orders[i].urgent) {
            anyUrgent = true;
        }
    }
    check(!anyUrgent, "数据参半③：无紧急单（is_urgent 全 0）");

    // Q1：跨节校验是**严格小于**——任何一台车都要能至少单独装下一张单
    check(maxOrderDemand < minCapacity(cfg),
          "单个订单最大货量 " + std::to_string(maxOrderDemand) + " < 最小车辆载重 " +
              std::to_string(minCapacity(cfg)));

    // [general] 参数
    // 注：service_time_min 已退役且保持不存在——要求原文从未提及服务时间，
    // 停靠零耗时（ADR 0002 Q3）。
    check(cfg.general.trafficChangeRatio == 0.1, "traffic_change_ratio == 0.1");
    check(cfg.general.trafficTimeIncreaseMin == 0.2, "traffic_time_increase_min == 0.2");
    check(cfg.general.trafficTimeIncreaseMax == 0.5, "traffic_time_increase_max == 0.5");
    check(cfg.general.eventIntervalMin == 15, "event_interval_min == 15");

    // 事件等级：默认组合 高/中/低/极低 -> 27:9:3:1（#23 §3.3）
    check(cfg.general.eventLevelTraffic == EventLevel::High, "默认 traffic = high");
    check(cfg.general.eventLevelCustomer == EventLevel::Medium, "默认 customer = medium");
    check(cfg.general.eventLevelUrgent == EventLevel::Low, "默认 urgent = low");
    check(cfg.general.eventLevelClosure == EventLevel::ExtraLow, "默认 closure = extra_low");
    check(logistics::eventLevelWeight(cfg.general.eventLevelTraffic) == 27, "high -> 27");
    check(logistics::eventLevelWeight(cfg.general.eventLevelCustomer) == 9, "medium -> 9");
    check(logistics::eventLevelWeight(cfg.general.eventLevelUrgent) == 3, "low -> 3");
    check(logistics::eventLevelWeight(cfg.general.eventLevelClosure) == 1, "extra_low -> 1");
}

// [vehicles] 三车夹具（规格二 §3.1 的六列 + 八条校验在多行下的行为）。
void testThreeVehicleFixture() {
    const std::string rows =
        "V01, W01, 400, 08:00, 1.00, 1.50\n"
        "V02, W01, 160, 08:00, 1.25, 0.80\n"
        "V03, W01, 160, 08:00, 1.25, 0.80\n";
    Config cfg;
    std::string error;
    const bool ok = ConfigLoader::parse(
        std::string(kNodesBase) + kEdgesBase + "[vehicles]\n" + rows +
            "[orders]\nO01, D01, D02, 10, 09:00, 18:00, 0\n",
        cfg, error);
    check(ok, "三车夹具可加载（错误: " + error + "）");
    if (ok) {
        check(cfg.vehicles.size() == 3, "三车夹具恰有 3 台车");
        check(cfg.vehicles[0].id == "V01" && cfg.vehicles[1].id == "V02"
                  && cfg.vehicles[2].id == "V03",
              "车辆序 = [vehicles] 行序");
        check(cfg.vehicles[0].capacityKg == 400.0, "V01 400kg");
        check(cfg.vehicles[1].capacityKg == 160.0, "V02 160kg");
        check(cfg.vehicles[2].timeFactor == 1.25, "V03 耗时倍率 1.25");
        check(cfg.vehicles[0].costFactor == 1.50, "V01 成本倍率 1.50");
    }
    // 跨节校验随车队变：min(capacity) = 160 ⇒ 单笔 160kg 必须报错、159.9 合法。
    {
        Config big;
        std::string err;
        const bool bad = ConfigLoader::parse(
            std::string(kNodesBase) + kEdgesBase + "[vehicles]\n" + rows +
                "[orders]\nO01, D01, D02, 160, 09:00, 18:00, 0\n",
            big, err);
        check(!bad, "单笔 160kg == min(容量) 时按严格小于报错");
        const bool good = ConfigLoader::parse(
            std::string(kNodesBase) + kEdgesBase + "[vehicles]\n" + rows +
                "[orders]\nO01, D01, D02, 159.9, 09:00, 18:00, 0\n",
            big, err);
        check(good, "单笔 159.9kg < min(容量) 时合法（错误: " + err + "）");
    }
}

void testDefaultConfigRoutingPreconditions() {
    Config cfg;
    std::string error;
    if (!ConfigLoader::load(DEFAULT_CONFIG_PATH, cfg, error)) {
        check(false, "默认配置加载成功（可达性测试前置）");
        return;
    }

    // B6 要求遍历全部待配送点后返回仓库：取货点 A 与送达点 B 都必须能从起点仓库
    // 到达、且能返回起点仓库，否则默认数据无法支撑完整路线。
    const std::string& start = cfg.vehicles[0].startNodeId;
    for (const logistics::Order& o : cfg.orders) {
        check(reaches(cfg.graph, start, o.pickupNodeId), "从起点可达取货点: " + o.pickupNodeId);
        check(reaches(cfg.graph, o.pickupNodeId, start), "从取货点可返回起点: " + o.pickupNodeId);
        check(reaches(cfg.graph, start, o.nodeId), "从起点可达送达点: " + o.nodeId);
        check(reaches(cfg.graph, o.nodeId, start), "从送达点可返回起点: " + o.nodeId);
    }
}

void testOrderSourcePrefixes() {
    // #24 R5：来源是运行时字段，ID 前缀 O/N/U/C（规格二预留 M）。
    check(logistics::orderIdPrefix(OrderSource::Initial) == std::string("O"), "initial -> O");
    check(logistics::orderIdPrefix(OrderSource::EventNormal) == std::string("N"),
          "event_normal -> N");
    check(logistics::orderIdPrefix(OrderSource::EventUrgent) == std::string("U"),
          "event_urgent -> U");
    check(logistics::orderIdPrefix(OrderSource::ManualCustomer) == std::string("C"),
          "manual_customer -> C");
    const logistics::Order fresh;
    check(fresh.source == OrderSource::Initial, "订单来源默认 initial");
    check(!fresh.urgent, "订单默认非紧急");
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
    check(cfg.graph.nodeCount() == before, "删除节点数还原");
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
        std::string(kNodesBase) + kEdgesBase + "[orders]\n", "缺少 [vehicles] 节");
    expectParseFailure(
        std::string(kNodesBase) + kEdgesBase +
            "[vehicles]\nV01, W01, 100, 08:00, 1.00, 1.00\n", "缺少 [orders] 节");

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

    // ---- [vehicles] 八条校验（数据行在第 10 行） ----
    expectParseFailureAtLineWithMessage(
        configWithVehicleRow("V01, W01, 200, 08:00"), 10, "需要 6 个字段，实际 4 个",
        "[vehicles] 旧 4 列按字段数报错");
    expectParseFailure(
        std::string(kNodesBase) + kEdgesBase + "[vehicles]\n[orders]\n",
        "[vehicles] 空节报错（车队不能为空）");
    expectParseFailureAtLineWithMessage(
        configWithVehicleRow(", W01, 100, 08:00, 1.00, 1.00"), 10, "车辆 ID 为空",
        "[vehicles] 车辆 ID 为空");
    expectParseFailureAtLineWithMessage(
        configWithVehicleRow("V01, W01, 100, 08:00, 1.00, 1.00\n"
                             "V01, W01, 100, 08:00, 1.00, 1.00"),
        11, "车辆 ID 重复: V01", "[vehicles] 车辆 ID 重复（第 11 行）");
    expectParseFailureAtLineWithMessage(
        configWithVehicleRow("V01, , 100, 08:00, 1.00, 1.00"), 10, "起始节点为空",
        "[vehicles] 起始节点为空");
    expectParseFailureAtLineWithMessage(
        configWithVehicleRow("V01, NOPE, 100, 08:00, 1.00, 1.00"), 10, "起始节点不存在: NOPE",
        "[vehicles] 起始节点不存在");
    expectParseFailureAtLineWithMessage(
        configWithVehicleRow("V01, D01, 100, 08:00, 1.00, 1.00"), 10,
        "起始节点必须是仓库或中转站: D01", "[vehicles] 起始节点不能是客户节点");
    // 正例（规格七 #63）：慢车可以出生在**中转站**上（快车仍从仓库出发）。
    // 反向验证：把 ConfigLoader 的判据退回「必须是仓库」⇒ 本条立刻红，默认数据也加载不了。
    {
        const std::string text =
            "[nodes]\n"
            "W01, warehouse, 0, 0, 仓库, 0\n"
            "T01, transit, 5, 5, 中转站甲, 1\n"
            "D01, delivery, 10, 10, 客户一, 1\n"
            "D02, delivery, 20, 10, 客户二, 1\n"
            "[edges]\n"
            "W01, T01, 1.0, 2.0, 1.5\n"
            "T01, D01, 1.0, 2.0, 1.5\n"
            "D01, D02, 1.0, 2.0, 1.5\n"
            "[vehicles]\n"
            "V01, W01, 100, 08:00, 1.00, 1.00\n"
            "V02, T01, 100, 08:00, 1.00, 1.00\n"
            "[orders]\n"
            "O01, D01, D02, 10, 09:00, 18:00, 0\n";
        Config cfg;
        std::string error;
        const bool ok = ConfigLoader::parse(text, cfg, error);
        check(ok, "起始节点可以是中转站（规格七 #63）：" + error);
        check(cfg.vehicles.size() == 2 && cfg.vehicles[1].startNodeId == "T01",
              "从 T01 出发的车辆被正确读入");
    }
    expectParseFailureAtLineWithMessage(
        configWithVehicleRow("V01, W01, abc, 08:00, 1.00, 1.00"), 10,
        "载重上限不是合法数值", "[vehicles] 载重不是合法数值");
    expectParseFailureAtLineWithMessage(
        configWithVehicleRow("V01, W01, 0, 08:00, 1.00, 1.00"), 10, "载重上限必须为正数",
        "[vehicles] 载重必须为正数");
    expectParseFailureAtLineWithMessage(
        configWithVehicleRow("V01, W01, 100, 8点, 1.00, 1.00"), 10,
        "发车时刻格式应为 HH:MM", "[vehicles] 发车时刻格式非法");
    expectParseFailureAtLineWithMessage(
        configWithVehicleRow("V01, W01, 100, 25:00, 1.00, 1.00"), 10,
        "发车时刻格式应为 HH:MM", "[vehicles] 发车时刻超范围");
    expectParseFailureAtLineWithMessage(
        configWithVehicleRow("V01, W01, 100, 08:00, abc, 1.00"), 10,
        "耗时倍率不是合法数值", "[vehicles] 耗时倍率不是数值");
    expectParseFailureAtLineWithMessage(
        configWithVehicleRow("V01, W01, 100, 08:00, 0, 1.00"), 10, "耗时倍率必须为正数",
        "[vehicles] 耗时倍率必须为正数");
    expectParseFailureAtLineWithMessage(
        configWithVehicleRow("V01, W01, 100, 08:00, -1.5, 1.00"), 10, "耗时倍率必须为正数",
        "[vehicles] 耗时倍率为负");
    expectParseFailureAtLineWithMessage(
        configWithVehicleRow("V01, W01, 100, 08:00, 1.00, abc"), 10,
        "成本倍率不是合法数值", "[vehicles] 成本倍率不是数值");
    expectParseFailureAtLineWithMessage(
        configWithVehicleRow("V01, W01, 100, 08:00, 1.00, 0"), 10, "成本倍率必须为正数",
        "[vehicles] 成本倍率必须为正数");
    expectParseFailureAtLineWithMessage(
        configWithVehicleRow("V01, W01, 100, 08:00, , 1.00"), 10,
        "耗时倍率不是合法数值", "[vehicles] 耗时倍率为空");

    // ---- [orders] 十条校验（数据行在第 12 行） ----
    expectParseFailureAtLineWithMessage(
        configWithOrderRows("O01, D01, 10, 09:00, 18:00, 0"), 12,
        "需要 7 个字段，实际 6 个", "[orders] 旧 6 列按字段数报错");
    expectParseFailureAtLineWithMessage(
        configWithOrderRows("O01, D01, D02, 10, 09:00, 18:00, 0, 998"), 12,
        "需要 7 个字段，实际 8 个", "[orders] 多一列按字段数报错");
    expectParseFailureAtLineWithMessage(
        configWithOrderRows(", D01, D02, 10, 09:00, 18:00, 0"), 12, "订单 ID 为空",
        "[orders] 订单 ID 为空");
    expectParseFailureAtLineWithMessage(
        configWithOrderRows("O01, D01, D02, 10, 09:00, 18:00, 0\n"
                            "O01, D01, D02, 10, 09:00, 18:00, 0"),
        13, "订单 ID 重复: O01", "[orders] 订单 ID 重复（第 13 行）");
    expectParseFailureAtLineWithMessage(
        configWithOrderRows("O01, , D02, 10, 09:00, 18:00, 0"), 12, "取货点为空",
        "[orders] 取货点为空");
    expectParseFailureAtLineWithMessage(
        configWithOrderRows("O01, NOPE, D02, 10, 09:00, 18:00, 0"), 12,
        "取货点不存在: NOPE", "[orders] 取货点不存在");
    expectParseFailureAtLineWithMessage(
        configWithOrderRows("O01, W01, D02, 10, 09:00, 18:00, 0"), 12,
        "取货点必须是配送点: W01", "[orders] 取货点不是配送点");
    expectParseFailureAtLineWithMessage(
        configWithOrderRows("O01, D01, , 10, 09:00, 18:00, 0"), 12, "送达点为空",
        "[orders] 送达点为空");
    expectParseFailureAtLineWithMessage(
        configWithOrderRows("O01, D01, NOPE, 10, 09:00, 18:00, 0"), 12,
        "送达点不存在: NOPE", "[orders] 送达点不存在");
    expectParseFailureAtLineWithMessage(
        configWithOrderRows("O01, D01, W01, 10, 09:00, 18:00, 0"), 12,
        "送达点必须是配送点: W01", "[orders] 送达点不是配送点");
    expectParseFailureAtLineWithMessage(
        configWithOrderRows("O01, D01, D01, 10, 09:00, 18:00, 0"), 12,
        "取货点与送达点不能相同: D01", "[orders] A == B");
    expectParseFailureAtLineWithMessage(
        configWithOrderRows("O01, D01, D02, abc, 09:00, 18:00, 0"), 12,
        "货物量不是合法数值", "[orders] 货物量不是数值");
    expectParseFailureAtLineWithMessage(
        configWithOrderRows("O01, D01, D02, 0, 09:00, 18:00, 0"), 12, "货物量必须为正数",
        "[orders] 货物量为 0");
    expectParseFailureAtLineWithMessage(
        configWithOrderRows("O01, D01, D02, -5, 09:00, 18:00, 0"), 12, "货物量必须为正数",
        "[orders] 货物量为负");
    expectParseFailureAtLineWithMessage(
        configWithOrderRows("O01, D01, D02, 10, 9点, 18:00, 0"), 12,
        "时间窗口格式应为 HH:MM", "[orders] 窗口格式非法");
    expectParseFailureAtLineWithMessage(
        configWithOrderRows("O01, D01, D02, 10, 18:00, 09:00, 0"), 12,
        "窗口起必须早于窗口止", "[orders] 窗口起晚于止");
    expectParseFailureAtLineWithMessage(
        configWithOrderRows("O01, D01, D02, 10, 09:00, 18:00, 2"), 12,
        "is_urgent 只能为 0 或 1", "[orders] is_urgent 非法");

    // 跨节校验（Q1）：demand_kg 严格小于 min(capacity)
    expectParseFailureAtLineWithMessage(
        configWithOrderRows("O01, D01, D02, 10, 09:00, 18:00, 0",
                            "V01, W01, 10, 08:00, 1.00, 1.00"),
        12, "货物量 10 kg 不小于最小车辆载重 10 kg（任何车都装不下这一单）",
        "[orders] 单笔货量等于最小载重时报错");

    // ---- [general] 退役键白名单（每个键都带行号 + 迁移提示） ----
    const std::string baseWithGeneral = std::string("[general]\n");
    expectParseFailureAtLineWithMessage(baseWithGeneral + "service_time_min = 5\n" + base, 2,
                                        "service_time_min", "退役键 service_time_min");
    expectParseFailureAtLineWithMessage(baseWithGeneral + "event_weight_traffic = 10\n" + base, 2,
                                        "event_weight_traffic", "退役键 event_weight_traffic");
    expectParseFailureAtLineWithMessage(baseWithGeneral + "event_weight_urgent = 3\n" + base, 2,
                                        "event_weight_urgent", "退役键 event_weight_urgent");
    expectParseFailureAtLineWithMessage(baseWithGeneral + "event_weight_customer = 1\n" + base, 2,
                                        "event_weight_customer", "退役键 event_weight_customer");
    expectParseFailureAtLineWithMessage(baseWithGeneral + "event_weight_closure = 1\n" + base, 2,
                                        "event_weight_closure", "退役键 event_weight_closure");
    expectParseFailureAtLineWithMessage(
        baseWithGeneral + "traffic_change_interval_sec = 60\n" + base, 2,
        "traffic_change_interval_sec", "退役键 traffic_change_interval_sec");
    expectParseFailureAtLineWithMessage(
        baseWithGeneral + "urgent_order_interval_sec = 10\n" + base, 2,
        "urgent_order_interval_sec", "退役键 urgent_order_interval_sec");

    // 非退役的未知键仍按既有口径静默忽略（白名单之外不炸）
    {
        Config cfg;
        std::string error;
        const bool ok = ConfigLoader::parse(
            std::string("[general]\nsome_future_key = 1\n") + base, cfg, error);
        check(ok, "未知但非退役的键仍被忽略（错误: " + error + "）");
    }

    // 重复 section
    expectParseFailure(std::string(base) + "[nodes]\n", "重复 [section]");

    // 配置项出现在任何 section 之前
    expectParseFailure("key = value\n" + std::string(base), "配置项出现在节之前");

    // 数据行混入 '='
    expectParseFailure(
        "[nodes]\nW01 = warehouse\n[edges]\n[vehicles]\n[orders]\n", "数据行含 '='");

    // [section] 缺少右括号
    expectParseFailure("[nodes\nW01, warehouse, 0, 0, 仓, 0\n", "[section] 缺少右括号");
    // 报错必须带行号，且行号 = 该错误在配置里实际所在行（IniFile 层，第 1 行）
    expectParseFailureAtLine("[nodes\nW01, warehouse, 0, 0, 仓, 0\n", 1,
                             "IniFile 层：节缺右括号，行号应为 1");

    // 字段数不符（ConfigLoader 层，错误行在第 2 行）
    expectParseFailureAtLine(
        "[nodes]\nW01, warehouse, 0, 0\n[edges]\n[vehicles]\n[orders]\n", 2,
        "ConfigLoader 层：[nodes] 字段数不符，行号应为 2");

    // 错误行不在首行时也要带对行号（[edges] 节的数据行在第 5 行）
    expectParseFailureAtLine(
        "[nodes]\nW01, warehouse, 0, 0, 仓, 0\nD01, delivery, 1, 1, 客, 0\n"
        "[edges]\nW01, NOPE, 1, 1, 1\n[vehicles]\n[orders]\n", 5,
        "边端点不存在，行号应为 5");

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
            check(cfg.orders.size() == 1 && cfg.orders[0].pickupNodeId == "N1" &&
                      cfg.orders[0].nodeId == "N2",
                  "七列订单的 A/B 解析正确");
        }
    }

    // 空订单列表：应合法（[orders] 节存在但无数据行）
    {
        Config cfg;
        std::string error;
        const bool ok = ConfigLoader::parse(
            "[nodes]\nW01, warehouse, 0, 0, 仓, 0\n"
            "[edges]\n[vehicles]\nV01, W01, 100, 08:00, 1.00, 1.00\n[orders]\n", cfg, error);
        check(ok, "空订单列表合法（错误: " + error + "）");
        check(ok && cfg.orders.empty(), "空订单列表解析为空");
    }

    // MC-D05 / B7：单笔货量 == 最小载重 → **不合法**（严格小于）
    {
        const std::string text = configWithOrderRows(
            "O01, D01, D02, 10, 09:00, 18:00, 0", "V01, W01, 10, 08:00, 1.00, 1.00");
        expectParseFailureAtLine(text, 12, "单笔货量等于最小载重时应报错（B7）");
    }
    // 略小于最小载重 → 合法
    {
        Config cfg;
        std::string error;
        const bool ok = ConfigLoader::parse(
            configWithOrderRows("O01, D01, D02, 9.5, 09:00, 18:00, 0",
                                "V01, W01, 10, 08:00, 1.00, 1.00"),
            cfg, error);
        check(ok, "单笔货量略小于最小载重时合法（错误: " + error + "）");
        if (ok) {
            check(cfg.orders.size() == 1 && cfg.orders[0].demandKg == 9.5,
                  "小数货量 9.5 保留");
        }
    }

    // 时间窗口卡点：到达时刻恰好等于窗口止不算超时（此处只验证配置可加载）
    {
        Config cfg;
        std::string error;
        const bool ok = ConfigLoader::parse(
            configWithOrderRows("O01, D01, D02, 10, 08:00, 08:02, 0"), cfg, error);
        check(ok, "紧时间窗口配置可加载（错误: " + error + "）");
        if (ok) {
            check(cfg.orders[0].windowStartMin == 480 && cfg.orders[0].windowEndMin == 482,
                  "08:00-08:02 解析为 480-482 分钟");
        }
    }

    // 六列车辆的因子确实被读取（非默认值）
    {
        Config cfg;
        std::string error;
        const bool ok = ConfigLoader::parse(
            configWithVehicleRow("V01, W01, 160, 08:15, 1.25, 0.80"), cfg, error);
        check(ok, "六列车辆可加载（错误: " + error + "）");
        if (ok) {
            check(cfg.vehicles.size() == 1, "恰有 1 台车");
            check(cfg.vehicles[0].capacityKg == 160.0, "载重 160");
            check(cfg.vehicles[0].departTimeMin == 8 * 60 + 15, "发车 08:15");
            check(cfg.vehicles[0].timeFactor == 1.25, "耗时倍率 1.25（非默认）");
            check(cfg.vehicles[0].costFactor == 0.80, "成本倍率 0.80（非默认）");
        }
    }

    // 可选 [general]：完全不提供时使用默认值（含事件等级回落）
    {
        Config cfg;
        std::string error;
        const bool ok = ConfigLoader::parse(kMinimalConfig, cfg, error);
        check(ok, "无 [general] 节时使用默认值（错误: " + error + "）");
        check(ok && cfg.general.trafficChangeRatio == 0.1,
              "默认 traffic_change_ratio == 0.1");
        check(ok && cfg.general.eventIntervalMin == 15,
              "默认 event_interval_min == 15");
        check(ok && cfg.general.eventLevelTraffic == EventLevel::High,
              "缺键回落 traffic = high");
        check(ok && cfg.general.eventLevelCustomer == EventLevel::Medium,
              "缺键回落 customer = medium");
        check(ok && cfg.general.eventLevelUrgent == EventLevel::Low,
              "缺键回落 urgent = low");
        check(ok && cfg.general.eventLevelClosure == EventLevel::ExtraLow,
              "缺键回落 closure = extra_low");
    }

    // [general] 存在但缺事件等级键：逐个回落到各自默认等级
    {
        Config cfg;
        std::string error;
        const bool ok = ConfigLoader::parse(std::string("[general]\n")
                                                + "traffic_change_ratio = 0.25\n"
                                                + kMinimalConfig, cfg, error);
        check(ok, "[general] 缺事件键时用默认等级（错误: " + error + "）");
        check(ok && cfg.general.trafficChangeRatio == 0.25, "已给键取文件值");
        check(ok && cfg.general.eventIntervalMin == 15,
              "缺口键 event_interval_min 回落到 15");
        check(ok && cfg.general.eventLevelTraffic == EventLevel::High,
              "缺口键 event_level_traffic 回落到 high");
        check(ok && cfg.general.eventLevelCustomer == EventLevel::Medium,
              "缺口键 event_level_customer 回落到 medium");
        check(ok && cfg.general.eventLevelUrgent == EventLevel::Low,
              "缺口键 event_level_urgent 回落到 low");
        check(ok && cfg.general.eventLevelClosure == EventLevel::ExtraLow,
              "缺口键 event_level_closure 回落到 extra_low");
    }

    // 事件等级键**确实被读取**——而不是"恰好永远等于默认值"。
    // 喂非默认等级：删掉任意一条绑定，对应断言立刻失败。
    {
        Config cfg;
        std::string error;
        const bool ok = ConfigLoader::parse(std::string("[general]\n")
                                                + "event_interval_min = 7\n"
                                                + "event_level_traffic = extra_low\n"
                                                + "event_level_customer = high\n"
                                                + "event_level_urgent = medium\n"
                                                + "event_level_closure = low\n"
                                                + kMinimalConfig, cfg, error);
        check(ok, "非默认事件等级可加载（错误: " + error + "）");
        check(ok && cfg.general.eventIntervalMin == 7, "已给键取文件值 event_interval_min == 7");
        check(ok && cfg.general.eventLevelTraffic == EventLevel::ExtraLow,
              "traffic == extra_low（非默认）");
        check(ok && cfg.general.eventLevelCustomer == EventLevel::High,
              "customer == high（非默认）");
        check(ok && cfg.general.eventLevelUrgent == EventLevel::Medium,
              "urgent == medium（非默认）");
        check(ok && cfg.general.eventLevelClosure == EventLevel::Low,
              "closure == low（非默认）");
        check(ok && logistics::eventLevelWeight(cfg.general.eventLevelTraffic) == 1, "1");
        check(ok && logistics::eventLevelWeight(cfg.general.eventLevelCustomer) == 27, "27");
        check(ok && logistics::eventLevelWeight(cfg.general.eventLevelUrgent) == 9, "9");
        check(ok && logistics::eventLevelWeight(cfg.general.eventLevelClosure) == 3, "3");
    }

    // 值 trim 后不区分大小写；键名仍精确匹配
    {
        Config cfg;
        std::string error;
        const bool ok = ConfigLoader::parse(std::string("[general]\n")
                                                + "event_level_traffic = HIGH\n"
                                                + kMinimalConfig, cfg, error);
        check(ok, "事件等级大小写不敏感（错误: " + error + "）");
        check(ok && cfg.general.eventLevelTraffic == EventLevel::High, "HIGH -> high");
    }

    // 非法等级值：带行号报错，不猜、不回落
    expectParseFailureAtLineWithMessage(std::string("[general]\n")
                                            + "event_level_traffic = hihg\n" + kMinimalConfig,
                                        2, "event_level_traffic 取值非法: hihg（允许 extra_low/low/medium/high）",
                                        "非法事件等级值");
    expectParseFailureAtLineWithMessage(std::string("[general]\n")
                                            + "event_level_closure = 27\n" + kMinimalConfig,
                                        2, "event_level_closure 取值非法: 27",
                                        "事件等级写成权重数字也应报错");

    // 等级 → 固定表 1/3/9/27 的唯一派生
    check(logistics::eventLevelWeight(EventLevel::ExtraLow) == 1, "extra_low -> 1");
    check(logistics::eventLevelWeight(EventLevel::Low) == 3, "low -> 3");
    check(logistics::eventLevelWeight(EventLevel::Medium) == 9, "medium -> 9");
    check(logistics::eventLevelWeight(EventLevel::High) == 27, "high -> 27");
}

} // namespace

int main() {
    testLoadDefaultConfig();
    testThreeVehicleFixture();
    testDefaultConfigRoutingPreconditions();
    testOrderSourcePrefixes();
    testDynamicAddRemove();
    testErrorCases();
    testBoundaryCases();
    return testutil::summarize("config_tests");
}
