#include "io/ConfigLoader.h"

#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "io/IniFile.h"
#include "io/StringUtil.h"

namespace logistics {

namespace {

void fail(std::string& error, int lineNo, const char* section, const std::string& msg) {
    error = "第 " + std::to_string(lineNo) + " 行 [" + section + "]: " + msg;
}

bool parseDouble(const std::string& s, double& out) {
    if (s.empty()) {
        return false;
    }
    char* end = nullptr;
    const double value = std::strtod(s.c_str(), &end);
    if (end == nullptr || *end != '\0' || !std::isfinite(value)) {
        return false;
    }
    out = value;
    return true;
}

bool parseInt(const std::string& s, int& out) {
    double value = 0.0;
    if (!parseDouble(s, value) || value != std::floor(value)) {
        return false;
    }
    out = static_cast<int>(value);
    return true;
}

// "HH:MM" -> 相对当日 00:00 的分钟数
bool parseClockMinutes(const std::string& s, int& out) {
    const std::size_t colon = s.find(':');
    if (colon == std::string::npos) {
        return false;
    }
    int hour = 0;
    int minute = 0;
    if (!parseInt(trim(s.substr(0, colon)), hour) || !parseInt(trim(s.substr(colon + 1)), minute)) {
        return false;
    }
    if (hour < 0 || hour > 23 || minute < 0 || minute > 59) {
        return false;
    }
    out = hour * 60 + minute;
    return true;
}

// 数值 → 报错文案里的写法：整数不带小数点，小数按最小必要位数。
std::string formatNumber(double value) {
    if (std::isfinite(value) && value == std::floor(value) && std::fabs(value) < 1e15) {
        return std::to_string(static_cast<long long>(value));
    }
    std::ostringstream os;
    os << std::setprecision(10) << value;
    return os.str();
}

// 数据行：记录本体 + 已按逗号切分并裁剪的字段
struct Row {
    const IniRecord*         rec = nullptr;
    std::vector<std::string> fields;
};

bool collectRows(const IniSection& sec, const char* name, std::size_t expectedFields,
                 std::vector<Row>& out, std::string& error) {
    for (const IniRecord& r : sec.records) {
        if (r.isKeyValue) {
            fail(error, r.lineNo, name, "数据行不应包含 '=' : " + r.text);
            return false;
        }
        Row row;
        row.rec = &r;
        row.fields = splitAndTrim(r.text, ',');
        if (row.fields.size() != expectedFields) {
            fail(error, r.lineNo, name,
                 "需要 " + std::to_string(expectedFields) + " 个字段，实际 "
                     + std::to_string(row.fields.size()) + " 个: " + r.text);
            return false;
        }
        out.push_back(row);
    }
    return true;
}

bool requireSection(const IniFile& ini, const char* name, const IniSection*& out, std::string& error) {
    out = ini.findSection(name);
    if (out == nullptr) {
        error = std::string("缺少 [") + name + "] 节";
        return false;
    }
    return true;
}

// 退役键白名单（规格一 #23 §3.4）：出现即带行号报错 + 迁移提示；其余未知键仍忽略。
// service_time_min **保持不存在**：不重新引入、不参与任何计算（ADR 0002 Q3）。
bool retiredGeneralKeyHint(const std::string& key, std::string& hint) {
    if (key == "service_time_min") {
        hint = "停靠零耗时（ADR 0002 Q3），该键保持不存在、不参与任何计算，请删除";
        return true;
    }
    if (key == "event_weight_traffic" || key == "event_weight_urgent"
        || key == "event_weight_customer" || key == "event_weight_closure") {
        const std::string levelKey = "event_level_" + key.substr(std::string("event_weight_").size());
        hint = "概率改由等级派生，请改用 " + levelKey
               + " = extra_low|low|medium|high（代码内固定表 1/3/9/27）";
        return true;
    }
    if (key == "traffic_change_interval_sec") {
        hint = "事件只由软件内事件刻驱动，请改用 event_interval_min（软件内分钟）";
        return true;
    }
    if (key == "urgent_order_interval_sec") {
        hint = "紧急订单改由事件刻与待处理上限驱动，请删除该键";
        return true;
    }
    return false;
}

bool loadGeneral(const IniFile& ini, GeneralConfig& out, std::string& error) {
    const IniSection* sec = ini.findSection("general");
    if (sec == nullptr) {
        return true;  // [general] 可选，缺省用默认值
    }

    // 退役键先查：即使数值本身能解析也必须报错，否则旧配置会静默变默认值。
    for (const IniRecord& rec : sec->records) {
        if (!rec.isKeyValue) {
            continue;
        }
        std::string hint;
        if (retiredGeneralKeyHint(rec.key, hint)) {
            error = "第 " + std::to_string(rec.lineNo) + " 行 [general]: " + rec.key
                    + " 已退役；" + hint;
            return false;
        }
    }

    struct Binder {
        const char* key;
        double*     target;
    };
    const Binder binders[] = {
        {"traffic_change_ratio", &out.trafficChangeRatio},
        {"traffic_time_increase_min", &out.trafficTimeIncreaseMin},
        {"traffic_time_increase_max", &out.trafficTimeIncreaseMax},
    };
    struct IntBinder {
        const char* key;
        int*        target;
    };
    const IntBinder intBinders[] = {
        {"event_interval_min", &out.eventIntervalMin},
    };
    for (const Binder& b : binders) {
        const IniRecord* rec = sec->findValue(b.key);
        if (rec == nullptr) {
            continue;
        }
        double value = 0.0;
        if (!parseDouble(rec->value, value)) {
            fail(error, rec->lineNo, "general", std::string(b.key) + " 不是合法数值: " + rec->value);
            return false;
        }
        *b.target = value;
    }
    for (const IntBinder& b : intBinders) {
        const IniRecord* rec = sec->findValue(b.key);
        if (rec == nullptr) {
            continue;
        }
        int value = 0;
        if (!parseInt(rec->value, value)) {
            fail(error, rec->lineNo, "general", std::string(b.key) + " 不是合法整数: " + rec->value);
            return false;
        }
        *b.target = value;
    }

    // 事件等级键（#23 §3.3）：缺键回落各自默认等级，非法值带行号报错、不猜不回落。
    // 键名精确匹配（沿用 IniFile::findValue）；值 trim 后不区分大小写。
    struct LevelBinder {
        const char* key;
        EventLevel* target;
    };
    const LevelBinder levelBinders[] = {
        {"event_level_traffic", &out.eventLevelTraffic},
        {"event_level_customer", &out.eventLevelCustomer},
        {"event_level_urgent", &out.eventLevelUrgent},
        {"event_level_closure", &out.eventLevelClosure},
    };
    for (const LevelBinder& b : levelBinders) {
        const IniRecord* rec = sec->findValue(b.key);
        if (rec == nullptr) {
            continue;
        }
        EventLevel value = EventLevel::ExtraLow;
        if (!parseEventLevel(rec->value, value)) {
            fail(error, rec->lineNo, "general",
                 std::string(b.key) + " 取值非法: " + rec->value
                     + "（允许 extra_low/low/medium/high）");
            return false;
        }
        *b.target = value;
    }
    return true;
}

bool loadNodes(const IniFile& ini, LogisticsGraph& graph, std::string& error) {
    const IniSection* sec = nullptr;
    if (!requireSection(ini, "nodes", sec, error)) {
        return false;
    }
    std::vector<Row> rows;
    if (!collectRows(*sec, "nodes", 6, rows, error)) {
        return false;
    }
    for (const Row& row : rows) {
        Node node;
        node.id = row.fields[0];
        if (node.id.empty()) {
            fail(error, row.rec->lineNo, "nodes", "节点 ID 为空");
            return false;
        }
        if (!parseNodeType(row.fields[1], node.type)) {
            fail(error, row.rec->lineNo, "nodes", "未知节点类型: " + row.fields[1]);
            return false;
        }
        if (!parseDouble(row.fields[2], node.x) || !parseDouble(row.fields[3], node.y)) {
            fail(error, row.rec->lineNo, "nodes", "坐标不是合法数值: " + row.fields[2] + "," + row.fields[3]);
            return false;
        }
        node.name = row.fields[4];
        if (!parseInt(row.fields[5], node.subNetworkId)) {
            fail(error, row.rec->lineNo, "nodes", "sub_network_id 不是整数: " + row.fields[5]);
            return false;
        }
        if (!graph.addNode(node)) {
            fail(error, row.rec->lineNo, "nodes", "节点 ID 重复: " + node.id);
            return false;
        }
    }
    return true;
}

bool loadEdges(const IniFile& ini, LogisticsGraph& graph, std::string& error) {
    const IniSection* sec = nullptr;
    if (!requireSection(ini, "edges", sec, error)) {
        return false;
    }
    std::vector<Row> rows;
    if (!collectRows(*sec, "edges", 5, rows, error)) {
        return false;
    }
    for (const Row& row : rows) {
        Edge edge;
        edge.fromId = row.fields[0];
        edge.toId = row.fields[1];
        if (!parseDouble(row.fields[2], edge.distanceKm)
            || !parseDouble(row.fields[3], edge.timeMin)
            || !parseDouble(row.fields[4], edge.costYuan)) {
            fail(error, row.rec->lineNo, "edges", "权重不是合法数值");
            return false;
        }
        if (edge.distanceKm < 0.0 || edge.timeMin < 0.0 || edge.costYuan < 0.0) {
            fail(error, row.rec->lineNo, "edges", "权重不能为负: " + row.rec->text);
            return false;
        }
        if (!graph.addEdge(edge)) {
            fail(error, row.rec->lineNo, "edges",
                 "端点不存在或 (from, to) 重复: " + edge.fromId + " -> " + edge.toId);
            return false;
        }
    }
    return true;
}

// [vehicles] 六列 + 八条校验（规格一 #19 Q1、规格二 §3.1）。
bool loadVehicles(const IniFile& ini, const LogisticsGraph& graph,
                  std::vector<Vehicle>& out, std::string& error) {
    const IniSection* sec = nullptr;
    if (!requireSection(ini, "vehicles", sec, error)) {
        return false;
    }
    std::vector<Row> rows;
    if (!collectRows(*sec, "vehicles", 6, rows, error)) {
        return false;
    }
    // 与 [orders] 不同：车队不能为空（没有车就没有任何配送能力）。
    if (rows.empty()) {
        error = "[vehicles] 节为空：至少需要 1 台车";
        return false;
    }
    std::set<std::string> seenIds;
    for (const Row& row : rows) {
        Vehicle vehicle;
        vehicle.id = row.fields[0];
        if (vehicle.id.empty()) {
            fail(error, row.rec->lineNo, "vehicles", "车辆 ID 为空");
            return false;
        }
        if (!seenIds.insert(vehicle.id).second) {
            fail(error, row.rec->lineNo, "vehicles", "车辆 ID 重复: " + vehicle.id);
            return false;
        }
        vehicle.startNodeId = row.fields[1];
        if (vehicle.startNodeId.empty()) {
            fail(error, row.rec->lineNo, "vehicles", "起始节点为空");
            return false;
        }
        const Node* start = graph.findNode(vehicle.startNodeId);
        if (start == nullptr) {
            fail(error, row.rec->lineNo, "vehicles", "起始节点不存在: " + vehicle.startNodeId);
            return false;
        }
        // 起始节点 = 仓库 **或** 中转站（规格七 #63）：慢车出生在**本簇中转站**上，
        // 这是「慢车不出簇」的前提 —— 出生在仓库就意味着每天先要一段跨簇通勤，
        // 而断路之后那段路根本不存在。客户节点仍然不许当起点（车不能驻在客户门口）。
        if (start->type != NodeType::Warehouse && start->type != NodeType::Transit) {
            fail(error, row.rec->lineNo, "vehicles",
                 "起始节点必须是仓库或中转站: " + vehicle.startNodeId);
            return false;
        }
        if (!parseDouble(row.fields[2], vehicle.capacityKg)) {
            fail(error, row.rec->lineNo, "vehicles", "载重上限不是合法数值");
            return false;
        }
        if (vehicle.capacityKg <= 0.0) {
            fail(error, row.rec->lineNo, "vehicles", "载重上限必须为正数");
            return false;
        }
        if (!parseClockMinutes(row.fields[3], vehicle.departTimeMin)) {
            fail(error, row.rec->lineNo, "vehicles", "发车时刻格式应为 HH:MM");
            return false;
        }
        if (!parseDouble(row.fields[4], vehicle.timeFactor)) {
            fail(error, row.rec->lineNo, "vehicles", "耗时倍率不是合法数值");
            return false;
        }
        if (vehicle.timeFactor <= 0.0) {
            fail(error, row.rec->lineNo, "vehicles", "耗时倍率必须为正数");
            return false;
        }
        if (!parseDouble(row.fields[5], vehicle.costFactor)) {
            fail(error, row.rec->lineNo, "vehicles", "成本倍率不是合法数值");
            return false;
        }
        if (vehicle.costFactor <= 0.0) {
            fail(error, row.rec->lineNo, "vehicles", "成本倍率必须为正数");
            return false;
        }
        out.push_back(vehicle);
    }
    return true;
}

// [orders] 七列 + 十条校验 + 跨节严格小于（规格一 #23 §一 / Q1）。
bool loadOrders(const IniFile& ini, const LogisticsGraph& graph,
                const std::vector<Vehicle>& vehicles, std::vector<Order>& out,
                std::string& error) {
    const IniSection* sec = nullptr;
    if (!requireSection(ini, "orders", sec, error)) {
        return false;
    }
    std::vector<Row> rows;
    if (!collectRows(*sec, "orders", 7, rows, error)) {
        return false;
    }
    // 空节合法（0 单，测试夹具要用）；非空时按 cross-section 口径取最小载重。
    double minCapacityKg = 0.0;
    for (std::size_t i = 0; i < vehicles.size(); ++i) {
        if (i == 0 || vehicles[i].capacityKg < minCapacityKg) {
            minCapacityKg = vehicles[i].capacityKg;
        }
    }

    std::set<std::string> seenIds;
    for (const Row& row : rows) {
        Order order;
        order.id = row.fields[0];
        if (order.id.empty()) {
            fail(error, row.rec->lineNo, "orders", "订单 ID 为空");
            return false;
        }
        if (!seenIds.insert(order.id).second) {
            fail(error, row.rec->lineNo, "orders", "订单 ID 重复: " + order.id);
            return false;
        }
        order.pickupNodeId = row.fields[1];
        order.nodeId = row.fields[2];
        if (order.pickupNodeId.empty()) {
            fail(error, row.rec->lineNo, "orders", "取货点为空");
            return false;
        }
        if (order.nodeId.empty()) {
            fail(error, row.rec->lineNo, "orders", "送达点为空");
            return false;
        }
        const Node* pickup = graph.findNode(order.pickupNodeId);
        if (pickup == nullptr) {
            fail(error, row.rec->lineNo, "orders", "取货点不存在: " + order.pickupNodeId);
            return false;
        }
        if (pickup->type != NodeType::Delivery) {
            fail(error, row.rec->lineNo, "orders", "取货点必须是配送点: " + order.pickupNodeId);
            return false;
        }
        const Node* delivery = graph.findNode(order.nodeId);
        if (delivery == nullptr) {
            fail(error, row.rec->lineNo, "orders", "送达点不存在: " + order.nodeId);
            return false;
        }
        if (delivery->type != NodeType::Delivery) {
            fail(error, row.rec->lineNo, "orders", "送达点必须是配送点: " + order.nodeId);
            return false;
        }
        if (order.pickupNodeId == order.nodeId) {
            // 报同一个节点 ID（与规则 3 的 <ID> 同指节点 ID）；行号已能定位订单行。
            fail(error, row.rec->lineNo, "orders",
                 "取货点与送达点不能相同: " + order.pickupNodeId);
            return false;
        }
        if (!parseDouble(row.fields[3], order.demandKg)) {
            fail(error, row.rec->lineNo, "orders", "货物量不是合法数值");
            return false;
        }
        if (order.demandKg <= 0.0) {
            fail(error, row.rec->lineNo, "orders", "货物量必须为正数");
            return false;
        }
        if (!parseClockMinutes(row.fields[4], order.windowStartMin)
            || !parseClockMinutes(row.fields[5], order.windowEndMin)) {
            fail(error, row.rec->lineNo, "orders", "时间窗口格式应为 HH:MM");
            return false;
        }
        if (order.windowStartMin >= order.windowEndMin) {
            fail(error, row.rec->lineNo, "orders", "窗口起必须早于窗口止");
            return false;
        }
        int urgent = 0;
        if (!parseInt(row.fields[6], urgent) || (urgent != 0 && urgent != 1)) {
            fail(error, row.rec->lineNo, "orders", "is_urgent 只能为 0 或 1");
            return false;
        }
        order.urgent = (urgent == 1);
        // Q1 跨节校验（本人拍板）：demand_kg < min(capacity_kg) **严格小于**，
        // 口径是"任何车都能至少单独装一个订单"。取 min、不写死 200。
        if (!vehicles.empty() && !(order.demandKg < minCapacityKg)) {
            fail(error, row.rec->lineNo, "orders",
                 "货物量 " + formatNumber(order.demandKg) + " kg 不小于最小车辆载重 "
                     + formatNumber(minCapacityKg) + " kg（任何车都装不下这一单）");
            return false;
        }
        out.push_back(order);
    }
    return true;
}

// 把已解析的 INI 组装成 Config。load 与 parse 共用这一条路径，保证行为一致。
bool buildFromIni(const IniFile& ini, Config& out, std::string& error) {
    out = Config{};
    if (!loadGeneral(ini, out.general, error)
        || !loadNodes(ini, out.graph, error)
        || !loadEdges(ini, out.graph, error)
        || !loadVehicles(ini, out.graph, out.vehicles, error)
        || !loadOrders(ini, out.graph, out.vehicles, out.orders, error)) {
        return false;
    }
    return true;
}

} // namespace

bool ConfigLoader::parse(const std::string& text, Config& out, std::string& error) {
    IniFile ini;
    if (!IniFile::parse(text, ini, error)) {
        return false;
    }
    return buildFromIni(ini, out, error);
}

bool ConfigLoader::load(const std::string& path, Config& out, std::string& error) {
    IniFile ini;
    if (!IniFile::loadFromFile(path, ini, error)) {
        return false;
    }
    return buildFromIni(ini, out, error);
}

} // namespace logistics
