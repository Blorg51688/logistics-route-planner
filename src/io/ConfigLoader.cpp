#include "io/ConfigLoader.h"

#include <cmath>
#include <cstdlib>
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

bool loadGeneral(const IniFile& ini, GeneralConfig& out, std::string& error) {
    const IniSection* sec = ini.findSection("general");
    if (sec == nullptr) {
        return true;  // [general] 可选，缺省用默认值
    }
    struct Binder {
        const char* key;
        double*     target;
    };
    const Binder binders[] = {
        {"traffic_change_interval_sec", &out.trafficChangeIntervalSec},
        {"traffic_change_ratio", &out.trafficChangeRatio},
        {"traffic_time_increase_min", &out.trafficTimeIncreaseMin},
        {"traffic_time_increase_max", &out.trafficTimeIncreaseMax},
        {"urgent_order_interval_sec", &out.urgentOrderIntervalSec},
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

bool loadVehicles(const IniFile& ini, const LogisticsGraph& graph,
                  std::vector<Vehicle>& out, std::string& error) {
    const IniSection* sec = nullptr;
    if (!requireSection(ini, "vehicles", sec, error)) {
        return false;
    }
    std::vector<Row> rows;
    if (!collectRows(*sec, "vehicles", 4, rows, error)) {
        return false;
    }
    for (const Row& row : rows) {
        Vehicle vehicle;
        vehicle.id = row.fields[0];
        vehicle.startNodeId = row.fields[1];
        if (!parseDouble(row.fields[2], vehicle.capacityKg)) {
            fail(error, row.rec->lineNo, "vehicles", "载重上限不是合法数值: " + row.fields[2]);
            return false;
        }
        if (vehicle.capacityKg <= 0.0) {
            fail(error, row.rec->lineNo, "vehicles", "载重上限必须为正数");
            return false;
        }
        if (!parseClockMinutes(row.fields[3], vehicle.departTimeMin)) {
            fail(error, row.rec->lineNo, "vehicles", "发车时刻格式应为 HH:MM: " + row.fields[3]);
            return false;
        }
        const Node* start = graph.findNode(vehicle.startNodeId);
        if (start == nullptr) {
            fail(error, row.rec->lineNo, "vehicles", "起始节点不存在: " + vehicle.startNodeId);
            return false;
        }
        if (start->type != NodeType::Warehouse) {
            fail(error, row.rec->lineNo, "vehicles", "起始节点必须是仓库: " + vehicle.startNodeId);
            return false;
        }
        out.push_back(vehicle);
    }
    return true;
}

bool loadOrders(const IniFile& ini, const LogisticsGraph& graph,
                std::vector<Order>& out, std::string& error) {
    const IniSection* sec = nullptr;
    if (!requireSection(ini, "orders", sec, error)) {
        return false;
    }
    std::vector<Row> rows;
    if (!collectRows(*sec, "orders", 6, rows, error)) {
        return false;
    }
    for (const Row& row : rows) {
        Order order;
        order.id = row.fields[0];
        order.nodeId = row.fields[1];
        if (!parseDouble(row.fields[2], order.demandKg)) {
            fail(error, row.rec->lineNo, "orders", "货物量不是合法数值: " + row.fields[2]);
            return false;
        }
        if (order.demandKg <= 0.0) {
            fail(error, row.rec->lineNo, "orders", "货物量必须为正数: " + row.fields[2]);
            return false;
        }
        if (!parseClockMinutes(row.fields[3], order.windowStartMin)
            || !parseClockMinutes(row.fields[4], order.windowEndMin)) {
            fail(error, row.rec->lineNo, "orders", "时间窗口格式应为 HH:MM");
            return false;
        }
        if (order.windowStartMin >= order.windowEndMin) {
            fail(error, row.rec->lineNo, "orders",
                 "窗口起必须早于窗口止: " + row.fields[3] + "-" + row.fields[4]);
            return false;
        }
        int urgent = 0;
        if (!parseInt(row.fields[5], urgent) || (urgent != 0 && urgent != 1)) {
            fail(error, row.rec->lineNo, "orders", "is_urgent 只能为 0 或 1: " + row.fields[5]);
            return false;
        }
        order.urgent = (urgent == 1);

        const Node* node = graph.findNode(order.nodeId);
        if (node == nullptr) {
            fail(error, row.rec->lineNo, "orders", "配送点不存在: " + order.nodeId);
            return false;
        }
        if (node->type != NodeType::Delivery) {
            fail(error, row.rec->lineNo, "orders", "订单目标必须是配送点: " + order.nodeId);
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
        || !loadOrders(ini, out.graph, out.orders, error)) {
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
