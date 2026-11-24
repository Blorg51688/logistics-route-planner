// L2 文本守卫：跑**真二进制**读 --plan-summary 的四行 stdout，断言
//   G1 标签/单位/顺序与规格一 §6.3 逐字一致；
//   G2 13 字段数值 == tests/plan_golden.h（单一真值表，容差 §6.4）；
//   G3 A == 2 × 取送对数 且 S ≤ A；
//   G6 分母 0 打印「—」（无紧急单时是「紧急 —」）；
//   L3 用 --dump-plan + oracle 复算（tools/oracle.py --plan）做算术对拍。
//
// 用法（CMake 传参）：cli_plan_summary <app 路径> <策略|all>
//   策略 ∈ distance|cost|time|all（all = 三种策略都过）。
//
// 本文件**只消费** tests/plan_golden.h，不写第二份真值（§6.4 / §10 裁定 4）。

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <string>
#include <vector>

#include <unistd.h>

#include "core/Config.h"
#include "core/RoutePlanner.h"
#include "io/ConfigLoader.h"
#include "plan_golden.h"
#include "test_util.h"

#ifndef DEFAULT_CONFIG_PATH
#error "缺少 DEFAULT_CONFIG_PATH 编译定义"
#endif
#ifndef PYTHON_EXECUTABLE
#error "缺少 PYTHON_EXECUTABLE 编译定义"
#endif
#ifndef ORACLE_PY_PATH
#error "缺少 ORACLE_PY_PATH 编译定义"
#endif

using testutil::check;

namespace {

std::string runCapture(const std::string& command) {
    std::string out;
    FILE* pipe = popen(command.c_str(), "r");
    if (pipe == nullptr) {
        return out;
    }
    char buf[4096];
    while (std::fgets(buf, sizeof(buf), pipe) != nullptr) {
        out += buf;
    }
    pclose(pipe);
    return out;
}

std::vector<std::string> splitLines(const std::string& text) {
    std::vector<std::string> lines;
    std::string cur;
    for (char c : text) {
        if (c == '\n') {
            if (!cur.empty()) {
                lines.push_back(cur);
            }
            cur.clear();
        } else if (c != '\r') {
            cur.push_back(c);
        }
    }
    if (!cur.empty()) {
        lines.push_back(cur);
    }
    return lines;
}

// 在 s 中找 marker，返回其后第一个数值；找不到 marker 返回 NAN。
// 用于按标签定位数值，避免依赖固定列宽。
bool numberAfter(const std::string& s, const std::string& marker, double& out,
                 std::size_t* endPos = nullptr) {
    const std::size_t p = s.find(marker);
    if (p == std::string::npos) {
        return false;
    }
    std::size_t i = p + marker.size();
    while (i < s.size() && (s[i] == ' ' || s[i] == '\t')) {
        ++i;
    }
    const char* start = s.c_str() + i;
    char* end = nullptr;
    out = std::strtod(start, &end);
    if (end == start) {
        return false;
    }
    if (endPos != nullptr) {
        *endPos = static_cast<std::size_t>(end - s.c_str());
    }
    return true;
}

// 取 marker 之后的一个「率」字段：可能是「NN.N%」或「—」。
// 返回 true 表示拿到了数值（isDash=false）；isDash=true 表示打印的是「—」。
bool rateAfter(const std::string& s, const std::string& marker, double& out, bool& isDash) {
    const std::size_t p = s.find(marker);
    if (p == std::string::npos) {
        return false;
    }
    std::size_t i = p + marker.size();
    while (i < s.size() && (s[i] == ' ' || s[i] == '\t')) {
        ++i;
    }
    if (i + 2 < s.size() && s.compare(i, 3, "—") == 0) {
        isDash = true;
        out = 0.0;
        return true;
    }
    const char* start = s.c_str() + i;
    char* end = nullptr;
    out = std::strtod(start, &end);
    if (end == start) {
        return false;
    }
    isDash = false;
    return true;
}

// 第 3–5 行（#43 / §7.4）：每车一行 7 项。停靠数**打印但不进 102 常量**
// （§7.2：逐车路线序列已冻结，停靠数是它的函数），这里解析出来只做结构核对。
struct ParsedVehicle {
    std::string id;
    double distance = 0.0, time = 0.0, cost = 0.0;
    int penalty = 0, stops = 0;
    double utilization = 0.0;
    double pickup = 0.0, delivery = 0.0;
    int pickupHit = 0, pickupTotal = 0, deliveryHit = 0, deliveryTotal = 0;
    bool pickupDash = false, deliveryDash = false;
};

struct Parsed {
    double distance = 0.0, time = 0.0, cost = 0.0;
    int penalty = 0, stops = 0, actions = 0;
    double pickup = 0.0, delivery = 0.0, urgent = 0.0, normal = 0.0;
    bool urgentDash = false;
    double avgLoad = 0.0, utilization = 0.0, peak = 0.0;
    std::vector<ParsedVehicle> vehicles;          // 行序 = [vehicles] 行序
    std::vector<std::vector<std::string> > routes;  // 第 7–9 行的节点序列
};

// 断言一串 token 在 text 中按顺序出现（G1：标签与单位顺序与 §6.3 一致）。
void checkOrdered(const std::string& text, const std::vector<std::string>& tokens,
                  const std::string& tag) {
    std::size_t pos = 0;
    for (const std::string& tok : tokens) {
        const std::size_t at = text.find(tok, pos);
        check(at != std::string::npos, tag + "：标签/单位「" + tok + "」按序出现");
        if (at == std::string::npos) {
            return;
        }
        pos = at + tok.size();
    }
}

// 「rate（a/b）」里的 (a/b)：返回 marker 后第一个括号里的两个整数。
bool ratioAfter(const std::string& s, const std::string& marker, int& hit, int& total) {
    const std::size_t p = s.find(marker);
    if (p == std::string::npos) {
        return false;
    }
    const std::size_t open = s.find("（", p);
    const std::size_t close = s.find("）", open == std::string::npos ? p : open);
    if (open == std::string::npos || close == std::string::npos || close <= open) {
        return false;
    }
    return std::sscanf(s.c_str() + open + std::string("（").size(), "%d/%d", &hit, &total) == 2;
}

// 解析 --plan-summary 的 **9 行**（§7.4 行序固定）：
//   0–1 车队汇总 / 2–4 每车 7 项 / 5 逐单超时 / 6–8 每车路线。
// vehicleCount 由配置给出：行数 == 3 + 2 × 车数（默认三车 = 9）。
bool parseSummary(const std::string& text, Parsed& p, std::vector<std::string>& lines,
                  int vehicleCount) {
    lines = splitLines(text);
    if (lines.size() < static_cast<std::size_t>(3 + 2 * vehicleCount)) {
        return false;
    }
    const std::string& l1 = lines[0];
    const std::string& l2 = lines[1];
    bool ok = true;
    double v = 0.0;
    ok = numberAfter(l1, "总距离 ", p.distance) && ok;
    ok = numberAfter(l1, "总耗时 ", p.time) && ok;
    ok = numberAfter(l1, "总成本 ", p.cost) && ok;
    ok = numberAfter(l1, "总 penalty ", v) && ok;
    p.penalty = static_cast<int>(std::lround(v));
    ok = numberAfter(l1, "停靠 ", v) && ok;
    p.stops = static_cast<int>(std::lround(v));
    ok = numberAfter(l1, "动作 ", v) && ok;
    p.actions = static_cast<int>(std::lround(v));

    bool dash = false;
    ok = rateAfter(l2, "取货准时率 ", p.pickup, dash) && ok;
    ok = rateAfter(l2, "送达准时率 ", p.delivery, dash) && ok;
    ok = rateAfter(l2, "紧急 ", p.urgent, p.urgentDash) && ok;
    ok = rateAfter(l2, "普通 ", p.normal, dash) && ok;
    ok = numberAfter(l2, "平均在车 ", p.avgLoad) && ok;
    ok = numberAfter(l2, "利用率 ", p.utilization) && ok;
    ok = numberAfter(l2, "峰值 ", p.peak) && ok;

    // 第 3–5 行：每车 7 项（行序 = [vehicles]）。
    for (int vi = 0; vi < vehicleCount; ++vi) {
        const std::string& line = lines[static_cast<std::size_t>(2 + vi)];
        ParsedVehicle pv;
        const std::size_t bar = line.find(" | ");
        if (line.rfind("车 ", 0) != 0 || bar == std::string::npos) {
            ok = false;
            continue;
        }
        pv.id = line.substr(std::string("车 ").size(),
                            bar - std::string("车 ").size());
        ok = numberAfter(line, "距离 ", pv.distance) && ok;
        ok = numberAfter(line, "耗时 ", pv.time) && ok;
        ok = numberAfter(line, "成本 ", pv.cost) && ok;
        ok = numberAfter(line, "penalty ", v) && ok;
        pv.penalty = static_cast<int>(std::lround(v));
        ok = numberAfter(line, "停靠 ", v) && ok;
        pv.stops = static_cast<int>(std::lround(v));
        ok = numberAfter(line, "利用率 ", pv.utilization) && ok;
        ok = rateAfter(line, "取货 ", pv.pickup, pv.pickupDash) && ok;
        ok = rateAfter(line, "送达 ", pv.delivery, pv.deliveryDash) && ok;
        ok = ratioAfter(line, "取货 ", pv.pickupHit, pv.pickupTotal) && ok;
        ok = ratioAfter(line, "送达 ", pv.deliveryHit, pv.deliveryTotal) && ok;
        p.vehicles.push_back(pv);
    }

    // 第 7–9 行：路线 <VID> <节点...>；节点之间单个空格。
    for (int vi = 0; vi < vehicleCount; ++vi) {
        const std::string& line = lines[static_cast<std::size_t>(3 + vehicleCount + vi)];
        std::vector<std::string> nodes;
        const std::size_t sp = line.find(' ');
        const std::size_t sp2 = (sp == std::string::npos) ? std::string::npos
                                                          : line.find(' ', sp + 1);
        if (line.rfind("路线 ", 0) != 0 || sp == std::string::npos) {
            ok = false;
            p.routes.push_back(nodes);
            continue;
        }
        // 「路线 <VID> <节点…>」：跳过车辆 ID，只收节点序列（空车只有起点一个节点）。
        const std::string rest =
            (sp2 == std::string::npos) ? std::string() : line.substr(sp2 + 1);
        std::string cur;
        for (char c : rest) {
            if (c == ' ') {
                if (!cur.empty()) {
                    nodes.push_back(cur);
                    cur.clear();
                }
            } else {
                cur.push_back(c);
            }
        }
        if (!cur.empty()) {
            nodes.push_back(cur);
        }
        p.routes.push_back(nodes);
    }
    return ok;
}

std::map<std::string, std::string> parseKeyVals(const std::string& text) {
    std::map<std::string, std::string> kv;
    for (const std::string& line : splitLines(text)) {
        const std::size_t eq = line.find('=');
        if (eq == std::string::npos) {
            continue;
        }
        std::string key = line.substr(0, eq);
        std::string val = line.substr(eq + 1);
        auto trim = [](std::string& s) {
            const char* ws = " \t";
            const std::size_t b = s.find_first_not_of(ws);
            const std::size_t e = s.find_last_not_of(ws);
            s = (b == std::string::npos) ? std::string() : s.substr(b, e - b + 1);
        };
        trim(key);
        trim(val);
        kv[key] = val;
    }
    return kv;
}

bool near(double a, double b, double tol) {
    return std::fabs(a - b) <= tol + 1e-9;
}

// ---- 规格二 §4（#40）：--dump-plan 的 ALLOC / ASSIGN 行（用户可见输出）----
//
//   ALLOC  <orderId>=<vehicleId>,…      按订单 ID 升序
//   ASSIGN <vehicleId> <簇N | 跨簇 | 空车> <订单数>
//
// 这是**分配的可核对视图**：ALLOC 给逐单归属，ASSIGN 给逐车汇总。
// 守卫读这两行 + 既有的 PLAN/STOP 行，不读规划器内部状态。
struct MetricRow {
    double distance = 0.0, time = 0.0, cost = 0.0, avgLoad = 0.0, utilization = 0.0,
           peak = 0.0;
    int penalty = 0, stopCount = 0, actionCount = 0, pickupHit = 0, pickupTotal = 0,
        deliveryHit = 0, deliveryTotal = 0;
};

struct AllocDump {
    bool hasAlloc = false;
    bool hasAssign = false;
    std::vector<std::string> orderIds;      // ALLOC 的订单 ID（应升序）
    std::vector<std::string> vehicleIds;    // 与 orderIds 一一对应
    std::vector<std::string> assignVehicle; // ASSIGN 行的车辆
    std::vector<std::string> assignLabel;   // 簇N / 跨簇 / 空车
    std::vector<int> assignCount;           // 该行订单数
    // 每台车**送达**的订单 ID（STOP 行的 deliveries 列），用于与 ALLOC 双向对拍。
    std::map<std::string, std::vector<std::string> > deliveredByVehicle;
    // 每车 METRICS 自报值（--dump-plan 的「用户可见」逐车指标行）。
    std::map<std::string, MetricRow> metricsByVehicle;
};

std::vector<std::string> splitSpaces(const std::string& line) {
    std::vector<std::string> out;
    std::string cur;
    for (char c : line) {
        if (c == ' ' || c == '\t') {
            if (!cur.empty()) {
                out.push_back(cur);
                cur.clear();
            }
        } else {
            cur.push_back(c);
        }
    }
    if (!cur.empty()) {
        out.push_back(cur);
    }
    return out;
}

AllocDump parseAllocDump(const std::string& text) {
    AllocDump d;
    std::string curVehicle;
    for (const std::string& line : splitLines(text)) {
        const std::vector<std::string> tok = splitSpaces(line);
        if (tok.empty()) {
            continue;
        }
        if (tok[0] == "PLAN" && tok.size() >= 2) {
            curVehicle = tok[1];
        } else if (tok[0] == "STOP" && !curVehicle.empty() && tok.size() >= 9) {
            if (tok[8] != "-") {
                std::string item;
                for (char c : tok[8]) {
                    if (c == ',') {
                        const std::size_t colon = item.find(':');
                        d.deliveredByVehicle[curVehicle].push_back(
                            colon == std::string::npos ? item : item.substr(0, colon));
                        item.clear();
                    } else {
                        item.push_back(c);
                    }
                }
                if (!item.empty()) {
                    const std::size_t colon = item.find(':');
                    d.deliveredByVehicle[curVehicle].push_back(
                        colon == std::string::npos ? item : item.substr(0, colon));
                }
            }
        } else if (tok[0] == "METRICS" && tok.size() >= 19) {
            MetricRow m;
            m.distance = std::strtod(tok[2].c_str(), nullptr);
            m.time = std::strtod(tok[3].c_str(), nullptr);
            m.cost = std::strtod(tok[4].c_str(), nullptr);
            m.penalty = std::atoi(tok[5].c_str());
            m.stopCount = std::atoi(tok[6].c_str());
            m.actionCount = std::atoi(tok[7].c_str());
            m.pickupHit = std::atoi(tok[8].c_str());
            m.pickupTotal = std::atoi(tok[9].c_str());
            m.deliveryHit = std::atoi(tok[10].c_str());
            m.deliveryTotal = std::atoi(tok[11].c_str());
            m.avgLoad = std::strtod(tok[16].c_str(), nullptr);
            m.utilization = std::strtod(tok[17].c_str(), nullptr);
            m.peak = std::strtod(tok[18].c_str(), nullptr);
            d.metricsByVehicle[tok[1]] = m;
        } else if (tok[0] == "ALLOC" && tok.size() >= 2) {
            d.hasAlloc = true;
            std::string item;
            for (char c : tok[1]) {
                if (c == ',') {
                    const std::size_t eq = item.find('=');
                    if (eq != std::string::npos) {
                        d.orderIds.push_back(item.substr(0, eq));
                        d.vehicleIds.push_back(item.substr(eq + 1));
                    }
                    item.clear();
                } else {
                    item.push_back(c);
                }
            }
            if (!item.empty()) {
                const std::size_t eq = item.find('=');
                if (eq != std::string::npos) {
                    d.orderIds.push_back(item.substr(0, eq));
                    d.vehicleIds.push_back(item.substr(eq + 1));
                }
            }
        } else if (tok[0] == "ASSIGN" && tok.size() >= 4) {
            d.hasAssign = true;
            d.assignVehicle.push_back(tok[1]);
            d.assignLabel.push_back(tok[2]);
            d.assignCount.push_back(std::atoi(tok[3].c_str()));
        }
    }
    return d;
}

} // namespace


// --dump-plan 的 TRANSIT 行（规格二 §5.3 / #41）：**只加行、不改既有行**。
// 用一个小夹具配置跑真二进制：时间策略接受（V02 在 T01 卸、V01 取走），
// 距离/成本策略不启用 ⇒ 没有 TRANSIT 行。断言读的是 stdout 文本。
void checkDumpPlanTransitLines(const std::string& appPath) {
    const std::string tmp = "/tmp/cli_plan_summary_transit_"
                            + std::to_string(static_cast<long long>(getpid())) + ".ini";
    {
        std::ofstream f(tmp.c_str());
        f << "[general]\n"
             "event_interval_min = 1440\n"
             "event_level_traffic = high\n"
             "event_level_customer = medium\n"
             "event_level_urgent = low\n"
             "event_level_closure = extra_low\n"
             "\n[nodes]\n"
             "W01, warehouse, 80, 160, 中央仓库, 0\n"
             "T01, transit, 380, 200, 中转站甲, 1\n"
             "D01, delivery, 180, 90, 客户01, 1\n"
             "D02, delivery, 200, 180, 客户02, 1\n"
             "D03, delivery, 250, 280, 客户03, 1\n"
             "D18, delivery, 620, 120, 客户18, 2\n"
             "\n[edges]\n"
             "W01, T01, 1.0, 1.0, 1.0\n"
             "T01, W01, 1.0, 1.0, 1.0\n"
             "W01, D01, 1.0, 1.0, 1.0\n"
             "D01, W01, 1.0, 1.0, 1.0\n"
             "D01, T01, 1.0, 1.0, 1.0\n"
             "T01, D01, 1.0, 1.0, 1.0\n"
             "W01, D02, 1.0, 1.0, 1.0\n"
             "D02, W01, 1.0, 1.0, 1.0\n"
             "D02, T01, 2.0, 2.0, 2.0\n"
             "T01, D02, 2.0, 2.0, 2.0\n"
             "T01, D18, 1.0, 1.0, 1.0\n"
             "D18, T01, 1.0, 1.0, 1.0\n"
             "D18, D03, 1.0, 1.0, 1.0\n"
             "D03, D18, 1.0, 1.0, 1.0\n"
             "\n[vehicles]\n"
             "V01, W01, 400, 08:00, 1.00, 1.50\n"
             "V02, W01, 160, 08:00, 1.25, 0.80\n"
             "\n[orders]\n"
             "O2, D02, D03, 10, 00:00, 23:59, 0\n"
             "O1, D01, D18, 10, 00:00, 23:59, 0\n";
    }
    const std::string timeOut =
        runCapture("'" + appPath + "' --config '" + tmp + "' --dump-plan time 2>/dev/null");
    const std::vector<std::string> timeLines = splitLines(timeOut);
    int transitLines = 0;
    bool shapeOk = true;
    for (const std::string& line : timeLines) {
        if (line.rfind("TRANSIT ", 0) != 0) {
            continue;
        }
        ++transitLines;
        // TRANSIT <订单> <站> <kg> <卸入车> <HH:MM> <取走车> <HH:MM> <dwell> <状态>
        const std::vector<std::string> tokens = splitSpaces(line);
        if (tokens.size() != 10 || tokens[0] != "TRANSIT" || tokens[1] != "O1"
            || tokens[2] != "T01" || tokens[4] == tokens[6] || tokens[9] != "已送达") {
            shapeOk = false;
        }
        const int dwell = std::atoi(tokens[8].c_str());
        if (dwell < 0 || dwell > 60) {
            shapeOk = false;
        }
    }
    check(transitLines == 1,
          "TRANSIT：时间策略下 --dump-plan 恰有一行 TRANSIT（实际 "
              + std::to_string(transitLines) + "）");
    check(shapeOk, "TRANSIT：字段与账本同序（订单/站/kg/卸入/取走/等待/状态），in≠out、dwell≤60");
    const std::string distanceOut =
        runCapture("'" + appPath + "' --config '" + tmp + "' --dump-plan distance 2>/dev/null");
    bool noTransit = true;
    for (const std::string& line : splitLines(distanceOut)) {
        if (line.rfind("TRANSIT ", 0) == 0) {
            noTransit = false;
        }
    }
    check(noTransit, "TRANSIT：距离策略两版并列 ⇒ 保留禁转运版（无 TRANSIT 行）");
    std::remove(tmp.c_str());
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::printf("用法: cli_plan_summary <app 路径> <策略|all>\n");
        return 2;
    }
    const std::string appPath = argv[1];
    const std::string strategy = (argc >= 3) ? argv[2] : "all";

    logistics::Config cfg;
    std::string error;
    if (!logistics::ConfigLoader::load(DEFAULT_CONFIG_PATH, cfg, error)) {
        std::printf("FAIL  默认配置加载失败：%s\n", error.c_str());
        return 1;
    }
    const int orderCount = static_cast<int>(cfg.orders.size());
    // 规格二第 2 轮捕获（#43 / §7.1）：真值表冻结的是**三车** 102 常量。
    // 车辆数变了就必须重捕；这里先断言口径一致，防止逐车 63 项被静默跳过。
    const int vehicleCount = static_cast<int>(cfg.vehicles.size());
    check(vehicleCount == plan_golden::kVehicleCount,
          "默认数据车辆数 == 真值表冻结车辆数（" + std::to_string(vehicleCount) + " vs "
              + std::to_string(plan_golden::kVehicleCount) + "）");

    // 转运账本行（独立入口：--dump-plan，不影响 --plan-summary 九行）。
    checkDumpPlanTransitLines(appPath);

    std::vector<std::string> strategies;
    if (strategy == "all") {
        strategies = {"distance", "cost", "time"};
    } else {
        strategies = {strategy};
    }

    std::map<std::string, std::vector<std::string> > allocByStrategy;
    std::map<std::string, Parsed> parsedByStrategy;

    for (const std::string& name : strategies) {
        const plan_golden::StrategyGolden* g = nullptr;
        for (const plan_golden::StrategyGolden& cand : plan_golden::kAll) {
            if (name == cand.name) {
                g = &cand;
            }
        }
        if (g == nullptr) {
            std::printf("FAIL  未知策略：%s\n", name.c_str());
            testutil::g_failures++;
            continue;
        }
        const std::string tag = name;

        // ---- 跑真二进制：--plan-summary ----
        const std::string summaryCmd =
            "'" + appPath + "' --plan-summary " + name + " 2>/dev/null";
        const std::string out = runCapture(summaryCmd);
        Parsed p;
        std::vector<std::string> lines;
        check(parseSummary(out, p, lines, vehicleCount),
              tag + "：--plan-summary 输出 9 行（3 + 2 × 车数）且可解析");

        // ---- G1：标签 / 单位 / 顺序（规格二 §7.4 的 **9 行**，行序固定）----
        // 第 1–2 行车队；第 3–5 行每车 7 项；第 6 行超时订单**逐单且带车辆**；
        // 第 7–9 行每车一条路线。
        const std::vector<std::string> tokens1 = {"总距离", "km", "总耗时", "min", "总成本", "元",
                                                  "总 penalty", "min", "停靠", "站 · 动作", "次"};
        const std::vector<std::string> tokens2 = {
            "取货准时率", "%", "） | 送达准时率", "；紧急 ", "普通 ", "平均在车", "kg",
            "利用率 ", "% · 峰值 ", "kg"};
        const int lateIndex = 2 + vehicleCount;
        if (lines.size() >= static_cast<std::size_t>(3 + 2 * vehicleCount)) {
            checkOrdered(lines[0], tokens1, tag + "·G1 第1行");
            checkOrdered(lines[1], tokens2, tag + "·G1 第2行");
            // 第 3–5 行：每车一行 7 项；标签/单位/分隔符顺序逐字。
            const std::vector<std::string> vehicleTokens = {
                " | 距离 ", " km | 耗时 ", " min | 成本 ", " 元 | penalty ",
                " | 停靠 ", " | 利用率 ", "% | 取货 ", "%（", "） · 送达 ", "%（", "）"};
            for (int vi = 0; vi < vehicleCount; ++vi) {
                const std::string& line = lines[static_cast<std::size_t>(2 + vi)];
                const std::string want = "车 " + cfg.vehicles[static_cast<std::size_t>(vi)].id;
                check(line.rfind(want + " |", 0) == 0,
                      tag + "·G1 第" + std::to_string(3 + vi) + "行以「" + want
                          + " |」开头（车辆行序）");
                checkOrdered(line, vehicleTokens,
                             tag + "·G1 第" + std::to_string(3 + vi) + "行（每车 7 项）");
            }
            const std::string& lateLine = lines[static_cast<std::size_t>(lateIndex)];
            check(lateLine.rfind("超时订单：", 0) == 0,
                  tag + "·G1 第" + std::to_string(lateIndex + 1) + "行以「超时订单：」开头");
            if (p.penalty == 0) {
                check(lateLine == "超时订单：（无）",
                      tag + "·G1 无迟到单时超时行为「超时订单：（无）」");
            } else {
                // 多车逐单格式：`ID@节点(车 VID, +nmin)、…`
                const std::string body = lateLine.substr(std::strlen("超时订单："));
                bool shapeOk = !body.empty();
                std::size_t pos = 0;
                while (pos < body.size()) {
                    const std::size_t sep = body.find("、", pos);
                    const std::string one = body.substr(pos, sep == std::string::npos
                                                                ? std::string::npos
                                                                : sep - pos);
                    if (one.find('@') == std::string::npos
                        || one.find("(车 ") == std::string::npos
                        || one.find(", +") == std::string::npos
                        || one.size() < 10
                        || one.compare(one.size() - 4, 4, "min)") != 0) {
                        shapeOk = false;
                    }
                    if (sep == std::string::npos) {
                        break;
                    }
                    pos = sep + std::strlen("、");
                }
                check(shapeOk, tag + "·G1 超时行逐单格式 `ID@节点(车 VID, +nmin)、…`");
            }
            // 第 7–9 行：每车一条路线，行序 = [vehicles] 行序。
            for (int vi = 0; vi < vehicleCount; ++vi) {
                const std::string want = "路线 " + cfg.vehicles[static_cast<std::size_t>(vi)].id;
                const std::string& line =
                    lines[static_cast<std::size_t>(3 + vehicleCount + vi)];
                check(line.rfind(want + " ", 0) == 0,
                      tag + "·G1 第" + std::to_string(4 + vehicleCount + vi) + "行以「" + want
                          + " 」开头（车辆行序）");
                // 空车只打印起点（§7.4）
                const std::string nodes = line.substr(want.size() + 1);
                check(!nodes.empty(), tag + "·G1 路线行含节点序列（空车只有起点）");
                if (nodes.find("  ") != std::string::npos) {
                    check(false, tag + "·G1 路线行节点之间用单个空格分隔");
                }
            }
        }

        // ---- G2：**102 个常量**（车队 13×3 + 每车 7×3×3，§7.2 容差）----
        check(near(p.distance, g->distanceKm, plan_golden::kTolMetric), tag + "·G2 总距离");
        check(near(p.time, g->timeMin, plan_golden::kTolMetric), tag + "·G2 总耗时（完工耗时）");
        check(near(p.cost, g->costYuan, plan_golden::kTolMetric), tag + "·G2 总成本");
        check(p.penalty == g->penaltyMin, tag + "·G2 总 penalty");
        check(p.stops == g->stopCount, tag + "·G2 停靠点数 S");
        check(p.actions == g->actionCount, tag + "·G2 动作数 A");
        check(near(p.pickup, g->pickupRatePct, plan_golden::kTolRate), tag + "·G2 取货准时率");
        check(near(p.delivery, g->deliveryRatePct, plan_golden::kTolRate),
              tag + "·G2 送达准时率·总");
        check(near(p.normal, g->normalRatePct, plan_golden::kTolRate),
              tag + "·G2 送达准时率·普通");
        check(near(p.avgLoad, g->avgLoadKg, plan_golden::kTolRate), tag + "·G2 平均在车");
        check(near(p.utilization, g->utilizationPct, plan_golden::kTolRate), tag + "·G2 利用率");
        check(near(p.peak, g->peakLoadKg, plan_golden::kTolRate), tag + "·G2 峰值");

        // 每车 7 字段 × 3 车：ADR 0008 的核心（V02↔V03 互换时车队级 13 项全不变）。
        check(static_cast<int>(p.vehicles.size()) == vehicleCount,
              tag + "·G2 每车行数 == 车辆数（缺行 = 63 项被静默跳过）");
        for (int vi = 0; vi < vehicleCount && vi < static_cast<int>(p.vehicles.size()); ++vi) {
            const ParsedVehicle& pv = p.vehicles[static_cast<std::size_t>(vi)];
            const plan_golden::VehicleGolden& vg = g->vehicles[vi];
            const std::string vtag = tag + "·G2 车 " + vg.id;
            check(pv.id == vg.id, vtag + " 车辆 ID/行序与真值表一致");
            check(near(pv.distance, vg.distanceKm, plan_golden::kTolMetric),
                  vtag + " 距离");
            check(near(pv.time, vg.timeMin, plan_golden::kTolMetric), vtag + " 耗时");
            check(near(pv.cost, vg.costYuan, plan_golden::kTolMetric), vtag + " 成本");
            check(pv.penalty == vg.penaltyMin, vtag + " penalty");
            check(near(pv.utilization, vg.utilizationPct, plan_golden::kTolRate),
                  vtag + " 利用率（÷ 该车 capacity）");
            check(pv.pickupTotal > 0 && pv.deliveryTotal > 0,
                  vtag + " 两率分母 = 该车订单数（> 0）");
            check(near(pv.pickup, vg.pickupRatePct, plan_golden::kTolRate)
                      && pv.pickupHit == pv.pickupTotal,
                  vtag + " 取货准时率");
            check(near(pv.delivery, vg.deliveryRatePct, plan_golden::kTolRate)
                      && pv.deliveryHit == pv.deliveryTotal,
                  vtag + " 送达准时率");
            // 停靠数**不冻结**（§7.2），但必须与路线序列长度自洽：
            // 路线行里每个停靠节点至少出现一次；这里只做 S ≤ 动作数的旧结构约束。
            check(pv.stops <= p.actions, vtag + " 停靠数 ≤ 车队动作数（结构）");
        }

        // ---- G17：每车路线序列**逐字**（§7.3；与真值表 routeNodes 完全相等）----
        check(static_cast<int>(p.routes.size()) == vehicleCount,
              tag + "·G17 路线行数 == 车辆数");
        for (int vi = 0; vi < vehicleCount && vi < static_cast<int>(p.routes.size()); ++vi) {
            const std::vector<std::string>& nodes = p.routes[static_cast<std::size_t>(vi)];
            const std::string& want = g->routeNodes[vi];
            std::string got;
            for (std::size_t n = 0; n < nodes.size(); ++n) {
                if (n > 0) {
                    got += " ";
                }
                got += nodes[n];
            }
            check(got == want, tag + "·G17 路线 " + cfg.vehicles[static_cast<std::size_t>(vi)].id
                                   + " 逐字一致\n        实际: " + got + "\n        真值: " + want);
            check(nodes.empty() || nodes.front() == cfg.vehicles[static_cast<std::size_t>(vi)].startNodeId,
                  tag + "·G17 路线从该车起始仓库出发");
        }

        // ---- G6：第 9 项（紧急）在分母 0 时打印「紧急 —」----
        if (g->urgentRatePct == plan_golden::kNoDenominatorSentinel) {
            check(p.urgentDash, tag + "·G6 无紧急单时打印「紧急 —」（不是 0%/100%）");
        } else {
            check(!p.urgentDash && near(p.urgent, g->urgentRatePct, plan_golden::kTolRate),
                  tag + "·G2 送达准时率·紧急");
        }

        // ---- G3：A == 2 × 取送对数 且 S ≤ A ----
        check(p.actions == 2 * orderCount,
              tag + "·G3 动作数 A == 2 × 取送对数（" + std::to_string(orderCount) + "）");
        check(p.stops <= p.actions, tag + "·G3 停靠点数 S ≤ 动作数 A");

        // ---- L3：--dump-plan + oracle 逐车复算再汇总，与 app 自报值对拍 ----
        const std::string dumpText = runCapture("'" + appPath + "' --dump-plan " + name
                                                + " 2>/dev/null");
        char tmpPath[256];
        std::snprintf(tmpPath, sizeof(tmpPath), "/tmp/cli_plan_summary_%s_%d.txt", name.c_str(),
                      static_cast<int>(getpid()));
        if (!dumpText.empty()) {
            FILE* fp = std::fopen(tmpPath, "w");
            if (fp != nullptr) {
                std::fwrite(dumpText.data(), 1, dumpText.size(), fp);
                std::fclose(fp);
            }
        }
        const std::string oracleCmd = std::string("'") + PYTHON_EXECUTABLE + "' '" + ORACLE_PY_PATH
                                      + "' --plan '" + tmpPath + "' " + name + " 2>/dev/null";
        const std::string oracleOut = runCapture(oracleCmd);
        std::remove(tmpPath);
        const std::map<std::string, std::string> ov = parseKeyVals(oracleOut);
        check(!ov.empty(), tag + "·L3 oracle --plan 有输出");

        auto ovNum = [&](const std::string& key) -> double {
            const auto it = ov.find(key);
            return (it == ov.end()) ? NAN : std::strtod(it->second.c_str(), nullptr);
        };
        check(near(ovNum("distance"), p.distance, plan_golden::kTolMetric), tag + "·L3 距离对拍");
        check(near(ovNum("time"), p.time, plan_golden::kTolMetric), tag + "·L3 耗时对拍");
        check(near(ovNum("cost"), p.cost, plan_golden::kTolMetric), tag + "·L3 成本对拍");
        check(static_cast<int>(std::lround(ovNum("penalty"))) == p.penalty, tag + "·L3 penalty 对拍");
        check(static_cast<int>(std::lround(ovNum("stops"))) == p.stops, tag + "·L3 停靠对拍");
        check(static_cast<int>(std::lround(ovNum("actions"))) == p.actions, tag + "·L3 动作对拍");
        check(near(ovNum("pickup_rate"), p.pickup, plan_golden::kTolRate), tag + "·L3 取货率对拍");
        check(near(ovNum("delivery_rate"), p.delivery, plan_golden::kTolRate),
              tag + "·L3 送达率对拍");
        check(near(ovNum("normal_rate"), p.normal, plan_golden::kTolRate), tag + "·L3 普通率对拍");
        check(near(ovNum("avg_load"), p.avgLoad, plan_golden::kTolRate), tag + "·L3 平均在车对拍");
        check(near(ovNum("utilization"), p.utilization, plan_golden::kTolRate),
              tag + "·L3 利用率对拍");
        check(near(ovNum("peak"), p.peak, plan_golden::kTolRate), tag + "·L3 峰值对拍");
        const auto uit = ov.find("urgent_rate");
        const bool oracleUrgentDash = (uit != ov.end() && uit->second == "—");
        check(oracleUrgentDash == p.urgentDash, tag + "·L3 紧急率「—」一致性");
        // 车队恒等式（§3.6-8）：完工耗时 ≠ 车时合计；Σ每车耗时 == 车时合计。
        check(ov.find("completion") != ov.end() && ov.find("sum_time") != ov.end(),
              tag + "·L3 oracle 输出完工耗时与车时合计");
        if (ov.find("completion") != ov.end()) {
            check(near(ovNum("completion"), p.time, plan_golden::kTolMetric),
                  tag + "·L3 完工耗时对拍");
        }
        if (vehicleCount > 1 && ov.find("sum_time") != ov.end()) {
            check(ovNum("sum_time") > ovNum("completion") + 1e-6,
                  tag + "·L3 车时合计 > 完工耗时（两口径不互相顶替）");
        }
        // 逐车对拍（§7.2 的每车 7 字段来源）：oracle 每车都打印 `<VID>.<字段> = 值`。
        for (int vi = 0; vi < vehicleCount && vi < static_cast<int>(p.vehicles.size()); ++vi) {
            const ParsedVehicle& pv = p.vehicles[static_cast<std::size_t>(vi)];
            const std::string pre = pv.id + ".";
            const std::string vtag = tag + "·L3 车 " + pv.id;
            check(ov.find(pre + "distance") != ov.end() && ov.find(pre + "time") != ov.end()
                      && ov.find(pre + "cost") != ov.end()
                      && ov.find(pre + "penalty") != ov.end()
                      && ov.find(pre + "utilization") != ov.end()
                      && ov.find(pre + "pickup_rate") != ov.end()
                      && ov.find(pre + "delivery_rate") != ov.end(),
                  vtag + "：oracle 输出该车 7 字段（距离/耗时/成本/penalty/利用率/两率）");
            check(near(ovNum(pre + "distance"), pv.distance, plan_golden::kTolMetric),
                  vtag + " 距离对拍");
            check(near(ovNum(pre + "time"), pv.time, plan_golden::kTolMetric), vtag + " 耗时对拍");
            check(near(ovNum(pre + "cost"), pv.cost, plan_golden::kTolMetric), vtag + " 成本对拍");
            check(static_cast<int>(std::lround(ovNum(pre + "penalty"))) == pv.penalty,
                  vtag + " penalty 对拍");
            check(near(ovNum(pre + "utilization"), pv.utilization, plan_golden::kTolRate),
                  vtag + " 利用率对拍（÷ 该车 capacity）");
            check(near(ovNum(pre + "pickup_rate"), pv.pickup, plan_golden::kTolRate),
                  vtag + " 取货准时率对拍");
            check(near(ovNum(pre + "delivery_rate"), pv.delivery, plan_golden::kTolRate),
                  vtag + " 送达准时率对拍");
        }

        // ---- B/C（评审修复）：逐车 METRICS 反过来核验车队级第 12 项与每车停靠/动作数 ----
        //
        // B：车队级「载重利用率」是**派生聚合**：Σ(在车量×段时长) / Σ(该车 capacity×该车运行时长)。
        //    用 --dump-plan 的逐车 METRICS（平均在车 kg × 该车耗时）反算，与 --plan-summary
        //    车队第 12 项比对（容差 0.1）。G5 只钉逐车，这条守住车队级口径。
        // C：每车行打印的「停靠 s」必须等于该车 METRICS 的 stopCount；Σ每车停靠 == 车队第 5 项、
        //    Σ每车动作 == 车队第 6 项。路线行打的是 plan.nodes（含途经节点），不能代替停靠数。
        {
            const AllocDump md = parseAllocDump(dumpText);
            double loadIntegral = 0.0;
            double capacityTime = 0.0;
            double timeSum = 0.0;
            int sumStops = 0;
            int sumActions = 0;
            bool allMetrics = true;
            for (int vi = 0; vi < vehicleCount; ++vi) {
                const ParsedVehicle& pv = p.vehicles[static_cast<std::size_t>(vi)];
                const std::map<std::string, MetricRow>::const_iterator mit =
                    md.metricsByVehicle.find(pv.id);
                if (mit == md.metricsByVehicle.end()) {
                    allMetrics = false;
                    continue;
                }
                const MetricRow& mr = mit->second;
                check(pv.stops == mr.stopCount,
                      tag + "·C 车 " + pv.id + " 停靠数 == --dump-plan METRICS stopCount（"
                          + std::to_string(pv.stops) + " vs "
                          + std::to_string(mr.stopCount) + "）");
                sumStops += mr.stopCount;
                sumActions += mr.actionCount;
                if (mr.time > 0.0) {
                    loadIntegral += mr.avgLoad * mr.time;
                    capacityTime +=
                        cfg.vehicles[static_cast<std::size_t>(vi)].capacityKg * mr.time;
                    timeSum += mr.time;
                }
            }
            check(allMetrics, tag + "·C 每车都有 --dump-plan METRICS 行");
            check(sumStops == p.stops,
                  tag + "·C Σ每车停靠 == 车队第 5 项（" + std::to_string(sumStops) + " vs "
                      + std::to_string(p.stops) + "）");
            check(sumActions == p.actions,
                  tag + "·C Σ每车动作 == 车队第 6 项（" + std::to_string(sumActions) + " vs "
                      + std::to_string(p.actions) + "）");
            check(timeSum > 0.0 && capacityTime > 0.0, tag + "·B 逐车 METRICS 有正耗时");
            if (timeSum > 0.0 && capacityTime > 0.0) {
                const double fleetAvgFromVehicles = loadIntegral / timeSum;
                const double fleetUtilFromVehicles = 100.0 * loadIntegral / capacityTime;
                check(near(fleetAvgFromVehicles, p.avgLoad, plan_golden::kTolRate),
                      tag + "·B 车队平均在车 = Σ(逐车平均×该车耗时)/Σ耗时（时间加权，容差 0.1）："
                          + std::to_string(fleetAvgFromVehicles) + " vs "
                          + std::to_string(p.avgLoad));
                check(near(fleetUtilFromVehicles, p.utilization, plan_golden::kTolRate),
                      tag + "·B 车队第 12 项利用率 = 时间加权 Σ(在车量×段时长)/Σ(capacity×运行时长)"
                          "（容差 0.1）：" + std::to_string(fleetUtilFromVehicles) + " vs "
                          + std::to_string(p.utilization));
            }
        }

        // ---- G11（#40）：--dump-plan 的 ALLOC / ASSIGN 让分配可核对 ----
        //
        // ALLOC 给逐单归属（按订单 ID 升序），ASSIGN 给逐车汇总；两者都要与
        // 逐车 STOP 的送达动作**双向对拍**。守卫读的是用户可见输出，不读规划器内部。
        {
            const AllocDump ad = parseAllocDump(dumpText);
            check(ad.hasAlloc, tag + "·G11 转储含 ALLOC 行");
            check(ad.hasAssign, tag + "·G11 转储含 ASSIGN 行");
            check(ad.orderIds.size() == static_cast<std::size_t>(orderCount),
                  tag + "·G11 ALLOC 覆盖全部订单（" + std::to_string(ad.orderIds.size()) + " / "
                      + std::to_string(orderCount) + "）");
            bool sortedIds = true;
            for (std::size_t i = 1; i < ad.orderIds.size(); ++i) {
                if (!(ad.orderIds[i - 1] < ad.orderIds[i])) {
                    sortedIds = false;
                }
            }
            check(sortedIds, tag + "·G11 ALLOC 按订单 ID 升序");

            // 每张单恰好一次归属，且归属车真的送达了它（ALLOC ↔ STOP 双向对拍）。
            bool allocMatchesStops = true;
            for (const logistics::Order& o : cfg.orders) {
                int hits = 0;
                std::string vid;
                for (std::size_t i = 0; i < ad.orderIds.size(); ++i) {
                    if (ad.orderIds[i] == o.id) {
                        ++hits;
                        vid = ad.vehicleIds[i];
                    }
                }
                if (hits != 1) {
                    allocMatchesStops = false;
                }
                int delivered = 0;
                for (std::map<std::string, std::vector<std::string> >::const_iterator it =
                         ad.deliveredByVehicle.begin();
                     it != ad.deliveredByVehicle.end(); ++it) {
                    int here = 0;
                    for (std::size_t k = 0; k < it->second.size(); ++k) {
                        if (it->second[k] == o.id) {
                            ++here;
                        }
                    }
                    delivered += here;
                    if (it->first == vid && here != 1) {
                        allocMatchesStops = false;
                    }
                }
                if (delivered != 1) {
                    allocMatchesStops = false;
                }
            }
            check(allocMatchesStops, tag + "·G11 每单恰好一台车，且 ALLOC 与逐车 STOP 送达一致");

            // ASSIGN 的单数 == 该车 STOP 送达数；标签只能是 簇N / 跨簇 / 空车。
            bool assignOk = true;
            for (const logistics::Vehicle& v : cfg.vehicles) {
                int assigned = 0;
                bool hasLabel = false;
                for (std::size_t i = 0; i < ad.assignVehicle.size(); ++i) {
                    if (ad.assignVehicle[i] != v.id) {
                        continue;
                    }
                    hasLabel = true;
                    assigned += ad.assignCount[i];
                    const std::string& lab = ad.assignLabel[i];
                    const bool clusterLabel = lab.size() > 3 && lab.compare(0, 3, "簇") == 0
                                              && std::atoi(lab.c_str() + 3) >= 1;
                    if (!clusterLabel && lab != "跨簇" && lab != "空车") {
                        assignOk = false;
                    }
                }
                if (!hasLabel) {
                    assignOk = false;
                }
                int delivered = 0;
                const std::map<std::string, std::vector<std::string> >::const_iterator it =
                    ad.deliveredByVehicle.find(v.id);
                if (it != ad.deliveredByVehicle.end()) {
                    delivered = static_cast<int>(it->second.size());
                }
                if (assigned != delivered) {
                    assignOk = false;
                }
            }
            check(assignOk, tag + "·G11 ASSIGN 每车单数 == 该车 STOP 送达数，标签为 簇N/跨簇/空车");
            check(ad.assignVehicle.size() >= cfg.vehicles.size(),
                  tag + "·G11 每台车至少一条 ASSIGN（空车也打「空车 0」）");
            allocByStrategy[name] = ad.vehicleIds;
            parsedByStrategy[name] = p;
        }

        std::printf("    %-8s 距离 %.3f | 完工耗时 %.3f | 成本 %.3f | penalty %d | S %d A %d "
                    "| 平均在车 %.1f 峰值 %.1f | 车 %d 台\n",
                    name.c_str(), p.distance, p.time, p.cost, p.penalty, p.stops, p.actions,
                    p.avgLoad, p.peak, vehicleCount);
    }

    // ---- G12（#40 验收）：三策略两两分化 + 各自在**自身目标**上不劣于另两条 ----
    //
    // 口径与用户看到的一致：距离 / 成本是 Σ（--plan-summary 第 1 行），
    // 耗时是**完工耗时**（#19 Q5 主口径，same 第 1 行的「总耗时」）。
    if (strategies.size() == 3) {
        check(allocByStrategy["distance"] != allocByStrategy["cost"],
              "G12 距离与成本的 ALLOC 分配映射不同（三策略分化）");
        check(allocByStrategy["distance"] != allocByStrategy["time"],
              "G12 距离与时间的 ALLOC 分配映射不同");
        check(allocByStrategy["cost"] != allocByStrategy["time"],
              "G12 成本与时间的 ALLOC 分配映射不同");
        const double tol = 1e-6;
        check(parsedByStrategy["distance"].distance
                  <= parsedByStrategy["cost"].distance + tol
                  && parsedByStrategy["distance"].distance
                         <= parsedByStrategy["time"].distance + tol,
              "G12 距离策略的总距离不劣于成本/时间策略");
        check(parsedByStrategy["cost"].cost <= parsedByStrategy["distance"].cost + tol
                  && parsedByStrategy["cost"].cost <= parsedByStrategy["time"].cost + tol,
              "G12 成本策略的总成本不劣于距离/时间策略");
        check(parsedByStrategy["time"].time <= parsedByStrategy["distance"].time + tol
                  && parsedByStrategy["time"].time <= parsedByStrategy["cost"].time + tol,
              "G12 时间策略的完工耗时不劣于距离/成本策略");
        std::printf("    G12 三策略分配映射两两不同；自身目标各自最优（车端口径）\n");
    }


    // ---- G19（#42）：界面初始规划与 CLI 三阶段 ALLOC **逐单一致** ----
    //
    // 两条用户可见路径：
    //   · 界面：`--ui-probe` 打印的「[ui-probe] 分配 O01=V01,…」（MainWindow 的初始规划）；
    //   · CLI：`--dump-plan distance` 的 ALLOC 行（planFleet 的分配表）。
    // 守卫盯的是 #41 关闭时定位的真缺陷：GUI 曾自己按「订单 ID 升序 + i % 车数」顺序轮转
    // 分派、根本没走 #40 的三阶段分配；只有 CLI 走 planFleet ⇒ 界面与命令行是两套分配。
    {
        const auto pairsOf = [](const std::string& text) {
            std::vector<std::pair<std::string, std::string> > out;
            std::size_t pos = 0;
            while (pos <= text.size()) {
                const std::size_t comma = text.find(',', pos);
                const std::string chunk =
                    text.substr(pos, comma == std::string::npos ? std::string::npos : comma - pos);
                const std::size_t eq = chunk.find('=');
                if (eq != std::string::npos && eq > 0) {
                    out.push_back(std::make_pair(chunk.substr(0, eq), chunk.substr(eq + 1)));
                }
                if (comma == std::string::npos) {
                    break;
                }
                pos = comma + 1;
            }
            return out;
        };

        const std::string probe = runCapture(
            "QT_QPA_PLATFORM=offscreen '" + appPath + "' --ui-probe 2>/dev/null");
        std::string guiAlloc;
        for (const std::string& line : splitLines(probe)) {
            const std::string prefix = "[ui-probe] 分配 ";
            if (line.compare(0, prefix.size(), prefix) == 0) {
                guiAlloc = line.substr(prefix.size());
            }
        }
        check(!guiAlloc.empty(), "G19 --ui-probe 打印界面侧的分配快照（用户可见）");

        const std::string dumpDistance =
            runCapture("'" + appPath + "' --dump-plan distance 2>/dev/null");
        const AllocDump ad = parseAllocDump(dumpDistance);
        check(ad.hasAlloc, "G19 --dump-plan distance 含 ALLOC 行");

        // ALLOC 行 -> "O01=V01,…" 的同一形状（订单 ID 升序），逐单比对
        std::string cliAlloc;
        for (std::size_t i = 0; i < ad.orderIds.size(); ++i) {
            if (!cliAlloc.empty()) {
                cliAlloc += ",";
            }
            cliAlloc += ad.orderIds[i] + "=" + ad.vehicleIds[i];
        }
        const std::vector<std::pair<std::string, std::string> > guiPairs = pairsOf(guiAlloc);
        const std::vector<std::pair<std::string, std::string> > cliPairs = pairsOf(cliAlloc);
        bool same = guiPairs.size() == cliPairs.size() && !guiPairs.empty();
        std::string firstDiff;
        if (same) {
            for (std::size_t i = 0; i < guiPairs.size(); ++i) {
                if (guiPairs[i].first != cliPairs[i].first
                    || guiPairs[i].second != cliPairs[i].second) {
                    same = false;
                    if (firstDiff.empty()) {
                        firstDiff = guiPairs[i].first + "=" + guiPairs[i].second + " / CLI "
                                    + cliPairs[i].first + "=" + cliPairs[i].second;
                    }
                }
            }
        } else {
            firstDiff = "单数 " + std::to_string(guiPairs.size()) + " vs "
                        + std::to_string(cliPairs.size());
        }
        check(same, "G19 界面分配与 CLI 三阶段 ALLOC 逐单一致（" + std::to_string(guiPairs.size())
                        + " 单" + (firstDiff.empty() ? std::string() : "，差异 " + firstDiff) + "）");

        // 判别力：顺序轮转基线与三阶段分配**不同** —— 界面若退回轮转基线，这条必红。
        const logistics::FleetPlan seq = logistics::planFleetSequentialBaseline(
            cfg.graph, cfg.vehicles, cfg.orders, logistics::WeightType::Distance);
        std::size_t seqDiff = 0;
        for (std::size_t i = 0; i < seq.allocationOrderIds.size() && i < cliPairs.size(); ++i) {
            if (seq.allocationOrderIds[i] != cliPairs[i].first) {
                ++seqDiff;
                continue;
            }
            const std::size_t v = seq.allocationVehicleIndex[i];
            const std::string vid = (v < cfg.vehicles.size()) ? cfg.vehicles[v].id : "-";
            if (vid != cliPairs[i].second) {
                ++seqDiff;
            }
        }
        check(seqDiff > 0,
              "G19（判别力）顺序轮转基线与三阶段分配在 " + std::to_string(seqDiff)
                  + " 单上不同 —— 界面若退回轮转基线，上面的逐单比对必红");
        std::printf("    G19 界面分配 == CLI ALLOC（%d 单）；轮转基线与之相差 %d 单\n",
                    static_cast<int>(guiPairs.size()), static_cast<int>(seqDiff));
    }

    return testutil::summarize("cli_plan_summary");
}
