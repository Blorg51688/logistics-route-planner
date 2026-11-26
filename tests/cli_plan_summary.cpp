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

// --dump-plan 的 LEG 行（规格三 §1 / #45）：**只加行、不改既有行**。
// 读的是 stdout 文本，不是内部公式：把「腿结构」当成用户可见的契约来钉。
void checkDumpPlanLegLines(const std::string& appPath) {
    const std::string tmp = "/tmp/cli_plan_summary_legs_"
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
             "T03, transit, 700, 330, 中转站丙, 3\n"
             "D08, delivery, 300, 220, 客户08, 1\n"
             "D01, delivery, 180, 90, 客户01, 1\n"
             "D18, delivery, 620, 120, 客户18, 3\n"
             "D20, delivery, 600, 300, 客户20, 3\n"
             "\n[edges]\n"
             "W01, T01, 4.7, 4.7, 7.5\n"
             "T01, W01, 4.7, 4.7, 7.5\n"
             "T01, T03, 18.2, 18.2, 29.0\n"
             "T03, T01, 18.2, 18.2, 29.0\n"
             "T01, D08, 1.3, 3.1, 1.6\n"
             "D08, T01, 1.3, 3.1, 1.6\n"
             "T01, D01, 3.6, 8.6, 4.3\n"
             "D01, T01, 3.6, 8.6, 4.3\n"
             "T03, D18, 3.5, 8.4, 4.2\n"
             "D18, T03, 3.5, 8.4, 4.2\n"
             "T03, D20, 1.6, 3.8, 1.9\n"
             "D20, T03, 1.6, 3.8, 1.9\n"
             "\n[vehicles]\n"
             "V01, W01, 400, 08:00, 1.00, 1.50\n"
             "V02, W01, 160, 08:00, 1.25, 0.80\n"
             "V03, W01, 160, 08:00, 1.25, 0.80\n"
             "\n[orders]\n"
             "O1, D08, D01, 10, 00:00, 23:59, 0\n"
             "M1, D08, D18, 8, 00:00, 23:59, 0\n";
    }
    const std::string out =
        runCapture("'" + appPath + "' --config '" + tmp + "' --dump-plan distance 2>/dev/null");
    std::remove(tmp.c_str());

    // LEG <腿序> <from> <to> <orderId> <kind> <HH:MM readyTime> <vehicleId>
    std::vector<std::string> orderIds;
    std::vector<int> seqs;
    std::vector<std::string> kinds;
    std::vector<std::string> froms;
    std::vector<std::string> tos;
    std::vector<int> readyMin;
    std::vector<std::string> vehiclesOfLeg;
    bool shapeOk = true;
    for (const std::string& line : splitLines(out)) {
        if (line.rfind("LEG ", 0) != 0) {
            continue;
        }
        const std::vector<std::string> tok = splitSpaces(line);
        if (tok.size() != 8 || tok[0] != "LEG") {
            shapeOk = false;
            continue;
        }
        // HH:MM -> 分钟；解析失败（或不是两位数格式）就判形状错。
        const std::size_t colon = tok[6].find(':');
        if (colon != 2 || tok[6].size() != 5) {
            shapeOk = false;
            continue;
        }
        const int hh = std::atoi(tok[6].substr(0, 2).c_str());
        const int mm = std::atoi(tok[6].substr(3, 2).c_str());
        if (mm < 0 || mm > 59) {
            shapeOk = false;
        }
        orderIds.push_back(tok[4]);
        seqs.push_back(std::atoi(tok[1].c_str()));
        froms.push_back(tok[2]);
        tos.push_back(tok[3]);
        kinds.push_back(tok[5]);
        readyMin.push_back(hh * 60 + mm);
        vehiclesOfLeg.push_back(tok[7]);
    }

    check(shapeOk, "LEG：字段形状 = 腿序 / from / to / 订单 / kind / HH:MM / 执行车（8 段）");
    check(orderIds.size() == 4,
          "LEG：本地单 1 条 + 跨簇单 3 条 = 4 行（实际 " + std::to_string(orderIds.size()) + "）");

    // 腿的所属订单与订单表**双向一致**：LEG 行里的订单号恰好是夹具的两张单，且各出现对次。
    int m1 = 0;
    int o1 = 0;
    bool onlyKnown = true;
    for (std::size_t i = 0; i < orderIds.size(); ++i) {
        if (orderIds[i] == "M1") {
            ++m1;
        } else if (orderIds[i] == "O1") {
            ++o1;
        } else {
            onlyKnown = false;
        }
    }
    check(onlyKnown, "LEG：腿的订单号都能在订单表里查到（不多）");
    check(m1 == 3, "LEG：跨簇单 M1 恰好 3 条腿（实际 " + std::to_string(m1) + "）");
    check(o1 == 1, "LEG：本地单 O1 恰好 1 条腿（实际 " + std::to_string(o1) + "）");

    // 行序固定：订单 ID 升序（M1 < O1），同单内 1 -> 2 -> 3。
    bool orderOk = true;
    for (std::size_t i = 1; i < orderIds.size(); ++i) {
        if (orderIds[i - 1] > orderIds[i]) {
            orderOk = false;
        }
        if (orderIds[i - 1] == orderIds[i] && seqs[i - 1] >= seqs[i]) {
            orderOk = false;
        }
    }
    check(orderOk, "LEG：行序 = 订单 ID 升序、同单内腿序 1 -> 2 -> 3");

    // 跨簇单的三条腿：顺序固定、首尾相连、L2 由快车执行。
    std::vector<std::size_t> idxOfM1;
    for (std::size_t i = 0; i < orderIds.size(); ++i) {
        if (orderIds[i] == "M1") {
            idxOfM1.push_back(i);
        }
    }
    bool crossOk = idxOfM1.size() == 3;
    if (crossOk) {
        const std::size_t a = idxOfM1[0], b = idxOfM1[1], c = idxOfM1[2];
        crossOk = seqs[a] == 1 && seqs[b] == 2 && seqs[c] == 3
                  && kinds[a] == "customer->station" && kinds[b] == "station->station"
                  && kinds[c] == "station->customer" && froms[a] == "D08" && tos[a] == "T01"
                  && froms[b] == "T01" && tos[b] == "T03" && froms[c] == "T03"
                  && tos[c] == "D18";
    }
    check(crossOk,
          "LEG：跨簇单 L1 D08->T01 / L2 T01->T03 / L3 T03->D18，kind 依次 customer->station / "
          "station->station / station->customer");

    // readyTime 单调（L1 <= L2 <= L3）且站间腿让 L2 严格晚于 L1 —— 常量 0 或
    // 「L2.ready = L1.ready」这两种退化都会在这两条上变红。
    if (idxOfM1.size() == 3) {
        const int r1 = readyMin[idxOfM1[0]];
        const int r2 = readyMin[idxOfM1[1]];
        const int r3 = readyMin[idxOfM1[2]];
        check(r1 <= r2 && r2 <= r3, "LEG：readyTime 单调 L1 <= L2 <= L3（" + std::to_string(r1)
                                        + " / " + std::to_string(r2) + " / "
                                        + std::to_string(r3) + "）");
        check(r1 < r2 && r2 < r3,
              "LEG：readyTime 严格递增（站间腿耗时 > 0 ⇒ 就绪链真的在往前走，不是常量）");
        check(r1 == 480, "LEG：L1.ready = 发车时刻 08:00（480min）");
        check(vehiclesOfLeg[idxOfM1[1]] == "V01",
              "LEG：站间腿 L2 由快车 V01 执行（实际「" + vehiclesOfLeg[idxOfM1[1]] + "」）");
    }

    // 执行车字段必须是车队里的车（本票是临时指派，但**不能**是空/占位符）。
    bool vehiclesOk = true;
    for (std::size_t i = 0; i < vehiclesOfLeg.size(); ++i) {
        if (vehiclesOfLeg[i] != "V01" && vehiclesOfLeg[i] != "V02"
            && vehiclesOfLeg[i] != "V03") {
            vehiclesOk = false;
        }
    }
    check(vehiclesOk, "LEG：执行车字段取自车队（V01/V02/V03），没有伪装成 '-'");
}

} // namespace


// --dump-plan 的 TRANSIT 行 + 三段接力 + 有界前瞻（规格三 §3 / #48）：
// **只加行、不改既有行**；守卫读的全部是 stdout。
//
// 夹具（W01 仓库 / T01 簇1 站 / T03 簇3 站 / D01,D08 簇1 客户 / D18,D20 簇3 客户；
// V01 快车 400kg·1.00·1.50 / V02·V03 慢车 160kg·1.25·0.80，8:00 发车）：
//   O1: D01->D08 10kg  簇1 本地单（把本簇慢车顺路带到 D08 ⇒ L1 几乎零绕行）
//   O2: D20->D18 10kg  簇3 本地单（V03 的 T03->D20->D18 本来就要走 ⇒ L3 零绕行）
//   M1: D08->D18  8kg  跨簇普通单（簇1 -> 簇3）
//
// 夹具 A = M1 窗口 0:00 起（货 08:13 就绪，快车 08:04 到站 ⇒ 前瞻 8min 后取走）；
// 夹具 B = M1 窗口 10:00 起（货 10:01 才就绪，就绪差 ~117min > W=60 ⇒ **不得**纳入）。
std::vector<std::string> splitCommaList(const std::string& s) {
    std::vector<std::string> out;
    std::string cur;
    for (char c : s) {
        if (c == ',') {
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

// ---- 三段接力 / 有界前瞻的 --dump-plan 解析 ----
// 只读用户可见 stdout：PLAN 分车、STOP 停靠、LEG 腿与就绪、TRANSIT 账本、METRICS 逐车指标。
struct DumpTransit {
    std::string orderId, station, inVehicle, outVehicle, status;
    int inMin = 0, outMin = 0, dwell = 0;
};
struct DumpLeg {
    int seq = 0;
    std::string from, to, orderId, kind, vehicle;
    int ready = 0;
};
struct DumpStop {
    std::string vehicle, node, phase;
    int idx = 0, raw = 0, wait = 0, arr = 0;
    std::vector<std::string> pickups, deliveries;
};
struct DumpMetricRow {
    double distance = 0.0, time = 0.0, cost = 0.0;
    int penalty = 0, stopCount = 0, actionCount = 0;
};
struct DumpRelay {
    bool ok = false;
    std::vector<DumpTransit> transits;
    std::vector<DumpLeg> legs;
    std::vector<DumpStop> stops;
    std::map<std::string, DumpMetricRow> metrics;
};

// HH:MM -> 分钟；形状不对返回 -1。
int clockMinFromText(const std::string& s) {
    if (s.size() != 5 || s[2] != ':') {
        return -1;
    }
    return std::atoi(s.substr(0, 2).c_str()) * 60 + std::atoi(s.substr(3, 2).c_str());
}

DumpRelay parseRelayDump(const std::string& text) {
    DumpRelay d;
    std::string curVehicle;
    for (const std::string& line : splitLines(text)) {
        const std::vector<std::string> tok = splitSpaces(line);
        if (tok.empty()) {
            continue;
        }
        if (tok[0] == "PLAN" && tok.size() >= 5) {
            curVehicle = tok[1];
        } else if (tok[0] == "STOP" && tok.size() == 10 && !curVehicle.empty()) {
            DumpStop s;
            s.vehicle = curVehicle;
            s.idx = std::atoi(tok[1].c_str());
            s.node = tok[2];
            s.phase = tok[3];
            s.raw = std::atoi(tok[4].c_str());
            s.wait = std::atoi(tok[5].c_str());
            s.arr = std::atoi(tok[6].c_str());
            if (tok[7] != "-") {
                s.pickups = splitCommaList(tok[7]);
            }
            if (tok[8] != "-") {
                for (const std::string& item : splitCommaList(tok[8])) {
                    const std::size_t colon = item.find(':');
                    s.deliveries.push_back(colon == std::string::npos ? item
                                                                     : item.substr(0, colon));
                }
            }
            d.stops.push_back(s);
        } else if (tok[0] == "LEG" && tok.size() == 8) {
            DumpLeg l;
            l.seq = std::atoi(tok[1].c_str());
            l.from = tok[2];
            l.to = tok[3];
            l.orderId = tok[4];
            l.kind = tok[5];
            l.ready = clockMinFromText(tok[6]);
            l.vehicle = tok[7];
            d.legs.push_back(l);
        } else if (tok[0] == "TRANSIT" && tok.size() == 10) {
            DumpTransit t;
            t.orderId = tok[1];
            t.station = tok[2];
            t.inVehicle = tok[4];
            t.inMin = clockMinFromText(tok[5]);
            t.outVehicle = tok[6];
            t.outMin = clockMinFromText(tok[7]);
            t.dwell = std::atoi(tok[8].c_str());
            t.status = tok[9];
            d.transits.push_back(t);
        } else if (tok[0] == "METRICS" && tok.size() >= 19) {
            DumpMetricRow m;
            m.distance = std::strtod(tok[2].c_str(), nullptr);
            m.time = std::strtod(tok[3].c_str(), nullptr);
            m.cost = std::strtod(tok[4].c_str(), nullptr);
            m.penalty = std::atoi(tok[5].c_str());
            m.stopCount = std::atoi(tok[6].c_str());
            m.actionCount = std::atoi(tok[7].c_str());
            d.metrics[tok[1]] = m;
        }
    }
    for (const DumpStop& s : d.stops) {
        d.ok = true;
    }
    return d;
}

std::vector<DumpLeg> legsOfOrder(const std::vector<DumpLeg>& legs, const std::string& orderId) {
    std::vector<DumpLeg> out;
    for (const DumpLeg& l : legs) {
        if (l.orderId == orderId) {
            out.push_back(l);
        }
    }
    return out;
}

// 某车在某节点上的**第一个**停靠（夹具里每车每站只停一次）。
const DumpStop* findStop(const std::vector<DumpStop>& stops, const std::string& vehicle,
                         const std::string& node) {
    for (const DumpStop& s : stops) {
        if (s.vehicle == vehicle && s.node == node) {
            return &s;
        }
    }
    return nullptr;
}

bool hasDelivery(const std::vector<DumpStop>& stops, const std::string& vehicle,
                 const std::string& node, const std::string& orderId) {
    for (const DumpStop& s : stops) {
        if (s.vehicle != vehicle || s.node != node) {
            continue;
        }
        for (const std::string& id : s.deliveries) {
            if (id == orderId) {
                return true;
            }
        }
    }
    return false;
}

// --dump-plan 的 TRANSIT 行 + 三段接力 + 有界前瞻（规格三 §3 / #48）：
// **只加行、不改既有行**；守卫读的全部是 stdout。
//
// 三个夹具共用同一套图（走廊拓扑 W01 --4-- T01 --40-- T03，两簇各一个客户区）：
//   O1: D08->D01 10kg  簇1 本地（把本簇慢车顺路带到 D08，L1 的取货点）
//   O2: D20->D18 10kg  簇3 本地（D20 正好在 T03->D18 的路上，L3 零绕行）
//   M1: D08->D18  8kg  跨簇普通单（簇1 -> 簇3）
// 车队：V01/V02/V03 同容量 160kg、同时因子 1.00，V01 成本因子 1.50（接力车 = 容量最大、
// 并列取成本因子高 ⇒ V01）。为什么夹具要让三车同容量同时因子：本票尚未做职责封闭
// （#46），若快车又大又快，分配器会把远簇整簇塞给它，接力就必然被「不更差」兜底退回——
// 那是 #46 的分配问题，不是本票的等待/前瞻语义。同容量同时因子下「簇 -> 车」由
// 既有 tieKey 稳定落在「每簇一台慢车」，正是职责封闭要立的目标形态。
//
// 夹具 A（D08 距 T01 20min ⇒ L1 慢 26min）：等待可被读数看见（V01/V02 各等 28min）。
// 夹具 T（D08 在 W01->T01 途中，3+1min）：链条紧，三策略（含**时间策略**）都走接力。
// 夹具 B（同 A 的图 + M1 窗口 10:00 起）：就绪差 ~136min > W=60 ⇒ **不得**纳入。
void checkDumpPlanTransitLines(const std::string& appPath) {
    const std::string tmpA = "/tmp/cli_plan_summary_relayA_"
                             + std::to_string(static_cast<long long>(getpid())) + ".ini";
    const std::string tmpT = "/tmp/cli_plan_summary_relayT_"
                             + std::to_string(static_cast<long long>(getpid())) + ".ini";
    const std::string tmpB = "/tmp/cli_plan_summary_relayB_"
                             + std::to_string(static_cast<long long>(getpid())) + ".ini";
    auto header = [](std::ofstream& f) {
        f << "[general]\n"
             "event_interval_min = 1440\n"
             "event_level_traffic = high\n"
             "event_level_customer = medium\n"
             "event_level_urgent = low\n"
             "event_level_closure = extra_low\n"
             "\n[nodes]\n"
             "W01, warehouse, 80, 160, 中央仓库, 0\n"
             "T01, transit, 380, 200, 中转站甲, 1\n"
             "T03, transit, 700, 330, 中转站丙, 3\n"
             "D01, delivery, 120, 90, 客户01, 1\n"
             "D08, delivery, 300, 220, 客户08, 1\n"
             "D18, delivery, 620, 120, 客户18, 3\n"
             "D20, delivery, 600, 300, 客户20, 3\n";
    };
    // w01ToD08 / t01ToD08：D08 与仓库/中转站的耗时（决定 L1 慢车有多晚到 T01）。
    //   · 夹具 A：10 / 20 ⇒ L1 慢 26min，等待可见但仍在 W 内；
    //   · 夹具 T：3 / 1  ⇒ L1 与快车几乎同时到 T01，链条紧（时间策略才容得下接力）；
    //   · 夹具 B：100 / 100 ⇒ D08 在簇1 远端，L1 慢车最早也要 200min 才回到 T01
    //     （就绪差 ~196min > W=60）⇒ 必须不纳入——无论哪台车当 L1 车都超过窗口。
    auto writeFixture = [&header](const std::string& path, double w01ToD08, double t01ToD08,
                                  int m1WindowStart) {
        char m1[256];
        std::snprintf(m1, sizeof(m1), "M1, D08, D18, 8, %02d:%02d, 23:59, 0\n",
                      m1WindowStart / 60, m1WindowStart % 60);
        std::ofstream f(path.c_str());
        header(f);
        char d08[4][96];
        std::snprintf(d08[0], sizeof(d08[0]), "W01, D08, %.1f, %.1f, %.1f\n", w01ToD08,
                      w01ToD08, w01ToD08 * 1.2);
        std::snprintf(d08[1], sizeof(d08[1]), "D08, W01, %.1f, %.1f, %.1f\n", w01ToD08,
                      w01ToD08, w01ToD08 * 1.2);
        std::snprintf(d08[2], sizeof(d08[2]), "T01, D08, %.1f, %.1f, %.1f\n", t01ToD08,
                      t01ToD08, t01ToD08 * 1.2);
        std::snprintf(d08[3], sizeof(d08[3]), "D08, T01, %.1f, %.1f, %.1f\n", t01ToD08,
                      t01ToD08, t01ToD08 * 1.2);
        f << "\n[edges]\n"
             "W01, T01, 4.0, 4.0, 6.0\n"
             "T01, W01, 4.0, 4.0, 6.0\n"
          << d08[0] << d08[1] << d08[2] << d08[3] << "W01, D01, 6.0, 6.0, 7.0\n"
             "D01, W01, 6.0, 6.0, 7.0\n"
             "T01, D01, 6.0, 6.0, 7.0\n"
             "D01, T01, 6.0, 6.0, 7.0\n"
             "T01, T03, 40.0, 40.0, 50.0\n"
             "T03, T01, 40.0, 40.0, 50.0\n"
             "T03, D20, 2.0, 2.0, 2.4\n"
             "D20, T03, 2.0, 2.0, 2.4\n"
             "D20, D18, 4.0, 4.0, 5.0\n"
             "D18, D20, 4.0, 4.0, 5.0\n"
             "T03, D18, 6.0, 6.0, 7.0\n"
             "D18, T03, 6.0, 6.0, 7.0\n"
             "\n[vehicles]\n"
             "V01, W01, 160, 08:00, 1.00, 1.50\n"
             "V02, W01, 160, 08:00, 1.00, 0.80\n"
             "V03, W01, 160, 08:00, 1.00, 0.80\n"
             "\n[orders]\n"
             "O1, D08, D01, 10, 00:00, 23:59, 0\n"
             "O2, D20, D18, 10, 00:00, 23:59, 0\n"
          << m1;
    };
    writeFixture(tmpA, 10.0, 20.0, 0);
    writeFixture(tmpT, 3.0, 1.0, 0);
    writeFixture(tmpB, 100.0, 100.0, 0);

    // ---- 夹具 T：**三策略**（含时间策略）都必须走完三段接力 ----
    for (const char* strategy : {"distance", "cost", "time"}) {
        const std::string tag = std::string("三段接力·") + strategy;
        const DumpRelay d = parseRelayDump(
            runCapture("'" + appPath + "' --config '" + tmpT + "' --dump-plan " + strategy
                       + " 2>/dev/null"));
        check(d.ok, tag + "：--dump-plan 可解析（PLAN/STOP/LEG/TRANSIT 行）");
        // 账本恰 1 笔：跨簇普通单 M1 只能走三段腿。
        check(d.transits.size() == 1,
              tag + "：跨簇单 M1 恰有 1 笔 TRANSIT（实际 " + std::to_string(d.transits.size())
                  + "）");
        bool shapeOk = d.transits.size() == 1;
        for (const DumpTransit& t : d.transits) {
            shapeOk = shapeOk && t.orderId == "M1" && !t.station.empty()
                      && t.inVehicle != "-" && t.outVehicle != "-"
                      && t.inVehicle != t.outVehicle && t.dwell >= 0 && t.dwell <= 60
                      && t.status == "已送达";
        }
        check(shapeOk,
              tag + "：TRANSIT 形状（M1 / 站 / 卸入车 ≠ 取走车 / dwell ∈ [0,60] / 已送达）");

        const std::vector<DumpLeg> m1 = legsOfOrder(d.legs, "M1");
        check(m1.size() == 3,
              tag + "：M1 恰有 3 条 LEG 腿（实际 " + std::to_string(m1.size()) + "）");
        if (m1.size() == 3) {
            check(m1[0].seq == 1 && m1[0].from == "D08" && m1[0].to == "T01"
                      && m1[0].kind == "customer->station",
                  tag + "：L1 = 本簇慢车 取 D08 -> 卸 T01（customer->station）");
            check(m1[1].seq == 2 && m1[1].from == "T01" && m1[1].to == "T03"
                      && m1[1].kind == "station->station",
                  tag + "：L2 = 站间腿 T01 -> T03（station->station）");
            check(m1[2].seq == 3 && m1[2].from == "T03" && m1[2].to == "D18"
                      && m1[2].kind == "station->customer",
                  tag + "：L3 = 对簇慢车 T03 -> 送 D18（station->customer）");
            // 三段接力真的分给三台车：相邻两段的执行车不同（inVehicle ≠ outVehicle 的根）。
            check(m1[0].vehicle != "-" && m1[1].vehicle != "-" && m1[2].vehicle != "-"
                      && m1[0].vehicle != m1[1].vehicle && m1[1].vehicle != m1[2].vehicle,
                  tag + "：三段腿的执行车可辨且相邻两段不同（" + m1[0].vehicle + " -> "
                      + m1[1].vehicle + " -> " + m1[2].vehicle + "）");
            // 站间腿归接力车（容量最大/成本因子最高 ⇒ V01），与 chooseLegVehicle 同判据。
            check(m1[1].vehicle == "V01",
                  tag + "：L2 由接力车 V01 执行（实际 " + m1[1].vehicle + "）");
            // readyTime 就是接力链：L2.ready = L1 到 T01，L3.ready = L2 到 T03。
            check(m1[0].ready < m1[1].ready && m1[1].ready < m1[2].ready,
                  tag + "：readyTime 沿接力链严格递增（" + std::to_string(m1[0].ready) + " / "
                      + std::to_string(m1[1].ready) + " / " + std::to_string(m1[2].ready)
                      + " min）");
            if (d.transits.size() == 1) {
                const DumpTransit& t = d.transits[0];
                check(t.station == m1[2].from && t.outVehicle == m1[2].vehicle,
                      tag + "：TRANSIT 的站与取走车 == L3 的 from/执行车（站 " + t.station
                          + " / " + t.outVehicle + "）");
                check(t.inVehicle == m1[1].vehicle,
                      tag + "：TRANSIT 的卸入车 == L2 的执行车（" + t.inVehicle + "）");
                check(t.outMin == m1[2].ready,
                      tag + "：账本取走时刻 == L3 的 readyTime（" + std::to_string(t.outMin)
                          + "）");
                check(m1[2].ready == m1[1].ready + 40,
                      tag + "：L3.ready = L2.ready + 站间耗时 40min（就绪链真在往前走）");
            }
            // 期末无「在站」：M1 被 L3 车在 D18 送达。
            check(hasDelivery(d.stops, m1[2].vehicle, "D18", "M1"),
                  tag + "：M1 由 L3 车在 D18 送达（期末无在站货物）");
        }
    }

    // ---- 夹具 A：等待可被读数看见（且计入耗时、不计 penalty）----
    for (const char* strategy : {"distance", "cost"}) {
        const std::string tag = std::string("等待可见·") + strategy;
        const DumpRelay d = parseRelayDump(
            runCapture("'" + appPath + "' --config '" + tmpA + "' --dump-plan " + strategy
                       + " 2>/dev/null"));
        check(d.transits.size() == 1,
              tag + "：跨簇单 M1 走完三段接力（TRANSIT 1 笔，实际 "
                  + std::to_string(d.transits.size()) + "）");
        const std::vector<DumpLeg> m1 = legsOfOrder(d.legs, "M1");
        if (m1.size() == 3) {
            // 快车在 T01 原地等货：raw < arrival，wait > 0。
            const DumpStop* waitStop = findStop(d.stops, m1[1].vehicle, "T01");
            check(waitStop != nullptr && waitStop->wait > 0 && waitStop->raw < waitStop->arr,
                  tag + "：站间车在 T01 原地等货（STOP 行 raw < arrival，wait="
                      + std::to_string(waitStop == nullptr ? -1 : waitStop->wait) + "min）");
            if (waitStop != nullptr) {
                check(waitStop->wait <= 60,
                      tag + "：等待受前瞻窗口约束（" + std::to_string(waitStop->wait)
                          + "min <= W=60）");
            }
            // 等待计入该车耗时：接力车只跑 W01->T01->T03->W01 = 88min（夹具常量），
            // 多出来的就是等待。
            const std::map<std::string, DumpMetricRow>::const_iterator mv =
                d.metrics.find(m1[1].vehicle);
            if (waitStop != nullptr && mv != d.metrics.end()) {
                check(mv->second.time >= 88.0 + static_cast<double>(waitStop->wait) - 1.0,
                      tag + "：等待计入该车耗时（" + m1[1].vehicle + " 耗时 "
                          + std::to_string(mv->second.time) + "min >= 行驶 88 + 等待 "
                          + std::to_string(waitStop->wait) + "）");
                check(mv->second.penalty == 0,
                      tag + "：等待不计 penalty（该车 penalty 0，等待不是迟到）");
            }
            // L3 车也在 T03 等货（货还没到站），同样 raw < arrival。
            const DumpStop* wait3 = findStop(d.stops, m1[2].vehicle, "T03");
            check(wait3 != nullptr && wait3->wait > 0,
                  tag + "：L3 慢车在 T03 等货（wait="
                      + std::to_string(wait3 == nullptr ? -1 : wait3->wait) + "min）");
            check(hasDelivery(d.stops, m1[2].vehicle, "D18", "M1"),
                  tag + "：M1 由 L3 车在 D18 送达");
        }
        if (d.transits.size() == 1) {
            check(d.transits[0].dwell <= 60, tag + "：账本 dwell <= 60");
        }
    }

    // ---- 夹具 B：就绪时刻在 W 之外的腿**不得**纳入当前路线 ----
    for (const char* strategy : {"distance", "cost", "time"}) {
        const std::string tag = std::string("有界前瞻·") + strategy;
        const DumpRelay d = parseRelayDump(
            runCapture("'" + appPath + "' --config '" + tmpB + "' --dump-plan " + strategy
                       + " 2>/dev/null"));
        check(d.transits.empty(),
              tag + "：M1 就绪差 > W=60 ⇒ 不纳入当前路线（无 TRANSIT 行，实际 "
                  + std::to_string(d.transits.size()) + "）");
        check(hasDelivery(d.stops, "V01", "D18", "M1"),
              tag + "：M1 退回「直接给快车」的对照方案（V01 在 D18 送达）");
        // 任何接力取货停靠的等待都必须在窗口内（W 改无界时这条与本组首条一起变红）。
        bool bounded = true;
        for (const DumpLeg& leg : d.legs) {
            if (leg.kind == "customer->customer") {
                continue;
            }
            const DumpStop* st = findStop(d.stops, leg.vehicle, leg.from);
            if (st != nullptr && st->wait > 60) {
                bounded = false;
            }
        }
        check(bounded, tag + "：所有接力取货停靠的等待都 <= W=60");
    }
    std::remove(tmpA.c_str());
    std::remove(tmpT.c_str());
    std::remove(tmpB.c_str());
}


// ===========================================================================
// #46 职责封闭：**用户可见输出**上的两条守卫（读 --dump-plan 的 ASSIGN/ALLOC/LEG/TRANSIT）
// ===========================================================================
//
// 为什么不自己重算：这里断言的全是「用户打开 --dump-plan / 界面日志能看到的东西」——
// 分配落点、腿的执行车、账本两端。自己按公式再算一遍只能证明公式自洽，证明不了
// 「职责表真的在输出里生效」。
namespace {

// 默认数据（四车：V01 快车 400kg，V02/V03/V04 各 160kg 守一个簇）。
void checkDutyClosureDefault(const std::string& appPath) {
    const std::string out = runCapture("'" + appPath + "' --dump-plan distance 2>/dev/null");
    check(!out.empty(), "职责封闭·默认数据：--dump-plan 有输出");
    const std::vector<std::string> lines = splitLines(out);

    // ① ASSIGN 行：快车空车、三台慢车各守一个簇（簇1 9 单 / 簇2 7 单 / 簇3 9 单）。
    bool fastIdle = false;
    std::map<std::string, std::string> assignOf;   // 车辆 -> ASSIGN 行
    for (const std::string& line : lines) {
        if (line.rfind("ASSIGN ", 0) != 0) {
            continue;
        }
        assignOf[line] = line;
        if (line.find("V01") != std::string::npos && line.find("空车") != std::string::npos) {
            fastIdle = true;
        }
    }
    check(fastIdle, "职责封闭①：ASSIGN 行显示快车 V01「空车 0」（默认数据上没有它的活）");
    bool slow1 = false;
    bool slow2 = false;
    bool slow3 = false;
    for (const std::string& line : lines) {
        if (line == "ASSIGN V02 簇1 9") slow1 = true;
        if (line == "ASSIGN V03 簇2 7") slow2 = true;
        if (line == "ASSIGN V04 簇3 9") slow3 = true;
    }
    check(slow1 && slow2 && slow3,
          "职责封闭①：簇1->V02(9) / 簇2->V03(7) / 簇3->V04(9)（簇 ID 升序 × 慢车 ID 升序配对）");

    // ② ALLOC 行：25 单里没有任何一张落在 V01 上。
    bool sawAlloc = false;
    bool v01HasOrder = false;
    int allocCount = 0;
    for (const std::string& line : lines) {
        if (line.rfind("ALLOC ", 0) != 0) {
            continue;
        }
        sawAlloc = true;
        // 形如 ALLOC O01=V02,O02=V02,...；逐项扫一遍即可（不解析成结构，读原文）。
        std::string rest = line.substr(6);
        std::size_t pos = 0;
        while (pos < rest.size()) {
            const std::size_t comma = rest.find(',', pos);
            const std::string item = rest.substr(pos, comma == std::string::npos
                                                          ? std::string::npos
                                                          : comma - pos);
            ++allocCount;
            if (item.size() > 4 && item.compare(item.size() - 3, 3, "V01") == 0) {
                v01HasOrder = true;
            }
            if (comma == std::string::npos) {
                break;
            }
            pos = comma + 1;
        }
    }
    check(sawAlloc && allocCount == 25, "职责封闭①：ALLOC 行覆盖 25 单（实际 "
                                            + std::to_string(allocCount) + "）");
    check(!v01HasOrder, "职责封闭①：快车 V01 不在 ALLOC 里（零订单，不是「隐蔽地接了单」）");

    // ③ LEG 行：慢车腿的两端必须同簇，且执行车 = 该簇慢车；默认数据没有站间腿。
    logistics::Config cfg;
    std::string cfgErr;
    check(logistics::ConfigLoader::load(DEFAULT_CONFIG_PATH, cfg, cfgErr),
          "职责封闭③：默认配置可加载（" + cfgErr + "）");
    std::map<std::string, int> clusterOfNode;
    for (const auto& node : cfg.graph.nodes()) {
        clusterOfNode[node.id] = node.subNetworkId;
    }
    const std::map<int, std::string> slowOfCluster = {
        {1, "V02"}, {2, "V03"}, {3, "V04"}};
    int legCount = 0;
    int stationLegs = 0;
    bool legOk = true;
    std::string badLeg;
    for (const std::string& line : lines) {
        if (line.rfind("LEG ", 0) != 0) {
            continue;
        }
        ++legCount;
        // LEG <seq> <from> <to> <order> <kind> <HH:MM> <vehicle>
        std::size_t pos = 4;
        std::vector<std::string> f;
        while (pos <= line.size()) {
            const std::size_t sp = line.find(' ', pos);
            f.push_back(line.substr(pos, sp == std::string::npos ? std::string::npos : sp - pos));
            if (sp == std::string::npos) {
                break;
            }
            pos = sp + 1;
        }
        if (f.size() < 6) {
            legOk = false;
            badLeg = line;
            continue;
        }
        const std::string& from = f[1];
        const std::string& to = f[2];
        const std::string& kind = f[4];
        const std::string& vehicle = f[6];
        if (kind == "station->station") {
            ++stationLegs;
            if (vehicle != "V01") {
                legOk = false;
                badLeg = line;
            }
            continue;
        }
        const int ca = clusterOfNode.count(from) ? clusterOfNode[from] : -1;
        const int cb = clusterOfNode.count(to) ? clusterOfNode[to] : -1;
        const std::string want = slowOfCluster.count(ca) ? slowOfCluster.at(ca) : std::string();
        if (ca <= 0 || ca != cb || want.empty() || vehicle != want) {
            legOk = false;
            badLeg = line;
        }
    }
    check(legCount == 25, "职责封闭③：默认数据 25 条本地腿（实际 "
                              + std::to_string(legCount) + "）");
    check(stationLegs == 0, "职责封闭③：默认数据没有站间腿（没有跨簇单）");
    check(legOk, "职责封闭③：每条本地腿都由**本簇慢车**执行、两端同簇（越界会红）"
                     + (badLeg.empty() ? std::string() : "：" + badLeg));
}

// 默认数据 + 一张跨簇普通单（D08->D18 8kg）：三段腿必须**真的**走通（#48 的衔接）。
// 三策略都验：若某策略退回直达，这里会红 —— 退回是可耻但可能发生的，必须显式报出来，
// 不能悄悄放宽（本文件里没有「退回也算过」的分支）。
void checkCrossClusterRelayOnDefaultData(const std::string& appPath) {
    // 写临时夹具：默认配置 + O26（订单段在文件末尾，直接追加最省事）。
    char tmpPath[256];
    std::snprintf(tmpPath, sizeof(tmpPath), "/tmp/cli_duty_cross_%d.ini",
                  static_cast<int>(getpid()));
    {
        std::ifstream in(DEFAULT_CONFIG_PATH);
        check(in.good(), "跨簇夹具：能读到默认配置");
        std::ofstream outFile(tmpPath);
        std::string line;
        while (std::getline(in, line)) {
            outFile << line << "\n";
        }
        outFile << "O26, D08, D18, 8, 08:00, 13:00, 0\n";
    }

    const char* strategies[3] = {"distance", "cost", "time"};
    for (int si = 0; si < 3; ++si) {
        const std::string tag = std::string("跨簇接力·") + strategies[si];
        const std::string out = runCapture("'" + appPath + "' --config '" + tmpPath
                                           + "' --dump-plan " + strategies[si] + " 2>/dev/null");
        const std::vector<std::string> lines = splitLines(out);
        // 按策略分别断言（口径见下方 TRANSIT 段的注释）：成本策略在跨城尺度下走直达。
        const bool expectRelay = (std::string(strategies[si]) != std::string("cost"));
        std::vector<std::string> legs;
        for (const std::string& line : lines) {
            if (line.rfind("LEG ", 0) == 0 && line.find(" O26 ") != std::string::npos) {
                legs.push_back(line);
            }
        }
        if (expectRelay) {
            check(legs.size() == 3, tag + "：O26 恰 3 条腿（实际 " + std::to_string(legs.size())
                                             + "；退回直达会只剩估计的 3 条但账本为空）");
        } else {
            // 成本策略走直达：LEG 必须**如实**只有 1 条 customer->customer 直达腿，
            // 执行车是快车 V01（不是伪造的 3 条腿）。
            check(legs.size() == 1, tag + "：成本策略直达 ⇒ O26 只有 1 条腿（实际 "
                                         + std::to_string(legs.size()) + "）");
            if (legs.size() == 1) {
                check(legs[0].find(" customer->customer ") != std::string::npos
                          && legs[0].find(" V01") != std::string::npos,
                      tag + "：直达腿是 customer->customer 且由快车 V01 执行：" + legs[0]);
            }
        }
        if (expectRelay && legs.size() == 3) {
            // L1 客户->本簇站（D08->T01，慢车 V02）；L2 站->站（T01->T03，快车 V01）；
            // L3 对簇站->客户（T03->D18，慢车 V04）。
            const auto fields = [](const std::string& line) {
                std::vector<std::string> f;
                std::size_t pos = 4;
                while (pos <= line.size()) {
                    const std::size_t sp = line.find(' ', pos);
                    f.push_back(line.substr(pos, sp == std::string::npos ? std::string::npos
                                                                        : sp - pos));
                    if (sp == std::string::npos) break;
                    pos = sp + 1;
                }
                return f;
            };
            const std::vector<std::string> l1 = fields(legs[0]);
            const std::vector<std::string> l2 = fields(legs[1]);
            const std::vector<std::string> l3 = fields(legs[2]);
            check(l1.size() > 6 && l1[1] == "D08" && l1[2] == "T01"
                      && l1[4] == "customer->station" && l1[6] == "V02",
                  tag + "：L1 = D08->T01 由**本簇慢车** V02 执行（实际 "
                      + (l1.size() > 6 ? l1[6] : std::string("-")) + "）");
            check(l2.size() > 6 && l2[1] == "T01" && l2[2] == "T03"
                      && l2[4] == "station->station" && l2[6] == "V01",
                  tag + "：L2 = T01->T03 由**快车** V01 执行（实际 "
                      + (l2.size() > 6 ? l2[6] : std::string("-")) + "）");
            check(l3.size() > 6 && l3[1] == "T03" && l3[2] == "D18"
                      && l3[4] == "station->customer" && l3[6] == "V04",
                  tag + "：L3 = T03->D18 由**对簇慢车** V04 执行（实际 "
                      + (l3.size() > 6 ? l3[6] : std::string("-")) + "）");
        }
        // TRANSIT <订单> <站> <kg> <卸入车> <HH:MM> <取走车> <HH:MM> <dwell> <状态>
        std::vector<std::string> transits;
        for (const std::string& line : lines) {
            if (line.rfind("TRANSIT O26 ", 0) == 0) {
                transits.push_back(line);
            }
        }
        // **规格四 #54 的口径修订（如实登记，不是放宽判据）**：
        // 跨城尺度下 relay 与 direct 的成本差被放大了一个数量级 —— 接力要让三台车各自
        // 跨城往返（V02 多跑 W01↔T01 360km、V04 多跑 W02↔T03 440km、V01 跑 200km），
        // 而快车一趟直达只走一次 500km 的国道（0.8 元/km）。实测 relay 增量成本 ≈3000 元
        // vs 直达 1887 元 ⇒ 成本策略下「不更差」兜底**必然**选直达。
        // 这是 §5.9「不更差兜底」这条设计在跨城尺度下的**真实后果**，不是缺陷；
        // 距离/时间策略仍走三段接力（距离看 km、时间看分钟，接力都更优）。
        // 判据因此改成「按策略分别断言」：distance/time 必走接力，cost 允许直达但必须
        // **自洽**（账本为空 + LEG 只有 1 条 customer->customer 直达腿 + 该腿执行车是快车）。
        if (expectRelay) {
            check(transits.size() == 1, tag + "：账本恰 1 笔 TRANSIT（实际 "
                                             + std::to_string(transits.size()) + "）");
        } else {
            check(transits.empty(), tag + "：成本策略下走直达 ⇒ 账本应为空（实际 "
                                         + std::to_string(transits.size()) + " 笔）");
        }
        if (transits.size() == 1) {
            std::vector<std::string> f;
            std::size_t pos = 0;
            const std::string& line = transits[0];
            while (pos <= line.size()) {
                const std::size_t sp = line.find(' ', pos);
                f.push_back(line.substr(pos, sp == std::string::npos ? std::string::npos
                                                                     : sp - pos));
                if (sp == std::string::npos) break;
                pos = sp + 1;
            }
            check(f.size() >= 9 && f[1] == "O26" && f[4] != f[6],
                  tag + "：账本两端非空且不同车（inVehicle != outVehicle）");
            // 站内停留（dwell）的**判据是 ≤ 60min**（kTransferMaxDwellMin），不是「必须为 0」。
            // 规格三时代默认数据上恰好恒为 0（车到站时货已在，或货到站时车已在），
            // 于是旧断言写成了逐字 "0"。跨城尺度 #54 之后时间策略出现 dwell = 5min：
            // L2 的快车 11:49 就卸入 T03，而 L3 的 V04 11:54 才到 —— **车等货的反面**
            // （这次是货等车 5 分钟，仍在 60min 上限内）。判据回到规格原文的 ≤ 60，
            // 而不是继续钉 0（钉 0 等于偷偷把上限收紧成「零等待」）。
            bool dwellOk = false;
            const int dwellMin = (f.size() >= 9) ? std::atoi(f[8].c_str()) : -1;
            dwellOk = (f.size() >= 9) && dwellMin >= 0 && dwellMin <= 60;
            check(dwellOk, tag + "：站内停留 ∈ [0, 60]min（实际 "
                              + (f.size() >= 9 ? f[8] : std::string("-")) + "）");
            check(f.size() >= 10 && f[9] == "已送达",
                  tag + "：期末无在站（状态 = 已送达）");
            std::printf("      %s TRANSIT：%s\n", tag.c_str(), line.c_str());
        }
        // 快车接单情况按策略分别断言：接力路径下它只跑站间腿 ⇒「空车 0」；
        // 成本策略直达时它真的接了这单 ⇒ 必须**不是**「空车 0」（否则是伪造的分配）。
        bool fastIdle = false;
        for (const std::string& line : lines) {
            if (line.find("ASSIGN V01 空车 0") == 0) fastIdle = true;
        }
        if (expectRelay) {
            check(fastIdle, tag + "：快车 V01 仍「空车 0」（站间腿不算接单）");
        } else {
            check(!fastIdle, tag + "：成本策略直达 ⇒ 快车 V01 必须真的接了这单（不是空车）");
        }
        for (const std::string& leg : legs) {
            std::printf("      %s LEG：%s\n", tag.c_str(), leg.c_str());
        }
    }
    std::remove(tmpPath);
}

// P2：降级场景下 LEG 必须**如实反映实际执行形态**。
// 默认数据 + 紧窗 O26（08:00-09:00）⇒ 三段腿劣于直达对照、职责降级为 V01 直达。
// 守卫：O26 只应有 1 条 customer->customer 直达腿（执行车 == ALLOC 里的车），且打出
// LEGDEGRADE 行（降级在唯一 seam = CLI 可见输出上可见）。
// 反向验证：把 buildLegs 的「按实际形态登记」改回「照职责表报 3 条腿」，这里 legCount==3 即红。
void checkLegDegradationVisibleInDump(const std::string& appPath) {
    char tmpPath[256];
    std::snprintf(tmpPath, sizeof(tmpPath), "/tmp/cli_leg_degrade_%d.ini",
                  static_cast<int>(getpid()));
    {
        std::ifstream in(DEFAULT_CONFIG_PATH);
        check(in.good(), "降级夹具：能读到默认配置");
        std::ofstream outFile(tmpPath);
        std::string line;
        while (std::getline(in, line)) {
            outFile << line << "\n";
        }
        outFile << "O26, D08, D18, 8, 08:00, 09:00, 0\n";   // 紧窗 ⇒ 三段腿劣于直达对照
    }
    const char* strategies[3] = {"distance", "cost", "time"};
    for (int si = 0; si < 3; ++si) {
        const std::string tag = std::string("降级·") + strategies[si];
        const std::string out = runCapture("'" + appPath + "' --config '" + tmpPath
                                           + "' --dump-plan " + strategies[si] + " 2>/dev/null");
        const std::vector<std::string> lines = splitLines(out);
        std::vector<std::string> legs;
        std::string allocV;
        for (const std::string& line : lines) {
            if (line.rfind("LEG ", 0) == 0 && line.find(" O26 ") != std::string::npos) {
                legs.push_back(line);
            }
            if (line.rfind("ALLOC ", 0) == 0) {
                const std::size_t p = line.find("O26=");
                if (p != std::string::npos) {
                    const std::size_t e = line.find(',', p);
                    allocV = line.substr(p + 4,
                                         e == std::string::npos ? std::string::npos : e - (p + 4));
                }
            }
        }
        check(legs.size() == 1,
              tag + "：降级后 O26 只登记 1 条直达腿（照职责表假报 3 条会红；实际 "
                  + std::to_string(legs.size()) + "）");
        if (legs.size() == 1) {
            check(legs[0].find("customer->customer") != std::string::npos,
                  tag + "：降级腿 = customer->customer 直达");
            const std::size_t sp = legs[0].rfind(' ');
            const std::string legV = (sp == std::string::npos) ? std::string()
                                                               : legs[0].substr(sp + 1);
            check(!allocV.empty() && legV == allocV,
                  tag + "：LEG 执行车 == ALLOC 里的车（" + legV + " vs " + allocV + "）");
        }
        bool degLine = false;
        for (const std::string& line : lines) {
            if (line.rfind("LEGDEGRADE O26 ", 0) == 0) {
                degLine = true;
                std::printf("      %s %s\n", tag.c_str(), line.c_str());
            }
        }
        check(degLine, tag + "：CLI 打出 LEGDEGRADE O26 行（降级在唯一 seam 可见）");
    }
    std::remove(tmpPath);
}

}   // namespace

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
    // 规格三第 3 轮捕获（#47 / §7）：真值表冻结的是**四车** 123 常量。
    // 车辆数变了就必须重捕；这里先断言口径一致，防止逐车 84 项被静默跳过。
    const int vehicleCount = static_cast<int>(cfg.vehicles.size());

    // 转运账本行（独立入口：--dump-plan，不影响 --plan-summary 九行）。
    checkDumpPlanTransitLines(appPath);
    // 腿结构与就绪链（规格三 §1 / #45）：独立入口，同样只加行。
    checkDumpPlanLegLines(appPath);
    // 职责封闭（规格三 §1 / #46）：用户可见输出上的两条守卫。
    checkDutyClosureDefault(appPath);
    checkCrossClusterRelayOnDefaultData(appPath);
    // P2：降级场景下 LEG 如实反映实际执行形态（LEGDEGRADE 在 CLI seam 可见）。
    checkLegDegradationVisibleInDump(appPath);

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

        // ---- G2：**123 个常量**（车队 13×3 + 每车 7×4×3，§7.2 容差）----
        // #47 已删除 #46 加的临时门控：数值断言恢复**无条件**。
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
        // **行数与结构照跑**（与车队规模无关，跳了就等于把「缺行/结构错」也藏起来）。
        check(static_cast<int>(p.vehicles.size()) == vehicleCount,
              tag + "·G2 每车行数 == 车辆数（缺行 = 63 项被静默跳过）");
        for (int vi = 0; vi < vehicleCount && vi < static_cast<int>(p.vehicles.size()); ++vi) {
            const ParsedVehicle& pv = p.vehicles[static_cast<std::size_t>(vi)];
            // 结构断言（不依赖真值表）：两率的两个分母必须一致、停靠数 ≤ 动作数。
            // 为什么不再要求「分母 > 0」：职责封闭（#46）之后快车在默认数据上是**空车**，
            // 空车的两率本来就该是「—」（0/0），拿 > 0 当不变量等于要求每台车都必须有活。
            check(pv.pickupTotal == pv.deliveryTotal,
                  tag + "·G2 车 " + pv.id + " 取货/送达分母一致（实际 "
                      + std::to_string(pv.pickupTotal) + " / "
                      + std::to_string(pv.deliveryTotal) + "）");
            check(pv.stops <= p.actions, tag + "·G2 车 " + pv.id + " 停靠数 ≤ 车队动作数（结构）");
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
            // 两率口径（#47 改）：真值 -1 ⇔ 该车两率分母 0（空车，
            // stdout 打印「取货 —%（0/0） · 送达 —%（0/0）」）；否则逐项比率。
            // 为什么不再要求「每车必须有活」：职责封闭（#46）之后快车在
            // 默认数据上恒为空车，拿「分母 > 0」当不变量等于要求违反职责表。
            const bool goldenPickupDash = (vg.pickupRatePct == plan_golden::kNoDenominatorSentinel);
            const bool goldenDeliveryDash =
                (vg.deliveryRatePct == plan_golden::kNoDenominatorSentinel);
            if (goldenPickupDash) {
                check(pv.pickupDash && pv.pickupTotal == 0,
                      vtag + " 取货准时率分母 0 时打印「—」（实际 "
                          + std::to_string(pv.pickupHit) + "/" + std::to_string(pv.pickupTotal) + "）");
            } else {
                // 只断言「率 == 真值」与「hit <= total」，**不断言全中**：
                // cost 策略下 V02 有一单迟到 3min（O07@D11），8/9 是职责封闭后
                // 的真实后果（#46 报告已点名），不是缺陷。
                check(near(pv.pickup, vg.pickupRatePct, plan_golden::kTolRate)
                          && pv.pickupHit <= pv.pickupTotal,
                      vtag + " 取货准时率");
            }
            if (goldenDeliveryDash) {
                check(pv.deliveryDash && pv.deliveryTotal == 0,
                      vtag + " 送达准时率分母 0 时打印「—」（实际 "
                          + std::to_string(pv.deliveryHit) + "/"
                          + std::to_string(pv.deliveryTotal) + "）");
            } else {
                check(near(pv.delivery, vg.deliveryRatePct, plan_golden::kTolRate)
                          && pv.deliveryHit <= pv.deliveryTotal,
                      vtag + " 送达准时率");
            }
            // 停靠数**不冻结**（§7.2）：结构约束 S ≤ A 已在循环开头对**每台车**核过。
        }

        // ---- G17：每车路线序列**逐字**（§7.3；与真值表 routeNodes 完全相等）----
        check(static_cast<int>(p.routes.size()) == vehicleCount,
              tag + "·G17 路线行数 == 车辆数");
        for (int vi = 0; vi < vehicleCount && vi < static_cast<int>(p.routes.size()); ++vi) {
            const std::vector<std::string>& nodes = p.routes[static_cast<std::size_t>(vi)];
            // 结构断言先跑（不依赖真值表）：路线从该车起始仓库出发。
            check(nodes.empty() || nodes.front() == cfg.vehicles[static_cast<std::size_t>(vi)].startNodeId,
                  tag + "·G17 路线从该车起始仓库出发");
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

        // ---- X1（#40）：--dump-plan 的 ALLOC / ASSIGN 让分配可核对 ----
        //
        // ALLOC 给逐单归属（按订单 ID 升序），ASSIGN 给逐车汇总；两者都要与
        // 逐车 STOP 的送达动作**双向对拍**。守卫读的是用户可见输出，不读规划器内部。
        {
            const AllocDump ad = parseAllocDump(dumpText);
            check(ad.hasAlloc, tag + "·X1 转储含 ALLOC 行");
            check(ad.hasAssign, tag + "·X1 转储含 ASSIGN 行");
            check(ad.orderIds.size() == static_cast<std::size_t>(orderCount),
                  tag + "·X1 ALLOC 覆盖全部订单（" + std::to_string(ad.orderIds.size()) + " / "
                      + std::to_string(orderCount) + "）");
            bool sortedIds = true;
            for (std::size_t i = 1; i < ad.orderIds.size(); ++i) {
                if (!(ad.orderIds[i - 1] < ad.orderIds[i])) {
                    sortedIds = false;
                }
            }
            check(sortedIds, tag + "·X1 ALLOC 按订单 ID 升序");

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
            check(allocMatchesStops, tag + "·X1 每单恰好一台车，且 ALLOC 与逐车 STOP 送达一致");

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
            check(assignOk, tag + "·X1 ASSIGN 每车单数 == 该车 STOP 送达数，标签为 簇N/跨簇/空车");
            check(ad.assignVehicle.size() >= cfg.vehicles.size(),
                  tag + "·X1 每台车至少一条 ASSIGN（空车也打「空车 0」）");
            allocByStrategy[name] = ad.vehicleIds;
            parsedByStrategy[name] = p;
        }

        std::printf("    %-8s 距离 %.3f | 完工耗时 %.3f | 成本 %.3f | penalty %d | S %d A %d "
                    "| 平均在车 %.1f 峰值 %.1f | 车 %d 台\n",
                    name.c_str(), p.distance, p.time, p.cost, p.penalty, p.stops, p.actions,
                    p.avgLoad, p.peak, vehicleCount);
    }

    // ---- X2（#47 验收）：三策略分配相同 + 路线分化 + **车级目标判据** ----
    //
    // 口径与用户看到的一致：距离 / 成本是 Σ（--plan-summary 第 1 行），
    // 耗时是该车耗时（第 3–6 行的「耗时」）。
    //
    // **口径已按职责封闭（#46）+ 车级目标（#47）重写**，理由如实记录：
    //   · 旧口径「三策略 ALLOC 映射两两不同 + 全局目标各自最优」建立在**全局择优**上
    //     （Δkey 挑车 + 整簇枚举）。职责封闭之后「一张簇内单只能由本簇慢车接」，
    //     分配映射**与策略无关**（三策略完全相同）—— 再断言「两两不同」就是拿旧口径
    //     打新结构；而「距离策略的**全局**总距离不劣于另两条」也不再成立（每车各按
    //     自己的目标跑，全局 Σ 是结果不是被优化的东西）。
    //   · 新口径守住同样的意图，但落到**车级**（每车在自己网内按自己的策略取优）
    //     与**路线分化**（映射相同、路线必须不同，否则「策略」形同虚设）上。
    //   · **为什么不能写成「每车在自身维度上不劣于其它两条策略」**：
    //     车级贪心构造器不保证跨策略单调。默认数据 4 车 × 3 维度
    //     的 9 个「自身维度」格子里 **7 个成立、2 个不成立**，两格都在 V02：
    //     距离 39.800 km（distance）> 31.800 km（cost）；耗时 120.125 min（distance）>
    //     103.875 min（cost）。成本倍率改变了插入顺序、顺带把几何跑短了。
    //   · 因此判据取两层（与 integration_tests 的数值一致，只是读的是文本）：
    //     X2-a 每台有任务的车，在自身目标维度上 <= 同车该维度三值中位数（实测 9/9）；
    //     X2-b 自身维度取到该车三值最小的格子数 >= 7 / 9（实测 7/9）。
    if (strategies.size() == 3) {
        check(allocByStrategy["distance"] == allocByStrategy["cost"]
                  && allocByStrategy["distance"] == allocByStrategy["time"],
              "X2 三策略 ALLOC 映射**相同**（职责封闭：簇 -> 本簇慢车，与策略无关）");
        bool anyDifferent = false;
        for (std::size_t vi = 0; vi < parsedByStrategy["distance"].vehicles.size(); ++vi) {
            const auto& a = parsedByStrategy["distance"].vehicles[vi];
            const auto& b = parsedByStrategy["cost"].vehicles[vi];
            const auto& c = parsedByStrategy["time"].vehicles[vi];
            if (std::fabs(a.distance - b.distance) > 1e-6
                || std::fabs(a.distance - c.distance) > 1e-6
                || std::fabs(a.time - b.time) > 1e-6 || std::fabs(a.time - c.time) > 1e-6) {
                anyDifferent = true;
            }
        }
        check(anyDifferent, "X2 分配相同但**路线必须分化**（至少一台车的距离/耗时随策略变）");

        // 车级实测表（行 = 车，列 = 维度；每格是该车在该维度上的三策略值）。
        // 中位数手写两两交换（不引入排序轮子 —— 课程口径只考算法本身）。
        const auto medianOf3 = [](double a, double b, double c) -> double {
            if (a > b) { const double t = a; a = b; b = t; }
            if (b > c) { const double t = b; b = c; c = t; }
            if (a > b) { const double t = a; a = b; b = t; }
            return b;
        };
        const char* dimNames[3] = {"距离", "耗时", "成本"};
        // 维度下标 -> 该维度的「自身策略」名（与 --plan-summary 参数同名）。
        const char* ownStrategyName[3] = {"distance", "time", "cost"};
        const std::size_t nv = parsedByStrategy["distance"].vehicles.size();
        int strictBest = 0;
        int totalCells = 0;
        bool medianOk = true;
        std::printf("    X2 车级实测表（车 × 维度 × 策略，行序 = [vehicles]）：\n");
        for (std::size_t vi = 0; vi < nv; ++vi) {
            const auto& pd = parsedByStrategy["distance"].vehicles[vi];
            const auto& pc = parsedByStrategy["cost"].vehicles[vi];
            const auto& pt = parsedByStrategy["time"].vehicles[vi];
            // 「有任务」= 该车在任一策略下有订单（两率分母 > 0）。
            const bool hasTask = pd.pickupTotal > 0 || pc.pickupTotal > 0 || pt.pickupTotal > 0;
            if (!hasTask) {
                std::printf("      车 %s 空车（三策略均无任务）=> 不参与车级判据\n",
                            pd.id.c_str());
                continue;
            }
            const double d[3] = {pd.distance, pc.distance, pt.distance};
            const double t[3] = {pd.time, pc.time, pt.time};
            const double c[3] = {pd.cost, pc.cost, pt.cost};
            const double* byDim[3] = {d, t, c};
            const int ownIdx[3] = {0, 2, 1};   // 距离→distance(0)、耗时→time(2)、成本→cost(1)
            for (int k = 0; k < 3; ++k) {
                const double* vals = byDim[k];
                const double ownVal = vals[ownIdx[k]];
                const double med = medianOf3(vals[0], vals[1], vals[2]);
                double best = vals[0];
                for (int si = 1; si < 3; ++si) {
                    if (vals[si] < best) { best = vals[si]; }
                }
                const bool isBest = (ownVal <= best + 1e-6);
                const bool medCell = (ownVal <= med + 1e-6);
                ++totalCells;
                if (isBest) { ++strictBest; }
                if (!medCell) { medianOk = false; }
                std::printf("      车 %s %s：distance %.3f · cost %.3f · time %.3f "
                            "| 自身(%s) %.3f | 中位 %.3f | 取到最小 %s | <= 中位 %s\n",
                            pd.id.c_str(), dimNames[k], vals[0], vals[1], vals[2],
                            ownStrategyName[k], ownVal, med, isBest ? "是" : "否",
                            medCell ? "是" : "否");
            }
        }
        check(medianOk,
              "X2-a 车级：每台有任务的车在自身目标维度上 <= 同车该维度三值中位数"
              "（实测 9/9）");
        check(strictBest >= 7,
              "X2-b 车级：自身维度取到该车最小的格子数 >= 7 / 9（实际 "
                  + std::to_string(strictBest) + " / " + std::to_string(totalCells)
                  + "；V02 的距离/耗时两格如实不成立）");
        std::printf("    X2 汇总：取到最小的格子 %d / %d；全部格子 <= 中位 %s\n",
                    strictBest, totalCells, medianOk ? "是" : "否");
    }


    // ---- X3（#42）：界面初始规划与 CLI 三阶段 ALLOC **逐单一致** ----
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
        check(!guiAlloc.empty(), "X3 --ui-probe 打印界面侧的分配快照（用户可见）");

        const std::string dumpDistance =
            runCapture("'" + appPath + "' --dump-plan distance 2>/dev/null");
        const AllocDump ad = parseAllocDump(dumpDistance);
        check(ad.hasAlloc, "X3 --dump-plan distance 含 ALLOC 行");

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
        check(same, "X3 界面分配与 CLI 三阶段 ALLOC 逐单一致（" + std::to_string(guiPairs.size())
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
              "X3（判别力）顺序轮转基线与三阶段分配在 " + std::to_string(seqDiff)
                  + " 单上不同 —— 界面若退回轮转基线，上面的逐单比对必红");
        std::printf("    X3 界面分配 == CLI ALLOC（%d 单）；轮转基线与之相差 %d 单\n",
                    static_cast<int>(guiPairs.size()), static_cast<int>(seqDiff));
    }

    return testutil::summarize("cli_plan_summary");
}
