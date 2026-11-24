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
#include <map>
#include <string>
#include <vector>

#include <unistd.h>

#include "core/Config.h"
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

struct Parsed {
    double distance = 0.0, time = 0.0, cost = 0.0;
    int penalty = 0, stops = 0, actions = 0;
    double pickup = 0.0, delivery = 0.0, urgent = 0.0, normal = 0.0;
    bool urgentDash = false;
    double avgLoad = 0.0, utilization = 0.0, peak = 0.0;
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

bool parseSummary(const std::string& text, Parsed& p, std::vector<std::string>& lines) {
    lines = splitLines(text);
    if (lines.size() < 4) {
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

} // namespace

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

    std::vector<std::string> strategies;
    if (strategy == "all") {
        strategies = {"distance", "cost", "time"};
    } else {
        strategies = {strategy};
    }

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
        check(parseSummary(out, p, lines), tag + "：--plan-summary 输出四行且可解析");

        // ---- G1：标签 / 单位 / 顺序与 §6.3 逐字一致 ----
        const std::vector<std::string> tokens1 = {"总距离", "km", "总耗时", "min", "总成本", "元",
                                                  "总 penalty", "min", "停靠", "站", "动作", "次"};
        const std::vector<std::string> tokens2 = {
            "取货准时率", "%", "送达准时率", "%", "紧急", "普通", "平均在车", "kg",
            "利用率", "%", "峰值", "kg"};
        if (lines.size() >= 4) {
            checkOrdered(lines[0], tokens1, tag + "·G1 第1行");
            checkOrdered(lines[1], tokens2, tag + "·G1 第2行");
            check(lines[2].rfind("超时订单：", 0) == 0, tag + "·G1 第3行以「超时订单：」开头");
            check(lines[3].rfind("路线", 0) == 0, tag + "·G1 第4行以「路线」开头");
        }

        // ---- G2：13 字段数值 == plan_golden.h ----
        check(near(p.distance, g->distanceKm, plan_golden::kTolMetric), tag + "·G2 总距离");
        check(near(p.time, g->timeMin, plan_golden::kTolMetric), tag + "·G2 总耗时");
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

        // ---- G6：第 9 项（紧急）在分母 0 时打印「紧急 —」 ----
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

        // ---- L3：--dump-plan + oracle 复算对拍 ----
        const std::string dumpText = runCapture("'" + appPath + "' --dump-plan " + name
                                                + " 2>/dev/null");
        char tmpPath[256];
        std::snprintf(tmpPath, sizeof(tmpPath), "/tmp/cli_plan_summary_%s_%d.txt", name.c_str(),
                      static_cast<int>(getpid()));
        if (!dumpText.empty()) {
            FILE* f = std::fopen(tmpPath, "w");
            if (f != nullptr) {
                std::fwrite(dumpText.data(), 1, dumpText.size(), f);
                std::fclose(f);
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

        std::printf("    %-8s 距离 %.3f | 耗时 %.3f | 成本 %.3f | penalty %d | S %d A %d "
                    "| 平均在车 %.1f 峰值 %.1f\n",
                    name.c_str(), p.distance, p.time, p.cost, p.penalty, p.stops, p.actions,
                    p.avgLoad, p.peak);
    }

    return testutil::summarize("cli_plan_summary");
}
