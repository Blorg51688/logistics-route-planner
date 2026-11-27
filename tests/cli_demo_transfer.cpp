// L2 文本守卫（#43 / 规格二 §7.9 G20）：无头演示序列跑通 + 固定步骤横幅顺序正确。
//
// #47 扩到 **M2**（规格三 §7：跨车转运在默认数据上可演示）：
//   「插入一张跨簇普通单 → 账本出现一笔三段接力」。
//   用一份「默认配置 + O26（D08→D18 8kg）」的临时 ini 跑 --dump-plan，
//   断言 TRANSIT 行出现且卸入车 ≠ 取走车、dwell ≤ 60、期末无在站。
//   为什么不跑 GUI 演示路径：--demo 的插单动作是「插入紧急订单」（窗口 60min），
//   它的跨簇单走快车直达特快通道、**不进接力**；而 M2 要演的是「普通单」的
//   三段腿。两者是不同语义，拿快车直达当接力断言只会一直红。
//
// 用法（CMake 传参）：cli_demo_transfer <app 路径>
//
// 为什么必须带 --render-window：单独 --demo N 在无头环境下会进入交互事件循环并挂死
// （实测超时；规格二 §7.7 / resolution-25 P11）。本测试用**真二进制**跑完整演示路径，
// 断言退出码与导出日志里的 **[演示] k/N …** 横幅——只钉横幅，不钉数字（哪台车接单、
// 转运增益多少随实现与数据变）。

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>

#include <unistd.h>

#include "test_util.h"

#ifndef DEFAULT_CONFIG_PATH
#error "缺少 DEFAULT_CONFIG_PATH 编译定义"
#endif

using testutil::check;

namespace {

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

bool fileExists(const std::string& path) {
    std::ifstream f(path.c_str());
    return f.good();
}

std::string readFile(const std::string& path) {
    std::ifstream f(path.c_str());
    std::string out((std::istreambuf_iterator<char>(f)),
                    std::istreambuf_iterator<char>());
    return out;
}

// M2 演示的可执行形式：默认配置 + 一张跨簇普通单（O26：D08→D18 8kg）。
//
// 为什么用 --dump-plan 而不是读 GUI 账本面板：--dump-plan 的 TRANSIT 行与 GUI 账本
// 同源（buildTransitLedger 是唯一实现），且无需图形环境。守卫读的仍然是
// 「用户能看到的输出」，不是内部函数。
void checkM2RelayOnDefaultData(const std::string& appPath) {
    char tmpPath[256];
    std::snprintf(tmpPath, sizeof(tmpPath), "/tmp/cli_demo_m2_%d.ini",
                  static_cast<int>(getpid()));
    {
        std::ifstream in(DEFAULT_CONFIG_PATH);
        check(in.good(), "M2：能读到默认配置（" DEFAULT_CONFIG_PATH "）");
        std::ofstream outFile(tmpPath);
        std::string line;
        while (std::getline(in, line)) {
            outFile << line << "\n";
        }
        // 跨簇普通单：D08（子网 1）→ D18（子网 3）。
        // 规格六 #60：1->3 走廊的批量站间腿 14:02 之后才卸到 T03，13:00 的截止会让它
        // 按客户 penalty 退回直达（那就测不到接力结构）。夹具参数放到 15:00。
        outFile << "O26, D08, D18, 8, 08:00, 15:00, 0\n";
    }

    const char* strategies[3] = {"distance", "cost", "time"};
    for (int si = 0; si < 3; ++si) {
        const std::string tag = std::string("M2·") + strategies[si];
        const std::string cmd = "'" + appPath + "' --config '" + tmpPath + "' --dump-plan "
                                + strategies[si] + " 2>/dev/null";
        FILE* pipe = popen(cmd.c_str(), "r");
        std::string out;
        if (pipe != nullptr) {
            char buf[4096];
            while (std::fgets(buf, sizeof(buf), pipe) != nullptr) {
                out += buf;
            }
            pclose(pipe);
        }
        const std::vector<std::string> lines = splitLines(out);

        // ① 账本笔数**按策略分别断言**（规格四 #54 的口径修订，理由见下方 ③ 的注释）。
        //   distance / time：三段接力优于直达 ⇒ 账本恰 1 笔；
        //   cost：跨城尺度下 relay 要让三台车各自跨城往返，增量成本 ≈3000 元 vs
        //         快车一趟直达 1887 元 ⇒ §5.9「不更差兜底」必然选直达 ⇒ 账本 0 笔。
        //   这不是放宽判据：直达时必须**自洽**（LEG 只有 1 条 customer->customer 且
        //   由快车执行），任一条不符仍然红。
        const bool expectRelay = (std::string(strategies[si]) != std::string("cost"));
        std::string transit;
        int transitCount = 0;
        for (const std::string& line : lines) {
            if (line.rfind("TRANSIT O26 ", 0) == 0) {
                ++transitCount;
                transit = line;
            }
        }
        if (expectRelay) {
            check(transitCount == 1,
                  tag + "：默认数据 + 跨簇单后账本恰 1 笔 TRANSIT（实际 "
                      + std::to_string(transitCount) + "）");
        } else {
            check(transitCount == 0,
                  tag + "：成本策略走直达 ⇒ 账本应为空（实际 "
                      + std::to_string(transitCount) + " 笔）");
        }

        // ② 形状：TRANSIT <订单> <站> <kg> <卸入车> <HH:MM> <取走车> <HH:MM> <dwell> <状态>
        //    卸入车 ≠ 取走车（真转运，不是同车降载）、dwell ∈ [0,60]、状态「已送达」（期末无在站）。
        if (transitCount == 1) {
            std::vector<std::string> f;
            std::size_t pos = 0;
            while (pos <= transit.size()) {
                const std::size_t sp = transit.find(' ', pos);
                f.push_back(transit.substr(pos, sp == std::string::npos ? std::string::npos
                                                                        : sp - pos));
                if (sp == std::string::npos) {
                    break;
                }
                pos = sp + 1;
            }
            const bool shapeOk = f.size() >= 10 && f[0] == "TRANSIT" && f[1] == "O26"
                                 && !f[2].empty() && f[4] != "-" && f[6] != "-" && f[4] != f[6];
            check(shapeOk, tag + "：TRANSIT 两端齐全且 inVehicle ≠ outVehicle（"
                               + transit + "）");
            if (f.size() >= 10) {
                const int dwell = std::atoi(f[8].c_str());
                check(dwell >= 0 && dwell <= 60,
                      tag + "：dwell " + f[8] + "min ∈ [0, 60]（货物停留上限）");
                check(f[9] == "已送达",
                      tag + "：状态「已送达」（期末无在站，实际 " + f[9] + "）");
            }
            std::printf("      %s TRANSIT：%s\n", tag.c_str(), transit.c_str());
        }

        // ③ 三段腿都在：O26 恰 3 条 LEG（L1 本簇慢车 / L2 快车 / L3 对簇慢车）。
        int legCount = 0;
        for (const std::string& line : lines) {
            if (line.rfind("LEG ", 0) == 0 && line.find(" O26 ") != std::string::npos) {
                ++legCount;
                std::printf("      %s LEG：%s\n", tag.c_str(), line.c_str());
            }
        }
        if (expectRelay) {
            check(legCount == 3,
                  tag + "：O26 恰 3 条 LEG 腿（实际 " + std::to_string(legCount) + "）");
        } else {
            // 直达时必须**如实**只有 1 条腿，且是 customer->customer（不伪造三段腿）。
            check(legCount == 1,
                  tag + "：成本策略直达 ⇒ O26 只有 1 条 LEG 腿（实际 "
                      + std::to_string(legCount) + "）");
            for (const std::string& line : lines) {
                if (line.rfind("LEG ", 0) == 0 && line.find(" O26 ") != std::string::npos) {
                    check(line.find(" customer->customer ") != std::string::npos,
                          tag + "：直达腿必须是 customer->customer：" + line);
                }
            }
        }
    }
    std::remove(tmpPath);
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::printf("用法: cli_demo_transfer <app 路径>\n");
        return 2;
    }
    const std::string appPath = argv[1];
    const std::string tag = "/tmp/cli_demo_transfer_"
                            + std::to_string(static_cast<long long>(getpid()));
    const std::string png = tag + ".png";
    const std::string log = tag + ".log";

    // 3 轮 = 8 个固定步骤：每轮「推进一站 + 模拟路况」，第 2 轮加插紧急单，第 3 轮加封路。
    const std::string cmd = std::string("QT_QPA_PLATFORM=offscreen '") + appPath
                            + "' --demo 3 --render-window '" + png + "' --export-log '" + log
                            + "' >/dev/null 2>&1";
    const int rc = std::system(cmd.c_str());
    const int exitCode = (rc >= 0) ? WEXITSTATUS(rc) : -1;
    check(exitCode == 0, "G20：--demo 3 --render-window --export-log 必须 exit 0（实际 "
                             + std::to_string(exitCode) + "）");
    check(fileExists(png), "G20：演示的窗口渲染 PNG 已落盘");
    check(fileExists(log), "G20：演示的运行日志已导出");
    if (!fileExists(log)) {
        std::remove(png.c_str());
        std::remove(log.c_str());
        return testutil::summarize("cli_demo_transfer");
    }

    const std::vector<std::string> lines = splitLines(readFile(log));
    const std::vector<std::string> expect = {
        "推进一站", "模拟路况", "推进一站", "模拟路况", "插入紧急订单",
        "推进一站", "模拟路况", "封闭道路"};
    std::vector<std::string> actions;
    bool countOk = true;
    std::string firstBad;
    for (const std::string& line : lines) {
        const std::string marker = "[演示] ";
        const std::size_t at = line.find(marker);
        if (at == std::string::npos) {
            continue;
        }
        // 形如：08:00  [演示] 1/8 推进一站
        const std::string body = line.substr(at + marker.size());
        const std::size_t slash = body.find('/');
        const std::size_t space = body.find(' ');
        if (slash == std::string::npos || space == std::string::npos || space < slash) {
            countOk = false;
            if (firstBad.empty()) {
                firstBad = line;
            }
            continue;
        }
        const int k = std::atoi(body.substr(0, slash).c_str());
        const int n = std::atoi(body.substr(slash + 1, space - slash - 1).c_str());
        if (k != static_cast<int>(actions.size()) + 1 || n != static_cast<int>(expect.size())) {
            countOk = false;
            if (firstBad.empty()) {
                firstBad = line;
            }
        }
        actions.push_back(body.substr(space + 1));
    }
    check(countOk, "G20：横幅形如「[演示] k/8 步骤」且 k 从 1 连续递增"
                       + std::string(firstBad.empty() ? "" : "（首个不符：" + firstBad + "）"));
    check(actions.size() == expect.size(),
          "G20：--demo 3 的固定步骤横幅数 == " + std::to_string(expect.size())
              + "（实际 " + std::to_string(actions.size()) + "）");
    bool orderOk = (actions.size() == expect.size());
    std::string diff;
    for (std::size_t i = 0; orderOk && i < expect.size(); ++i) {
        if (actions[i] != expect[i]) {
            orderOk = false;
            diff = "第 " + std::to_string(i + 1) + " 步「" + actions[i] + "」≠「" + expect[i]
                   + "」";
        }
    }
    check(orderOk, "G20：横幅顺序固定（推进一站/模拟路况，隔轮插紧急单，隔三轮封路）"
                       + (diff.empty() ? std::string() : "：" + diff));

    std::printf("    演示日志横幅：\n");
    for (const std::string& a : actions) {
        std::printf("      [演示] %s\n", a.c_str());
    }

    std::remove(png.c_str());
    std::remove(log.c_str());

    // ---- M2：默认数据上「插一张跨簇普通单 → 账本一笔三段接力」----
    checkM2RelayOnDefaultData(appPath);

    return testutil::summarize("cli_demo_transfer");
}
