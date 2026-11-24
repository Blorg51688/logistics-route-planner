// L2 文本守卫（#43 / 规格二 §7.9 G20）：无头演示序列跑通 + 固定步骤横幅顺序正确。
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
    return testutil::summarize("cli_demo_transfer");
}
