#pragma once

#include <cstdio>
#include <string>

// 极简测试工具：无框架、无第三方依赖，供各测试可执行文件共用。
namespace testutil {

inline int g_checks = 0;
inline int g_failures = 0;

inline void check(bool ok, const std::string& what) {
    ++g_checks;
    if (!ok) {
        ++g_failures;
        std::printf("FAIL  %s\n", what.c_str());
    }
}

inline int summarize(const char* suite) {
    std::printf("[%s] %d 项检查，%d 项失败\n", suite, g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}

} // namespace testutil
