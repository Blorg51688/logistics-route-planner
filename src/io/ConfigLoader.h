#pragma once

#include <string>

#include "core/Config.h"

namespace logistics {

// 把手写 INI 文本组装成 Config（图 + 车辆 + 订单 + 全局参数）。
// 所有结构性问题（缺节、字段数不符、类型错误、端点不存在、ID 重复）都返回 false，
// 并通过 error 给出带行号的说明。
class ConfigLoader {
public:
    static bool load(const std::string& path, Config& out, std::string& error);
    // 直接从 INI 文本加载，便于测试构造各种畸形配置
    static bool parse(const std::string& text, Config& out, std::string& error);
};

} // namespace logistics
