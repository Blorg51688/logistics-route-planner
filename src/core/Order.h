#pragma once

#include <string>

namespace logistics {

// 一个配送订单。时间窗口统一用「相对当日 00:00 的分钟数」表示。
struct Order {
    std::string id;
    std::string nodeId;              // 目标配送点 ID
    double      demandKg = 0.0;      // 货物量
    int         windowStartMin = 0;  // 窗口起
    int         windowEndMin = 0;    // 窗口止
    bool        urgent = false;      // 是否紧急订单
    bool        served = false;      // 是否已配送
};

} // namespace logistics
