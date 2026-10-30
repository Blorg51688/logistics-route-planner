#pragma once

#include <string>

namespace logistics {

// 一个配送订单。时间窗口统一用「相对当日 00:00 的分钟数」表示。
struct Order {
    std::string id;
    std::string nodeId;              // 送达点 B（普通单=客户配送点；紧急取送单=送达点 B）
    std::string pickupNodeId;        // 取货点 A（仅紧急取送单使用；普通单为空）
    double      demandKg = 0.0;      // 货物量
    int         windowStartMin = 0;  // 窗口起
    int         windowEndMin = 0;    // 窗口止
    bool        urgent = false;      // 是否紧急订单
    bool        picked = false;      // 紧急取送单是否已从 A 取到包裹（执行中：已取未送）
    bool        served = false;      // 是否已配送
};

} // namespace logistics
