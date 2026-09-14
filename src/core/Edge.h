#pragma once

#include <string>

namespace logistics {

struct Edge {
    std::string fromId;
    std::string toId;
    double      distanceKm  = 0.0;   // 距离（公里）
    double      timeMin     = 0.0;   // 当前耗时（分钟），路况变化后会被改写
    double      costYuan    = 0.0;   // 运输成本（元）
    double      baseTimeMin = 0.0;   // 初始耗时，用于判断“耗时增加 ≥20%”与还原路况
    bool        congested   = false; // 是否处于拥堵状态
};

} // namespace logistics
