#pragma once

#include <string>

namespace logistics {

// 配送车辆。第一版单车辆，接口按集合形式设计以便扩展多车辆。
struct Vehicle {
    std::string id;
    std::string startNodeId;          // 起始仓库 ID
    double      capacityKg = 0.0;     // 载重上限
    int         departTimeMin = 0;    // 发车时刻，相对当日 00:00 的分钟数
};

} // namespace logistics
