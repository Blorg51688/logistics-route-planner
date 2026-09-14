#pragma once

#include <vector>

#include "core/LogisticsGraph.h"
#include "core/Order.h"
#include "core/Vehicle.h"

namespace logistics {

// [general] 节的全局参数
struct GeneralConfig {
    double serviceTimeMin = 5.0;            // 每个停靠点的服务时间
    double trafficChangeIntervalSec = 30.0; // 路况模拟间隔
    double trafficChangeRatio = 0.1;        // 每次变化的边占比
    double trafficTimeIncreaseMin = 0.2;    // 耗时增加比例下限
    double trafficTimeIncreaseMax = 0.5;    // 耗时增加比例上限
    double urgentOrderIntervalSec = 60.0;   // Debug 模式自动生成紧急订单的间隔
};

// 一次配置加载的完整结果
struct Config {
    LogisticsGraph      graph;
    std::vector<Vehicle> vehicles;
    std::vector<Order>   orders;
    GeneralConfig        general;
};

} // namespace logistics
