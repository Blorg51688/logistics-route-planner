#pragma once

#include <vector>

#include "core/LogisticsGraph.h"
#include "core/Order.h"
#include "core/Vehicle.h"

namespace logistics {

// [general] 节的全局参数
struct GeneralConfig {
    double trafficChangeRatio = 0.1;        // 每次变化的边占比
    double trafficTimeIncreaseMin = 0.2;    // 耗时增加比例下限
    double trafficTimeIncreaseMax = 0.5;    // 耗时增加比例上限
    // 软件内时间的事件模型（设计 D24）：每 eventIntervalMin 分钟一个**事件刻**，每刻恰抽 1 个事件；
    // 四类事件的权重（路况 / 紧急订单 / 新客户 / 道路封闭）。
    // 注：原先的 `traffic_change_interval_sec`（真实秒级）与 `urgent_order_interval_sec`（每 tick 节流）
    // 已被本模型取代并**移除**——事件只由软件内时钟驱动，与真实时间和推进粒度都无关。
    int eventIntervalMin = 15;              // 事件抽取间隔（软件内分钟）
    int eventWeightTraffic = 10;            // 路况变化权重
    int eventWeightUrgent = 3;              // 紧急订单权重
    int eventWeightCustomer = 1;            // 新客户权重
    int eventWeightClosure = 1;             // 道路封闭权重
};

// 一次配置加载的完整结果
struct Config {
    LogisticsGraph      graph;
    std::vector<Vehicle> vehicles;
    std::vector<Order>   orders;
    GeneralConfig        general;
};

} // namespace logistics
