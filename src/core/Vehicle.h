#pragma once

#include <string>

namespace logistics {

// 配送车辆。接口按集合形式设计以便扩展多车辆。
//
// 六字段（规格一 #19 Q1/Q2 + 规格二 §3.1）：两因子只作用于**当前耗时**与成本，
// 距离 / 等待 / penalty 均不乘；图表示继续显示基础边权。
struct Vehicle {
    std::string id;
    // 起始节点 ID：快车 = 起始仓库；慢车 = 本簇中转站（规格七 #63：慢车出生在自己簇里）。
    std::string startNodeId;
    double      capacityKg = 0.0;     // 载重上限
    int         departTimeMin = 0;    // 发车时刻，相对当日 00:00 的分钟数
    double      timeFactor = 1.0;     // 耗时倍率（> 1 = 更慢）
    double      costFactor = 1.0;     // 成本倍率
};

} // namespace logistics
