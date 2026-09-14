#pragma once

namespace logistics {

// 图上权重的维度。
// 两种规划策略（最短距离 / 最低成本）共用同一份最短路实现，只在这里切换
// 权重来源；图表示输出（邻接矩阵）同样按维度切换显示内容。
enum class WeightType {
    Distance,
    Time,
    Cost
};

// 从三个维度的值中取出指定维度的权重。
// 放在这里是为了让 Dijkstra 与 LogisticsGraph 共用同一份取值逻辑，避免各写一遍。
inline double pickWeight(double distance, double time, double cost, WeightType type) {
    switch (type) {
        case WeightType::Distance: return distance;
        case WeightType::Time:     return time;
        case WeightType::Cost:     return cost;
    }
    return distance;
}

} // namespace logistics
