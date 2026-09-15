#pragma once

#include <string>
#include <vector>

#include "core/Dijkstra.h"
#include "core/LogisticsGraph.h"
#include "core/Order.h"
#include "core/Vehicle.h"

namespace logistics {

// 规划结果状态。不可行时会给出 reason 且不产生路线。
enum class PlanStatus {
    Ok,
    OverCapacity,   // 总需求超过车辆载重上限
    Unreachable     // 存在无法到达的配送点，或无法返回仓库
};

// 一个配送停靠点的时间与载重记录。
// 时间单位均为分钟；内部以双精度推进，输出时四舍五入为整数。
struct Stop {
    std::string nodeId;
    int    rawArrivalMin   = 0;    // 未经等待修正的到达时刻
    int    waitMin         = 0;    // 早到等待时长
    int    arrivalMin      = 0;    // 送达时刻 = max(窗口起, rawArrival)
    int    departureMin    = 0;    // = arrival + serviceTime
    bool   late            = false;
    int    penaltyMin      = 0;    // = max(0, arrival - 窗口止)
    double remainingLoadKg = 0.0;  // 离开该站时的剩余载重
};

struct RoutePlan {
    PlanStatus               status = PlanStatus::Ok;
    std::string              reason;      // 不可行原因
    std::vector<std::string> nodes;       // 完整序列：起点、途经节点、终点仓库
    std::vector<Stop>        stops;       // 仅配送停靠点，按服务顺序
    double totalDistanceKm = 0.0;
    double totalTimeMin    = 0.0;         // 返回仓库时刻 − 起始时刻（含等待与服务）
    // 抵达起始仓库的时刻。用于让界面的"推进"在送完最后一站后还能再走一步回到仓库，
    // 而不是把车停在最后一个客户处。
    int    returnArrivalMin = 0;
    double totalCostYuan   = 0.0;
    int    totalPenaltyMin = 0;
};

// 从车辆起始仓库出发、按其发车时刻规划，服务完全部订单后返回该仓库。
// 等价于以 (仓库, 发车时刻) 为起点调用 replan。
//
// 贪心规则：每步在所有未服务停靠点中，选"当前位置到它的最短路权重"最小者；
// 权重并列时按节点 ID 字典序取，以保证输出确定（同一输入必得同一条路线）。
RoutePlan planRoute(const LogisticsGraph& graph,
                    const Vehicle& vehicle,
                    const std::vector<Order>& orders,
                    double serviceTimeMin,
                    WeightType weight);

// 从指定位置与指定时刻出发，对剩余未服务订单重新规划，最后返回车辆起始仓库。
// 供"配送过程中插入紧急订单"与"路况变化触发重规划"复用同一套贪心逻辑。
RoutePlan replan(const LogisticsGraph& graph,
                 const Vehicle& vehicle,
                 const std::vector<Order>& remainingOrders,
                 const std::string& currentPositionId,
                 int currentTimeMin,
                 double serviceTimeMin,
                 WeightType weight);

struct InsertResult {
    RoutePlan   plan;
    std::string warning;   // 空表示无冲突
};

// 动态插入紧急订单（§5.3）：
//   把 newOrder **强制标记为紧急**（无论调用方传入什么）后并入剩余订单，
//   以车辆当前位置、当前时刻为起点重算路线。
// 冲突判定：若新订单窗口止早于"从当前位置出发的最早到达时刻"（按耗时维度计算），
// 则无论怎么排都会超时，此时在 warning 中给出提示。
// 按 D13（超时不弃、只记 penalty），订单**仍会被纳入规划**。
InsertResult insertUrgentOrder(const LogisticsGraph& graph,
                               const Vehicle& vehicle,
                               const std::vector<Order>& remainingOrders,
                               const Order& newOrder,
                               const std::string& currentPositionId,
                               int currentTimeMin,
                               double serviceTimeMin,
                               WeightType weight);

// 判断某条有向边是否落在给定路线序列的**相邻两站**之间。
// GUI 的路径高亮与路况重规划的触发判定共用这一条逻辑。
bool isEdgeOnRoute(const std::vector<std::string>& routeNodes,
                   const std::string& fromId,
                   const std::string& toId);

} // namespace logistics
