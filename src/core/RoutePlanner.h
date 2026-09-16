#pragma once

#include <map>
#include <string>
#include <vector>

#include "core/Dijkstra.h"
#include "core/LogisticsGraph.h"
#include "core/Order.h"
#include "core/Traffic.h"
#include "core/Vehicle.h"

namespace logistics {

// 规划结果状态。不可行时会给出 reason 且不产生路线。
enum class PlanStatus {
    Ok,
    // 单个订单的货量就超过载重上限——多趟也无法解决，这才是真正的载重不可行。
    // 注意：**总需求超过载重不再属于不可行**（D21）：车辆可反复返回仓库取货，
    // 把货暂存在中转站再二次配发，因此改为多趟配送。
    OrderExceedsCapacity,
    Unreachable     // 存在无法到达的配送点，或无法返回仓库
};

// 一个配送停靠点的时间与载重记录。
// 时间单位均为分钟；内部以双精度推进，输出时四舍五入为整数。
struct Stop {
    std::string nodeId;
    int    rawArrivalMin   = 0;    // 未经等待修正的到达时刻
    int    waitMin         = 0;    // 早到等待时长
    int    arrivalMin      = 0;    // 送达时刻 = max(窗口起, rawArrival)
    bool   late            = false;
    int    penaltyMin      = 0;    // = max(0, arrival - 窗口止)
    double remainingLoadKg = 0.0;  // 离开该站时的剩余载重
};

// 一趟行程：车辆的一段连续行程。
// 起点 = 上一趟的终点（首趟为规划起点），终点 = 起始仓库或某个中转站。
struct Trip {
    std::vector<std::string> nodes;         // 含本趟起点
    std::vector<int>         nodeArrivalMin;
    std::vector<bool>        nodeIsStop;
    std::vector<Stop>        stops;
    // 本趟车上装载的货量（不变量：任一趟都不得超过载重上限）。
    // 与"剩余待送总量"是两回事——多趟模式下车辆不会一次装完全部货物。
    double loadKg = 0.0;
    double totalDistanceKm = 0.0;
    double totalCostYuan   = 0.0;
    double totalTimeMin    = 0.0;
    std::string endNodeId;                  // 本趟终点
};

struct RoutePlan {
    PlanStatus               status = PlanStatus::Ok;
    std::string              reason;      // 不可行原因
    std::vector<std::string> nodes;       // 完整序列：起点、途经节点、终点仓库
    // 与 nodes 一一对应的抵达时刻。界面的"推进"据此逐个节点前进，
    // 而不是只跳过配送点——仓库与中转站的到达也算一次位置变化。
    std::vector<int>         nodeArrivalMin;
    // 与 nodes 一一对应的标记：该节点是否为一次配送停靠
    // （同一节点可能在序列中出现多次，只有作为停靠点的那一次为 true）
    std::vector<bool>        nodeIsStop;
    // 与 nodes 一一对应：该节点属于第几趟。界面据此把"到达某节点"翻译成
    // "在某个中转站卸货/取货"，而不必自己反推趟边界。
    std::vector<std::size_t> nodeTripIndex;
    std::vector<Stop>        stops;       // 仅配送停靠点，按服务顺序
    double totalDistanceKm = 0.0;
    double totalTimeMin    = 0.0;         // 返回仓库时刻 − 起始时刻（含等待与服务）
    // 抵达起始仓库的时刻（路线汇总字段之一，供界面与测试核对配送结束时间）。
    int    returnArrivalMin = 0;
    double totalCostYuan   = 0.0;
    int    totalPenaltyMin = 0;

    // ---- 多趟结构（D20/D21）----
    // trips 是**真源**；上面的 nodes/nodeArrivalMin/nodeIsStop/stops
    // 是由它展平（flatten）出来的兼容视图，只在一处生成，不会各自维护。
    std::vector<Trip>         trips;
};

// 从车辆起始仓库出发、按其发车时刻规划，服务完全部订单后返回该仓库。
// 等价于以 (仓库, 发车时刻) 为起点调用 replan。
//
// 贪心规则：每步在所有未服务停靠点中，选"当前位置到它的最短路权重"最小者；
// 权重并列时按节点 ID 字典序取，以保证输出确定（同一输入必得同一条路线）。
RoutePlan planRoute(const LogisticsGraph& graph,
                    const Vehicle& vehicle,
                    const std::vector<Order>& orders,
                    WeightType weight);

// 从指定位置与指定时刻出发，对剩余未服务订单重新规划，最后返回车辆起始仓库。
// 供"配送过程中插入紧急订单"与"路况变化触发重规划"复用同一套贪心逻辑。
// OnboardItem：车辆**此刻已经载在车上**的货（节点 + 货量）。
// 供"途中重规划"时告诉规划器车不是空的——否则规划会假设车在起点重新装货。
struct OnboardItem {
    std::string nodeId;
    double      kg = 0.0;
};

// 中转站不参与排线（实测参与更差，见设计 §16 P17/P25）。
RoutePlan replan(const LogisticsGraph& graph,
                 const Vehicle& vehicle,
                 const std::vector<Order>& remainingOrders,
                 const std::string& currentPositionId,
                 int currentTimeMin,
                 WeightType weight,
                 const std::vector<OnboardItem>& onboard = std::vector<OnboardItem>());

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
                               WeightType weight,
                               const std::vector<OnboardItem>& onboard
                                   = std::vector<OnboardItem>());

// 从已规划好的 plan 中切出"尚未走完的部分"，**保留趟结构**。
// 供增量式重规划使用：若把整条剩余路线压成一趟，增量重规划之后
// plan.trips 会只剩 1 趟，界面上的「第 N 趟」就全变成「第 1 趟」。
RoutePlan sliceRemainder(const RoutePlan& plan, std::size_t fromNodeIndex);

// 增量式重规划（设计 §5.7 / D22）。
// 把当前路线按**停靠点**切成若干 leg，只对"走过被路况命中的边"的那些 leg
// 重新求最短路，其余 leg 原样复用，**停靠顺序不变**。
// **逐趟增量**：剩余路线的每一趟各自只重算受影响的 leg，往返趟之间互不影响。
// 仅当任何一趟增量失败（序列不全 / 受影响 leg 已不可达）时，才整体退回全量重算。
// 是否真的走了增量由 usedIncremental 回报——**调用方不要自己从趟数猜**：
// 早期实现按"剩余路线是否单趟"猜，多趟时会把走成增量的情况误报成"退回全量"。
RoutePlan replanIncremental(const LogisticsGraph& graph,
                            const Vehicle& vehicle,
                            const std::vector<Order>& remainingOrders,
                            const RoutePlan& previous,
                            int currentTimeMin,
                            WeightType weight,
                            const TrafficReport& report,
                            double thresholdRatio,
                            // 与 replan 一致：回退到全量重算时在途货必须一并带上
                            const std::vector<OnboardItem>& onboard
                                = std::vector<OnboardItem>(),
                            // 回报本函数是否真的走了增量路径（false = 退回了全量）
                            bool* usedIncremental = nullptr);

// 判断某条有向边是否落在给定路线序列的**相邻两站**之间。
// GUI 的路径高亮与路况重规划的触发判定共用这一条逻辑。
bool isEdgeOnRoute(const std::vector<std::string>& routeNodes,
                   const std::string& fromId,
                   const std::string& toId);

} // namespace logistics
