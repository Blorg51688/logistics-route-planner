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
    // 因此改为多趟分批配送。（注 2026-09-26：中转站不参与排线、不持有库存，
    // 见 docs/设计.md §16 P25/P30。）
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
// 起点 = 上一趟的终点（首趟为规划起点），终点 = 起始仓库。
// 一次站内装卸（缓冲库存机制的记账单元）。
// kgDelta > 0 = 入库（返程顺路寄存缓冲货）；< 0 = 出库（紧急单就地取用）。
struct TransitOp {
    std::string nodeId;        // 中转站
    double      kgDelta = 0.0; // 变动量（kg）
    int         atMin   = 0;   // 发生的软件内时刻
};

// 站内库存快照：供界面显示与测试核对。
struct TransitStock {
    std::string nodeId;
    double      initialKg = 0.0;   // 本次规划开始时的库存
    double      finalKg   = 0.0;    // 本次规划结束时的库存
};

// （注 2026-09-26：中转站不参与排线，故每趟终点都是起始仓库，见 docs/设计.md §16 P25/P30。）
struct Trip {
    std::vector<std::string> nodes;         // 含本趟起点
    std::vector<int>         nodeArrivalMin;
    std::vector<bool>        nodeIsStop;
    std::vector<Stop>        stops;
    // 本趟车上装载的货量（不变量：任一趟都不得超过载重上限）。
    // 与"剩余待送总量"是两回事——多趟模式下车辆不会一次装完全部货物。
    // **口径提醒**：loadKg 只统计"订单货量"，缓冲货一律计入 bufferKg，两者绝不混用；
    // 界面「本趟装载」与既有的装载/容量一致性守卫都只读 loadKg。
    double loadKg = 0.0;
    // 本趟装载的"不属于任何订单的缓冲货"（= 满载出仓时本趟订单用不到的那部分）。
    // 不变量：loadKg + bufferKg <= 车辆载重上限，且 bufferKg >= 0。
    double bufferKg = 0.0;
    // 本趟是否**在仓库装过货**（"出仓"）。用于把"出仓装车"与"接着送车上已有的货"
    // 在导出证据里区分开——否则用车上/站内存货就地满足的那趟（bufferKg 本就该是 0）
    // 会被误报成"出仓却未满载"，把读证据的人引去追不存在的 bug。
    bool loadedFromDepot = false;
    // 本趟是"用车上缓冲/站内库存**就地满足**紧急单"——它**没在仓库装过货**，
    // 因此 bufferKg 本就该是 0，而不是"出仓却未满载"。与上一标记一起，把三种趟分开，
    // 导出证据才不会把合法的 0 误报成缺陷。
    bool supplyInPlace = false;
    // 本趟在返程顺路处把缓冲卸进站的入库操作（至多一条；不顺路则为空）。
    std::vector<TransitOp> bankOps;
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
    // 与 nodes 一一对应：该节点属于第几趟。界面据此跨趟连续推进车辆，
    // 而不必自己反推趟边界。（注 2026-09-26：不再据此翻译"中转站卸货/取货"，
    // 该文案随中转站退出排线一并移除，见 docs/设计.md §16 P25。）
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

    // ---- 中转站缓冲库存（2026-09-26 用户裁定恢复；仅服务紧急单）----
    // 各站期末库存快照（含期初值，便于核对守恒）。
    std::vector<TransitStock> transitStock;
    // 展平后的全部站内装卸（按时刻），供界面按时刻回放与守恒断言。
    std::vector<TransitOp>    transitOps;
};

// 从车辆起始仓库出发、按其发车时刻规划，服务完全部订单后返回该仓库。
// 等价于以 (仓库, 发车时刻) 为起点调用 replan。
//
// 贪心规则：每步在所有未服务停靠点中，选"当前位置到它的最短路权重"最小者；
// 权重并列时按节点 ID 字典序取，以保证输出确定（同一输入必得同一条路线）。
RoutePlan planRoute(const LogisticsGraph& graph,
                    const Vehicle& vehicle,
                    const std::vector<Order>& orders,
                    WeightType weight,
                    // 期初站内库存（缓冲库存机制）。默认空 = 无库存可用（黄金值口径）。
                    // 仓库可发出不属于任何订单的缓冲货——2026-09-26 用户裁定，见
                    // .omd/plans/transit-urgent-buffer.md 的"建模前提"。
                    const std::map<std::string, double>& initialStock
                        = std::map<std::string, double>());

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
                 const std::vector<OnboardItem>& onboard = std::vector<OnboardItem>(),
                 // **当前时刻**的站内库存快照（语义：不是"规划期初"，因为本函数
                 // 就是从当前位置与当前时刻重算）。**不回传**——期末库存由
                 // RoutePlan::transitStock 输出。
                 const std::map<std::string, double>& initialStock
                     = std::map<std::string, double>());

struct InsertResult {
    RoutePlan   plan;
    std::string warning;   // 空表示无冲突

    // ---- 就地满足（缓冲库存机制）的**取用明细**，供调用方把它们落成物理事实 ----
    // 全为 0 / 空 表示这一次走的是现状（候选 A：独立紧急趟回仓库装货）。
    double      carBufferUsedKg = 0.0;   // 车上缓冲被消耗的货量
    std::string stationUsed;             // 被取用的中转站（空 = 没动站内库存）
    double      stationUsedKg = 0.0;     // 从该站取走多少
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
                                   = std::vector<OnboardItem>(),
                               // 车上当前剩余的**缓冲货**（不属于任何订单）。紧急单可先用它
                               // 就地满足，无需回仓库。与 onboard（订单货）是两回事：本机制
                               // 绝不动本趟要送的订单货（不借货）。
                               double carBufferKg = 0.0,
                               // **当前时刻**的站内库存快照（语义同 replan）。
                               const std::map<std::string, double>& initialStock
                                   = std::map<std::string, double>());

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
                            bool* usedIncremental = nullptr,
                            // 与 replan 一致：**当前时刻**的站内库存快照。
                            // 增量路径必须把它**带走**（否则路况重规划后库存凭空归零）。
                            const std::map<std::string, double>& initialStock
                                = std::map<std::string, double>());

// 判断某条有向边是否落在给定路线序列的**相邻两站**之间。
// GUI 的路径高亮与路况重规划的触发判定共用这一条逻辑。
bool isEdgeOnRoute(const std::vector<std::string>& routeNodes,
                   const std::string& fromId,
                   const std::string& toId);

// 软件内时刻 timeMin 时，车辆恰好已到达的**最后一个节点下标**：
// 即最大的 i 使 plan.nodeArrivalMin[i] <= timeMin；若没有任何节点满足则返回 0。
//
// **这个函数的语义就是「半路显示所在段的起点」**：车在 i -> i+1 途中时，返回 i。
// 例：A -> B -> C 都已过、正驶向 D，则停在 C —— 显示的就是 C（段的起点），
// 而不是"还没到的 D"，也不是"已经离开的 B"。
//
// 边界：
//   - 早于首节点到达时刻      -> 0（车停在起点）
//   - 恰等于某节点到达时刻    -> 返回该节点下标（"恰好已到达"算作到了）
//   - 落在两节点之间          -> 返回**前一个**节点下标（段起点）
//   - 超过末节点到达时刻      -> 末节点下标
//   - plan.nodes 为空         -> 返回 0（下标无法有意义，**调用方需自行判空**）
// 下标一律 clamp 到 [0, nodes.size()-1]。
std::size_t nodeIndexAtTime(const RoutePlan& plan, int timeMin);

} // namespace logistics
