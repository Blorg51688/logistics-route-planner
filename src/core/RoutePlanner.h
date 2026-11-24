#pragma once

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
    // 单个订单的货量就超过载重上限——降载/兜底都救不了，这才是真正的载重不可行。
    // 注意：**总需求超过载重不再属于不可行**（规格一 #21 / #18）：单条连续路线
    // 边取边送，前缀载重才是约束，「总需求 ≤ 容量」这条旧判据已退役。
    OrderExceedsCapacity,
    Unreachable     // 存在无法到达的取货点/送达点，或无法返回仓库
};

// 一次送达动作（规格一 #18 Q14=A：**逐单窗口**、**逐单 penalty**）。
// 停靠不再有「合并窗口」这个概念：一个停靠只有一个到达时刻，但每张送达单
// 按自己的 [ws, we] 判迟到，penalty 逐单累加。
struct StopDelivery {
    std::string orderId;
    bool        late = false;
    int         penaltyMin = 0;
};

// 停靠所属阶段（规格一 #18 §4.1）。硬优先：紧急阶段的全部停靠都排在普通阶段之前。
enum class StopPhase {
    Urgent,   // 紧急阶段：紧急单的 A/B、顺路交货、降载卸货停靠、FIFO 强制重访
    Normal    // 普通阶段：普通单的 A/B、暂存取回停靠
};

// 一次停靠。**动作集是真源**：取货 / 送达 / 降载 / 取回都记在这里，
// GUI 的 onAdvanceStop 按动作集执行状态转移（规格一 MC-E09）。
//
// 停靠内执行顺序恒为「先全部卸（deliveries + offloads），再全部装（pickups + reclaims）」，
// 因此停靠内的载重峰值取「卸后 / 装后」较大者（#18 §3）。
struct Stop {
    std::string               nodeId;
    std::vector<StopDelivery> deliveries;   // 送达（含顺路交货）
    std::vector<std::string>  pickups;      // 取货（订单 ID）
    std::vector<std::string>  offloads;     // 降载卸到本中转站的订单 ID
    std::vector<std::string>  reclaims;     // 从本中转站取回的订单 ID
    StopPhase                 phase = StopPhase::Normal;
    int    rawArrivalMin = 0;               // 未经等待修正的到达时刻
    int    waitMin       = 0;               // 早到等待时长
    int    arrivalMin    = 0;               // 到达时刻 = raw + wait（停靠零耗时）
    // 停靠级聚合量：= deliveries 逐单之和。口径本身是**逐单**（#18 Q14），
    // 这里只是给界面/汇总用的便捷读法，不是「合并窗口」。
    bool   late       = false;
    int    penaltyMin = 0;
    // 语义（MC-A04）：**离开该站时的在车货量**
    // = 到达在车量 − 送达 − offload + 取货 + reclaim（#29 §2.1 第 3 条）。
    double remainingLoadKg = 0.0;

    // 本停靠是否只是中转站动作（降载 / 取回），不含取送动作。
    bool isTransitStop() const { return !offloads.empty() || !reclaims.empty(); }
    // 排序后的动作订单 ID（tieKey 的第二个分量，见 #18 §5）。
    std::vector<std::string> sortedActionOrderIds() const;
};

// 一次降载（规格一 #18 §11 / MC-A09）：把一张**整张普通订单**卸到中转站暂存，
// 事后由**同一辆车**取回继续配送。
//
// 与（将来的）转运账本**分型**：降载是「同车卸、同车取」，转运是「卸入车 ≠ 取走车」。
// 判据就是验收断言本身——同一台车卸了又取回，永远算降载，不算转运（#28 §2）。
struct OffloadLeg {
    std::string station;         // 暂存中转站 S
    std::string orderId;         // 被降载的订单
    std::string deliverNodeId;   // 该单送达点 B
    double      kg = 0.0;
    int         offloadTimeMin = 0;   // 卸下时刻
    int         reclaimTimeMin = -1;  // 取回时刻（-1 = 尚未取回；计划结束时必须已取回）
};

// 逐单指标（#29 §2.1 / ADR 0006）。取货准时判据：
//   t_pick + shortestTime_vehicle(A_i -> B_i) <= we_i
// 其中 shortestTime_vehicle 是**该车缩放后**的耗时（== 基础图 A->B 最短耗时 × time_factor）。
struct OrderMetric {
    std::string orderId;
    std::string nodeId;
    bool        urgent = false;
    int         pickupArrivalMin = 0;
    int         deliveryArrivalMin = 0;
    int         windowEndMin = 0;
    int         penaltyMin = 0;
    bool        late = false;
    bool        pickupOnTime = false;
    bool        deliveryOnTime = false;
};

// 摘要与验收用的可读指标（规格一 §6.1；--plan-summary 的四行即由此渲染）。
struct RouteMetrics {
    int stopCount = 0;    // 停靠点数 S（重访计两次；含降载/取回停靠）
    int actionCount = 0;  // 动作数 A = 取货动作数 + 送达动作数（结构性恒等 A = 2 × 取送对数）
    std::vector<OrderMetric> orders;   // 逐单 penalty / 准时
    int pickupOnTimeCount = 0;
    int pickupTotal = 0;
    int deliveryOnTimeCount = 0;
    int deliveryTotal = 0;
    int urgentOnTimeCount = 0;
    int urgentTotal = 0;
    int normalOnTimeCount = 0;
    int normalTotal = 0;
    // 时间加权在车货量：Σ_段(在车量 × 该段时长) / 总运行时长；段含首末空驶段。
    double avgLoadKg = 0.0;
    double loadUtilization = 0.0;   // avgLoadKg / 该车 capacity
    double peakLoadKg = 0.0;
};

struct RoutePlan {
    PlanStatus               status = PlanStatus::Ok;
    std::string              reason;      // 不可行原因
    std::vector<std::string> nodes;       // 完整序列：起点、途经节点、终点仓库
    // 与 nodes 一一对应的抵达时刻。界面的"推进"据此逐个节点前进。
    std::vector<int>         nodeArrivalMin;
    // 与 nodes 一一对应的标记：该节点是否为一次停靠
    // （同一节点可能在序列中出现多次，只有作为停靠点的那一次为 true）
    std::vector<bool>        nodeIsStop;
    // **真源**：按服务顺序的停靠（含动作集）。nodes 视图由它一次展平生成。
    std::vector<Stop>        stops;
    // 本计划的降载/取回记录（custody：订单、站、kg、卸下时刻、取回时刻）。
    std::vector<OffloadLeg>  offloads;
    double totalDistanceKm = 0.0;
    double totalTimeMin    = 0.0;         // 返回仓库时刻 − 起始时刻（含等待）
    // 抵达起始仓库的时刻（路线汇总字段之一，供界面与测试核对配送结束时间）。
    int    returnArrivalMin = 0;
    double totalCostYuan   = 0.0;
    int    totalPenaltyMin = 0;
    RouteMetrics metrics;
};

// 从车辆起始仓库出发、按其发车时刻规划，服务完全部订单后返回该仓库。
RoutePlan planRoute(const LogisticsGraph& graph,
                    const Vehicle& vehicle,
                    const std::vector<Order>& orders,
                    WeightType weight);

// ---- 类型感知分配（规格二 §4 / #40）：日志与度量类型 ----

// 一条被接受的跨车改派（阶段三日志）。**不变量 8 直接读它**：
// completionAfterMin 必须 ≤ completionBeforeMin（完工耗时不变差）。
struct RelocateRecord {
    int         round = 0;              // 1..3
    std::string orderId;
    std::string fromVehicleId;
    std::string toVehicleId;
    int         penaltyBefore = 0;      // 改派前后**车队键**（三层）
    int         penaltyAfter = 0;
    double      weightBefore = 0.0;
    double      weightAfter = 0.0;
    double      balanceBefore = 0.0;    // 均衡键 = 峰值利用率极差
    double      balanceAfter = 0.0;
    double      completionBeforeMin = 0.0;   // 改派前后**完工耗时**
    double      completionAfterMin = 0.0;
};

// 边际代价（规格二 §4.7 / #27 的接口）：key(在该车插入后) − key(该车现状)。
// 三元组按字典序比较；**必须用候选车自己的因子重放**（候选计划由 planRoute 以
// 该车的 time_factor / cost_factor / capacity 重建）。
// currentVehicleOrders = 该车当前计划所服务的订单（顺序无关，构造器内部按 EDD 排）；
// currentPlan = 该车当前计划（提供「现状」侧的三项）。
// balanceDelta = 该车峰值利用率的变化（本车容量口径）；跨车的极差变化只有拿着
// 整队计划才算得出，这里如实只给本车分量（#27 用它排序候选车）。
struct MarginalCost {
    double penaltyDelta = 0.0;
    double weightDelta = 0.0;
    double balanceDelta = 0.0;
};

// ---- 车队（规格二 §3 / #39）----
//
// 每车一条**连续路线**（自己的 start_node / depart_time 空载出发，回同一基地）。
// 分配有**两个入口**（规格二 §4）：
//   · `planFleet`（默认）= 类型感知三阶段：聚类 -> 整簇指派（全排列枚举）-> 有界改派；
//   · `planFleetSequentialBaseline` = 顺序轮转基线（§4.5），只作对照与确定性守卫，**不要删**。
struct FleetPlan {
    // 与传入 vehicles 下标**一一对应**（失败的车上也留一个占位计划，便于定长对齐）。
    std::vector<RoutePlan> vehicles;
    // 车队汇总（规格二 §3.4）：
    double totalDistanceKm = 0.0;      // Σ 每车距离
    double totalCostYuan = 0.0;        // Σ 每车成本
    int    totalPenaltyMin = 0;        // Σ 每车 penalty
    int    stopCount = 0;              // Σ 每车停靠数
    int    actionCount = 0;            // Σ 每车动作数
    // **总耗时主口径 = 完工耗时** = max(各车返回时刻) − min(各车发车时刻)；
    // 返回时刻按实值（发车 + 该车耗时）计；空车返回 = 发车，不抬升完工耗时，
    // 但参与 min(发车时刻)。整数 returnArrivalMin 只是它的显示取整。
    double completionTimeMin = 0.0;
    // 参考口径 = Σ 每车耗时（含早到等待）。与完工耗时**并列**，不得互相顶替。
    double sumVehicleTimeMin = 0.0;
    bool   allOk = true;
    std::string reason;
    // 车队级聚合指标（§29 的 Σ/Σ 口径）：
    //   两率 = Σ命中 / Σ分母；平均在车 / 利用率按**时间加权**跨车积分；
    //   峰值 = max(各车峰值)。
    RouteMetrics metrics;

    // ---- 类型感知分配的产物（规格二 §4 / #40）----
    //
    // 均衡度（§4.3 / Q5 定稿）= max_v(峰值在车_v ÷ capacity_v) − min_v(…)。
    // **分母口径按「全部车」**：空车 utilization 记 0 并参与极差（否则「一台车干活、
    // 两台空转」会被算成最均衡）。地位 = 决策键第三级 + 报告指标，**不作硬约束**。
    // 由 aggregateFleet 统一算（车队汇总只有这一处实现），GUI 的均衡度行也读它。
    double balanceSpread = 0.0;

    // 分配表：订单 ID -> 车辆**行下标**。按订单 ID 升序排列（与 --dump-plan 的
    // ALLOC 行同序）；allocationClusterId 同下标：>=1 = 该单所属本地簇；-1 = 跨簇单
    // （跨簇单不属任何簇，见 §4.2）。
    std::vector<std::string> allocationOrderIds;
    std::vector<std::size_t> allocationVehicleIndex;
    std::vector<int>         allocationClusterId;

    // 每车分到的订单数 / 本地簇编号（升序去重）/ 跨簇单数（按车辆行下标）。
    std::vector<int>               vehicleOrderCount;
    std::vector<std::vector<int> > vehicleClusters;
    std::vector<int>               vehicleCrossClusterCount;

    // 阶段三被接受的跨车改派日志（不变量 8：逐条核对「完工耗时不变差」）。
    std::vector<RelocateRecord> relocations;

    // 分配过程的文字日志（规格二 §4.6：拆簇「仅此级触发并留日志」；子网络 0 的
    // 异常数据归跨簇队列也记这里；被拒绝的订单点名原因）。
    std::vector<std::string> allocationLog;

    // 阶段二的枚举留痕：每种「簇 -> 车」排列一行，供人工核对「6 种排列各自怎么取优」。
    std::vector<std::string> candidateKeyLog;
};

// 类型感知三阶段分配（规格二 §4，**默认入口**）。
//   阶段一 聚类：簇 = 取货点 A 的 subNetworkId；A、B 跨网 ⇒ 跨簇单（单独成队列）；
//                  subNetworkId == 0 的异常数据也并入跨簇队列并记日志（不静默丢单）。
//   阶段二 整簇指派：枚举全部「簇 -> 车」排列（默认 3 簇 3 车 = 6 种，**手写全排列**），
//                  每簇的订单交给该车跑**规格一的单车构造器**；候选按车队键
//                  (Σpenalty, Σ策略权重, 均衡键, tieKey) 取优。簇数 > 车数时按簇货量
//                  降序取前 k 个各占一车、其余并入「货量最接近且仍可行」的车。
//   阶段三 有界跨车改派：R ≤ 3 轮、只普通段、EDD 序；两条接受条件**同时**成立才接受
//                  （① 车队键严格变优；② 完工耗时不得变差）。一轮无改进即停。
// 失败兜底链（§4.6）：换车（枚举覆盖）-> 拆簇（仅此级触发并记 allocationLog）->
// 规格一的降载/全量兜底（在 planRoute 内部）-> 拒绝该单并报原因。
FleetPlan planFleet(const LogisticsGraph& graph,
                    const std::vector<Vehicle>& vehicles,
                    const std::vector<Order>& orders,
                    WeightType weight);

// 顺序轮转基线（规格二 §4.5，**对照组不是候选方案**）：
// 订单 ID 升序、第 i 单 -> 第 (i mod k) 台车、车辆序 = [vehicles] 行序；
// 不跳车、不重分配、不做可行性预判；分完后各车独立跑 planRoute。
// 分配只由「订单 ID 升序 + 车队行序」决定，与策略/容量/窗口**全部无关**（不变量 9）。
FleetPlan planFleetSequentialBaseline(const LogisticsGraph& graph,
                                      const std::vector<Vehicle>& vehicles,
                                      const std::vector<Order>& orders,
                                      WeightType weight);

// ---- 阶段一的两个判据（规格二 §4.2；#27 也直接用）----
// 簇(order) = node(order.pickupNodeId).subNetworkId；取货点不在图上返回 -1。
int clusterOfOrder(const LogisticsGraph& graph, const Order& order);
// isCrossCluster(order) = 簇(A) ≠ 簇(B)；任一端不在图上返回 false（不可达另由拒绝链报）。
bool isCrossCluster(const LogisticsGraph& graph, const Order& order);

// 边际代价（规格二 §4.7 / #27 的接口）：key(在该车插入后) − key(该车现状)。
MarginalCost marginalCost(const LogisticsGraph& graph, const Vehicle& vehicle,
                          const Order& order, const std::vector<Order>& currentVehicleOrders,
                          const RoutePlan& currentPlan, WeightType weight);

// 把**已经算好的**逐车计划聚合成车队汇总（规格二 §3.4 的可加口径）。
// planFleet 用它，GUI 的中途重规划也用它 —— 汇总口径只有这一处实现。
// plans 与 vehicles 必须等长且按下标对应。
FleetPlan aggregateFleet(const std::vector<Vehicle>& vehicles,
                         const std::vector<RoutePlan>& plans);

// 车辆**此刻已经载在车上**的货（订单 + 目的 B + 货量 + 订单号）。
// 规格一 MC-A05：按**订单**记账（同节点多单下按 nodeId 扣货会把别的单一起扣掉）。
struct OnboardItem {
    std::string nodeId;   // 目的送达点 B
    double      kg = 0.0;
    std::string orderId;  // 该货所属订单（空 = 旧数据/未知来源）
};

// 中转站上的暂存货（降载后仍留在站里、待取回的普通货）。
// station 为空 = 无暂存。跨重规划时由调用方（GUI）传入，规划器据此在计划里
// 保留「回站取回并继续配送」，避免暂存货在重规划后凭空消失。
struct StashState {
    std::string              station;
    std::vector<OnboardItem> items;
};

// 从指定位置与指定时刻出发，对剩余未服务订单重新规划，最后返回车辆起始仓库。
RoutePlan replan(const LogisticsGraph& graph,
                 const Vehicle& vehicle,
                 const std::vector<Order>& remainingOrders,
                 const std::string& currentPositionId,
                 int currentTimeMin,
                 WeightType weight,
                 const std::vector<OnboardItem>& onboard = std::vector<OnboardItem>(),
                 const StashState& stash = StashState());

struct InsertResult {
    RoutePlan   plan;
    std::string warning;   // 空表示无冲突
};

// 动态插入一张订单（规格一 MC-B04：通用插单，取代只服务紧急单的 insertUrgentOrder）。
//
// **走哪条链由订单自己的 urgent 标记决定**（用参数区分，不再复制两个函数）：
//   · urgent == true  -> 紧急段**尾插**（订单级 FIFO，#18 §7；后来的紧急单排在后面）；
//   · urgent == false -> 普通段 **pair-insertion 链**（#18 §8：试插取送对 -> repair(≤3)
//                        -> 全量兜底 -> 降载）。
// 调用方负责把来源（OrderSource）与 ID 前缀带进 newOrder；本函数只读 urgent。
// 以车辆当前位置、当前时刻为起点重算路线。
// 冲突判定：若新订单窗口止早于"从当前位置出发的最早到达时刻"（按该车缩放后的
// 耗时维度计算），则无论怎么排都会超时，此时在 warning 中给出提示。
// 按 D13（超时不弃、只记 penalty），订单**仍会被纳入规划**。
InsertResult insertOrder(const LogisticsGraph& graph,
                          const Vehicle& vehicle,
                          const std::vector<Order>& remainingOrders,
                          const Order& newOrder,
                          const std::string& currentPositionId,
                          int currentTimeMin,
                          WeightType weight,
                          const std::vector<OnboardItem>& onboard
                              = std::vector<OnboardItem>(),
                          const StashState& stash = StashState());

// 从已规划好的单条路线中切出"尚未走完的部分"（规格一 MC-A01/A10）：
// 节点序列从 fromNodeIndex 起，停靠只保留该下标及其之后的。
// 供增量式重规划使用；趟结构已退役，因此这里**不再**保留任何趟边界。
RoutePlan sliceRemainder(const RoutePlan& plan, std::size_t fromNodeIndex);

// 增量式重规划（规格二 §10 裁定 1 / ADR 0007）：**顺序不变版重放**。
//
// 停靠顺序完全不变，只把剩余停靠序列按**当前边权重**重新放一遍
// （任一 leg 不可达 ⇒ 该版不可行）；可行则采用重放结果并把 usedIncremental 置 true。
// 重放不可行时退回全量重规划（usedIncremental = false）。
//
// 如实说明（裁定原文）：**不重写 A→B 版 leg 级增量**。旧 replanIncremental 对新模型
// 恒退全量，那条链没有实现载体；改由「重放 + 有界重排」承担局部性。
// usedIncremental 的语义因此改为「本次是否用了顺序不变版重放」，而不是
// 「是否只重算了受影响的 leg」。
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
                            // 回报本次是否用了「顺序不变版重放」（false = 退回全量）
                            bool* usedIncremental = nullptr,
                            // 与 replan 一致：跨重规划保住中转站暂存货
                            const StashState& stash = StashState());

// 判断某条有向边是否落在给定路线序列的**相邻两站**之间。
// GUI 的路径高亮与路况重规划的触发判定共用这一条逻辑。
bool isEdgeOnRoute(const std::vector<std::string>& routeNodes,
                   const std::string& fromId,
                   const std::string& toId);

// 软件内时刻 timeMin 时，车辆恰好已到达的**最后一个节点下标**：
// 即最大的 i 使 plan.nodeArrivalMin[i] <= timeMin；若没有任何节点满足则返回 0。
//
// **这个函数的语义就是「半路显示所在段的起点」**：车在 i -> i+1 途中时，返回 i。
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