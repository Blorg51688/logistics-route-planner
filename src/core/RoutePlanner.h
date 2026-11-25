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

// 一次停靠。**动作集是真源**：取货 / 送达 / 降载 / 取回 / 转运卸入 / 转运取走都记在这里，
// GUI 的 onAdvanceStop 按动作集执行状态转移（规格一 MC-E09）。
//
// 停靠内执行顺序恒为「先全部卸（deliveries + offloads + transitUnloads），
// 再全部装（pickups + reclaims + transitPickups）」，
// 因此停靠内的载重峰值取「卸后 / 装后」较大者（#18 §3）。
struct Stop {
    std::string               nodeId;
    std::vector<StopDelivery> deliveries;   // 送达（含顺路交货）
    std::vector<std::string>  pickups;      // 取货（订单 ID）
    std::vector<std::string>  offloads;     // 降载卸到本中转站的订单 ID（同车取回）
    std::vector<std::string>  reclaims;     // 从本中转站取回的订单 ID（降载取回）
    // 跨车转运（规格二 §5 / #28）：**转运卸入**与**转运取走**是两种不同的动作。
    // 降载与转运的分界就是「取走车 == 卸入车？」：
    //   · offloads + reclaims ⇒ 同一台车（降载，OffloadLeg）；
    //   · transitUnloads + 另一台车的 transitPickups ⇒ 转运（TransitLeg，inVehicle ≠ outVehicle）。
    // 站停靠无窗口、不计 penalty、零耗时（departure = arrival），并计入停靠点数。
    std::vector<std::string>  transitUnloads;  // 本车在此站整单卸下、交给另一台车
    std::vector<std::string>  transitPickups;  // 本车在此站整单取走（由另一台车卸入）
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

    // 本停靠是否只是中转站动作（降载 / 取回 / 转运卸入 / 转运取走），不含取送动作。
    bool isTransitStop() const {
        return !offloads.empty() || !reclaims.empty() || !transitUnloads.empty()
               || !transitPickups.empty();
    }
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

// 站内等待上限（规格二 §5.5 / #28 Q6=A）：**代码常量，不加配置键**。
// outTimeMin − inTimeMin > kTransferMaxDwellMin ⇒ 该转运候选不可行（构造期过滤）。
// 为什么用可行性而不是指标管住等待：站无窗口、停靠零耗时 ⇒ 货在站里等多久不产生
// 耗时、不产生 penalty；若不管，转运可以把等待搬进站里让完工耗时变好看，而货实际更晚到。
constexpr int kTransferMaxDwellMin = 60;

// 有界前瞻窗口（规格三 §3 / #48）：**代码常量，不加配置键**（同 kTransferMaxDwellMin）。
//
// 语义：车辆可以把「尚未就绪、但可接时刻 readyTime 不晚于 预计到达时刻 + W」的腿纳入
// 当前路线；到达后若货还没到就原地等到 readyTime（车等货）。可接时刻落在窗口之外的腿
// **不纳入**——否则车辆会为了一小时后的货物在站里空等，行程荒谬。
//
// 为什么有界：readyTime 是精确值（前驱腿的计划完成时刻），但「精确」不等于「值得等」。
// 窗口就是「值得等多久」这一条唯一的门槛，且它同时是可行性判据（超窗即该候选不可行），
// 于是有界性由构造保证，不靠事后抽查。反向：窗口=0 退化成就绪才出发（反应式），
// 接力链会因「货未到 → 不可行」整条退回直达；窗口=无界则空等任意久。
//
// 与 kTransferMaxDwellMin 的分工：W 管「车等货」（货在**未来**），dwell 管「货等车」
// （货在**过去**，货物在站停留上限）。两者都在 replay 的同一处按 (订单, 节点) 判。
constexpr int kLookaheadWindowMin = 60;

// 转运账本的最小记录单位（规格二 §5.3 / #28 §5.1）。**两端都必填**：
// 没有「只卸不取」的合法记录；无 outVehicleId 的候选在构造期就被丢弃。
// 与 OffloadLeg 的分型判据：inVehicleId ≠ outVehicleId（同车卸又取回永远算降载）。
struct TransitLeg {
    std::string orderId;      // 整单转运：kg 即该单全部需求
    std::string stationId;    // 中转站
    double      kg = 0.0;     // = 订单需求
    std::string inVehicleId;  // 卸入车（≠ outVehicleId）
    int         inTimeMin = 0;    // 卸入时刻（软件内时间）
    std::string outVehicleId; // 取走车
    int         outTimeMin = 0;   // 取走时刻
};

// ---- 腿（Leg）与就绪链（规格三 §1 / #45）----
//
// 规格三把「订单 → 一台车」改写成「订单展开成带先后依赖的腿」：
//   跨簇单 A→B（簇(A)=c1 ≠ 簇(B)=c2）⇒ 三段腿 L1: A→S(c1) / L2: S(c1)→S(c2) / L3: S(c2)→B；
//   本地单 A→B ⇒ 一条腿 A→B。
//
// **本票（#45）只立结构与就绪链**：腿的执行车是临时指派、规划上**不执行** L2/L3，
// 实际路线仍由既有分配/转运链决定（接力与等待留给 #48），因此既有路线与黄金值逐字不变。
enum class LegKind {
    CustomerToCustomer,   // 本地单：客户 → 客户（可途经本簇站）
    CustomerToStation,    // 跨簇 L1：客户 → 本簇中转站
    StationToStation,     // 跨簇 L2：本簇站 → 对簇站（#46 起只能由快车执行）
    StationToCustomer     // 跨簇 L3：对簇站 → 客户
};

// 腿类型的文本写法：--dump-plan 的 LEG 行与日志共用，取值与设计文档的词汇表一致。
const char* toString(LegKind kind);

// 一条腿。**结构层真源**：--dump-plan 的 LEG 行、GUI 的腿登记都读它。
struct Leg {
    int         seq = 1;              // 腿序：本地单恒 1；跨簇单 1 → 2 → 3
    std::string from;                 // 起点（客户或中转站）
    std::string to;                   // 止点
    LegKind     kind = LegKind::CustomerToCustomer;
    std::string orderId;              // 所属订单（站间腿也带，跨簇单 3 条腿同单号）
    // 可接时刻。链式定义：L1 = 执行车发车时刻；L2 = L1 的计划完成时刻；
    // L3 = L2 的计划完成时刻。**由前驱腿的计划时刻反推**，不是常量、不是估计值
    // （怎么算见 buildLegs 的注释）。单调不减因此是构造性的，不是抽查出来的。
    int         readyTimeMin = 0;
    // 该腿的计划完成时刻（到达 to）。本票的算法 = 「readyTime 起步 + 该车因子缩放的
    // 该腿最短路耗时」；#48 用接力实际执行时刻替换它，字段语义不变。
    int         plannedArrivalMin = 0;
    std::string vehicleId;            // 执行车（由职责表定，见 legVehicleEligible）
    // true = 执行车是**规划期估计**的落点（接力没跑，时刻是估计值）；
    // false = 接力真的跑了，执行车与时刻都是实际值（#48 的 readRelayExecution 覆盖）。
    bool        temporaryAssignment = true;

    // ---- 职责表的判据输入（规格三 §1 / #46）----
    //
    // 「跨簇紧急单直达」是快车唯一能接客户↔客户腿的通道，判据是**该单**的直达 penalty
    // 严格小于 relay penalty。Leg 必须自带这两个数，否则职责谓词只能看 kind，
    // 「紧急直达」这条通道就无从表达（谓词拿不到订单表）。
    //
    // 两个 penalty 都是**规划期估计**（图 + 各车 time_factor 的最短路），不是执行值：
    // 职责表要在规划**之前**回答「谁可以接这条腿」，那时还没有执行时刻。
    // 估计口径见 RoutePlanner.cpp 的 urgentDirectPenaltyPair。
    bool        urgent = false;              // 所属订单是否紧急
    int         directPenaltyMin = -1;       // 直达（快车 A→B）估计 penalty；-1 = 不适用
    int         relayPenaltyMin = -1;        // 三段接力估计 penalty；-1 = 不适用
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
    // 转运动作的 kg（订单 -> 需求），账本渲染用；由 finalize 从 Ctx 填。
    // 与 Stop::transitUnloads / transitPickups 一一对应。
    std::map<std::string, double> transitKg;
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

    // ---- 跨车转运（规格二 §5 / #28）----
    //
    // 账本：每笔**两端齐全**的转运记录（按卸入时刻、订单 ID 升序）。
    // 只记规划产物；「已实际执行」的过滤由 GUI 运行态账本负责（§7.5⑥）。
    // 默认数据上三条判据全部不触发 ⇒ 恒为空（§5.13，这是「绝不更差」的最强证据）。
    std::vector<TransitLeg> transitLegs;
    // 转运分级播报（判据 / 轮次 / 接受或退回及原因）。GUI 直接追加到运行日志；
    // **不进 --plan-summary**（六行格式不动）。
    std::vector<std::string> transitLog;

    // ---- 腿清单（规格三 §1 / #45：结构层）----
    //
    // 订单展开成的腿，**按订单 ID 升序、同单内按腿序 1 -> 2 -> 3**（--dump-plan 的 LEG 行同序）。
    // 默认 25 单全簇内 => 25 条 customer->customer 单腿；只有跨簇单才出现 3 条腿。
    std::vector<Leg> legs;
    // 腿结构的登记日志（「结构已立、接力未启用」+ 临时指派 + 展开退化 + 缺站/无车）。
    // **不进 --plan-summary**；GUI 追加到运行日志，--dump-plan 不打印。
    std::vector<std::string> legLog;
};

// 职责封闭的簇 -> 慢车指派（规格三 §4 / #46，**默认入口**）。
//   阶段一 聚类：簇 = 取货点 A 的 subNetworkId；A、B 跨网 ⇒ 跨簇单（单独成队列）；
//                  subNetworkId == 0 的异常数据也并入跨簇队列并记日志（不静默丢单）。
//   阶段二 固定指派：慢车按 **ID 升序**、簇按 **ID 升序**逐位配对（V02→簇1、V03→簇2、
//                  V04→簇3），每簇的订单交给它的本簇慢车跑**规格一的单车构造器**；
//                  快车只拿「站间腿」与「跨簇紧急直达」——它不进本簇的订单队列。
//                  **没有枚举**：职责封闭后「簇 -> 慢车」的自由度是 1，全排列枚举
//                  （规格二阶段二，3 簇 3 车 = 6 种）已退役，理由见 planFleet 的注释。
//                  簇数 > 慢车数时按簇 ID 升序**循环**配对（一台慢车守多簇）并记日志；
//                  慢车数 > 簇数时多出来的慢车空转（不强行派活）。
//   阶段三 **退役**：有界跨车改派（R ≤ 3 轮、RelocateRecord）在职责封闭后没有改派空间
//                  —— 一张簇内单只能由本簇慢车接，挪给别的车就越界。relocations 恒空。
// 跨簇单的处置：普通单走三段接力（#48 的 applyRelayLayer，L1/L2/L3 分别由
//   本簇慢车 / 快车 / 对簇慢车执行）；紧急单保留「快车直达」通道，判据是
//   该单直达 penalty **严格小于** relay penalty（见 legVehicleEligible）。
// 失败兜底链（口径按职责封闭收紧，见 #46）：职责封闭后「换车」与「把半簇挪给别的车」
//   都会越界，因此那两级随枚举一起退役，只剩「规格一的降载/全量兜底（planRoute 内部）
//   -> 拒绝该单并点名」；跨簇单另有「接力不可行 ⇒ 职责降级直达（大声记日志）或拒绝」。
FleetPlan planFleet(const LogisticsGraph& graph,
                    const std::vector<Vehicle>& vehicles,
                    const std::vector<Order>& orders,
                    WeightType weight);

// 订单 -> 腿的展开（规格三 §1 / #45）。读 fleet 的分配表定执行车，产出按「订单 ID 升序、
// 同单内腿序 1 -> 2 -> 3」排序的腿清单；「结构已立、接力未启用」「执行车是临时指派」
// 「缺中转站 / 端点不在图上导致退化」如实写进 log，**不声称已转运**。
// 规划上不执行 L2/L3：本函数只登记结构，**不改任何路线**。
std::vector<Leg> buildLegs(const LogisticsGraph& graph,
                           const std::vector<Vehicle>& vehicles,
                           const std::vector<Order>& orders,
                           const FleetPlan& fleet,
                           std::vector<std::string>& log);

// ---- 腿的执行车指派：「谁可以接哪条腿」的**唯一决定点**（规格三 §1 / #46）----
//
// 职责表（**可执行判据**，逐字即实现）：
//   · **慢车**只能接「两端都在本簇」且**非** station→station 的腿；
//   · **快车**只能接 station→station 腿，或「跨簇**紧急**单且该单 直达 penalty
//     **严格小于** relay penalty」的 customer→customer 直达腿。
//
// 三个分类都是**从数据推出来的**，不硬编码车号：
//   · 快车 = 站间腿最合适的那台（容量最大 ⇒ 成本因子更高 ⇒ 车辆 ID 小，与接力层同判据）；
//   · 慢车 = 其余车辆，按**车辆 ID 升序**；
//   · 簇 = 有配送点的非 0 子网，按**簇 ID 升序**；慢车与簇逐位配对
//     （⇒ 默认 3 簇 3 慢车时排列数 = 1，#46 因此退役整簇全排列枚举）。
//
// 为什么签名要带整个车队（而不是只看单台车 + 腿）：职责是**车队的划分**，
// 「这台车是慢车还是快车」「本簇的慢车是哪一台」都只有拿着整队才回答得了。
// 谓词仍然是唯一决定点：chooseLegVehicle 与腿的展开逻辑都只调它。
bool legVehicleEligible(const LogisticsGraph& graph, const std::vector<Vehicle>& vehicles,
                        std::size_t vehicleIndex, const Leg& leg);

// 在合资格的车里挑执行车（职责表收窄后的**确定性落点**）：
// 职责封闭后每类腿的合资格车**只有一个**（慢车腿 = 本簇慢车；站间腿 = 快车）。
// 若出现多台合资格，只可能是职责表被放宽了 —— 那时按「车辆行序」取第一台，
// 让越界立刻出现在 LEG 行里（而不是挑一台看起来合理的车掩盖过去）。
// 站间腿例外：用 betterForStationLeg（与接力层同判据），保证 LEG 与真跑的车一致。
// 合资格车为空时返回空串（调用方记日志，**不伪造**执行车）。
std::string chooseLegVehicle(const LogisticsGraph& graph, const std::vector<Vehicle>& vehicles,
                             const Leg& leg);

// 「哪台是快车」的**公开只读**判定（GUI 按角色取色/画环用，规格三 GUI §7）：
// 返回与 vehicles 同长的 bool 向量，true = 快车（站↔站 + 跨簇紧急单直达）。
// 判据与 legVehicleEligible / 接力层**同一份**（relayVehicleIndex → betterForStationLeg）：
// 容量最大 ⇒ 成本因子更高 ⇒ 车辆 ID 小；只有一台车时它就是快车。
// 为什么公开：GUI 不能自己再推一遍角色 —— 口径一漂，画布颜色与 LEG 行就会互相矛盾。
std::vector<bool> fastVehicleFlags(const std::vector<Vehicle>& vehicles);

// 顺序轮转基线（规格二 §4.5，**对照组不是候选方案**）：
// 订单 ID 升序、第 i 单 -> 第 (i mod k) 台车、车辆序 = [vehicles] 行序；
// 不跳车、不重分配、不做可行性预判；分完后各车独立跑 planRoute。
// 分配只由「订单 ID 升序 + 车队行序」决定，与策略/容量/窗口**全部无关**（不变量 9）。
// 腿清单的产出口径与 planFleet **相同**（buildLegs 读它自己的分配表）：
// 对照组也要能回答「每条腿归谁、readyTime 多少」，否则两条入口的结构会漂。
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

// ---- 跨车转运（规格二 §5 / #28）----
//
// 从各车计划里**反查**转运账本：扫描 Stop::transitUnloads / transitPickups，
// 按订单把卸入车与取走车配对，读出两端时刻与 kg。构造上保证两端齐全；
// 若出现只有一端（不该发生）则跳过并**不伪造**另一端。按 (inTimeMin, orderId) 升序。
// 这是账本口径的**唯一**实现（--dump-plan、GUI 面板、测试都读它）。
std::vector<TransitLeg> buildTransitLedger(const std::vector<Vehicle>& vehicles,
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

// 一车的转运候选构造起点（静态规划 = 仓库空车；中途重规划 = 当前位置/在车货/暂存）。
struct VehicleStartState {
    std::string              startNodeId;
    int                      startTimeMin = 0;
    std::vector<OnboardItem> onboard;
    StashState               stash;
};

struct FleetTransitResult {
    std::vector<RoutePlan>   plans;      // 未启用时 == basePlans（同一份内容）
    std::vector<TransitLeg>  legs;       // buildTransitLedger 的结果
    std::vector<std::string> log;        // 分级播报
    int acceptedRounds = 0;              // 被接受的候选总数（≤ 3 判据 × 3 轮）
};

// 跨车整单转运层（规格二 §5 / #28）——**后置的有界改进**，不改分配、不改构造器。
//
// 输入 = 禁转运版（basePlans，由规格一构造 + #40 分配得到）+ 每车的构造起点。
// 判据固定顺序：① 跨簇 ② 容量腾挪（禁转运版里有降载） ③ 紧急响应（紧急单 penalty>0）。
// 每类最多 R = 3 轮、每轮至多接受一个 key 最优候选；**严格字典序更优**才替换，
// 完全并列保留禁转运版（关键：本图 84.2% 点对存在精确零绕行站 + 停靠零耗时，
// 用「不更差即启用」会把默认数据改成到处零收益转运）。
// 「绝不更差」是构造保证：plan 只会被逐轮严格变优替换。
FleetTransitResult applyTransitLayer(const LogisticsGraph& graph,
                                     const std::vector<Vehicle>& vehicles,
                                     const std::vector<std::vector<Order> >& vehicleOrders,
                                     const std::vector<RoutePlan>& basePlans,
                                     const std::vector<VehicleStartState>& starts,
                                     WeightType weight);

// 从指定位置与指定时刻出发，对剩余未服务订单重新规划，最后返回车辆起始仓库。
//
// allowOffload（规格二 §6.4 / #27）：是否允许构造器在最后手段里**降载**。
//   · 默认 true —— 规格一的整条构造链（含降载）原样不变；
//   · 复调度的第 ①②③ 级传 false：降载属于第 ④ 级的「改变货物路径的重手段」，
//     不在局部/再分配/全车队三级的射程内（否则第 ④ 级永不可达，分级就退化成摆设）。
RoutePlan replan(const LogisticsGraph& graph,
                 const Vehicle& vehicle,
                 const std::vector<Order>& remainingOrders,
                 const std::string& currentPositionId,
                 int currentTimeMin,
                 WeightType weight,
                 const std::vector<OnboardItem>& onboard = std::vector<OnboardItem>(),
                 const StashState& stash = StashState(),
                 bool allowOffload = true);

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

// ===========================================================================
// 动态事件复调度（规格二 §6 / #27）
// ===========================================================================
//
// 分工：**本层只做「计划」**，不碰任何已发生的物理事实。调用方（GUI）负责：
//   · 用 VehicleState 提供「复调度的输入常量」（§6.2：位置/时刻/在车货/已执行账本）；
//   · 把本层产出的计划写回该车游标（nodeIndex/stopCursor 归零），并**只**对被重算的车写回
//     （未受影响车的路线逐字不变，不变量 5）。
// 本层属核心层：不得出现任何 Qt 类型。

// 触发复调度的事件种类。只列**会改变规划输入**的那几类。
enum class RescheduleTrigger {
    Traffic,      // 路况：改动一批边的权重（带 increaseRatio）
    RoadClosure,  // 封路：删掉一批边（双向各一条），图已先被改过
    NewOrder,     // 新订单（普通/紧急/手动三者共用 §6.6 的选车规则）
    Manual        // 手动「重新规划」：全车队都算受影响
};

struct RescheduleEvent {
    RescheduleTrigger trigger = RescheduleTrigger::Manual;
    // Traffic / RoadClosure：本次被改变的**有向**边（封路时两个方向各一条）。
    std::vector<TrafficChange> changes;
    // Traffic：增幅门槛（config 的 traffic_time_increase_min）。低于它不算「受影响」。
    double thresholdRatio = 0.0;
    // NewOrder：新插入的订单（已并入调用方的订单集合）。
    Order newOrder;
};

// 站内**已执行**的转运卸入（§6.9）：货已经在站里，只等某台车取走。
// 它是复调度的一等输入：不得凭空消失（期末 0 是硬门槛）。
struct ExecutedTransitEntry {
    TransitLeg leg;    // 站、kg、卸入车、卸入时刻（out 端此刻通常还没发生）
    Order      order;  // 订单本体（A/B/窗口/kg）
};

// 复调度的输入常量（§6.2：已执行前缀不重排）。
struct FleetRescheduleInput {
    // 每车当前位置/时刻/在车货/暂存（StashState 归**原车**）。
    std::vector<VehicleStartState> starts;
    // 每车当前计划所服务的**未送达**订单（顺序无关；构造器内部按 EDD/FIFO 排）。
    std::vector<std::vector<Order> > vehicleOrders;
    // 每车**剩余**路线的上一版计划（R1 的重放来源；由 sliceRemainder 切出）。
    std::vector<RoutePlan> remainingPlans;
    // 全队未送达订单（第 ③ 级「全车队重调度」的输入）。
    std::vector<Order> allRemainingOrders;
    // 站内已执行、尚未被取走的转运卸入（§6.9：任一可行车可取走；dwell ≤ 60）。
    std::vector<ExecutedTransitEntry> executedInStation;
};

// ---- 规格三 #49：custody（货物保管）守恒的「读计划产物」审计 ----
//
// 这是 US22/US29「期末 0」的**真断言**：只读 RoutePlan::stops 里的
// transitUnloads / transitPickups / deliveries（即 --dump-plan 的 STOP/TRANSIT 行给
// 用户看到的东西），**不重算任何公式**。它回答三个问题：
//   · 逐单：每一笔「卸入」(进站) 是否都有一笔「同站、另一台车、时刻不早于它」的取走？
//           （「有卸入、无同站取走」= 货物跨站却无车承运 —— 封路打断接力链的病灶）
//   · kg：整单不可拆（C1）⇒ 每次易手都是该单全部需求，Σ卸入 kg 必须 == Σ取走 kg。
//   · 期末：每单要么在计划里送达、要么在 rejected 名单里被**显式拒绝**（有原因）。
struct CustodyGap {
    std::string orderId;    // 涉事订单
    std::string stationId;  // 涉事中转站（undelivered-not-rejected 时为空）
    std::string kind;       // unload-no-pickup / pickup-no-unload / kg-mismatch / undelivered-not-rejected
    std::string detail;     // 可读说明（点名订单 / 站 / 车），供日志与守卫摘录
};
struct CustodyReport {
    bool ok = true;                 // 无任何缺口
    std::vector<CustodyGap> gaps;   // 逐条缺口（按订单表顺序 + 事件顺序，确定性）
};

// 读计划产物逐单核 custody 守恒。rejectedOrderIds = 被显式拒绝的订单（有原因）；
// 计划里出现、但既未送达也不在拒绝名单里的单，记为 undelivered-not-rejected。
CustodyReport auditCustody(const std::vector<Vehicle>& vehicles,
                           const std::vector<RoutePlan>& plans,
                           const std::vector<Order>& orders,
                           const std::vector<std::string>& rejectedOrderIds);

// 局部重规划的两版结果（§6.3）。
struct LocalReplanOutcome {
    RoutePlan chosen;            // 最终采用的那一版（都不可行时 status != Ok）
    RoutePlan byReplay;          // R1 顺序不变版（重放）
    RoutePlan byRebuild;         // R2 重排版（规格一构造器）
    bool replayFeasible = false;
    bool rebuildFeasible = false;
    bool adoptedRebuild = false; // 采用 R2（当且仅当 R2 可行且 key(R2) < key(R1)，严格更优）
    bool feasible = false;       // chosen 可用
    int    penaltyReplay = 0;
    int    penaltyRebuild = 0;
    double weightReplay = 0.0;
    double weightRebuild = 0.0;
};

// §6.3：R1 = 停靠顺序完全不变、只按新边权重重放；R2 = 剩余订单重跑规格一构造器。
// **采用 R2 当且仅当 R2 可行且 key(R2) < key(R1)**（严格字典序，并列保留 R1）。
// R1 不可行 ⇒ 本级失败（由调用方进入升级链第 ② 级）。
LocalReplanOutcome localReplanTwoVersion(const LogisticsGraph& graph, const Vehicle& vehicle,
                                         const RoutePlan& previousRemaining,
                                         const std::vector<Order>& remainingOrders,
                                         const std::string& startNodeId, int startTimeMin,
                                         WeightType weight,
                                         const std::vector<OnboardItem>& onboard,
                                         const StashState& stash);

// §6.5 受影响车集合（路况）：存在被改动的边 e 使「e 在该车剩余路线上（有向、相邻两站）」
// 且 e.increaseRatio >= thresholdRatio。返回车辆**行下标**升序。
std::vector<std::size_t> affectedVehiclesForTraffic(
    const std::vector<RoutePlan>& remainingPlans, const std::vector<TrafficChange>& changes,
    double thresholdRatio);

// §6.5 受影响车集合（封路）：该车剩余路线**用到**被封闭的边（任一方向）。
// changes 里封路的两条有向边各一条，这里只关心 (from,to) 的**无向**出现。
std::vector<std::size_t> affectedVehiclesForClosure(
    const std::vector<RoutePlan>& remainingPlans, const std::vector<TrafficChange>& closedEdges);

// §6.6 新订单的选车（普通/紧急共用）：
//   cands = 全部车按 Δkey(o, v) 升序（并列按车辆 ID）；逐个试可行插入，首个可行即接受。
// Δkey 用**候选车自己的因子**重放。局部可行即接受，**不做「局部 vs 全局两版取优」**。
struct NewOrderAssignment {
    bool                     feasible = false;
    std::size_t              vehicleIndex = 0;   // 命中车（feasible 时有效）
    RoutePlan                plan;               // 命中车的重规划结果
    std::vector<std::size_t> candidates;         // 实际尝试顺序（Δkey 升序，供守卫/播报）
};
NewOrderAssignment chooseVehicleForNewOrder(const LogisticsGraph& graph,
                                             const std::vector<Vehicle>& vehicles,
                                             const std::vector<std::vector<Order> >& vehicleOrders,
                                             const std::vector<RoutePlan>& currentPlans,
                                             const std::vector<VehicleStartState>& starts,
                                             WeightType weight, const Order& order);

// 五级升级链的产物（§6.4）。
struct FleetRescheduleResult {
    // 每车的新计划（与 vehicles 下标对应）。**未被重算的车保持输入的原样**。
    std::vector<RoutePlan> plans;
    std::vector<std::vector<Order> > vehicleOrders;   // 重分配后的逐车订单集合
    int  level = 0;            // 0 = 无事可做（受影响集合为空）；1–4 = 采用的分级；5 = 拒绝
    bool rejected = false;
    std::string reason;
    std::vector<std::size_t> replanned;               // 被重算的车（不变量 5 用它判「未受影响车不变」）
    std::vector<std::string> log;                     // 分级播报行（每行都以「[复调度] 」开头）
    // 事件**之后**重建的腿结构（规格三 §2④ / US21：事件刷出的跨簇单也走三段腿）。
    // rescheduleFleet 只在逐车重规划上工作，腿/账本要单独重建 —— 这里用**与 planFleet 同一套**
    // buildLegs 读事件后的真实计划，让「LEG 说的形态」= 「STOP 里的实际执行」。
    std::vector<Leg>         legs;
    std::vector<std::string> legLog;                  // 腿登记日志（含 LEGDEGRADE 降级行）
};

// §6.4 五级升级链：① 局部重排 → ② 跨车再分配 → ③ 全车队重调度 → ④ 降载/转运并列 → ⑤ 拒绝。
// 每级都复用既有构造器/Δkey/转运层，**不新增算法**。
// 「失败」= 该级产不出可行解（容量 + 可达 + 先取后送 + 紧急硬优先/FIFO 全成立；
// 窗口只影响 penalty，不判不可行）。
FleetRescheduleResult rescheduleFleet(const LogisticsGraph& graph,
                                       const std::vector<Vehicle>& vehicles, WeightType weight,
                                       const FleetRescheduleInput& input,
                                       const RescheduleEvent& event);

// 从逐车计划反推「每车此刻服务的未送达订单」（GUI 用它把计划翻译成逐车订单集合，
// 免得再存一份可能与计划不一致的副本）。按 allOrders 的顺序输出。
std::vector<std::vector<Order> > vehicleOrdersFromPlans(
    const std::vector<RoutePlan>& plans, const std::vector<Order>& allOrders);

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