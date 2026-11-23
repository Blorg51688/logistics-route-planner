# 规格一迁移与退役清单（AFK 审计 · wayfinder map #16 票 #20）

> **日期**：2026-11-23（Asia/Shanghai）｜**基线**：HEAD `5b258f7`（工作区另有他人未提交改动，本清单未触碰）
> **用途**：把规格一/规格二各票的定稿结论落成**逐项可执行的迁移清单**，供实施票直接引用。每项给出：位置（文件:符号，行号仅作当前参考、会漂）、旧行为、目标行为、迁移动作、受影响测试/守卫、风险。
> **依据**：[#21](https://github.com/Blorg51688/logistics-route-planner/issues/21)（领域模型）、[#18](https://github.com/Blorg51688/logistics-route-planner/issues/18)（算法）、[#23](https://github.com/Blorg51688/logistics-route-planner/issues/23)（数据契约）、[#24](https://github.com/Blorg51688/logistics-route-planner/issues/24)（事件）、[#19](https://github.com/Blorg51688/logistics-route-planner/issues/19)（车队）、[#28](https://github.com/Blorg51688/logistics-route-planner/issues/28)（转运）、[#29](https://github.com/Blorg51688/logistics-route-planner/issues/29)（指标）、[#30](https://github.com/Blorg51688/logistics-route-planner/issues/30)（分配）各 resolution；[ADR 0002](../../adr/0002-all-orders-pickup-delivery.md)–[0006](../../adr/0006-metric-definitions.md)；[GLOSSARY.md](../../../GLOSSARY.md)；[设计.md](../../设计.md)。
> **纪律**：本清单**不含任何实现代码**；Glossary/ADR 是共享区，本票未改（增补建议见 §14，交 lead 落盘）。

---

## 0. 使用方式（给实施票）

- 每项一个编号 `MC-<组><序号>`，可独立交付、独立提交；批次建议见 §11。
- 「受影响测试/守卫」列的是**现状里会红/会漂**的项，不是要新写的全部守卫；新守卫集中登记在 §8。
- 引用基线数字一律标口径：本文所有「实测」= 2026-11-23 在 HEAD `5b258f7` 上的命令输出（`ctest -N` = 15 个目标；9 个测试二进制自报 **983** 项检查；`--self-check-actions` **27** 条 OK；三策略 `--plan-summary` 仍为 178.200/378.300/244.200/166、186.300/461.000/235.900/207、183.200/372.100/255.200/201，各「共 6 趟」——**这些旧值全部作废，只在迁移完成后重新捕获**，见 MC-F05）。
- 完成判据（DoD）：§12 清单逐条勾掉。

---

## 1. 变更总览

| 组 | 内容 | 主要来源票 |
| --- | --- | --- |
| A 数据模型 | 趟结构退役、仓库装货退役、Stop 动作集、逐单窗口、订单/车辆结构 | #21/#18/#19/#29 |
| B 算法 | 构造器换 pair-insertion、降载/取回口径、增量重规划复活、因子重放、12 条不变量 | #18/#19/#28 |
| C 事件 | NewCustomer→NormalOrder、等级键、上限与来源计数、事件刻窗口、A/B 条件均匀、新客户 sub_network_id | #24/#23/#30 |
| D IO/配置 | [orders] 7 列、[vehicles] 6 列、event_level_*、退役键白名单 | #23/#19 |
| E GUI/CLI | 摘要行四行、停靠明细 8 列、超时面板 7 列、路线面板删趟、每车页签、探针/自检改造 | #29/#21/#19/#24 |
| F 数据与工具 | default.ini 重生成、生成器自检、oracle A→B 语义、新 ctest 目标、黄金值两轮捕获 | #23/#29 |
| G 测试与守卫 | 逐测试文件的改造清单、#18 不变量与 #29 G1–G10 守卫落位 | #18/#29 |
| H 文档同步 | 设计.md 逐节、功能实现清单、测试用例表、GLOSSARY、ADR、数据结构地图、figures | 本票汇总 |

---

## 2. A 组 · 数据模型与结构（src/core）

### MC-A01 趟（Trip）结构整体退役
- **位置**：`src/core/RoutePlanner.h`（`struct Trip`、`RoutePlan::trips`、`RoutePlan::nodeTripIndex`、`sliceRemainder` 注释）；`src/core/RoutePlanner.cpp`（`beginTrip`、`appendLeg` 的 `Trip&` 形参、`flatten`、`multiTripPlanImpl`、`multiTripPlan`、`replanOneTrip`、`sliceRemainder`）。
- **旧行为**：`RoutePlan` 以 `trips` 为真源展平出扁平视图；多趟按载重分批，每趟终点都是起始仓库。
- **目标行为**：**单条连续路线**（基地 → 紧急段 → 普通段 → 基地）；`stops`（含动作集）为真源，`nodes/nodeArrivalMin/nodeIsStop` 保留（GUI 按下标推进依赖它们）；删 `trips/nodeTripIndex`。
- **迁移动作**：删 `Trip` 及全部消费方；`flatten` 退化为「stops → nodes/arrival/isStop」一次生成；`RoutePlan` 汇总字段（距离/耗时/成本/penalty/returnArrivalMin）保留。
- **受影响测试/守卫**：`tests/routeplanner_tests.cpp` 全部 34 个用例中 **14 个**直接断言 trips/offload/reclaim（清单见 §8 MC-G01）；`tests/integration_tests.cpp::checkMultiTripAndTransitOnRealData` 整段前提作废（总需求 516kg > 200kg 在新模型下**完全合法**）；`src/main.cpp` `--ui-probe` 的趟号断言（`tripCell` 正则、`tripCount()`、`completedTripOffset()`）。
- **风险**：GUI 的取货/降载/取回分支现在都靠 `nodeTripIndex` 反查 `Trip::pickups/offloads/reclaimStation`（`MainWindow.cpp::onAdvanceStop`）；必须先落 MC-A03（Stop 动作集）再拆 Trip，否则状态转移整段失据。

### MC-A02 仓库装货语义退役
- **位置**：`src/gui/MainWindow.cpp`（`loadForTrip`、`loadForCurrentTrip`、`arriveAt` 的趟次递增、`VehicleState::{tripNumber,tripLoadKg,departed,completedTrips}`）；`src/core/RoutePlanner.cpp`（`replan` 单趟分支假设「车辆在起点已装载全部货物」、`weave(..., total, ...)` 的装载口径、`Candidate` 注释「普通单=仓库→客户」）。
- **旧行为**：车在仓库时按计划的当前趟把货装上车（`出仓装车` 日志）；普通单无取货点、货默认在仓库。
- **目标行为**：车**空载出发**，货只经取货点 A 的动作装车；`onboard` 只由 A 取货增加、由 B 送达/offload 减少；`state_.loadKg` 从 0 起算。
- **迁移动作**：删 `loadForTrip/loadForCurrentTrip` 与趟次状态；`arriveAt` 只更新位置/时刻（回基地即收尾，不再计趟）；`recordRun` 的「出仓装车/接着送」两段文案退役。
- **受影响测试/守卫**：`src/main.cpp` `--ui-probe` 硬断言「本趟出发装载：N kg」必须同步删（否则探针永远红）；`--self-check-actions` 第 6 段用 `cap - state_.loadKg` 构造降载场景，需改为「在 A 装满再触发」的构造。
- **风险**：`OnboardItem` 现在按 nodeId 记货（见 MC-A05），仓库装货退役后若不同步改成 orderId，送达/降载扣货会扣错对象。

### MC-A03 Stop 增动作集（pickups[]/deliveries[]）+ 逐单 penalty
- **位置**：`src/core/RoutePlanner.h::Stop`（现仅 nodeId/rawArrivalMin/waitMin/arrivalMin/late/penaltyMin/remainingLoadKg）；`src/core/RoutePlanner.cpp::makeStop`（按合并窗口算 penalty）。
- **旧行为**：Stop 只有停靠级窗口与 penalty；无动作集，取货/降载/取回挂在 Trip 上。
- **目标行为**：`Stop` 携带 `deliveries[]`（订单 ID + 该单自己的 penalty/late）、`pickups[]`、`offloads[]`、`reclaims[]`、`phase`（紧急/普通）；停靠内**先卸后装**；每张送达单按自己的 we 判迟到（#18 Q14）。
- **迁移动作**：按 #18 §3 的 Stop 形状重写；`makeStop` 改逐单口径；顺路交货只在不增加等待时并入（#18 §5）。
- **受影响测试/守卫**：`tests/routeplanner_tests.cpp::testMultipleOrdersOnSameNodeMergeIntoOneStop`（并集窗口断言）；`tests/integration_tests.cpp::windowSpanMinutes`；`tools/oracle.py::route_metrics`（见 MC-F03）。
- **风险**：这是 #29 全部新指标的数据来源（MC-E 系列）；先做本项，E 组才有数据。

### MC-A04 remainingLoadKg 语义改造
- **位置**：`src/core/RoutePlanner.cpp::makeStop`（只扣送达、不含取货）；消费方 `MainWindow.cpp` 停靠明细「送后余货(kg)」列。
- **目标行为**：改名或改释为「**离开该站时的在车货量**」= 到达在车 − 送达 − offload + 取货 + reclaim（#29 §2.1 第 3 条）；「余货(kg)」列同步。
- **受影响测试/守卫**：`routeplanner_tests` 各处 `remainingLoadKg` 字面断言；`integration_tests` 的「任一趟在车货量不超过上限」检查口径。
- **风险**：旧口径只扣送达，会在合并停靠 + 转运下得出负值/假峰值；改语义而不改列名会被误读。

### MC-A05 OnboardItem 增 orderId
- **位置**：`src/core/RoutePlanner.h::OnboardItem`（nodeId/kg 两个字段）；消费方 `MainWindow.cpp::{deliverAt, onAdvanceStop 降载分支, onboardGoods}`、`multiTripPlanImpl` 在途货批。
- **目标行为**：按**订单**记账（orderId + 目的 B + kg）；送达按 stop.deliveries 的订单集合扣；降载/取回/转运都按订单移动（#18 §3 State、#28 账本）。
- **风险**：同节点多单（A∪B 合并停靠）下按 nodeId 扣货会把别的单一起扣掉——这是现有 `deliverAt` 已知的坑（`issue #5`），改造时必须连同守卫一起搬。

### MC-A06 Order 结构：pickup 必填 + 来源 + ID 前缀
- **位置**：`src/core/Order.h`（`pickupNodeId` 现仅紧急单使用；无来源字段）。
- **目标行为**：全部订单 A→B，`pickupNodeId` 必填；新增**运行时来源**字段 `initial/event_normal/event_urgent/manual_customer`（不进 [orders] 七列契约，#24 R5）；ID 前缀 O/N/U/C；`picked/served` 保留。
- **迁移动作**：`buildCandidates` 里「有 pickupNodeId ⇒ 紧急取送」的分流删除（见 MC-B01）；`Order::nodeId` 语义改为「送达点 B」并补注释。
- **受影响测试/守卫**：`tests/graph_fixtures.h::makeOrder` 无 pickup 参数（50 处调用）；`tests/routeplanner_tests.cpp` 全部手算字面量重算。
- **风险**：**这是全清单影响面最大的一项**——routeplanner_tests 351 项检查几乎全部建立在小图「仓库供货」语义上，需整体重写期望值（建议用 oracle 先算，MC-F03）。

### MC-A07 Vehicle 增 time_factor/cost_factor
- **位置**：`src/core/Vehicle.h`（四字段）。
- **目标行为**：六字段（+#timeFactor/#costFactor，#19 Q1/Q2）；因子只作用于**当前耗时**与成本，距离/等待/penalty 不乘（#19 Q4）。
- **受影响测试/守卫**：`tests/graph_fixtures.h::makeVehicle`；`config_tests` 的 vehicles 断言（`vehicles.size()==1` 要改为按数据断言）。
- **风险**：`weave/appendLeg/replanOneTrip` 累加处若漏乘，距离策略不红、时间策略悄悄错——须按 MC-B05 的「缩放后重放」守卫兜底。

### MC-A08 PlanStatus::OrderExceedsCapacity 语义与注释
- **位置**：`src/core/RoutePlanner.h::PlanStatus`（注释整段讲多趟分批）。
- **目标行为**：注释改写为「单张订单 q > 车容量 ⇒ 不可行」；配合 #23 的 `demand_kg < min(capacity)` 跨节校验后，静态数据走不到此分支，仅运行时事件/手动输入可触发（`insertUrgentOrderAt` 的 (0, cap] 校验同步改为**严格小于** min capacity）。
- **受影响测试/守卫**：`tests/config_tests.cpp::testLoadDefaultConfig` 的「单个订单最大货量 ≤ 载重」断言（改为 < min）；`routeplanner_tests::testSingleOrderExceedingCapacityIsInfeasible`（保留，语义不变）。

### MC-A09 StashState/OffloadOp 与 TransitLeg 分开（#28 §2）
- **位置**：`src/core/RoutePlanner.h`（`OffloadOp`、`StashState`、`Trip::{offloads,reclaimStation,reclaimKg,reclaimAfterOrderId}`）；`src/gui/MainWindow.cpp`（`VehicleState::OffloadKey/appliedOffloads/stashItems/stashStation`）；`src/main.cpp` 中转站面板「当前暂存货(kg)」列。
- **旧行为**：只有一种站内货——同车降载暂存（单堆、锁站、同趟取回）；GUI 第 4 列只反映这一堆。
- **目标行为**：**两类机制分开建模、分开验收**：① 降载暂存（`OffloadLeg`：同车卸、同车取，全局至多一堆、锁站）；② 转运（`TransitLeg`：卸入车 ≠ 取走车，账本守恒三式 C1–C3、期末 0、dwell ≤ 60、严格字典序更优才启用，#28 §5–§7）。判据就是验收断言：**同一台车卸了又取回 ⇒ 永远算降载，不算转运**。
- **迁移动作**：`OffloadOp` 按 #28 §2 的字段改名/扩为 `OffloadLeg`（补 orderId/时刻）；新增独立 `TransitLedger`（不在本清单展开实现，接口见 #28 §5.1）；GUI 中转站面板按 §E07 改。
- **受影响测试/守卫**：`routeplanner_tests` 的降载族（MC-G01 列表）；`--ui-probe` 中转站面板 4 列断言语义；`--self-check-actions` 第 6 段。
- **风险**：若把两类合并成「站内库存」，#28 的「卸货车 ≠ 取走车」判据与守恒三式会静默失效——**必须两个类型、两组守卫**。

### MC-A10 sliceRemainder / 增量重规划按单条路线重写
- **位置**：`src/core/RoutePlanner.cpp`（`sliceRemainder` 保留趟结构；`replanIncremental` 对「任一订单有 pickupNodeId」**直接退回全量**——新模型下**所有**订单都有 pickup ⇒ 增量重规划实质已死）。
- **旧行为**：逐趟增量、任一趟失败整体退回；取送单在场时永远全量。
- **目标行为**：按**停靠点**把单条路线切 leg，只重算受影响 leg、停靠顺序不变；失败退全量（#18 §10 链）；`usedIncremental` 回报保留。**这条必须配守卫**：「A→B 场景下 usedIncremental == true」否则设计.md H4 与 §5.7 的增量声明名存实亡。
- **受影响测试/守卫**：`routeplanner_tests::testIncrementalReplanRecomputesOnlyAffectedLeg`、`testIncrementalReplanHandlesMultiTrip`、`testIncrementalReplanForwardsOnboard`、`testIncrementalFallbackKeepsStash`；`docs/设计.md` §5.7「范围限制：只适用于单趟」整段要改。
- **风险**：高——这是当前代码里**已经在退化、但没有任何守卫能看见**的一条（对 A→B 输入恒为全量重算）。

---

## 3. B 组 · 算法（src/core/RoutePlanner.cpp）

### MC-B01 buildCandidates（合并窗口 + 紧急候选池）退役
- **位置**：`buildCandidates`（同点订单窗口取并集、urgent/pickedUrgent 分池）、`eraseCandidateByNode`、`findCandidate`。
- **目标行为**：请求层改为 (A,B,q,窗口,urgent) 的**取送对**；普通单进 EDD 序、紧急单按 FIFO 键 (arrivedAt, seq)；不再有「合并窗口」概念（#18 §2–§3）。
- **受影响测试/守卫**：`testMultipleOrdersOnSameNodeMergeIntoOneStop`（改为断言「合并停靠 + 逐单 penalty」）；`tools/oracle.py` windows 字典。

### MC-B02 weave 家族退役 → pair-insertion 构造器（新写）
- **位置**：`weave`、`appendLeg`（保留但改签名带 vehicle 因子）、`weaveUrgentWithPassing`、`commitWeaveStop`、`collectPassingCandidates`、`erasePassingCandidates`、`totalDemand` 的装箱用法、`multiTripPlan/multiTripPlanImpl`。
- **目标行为**：#18 §6–§10 的构造器：紧急段订单级 FIFO 追加 → 普通段 EDD + 全位置 (i,j) pair-insertion → repair(R≤3) → 全量兜底 → 降载最后手段；目标键 (Σpenalty, 策略权重, tieKey)。
- **迁移动作**：新构造器**不是迁移而是重写**；本清单只登记退役面与新入口签名（`planRoute/replan/insertOrder` 保留原名）。复杂度从 O(k²) 次最短路升到 O(m·n³)（m≤50/n≤100，#18 §13）。
- **受影响测试/守卫**：`routeplanner_tests` 期望值全部由 oracle 重算（MC-F03）；`integration_tests` 黄金值重捕获（MC-F05）。
- **风险**：**范围失控风险**——建议实施票把「构造器重写」独立成第一批次（§11），其余项都以它为前提。

### MC-B03 降载/取回口径对齐 #18 §11
- **位置**：`multiTripPlanImpl` 的降载分支（选货「离仓库最远优先」、选站「最小绕行增量」、`planStashStation` 单堆）。
- **目标行为**：选货 = 最少订单数（kg 降序、同 kg 按 ID，取前缀）；选站 = 构造式（每站构造降载版整条路线按 key 取优，并列按站 ID）；取回 = 普通段必做单点停靠、先于暂存单送达；全局单堆锁站。
- **受影响测试/守卫**：`testOffloadStationUsesFullDetour`（改为构造式口径）、`testTwoUrgentOffloadsMergeIntoSingleStashPile`、`testReplanKeepsStash`、`testIncrementalFallbackKeepsStash`、`testPickedUrgentStashReclaimMergedIntoSameTrip`。

### MC-B04 insertUrgentOrder → 通用插单
- **位置**：`src/core/RoutePlanner.cpp::insertUrgentOrder`（**强制 urgent=true**）；`src/gui/MainWindow.cpp::insertUrgentOrderAt`。
- **目标行为**：紧急/普通事件单与手动单共用插入链（#24 R8）；紧急 → FIFO 尾插；普通 → pair-insertion；来源与前缀由调用方带入。
- **受影响测试/守卫**：`traffic_tests::testInsertUrgentOrderWarnsButStillPlansWhenWindowCannotBeMet`；`--self-check-actions` 第 1/2/3 段。

### MC-B05 车辆因子重放（#19 §2）
- **位置**：`appendLeg/replanOneTrip/makeStop` 的耗时/成本累加；`insertUrgentOrder` 的 toA/ab 估计（`WeightType::Time` 的 totalWeight）。
- **目标行为**：到达时刻、penalty 比较全部用**该车缩放后**的耗时重放；等待与 penalty 不乘因子；距离与图表示显示基础边权；**不得**先按基础权重排好路线再乘因子。
- **受影响测试/守卫**：新增小图语义夹具（§8 MC-G02：三策略两两分化仍成立 + 同一节点序列下因子只改时刻）。
- **风险**：单点验证容易（默认 fleet 只有 1.00 时因子恒等）——**必须**在带 1.25 慢车的夹具上断言，否则因子代码全错也不红。

### MC-B06 #18 的 12 条不变量：落点与守卫映射

| # | 不变量（#18 §12） | 实现落点 | 守卫（新增/改造） |
| --- | --- | --- | --- |
| 1 | 每张订单恰服务一次 | 构造器可行性 F1 | 改造 `integration_tests::checkPlanCoversAllOrders`（按订单断言，不再按节点） |
| 2 | A 停靠索引 < B 停靠索引 | 构造器 F2 | 新增 routeplanner_tests 断言（全部订单） |
| 3 | 前缀载重 [0,C]、先卸后装 | Stop 动作集 + F3 | 新增前缀峰值守卫（含合并停靠峰值口径） |
| 4 | 硬优先（紧急停靠全先于普通；紧急阶段无普通取货） | 构造器 F4 | 新增「阶段」断言（与停靠明细「阶段」列同源，G8） |
| 5 | 订单级 FIFO（U_k 的 B ≤ U_{k+1} 的 A） | 紧急段追加 | 改造 `testUrgentFifoOrder`（索引/时刻双断言） |
| 6 | 同刻至多一张紧急单在车 | 构造器 F5 | 新增守卫 |
| 7 | 同阶段同节点至多一次停靠，例外仅 FIFO 强制重访 | 构造器 F5 | 新增守卫（重访只含该单的这一个动作） |
| 8 | 停靠零耗时；早到等待计耗时不计 penalty；arrival == we 不算迟到 | `makeStop`（现已是严格 >，保留） | oracle 夹具补 `arrival == we` 边界（#23 §5.4） |
| 9 | penalty 逐单累加 | Stop.deliveries[].penalty | **G7**：Σ逐单 == 总 penalty == 超时面板各行之和 |
| 10 | 暂存守恒、期末 0、单堆锁站 | 降载实现（MC-B03） | 改造降载族测试 + 期末 0 守卫 |
| 11 | 取回先于该单送达 | 取回停靠硬约束 | 改造 `testPickedUrgentStashReclaimMergedIntoSameTrip` |
| 12 | 输出确定性（字典序 + tieKey） | 构造器评价器 | 改造 `testTieBreakIsDeterministicByNodeId` 到 key 三元组 |

### MC-B07 #19 不变量 4 的精确化落点（转运例外）
- **现状**：`tests/integration_tests.cpp` 断言「每个有订单的配送点恰好停靠一次」= 隐含「一单一车一停」；ADR 0003 只有决定条 3「一单由一台车全程完成（跨车转运归 #28）」，**编号不变的「不变量 4」只在 #19 resolution §3 里，ADR 正文没有**。
- **目标行为**：表述统一为「每张订单**恰好被服务一次**；默认一台车全程，**转运版例外**——取货车与送达车可不同」（#28 §12）。
- **迁移动作**：① 改写 integration 断言为逐订单口径；② ADR 0003 补一段「不变量清单（#19 §3）」或在决定条 3 后加精确化引注（**共享区，交 lead 落盘**，见 §14）；③ GLOSSARY「跨车改派」词条已区分，无需动。
- **风险**：不落 ADR，未来读者会在「一单一车」与转运例外之间来回打架——这正是 #28 点名要 #20 处置的项。

---

## 4. C 组 · 事件与动态订单（core + gui）

### MC-C01 SimEventKind::NewCustomer → NormalOrder
- **位置**：`src/core/SimEvent.h`（枚举值）、`simEventName`（「新客户」→「普通订单」）、`src/gui/MainWindow.cpp::settleEventsUpTo` 的 switch 分支（现指向 `addRandomCustomerAction`）、`src/core/SimEvent.cpp::pickEvent` 的 kinds 数组。
- **目标行为**：事件池四类 = 路况/普通订单/紧急订单/封路；动态加节点只保留**手动**「模拟新客户」动作（#24 Q6/R10）。
- **迁移动作**：**保位改名**——枚举位置不动（位次即抽签扫描顺序与并列确定性），只改枚举名与中文名；`settleEventsUpTo` 的该分支改调「普通单刷单」（A 均匀、B 在剩余里一次抽出、3–15kg、窗口 [mark, 24:00]）。
- **受影响测试/守卫**：`tests/simevent_tests.cpp`（names 数组含「新客户」、分布断言 10/15·3/15·1/15·1/15）；`config_tests.cpp:248` 附近的「新客户」夹具文案；`--self-check-actions` 第 4 段。
- **风险**：若同时改枚举**顺序**，固定种子的随机串会整条变——分布断言红、等价性断言红，误以为实现错了。

### MC-C02 事件数值权重键 → event_level_*（#23）
- **位置**：`src/core/Config.h::GeneralConfig`（`eventWeight{Traffic,Urgent,Customer,Closure}` 四个 int）；`src/io/ConfigLoader.cpp::loadGeneral` 的 intBinders（`event_weight_*`）；`src/gui/MainWindow.cpp::settleEventsUpTo`（`EventWeights w;` 四行赋值）；`config/default.ini` 四键。
- **目标行为**：四键 `event_level_{traffic,urgent,customer,closure}`，值域 extra_low|low|medium|high（不区分大小写、键名精确），代码内固定表 **1|3|9|27**；缺键回落 traffic=high、customer=medium、urgent=low、closure=extra_low（即默认 27:9:3:1 → 67.5/22.5/7.5/2.5%）；未知等级**带行号报错**（`event_level_traffic 取值非法: <值>（允许 extra_low/low/medium/high）`）。
- **受影响测试/守卫**：`config_tests.cpp` 213–216/446–472 的全部 `event_weight_*` 断言；`simevent_tests` 的默认分布断言（10:3:1:1 → 27:9:3:1）。
- **风险**：旧配置文件会**静默变默认**而非报错——所以退役键白名单（MC-D03）必须与键替换**同一提交**落地。

### MC-C03 settleEventsUpTo：上限、来源计数、事件刻窗口基准（#24）
- **位置**：`src/gui/MainWindow.cpp::settleEventsUpTo`（`kMaxPendingUrgent = 2` 硬编码、只数 urgent、无普通上限）；`insertUrgentOrderAt`（窗口 = `currentTimeMin()`/**当前时刻**，非事件刻）；`insertUrgentOrderAction`（`ib == ia → (ib+1)%n` 的**有偏** B 选择）。
- **目标行为**：上限紧急 2 / 普通 6，计数 = **事件来源**且未送达（含在车，#24 R6）；满额类别抽签前权重置 0、两类都满仍每刻 1 抽不空刻（R7）；窗口基准 = **该事件的事件刻 mark**（R4；普通 [mark,24:00]、紧急 [mark, min(mark+60,1440)]）；A/B 条件均匀（A 均匀抽 + B 在剩余 n−1 里一次抽出，固定 2 次随机数，#24 §1.1）。
- **迁移动作**：`settleEventsUpTo` 把 mark 传进刷单路径（现在只传给了日志前缀）；`insertUrgentOrderAt` 加 mark 参数或重载；抽签前只把满额类别置 0、不预扫可执行性；抽中不可执行照实播报不重抽（R8）。
- **受影响测试/守卫**：`--self-check-actions` 第 2 段参数断言（窗口 60min 改为「基准 = 事件刻」断言）；simevent 等价性断言（一次推进一刻 == 逐个推进）保持为总守卫。
- **风险**：批量结算跨多刻时，现在所有单共用「结算后时刻」做窗口——改成事件刻后事件单窗口**变早**，行为可观察地不同（这是修复，但要写进提交说明）。

### MC-C04 手动「模拟新客户」：B=X + 来源 + sub_network_id（#24 R10 + #30 缺陷）
- **位置**：`src/gui/MainWindow.cpp::addRandomCustomerAction`（订单 nodeId = 新节点、**pickupNodeId 为空**、窗口 [now,24:00]、前缀 C）；`src/core/GraphEdit.cpp::addRandomCustomer`（`Node created` 只设 type/x/y/id/name——**从不设 subNetworkId**，而 `Node.h` 默认 0）。
- **目标行为**：建节点 X（保留现有 10% 边距/最近邻双向合成边/ID X###/名称「新客户###」）+ 一张普通单 **A=随机现有配送点(≠X) → B=X**，3–15kg、[now,24:00]、来源 manual_customer、前缀 C；**新节点继承最近邻节点的 sub_network_id**（#30 指定的口径），否则该单落进不存在的「子网络 0」无法归簇。
- **迁移动作**：`addRandomCustomer` 在已算出的 `nearest` 上继承 `subNetworkId`；若最近邻是仓库（sub=0），**建议**回落到最近中转站/配送点的非 0 编号（该边界票面未拍板，实施票按「必须非 0 且确定」自行定，本清单默认建议：取最近**配送点或中转站**的编号）。
- **受影响测试/守卫**：`tests/graphedit_tests.cpp::testAddRandomCustomerProperties` 30 种子断言**新增**「新节点 subNetworkId == 最近邻的且 ≠ 0」；`--ui-probe` 中转站面板子网络列（现只查中转站行，不查新配送点）可加配；`--self-check-actions` 第 4 段改为「订单 B=X 且 X 进路线」。
- **风险**：`--ui-probe` 现断言中转站子网络非 0，但**没有任何守卫查新配送点**——该缺陷今天无声（地图 #16 Not yet specified 的原话），修复必须带反向验证（回退继承 → 守卫红）。

### MC-C05 订单来源标记与 GUI 呈现
- **位置**：`Order` 新字段（MC-A06）；`MainWindow::updatePanels` 订单表「状态」列；日志文案。
- **目标行为**：来源只作呈现与计数（不进配置契约）；待处理上限只数事件来源。
- **风险**：低。

---

## 5. D 组 · IO 与配置契约（src/io + config）

### MC-D01 [orders] 6 列 → 7 列 + 十条校验（#23 §一）
- **位置**：`src/io/ConfigLoader.cpp::loadOrders`（`collectRows(..., 6, ...)`；现不查 ID 重复、不查 A/B 存在与类型）。
- **目标行为**：七列 `ID, pickup, delivery, demand_kg, window_start, window_end, is_urgent`（顺序固定）；校验十条按 #23 §一表逐条落地（ID 非空唯一、pickup/delivery 存在且为 delivery 类型、A≠B、demand 正实数、窗口必填 HH:MM 且 start<end、is_urgent∈{0,1}、空字段逐条报错、缺节报错/空节允许、旧 6 列按字段数报错）。
- **受影响测试/守卫**：`config_tests.cpp` 中 6 列订单夹具（`kMinimalConfig`、`buildScaleBoundaryConfig`、错误用例段约 8 处）需改 7 列；新增 10 类报错断言（含行号，复用 `expectParseFailureAtLine`）。

### MC-D02 [vehicles] 4 列 → 6 列 + 八条校验（#19 Q1）
- **位置**：`src/io/ConfigLoader.cpp::loadVehicles`（`collectRows(..., 4, ...)`；不查 ID 重复、空节不报错）。
- **目标行为**：六列 `ID, start_node, capacity_kg, depart_time, time_factor, cost_factor`；空节报错（车队不能为空）；ID 非空唯一；两因子为有限正实数。
- **受影响测试/守卫**：`config_tests.cpp` 中 4 列车辆夹具（`grep` 实测 **14 处** `V0x, …` 行）；`vehicles.size()==1` 断言改为按数据断言；`integration_tests` 里 `cfg.vehicles[0]/front()` 的 8 处。
- **风险**：`src/main.cpp`/`MainWindow.cpp` 共 **24 处** `vehicles.front()/vehicles[0]`（grep 实测：MainWindow 21、main.cpp 3）；规格一阶段先保单车语义（每处改「按配置逐车」或显式取 0 号车），完整多车归 #27/#25 实施票——**不要**在本批顺手改多车 UI。

### MC-D03 [general] 退役键白名单报错（#23 §3）
- **位置**：`src/io/ConfigLoader.cpp::loadGeneral`（未知键现**静默忽略**）。
- **目标行为**：白名单式——`service_time_min`、`event_weight_{traffic,urgent,customer,closure}`、`traffic_change_interval_sec`、`urgent_order_interval_sec` 出现即**带行号报错 + 迁移提示文案**（提示改用 event_level_*、等级表）；其余未知键保持忽略。
- **受影响测试/守卫**：新增 6 类报错断言；`oracle.py`/`generate_dataset.py` 若仍引用旧键会自检红（MC-F02）。
- **风险**：用户手头的旧 ini 会直接加载失败——**这是预期**，但 README/课设要求整理不动，提示文案要写清「本仓库 config/default.ini 已由生成器重生成」。

### MC-D04 service_time_min 保持不存在
- **位置**：`grep -rn service_time_min src config` 实测 **0 命中**（提交 66e282e 已删）。
- **目标行为**：不重新引入；`docs/设计.md` §4.3 的 `service_time` 推进式与「默认 5 分钟」、§8.2 示例里的 `service_time_min = 5` 是**文档残留**，随 MC-H01 删除；`docs/archive/设计-交接.md`、`docs/中转站策略分析.md` 是历史件，保留并标注。
- **受影响测试/守卫**：无代码项；`tools/check_doc_links.py` 不查文字，需人工核对。

### MC-D05 跨节校验 demand_kg < min(capacity_kg)（严格小于）
- **位置**：`ConfigLoader::buildFromIni`（解析顺序已是 vehicles 先于 orders，天然可用）。
- **目标行为**：违反报 `货物量 X kg 不小于最小车辆载重 Y kg（任何车都装不下这一单）`；边界 `q == min(capacity)` **不合法**（严格小于）。
- **受影响测试/守卫**：`config_tests.cpp` B7 边界用例「载重恰好等于总需求」**前提作废**，改为「单笔 q == min(capacity) 报错 / 略小合法」两条。

---

## 6. E 组 · GUI 与 CLI（src/gui + src/main.cpp）

### MC-E01 停靠明细 6 列 → 8 列（#29 §4.2）
- **位置**：`MainWindow.cpp::buildDocks`（`stopTable_` 6 列表头：趟/配送点/原始到达/等待(分)/送达/送后余货(kg)）；`updatePanels` 填充；`src/main.cpp` `--ui-probe` 的表头断言（**硬编码 6 列清单**）与趟号正则。
- **目标行为**：8 列 = `序号 | 阶段 | 配送点 | 动作（取<a>·送<b>） | 原始到达 | 等待(分) | 送达 | 余货(kg)`；无「趟」列。
- **迁移动作**：表头、填充、探针三处**同提交**改（历史教训：服务时间移除时只改数据漏改表头，P32）。
- **受影响测试/守卫**：`--ui-probe` 的表头断言（现 `:480-514`）、趟号单调断言整段删除、`cli_ui_probe` ctest。

### MC-E02 超时订单面板 6 列 → 7 列且逐单一行（#29 §4.3）
- **位置**：`MainWindow.cpp`（`lateTable_` 6 列；`updatePanels` 按**停靠点**一行，订单列是 `orderIdsAt` 拼接、窗口列是 `windowTextAt` **合并窗口**）。
- **目标行为**：逐单一行：`订单 | 配送点 | 档位（紧急·普通） | 状态 | 到达 | 窗口 | penalty(min)`；窗口用**该单自己的**窗口。
- **迁移动作**：`orderIdsAt/windowTextAt` 从超时面板消费方退役（保留给订单表用）；`VehicleState::LateStop::orderId` 现存的是拼接串，改为单订单 ID。
- **受影响测试/守卫**：G10（行数 == 迟到订单数）；`--ui-probe` 无现成断言（面板文本探针补）。

### MC-E03 路线信息面板：删趟次三段与「各趟」，加三项新指标（#29 §4.1）
- **位置**：`MainWindow.cpp::updatePanels`（`趟次：已完成…当前第…全部完成约需` 三段 + `各趟：` 列表，实测当前输出 6 趟明细）。
- **目标行为**：删三段与列表；加 `取货准时率 / 送达准时率（紧急·普通分列，分母 0 记「—」）/ 平均在车（利用率·峰值）` 三行；五项汇总的「停靠 N 站」改「停靠 S 站 · 动作 A 次 · 已送达 M 站」。
- **受影响测试/守卫**：G9（四个片段出现在面板文本）；`docs/测试用例表.md` 人工目视项。

### MC-E04 车辆信息面板：每车页签 + 固定信息两因子（#19 Q6）
- **位置**：`MainWindow.cpp::buildDocks`（`vehicleTabs_` 现固定两页签「固定信息/运行状态」）；`updatePanels`（`vehicles.front()`）；`src/main.cpp` `--ui-probe` 断言 `vehicleTabs == 2` 与「车上载重/本趟出发装载」正则。
- **目标行为**：页签数 = 车辆数，标题 = `<ID> · <容量>kg`，页内再分「固定信息/运行状态」两块；固定信息加耗时倍率/成本倍率两行；探针改断言「页签数 == vehicles.size()、逐车能读到 ID/容量/两因子、载重 ≤ 该车容量」。
- **风险**：`--ui-probe` 的断言与面板改造必须同批，否则 ctest 永远红；「运行状态字段清单」归 #25/#27，本批只搭容器。

### MC-E05 --plan-summary 四行格式（#29 §3.1）
- **位置**：`src/main.cpp`（现两行：五项汇总 + `超时停靠点`；`共 N 趟` 仅 N>1 时打印）。
- **目标行为**：四行 = ①五项汇总（`停靠 <S> 站 · 动作 <A> 次`）②三项率（分母 0 记 `—`）③`超时订单：`逐单 ④`路线 <N0> <N1> …`（供 oracle 对拍；GUI 不显示）。
- **受影响测试/守卫**：新增 ctest `cli_plan_summary`（G1/G2/G6 读 stdout）；`tests/integration_tests` 黄金值断言改读 `tests/plan_golden.h`。

### MC-E06 --ui-probe 断言改造清单（`src/main.cpp`）
- 删：停靠明细 6 列表头清单、趟号正则/单调/种类断言、`tripCount()/completedTripOffset()`、`vehicleTabs == 2`、「本趟出发装载」正则。
- 改：车辆面板断言按 §E04；载重上限按**逐车** capacity（现用 `vehicles.front().capacityKg`）。
- 增：停靠明细 8 列表头（G8）、路线信息面板四片段（G9）、超时面板逐单（G10）。
- 风险：探针是「守卫读用户可见输出」的主力，改造时**先写期望再改面板**（红→绿），否则一次全绿掩盖中间错位。

### MC-E07 中转站面板第 4 列：单堆暂存 vs 转运账本（#28 交接）
- **位置**：`MainWindow.cpp::buildDocks`（`当前暂存货(kg)` 列）、`updatePanels`（只由 `stashStation/stashItems` 单堆驱动）、`--ui-probe` 4 列断言、`--self-check-actions` 第 6 段面板逐帧断言。
- **目标行为**：第 4 列继续反映**降载暂存堆**（同车取回）；**转运账本**（多笔、各笔卸入/取走车不同）另立呈现——具体面板归 #25 演示口径，本清单只登记接口要求：「第 4 列 ≠ 转运量」，账本要有可读出口（面板或日志）。
- **受影响测试/守卫**：`cli_ui_probe` 的 4 列断言保留；转运账本守卫随 #28 实施票加。

### MC-E08 --self-check-actions 改造
- **位置**：`MainWindow.cpp::runActionSelfCheck`（第 1–6 段）。
- **目标行为**：第 2 段参数断言的窗口基准改事件刻；第 4 段改「订单 B=X 且 X 进路线」（现断新节点进路线，语义弱）；第 6 段降载构造改为「在 A 装满后触发」，并按 MC-A09 断言「卸货车 == 取回车」。
- **受影响测试/守卫**：`cli_action_self_check`；断言数 27 → 实测后按新口径登记（AGENTS §3 同步）。

### MC-E09 onAdvanceStop 状态转移改造
- **位置**：`MainWindow.cpp::onAdvanceStop`（降载/取回/取货三段全靠 `nodeTripIndex → Trip` 反查；`deliverAt` 按 nodeId 扣货、标记该节点**全部**订单 served）。
- **目标行为**：按当前停靠的**动作集**执行：deliveries 逐单扣（按 orderId）、pickups 逐单装、offloads/reclaims 按账本；`order.served` 只对该单置位。
- **风险**：这是物理状态唯一权威源（`VehicleState`），改造必须保「重规划不得改写物理事实」断言（`--self-check-actions` 第 5 段保留）。

---

## 7. F 组 · 数据与工具（config + tools + CMake）

### MC-F01 config/default.ini 整体重生成
- **位置**：`config/default.ini`（现 [general] 数值权重四键、[vehicles] 单行 4 列 V01 200、[orders] 25 行 6 列旧数据）。
- **目标行为**：[general] 四个 event_level_*（默认 27:9:3:1）；[vehicles] 6 列（规格一阶段 V01 单车 200kg·1.00·1.00；规格二阶段 3 行）；[orders] 7 列 25 行 = #23 §四的**权威订单表**（不要手抄，从 #23 resolution 引用）。
- **迁移动作**：改 `tools/generate_dataset.py` 后 `--write` 再自检（`dataset_selfcheck` 是逐字节比对，手改必红）。
- **风险**：黄金值断言在数据切换的瞬间**必然全红**——数据提交与黄金值捕获必须同一批（§11 批次 4）。

### MC-F02 generate_dataset.py 自检扩展（#23 §5.2 / #19 §1）
- **位置**：`tools/generate_dataset.py`（`CAPACITY_KG = 200` 单车常量；`build_text` 输出 4 列/6 列；`self_check` 断言 `[vehicles] 字段数不是 4`、`[orders] 字段数不是 6`、单笔货量 vs 单车容量）。
- **目标行为**：输出 6 列/7 列；自检新增 #23 §5.2 全部断言（A/B 同子网络、B 集合 == 全部配送点、A 集合 == 6 枢纽/回程点、总量 516kg、最大单笔 < min 容量、全 is_urgent=0、紧窗/晚开窗仍在）+ #19 §1 车队断言（因子 ∈[0.5,2]、至少一台基准车、容量分化、两型分化）。
- **受影响测试/守卫**：`dataset_selfcheck`。

### MC-F03 oracle.py 的 A→B 时间模型（#23 §5.4 / #29 §5.2）
- **位置**：`tools/oracle.py::route_metrics`（签名带 `service_min`、`windows` 是**已合并**窗口字典、单停靠模型；17 处调用全带 5.0 服务时间）。
- **目标行为**：取送语义——取货停靠无窗口不记 penalty、停靠零耗时、只在 B 做早到等待、`arrival == we` 严格不算迟到；窗口按单；新增三项指标复算；**oracle 不实现选路**（只对给定节点序列复算算术）。
- **受影响测试/守卫**：`oracle_selfcheck`；全部 C++ 黄金字面量的来源。
- **风险**：oracle 与 C++ 夹具的双份定义是已知遗留（设计.md §14.4）——A→B 化时**先改 oracle 再抄值**，避免人算。

### MC-F04 CMakeLists 新增目标与真值表
- **位置**：`CMakeLists.txt`（现 15 个 ctest 目标）。
- **目标行为**：新增 `cli_plan_summary`（读 `--plan-summary` stdout，断言 8 个标签/单位/格式/数值 == plan_golden.h）；新增 `tests/plan_golden.h`（13 字段 × 3 策略 constexpr，L1/L2 共用，**不允许双份定义**）。
- **受影响测试/守卫**：`ctest -N` 15 → **16**；`docs/测试用例表.md` 口径表现写 20（本就过期，实测 15）→ 一并订正为 16。

### MC-F05 黄金值两轮捕获（#29 §5.4）
- **流程**：① 构造器/数据落地后 `--plan-summary distance|cost|time` 各跑一次，记录四行原文 + 提交号 + 日期；② oracle 用第 4 行路线序列复算 13 项（容差 0.001）——**不一致先查实现、不改数据**；③ 拆值进 plan_golden.h；④ 四处落值（真值表 / 测试用例表 C25 / AGENTS §3 / 设计.md §14.3 ⑤）；⑤ 旧三行标注作废保留。
- **两轮**：规格一（单车 V01 200kg）一轮；规格二（三车）一轮；schema 不变数值变。
- **纪律**：**实施前不许预写任何数字**（#29 明令）。

---

## 8. G 组 · 测试与守卫（tests/）

### MC-G01 routeplanner_tests.cpp（351 项 / 34 用例；14 个直接依赖 trips/offload/reclaim）
- **删除/重写前提的**：`testTotalDemandOverCapacityBecomesMultiTrip`、`testWithinCapacityStaysSingleTrip`（趟数断言作废，改前缀载重断言）、`testIncrementalReplanHandlesMultiTrip`（改单条路线增量）、`testTripContinuityAfterUrgentInsertWhileEnRoute`（改「路线首尾相接」断言）。
- **语义升级的**：`testUrgentOffloadToTransitWhenNotFitting`、`testUrgentFitsLeavesOffloadsEmpty`、`testUrgentPassingDeliversAAndBGoods`、`testUrgentOffloadKeepsAAndBDeliversBoth`、`testUrgentFifoOrder`、`testReplanKeepsStash`、`testPickedUrgentNotOvertakenByNewUrgent`、`testOffloadStationUsesFullDetour`、`testTwoUrgentOffloadsMergeIntoSingleStashPile`、`testIncrementalFallbackKeepsStash`、`testPickedUrgentStashReclaimMergedIntoSameTrip`。
- **期望值来源**：`makeOrder` 无 pickup（`tests/graph_fixtures.h`，50 处调用）全改；全部字面量从 oracle 重取。
- **注意**：`:423-425` 注释引用的 `testTransitBufferProducer()` **已不存在**（D23 残留的悬空引用），随本轮清理。

### MC-G02 integration_tests.cpp（108 项）
- `windowSpanMinutes`（并集）→ 逐单跨度；`checkPlanCoversAllOrders`（停靠数 == 去重订单点、每点恰停一次）→ 按订单断言 + A∪B 停靠数；`checkMultiTripAndTransitOnRealData` 整段重写为「前缀载重 + 单条路线」；`checkDataQualityInvariants` ①–⑤ 升级（三策略两两分化、自身目标不劣于另两条、完工耗时 ≤ 12h、迟到**订单**数阈值 + 逐单窗口跨度 ≤4h、黄金值 13 字段 × 3 策略读 plan_golden.h）；`checkTrafficAndUrgentOrderOnRealData` 的紧急单窗口 0–1440 改 [t, t+60]。
- **已知教训**：④ 的阈值不是硬编码 3（characterization，实施后按实测重定并标注）。

### MC-G03 config_tests.cpp（232 项）
- 夹具：14 处 4 列车辆行、约 8 处 6 列订单行全改；`testLoadDefaultConfig` 的 vehicles/orders/事件键断言重写；`testDefaultConfigRoutingPreconditions` 补「A 也可从基地往返达」；错误段补 [orders] 10 类 / [vehicles] 8 类 / 退役键 6 类报错断言；边界段 B7 改 MC-D05 口径、B8（紧窗）保留。
- `kMinimalConfig`、`buildScaleBoundaryConfig` 是所有错误用例的底座，**先改它们**。

### MC-G04 simevent_tests.cpp（27 项）
- `testDefaultWeightsDistribution` 期望 10/15·3/15·1/15·1/15 → 27/40·9/40·3/40·1/40；`names` 数组「新客户」→「普通订单」；`testSimEventNames` 同步；等级→权重表与非法等级报错断言新增；保底分支（全零权重）保留。

### MC-G05 graphedit_tests.cpp（32 项）
- `testAddRandomCustomerProperties`（30 种子）新增「subNetworkId 继承最近邻且 ≠ 0」断言 + **反向验证**（去掉继承 → 必须红）；这是地图 fog 毕业项的落点。

### MC-G06 traffic_tests.cpp（85 项）
- `insertUrgentOrder` 相关用例随 MC-B04 的通用插单改签名；`simulateTrafficChange/needsReplan` 不动（语义不变）；「已送达 72 条边」数字按当前边数核对（现注释与实测 106 不一致，顺手修）。

### MC-G07 #29 的 G1–G10 守卫与 R1–R10 反向验证
- 全部落位：G1 摘要行字段、G2 黄金值 13×3、G3 动作数恒等式（A == 2×取送对数、S ≤ A）、G4 两率序关系、G5 在车量上界、G6 分母 0 记「—」、G7 逐单 penalty 一致性、G8 表头 8 列、G9 面板四片段、G10 超时逐单。
- 反向验证按 #29 §6.3 表逐条执行并留档到 `docs/测试用例表.md` §六；G3/G4 无法单独回退，靠 R2/R9 兜底（如实写）。

### MC-G08 ctest 目标数与文档口径
- 15 → 16 后同步：`docs/测试用例表.md` 口径表（现写 20、实测 15——**本就过期**）、`docs/功能实现清单.md` A8 与附录、`AGENTS.md` §2/§3、`README.md`（如引用）。

---

## 9. H 组 · 文档同步（docs/）

### MC-H01 docs/设计.md（逐节）
| 节 | 动作 |
| --- | --- |
| §2 决策表 | D14/D16/D20/D21/D23/D24 追加「已由规格一取代」注（**保留原文作历史**，仿 P25/P30 注式样）；D12 单车辆补多车现状注 |
| §4.1 | 仓库语义改「基地不供货」；中转站改「只存实际卸下的货（容量不限、期末 0）」 |
| §4.3 | 删 `service_time` 推进式与「默认 5 分钟」；改停靠零耗时 + 逐单窗口 |
| §4.4 | **并集窗口 → 逐单窗口**（票面点名项）；订单属性改七列契约；停靠合并规则改「动作集、先卸后装」 |
| §4.5 | 车辆六字段 + 因子语义 + 「单车辆（规格一）/异构（规格二）」 |
| §4.6 | `Trip`/趟列表/中转站暂存状态段删除，改 Stop 动作集 + StashState/TransitLeg |
| §5.2 | 贪心串联 → pair-insertion + EDD + 有界 relocate（含字典序目标） |
| §5.3 | 紧急插单 → 通用插单 + FIFO 硬优先 + 降载最后手段 |
| §5.5 | 载重约束改「前缀在车量」；**删「总需求可超载 → 多趟」** |
| §5.6 | **整节退役**（多趟分批），替换为「单条连续路线构造」概述 + 指向规格一成稿 |
| §5.7 | 「只适用于单趟」限制删除；增量重规划按停靠 leg |
| §7 | 工具栏「模拟新客户」保留**手动**；事件池四类；面板列/页签按 E 组 |
| §8.1/§8.2 | 键清单换 event_level_*；示例删 service_time_min、vehicles 6 列、orders 7 列 |
| §8.4 | **删「总订单需求 ≤ 车辆载重」**（票面点名项；新约束 = 单笔 < min 容量 + 每转站归属 ≥1 点） |
| §9/§14.3/§14.6 | 三类用例补逐单口径；不变量表 ①–⑤ 升级；测试分层 + cli_plan_summary |
| §16 | **按轮次历史日志，不重写**；在 P38 后追加 P39 条目记录本轮（由 #17 成稿票统一写） |

### MC-H02 docs/功能实现清单.md
- B1.5（中转站语义 + 「实测 6 趟」全部改写）、B2.3/B2.4（列数）、B3.3（单车辆注）、E2.1（超时订单面板列）、E2.2（插单）、H3（event_level_*）、A1/A2/A8、§五 5.1（多车辆 → 规格二已定稿）、§七 P1/P4、附录验证规模（983/15 → 实测重登）。

### MC-H03 docs/测试用例表.md
- 口径表 20 → 16；C8/C9/C17（多趟/单趟/逐趟增量三行重写）、C21/C22（事件权重 → 等级）、C25（黄金值重捕获后重登记）、C26/C27；B17/B18 **引用了不存在的测试名**（`testUrgentBufferChoiceIsNeverWorse`/`testEveryDepotTripIsFullyLoaded`，grep 实测 0 命中）删除或改指现存用例；B19 随 sliceRemainder 改写；§六补 R1–R10 新表。

### MC-H04 GLOSSARY.md
- 已由 #21/#18/#19/#24/#28/#29/#30 各票落盘（提交 caa3191/64b7d11/d6a89b5/611afc0/fc2d63a/5b258f7），术语齐备；实施票只需在使用处对齐，**无需新增**（唯一建议词条见 §14）。

### MC-H05 ADR
- 不新增（本轮无新架构决策）；ADR 0003 需补 #19 不变量清单或精确化引注（MC-B07，共享区交 lead）；ADR 0002 已写明 Supersedes 0001 的范围，无需动。

### MC-H06 docs/数据结构地图.md、docs/项目速懂-详解.md、start-here/read/项目速懂.md、数据结构总览.html、项目结构图.html
- 数据结构地图：`Trip/multiTripPlan/flatten/PickupOp/OffloadOp/StashState/Candidate` 各行按新结构重写（先改地图，再机械转换 HTML）。
- 项目速懂（图版/详解/HTML）：三策略表删「趟数」列；「trips[] 真源」数据流图改 stops。
- **注意**：`start-here/` 与 `docs/文档导航.md` 当前有他人未提交改动，本票一律不动；上列文件的同步由实施票在成稿（#17/#26）登记后进行。

### MC-H07 docs/figures（F1–F6/U1–U4）
- **F4（多趟分批）整张退役或重画**：其 mmd 头注引用的 `fillBufferFromDepot/bankBufferEnRoute/tripHub/sortTransitOpsByTime/recordStationDraw/candidateDemand/hubOfNode/stockOf` 在 src 中**已不存在**（grep 实测 0 命中）——图早已与代码脱节，先在 figures/README 标注再排重画。
- F5（紧急插单）重画（两版取优叙事已随 D23 退役）；F3/F6/U1/U3/U4 按 weave→pair-insertion、趟→路线、事件四类更新；`figures/README.md` §五的「设计.md 滞后于源码」清单随本轮重核。

### MC-H08 历史件标注（不改内容）
- `docs/中转站策略分析.md`、`docs/records/specs/transit-urgent-buffer.md`、`docs/records/decisions/transit-stock-full-load.md`、`docs/records/decisions/probes/`、`docs/archive/设计-交接.md`：均含「趟/满载出仓/缓冲库存」叙事，**保留原文**，只在入口处（AGENTS §6 已有索引）确认不被当成现状引用。

### MC-H09 AGENTS.md 与 README
- §2 ctest 目标数、§3 三策略黄金值三行（重捕获后换新值 + 标注口径）、983 断言数重登；§4 引用的「趟」措辞顺手核对。README 若引用趟数/黄金值同步。

---

## 10. 专项核对表（票面点名的必覆盖项）

| # | 票面点名项 | 本清单落点 |
| --- | --- | --- |
| 1 | multiTripPlan | MC-A01/MC-B02 |
| 2 | 仓库装货 | MC-A02 |
| 3 | 「新客户」事件 | MC-C01/MC-C04 |
| 4 | 事件数值权重键 | MC-C02/MC-D03 |
| 5 | [orders] 旧 6 列解析 | MC-D01/MC-A06 |
| 6 | [vehicles] 旧 4 列 | MC-D02/MC-A07 |
| 7 | D23 暂存货残留 | MC-A09/MC-E07/MC-H08 + §8 悬空注释清理 |
| 8 | #18 的 12 条不变量 | MC-B06 |
| 9 | #19 不变量 4 精确化（转运例外） | MC-B07 |
| 10 | #28 StashState/OffloadLeg 与 TransitLeg 分开 | MC-A09/MC-E07 |
| 11 | #29 摘要行 / 停靠明细 8 列 / 超时面板 7 列 | MC-E01/E02/E03/E05 |
| 12 | #30 GraphEdit sub_network_id 缺陷 | MC-C04/MC-G05 |
| 13 | #24 NewCustomer→NormalOrder、上限与来源计数、窗口基准 | MC-C01/MC-C03 |
| 14 | #23 退役键白名单 / service_time_min 不存在 | MC-D03/MC-D04 |
| 15 | 设计.md §4.4 并集窗口 → 逐单窗口 | MC-H01（§4.4 行）+ MC-B01 |
| 16 | 设计.md §8.4「总需求 ≤ 载重」过时约束 | MC-H01（§8.4 行）+ MC-G02 |

---

## 11. 实施顺序建议（批次）

1. **批次 1 · 契约与数据模型**：MC-A05/A06/A07、MC-D01/D02/D03/D05、MC-F02 → 数据能加载（ctest 中 config/dataset 两目标绿）。
2. **批次 2 · 构造器重写**：MC-A01/A03/A04、MC-B01/B02/B03/B05、MC-A10 —— 一次把 routeplanner_tests 重写到绿（用 oracle 先算值）。
3. **批次 3 · 事件**：MC-C01/C02/C03/C04、MC-B04、MC-G04/G05。
4. **批次 4 · 数据切换 + 黄金值**：MC-F01/F03/F04/F05、MC-G02（integration）——数据提交与捕获同批。
5. **批次 5 · GUI/CLI**：MC-E01–E09、MC-G08（探针/自检与面板同批红→绿）。
6. **批次 6 · 文档**：MC-H01–H09（AGENTS/测试用例表的数字等批次 4 的捕获结果）。

> 依赖理由：Stop 动作集（A03）先于一切 GUI 项；构造器先于黄金值；黄金值先于文档数字。批次 1/2 之间**不可**并行（夹具互相咬）。

---

## 12. 完成判据（DoD）

- [ ] `ctest` 全绿，目标数 16（以 `ctest -N` 实测为准），各二进制自报数重登四处。
- [ ] 三策略 `--plan-summary` 四行输出可复现，13 字段 × 3 策略落在 plan_golden.h 与四处落值点，oracle 对拍一致（容差 0.001）。
- [ ] §10 的 16 项逐条有对应提交号；G1–G10 各自至少一条反向验证记录（`docs/测试用例表.md` §六）。
- [ ] `grep -rn 'multiTripPlan|Trip' src` 0 命中；`grep -rn 'event_weight_' src config` 0 命中；`grep -rn 'service_time_min' src config` 保持 0 命中。
- [ ] 旧黄金值三行标注作废保留；AGENTS §3 换新值并标口径。
- [ ] 设计.md §4.4/§5.6/§8.4 等按 MC-H01 落定；§16 只追加不重写。

---

## 13. 明确不做 / 交接出去

- **多车渲染、跨车改派、转运构造器本体**：#28/#30 的机制实现不在规格一迁移批（清单只登记接口与断言落位，MC-A09/MC-B07）。
- **每车「运行状态」字段清单、转运账本的 GUI 呈现、演示配置**：#25（演示口径）/#27（复调度）。
- **文档导航.md 与 start-here/ 的登记与同步**：工作区现有他人未提交改动，本票不动；由 #17/#26 成稿票登记。
- **AGENTS.md §3 数字**：等批次 4 捕获后由实施票换值（本清单只写流程）。
- **`requirements/`**：只读，永不改。

---

## 14. Glossary 增补（交 lead 落盘；本票未改 GLOSSARY.md）

- **阶段（phase）**：路线被硬优先切成的两段——**紧急段**（全部紧急停靠，订单级 FIFO）与**普通段**（普通单 EDD + 取回停靠）。停靠明细「阶段」列与不变量 4/7 的断言都以它为口径。→ 建议插「订单与优先级」节「优先级（硬优先）」之后。

（其余术语已由 #21/#18/#19/#24/#28/#29/#30 各票落盘完成，2026-11-23 核对无缺口。）
