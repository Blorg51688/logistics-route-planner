# docs/figures —— 报告与 PPT 用的流程图 / UML

课程设计报告要用的全部图：**9 张函数流程图**（报告【详细设计】「主要函数的流程图」明文必需）
+ **4 张 UML 图**（PPT 补充建议「用专业 UML 工具有加分」）。

> **依据原则**：图的每个节点、字段、调用方向都指向具体的 `文件:符号名`。
> 文档与源码冲突时**以 `src/` 为准**——见 §五「文档与代码差异」。
> **用户区不放图**（准入规则见 [`文档导航.md`](../文档导航.md) §四）：图放 `docs/` 下，
> 报告排版时从这里取 PNG。

---

## 一、函数流程图（F1–F9）

| 编号 | 文件 | 图名 | 依据（源码符号名） | 对应报告 |
|---|---|---|---|---|
| F1 | [F1-minheap.png](F1-minheap.png) | 手写最小堆：入堆 / 出堆 / 上浮 / 下沉 | `src/core/MinHeap.h:MinHeap`；`src/core/MinHeap.cpp:MinHeap::push`/`popMin`/`siftUp`/`siftDown`/`swapEntries` | 【详细设计】最短路的手写堆 |
| F2 | [F2-dijkstra.png](F2-dijkstra.png) | Dijkstra 单源最短路：`dist[]`/`previous[]` + 惰性删除 + 路径复原 | `src/core/Dijkstra.cpp:shortestPath`；`src/core/WeightType.h:pickWeight`；`src/core/MinHeap.cpp:push`/`popMin` | 【详细设计】 |
| F3 | [F3-greedy-weave.png](F3-greedy-weave.png) | **pair-insertion**：普通段全位置对枚举 → 逐个重放评价 → 字典序三键取最小；失败走 repair / 兜底 / 降载 | `src/core/RoutePlanner.cpp:tryInsertPair`/`replay`/`compareRoutes`/`repairRoute`/`insertNormalAll` | 【详细设计】主构造 |
| F4 | [F4-multitrip.png](F4-multitrip.png) | **单条连续路线构造**：`buildAll` 多起点（EDD / 按 A 聚簇 / 按 B 聚簇）→ 紧急段 FIFO → 普通段 pair-insertion → 两级兜底 → 降载 → 拒绝 | `src/core/RoutePlanner.cpp:buildAll`/`insertionStarts`/`buildWithOrder`/`appendUrgentAll`/`insertNormalAll`/`finalize` | 【详细设计】单条连续路线与载重约束 |
| F5 | [F5-urgent-insert.png](F5-urgent-insert.png) | **通用插单**：`insertOrder` 按订单自己的 `urgent` 分流——紧急 FIFO 尾插 / 普通 pair-insertion；装不下先降载 | `src/core/RoutePlanner.cpp:insertOrder`/`replan`/`appendUrgentAll`/`insertNormalAll`/`shortestPath`（冲突判定） | 【详细设计】动态订单插入 |
| F6 | [F6-traffic-replan.png](F6-traffic-replan.png) | 路况 → **顺序不变版重放**的增量重规划 → 不可行退回全量 | `src/gui/MainWindow.cpp:onSimulateTraffic`；`src/core/Traffic.cpp:simulateTrafficChange`/`needsReplan`；`src/core/RoutePlanner.cpp:sliceRemainder`/`replanIncremental`/`replay`/`finalize`/`replan` | 【详细设计】路况与增量重规划 |
| F7 | [F7-legs-ready-chain.png](F7-legs-ready-chain.png) | **腿与就绪链**（规格三 §1/#45）：跨簇单展开 L1/L2/L3 → `readyTime` 链（L1=发车、L2=L1 完成、L3=L2 完成）→ `--dump-plan` 的 LEG 行；含退化/降级分支 | `src/core/RoutePlanner.cpp:buildLegs`/`stationOfCluster`/`chooseLegVehicle`/`readRelayExecution`；`src/core/RoutePlanner.h:Leg`/`LegKind`；`src/main.cpp` 的 LEG 行 | 【详细设计】腿模型 |
| F8 | [F8-relay-wait-lookahead.png](F8-relay-wait-lookahead.png) | **三段接力与车等货**（规格三/#48）：`applyRelayLayer` → L1 慢车 → t1 → L2 快车（前瞻窗 W=60）→ t2 → L3 对簇慢车 → `replay` 的「车等货」等待 → 账本与腿结构；含不更差兜底/职责降级 | `src/core/RoutePlanner.cpp:applyRelayLayer`/`buildRelayChain`/`buildRelayOnwardSpecs`/`replay`/`relayChainConsistent`/`buildTransitLedger`；`src/core/RoutePlanner.h:kLookaheadWindowMin` | 【详细设计】三段接力与前瞻窗口 |
| F9 | [F9-custody-repair.png](F9-custody-repair.png) | **custody 修复与审计**（规格三/#49）：封路/路况打断接力链 → `repairCustodyBreaks` 用新图重跑该单三段接力 → 自洽复核 → 拒绝兜底；另一条是只读口径 `auditCustody`（4 类缺口） | `src/core/RoutePlanner.cpp:repairCustodyBreaks`/`unpairedUnloadOrders`/`rescheduleFleet::accept`/`auditCustody`；`src/core/RoutePlanner.h:CustodyReport`/`CustodyGap` | 【详细设计】custody 守恒 |

## 二、UML 图（U1–U4）

| 编号 | 文件 | 图名 | 依据 | 对应报告 |
|---|---|---|---|---|
| U1 | [U1-core-classes.png](U1-core-classes.png) | core 层类图：领域类型 / 图与堆 / 规划结果 / **腿与转运账本（Leg/LegKind/TransitLeg/FleetPlan）/ custody（CustodyGap/CustodyReport）** / 路况事件 / 配置随机 / io | 各 `src/core/*.h` 与 `src/io/*.h`（`.mmd` 顶部注释逐条列出文件名:符号名；2026-11-26 #52 按规格三补 Leg/LegKind/TransitLeg/CustodyGap/CustodyReport/FleetPlan 与对应自由函数） | 【数据结构设计】+【详细设计】 |
| U2 | [U2-gui-io-classes.png](U2-gui-io-classes.png) | gui 层 + io 层类图：`MainWindow`/`VehicleState`/`GraphScene`（**#50：`setFleetRoutes(fastFlags)` / `setVehicleMarks` / `roleColor`**）/`NodeItem`/`EdgeItem` + 配置链 | `src/gui/*.h`、`src/io/*.h`、`src/core/Config.h`、`src/core/RoutePlanner.h` | 【数据结构设计】模块结构 |
| U3 | [U3-seq-startup.png](U3-seq-startup.png) | 时序图：启动 → 加载配置 → 首次规划 → 画布渲染（**#50：`syncScene` 调 `setFleetRoutes` + `setVehicleMarks` 角色两色 + 车辆环**） | `src/main.cpp`、`src/io/ConfigLoader.cpp:ConfigLoader::load`、`src/io/IniFile.cpp:IniFile::loadFromFile`、`src/gui/MainWindow.cpp`（构造 / `replan` / `afterPlanReplaced` / `syncScene` / `canvasTargetRoutes`）、`src/gui/GraphScene.cpp:build`/`setFleetRoutes`/`setVehicleMarks`、`src/core/RoutePlanner.h:fastVehicleFlags` | 【详细设计】 |
| U4 | [U4-seq-traffic-replan.png](U4-seq-traffic-replan.png) | 时序图：路况触发 → 增量重规划 → 界面刷新 | 同 F6 的符号名 + `src/core/LogisticsGraph.cpp:updateEdgeTime` | 【详细设计】 |

**U1/U2 两条读法（评审常问）**：

1. `Dijkstra` / `RoutePlanner` / `Traffic` / `SimEvent` / `GraphEdit` 画成 `«utility»`，
   **因为它们不是类**——源码里是 `logistics` 命名空间下的自由函数，不存在 `class RoutePlanner`。
2. `tryInsertPair` / `replay` / `repairRoute` / `buildAll` / `insertionStarts` / `finalize` 在
   `RoutePlanner.cpp` 的匿名命名空间里、无头文件声明，所以 U1 只列头文件里的公开 API；
   这几个内部函数看 F3–F6。（旧图里的 `weave`/`multiTripPlanImpl`/`replanOneTrip`/`appendLeg`/`flatten`
   已随规格一删除，`grep` 0 命中。）
3. `VehicleState` 是 `MainWindow` 的嵌套 struct，不是独立顶层类；它是**物理事实唯一权威来源**，
   重规划只读它作输入，绝不改写（`plan_` 只描述"接下来怎么走"）。

## 三、文件格式与「谁权威」

- **`.mmd` 是真相源，`.png` 是它的渲染产物。** 要改图，改 `src/*.mmd` 后重跑 `docs/figures/src/render.sh`，
  连 PNG 一起提交；不要直接改 PNG（改不了，也查不出 diff）。
- **本机可重渲（2026-11-24 实测）**：`render.sh` 存在且可用——它经 `npx @mermaid-js/mermaid-cli@11`
  调用 mermaid（不需要全局装 `mmdc`），依赖 `chrome-headless-shell`（已缓存在 `~/.cache/puppeteer/`）。
  本轮 F3/F4/F5/F6/U1/U2/U3/U4 的 `.mmd` 更新后**已重渲 PNG**；F1/F2 未改动，PNG 保持原样。
- `.mmd` 是纯文本、可 `git diff`，改图不用重画；PNG 宽 1600px、白底，报告可直接插图。
- 选 mermaid 而非 StarUML / PlantUML 的理由：源文件是纯文本，便于版本管理与评审；
  PNG 满足报告插图需求；若后续要更"专业"的 UML 记号，可从 `.mmd` 快速迁移到 PlantUML。
- **引用这些图时沿用「文件名:符号名」，不要用行号。** 本批图 2026-10-18 收编时就是因
  `F1` 写了 `MinHeap.cpp:44` / `:57` 而返工——那两行实际落在 `siftDown` 内部
  （`MinHeap::siftUp` 与 `siftDown` 各自是完整函数）。行号会漂，符号名不会。

## 四、重新渲染

```bash
bash docs/figures/src/render.sh          # 全量重渲 13 张 → docs/figures/*.png
```

依赖：`npx`（随 Node.js）+ `chrome-headless-shell`。本机首次使用前装一次：

```bash
npx -y puppeteer browsers install chrome-headless-shell
```

脚本会在 `~/.cache/puppeteer/` 下自动查找；找不到会给出上面这条可操作提示并退出（不静默失败）。
渲染参数固定为 `-w 1600 -b white`，因此同一份 `.mmd` 在不同机器上应得到同一张 PNG
（本机实测：重渲 U3 与仓库内 PNG 字节数一致）。

## 五、文档与代码差异（**2026-11-26 #52 重核：原四条已全部消解；规格三新图 F7/F8/F9 与 #50 画布形态已对齐源码，无新增差异**）

本节原先列了 4 处「`docs/设计.md` 滞后于源码」的地方。规格一实施（#32–#37）+ 本轮文档同步（#38）后，**四处都已对齐**：

| # | 原差异 | 现状 |
| --- | --- | --- |
| 1 | 设计 §5.6 仍保留 `stock[t]` / 「枢纽 = 中转站」分支 | **已消解**：§5.6 整节退役为「单条连续路线构造概述」；F4 按新构造画 |
| 2 | 设计 §5.5/§5.6 不变量表写「中转站暂存 ≥0 / 期末 ==0」但语义是缓冲库存 | **已消解**：现行不变量是**降载暂存**守恒（Σ卸入 − Σ取回、期末 0、全局一堆锁站），见设计 §4.6；U1 按 `OffloadLeg`/`StashState` 画 |
| 3 | 设计 §5.3 未提 A/B 取优、penalty 一票否决、车上缓冲就地满足 | **已消解（机制本身退役）**：D23 缓冲库存整体删除；现行是通用插单 + FIFO 硬优先 + 降载最后手段（设计 §5.3），F5 按此画 |
| 4 | 设计 §5.7「只适用于单趟」vs 逐趟增量 | **已消解**：趟结构不存在；增量改为**顺序不变版重放**（规格二 §10 裁定 1 / ADR 0007），设计 §5.7 与 F6/U4 同口径 |

> **仍需注意的历史件**（不是差异，是历史）：`docs/中转站策略分析.md`、`docs/records/specs/transit-urgent-buffer.md`、
> `docs/records/decisions/transit-stock-full-load.md`、`docs/records/decisions/probes/`、`docs/archive/设计-交接.md`
> 都含「趟 / 满载出仓 / 缓冲库存」叙事——**保留原文作历史**，别当现状引用。

## 六、事实与口径

- 所有图的事实取自 `src/` 与 [`docs/设计.md`](../设计.md) §4/§5/§6，未新编。
- 图里的数字（30 节点 / 106 有向边 / 25 订单 / **516kg 总需求** / 车 V01 200kg·08:00·1.00·1.00 /
  事件等级派生权重 **27:3:9:1** / 事件刻 15min）均来自 `config/default.ini` 与当前源码，
  属**已提交基线**口径（HEAD `cdc98e2`）。
- **四个口径不许混**（2026-11-25 实测，规格三 #44 收尾轮，HEAD `a028865` → #50 并入后 `fc4f98e` 行为不变）：`ctest` **目标数 17**（`ctest -N` 为准）、9 个二进制**自报合计 3955 项**、`cli_plan_summary` **773 项** + `cli_demo_transfer` **22 项**、`--self-check-actions` **112 条 OK**。
- 图不是功能清单：每张图都落在「结构怎么支撑调度决策」上——堆支撑「取最小」、
  前驱数组支撑路径复原、`CostMatrix` 支撑 O(1) leg 查询、`stops` 动作集支撑状态转移与顺序不变版重放、
  `TrafficReport` 支撑「要不要重算」的判定。
- 结构侧的完整依据（为什么选这个结构、代价多大、没有它得自己写什么）见
  [`docs/数据结构地图.md`](../数据结构地图.md)；本目录只放图本身。