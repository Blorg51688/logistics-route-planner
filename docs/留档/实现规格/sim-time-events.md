# 规格与计划：软件内时间的模拟事件模型（sim-time-events）

> autopilot `sim-time-events`（2026-09-26）。用户原话要点：
> 模拟路况**未按软件内时间**触发，而是以两种错误方式触发：① 界面无操作时**随系统时间**触发；
> ② Debug 模式中**每次推进就触发**。要求改为：按**软件内时间每 15min 抽一个带权重的事件**
> （破坏性事件权重最低、紧急订单次之、路况最大）；新增「**推进一刻**」；Debug 更名「**按站模拟**」
> 并新增并列的「**按时间模拟**」。

## 1. 已定位的现状（代码证据）

| # | 事实 | 位置 |
|---|---|---|
| 1 | 真实秒级定时器 `trafficTimer_` 按 `traffic_change_interval_sec`(默认 **30 真实秒**) 触发 `onSimulateTraffic()`，界面无操作也照跑 → **bug ①** | `MainWindow.cpp:141-147`、`:743-750` |
| 2 | `onDebugTick()` **每个 tick 无条件** `onSimulateTraffic()`；紧急单按 `urgent_order_interval_sec` 节流；封路每 20 tick 一次 → **bug ②** | `MainWindow.cpp:752-786` |
| 3 | `onAdvanceStop()` 是**唯一**的物理状态转移点：`arriveAt` / `deliverAt` / `loadForCurrentTrip` | `MainWindow.cpp:645-720` |
| 4 | 软件内时钟：`state_.atTimeMin` / `currentTimeMin_`；每个节点有 `plan_.nodeArrivalMin` | `MainWindow.h:45,161`、`MainWindow.cpp:253` |
| 5 | `Debug` 字样连带：`MainWindow.{h,cpp}`、`scripts/manual_test_wizard.sh`（**第 7 关名** + 第 381 行布局描述）、`tools/check_wizard_ui_names.py:19`（UI 名清单）、`src/core/Config.h`、`src/main.cpp`、`tools/generate_dataset.py` | grep |
| 6 | `--ui-probe` 断言「至少 9 个工具栏动作」并打印实际动作名 | `main.cpp:302-314` |
| 7 | 配置键 `traffic_change_interval_sec` / `traffic_change_ratio` / `traffic_time_increase_{min,max}` / `urgent_order_interval_sec` | `config/default.ini [general]` |

## 2. 用户已确认的决策（Phase 0 澄清结果）

| 决策点 | 选定 |
|---|---|
| 要求原文「每 30 秒改 10% 路况」怎么办 | **以软件内时间为准**，删掉真实秒级定时器；保留「改 10%」（`traffic_change_ratio=0.1` 不动）；**同步改写文档口径**并注明「30 秒」是旧口径 |
| 事件权重 | 路况 **10** : 紧急订单 **3** : 新客户 **1** : 道路封闭 **1**（合计 15 → 66.7% / 20% / 6.7% / 6.7%） |
| 路线跑完后 | **停在目的地并结束模拟**（不再空转抽事件） |

## 3. 设计（关键约束）

### 3.1 「推进一刻」必须**逐节点重放**，不能跳节点

`onAdvanceStop()` 是唯一状态转移点，项目在 P25/P28 花了好几轮才把
「记账从计划反推」这类缺陷根除。因此 15min 推进**不得**直接把
`nodeIndex_` 跳到目标时间对应的节点——必须**依次走过其间每个节点**、每个都调
同一套 `arriveAt`/`deliverAt`/`loadForCurrentTrip`。否则送达/装卸/存货记账与物理事实脱节，
正是本项目反复踩过的那类回归。

> 由此「半路显示所在段的起点」**自然成立**：推进到「最后一个到达时刻 ≤ T 的节点」，
> 该节点就是车辆当前所在段的起点（用户例子：A→B→C 都已过、正驶向 D ⇒ 停在 C）。

### 3.2 事件时钟

- 绝对软件内时间对齐：`nextEventMin = ((now / interval) + 1) * interval`（`interval = event_interval_min`，默认 15）。
  起点 08:00=480 是 15 的整数倍，故首个事件落在 08:15。
- **每个事件刻必抽一个事件**（用户要求"跑长途不会长时间不触发"）。
  抽中「紧急订单」但已达"待处理紧急单上限 2"时，**改抽其余事件**（保证不出现空刻），并记一行日志说明。

### 3.3 两种模拟模式（并列、互斥）

| 模式 | 自动推进粒度 | 刷新节奏 |
|---|---|---|
| **按站模拟**（原 Debug 模式） | 一站（现有 `onAdvanceStop`） | 1 秒/步 |
| **按时间模拟**（新增） | 一刻 = 15min（新的时间推进） | 1 秒/步 |

两者用 `QActionGroup` 互斥；到路线终点时**自动停止并播报一次**（不再空转刷日志）。

## 4. 交付物与文件级边界

**核心层（可独立测试，不含 Qt）**
- 新增 `src/core/SimEvent.h/.cpp`：`SimEventKind{Traffic,UrgentOrder,NewCustomer,RoadClosure}`、
  `EventWeights`、`pickEvent(weights, Random&)`（加权抽签）、`nextEventTimeMin(now, interval)`。
- `src/core/RoutePlanner.h`：新增纯函数 `nodeIndexAtTime(const RoutePlan&, int timeMin)`（最后一个 `nodeArrivalMin ≤ T` 的下标；早于首个则 0）。
- 新增 `tests/simevent_tests.cpp`：抽签分布（大样本落在容差内）、零/负权重、单选权重、
  `nextEventTimeMin` 边界（正好在刻上 / 跨日）、`nodeIndexAtTime` 边界（早于首节点 / 超过末节点 / 恰等于到达时刻）。

**配置**
- `config/default.ini [general]`：**删除** `traffic_change_interval_sec`（口径消失）与
  `urgent_order_interval_sec`（被事件模型取代）；**新增**
  `event_interval_min = 15`、`event_weight_traffic = 10`、`event_weight_urgent = 3`、
  `event_weight_customer = 1`、`event_weight_closure = 1`。
  保留 `traffic_change_ratio / traffic_time_increase_min / traffic_time_increase_max`。
- `src/core/Config.h`、`src/io/ConfigLoader.cpp` 同步；`tools/generate_dataset.py` 同步输出。

**GUI**
- `MainWindow.{h,cpp}`：删 `trafficTimer_`/`onTrafficTick`；删 `onDebugTick` 的每 tick 触发；
  新增 `onAdvanceMoment()`（推进一刻）、`onSimModeToggled(mode)`、事件抽取与播报；
  工具栏改名「按站模拟」+ 新增「按时间模拟」「推进一刻」；日志区分「事件播报」与「送达播报」。
- `--ui-probe`：动作计数断言与动作名清单同步。

**文档 / 向导 / 检查器（口径改写 + 更名连带的必改项）**
- `scripts/manual_test_wizard.sh`：第 7 关改名（`Debug 模式（A1）` → `按站模拟（A1）`）、第 381 行布局描述、相关文案；
  若有「每 30 秒」表述一并改写。
- `tools/check_wizard_ui_names.py:19` 的 UI 名清单；
  `tools/check_wizard_claims.py` 若引用被删的配置键需同步。
- `docs/设计.md`：§7（GUI 语义）、§8（配置契约）、§16 追加一节记录本次口径变更（**30 秒 → 软件内 15min**）
  与新事件模型；`docs/功能实现清单.md` 的 H3（每 30 秒）与 A1（Debug 模式）两行。
- `README.md` / `文档导航.md`：若含 Debug 字样则同步。

## 5. 验收判据（verifier 逐条对照）

1. **不再有真实时间触发**：`grep` 证明 `trafficTimer_`/`onTrafficTick` 已删；任何自动事件都源自软件内时间推进。
2. **不再有"每推进一次就触发"**：`onDebugTick` 的每 tick `onSimulateTraffic()` 调用已消失。
3. **每 15min 软件内时间恰好抽 1 个事件**，且权重为 10:3:1:1（大样本频率落在容差内；权重可由配置覆盖）。
4. **推进一刻**：一次推进到下一个 15min 刻；其间每个跨越节点都经过同一套状态转移
   （断言：一次推进后 `已送达` 与逐个推进到同一时刻的结果**完全一致**——这是防"跳节点"的核心断言）。
5. **半路显示段起点**：构造"某刻车辆位于 i→i+1 途中"，断言界面车辆位置 == 节点 i。
6. **到终点即停止**：两种模拟模式在路线跑完后都停止计时器并只播报一次。
7. **更名与并列模式**：工具栏存在「按站模拟」「按时间模拟」「推进一刻」；两者互斥；向导第 7 关与新名字一致。
8. **回归**：`ctest` 全绿（含新增 simevent 测试）；**三策略黄金值逐位不变**
   （distance 178.200/378.300/244.200/166/6、cost 186.300/461.000/235.900/207/6、time 183.200/372.100/255.200/201/6）。
9. **文档口径**：`设计.md` 与 `功能实现清单.md` 中「每 30 秒」的口径已改写并说明理由；
   `要求源文件/` 与 `课设要求整理.md`（要求原文）**不动**。
10. **守卫**：新增行为必须有自动化断言且**反向验证**（把修复回退，断言必须失败）。

## 6. 执行顺序（按依赖）

1. 核心层 `SimEvent` + `nodeIndexAtTime` + 单测（无 GUI 依赖，先做且可独立验证）。
2. 配置层（`Config.h` / `ConfigLoader` / `default.ini` / `generate_dataset.py`）+ `config_tests` 同步。
3. GUI：删旧触发、加事件时钟与抽签、加「推进一刻」与两种模拟模式、改名。
4. 向导 / UI 名清单 / `--ui-probe` / 文档口径改写。
5. `--self-check-actions` 增加守卫（一次推进一刻 == 逐个推进到同一时刻）。
6. QA（build / ctest / self-check / ui-probe / 黄金值）→ 双评审门。

## 7. Round 2 细化（GUI 重做）—— 下一轮的执行依据

### 7.1 工具栏结构（`MainWindow.cpp` 建栏处）

| 动作 | 类型 | 说明 |
|---|---|---|
| `推进一站` | 既有，不动 | 现有的 `onAdvanceStop` |
| `推进一刻` | **新增** | 进到下一个 15min 事件刻；`onAdvanceMoment()` |
| `按站模拟` | **由「Debug 模式」改名** | 可勾选，自动每 1 秒调 `onAdvanceStop` |
| `按时间模拟` | **新增** | 可勾选，自动每 1 秒调 `onAdvanceMoment` |

- 两个模拟动作放进同一个 `QActionGroup`（**互斥**）；勾选其一自动取消另一个。
- 停止方式沿用"再点一次取消勾选"（与现有 Debug 一致），并保留「关闭后不再产生副作用」的既有不变式。
- `--ui-probe` 的动作名清单与最低动作数断言同步（现为 15，将变 17）。

### 7.2 `onAdvanceMoment()` 的正确实现骨架

```
目标时刻 T = nextEventTimeMin(当前软件内时刻, eventIntervalMin)
while (nodeIndex_+1 < plan_.nodes.size()
       && plan_.nodeArrivalMin[nodeIndex_+1] <= T) {
    onAdvanceStop();            // 复用唯一状态转移点：arriveAt / deliverAt / loadForCurrentTrip
}
抽 1 个事件并执行 + 播报
```
- **绝不允许**直接把 `nodeIndex_` 跳到 `nodeIndexAtTime(plan_, T)` 然后只补一次状态——那会跳过沿途
  的送达/装卸/存货记账（正是 P25/P28 花多轮根除的那类缺陷）。
- `nodeIndexAtTime` 用于**显示**（半路取所在段起点）与**断言比对**，不用于跳过。

### 7.3 事件执行与播报

| 事件 | 落地动作 | 播报格式（日志） |
|---|---|---|
| 路况 | `onSimulateTraffic()` 内部逻辑 | `[事件] 路况变化 …`（沿用既有"改动 N 条边"文案） |
| 紧急订单 | `insertUrgentOrderAction()` | `[事件] 紧急订单 …` |
| 新客户 | `addRandomCustomerAction()` | `[事件] 新增客户 …` |
| 道路封闭 | `onCloseRandomRoad()` | `[事件] 道路封闭 …` |

- 每刻**必播报一行**（带上该刻的软件内时刻，如 `08:15`），保证"跑长途也不会长时间无输出"。
- 抽中紧急订单但待处理已达上限 2 → **改抽其余事件**并加一行说明（不出现空刻）。
- 送达仍按既有 `送达 %1（到达 %2…）` 文案播报；**一轮时刻内跨越多个节点时会连播多条**。

### 7.4 到终点即停

两种模式都在 `nodeIndex_+1 >= plan_.nodes.size()` 时：`stop()` 计时器 + 播报一次
`本次配送已完成，模拟结束`，并**清除勾选状态**。同时修掉现有"终点后每 tick 重复刷
『本次配送已完成』"的日志噪音。

### 7.5 防跳节点的核心守卫（判据 #3）

在 `--self-check-actions` 里加：把同一初始状态复制两份，一份「推进一刻」到 T，
另一份循环「推进一站」直到所有 `nodeArrivalMin <= T` 的节点走完；断言两者的
**`已送达` 计数 / 已发生 penalty / 车上货量 / 地图车辆位置**完全一致。
反向验证：把 `onAdvanceMoment` 改成"直接跳 `nodeIndexAtTime` 再补一次转移"，该断言必须失败。

### 7.6 `--demo N` 的处置

`--demo N` 现用于截图与回归。**保持"按站模拟 N 步"语义不变**（避免连带破坏 `--render-window`
与既有截图流程），并在 `--help` 里写明；如需时间模式演示，后续再加 `--demo-time N`（本任务不做）。

### 7.7 事件结算纪律（**关键**：任何推进路径都必须结算事件刻）

事件由**软件内时钟**决定，而不是由你按了哪个按钮决定。因此：

- 维护一个成员 `nextEventMark_`（下一个未结算的事件刻）。初始
  `nextEventMark_ = nextEventTimeMin(出发时刻, eventIntervalMin)`。
- 新增一个 helper `settleEventsUpTo(int timeMin)`：
  ```
  while (nextEventMark_ <= timeMin) {
      抽 1 个事件 → 执行 → 播报（带该刻时刻）
      nextEventMark_ = nextEventTimeMin(nextEventMark_, eventIntervalMin);
  }
  ```
- **`onAdvanceStop()` 与 `onAdvanceMoment()` 在推进位置之后都必须调用它**（`now = state_.atTimeMin`）。

**为什么必须这么做**：如果只有「推进一刻」结算事件、而「按站模拟」不结算，
那么判据 #3 的等价性断言（一次推进一刻 == 逐个推进到同一时刻）**必然失败**——
两条路径会抽到不同次数的事件、走不同的重规划。等价性要求两条路径看到**同一串事件**，
所以事件结算只能挂在"软件时间前进"这件事上，与推进粒度无关。

推论与注意事项：
- 一次 `onAdvanceStop()` 若跨过多个事件刻（长途 leg），`settleEventsUpTo` 会连抽多次——这是正确的。
- 事件执行**可能触发重规划**（路况 → 重规划），从而改写 `plan_`。因此
  `onAdvanceMoment()` 的 `while` 循环条件必须在**每轮重新读取当前 `plan_`**，
  不能把 `plan_.nodeArrivalMin` 缓存在循环外。
- 抽签的随机源必须复用窗口既有的 `rng_`，且**每次事件恰消耗一次抽签**——
  两条路径的抽签次数与顺序一致，才能保证等价性断言的确定性。
- 到路线终点后不再结算（`settleEventsUpTo` 由推进触发，终点后不再推进）。

## 8. Round 3 清理清单（队长在 Round 1 复核中发现）

1. **【守卫没牙齿·已确认】** `tests/config_tests.cpp` 新增的 18 条事件键断言，**期望值全部等于代码默认值**
   （15/10/3/1/1）。因此"从文件读到"与"缺键回落"两种情形**无法区分**——
   实测：**把 `ConfigLoader` 里某个新键的解析行删掉，测试依然全绿**（w-config 已如实自报，队长复核确认）。
   修法：在测试里喂一份**非默认值**的临时 ini（例如 `event_interval_min = 7`），断言读到 7；
   再保留"缺键回落默认值"的用例。两条合起来才有牙齿。并做**反向验证**（删解析行 → 必须失败）。
2. **移除已被取代的配置键**：`traffic_change_interval_sec`、`urgent_order_interval_sec`
   （生成器 + `[general]` + `Config.h` + `ConfigLoader` + 生成器常量 + 相关测试与文档同步）。
   注意：`Config.h` 里 `urgentOrderIntervalSec` 被 `onDebugToggled` 的日志引用过，w-gui 改造后应已不再需要。
3. **断言总数口径同步**：9 个测试二进制自报断言数现为 **790**
   （core 101 / config **223** / minheap 28 / **simevent 27** / dijkstra 19 / **routeplanner 169** / integration 108 / traffic 83 / graphedit 32），
   旧文档里的 **718** 已过期。需要同步的地方：
   `AGENTS.md` §3、`README.md`、`文档导航.md`、`docs/功能实现清单.md` 附录、`docs/设计.md` 相关小节。
   **一律写明"以实测为准 + 目标数以 `ctest -N` 为准"**，并标明属于「已提交基线」还是「工作区修订」。
4. **`ctest` 目标数**：注册 `simevent_tests` 后由 19 → **20**，同样要全量同步上述文档与 AGENTS.md。
   ⚠️ **当前工作区处于预期的中间态**：`wizard_ui_names_lint` 失败——向导已引导用户点
   「按站模拟 / 按时间模拟 / 推进一刻」，但 GUI 尚未改名/新增这三个按钮。**等 w-gui 落盘即自动转绿**，
   不得通过回退向导来消红。
5. **`tools/check_wizard_claims.py`** 若仍引用被删的配置键，随第 2 条一并清掉。

### 8.1 已完成的 Round 3 项
- ✅ **修掉"守卫没牙齿"**：`tests/config_tests.cpp` 新增第 4 个用例——喂**非默认值**
  （`event_interval_min = 7`、权重 11/5/2/3）并断言读到该值。反向验证：删掉
  `event_interval_min` 的 `IntBinder` 绑定 → `FAIL 已给键取文件值 event_interval_min == 7（非默认值）`。
  断言数 217 → **223**。




