# 实现规格：紧急单"缓冲库存"就地满足（transit-urgent-buffer）

> 来源：`docs/留档/决策/transit-stock-full-load.md`（四轮商议记录）。
> 结论已由用户裁定为**做**（第四轮）：前置 A = **读法 2**（每次满载出仓强制带一部分
> **不属于任何订单的缓冲货**，它是站内库存与车上盈余的唯一来源）；前置 B = 本规格强制的
> **两版取优 + penalty 一票否决 + 四条守卫**。
>
> 本文件是**规格**，不是已实现的功能。实现由**另一个对话**负责。

---

## 0. 一句话规格

**每次出仓装满载重上限；其中"本趟订单用不到的那部分"记作独立的 `bufferKg`（缓冲货，不属任何订单）。
返程若顺路经过本簇中转站，就地把缓冲卸进站里（零节点/零里程/零时刻改动）。
出现紧急订单时，先用"车上剩余缓冲 + 紧急点所属站内库存"就地满足；若两版取优判定不划算，
退回现状"独立紧急趟回仓库取货"。**

### 建模前提（用户 2026-09-26 裁定，实现不得悄悄改）

> 「每次出仓强制满载就会有一部分**不属于任何订单的缓冲货**，这些货就是中转站存货的来源和盈余的供给者。」

因此：
- **允许**仓库发出不属于任何订单的缓冲货（= 安全库存/不记名库存）。
- **未消耗的缓冲货** = 被多发出的货；这是**已知并接受的建模代价**，但必须在文档里如实写明
  （结算口径：`Σ发出缓冲 − Σ紧急单选消耗 = 期末站内存货 + 期末车上缓冲`，必须逐位守恒）。

---

## 1. 完成判据（每条给验证命令）

| # | 判据 | 验证命令 | 通过标准 |
|---|---|---|---|
| C1 | 能构建 | `cmake --build build -j4` | 退出码 0，无 warning 新增 |
| C2 | 全量回归 | `(cd build && ctest)` | `100% tests passed`；`(cd build && ctest -N)` 目标数 **仍为 20**（不新增测试二进制 ⇒ 不改 `CMakeLists.txt`） |
| C3 | **黄金值三条逐位不变** | `for s in distance cost time; do ./build/app --plan-summary $s; done` | 与 `AGENTS.md` §3 三行**逐位一致**（178.2/378.3/244.2/166/6；186.3/461.0/235.9/207/6；183.2/372.1/255.2/201/6） |
| C4 | 无紧急单时新机制是 **no-op** | 新增断言（见 T5-G2）：空 stock/空 buffer 调用 vs 启用调用 | `totalDistanceKm`/`totalTimeMin`/`totalCostYuan`/`totalPenaltyMin`/`trips.size()` **逐位相同** |
| C5 | 交互性不变量不回归 | `./build/app --self-check-actions` | 全部通过，且输出中出现新增的缓冲/库存检查行 |
| C6 | 界面构成 | `./build/app --ui-probe` | 通过；中转站面板由 3 列变 **4 列**（新增「当前库存」），且第 4 列初值为 0 |
| C7 | 紧急单就地满足真的发生 | `./build/app --self-check-actions` 的输出（新增行） | 至少 1 次紧急单由「车上缓冲 + 站内库存」满足，且打印省下的 km |
| C8 | 守卫有牙齿 | 逐条反向验证（见 §5） | 每条守卫在回退修复后**必须失败**，贴原始失败输出 |

---

## 2. 数据与 API 设计（**只加字段，不改语义**）

关键约束：**缓冲货不得混入 `Trip::loadKg`**。理由：`loadKg` 是既有的"订单货量"口径，
被界面「本趟装载/送后余载」、`--self-check-actions` 的装载一致性守卫、`integration_tests` 的容量守卫共用；
把它改成载重上限会污染这些口径。故：

```cpp
// src/core/RoutePlanner.h
struct TransitOp {                 // 一次站内装卸（新增）
    std::string nodeId;            // 中转站
    double      kgDelta = 0.0;     // >0 入库（寄存），<0 出库（紧急单取用）
    int         atMin   = 0;       // 发生的软件内时刻（供界面按时刻回放）
};

struct TransitStock {              // 站内库存快照（新增，供界面与测试核对）
    std::string nodeId;
    double      initialKg = 0.0;
    double      finalKg   = 0.0;
};

// struct Trip：新增
double bufferKg = 0.0;             // 本趟装载的"不属于任何订单的缓冲货"（不参与 loadKg）
std::vector<TransitOp> bankOps;    // 本趟在返程顺路处的入库操作（至多一条）

// struct RoutePlan：新增
std::vector<TransitStock> transitStock;   // 各站期末库存
std::vector<TransitOp>    transitOps;     // 展平后的全部站内装卸（按时刻）
```

**不变量（必须由断言守住）**：

```
I1  trip.loadKg + trip.bufferKg <= vehicle.capacityKg + 1e-9      （缓冲不超载）
I2  trip.bufferKg >= -1e-9                                        （缓冲非负）
I3  Σ(所有 bankOps.kgDelta) + Σ(所有第 5 类消耗) == Σ transitStock.finalKg - Σ initialKg
I4  transitStock.finalKg >= -1e-9                                 （库存非负）
I5  任何 bufferKg>0 的趟，其 bufferKg == capacity - loadKg 的剩余部分（即"装满"的确切含义）
```

**签名穿透**（与历史 `3b27114` 被删字段同名同形，便于对照）：

- `replan(..., const std::map<std::string,double>& initialStock = {})`
- `insertUrgentOrder(..., const std::map<std::string,double>& initialStock = {})`
- `replanIncremental(..., const std::map<std::string,double>& initialStock = {})`
- `multiTripPlanImpl(..., const std::map<std::string,double>& initialStock)`

**不加配置项**（不需要新 `[general]` 开关；启用/停用通过传空 `initialStock` 实现，
避免本项目历史上出过的"死配置"问题）。

---

## 3. 生产者：满载出仓 + 返程顺路寄存（零成本，纯记账）

### 3.1 规则

对 `multiTripPlanImpl` 产出的**每一趟**：

1. `trip.bufferKg = capacity − Σ(本批订单货量)`（即把车装满；`I5`）。
2. 求本趟停靠点所属子网络的**唯一**簇站 `H`（`subNetworkId → Transit` 映射，`RoutePlanner.cpp:290-295`；
   若一趟跨簇，或该簇无站，则 `H` 为空 ⇒ **不寄存**，缓冲跟车回仓库）。
3. 取本趟**最后一个停靠点** `last`（在 `trip.nodes` 中 `nodeIsStop` 为真的最后一个）。
4. **顺路判定**（必须逐位成立才寄存）：
   `dist(last, H) + dist(H, startNode) ≤ dist(last, startNode) + 1e-6`
   （即 `H` 落在回程最短路线上）。不成立 ⇒ **不寄存**，缓冲跟车回仓库。
5. 成立 ⇒ 记一条 `bankOps = {H, +trip.bufferKg, atMin = H 在该趟序列中的到达时刻}`，
   并把它并入 `plan.transitOps`。

> **为什么这是零成本**：`appendLeg`（`RoutePlanner.cpp:67-95`）会把最短路上的**全部途经节点**写入
> `trip.nodes`。所以"回程顺路"意味着 `H` **已经在序列里**——寄存不新增任何节点，
> 该趟的 `totalDistanceKm`/`totalTimeMin`/`totalCostYuan`/`nodeArrivalMin` 一律不变
> （本项目无停靠服务时间，`66e282e` 已移除）。**距离/耗时/成本/penalty 一个字节都不动。**

### 3.2 不做的事

- ❌ **不得**让常规趟为了寄存或取货而绕路（那是本文档第一/二轮否证过的 +10km/客户）。
- ❌ **不得**恢复历史上的 `useStation` 常规路由分支（"有货就用"会算错，范本 §2.4(b) 实测）。
- ❌ **不得**为了让寄存发生而改装箱/改趟次。

---

## 4. 消费者：紧急单三链 + 两版取优（含 penalty 一票否决）

在 `insertUrgentOrder`（`RoutePlanner.cpp:859+`）内部，对**传入的那一张紧急单**：

```
d   = 紧急单需求
U   = 紧急单节点；H = U 所属子网络的中转站（可能为空）
pos = 车辆当前位置；T = 当前时刻

候选 B（本提案）：
  avail = 车上剩余缓冲（当前趟 bufferKg 减去本趟已消耗）
  need  = d - avail
  if need <= 0            -> 取货点 = 车上（不新增节点），就地满足
  else if H 非空 且 stock[H] >= need  -> 取货点 = H（顺路；不要求零绕路，允许绕，由取优裁决），取 need
  else if H 非空 且 stock[H] > 0       -> 只取 stock[H]，若 avail+stock[H] >= d 则取货点 = H
  else                    -> 候选 B 不可用（退回 A）

候选 A（现状，兜底）：独立紧急趟：pos -> 仓库装货 -> U -> ...

取优（构造性保证"绝不更差"）：
  A = 现状版本；B = 候选 B 版本（若可用）
  if (B.penaltyMin > A.penaltyMin)  -> 选 A                // penalty 更差：一票否决
  else if (B 的目标值 < A 的目标值) -> 选 B
  else if (目标相同 且 B.trips.size() < A.trips.size()) -> 选 B
  else                              -> 选 A
```

**E3 优先性不得被取优牺牲**：无论选 A 还是 B，紧急批次都排在**所有普通停靠点之前**
（沿用 `multiTripPlanImpl:297-349` 的独立紧急批结构）。候选 B 只是把该批的**取货点**
从"仓库"换成"车上缓冲 / H"，不得改变"紧急在先"。

**消耗记账**（必须同时做，否则 `I3` 失败）：
- 用了车上缓冲 ⇒ 车辆状态 `bufferKg_ -= used`；该趟 `bankOps` 的入库量相应减少。
- 用了站内库存 ⇒ `stock[H] -= take`，并在 `transitOps` 追加一条 `{H, −take, atMin=T}`。

---

## 5. 文件级任务拆解（相互独立，边界清晰）

| 任务 | 写入范围 | 内容 | 依赖 |
|---|---|---|---|
| **T1** 数据与 API | `src/core/RoutePlanner.h`、`src/core/RoutePlanner.cpp`（仅签名与结构，不动算法） | §2 的 `TransitOp`/`TransitStock`/`Trip::bufferKg`/`Trip::bankOps`/`RoutePlan::transitStock`/`transitOps`；四条签名穿透 `initialStock` | — |
| **T2** 生产者 | `src/core/RoutePlanner.cpp`（`multiTripPlanImpl` 的趟收尾处 + `multiTripPlan`） | §3 的满载与顺路寄存 + `transitStock` 汇总 | T1 |
| **T3** 消费者 | `src/core/RoutePlanner.cpp`（`insertUrgentOrder` 与其调用的 `multiTripPlanImpl` 紧急批段） | §4 的三链 + 两版取优 + 消耗记账 | T1、T2 |
| **T4** 界面与状态 | `src/gui/MainWindow.h`、`src/gui/MainWindow.cpp` | `VehicleState` 新增 `stationStock_`、`bufferKg_`；三个状态转移（装货含缓冲 / 到站卸缓冲 / 紧急单消耗）；三处规划出口透传并回写期末库存；中转站面板 3 列 → 4 列（「当前库存」） | T1 |
| **T5** 测试与守卫 | `tests/routeplanner_tests.cpp`、`tests/integration_tests.cpp`、`src/main.cpp`（`--self-check-actions` 增行） | §4 的 G1–G6 守卫 + 反向验证 | T1–T4 |
| **T6** 文档事实 | `docs/设计.md`（**只加注记，不重写历史**）、`docs/功能实现清单.md` | 见 §7 | T3、T5 |

**T6 必须在 T1–T5 全绿之后做**；`docs/设计.md` §16 是历史日志，只加「注（日期）」不删原文。

---

## 6. 不许动的文件（硬边界）

| 路径 | 原因 |
|---|---|
| `要求源文件/` | 老师给的原始文件，只读 |
| `docs/人工测试记录.md` | 陈旧记录，需真人在终端重跑向导才会刷新；不得手改 |
| `config/default.ini` | 本规格不需要新配置；改它会影响黄金值口径 |
| `CMakeLists.txt` | 不新增测试二进制（守卫并入既有 `routeplanner_tests`/`integration_tests`） |
| `tools/check_*.py`、`tools/check_*.sh` | 既有外围守卫**只能新增、不得改弱**（不得为了让自己通过而放宽） |
| `src/core/RoutePlanner.cpp` 的**常规排线**路径 | 不得加入任何"经站路由"分支（见 §3.2） |
| `MainWindow` 的既有状态转移语义 | 只允许**新增**状态与转移；"只有车真的做了什么才能改它"（P25）不得违反 |

---

## 7. 回归不变量（改坏了必须立刻发现）

**三策略黄金值（必须逐位不变）**：

```
distance  178.200km / 378.300min / 244.200元 / penalty 166 / 6 趟
cost      186.300km / 461.000min / 235.900元 / penalty 207 / 6 趟
time      183.200km / 372.100min / 255.200元 / penalty 201 / 6 趟
```

`ctest` **20/20 全绿**；9 个测试二进制自报断言合计**从 790 起只增不减**（新增守卫 + 反向验证用例）。
`--self-check-actions` 全部通过。`--ui-probe` 通过且中转站面板为 4 列。

**文档同步（T6）**：`docs/设计.md` 的 **D23 必须修订**——它现在写着"不恢复（方案 A）"，
与本规格冲突；修订须**引用用户 2026-09-26 的裁定原话**并注明本规格路径，
**不得悄悄改口径**。D16 的「注（2026-09-26）」也要补一句"缓冲货机制已于本轮按用户裁定恢复，
但仅用于紧急单，常规排线仍不进站"。`docs/功能实现清单.md` B1.5 的 ⚠️ 需复核后据实更新。

---

## 8. 新增守卫清单（每条附反向验证方式）

> 纪律：守卫必须读**用户能看到的东西**（界面文本、落盘/打印的规划产物），不得自己把公式重算一遍去断言。

| # | 守卫 | 断言什么 | 反向验证（把修复回退，守卫必须失败） |
|---|---|---|---|
| **G1** | 常规趟绝不进站 | 对**无紧急单**的规划：①`plan.transitOps` 中**不含** `−`（出库）操作；②每趟的 `bankOps` 所指向的站，其 `atMin` 必须等于该趟序列中该站的 `nodeArrivalMin`（证明是"顺路"而非绕路） | 让 `multiTripPlanImpl` 在回程不顺路时也记一条 `bankOps` ⇒ G1② 必须报"站点不在该趟序列里/时刻对不上" |
| **G2** | 寄存零成本（no-op） | 同一输入，`initialStock={}` vs 有存货：`totalDistanceKm`/`totalTimeMin`/`totalCostYuan`/`totalPenaltyMin`/`trips.size()` 逐位相同 | 在寄存处顺手改一次 `appendLeg` 的入参 ⇒ G2 必须报距离不等 |
| **G3** | 库存守恒 | `ΣbankOps.kgDelta + Σ(−消耗) == Σ(期末 − 期初)`；且任何时刻 `stock ≥ 0`；且**库存变动只发生在车真的到过该站**（`atMin` 可追溯到某趟的 `nodeArrivalMin`） | 让某次紧急单取用不记账（只减 `stock` 不写 `transitOps`）⇒ G3 必须报不守恒 |
| **G4** | 缓冲不超载 + 非负 | 每趟 `loadKg + bufferKg ≤ capacity + 1e-9`；`bufferKg ≥ -1e-9`；（既有 `loadKg ≤ capacity` 断言**保持不动**） | 把 `bufferKg` 设为 `capacity`（不扣本批订单量）⇒ G4 必须报超载 |
| **G5** | E3 紧急单优先性 | 插单后：紧急单的 `arrivalMin` ≤ **所有**普通停靠点的 `arrivalMin` | 把取优改成"谁近选谁"（允许普通停靠先于紧急单）⇒ G5 必须报"紧急单不再是第一个送达" |
| **G6** | 绝不更差（取优守卫） | 含紧急单场景：`chosen.penaltyMin ≤ variantA.penaltyMin`；且 `chosen` 在 (目标值, penalty, 趟数) 三元序下不劣于 A | 去掉 penalty 一票否决 ⇒ 构造"B 的 penalty 更差但距离更好"的用例 ⇒ G6 必须失败 |
| **G7** | 界面口径 | `--ui-probe`：中转站面板 4 列且第 4 列为数字；车辆面板「本趟装载」仍 ≤ 载重上限（缓冲不混入该列） | 把缓冲加进界面「本趟装载」列 ⇒ G7 必须报 > 载重上限 |

**守卫不得挂住**：任何自写循环必须有迭代上限；无进展即退出（本项目既有纪律）。

---

## 9. 实现顺序与验收节奏

1. T1 → 编译通过、`ctest` 仍 20/20 全绿（纯加字段，零行为变化）**先交一次**。
2. T2 → G1/G2/G4 上线并全部反向验证；黄金值三条仍逐位一致。
3. T3 → G3/G5/G6 上线并全部反向验证；`--self-check-actions` 新增 C7 的打印。
4. T4 → G7 上线；`--ui-probe` 通过。
5. T5/T6 → 断言数更新、`docs/设计.md` 的 D23/D16 注记与 B1.5 复核。

**每一阶段都必须先跑**：`cmake --build build -j4` → `(cd build && ctest)` → 三策略黄金值对比。
**任何行为改动后数字变了就必须解释为什么**，并同步断言与文档。

---

## 10. 已知风险与回退预案（实现时必须知道）

| 风险 | 症状 | 预案 |
|---|---|---|
| 库存单调膨胀 | 期末库存无限增长、后续规划被污染（P25 实测 870kg） | G3 守恒 + 期末库存纳入 `transitOps` 回放；若膨胀仍出现 ⇒ **停在本阶段**并回报，不得"顺手调参"掩盖 |
| "有货就用"算错 | 常规趟被引向中转站、距离变差 | G1 + §3.2 硬禁令；取优只用于**紧急单**处置 |
| 缓冲混入订单口径 | 界面「本趟装载」显示 200、「已送达」kg 对不上 | §2 的字段隔离 + G4/G7；P26 的"车送它没装在车上的货"守卫必须继续通过 |
| 紧急单被延误 | 紧急单不再是第一个送达 | G5 + §4 的取优只用 penalty 一票否决，不得交换优先性 |
| 与 P25 的 `VehicleState` 冲突 | 站内库存变成"第二份物理事实"，重规划改写已发生事实 | 库存变动的三处入口全部收敛到 `VehicleState` 的状态转移（T4），并沿用 P25 的守卫（重规划绝不改写车辆状态，三条路径都要测） |

---

*本规格由评审伙伴产出，未改任何源码。实现前请用 `git show --numstat` 复核 §5 各任务的真实量级；
若实现中发现本文档与代码事实冲突，**以代码事实为准并回报**，不要静默改设计口径。*
