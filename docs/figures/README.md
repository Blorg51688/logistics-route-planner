# docs/figures —— 报告与 PPT 用的流程图 / UML

课程设计报告要用的全部图：**6 张函数流程图**（报告【详细设计】「主要函数的流程图」明文必需）
+ **4 张 UML 图**（PPT 补充建议「用专业 UML 工具有加分」）。

> **依据原则**：图的每个节点、字段、调用方向都指向具体的 `文件:符号名`。
> 文档与源码冲突时**以 `src/` 为准**——见 §五「文档与代码差异」。
> **用户区不放图**（准入规则见 [`文档导航.md`](../文档导航.md) §四）：图放 `docs/` 下，
> 报告排版时从这里取 PNG。

---

## 一、函数流程图（F1–F6）

| 编号 | 文件 | 图名 | 依据（源码符号名） | 对应报告 |
|---|---|---|---|---|
| F1 | [F1-minheap.png](F1-minheap.png) | 手写最小堆：入堆 / 出堆 / 上浮 / 下沉 | `src/core/MinHeap.h:MinHeap`；`src/core/MinHeap.cpp:MinHeap::push`/`popMin`/`siftUp`/`siftDown`/`swapEntries` | 【详细设计】最短路的手写堆 |
| F2 | [F2-dijkstra.png](F2-dijkstra.png) | Dijkstra 单源最短路：`dist[]`/`previous[]` + 惰性删除 + 路径复原 | `src/core/Dijkstra.cpp:shortestPath`；`src/core/WeightType.h:pickWeight`；`src/core/MinHeap.cpp:push`/`popMin` | 【详细设计】 |
| F3 | [F3-greedy-weave.png](F3-greedy-weave.png) | 贪心串联 weave：逐轮选最近、紧急优先内嵌、tie-break、`Stop` 结算、三个 `appendLeg` 失败出口 | `src/core/RoutePlanner.cpp:weave`/`appendLeg`/`beginTrip`/`buildCandidates` | 【详细设计】 |
| F4 | [F4-multitrip.png](F4-multitrip.png) | 多趟分批：紧急批 → 在途货批 → 按簇分批 → 收尾；满载出仓 + 顺路寄存 + 唯一展平点 | `src/core/RoutePlanner.cpp:multiTripPlanImpl`/`multiTripPlan`/`fillBufferFromDepot`/`bankBufferEnRoute`/`flatten` | 【详细设计】多趟与载重约束 |
| F5 | [F5-urgent-insert.png](F5-urgent-insert.png) | 紧急插单：候选 A（现状）vs 候选 B（缓冲库存就地满足）两版取优 | `src/core/RoutePlanner.cpp:insertUrgentOrder`/`replan`/`multiTripPlan`/`recordStationDraw`/`candidateDemand`/`hubOfNode`/`stockOf` | 【详细设计】动态订单插入 |
| F6 | [F6-traffic-replan.png](F6-traffic-replan.png) | 路况 → 增量式重规划（按 leg 只重算受影响段）→ 任一趟失败退回全量 | `src/gui/MainWindow.cpp:onSimulateTraffic`；`src/core/Traffic.cpp:simulateTrafficChange`/`needsReplan`；`src/core/RoutePlanner.cpp:sliceRemainder`/`replanIncremental`/`replanOneTrip`/`makeStop`/`findCandidate` | 【详细设计】路况与增量重规划 |

## 二、UML 图（U1–U4）

| 编号 | 文件 | 图名 | 依据 | 对应报告 |
|---|---|---|---|---|
| U1 | [U1-core-classes.png](U1-core-classes.png) | core 层类图：领域类型 / 图与堆 / 规划结果 / 路况事件 / 配置随机 / io | 各 `src/core/*.h` 与 `src/io/*.h`（`.mmd` 顶部注释逐条列出文件名:符号名） | 【数据结构设计】+【详细设计】 |
| U2 | [U2-gui-io-classes.png](U2-gui-io-classes.png) | gui 层 + io 层类图：`MainWindow`/`VehicleState`/`GraphScene`/`NodeItem`/`EdgeItem` + 配置链 | `src/gui/*.h`、`src/io/*.h`、`src/core/Config.h`、`src/core/RoutePlanner.h` | 【数据结构设计】模块结构 |
| U3 | [U3-seq-startup.png](U3-seq-startup.png) | 时序图：启动 → 加载配置 → 首次规划 → 画布高亮 | `src/main.cpp`、`src/io/ConfigLoader.cpp:ConfigLoader::load`、`src/io/IniFile.cpp:IniFile::loadFromFile`、`src/gui/MainWindow.cpp`（构造 / `replan` / `afterPlanReplaced` / `syncScene`）、`src/gui/GraphScene.cpp:build`/`highlightRoute` | 【详细设计】 |
| U4 | [U4-seq-traffic-replan.png](U4-seq-traffic-replan.png) | 时序图：路况触发 → 增量重规划 → 界面刷新 | 同 F6 的符号名 + `src/core/LogisticsGraph.cpp:updateEdgeTime` | 【详细设计】 |

**U1/U2 两条读法（评审常问）**：

1. `Dijkstra` / `RoutePlanner` / `Traffic` / `SimEvent` / `GraphEdit` 画成 `«utility»`，
   **因为它们不是类**——源码里是 `logistics` 命名空间下的自由函数，不存在 `class RoutePlanner`。
2. `weave` / `multiTripPlanImpl` / `replanOneTrip` / `appendLeg` / `flatten` / `makeStop` 在
   `RoutePlanner.cpp` 的匿名命名空间里、无头文件声明（`replanOneTrip` 例外：外部链接但同样没有
   头文件声明），所以 U1 只列头文件里的公开 API；这几个内部函数看 F3–F6。
3. `VehicleState` 是 `MainWindow` 的嵌套 struct，不是独立顶层类；它是**物理事实唯一权威来源**，
   重规划只读它作输入，绝不改写（`plan_` 只描述"接下来怎么走"）。

## 三、文件格式与「谁权威」

- **`.mmd` 是真相源，`.png` 是它的渲染产物。** 要改图，改 `src/*.mmd` 后重跑 `render.sh`，
  连 PNG 一起提交；不要直接改 PNG（改不了，也查不出 diff）。
- `.mmd` 是纯文本、可 `git diff`，改图不用重画；PNG 宽 1600px、白底，报告可直接插图。
- 选 mermaid 而非 StarUML / PlantUML 的理由：源文件是纯文本，便于版本管理与评审；
  PNG 满足报告插图需求；若后续要更"专业"的 UML 记号，可从 `.mmd` 快速迁移到 PlantUML。
- **引用这些图时沿用「文件名:符号名」，不要用行号。** 本批图 2026-10-18 收编时就是因
  `F1` 写了 `MinHeap.cpp:44` / `:57` 而返工——那两行实际落在 `siftDown` 内部
  （`MinHeap::siftUp` 与 `siftDown` 各自是完整函数）。行号会漂，符号名不会。

## 四、重新渲染

```bash
bash docs/figures/src/render.sh          # 全量重渲 10 张 → docs/figures/*.png
```

依赖：`npx`（随 Node.js）+ `chrome-headless-shell`。本机首次使用前装一次：

```bash
npx -y puppeteer browsers install chrome-headless-shell
```

脚本会在 `~/.cache/puppeteer/` 下自动查找；找不到会给出上面这条可操作提示并退出（不静默失败）。
渲染参数固定为 `-w 1600 -b white`，因此同一份 `.mmd` 在不同机器上应得到同一张 PNG
（本机实测：重渲 U3 与仓库内 PNG 字节数一致）。

## 五、文档与代码差异（图以代码为准）

以下几处 `docs/设计.md` 的文字描述已滞后于源码实现，图的画法以 `src/` 为准：

1. **中转站不参与排线**：`设计.md` §5.6 仍保留 `stock[t]` 中转站暂存分支、`枢纽 = 中转站 t`
   代码块，但 `RoutePlanner.cpp:multiTripPlanImpl` 已删除该分支（§16 P25/P30）。
   F4 按实际实现画——每趟从仓库直达后返回，不经中转站；`sub_network_id` 只用于**簇划分**。
2. **中转站暂存不变量**：`设计.md` §5.5/§5.6 不变量表 #3/#4 仍写"中转站暂存 ≥ 0 /
   规划结束时 == 0"，但代码里 `TransitStock` 的用途已变为"仅服务紧急单的缓冲库存"
   （2026-09-26 用户裁定恢复，D23），语义不同。U1 按实际字段画。
3. **紧急插单的取优判据**：`设计.md` §5.3 未提及 A/B 取优、penalty 一票否决、
   车上缓冲就地满足等机制，这些在 `insertUrgentOrder` 里是核心逻辑。F5 按源码画全。
4. **增量重规划的适用范围**：`设计.md` §5.7「范围限制」写的是"只适用于单趟路线，
   上一版是多趟时直接退回全量"，但 `replanIncremental` 已改为**逐趟增量**
   （任何一趟失败才整体退回全量）。F6 按当前实现画。

## 六、事实与口径

- 所有图的事实取自 `src/` 与 [`docs/设计.md`](../设计.md) §4/§5/§6，未新编。
- 图里的数字（30 节点 / 106 有向边 / 25 订单 / 740kg 总需求 / 车 200kg·08:00 /
  事件权重 10:3:1:1 / 事件刻 15min）均来自 `config/default.ini` 与当前源码，
  属**已提交基线**口径。
- `ctest` **目标数 20**（`ctest -N` 为准）与测试二进制**自报检查项 838 项**是两个口径，不许混。
- 图不是功能清单：每张图都落在「结构怎么支撑调度决策」上——堆支撑"取最小"、
  前驱数组支撑路径复原、`trips` 真源支撑逐趟增量、`TrafficReport` 支撑"只重算受影响段"的判定。
- 结构侧的完整依据（为什么选这个结构、代价多大、没有它得自己写什么）见
  [`docs/数据结构地图.md`](../数据结构地图.md)；本目录只放图本身。
