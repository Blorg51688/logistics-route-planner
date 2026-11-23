# PDP / 多车取送 / 中转站 / 动态插入：primary-source 调研笔记

> **日期**：2026-11-23（Asia/Shanghai）
> **范围**：带容量与时间窗的取送问题（PDPTW）、多车取送（MDPDP）、中转站网络、动态订单插入。
> **用途**：为「订单统一为 A→B 取送、车辆空载出发、多车 + 中转站」的两阶段重构提供可落地的算法依据。
> **方法**：OpenAlex / Semantic Scholar 核对卷期页码与摘要；开放获取综述全文（Parragh Part II、Cordeau & Laporte 2003）逐节阅读；付费论文只读摘要与综述转述，未逐篇通读全文（限制见 §7）。
> **仓库依据**：[设计.md §4/§5](../../设计.md)、[ADR 0001](../../adr/0001-urgent-pickup-delivery.md)、[records/specs/](.)、[中转站策略分析.md](../../中转站策略分析.md)。

---

## 0. 结论先行

1. **阶段一（单车、全 A→B）用「带惩罚的 cheapest pair-insertion」做主构造器**：实现简单、天然支持动态插单、可解释；最近邻只作基线，regret 插入作为质量升级项。
2. **阶段二（3 车）用「逐单跨车 cheapest insertion + 成对 relocate 修复」**，比 cluster-first 更适合本项目的杂乱地理与少量车辆；拍卖在多车同属一个决策者时**退化为贪心分配**，只宜作叙事，不宜作核心。
3. **中转站不要做 hub-and-spoke / two-echelon 路由**：这两类模型的收益依赖「枢纽间规模经济 + 仓库供货」，而本项目是逐边计费、货全部来自客户 A，两个前提都不成立；仓库自测也已否证（178.2 vs 198.2 km，6 vs 13 趟）。
4. 中转站唯一站得住的模型是 **PDP with transshipment（接力/换装）**：收益只能来自「车辆路线重叠、装卸解耦等待、容量腾挪、动态改派」，**不可能来自缩短货物路径**（三角不等式下 `d(A,S)+d(S,B) ≥ d(A,B)`）。
5. **动态插入**：紧急单立即插入、普通单按事件刻批量缓冲；插入后做有界局部修复；全量重规划只作兜底/定期对照，并用「两版取优 + penalty 一票否决」保证不劣化。
6. **重构会作废旧黄金值**：现有 178.2/378.3/… 依赖「仓库供货 + 多趟装货」，新语义下必须重新冻结基准（这一点必须先写进规格）。

## 1. 与仓库现状的映射（规格设计前必须对齐）

| 维度 | 现状（HEAD） | 目标语义 | 对算法的直接影响 |
|---|---|---|---|
| 订单/车辆 | 送达点 B + 需求量；每趟到仓库装货 | **全 A→B 取送对**；车辆**空载出发**、仓库只是基地 | 每条请求带 (A,B,q,窗口)；载重变成**路线前缀载荷** |
| 仓库 | 供货点、满载往返补给 | **仅基地**（start/end） | 旧的「多趟回仓补货」机制失去对象；一条连续路线可服务任意多请求 |
| 中转站 | 仅紧急降载暂存，不参与排线 | 可作换装/暂存点（货必须是车辆实际卸下的） | 只有 PDP-T/relay 有意义；站内「无主库存」不再可能 |
| 车辆数 | 1 | 3（阶段二） | 需要订单→车辆的分配与跨车修复 |
| 事件/基准 | 紧急 20% > 普通 6.7%；基准 178.2/378.3/… | 普通小概率、紧急更小；语义已变 | 权重需重设（§5）；**旧黄金值作废、需重新冻结** |

> 关键句：新模型是**一对一的 PDPTW**（每条请求一个取货点 A、一个送达点 B），不是「仓库→客户的 VRP」；文献中的 depot 只对应「车辆基地」，不供货。分类见 Berbeglia et al. 2007 的 one-to-one / many-to-many 划分。

## 2. 发现（Findings）

### 2.1 Q1：带容量与时间窗的取送问题，哪些启发式适合手写 C++？

**问题背景**：PDPTW 是 NP-hard（VRPTW 为其特例）；精确法只能做到几十个请求，课程规模必须用构造 + 局部搜索。综述见 Savelsbergh & Sol (1995)、Berbeglia et al. (2007)、Parragh et al. (2008 Part II)。

#### (a) 最近邻贪心（nearest neighbor）
- **做法与候选集**：从当前位置出发，在「所有未服务请求的 A」与「已取未送的 B」中，选**最短路权重最小且可行**的下一站。Daganzo (1978) 的就近策略是早期原型；Solomon (1987) 把 NN 作为 VRPTW 构造基线之一。候选集可用「最近 k 个点」裁剪，但 k 太小会漏掉唯一可行点。
- **复杂度**：每步 O(n) 候选、共 O(n) 步 → **O(n²)**；若未预计算最短路则退化为 O(n²·(E log V))。建议先做**全源最短路矩阵**（手写 Dijkstra 跑 V 次，V≈30–100 可接受），之后边权 O(1) 查表。
- **陷阱与适配**：只看距离会先取「近但送达窗口很远」的货把车堵死；无回退易在最后几个请求上无路可走。适合做**基线/对照**与紧急直送兜底，不建议做唯一主算法。

#### (b) Cheapest insertion（最便宜插入）
- **做法**：维护一条（或多条）路线；每次取一条未插入请求 (A,B)，枚举 A 的插入位置 i 与 B 的插入位置 j（j 在 A 之后），计算增量代价，取最小者插入。Solomon (1987) 的 I1 用 `c1 = α1·c11 + α2·c12`（`c11` 距离增量、`c12` 对后继点的时间推移）选位置；PDP 的成对插入在 Lu & Dessouky (2006) 中被系统化，评价除距离外还加入**时间窗松弛量下降**与**交叉长度百分比**。
- **候选集**：所有 (i,j) 位置对；可用「A 的最近 k 个停靠点 × B 的最近 k 个停靠点」裁剪到 O(k²)。
- **可行性**：容量用**前缀载荷**（取货 +q、送货 −q，任一时刻 ≤ C）；先取后送由 i<j 保证；时间窗从 i 开始**重放**到路线尾（等待、到达、penalty），软窗把 penalty 增量并入目标。
- **复杂度**：单条 n 站路线、单个请求：O(n²) 个位置对 × O(n) 重放 = **O(n³)**；m 条请求 → O(m·n³)（n 随插入增长，最坏约 O(m⁴)）。用前向/后向时间松弛预计算可把每次检查降到 O(1)，位置对枚举降到 **O(n²)/请求**（Campbell & Savelsbergh 2004 给出插入效率框架：基本 O(n³)，朴素处理复杂约束会到 O(n⁴)，精心实现可保持 O(n³) 或 O(n³ log n)）。
- **常见陷阱**：插入 A 后 B 的下标会移一位（建议**直接构造候选序列再重放**，不要做下标算术——本项目历史上多次因「聪明记账」出缺陷）；容量必须查前缀而非总量；等待可能让「距离增量小」的位置实际更晚；penalty 对插入**非单调**（插入有时会吸收等待、有时会推迟后续），必须重放；浮点比较要容差；并列时按 ID 稳定决胜。
- **课程适配**：**最推荐**。逻辑直白、可逐行解释、天然支持「新单插入」，且不需要 STL 算法（最小值用手写扫描即可）。

#### (c) Regret insertion（遗憾插入）
- **做法**：对每条未插入请求，算它在所有位置/车辆上的最优代价 c1 与次优 c2（regret-2 = c2 − c1，可扩展到 regret-k）；优先插入 regret 最大者——它「再拖会明显变贵」。Potvin & Rousseau (1993) 在并行路线构造中提出 generalized regret；Diana & Dessouky (2004) 用于大规模 DARP（1000 请求）并报告优于其他插入规则。
- **复杂度**：单轮 O(m·k·n³)；插入后其余请求代价会变，可缓存并只重算受影响路线，摊还 O(m·k·n²)。**陷阱**：不重算缓存会用过期代价；k 过大退化贪心；urgent 与普通混排要先分层。**课程适配**：作为阶段一的质量升级（先 cheapest、测出差距再上）。

#### (d) Cluster-first route-second（先分簇、再路径）
- **做法**：先把请求分成 k 簇（按 `sub_network_id`、极角 sweep、或手写 k-means / 容量约束的 GAP 分配），每簇再跑单车构造。Fisher & Jaikumar (1981) 的 GAP 分配是经典：把「客户→车辆」建成广义分配，目标近似路线成本，且「只要存在可行解就一定能找到」；DARP 文献（Desrosiers et al. 1988、Borndörfer et al. 1997）用 mini-cluster + 路由两阶段。
- **陷阱与复杂度**：纯距离分簇是**时间窗盲**的，可能把窗口冲突的点分进同簇；A、B 必须**同簇**（成对请求不能拆）；簇内不可行需要再平衡。分簇 O(m²)~O(m log m) + 每簇 O(m_c·n_c³)。**课程适配**：适合阶段二做初始解/可视化，配跨簇 relocate 修边界。

#### (e) 统一可行性检查（所有构造器共用）
```
检查(候选路线):
  1) 先取后送：每条请求 B 的位置 > A 的位置
  2) 容量：前缀载荷 prefix[t] = prefix[t-1] + (A 处 +q / B 处 -q)，要求 0 ≤ prefix[t] ≤ C
  3) 时间：从路线头按 §4.3 递归重放（rawArrival → wait → arrival → penalty → departure）
     - 硬窗：arrival ≤ windowEnd；软窗：允许超时、累计 penalty（与执行阶段同一函数，D13 口径）
  4) 全部请求恰好服务一次；路线起止于基地
```
**复杂度**：一次完整检查 O(n)；插入枚举里每次检查 O(n)，这就是 O(n³) 的来源；预计算「每个点的最晚可延时间 slack」可把增量 penalty 检查降到 O(1)。

#### (f) 候选集裁剪（工程要点）
- 位置对裁剪：只考虑 A 的「前 k 近停靠点之后」与 B 的「前 k 近停靠点之前」，k≈8–15；保留**全量扫描兜底**（小规模 n≤50 直接全扫）。
- 请求排序：urgent 先、普通按「最早送达窗止（EDD）」；同层按 ID 稳定排序（手写插入排序）。依据：Solomon (1987)、Lu & Dessouky (2006)、Campbell & Savelsbergh (2004)。

**Q1 小结表**

| 方法 | 候选集 | 可行性检查 | 复杂度（单请求/整体） | 主要陷阱 | 课程适配 |
|---|---|---|---|---|---|
| 最近邻 | 未取 A + 已取 B | 容量/先取后送/窗口 | O(n²)/路线 | 短视、易堵死 | 基线/兜底 |
| Cheapest insertion | 所有 (i,j)，可 top-k 裁剪 | 前缀载荷 + 重放 | O(n³)/请求；O(m n³) | 下标位移、penalty 非单调 | **首选** |
| Regret insertion | 同 insertion，需 top-k | 同上 | O(m k n³)/轮；缓存后 O(m k n²) | 缓存过期、k 过大 | 质量升级 |
| Cluster-first | 先簇后位置对 | 分簇容量 + 簇内重放 | O(m²)+ΣO(m_c n_c³) | 窗口盲、边界切解 | 阶段二初始解 |

---

### 2.2 Q2：多车取送（MDPDP）的订单分配 + 路径规划

**文献图景**（Parragh Part II 表 3/4 与 Cordeau & Laporte 2003 综述）：
- **顺序插入**：Jaw et al. (1986) 对多车 DARP 逐请求插入，规模到 2617 客户/28 车——最早、最可解释的多车构造法。
- **并行插入**：Toth & Vigo (1996, 1997) 先并行分配请求到路线，再做 intra/inter-route exchange。
- **cluster-first route-second**：Desrosiers et al. (1988)、Borndörfer et al. (1997) 等 mini-cluster + 路由两阶段，适合地理分簇明显、规模大的场景。
- **regret 插入**：Diana & Dessouky (2004) 的并行 regret 插入（1000 请求）在多车 DARP 上优于其他插入规则。
- **两阶段（先减车、再减成本）**：Bent & Van Hentenryck (2006) 第一阶段模拟退火减少车辆数、第二阶段 LNS 降总里程，在 200/600 客户基准上分别改进 47%/76% 的最好解。本项目车辆数固定为 3，第一阶段意义有限。
- **邻域修复**：Cordeau & Laporte (2003) 的 tabu search 用「移除一条请求对 (A,B) → 重插到另一辆车」的**成对 relocate** 邻域，并允许中间解违反容量/时长/时间窗（自适应罚系数 `f(s)=c(s)+αq+βd+γw+στ`），比「严格可行插入」更容易脱离局部困境。
- **ALNS**：Ropke & Pisinger (2006) 多种破坏/修复子启发式自适应选择，350 个基准（最多 500 请求）改进 50% 最好解；课程层面可简化为「移除若干请求再 cheapest 重插」。
- **拍卖/市场**：Figliozzi et al. (2004) 的 sequential second-price auction 用于动态 truckload PDP（四种分配技术、one-step look-ahead）。**关键洞察**：本项目 3 辆车同属一个决策者，「竞价」等于「每辆车报告自己的边际插入成本」，最终仍是**最小边际成本分配**——拍卖只增加叙事与实现复杂度，不改变最优性；真正需要拍卖的是多主体、有自私目标的市场。

**适用条件与复杂度**

| 方法 | 适用条件 | 复杂度（k 车、m 请求） | 可解释性 | 课程适配 |
|---|---|---|---|---|
| 逐单贪心分配 | 车少、请求序合理 | O(m·k·n³) 朴素；slack 后 O(m·k·n²) | 高（“谁最便宜谁接”） | **首选** |
| 聚类后分配 | 地理/子网络分簇明显 | O(m²)+ΣO(m_c n_c³) | 高（可画图） | 初始解/备选 |
| 拍卖/市场 | 多主体、有报价/激励 | 每轮 O(m·k·n³) + 竞价博弈 | 中（需解释机制） | 不推荐做核心 |
| 两阶段（减车+降本） | 车辆数可压缩 | 两倍以上构造+搜索 | 中 | 车辆数固定时不划算 |
| 成对 relocate 修复 | 任意初始解 | 每轮 O(m·k·n²·n) | 中高（“把单挪到更合适的车”） | **推荐叠加** |

**Q2 结论**：3 辆车、可解释、手写 → **「逐单跨车 cheapest insertion + 成对 relocate/swap 修复」**最合适；cluster-first 只作为初始解或可视化手段；拍卖只作为报告里的机制讨论。

---

### 2.3 Q3：中转站网络——hub-and-spoke / two-echelon / cross-docking / relay

#### (a) 四类模型及其隐含前提
- **Hub-and-spoke**：枢纽作为 switching/transshipment/sorting 点，O-D 流在枢纽汇合；经典 hub location 模型假设**枢纽间运输存在规模经济**（折扣系数），并据此选址与分配（Alumur & Kara 2008 综述）。
- **Two-echelon（2E-CVRP）**：depot → satellite → customers 两级；货物在 satellite **集中/合并**后再由小车配送（Perboli, Tadei & Vigo 2011）；其收益同样依赖「第一级大批量、第二级小批量」的规模经济与仓库供货。
- **Cross-docking**：收货 → 拆解/分拣 → 合并 → 装车发运；需要 dock 门、暂存区、进出同步（Van Belle et al. 2012 综述；2019 综述把「temporary storage capacity / doors / synchronization」列为核心属性）。
- **Relay / PDP with transshipment（PDP-T）**：一条请求可由两辆车接力完成——第一辆在 A 取货、送到换装点 S 卸下；第二辆在 S 装上、送到 B（Mitrović-Minić & Laporte 2006）。DARP 的 transfers 版本（Masson, Lehuedé & Péton 2013）把换乘点与同步约束显式建模。

#### (b) 在「货全部来自客户 A、仓库只是基地」前提下的适配性
- **Hub-and-spoke / two-echelon 不成立**：两者收益依赖「多 O-D 流枢纽合并 → 干线规模经济（折扣系数）+ 仓库大批量供货」；本项目逐边计费、边成本与载重无关，合并不降单位成本，仓库又不供货——前提全部落空（two-echelon 退化为 PDP-T 换名）。
- **Cross-docking 只剩「同步换装」内核**：本站无月台/分拣资源，能建模的只有「V1 卸、V2 装」的时间同步与暂存容量。
- **唯一适配的是 PDP-T / relay**：站内货物必须是**已被某车从 A 取走、尚未送达 B 的订单段**，不存在「无主库存」。

#### (c) 中转站能提供的可量化收益（在逐边计费模型下）
> 先给一个硬下界：最短路满足三角不等式，故 `d(A,S)+d(S,B) ≥ d(A,B)`。**经站不可能缩短货物的行驶路径**；它只能改变**车辆**的行驶/等待，从而在以下四种情形产生收益：

1. **装卸解耦等待（timing decoupling）**：A 的取货窗早、B 的送达窗晚，直送要让车在 B 附近干等；V1 取货后把货卸在 S，V2 晚些从 S 取走送达。收益 = 减少的等待/停留时间（可直接换算成总耗时与 penalty）。
2. **路线重叠（route overlap）**：V1 本来就经过 S，V2 本来也从 S 去 B 方向；接力避免「某辆车为 A→B 专程绕行」。收益 = 避免的绕行里程，上界是直送所需的 detour。
3. **容量腾挪（capacity relief）**：车上货接近 C，先卸一部分在 S，腾出空间继续在 A 密集区取货；收益 = 减少的趟数/空驶，或允许更多请求同车。
4. **动态改派（re-assignment）**：已取货暂存在 S 的订单段，可由「此刻离 S 最近」的车接手；收益 = 响应时间/penalty，尤其对紧急单。
- **成本与必要条件**：额外站访问（若不在路线上）、同步等待、库存/custody 复杂度；站必须在至少一辆车的自然路线上，订单段可追溯、同步窗口可行。

#### (d) 必须的约束（规格要写死）
- **库存守恒**：`站内库存 = Σ(已卸入的订单段) − Σ(已取走的订单段)`；只能由车辆实际卸入，**不得凭空供货**（ADR 0001 已退役 D23 的「无主缓冲货」）。
- **容量**：站按 kg 限容（建议有限，否则退化为「无限仓库」）；同一订单段在站内只能有一份。
- **同步/先卸后取**：V2 到站时刻 ≥ V1 卸货完成时刻；若要求「短窗换装」，再加 `V2 到站 − V1 卸完 ≤ 同步窗`。
- **custody/一致性**：每个订单段记录 (orderId, A, B, q, 卸入车/时刻, 取出车/时刻)；送达时校验「这段货确实来自该订单的 A」。
- **站访问成本**：若站不在路线上，绕行成本要计入目标；若站有装卸时间，按服务时间计入。
- **不劣化保证**：实现「直送版 vs 接力版」两版取优，penalty 更差一票否决（沿用仓库历史做法）；默认数据不启用 relay 时结果不变。

#### (e) 与仓库实测的连接
仓库已实测：单车、逐边计费下「每簇必经中转站」全面更差——距离 198.2 vs 178.2 km、趟数 13 vs 6、成本 277.2 vs 244.2 元、penalty 323 vs 256（旧时间口径）；「有货就用」在期初库存=满载时也会更差（179.6 vs 178.2 km）。这正好印证 (b)：**没有规模经济、没有仓库供货时，hub 路由只是多绕路**。多车 + relay 是新的变量，但必须用两版取优来证明收益，而不是假设收益。

### 2.4 Q4：动态订单插入——全量重规划 vs 局部插入 vs 插入后修复

**文献共识**：
- 动态 PDP 的标准做法是**插入**，不是每次全量重算；Mitrović-Minić & Laporte (2004) =「cheapest insertion 构造 + tabu 改进 + 等待策略（drive first / wait first / dynamic / advanced dynamic waiting）」，规模到 1000 请求。
- **插入后局部修复**常见：Coslovich et al. (2005) 两阶段（快速接受/拒绝 + 2-opt arc swap）；Horn (2002) least-cost insertion + 周期性 steepest descent。
- **等待/缓冲**显著改善解：Mitrović-Minić & Laporte (2008) 的 request buffering 把非紧急新请求推迟到下一轮再分配，「丢失请求数/车辆数」更优；Pillac et al. (2013) 用 degree of dynamism 刻画动态程度。
- **软时间窗**：Taillard et al. (1997) 把迟到量乘罚系数加入目标（罚系数够大即等价硬窗）；Gendreau et al. (1999) 的实时 tabu 也面向 soft time windows（Parragh 综述称其目标含 lateness 项）。本项目 penalty 口径可直接沿用。
- **允许中间不可行**：Cordeau & Laporte (2003) 的罚函数松弛说明「先破坏再修复」比「步步可行」更容易跳出局部最优；课程实现可只对 penalty 松弛（容量仍硬约束）。

**三种策略对比**

| 策略 | 做法 | 复杂度（单条新单） | 优点 | 缺点/风险 |
|---|---|---|---|---|
| 全量重规划 | 所有请求重新构造 | O(m·k·n³) | 质量上限高、实现复用构造器 | 路线大改、计划抖动、慢；紧急单等不起 |
| 局部插入 | 只把新单插到现有路线最优位置 | O(k·n²·n)=O(k·n³) 朴素；slack 后 O(k·n²) | 快、稳定、可解释 | 短视；可能错过更好的全局安排 |
| 插入后修复 | 插入 + 有界 relocate/swap/2-opt | 插入 O(k·n³) + R 轮 O(m·k·n²) | 质量/稳定性折中 | 需要终止条件与「不劣化」守卫 |

**A→B 订单的增量代价（可直接写进规格）**：
```
设路线 … p → q …，在 p 后插 A、在 q 前插 B（A 在 B 前）：
ΔD = d(p,A)+d(A,q) - d(p,q)                  # 若 A、B 相邻
   + d(q,B)+d(B,r) - d(q,r)                  # 若 B 另插在 q 后
一般化：分别对 (A 的位置 i, B 的位置 j>i) 计算两段替换，
       再把从 i 到路线尾的所有到达时刻重放一遍，得 Δpenalty、Δtime。
Δcost = lex(Δpenalty, ΔD)  或  wD·ΔD + wP·Δpenalty
跨车：Δ* = min_k Δcost_k；并列时比负载均衡/ID。
```
- **penalty 增量**用重放最稳（O(n)）；提速可预计算每点「可延时间 slack」，把 Δpenalty 变前缀和查询（Campbell & Savelsbergh 2004）。**无可行插入**的兜底：开新路线/允许 penalty 但设上限/缓冲到下一事件刻；**不要**违反容量或先取后送。

**紧急单优先的可行做法**：
1. **字典序目标**：`(紧急单迟到量, 总 penalty, 总距离, 车辆数)`；紧急单先插入、普通单后插入；新紧急单不凌驾执行中的紧急单（保持 ADR 0001 的严格 FIFO）。
2. **插入时保护紧急单**：每次普通单插入后重放，若使任一紧急单的到达时刻超过其窗止（或 penalty 增加），拒绝该插入/换位置。
3. **专用紧急趟**：仅当所有车辆都无法在窗口内插入时，从车辆当前位置直接 `pos→A→B`（不是回仓库）；这与 ADR 的「紧急取送独立成趟」一致，但取货点从仓库改为 A。
4. **批量缓冲**：普通单在事件刻统一插入（小概率事件本来就稀疏），紧急单立即插入；避免每来一单就重排造成的抖动（Mitrović-Minić & Laporte 2008 的 buffering 证据）。
5. **两版取优**：紧急单场景同时算「插入现有路线」与「独立紧急趟」，按字典序取优；penalty 更差一票否决（仓库既有纪律）。

## 3. 推荐（Recommendations）

### 3.1 阶段一（单车、全 A→B）：方案 A —— 惩罚感知的 cheapest pair-insertion

```
plan = 空路线（起点=基地，空载）
requests = 按 (urgent 先, 送达窗止 EDD, ID) 排序
for r=(A,B,q,winA,winB) in requests:
    best = ∞; bestPos = none
    for i in 0..len(route):                 # A 插在 stop i 之前
        for j in i..len(route):             # B 插在 stop j 之前（j≥i）
            cand = route[0..i) + A + route[i..j) + B + route[j..)
            if not feasible(cand): continue # 前缀容量/先取后送/软窗
            ΔD = dist(cand) - dist(route)
            ΔP = penalty(cand) - penalty(route)   # 同一 §4.3 递归
            if lex(ΔP, ΔD) < best: best, bestPos = ...
    if bestPos: route = apply(route, bestPos)
    else: 开新路线 / 允许 penalty / 拒绝（按规格）
// 可选改进：2-opt、or-opt（手写；只接受字典序更优）
```
- **复杂度**：O(m·n²·n) 朴素；n≤50 时完全够用；slack 优化后 O(m·n²)。
- **优点**：最贴近 Solomon/Lu & Dessouky 的经典插入；实现短、可解释、天然支持插单；不需要 STL 算法。
- **缺点**：短视；请求顺序影响大；无可行位置时要设计兜底。
- **验收**：所有请求服务一次；前缀载荷 ≤ C；先取后送；penalty 由同一递归计算；用固定数据冻结新基准。

### 3.2 阶段二（3 车）：方案 B —— 逐单跨车 cheapest insertion + 成对 relocate 修复

```
plans = {空路线 × 3}（全部在基地、空载）
for r in ordered_requests:
    best = ∞
    for k in 0..2:
        pos = insertPair(plans[k], r)       # 方案 A 的插入评估器
        if pos and lex(pos) < best: best = (k, pos)
    if best: apply(plans[best.k], best.pos)
    else: 缓冲/开新路线/拒绝（按规格）
// 修复（有界、可关）
repeat R rounds:
    for each r in plans:
        remove r; 试插回所有车的最优位置; 若字典序更优则接受
    for each pair (r1,r2) 跨车:
        试交换两请求的车辆（保持各自 A→B 顺序）; 更优则接受
    if 无改进: break
```
- **复杂度**：分配 O(m·k·n²·n) 朴素（k=3）；修复每轮 O(m·k·n²·n)；R 取 3–5。
- **优点**：比 cluster-first 更抗「杂乱地理」；天然均衡负载；修复邻域与 Cordeau & Laporte 的成对 relocate 同源；可解释为「每辆车报边际成本」。
- **缺点**：顺序依赖；修复只到局部最优；3 辆车时收益有限，但足够课程展示。
- **可选升级**：regret 排序（每轮算 best/2nd best 车，先插 regret 最大者）；只改排序，不改插入器。

### 3.3 方案 C（可选，阶段二进阶）：cluster-first + 中转站 relay

```
（1）分簇：按 sub_network_id / 极角 sweep / 手写 k-means（A、B 必须同簇；容量约束）
（2）簇→车：按簇货量/时间跨度做均衡分配
（3）簇内：方案 A 构造
（4）跨簇修复：成对 relocate（同方案 B）
（5）relay（默认关）：对直送 detour 大的请求，试
      V1: A → S 卸；V2: S 取 → B 送
    仅当 站容量/同步/custody 可行，且 两版取优（直送 vs relay）
    在字典序下更优、penalty 不更差 时启用
```
- **复杂度**：分簇 O(m²)（或 O(m log m)）+ 路由 O(m·n²) + 修复 O(m²) + relay 评估 O(m²)。
- **优点**：能用上现有 `sub_network_id` 数据；图上有簇、有站，展示性强；relay 可捕捉「等待解耦/路线重叠」两类真实收益。
- **缺点**：窗口盲的簇会切坏解；relay 的同步/库存/custody 是新的缺陷源（仓库历史上站相关缺陷反复出现）；必须默认关 + 两版取优 + 严格守卫。
- **建议**：只作为**可选扩展**；不要作为阶段二的默认排线；先把方案 B 的基准与守卫做扎实。

### 3.4 推荐总表

阶段一 → 方案 A；阶段二 → 方案 B（可选叠加 C 的 relay）；动态插入 → 紧急立即 + 普通批量 + 插入后修复；中转站 → 默认不参与、relay 两版取优；拍卖 / hub 路由不作核心。

---

## 4. 与课程约束的兼容性（手写算法、不用 STL 算法）

| 需求 | 做法 | 备注 |
|---|---|---|
| 排序/最小值 | 手写插入排序/选择排序；固定大小 top-k；手写循环选最小 | 禁用 `std::sort`/`std::nth_element`/`std::min_element` |
| 堆/二分 | 复用已手写的 MinHeap；若需二分则手写 lower_bound | 禁用 `std::priority_queue`/`std::lower_bound` |
| 容器 | `std::vector`/`std::map`/`std::string` 可用 | 迭代 `std::map` 有序，避免 `unordered_map` 的遍历不确定性 |
| 全源最短路 | 手写 Dijkstra 跑 V 次，存矩阵 | V≈30–100；内存 O(V²) 可接受 |
| 确定性 | 并列按 ID/坐标稳定决胜；固定遍历顺序 | 保证测试可复现 |
| 守卫 | 读规划产物断言（服务数、前缀载荷、先取后送、penalty 递归、紧急优先） | 反向验证：回退修复必须失败（仓库纪律） |

---

## 5. 开放问题（规格设计前需要拍板）

1. **目标函数口径**：字典序 `(紧急迟到, 总 penalty, 总距离, 车辆数)`，还是加权和？建议字典序，避免「距离略好但 penalty 崩」。
2. **A 点窗口/服务时间**：A 是否也有取货时间窗？取货/卸货是否计服务时间（现状默认 5 min，但旧模型 A 不是停靠点）？文献 PDPTW 通常两点都有窗与服务时间。
3. **车辆是否必须回基地**：阶段二 3 车从基地出发，是否要求结束回基地（闭路线）？这决定「开放路线」是否允许。
4. **「趟」的概念**：仓库不再供货后，载重是**路线前缀约束**，一条路线可服务任意多请求；旧的「多趟回仓装货」是否正式退役、改称「路线」？
5. **中转站参数**：容量有限/无限？装卸时间？同步窗多大？relay 是否允许「一单拆给两车」以及是否计入停靠/penalty？
6. **紧急单优先级**：保持严格 FIFO（ADR 0001），还是允许「更早 deadline 的紧急单超车」？建议保持 FIFO，避免语义反复。
7. **事件权重**：现权重 10:3:1:1 使紧急（20%）> 普通（6.7%），与新要求「普通小概率、紧急更小」相反；需要新权重（如 10:2:1:1）并同步 D24 文档。
8. **规模与拒绝**：目标订单数/节点数？n≤50 时 O(n³) 可接受、n>150 需 slack/裁剪；软窗只记 penalty 不拒绝（文献两种都有，课程建议前者）。
9. **基准值**：旧黄金值（178.2/…）必须作废并重新冻结；新基准要绑定固定数据与固定排序规则。

---

## 6. 来源（按首次引用顺序；DOI 均已核对卷期页码）
1. Savelsbergh & Sol (1995), *The General Pickup and Delivery Problem*, Transportation Science 29(1):17–29. https://doi.org/10.1287/trsc.29.1.17
2. Berbeglia, Cordeau, Gendreau, Laporte (2007), *Static pickup and delivery problems: a classification scheme and survey*, TOP 15(1):1–31. https://doi.org/10.1007/s11750-007-0009-0
3. Parragh, Doerner, Hartl (2008), *A survey on pickup and delivery problems, Part I/II*, J. für Betriebswirtschaft 58(1):21–51 / 58(2):81–117. https://doi.org/10.1007/s11301-008-0033-7 · https://doi.org/10.1007/s11301-008-0036-4 · 开放版：https://prolog.univie.ac.at/research/surveyPDP/PDPsurvey_Part2_web.pdf（本笔记逐节阅读）
4. Solomon (1987), *Algorithms for the VR and Scheduling Problems with Time Window Constraints*, Operations Research 35(2):254–265. https://doi.org/10.1287/opre.35.2.254
5. Bräysy & Gendreau (2005), *VRPTW, Part I: Route Construction and Local Search Algorithms*, Transportation Science 39(1):104–118. https://doi.org/10.1287/trsc.1030.0056
6. Campbell & Savelsbergh (2004), *Efficient Insertion Heuristics…*, Transportation Science 38(3):369–378. https://doi.org/10.1287/trsc.1030.0046
7. Lu & Dessouky (2006), *A new insertion-based construction heuristic for the PDPTW*, EJOR 175(2):672–687. https://doi.org/10.1016/j.ejor.2005.05.012
8. Potvin & Rousseau (1993), *A parallel route building algorithm…（generalized regret）*, EJOR 66(3):331–340. https://doi.org/10.1016/0377-2217(93)90221-8
9. Diana & Dessouky (2004), *A new regret insertion heuristic for large-scale dial-a-ride…*, TR-B 38(6):539–557. https://doi.org/10.1016/j.trb.2003.07.001
10. Fisher & Jaikumar (1981), *A generalized assignment heuristic for vehicle routing*, Networks 11(2):109–124. https://doi.org/10.1002/net.3230110205
11. Jaw, Odoni, Psaraftis, Wilson (1986), *A heuristic algorithm for the multi-vehicle advance request dial-a-ride…*, TR-B 20(3):243–257. https://doi.org/10.1016/0191-2615(86)90020-2
12. Cordeau & Laporte (2003), *A tabu search heuristic for the static multi-vehicle dial-a-ride problem*, TR-B 37(6):579–594. https://doi.org/10.1016/S0191-2615(02)00045-0（开放版全文已读）
13. Bent & Van Hentenryck (2006), *A two-stage hybrid algorithm for PDPTW*, Computers & OR 33(4):875–893. https://doi.org/10.1016/j.cor.2004.08.001
14. Ropke & Pisinger (2006), *An Adaptive Large Neighborhood Search Heuristic for the PDPTW*, Transportation Science 40(4):455–472. https://doi.org/10.1287/trsc.1050.0135
15. Figliozzi, Mahmassani, Jaillet (2004), *Competitive Performance Assessment of Dynamic Vehicle Routing Technologies Using Sequential Auctions*, TRR 1882:10–18. https://doi.org/10.3141/1882-02
16. Alumur & Kara (2008), *Network hub location problems: The state of the art*, EJOR 190(1):1–21. https://doi.org/10.1016/j.ejor.2007.06.008
17. Perboli, Tadei, Vigo (2011), *The Two-Echelon Capacitated VRP…*, Transportation Science 45(3):364–380. https://doi.org/10.1287/trsc.1110.0368
18. Van Belle, Valckenaers, Cattrysse (2012), *Cross-docking: State of the art*, Omega 40(6):827–846. https://doi.org/10.1016/j.omega.2012.01.005
19. Mitrović-Minić & Laporte (2006), *The PDPTW and Transshipment*, INFOR 44(3):217–227. https://doi.org/10.1080/03155986.2006.11732749
20. Masson, Lehuedé, Péton (2013), *The Dial-A-Ride Problem with Transfers*, Computers & OR 41:12–23. https://doi.org/10.1016/j.cor.2013.07.020
21. Berbeglia, Cordeau, Laporte (2010), *Dynamic pickup and delivery problems*, EJOR 202(1):8–15. https://doi.org/10.1016/j.ejor.2009.04.024
22. Pillac, Gendreau, Guéret, Medaglia (2013), *A review of dynamic vehicle routing problems*, EJOR 225(1):1–11. https://doi.org/10.1016/j.ejor.2012.08.015（开放版：https://www.cirrelt.ca/documentstravail/cirrelt-2011-62.pdf）
23. Psaraftis, Wen, Kontovas (2016), *Dynamic vehicle routing problems: Three decades and counting*, Networks 67(1):3–31. https://doi.org/10.1002/net.21628
24. Mitrović-Minić & Laporte (2004), *Waiting strategies for the dynamic PDPTW*, TR-B 38(7):635–655. https://doi.org/10.1016/j.trb.2003.09.002
25. Mitrović-Minić & Laporte (2008), *Waiting and Buffering Strategies for the Dynamic PDPTW*, INFOR 46(3):165–175. https://doi.org/10.3138/infor.46.3.165
26. Gendreau, Guertin, Potvin, Taillard (1999), *Parallel Tabu Search for Real-Time Vehicle Routing and Dispatching*, Transportation Science 33(4):381–390. https://doi.org/10.1287/trsc.33.4.381
27. Taillard et al. (1997), *A Tabu Search Heuristic for the VRP with Soft Time Windows*, Transportation Science 31(2):170–186. https://doi.org/10.1287/trsc.31.2.170
28. Coslovich, Pesenti, Ukovich (2005), *A two-phase insertion technique of unexpected customers for a dynamic dial-a-ride problem*, EJOR 175(3):1605–1615. https://doi.org/10.1016/j.ejor.2005.02.038
29. [中转站策略分析.md](../../中转站策略分析.md) §六/§2.4（经站 198.2 vs 178.2 km、13 vs 6 趟；「有货就用」179.6 vs 178.2）；[ADR 0001](../../adr/0001-urgent-pickup-delivery.md)（D23 退役、A→B 取送、紧急 FIFO）；[sim-time-events.md](sim-time-events.md)（事件刻与权重）。

---

## 7. 方法与限制

- **检索**：OpenAlex `works` API（标题/DOI 检索、摘要反索引）与 Semantic Scholar Graph API 核对元数据；开放获取 PDF 用本地 `pdftotext` 阅读（Parragh Part II 38 页、Cordeau & Laporte 2003 17 页）。
- **限制**：多数 Elsevier/INFORMS 论文为付费墙，本笔记只依据**公开摘要 + 综述转述 + 卷期页码**；未逐篇通读全文。凡未标注来源的句子，均为**结合本项目模型（逐边计费、软窗、空载出发、3 车）的推导**，不是文献原话。
- **未读**：`raw-docs/`（按工作区规则）；未改任何代码/配置/测试。
- **复现**：本笔记的文献检索式、DOI 与开放 PDF 链接均在上表；仓库侧数字可复现于 `./build/app --plan-summary` 与 `docs/中转站策略分析.md` 的复现装置。