#pragma once

#include <QMainWindow>
#include <QMenu>
#include <QMenuBar>
#include <QToolBar>
#include <QTabWidget>

#include <set>
#include <string>
#include <vector>

#include "core/Config.h"
#include "core/Random.h"
#include "core/RoutePlanner.h"
#include "core/WeightType.h"

class QCloseEvent;
class GraphScene;
class QAction;
class QComboBox;
class QGraphicsView;
class QLabel;
class QPlainTextEdit;
class QTableWidget;
class QTextBrowser;
class QTimer;

// 主窗口：工具栏 + 画布 + 侧栏。
// 这里只做**编排**：规划、路况、插单、增删图这些逻辑都在 core 里且已有测试，
// 本类负责把它们串起来并更新界面。
class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    // ---- 车辆状态：所有"物理事实"的唯一权威来源 ----
    //
    // 这一组数据是**唯一权威**。重规划会整个重建 plan_，但**绝不允许**改写这里任何一项。
    //
    // 本项目反复栽在同一件事上：某个物理事实被"从当前计划反推"，于是重规划一发生，
    // 它就变错 —— 「已送达」归零、「停靠 N 站」缩水、
    // 已发生的 penalty 消失、在途货被反复寄存。
    //
    // 规则（新增功能时请遵守）：
    //   · 只有车辆**真的做了什么**才能改它 —— 装载 / 到达 / 送达，全部发生在
    //     onAdvanceStop() 里的状态转移中；
    //   · **第五条、也是唯一的例外：时间流逝**（advanceClockTo）——它只把 atTimeMin
    //     往前推，绝不改 atNodeId / loadKg / onboard / 各项累计量；"车停在半路"时
    //     时刻领先于位置是**正常**的（位置留在所在路段的起点）。调用方必须满足
    //     "不得越过下一个节点的到达时刻"，且推进后立刻 settleEventsUpTo()。
    //   · 重规划只**读**它，作为输入喂给规划器；plan_ 只描述"接下来怎么走"，
    //     不作为任何已经发生过的事实的依据。
    struct VehicleState {
        // 位置与时刻
        std::string atNodeId;             // 此刻在哪
        int         atTimeMin = 0;        // 此刻时刻

        // 车上载货：**显式**列出具体是哪些货，不再从计划反推
        double                              loadKg = 0.0;
        std::vector<logistics::OnboardItem> onboard;

        // 趟次结构已退役（规格一 MC-A01；GUI 现行是「一车一行」车辆表，见 #56/#59）。
        // 「是否已驶离」保留：回仓库后置 false，是**物理事实**（重规划不得改写），
        // 也是 fleetPristine()（车队是否还没动过）的判据之一。
        bool   departed = false;          // 是否已驶离出发点

        // 累计量
        int    servedStops = 0;           // 已送达停靠点总数
        // 违例计数：送达了**车上并没有**的货。物理上不可能，只作守卫用。
        int    offloadedWithoutGoods = 0;
        int    incurredPenaltyMin = 0;    // 已经发生的超时惩罚
        // 已送达且超时的停靠点：**已经发生**的超时，跨重规划保留
        struct LateStop {
            std::string orderId;
            std::string nodeId;
            int         arrivalMin = 0;
            int         penaltyMin = 0;
        };
        std::vector<LateStop> deliveredLate;

        // 已取货但未送达的紧急取送单（订单 id）：用于"执行中（已取未送）"状态显示，
        // 以及防止重规划后重复装同一包裹。它是**物理事实**，跨重规划保留。
        std::set<std::string> pickedOrderIds;
        // 该车**已经送达**的订单 ID（物理事实，跨重规划保留）。
        // 等价性守卫 A2 要「已送达订单集合（按车分组）」——分组只能读这里，
        // 不能从当前计划反推（重规划会换掉计划，已送完的单就不在计划里了）。
        std::set<std::string> deliveredOrderIds;

        // 已执行过的降载键（issue #4），防止同一辆车在路线中两次经过 S 时重复执行。
        // 结构化键取代旧的「站@点@kg」分隔符字符串：kg 存 lround 后的整数值，
        // 比较口径与旧字符串键一致（同站同点同整数 kg 视为同一件事）。
        // 取回不再进这个键（由本停靠动作集 pickups/reclaims 单独判定）。
        // 同样是**物理事实**，跨重规划保留。
        // 降载去重键（规格一 MC-A05：按**订单**记账）。同一张单的降载只发生一次。
        struct OffloadKey {
            std::string station;
            std::string orderId;
            bool operator<(const OffloadKey& o) const {
                if (station != o.station)  return station < o.station;
                return orderId < o.orderId;
            }
        };
        std::set<OffloadKey> appliedOffloads;

        // 中转站暂存货（降载后未取回）：跨重规划保留，传给规划器以保住取回。
        std::vector<logistics::OnboardItem> stashItems;
        std::string stashStation;

        // 车辆现有货量之和（单一算法，避免两处各算一遍）
        double sumOnboard() const {
            double t = 0.0;
            for (const logistics::OnboardItem& it : onboard) {
                t += it.kg;
            }
            return t;
        }
    };

    // 转运账本的一行（**只记已实际执行的卸入**，规格二 §7.5⑥）。
    // 计划期的转运候选不在这里：它由 FleetPlan::transitLegs 提供，不参与面板；
    // 只有车真的到站、真的卸下，才追加一行（未执行的计划卸入在重规划时作废）。
    struct TransitLedgerEntry {
        std::string orderId;
        std::string stationId;
        double      kg = 0.0;
        std::string inVehicleId;
        int         inTimeMin = 0;
        std::string outVehicleId;   // 计划里的取走车（取走前也可显示，取走时刻留空）
        int         outTimeMin = -1; // -1 = 尚未取走
        bool        pickedUp = false;
        bool        delivered = false;
    };

    // 每车一份运行状态（规格二 §6.1 / #39）：位置/时刻/在车货/已送达/已发生 penalty/
    // 当前停靠游标 + **该车自己的**计划。所有状态转移只经由 advanceFleetTo 里的
    // onAdvanceStopFor(v)；重规划只读它、绝不改写。
    struct VehicleRuntime {
        VehicleState         state;
        logistics::RoutePlan plan;
        std::size_t          nodeIndex = 0;    // 该车沿 plan.nodes 的下标
        std::size_t          stopCursor = 0;   // 该车下一个待服务停靠
        int                  departTimeMin = 0;  // 该车发车时刻（配置值）
    };

    // 供 --self-check-actions 核对（车队口径：活跃车的状态 + 全部车辆）
    const VehicleState& vehicleState() const { return fleet_.empty() ? emptyState_ : active().state; }
    const std::vector<VehicleRuntime>& fleet() const { return fleet_; }
    std::size_t activeVehicle() const { return activeVehicle_; }
    // 车队时钟（= 各车时刻的最大值；事件刻、窗口、penalty 的共同基准）
    int fleetClockMin() const { return fleetClockMin_; }
    // 各车发车时刻的最小值（完工耗时的下界，§3.4）
    int minDepartMin() const;
    // 该车下一站的节点 ID；无剩余则空串（车辆表「位置→下一站」列显示「—」）
    std::string nextStopNodeId(std::size_t v) const;
    // 该车阶段：紧急 / 普通 / 已收工（由剩余路线的阶段标记推出，§7.5②）
    QString vehiclePhaseText(std::size_t v) const;
    // 该车剩余待送货量（kg）：它自己还没送达的订单之和
    double vehicleRemainingDemandKg(std::size_t v) const;
    // 该车已送达订单数（物理事实，跨重规划保留）
    int vehicleServedOrderCount(std::size_t v) const;
    // 画布归属快照（G15）：每条车路线边 -> 归属车（行序最前）+ 是否属于选中车。
    // 格式：`<from>-><to> owner=<VID> selected=<0|1>`，逐行。
    QString canvasOwnershipSnapshot() const;
    // 画布与快照共用的**唯一**路线数据源（规格三 GUI §7）：每台车只给「当前位置 → 下一目标」
    // 那一段节点序列（含目标节点）；空车/已回库给空序列。两处都调它 ⇒ 画出来的与探针读到的一致。
    std::vector<std::vector<std::string> > canvasTargetRoutes() const;
    // 车辆表当前选中行（= 画布加粗车；#56 起一车一行，#59 起在右侧「车辆」标签页）
    int selectedVehicleIndex() const;

public:
    // configPath 用于「等价性守卫」里**重新加载配置**构造第二个独立窗口
    // （不深拷贝同一个 MainWindow 对象；见 runActionSelfCheck 第 7 段）。
    explicit MainWindow(logistics::Config config,
                        const std::string& configPath = std::string(),
                        QWidget* parent = nullptr);

    // 关窗即**自动保存**本次运行日志到 `logs/`（只保留最近 kRunLogKeep 份）。
    // 放在关窗路径上：用户跑完模拟直接关窗是最常见的收尾动作，不必记得手动导出。
    void closeEvent(QCloseEvent* event) override;
    // 自动保存（含淘汰旧文件）的次数。仅用于 --self-check-actions 观察行为，
    // 避免为了测一次淘汰就往用户真实 logs 目录里塞文件。
    int autoSaveCount() const { return autoSaveCount_; }

    // 交互式显示：把窗口尺寸限制在可用屏幕之内，避免默认尺寸大于屏幕
    // 导致侧栏或工具栏被推到屏幕之外而"看不见"
    void showInteractive();

    // 供无头渲染验证使用
    void renderToFile(const QString& path, int width, int height);

    // 供 UI 完整性探针使用（--ui-probe）
    int toolbarActionCount() const;
    int dockCount() const;
    // ---- 运行日志导出（「导出运行日志…」按钮与 --export-log 共用同一条代码路径）----
    // 记录：每次**事件触发**、每次**到达配送点时的车辆载重明细**（订单货 / 车上载重）、
    // 每次**出仓装车**、以及紧急单的处置方式。
    // 目的：把"确实会出现的问题"变成可以直接带出界面的证据，而不是要求对方复现。
    QString runLogText() const;
    bool exportRunLogTo(const QString& path) const;
    // 自动保存到 `logs/运行日志-yyyyMMdd-HHmmss.txt`，并**只保留最近 kRunLogKeep 份**。
    //
    // **缓冲队列的成员判定**：文件名**严格**匹配上面这个格式的才算。
    // 用户手动改名后一定不符合这个格式 ⇒ 自动排除、**永不删除**（那些是要长期保存的）。
    // 返回实际写入的路径（失败返回空串）。
    QString autoSaveRunLog();
    static constexpr int kRunLogKeep = 2;
    // 列出某目录下"算作自动日志"的文件（按名升序 = 时间戳升序）。静态、不写盘，
    // 供 --self-check-actions 核对命名判定与淘汰规则。
    static QStringList autoLogCandidates(const QString& dir);
    // GUI 侧的三阶段分配快照（§6.6 / 本票的 GUI↔CLI 一致性守卫）：
    // 格式与 CLI `--dump-plan` 的 ALLOC 行**同序同形**（订单 ID 升序、`O01=V01,…`）。
    // 只在「初始规划 / 全车队重调度」走 planFleet 时更新；途中重规划不改分配，
    // 因此它始终是**这一局车队分配**的用户可见证据（--ui-probe 会打印它）。
    QString allocationSnapshot() const { return allocationSnapshot_; }

    // 转运账本（规格二 §7.5⑥；#56 起并入「中转与配送网」dock 的下半）：
    // 7 列表头与**已执行**逐笔内容的文本快照。
    // 供 --ui-probe 校验表头与 --self-check-actions 校验「两端非空 / in≠out / dwell≤60 / 期末0」。
    QString     transitLedgerSummary() const;
    QStringList transitLedgerHeaders() const;
    int         transitLedgerRowCount() const;
    // 账本空态提示（#56）：无行时在面板里明写「本次运行未发生入站转运（默认数据无跨簇单）」。
    // 目的不是装饰，而是**把空表解释掉**——否则下一个人会把它当"面板坏了"再立一张票。
    bool        transitLedgerEmptyHintVisible() const;
    QString     transitLedgerEmptyHintText() const;

    // 中转站表的文本快照，供 --ui-probe 在无头环境下确定性核对。
    // #56 起 5 列：末列新增「站内待取走(kg)」= 账本里 pickedUp == false 的行求和。
    QString transitPanelSummary() const;
    // 中转站表**真实表头**（供 --ui-probe 校验「当前暂存货」「站内待取走」两列都在界面上）。
    // 与 stopPanelHeaders() 同理：守卫读界面上真正的列名，不背预期清单。
    QStringList transitPanelHeaders() const;
    // 「站内待取走(kg)」按站的**用户可见值**（直接读表格单元格，不重算公式）。
    // 新增列的守卫靠它逐帧核对：卸入后、取走前必须 > 0；期末必须回到 0。
    double stationPendingKg(const std::string& stationId) const;
    // 订单表里「超时 +Nmin」标记的**应有**行数 = 已经发生的迟到单 + 剩余计划预计迟到单。
    // 读的是 `RoutePlan::metrics.orders`（规划器逐单真源）+ `deliveredLate`，**按订单去重**
    // （一张跨簇单在接力中会被多台车的计划各排一段，按车计数会重复），
    // 供 --ui-probe 校验「订单表超时行数 == 迟到订单数」（G10 并入订单表后的原意）。
    int lateOrderCount() const;
    // 停靠明细面板的文本快照（供 --ui-probe）
    QString stopPanelSummary() const;
    // 路线信息面板的文本快照（供 --ui-probe 校验三项新指标与「停靠·动作」片段）
    QString routePanelSummary() const;
    // 停靠明细面板**真实表头**（供 --ui-probe 校验表头与数据不错位）。
    // 守卫必须读界面上真正的列名，而不是把预期列表再背一遍——本项目
    // 正是因此漏过"表头 7 列、数据只填 6 列"的错位。
    QStringList stopPanelHeaders() const;
    // 工具栏动作名与面板标题（供 --ui-probe，供向导检查器比对）
    QString toolbarActionTexts() const;
    QString dockTitles() const;
    // #59：右侧单一「信息」dock 内三个标签页、底部日志 dock 两个标签页的**真实标题**
    // （直接读 QTabWidget::tabText，供 --ui-probe 逐名核对，不背预期清单）。
    QStringList rightTabTitles() const;
    QStringList bottomTabTitles() const;
    // 当前选中页（-1 = 控件不存在）。默认 0 = 车辆 / 日志。
    int rightTabCurrentIndex() const;
    int bottomTabCurrentIndex() const;
    // 供 --ui-probe 逐页核对：页内控件（如账本空态提示）的 isVisible() 只在该页
    // 被选中时为真，探针需要临时切到目标页，核对完切回默认页。
    void activateRightTab(int index);
    // 「同一时刻只有一页可见」守卫：数 QTabWidget 里 isVisible() 为真的页数。
    // 正常恰 1；若三页被摆成同屏（如塞进 QVBoxLayout），这里会 > 1 ⇒ 守卫红。
    int rightVisiblePageCount() const;
    int bottomVisiblePageCount() const;
    // 车辆表（#56：一车一行，取代「每车一个页签」）的文本快照与**真实表头**。
    // 逐行核对「载货 ≤ 该车容量」「行数 == 车辆数」时读这里——读的是界面上真实单元格。
    int         vehicleRowCount() const;
    QStringList vehiclePanelHeaders() const;
    QString     vehiclePanelSummary() const;
    QString     vehicleRowText(int row) const;
    // 行 tooltip：固定信息（ID / 起始仓库 / 载重上限 / 发车时刻 / 两因子）与
    // 面板上已不再单列的运行细节（阶段 / 已送达 / 剩余待送 / 已发生 penalty）都在这里。
    // 原「车辆信息」六项运行状态**没有丢**，只是从占版面的标签块搬进了悬停提示。
    QString vehicleRowTooltip(int row) const;
    // 订单表（#56：一单一行 8 列，含新增「当前位置」）的真实表头与文本快照。
    // 「当前位置」= 车上(车号) / 站内·站号 / 已送达(站点) / 待取·取货点。
    QStringList orderPanelHeaders() const;
    QString     orderPanelSummary() const;
    // 状态栏"软件内时间"的文本快照（供 --ui-probe 断言它确实常驻可见）
    QString statusClockSummary() const;
    // 供 --self-check-actions 使用：自动验证两个由人工测试发现的缺陷不再复现
    //   ① 连续插入多个紧急订单不得丢单，且它们必须整体优先于普通订单
    //   ② 模拟新客户必须带来订单，且新节点必须被纳入配送
    // 返回失败项数，0 表示全部通过。
    int runActionSelfCheck();
    // 供无头验证交互后的状态：按顺序触发推进 / 路况 / 插单
    void runDemoActions(int rounds);

private slots:
    void onStrategyChanged();
    void onWeightChanged();
    void onSimulateTraffic();
    void onInsertUrgentOrder();
    // 手动「插入普通订单…」（规格二 §7.6 / #25 Q9）：跨簇演示入口，来源 manual_normal / 前缀 M。
    void onInsertNormalOrder();
    void onAddRandomCustomer();
    void onCloseRandomRoad();
    void onAdvanceStop();
    void onAdvanceMoment();
    void onReplan();
    // 两种模拟模式共用一个入口：状态只在这里收口，"再点一次正在运行的模式"= 停止。
    // （不用 QAction::toggled：它在 QActionGroup::triggered 之前发出，且 exclusive 组
    //   不允许用户直接取消勾选，两件事叠起来会把"首次勾选"误判成"请求停止"。）
    void onSimModeTriggered(QAction* action);
    void onSimTick();
    void onManualEdit();
    // 「导出运行日志…」：把本次模拟的事件触发与每个配送点到达时的载重明细写成文件，
    // 好把"确实出现过的问题"作为证据交给别人（而不必要求对方复现）。
    void onExportRunLog();
    void onAutoSaveRunLog();

private:
    void buildActions();
    void buildDocks();

    // 事件刷单 / 手动动作共用的一条插入链（规格一 MC-B04 / R8–R10）。
    // urgent=true 走紧急段尾插（FIFO），false 走普通段 pair-insertion；
    // 来源与前缀由调用方带入，窗口基准 = 事件刻（自动刷单）或当前时刻（手动动作）。
    std::string insertOrderAt(const std::string& a, const std::string& b, double demand,
                              bool urgent, logistics::OrderSource source, int windowBaseMin);
    // 随机取送单：A/B 条件均匀（R2，固定消耗 2 次随机数）、kg 3–15/3–10 整数（R3）、
    // 窗口 [基准, 24:00] / [基准, 基准+60]（R4）。事件刻刷单与手动「插入紧急订单」共用。
    std::string insertRandomOrder(bool urgent, logistics::OrderSource source, int windowBaseMin);
    std::string addRandomCustomerAction();
    // 四个事件动作的"带时刻"版本：事件刻结算时把**该事件刻**当日志前缀传进来
    // （批量结算跨越多个刻时，用"结算时刻"做前缀会把好几个事件记成同一时刻）。
    // 工具栏按钮走 -1 = 当前软件内时刻。
    void simulateTrafficAction(int atMin);
    void closeRandomRoadAction(int atMin);
    std::string nextFreeId(const char* prefix) const;

    void replan();
    void syncScene();
    void updatePanels();
    // atMin < 0 = 用"当前时刻"做运行记录的前缀；>= 0 = 用调用方给的那个时刻
    // （事件行必须用它自己的事件刻，否则批量结算时会全部记成同一时刻）。
    void appendLog(const QString& text, int atMin = -1);
    // 运行记录：与界面日志同源（每条带软件内时刻），另外还收录**只在导出文件里**
    // 才需要的载重明细。不受日志控件行数上限影响。
    void recordRun(const QString& text, int atMin = -1);
    QStringList runLog_;
    int         runEventCount_ = 0;   // 本次模拟抽到的事件次数（供汇总行）
    int         autoSaveCount_ = 0;   // 自动保存次数（仅自检观察用）

    // ---- 软件内时间的事件模型（设计 §7 / D24）----
    //
    // 事件由**软件时间前进**驱动，而不是由"你按了哪个按钮"决定：因此
    // 「推进一站」与「推进一刻」看到的是同一串事件（等价性断言的前提）。
    void settleEventsUpTo(int timeMin);   // 结算所有 <= timeMin 的事件刻（每刻恰 1 个事件）
    // 待处理额度计数（R6）：**只数事件来源**（event_normal / event_urgent）且未送达的单，
    // 含"已取在车"（picked 但未 served）；初始单与手动动作单不占额度。
    int  pendingEventOrderCount(bool urgent) const;
    // 等价性/事件规则守卫用：**重新加载配置**得到一份独立配置（两遍运行互不影响）；
    // 没有配置路径（测试内构造）时退回构造快照 initialConfig_。
    logistics::Config reloadInitialConfig() const;
    // 等价性守卫 A1 的取数口径：用户可见运行日志里所有「[事件] …」行（含时刻前缀）。
    QStringList eventLogLines() const;
    // 等价性守卫 A2–A7 的终态快照（与界面面板同源的事实，不重算任何公式）。
    QString equivalenceSnapshot() const;
    void advanceClockTo(int timeMin);     // 让"时间流逝"走到 timeMin（车留在所在段的起点）
    bool atRouteEnd() const;              // 路线是否已走完（**含"计划不可行/为空"**）
    // 与 atRouteEnd() 的区别：只有"计划可行且路线真的走完"才算**配送完成**。
    // 计划不可行/为空时 atRouteEnd() 也为真，但那不是"送完了"——
    // 此时播报「本次配送已完成，模拟结束」会与界面上的「规划不可行」自相矛盾。
    bool routeTrulyFinished() const;
    void stopSim(const QString& reason);  // 停止模拟：停表 + 取消勾选 + 记一次日志
    void reportRouteFinishedOnce();       // 终点只播报一次（消除原来每 tick 重复刷的噪音）

    std::string        currentPositionId() const;
    // 某张订单的货量（降载/取回日志用；按订单查，不按节点查）。
    double             orderKg(const std::string& orderId) const;
    // 按 ID 查订单（只读；找不到返回 nullptr）。
    const logistics::Order* findOrderById(const std::string& orderId) const;
    // 车辆**当前**载着的货量（随送达递减），不是出发时装载量
    double             currentLoadKg() const;
    // 在车明细的**一行文本**（`O01:20kg@D01` 顿号相连；空车记「（空）」）。
    // 「运行状态」块显示构成，不只是总量。
    QString            onboardGoodsText() const;
    // 车辆此刻**已经载在车上**的货（显式状态，不从计划反推）。
    // 重规划时交给规划器，才能让它知道"车不是空的"。
    std::vector<logistics::OnboardItem> onboardGoods() const;
    void               onShowGraphTables();
    int                currentTimeMin() const;
    std::vector<logistics::Order> remainingOrders() const;

    logistics::Config     config_;
    // 构造时的配置快照与路径：等价性守卫据此**重新加载**出一份独立配置
    // （两个窗口各跑一遍，随机源同种子、互不影响）。
    logistics::Config     initialConfig_;
    std::string           configPath_;
    GraphScene*           scene_ = nullptr;
    QGraphicsView*        view_ = nullptr;
    // 车队物理状态（唯一权威来源，重规划不得改写）：**每车一份**。
    //
    // 规格二 §6.1：每车一份游标（nodeIndex/stopCursor）与一份 VehicleState；
    // 所有状态转移只发生在 advanceFleetTo() -> onAdvanceStopFor(v)。
    std::vector<VehicleRuntime> fleet_;
    VehicleState                emptyState_;   // 无车辆配置时的只读占位
    // 车队时钟：全局唯一，= 各车时刻的最大值（事件刻/窗口/penalty 的共同基准）。
    int                         fleetClockMin_ = 0;
    // 当前选中的车（手动动作、画布加粗的作用对象；#59 起是「车辆」标签页里选中的
    // 行，默认 0 = 队首）。
    std::size_t                 activeVehicle_ = 0;
    // 车队级汇总（每次 replan 后由 planFleet/逐车计划重算；GUI 与 CLI 同口径）
    logistics::FleetPlan        fleetPlan_;

    VehicleRuntime&       active() { return fleet_[activeVehicle_]; }
    const VehicleRuntime& active() const { return fleet_[activeVehicle_]; }
    VehicleRuntime&       veh(std::size_t i) { return fleet_[i]; }
    const VehicleRuntime& veh(std::size_t i) const { return fleet_[i]; }
    // 该车路线是否已走完（含计划不可行/为空）
    bool vehAtEnd(std::size_t v) const;
    // 该车下一节点的到达时刻（走完则返回 INT_MAX）
    int  vehNextArrival(std::size_t v) const;
    // **全队**下一到达时刻（所有车的最小值；全走完返回 INT_MAX）
    int  nextFleetArrival() const;
    // 单一车队时钟原语（规格二 §6.1 的最小版；五级升级链归 #42）：
    //   repeat: Tnext = min(各车下一节点到达)；if 无或 Tnext > T: break
    //           所有「下一到达 == Tnext」的车一起 onAdvanceStopFor(v)
    //           settleEventsUpTo(Tnext)
    // 两条推进路径（推进一站 / 推进一刻）都只经由它改状态。
    void advanceFleetTo(int targetMin);
    // 该车前进一个节点并执行该节点的动作集（唯一的状态转移点）
    void onAdvanceStopFor(std::size_t v);
    // 车队规划入口（规格二 §6）：
    //   · **初始规划 / 全车队重调度**（车队还没动过）⇒ #40 的三阶段分配 planFleet，
    //     并把每车计划回填；分配快照进 allocationSnapshot_（与 CLI 的 ALLOC 同源可比）；
    //   · **途中重规划**（车辆已带位置/时刻/在车货）⇒ **逐车** replan（已执行前缀不重排）。
    void replanFleet();
    // 车队是否「还没动过」：全部车停在各自仓库、各自发车时刻、车上无货、无暂存、
    // 没有任何订单被取走/送达。是 ⇒ 走三阶段分配；否 ⇒ 途中重规划。
    bool fleetPristine() const;
    // 复调度输入常量（§6.2）：各车当前位置/时刻/在车货/暂存 + 逐车订单集合 +
    // 逐车剩余路线 + 全队未送达订单 + 站内已执行卸入。
    logistics::FleetRescheduleInput buildRescheduleInput() const;
    // 把一次复调度的结果写回：**只**覆盖被重算的车（未受影响车路线逐字不变），
    // 分级播报行进运行日志（用户可见，守卫读它）。
    void applyRescheduleResult(const logistics::FleetRescheduleResult& res);
    // 跑一次复调度（事件 → 五级升级链 → 写回）。返回结果，供调用方播报/守卫。
    logistics::FleetRescheduleResult runReschedule(const logistics::RescheduleEvent& event);
    // 该车的计划替换后的统一收尾（游标归零 + 终点播报复位）。
    void afterVehiclePlanReplaced(std::size_t v);
    // #41 的转运层作为**后置有界改进**再跑一遍（只在跨簇新单插入后调用：
    // 判据①跨簇；不改分配，严格更优才替换计划，完全并列保留禁转运版）。
    void runTransitPostPass();
    // 分配快照的渲染（FleetPlan -> `O01=V01,…`，订单 ID 升序）。
    QString allocationTextOf(const logistics::FleetPlan& plan) const;
    QString allocationSnapshot_;
    logistics::Rng        rng_{20260914u};
    logistics::WeightType planWeight_ = logistics::WeightType::Distance;

    QComboBox*     strategyBox_ = nullptr;
    QComboBox*     weightBox_ = nullptr;
    // 模拟步进间隔固定 1 秒（两种模拟模式共用同一节奏）
    int            simIntervalMs_ = 1000;

    // ---- 重规划必须继承的"既定事实" ----
    //
    // 重规划会重建整个计划，但以下各项在物理上早已确定，不能从 plan_ 反推：
    //   · servedStops_：本次计划之前**已经送达**的停靠点数。
    //     重规划后 plan_.stops 只剩"剩余要送的"，若不记账，
    //     界面上的「已送达」会归 0、「停靠 N 站」会缩水成剩余数。
    //   · incurredPenaltyMin_：**已经发生**的超时惩罚合计。
    //     重规划只算剩余计划的 penalty，不记账的话总数会往回跳。
    //   · deliveredLate_：已送达且超时的订单（逐单：订单/配送点/到达/penalty），
    //     供订单表的「· 超时 +Nmin」状态区分「已经超时」与「预计会超时」
    //     （独立的"超时订单"窗已在 #56 退役）。
    struct LateStop {
        std::string orderId;
        std::string nodeId;
        int         arrivalMin = 0;
        int         penaltyMin = 0;   // 窗口止在渲染时由订单反查，Stop 里没有这个字段
    };


    // ---- 状态转移：只有这几处能改 state_ ----
    //
    // 装载：单条连续路线下，全部订单都是 A→B 取送 ⇒ 车辆空载出发，
    // 货在取货点 A 装车（onAdvanceStop 的取货动作）。仓库不再装货，
    // 因此**没有** loadForTrip / loadForCurrentTrip（规格一 MC-A01/MC-A02 退役）。
    // 换计划后的统一收尾（三处重规划路径共用）
    void afterPlanReplaced();
    // 到达：更新**该车**位置与时刻（回到仓库只是"已驶离"复位，趟结构已退役）
    void arriveAt(std::size_t v, const std::string& nodeId, int timeMin);
    // 送达：把该点的货从**该车**卸下、记账（含超时惩罚与已送达计数）
    void deliverAt(std::size_t v, const logistics::Stop& stop);

    QTextBrowser*  routeInfo_ = nullptr;
    // 车辆表（#56 重设计）：**一车一行**，取代原来的「每车一个页签 + 两块标签」。
    // 为什么合并：原布局把同一台车的「载货/额定/利用率」拆在页签里，用户要切页才能
    // 逐车比；改成表格后四台车同屏并排，正是用户说的「各个车辆的载货量/总货量」。
    // 固定参数（两因子 / 起始仓库 / 发车时刻）不占列，挂在行 tooltip 上。
    QTableWidget*  vehicleTable_ = nullptr;
    // 状态栏上的**软件内时间**：它是这套模拟的核心驱动量（事件刻、订单窗口、
    // penalty 全按它算），但此前只出现在日志里，一滚就看不见了。
    QLabel*        clockLabel_ = nullptr;
    QTableWidget*  orderTable_ = nullptr;
    QTableWidget*  transitTable_ = nullptr;
    QTableWidget*  transitLedgerTable_ = nullptr;
    // 已实际执行的转运账本（跨重规划保留；计划期候选不在这里）。
    std::vector<TransitLedgerEntry> transitLedger_;
    QTableWidget*  stopTable_ = nullptr;
    // #59：右侧「信息」dock 的三页标签（车辆 / 订单 / 中转与配送网）与底部
    // 「日志」dock 的两页标签（日志 / 路线与停靠）。表格对象的指针与填充逻辑不变，
    // 只是父容器从多个 dock 换成 QTabWidget 的页。
    QTabWidget*    rightTabs_ = nullptr;
    QTabWidget*    bottomTabs_ = nullptr;
    // #59：「同一时刻只有一页可见」守卫读这三页的**真实可见状态**。
    // 存页面指针（而不只靠 QTabWidget）是为了让守卫对「改成同屏」也敏感：
    // 三页若被 QVBoxLayout 一次铺开，三者的 isVisible() 都为真 ⇒ 守卫红。
    QWidget*       rightVehiclePage_ = nullptr;
    QWidget*       rightOrderPage_ = nullptr;
    QWidget*       rightTransitPage_ = nullptr;
    // 账本空态提示（#56）：只在 transitLedger_ 无行时可见。
    QLabel*        ledgerEmptyHint_ = nullptr;
    // 道路封闭不再用 tick 计数节流——它现在是加权事件的一种（权重最低）。
    QPlainTextEdit* logView_ = nullptr;
    // 「视图」菜单：buildActions 建菜单，dock 建好后在 buildDocks 末尾补挂显隐开关
    // （旧代码建菜单时 dock 还不存在，findChildren<QDockWidget*>() 恒为空）。
    QMenu*         viewMenu_ = nullptr;

    QTimer* simTimer_ = nullptr;
    // 模拟模式：0 = 关闭；1 = 按站模拟；2 = 按时间模拟。两者**互斥**（同一 QActionGroup）。
    int     simMode_ = 0;
    // 下一个待结算的**事件刻**（软件内时刻，绝对时刻对齐到 event_interval_min 的整数倍）。
    // 事件由"软件时间前进"驱动、与推进粒度无关 —— 因此「按站模拟」与「推进一刻」
    // 看到的是**同一串事件**（这是等价性断言成立的前提，见设计 D24/P34）。
    int     nextEventMark_ = 0;
    // 到终点只播报一次（原先每个 tick 都重复刷「本次配送已完成」）
    bool    routeFinishedReported_ = false;
    QAction* stationSimAction_ = nullptr;
    QAction* timeSimAction_ = nullptr;
};