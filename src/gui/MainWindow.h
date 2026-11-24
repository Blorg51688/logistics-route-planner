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
    // 它就变错 —— 趟号塌成「第 1 趟」、「已送达」归零、「停靠 N 站」缩水、
    // 已发生的 penalty 消失、「本趟装载」被重算、在途货被反复寄存。
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

        // 趟次（规格一 MC-A01：趟结构已退役，单条连续路线 ⇒ 恒为第 1 趟）
        // 三个字段保留**只为界面文案与探针不改造**（列数与文本归 #37）；
        // 语义固定为「1 / 出发时装载 / 是否已驶离」。
        int    tripNumber = 1;            // 恒为 1
        double tripLoadKg = 0.0;          // 出发时装载（驶离后冻结）
        bool   departed = false;          // 是否已驶离
        int    completedTrips = 0;        // 恒为 0（单条路线不"跑完几趟"）

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

        // 已执行过的降载键（issue #4），防止 S 一趟出现两次时重复执行。
        // 结构化键取代旧的「站@点@kg」分隔符字符串：kg 存 lround 后的整数值，
        // 比较口径与旧字符串键一致（同站同点同整数 kg 视为同一件事）。
        // 取回不再进这个键（由本趟 pickups/reclaim 状态单独判定，reclaim 字段已退役）。
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

    // 供 --self-check-actions 核对
    const VehicleState& vehicleState() const { return state_; }

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
    // 中转站面板的文本快照，供 --ui-probe 在无头环境下确定性核对
    QString transitPanelSummary() const;
    // 中转站面板**真实表头**（供 --ui-probe 校验「当前暂存货」列确实在界面上）。
    // 与 stopPanelHeaders() 同理：守卫读界面上真正的列名，不背预期清单。
    QStringList transitPanelHeaders() const;
    // 停靠明细面板的文本快照（供 --ui-probe）
    QString stopPanelSummary() const;
    // 停靠明细面板**真实表头**（供 --ui-probe 校验表头与数据不错位）。
    // 守卫必须读界面上真正的列名，而不是把预期列表再背一遍——本项目
    // 正是因此漏过"表头 7 列、数据只填 6 列"的错位。
    QStringList stopPanelHeaders() const;
    // 工具栏动作名与面板标题（供 --ui-probe，供向导检查器比对）
    QString toolbarActionTexts() const;
    QString dockTitles() const;
    // 当前规划的趟数（供 --ui-probe 核对停靠明细的趟号）
    int     tripCount() const;
    // 已经跑完的趟数（趟号的偏移量）
    int     completedTripOffset() const;
    // 本趟装载量（供 --self-check-actions 核对）
    double  currentTripLoadKg() const { return state_.tripLoadKg; }
    // 车辆信息面板的文本快照（供 --ui-probe）
    QString vehiclePanelSummary() const;
    // 车辆信息的页签数与标题（供 --ui-probe 核对"固定信息 / 运行状态"确实分开了）
    int     vehicleTabCount() const;
    QString vehicleTabTitles() const;
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
    QString            orderIdsAt(const std::string& nodeId) const;
    // 某张订单的货量（降载/取回日志用；按订单查，不按节点查）。
    double             orderKg(const std::string& orderId) const;
    // 车辆**当前**载着的货量（随送达递减），不是本趟出发时的装载量
    double             currentLoadKg() const;
    // 车辆此刻**已经载在车上**的货（显式状态，不从计划反推）。
    // 重规划时交给规划器，才能让它知道"车不是空的"。
    std::vector<logistics::OnboardItem> onboardGoods() const;
    void               onShowGraphTables();
    QString            windowTextAt(const std::string& nodeId) const;
    int                currentTimeMin() const;
    std::vector<logistics::Order> remainingOrders() const;

    logistics::Config     config_;
    // 构造时的配置快照与路径：等价性守卫据此**重新加载**出一份独立配置
    // （两个窗口各跑一遍，随机源同种子、互不影响）。
    logistics::Config     initialConfig_;
    std::string           configPath_;
    GraphScene*           scene_ = nullptr;
    QGraphicsView*        view_ = nullptr;
    // 车辆物理状态（唯一权威来源，重规划不得改写）
    VehicleState          state_;
    logistics::RoutePlan  plan_;
    // 车辆当前位置与当前时刻必须**显式保存**：重规划会用一个全新的
    // plan_ 覆盖旧计划，此时按下标回查 plan_.stops 会索引错位
    // （新计划里的停靠点全都没送过），进而取到错误的时刻、penalty 暴涨。
    std::string           currentNodeId_;
    int                   currentTimeMin_ = 0;
    // 车辆沿 plan_.nodes 逐个节点前进（含仓库与中转站），而不再只跳配送点。
    // nodeIndex_ 指向车辆当前所在的节点，stopCursor_ 指向下一个待服务的停靠记录。
    std::size_t           nodeIndex_ = 0;
    std::size_t           stopCursor_ = 0;
    logistics::Rng        rng_{20260914u};
    logistics::WeightType planWeight_ = logistics::WeightType::Distance;

    QComboBox*     strategyBox_ = nullptr;
    QComboBox*     weightBox_ = nullptr;
    // 模拟步进间隔固定 1 秒（两种模拟模式共用同一节奏）
    int            simIntervalMs_ = 1000;

    // ---- 重规划必须继承的"既定事实" ----
    //
    // 重规划会重建整个计划，但有两件事在物理上早已确定，不该被抹掉：
    //   · completedTrips_：已经跑完了几趟（即回过几次仓库）。趟号要接着往下编，
    //     否则跑一段时间后最上面一行又变回「第 1 趟」，与"已经完成 2 趟"的事实矛盾。
    //   · currentTripLoadKg_：本趟**出发时**装了多少。一旦驶离仓库，这一趟的装载量
    //     就不会再变了；重规划把它重算，会得到与事实不符的值。
    // 物理事实：车回过几次仓库（= 跑完了几趟）。
    // **不能**用"计划内游标 curTrip"来推：Debug 每 tick 都重规划，
    // nodeIndex_ 随之归零，curTrip 永远是 0，偏移量就永远加 0。
    // 本趟是否已经驶离出发点。不能用"车辆是否停在该趟起点"来判断：
    // 重规划后车辆恰好位于新计划的起点，会被误判成"还没出发"，
    // 于是本趟装载被重算成与事实不符的值。

    // 同样是"既定事实"，不能从 plan_ 反推：
    //   · servedStopsBase_：本次计划之前**已经送达**的停靠点数。
    //     重规划后 plan_.stops 只剩"剩余要送的"，若不记账，
    //     界面上的「已送达」会归 0、「停靠 N 站」会缩水成剩余数。
    //   · incurredPenaltyMin_：**已经发生**的超时惩罚合计。
    //     重规划只算剩余计划的 penalty，不记账的话总数会往回跳。
    //   · deliveredLate_：已送达且超时的停靠点（订单/配送点/到达/窗口/penalty），
    //     供"超时订单"表区分「已经超时」与「预计会超时」。
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
    // 到达：更新位置与时刻；车回到仓库 = 一趟跑完，推进趟次并装载下一趟
    void arriveAt(const std::string& nodeId, int timeMin);
    // 送达：把该点的货从车上卸下、记账（含超时惩罚与已送达计数）
    void deliverAt(const logistics::Stop& stop);
    // 换计划时应把多少"已完成趟"并进 completedTrips_：
    //   · 车在仓库且不是初始位置 -> 刚跑完的那一趟已经结束，记 curTrip + 1
    //   · 否则（途中）-> 当前这一趟还没跑完，只记 curTrip
    // 不能一律用 curTrip：车停在仓库时，它的 nodeTripIndex 指向的正是
    // "它刚跑完的那一趟"（该趟终点就是仓库），用 curTrip 会少记一趟，
    // 于是趟号永远停在「第 1 趟」——这正是人工测试反馈的现象。
    QTextBrowser*  routeInfo_ = nullptr;
    // 车辆信息分两页签：固定信息（固有参数）/ 运行状态（随模拟变化）
    QTabWidget*    vehicleTabs_ = nullptr;
    QLabel*        vehicleFixedInfo_ = nullptr;
    QLabel*        vehicleStateInfo_ = nullptr;
    // 状态栏上的**软件内时间**：它是这套模拟的核心驱动量（事件刻、订单窗口、
    // penalty 全按它算），但此前只出现在日志里，一滚就看不见了。
    QLabel*        clockLabel_ = nullptr;
    QTableWidget*  orderTable_ = nullptr;
    QTableWidget*  transitTable_ = nullptr;
    QTableWidget*  stopTable_ = nullptr;
    // 道路封闭不再用 tick 计数节流——它现在是加权事件的一种（权重最低）。
    QTableWidget*  lateTable_ = nullptr;
    QPlainTextEdit* logView_ = nullptr;

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
