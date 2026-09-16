#pragma once

#include <QMainWindow>

#include <string>
#include <vector>

#include "core/Config.h"
#include "core/Random.h"
#include "core/RoutePlanner.h"
#include "core/WeightType.h"

class GraphScene;
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
    //   · 只有车辆**真的做了什么**才能改它 —— 装载 / 到达 / 送达 / 寄存，
    //     全部发生在 onAdvanceStop() 里的状态转移中；
    //   · 重规划只**读**它，作为输入喂给规划器；plan_ 只描述"接下来怎么走"，
    //     不作为任何已经发生过的事实的依据。
    struct VehicleState {
        // 位置与时刻
        std::string atNodeId;             // 此刻在哪
        int         atTimeMin = 0;        // 此刻时刻

        // 车上载货：**显式**列出具体是哪些货，不再从计划反推
        double                              loadKg = 0.0;
        std::vector<logistics::OnboardItem> onboard;

        // 趟次
        int    tripNumber = 1;            // 正在跑第几趟（绝对编号，1 起）
        double tripLoadKg = 0.0;          // 本趟出发时装了多少（驶离后冻结）
        bool   departed = false;          // 本趟是否已驶离
        int    completedTrips = 0;        // 已跑完几趟

        // 累计量
        int    servedStops = 0;           // 已送达停靠点总数
        int    incurredPenaltyMin = 0;    // 已经发生的超时惩罚
        // 已送达且超时的停靠点：**已经发生**的超时，跨重规划保留
        struct LateStop {
            std::string orderId;
            std::string nodeId;
            int         arrivalMin = 0;
            int         penaltyMin = 0;
        };
        std::vector<LateStop> deliveredLate;

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
    explicit MainWindow(logistics::Config config, QWidget* parent = nullptr);

    // 交互式显示：把窗口尺寸限制在可用屏幕之内，避免默认尺寸大于屏幕
    // 导致侧栏或工具栏被推到屏幕之外而"看不见"
    void showInteractive();

    // 供无头渲染验证使用
    void renderToFile(const QString& path, int width, int height);

    // 供 UI 完整性探针使用（--ui-probe）
    int toolbarActionCount() const;
    int dockCount() const;
    // 中转站面板的文本快照，供 --ui-probe 在无头环境下确定性核对
    QString transitPanelSummary() const;
    // 停靠明细面板的文本快照（供 --ui-probe）
    QString stopPanelSummary() const;
    // 工具栏动作名与面板标题（供 --ui-probe，供向导检查器比对）
    QString toolbarActionTexts() const;
    QString dockTitles() const;
    // 当前规划的趟数（供 --ui-probe 核对停靠明细的趟号）
    int     tripCount() const;
    // 已经跑完的趟数（趟号的偏移量）
    int     completedTripOffset() const;
    // 车回过几次仓库（物理事实，供 --self-check-actions）
    int     depotArrivalCount() const { return state_.completedTrips; }
    // 本趟装载量（供 --self-check-actions 核对）
    double  currentTripLoadKg() const { return state_.tripLoadKg; }
    // 车辆信息面板的文本快照（供 --ui-probe）
    QString vehiclePanelSummary() const;

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
    void onReplan();
    void onDebugToggled(bool on);
    void onDebugTick();
    void onManualEdit();

private:
    void buildActions();
    void buildDocks();

    std::string insertUrgentOrderAction();
    std::string addRandomCustomerAction();
    std::string nextFreeId(const char* prefix) const;

    void replan();
    void syncScene();
    void updatePanels();
    void appendLog(const QString& text);

    std::string        currentPositionId() const;
    QString            orderIdsAt(const std::string& nodeId) const;
    // 某中转站**截至当前推进位置**的暂存货量（不是规划终值——终值必为 0，
    // 那样界面上这一列永远是 0，没有观测价值）
    // 中转站暂存：截至当前推进位置的**当前值**与**历史峰值**。
    // 二者都随推进实时变化——峰值不是在规划时定死的，而是"存进去时才比较是否刷新"。
    void               stockTrace(const std::string& stationId, double& current,
                                  double& peak) const;
    // 车辆**当前**载着的货量（随送达递减），不是本趟出发时的装载量
    double             currentLoadKg() const;
    // 车辆此刻**已经载在车上**的货（当前趟里还没经过的停靠点）。
    // 重规划时交给规划器，才能让它知道"车不是空的"。
    std::vector<logistics::OnboardItem> onboardGoods() const;
    // 把最近一次规划的结果里各站的期末存货回写为"当前存货"，
    // 供下一次重规划作为期初存货传入 —— 这就是"积少成多"的回路。
    void               syncStationStock();
    // 记录本次规划中被「顺路寄存」到站里的货所对应的节点
    void               onShowGraphTables();
    QString            windowTextAt(const std::string& nodeId) const;
    int                currentTimeMin() const;
    std::vector<logistics::Order> remainingOrders() const;

    logistics::Config     config_;
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
    // Debug 模拟间隔固定 1 秒（用户要求删掉速度选择卡片以节约工具栏空间）
    int            debugIntervalMs_ = 1000;
    // 各中转站的当前存货（跨重规划延续）
    std::map<std::string, double> stationStock_;

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
    // 已经被「顺路寄存」到中转站的货所对应的节点。
    // 这些货已经不在车上，必须从 onboardGoods() 里剔除，否则每重规划一次
    // 就会被再寄存一次，stationStock_ 单调膨胀并污染后续路由（审计发现的 F5）。
    // 车辆当前所在的趟在 plan_.trips 里的下标
    std::size_t currentTripIndex() const;

    // ---- 状态转移：只有这四个函数能改 state_ ----
    //
    // 装载：车在仓库时，按计划的当前趟把货装上车（并冻结本趟装载量）
    void loadForCurrentTrip();
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
    QLabel*        vehicleInfo_ = nullptr;
    QTableWidget*  orderTable_ = nullptr;
    QTableWidget*  transitTable_ = nullptr;
    QTableWidget*  stopTable_ = nullptr;
    int            debugClosureCounter_ = 0;
    QTableWidget*  lateTable_ = nullptr;
    QPlainTextEdit* logView_ = nullptr;

    QTimer* debugTimer_ = nullptr;
    bool    debugOn_ = false;
    int     debugTicks_ = 0;
    int     debugTickMs_ = 3000;
};
