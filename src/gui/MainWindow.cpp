#include "gui/MainWindow.h"

#include "core/SimEvent.h"

#include <QAction>
#include <QActionGroup>
#include <QPainter>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QMessageBox>
#include <QDialog>
#include <QCoreApplication>
#include <QDir>
#include <QScreen>
#include <QSignalBlocker>
#include <QFileInfo>
#include <QDialogButtonBox>
#include <QFormLayout>
#include <QGraphicsView>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QGuiApplication>
#include <QInputDialog>
#include <QLabel>
#include <QStatusBar>
#include <QFile>
#include <QFileDialog>
#include <QDateTime>
#include <QTextStream>
#include <QStringList>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QTableWidget>
#include <QTabWidget>
#include <QRegularExpression>
#include <QTextBrowser>
#include <QTimer>
#include <QTemporaryDir>
#include <QToolBar>
#include <QToolButton>
#include <QVBoxLayout>
#include <QDockWidget>
#include <QGuiApplication>
#include <QToolBar>

#include <algorithm>
#include <memory>
#include <cstdio>
#include <cmath>
#include <utility>

#include "core/GraphEdit.h"
#include "core/Traffic.h"
#include "gui/GraphScene.h"
#include "io/ConfigLoader.h"

using logistics::Config;
using logistics::Order;
using logistics::RoutePlan;
using logistics::Stop;
using logistics::WeightType;

namespace {

// 标签显示：界面上的行标文字统一由这里产出
QString typeText(logistics::NodeType type) {
    switch (type) {
        case logistics::NodeType::Warehouse: return QStringLiteral("仓库");
        case logistics::NodeType::Delivery:  return QStringLiteral("配送点");
        case logistics::NodeType::Transit:   return QStringLiteral("中转站");
    }
    return QStringLiteral("?");
}

// R6 · 自动刷单的待处理额度（**代码常量**，不新增 [general] 键，见 #24 §1.9 决策 3）。
// 计数口径 = 事件来源且未送达（含已取在车）；初始单与手动动作单不计。
constexpr int kMaxPendingNormalOrders = 6;
constexpr int kMaxPendingUrgentOrders = 2;

QString minutesToClock(int minutes) {
    const int h = minutes / 60;
    const int m = minutes % 60;
    return QStringLiteral("%1:%2")
        .arg(h, 2, 10, QLatin1Char('0'))
        .arg(m, 2, 10, QLatin1Char('0'));
}

} // namespace

MainWindow::MainWindow(Config config, const std::string& configPath, QWidget* parent)
    : QMainWindow(parent), config_(std::move(config)), initialConfig_(config_),
      configPath_(configPath) {
    setWindowTitle(QStringLiteral("电商物流配送路径规划系统"));

    scene_ = new GraphScene(this);
    view_ = new QGraphicsView(scene_);
    view_->setRenderHint(QPainter::Antialiasing, true);
    setCentralWidget(view_);

    if (!config_.vehicles.empty()) {
        currentNodeId_ = config_.vehicles.front().startNodeId;
        currentTimeMin_ = config_.vehicles.front().departTimeMin;
        // 车辆状态的初始值：停在仓库、时刻为发车时刻、车上无货。
        // state_ 是物理事实的唯一来源，必须与界面用的两个字段同时初始化。
        state_.atNodeId = config_.vehicles.front().startNodeId;
        state_.atTimeMin = config_.vehicles.front().departTimeMin;
        state_.loadKg = 0.0;
    }

    // 事件刻：绝对时刻对齐（08:00 = 480 是 15 的整数倍，故首个事件落在 08:15）。
    // 事件只由软件内时钟驱动，见 settleEventsUpTo。
    const int eventInterval = config_.general.eventIntervalMin > 0
                                  ? config_.general.eventIntervalMin : 15;
    nextEventMark_ = logistics::nextEventTimeMin(state_.atTimeMin, eventInterval);

    buildActions();
    buildDocks();

    syncScene();
    replan();
    appendLog(QStringLiteral("已加载配置：节点 %1，边 %2，订单 %3")
                  .arg(config_.graph.nodeCount())
                  .arg(config_.graph.edgeCount())
                  .arg(config_.orders.size()));
}

void MainWindow::buildActions() {
    // 顶部原本是一条"什么都有"的工具栏，19 个动作放不下会溢出成 >>。
    // 按**作用对象**重新归类成 5 条具名工具栏（都可拖动/浮动，行数由 Qt 自动折行）：
    //   规划策略 · 事件注入 · 推进与模拟 · 工具 · 导出
    // 「视图」菜单提供各停靠面板与工具栏的显隐开关（面板被关掉后还能找回来）。
    auto makeBar = [this](const QString& name) {
        QToolBar* b = addToolBar(name);
        b->setObjectName(name);   // 供 saveState/restoreState 与 --ui-probe 识别
        b->setMovable(true);
        return b;
    };

    // ---- 1. 规划策略：选策略与权重标签 ----
    QToolBar* planBar = makeBar(QStringLiteral("规划策略"));
    strategyBox_ = new QComboBox(this);
    // 顺序与「权重标签」下拉保持一致：距离 -> 耗时 -> 成本
    strategyBox_->addItem(QStringLiteral("最短距离策略"), QVariant(QStringLiteral("distance")));
    strategyBox_->addItem(QStringLiteral("最低耗时策略"), QVariant(QStringLiteral("time")));
    strategyBox_->addItem(QStringLiteral("最低成本策略"), QVariant(QStringLiteral("cost")));
    planBar->addWidget(new QLabel(QStringLiteral("规划策略 "), this));
    planBar->addWidget(strategyBox_);
    connect(strategyBox_, &QComboBox::currentIndexChanged, this, &MainWindow::onStrategyChanged);

    weightBox_ = new QComboBox(this);
    weightBox_->addItem(QStringLiteral("显示距离"), QVariant(QStringLiteral("distance")));
    weightBox_->addItem(QStringLiteral("显示耗时"), QVariant(QStringLiteral("time")));
    weightBox_->addItem(QStringLiteral("显示成本"), QVariant(QStringLiteral("cost")));
    planBar->addWidget(new QLabel(QStringLiteral("  权重标签 "), this));
    planBar->addWidget(weightBox_);
    connect(weightBox_, &QComboBox::currentIndexChanged, this, &MainWindow::onWeightChanged);

    // ---- 2. 事件注入：四类模拟事件 ----
    QToolBar* eventBar = makeBar(QStringLiteral("事件注入"));
    eventBar->addAction(QStringLiteral("插入紧急订单"), this, &MainWindow::onInsertUrgentOrder);
    eventBar->addAction(QStringLiteral("模拟新客户"), this, &MainWindow::onAddRandomCustomer);
    eventBar->addAction(QStringLiteral("模拟路况"), this, &MainWindow::onSimulateTraffic);
    eventBar->addAction(QStringLiteral("模拟道路封闭"), this, &MainWindow::onCloseRandomRoad);

    // ---- 3. 推进与模拟：手动步进 + 两种自动模拟模式（互斥） ----
    QToolBar* simBar = makeBar(QStringLiteral("推进与模拟"));
    simBar->addAction(QStringLiteral("推进一站"), this, &MainWindow::onAdvanceStop);
    simBar->addAction(QStringLiteral("推进一刻"), this, &MainWindow::onAdvanceMoment);
    // 两种**模拟模式**并列且互斥（同一 QActionGroup）：
    //   按站模拟：每步推进一站；按时间模拟：每步推进一刻（= 下一个 15min 事件刻）。
    // 两者共用同一步进节奏（1 秒/步）。事件都不再"按推进次数"或"按真实秒数"触发，
    // 而是由**软件内时钟**跨过事件刻决定（见 settleEventsUpTo）。
    stationSimAction_ = simBar->addAction(QStringLiteral("按站模拟"));
    stationSimAction_->setCheckable(true);
    timeSimAction_ = simBar->addAction(QStringLiteral("按时间模拟"));
    timeSimAction_->setCheckable(true);

    auto* simGroup = new QActionGroup(this);
    simGroup->addAction(stationSimAction_);
    simGroup->addAction(timeSimAction_);
    simGroup->setExclusive(true);
    // 状态只在 QActionGroup::triggered 一处收口（不用 toggled，原因见头文件注释）。
    // exclusive 组保证"勾一个自动取消另一个"；"再点一次正在运行的那个"由
    // onSimModeTriggered 显式取消勾选（向导第 7 关要求取消后必须立刻停下）。
    connect(simGroup, &QActionGroup::triggered, this, &MainWindow::onSimModeTriggered);

    // ---- 4. 工具：重规划与手工增删 ----
    QToolBar* toolBar = makeBar(QStringLiteral("工具"));
    toolBar->addAction(QStringLiteral("重新规划"), this, &MainWindow::onReplan);
    toolBar->addAction(QStringLiteral("手工增删…"), this, &MainWindow::onManualEdit);
    toolBar->addAction(QStringLiteral("图表示…"), this, &MainWindow::onShowGraphTables);

    // ---- 5. 导出 ----
    QToolBar* logBar = makeBar(QStringLiteral("导出"));
    logBar->addAction(QStringLiteral("自动保存日志"), this, &MainWindow::onAutoSaveRunLog);
    logBar->addAction(QStringLiteral("另存日志…"), this, &MainWindow::onExportRunLog);

    // ---- 视图菜单：面板与工具栏的显隐（面板关闭后还能找回来）----
    QMenu* viewMenu = menuBar()->addMenu(QStringLiteral("视图"));
    for (QToolBar* b : {planBar, eventBar, simBar, toolBar, logBar}) {
        viewMenu->addAction(b->toggleViewAction());
    }
    viewMenu->addSeparator();
    for (QDockWidget* dock : findChildren<QDockWidget*>()) {
        viewMenu->addAction(dock->toggleViewAction());
    }

    simTimer_ = new QTimer(this);
    connect(simTimer_, &QTimer::timeout, this, &MainWindow::onSimTick);
}

void MainWindow::buildDocks() {
    // ---- 状态栏：常驻显示**软件内时间** ----
    // 放在状态栏而不是右侧面板：① 永远可见（面板可被拖走/折叠、日志会被刷掉）；
    // ② 不与其他面板抢宽度；③ "现在是模拟里的几点"属于全局状态，不属于某一类对象。
    clockLabel_ = new QLabel(this);
    clockLabel_->setTextFormat(Qt::PlainText);
    statusBar()->addPermanentWidget(clockLabel_);

    auto* routeDock = new QDockWidget(QStringLiteral("路线信息"), this);
    routeDock->setMinimumWidth(360);
    routeInfo_ = new QTextBrowser(routeDock);
    routeDock->setWidget(routeInfo_);
    addDockWidget(Qt::RightDockWidgetArea, routeDock);

    auto* vehicleDock = new QDockWidget(QStringLiteral("车辆信息"), this);
    vehicleDock->setMinimumWidth(360);
    // 车辆信息（MC-E04）：**每车一个页签**，标题 = `<车辆 ID> · <容量>kg`；
    // 页内再分「固定信息 / 运行状态」两块标题：
    //   · 固定信息 —— 车辆的**固有参数**（ID / 起始仓库 / 载重上限 / 发车时刻 /
    //     耗时倍率 / 成本倍率），整场模拟都不会变，任何时候看都一样；
    //   · 运行状态 —— **随模拟推进而变**的量（车上载重及构成 / 剩余待送 / 已送达 /
    //     已发生 penalty）。软件内时间不在这里重复打印（状态栏常驻显示）。
    // 页签数由 updatePanels 首刷时按 config_.vehicles 重建。
    vehicleTabs_ = new QTabWidget(vehicleDock);
    vehicleDock->setWidget(vehicleTabs_);
    addDockWidget(Qt::RightDockWidgetArea, vehicleDock);

    auto* orderDock = new QDockWidget(QStringLiteral("订单列表"), this);
    orderDock->setMinimumWidth(360);
    orderTable_ = new QTableWidget(0, 5, orderDock);
    orderTable_->setHorizontalHeaderLabels(
        {QStringLiteral("订单"), QStringLiteral("配送点"), QStringLiteral("货量"),
         QStringLiteral("窗口"), QStringLiteral("状态")});
    // 前 4 列按内容自适应、状态列拉伸，保证"状态"不会被挤到可视区之外
    orderTable_->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    orderTable_->horizontalHeader()->setStretchLastSection(true);
    orderTable_->verticalHeader()->setVisible(false);
    orderTable_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    orderDock->setWidget(orderTable_);
    addDockWidget(Qt::RightDockWidgetArea, orderDock);

    // 中转站 / 集散面板：承担 P3 的"子网络分组标识"与下属配送点归属；
    // 第 4 列反映紧急降载暂存货（issue #10，无暂存为 0）。
    auto* transitDock = new QDockWidget(QStringLiteral("中转站 / 集散"), this);
    transitDock->setMinimumWidth(360);
    transitTable_ = new QTableWidget(0, 4, transitDock);
    transitTable_->setHorizontalHeaderLabels(
        {QStringLiteral("中转站"), QStringLiteral("子网络"), QStringLiteral("下属配送点"),
         QStringLiteral("当前暂存货(kg)")});
    transitTable_->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    transitTable_->horizontalHeader()->setStretchLastSection(true);
    transitTable_->verticalHeader()->setVisible(false);
    transitTable_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    transitDock->setWidget(transitTable_);
    addDockWidget(Qt::RightDockWidgetArea, transitDock);

    // 停靠明细（MC-E01）：**9 列**、按每车铺行（规格一阶段只有 1 台车，但车辆列
    // 不写死单值）：车辆 | 序号 | 阶段 | 配送点 | 动作（取<a>·送<b>）| 原始到达 |
    // 等待(分) | 送达 | 余货(kg)。趟结构已退役，**无「趟」列**；「动作」列显示**计数**
    // （取<a>·送<b>，380px 面板放不下订单 ID 列表，见 #29 §4.2）；
    // 「余货(kg)」= `Stop::remainingLoadKg`（离开该站时的**在车货量**）。
    auto* stopDock = new QDockWidget(QStringLiteral("停靠明细"), this);
    stopDock->setMinimumWidth(360);
    stopTable_ = new QTableWidget(0, 9, stopDock);
    stopTable_->setHorizontalHeaderLabels(
        {QStringLiteral("车辆"), QStringLiteral("序号"), QStringLiteral("阶段"),
         QStringLiteral("配送点"), QStringLiteral("动作"), QStringLiteral("原始到达"),
         QStringLiteral("等待(分)"), QStringLiteral("送达"),
         QStringLiteral("余货(kg)")});
    stopTable_->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    stopTable_->horizontalHeader()->setStretchLastSection(true);
    stopTable_->verticalHeader()->setVisible(false);
    stopTable_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    stopDock->setWidget(stopTable_);
    addDockWidget(Qt::RightDockWidgetArea, stopDock);

    auto* lateDock = new QDockWidget(QStringLiteral("超时订单"), this);
    lateDock->setMinimumWidth(360);
    // 超时订单（MC-E02）：**逐单一行**、8 列：车辆 | 订单 | 配送点 | 档位 | 状态 |
    // 到达 | 窗口 | penalty(min)。窗口用**该单自己的** [ws, we]（不再取同停靠
    // 多单的并集——那正是合并窗口口径的残留）。
    lateTable_ = new QTableWidget(0, 8, lateDock);
    lateTable_->setHorizontalHeaderLabels(
        {QStringLiteral("车辆"), QStringLiteral("订单"), QStringLiteral("配送点"),
         QStringLiteral("档位"), QStringLiteral("状态"), QStringLiteral("到达"),
         QStringLiteral("窗口"), QStringLiteral("penalty(min)")});
    lateTable_->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    lateTable_->horizontalHeader()->setStretchLastSection(true);
    lateTable_->verticalHeader()->setVisible(false);
    lateTable_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    lateDock->setWidget(lateTable_);
    addDockWidget(Qt::RightDockWidgetArea, lateDock);

    auto* logDock = new QDockWidget(QStringLiteral("日志"), this);
    logView_ = new QPlainTextEdit(logDock);
    logView_->setReadOnly(true);
    logDock->setWidget(logView_);
    addDockWidget(Qt::BottomDockWidgetArea, logDock);

    // 订单列表与超时订单叠成标签页：笔记本屏幕高度有限，
    // 四个面板纵向平铺会把每个都压到不可用
    tabifyDockWidget(orderDock, lateDock);
    tabifyDockWidget(lateDock, transitDock);
    tabifyDockWidget(transitDock, stopDock);
    orderDock->raise();

    resizeDocks({routeDock, vehicleDock}, {240, 110}, Qt::Vertical);
    resizeDocks({orderDock}, {240}, Qt::Vertical);
    resizeDocks({logDock}, {140}, Qt::Vertical);

    // 右侧栏整体留出足够宽度，避免"总距离：220.1 / km"这种被折断的显示
    resizeDocks({routeDock, vehicleDock, orderDock}, {380, 380, 380}, Qt::Horizontal);
}

std::string MainWindow::currentPositionId() const {
    return currentNodeId_;
}

int MainWindow::currentTimeMin() const {
    return currentTimeMin_;
}

// 某配送点上的全部订单号，用 / 连接（同一节点可能有多单合并成一次停靠）

double MainWindow::currentLoadKg() const {
    // 直接读车辆状态。**不再**从计划的 remainingLoadKg 反推——
    // 重规划会换掉 plan_，反推出来的"当前载重"会跟着变，与车上真实货量不符。
    return state_.loadKg;
}


QString MainWindow::onboardGoodsText() const {
    const std::vector<logistics::OnboardItem> items = onboardGoods();
    if (items.empty()) {
        return QStringLiteral("（空）");
    }
    QStringList parts;
    for (const logistics::OnboardItem& it : items) {
        parts << QStringLiteral("%1:%2kg@%3")
                     .arg(QString::fromStdString(it.orderId))
                     .arg(it.kg, 0, 'f', 0)
                     .arg(QString::fromStdString(it.nodeId));
    }
    return parts.join(QStringLiteral("、"));
}

std::vector<logistics::OnboardItem> MainWindow::onboardGoods() const {
    // 车上有哪些货是**显式状态**，不再从计划反推。
    // 反推的版本会在重规划后把已经送掉/已经寄存的货又当成在途货，造成重复寄存与
    // "车不回仓库装货"的坏轨迹（设计 §16 P17）。
    return state_.onboard;
}

void MainWindow::arriveAt(const std::string& nodeId, int timeMin) {
    state_.atNodeId = nodeId;
    state_.atTimeMin = timeMin;
    // 单条连续路线（规格一 MC-A01）：没有"趟"这回事，回仓库只是路线终点。
    // 只记"是否已驶离"，供界面上「本趟出发装载」的冻结语义继续成立。
    if (!config_.vehicles.empty() && nodeId == config_.vehicles.front().startNodeId
        && plan_.nodes.size() > 1) {
        state_.departed = false;
    } else {
        state_.departed = true;
    }
}

void MainWindow::deliverAt(const logistics::Stop& stop) {
    // 记账（"已经发生的事实"，重规划不得改写）。
    // 卸货本身已由 onAdvanceStop 的**动作集**按 orderId 完成（规格一 MC-E09/A05）；
    // 这里只做计数与超时登记，不再按 nodeId 扣货（那样会把同节点别的单一起扣掉）。
    ++state_.servedStops;
    if (stop.late) {
        state_.incurredPenaltyMin += stop.penaltyMin;
        for (const logistics::StopDelivery& d : stop.deliveries) {
            if (!d.late) {
                continue;
            }
            VehicleState::LateStop ls;
            ls.orderId = d.orderId;
            ls.nodeId = stop.nodeId;
            ls.arrivalMin = stop.arrivalMin;
            ls.penaltyMin = d.penaltyMin;
            state_.deliveredLate.push_back(ls);
        }
    }
}

// 某张订单的货量（供降载/取回日志显示；按订单查，不按节点查）。
const logistics::Order* MainWindow::findOrderById(const std::string& orderId) const {
    for (const Order& o : config_.orders) {
        if (o.id == orderId) {
            return &o;
        }
    }
    return nullptr;
}

double MainWindow::orderKg(const std::string& orderId) const {
    for (const Order& o : config_.orders) {
        if (o.id == orderId) {
            return o.demandKg;
        }
    }
    return 0.0;
}

std::vector<Order> MainWindow::remainingOrders() const {
    std::vector<Order> remaining;
    for (const Order& order : config_.orders) {
        if (order.served) {
            continue;
        }
        remaining.push_back(order);
    }
    return remaining;
}

void MainWindow::syncScene() {
    scene_->build(config_.graph, scene_->weightType());
    scene_->setAllLabelsVisible(false);
    if (plan_.status == logistics::PlanStatus::Ok) {
        // 只高亮**尚未走完**的那一段：已走过的路段继续标红会让人误以为还没送到
        const std::size_t from = (nodeIndex_ < plan_.nodes.size()) ? nodeIndex_ : 0;
        const std::vector<std::string> remaining(plan_.nodes.begin()
                                                     + static_cast<std::ptrdiff_t>(from),
                                                 plan_.nodes.end());
        scene_->highlightRoute(remaining);
    } else {
        scene_->clearHighlight();
    }
    scene_->setVehiclePosition(currentNodeId_);
}

// 换计划之后必须做的三件事，只此一处。三处重规划路径（replan /
// replanIncremental / insertOrder）都调它，避免各写一份而漏改。
void MainWindow::afterPlanReplaced() {
    nodeIndex_ = 0;      // 新路线从车辆当前位置起算，推进游标归零
    stopCursor_ = 0;
    // 新计划里还有路要走 => 重新允许"到终点播报一次"。
    // 否则跑完一次后再「重新规划」，第二次跑完就再也播报不出来了。
    if (!atRouteEnd()) {
        routeFinishedReported_ = false;
    }
    // 单条连续路线下车辆空载出发、货在取货点 A 装车，重规划后**不需要**装车。
}

void MainWindow::replan() {
    if (config_.vehicles.empty()) {
        appendLog(QStringLiteral("配置中没有车辆，无法规划"));
        return;
    }
    const std::string here = currentPositionId();
    const int now = currentTimeMin();
    const std::vector<Order> remaining = remainingOrders();

    logistics::StashState stash;
    stash.station = state_.stashStation;
    stash.items = state_.stashItems;
    plan_ = logistics::replan(config_.graph, config_.vehicles.front(), remaining, here, now,
                              planWeight_, onboardGoods(), stash);
    afterPlanReplaced();

    if (plan_.status == logistics::PlanStatus::Ok) {
        appendLog(QStringLiteral("规划成功：%1 站，总距离 %2km，总耗时 %3min，penalty %4min")
                      .arg(plan_.stops.size())
                      .arg(plan_.totalDistanceKm, 0, 'f', 1)
                      .arg(plan_.totalTimeMin, 0, 'f', 1)
                      .arg(plan_.totalPenaltyMin));
    } else {
        appendLog(QStringLiteral("规划不可行：%1").arg(QString::fromStdString(plan_.reason)));
    }
    syncScene();
    updatePanels();
}

void MainWindow::onStrategyChanged() {
    const QString strategy = strategyBox_->currentData().toString();
    planWeight_ = (strategy == QStringLiteral("cost"))
                      ? WeightType::Cost
                      : (strategy == QStringLiteral("time") ? WeightType::Time
                                                            : WeightType::Distance);
    // 不在这里清零 stopCursor_/nodeIndex_：replan() 需要先用它们记账，
    // 清零后记账会加 0，「已送达」就丢了。
    appendLog(QStringLiteral("切换规划策略 -> %1").arg(strategyBox_->currentText()));
    replan();
}

void MainWindow::onWeightChanged() {
    const QString key = weightBox_->currentData().toString();
    scene_->setWeightType(key == QStringLiteral("time")
                              ? WeightType::Time
                              : (key == QStringLiteral("cost") ? WeightType::Cost
                                                               : WeightType::Distance));
    appendLog(QStringLiteral("切换权重标签 -> %1").arg(weightBox_->currentText()));
}

void MainWindow::onSimulateTraffic() {
    simulateTrafficAction(-1);
}

void MainWindow::simulateTrafficAction(int atMin) {
    auto log = [this, atMin](const QString& text) { appendLog(text, atMin); };
    const logistics::TrafficReport report = logistics::simulateTrafficChange(
        config_.graph, config_.general.trafficChangeRatio, config_.general.trafficTimeIncreaseMin,
        config_.general.trafficTimeIncreaseMax, rng_.nextU32());

    std::size_t congestedCount = 0;
    std::size_t clearedCount = 0;
    for (const logistics::TrafficChange& change : report.changes) {
        if (change.congested) {
            ++congestedCount;
        } else {
            ++clearedCount;
        }
    }
    log(QStringLiteral("路况变化：改动 %1 条边（新增拥堵 %2 条，转为畅通 %3 条）")
            .arg(report.changes.size())
            .arg(congestedCount)
            .arg(clearedCount));

    const bool trigger = logistics::needsReplan(plan_.nodes, report,
                                                config_.general.trafficTimeIncreaseMin);
    if (!trigger) {
        log(QStringLiteral("受影响边不在当前路径或增幅不足 -> 不触发重规划"));
        syncScene();   // 拥堵标记需要重绘
        return;
    }

    log(QStringLiteral("受影响边位于当前路径且增幅达标 -> 自动触发重规划"));

    // 增量式重规划（D22）：只把**尚未走完的那一段**交给它，
    // 它会按停靠点切段、仅重算走过被命中边的段，停靠顺序不变。
    // 多趟方案或某段重算后不可达时，它内部自动退回全量重算。
    // 切出"尚未走完的那一段"（停靠顺序不变；趟结构已退役，无需保留趟边界）。
    const logistics::RoutePlan remainder = logistics::sliceRemainder(plan_, nodeIndex_);

    // 由核心**回报**是否用了「顺序不变版重放」（规格二 §10 裁定 1），界面不要自己猜。
    bool usedIncremental = false;
    // 换计划之前，把"当前计划里已经走过的停靠点"并入记账（三条重规划路径都要做，
    // 否则界面上的「已送达 N 站」会归 0、「停靠 N 站」会缩水成剩余数）
    logistics::StashState stashInc;
    stashInc.station = state_.stashStation;
    stashInc.items = state_.stashItems;
    plan_ = logistics::replanIncremental(config_.graph, config_.vehicles.front(),
                                         remainingOrders(), remainder, currentTimeMin(),
                                         planWeight_,
                                         report, config_.general.trafficTimeIncreaseMin,
                                         onboardGoods(), &usedIncremental, stashInc);
    afterPlanReplaced();
    log(usedIncremental
            ? QStringLiteral("  → 顺序不变版重放：停靠顺序不变，按新边权重重新放一遍")
            : QStringLiteral("  → 重放不可行（序列不全或路段已不可达），退回全量重算"));
    syncScene();
    updatePanels();
}

std::string MainWindow::nextFreeId(const char* prefix) const {
    char buf[32];
    for (int i = 1; i <= 999; ++i) {
        std::snprintf(buf, sizeof(buf), "%s%03d", prefix, i);
        bool used = false;
        for (const Order& order : config_.orders) {
            if (order.id == buf) {
                used = true;
                break;
            }
        }
        if (!used) {
            return std::string(buf);
        }
    }
    return std::string(prefix) + "999";
}

std::string MainWindow::insertOrderAt(const std::string& a, const std::string& b,
                                     double demand, bool urgent,
                                     logistics::OrderSource source, int windowBaseMin) {
    const QString kindText = urgent ? QStringLiteral("紧急取送单") : QStringLiteral("取送单");
    if (config_.vehicles.empty()) {
        appendLog(QStringLiteral("配置中没有车辆"), windowBaseMin);
        return std::string();
    }
    const logistics::Node* na = config_.graph.findNode(a);
    const logistics::Node* nb = config_.graph.findNode(b);
    if (na == nullptr || nb == nullptr
        || na->type != logistics::NodeType::Delivery
        || nb->type != logistics::NodeType::Delivery) {
        appendLog(kindText + QStringLiteral("：取货点 A、送达点 B 必须是客户（配送）节点"),
                  windowBaseMin);
        return std::string();
    }
    if (a == b) {
        appendLog(kindText + QStringLiteral("：起点 A 与终点 B 不能相同"), windowBaseMin);
        return std::string();
    }
    const double cap = config_.vehicles.front().capacityKg;
    if (!(demand > 0.0) || demand > cap + 1e-9) {
        appendLog(kindText + QStringLiteral("：送货量须在 (0, 载重上限] 内"), windowBaseMin);
        return std::string();
    }
    const logistics::PathResult toA = logistics::shortestPath(
        config_.graph, currentPositionId(), a, logistics::WeightType::Time);
    const logistics::PathResult ab =
        logistics::shortestPath(config_.graph, a, b, logistics::WeightType::Time);
    if (!toA.found || !ab.found) {
        appendLog(kindText + QStringLiteral("：取货点 A 或送达点 B 不可达"), windowBaseMin);
        return std::string();
    }

    Order order;
    order.id = nextFreeId(logistics::orderIdPrefix(source));
    order.pickupNodeId = a;
    order.nodeId = b;
    order.demandKg = std::floor(demand);
    // R4：窗口基准是**这张单的基准时刻**（事件刷单 = 事件刻 mark；手动动作 = 当前时刻），
    // 不是"结算发生的时刻"——批量结算时两者会差出好几个刻。
    order.windowStartMin = windowBaseMin;
    // 普通 [t, 24:00]；紧急 [t, min(t+60, 24:00)]（截断只为不产生跨日窗口，防御性）。
    order.windowEndMin = urgent ? (windowBaseMin + 60 < 24 * 60 ? windowBaseMin + 60 : 24 * 60)
                                : 24 * 60;
    if (order.windowEndMin < order.windowStartMin) {
        order.windowEndMin = order.windowStartMin;
    }
    order.urgent = urgent;
    order.source = source;
    order.served = false;

    const std::vector<Order> pending = remainingOrders();
    logistics::StashState stashIns;
    stashIns.station = state_.stashStation;
    stashIns.items = state_.stashItems;
    // R8：紧急 -> 紧急段尾插（FIFO）；普通 -> 普通段 pair-insertion。urgent 由订单自己带。
    const logistics::InsertResult inserted = logistics::insertOrder(
        config_.graph, config_.vehicles.front(), pending, order, currentPositionId(),
        currentTimeMin(), planWeight_, onboardGoods(), stashIns);

    // R9：通过输入校验即并入订单集合（插入失败也不丢单，保留未服务，后续事件/重规划再试）。
    config_.orders.push_back(order);

    appendLog(QStringLiteral("插入%1 %2：%3 取 → %4 送（货量 %5kg，%6 前送达）")
                  .arg(urgent ? QStringLiteral("紧急取送单") : QStringLiteral("普通取送单"))
                  .arg(QString::fromStdString(order.id))
                  .arg(QString::fromStdString(order.pickupNodeId))
                  .arg(QString::fromStdString(order.nodeId))
                  .arg(order.demandKg, 0, 'f', 0)
                  .arg(minutesToClock(order.windowEndMin)),
              windowBaseMin);
    if (!inserted.warning.empty()) {
        appendLog(QStringLiteral("  ⚠ %1").arg(QString::fromStdString(inserted.warning)),
                  windowBaseMin);
    }
    plan_ = inserted.plan;
    afterPlanReplaced();
    syncScene();
    updatePanels();
    return order.id;
}

std::string MainWindow::insertRandomOrder(bool urgent, logistics::OrderSource source,
                                          int windowBaseMin) {
    // 事件刻刷单 / 手动「插入紧急订单」/ 自检共用：A、B 在**当前图全部配送点**里条件均匀抽取
    // （R2：A 均匀、B 在剩余 n-1 个里一次抽出，固定消耗 2 次随机数）。
    std::vector<std::string> cust;
    for (const logistics::Node& n : config_.graph.nodes()) {
        if (n.type == logistics::NodeType::Delivery) {
            cust.push_back(n.id);
        }
    }
    if (cust.size() < 2) {
        // Q8：抽中了但客观不可执行 -> 照实播报原因、**不重抽**（该刻仍算已抽）。
        appendLog(QStringLiteral("客户节点不足 %1 个，无法生成%2（本刻不重抽）")
                      .arg(cust.size())
                      .arg(urgent ? QStringLiteral("紧急订单") : QStringLiteral("普通订单")),
                  windowBaseMin);
        return std::string();
    }
    int ia = 0;
    int ib = 0;
    if (!logistics::pickDistinctPair(static_cast<int>(cust.size()), rng_, ia, ib)) {
        return std::string();   // 上面已排除 size() < 2，这里只是防御
    }
    // R3：普通 3–15kg、紧急 3–10kg，一律整公斤（floor）。区间上界用开区间，
    // 故 high 分别是 13 / 8（3 + [0,13) / 3 + [0,8)），恰好覆盖到 15 / 10。
    const double demand = std::floor(3.0 + rng_.nextRange(0.0, urgent ? 8.0 : 13.0));
    return insertOrderAt(cust[static_cast<std::size_t>(ia)],
                         cust[static_cast<std::size_t>(ib)], demand, urgent, source,
                         windowBaseMin);
}

void MainWindow::onInsertUrgentOrder() {
    // 无头环境（--self-check-actions / --demo / offscreen 渲染）不能弹模态框：
    // 退化为随机 A/B，保持自动化路径可跑。
    // 手动动作的窗口基准 = **当前软件内时刻**（不是事件刻，R4 只管自动刷单）。
    if (QGuiApplication::platformName() == QStringLiteral("offscreen")) {
        insertRandomOrder(true, logistics::OrderSource::ManualUrgent, currentTimeMin());
        return;
    }

    QStringList customers;
    for (const logistics::Node& n : config_.graph.nodes()) {
        if (n.type == logistics::NodeType::Delivery) {
            customers << QString::fromStdString(n.id);
        }
    }
    if (customers.size() < 2) {
        appendLog(QStringLiteral("客户节点不足，无法插入紧急取送单"));
        return;
    }

    // 与「手工增删」同形态：一张表单一次填完 A / B / 送货量，点确定即插入，
    // 不再连续弹三个输入框。B 的下拉始终排除当前 A，非法组合从源头拦掉。
    QDialog dialog(this);
    dialog.setWindowTitle(QStringLiteral("插入紧急取送单"));

    auto* aCombo = new QComboBox(&dialog);
    aCombo->addItems(customers);
    auto* bCombo = new QComboBox(&dialog);

    const double cap = config_.vehicles.empty() ? 0.0 : config_.vehicles.front().capacityKg;
    auto* demand = new QDoubleSpinBox(&dialog);
    demand->setDecimals(0);
    demand->setRange(1.0, cap > 1.0 ? cap : 1.0);
    demand->setSingleStep(1.0);
    demand->setValue(cap >= 10.0 ? 10.0 : (cap > 1.0 ? cap : 1.0));

    auto refreshB = [&]() {
        const QString a = aCombo->currentText();
        const QString keep = bCombo->currentText();
        bCombo->clear();
        for (const QString& s : customers) {
            if (s != a) {
                bCombo->addItem(s);
            }
        }
        if (!keep.isEmpty()) {
            const int idx = bCombo->findText(keep);
            if (idx >= 0) {
                bCombo->setCurrentIndex(idx);
            }
        }
    };
    connect(aCombo, &QComboBox::currentTextChanged, &dialog,
            [&](const QString&) { refreshB(); });
    refreshB();

    auto* form = new QFormLayout;
    form->addRow(QStringLiteral("取货点 A"), aCombo);
    form->addRow(QStringLiteral("送达点 B"), bCombo);
    form->addRow(QStringLiteral("送货量(kg)"), demand);

    auto* box = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dialog);
    connect(box, &QDialogButtonBox::accepted, &dialog, [&]() {
        if (aCombo->currentText() == bCombo->currentText()) {
            QMessageBox::warning(&dialog, QStringLiteral("插入紧急取送单"),
                                 QStringLiteral("取货点 A 与送达点 B 不能相同"));
            return;
        }
        const std::string id = insertOrderAt(aCombo->currentText().toStdString(),
                                             bCombo->currentText().toStdString(),
                                             demand->value(), true,
                                             logistics::OrderSource::ManualUrgent,
                                             currentTimeMin());
        if (id.empty()) {
            QMessageBox::warning(&dialog, QStringLiteral("插入紧急取送单"),
                                 QStringLiteral("插入失败（端点不可达或货量超限），详见运行日志"));
            return;
        }
        dialog.accept();
    });
    connect(box, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);

    auto* layout = new QVBoxLayout(&dialog);
    layout->addLayout(form);
    layout->addWidget(box);
    dialog.exec();
}

// R10 · 手动「模拟新客户」：建节点 X + 一张**普通取送单**（A = 随机现有配送点 -> B = X）。
// 只由工具栏触发，**不进事件池、不受待处理上限**，但与自动刷单走**同一条插入链**
// （insertOrderAt -> insertOrder -> 普通段 pair-insertion）。
// 新客户必然带来一个配送需求：只加节点而不加订单的话，规划器没有理由访问它——
// 停靠点只由订单决定，新节点会永远不出现在路线里（人工测试第 11 关的缺陷）。
std::string MainWindow::addRandomCustomerAction() {
    // 候选 A 必须在**建节点之前**取：新节点 X 自己也是配送点，先取列表才不会抽到 X。
    std::vector<std::string> cust;
    for (const logistics::Node& n : config_.graph.nodes()) {
        if (n.type == logistics::NodeType::Delivery) {
            cust.push_back(n.id);
        }
    }

    const std::string nodeId = logistics::addRandomCustomer(config_.graph, rng_);
    if (nodeId.empty()) {
        appendLog(QStringLiteral("新增客户失败"));
        return std::string();
    }
    const logistics::Node* created = config_.graph.findNode(nodeId);
    if (created != nullptr) {
        appendLog(QStringLiteral("模拟新客户：新增配送点 %1（%2，子网络 %3）")
                      .arg(QString::fromStdString(nodeId))
                      .arg(QString::fromStdString(created->name))
                      .arg(created->subNetworkId));
    }
    if (cust.empty()) {
        // Q8 同口径：动作客观不可执行时照实播报，不臆造取货点。
        appendLog(QStringLiteral("新增客户失败：找不到可作为取货点 A 的既有配送点"));
        return nodeId;
    }

    // A = 随机现有配送点（R10）；kg 3–15 整公斤（普通单区间，R3）；
    // 窗口 [当前时刻, 24:00]（手动动作的基准就是"用户此刻"，R4/§1.9-1）。
    const int now = currentTimeMin();
    const int ia = rng_.nextInt(0, static_cast<int>(cust.size()) - 1);
    const double demand = std::floor(3.0 + rng_.nextRange(0.0, 13.0));
    insertOrderAt(cust[static_cast<std::size_t>(ia)], nodeId, demand, false,
                  logistics::OrderSource::ManualCustomer, now);
    return nodeId;
}

void MainWindow::onAddRandomCustomer() {
    addRandomCustomerAction();
}

void MainWindow::onCloseRandomRoad() {
    closeRandomRoadAction(-1);
}

void MainWindow::closeRandomRoadAction(int atMin) {
    const std::string closed = logistics::closeRandomRoad(config_.graph, rng_);
    if (closed.empty()) {
        // Q8：抽中了封路但图里没有非桥边 -> 照实播报"无动作"，该刻仍算已抽，不重抽。
        appendLog(QStringLiteral("没有可封闭的道路（本刻不重抽）"), atMin);
        return;
    }
    appendLog(QStringLiteral("模拟道路封闭：%1（含反向）").arg(QString::fromStdString(closed)),
              atMin);
    replan();
}

void MainWindow::onAdvanceStop() {
    if (plan_.status != logistics::PlanStatus::Ok || plan_.nodes.empty()) {
        return;
    }
    if (nodeIndex_ + 1 >= plan_.nodes.size()) {
        reportRouteFinishedOnce();   // 终点只播报一次（原先每个 tick 都刷一遍）
        return;                      // 已结束，不再刷新
    }

    // ---- 先结算"到达之前"的事件刻（D24 §7.7 的严格版）----
    //
    // 一次推进可能跨过好几个事件刻（长途 leg）。**每个刻都在它自己的时刻结算**，
    // 而不是等到达后统一按"到达时刻"结算：后者会让「按站模拟」抽到的同一串事件
    // 全部落到更晚的时钟上（重规划锚点、窗口基准都会跟着漂），于是同一时刻在
    // 「推进一刻」与「推进一站」下不是同一个世界——A1–A7 等价性断言就会红。
    // 结算时车仍在**所在路段的起点**（半路显示语义），与「推进一刻」逐步走刻同构。
    //
    // 期间抽到的事件可能触发重规划并改写 plan_，所以每轮都要重新读计划
    //（不能把 nodeArrivalMin 缓存在循环外）。
    while (!atRouteEnd() && nodeIndex_ + 1 < plan_.nodeArrivalMin.size()
           && nextEventMark_ < plan_.nodeArrivalMin[nodeIndex_ + 1]) {
        advanceClockTo(nextEventMark_);
        settleEventsUpTo(nextEventMark_);
        if (plan_.status != logistics::PlanStatus::Ok || plan_.nodes.empty()) {
            return;   // 事件把计划打成不可行：本刻无可推进
        }
    }
    if (atRouteEnd()) {
        reportRouteFinishedOnce();
        return;
    }

    // 逐个节点前进。**状态转移只发生在这一处**：
    // 位置/时刻 -> arriveAt，动作集（卸/装/降载/取回）-> 下面的动作集段，记账 -> deliverAt。
    // 除此之外任何地方都不得改 state_。
    ++nodeIndex_;
    const std::string node = plan_.nodes[nodeIndex_];
    int arrTime = state_.atTimeMin;
    if (nodeIndex_ < plan_.nodeArrivalMin.size()) {
        arrTime = plan_.nodeArrivalMin[nodeIndex_];
    }
    arriveAt(node, arrTime);
    currentNodeId_ = state_.atNodeId;
    currentTimeMin_ = state_.atTimeMin;

    // ---- 按**停靠动作集**执行状态转移（规格一 MC-E09）----
    //
    // 旧实现靠 nodeTripIndex -> Trip 反查"本趟的 pickups/offloads/reclaimStation"，
    // 并按 nodeId 扣货。新模型下 stops 的动作集就是真源，且**按订单记账**：
    // 同节点多单（A∪B 合并停靠）下按 nodeId 扣货会把别的单一起扣掉（issue #5 的坑）。
    const bool isStopHere = nodeIndex_ < plan_.nodeIsStop.size()
                            && plan_.nodeIsStop[nodeIndex_];
    const bool cursorMatches = isStopHere && stopCursor_ < plan_.stops.size()
                               && plan_.stops[stopCursor_].nodeId == plan_.nodes[nodeIndex_];
    const logistics::Stop* hereStop = cursorMatches ? &plan_.stops[stopCursor_] : nullptr;

    if (hereStop != nullptr) {
        // ① 先全部卸：送达 + 降载（停靠内先卸后装）。
        for (const logistics::StopDelivery& d : hereStop->deliveries) {
            for (std::size_t i = 0; i < state_.onboard.size(); ++i) {
                if (state_.onboard[i].orderId != d.orderId) {
                    continue;
                }
                state_.onboard.erase(state_.onboard.begin() + static_cast<std::ptrdiff_t>(i));
                break;
            }
            for (Order& o : config_.orders) {
                if (o.id == d.orderId) {
                    o.served = true;
                }
            }
        }
        for (const std::string& orderId : hereStop->offloads) {
            VehicleState::OffloadKey key;
            key.station = hereStop->nodeId;
            key.orderId = orderId;
            if (!state_.appliedOffloads.insert(key).second) {
                continue;   // 物理事实只发生一次
            }
            bool removed = false;
            for (std::size_t i = 0; i < state_.onboard.size(); ++i) {
                if (state_.onboard[i].orderId != orderId) {
                    continue;
                }
                state_.stashItems.push_back(state_.onboard[i]);
                state_.onboard.erase(state_.onboard.begin() + static_cast<std::ptrdiff_t>(i));
                removed = true;
                break;
            }
            state_.stashStation = hereStop->nodeId;
            appendLog(QStringLiteral("降载：在 %1 卸下订单 %2 的货 %3kg（暂存，稍后取回）%4")
                          .arg(QString::fromStdString(hereStop->nodeId))
                          .arg(QString::fromStdString(orderId))
                          .arg(orderKg(orderId), 0, 'f', 0)
                          .arg(removed ? QString() : QStringLiteral("（车上无此货，仅记账）")));
        }
        state_.loadKg = state_.sumOnboard();

        // ② 再全部装：取货 + 取回。
        for (const std::string& orderId : hereStop->pickups) {
            if (!state_.pickedOrderIds.insert(orderId).second) {
                continue;   // 已取过，不重复装
            }
            logistics::Order* target = nullptr;
            for (Order& o : config_.orders) {
                if (o.id == orderId) {
                    target = &o;
                }
            }
            if (target != nullptr) {
                target->picked = true;
                logistics::OnboardItem item;
                item.nodeId = target->nodeId;
                item.kg = target->demandKg;
                item.orderId = target->id;
                state_.onboard.push_back(item);
                appendLog(QStringLiteral("取货 %1：在 %2 装上包裹 %3kg（送往 %4）")
                              .arg(QString::fromStdString(orderId))
                              .arg(QString::fromStdString(hereStop->nodeId))
                              .arg(target->demandKg, 0, 'f', 0)
                              .arg(QString::fromStdString(target->nodeId)));
            }
        }
        for (const std::string& orderId : hereStop->reclaims) {
            for (std::size_t i = 0; i < state_.stashItems.size(); ++i) {
                if (state_.stashItems[i].orderId != orderId) {
                    continue;
                }
                state_.onboard.push_back(state_.stashItems[i]);
                appendLog(QStringLiteral("取回暂存货：在 %1 取回订单 %2 的货 %3kg，继续配送")
                              .arg(QString::fromStdString(hereStop->nodeId))
                              .arg(QString::fromStdString(orderId))
                              .arg(state_.stashItems[i].kg, 0, 'f', 0));
                state_.stashItems.erase(state_.stashItems.begin()
                                        + static_cast<std::ptrdiff_t>(i));
                break;
            }
        }
        if (state_.stashItems.empty()) {
            state_.stashStation.clear();
        }
        state_.loadKg = state_.sumOnboard();
    }

    // 只有**下标与节点对得上**时才认作一次停靠送达。
    // 曾经这里只看 nodeIsStop 与下标，重规划后两者可能失同步，
    // 于是拿到的是**另一个停靠点**：扣错货（车上有的没扣、没有的扣了），
    // 表现为"2 趟送完 740kg"这种物理上不可能的结果。
    if (hereStop != nullptr) {
        const Stop& stop = *hereStop;
        ++stopCursor_;
        const bool transitOnly = stop.pickups.empty() && stop.deliveries.empty();
        if (transitOnly) {
            appendLog(QStringLiteral("中转站 %1（到达 %2）：%3%4")
                          .arg(QString::fromStdString(stop.nodeId))
                          .arg(minutesToClock(stop.arrivalMin))
                          .arg(stop.offloads.empty()
                                   ? QString()
                                   : QStringLiteral("降载 %1 张单 ")
                                         .arg(stop.offloads.size()))
                          .arg(stop.reclaims.empty()
                                   ? QString()
                                   : QStringLiteral("取回 %1 张单").arg(stop.reclaims.size())));
        } else if (!stop.pickups.empty() && stop.deliveries.empty()) {
            appendLog(QStringLiteral("取货停靠 %1（到达 %2）")
                          .arg(QString::fromStdString(stop.nodeId))
                          .arg(minutesToClock(stop.arrivalMin)));
        } else {
            // 该停靠若同时是取货点 A，说明这里是「顺路交付」。
            const bool passingDelivery = !stop.pickups.empty();
            QString ids;
            for (const logistics::StopDelivery& d : stop.deliveries) {
                if (!ids.isEmpty()) {
                    ids += QLatin1Char('/');
                }
                ids += QString::fromStdString(d.orderId);
            }
            appendLog(QStringLiteral("%1 %2（订单 %3，到达 %4%5）")
                          .arg(passingDelivery ? QStringLiteral("顺路交付")
                                               : QStringLiteral("送达"))
                          .arg(QString::fromStdString(stop.nodeId))
                          .arg(ids)
                          .arg(minutesToClock(stop.arrivalMin))
                          .arg(stop.late
                                   ? QStringLiteral("，超时 penalty %1min").arg(stop.penaltyMin)
                                   : QString()));
        }
        // 导出用的**到达时载重明细**（在卸货之前记，反映"到达这一刻车上载着什么"）。
        {
            const double cap = config_.vehicles.empty() ? 0.0 : config_.vehicles.front().capacityKg;
            recordRun(QStringLiteral("    └ 到达 %1 时车上载重：订单货 %2 kg"
                                     "（载重上限 %3 kg）")
                          .arg(QString::fromStdString(stop.nodeId))
                          .arg(state_.loadKg, 0, 'f', 0)
                          .arg(cap, 0, 'f', 0));
        }
        deliverAt(stop);
    } else if (nodeIndex_ + 1 == plan_.nodes.size()) {
        appendLog(QStringLiteral("返回仓库 %1（到达 %2），本次配送结束")
                      .arg(QString::fromStdString(currentNodeId_))
                      .arg(minutesToClock(currentTimeMin_)));
    } else {
        appendLog(QStringLiteral("经过 %1（到达 %2）")
                      .arg(QString::fromStdString(currentNodeId_))
                      .arg(minutesToClock(currentTimeMin_)));
    }

    // 恰好落在**到达时刻**上的事件刻：到达之后结算（与「推进一刻」在同一个位置、
    // 同一个时钟上结算，顺序一致）。必须放在 syncScene 之前 —— 事件可能重规划。
    settleEventsUpTo(state_.atTimeMin);

    // 推进后必须刷新画布：车辆位置标记要跟着移动
    syncScene();
    updatePanels();
}

void MainWindow::onReplan() {
    appendLog(QStringLiteral("手动触发重新规划（从 %1，时刻 %2）")
                  .arg(QString::fromStdString(currentPositionId()))
                  .arg(minutesToClock(currentTimeMin())));
    replan();
}

// ---- 两种模拟模式（互斥；同一 QActionGroup）-----------------------------------
//
// 事件**不**由"推进次数"或"真实秒数"触发，而由**软件内时钟跨过事件刻**触发。
// 两种模式的差别只在推进粒度：按站模拟 = 一站；按时间模拟 = 一刻（下一个事件刻）。
//
// 所有状态迁移都收口在本函数：QActionGroup::triggered 每次点击恰发一次。
//   · 点到"另一个模式"      -> 切换：旧的已被 exclusive 组自动取消，这里只启新表
//   · 点到"正在运行的模式"  -> 再点一次 = 停止：主动取消勾选 + 停表
// 之所以不用 QAction::toggled：它在 triggered **之前**发出，且 exclusive 组不允许
// 用户把已选中的动作取消勾选 —— 两者叠加会令"首次勾选"被误判成"请求停止"。
void MainWindow::onSimModeTriggered(QAction* action) {
    if (action == nullptr) {
        return;
    }
    const int mode = (action == timeSimAction_) ? 2 : 1;

    if (simMode_ == mode) {
        // 再点一次正在运行的模式 = 请求停止。
        if (action->isChecked()) {
            action->setChecked(false);   // exclusive 组里必须程序化取消
        }
        simMode_ = 0;
        if (simTimer_ != nullptr) {
            simTimer_->stop();
        }
        appendLog(mode == 1 ? QStringLiteral("按站模拟关闭：自动推进已停止")
                            : QStringLiteral("按时间模拟关闭：自动推进已停止"));
        return;
    }

    simMode_ = mode;
    if (simTimer_ != nullptr) {
        simTimer_->start(simIntervalMs_);   // 1 秒/步，两种模式共用同一节奏
    }
    if (mode == 1) {
        appendLog(QStringLiteral("按站模拟开启：每 1 秒自动推进一站；"
                                 "事件仍按软件内时间每 %1min 抽取")
                      .arg(config_.general.eventIntervalMin));
    } else {
        appendLog(QStringLiteral("按时间模拟开启：每 1 秒自动推进一刻"
                                 "（= 下一个事件刻，每 %1min 一个）")
                      .arg(config_.general.eventIntervalMin));
    }
}

void MainWindow::onSimTick() {
    if (simMode_ == 0) {
        return;   // 必须可随时关闭，且关闭后不再产生任何副作用
    }
    if (simMode_ == 1) {
        onAdvanceStop();
    } else {
        onAdvanceMoment();
    }
    if (atRouteEnd()) {
        // 两种模拟模式共用这一条停机路径（停表 + 清除勾选）；
        // 但**播报要分两种**：不可行不是"完成"，播成完成就是假成功。
        stopSim(routeTrulyFinished()
                    ? QStringLiteral("本次配送已完成，模拟结束")
                    : QStringLiteral("计划不可行或为空，模拟停止"));
    }
}

void MainWindow::onAdvanceMoment() {
    if (plan_.status != logistics::PlanStatus::Ok || plan_.nodes.empty()) {
        return;
    }
    if (atRouteEnd()) {
        reportRouteFinishedOnce();
        return;
    }

    // 「推进一刻」= 推进到**下一个事件刻**。
    const int target = nextEventMark_;

    // **逐节点重放**，绝不跳节点：其间每个跨越的节点都必须走同一套状态转移
    // （arriveAt / deliverAt / loadForCurrentTrip），否则送达/装卸/存货记账会与实际
    // 脱节 —— 这正是 P25/P28 花好几轮才根除的那类缺陷。
    // 循环条件**每轮重新读当前 plan_**：期间抽到的事件可能触发重规划并改写它。
    while (!atRouteEnd() && nodeIndex_ + 1 < plan_.nodeArrivalMin.size()
           && plan_.nodeArrivalMin[nodeIndex_ + 1] <= target) {
        onAdvanceStop();
    }

    // 车卡在 i→i+1 途中：剩余的事件刻也**各按自己的时刻**结算（位置留在 i），
    // 最后才把显示时钟推到 target —— 与「推进一站」在同一个刻、同一个位置结算。
    while (!atRouteEnd() && nextEventMark_ <= target) {
        advanceClockTo(nextEventMark_);
        settleEventsUpTo(nextEventMark_);
    }
    if (!atRouteEnd() && state_.atTimeMin < target) {
        advanceClockTo(target);
    }

    syncScene();
    updatePanels();
}

bool MainWindow::atRouteEnd() const {
    return plan_.status != logistics::PlanStatus::Ok || plan_.nodes.empty()
           || nodeIndex_ + 1 >= plan_.nodes.size();
}

// 「走完了」与「计划不可行/为空」是两件事：后者也满足 atRouteEnd()，
// 但把它当成"配送完成"会播报假成功（界面上同时写着「规划不可行」）。
bool MainWindow::routeTrulyFinished() const {
    return plan_.status == logistics::PlanStatus::Ok && !plan_.nodes.empty()
           && nodeIndex_ + 1 >= plan_.nodes.size();
}

void MainWindow::reportRouteFinishedOnce() {
    if (routeFinishedReported_) {
        return;   // 终点只播报一次，消除原来"每个 tick 都刷一遍"的噪音
    }
    routeFinishedReported_ = true;
    appendLog(QStringLiteral("本次配送已完成：车辆已在仓库 %1")
                  .arg(QString::fromStdString(currentNodeId_)));
}

void MainWindow::stopSim(const QString& reason) {
    if (simTimer_ != nullptr) {
        simTimer_->stop();
    }
    simMode_ = 0;
    // 勾选状态只由 QActionGroup::triggered 收口，**没有**接 QAction::toggled；
    // 程序化 setChecked(false) 也不会发 triggered —— 所以这里不需要信号屏蔽器。
    // （曾用 QSignalBlocker 挡住一个并不存在的递归触发，属死代码，已删。）
    if (stationSimAction_ != nullptr) {
        stationSimAction_->setChecked(false);
    }
    if (timeSimAction_ != nullptr) {
        timeSimAction_->setChecked(false);
    }
    // 播报**一次**（调用方把整句文案传进来，如「本次配送已完成，模拟结束」）
    appendLog(reason);
}

// 时间流逝也是一次物理事件——但它**只动时刻、不动位置**：
// 车留在最后一个已到达的节点上，那正是它此刻所在路段的起点。
void MainWindow::advanceClockTo(int timeMin) {
    if (timeMin <= state_.atTimeMin) {
        return;   // 时间不倒退
    }
    state_.atTimeMin = timeMin;
    currentTimeMin_ = timeMin;
}

// R6 计数：只数**事件来源**（event_normal / event_urgent）且未送达的订单。
//   · "未送达" 含"已取在车"（picked && !served）——上限管的是待处理量，不是"还没取"；
//   · 初始 25 单不占额度，否则普通额度开局即满（25 > 6），普通订单事件等于被关掉；
//   · 手动动作（manual_customer / manual_urgent）是用户显式动作，同样不占额度。
int MainWindow::pendingEventOrderCount(bool urgent) const {
    const logistics::OrderSource want = urgent ? logistics::OrderSource::EventUrgent
                                               : logistics::OrderSource::EventNormal;
    int count = 0;
    for (const Order& order : config_.orders) {
        if (order.served || order.source != want) {
            continue;
        }
        ++count;
    }
    return count;
}

logistics::Config MainWindow::reloadInitialConfig() const {
    logistics::Config config;
    std::string error;
    if (!configPath_.empty() && logistics::ConfigLoader::load(configPath_, config, error)) {
        return config;
    }
    return initialConfig_;
}

// 等价性守卫 A1 的取数口径：运行日志里所有「[事件] …」行（含时刻前缀）。
// 与「导出运行日志」看到的是同一批行；不经过任何显示函数的再加工。
QStringList MainWindow::eventLogLines() const {
    QStringList out;
    for (const QString& line : runLog_) {
        if (line.contains(QStringLiteral("[事件]"))) {
            out << line;
        }
    }
    return out;
}

// 等价性守卫 A2–A7 的终态快照：逐项列出**界面面板同源的事实**
//（订单表的已送达集合、超时面板的已发生 penalty、车辆面板的在车货、
//  停靠面板的已执行账本、状态栏的位置与时刻、事件刻）。
// 刻意不调用 nodeIndexAtTime 这类"显示用"函数反推——那会把断言变成自证。
QString MainWindow::equivalenceSnapshot() const {
    QString out;

    // A2 已送达订单集合（订单表「状态」列的数据源）
    QStringList served;
    for (const Order& order : config_.orders) {
        if (order.served) {
            served << QString::fromStdString(order.id);
        }
    }
    out += QStringLiteral("A2 已送达订单: %1\n").arg(served.join(QLatin1Char(',')));

    // A3 已发生 penalty（车辆面板 / 超时面板的数据源）
    out += QStringLiteral("A3 已发生 penalty: %1 min\n").arg(state_.incurredPenaltyMin);
    QStringList lateList;
    for (const VehicleState::LateStop& late : state_.deliveredLate) {
        lateList << QStringLiteral("%1@%2:+%3")
                        .arg(QString::fromStdString(late.orderId))
                        .arg(QString::fromStdString(late.nodeId))
                        .arg(late.penaltyMin);
    }
    out += QStringLiteral("A3 已发生超时明细: %1\n").arg(lateList.join(QLatin1Char(',')));

    // A4 在车货（订单 + kg）
    QStringList onboard;
    for (const logistics::OnboardItem& item : state_.onboard) {
        onboard << QStringLiteral("%1:%2kg@%3")
                       .arg(QString::fromStdString(item.orderId))
                       .arg(item.kg, 0, 'f', 0)
                       .arg(QString::fromStdString(item.nodeId));
    }
    out += QStringLiteral("A4 在车货: %1（合计 %2kg / 载重 %3kg）\n")
               .arg(onboard.join(QLatin1Char(',')))
               .arg(state_.sumOnboard(), 0, 'f', 0)
               .arg(state_.loadKg, 0, 'f', 0);

    // A5 车内/站内的已执行账本（停靠面板 + 中转站面板的数据源）
    QStringList picked;
    for (const std::string& id : state_.pickedOrderIds) {
        picked << QString::fromStdString(id);
    }
    QStringList offloaded;
    for (const VehicleState::OffloadKey& key : state_.appliedOffloads) {
        offloaded << (QString::fromStdString(key.station) + QLatin1Char('/')
                      + QString::fromStdString(key.orderId));
    }
    QStringList stash;
    for (const logistics::OnboardItem& item : state_.stashItems) {
        stash << QStringLiteral("%1:%2kg").arg(QString::fromStdString(item.orderId))
                     .arg(item.kg, 0, 'f', 0);
    }
    out += QStringLiteral("A5 已送达停靠 %1 ｜ 已取货 [%2] ｜ 已降载 [%3] ｜ 暂存站 %4 [%5]\n")
               .arg(state_.servedStops)
               .arg(picked.join(QLatin1Char(',')))
               .arg(offloaded.join(QLatin1Char(',')))
               .arg(QString::fromStdString(state_.stashStation))
               .arg(stash.join(QLatin1Char(',')));

    // A6 位置与时刻（状态栏时钟 + 地图车辆位置的数据源）
    out += QStringLiteral("A6 位置 %1 @ %2\n")
               .arg(QString::fromStdString(state_.atNodeId))
               .arg(minutesToClock(state_.atTimeMin));

    // A7 下一个事件刻
    out += QStringLiteral("A7 下一个事件刻 %1\n").arg(minutesToClock(nextEventMark_));
    return out;
}

// 结算所有 <= timeMin 的**事件刻**，每刻恰抽 1 个事件。
//
// 这一层刻意挂在"软件时间前进"上、而不是挂在某个按钮上：因此「按站模拟」（一站一步）
// 与「推进一刻」（一刻一步）看到的是**同一串事件** —— 这也是等价性断言能成立的前提。
void MainWindow::settleEventsUpTo(int timeMin) {
    const int interval = config_.general.eventIntervalMin > 0
                             ? config_.general.eventIntervalMin : 15;
    while (nextEventMark_ <= timeMin) {
        const int mark = nextEventMark_;
        nextEventMark_ = logistics::nextEventTimeMin(mark, interval);

        // 权重由 event_level_* 按代码内固定表（1/3/9/27）派生——唯一派生点见 core/Config.h。
        logistics::EventWeights w;
        w.traffic  = logistics::eventLevelWeight(config_.general.eventLevelTraffic);
        w.urgent   = logistics::eventLevelWeight(config_.general.eventLevelUrgent);
        w.customer = logistics::eventLevelWeight(config_.general.eventLevelCustomer);
        w.closure  = logistics::eventLevelWeight(config_.general.eventLevelClosure);

        // R6/R7：抽签前**只**把"已达上限"的类别权重置 0（不预扫"动作可不可执行"），
        // 两类都满时仍每刻恰 1 抽、不出现空刻。额度只数事件来源、未送达（含在车）。
        const bool normalCapped =
            pendingEventOrderCount(false) >= kMaxPendingNormalOrders;
        const bool urgentCapped =
            pendingEventOrderCount(true) >= kMaxPendingUrgentOrders;
        if (normalCapped) {
            w.customer = 0;
        }
        if (urgentCapped) {
            w.urgent = 0;
        }
        QString capNote;
        if (normalCapped && urgentCapped) {
            capNote = QStringLiteral("（普通订单待处理已满 %1、紧急订单待处理已满 %2，"
                                     "本次抽签不含这两类）")
                          .arg(kMaxPendingNormalOrders)
                          .arg(kMaxPendingUrgentOrders);
        } else if (normalCapped) {
            capNote = QStringLiteral("（普通订单待处理已满 %1，本次抽签不含普通订单）")
                          .arg(kMaxPendingNormalOrders);
        } else if (urgentCapped) {
            capNote = QStringLiteral("（紧急订单待处理已满 %1，本次抽签不含紧急订单）")
                          .arg(kMaxPendingUrgentOrders);
        }

        // **每刻恰 1 次抽签、1 次抽签恰消耗 1 次随机数**（含全零权重的保底分支）。
        const logistics::SimEventKind kind = logistics::pickEvent(w, rng_);
        // **传事件刻 mark**：一次推进可能跨过好几个事件刻（"推进一刻"或按站模拟跨 15min），
        // 那些事件是在同一瞬间被批量结算的。若用当前时刻做前缀，导出文件会写成
        // 「09:00 [事件] 08:15 抽取…」——把"记录时刻"误当"事件时刻"，正是最容易被
        // 误读的地方。事件行一律用**它自己的时刻**。
        appendLog(QStringLiteral("[事件] %1 抽取：%2%3")
                      .arg(minutesToClock(mark))
                      .arg(QString::fromUtf8(logistics::simEventName(kind)))
                      .arg(capNote),
                  mark);

        // Q8：抽中后动作客观不可执行（配送点 <2、无非桥边）时，动作函数**照实播报原因**、
        // 本刻仍算已抽、**不重抽** —— 抽签次数确定，等价性断言才成立。
        // 四个动作都把**事件刻**当日志前缀传进去（批量结算时不会全记成同一时刻）。
        switch (kind) {
            case logistics::SimEventKind::Traffic:
                simulateTrafficAction(mark);
                break;
            case logistics::SimEventKind::UrgentOrder:
                insertRandomOrder(true, logistics::OrderSource::EventUrgent, mark);
                break;
            case logistics::SimEventKind::NormalOrder:
                insertRandomOrder(false, logistics::OrderSource::EventNormal, mark);
                break;
            case logistics::SimEventKind::RoadClosure:
                closeRandomRoadAction(mark);
                break;
        }
    }
}

// 邻接表 / 邻接矩阵的**表格**展示（B3 修订）。
// 原先只有命令行打印的文本文件，用户反馈"不够直观"——这里用真正的表格呈现。
void MainWindow::onShowGraphTables() {
    QDialog dialog(this);
    dialog.setWindowTitle(QStringLiteral("图的表示（邻接表 / 邻接矩阵）"));
    dialog.resize(1100, 720);

    const std::vector<logistics::Node>& nodes = config_.graph.nodes();
    const int n = static_cast<int>(nodes.size());
    auto* tabs = new QTabWidget(&dialog);

    // ---- 邻接表：每个节点一行，列出其全部出边
    auto* listTable = new QTableWidget(n, 3, tabs);
    listTable->setHorizontalHeaderLabels({QStringLiteral("节点"), QStringLiteral("类型"),
                                          QStringLiteral("出边  目标(距离km/耗时min/成本元)")});
    for (int i = 0; i < n; ++i) {
        const logistics::Node& node = nodes[static_cast<std::size_t>(i)];
        listTable->setItem(i, 0, new QTableWidgetItem(QString::fromStdString(node.id)));
        listTable->setItem(i, 1, new QTableWidgetItem(typeText(node.type)));
        QString out;
        for (const logistics::Edge& e : config_.graph.outEdges(node.id)) {
            if (!out.isEmpty()) {
                out += QStringLiteral("   ");
            }
            out += QString::fromStdString(e.toId) + QStringLiteral("(")
                   + QString::number(e.distanceKm, 'f', 1) + QStringLiteral("/")
                   + QString::number(e.timeMin, 'f', 1) + QStringLiteral("/")
                   + QString::number(e.costYuan, 'f', 1) + QStringLiteral(")")
                   + (e.congested ? QStringLiteral("[堵]") : QString());
        }
        listTable->setItem(i, 2, new QTableWidgetItem(out));
    }
    listTable->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
    listTable->horizontalHeader()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
    listTable->horizontalHeader()->setSectionResizeMode(2, QHeaderView::Stretch);
    listTable->verticalHeader()->setVisible(false);
    listTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
    tabs->addTab(listTable, QStringLiteral("邻接表"));

    // ---- 邻接矩阵：真正的 N x N 表格
    auto* matrixTable = new QTableWidget(n, n + 1, tabs);
    QStringList headers;
    headers << QStringLiteral("ID");
    for (const logistics::Node& node : nodes) {
        headers << QString::fromStdString(node.id);
    }
    matrixTable->setHorizontalHeaderLabels(headers);
    for (int r = 0; r < n; ++r) {
        const logistics::Node& row = nodes[static_cast<std::size_t>(r)];
        matrixTable->setItem(r, 0, new QTableWidgetItem(QString::fromStdString(row.id)));
        for (int c = 0; c < n; ++c) {
            const logistics::Node& col = nodes[static_cast<std::size_t>(c)];
            const logistics::Edge* e = config_.graph.findEdge(row.id, col.id);
            const QString text =
                (e == nullptr)
                    ? QStringLiteral("-")
                    : QString::number(logistics::pickWeight(e->distanceKm, e->timeMin,
                                                            e->costYuan, scene_->weightType()),
                                      'f', 1);
            matrixTable->setItem(r, c + 1, new QTableWidgetItem(text));
        }
    }
    matrixTable->verticalHeader()->setVisible(false);
    matrixTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
    tabs->addTab(matrixTable, QStringLiteral("邻接矩阵"));

    auto* layout = new QVBoxLayout(&dialog);
    layout->addWidget(tabs);
    auto* box = new QDialogButtonBox(QDialogButtonBox::Close, &dialog);
    connect(box, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
    layout->addWidget(box);
    dialog.exec();
}

void MainWindow::onManualEdit() {
    QDialog dialog(this);
    dialog.setWindowTitle(QStringLiteral("手工增删节点 / 边"));

    // 简化版：只选起点、终点两个节点。
    //   · 起点 == 终点 → 删除该节点（连同其全部出入边）；
    //   · 起点 != 终点 → 删除两者之间的双向边。
    // 添加边仍要求起点 != 终点。节点一律从图中选，杜绝手输非法 ID。
    auto* fromCombo = new QComboBox(&dialog);
    auto* toCombo = new QComboBox(&dialog);

    auto rebuild = [&](const QString& keepFrom, const QString& keepTo) {
        QStringList ids;
        for (const logistics::Node& n : config_.graph.nodes()) {
            ids << QString::fromStdString(n.id);
        }
        {
            const QSignalBlocker blocker(fromCombo);
            fromCombo->clear();
            fromCombo->addItems(ids);
            const int i = fromCombo->findText(keepFrom);
            if (i >= 0) {
                fromCombo->setCurrentIndex(i);
            }
        }
        {
            const QSignalBlocker blocker(toCombo);
            toCombo->clear();
            toCombo->addItems(ids);
            int i = keepTo.isEmpty() ? -1 : toCombo->findText(keepTo);
            if (i < 0 && toCombo->count() > 1) {
                i = 1;   // 默认选另一个节点，落到"删边"模式
            }
            if (i >= 0) {
                toCombo->setCurrentIndex(i);
            }
        }
    };
    rebuild(QString(), QString());

    auto* form = new QFormLayout;
    form->addRow(QStringLiteral("起点节点"), fromCombo);
    form->addRow(QStringLiteral("终点节点"), toCombo);

    auto* hint = new QLabel(
        QStringLiteral("起点 = 终点：删除该节点；起点 ≠ 终点：删除两者之间的双向边。\n"
                       "添加边要求起点与终点不同。"),
        &dialog);
    hint->setWordWrap(true);

    auto* addEdgeBtn = new QPushButton(QStringLiteral("添加边"), &dialog);
    auto* delBtn = new QPushButton(QStringLiteral("删除"), &dialog);
    auto* buttons = new QHBoxLayout;
    buttons->addWidget(addEdgeBtn);
    buttons->addWidget(delBtn);

    auto* closeBox = new QDialogButtonBox(QDialogButtonBox::Close, &dialog);

    auto* layout = new QVBoxLayout(&dialog);
    layout->addLayout(form);
    layout->addWidget(hint);
    layout->addLayout(buttons);
    layout->addWidget(closeBox);

    connect(closeBox, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);

    connect(addEdgeBtn, &QPushButton::clicked, this, [&] {
        const std::string from = fromCombo->currentText().toStdString();
        const std::string to = toCombo->currentText().toStdString();
        if (from.empty() || to.empty()) {
            appendLog(QStringLiteral("添加边失败：请选择两个节点"));
            return;
        }
        if (from == to) {
            appendLog(QStringLiteral("添加边失败：起点与终点不能相同（同点表示删除节点）"));
            return;
        }

        // 与"删除边"对称：删除会同时删掉两个方向，添加也应构建双向边，
        // 否则操作不对称，用户加完会发现只多了一条单向边。
        logistics::Edge forward;
        logistics::Edge backward;
        if (!logistics::makeSyntheticEdge(config_.graph, from, to, forward)
            || !logistics::makeSyntheticEdge(config_.graph, to, from, backward)) {
            appendLog(QStringLiteral("添加边失败：端点坐标重合（%1 <-> %2）")
                          .arg(QString::fromStdString(from), QString::fromStdString(to)));
            return;
        }
        if (!config_.graph.addEdge(forward)) {
            appendLog(QStringLiteral("添加边失败：%1 <-> %2 已存在")
                          .arg(QString::fromStdString(from), QString::fromStdString(to)));
            return;
        }
        config_.graph.addEdge(backward);
        appendLog(QStringLiteral("手工添加边 %1 <-> %2（双向）")
                      .arg(QString::fromStdString(from), QString::fromStdString(to)));
        rebuild(fromCombo->currentText(), toCombo->currentText());
        replan();
    });

    connect(delBtn, &QPushButton::clicked, this, [&] {
        const std::string from = fromCombo->currentText().toStdString();
        const std::string to = toCombo->currentText().toStdString();
        if (from.empty() || to.empty()) {
            appendLog(QStringLiteral("删除失败：请选择节点"));
            return;
        }
        if (from == to) {
            // 同点 = 节点删除模式（含其全部出入边）。
            if (!config_.graph.removeNode(from)) {
                appendLog(QStringLiteral("删除节点失败：%1 不存在")
                              .arg(QString::fromStdString(from)));
                return;
            }
            appendLog(QStringLiteral("手工删除节点 %1（含其全部出入边）")
                          .arg(QString::fromStdString(from)));
        } else {
            // 异点 = 删除两点之间的双向边。
            const bool okForward = config_.graph.removeEdge(from, to);
            const bool okBackward = config_.graph.removeEdge(to, from);
            appendLog(okForward || okBackward
                          ? QStringLiteral("手工删除边 %1 <-> %2")
                                .arg(QString::fromStdString(from), QString::fromStdString(to))
                          : QStringLiteral("删除边失败：不存在 %1 <-> %2")
                                .arg(QString::fromStdString(from), QString::fromStdString(to)));
        }
        rebuild(fromCombo->currentText(), toCombo->currentText());
        replan();
    });

    dialog.exec();
}

void MainWindow::updatePanels() {
    // 状态栏的软件内时间：无条件刷新（没有车辆、计划不可行时照样要能看到"现在几点"）
    if (clockLabel_ != nullptr) {
        clockLabel_->setText(QStringLiteral("软件内时间：%1")
                                 .arg(minutesToClock(currentTimeMin())));
    }

    // 分母为 0 的率显示「—」（规格一 §6.3），不是 0%/100%。
    const auto rateText = [](int hit, int total) -> QString {
        if (total <= 0) {
            return QStringLiteral("—");
        }
        return QStringLiteral("%1%（%2/%3）")
            .arg(100.0 * static_cast<double>(hit) / static_cast<double>(total), 0, 'f', 1)
            .arg(hit)
            .arg(total);
    };

    // 路线信息
    QString route;
    if (plan_.status == logistics::PlanStatus::Ok) {
        route += QStringLiteral("策略：%1\n").arg(strategyBox_ != nullptr
                                                      ? strategyBox_->currentText()
                                                      : QString());
        route += QStringLiteral("总距离：%1 km\n").arg(plan_.totalDistanceKm, 0, 'f', 1);
        route += QStringLiteral("总耗时：%1 min（%2）\n")
                     .arg(plan_.totalTimeMin, 0, 'f', 1)
                     .arg(minutesToClock(static_cast<int>(std::llround(plan_.totalTimeMin))));
        route += QStringLiteral("总成本：%1 元\n").arg(plan_.totalCostYuan, 0, 'f', 1);
        // 已经发生的超时惩罚 + 剩余计划预计的惩罚。只显示 plan_.totalPenaltyMin
        // 会让"已经发生的"在重规划后凭空消失。
        route += QStringLiteral("总 penalty：%1 min（已发生 %2 + 剩余计划 %3）\n")
                     .arg(state_.incurredPenaltyMin + plan_.totalPenaltyMin)
                     .arg(state_.incurredPenaltyMin)
                     .arg(plan_.totalPenaltyMin);
        // 「已送达」与「停靠总数」都必须跨重规划累计：
        // 只看 plan_.stops 的话，重规划后前者归 0、后者缩水成"剩余要送的"。
        // 三者全部来自车辆状态：已送达是累计事实；"停靠 N 站"= 已送达 + 尚未走完的；
        const std::size_t remainingStops =
            plan_.stops.size() > stopCursor_ ? plan_.stops.size() - stopCursor_ : 0;
        // 停靠 S 站 · 动作 A 次 · 已送达 M 站（MC-E03：动作数 = Σ取+Σ送，
        // 与 --plan-summary 同口径；趟次三段与「各趟」列表已随 #37 退役）。
        int actionCount = 0;
        for (const Stop& st : plan_.stops) {
            actionCount += static_cast<int>(st.pickups.size() + st.deliveries.size());
        }
        route += QStringLiteral("停靠 %1 站 · 动作 %2 次 · 已送达 %3 站\n")
                     .arg(state_.servedStops + static_cast<int>(remainingStops))
                     .arg(actionCount)
                     .arg(state_.servedStops);
        // 三项新指标（MC-E03）：分母 0 记「—」；送达准时率按紧急/普通分列。
        route += QStringLiteral("取货准时率：%1\n").arg(rateText(plan_.metrics.pickupOnTimeCount,
                                                                plan_.metrics.pickupTotal));
        route += QStringLiteral("送达准时率：%1（紧急 %2 · 普通 %3）\n")
                     .arg(rateText(plan_.metrics.deliveryOnTimeCount,
                                   plan_.metrics.deliveryTotal))
                     .arg(rateText(plan_.metrics.urgentOnTimeCount,
                                   plan_.metrics.urgentTotal))
                     .arg(rateText(plan_.metrics.normalOnTimeCount,
                                   plan_.metrics.normalTotal));
        route += QStringLiteral("平均在车：%1 kg（利用率 %2% · 峰值 %3 kg）\n")
                     .arg(plan_.metrics.avgLoadKg, 0, 'f', 1)
                     .arg(plan_.metrics.loadUtilization * 100.0, 0, 'f', 1)
                     .arg(plan_.metrics.peakLoadKg, 0, 'f', 1);
        route += QStringLiteral("\n完整序列：\n");
        for (std::size_t i = 0; i < plan_.nodes.size(); ++i) {
            route += QString::fromStdString(plan_.nodes[i]);
            route += (i + 1 == plan_.nodes.size()) ? QString() : QStringLiteral(" -> ");
        }
    } else {
        route = QStringLiteral("不可行：%1").arg(QString::fromStdString(plan_.reason));
    }
    routeInfo_->setPlainText(route);

    // 车辆信息（MC-E04）：每车一个页签，标题 `<车辆 ID> · <容量>kg`；
    // 页内「固定信息 / 运行状态」两块。软件内时间不重复打印（状态栏常驻）。
    //
    // 这里必须区分两个量（此前混为一谈，导致"载重上限 200kg / 当前载重 740kg"）：
    //   · 车上载重：车上此刻实际载的订单货量，不变量是 **不得超过该车载重上限**
    //   · 剩余待送：全部未送达订单的货量之和，可以远超单车载重上限
    if (vehicleTabs_ != nullptr) {
        // 页签按车辆重建：标题、固定信息、运行状态全部逐车来自 config_。
        // 重建以 vehicles.size() 为准（首刷一次；之后只在页签数不符时重建，
        // 保持页签当前选择不被无谓重置）。
        while (vehicleTabs_->count() > static_cast<int>(config_.vehicles.size())) {
            QWidget* w = vehicleTabs_->widget(vehicleTabs_->count() - 1);
            vehicleTabs_->removeTab(vehicleTabs_->count() - 1);
            delete w;
        }
        for (std::size_t vi = 0; vi < config_.vehicles.size(); ++vi) {
            const logistics::Vehicle& v = config_.vehicles[vi];
            const QString tabTitle = QStringLiteral("%1 · %2kg")
                                         .arg(QString::fromStdString(v.id))
                                         .arg(v.capacityKg, 0, 'f', 0);
            if (vi >= static_cast<std::size_t>(vehicleTabs_->count())) {
                // 首次出现这辆车：两块 QLabel（固定信息 + 运行状态）叠成一个容器页。
                QWidget* page = new QWidget(vehicleTabs_);
                QVBoxLayout* lay = new QVBoxLayout(page);
                lay->setContentsMargins(0, 0, 0, 0);
                lay->setSpacing(0);
                QLabel* fixed = new QLabel(page);
                QLabel* run = new QLabel(page);
                for (QLabel* l : {fixed, run}) {
                    l->setTextFormat(Qt::PlainText);
                    l->setMargin(6);
                    l->setAlignment(Qt::AlignTop | Qt::AlignLeft);
                }
                lay->addWidget(fixed);
                lay->addWidget(run);
                page->setProperty("vehicleFixedLabel",
                                  QVariant::fromValue<QLabel*>(fixed));
                page->setProperty("vehicleRunLabel", QVariant::fromValue<QLabel*>(run));
                vehicleTabs_->addTab(page, tabTitle);
            } else {
                vehicleTabs_->setTabText(static_cast<int>(vi), tabTitle);
            }
        }

        // 状态由**当前页签**对应的车辆驱动（规格一阶段只有 V01；多车时页签即切换视角）。
        const int cur = vehicleTabs_->currentIndex() >= 0 ? vehicleTabs_->currentIndex() : 0;
        const std::size_t vi = static_cast<std::size_t>(cur);
        if (vi < config_.vehicles.size()) {
            const logistics::Vehicle& v = config_.vehicles[vi];
            QWidget* page = vehicleTabs_->widget(cur);
            QLabel* fixed = page->property("vehicleFixedLabel").value<QLabel*>();
            QLabel* run = page->property("vehicleRunLabel").value<QLabel*>();
            if (fixed != nullptr && run != nullptr) {
                double remainingDemand = 0.0;
                for (const Order& o : remainingOrders()) {
                    remainingDemand += o.demandKg;
                }

                // ---- 固定信息：整场模拟都不变的车辆固有参数（6 行）----
                fixed->setText(
                    QStringLiteral("固定信息\n"
                                   "ID：%1\n起始仓库：%2\n载重上限：%3 kg\n发车时刻：%4\n"
                                   "耗时倍率：%5\n成本倍率：%6\n"
                                   "（以上为配置给定的固有参数，模拟过程中不会变化）")
                        .arg(QString::fromStdString(v.id))
                        .arg(QString::fromStdString(v.startNodeId))
                        .arg(v.capacityKg, 0, 'f', 0)
                        .arg(minutesToClock(v.departTimeMin))
                        .arg(v.timeFactor, 0, 'f', 2)
                        .arg(v.costFactor, 0, 'f', 2));

                // ---- 运行状态：随模拟推进而变的量（规格一口径）----
                run->setText(
                    QStringLiteral("运行状态\n"
                                   "位置：%1\n"
                                   "时刻：%2\n"
                                   "车上载重：%3 kg\n"
                                   "在车明细：%4\n"
                                   "剩余待送：%5 kg\n"
                                   "已送达：%6 站\n"
                                   "已发生 penalty：%7 min")
                        .arg(QString::fromStdString(currentPositionId()))
                        // 车辆自己的时刻（= 软件内时间）。状态栏常驻显示时钟，
                        // 这里不重复「软件内时间」那一行，只作为运行状态字段。
                        .arg(minutesToClock(state_.atTimeMin))
                        // 「车上载重」是**物理事实**：车上此刻实际载着的订单货量。
                        .arg(currentLoadKg(), 0, 'f', 0)
                        .arg(onboardGoodsText())
                        .arg(remainingDemand, 0, 'f', 0)
                        .arg(state_.servedStops)
                        .arg(state_.incurredPenaltyMin));
            }
        }
    }

    // 订单列表：按"未送达的紧急 → 未送达普通 → 已送达"排序。
    // 新插入的紧急订单会立刻出现在**第一行**，不必滚动到表格末尾去找
    // （早先直接按 config_.orders 的插入顺序显示，紧急单被追加在最底下，
    //  用户看不见，会以为"根本没有添加这条紧急订单"）。
    std::vector<std::size_t> order;
    for (std::size_t pass = 0; pass < 3; ++pass) {
        for (std::size_t i = 0; i < config_.orders.size(); ++i) {
            const Order& o = config_.orders[i];
            const bool wanted = (pass == 0) ? (!o.served && o.urgent)
                                : (pass == 1) ? (!o.served && !o.urgent)
                                              : o.served;
            if (wanted) {
                order.push_back(i);
            }
        }
    }

    orderTable_->setRowCount(static_cast<int>(order.size()));
    for (int row = 0; row < static_cast<int>(order.size()); ++row) {
        const Order& o = config_.orders[order[static_cast<std::size_t>(row)]];
        orderTable_->setItem(row, 0, new QTableWidgetItem(QString::fromStdString(o.id)));
        orderTable_->setItem(row, 1, new QTableWidgetItem(QString::fromStdString(o.nodeId)));
        orderTable_->setItem(row, 2, new QTableWidgetItem(QString::number(o.demandKg, 'f', 0)));
        orderTable_->setItem(row, 3,
                             new QTableWidgetItem(minutesToClock(o.windowStartMin)
                                                  + QStringLiteral("-")
                                                  + minutesToClock(o.windowEndMin)));
        orderTable_->setItem(row, 4,
                             new QTableWidgetItem(
                                 o.served ? QStringLiteral("已送达")
                                          : (o.urgent ? (state_.pickedOrderIds.count(o.id)
                                                             ? QStringLiteral("执行中")
                                                             : QStringLiteral("紧急"))
                                                      : QStringLiteral("待配送"))));
    }

    // 停靠明细（MC-E01）：9 列、按**每车**铺行。规格一阶段只有一台车，但列按
    // 每车铺（车辆列填该车 ID），不把「车辆」写死成单值。动作列显示**计数**
    // （取<a>·送<b>），Σ取 == Σ送 == 取送对数 是可断言的结构关系；
    // 余货(kg) = `Stop::remainingLoadKg`（离开该站时的在车量）。
    // 只列**尚未走过**的停靠点：推进一站与重规划应当有一致的效果，
    // 都表现为"已送达的那一行从表里消失"。
    {
        const std::size_t fromStop =
            (stopCursor_ < plan_.stops.size()) ? stopCursor_ : plan_.stops.size();
        const int shown = static_cast<int>(plan_.stops.size() - fromStop);
        int row = 0;
        // 规格一：单台车（vehicles 非空才有计划）。多车时逐车各铺一段行。
        for (std::size_t vi = 0; vi < config_.vehicles.size(); ++vi) {
            const logistics::Vehicle& v = config_.vehicles[vi];
            const bool myPlan = (vi == 0);   // 规格一只有第 0 台车有计划；后续票按车拆计划
            const int rowsForVehicle = myPlan ? shown : 0;
            stopTable_->setRowCount(row + rowsForVehicle);
            for (int i = 0; i < rowsForVehicle; ++i) {
                const std::size_t idx = fromStop + static_cast<std::size_t>(i);
                const Stop& s = plan_.stops[idx];
                const QString actionText = QStringLiteral("取%1·送%2")
                                               .arg(s.pickups.size())
                                               .arg(s.deliveries.size());
                stopTable_->setItem(row, 0,
                                    new QTableWidgetItem(QString::fromStdString(v.id)));
                stopTable_->setItem(row, 1,
                                    new QTableWidgetItem(QString::number(i + 1)));
                stopTable_->setItem(
                    row, 2,
                    new QTableWidgetItem(s.phase == logistics::StopPhase::Urgent
                                             ? QStringLiteral("紧急")
                                             : QStringLiteral("普通")));
                stopTable_->setItem(row, 3,
                                    new QTableWidgetItem(QString::fromStdString(s.nodeId)));
                stopTable_->setItem(row, 4, new QTableWidgetItem(actionText));
                stopTable_->setItem(row, 5,
                                    new QTableWidgetItem(minutesToClock(s.rawArrivalMin)));
                stopTable_->setItem(row, 6,
                                    new QTableWidgetItem(QString::number(s.waitMin)));
                stopTable_->setItem(row, 7,
                                    new QTableWidgetItem(minutesToClock(s.arrivalMin)));
                stopTable_->setItem(row, 8,
                                    new QTableWidgetItem(
                                        QString::number(s.remainingLoadKg, 'f', 0)));
                ++row;
            }
        }
        stopTable_->setRowCount(row);
    }

    // 中转站 / 集散：子网络标识 + 下属配送点数 + 当前暂存货（kg）
    {
        struct TransitRow {
            QString     name;
            std::string id;
            int         sub = 0;
            int         serves = 0;
        };
        std::vector<TransitRow> rows;
        for (const logistics::Node& n : config_.graph.nodes()) {
            if (n.type != logistics::NodeType::Transit) {
                continue;
            }
            TransitRow row;
            row.id = n.id;
            row.name = QString::fromStdString(n.id) + QStringLiteral("（")
                       + QString::fromStdString(n.name) + QStringLiteral("）");
            row.sub = n.subNetworkId;
            for (const logistics::Node& other : config_.graph.nodes()) {
                if (other.type == logistics::NodeType::Delivery
                    && other.subNetworkId == n.subNetworkId) {
                    ++row.serves;
                }
            }
            rows.push_back(row);
        }
        // 物理事实：全局至多一堆暂存货；只有 state_.stashStation 那一站显示非 0。
        // 数据直接来自车辆状态（唯一权威），不从计划反推。
        double stashKg = 0.0;
        for (const logistics::OnboardItem& it : state_.stashItems) {
            stashKg += it.kg;
        }
        transitTable_->setRowCount(static_cast<int>(rows.size()));
        for (int i = 0; i < static_cast<int>(rows.size()); ++i) {
            transitTable_->setItem(i, 0, new QTableWidgetItem(rows[i].name));
            transitTable_->setItem(i, 1,
                                   new QTableWidgetItem(QString::number(rows[i].sub)));
            transitTable_->setItem(i, 2,
                                   new QTableWidgetItem(QString::number(rows[i].serves)));
            const bool here = !state_.stashStation.empty() && state_.stashStation == rows[i].id;
            transitTable_->setItem(i, 3,
                                   new QTableWidgetItem(
                                       QString::number(here ? stashKg : 0.0, 'f', 0)));
        }
    }

    // 超时订单（MC-E02）：**按订单一行**，窗口用**该单自己的** [ws, we]。
    // 区分**已经发生**的超时（事实，跨重规划保留）与**剩余计划预计**的超时。
    // 只列 plan_.stops 里 late 的那些是不够的：重规划后剩余计划里可能一个都不 late，
    // 于是"已经超时过"的订单会从表里凭空消失，看起来像超时被抹掉了。
    {
        // 订单 ID -> 订单（窗口 / 档位按单查，不再合并）。
        std::map<std::string, const Order*> orderById;
        for (const Order& o : config_.orders) {
            orderById[o.id] = &o;
        }
        const QString vehicleId =
            config_.vehicles.empty()
                ? QStringLiteral("-")
                : QString::fromStdString(config_.vehicles.front().id);
        int lateRows = 0;
        const auto putLate = [&](const std::string& orderId, const std::string& nodeId,
                                 const QString& status, int arrivalMin, int penaltyMin) {
            QString windowText = QStringLiteral("-");
            QString tier = QStringLiteral("普通");
            const std::map<std::string, const Order*>::const_iterator it =
                orderById.find(orderId);
            if (it != orderById.end()) {
                windowText = minutesToClock(it->second->windowStartMin)
                             + QStringLiteral("-")
                             + minutesToClock(it->second->windowEndMin);
                tier = it->second->urgent ? QStringLiteral("紧急") : QStringLiteral("普通");
            }
            lateTable_->setRowCount(lateRows + 1);
            lateTable_->setItem(lateRows, 0, new QTableWidgetItem(vehicleId));
            lateTable_->setItem(lateRows, 1,
                                new QTableWidgetItem(QString::fromStdString(orderId)));
            lateTable_->setItem(lateRows, 2,
                                new QTableWidgetItem(QString::fromStdString(nodeId)));
            lateTable_->setItem(lateRows, 3, new QTableWidgetItem(tier));
            lateTable_->setItem(lateRows, 4, new QTableWidgetItem(status));
            lateTable_->setItem(lateRows, 5, new QTableWidgetItem(minutesToClock(arrivalMin)));
            lateTable_->setItem(lateRows, 6, new QTableWidgetItem(windowText));
            lateTable_->setItem(lateRows, 7, new QTableWidgetItem(QString::number(penaltyMin)));
            ++lateRows;
        };
        for (const VehicleState::LateStop& ls : state_.deliveredLate) {
            putLate(ls.orderId, ls.nodeId,
                    QStringLiteral("已超时"), ls.arrivalMin, ls.penaltyMin);
        }
        if (plan_.status == logistics::PlanStatus::Ok) {
            // 逐单列（规格一 §6.3：penalty 逐单，一个停靠上可能一单一迟一早）。
            for (std::size_t k = stopCursor_; k < plan_.stops.size(); ++k) {
                const Stop& s = plan_.stops[k];
                for (const logistics::StopDelivery& d : s.deliveries) {
                    if (!d.late) {
                        continue;
                    }
                    putLate(d.orderId, s.nodeId,
                            QStringLiteral("预计超时"), s.arrivalMin, d.penaltyMin);
                }
            }
        }
        lateTable_->setRowCount(lateRows);
    }
}

void MainWindow::appendLog(const QString& text, int atMin) {
    logView_->appendPlainText(text);
    // 同一条内容也进运行记录（带软件内时刻），供「导出运行日志…」带出界面。
    recordRun(text, atMin);
}

void MainWindow::recordRun(const QString& text, int atMin) {
    if (text.contains(QStringLiteral("抽取："))) {
        ++runEventCount_;
    }
    const int stamp = (atMin >= 0) ? atMin : currentTimeMin();
    runLog_ << QStringLiteral("%1  %2").arg(minutesToClock(stamp), text);
}

QString MainWindow::runLogText() const {
    QString out;
    out += QStringLiteral("电商物流配送路径规划系统 —— 运行日志导出\n");
    out += QStringLiteral("用途：把「确实出现过的问题」作为证据带出界面。\n");
    out += QStringLiteral("内容：每次事件触发、每次到达配送点时的车辆载重明细、"
                          "每次出仓装车、以及紧急单的处置方式。\n");
    out += QStringLiteral("策略：%1    载重上限：%2 kg    事件间隔：每 %3 min（软件内时间）\n")
               .arg(strategyBox_ != nullptr ? strategyBox_->currentText() : QString())
               .arg(config_.vehicles.empty() ? 0.0 : config_.vehicles.front().capacityKg, 0, 'f', 0)
               .arg(config_.general.eventIntervalMin);
    out += QString(72, QLatin1Char('=')) + QStringLiteral("\n");
    for (const QString& line : runLog_) {
        out += line + QStringLiteral("\n");
    }
    out += QString(72, QLatin1Char('=')) + QStringLiteral("\n");
    out += QStringLiteral("汇总：事件 %1 次 ｜ 已送达 %2 站 ｜ 期末车上载重 %3 kg\n")
               .arg(runEventCount_)
               .arg(state_.servedStops)
               .arg(state_.loadKg, 0, 'f', 0);
    return out;
}

bool MainWindow::exportRunLogTo(const QString& path) const {
    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        return false;
    }
    QTextStream stream(&file);
    stream << runLogText();
    file.close();
    return true;
}

// 自动保存的文件名格式。**只有严格匹配它的文件才算缓冲队列成员**——
// 用户手动改名后一定不符合（哪怕只多一个空格），因此永远不会被自动清理删掉。
static const char* const kAutoLogPrefix = "运行日志-";
static const char* const kAutoLogRegex = "^运行日志-[0-9]{8}-[0-9]{6}\\.txt$";

QStringList MainWindow::autoLogCandidates(const QString& dir) {
    QStringList out;
    QDir d(dir);
    if (!d.exists()) {
        return out;
    }
    const QRegularExpression re(QString::fromUtf8(kAutoLogRegex));
    const QStringList names = d.entryList(QStringList() << QStringLiteral("*.txt"),
                                          QDir::Files, QDir::Name);
    for (const QString& name : names) {
        if (re.match(name).hasMatch()) {
            out << name;   // entryList 已按 Name 升序 ⇒ 时间戳升序 = 最旧在最前
        }
    }
    return out;
}

QString MainWindow::autoSaveRunLog() {
    ++autoSaveCount_;
    // 没有事件也没有送达 ⇒ 这次啥也没发生（例如只是打开看了看），不占用缓冲位。
    if (runLog_.isEmpty() && state_.servedStops == 0 && runEventCount_ == 0) {
        return QString();
    }
    // 日志目录放在**可执行文件所在目录**下（用户从根目录或用启动脚本都能找到），
    // 而不是当前工作目录——后者会随启动方式变化，用户会找不到自己刚导出的文件。
    const QString dir = QCoreApplication::applicationDirPath() + QStringLiteral("/logs");
    QDir().mkpath(dir);
    const QString name = QString::fromUtf8(kAutoLogPrefix)
                         + QDateTime::currentDateTime().toString(
                               QStringLiteral("yyyyMMdd-HHmmss"))
                         + QStringLiteral(".txt");
    const QString path = dir + QStringLiteral("/") + name;
    if (!exportRunLogTo(path)) {
        return QString();
    }
    // 只保留最近 kRunLogKeep 份：**只数严格匹配命名格式的文件**，
    // 手动改名/改格式的一律不参与，也就永远不会被淘汰。
    const QStringList existing = autoLogCandidates(dir);
    const int excess = existing.size() - kRunLogKeep;
    for (int i = 0; i < excess; ++i) {
        QFile::remove(dir + QStringLiteral("/") + existing[i]);
    }
    return path;
}

void MainWindow::onAutoSaveRunLog() {
    const QString saved = autoSaveRunLog();
    appendLog(saved.isEmpty()
                  ? QStringLiteral("自动保存：本次运行没有可保存的内容")
                  : QStringLiteral("已自动保存运行日志：%1（自动日志仅保留最近 %2 份）")
                        .arg(saved)
                        .arg(kRunLogKeep));
}

void MainWindow::closeEvent(QCloseEvent* event) {
    const QString saved = autoSaveRunLog();
    logView_->appendPlainText(saved.isEmpty()
                                  ? QStringLiteral("关闭（本次运行无可保存内容）")
                                  : QStringLiteral("已自动保存运行日志：%1（自动日志仅保留最近 %2 份）")
                                        .arg(saved)
                                        .arg(kRunLogKeep));
    QMainWindow::closeEvent(event);
}

void MainWindow::onExportRunLog() {    // 「另存为…」：给需要长期保存的那一份用。**不参与**自动保存的缓冲队列，
    // 因此不会被自动清理删掉（它通常已经被改名，格式也对不上）。
    const QString dir = QCoreApplication::applicationDirPath() + QStringLiteral("/logs");
    QDir().mkpath(dir);
    const QString suggested = dir + QStringLiteral("/运行日志-%1.txt")
                                        .arg(QDateTime::currentDateTime().toString(
                                            QStringLiteral("yyyyMMdd-HHmmss")));
    const QString path = QFileDialog::getSaveFileName(
        this, QStringLiteral("另存运行日志（自动保存另外保留最近 %1 份）")
                  .arg(kRunLogKeep),
        suggested, QStringLiteral("文本文件 (*.txt)"));
    if (path.isEmpty()) {
        return;   // 用户取消
    }
    if (exportRunLogTo(path)) {
        appendLog(QStringLiteral("已另存运行日志：%1（本次记录事件 %2 次、送达 %3 站）")
                      .arg(path)
                      .arg(runEventCount_)
                      .arg(state_.servedStops));
    } else {
        appendLog(QStringLiteral("另存失败（无法写入）：%1").arg(path));
    }
}

void MainWindow::runDemoActions(int rounds) {
    for (int i = 0; i < rounds; ++i) {
        onAdvanceStop();
        onSimulateTraffic();
        if (i % 2 == 1) {
            onInsertUrgentOrder();
        }
        if (i % 3 == 2) {
            onCloseRandomRoad();
        }
        QCoreApplication::processEvents();
    }
}

int MainWindow::toolbarActionCount() const {
    int total = 0;
    for (const QToolBar* bar : findChildren<QToolBar*>()) {
        total += bar->actions().size();
    }
    return total;
}

QString MainWindow::transitPanelSummary() const {
    QString out;
    for (int row = 0; row < transitTable_->rowCount(); ++row) {
        QStringList cells;
        for (int col = 0; col < transitTable_->columnCount(); ++col) {
            const QTableWidgetItem* item = transitTable_->item(row, col);
            cells << (item != nullptr ? item->text() : QString());
        }
        out += cells.join(QStringLiteral(" | ")) + QLatin1Char('\n');
    }
    return out;
}

int MainWindow::lateOrderCount() const {
    int late = 0;
    for (const logistics::OrderMetric& om : plan_.metrics.orders) {
        if (om.late) {
            ++late;
        }
    }
    late += static_cast<int>(state_.deliveredLate.size());
    return late;
}

QStringList MainWindow::transitPanelHeaders() const {
    QStringList names;
    if (transitTable_ != nullptr) {
        for (int col = 0; col < transitTable_->columnCount(); ++col) {
            const QTableWidgetItem* item = transitTable_->horizontalHeaderItem(col);
            names << (item != nullptr ? item->text() : QString());
        }
    }
    return names;
}

int MainWindow::vehicleTabCount() const {
    return vehicleTabs_ != nullptr ? vehicleTabs_->count() : 0;
}

QString MainWindow::vehicleTabTitles() const {
    QStringList names;
    if (vehicleTabs_ != nullptr) {
        for (int i = 0; i < vehicleTabs_->count(); ++i) {
            names << vehicleTabs_->tabText(i);
        }
    }
    return names.join(QStringLiteral(" | "));
}

QString MainWindow::vehiclePanelSection(int index) const {
    if (vehicleTabs_ == nullptr || index < 0 || index >= vehicleTabs_->count()) {
        return QString();
    }
    const QWidget* page = vehicleTabs_->widget(index);
    if (page == nullptr) {
        return QString();
    }
    const QLabel* fixed = page->property("vehicleFixedLabel").value<QLabel*>();
    const QLabel* run = page->property("vehicleRunLabel").value<QLabel*>();
    QString out = QStringLiteral("[") + vehicleTabs_->tabText(index) + QStringLiteral("]\n");
    if (fixed != nullptr) {
        out += fixed->text();
        out += QLatin1Char('\n');
    }
    if (run != nullptr) {
        out += run->text();
    }
    return out;
}

QString MainWindow::vehiclePanelSummary() const {
    // **每车**页签都给出去：--ui-probe 要能逐车核对页签标题、固定信息与运行状态
    //（只给当前页签的话，切页状态会影响断言结果）。
    QString out;
    if (vehicleTabs_ != nullptr) {
        for (int i = 0; i < vehicleTabs_->count(); ++i) {
            const QWidget* page = vehicleTabs_->widget(i);
            if (page == nullptr) {
                continue;
            }
            const QLabel* fixed = page->property("vehicleFixedLabel").value<QLabel*>();
            const QLabel* run = page->property("vehicleRunLabel").value<QLabel*>();
            if (!out.isEmpty()) {
                out += QLatin1Char('\n');
            }
            out += QStringLiteral("[") + vehicleTabs_->tabText(i)
                    + QStringLiteral("]\n");
            if (fixed != nullptr) {
                out += fixed->text();
                out += QLatin1Char('\n');
            }
            if (run != nullptr) {
                out += run->text();
            }
        }
    }
    return out;
}

QString MainWindow::statusClockSummary() const {
    return clockLabel_ != nullptr ? clockLabel_->text() : QString();
}

QString MainWindow::toolbarActionTexts() const {
    QStringList names;
    // 只列**工具栏**上的动作名。原先用 findChildren<QAction*>()，会把各停靠面板的
    // 切换动作也一起列进来（列出的名字比"工具栏动作 N 个"的口径多），
    // 而面板名另有 dockTitles() 提供 —— 名字与语义不符，也容易被同名面板动作满足。
    for (const QToolBar* bar : findChildren<QToolBar*>()) {
        for (QAction* a : bar->actions()) {
            if (a != nullptr && !a->text().isEmpty() && a->isEnabled()) {
                names << a->text();
            }
        }
    }
    names.removeDuplicates();
    names.sort();
    return names.join(QStringLiteral(" | "));
}

QString MainWindow::dockTitles() const {
    QStringList names;
    for (QDockWidget* d : findChildren<QDockWidget*>()) {
        if (d != nullptr) {
            names << d->windowTitle();
        }
    }
    names.removeDuplicates();
    names.sort();
    return names.join(QStringLiteral(" | "));
}

QString MainWindow::routePanelSummary() const {
    return routeInfo_ != nullptr ? routeInfo_->toPlainText() : QString();
}

QString MainWindow::latePanelSummary() const {
    QString out;
    for (int row = 0; row < lateTable_->rowCount(); ++row) {
        QStringList cells;
        for (int col = 0; col < lateTable_->columnCount(); ++col) {
            const QTableWidgetItem* item = lateTable_->item(row, col);
            cells << (item != nullptr ? item->text() : QString());
        }
        out += cells.join(QStringLiteral(" | ")) + QLatin1Char('\n');
    }
    return out;
}

QStringList MainWindow::latePanelHeaders() const {
    QStringList out;
    for (int col = 0; col < lateTable_->columnCount(); ++col) {
        const QTableWidgetItem* item = lateTable_->horizontalHeaderItem(col);
        out << (item != nullptr ? item->text() : QString());
    }
    return out;
}

QString MainWindow::stopPanelSummary() const {
    QString out;
    for (int row = 0; row < stopTable_->rowCount(); ++row) {
        QStringList cells;
        for (int col = 0; col < stopTable_->columnCount(); ++col) {
            const QTableWidgetItem* item = stopTable_->item(row, col);
            cells << (item != nullptr ? item->text() : QString());
        }
        out += cells.join(QStringLiteral(" | ")) + QLatin1Char('\n');
    }
    return out;
}

QStringList MainWindow::stopPanelHeaders() const {
    QStringList out;
    for (int col = 0; col < stopTable_->columnCount(); ++col) {
        const QTableWidgetItem* item = stopTable_->horizontalHeaderItem(col);
        out << (item != nullptr ? item->text() : QString());
    }
    return out;
}

int MainWindow::dockCount() const {
    return findChildren<QDockWidget*>().size();
}

void MainWindow::showInteractive() {
    // 关键：窗口尺寸一旦超过屏幕，侧栏就会被推到可见区域之外，
    // 用户会以为"这些面板根本不存在"。直接最大化是唯一不必猜测用户
    // 屏幕尺寸的做法——工具栏与全部面板必然可见。
    // （原先此函数接收 --width/--height 并计算 width/height 局部变量，
    //   但从未使用它们；已删除这两个无效参数与死变量，--width/--height
    //   现在只对 --render / --render-window 生效。）
    setMinimumSize(640, 480);
    showMaximized();
    raise();
    activateWindow();
}

int MainWindow::runActionSelfCheck() {
    int failures = 0;
    auto expect = [&failures](bool ok, const QString& what) {
        const QByteArray line = what.toUtf8();
        std::printf("[self-check] %s %s\n", ok ? "OK  " : "FAIL", line.constData());
        if (!ok) {
            ++failures;
        }
    };

    // 收集真实客户节点（A/B 的取值域）
    std::vector<std::string> cust;
    for (const logistics::Node& n : config_.graph.nodes()) {
        if (n.type == logistics::NodeType::Delivery) {
            cust.push_back(n.id);
        }
    }
    expect(cust.size() >= 2, QStringLiteral("（前置）至少存在两个客户节点"));


    // ---- 1. 插入一张装得下的紧急取送单：A 取 → B 送，先 A 后 B，B 为第一个停靠点 ----
    if (cust.size() >= 2) {
        const std::string A = cust[0];
        const std::string B = cust[1];
        const std::size_t before = config_.orders.size();
        const std::string id = insertOrderAt(A, B, 5.0, true,
                                            logistics::OrderSource::ManualUrgent, currentTimeMin());
        expect(!id.empty(), QStringLiteral("紧急取送单应插入成功"));
        expect(config_.orders.size() == before + 1,
               QStringLiteral("紧急单必须并入订单集合（不丢单）"));
        expect(plan_.status == logistics::PlanStatus::Ok,
               QStringLiteral("插入后计划仍可行"));
        std::size_t ia = plan_.nodes.size();
        std::size_t ib = plan_.nodes.size();
        for (std::size_t i = 0; i < plan_.nodes.size(); ++i) {
            if (plan_.nodes[i] == A && ia == plan_.nodes.size()) {
                ia = i;
            }
            if (plan_.nodes[i] == B && ib == plan_.nodes.size()) {
                ib = i;
            }
        }
        expect(ia < ib, QStringLiteral("取送顺序：A 必须早于 B（A@%1, B@%2）").arg(ia).arg(ib));
        // 紧急优先守卫（issue #5 放宽）：B 的到达 ≤ 其余普通停靠点；A/B 同点顺路交付
        // 时 A 可能排在 B 之前（到 A 取货的同时交付 A 的普通货），这是允许的例外。
        bool bServed = false;
        int bArrival = -1;
        for (const logistics::Stop& s : plan_.stops) {
            if (s.nodeId == B) {
                bServed = true;
                bArrival = s.arrivalMin;
                break;
            }
        }
        bool bFirst = bServed;
        for (const logistics::Stop& s : plan_.stops) {
            if (s.nodeId == A || s.nodeId == B) {
                continue;
            }
            if (bArrival >= 0 && bArrival > s.arrivalMin) {
                bFirst = false;
            }
        }
        expect(bServed, QStringLiteral("紧急送达点 B 必须出现在计划停靠点里"));
        expect(bFirst, QStringLiteral("紧急送达点 B 的到达不晚于其它普通停靠点（A/B 顺路交付除外）"));
        // 推进到取货点 A：应进入「执行中（已取未送）」；到达 B：应完成。
        bool picked = false;
        for (int i = 0; i < 60 && !picked; ++i) {
            onAdvanceStop();
            picked = state_.pickedOrderIds.count(id) > 0;
        }
        expect(picked, QStringLiteral("到达取货点 A 后应进入执行中（已取未送）"));
        bool served = false;
        for (int i = 0; i < 60 && !served; ++i) {
            onAdvanceStop();
            for (const Order& o : config_.orders) {
                if (o.id == id && o.served) {
                    served = true;
                }
            }
        }
        expect(served, QStringLiteral("到达送达点 B 后紧急取送单应完成"));
    }

    // ---- 2. 事件刻刷单：连续两张紧急单都不丢；**窗口基准 = 事件刻**（不是结算时刻）----
    {
        // 用一个**真事件刻**当基准；它严格大于当前软件内时刻，因此
        // "把基准错写成结算时刻（currentTimeMin()）"会让下面的断言立刻变红。
        const int mark = nextEventMark_;
        const std::size_t before = config_.orders.size();
        const std::string id1 = insertRandomOrder(true, logistics::OrderSource::EventUrgent, mark);
        const std::string id2 = insertRandomOrder(true, logistics::OrderSource::EventUrgent, mark);
        expect(!id1.empty() && !id2.empty(), QStringLiteral("连续插单应都成功"));
        expect(config_.orders.size() == before + 2,
               QStringLiteral("连续两次插单不得丢单"));
        std::size_t unserved = 0;
        std::size_t inPlan = 0;
        for (const Order& o : config_.orders) {
            if (o.served || !o.urgent) {
                continue;
            }
            ++unserved;
            for (const logistics::Stop& s : plan_.stops) {
                if (s.nodeId == o.nodeId) {
                    ++inPlan;
                    break;
                }
            }
        }
        expect(unserved > 0 && inPlan == unserved,
               QStringLiteral("全部未完成紧急单都必须出现在计划停靠点里（%1/%2）")
                   .arg(inPlan).arg(unserved));
        // 事件刷单参数（R2–R5，读订单集合状态）
        auto paramOk = [mark](const Order& o) {
            return o.urgent && o.pickupNodeId != o.nodeId
                   && o.demandKg >= 3.0 - 1e-9 && o.demandKg <= 10.0 + 1e-9
                   && std::fabs(o.demandKg - std::floor(o.demandKg)) < 1e-9
                   // 窗口基准 = **该事件的事件刻**，止 = min(mark+60, 24:00)
                   && o.windowStartMin == mark
                   && o.windowEndMin == (mark + 60 < 24 * 60 ? mark + 60 : 24 * 60)
                   && o.source == logistics::OrderSource::EventUrgent
                   && !o.id.empty() && o.id[0] == 'U';
        };
        const std::size_t n = config_.orders.size();
        expect(n >= 2 && paramOk(config_.orders[n - 2]) && paramOk(config_.orders[n - 1]),
               QStringLiteral("事件刷单参数 R2–R5：A≠B、3–10kg 整数、窗口 [事件刻 %1, +60]、"
                              "来源 event_urgent / 前缀 U")
                   .arg(minutesToClock(mark)));
        expect(mark > currentTimeMin(),
               QStringLiteral("（前提）事件刻 %1 严格晚于当前时刻 %2 —— 窗口基准断言才有区分力")
                   .arg(minutesToClock(mark))
                   .arg(minutesToClock(currentTimeMin())));
    }

    // ---- 3. 非法输入被拒（A==B / 超载），且不得并入订单集合 ----
    {
        const std::size_t before = config_.orders.size();
        const std::string bad1 = insertOrderAt("D01", "D01", 5.0, true,
                                              logistics::OrderSource::ManualUrgent, currentTimeMin());
        const double cap = config_.vehicles.empty() ? 0.0 : config_.vehicles.front().capacityKg;
        const std::string bad2 = insertOrderAt("D01", "D02", cap + 100.0, true,
                                              logistics::OrderSource::ManualUrgent, currentTimeMin());
        expect(bad1.empty() && bad2.empty(),
               QStringLiteral("A==B 与超载必须被拒绝"));
        expect(config_.orders.size() == before,
               QStringLiteral("被拒绝的紧急单不得并入订单集合"));
    }

    // ---- 4. 手动「模拟新客户」：建节点 X + 一张 B=X 的普通单，且 X 被纳入配送（R10）----
    {
        const std::size_t before = config_.orders.size();
        const int now = currentTimeMin();
        const std::string node = addRandomCustomerAction();
        expect(!node.empty(), QStringLiteral("模拟新客户应成功"));
        expect(config_.orders.size() == before + 1,
               QStringLiteral("新客户必须带来一张订单"));

        // 订单 B = X（货送到新客户），A = 随机**既有**配送点（≠X）
        const Order* customerOrder = nullptr;
        for (const Order& order : config_.orders) {
            if (order.source == logistics::OrderSource::ManualCustomer) {
                customerOrder = &order;
            }
        }
        expect(customerOrder != nullptr,
               QStringLiteral("新客户订单必须以 manual_customer 来源并入订单集合"));
        if (customerOrder != nullptr) {
            expect(customerOrder->nodeId == node,
                   QStringLiteral("新客户订单的送达点 B 必须是新节点 X（B=%1 / X=%2）")
                       .arg(QString::fromStdString(customerOrder->nodeId))
                       .arg(QString::fromStdString(node)));
            expect(customerOrder->pickupNodeId != node && !customerOrder->pickupNodeId.empty(),
                   QStringLiteral("新客户订单的取货点 A 必须是另一个既有配送点"));
            const logistics::Node* pickupNode =
                config_.graph.findNode(customerOrder->pickupNodeId);
            expect(pickupNode != nullptr
                       && pickupNode->type == logistics::NodeType::Delivery,
                   QStringLiteral("取货点 A 必须是配送点"));
            expect(customerOrder->demandKg >= 3.0 - 1e-9
                       && customerOrder->demandKg <= 15.0 + 1e-9
                       && std::fabs(customerOrder->demandKg - std::floor(customerOrder->demandKg))
                              < 1e-9,
                   QStringLiteral("新客户订单货量必须是 3–15kg 整数（实际 %1kg）")
                       .arg(customerOrder->demandKg, 0, 'f', 0));
            expect(customerOrder->windowStartMin == now && customerOrder->windowEndMin == 24 * 60,
                   QStringLiteral("新客户订单窗口 = [当前时刻 %1, 24:00]")
                       .arg(minutesToClock(now)));
            expect(!customerOrder->urgent && !customerOrder->id.empty()
                       && customerOrder->id[0] == 'C',
                   QStringLiteral("新客户订单必须是普通单、前缀 C（实际 %1）")
                       .arg(QString::fromStdString(customerOrder->id)));
        }

        // X 必须真的进路线（只看"新增节点"太弱：节点加了但没被配送等于没有需求）
        bool inPlan = false;
        for (const std::string& n : plan_.nodes) {
            if (n == node) {
                inPlan = true;
                break;
            }
        }
        expect(inPlan, QStringLiteral("新客户节点 X 必须出现在路线里（B=X 的结果）"));

        // 子网络编号继承（#30 缺陷）：新节点必须落进某个已有子网络，不能是 0
        const logistics::Node* created = config_.graph.findNode(node);
        expect(created != nullptr && created->subNetworkId != 0,
               QStringLiteral("新节点子网络编号必须非 0（继承最近邻，实际 %1）")
                   .arg(created != nullptr ? created->subNetworkId : -1));
    }

    // ---- 5. 重规划绝不改写车辆物理状态 ----
    // 规格一 MC-A01：趟字段已退役（#37 删除），这里改断**仍在的**物理事实：
    // 位置 / 时刻 / 在车货 / 已送达 / 已发生 penalty / 逐单超时账本。
    {
        for (int i = 0; i < 4; ++i) {
            onAdvanceStop();
        }
        const VehicleState snapshot = state_;
        onSimulateTraffic();
        onReplan();
        expect(state_.atNodeId == snapshot.atNodeId,
               QStringLiteral("重规划不得改变车辆位置"));
        expect(state_.atTimeMin == snapshot.atTimeMin,
               QStringLiteral("重规划不得改变车辆时刻"));
        expect(state_.loadKg == snapshot.loadKg,
               QStringLiteral("重规划不得改变车上载重"));
        expect(state_.onboard.size() == snapshot.onboard.size()
                   && state_.sumOnboard() == snapshot.sumOnboard(),
               QStringLiteral("重规划不得改变在车货构成（数量 / 合计）"));
        expect(state_.servedStops == snapshot.servedStops,
               QStringLiteral("重规划不得改变已送达计数"));
        expect(state_.incurredPenaltyMin == snapshot.incurredPenaltyMin,
               QStringLiteral("重规划不得改变已发生的 penalty"));
        expect(state_.deliveredLate.size() == snapshot.deliveredLate.size(),
               QStringLiteral("重规划不得改变已发生的超时账本"));
    }

    // ---- 6. 「在 A 装满后」触发降载 + 同车取回（MC-E08 / MC-A09）----
    // 放在最后：前五段的断言依赖"全新状态 + 恰有一张紧急单"的前提，而本段要
    // 长距离推进（会跨事件刻、抽到插单/路况事件），放在前面会打乱它们的前提。
    //
    // 构造（MC-E08「在 A 装满后触发」，确定性）：
    //
    // 规划器里"在 A 装不下"的判定：新紧急单的取货停靠排在**全部普通停靠之前**
    //（硬优先），而车上的货属于普通单 ⇒ 其送达都在普通段 ⇒ 在新紧急取货点 A 处
    // **全部仍在车上**。于是只要：
    //   ① 车上已有货（在车 = X kg，来自已取未送的普通单）；
    //   ② 再插一张需求 = 上限 − X + 1 的紧急单（单笔 ≤ 上限，合法）；
    // A 处装载 = X + (上限 − X + 1) = 上限 + 1，**必然**溢出 1kg ⇒ 降载。
    // 前提①②都由构造保证，不再是"恰好装不下"的概率条件。
    // 事件刷出的未完成紧急单会插进紧急段 FIFO、在 A 之前先卸货（把溢出吃掉）
    // ⇒ 开跑前先把未完成紧急单逐站送完（排空）。
    if (cust.size() >= 2 && !config_.vehicles.empty()) {
        const double cap = config_.vehicles.front().capacityKg;
        // 排空 + 等车上有货：事件刷出的紧急单逐站送完；普通货随车推进自然装上。
        {
            bool ready = false;
            for (int i = 0; i < 1200 && !ready && !atRouteEnd(); ++i) {
                bool pendingUrgent = false;
                for (const Order& o : config_.orders) {
                    if (o.urgent && !o.served) {
                        pendingUrgent = true;
                        break;
                    }
                }
                if (pendingUrgent) {
                    onAdvanceStop();   // 紧急单 FIFO 最先，送完即清
                    continue;
                }
                if (state_.loadKg > 0.0) {
                    ready = true;
                    break;
                }
                onAdvanceStop();       // 车还空着：推进到下一个取货停靠
            }
            bool nonePending = true;
            for (const Order& o : config_.orders) {
                if (o.urgent && !o.served) {
                    nonePending = false;
                    break;
                }
            }
            expect(ready && nonePending && state_.loadKg > 0.0,
                   QStringLiteral("（前提）未完成紧急单清零且车上有货（在车 %1kg）")
                       .arg(state_.loadKg, 0, 'f', 0));
        }
        // 在车货必须是**普通单**的货（其送达在普通段 ⇒ 在新紧急 A 处仍在车上）。
        // 紧急段已排空 ⇒ 车上的货必然全部来自普通单，此断言只是把前提显式化。
        bool onboardAllNormal = true;
        for (const logistics::OnboardItem& it : state_.onboard) {
            const Order* o = findOrderById(it.orderId);
            if (o == nullptr || o->urgent) {
                onboardAllNormal = false;
            }
        }
        expect(onboardAllNormal,
               QStringLiteral("（前提）在车货全部属于普通单（送达在普通段，A 处仍在车上）"));
        const double overflowDemand = cap - state_.loadKg + 1.0;
        expect(overflowDemand <= cap + 1e-9,
               QStringLiteral("（前提）溢出单需求 %1kg ≤ 上限 %2kg（单笔合法）")
                   .arg(overflowDemand, 0, 'f', 0)
                   .arg(cap, 0, 'f', 0));
        // 选 A/B：任意两个不同客户点（紧急段已排空 ⇒ 不存在可并入的既有紧急停靠）。
        std::string A;
        std::string B;
        for (std::size_t i = 0; i < cust.size() && A.empty(); ++i) {
            for (std::size_t j = 0; j < cust.size(); ++j) {
                if (j == i) {
                    continue;
                }
                A = cust[i];
                B = cust[j];
                break;
            }
        }
        const std::string id = insertOrderAt(A, B, overflowDemand, true,
                                            logistics::OrderSource::ManualUrgent, currentTimeMin());
        expect(!id.empty(), QStringLiteral("A 装满后溢出的紧急取送单应插入成功"));
        if (!id.empty()) {
            // 计划层（确定性）：A 装满后溢出 ⇒ **必然**出现降载记录。
            expect(!plan_.offloads.empty(),
                   QStringLiteral("A 装满后溢出 1kg ⇒ 应出现降载记录"
                                  "（在车 %1kg + 需求 %2kg > 上限 %3kg）")
                       .arg(state_.loadKg, 0, 'f', 0)
                       .arg(overflowDemand, 0, 'f', 0)
                       .arg(cap, 0, 'f', 0));
            bool conserve = true;
            bool loadOk = true;
            bool sameVehicleCustody = true;
            for (const logistics::OffloadLeg& leg : plan_.offloads) {
                if (leg.reclaimTimeMin < 0) {
                    conserve = false;   // 期末必须已取回（暂存守恒）
                }
                if (leg.reclaimTimeMin >= 0 && leg.reclaimTimeMin < leg.offloadTimeMin) {
                    sameVehicleCustody = false;   // 取回必须晚于卸下
                }
                // MC-A09 同车卸同车取：取回动作必须出现在**同一条路线**的某停靠里。
                bool reclaimSeen = false;
                for (const logistics::Stop& st : plan_.stops) {
                    if (st.nodeId != leg.station) {
                        continue;
                    }
                    for (const std::string& rid : st.reclaims) {
                        if (rid == leg.orderId) {
                            reclaimSeen = true;
                            break;
                        }
                    }
                    if (reclaimSeen) {
                        break;
                    }
                }
                if (!reclaimSeen) {
                    sameVehicleCustody = false;
                }
            }
            for (const logistics::Stop& st : plan_.stops) {
                if (st.remainingLoadKg > cap + 1e-6) {
                    loadOk = false;
                }
            }
            expect(conserve, QStringLiteral("暂存守恒：每条降载记录期末都已取回"));
            expect(sameVehicleCustody,
                   QStringLiteral("MC-A09：降载与取回在同一条路线（同车卸、同车取），不是转运"));
            expect(loadOk, QStringLiteral("停靠离站在车量不超过上限"));
            // 推进到结束：全程载重不超限 + 中转站面板逐帧反映物理状态
            bool traceOk = true;
            double maxLoad = 0.0;
            std::string maxNode;
            std::string maxBreak;
            bool panelMatchesState = true;
            bool panelColumnsOk = true;
            bool panelNumeric = true;
            for (int i = 0; i < 600 && !atRouteEnd(); ++i) {
                onAdvanceStop();
                // issue #10：中转站面板第 4 列必须逐帧反映车辆物理状态
                //（stashStation/stashItems），无暂存为 0、有暂存为该站的非 0 kg。
                const QStringList rows =
                    transitPanelSummary().split(QLatin1Char('\n'), Qt::SkipEmptyParts);
                bool anyNonZero = false;
                for (const QString& row : rows) {
                    const QStringList cells = row.split(QStringLiteral(" | "));
                    if (cells.size() != 4) {
                        panelColumnsOk = false;
                        continue;
                    }
                    bool isNumber = false;
                    cells[3].trimmed().toDouble(&isNumber);
                    if (!isNumber) {
                        panelNumeric = false;
                    }
                    if (cells[3].trimmed().toDouble() > 0.0) {
                        anyNonZero = true;
                    }
                }
                const bool stashExists =
                    !state_.stashStation.empty() && !state_.stashItems.empty();
                if (anyNonZero != stashExists) {
                    panelMatchesState = false;
                }
                if (state_.loadKg > maxLoad) {
                    maxLoad = state_.loadKg;
                    maxNode = state_.atNodeId;
                    QStringList parts;
                    for (const logistics::OnboardItem& it : state_.onboard) {
                        parts << (QString::fromStdString(it.nodeId) + ":"
                                  + QString::number(it.kg, 'f', 0));
                    }
                    maxBreak = parts.join(QStringLiteral(" | ")).toStdString();
                }
                if (state_.loadKg > cap + 1e-6) {
                    traceOk = false;
                }
            }
            expect(traceOk, QStringLiteral("全程车辆载重不超过上限（峰值 %1kg @ %2 / 上限 %3，车上明细 %4）")
                               .arg(maxLoad, 0, 'f', 0)
                               .arg(QString::fromStdString(maxNode))
                               .arg(cap, 0, 'f', 0)
                               .arg(QString::fromStdString(maxBreak)));
            expect(panelColumnsOk, QStringLiteral("中转站面板始终为 4 列"));
            expect(panelNumeric, QStringLiteral("中转站「当前暂存货」列始终是数字"));
            expect(panelMatchesState,
                   QStringLiteral("中转站「当前暂存货」列必须逐帧反映车辆暂存物理状态"
                                  "（无暂存为 0、有暂存为非 0）"));
            // 若降载真的执行了（未被中途重规划改道），日志里应能看到「降载」
            if (!state_.appliedOffloads.empty()) {
                expect(runLogText().contains(QStringLiteral("降载")),
                       QStringLiteral("执行降载后日志应出现「降载」"));
                // MC-A09 用户可见证据：同一张降载单的「卸下」与「取回」都出现在
                // 这台车的运行日志里（规格一只有一台车；转运的「卸入车 ≠ 取走车」
                // 归规格二 #41，本票不建转运面板）。
                bool custodyOk = true;
                for (const VehicleState::OffloadKey& key : state_.appliedOffloads) {
                    const QString unloadMark =
                        QStringLiteral("卸下订单 %1 ").arg(QString::fromStdString(key.orderId));
                    const QString reclaimMark =
                        QStringLiteral("取回订单 %1 ").arg(QString::fromStdString(key.orderId));
                    if (!runLogText().contains(unloadMark)
                        || !runLogText().contains(reclaimMark)) {
                        custodyOk = false;
                    }
                }
                expect(custodyOk,
                       QStringLiteral("MC-A09 卸货车 == 取回车：同一张单同车卸下、同车取回"));
                expect(state_.stashItems.empty(),
                       QStringLiteral("期末暂存堆清空：暂存货已由同一台车全部取回"));
            }
        }
    }

    // ---- 7. 自动刷单额度 R6/R7 与 Q8（独立窗口：额度/随机流都不受前六段影响）----
    //
    // 读的是**用户可见输出**（运行日志里的「[事件] … 抽取：…」行）与订单集合状态，
    // 不断言内部计数器自身的实现。
    {
        MainWindow rules(reloadInitialConfig(), configPath_);
        std::vector<std::string> deliveryIds;
        for (const logistics::Node& n : rules.config_.graph.nodes()) {
            if (n.type == logistics::NodeType::Delivery) {
                deliveryIds.push_back(n.id);
            }
        }
        expect(deliveryIds.size() >= 4,
               QStringLiteral("（前置）额度守卫需要至少 4 个配送点（实际 %1）")
                   .arg(deliveryIds.size()));

        // R6 计数口径：初始 25 单是 initial 来源 -> 不占额度
        expect(rules.pendingEventOrderCount(false) == 0
                   && rules.pendingEventOrderCount(true) == 0,
               QStringLiteral("R6：配置里的初始单不占事件刷单额度（普通 0 / 紧急 0）"));

        // R2–R5（普通单）：事件刻刷一张普通单 —— A≠B、3–15kg 整数、窗口 [事件刻, 24:00]、
        // 来源 event_normal / 前缀 N（读订单集合状态）。
        const int normalMark = rules.nextEventMark_;
        const std::size_t beforeNormal = rules.config_.orders.size();
        const std::string normalId =
            rules.insertRandomOrder(false, logistics::OrderSource::EventNormal, normalMark);
        expect(!normalId.empty() && rules.config_.orders.size() == beforeNormal + 1,
               QStringLiteral("R1/R9：事件刻刷普通单必须并入订单集合（不丢单）"));
        if (rules.config_.orders.size() == beforeNormal + 1) {
            const Order& order = rules.config_.orders.back();
            expect(order.source == logistics::OrderSource::EventNormal && !order.urgent
                       && !order.id.empty() && order.id[0] == 'N',
                   QStringLiteral("R5：事件普通单来源 event_normal / 前缀 N（实际 %1）")
                       .arg(QString::fromStdString(order.id)));
            expect(order.pickupNodeId != order.nodeId,
                   QStringLiteral("R2：事件普通单 A != B（%1 / %2）")
                       .arg(QString::fromStdString(order.pickupNodeId))
                       .arg(QString::fromStdString(order.nodeId)));
            expect(order.demandKg >= 3.0 - 1e-9 && order.demandKg <= 15.0 + 1e-9
                       && std::fabs(order.demandKg - std::floor(order.demandKg)) < 1e-9,
                   QStringLiteral("R3：事件普通单货量 3–15kg 整数（实际 %1kg）")
                       .arg(order.demandKg, 0, 'f', 0));
            expect(order.windowStartMin == normalMark && order.windowEndMin == 24 * 60,
                   QStringLiteral("R4：事件普通单窗口 [事件刻 %1, 24:00]")
                       .arg(minutesToClock(normalMark)));
        }
        expect(rules.pendingEventOrderCount(false) == 1,
               QStringLiteral("R6：事件刻刷出的普通单占 1 个额度"));

        // R1/R9：连走 15 个事件刻（此刻普通额度未满），逐刻比对订单集合增量：
        //   ① 1 个刻至多生 1 张单；② 抽中订单类事件**恰生 1 张**、路况/封路不生单。
        const int marksForOrderRules = 15;
        const int orderRulesBaseline = rules.eventLogLines().size();
        int addedTotal = 0;
        int maxAddedOrderRules = 0;
        for (int i = 0; i < marksForOrderRules; ++i) {
            const std::size_t before = rules.config_.orders.size();
            rules.settleEventsUpTo(rules.nextEventMark_);
            const int added = static_cast<int>(rules.config_.orders.size())
                              - static_cast<int>(before);
            addedTotal += added;
            if (added > maxAddedOrderRules) {
                maxAddedOrderRules = added;
            }
        }
        const QStringList orderRuleLines = rules.eventLogLines().mid(orderRulesBaseline);
        int normalDraws = 0;
        int urgentDraws = 0;
        for (const QString& line : orderRuleLines) {
            if (line.contains(QStringLiteral("抽取：普通订单"))) {
                ++normalDraws;
            } else if (line.contains(QStringLiteral("抽取：紧急订单"))) {
                ++urgentDraws;
            }
        }
        expect(orderRuleLines.size() == marksForOrderRules,
               QStringLiteral("R7：每刻恰 1 次抽签、不出现空刻（%1 刻 -> %2 行）")
                   .arg(marksForOrderRules)
                   .arg(orderRuleLines.size()));
        expect(maxAddedOrderRules <= 1,
               QStringLiteral("R1：1 个事件刻最多生 1 张单（实测单刻最多 %1 张，共 +%2）")
                   .arg(maxAddedOrderRules)
                   .arg(addedTotal));
        expect(addedTotal == normalDraws + urgentDraws,
               QStringLiteral("R1/R9：抽中订单类事件恰生 1 张单、路况/封路不生单"
                              "（普通抽签 %1 + 紧急抽签 %2 -> 订单 +%3）")
                   .arg(normalDraws)
                   .arg(urgentDraws)
                   .arg(addedTotal));
        expect(normalDraws + urgentDraws >= 1,
               QStringLiteral("（非空跑）%1 个刻里至少抽中一次订单类事件（实测 %2 次）")
                   .arg(marksForOrderRules)
                   .arg(normalDraws + urgentDraws));

        // 补满普通额度：不足 6 张时补**事件来源、未送达**的普通单（只补差额，不越过额度）
        for (int i = 0; rules.pendingEventOrderCount(false) < kMaxPendingNormalOrders; ++i) {
            char id[16];
            std::snprintf(id, sizeof(id), "N%03d", 900 + i);
            Order order;
            order.id = id;
            order.pickupNodeId = deliveryIds[0];
            order.nodeId = deliveryIds[1];
            order.demandKg = 5.0;
            order.windowStartMin = 480;
            order.windowEndMin = 24 * 60;
            order.urgent = false;
            order.source = logistics::OrderSource::EventNormal;
            rules.config_.orders.push_back(order);
        }
        expect(rules.pendingEventOrderCount(false) == kMaxPendingNormalOrders,
               QStringLiteral("R6：普通额度 6 已填满（初始/手动单不计、只数事件来源未送达）"));

        // "已取在车、尚未送达"仍占额度（计数口径 = 未送达，不是"还没取"）
        rules.config_.orders.back().picked = true;
        expect(rules.pendingEventOrderCount(false) == kMaxPendingNormalOrders,
               QStringLiteral("R6：已取在车（未送达）的事件单仍占额度"));
        rules.config_.orders.back().picked = false;

        // 满额后连走 30 个事件刻：每刻恰 1 抽，且抽签里不再有普通订单。
        // 同时逐刻比对订单集合增量，守卫 R1（1 刻至多 1 单）与 R9（抽中就该生单）。
        const int marksFullNormal = 30;
        const int baselineLines = rules.eventLogLines().size();
        int maxAddedCapWindow = 0;
        for (int i = 0; i < marksFullNormal; ++i) {
            const std::size_t before = rules.config_.orders.size();
            rules.settleEventsUpTo(rules.nextEventMark_);
            const int added = static_cast<int>(rules.config_.orders.size())
                              - static_cast<int>(before);
            if (added > maxAddedCapWindow) {
                maxAddedCapWindow = added;
            }
        }
        expect(maxAddedCapWindow <= 1,
               QStringLiteral("R1：满额窗口里 1 个事件刻仍至多生 1 张单（实测最多 %1 张）")
                   .arg(maxAddedCapWindow));
        const QStringList normalCappedLines = rules.eventLogLines().mid(baselineLines);
        expect(normalCappedLines.size() == marksFullNormal,
               QStringLiteral("R7：每刻恰 1 次抽签、不出现空刻（%1 刻 -> %2 行）")
                   .arg(marksFullNormal)
                   .arg(normalCappedLines.size()));
        bool normalDrawn = false;
        bool normalCapNoted = false;
        for (const QString& line : normalCappedLines) {
            if (line.contains(QStringLiteral("抽取：普通订单"))) {
                normalDrawn = true;
            }
            if (line.contains(QStringLiteral("普通订单待处理已满"))) {
                normalCapNoted = true;
            }
        }
        expect(!normalDrawn,
               QStringLiteral("R6：普通额度满额后抽签不再含普通订单（连走 %1 刻）")
                   .arg(marksFullNormal));
        expect(normalCapNoted,
               QStringLiteral("R7：满额原因必须在事件行里注明（「普通订单待处理已满 6」）"));

        // 再把紧急额度填满：两类都满 -> 只剩路况/封路，仍每刻 1 抽
        for (int i = 1; i <= kMaxPendingUrgentOrders; ++i) {
            char id[16];
            std::snprintf(id, sizeof(id), "U%03d", 900 + i);
            Order order;
            order.id = id;
            order.pickupNodeId = deliveryIds[0];
            order.nodeId = deliveryIds[1];
            order.demandKg = 5.0;
            order.windowStartMin = 480;
            order.windowEndMin = 540;
            order.urgent = true;
            order.source = logistics::OrderSource::EventUrgent;
            rules.config_.orders.push_back(order);
        }
        expect(rules.pendingEventOrderCount(true) >= kMaxPendingUrgentOrders,
               QStringLiteral("R6：紧急额度已填满（>= 2，实际 %1）")
                   .arg(rules.pendingEventOrderCount(true)));

        const int marksBothCapped = 20;
        for (int i = 0; i < marksBothCapped; ++i) {
            rules.settleEventsUpTo(rules.nextEventMark_);
        }
        const QStringList bothLines = rules.eventLogLines().mid(baselineLines + marksFullNormal);
        expect(bothLines.size() == marksBothCapped,
               QStringLiteral("R7：两类都满仍每刻 1 抽、不空刻（%1 刻 -> %2 行）")
                   .arg(marksBothCapped)
                   .arg(bothLines.size()));
        bool orderDrawn = false;
        bool bothNoted = true;
        for (const QString& line : bothLines) {
            if (line.contains(QStringLiteral("抽取：普通订单"))
                || line.contains(QStringLiteral("抽取：紧急订单"))) {
                orderDrawn = true;
            }
            if (!line.contains(QStringLiteral("本次抽签不含这两类"))) {
                bothNoted = false;
            }
        }
        expect(!orderDrawn,
               QStringLiteral("R7：普通/紧急两类都满时只抽路况与封路"));
        expect(bothNoted,
               QStringLiteral("R7：两类都满的每一刻都注明「本次抽签不含这两类」"));

        // Q8：抽中但动作客观不可执行（可用配送点 < 2）-> 照实播报原因、**不重抽**、不丢既有单
        logistics::Config tiny = reloadInitialConfig();
        std::vector<std::string> dropIds;
        for (const logistics::Node& n : tiny.graph.nodes()) {
            if (n.type == logistics::NodeType::Delivery && n.id != deliveryIds.front()) {
                dropIds.push_back(n.id);
            }
        }
        for (const std::string& id : dropIds) {
            tiny.graph.removeNode(id);
        }
        MainWindow tinyWindow(tiny, configPath_);
        const std::size_t tinyBefore = tinyWindow.config_.orders.size();
        const std::string failed =
            tinyWindow.insertRandomOrder(false, logistics::OrderSource::EventNormal, 495);
        expect(failed.empty(),
               QStringLiteral("Q8：配送点 < 2 时刷单动作不可执行 -> 不生成订单（不重抽）"));
        expect(tinyWindow.config_.orders.size() == tinyBefore,
               QStringLiteral("Q8：不可执行时不生单、也不丢既有单"));
        expect(tinyWindow.runLogText().contains(QStringLiteral("客户节点不足")),
               QStringLiteral("Q8：失败原因必须照实播报（运行日志含「客户节点不足」）"));
    }

    // ---- 8. 等价性守卫 A1–A7：一次「推进一刻」== 逐个「推进一站」到同一时刻 ----
    //
    // 两条路径各自**重新加载配置**、各构造一个**独立的 MainWindow**：不深拷贝同一个
    // MainWindow 对象，也不共享 rng_ / state_。两条路径都推进到**路线终点**——此时
    // 车队时钟必然同值（末节点到达时刻），是唯一能严格对齐的 T（按站推进的
    // "到达 >= T" 会越过 T，而推进一刻恰好停在刻上）。
    // 断言全部只读**用户可见输出**（运行日志 / 面板同源事实），不经显示函数反推。
    {
        MainWindow pathA(reloadInitialConfig(), configPath_);
        MainWindow pathB(reloadInitialConfig(), configPath_);

        int stepsA = 0;
        while (!pathA.atRouteEnd() && stepsA < 4000) {
            pathA.onAdvanceMoment();   // 路径 A：反复推进一刻
            ++stepsA;
        }
        int stepsB = 0;
        while (!pathB.atRouteEnd() && stepsB < 4000) {
            pathB.onAdvanceStop();     // 路径 B：反复推进一站
            ++stepsB;
        }
        expect(pathA.atRouteEnd() && pathB.atRouteEnd() && stepsA > 0 && stepsB > 0,
               QStringLiteral("两条路径都推进到路线终点（推进一刻 %1 步 / 推进一站 %2 步）")
                   .arg(stepsA)
                   .arg(stepsB));
        expect(pathA.state_.atTimeMin == pathB.state_.atTimeMin,
               QStringLiteral("到达终点时车队时钟一致（A %1 / B %2）")
                   .arg(minutesToClock(pathA.state_.atTimeMin))
                   .arg(minutesToClock(pathB.state_.atTimeMin)));

        auto diffLines = [](const QStringList& a, const QStringList& b) -> QString {
            const int n = static_cast<int>(a.size() < b.size() ? a.size() : b.size());
            for (int i = 0; i < n; ++i) {
                if (a[i] != b[i]) {
                    return QStringLiteral("第 %1 行 A=「%2」 / B=「%3」")
                        .arg(i + 1)
                        .arg(a[i], b[i]);
                }
            }
            if (a.size() != b.size()) {
                return QStringLiteral("行数 A=%1 / B=%2").arg(a.size()).arg(b.size());
            }
            return QString();
        };

        // A1 事件日志序列（逐字/逐值，含时刻前缀）。先证明这条守卫不是空跑。
        const QStringList eventsA = pathA.eventLogLines();
        const QStringList eventsB = pathB.eventLogLines();
        expect(eventsA.size() >= 20,
               QStringLiteral("A1 事件日志至少 20 行（实际 %1 行，确保守卫真的抽到了事件）")
                   .arg(eventsA.size()));
        expect(eventsA == eventsB,
               QStringLiteral("A1 事件日志序列逐字一致（差异：%1）")
                   .arg(diffLines(eventsA, eventsB)));

        // A2–A7 终态快照逐字一致
        const QString snapA = pathA.equivalenceSnapshot();
        const QString snapB = pathB.equivalenceSnapshot();
        expect(snapA == snapB,
               QStringLiteral("A2–A7 终态逐字一致（差异：%1）")
                   .arg(diffLines(snapA.split(QLatin1Char('\n')),
                                  snapB.split(QLatin1Char('\n')))));
        std::printf("[self-check] 等价性终态（两条路径一致）：\n%s", snapA.toUtf8().constData());
    }

    return failures;
}
void MainWindow::renderToFile(const QString& path, int width, int height) {
    resize(width, height);
    show();
    QCoreApplication::processEvents();

    // 目标目录可能不存在（尤其是刚做过 clean build），主动创建
    const QFileInfo info(path);
    if (!info.absolutePath().isEmpty()) {
        QDir().mkpath(info.absolutePath());
    }

    const QPixmap shot = grab();
    const bool saved = !shot.isNull() && shot.save(path);
    std::fprintf(stderr, "[render-window] pixmap=%dx%d saved=%d path=%s\n", shot.width(),
                 shot.height(), saved ? 1 : 0, path.toUtf8().constData());
}
