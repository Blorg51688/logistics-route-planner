#include "gui/MainWindow.h"

#include "core/SimEvent.h"

#include <QAction>
#include <QActionGroup>
#include <QPainter>
#include <QComboBox>
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

QString minutesToClock(int minutes) {
    const int h = minutes / 60;
    const int m = minutes % 60;
    return QStringLiteral("%1:%2")
        .arg(h, 2, 10, QLatin1Char('0'))
        .arg(m, 2, 10, QLatin1Char('0'));
}

} // namespace

MainWindow::MainWindow(Config config, QWidget* parent)
    : QMainWindow(parent), config_(std::move(config)) {
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
        state_.tripNumber = 1;
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
    // 像「订单列表」那样分标签，把两类信息分开：
    //   · 固定信息 —— 车辆的**固有参数**（ID / 起始仓库 / 载重上限 / 发车时刻），
    //     整场模拟都不会变，任何时候看都一样；
    //   · 运行状态 —— **随模拟推进而变**的量（软件内时间 / 车上载重及构成 / 本趟装载 /
    //     剩余待送）。它们每推进一步都会变，混在一起会让"固定参数"被噪声淹没。
    // 分成两个只读页签后，用户不必在变化的数字里找不变的参数。
    vehicleTabs_ = new QTabWidget(vehicleDock);
    vehicleFixedInfo_ = new QLabel(vehicleTabs_);
    vehicleFixedInfo_->setTextFormat(Qt::PlainText);
    vehicleFixedInfo_->setMargin(6);
    vehicleFixedInfo_->setAlignment(Qt::AlignTop | Qt::AlignLeft);
    vehicleStateInfo_ = new QLabel(vehicleTabs_);
    vehicleStateInfo_->setTextFormat(Qt::PlainText);
    vehicleStateInfo_->setMargin(6);
    vehicleStateInfo_->setAlignment(Qt::AlignTop | Qt::AlignLeft);
    vehicleTabs_->addTab(vehicleFixedInfo_, QStringLiteral("固定信息"));
    vehicleTabs_->addTab(vehicleStateInfo_, QStringLiteral("运行状态"));
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

    // 中转站 / 集散面板：承担 P3 的"子网络分组标识"与下属配送点归属
    auto* transitDock = new QDockWidget(QStringLiteral("中转站 / 集散"), this);
    transitDock->setMinimumWidth(360);
    transitTable_ = new QTableWidget(0, 3, transitDock);
    transitTable_->setHorizontalHeaderLabels(
        {QStringLiteral("中转站"), QStringLiteral("子网络"), QStringLiteral("下属配送点")});
    transitTable_->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    transitTable_->horizontalHeader()->setStretchLastSection(true);
    transitTable_->verticalHeader()->setVisible(false);
    transitTable_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    transitDock->setWidget(transitTable_);
    addDockWidget(Qt::RightDockWidgetArea, transitDock);

    // 停靠明细：把每个停靠点的原始到达/等待/送达/送后余载摊开。
    // （"离开"列已随服务时间一并删除：服务时间移除后「离开 == 送达」是恒等式，
    //   再单列一列只会与"送达"重复，见 docs/设计.md §16 P-history 与 §4.6。）
    // 这些字段是设计 §4.6 明确要求记录的，此前只有测试在读、界面上看不到；
    // 报告要求【需求分析】⑵ 也要求呈现"到达时间"，而"等待"能解释早到的影响。
    auto* stopDock = new QDockWidget(QStringLiteral("停靠明细"), this);
    stopDock->setMinimumWidth(360);
    // 列里必须有「趟」：停靠明细是**跨趟拉平**的，不加这一列的话
    // "送后余货"会从 0 跳回几十公斤，看起来像数据错了，其实是新的一趟开始装货。
    // 列名用"**余货**"而不是"余载"：该值只统计**订单货**（`Stop::remainingLoadKg`）。
    stopTable_ = new QTableWidget(0, 6, stopDock);
    stopTable_->setHorizontalHeaderLabels(
        {QStringLiteral("趟"), QStringLiteral("配送点"), QStringLiteral("原始到达"),
         QStringLiteral("等待(分)"), QStringLiteral("送达"),
         QStringLiteral("送后余货(kg)")});
    stopTable_->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    stopTable_->horizontalHeader()->setStretchLastSection(true);
    stopTable_->verticalHeader()->setVisible(false);
    stopTable_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    stopDock->setWidget(stopTable_);
    addDockWidget(Qt::RightDockWidgetArea, stopDock);

    auto* lateDock = new QDockWidget(QStringLiteral("超时订单"), this);
    lateDock->setMinimumWidth(360);
    lateTable_ = new QTableWidget(0, 6, lateDock);
    lateTable_->setHorizontalHeaderLabels(
        {QStringLiteral("订单"), QStringLiteral("配送点"), QStringLiteral("状态"),
         QStringLiteral("到达"), QStringLiteral("窗口"), QStringLiteral("penalty(min)")});;
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


std::vector<logistics::OnboardItem> MainWindow::onboardGoods() const {
    // 车上有哪些货是**显式状态**，不再从计划反推。
    // 反推的版本会在重规划后把已经送掉/已经寄存的货又当成在途货，造成重复寄存与
    // "车不回仓库装货"的坏轨迹（设计 §16 P17）。
    return state_.onboard;
}

void MainWindow::loadForTrip(std::size_t tripIndex) {
    if (config_.vehicles.empty() || tripIndex >= plan_.trips.size()) {
        return;   // 没有这一趟，什么都不装
    }
    const logistics::Trip& trip = plan_.trips[tripIndex];
    // 取送式紧急趟（issue #3/#4）：货在取货点 A 装车（onAdvanceStop 的取货段），
    // **不能**按 stops 装载，也**不能**清空车上已有的普通货——否则刚装上车的
    // 在途货会凭空消失，随后的"降载/取回"也会因为货不在车上而失去物理意义。
    // 车上普通货保持不动；本趟装载量 = 出发时的实际载重。
    if (!trip.pickups.empty()) {
        state_.tripLoadKg = state_.loadKg;
        state_.departed = false;
        return;
    }
    state_.onboard.clear();
    for (const logistics::Stop& s : trip.stops) {
        double kg = 0.0;
        for (const logistics::Order& o : config_.orders) {
            if (o.nodeId == s.nodeId && !o.served) {
                kg += o.demandKg;
            }
        }
        if (kg > 1e-9) {
            logistics::OnboardItem item;
            item.nodeId = s.nodeId;
            item.kg = kg;
            state_.onboard.push_back(item);
        }
    }
    state_.loadKg = state_.sumOnboard();
    state_.tripLoadKg = state_.loadKg;
    // 导出用的装车台帐：这一趟出发时车上装了什么（订单货 / 合计）。
    //
    // **必须区分"出仓"与"接着送"**：只有 `nodes.front()` 是起始仓库的趟才是出仓。
    // 早先把每一次装载都写成「出仓装车」——那些趟的起点其实是 D01 / T03
    // （在途货续送），于是导出文件里凭空多出几条"出仓"记录，会把读证据的人
    // 引去追一个**根本不存在**的 bug。误报比没有证据更糟，所以这里分开写。
    {
        const double cap = config_.vehicles.empty() ? 0.0 : config_.vehicles.front().capacityKg;
        const bool fromDepot =
            trip.nodes.empty() || trip.nodes.front() == config_.vehicles.front().startNodeId;
        if (!fromDepot) {
            recordRun(QStringLiteral("接着送车上已有的货（第 %1 趟，起点 %2，**非出仓**）："
                                     "订单货 %3 kg（载重上限 %4 kg）")
                          .arg(tripIndex + 1)
                          .arg(QString::fromStdString(trip.nodes.front()))
                          .arg(trip.loadKg, 0, 'f', 0)
                          .arg(cap, 0, 'f', 0));
            state_.departed = false;
            return;
        }
        recordRun(QStringLiteral("出仓装车（第 %1 趟）：订单货 %2 kg（载重上限 %3 kg）")
                      .arg(tripIndex + 1)
                      .arg(trip.loadKg, 0, 'f', 0)
                      .arg(cap, 0, 'f', 0));
    }
    state_.departed = false;
}


void MainWindow::loadForCurrentTrip() {
    // 车在仓库时，装载它**即将开始**的那一趟。
    // 计划是从车辆当前位置起算的，所以车在仓库时"即将开始"的就是计划里的第 0 趟；
    // 若车是**刚跑完一趟回到仓库**（推进中到达），则下一趟才是要装的。
    if (config_.vehicles.empty()
        || state_.atNodeId != config_.vehicles.front().startNodeId) {
        return;
    }
    const std::size_t cur = currentTripIndex();
    const bool justArrived = (nodeIndex_ > 0 && !state_.departed
                              && cur + 1 < plan_.trips.size());
    loadForTrip(justArrived ? cur + 1 : cur);
}

void MainWindow::arriveAt(const std::string& nodeId, int timeMin) {
    state_.atNodeId = nodeId;
    state_.atTimeMin = timeMin;
    if (!config_.vehicles.empty() && nodeId == config_.vehicles.front().startNodeId
        && plan_.nodes.size() > 1) {
        // 回到仓库 = 上一趟结束。注意：首次位于仓库（尚未出发）不算"跑完一趟"。
        if (state_.tripNumber > 1 || state_.departed) {
            ++state_.completedTrips;
            state_.tripNumber = state_.completedTrips + 1;
        }
        state_.departed = false;
    } else {
        state_.departed = true;
    }
}

void MainWindow::deliverAt(const logistics::Stop& stop) {
    // 卸货 + 记账。这两件事都是"已经发生的事实"，重规划不得改写。
    //
    // issue #5：一次停靠可能载着**同一节点上的多条货**（例如 B 点既有紧急包裹、
    // 又有 B 的普通货，或同一节点合并的多张普通单）。必须把该节点上的货**全部**
    // 卸下并累加 kg，而不是只移除一条——否则会残留一条已送过的货，被后续
    // 重规划当成在途货再送一遍。
    bool offloaded = false;
    for (std::size_t i = state_.onboard.size(); i > 0; --i) {
        const std::size_t k = i - 1;
        if (state_.onboard[k].nodeId == stop.nodeId) {
            state_.onboard.erase(state_.onboard.begin() + static_cast<std::ptrdiff_t>(k));
            offloaded = true;
        }
    }
    if (!offloaded) {
        // 送了一批车上没有的货 —— 说明"装载"与"计划"脱节了（历史上真的发生过：
        // 车到仓库后装错了趟，于是空车出发去送货）。
        ++state_.offloadedWithoutGoods;
    }
    state_.loadKg = state_.sumOnboard();
    ++state_.servedStops;
    if (stop.late) {
        state_.incurredPenaltyMin += stop.penaltyMin;
        VehicleState::LateStop ls;
        ls.orderId = orderIdsAt(stop.nodeId).toStdString();
        ls.nodeId = stop.nodeId;
        ls.arrivalMin = stop.arrivalMin;
        ls.penaltyMin = stop.penaltyMin;
        state_.deliveredLate.push_back(ls);
    }
}


QString MainWindow::orderIdsAt(const std::string& nodeId) const {
    QString ids;
    for (const Order& order : config_.orders) {
        if (order.nodeId != nodeId) {
            continue;
        }
        if (!ids.isEmpty()) {
            ids += QLatin1Char('/');
        }
        ids += QString::fromStdString(order.id);
    }
    return ids;
}

// 某配送点的合并窗口 [min 起, max 止]，与规划器使用的口径一致
QString MainWindow::windowTextAt(const std::string& nodeId) const {
    int lo = -1;
    int hi = -1;
    for (const Order& order : config_.orders) {
        if (order.nodeId != nodeId) {
            continue;
        }
        if (lo < 0 || order.windowStartMin < lo) {
            lo = order.windowStartMin;
        }
        if (hi < 0 || order.windowEndMin > hi) {
            hi = order.windowEndMin;
        }
    }
    if (lo < 0) {
        return QStringLiteral("-");
    }
    return minutesToClock(lo) + QStringLiteral("-") + minutesToClock(hi);
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
// replanIncremental / insertUrgentOrder）都调它，避免各写一份而漏改。
void MainWindow::afterPlanReplaced() {
    nodeIndex_ = 0;      // 新路线从车辆当前位置起算，推进游标归零
    stopCursor_ = 0;
    // 新计划里还有路要走 => 重新允许"到终点播报一次"。
    // 否则跑完一次后再「重新规划」，第二次跑完就再也播报不出来了。
    if (!atRouteEnd()) {
        routeFinishedReported_ = false;
    }
    // 车若正在仓库，就把计划的当前趟装上车——装载必须在每次（重新）规划之后发生，
    // 否则新计划第一趟的货永远上不了车。
    loadForCurrentTrip();
}

void MainWindow::replan() {
    if (config_.vehicles.empty()) {
        appendLog(QStringLiteral("配置中没有车辆，无法规划"));
        return;
    }
    const std::string here = currentPositionId();
    const int now = currentTimeMin();
    const std::vector<Order> remaining = remainingOrders();

    plan_ = logistics::replan(config_.graph, config_.vehicles.front(), remaining, here, now,
                              planWeight_, onboardGoods());
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
    appendLog(QStringLiteral("路况变化：改动 %1 条边（新增拥堵 %2 条，转为畅通 %3 条）")
                  .arg(report.changes.size())
                  .arg(congestedCount)
                  .arg(clearedCount));

    const bool trigger = logistics::needsReplan(plan_.nodes, report,
                                                config_.general.trafficTimeIncreaseMin);
    if (!trigger) {
        appendLog(QStringLiteral("受影响边不在当前路径或增幅不足 -> 不触发重规划"));
        syncScene();   // 拥堵标记需要重绘
        return;
    }

    appendLog(QStringLiteral("受影响边位于当前路径且增幅达标 -> 自动触发重规划"));

    // 增量式重规划（D22）：只把**尚未走完的那一段**交给它，
    // 它会按停靠点切段、仅重算走过被命中边的段，停靠顺序不变。
    // 多趟方案或某段重算后不可达时，它内部自动退回全量重算。
    // 按**趟**切分剩余路线，保留真实的趟结构（逻辑在 core，可单测）。
    // 曾经这里把整条剩余路线塞成"一趟"，结果增量重规划之后 plan_.trips 只剩 1 趟，
    // 界面上的「第 N 趟」全变成「第 1 趟」、「共 N 趟」变成「共 1 趟」。
    const logistics::RoutePlan remainder = logistics::sliceRemainder(plan_, nodeIndex_);

    // 由核心**回报**是否真的走了增量，界面不要自己猜。
    // 曾经这里用 plan_.trips.size()==1 猜，多趟方案下恒为 false，
    // 于是真的走了增量也打印"退回全量重算"，与实际相反（审计发现的 #17）。
    bool usedIncremental = false;
    // 换计划之前，把"当前计划里已经走过的停靠点"并入记账（三条重规划路径都要做，
    // 否则界面上的「已送达 N 站」会归 0、「停靠 N 站」会缩水成剩余数）
    plan_ = logistics::replanIncremental(config_.graph, config_.vehicles.front(),
                                         remainingOrders(), remainder, currentTimeMin(),
                                         planWeight_,
                                         report, config_.general.trafficTimeIncreaseMin,
                                         onboardGoods(), &usedIncremental);
    afterPlanReplaced();
    appendLog(usedIncremental
                  ? QStringLiteral("  → 增量式重规划：仅重算受影响的路段，其余原样保留")
                  : QStringLiteral("  → 增量不适用（序列不全或路段已不可达），退回全量重算"));
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

std::string MainWindow::insertUrgentOrderAt(const std::string& a, const std::string& b,
                                            double demand) {
    if (config_.vehicles.empty()) {
        appendLog(QStringLiteral("配置中没有车辆"));
        return std::string();
    }
    const logistics::Node* na = config_.graph.findNode(a);
    const logistics::Node* nb = config_.graph.findNode(b);
    if (na == nullptr || nb == nullptr
        || na->type != logistics::NodeType::Delivery
        || nb->type != logistics::NodeType::Delivery) {
        appendLog(QStringLiteral("紧急取送单：取货点 A、送达点 B 必须是客户（配送）节点"));
        return std::string();
    }
    if (a == b) {
        appendLog(QStringLiteral("紧急取送单：起点 A 与终点 B 不能相同"));
        return std::string();
    }
    const double cap = config_.vehicles.front().capacityKg;
    if (!(demand > 0.0) || demand > cap + 1e-9) {
        appendLog(QStringLiteral("紧急取送单：送货量须在 (0, 载重上限] 内"));
        return std::string();
    }
    const logistics::PathResult toA = logistics::shortestPath(
        config_.graph, currentPositionId(), a, logistics::WeightType::Time);
    const logistics::PathResult ab =
        logistics::shortestPath(config_.graph, a, b, logistics::WeightType::Time);
    if (!toA.found || !ab.found) {
        appendLog(QStringLiteral("紧急取送单：取货点 A 或送达点 B 不可达"));
        return std::string();
    }

    Order urgent;
    urgent.id = nextFreeId("U");
    urgent.pickupNodeId = a;
    urgent.nodeId = b;
    urgent.demandKg = std::floor(demand);
    urgent.windowStartMin = currentTimeMin();
    urgent.windowEndMin = currentTimeMin() + 60;
    urgent.urgent = true;
    urgent.served = false;

    const std::vector<Order> pending = remainingOrders();
    const logistics::InsertResult inserted = logistics::insertUrgentOrder(
        config_.graph, config_.vehicles.front(), pending, urgent, currentPositionId(),
        currentTimeMin(), planWeight_, onboardGoods());

    // 必须并入 config_.orders，否则连续插单会丢单、订单表也看不见。
    config_.orders.push_back(urgent);

    appendLog(QStringLiteral("插入紧急取送单 %1：%2 取 → %3 送（货量 %4kg，%5 前送达）")
                  .arg(QString::fromStdString(urgent.id))
                  .arg(QString::fromStdString(urgent.pickupNodeId))
                  .arg(QString::fromStdString(urgent.nodeId))
                  .arg(urgent.demandKg, 0, 'f', 0)
                  .arg(minutesToClock(urgent.windowEndMin)));
    if (!inserted.warning.empty()) {
        appendLog(QStringLiteral("  ⚠ %1").arg(QString::fromStdString(inserted.warning)));
    }
    plan_ = inserted.plan;
    afterPlanReplaced();
    syncScene();
    updatePanels();
    return urgent.id;
}

std::string MainWindow::insertUrgentOrderAction() {
    // 事件触发 / 自检 / 演示路径：随机取两个不同客户节点作 A、B，货量 3–10kg 整公斤。
    std::vector<std::string> cust;
    for (const logistics::Node& n : config_.graph.nodes()) {
        if (n.type == logistics::NodeType::Delivery) {
            cust.push_back(n.id);
        }
    }
    if (cust.size() < 2) {
        appendLog(QStringLiteral("客户节点不足，无法插入紧急取送单"));
        return std::string();
    }
    int ia = rng_.nextInt(0, static_cast<int>(cust.size()) - 1);
    int ib = rng_.nextInt(0, static_cast<int>(cust.size()) - 1);
    if (ib == ia) {
        ib = (ib + 1) % static_cast<int>(cust.size());
    }
    const double demand = std::floor(3.0 + rng_.nextRange(0.0, 8.0));
    return insertUrgentOrderAt(cust[static_cast<std::size_t>(ia)],
                               cust[static_cast<std::size_t>(ib)], demand);
}

void MainWindow::onInsertUrgentOrder() {
    // 无头环境（--self-check-actions / --demo / offscreen 渲染）不能弹模态框：
    // 退化为随机 A/B，保持自动化路径可跑。
    if (QGuiApplication::platformName() == QStringLiteral("offscreen")) {
        insertUrgentOrderAction();
        return;
    }
    // GUI：手选取货点 A、送达点 B、送货量。
    QStringList items;
    for (const logistics::Node& n : config_.graph.nodes()) {
        if (n.type == logistics::NodeType::Delivery) {
            items << QString::fromStdString(n.id);
        }
    }
    if (items.size() < 2) {
        appendLog(QStringLiteral("客户节点不足，无法插入紧急取送单"));
        return;
    }
    bool ok = false;
    const QString a = QInputDialog::getItem(this, QStringLiteral("插入紧急取送单"),
                                            QStringLiteral("取货点 A："), items, 0, false, &ok);
    if (!ok || a.isEmpty()) {
        return;
    }
    QStringList itemsB;
    for (const QString& s : items) {
        if (s != a) {
            itemsB << s;
        }
    }
    const QString b = QInputDialog::getItem(this, QStringLiteral("插入紧急取送单"),
                                            QStringLiteral("送达点 B："), itemsB, 0, false, &ok);
    if (!ok || b.isEmpty()) {
        return;
    }
    const double cap = config_.vehicles.empty() ? 0.0 : config_.vehicles.front().capacityKg;
    const double dem = QInputDialog::getDouble(this, QStringLiteral("插入紧急取送单"),
                                               QStringLiteral("送货量(kg)："), 10.0, 1.0,
                                               cap > 1.0 ? cap : 1.0, 0, &ok);
    if (!ok) {
        return;
    }
    insertUrgentOrderAt(a.toStdString(), b.toStdString(), dem);
}

std::string MainWindow::addRandomCustomerAction() {
    const std::string nodeId = logistics::addRandomCustomer(config_.graph, rng_);
    if (nodeId.empty()) {
        appendLog(QStringLiteral("新增客户失败"));
        return std::string();
    }

    // 新客户必然带来一个配送需求。
    // 只加节点而不加订单的话，规划器没有理由访问它——停靠点只由订单决定，
    // 新节点会永远不出现在路线里。这正是人工测试第 11 关发现的问题。
    Order order;
    order.id = nextFreeId("C");
    order.nodeId = nodeId;
    // 同上：货量一律整公斤（nextRange + floor，理由见 insertUrgentOrderAction）；区间仍是 3–15kg。
    order.demandKg = std::floor(3.0 + rng_.nextRange(0.0, 13.0));
    order.windowStartMin = currentTimeMin();
    order.windowEndMin = 24 * 60;
    order.urgent = false;
    order.served = false;
    config_.orders.push_back(order);

    appendLog(QStringLiteral("模拟新客户：新增配送点 %1 与订单 %2（货量 %3kg）")
                  .arg(QString::fromStdString(nodeId))
                  .arg(QString::fromStdString(order.id))
                  .arg(order.demandKg, 0, 'f', 0));
    replan();
    return nodeId;
}

void MainWindow::onAddRandomCustomer() {
    addRandomCustomerAction();
}

void MainWindow::onCloseRandomRoad() {
    const std::string closed = logistics::closeRandomRoad(config_.graph, rng_);
    if (closed.empty()) {
        appendLog(QStringLiteral("没有可封闭的道路"));
        return;
    }
    appendLog(QStringLiteral("模拟道路封闭：%1（含反向）").arg(QString::fromStdString(closed)));
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

    // 逐个节点前进。**状态转移只发生在这一处**：
    // 位置/时刻 -> arriveAt，卸货/记账 -> deliverAt，回到仓库后的装载 -> loadForCurrentTrip。
    // 除此之外任何地方都不得改 state_。
    ++nodeIndex_;
    const std::string node = plan_.nodes[nodeIndex_];
    int arrTime = state_.atTimeMin;
    if (nodeIndex_ < plan_.nodeArrivalMin.size()) {
        arrTime = plan_.nodeArrivalMin[nodeIndex_];
    }

    const bool wasAtDepot = !config_.vehicles.empty()
                            && state_.atNodeId == config_.vehicles.front().startNodeId;
    arriveAt(node, arrTime);
    currentNodeId_ = state_.atNodeId;
    currentTimeMin_ = state_.atTimeMin;

    // 刚到仓库：上一趟跑完，装载**下一趟**（"回仓库装货再出发"这一物理事件）。
    // 必须在 arriveAt 之后调用（它会更新 tripNumber / departed）。
    if (!wasAtDepot && !config_.vehicles.empty()
        && state_.atNodeId == config_.vehicles.front().startNodeId) {
        loadForCurrentTrip();
    }

    // 降载 / 取回（issue #4）：紧急包裹装不下时，计划里那一趟会先到中转站 S
    // 把整条普通货卸下暂存，送完紧急包裹后**同一趟**再回 S 取回。
    // S 在一趟里出现两次：第一次执行降载、第二次执行取回，靠"是否已降载且
    // 包裹已取"区分（appliedOffloads 去重，物理事实只发生一次）。
    if (nodeIndex_ < plan_.nodeTripIndex.size()) {
        const std::size_t ti = plan_.nodeTripIndex[nodeIndex_];
        if (ti < plan_.trips.size()) {
            const logistics::Trip& trip = plan_.trips[ti];
            // ① 降载：第一次经过暂存站 S（此时包裹还没取，取货点 A 在 S 之后）
            for (const logistics::OffloadOp& op : trip.offloads) {
                if (op.station != node) {
                    continue;
                }
                const std::string key = op.station + "@" + op.nodeId + "@"
                                        + std::to_string(std::lround(op.kg));
                if (!state_.appliedOffloads.insert(key).second) {
                    continue;   // 第二次经过同一个站时不重复卸
                }
                bool removed = false;
                for (std::size_t i = 0; i < state_.onboard.size(); ++i) {
                    if (state_.onboard[i].nodeId == op.nodeId) {
                        state_.onboard.erase(state_.onboard.begin()
                                                 + static_cast<std::ptrdiff_t>(i));
                        removed = true;
                        break;
                    }
                }
                state_.loadKg = state_.sumOnboard();
                appendLog(QStringLiteral("降载：在 %1 卸下 %2 的货 %3kg（暂存，送完紧急包裹后取回）%4")
                              .arg(QString::fromStdString(op.station))
                              .arg(QString::fromStdString(op.nodeId))
                              .arg(op.kg, 0, 'f', 0)
                              .arg(removed ? QString()
                                           : QStringLiteral("（车上无此货，仅记账）")));
            }
            // ② 取回：再次经过 S（包裹已在车上）且尚未取回
            if (!trip.offloads.empty() && trip.reclaimStation == node) {
                bool allOffloaded = true;
                bool allPicked = true;
                for (const logistics::OffloadOp& op : trip.offloads) {
                    const std::string key = op.station + "@" + op.nodeId + "@"
                                            + std::to_string(std::lround(op.kg));
                    if (state_.appliedOffloads.count(key) == 0) {
                        allOffloaded = false;
                    }
                }
                for (const logistics::PickupOp& pk : trip.pickups) {
                    if (state_.pickedOrderIds.count(pk.orderId) == 0) {
                        allPicked = false;
                    }
                }
                if (allOffloaded && allPicked) {
                    bool anyReclaimed = false;
                    for (const logistics::OffloadOp& op : trip.offloads) {
                        const std::string key = "R@" + op.station + "@" + op.nodeId + "@"
                                                + std::to_string(std::lround(op.kg));
                        if (!state_.appliedOffloads.insert(key).second) {
                            continue;
                        }
                        logistics::OnboardItem item;
                        item.nodeId = op.nodeId;
                        item.kg = op.kg;
                        state_.onboard.push_back(item);
                        anyReclaimed = true;
                    }
                    if (anyReclaimed) {
                        state_.loadKg = state_.sumOnboard();
                        appendLog(QStringLiteral("取回暂存货：在 %1 取回 %2kg，返回仓库")
                                      .arg(QString::fromStdString(trip.reclaimStation))
                                      .arg(trip.reclaimKg, 0, 'f', 0));
                    }
                }
            }
        }
    }

    // 取货：到达某趟的取货点 A 时，把紧急包裹装上（物理事实，只加一次）。
    // 放在送达判定之前——A 不是配送停靠点（nodeIsStop=false），不应被当作"送达"。
    if (nodeIndex_ < plan_.nodeTripIndex.size()) {
        const std::size_t ti = plan_.nodeTripIndex[nodeIndex_];
        if (ti < plan_.trips.size()) {
            for (const logistics::PickupOp& pk : plan_.trips[ti].pickups) {
                if (pk.pickupNodeId != node) {
                    continue;
                }
                if (state_.pickedOrderIds.insert(pk.orderId).second) {
                    logistics::OnboardItem item;
                    item.nodeId = pk.deliverNodeId;
                    item.kg = pk.kg;
                    state_.onboard.push_back(item);
                    state_.loadKg = state_.sumOnboard();
                    appendLog(QStringLiteral("取货 %1：在 %2 装上包裹 %3kg（送往 %4）")
                                  .arg(QString::fromStdString(pk.orderId))
                                  .arg(QString::fromStdString(pk.pickupNodeId))
                                  .arg(pk.kg, 0, 'f', 0)
                                  .arg(QString::fromStdString(pk.deliverNodeId)));
                }
            }
        }
    }

    // 只有**下标与节点对得上**时才认作送达。
    // 曾经这里只看 nodeIsStop 与下标，重规划后两者可能失同步，
    // 于是拿到的是**另一个停靠点**：扣错货（车上有的没扣、没有的扣了），
    // 表现为"2 趟送完 740kg"这种物理上不可能的结果。
    const bool isStopHere = nodeIndex_ < plan_.nodeIsStop.size()
                            && plan_.nodeIsStop[nodeIndex_];
    const bool cursorMatches = isStopHere && stopCursor_ < plan_.stops.size()
                               && plan_.stops[stopCursor_].nodeId == plan_.nodes[nodeIndex_];
    if (cursorMatches) {
        const Stop& stop = plan_.stops[stopCursor_];
        ++stopCursor_;
        // issue #5：该停靠点若同时也是本趟的取货点 A，说明这里是「顺路交付」——
        // 到 A 取紧急包裹的同时，把车上本就要送到 A 的普通货一并交付。
        bool passingDelivery = false;
        if (nodeIndex_ < plan_.nodeTripIndex.size()) {
            const std::size_t ti = plan_.nodeTripIndex[nodeIndex_];
            if (ti < plan_.trips.size()) {
                for (const logistics::PickupOp& pk : plan_.trips[ti].pickups) {
                    if (!pk.pickupNodeId.empty() && pk.pickupNodeId == stop.nodeId) {
                        passingDelivery = true;
                        break;
                    }
                }
            }
        }
        if (passingDelivery) {
            appendLog(QStringLiteral("顺路交付 %1（取货点同点，到达 %2%3）")
                          .arg(QString::fromStdString(stop.nodeId))
                          .arg(minutesToClock(stop.arrivalMin))
                          .arg(stop.late ? QStringLiteral("，超时 penalty %1min").arg(stop.penaltyMin)
                                         : QString()));
        } else {
            appendLog(QStringLiteral("送达 %1（到达 %2%3）")
                          .arg(QString::fromStdString(stop.nodeId))
                          .arg(minutesToClock(stop.arrivalMin))
                          .arg(stop.late ? QStringLiteral("，超时 penalty %1min").arg(stop.penaltyMin)
                                         : QString()));
        }
        // 导出用的**到达时载重明细**（在卸货之前记，反映"到达这一刻车上载着什么"）。
        // 这是用户要向缺陷修复对话出示的核心证据。
        {
            const double cap = config_.vehicles.empty() ? 0.0 : config_.vehicles.front().capacityKg;
            recordRun(QStringLiteral("    └ 到达 %1 时车上载重：订单货 %2 kg"
                                     "（载重上限 %3 kg）")
                          .arg(QString::fromStdString(stop.nodeId))
                          .arg(state_.loadKg, 0, 'f', 0)
                          .arg(cap, 0, 'f', 0));
        }
        deliverAt(stop);
        for (Order& order : config_.orders) {
            if (order.nodeId == stop.nodeId) {
                order.served = true;
            }
        }
    } else if (nodeIndex_ + 1 == plan_.nodes.size()) {
        appendLog(QStringLiteral("返回仓库 %1（到达 %2），本次配送结束")
                      .arg(QString::fromStdString(currentNodeId_))
                      .arg(minutesToClock(currentTimeMin_)));
    } else {
        appendLog(QStringLiteral("经过 %1（到达 %2）")
                      .arg(QString::fromStdString(currentNodeId_))
                      .arg(minutesToClock(currentTimeMin_)));
    }

    // 软件内时钟可能刚跨过一个事件刻：事件由"时间前进"驱动，与推进粒度无关。
    // 必须放在 syncScene 之前 —— 事件可能触发重规划并改写 plan_。
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

    // 车卡在 i→i+1 途中：让时间走到 target，位置留在 i —— i 正是它所在路段的**起点**
    // （用户要的"半路显示所在段的起点"就是这个语义，不需要额外换算）。
    if (!atRouteEnd() && state_.atTimeMin < target) {
        advanceClockTo(target);
        settleEventsUpTo(target);
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

        logistics::EventWeights w;
        w.traffic  = config_.general.eventWeightTraffic;
        w.urgent   = config_.general.eventWeightUrgent;
        w.customer = config_.general.eventWeightCustomer;
        w.closure  = config_.general.eventWeightClosure;

        // 紧急订单已达"待处理上限"时**改抽其余事件**，保证不出现空刻。
        std::size_t pendingUrgent = 0;
        for (const Order& order : config_.orders) {
            if (!order.served && order.urgent) {
                ++pendingUrgent;
            }
        }
        const std::size_t kMaxPendingUrgent = 2;
        const bool urgentCapped = pendingUrgent >= kMaxPendingUrgent;
        if (urgentCapped) {
            w.urgent = 0;
        }

        const logistics::SimEventKind kind = logistics::pickEvent(w, rng_);
        // **传事件刻 mark**：一次推进可能跨过好几个事件刻（"推进一刻"或按站模拟跨 15min），
        // 那些事件是在同一瞬间被批量结算的。若用当前时刻做前缀，导出文件会写成
        // 「09:00 [事件] 08:15 抽取…」——把"记录时刻"误当"事件时刻"，正是最容易被
        // 误读的地方。事件行一律用**它自己的时刻**。
        appendLog(QStringLiteral("[事件] %1 抽取：%2%3")
                      .arg(minutesToClock(mark))
                      .arg(QString::fromUtf8(logistics::simEventName(kind)))
                      .arg(urgentCapped
                               ? QStringLiteral("（紧急订单待处理已满，本次抽签不含紧急订单）")
                               : QString()),
                  mark);

        switch (kind) {
            case logistics::SimEventKind::Traffic:     onSimulateTraffic();       break;
            case logistics::SimEventKind::UrgentOrder: insertUrgentOrderAction(); break;
            case logistics::SimEventKind::NewCustomer: addRandomCustomerAction(); break;
            case logistics::SimEventKind::RoadClosure: onCloseRandomRoad();       break;
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

    auto* fromEdit = new QLineEdit(&dialog);
    auto* toEdit = new QLineEdit(&dialog);
    auto* nodeEdit = new QLineEdit(&dialog);

    auto* form = new QFormLayout;
    form->addRow(QStringLiteral("起点 ID"), fromEdit);
    form->addRow(QStringLiteral("终点 ID"), toEdit);
    form->addRow(QStringLiteral("节点 ID（删除用）"), nodeEdit);

    auto* addEdgeBtn = new QPushButton(QStringLiteral("添加边"), &dialog);
    auto* delEdgeBtn = new QPushButton(QStringLiteral("删除边"), &dialog);
    auto* delNodeBtn = new QPushButton(QStringLiteral("删除节点"), &dialog);
    auto* buttons = new QHBoxLayout;
    buttons->addWidget(addEdgeBtn);
    buttons->addWidget(delEdgeBtn);
    buttons->addWidget(delNodeBtn);

    auto* closeBox = new QDialogButtonBox(QDialogButtonBox::Close, &dialog);

    auto* layout = new QVBoxLayout(&dialog);
    layout->addLayout(form);
    layout->addLayout(buttons);
    layout->addWidget(closeBox);

    connect(closeBox, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);

    connect(addEdgeBtn, &QPushButton::clicked, this, [&] {
        const std::string from = fromEdit->text().toStdString();
        const std::string to = toEdit->text().toStdString();

        // 与"删除边"对称：删除会同时删掉两个方向，添加也应构建双向边，
        // 否则操作不对称，用户加完会发现只多了一条单向边。
        logistics::Edge forward;
        logistics::Edge backward;
        if (!logistics::makeSyntheticEdge(config_.graph, from, to, forward)
            || !logistics::makeSyntheticEdge(config_.graph, to, from, backward)) {
            appendLog(QStringLiteral("添加边失败：端点不存在或坐标重合（%1 <-> %2）")
                          .arg(fromEdit->text(), toEdit->text()));
            return;
        }
        if (!config_.graph.addEdge(forward)) {
            appendLog(QStringLiteral("添加边失败：%1 <-> %2 已存在")
                          .arg(fromEdit->text(), toEdit->text()));
            return;
        }
        config_.graph.addEdge(backward);
        appendLog(QStringLiteral("手工添加边 %1 <-> %2（双向）")
                      .arg(fromEdit->text(), toEdit->text()));
        replan();
    });

    connect(delEdgeBtn, &QPushButton::clicked, this, [&] {
        const std::string from = fromEdit->text().toStdString();
        const std::string to = toEdit->text().toStdString();
        const bool okForward = config_.graph.removeEdge(from, to);
        const bool okBackward = config_.graph.removeEdge(to, from);
        appendLog(okForward || okBackward
                      ? QStringLiteral("手工删除边 %1 <-> %2").arg(fromEdit->text(), toEdit->text())
                      : QStringLiteral("删除边失败：不存在 %1 -> %2")
                            .arg(fromEdit->text(), toEdit->text()));
        replan();
    });

    connect(delNodeBtn, &QPushButton::clicked, this, [&] {
        const std::string id = nodeEdit->text().toStdString();
        if (!config_.graph.removeNode(id)) {
            appendLog(QStringLiteral("删除节点失败：%1 不存在")
                          .arg(QString::fromStdString(id)));
            return;
        }
        appendLog(QStringLiteral("手工删除节点 %1（含其全部出入边）")
                      .arg(QString::fromStdString(id)));
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
        route += QStringLiteral("停靠 %1 站，已送达 %2 站\n")
                     .arg(state_.servedStops + static_cast<int>(remainingStops))
                     .arg(state_.servedStops);
        // 拆成三段写明，避免"共 K 趟"被误读成"整趟配送总共几趟"：
        //   已完成 = 真的跑完了几趟（事实）
        //   当前第 N 趟 = 绝对趟号（跨重规划连续，不重置）
        //   全部完成约需 T 趟 = 已完成 + 本次规划还需（**含当前趟**）。
        // 三段之间必须满足：当前 == 已完成 + 1，且 全部完成 >= 当前。
        // 早先第三段写的是 plan_.trips.size()（"本计划还要跑几趟"），
        // 它跟已完成无关，会出现「已完成 4 趟 · 当前第 5 趟 · 本计划共 1 趟」这种自相矛盾的显示。
        const int totalTrips =
            state_.completedTrips + static_cast<int>(plan_.trips.size());
        route += QStringLiteral("趟次：已完成 %1 趟 · 当前第 %2 趟 · 全部完成约需 %3 趟\n")
                     .arg(state_.completedTrips)
                     .arg(state_.tripNumber)
                     .arg(totalTrips);
        if (plan_.trips.size() > 1) {
            route += QStringLiteral("\n各趟：\n");
            for (std::size_t i = 0; i < plan_.trips.size(); ++i) {
                const logistics::Trip& trip = plan_.trips[i];
                // 直达分批下每趟都从仓库出发、回仓库，"起点 → 终点"两个端点
                // 全是 W01，毫无信息量。改为显示**本趟服务了哪些配送点**。
                QString range;
                if (!trip.stops.empty()) {
                    range = QStringLiteral("%1 个配送点  %2 → %3")
                                .arg(trip.stops.size())
                                .arg(QString::fromStdString(trip.stops.front().nodeId))
                                .arg(QString::fromStdString(trip.stops.back().nodeId));
                } else {
                    range = QStringLiteral("无配送点（%1 → %2）")
                                .arg(QString::fromStdString(trip.nodes.empty()
                                                                ? std::string()
                                                                : trip.nodes.front()))
                                .arg(QString::fromStdString(trip.endNodeId));
                }
                route += QStringLiteral("  第 %1 趟：%2  %3km  装载 %4kg\n")
                             .arg(state_.tripNumber + static_cast<int>(i))
                             .arg(range)
                             .arg(trip.totalDistanceKm, 0, 'f', 1)
                             .arg(trip.loadKg, 0, 'f', 0);
            }
        }
        route += QStringLiteral("\n完整序列：\n");
        for (std::size_t i = 0; i < plan_.nodes.size(); ++i) {
            route += QString::fromStdString(plan_.nodes[i]);
            route += (i + 1 == plan_.nodes.size()) ? QString() : QStringLiteral(" -> ");
        }
    } else {
        route = QStringLiteral("不可行：%1").arg(QString::fromStdString(plan_.reason));
    }
    routeInfo_->setPlainText(route);

    // 车辆信息
    //
    // 这里必须区分两个量（此前混为一谈，导致"载重上限 200kg / 当前载重 740kg"）：
    //   · 本趟装载：当前这趟车上实际装的货量，不变量是 **不得超过载重上限**
    //   · 剩余待送：全部未送达订单的货量之和，可以远超载重上限
    //     —— 多趟模式下车辆正是一趟趟把这些货送完的
    if (!config_.vehicles.empty()) {
        const logistics::Vehicle& v = config_.vehicles.front();

        // 本趟尚未驶离出发点时，装载量以计划为准（还没装）；一经出发即冻结，
        // 之后的重规划不得改动它——那已经是被执行了的事实。
        // 本趟装载与趟号都来自车辆状态，重规划不改写
        const double tripLoad = state_.tripLoadKg;
        const std::size_t tripNo = static_cast<std::size_t>(state_.tripNumber);

        double remainingDemand = 0.0;
        for (const Order& o : remainingOrders()) {
            remainingDemand += o.demandKg;
        }

        // ---- 固定信息：整场模拟都不变的车辆固有参数 ----
        vehicleFixedInfo_->setText(
            QStringLiteral("ID：%1\n起始仓库：%2\n载重上限：%3 kg\n发车时刻：%4\n"
                           "（以上为配置给定的固有参数，模拟过程中不会变化）")
                .arg(QString::fromStdString(v.id))
                .arg(QString::fromStdString(v.startNodeId))
                .arg(v.capacityKg, 0, 'f', 0)
                .arg(minutesToClock(v.departTimeMin)));

        // ---- 运行状态：随模拟推进而变的量 ----
        vehicleStateInfo_->setText(
            QStringLiteral("软件内时间：%1\n"
                           "车上载重：%2 kg\n"
                           "本趟出发装载：%3 kg（第 %4 趟）\n"
                           "剩余待送：%5 kg")
                .arg(minutesToClock(currentTimeMin()))
                // 「车上载重」是**物理事实**：车上此刻实际载着的订单货量。
                .arg(currentLoadKg(), 0, 'f', 0)
                .arg(tripLoad, 0, 'f', 0)
                .arg(tripNo)
                .arg(remainingDemand, 0, 'f', 0));
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

    // 停靠明细（顺序与 plan_.stops 一致，即服务顺序）
    // 先算出每个停靠点属于第几趟：trips 里的 stops 顺序拼接即为 plan_.stops
    std::vector<int> stopTrip(plan_.stops.size(), 0);
    {
        std::size_t si = 0;
        for (std::size_t t = 0; t < plan_.trips.size(); ++t) {
            for (std::size_t k = 0;
                 k < plan_.trips[t].stops.size() && si < stopTrip.size(); ++k) {
                stopTrip[si++] = static_cast<int>(t + 1);
            }
        }
    }
    // 只列**尚未走过**的停靠点：推进一站与重规划应当有一致的效果，
    // 都表现为"已送达的那一行从表里消失"。此前推进不改计划、表也就不动，
    // 与重规划后表被重建的行为不一致，容易被误判为卡住。
    const std::size_t fromStop =
        (stopCursor_ < plan_.stops.size()) ? stopCursor_ : plan_.stops.size();
    const int shown = static_cast<int>(plan_.stops.size() - fromStop);
    stopTable_->setRowCount(shown);
    for (int i = 0; i < shown; ++i) {
        const std::size_t idx = fromStop + static_cast<std::size_t>(i);
        const Stop& s = plan_.stops[idx];
        // 趟号 = 已完成趟数 + 当前计划里的趟号。
        // 这样重规划后趟号接着往下编，而不是又变回「第 1 趟」。
        // 绝对趟号 = 车辆当前趟号 + 计划内相对趟号 - 1。
        // tripNumber 是车辆状态，重规划不改；相对趟号由计划给出。二者配合即得绝对编号。
        stopTable_->setItem(i, 0, new QTableWidgetItem(
            QStringLiteral("第 %1").arg(state_.tripNumber + stopTrip[idx] - 1)));
        stopTable_->setItem(i, 1, new QTableWidgetItem(QString::fromStdString(s.nodeId)));
        stopTable_->setItem(i, 2, new QTableWidgetItem(minutesToClock(s.rawArrivalMin)));
        stopTable_->setItem(i, 3, new QTableWidgetItem(QString::number(s.waitMin)));
        stopTable_->setItem(i, 4, new QTableWidgetItem(minutesToClock(s.arrivalMin)));
        stopTable_->setItem(i, 5, new QTableWidgetItem(QString::number(s.remainingLoadKg, 'f', 0)));
    }

    // 中转站 / 集散：子网络标识 + 下属配送点数
    {
        struct TransitRow {
            QString name;
            int     sub = 0;
            int     serves = 0;
        };
        std::vector<TransitRow> rows;
        for (const logistics::Node& n : config_.graph.nodes()) {
            if (n.type != logistics::NodeType::Transit) {
                continue;
            }
            TransitRow row;
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
        transitTable_->setRowCount(static_cast<int>(rows.size()));
        for (int i = 0; i < static_cast<int>(rows.size()); ++i) {
            transitTable_->setItem(i, 0, new QTableWidgetItem(rows[i].name));
            transitTable_->setItem(i, 1,
                                   new QTableWidgetItem(QString::number(rows[i].sub)));
            transitTable_->setItem(i, 2,
                                   new QTableWidgetItem(QString::number(rows[i].serves)));
        }
    }

    // 超时订单：区分**已经发生**的超时（事实，跨重规划保留）与**剩余计划预计**的超时。
    //
    // 只列 plan_.stops 里 late 的那些是不够的：重规划后剩余计划里可能一个都不 late，
    // 于是"已经超时过"的订单会从表里凭空消失，看起来像超时被抹掉了。
    int lateRows = 0;
    const auto putLate = [&](const QString& orderText, const std::string& nodeId,
                             const QString& status, int arrivalMin, int penaltyMin) {
        lateTable_->setRowCount(lateRows + 1);
        lateTable_->setItem(lateRows, 0, new QTableWidgetItem(orderText));
        lateTable_->setItem(lateRows, 1, new QTableWidgetItem(QString::fromStdString(nodeId)));
        lateTable_->setItem(lateRows, 2, new QTableWidgetItem(status));
        lateTable_->setItem(lateRows, 3, new QTableWidgetItem(minutesToClock(arrivalMin)));
        lateTable_->setItem(lateRows, 4, new QTableWidgetItem(windowTextAt(nodeId)));
        lateTable_->setItem(lateRows, 5, new QTableWidgetItem(QString::number(penaltyMin)));
        ++lateRows;
    };
    for (const VehicleState::LateStop& ls : state_.deliveredLate) {
        putLate(QString::fromStdString(ls.orderId), ls.nodeId,
                QStringLiteral("已超时"), ls.arrivalMin, ls.penaltyMin);
    }
    if (plan_.status == logistics::PlanStatus::Ok) {
        for (const Stop& s : plan_.stops) {
            if (!s.late || stopCursor_ >= plan_.stops.size()) {
                continue;
            }
            // 已经走过的不再重复列（上面已经作为"已超时"列出）
            bool alreadyDelivered = false;
            for (std::size_t k = 0; k < stopCursor_ && k < plan_.stops.size(); ++k) {
                if (plan_.stops[k].nodeId == s.nodeId) {
                    alreadyDelivered = true;
                    break;
                }
            }
            if (alreadyDelivered) {
                continue;
            }
            putLate(orderIdsAt(s.nodeId), s.nodeId, QStringLiteral("预计超时"),
                    s.arrivalMin, s.penaltyMin);
        }
    }
    lateTable_->setRowCount(lateRows);
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

QString MainWindow::vehiclePanelSummary() const {
    // 两个页签都给出去：--ui-probe 要能同时核对固定信息与运行状态
    // （只给当前页签的话，切页状态会影响断言结果）。
    QString out;
    if (vehicleFixedInfo_ != nullptr) {
        out += vehicleFixedInfo_->text();
    }
    if (vehicleStateInfo_ != nullptr) {
        if (!out.isEmpty()) {
            out += QLatin1Char('\n');
        }
        out += vehicleStateInfo_->text();
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

std::size_t MainWindow::currentTripIndex() const {
    if (plan_.nodeTripIndex.empty() || nodeIndex_ >= plan_.nodeTripIndex.size()) {
        return 0;
    }
    return plan_.nodeTripIndex[nodeIndex_];
}

int MainWindow::completedTripOffset() const {
    return state_.tripNumber - 1;
}



int MainWindow::tripCount() const {
    return static_cast<int>(plan_.trips.size());
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
        const std::string id = insertUrgentOrderAt(A, B, 5.0);
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

    // ---- 2. 连续插两张紧急单：都不丢，且都在计划里 ----
    {
        const std::size_t before = config_.orders.size();
        const std::string id1 = insertUrgentOrderAction();
        const std::string id2 = insertUrgentOrderAction();
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
    }

    // ---- 3. 非法输入被拒（A==B / 超载），且不得并入订单集合 ----
    {
        const std::size_t before = config_.orders.size();
        const std::string bad1 = insertUrgentOrderAt("D01", "D01", 5.0);
        const double cap = config_.vehicles.empty() ? 0.0 : config_.vehicles.front().capacityKg;
        const std::string bad2 = insertUrgentOrderAt("D01", "D02", cap + 100.0);
        expect(bad1.empty() && bad2.empty(),
               QStringLiteral("A==B 与超载必须被拒绝"));
        expect(config_.orders.size() == before,
               QStringLiteral("被拒绝的紧急单不得并入订单集合"));
    }

    // ---- 4. 新客户：必须带来订单并被纳入配送 ----
    {
        const std::size_t before = config_.orders.size();
        const std::string node = addRandomCustomerAction();
        expect(!node.empty(), QStringLiteral("模拟新客户应成功"));
        expect(config_.orders.size() == before + 1,
               QStringLiteral("新客户必须带来一张订单"));
        bool inPlan = false;
        for (const std::string& n : plan_.nodes) {
            if (n == node) {
                inPlan = true;
                break;
            }
        }
        expect(inPlan, QStringLiteral("新客户节点必须出现在路线里"));
    }

    // ---- 5. 重规划绝不改写车辆物理状态 ----
    {
        for (int i = 0; i < 4; ++i) {
            onAdvanceStop();
        }
        const VehicleState snapshot = state_;
        onSimulateTraffic();
        onReplan();
        expect(state_.tripNumber == snapshot.tripNumber,
               QStringLiteral("重规划不得改变车辆所在趟次"));
        expect(state_.completedTrips == snapshot.completedTrips,
               QStringLiteral("重规划不得改变已完成趟数"));
        expect(state_.servedStops == snapshot.servedStops,
               QStringLiteral("重规划不得改变已送达计数"));
        expect(state_.incurredPenaltyMin == snapshot.incurredPenaltyMin,
               QStringLiteral("重规划不得改变已发生的 penalty"));
    }

    // ---- 6. 装不下 → 整单降载 + 完成后取回（issue #4，轻量可见检查）----
    // 放在最后：前五段的断言依赖"全新状态 + 恰有一张紧急单"的前提，而本段要
    // 长距离推进（会跨事件刻、抽到插单/路况事件），放在前面会打乱它们的前提。
    // 插入一张顶到载重上限的紧急单，强制走"降载 → A 取 → B 送 → 取回"分支。
    if (cust.size() >= 2 && !config_.vehicles.empty()) {
        const double cap = config_.vehicles.front().capacityKg;
        const double freeKg = cap - state_.loadKg;
        const bool shouldOffload = !state_.onboard.empty() && std::floor(cap) > freeKg + 1e-9;
        // 选 A/B：避开车上已有货的节点——在途货是"整条普通货"的释放候选，
        // 规划器不会卸掉 A/B 本身的货，撞上它们会让降载凑不齐而判不可行。
        std::string A;
        std::string B;
        for (std::size_t i = 0; i < cust.size() && A.empty(); ++i) {
            bool iTaken = false;
            for (const logistics::OnboardItem& it : state_.onboard) {
                if (it.nodeId == cust[i]) {
                    iTaken = true;
                    break;
                }
            }
            if (iTaken) {
                continue;
            }
            for (std::size_t j = 0; j < cust.size(); ++j) {
                if (j == i) {
                    continue;
                }
                bool jTaken = false;
                for (const logistics::OnboardItem& it : state_.onboard) {
                    if (it.nodeId == cust[j]) {
                        jTaken = true;
                        break;
                    }
                }
                if (jTaken) {
                    continue;
                }
                A = cust[i];
                B = cust[j];
                break;
            }
        }
        if (A.empty()) {
            A = cust.front();
            B = cust[cust.size() > 1 ? 1 : 0];
        }
        const std::string id = insertUrgentOrderAt(A, B, std::floor(cap));
        expect(!id.empty(), QStringLiteral("接近满载的紧急取送单应插入成功"));
        if (!id.empty()) {
            // 计划层（确定性）：降载趟的暂存守恒与载重不变量
            bool hasOffloadTrip = false;
            bool conserve = true;
            bool loadOk = true;
            for (const logistics::Trip& t : plan_.trips) {
                if (t.offloads.empty()) {
                    continue;
                }
                hasOffloadTrip = true;
                double sum = 0.0;
                for (const logistics::OffloadOp& op : t.offloads) {
                    sum += op.kg;
                }
                if (std::fabs(sum - t.reclaimKg) > 1e-6) {
                    conserve = false;
                }
                if (t.loadKg > cap + 1e-6) {
                    loadOk = false;
                }
            }
            if (shouldOffload) {
                expect(hasOffloadTrip, QStringLiteral("装不下时应出现降载趟"));
            }
            if (hasOffloadTrip) {
                expect(conserve, QStringLiteral("暂存守恒：Σoffloads.kg == reclaimKg"));
                expect(loadOk, QStringLiteral("降载趟载重不超过上限"));
            }
            // 推进到结束：全程载重不超限
            bool traceOk = true;
            for (int i = 0; i < 600 && !atRouteEnd(); ++i) {
                onAdvanceStop();
                if (state_.loadKg > cap + 1e-6) {
                    traceOk = false;
                }
            }
            expect(traceOk, QStringLiteral("全程车辆载重不超过上限"));
            // 若降载真的执行了（未被中途重规划改道），日志里应能看到「降载」
            if (!state_.appliedOffloads.empty()) {
                expect(runLogText().contains(QStringLiteral("降载")),
                       QStringLiteral("执行降载后日志应出现「降载」"));
            }
        }
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
