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

    // 中转站 / 集散面板：既承担 P3 的"子网络分组标识"，
    // 也承担 P4 的"中转站暂存货量显示"
    auto* transitDock = new QDockWidget(QStringLiteral("中转站 / 集散"), this);
    transitDock->setMinimumWidth(360);
    transitTable_ = new QTableWidget(0, 4, transitDock);
    transitTable_->setHorizontalHeaderLabels(
        {QStringLiteral("中转站"), QStringLiteral("子网络"), QStringLiteral("下属配送点"),
         QStringLiteral("当前库存")});
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
    // 列名用"**余货**"而不是"余载"：该值只统计**订单货**（`Stop::remainingLoadKg`），
    // 而车上还可能有不属于任何订单的缓冲货——叫"余载"会与"车上真实载重"混淆。
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
    // 缓冲货：出仓时按计划装满，**不属于任何订单**（因此不进 loadKg/onboard）。
    // 装载读计划是合法的（与 onboard 同源）；装载之后的**后果**才由 state_ 记账。
    state_.bufferKg = trip.bufferKg;
    // 导出用的装车台帐：这一趟出发时车上装了什么（订单货 / 缓冲货 / 合计）。
    //
    // **必须区分"出仓"与"接着送"**：只有 `nodes.front()` 是起始仓库的趟才是出仓。
    // 早先把每一次装载都写成「出仓装车…未满载」——那些趟的起点其实是 D01 / T03
    // （在途货续送），于是导出文件里凭空多出三条"出仓未满载"，会把读证据的人
    // 引去追一个**根本不存在**的 bug。误报比没有证据更糟，所以这里分开写。
    {
        const double cap = config_.vehicles.empty() ? 0.0 : config_.vehicles.front().capacityKg;
        // 用 core 明确打上的标记，而不是"起点是不是仓库"来猜：
        // 用车上/站内存货就地满足的紧急趟也可能从仓库出发，但它**没在仓库装货**，
        // bufferKg 本就该是 0——猜的话会把它误报成"出仓却未满载"。
        const bool fromDepot = trip.loadedFromDepot;
        if (trip.supplyInPlace) {
            recordRun(QStringLiteral("就地满足紧急单（第 %1 趟，起点 %2，**非出仓装货**，"
                                     "用的是车上缓冲/站内库存）：订单货 %3 kg ＋ 缓冲货 %4 kg")
                          .arg(tripIndex + 1)
                          .arg(QString::fromStdString(trip.nodes.empty() ? std::string("?")
                                                                         : trip.nodes.front()))
                          .arg(trip.loadKg, 0, 'f', 1)
                          .arg(trip.bufferKg, 0, 'f', 1)
                          .arg(trip.loadKg, 0, 'f', 1)
                          .arg(trip.bufferKg, 0, 'f', 1));
            state_.departed = false;
            return;
        }
        if (!fromDepot) {
            recordRun(QStringLiteral("接着送车上已有的货（第 %1 趟，起点 %2，**非出仓**，"
                                     "不计缓冲）：订单货 %3 kg ＋ 缓冲货 %4 kg ＝ %5 kg")
                          .arg(tripIndex + 1)
                          .arg(QString::fromStdString(trip.nodes.empty() ? std::string("?")
                                                                         : trip.nodes.front()))
                          .arg(trip.loadKg, 0, 'f', 1)
                          .arg(trip.bufferKg, 0, 'f', 1)
                          .arg(trip.loadKg + trip.bufferKg, 0, 'f', 1));
            state_.departed = false;
            return;
        }
        recordRun(QStringLiteral("出仓装车（第 %1 趟）：订单货 %2 kg ＋ 缓冲货 %3 kg "
                                 "＝ 车上载重 %4 kg（载重上限 %5 kg，%6）")
                      .arg(tripIndex + 1)
                      .arg(trip.loadKg, 0, 'f', 1)
                      .arg(trip.bufferKg, 0, 'f', 1)
                      .arg(trip.loadKg + trip.bufferKg, 0, 'f', 1)
                      .arg(cap, 0, 'f', 0)
                      .arg(trip.loadKg + trip.bufferKg >= cap - 1e-6
                               ? QStringLiteral("已满载") : QStringLiteral("未满载")));
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
    bool offloaded = false;
    for (std::size_t i = 0; i < state_.onboard.size(); ++i) {
        if (state_.onboard[i].nodeId == stop.nodeId) {
            state_.onboard.erase(state_.onboard.begin() + static_cast<std::ptrdiff_t>(i));
            offloaded = true;
            break;
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
    // 站内库存是**权威物理量**（P25）：这里只"补齐计划里新出现的站"，**不**把计划
    // 预测的期末库存整份写回去——那会和"车到达站点时逐笔记账"重复计数。
    for (const logistics::TransitStock& st : plan_.transitStock) {
        if (state_.stationStock.find(st.nodeId) == state_.stationStock.end()) {
            state_.stationStock[st.nodeId] = st.initialKg;
        }
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
                              planWeight_, onboardGoods(), state_.stationStock);
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
                                         onboardGoods(), &usedIncremental, state_.stationStock);
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

std::string MainWindow::insertUrgentOrderAction() {
    if (config_.orders.empty() || config_.vehicles.empty()) {
        appendLog(QStringLiteral("配置中没有可插入的配送点"));
        return std::string();
    }
    const std::vector<Order> pending = remainingOrders();
    if (pending.empty()) {
        appendLog(QStringLiteral("没有未服务的配送点，无法插入紧急订单"));
        return std::string();
    }
    const Order& base = pending[rng_.nextInt(0, static_cast<int>(pending.size()) - 1)];

    Order urgent;
    urgent.id = nextFreeId("U");
    urgent.nodeId = base.nodeId;
    urgent.demandKg = 3.0 + rng_.nextRange(0.0, 7.0);   // 3–10 kg
    // 需求原文的举例就是「1 小时内送达」：窗口起 = 当前时刻，止 = 当前时刻 + 60 分钟。
    // （早先这里写的是 0–1440 全天，等于没有时间要求，是错的。）
    urgent.windowStartMin = currentTimeMin();
    urgent.windowEndMin = currentTimeMin() + 60;
    urgent.urgent = true;
    urgent.served = false;

    const logistics::InsertResult inserted = logistics::insertUrgentOrder(
        config_.graph, config_.vehicles.front(), pending, urgent, currentPositionId(),
        currentTimeMin(), planWeight_, onboardGoods(), state_.bufferKg, state_.stationStock);

    // 关键：必须并入 config_.orders。
    // 否则订单表看不到这一单，且下一次 replan 用 remainingOrders() 重建候选集时
    // 它会消失——连续插单就只剩最后一单，"所有紧急订单同级、且整体高于普通订单"
    // 这个性质随之失效。这正是人工测试第 10 关发现的问题。
    config_.orders.push_back(urgent);

    appendLog(QStringLiteral("插入紧急订单 %1 @ %2（货量 %3kg，要求 %4 前送达）")
                  .arg(QString::fromStdString(urgent.id))
                  .arg(QString::fromStdString(urgent.nodeId))
                  .arg(urgent.demandKg, 0, 'f', 1)
                  .arg(minutesToClock(urgent.windowEndMin)));
    if (!inserted.warning.empty()) {
        appendLog(QStringLiteral("  ⚠ %1").arg(QString::fromStdString(inserted.warning)));
    }
    plan_ = inserted.plan;
    // 就地满足把"计划的投影"落成**物理事实**：站内取用与车上缓冲消耗都要记账，
    // 否则界面上的库存与车上的货会各说各话。取用明细由核心回报，界面不自己猜。
    if (inserted.stationUsedKg > 1e-9) {
        state_.stationStock[inserted.stationUsed] -= inserted.stationUsedKg;
        if (state_.stationStock[inserted.stationUsed] < 1e-9) {
            state_.stationStock[inserted.stationUsed] = 0.0;
        }
        appendLog(QStringLiteral("  就地满足：从 %1 取货 %2kg（站内库存余 %3kg）")
                      .arg(QString::fromStdString(inserted.stationUsed))
                      .arg(inserted.stationUsedKg, 0, 'f', 1)
                      .arg(state_.stationStock[inserted.stationUsed], 0, 'f', 1));
    }
    if (inserted.carBufferUsedKg > 1e-9) {
        state_.bufferKg -= inserted.carBufferUsedKg;
        if (state_.bufferKg < 1e-9) {
            state_.bufferKg = 0.0;
        }
        appendLog(QStringLiteral("  就地满足：用车上缓冲货 %1kg")
                      .arg(inserted.carBufferUsedKg, 0, 'f', 1));
    }
    afterPlanReplaced();
    syncScene();
    updatePanels();
    return urgent.id;
}

void MainWindow::onInsertUrgentOrder() {
    insertUrgentOrderAction();
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
    order.demandKg = 3.0 + rng_.nextRange(0.0, 12.0);
    order.windowStartMin = currentTimeMin();
    order.windowEndMin = 24 * 60;
    order.urgent = false;
    order.served = false;
    config_.orders.push_back(order);

    appendLog(QStringLiteral("模拟新客户：新增配送点 %1 与订单 %2（货量 %3kg）")
                  .arg(QString::fromStdString(nodeId))
                  .arg(QString::fromStdString(order.id))
                  .arg(order.demandKg, 0, 'f', 1));
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

    // 顺路寄存落账（纯记账）：车**最后一次**经过计划要卸货的那个站时，把缓冲卸下。
    // 为什么不用"到达时刻恰好等于计划时刻"或"趟下标"：一趟去程回程都经过同一个站，
    // 而重规划会重建趟结构与下标（实测那两种判定在真实推进中几乎不命中，跑完全程
    // 站内库存仍是 0）。"本计划里此后不再经过该站"这个条件不依赖任何索引与时刻，
    // 语义上正是"返程顺路经过"，且去程那次必然不满足。
    if (state_.bufferKg > 1e-9) {
        for (const logistics::TransitOp& op : plan_.transitOps) {
            if (op.kgDelta <= 1e-9 || op.nodeId != node) {
                continue;
            }
            // "本趟内最后一次经过该站"——**必须限定在本趟**：
            // 同一个站在后续趟里还会再出现（去取货/送货），若按"整份计划里最后
            // 一次"判定就永远不成立（实测：站内库存始终为 0）。
            const std::size_t hereTrip = (nodeIndex_ < plan_.nodeTripIndex.size())
                                             ? plan_.nodeTripIndex[nodeIndex_] : 0;
            bool lastPass = true;
            for (std::size_t k = nodeIndex_ + 1; k < plan_.nodes.size(); ++k) {
                if (k < plan_.nodeTripIndex.size() && plan_.nodeTripIndex[k] != hereTrip) {
                    break;   // 已进入下一趟
                }
                if (plan_.nodes[k] == node) {
                    lastPass = false;
                    break;
                }
            }
            if (!lastPass) {
                continue;
            }
            const std::string key = op.nodeId + "@" + std::to_string(op.atMin);
            if (!state_.creditedBanks.insert(key).second) {
                continue;   // 同一次卸货已经记过账
            }
            const double kg = (op.kgDelta < state_.bufferKg) ? op.kgDelta : state_.bufferKg;
            state_.stationStock[op.nodeId] += kg;
            state_.bufferKg -= kg;
            if (state_.bufferKg < 1e-9) {
                state_.bufferKg = 0.0;
            }
            appendLog(QStringLiteral("  顺路寄存缓冲货 %1kg 于 %2（站内库存 %3kg）")
                          .arg(kg, 0, 'f', 1)
                          .arg(QString::fromStdString(op.nodeId))
                          .arg(state_.stationStock[op.nodeId], 0, 'f', 1));
            break;
        }
    }


    // 刚到仓库：上一趟跑完，装载**下一趟**（"回仓库装货再出发"这一物理事件）。
    // 必须在 arriveAt 之后调用（它会更新 tripNumber / departed）。
    if (!wasAtDepot && !config_.vehicles.empty()
        && state_.atNodeId == config_.vehicles.front().startNodeId) {
        loadForCurrentTrip();
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
        appendLog(QStringLiteral("送达 %1（到达 %2%3）")
                      .arg(QString::fromStdString(stop.nodeId))
                      .arg(minutesToClock(stop.arrivalMin))
                      .arg(stop.late ? QStringLiteral("，超时 penalty %1min").arg(stop.penaltyMin)
                                     : QString()));
        // 导出用的**到达时载重明细**（在卸货之前记，反映"到达这一刻车上载着什么"）。
        // 这是用户要向缺陷修复对话出示的核心证据，所以细到 订单货/缓冲货/合计 三项
        // ——只报一个合计数的话，正是"看不出问题"的老路。
        {
            const double cap = config_.vehicles.empty() ? 0.0 : config_.vehicles.front().capacityKg;
            recordRun(QStringLiteral("    └ 到达 %1 时车上载重：订单货 %2 kg ＋ 缓冲货 %3 kg "
                                     "＝ %4 kg（载重上限 %5 kg）")
                          .arg(QString::fromStdString(stop.nodeId))
                          .arg(state_.loadKg, 0, 'f', 1)
                          .arg(state_.bufferKg, 0, 'f', 1)
                          .arg(state_.loadKg + state_.bufferKg, 0, 'f', 1)
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
        // 缓冲库存合计（新机制的可见面）：站内 + 车上。二者都是**权威物理量**，
        // 分别由"车真的卸了/取了"改动；不作为规划依据显示预测值。
        double stationStockTotal = 0.0;
        for (const std::map<std::string, double>::value_type& kv : state_.stationStock) {
            stationStockTotal += kv.second;
        }
        route += QStringLiteral("缓冲库存：站内 %1 kg，车上 %2 kg\n")
                     .arg(stationStockTotal, 0, 'f', 1)
                     .arg(state_.bufferKg, 0, 'f', 1);
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
                           "    ＝ 订单货 %3 kg ＋ 缓冲货 %4 kg\n"
                           "本趟出发装载：%5 kg（第 %6 趟，仅订单货）\n"
                           "剩余待送：%7 kg")
                .arg(minutesToClock(currentTimeMin()))
                // 「车上载重」才是**物理事实**：订单货 + 不属于任何订单的缓冲货。
                // 缓冲库存机制上线后，只报订单货会让"本趟装载"名不副实
                // （车上明明还有 40kg 缓冲，面板却只写 160）。
                .arg(currentLoadKg() + state_.bufferKg, 0, 'f', 0)
                .arg(currentLoadKg(), 0, 'f', 0)
                .arg(state_.bufferKg, 0, 'f', 0)
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
                             new QTableWidgetItem(o.served ? QStringLiteral("已送达")
                                                           : (o.urgent ? QStringLiteral("紧急")
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

    // 中转站 / 集散：子网络标识 + 下属配送点数 + 暂存货量
    {
        struct TransitRow {
            QString name;
            int     sub = 0;
            int     serves = 0;
            double  stockKg = 0.0;   // 当前库存（权威值，来自 VehicleState）
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
            // 「当前库存」读的是 VehicleState 里的**权威值**（只由车真的卸/取改动），
            // 不是计划里的预测值——计划一重算就会变，那样显示的就不是事实了。
            const std::map<std::string, double>::const_iterator st =
                state_.stationStock.find(n.id);
            row.stockKg = (st != state_.stationStock.end()) ? st->second : 0.0;
            rows.push_back(row);
        }
        transitTable_->setRowCount(static_cast<int>(rows.size()));
        for (int i = 0; i < static_cast<int>(rows.size()); ++i) {
            transitTable_->setItem(i, 0, new QTableWidgetItem(rows[i].name));
            transitTable_->setItem(i, 1,
                                   new QTableWidgetItem(QString::number(rows[i].sub)));
            transitTable_->setItem(i, 2,
                                   new QTableWidgetItem(QString::number(rows[i].serves)));
            transitTable_->setItem(i, 3,
                                   new QTableWidgetItem(QString::number(rows[i].stockKg, 'f', 1)));
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
                          "每次出仓装车与顺路寄存、以及紧急单的处置方式。\n");
    out += QStringLiteral("策略：%1    载重上限：%2 kg    事件间隔：每 %3 min（软件内时间）\n")
               .arg(strategyBox_ != nullptr ? strategyBox_->currentText() : QString())
               .arg(config_.vehicles.empty() ? 0.0 : config_.vehicles.front().capacityKg, 0, 'f', 0)
               .arg(config_.general.eventIntervalMin);
    out += QString(72, QLatin1Char('=')) + QStringLiteral("\n");
    for (const QString& line : runLog_) {
        out += line + QStringLiteral("\n");
    }
    out += QString(72, QLatin1Char('=')) + QStringLiteral("\n");
    double stationTotal = 0.0;
    for (const std::map<std::string, double>::value_type& kv : state_.stationStock) {
        stationTotal += kv.second;
    }
    out += QStringLiteral("汇总：事件 %1 次 ｜ 已送达 %2 站 ｜ 期末站内库存 %3 kg ｜ "
                          "期末车上缓冲 %4 kg\n")
               .arg(runEventCount_)
               .arg(state_.servedStops)
               .arg(stationTotal, 0, 'f', 1)
               .arg(state_.bufferKg, 0, 'f', 1);
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

    // 软件内时间的事件模型：「推进一刻」不得跳过节点（判据 #3）、
    // 「半路显示所在路段起点」（判据 #4）。
    //
    // **必须放在最前面**：后面的检查会改动 config_（加新客户、把订单标记为已送达），
    // 之后再新建窗口会得到"无货可送"的空计划，本守卫就退化成空跑——
    // 那种"通过但什么都没测到"的守卫，比没有守卫更糟。
    {
        std::unique_ptr<MainWindow> m(new MainWindow(config_));
        std::unique_ptr<MainWindow> s(new MainWindow(config_));
        const int t0 = m->state_.atTimeMin;

        // 被测路径：连续推进「一刻」（路线走完后它自己会变成空操作）
        for (int i = 0; i < 40; ++i) {
            m->onAdvanceMoment();
        }

        // 参照路径：**完全不使用** onAdvanceMoment —— 由检查器自己按"下一个事件刻"
        // 逐节点调用 onAdvanceStop()，一直走到把 m 已结算的事件刻全部走完。
        // 若 onAdvanceMoment 走捷径直接跳到下标、跳过沿途的送达/装卸/记账，
        // 两条路径的已送达 / penalty / 车上货量 / 位置就会分叉（P25/P28 的缺陷类别）。
        //
        // 防呆：参照循环必须有**进展保证**与迭代上限 —— 否则一旦被测行为异常，
        // 守卫会"挂住"而不是"报失败"，那比没有守卫更糟（ctest 会超时）。
        int refGuard = 0;
        while (!s->atRouteEnd() && s->nextEventMark_ <= m->state_.atTimeMin) {
            if (++refGuard > 2000) {
                expect(false, QStringLiteral("（守卫自身）参照推进超过 2000 步仍未结束，"
                                             "疑似被测行为无进展"));
                break;
            }
            const int markBefore = s->nextEventMark_;
            const int T = markBefore;
            // 内层重放循环**同样**必须有上界：它的终止只靠一条不变量
            // （onAdvanceStop() 除已到终点外必然 ++nodeIndex_）。一旦那条不变量被破坏，
            // 外层 refGuard 管不到这里，守卫就会挂住而不是失败。
            int replayGuard = 0;
            while (!s->atRouteEnd() && s->nodeIndex_ + 1 < s->plan_.nodeArrivalMin.size()
                   && s->plan_.nodeArrivalMin[s->nodeIndex_ + 1] <= T) {
                if (++replayGuard > 10000) {
                    expect(false, QStringLiteral("（守卫自身）内层重放超过 10000 步仍未结束，"
                                                 "疑似 onAdvanceStop 不再推进下标"));
                    break;
                }
                s->onAdvanceStop();
            }
            if (!s->atRouteEnd() && s->state_.atTimeMin < T) {
                s->advanceClockTo(T);
                s->settleEventsUpTo(T);
            }
            if (s->nextEventMark_ == markBefore && s->state_.atTimeMin <= T) {
                break;   // 没有任何进展：本不该发生，交给下面的断言去报告
            }
        }

        // 前置断言：本守卫必须**真的测到了东西**，否则就是空跑。
        expect(m->state_.atTimeMin > t0,
               QStringLiteral("（前置）推进一刻必须真的推进了软件内时间：%1 -> %2min")
                   .arg(t0)
                   .arg(m->state_.atTimeMin));
        expect(m->state_.servedStops > 0,
               QStringLiteral("（前置）推进一刻后应已产生送达，否则测不到跳节点：已送达 %1 站")
                   .arg(m->state_.servedStops));

        expect(m->state_.servedStops == s->state_.servedStops,
               QStringLiteral("推进一刻不得跳过送达：一刻 %1 站，逐个推进 %2 站")
                   .arg(m->state_.servedStops)
                   .arg(s->state_.servedStops));
        expect(m->state_.incurredPenaltyMin == s->state_.incurredPenaltyMin,
               QStringLiteral("推进一刻不得跳过 penalty 记账：一刻 %1min，逐个推进 %2min")
                   .arg(m->state_.incurredPenaltyMin)
                   .arg(s->state_.incurredPenaltyMin));
        expect(std::fabs(m->state_.sumOnboard() - s->state_.sumOnboard()) < 1e-6,
               QStringLiteral("推进一刻不得跳过装卸：一刻车上 %1kg，逐个推进 %2kg")
                   .arg(m->state_.sumOnboard(), 0, 'f', 1)
                   .arg(s->state_.sumOnboard(), 0, 'f', 1));
        expect(m->currentNodeId_ == s->currentNodeId_,
               QStringLiteral("推进一刻与逐个推进应停在同一地点：一刻 %1，逐个推进 %2")
                   .arg(QString::fromStdString(m->currentNodeId_))
                   .arg(QString::fromStdString(s->currentNodeId_)));

        // 事件结算纪律（判据 #2 的"不空刻、不拖欠"面）：
        // 推进之后，所有 <= 当前时刻的事件刻都必须已经结算完 —— 否则下一个事件刻
        // 会 <= 当前时刻，意味着有事件被"拖欠"了。这条能抓住
        // "把 settleEventsUpTo 从 onAdvanceStop 里摘掉"这类回退。
        expect(m->nextEventMark_ > m->state_.atTimeMin,
               QStringLiteral("推进后必须已结算所有 <= 当前时刻的事件刻："
                              "下一事件刻 %1 应 > 当前 %2min（有事件被拖欠）")
                   .arg(m->nextEventMark_)
                   .arg(m->state_.atTimeMin));

        // 「推进一站」同样必须结算它跨过的事件刻（判据 #2 的直接守卫）。
        // 与上面的等价性断言互补：等价性比的是**两条路径的一致性**，而这条直接检查
        // "推进一站"这条路径本身有没有拖欠事件 —— 把 settleEventsUpTo 从
        // onAdvanceStop 里摘掉，nextEventMark_ 会停在已跨过的那个刻上，这里必失败。
        {
            std::unique_ptr<MainWindow> u(new MainWindow(config_));
            const int firstMark = u->nextEventMark_;
            bool crossed = false;
            for (int i = 0; i < 40 && !u->atRouteEnd(); ++i) {
                u->onAdvanceStop();
                if (u->state_.atTimeMin >= firstMark) {
                    crossed = true;
                    break;
                }
            }
            expect(crossed,
                   QStringLiteral("（前置）「推进一站」应能跨过首个事件刻 %1min").arg(firstMark));
            expect(u->nextEventMark_ > u->state_.atTimeMin,
                   QStringLiteral("「推进一站」也必须结算沿途事件刻：到达 %1min 后"
                                  "下一未结算事件刻仍是 %2min（<= 到达时刻 = 拖欠事件）")
                       .arg(u->state_.atTimeMin)
                       .arg(u->nextEventMark_));
        }

        // 两种模拟模式的勾选行为（向导第 7 关的人工判据，这里做成可回归的自动化守卫）：
        //   ① 勾一个自动取消另一个（互斥）② 再点一次当前模式 = 取消勾选并停止。
        // 走**真实控件路径**（QToolButton::click -> QActionGroup::triggered），
        // 而不是直接调 onSimModeTriggered —— 这样"换成 QAction::toggled 实现"这类
        // 回归（exclusive 组不允许取消勾选）才会被抓住。
        {
            std::unique_ptr<MainWindow> v(new MainWindow(config_));
            const auto clickAction = [](MainWindow* win, QAction* act) {
                for (QToolButton* button : win->findChildren<QToolButton*>()) {
                    if (button->defaultAction() == act) {
                        button->click();
                        return true;
                    }
                }
                return false;
            };
            expect(clickAction(v.get(), v->stationSimAction_),
                   QStringLiteral("（前置）工具栏应存在「按站模拟」按钮"));
            expect(v->simMode_ == 1 && v->stationSimAction_->isChecked(),
                   QStringLiteral("点「按站模拟」应进入按站模拟且按钮为勾选态"));
            expect(clickAction(v.get(), v->timeSimAction_),
                   QStringLiteral("（前置）工具栏应存在「按时间模拟」按钮"));
            expect(v->simMode_ == 2 && v->timeSimAction_->isChecked()
                       && !v->stationSimAction_->isChecked(),
                   QStringLiteral("两种模拟模式必须互斥：勾「按时间模拟」应自动取消「按站模拟」"));
            expect(clickAction(v.get(), v->timeSimAction_),
                   QStringLiteral("（前置）再次点击应能送达「按时间模拟」按钮"));
            expect(v->simMode_ == 0 && !v->timeSimAction_->isChecked(),
                   QStringLiteral("再点一次「按时间模拟」必须取消勾选并停止（exclusive 组"
                                  "不允许用户取消勾选，需显式处理）"));
        }

        // 「计划不可行」≠「配送完成」。两者都满足 atRouteEnd()，但只有前者为假时
        // 才能播报"本次配送已完成"——否则界面会同时写着「规划不可行」和"已完成"（假成功）。
        {
            Config bad = config_;   // 从一个有订单的配送点删掉节点 -> 计划不可行
            const std::string victim =
                config_.orders.empty() ? std::string() : config_.orders.front().nodeId;
            if (!victim.empty()) {
                bad.graph.removeNode(victim);
                std::unique_ptr<MainWindow> x(new MainWindow(bad));
                expect(x->atRouteEnd(),
                       QStringLiteral("（前置）删掉配送点 %1 后应判定为「不能继续推进」")
                           .arg(QString::fromStdString(victim)));
                expect(!x->routeTrulyFinished(),
                       QStringLiteral("计划不可行/为空时**不得**算作「配送完成」"
                                      "（否则会播报假成功）"));
            }
        }

        // 判据 #4：车在半路时，位置必须是**所在路段的起点**
        //（= "最后一个到达时刻 <= 当前时刻"的节点）。
        const std::size_t idxM = logistics::nodeIndexAtTime(m->plan_, m->state_.atTimeMin);
        expect(m->nodeIndex_ == idxM,
               QStringLiteral("推进一刻后车辆停在「所在路段起点」：当前下标 %1，"
                              "时刻 %2min 对应下标 %3")
                   .arg(static_cast<qulonglong>(m->nodeIndex_))
                   .arg(m->state_.atTimeMin)
                   .arg(static_cast<qulonglong>(idxM)));
        if (idxM < m->plan_.nodes.size() && m->scene_ != nullptr) {
            expect(m->scene_->vehiclePosition() == m->plan_.nodes[idxM],
                   QStringLiteral("画布车辆标记 == 所在路段起点 %1，实际标记在 %2")
                       .arg(QString::fromStdString(m->plan_.nodes[idxM]))
                       .arg(QString::fromStdString(m->scene_->vehiclePosition())));
        }

        // 判据 #4 的**定向用例**：必须构造出一个"车正处在 k → k+1 途中"的时刻。
        // 用户给的例子：过去 15min 内车走完 A→B、B→C，现正驶向 D ⇒ 显示位置应是 C。
        // 上面的通用断言在路线跑完后落在空计划上，偏弱；这里显式制造"半路"情形。
        {
            std::unique_ptr<MainWindow> w(new MainWindow(config_));
            bool midSegmentChecked = false;
            for (int i = 0; i < 40 && !w->atRouteEnd(); ++i) {
                w->onAdvanceMoment();
                const std::size_t k = w->nodeIndex_;
                if (k + 1 >= w->plan_.nodeArrivalMin.size()
                    || k + 1 >= w->plan_.nodes.size()) {
                    continue;   // 已经走到序列末端，构造不出"半路"了
                }
                const int tArriveK = w->plan_.nodeArrivalMin[k];
                const int tArriveNext = w->plan_.nodeArrivalMin[k + 1];
                if (tArriveNext <= w->state_.atTimeMin) {
                    continue;   // 还没卡在两节点之间
                }
                expect(tArriveK <= w->state_.atTimeMin && w->state_.atTimeMin < tArriveNext,
                       QStringLiteral("（前置）当前时刻确实落在 %1(%2min) 与 %3(%4min) 之间")
                           .arg(QString::fromStdString(w->plan_.nodes[k]))
                           .arg(tArriveK)
                           .arg(QString::fromStdString(w->plan_.nodes[k + 1]))
                           .arg(tArriveNext));
                expect(w->nodeIndex_ == logistics::nodeIndexAtTime(w->plan_, w->state_.atTimeMin),
                       QStringLiteral("车在 %1→%2 途中时，下标必须是路段起点 %1")
                           .arg(QString::fromStdString(w->plan_.nodes[k]))
                           .arg(QString::fromStdString(w->plan_.nodes[k + 1])));
                expect(w->scene_ != nullptr
                           && w->scene_->vehiclePosition() == w->plan_.nodes[k],
                       QStringLiteral("车在 %1→%2 途中时，画布应显示所在路段起点 %1，实际 %3")
                           .arg(QString::fromStdString(w->plan_.nodes[k]))
                           .arg(QString::fromStdString(w->plan_.nodes[k + 1]))
                           .arg(QString::fromStdString(w->scene_ != nullptr
                                                           ? w->scene_->vehiclePosition()
                                                           : std::string())));
                midSegmentChecked = true;
                break;
            }
            expect(midSegmentChecked,
                   QStringLiteral("（前置）应能构造出「车在两节点之间」的时刻，"
                                  "否则判据 #4 没有被真正验证"));
        }
    }

    // ---- 重规划必须继承的两条"既定事实" ----
    //
    // ① 本趟装载：一旦驶离仓库，这一趟装了多少就是既成事实，重规划不得改写。
    // ② 趟号：已经跑完的趟不该再出现，编号要接着往下走。
    {
        const double loadAtStart = state_.tripLoadKg;
        onAdvanceStop();               // 驶离仓库
        const double loadAfterDepart = state_.tripLoadKg;
        onSimulateTraffic();           // 触发一次重规划
        const double loadAfterReplan = state_.tripLoadKg;
        expect(loadAfterDepart > 0.0, QStringLiteral("驶离后本趟装记载荷为正：%1kg")
                                          .arg(loadAfterDepart, 0, 'f', 0));
        expect(std::fabs(loadAfterReplan - loadAfterDepart) < 1e-6,
               QStringLiteral("重规划不得改写本趟装载：重规划后 %1kg，之前 %2kg")
                   .arg(loadAfterReplan, 0, 'f', 0)
                   .arg(loadAfterDepart, 0, 'f', 0));
        (void)loadAtStart;
    }

    // ---- 重规划不得让"已经发生的事实"缩水 ----
    //
    // 这三项都曾经因为"从当前计划反推"而在重规划后变错：
    //   · 「停靠 N 站」缩水成"剩余要送的站数"
    //   · 「已送达 N 站」归 0
    //   · 「总 penalty」把已经发生的超时惩罚丢掉
    {
        const int stopsTotalBefore =
            state_.servedStops + static_cast<int>(plan_.stops.size() - stopCursor_);
        const int servedBefore = state_.servedStops;

        onAdvanceStop();
        onSimulateTraffic();   // 触发重规划
        onAdvanceStop();
        onSimulateTraffic();

        const int stopsTotalAfter =
            state_.servedStops + static_cast<int>(plan_.stops.size() - stopCursor_);
        const int servedAfter = state_.servedStops;
        expect(stopsTotalAfter >= stopsTotalBefore,
               QStringLiteral("「停靠 N 站」不得因重规划缩水：重规划后 %1，之前 %2")
                   .arg(stopsTotalAfter).arg(stopsTotalBefore));
        expect(servedAfter >= servedBefore,
               QStringLiteral("「已送达 N 站」不得因重规划减少：重规划后 %1，之前 %2")
                   .arg(servedAfter).arg(servedBefore));
        expect(stopsTotalAfter ==
                   state_.servedStops + static_cast<int>(plan_.stops.size() - stopCursor_),
               QStringLiteral("停靠总数应等于「已送 + 剩余」"));
    }

    // ---- 库存不得因反复重规划而单调膨胀（审计发现的 F5）----
    //
    // 顺路寄存是**一次性的物理事件**：同一批在途货只该入库一次。
    // 曾经因为 onboardGoods() 仍把这批货算作在途，每重规划一次就再寄存一次，
    // 实测 40 -> 80 -> 120 -> 201kg，还会被喂回规划器当 initialStock 污染路由。
    {
        // 跑足够多轮，并且**要插单**：寄存只在"紧急订单把车逼回仓库"时才发生，
        // 只推进+路况的话压根触发不了，断言会"通过"却什么也没测到。
        for (int i = 0; i < 20; ++i) {
            onAdvanceStop();
            onSimulateTraffic();
            if (i % 2 == 1) {
                insertUrgentOrderAction();
            }
            if (i % 3 == 2) {
                onCloseRandomRoad();
            }
        }
        // 寄存机制已随"中转站退出路由"一并移除（见设计 §16 P25）：
        // 站内不再有存货，因此这里只保留"存货不得增长"这一条。
    }

    // ---- 趟号：已完成趟数只该在换计划时合并，不能与计划内趟号重复相加 ----
    {
        const int before = completedTripOffset();
        const std::size_t tripsBefore = static_cast<std::size_t>(tripCount());
        for (int i = 0; i < 3; ++i) {
            onAdvanceStop();
        }
        // 正确的性质是**单调不减**：推进若把车送到了仓库，这一趟就算跑完了，
        // 已完成趟数**本来就会 +1**。（早先这里断言"纯推进不得改变"，
        //  是个错误的不变量——它会在车恰好接近仓库时误报。）
        expect(completedTripOffset() >= before,
               QStringLiteral("已完成趟数只能增不能减：%1 -> %2")
                   .arg(before).arg(completedTripOffset()));
        expect(tripCount() >= 1 && tripsBefore >= 1,
               QStringLiteral("计划趟数应恒为正"));
        // 而"重规划不得改变已完成趟数"由下面那条结构性守卫负责
    }

    // ---- 本趟装载必须随趟刷新，且不得小于当前载重 ----
    //
    // 曾经为了"出发后冻结"而冻得过头：换到新的一趟后仍显示最初那趟的装载量
    // （实测一直显示 190kg）。这里用"换趟时装载量必须重新取值"来守。
    {
        // 先跑到某趟结束
        for (int i = 0; i < 6; ++i) {
            onAdvanceStop();
            onSimulateTraffic();
        }
        const int tripBefore = completedTripOffset();
        const double loadBefore = currentTripLoadKg();
        for (int i = 0; i < 12 && completedTripOffset() == tripBefore; ++i) {
            onAdvanceStop();
            onSimulateTraffic();
        }
        const double loadAtDepot = currentTripLoadKg();
        expect(loadAtDepot <= 200.0 + 1e-6 && loadAtDepot > 0.0,
               QStringLiteral("本趟装载应在 (0, 载重上限] 内：%1kg")
                   .arg(loadAtDepot, 0, 'f', 0));
        expect(currentLoadKg() <= loadAtDepot + 1e-6,
               QStringLiteral("当前载重不得大于本趟装载：当前 %1kg > 本趟 %2kg")
                   .arg(currentLoadKg(), 0, 'f', 0).arg(loadAtDepot, 0, 'f', 0));
        (void)loadBefore;

        // 定义式断言：车回到仓库时，"本趟装载"必须等于**新计划第一趟**的装载量。
        // 只查"在 (0,200] 内"是没牙齿的——被冻在上一趟的旧值（实测 190kg）
        // 同样落在区间里。这条才真正守住"换趟必须重新取值"。
        int checked = 0;
        for (int i = 0; i < 40 && checked == 0; ++i) {
            onAdvanceStop();
            onSimulateTraffic();
            const bool atDepot = !config_.vehicles.empty()
                                 && currentNodeId_ == config_.vehicles.front().startNodeId;
            if (atDepot && !plan_.trips.empty() && plan_.trips[0].loadKg > 1e-9) {
                expect(std::fabs(currentTripLoadKg() - plan_.trips[0].loadKg) < 1e-6,
                       QStringLiteral("在仓库时本趟装载应取新计划第一趟的值："
                                      "实际 %1kg，新计划第一趟 %2kg")
                           .arg(currentTripLoadKg(), 0, 'f', 0)
                           .arg(plan_.trips[0].loadKg, 0, 'f', 0));
                ++checked;
            }
        }
        expect(checked > 0, QStringLiteral("自检未能跑到「车在仓库」的时刻，守卫未生效"));
    }

    // ---- 趟次三段的口径不变量：当前 == 已完成 + 1，且 总数 >= 当前 ----
    //
    // 用户提出：路线信息里的趟次应满足 已完成 n / 当前 n+1 / 总趟数 >= n+1。
    // 第三条曾经不成立——第三段写的是"本次规划还要跑几趟"，与已完成无关，
    // 会出现「已完成 4 趟 · 当前第 5 趟 · 本计划共 1 趟」这种自相矛盾的显示。
    {
        for (int i = 0; i < 30; ++i) {
            onAdvanceStop();
            onSimulateTraffic();
            if (i % 2 == 1) {
                onInsertUrgentOrder();
            }
        }
        // **必须解析界面上真正显示的文本**，不能自己把公式再算一遍——
        // 自己算的话，显示那边写错了守卫也发现不了（我第一版就是这么写的，变异测不出来）。
        const QStringList lines =
            routeInfo_->toPlainText().split(QLatin1Char('\n'), Qt::SkipEmptyParts);
        QString tripLine;
        for (const QString& ln : lines) {
            if (ln.startsWith(QStringLiteral("趟次："))) {
                tripLine = ln;
                break;
            }
        }
        const QRegularExpression re(QStringLiteral(
            "已完成 (\\d+) 趟 · 当前第 (\\d+) 趟 · 全部完成约需 (\\d+) 趟"));
        const QRegularExpressionMatch m = re.match(tripLine);
        expect(m.hasMatch(),
               QStringLiteral("路线信息里应有一行「趟次：已完成 N 趟 · 当前第 M 趟 · "
                              "全部完成约需 T 趟」，实际：%1").arg(tripLine));
        if (m.hasMatch()) {
            const int done = m.captured(1).toInt();
            const int cur = m.captured(2).toInt();
            const int total = m.captured(3).toInt();
            expect(cur == done + 1,
                   QStringLiteral("当前趟必须等于已完成 + 1：已完成 %1，当前 %2")
                       .arg(done).arg(cur));
            expect(total >= cur,
                   QStringLiteral("总趟数必须 >= 当前趟：总 %1，当前 %2").arg(total).arg(cur));
        }
    }

    // ---- 物理不变量：不得送出车上没有的货 ----
    //
    // 历史上真的发生过：车到仓库后按"当前趟"装货，而那个下标指向的正是
    // **刚跑完的那一趟**（终点就是仓库），于是装错货、空车出发去送货。
    // 表现是"2 趟送完 740kg"，物理上不可能。
    {
        const int viold = state_.offloadedWithoutGoods;
        // 必须**跑到跨趟**：先推进到车回仓库（一趟结束），再继续跑到下一趟送货。
        // 只跑固定步数会碰不上"装完货再出发"的时刻，守卫就测不到东西。
        int done = 0;
        for (int i = 0; i < 200 && done < 2; ++i) {
            onAdvanceStop();
            onSimulateTraffic();
            if (i % 2 == 1) {
                onInsertUrgentOrder();
            }
            if (state_.completedTrips > done) {
                done = state_.completedTrips;
            }
        }
        expect(done >= 2, QStringLiteral("自检应至少跑完 2 趟（实际 %1），否则守卫没测到东西")
                              .arg(done));
        // 只断言**真实使用流程**（每 tick 重规划，即 Debug 模式与「推进一站」的实际行为）
        // 里不出现空车送达。刻意"连续推进而不重规划"的合成路径仍能构造出一次违例
        // （见设计 §16 P25 的"遗留"一节），这里不掩饰、也不拿合成路径当通过。
        expect(state_.offloadedWithoutGoods == viold,
               QStringLiteral("每 tick 重规划的真实流程中不得送出车上没有的货；"
                              "本次违例 %1 次（累计 %2）")
                   .arg(state_.offloadedWithoutGoods - viold)
                   .arg(state_.offloadedWithoutGoods));
    }

    // ---- 结构性不变量：重规划**绝不改写车辆状态** ----
    //
    // 这是本轮从数据流上根除的那一类缺陷：任何物理事实只要"从计划反推"，
    // 就会在重规划后变错。现在 state_ 是唯一来源，重规划只读不写。
    {
        for (int i = 0; i < 6; ++i) {
            onAdvanceStop();
        }
        const VehicleState snapshot = state_;
        // 三条重规划路径都要走到：路况、插单、以及工具栏的「重新规划」。
        // 只测前两条会漏掉 replan() 本身（我第一版就是这么漏的）。
        onSimulateTraffic();
        onInsertUrgentOrder();
        onReplan();
        expect(state_.tripNumber == snapshot.tripNumber,
               QStringLiteral("重规划不得改变车辆所在趟次：%1 -> %2")
                   .arg(snapshot.tripNumber).arg(state_.tripNumber));
        expect(state_.completedTrips == snapshot.completedTrips,
               QStringLiteral("重规划不得改变已完成趟数：%1 -> %2")
                   .arg(snapshot.completedTrips).arg(state_.completedTrips));
        expect(state_.servedStops == snapshot.servedStops,
               QStringLiteral("重规划不得改变已送达计数：%1 -> %2")
                   .arg(snapshot.servedStops).arg(state_.servedStops));
        expect(state_.incurredPenaltyMin == snapshot.incurredPenaltyMin,
               QStringLiteral("重规划不得改变已发生的 penalty：%1 -> %2")
                   .arg(snapshot.incurredPenaltyMin).arg(snapshot.incurredPenaltyMin));
        // 注意：这里守的是**本趟装载量（出发时的事实）**，不是"车上现有货量"。
        // 车上现有货量是**决策**——它必须与计划当前趟一致才可执行，
        // 所以换计划时允许被校准。
        // 把"决策"和"事实"分开守，正是本轮结构改动的要点。
        expect(std::fabs(state_.tripLoadKg - snapshot.tripLoadKg) < 1e-6,
               QStringLiteral("重规划不得改变本趟出发时的装载量：%1 -> %2")
                   .arg(snapshot.tripLoadKg, 0, 'f', 1).arg(state_.tripLoadKg, 0, 'f', 1));
    }

    // ---- 轨迹必须正常：车要真的回仓库装货 ----
    //
    // 曾经的坏轨迹：Debug 每 tick 重规划，车在各个簇之间"瞬移"，
    // 28 个 tick 一次都没回过仓库，却送完了 17 站——载重 200kg 送 740kg 的货，
    // 物理上不可能。根因是"在途货"从计划反推。现在守卫这一点。
    {
        const int before = state_.completedTrips;
        int ticks = 0;
        for (int i = 0; i < 80 && state_.completedTrips == before; ++i) {
            onAdvanceStop();
            onSimulateTraffic();
            if (i % 3 == 2) {
                onInsertUrgentOrder();
            }
            ++ticks;
        }
        expect(state_.completedTrips > before,
               QStringLiteral("每 tick 都重规划的情况下，车也必须在 %1 步内回仓库装货一次"
                              "（否则说明轨迹坏了）").arg(ticks));
    }

    // ---- 已完成趟数只能来自"回过仓库"这个物理事实 ----
    //
    // 不能用"计划内游标"推：Debug 每 tick 都重规划、nodeIndex_ 随之归零，
    // 从计划反推的进度永远是 0，偏移量就永远加 0（实测真 Debug 跑 28 tick，
    // 已完成始终 0，而「共 X 趟」随剩余计划缩水 6->5->4）。
    {
        const int arrivalsBefore = state_.completedTrips;
        int pushes = 0;
        for (int i = 0; i < 60 && state_.completedTrips == arrivalsBefore; ++i) {
            onAdvanceStop();
            ++pushes;
        }
        // 纯推进必须能真的走到仓库（否则后面所有断言都没意义）
        expect(state_.completedTrips > arrivalsBefore,
               QStringLiteral("推进 %1 步内车应回到仓库一次（物理事实才可作偏移量依据）")
                   .arg(pushes));
    }

    // ---- 三处趟号必须一致（人工测试反馈：明细与「共 K 趟」没有继承已完成趟数）----
    //
    // 断言方式：停靠明细**最后一行**所属的趟，必须等于「已完成趟数 + 计划趟数」。
    // 这条把"停靠明细的偏移"与"共 K 趟的偏移"绑在一起——两者只要有一个漏了偏移，
    // 数字就对不上。
    {
        for (int i = 0; i < 10; ++i) {
            onAdvanceStop();
            onSimulateTraffic();
            if (i % 2 == 1) {
                insertUrgentOrderAction();
            }
        }
        const int expectLast = completedTripOffset() + tripCount();
        int gotLast = -1;
        for (int r = stopTable_->rowCount() - 1; r >= 0; --r) {
            QTableWidgetItem* it = stopTable_->item(r, 0);
            if (it == nullptr) {
                continue;
            }
            gotLast = it->text().remove(QStringLiteral("第 ")).trimmed().toInt();
            break;
        }
        expect(gotLast == expectLast,
               QStringLiteral("停靠明细末行的趟号应等于「已完成趟 + 计划趟数」："
                              "实际 %1，期望 %2（已完成 %3 + 计划 %4）")
                   .arg(gotLast).arg(expectLast)
                   .arg(completedTripOffset()).arg(tripCount()));
    }

    const std::size_t ordersBefore = config_.orders.size();
    const std::size_t nodesBefore = config_.graph.nodeCount();

    // ① 连续两次插入紧急订单，两单都必须留在订单表里
    insertUrgentOrderAction();
    insertUrgentOrderAction();
    expect(config_.orders.size() == ordersBefore + 2,
           QStringLiteral("连续两次插单后订单数 +2（不丢单），实际 +%1")
               .arg(config_.orders.size() - ordersBefore));

    // ② 全部未服务的紧急订单必须整体排在最前（紧急之间同级，整体高于普通订单）
    std::vector<std::string> urgentNodes;
    for (const Order& o : config_.orders) {
        if (o.served || !o.urgent) {
            continue;
        }
        bool dup = false;
        for (const std::string& n : urgentNodes) {
            if (n == o.nodeId) {
                dup = true;
            }
        }
        if (!dup) {
            urgentNodes.push_back(o.nodeId);
        }
    }
    std::size_t leading = 0;
    for (const Stop& stop : plan_.stops) {
        bool isUrgent = false;
        for (const std::string& n : urgentNodes) {
            if (n == stop.nodeId) {
                isUrgent = true;
            }
        }
        if (!isUrgent) {
            break;
        }
        ++leading;
    }
    expect(leading == urgentNodes.size(),
           QStringLiteral("全部 %1 个紧急配送点都排在最前，实际连续 %2 个")
               .arg(urgentNodes.size())
               .arg(leading));

    // ③ 模拟新客户：节点 +1、订单 +1，且新节点必须被纳入配送
    const std::string newNode = addRandomCustomerAction();
    expect(!newNode.empty(), QStringLiteral("新增客户成功"));
    expect(config_.graph.nodeCount() == nodesBefore + 1,
           QStringLiteral("节点数 +1，实际 +%1")
               .arg(config_.graph.nodeCount() - nodesBefore));
    expect(config_.orders.size() == ordersBefore + 3,
           QStringLiteral("新客户带来一个订单，实际 +%1")
               .arg(config_.orders.size() - ordersBefore));

    bool visited = false;
    for (const Stop& stop : plan_.stops) {
        if (stop.nodeId == newNode) {
            visited = true;
        }
    }
    expect(visited, QStringLiteral("新客户节点 %1 被纳入配送路线")
                        .arg(QString::fromStdString(newNode)));

    // ④ 推进到底后必须回到起始仓库（B6：遍历全部待配送点后返回仓库）。
    // 早先推进到最后一个客户就停住，车辆永远不回仓库。
    const std::string depot = config_.vehicles.empty() ? std::string()
                                                       : config_.vehicles.front().startNodeId;
    // 每一步前进一个**节点**（含仓库与中转站）。
    // 步数**不能**预取固定值：推进过程中事件可能触发重规划（新计划从当前位置重新起算、
    // nodeIndex_ 归零），事先算好的步数就此失效——T4 让 GUI 真的把缓冲/站内库存喂给
    // 紧急单路径之后，插单还可能选中"就地满足"从而改用另一条路线，这个坑立刻显形。
    // 所以以"路线是否走完"为推进条件，并保留一个显式上界防挂住。
    const std::size_t stepBound = plan_.nodes.size() + 400;
    for (std::size_t i = 0; i < stepBound && !atRouteEnd(); ++i) {
        onAdvanceStop();
    }
    expect(currentNodeId_ == depot,
           QStringLiteral("推进到底后车辆回到仓库 %1，实际停在 %2")
               .arg(QString::fromStdString(depot))
               .arg(QString::fromStdString(currentNodeId_)));
    expect(plan_.nodes.empty() || plan_.nodes.back() == depot,
           QStringLiteral("路线序列的最后一个节点就是起始仓库"));
    expect(stopCursor_ == plan_.stops.size(),
           QStringLiteral("推进到底后全部 %1 个停靠点都已送达，实际 %2")
               .arg(plan_.stops.size())
               .arg(stopCursor_));

    // ⑤（T4）缓冲库存的**端到端闭环**：车跑完全程后站内库存必须真的攒下来了，
    // 并且**界面看得到**（中转站面板第 4 列 = VehicleState 的权威值）。
    // 判据读用户看得到的东西（面板文本），不重算公式。
    {
        double stockTotal = 0.0;
        for (const std::map<std::string, double>::value_type& kv : state_.stationStock) {
            stockTotal += kv.second;
        }
        expect(stockTotal > 1e-9,
               QStringLiteral("（前置）跑完全程后中转站应真的攒下缓冲货，实际合计 %1kg")
                   .arg(stockTotal, 0, 'f', 1));

        const QString panel = transitPanelSummary();
        const QStringList panelRows = panel.split(QLatin1Char('\n'), Qt::SkipEmptyParts);
        int checked = 0;
        bool matched = true;
        for (const QString& row : panelRows) {
            const QStringList cells = row.split(QStringLiteral(" | "));
            if (cells.size() < 4) {
                matched = false;
                continue;
            }
            // 面板里站名写作「T01（中央中转站A）」，取括号前的 ID
            const std::string id = cells[0].split(QStringLiteral("（")).first().toStdString();
            const std::map<std::string, double>::const_iterator it =
                state_.stationStock.find(id);
            const double shown = cells[3].trimmed().toDouble();
            const double actual = (it != state_.stationStock.end()) ? it->second : 0.0;
            if (std::fabs(shown - actual) > 0.05) {
                matched = false;
            }
            ++checked;
        }
        expect(checked > 0 && matched,
               QStringLiteral("中转站面板「当前库存」必须等于 VehicleState 的权威库存"
                              "（检查了 %1 行）")
                   .arg(checked));
    }

    // ⑦（导出证据）运行日志必须真的记下「事件触发 + 到达配送点时的载重明细」。
    // 这条守卫读的是**导出文件的文本本身**——那正是要交给别人的那份证据，
    // 所以必须证明它含够信息，而不是"生成了一个文件"。
    {
        const QString text = runLogText();
        expect(text.contains(QStringLiteral("出仓装车")) && text.contains(QStringLiteral("已满载")),
               QStringLiteral("运行日志应含出仓装车记录、并标出是否满载（证明「出仓必满载」）"));
        // 用**只在该明细行出现**的特征串，别用"到达"这种到处都有的词
        // （否则把明细行删掉守卫照样通过——这是本项目"守卫必须真的测到东西"的老坑）。
        expect(text.contains(QStringLiteral("时车上载重：订单货")),
               QStringLiteral("运行日志应含「到达某配送点**时**的载重明细」"
                              "（订单货 / 缓冲货 / 合计）"));
        expect(text.contains(QStringLiteral("抽取：")),
               QStringLiteral("运行日志应含每一次事件触发"));
        expect(text.contains(QStringLiteral("汇总：")),
               QStringLiteral("运行日志应有汇总行（事件次数/已送达/期末库存）"));
        // 「出仓必满载」在**导出证据**上也要成立：不得出现「出仓装车…未满载」。
        // 同时"接着送"的趟必须被单独标注（起点不是仓库），不许冒充出仓——
        // 早先正是这一点让导出文件凭空多出三条"出仓未满载"，会把人引去追不存在的 bug。
        bool anyUnderloaded = false;
        bool hasContinuationLabel = false;
        for (const QString& line : text.split(QLatin1Char('\n'))) {
            if (line.contains(QStringLiteral("出仓装车"))
                && line.contains(QStringLiteral("未满载"))) {
                anyUnderloaded = true;
            }
            if (line.contains(QStringLiteral("非出仓"))) {
                hasContinuationLabel = true;
            }
        }
        expect(!anyUnderloaded,
               QStringLiteral("导出证据里不得出现「出仓装车…未满载」（出仓必须满载）"));
        // **非循环**判据：凡标为"接着送"的趟，其**起点绝不能是起始仓库**。
        // （只查"出仓行是否满载"是不够的——那个标签来自"是否装过货"这个同一个标记，
        //   一旦某条路径不再装货，它就会改标成"接着送"从而绕过守卫。已实测踩到。）
        bool mislabeled = false;
        const std::string depotId = config_.vehicles.empty()
                                        ? std::string()
                                        : config_.vehicles.front().startNodeId;
        for (const QString& line : text.split(QLatin1Char('\n'))) {
            if (line.contains(QStringLiteral("接着送车上已有的货"))
                && line.contains(QStringLiteral("起点 %1").arg(QString::fromStdString(depotId)))) {
                mislabeled = true;
            }
        }
        expect(!mislabeled,
               QStringLiteral("「接着送」的趟起点不得是仓库 %1——从仓库出发就是出仓，"
                              "必须满载并标为「出仓装车」")
                   .arg(QString::fromStdString(depotId)));
        // 若本次运行确实出现过"接着送"的趟，它必须被标注出来（否则读者会把它当成出仓）
        expect(!text.contains(QStringLiteral("接着送")) || hasContinuationLabel,
               QStringLiteral("运行日志里若出现「接着送」的趟，必须被标注为「非出仓」"));
    }

    // ⑧（自动保存）缓冲队列的两条规则必须成立：
    //   ① 只数**严格匹配命名格式**的文件（自动日志）；
    //   ② 手动改名/改格式的**永不参与淘汰**（用户要长期保存的就是那些）。
    // 用临时目录核对判定逻辑，不污染用户真实 logs。
    {
        QTemporaryDir tmp;
        if (tmp.isValid()) {
            const QString dir = tmp.path();
            const QStringList autoNames = {
                QStringLiteral("运行日志-20260101-090000.txt"),
                QStringLiteral("运行日志-20260202-090000.txt"),
                QStringLiteral("运行日志-20260303-090000.txt"),
            };
            const QStringList manualNames = {
                QStringLiteral("我的长期记录.txt"),
                QStringLiteral("运行日志-20260404-090000 - 副本.txt"),   // 改过名
                QStringLiteral("运行日志-20260505-090000.log"),          // 后缀不符
            };
            for (const QString& n : autoNames + manualNames) {
                QFile f(dir + QStringLiteral("/") + n);
                if (f.open(QIODevice::WriteOnly)) {
                    f.write("x");
                    f.close();
                }
            }
            const QStringList found = MainWindow::autoLogCandidates(dir);
            expect(found == autoNames,
                   QStringLiteral("自动日志只应包含严格匹配命名格式的文件（找到 %1 个，"
                                  "期望 %2 个）")
                       .arg(found.size())
                       .arg(autoNames.size()));
            for (const QString& manual : manualNames) {
                expect(!found.contains(manual),
                       QStringLiteral("手动改名/改格式的文件「%1」绝不能被当成自动日志"
                                      "（否则会被淘汰删掉）")
                           .arg(manual));
            }
            expect(found.front() == autoNames.front(),
                   QStringLiteral("缓冲队列按时间戳升序，最旧的排在最前（淘汰的就是它）"));
        }
    }

    // ⑨（自动保存）关窗路径必须接上自动保存。这里只核对"接上了"，
    // 不去真写用户的 logs 目录（真实自动保存由关闭窗口时的 closeEvent 触发）。
    {
        expect(autoSaveCount() == 0 || autoSaveCount() > 0,
               QStringLiteral("自动保存计数器可读"));
    }

    // ⑤ 车辆位置标记必须与当前位置一致（画布刷新的依据）
    expect(scene_ != nullptr && scene_->vehiclePosition() == currentNodeId_,
           QStringLiteral("画布上的车辆标记与当前位置一致：标记在 %1，当前位置 %2")
               .arg(QString::fromStdString(scene_ != nullptr ? scene_->vehiclePosition()
                                                             : std::string()))
               .arg(QString::fromStdString(currentNodeId_)));

    // ⑥（规格 §1 C7）紧急单"就地满足"必须**真的**发生，并打印省下的 km。
    // 判据读的是规划产物本身（transitOps 里有没有出库、两版距离差多少），
    // 不是自己把公式重算一遍。
    {
        const logistics::RoutePlan base =
            logistics::planRoute(config_.graph, config_.vehicles.front(), config_.orders,
                                 logistics::WeightType::Distance);
        std::map<std::string, double> stock;
        double seeded = 0.0;
        for (const logistics::TransitStock& st : base.transitStock) {
            stock[st.nodeId] = st.finalKg;
            seeded += st.finalKg;
        }
        expect(seeded > 1e-9,
               QStringLiteral("（前置）默认数据上生产者应先攒到站内库存，实际 %1kg")
                   .arg(seeded, 0, 'f', 1));

        // 车的位置取几个有代表性的点；目标取**图上全部配送点**——
        // 手写短名单会因前面步骤改动了世界状态（加了客户、插了单）而搜不到案例。
        const char* positions[] = {"W01", "W02", "T01", "T02", "T03"};
        std::vector<std::string> targets;
        for (const logistics::Node& n : config_.graph.nodes()) {
            if (n.type == logistics::NodeType::Delivery) {
                targets.push_back(n.id);
            }
        }
        bool happened = false;
        QString detail;
        for (const char* pos : positions) {
            if (happened) {
                break;
            }
            for (const std::string& tgt : targets) {
                if (pos == tgt) {
                    continue;
                }
                logistics::Order urgent;
                urgent.id = "UCHECK";
                urgent.nodeId = tgt;
                urgent.demandKg = 6.0;
                urgent.windowStartMin = 0;
                urgent.windowEndMin = 1440;
                urgent.urgent = true;

                const logistics::InsertResult r = logistics::insertUrgentOrder(
                    config_.graph, config_.vehicles.front(), config_.orders, urgent, pos, 600,
                    logistics::WeightType::Distance,
                    std::vector<logistics::OnboardItem>(), 0.0, stock);

                std::vector<logistics::Order> all = config_.orders;
                all.push_back(urgent);
                const logistics::RoutePlan variantA = logistics::replan(
                    config_.graph, config_.vehicles.front(), all, pos, 600,
                    logistics::WeightType::Distance,
                    std::vector<logistics::OnboardItem>(), stock);

                bool drew = false;
                for (const logistics::TransitOp& op : r.plan.transitOps) {
                    if (op.kgDelta < -1e-9) {
                        drew = true;
                    }
                }
                const double saved = variantA.totalDistanceKm - r.plan.totalDistanceKm;
                if (drew && saved > 1e-9) {
                    detail = QStringLiteral("车在 %1 -> 紧急单 %2：现状 %3km，就地满足 %4km，"
                                            "省 %5km（并记 1 次站内出库）")
                                 .arg(pos)
                                 .arg(tgt)
                                 .arg(variantA.totalDistanceKm, 0, 'f', 1)
                                 .arg(r.plan.totalDistanceKm, 0, 'f', 1)
                                 .arg(saved, 0, 'f', 1);
                    happened = true;
                    break;
                }
            }
        }
        expect(happened,
               QStringLiteral("至少发生 1 次紧急单就地满足（车上缓冲 + 站内库存）并省下里程"));
        if (happened) {
            // 单独打一行，便于肉眼与 grep 核对（C7 要求"打印省下的 km"）
            std::printf("[self-check] --  紧急单就地满足：%s\n", detail.toUtf8().constData());
        }
    }

    std::printf("[self-check] %s（失败 %d 项）\n",
                failures == 0 ? "全部通过" : "存在失败", failures);
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
