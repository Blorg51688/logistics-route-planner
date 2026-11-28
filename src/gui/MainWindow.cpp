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
#include <QSpinBox>
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
#include <vector>
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

// 分母为 0 的率显示「—」（规格一 §6.3），不是 0%/100%。
// #62：路线摘要在日志里也播报一次，与界面面板共用同一函数，口径不会漂。
QString onTimeRateText(int hit, int total) {
    if (total <= 0) {
        return QStringLiteral("—");
    }
    return QStringLiteral("%1%（%2/%3）")
        .arg(100.0 * static_cast<double>(hit) / static_cast<double>(total), 0, 'f', 1)
        .arg(hit)
        .arg(total);
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

    // 车队初始状态（规格二 §6.1）：**每车一份**。车停在各自仓库、时刻为各自发车时刻、
    // 车上无货；车队时钟 = 最早发车时刻（事件刻的起点）。
    fleet_.clear();
    fleet_.resize(config_.vehicles.size());
    int earliestDepart = 0;
    for (std::size_t v = 0; v < config_.vehicles.size(); ++v) {
        VehicleRuntime& rt = fleet_[v];
        rt.state.atNodeId = config_.vehicles[v].startNodeId;
        rt.state.atTimeMin = config_.vehicles[v].departTimeMin;
        rt.state.loadKg = 0.0;
        rt.departTimeMin = config_.vehicles[v].departTimeMin;
        if (v == 0 || rt.departTimeMin < earliestDepart) {
            earliestDepart = rt.departTimeMin;
        }
    }
    fleetClockMin_ = earliestDepart;
    activeVehicle_ = 0;

    // 事件刻：绝对时刻对齐（08:00 = 480 是 15 的整数倍，故首个事件落在 08:15）。
    // 事件只由软件内时钟驱动，见 settleEventsUpTo。
    const int eventInterval = config_.general.eventIntervalMin > 0
                                  ? config_.general.eventIntervalMin : 15;
    nextEventMark_ = logistics::nextEventTimeMin(fleetClockMin_, eventInterval);

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
    // 规格二 §7.6 / #25 Q9：紧跟「插入紧急订单」，名字带省略号表示会弹对话框。
    // 来源 manual_normal / 前缀 M；不计入自动刷单额度。没有这个动作，默认数据上
    // 三条转运判据全不触发 ⇒ 转运机制在验收现场根本不可演示。
    eventBar->addAction(QStringLiteral("插入普通订单…"), this, &MainWindow::onInsertNormalOrder);
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
    // 工具栏开关在这里挂；**停靠面板开关在 buildDocks 末尾补挂**——此时 dock 尚未
    // 创建，旧代码的 findChildren<QDockWidget*>() 恒为空，面板关掉后找不回来。
    viewMenu_ = menuBar()->addMenu(QStringLiteral("视图"));
    for (QToolBar* b : {planBar, eventBar, simBar, toolBar, logBar}) {
        viewMenu_->addAction(b->toggleViewAction());
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

    // ---- 右侧**一个**「信息」dock（#59：三页标签，同一时刻只显示一页）----
    //
    // #56 的 2×2 网格（车辆/订单 左列、中转与配送网/路线与停靠 右列）四块同屏，
    // 右栏总宽 ~760px，在 1360px 窗口里吃掉一半以上画布——用户原话「遮挡地图」。
    // #59 把右侧收敛成**一个 dock + 一个 QTabWidget**：① 车辆 ② 订单 ③ 中转与配送网，
    // 随时切换、同一时刻只显示一页；右栏宽降到 ~400px，画布恢复 ≥900px。
    // 信息本身**一条没少**：三张表 / 账本 / 空态提示都原样搬进各自页签。
    //
    // 路线摘要 + 停靠明细不再占右侧版面，移到底部 dock 的第二个标签页
    // （日志 / 路线与停靠）：这两份信息一条不删，G8/G9 守卫仍读表格对象。
    auto* infoDock = new QDockWidget(QStringLiteral("信息"), this);
    infoDock->setMinimumWidth(400);
    auto* infoTabs = new QTabWidget(infoDock);
    infoTabs->setDocumentMode(true);
    rightTabs_ = infoTabs;

    // ---- 页①：车辆（一车一行 6 列）----
    // 列序 = 车辆 | 状态 | 载货/额定(kg) | 利用率 | 在途单数 | 位置→下一站。
    // 「载货 / 额定 / 利用率」并排，正是用户说的「各个车辆的载货量/总货量」；
    // 表格化后四台车同屏可比，固定参数（两因子 / 起始仓库 / 发车时刻）移进行 tooltip。
    auto* vehiclePage = new QWidget(infoTabs);
    rightVehiclePage_ = vehiclePage;
    auto* vehicleLay = new QVBoxLayout(vehiclePage);
    vehicleLay->setContentsMargins(0, 0, 0, 0);
    vehicleLay->setSpacing(0);
    vehicleTable_ = new QTableWidget(0, 6, vehiclePage);
    vehicleTable_->setHorizontalHeaderLabels(
        // 8 列太宽、右侧 dock 里「下一站」会被裁掉（#56 lead 复核）。合并成 6 列：
        // 「载货/额定」同行并排 = 用户要的「载货量/总货量」，「位置→下一站」合成一格。
        {QStringLiteral("车辆"), QStringLiteral("状态"), QStringLiteral("载货/额定(kg)"),
         QStringLiteral("利用率"), QStringLiteral("在途单数"), QStringLiteral("位置→下一站")});
    vehicleTable_->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    vehicleTable_->horizontalHeader()->setStretchLastSection(true);
    vehicleTable_->verticalHeader()->setVisible(false);
    vehicleTable_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    // 一次选一行（选中行 = 画布上被加粗的那台车，替代旧的"选中页签"）。
    vehicleTable_->setSelectionBehavior(QAbstractItemView::SelectRows);
    vehicleTable_->setSelectionMode(QAbstractItemView::SingleSelection);
    vehicleLay->addWidget(vehicleTable_);
    infoTabs->addTab(vehiclePage, QStringLiteral("车辆"));

    // ---- 页②：订单（一单一行 7 列）----
    // 订单 | 取送(A→B) | kg | 窗口 | 状态 | 车辆 | 当前位置。
    // 新增「当前位置」是这张表的核心：车上(V02) / 站内·T01 / 已送达(D18) / 待取·D08——
    // 用户不必再去账本页对照"这批货现在在哪"。超时并入状态列（「· 超时 +Nmin」）。
    auto* orderPage = new QWidget(infoTabs);
    rightOrderPage_ = orderPage;
    auto* orderLay = new QVBoxLayout(orderPage);
    orderLay->setContentsMargins(0, 0, 0, 0);
    orderLay->setSpacing(0);
    orderTable_ = new QTableWidget(0, 7, orderPage);
    orderTable_->setHorizontalHeaderLabels(
        {QStringLiteral("订单"), QStringLiteral("取送"), QStringLiteral("kg"),
         QStringLiteral("窗口"), QStringLiteral("状态"), QStringLiteral("车辆"),
         QStringLiteral("当前位置")});
    orderTable_->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    orderTable_->horizontalHeader()->setStretchLastSection(true);
    orderTable_->verticalHeader()->setVisible(false);
    orderTable_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    orderLay->addWidget(orderTable_);
    infoTabs->addTab(orderPage, QStringLiteral("订单"));

    // ---- 页③：中转与配送网（上配送网表 + 下转运账本 + 空态提示）----
    // 上半 5 列 = 中转站 | 子网络 | 下属配送点 | 当前暂存货(kg) | 站内待取走(kg)；
    // 下半 7 列 = 订单 | 中转站 | kg | 卸入 | 取走 | 站内等待(min) | 状态。
    //
    // 为什么第 5 列非加不可：第 4 列读的是 veh.state.stashItems，只统计**降载暂存**；
    // 转运卸入的货**故意不进那个堆**（见 onAdvanceStopFor 的注释），所以跨簇接力期间
    // 站内明明有货、这一列永远是 0 —— 用户看到的就是"站里有货但面板说 0"。
    // 新列直接读 transitLedger_ 里 pickedUp == false 的行求和，卸入后、取走前必然 > 0。
    auto* transitPage = new QWidget(infoTabs);
    rightTransitPage_ = transitPage;
    auto* transitLay = new QVBoxLayout(transitPage);
    transitLay->setContentsMargins(0, 0, 0, 0);
    transitLay->setSpacing(2);
    auto* stationLabel = new QLabel(QStringLiteral("配送网（中转站）"), transitPage);
    stationLabel->setContentsMargins(6, 2, 0, 0);
    transitLay->addWidget(stationLabel);
    transitTable_ = new QTableWidget(0, 5, transitPage);
    transitTable_->setHorizontalHeaderLabels(
        {QStringLiteral("中转站"), QStringLiteral("子网络"), QStringLiteral("下属配送点"),
         QStringLiteral("当前暂存货(kg)"), QStringLiteral("站内待取走(kg)")});
    transitTable_->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    transitTable_->horizontalHeader()->setStretchLastSection(true);
    transitTable_->verticalHeader()->setVisible(false);
    transitTable_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    transitLay->addWidget(transitTable_, 1);
    auto* ledgerLabel = new QLabel(QStringLiteral("转运账本"), transitPage);
    ledgerLabel->setContentsMargins(6, 2, 0, 0);
    transitLay->addWidget(ledgerLabel);
    transitLedgerTable_ = new QTableWidget(0, 7, transitPage);
    transitLedgerTable_->setHorizontalHeaderLabels(
        {QStringLiteral("订单"), QStringLiteral("中转站"), QStringLiteral("kg"),
         QStringLiteral("卸入"), QStringLiteral("取走"), QStringLiteral("站内等待(min)"),
         QStringLiteral("状态")});
    transitLedgerTable_->horizontalHeader()->setSectionResizeMode(
        QHeaderView::ResizeToContents);
    transitLedgerTable_->horizontalHeader()->setStretchLastSection(true);
    transitLedgerTable_->verticalHeader()->setVisible(false);
    transitLedgerTable_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    transitLay->addWidget(transitLedgerTable_, 1);
    // 空态提示：账本无行时**明写原因**。默认数据没有跨簇单，账本必然为空；
    // 不解释的话下一个人会把它当"面板坏了/数据丢了"再立一张票。
    ledgerEmptyHint_ = new QLabel(
        QStringLiteral("本次运行未发生入站转运（默认数据无跨簇单）"), transitPage);
    ledgerEmptyHint_->setAlignment(Qt::AlignCenter);
    ledgerEmptyHint_->setWordWrap(true);
    ledgerEmptyHint_->setEnabled(false);   // 灰色提示，不抢表头的注意力
    transitLay->addWidget(ledgerEmptyHint_);
    infoTabs->addTab(transitPage, QStringLiteral("中转与配送网"));

    // 默认选中页①（车辆）——用户打开就要先看车。
    infoTabs->setCurrentIndex(0);
    infoDock->setWidget(infoTabs);
    addDockWidget(Qt::RightDockWidgetArea, infoDock);

    // ---- 底部 dock：#62 退役「路线与停靠」页，只剩「日志」一个标签 ----
    // 用户 2026-11-27 原话：「日志旁已经退役的标签页又出现了」——#59 把路线摘要与
    // 停靠明细塞进底部第二页是误判。这里删掉那页：路线关键信息（策略 / 总距离 /
    // 总耗时 / 总成本 / penalty / 取货率 / 送达率 / 逐车指标）改由**日志文本**承载
    //（规划完成时逐字播报，见 replan 的「路线摘要：」与「逐车指标 Vxx：」两行），
    // 停靠明细从界面退役。
    //
    // routeInfo_ / stopTable_ 两个对象**不再属于任何可见界面**，只是保留为守卫载体：
    // G8/G9 在 --ui-probe 里仍能读它们的真实表头/文本（「若存在则格式正确」），
    // 而新增的「不可见」守卫禁止它们被重新挂回界面。数据结构与填充逻辑不变，
    // 一旦有人再把它们塞进 dock，守卫立即变红。
    auto* logDock = new QDockWidget(QStringLiteral("日志"), this);
    auto* bottomTabs = new QTabWidget(logDock);
    bottomTabs->setDocumentMode(true);
    bottomTabs_ = bottomTabs;
    logView_ = new QPlainTextEdit(bottomTabs);
    logView_->setReadOnly(true);
    bottomTabs->addTab(logView_, QStringLiteral("日志"));
    bottomTabs->setCurrentIndex(0);
    logDock->setWidget(bottomTabs);
    addDockWidget(Qt::BottomDockWidgetArea, logDock);

    // 退役载体：无父布局、显式隐藏。不进 tab、不进 dock、不可见。
    routeInfo_ = new QTextBrowser(this);
    routeInfo_->setVisible(false);
    stopTable_ = new QTableWidget(0, 9, this);
    stopTable_->setHorizontalHeaderLabels(
        {QStringLiteral("车辆"), QStringLiteral("序号"), QStringLiteral("阶段"),
         QStringLiteral("配送点"), QStringLiteral("动作"), QStringLiteral("原始到达"),
         QStringLiteral("等待(分)"), QStringLiteral("送达"),
         QStringLiteral("余货(kg)")});
    stopTable_->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    stopTable_->horizontalHeader()->setStretchLastSection(true);
    stopTable_->verticalHeader()->setVisible(false);
    stopTable_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    stopTable_->setVisible(false);

    // 宽度：右侧一页一表，~400px 足够（#56 的四块同屏要 ~760px，正是「遮挡地图」的根因）。
    resizeDocks({infoDock}, {400}, Qt::Horizontal);
    // 高度：底部只有日志页，给若干行可读高度。
    resizeDocks({logDock}, {230}, Qt::Vertical);

    // 「视图」菜单的停靠面板显隐开关（#59 复核）：dock 此时才存在，在这里补挂。
    // 关掉「信息」或「日志」后必须能从菜单找回来（否则 #59 只剩一个右侧框，
    // 误关即再也看不到车辆/订单）。读的是 QDockWidget 自己的 toggleViewAction，
    // 菜单文字与面板标题同源，不会漂移。
    if (viewMenu_ != nullptr) {
        viewMenu_->addSeparator();
        for (QDockWidget* dock : findChildren<QDockWidget*>()) {
            viewMenu_->addAction(dock->toggleViewAction());
        }
    }
}
std::string MainWindow::currentPositionId() const {
    return active().state.atNodeId;
}

int MainWindow::currentTimeMin() const {
    return fleetClockMin_;
}

// 该车路线是否已走完（含"计划不可行/为空"）
bool MainWindow::vehAtEnd(std::size_t v) const {
    const VehicleRuntime& rt = veh(v);
    return rt.plan.status != logistics::PlanStatus::Ok || rt.plan.nodes.empty()
           || rt.nodeIndex + 1 >= rt.plan.nodes.size();
}

// 该车下一节点的到达时刻；走完返回 INT_MAX。
int MainWindow::vehNextArrival(std::size_t v) const {
    const VehicleRuntime& rt = veh(v);
    if (vehAtEnd(v) || rt.nodeIndex + 1 >= rt.plan.nodeArrivalMin.size()) {
        return std::numeric_limits<int>::max();
    }
    return rt.plan.nodeArrivalMin[rt.nodeIndex + 1];
}

// 全队下一到达时刻 = min over 车（§6.1）；全走完返回 INT_MAX。
int MainWindow::nextFleetArrival() const {
    int best = std::numeric_limits<int>::max();
    for (std::size_t v = 0; v < fleet_.size(); ++v) {
        const int t = vehNextArrival(v);
        if (t < best) {
            best = t;
        }
    }
    return best;
}

int MainWindow::minDepartMin() const {
    if (config_.vehicles.empty()) {
        return 0;
    }
    int best = config_.vehicles[0].departTimeMin;
    for (const logistics::Vehicle& v : config_.vehicles) {
        if (v.departTimeMin < best) {
            best = v.departTimeMin;
        }
    }
    return best;
}

std::string MainWindow::nextStopNodeId(std::size_t v) const {
    const VehicleRuntime& rt = veh(v);
    if (rt.stopCursor >= rt.plan.stops.size()) {
        // 规格五 #57：停靠走完之后，下一个目标就是**归位点**（还没到才显示；到了就显示已归位）。
        if (rt.plan.homed && !rt.plan.homeNodeId.empty()
            && rt.nodeIndex + 1 < rt.plan.nodes.size()) {
            return rt.plan.homeNodeId;
        }
        return std::string();
    }
    return rt.plan.stops[rt.stopCursor].nodeId;
}

QString MainWindow::vehiclePhaseText(std::size_t v) const {
    const VehicleRuntime& rt = veh(v);
    if (rt.plan.status != logistics::PlanStatus::Ok || rt.stopCursor >= rt.plan.stops.size()) {
        return QStringLiteral("已收工");
    }
    // 由剩余路线的阶段标记推出（#21 硬优先的可见证据）。
    bool hasUrgent = false;
    for (std::size_t i = rt.stopCursor; i < rt.plan.stops.size(); ++i) {
        if (rt.plan.stops[i].phase == logistics::StopPhase::Urgent) {
            hasUrgent = true;
            break;
        }
    }
    return hasUrgent ? QStringLiteral("紧急") : QStringLiteral("普通");
}

double MainWindow::vehicleRemainingDemandKg(std::size_t v) const {
    const VehicleRuntime& rt = veh(v);
    double kg = 0.0;
    for (std::size_t i = rt.stopCursor; i < rt.plan.stops.size(); ++i) {
        for (const logistics::StopDelivery& d : rt.plan.stops[i].deliveries) {
            for (const Order& o : config_.orders) {
                if (o.id == d.orderId && !o.served) {
                    kg += o.demandKg;
                }
            }
        }
    }
    return kg;
}

int MainWindow::vehicleServedOrderCount(std::size_t v) const {
    // 物理事实：该车**已经送达**的订单数（跨重规划保留）。
    // 口径 = 该车路线里出现、且订单集合里已标 served 的订单数。
    const VehicleRuntime& rt = veh(v);
    int served = 0;
    for (const Order& o : config_.orders) {
        if (!o.served) {
            continue;
        }
        for (const logistics::Stop& st : rt.plan.stops) {
            bool hit = false;
            for (const logistics::StopDelivery& d : st.deliveries) {
                if (d.orderId == o.id) {
                    hit = true;
                    break;
                }
            }
            if (hit) {
                ++served;
                break;
            }
        }
    }
    return served;
}

int MainWindow::selectedVehicleIndex() const {
    // #56：选中页签 -> 选中行。表格允许"无选中"（-1），此时按行序 0 处理，
    // 与旧实现的 currentIndex()<0 回退 0 保持同一语义。
    if (vehicleTable_ == nullptr) {
        return 0;
    }
    const int row = vehicleTable_->currentRow();
    return row >= 0 ? row : 0;
}

// 画布与 G15 快照共用的**唯一**路线数据源（规格三 GUI §7「只画到自己的目标」）。
// 每台车：从当前位置（nodeIndex）到**下一个停靠点**（nodeIsStop 为 true 的下一节点）那一段，
// 含起点与目标节点；没有下一个停靠点（已回库/空车）给空序列。
// 为什么用 nodeIsStop 而不是 stops：nodeIsStop 是 finalize 展平视图里「这一站要停」的标记，
// 与画布用同一份节点序列，stops 的下标与 nodes 的下标不同源，混用会错位。
std::vector<std::vector<std::string> > MainWindow::canvasTargetRoutes() const {
    std::vector<std::vector<std::string> > routes;
    routes.reserve(fleet_.size());
    for (std::size_t v = 0; v < fleet_.size(); ++v) {
        const VehicleRuntime& rt = fleet_[v];
        std::vector<std::string> seg;
        if (rt.plan.status == logistics::PlanStatus::Ok && rt.plan.nodes.size() >= 2) {
            const std::size_t from = (rt.nodeIndex < rt.plan.nodes.size()) ? rt.nodeIndex : 0;
            std::size_t target = rt.plan.nodes.size();
            for (std::size_t j = from + 1; j < rt.plan.nodes.size(); ++j) {
                if (j < rt.plan.nodeIsStop.size() && rt.plan.nodeIsStop[j]) {
                    target = j;
                    break;
                }
            }
            if (target >= rt.plan.nodes.size() && rt.plan.homed && from + 1 < rt.plan.nodes.size()) {
                // 规格五 #57：停靠走完但归位段还没走完 —— 画整段归位航线（这一段就是「回家」）。
                target = rt.plan.nodes.size() - 1;
            }
            if (target < rt.plan.nodes.size()) {
                seg.assign(rt.plan.nodes.begin() + static_cast<std::ptrdiff_t>(from),
                           rt.plan.nodes.begin() + static_cast<std::ptrdiff_t>(target) + 1);
            }
        }
        routes.push_back(seg);
    }
    return routes;
}

// 画布归属快照（G15）：逐条车路线边给出归属车（行序最前）与「是否属于选中车」。
// 读的是与画布同一份数据（canvasTargetRoutes + 车辆表选中行），不重算任何公式。
QString MainWindow::canvasOwnershipSnapshot() const {
    // 收集全部车路线边（有向），并记录**经过它的全部车**（行序）：
    // owner = 其中行序最前的那台车（共享边不重复绘制）。
    struct EdgeOwners {
        std::string from;
        std::string to;
        std::vector<std::size_t> owners;   // 按行序
    };
    // 与画布**同一份**数据：每车只到自己的下一个目标（canvasTargetRoutes）。
    const std::vector<std::vector<std::string> > routes = canvasTargetRoutes();
    std::vector<EdgeOwners> edges;
    for (std::size_t v = 0; v < routes.size(); ++v) {
        const std::vector<std::string>& seg = routes[v];
        for (std::size_t i = 1; i < seg.size(); ++i) {
            if (seg[i] == seg[i - 1]) {
                continue;   // 零位移（同节点连续两次停靠）不是边
            }
            std::size_t at = edges.size();
            for (std::size_t e = 0; e < edges.size(); ++e) {
                if (edges[e].from == seg[i - 1] && edges[e].to == seg[i]) {
                    at = e;
                    break;
                }
            }
            if (at == edges.size()) {
                EdgeOwners e;
                e.from = seg[i - 1];
                e.to = seg[i];
                edges.push_back(e);
            }
            edges[at].owners.push_back(v);   // 行序扫描 ⇒ 天然升序
        }
    }
    const int selected = selectedVehicleIndex();
    QString out;
    for (const EdgeOwners& e : edges) {
        // 加粗判定 = 该边是否属于**选中车**（与颜色归属解耦，§7.5⑤）。
        bool onSelected = false;
        QStringList all;
        for (const std::size_t v : e.owners) {
            all << QString::fromStdString(config_.vehicles[v].id);
            if (static_cast<int>(v) == selected) {
                onSelected = true;
            }
        }
        out += QStringLiteral("%1->%2 owner=%3 selected=%4 all=%5\n")
                   .arg(QString::fromStdString(e.from))
                   .arg(QString::fromStdString(e.to))
                   .arg(QString::fromStdString(config_.vehicles[e.owners.front()].id))
                   .arg(onSelected ? 1 : 0)
                   .arg(all.join(QStringLiteral(",")));
    }
    return out;
}

// 某配送点上的全部订单号，用 / 连接（同一节点可能有多单合并成一次停靠）

double MainWindow::currentLoadKg() const {
    // 直接读车辆状态。**不再**从计划的 remainingLoadKg 反推——
    // 重规划会换掉 plan_，反推出来的"当前载重"会跟着变，与车上真实货量不符。
    return active().state.loadKg;
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
    return active().state.onboard;
}

void MainWindow::arriveAt(std::size_t v, const std::string& nodeId, int timeMin) {
    VehicleRuntime& rt = veh(v);
    rt.state.atNodeId = nodeId;
    rt.state.atTimeMin = timeMin;
    // 慢车断路审计（规格七 #63）：**走过的路**是物理事实 —— 计划会被换掉，走过的节点不会。
    if (slowVisited_.size() != fleet_.size()) {
        slowVisited_.assign(fleet_.size(), std::vector<std::string>());
    }
    if (v < slowVisited_.size()) {
        slowVisited_[v].push_back(nodeId);
    }
    // 单条连续路线（规格一 MC-A01）：没有"趟"这回事，回仓库只是路线终点。
    // `departed` 是**物理事实**（是否已驶离出发点）：回仓库（路线终点）后置 false，
    // 重规划不得改写；唯一消费者是 fleetPristine()（判断「这一局还没动过」）。
    // 旧界面的「本趟出发装载」已退役（#56/#59），不再有别的消费者。
    const std::string& home = config_.vehicles[v].startNodeId;
    if (nodeId == home && rt.plan.nodes.size() > 1) {
        rt.state.departed = false;
    } else {
        rt.state.departed = true;
    }
}

void MainWindow::deliverAt(std::size_t v, const logistics::Stop& stop) {
    // 记账（"已经发生的事实"，重规划不得改写）。
    // 卸货本身已由 onAdvanceStopFor 的**动作集**按 orderId 完成（规格一 MC-E09/A05）；
    // 这里只做计数与超时登记，不再按 nodeId 扣货（那样会把同节点别的单一起扣掉）。
    VehicleRuntime& rt = veh(v);
    ++rt.state.servedStops;
    for (std::size_t d = 0; d < stop.deliveries.size(); ++d) {
        rt.state.deliveredOrderIds.insert(stop.deliveries[d].orderId);
    }
    if (stop.late) {
        rt.state.incurredPenaltyMin += stop.penaltyMin;
        for (const logistics::StopDelivery& d : stop.deliveries) {
            if (!d.late) {
                continue;
            }
            VehicleState::LateStop ls;
            ls.orderId = d.orderId;
            ls.nodeId = stop.nodeId;
            ls.arrivalMin = stop.arrivalMin;
            ls.penaltyMin = d.penaltyMin;
            rt.state.deliveredLate.push_back(ls);
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

// 画布（规格二 §7.5⑤；#50 起每车只画目标箭头、按角色两色）：选中行那台车加粗。
// 每条车路线边都有归属（共享边归行序最前那台车）；空车不参与绘制但车辆表里仍有行。
// **权重标签仍是基础边权**（pickWeight 不接车辆参数）。
void MainWindow::syncScene() {
    scene_->build(config_.graph, scene_->weightType());
    scene_->setAllLabelsVisible(false);
    // 规格三 GUI §7：**每车只画到自己的下一个目标**，不再把整条剩余路线铺满画布。
    // 路线数据与 G15 快照共用 canvasTargetRoutes（画什么 = 探针读到什么）。
    // 颜色按**角色**（慢车一色 / 快车另一色），不再按行序四色。
    const std::vector<std::vector<std::string> > routes = canvasTargetRoutes();
    const std::vector<bool> fastFlags = logistics::fastVehicleFlags(config_.vehicles);
    scene_->setFleetRoutes(routes, selectedVehicleIndex(), fastFlags);

    // 车辆环：每台车在自己所在节点外一个环；同节点多车按行序 slot 错开。
    // #62：车号一并交给节点 tooltip，解决 T03 上 V01 快车 + V04 慢车两环叠加的视觉误解。
    std::vector<std::string> atNodes;
    std::vector<std::string> vehicleIds;
    atNodes.reserve(fleet_.size());
    vehicleIds.reserve(fleet_.size());
    for (std::size_t v = 0; v < fleet_.size() && v < config_.vehicles.size(); ++v) {
        atNodes.push_back(fleet_[v].state.atNodeId);
        vehicleIds.push_back(config_.vehicles[v].id);
    }
    scene_->setVehicleMarks(atNodes, fastFlags, vehicleIds);
}

// 换计划之后必须做的三件事，只此一处。三处重规划路径（replan /
// replanIncremental / insertOrder）都调它，避免各写一份而漏改。
void MainWindow::afterPlanReplaced() {
    active().nodeIndex = 0;      // 新路线从车辆当前位置起算，推进游标归零
    active().stopCursor = 0;
    // 新计划里还有路要走 => 重新允许"到终点播报一次"。
    // 否则跑完一次后再「重新规划」，第二次跑完就再也播报不出来了。
    if (!atRouteEnd()) {
        routeFinishedReported_ = false;
    }
    // 单条连续路线下车辆空载出发、货在取货点 A 装车，重规划后**不需要**装车。
}


// ---- 车队规划的两个入口（规格二 §6 / #42）----
//
// ① **初始规划 / 全车队重调度**：车队还没动过（fleetPristine()）时走 #40 的
//    三阶段分配 planFleet。这就是 CLI \`--dump-plan\` 的 ALLOC 的**同一条**代码路径，
//    所以界面上的分配与命令行逐单一致（守卫：runActionSelfCheck 第 10 段）。
// ② **途中重规划**：车辆已带位置/时刻/在车货 ⇒ 逐车 replan（已执行前缀不重排，§6.2）。
//    这条路径**不改分配**：每台车只重算自己手上还没送的那些单。
void MainWindow::replanFleet() {
    if (config_.vehicles.empty() || fleet_.size() != config_.vehicles.size()) {
        appendLog(QStringLiteral("配置中没有车辆，无法规划"));
        return;
    }
    const std::size_t k = fleet_.size();
    std::vector<std::vector<Order> > assigned(k);
    std::vector<logistics::RoutePlan> plans(k);
    std::vector<logistics::VehicleStartState> starts(k);
    for (std::size_t v = 0; v < k; ++v) {
        const VehicleRuntime& rt = fleet_[v];
        starts[v].startNodeId = rt.state.atNodeId;
        starts[v].startTimeMin = rt.state.atTimeMin;
        starts[v].onboard = rt.state.onboard;
        starts[v].stash.station = rt.state.stashStation;
        starts[v].stash.items = rt.state.stashItems;
    }

    if (fleetPristine()) {
        // ① 三阶段分配（聚类 -> 整簇全排列 -> 有界改派）
        const logistics::FleetPlan fp =
            logistics::planFleet(config_.graph, config_.vehicles, remainingOrders(), planWeight_);
        for (std::size_t i = 0; i < fp.allocationLog.size(); ++i) {
            appendLog(QStringLiteral("[分配] %1")
                          .arg(QString::fromStdString(fp.allocationLog[i])));
        }
        // 腿结构的登记（规格三 §1 / #45）：「结构已立、接力未启用」必须在运行日志里
        // 看得见 —— 否则用户会以为 LEG 行意味着已经转运。日志行已自带「[腿] 」前缀。
        for (std::size_t i = 0; i < fp.legLog.size(); ++i) {
            appendLog(QString::fromStdString(fp.legLog[i]));
        }
        allocationSnapshot_ = allocationTextOf(fp);
        appendLog(QStringLiteral("[分配] 三阶段分配（%1 台车）：%2")
                      .arg(k)
                      .arg(allocationSnapshot_));
        if (!fp.allOk) {
            appendLog(QStringLiteral("[分配] 三阶段分配存在不可行车辆：%1")
                          .arg(QString::fromStdString(fp.reason)));
        }
        assigned = logistics::vehicleOrdersFromPlans(fp.vehicles, remainingOrders());
        for (std::size_t v = 0; v < k; ++v) {
            veh(v).plan = (v < fp.vehicles.size()) ? fp.vehicles[v] : logistics::RoutePlan();
            afterVehiclePlanReplaced(v);
            plans[v] = veh(v).plan;
        }
    } else {
        // ② 途中重规划（逐车，已执行前缀不重排）
        std::vector<logistics::RoutePlan> current(k);
        for (std::size_t v = 0; v < k; ++v) {
            current[v] = fleet_[v].plan;
        }
        assigned = logistics::vehicleOrdersFromPlans(current, remainingOrders());
        // 边准入（规格七 #63）：途中重规划也按**该车**的断路口径走 —— 否则事件/手动
        // 重规划会把慢车送到别簇（静态计划合规、运行态越界，正是本票要堵的洞）。
        const std::vector<logistics::ReachScope> scopes =
            logistics::reachScopesOf(config_.graph, config_.vehicles);
        for (std::size_t v = 0; v < k; ++v) {
            VehicleRuntime& rt = fleet_[v];
            rt.plan = logistics::replan(config_.graph, config_.vehicles[v], assigned[v],
                                        rt.state.atNodeId, rt.state.atTimeMin, planWeight_,
                                        rt.state.onboard, starts[v].stash, true,
                                        v < scopes.size() ? &scopes[v] : nullptr);
            afterVehiclePlanReplaced(v);
            plans[v] = rt.plan;
        }
    }

    // ---- 后置有界改进 · 跨车转运（规格二 §5 / #28）：与 CLI 同一层，绝不更差 ----
    {
        const logistics::FleetTransitResult transit = logistics::applyTransitLayer(
            config_.graph, config_.vehicles, assigned, plans, starts, planWeight_);
        if (transit.acceptedRounds > 0) {
            for (std::size_t v = 0; v < k; ++v) {
                if (v < transit.plans.size()) {
                    veh(v).plan = transit.plans[v];
                    afterVehiclePlanReplaced(v);
                    plans[v] = veh(v).plan;
                }
            }
        }
        for (std::size_t i = 0; i < transit.log.size(); ++i) {
            appendLog(QStringLiteral("[转运] %1").arg(QString::fromStdString(transit.log[i])));
        }
    }
    fleetPlan_ = logistics::aggregateFleet(config_.vehicles, plans);
    for (std::size_t v = 0; v < k; ++v) {
        const logistics::Vehicle& vehicle = config_.vehicles[v];
        const logistics::RoutePlan& plan = veh(v).plan;
        if (plan.status == logistics::PlanStatus::Ok) {
            appendLog(QStringLiteral("车 %1 规划成功：%2 站，距离 %3km，耗时 %4min，penalty %5min")
                          .arg(QString::fromStdString(vehicle.id))
                          .arg(plan.stops.size())
                          .arg(plan.totalDistanceKm, 0, 'f', 1)
                          .arg(plan.totalTimeMin, 0, 'f', 1)
                          .arg(plan.totalPenaltyMin));
        } else {
            appendLog(QStringLiteral("车 %1 规划不可行：%2")
                          .arg(QString::fromStdString(vehicle.id))
                          .arg(QString::fromStdString(plan.reason)));
        }
        if (!vehAtEnd(v)) {
            routeFinishedReported_ = false;
        }
    }
    if (fleetPlan_.allOk) {
        appendLog(QStringLiteral("车队规划完成：%1 台车，完工耗时 %2min（车时合计 %3min），"
                                 "总距离 %4km，总成本 %5 元，penalty %6min")
                      .arg(k)
                      .arg(fleetPlan_.completionTimeMin, 0, 'f', 1)
                      .arg(fleetPlan_.sumVehicleTimeMin, 0, 'f', 1)
                      .arg(fleetPlan_.totalDistanceKm, 0, 'f', 1)
                      .arg(fleetPlan_.totalCostYuan, 0, 'f', 1)
                      .arg(fleetPlan_.totalPenaltyMin));
    } else {
        appendLog(QStringLiteral("车队规划有不可行车辆：%1")
                      .arg(QString::fromStdString(fleetPlan_.reason)));
    }
    // ---- #62：底部「路线与停靠」页退役后，路线关键信息改由**日志文本**承载 ----
    // 规划完成时播报一次（不是每次 updatePanels 都写，避免刷屏）。口径与路线面板
    // 原文本逐项一致：策略 / 总距离 / 完工耗时 / 总成本 / penalty / 停靠·动作 /
    // 取货率 / 送达率（紧急·普通分列） / 平均在车 / 均衡度；随后每车一行 7 项。
    {
        const logistics::RouteMetrics& fm = fleetPlan_.metrics;
        const QString deliveryRate =
            onTimeRateText(fm.deliveryOnTimeCount, fm.deliveryTotal)
            + QStringLiteral("（紧急 %1 · 普通 %2）")
                  .arg(onTimeRateText(fm.urgentOnTimeCount, fm.urgentTotal),
                       onTimeRateText(fm.normalOnTimeCount, fm.normalTotal));
        appendLog(QStringLiteral("路线摘要：策略 %1 | 总距离 %2 km | 完工耗时 %3 min"
                                 "（车时合计 %4 min） | 总成本 %5 元 | penalty %6 min | "
                                 "停靠 %7 站 · 动作 %8 次 | 取货准时率 %9 | "
                                 "送达准时率 %10 | 平均在车 %11 kg"
                                 "（利用率 %12% · 峰值 %13 kg） | "
                                 "均衡度（峰值利用率极差） %14")
                      .arg(strategyBox_ != nullptr ? strategyBox_->currentText() : QString())
                      .arg(fleetPlan_.totalDistanceKm, 0, 'f', 1)
                      .arg(fleetPlan_.completionTimeMin, 0, 'f', 1)
                      .arg(fleetPlan_.sumVehicleTimeMin, 0, 'f', 1)
                      .arg(fleetPlan_.totalCostYuan, 0, 'f', 1)
                      .arg(fleetPlan_.totalPenaltyMin)
                      .arg(fleetPlan_.stopCount)
                      .arg(fleetPlan_.actionCount)
                      .arg(onTimeRateText(fm.pickupOnTimeCount, fm.pickupTotal))
                      .arg(deliveryRate)
                      .arg(fm.avgLoadKg, 0, 'f', 1)
                      .arg(fm.loadUtilization * 100.0, 0, 'f', 1)
                      .arg(fm.peakLoadKg, 0, 'f', 1)
                      .arg(fleetPlan_.balanceSpread, 0, 'f', 2));
        for (std::size_t v = 0; v < k && v < fleetPlan_.vehicles.size(); ++v) {
            const logistics::Vehicle& vehicle = config_.vehicles[v];
            const logistics::RoutePlan& plan = fleetPlan_.vehicles[v];
            const logistics::RouteMetrics& vm = plan.metrics;
            appendLog(QStringLiteral("逐车指标 %1：距离 %2 km · 耗时 %3 min · 成本 %4 元 · "
                                     "penalty %5 · 停靠 %6 · 利用率 %7% · 取货 %8 · 送达 %9")
                          .arg(QString::fromStdString(vehicle.id))
                          .arg(plan.totalDistanceKm, 0, 'f', 1)
                          .arg(plan.totalTimeMin, 0, 'f', 1)
                          .arg(plan.totalCostYuan, 0, 'f', 1)
                          .arg(plan.totalPenaltyMin)
                          .arg(vm.stopCount)
                          .arg(vm.loadUtilization * 100.0, 0, 'f', 1)
                          .arg(onTimeRateText(vm.pickupOnTimeCount, vm.pickupTotal))
                          .arg(onTimeRateText(vm.deliveryOnTimeCount, vm.deliveryTotal)));
        }
    }
    syncScene();
    updatePanels();
}

// 车队是否「还没动过」：全部车停在各自仓库、各自发车时刻、车上无货、无暂存，
// 且没有任何订单被取走/送达。是 ⇒ 初始规划 / 全车队重调度（走三阶段分配）。
bool MainWindow::fleetPristine() const {
    for (std::size_t v = 0; v < fleet_.size(); ++v) {
        const VehicleRuntime& rt = fleet_[v];
        const logistics::Vehicle& vehicle = config_.vehicles[v];
        if (rt.state.atNodeId != vehicle.startNodeId) {
            return false;
        }
        if (rt.state.atTimeMin != vehicle.departTimeMin) {
            return false;
        }
        if (rt.state.departed || rt.nodeIndex != 0 || rt.stopCursor != 0) {
            return false;
        }
        if (!rt.state.onboard.empty() || !rt.state.stashItems.empty()) {
            return false;
        }
        if (rt.state.servedStops != 0) {
            return false;
        }
    }
    for (std::size_t i = 0; i < config_.orders.size(); ++i) {
        if (config_.orders[i].served || config_.orders[i].picked) {
            return false;
        }
    }
    return true;
}

// 分配快照：与 CLI \`--dump-plan\` 的 ALLOC 行**同序同形**（订单 ID 升序、\`O01=V01,…\`）。
QString MainWindow::allocationTextOf(const logistics::FleetPlan& plan) const {
    QString out;
    for (std::size_t i = 0; i < plan.allocationOrderIds.size(); ++i) {
        if (!out.isEmpty()) {
            out += QLatin1Char(',');
        }
        out += QString::fromStdString(plan.allocationOrderIds[i]) + QLatin1Char('=');
        const std::size_t v = plan.allocationVehicleIndex[i];
        out += (v < config_.vehicles.size())
                   ? QString::fromStdString(config_.vehicles[v].id)
                   : QStringLiteral("-");
    }
    return out;
}

// 单车的计划替换后的统一收尾：新计划从该车当前位置起算，游标归零。
void MainWindow::afterVehiclePlanReplaced(std::size_t v) {
    if (v >= fleet_.size()) {
        return;
    }
    veh(v).nodeIndex = 0;
    veh(v).stopCursor = 0;
    if (!vehAtEnd(v)) {
        routeFinishedReported_ = false;
    }
    // 每一份新计划都要过一遍慢车断路审计（规格七 #63）：坏计划不许悄悄进运行态。
    auditSlowIsolationNow();
}

// ---- 慢车物理断路审计（规格七 #63）----
//
// 逐帧读**运行态的真实计划 + 已走过的节点**，交给 core 的独立审计函数判
// 「慢车是否出现在别簇节点 / 跨簇边 / 仓库边 / 站间边」。判据本体在 core
// （auditSlowIsolation），这里只负责喂数据与去重 —— 界面不另写一份规则。
void MainWindow::auditSlowIsolationNow() {
    ++slowAuditFrames_;
    if (config_.vehicles.empty() || fleet_.size() != config_.vehicles.size()) {
        return;
    }
    if (slowVisited_.size() != fleet_.size()) {
        slowVisited_.assign(fleet_.size(), std::vector<std::string>());
    }
    std::vector<logistics::RoutePlan> plans(fleet_.size());
    for (std::size_t v = 0; v < fleet_.size(); ++v) {
        plans[v] = fleet_[v].plan;
    }
    const std::vector<logistics::SlowIsolationViolation> bad =
        logistics::auditSlowIsolation(config_.graph, config_.vehicles, plans, &slowVisited_);
    for (std::size_t i = 0; i < bad.size(); ++i) {
        const std::string line = bad[i].vehicleId + " " + bad[i].detail;
        bool seen = false;
        for (std::size_t k = 0; k < slowViolations_.size(); ++k) {
            if (slowViolations_[k] == line) {
                seen = true;
            }
        }
        if (!seen) {
            slowViolations_.push_back(line);
        }
    }
}

// 复调度输入常量（§6.2）：各车当前位置/时刻/在车货/暂存 + 逐车订单集合 +
// 逐车**剩余**路线（R1 的重放来源）+ 全队未送达订单 + 站内已执行卸入。
logistics::FleetRescheduleInput MainWindow::buildRescheduleInput() const {
    logistics::FleetRescheduleInput in;
    const std::size_t k = fleet_.size();
    in.starts.resize(k);
    in.remainingPlans.resize(k);
    for (std::size_t v = 0; v < k; ++v) {
        const VehicleRuntime& rt = fleet_[v];
        in.starts[v].startNodeId = rt.state.atNodeId;
        in.starts[v].startTimeMin = rt.state.atTimeMin;
        in.starts[v].onboard = rt.state.onboard;
        in.starts[v].stash.station = rt.state.stashStation;
        in.starts[v].stash.items = rt.state.stashItems;
        in.remainingPlans[v] = logistics::sliceRemainder(rt.plan, rt.nodeIndex);
    }
    in.allRemainingOrders = remainingOrders();
    in.vehicleOrders =
        logistics::vehicleOrdersFromPlans(in.remainingPlans, in.allRemainingOrders);
    // 站内**已执行**的转运卸入（只含真的执行过的卸入；计划期候选不在这里）。
    for (std::size_t i = 0; i < transitLedger_.size(); ++i) {
        const TransitLedgerEntry& entry = transitLedger_[i];
        if (entry.pickedUp || entry.delivered) {
            continue;
        }
        const logistics::Order* order = findOrderById(entry.orderId);
        if (order == nullptr) {
            continue;
        }
        logistics::ExecutedTransitEntry item;
        item.leg.orderId = entry.orderId;
        item.leg.stationId = entry.stationId;
        item.leg.kg = entry.kg;
        item.leg.inVehicleId = entry.inVehicleId;
        item.leg.inTimeMin = entry.inTimeMin;
        item.leg.outVehicleId = entry.outVehicleId;
        item.leg.outTimeMin = entry.outTimeMin;
        item.order = *order;
        in.executedInStation.push_back(item);
    }
    return in;
}

// 把一次复调度的结果写回：**只**覆盖被重算的车（未受影响车路线逐字不变，不变量 5）。
// 分级播报行进运行日志——那是用户看得到的东西，守卫读它，不读内部字段。
void MainWindow::applyRescheduleResult(const logistics::FleetRescheduleResult& res) {
    for (std::size_t i = 0; i < res.replanned.size(); ++i) {
        const std::size_t v = res.replanned[i];
        if (v >= fleet_.size() || v >= res.plans.size()) {
            continue;
        }
        if (res.plans[v].nodes.empty()) {
            continue;   // 不拿空计划覆盖现有计划（拒绝路径不能留下半成品）
        }
        veh(v).plan = res.plans[v];
        afterVehiclePlanReplaced(v);
    }
    for (std::size_t i = 0; i < res.log.size(); ++i) {
        appendLog(QString::fromStdString(res.log[i]));
    }
    std::vector<logistics::RoutePlan> plans;
    plans.reserve(fleet_.size());
    for (std::size_t v = 0; v < fleet_.size(); ++v) {
        plans.push_back(veh(v).plan);
    }
    fleetPlan_ = logistics::aggregateFleet(config_.vehicles, plans);
    // 事件后重建的腿结构（P1-③ / US21）：aggregateFleet 不建腿，这里接上 rescheduleFleet
    // 带回的 buildLegs 结果，让 GUI 与守卫读到同一份「实际执行形态」；降级行写进运行日志
    // （用户可见），"假报 3 条腿"在界面上同样藏不住。
    if (!res.legs.empty()) {
        fleetPlan_.legs = res.legs;
        fleetPlan_.legLog = res.legLog;
        for (std::size_t i = 0; i < res.legLog.size(); ++i) {
            if (res.legLog[i].rfind("LEGDEGRADE ", 0) == 0) {
                appendLog(QString::fromStdString(res.legLog[i]));
            }
        }
    }
    syncScene();
    updatePanels();
}

logistics::FleetRescheduleResult MainWindow::runReschedule(
    const logistics::RescheduleEvent& event) {
    logistics::FleetRescheduleResult res;
    if (config_.vehicles.empty()) {
        return res;
    }
    const logistics::FleetRescheduleInput in = buildRescheduleInput();
    res = logistics::rescheduleFleet(config_.graph, config_.vehicles, planWeight_, in, event);
    applyRescheduleResult(res);
    return res;
}

// #41 的转运层作为后置有界改进再跑一遍（只在**跨簇**新单插入后调用）。
// 它不改分配、不改构造器，只在整计划键**严格更优**时替换；完全并列保留禁转运版。
void MainWindow::runTransitPostPass() {
    if (config_.vehicles.empty() || fleet_.size() != config_.vehicles.size()) {
        return;
    }
    const logistics::FleetRescheduleInput in = buildRescheduleInput();
    std::vector<logistics::RoutePlan> plans = in.remainingPlans;
    const logistics::FleetTransitResult transit = logistics::applyTransitLayer(
        config_.graph, config_.vehicles, in.vehicleOrders, plans, in.starts, planWeight_);
    for (std::size_t i = 0; i < transit.log.size(); ++i) {
        appendLog(QStringLiteral("[转运] %1").arg(QString::fromStdString(transit.log[i])));
    }
    if (transit.acceptedRounds > 0) {
        for (std::size_t v = 0; v < fleet_.size() && v < transit.plans.size(); ++v) {
            if (transit.plans[v].nodes.empty()) {
                continue;
            }
            veh(v).plan = transit.plans[v];
            afterVehiclePlanReplaced(v);
        }
    }
    std::vector<logistics::RoutePlan> full;
    for (std::size_t v = 0; v < fleet_.size(); ++v) {
        full.push_back(veh(v).plan);
    }
    fleetPlan_ = logistics::aggregateFleet(config_.vehicles, full);
    syncScene();
    updatePanels();
}

void MainWindow::replan() {
    replanFleet();
}

void MainWindow::onStrategyChanged() {
    const QString strategy = strategyBox_->currentData().toString();
    planWeight_ = (strategy == QStringLiteral("cost"))
                      ? WeightType::Cost
                      : (strategy == QStringLiteral("time") ? WeightType::Time
                                                            : WeightType::Distance);
    // 不在这里清零 active().stopCursor/active().nodeIndex：replan() 需要先用它们记账，
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

    // ---- 受影响车集合（§6.5）+ 复调度（§6.4 五级升级链）----
    // **不是**「任一车命中就全队重算」：只有剩余路线真的用到被改动边（且增幅达标）的车
    // 才进集合；未受影响车的路线逐字不变（不变量 5）。
    const logistics::FleetRescheduleInput in = buildRescheduleInput();
    const std::vector<std::size_t> affected = logistics::affectedVehiclesForTraffic(
        in.remainingPlans, report.changes, config_.general.trafficTimeIncreaseMin);
    if (affected.empty()) {
        log(QStringLiteral("受影响边不在任何车辆的剩余路径上或增幅不足 -> 不触发重规划"));
        syncScene();   // 拥堵标记需要重绘
        return;
    }
    QStringList names;
    for (std::size_t i = 0; i < affected.size(); ++i) {
        names << QString::fromStdString(config_.vehicles[affected[i]].id);
    }
    log(QStringLiteral("受影响车集合（%1 台）：%2")
            .arg(affected.size())
            .arg(names.join(QStringLiteral("、"))));

    logistics::RescheduleEvent event;
    event.trigger = logistics::RescheduleTrigger::Traffic;
    event.changes = report.changes;
    event.thresholdRatio = config_.general.trafficTimeIncreaseMin;
    const logistics::FleetRescheduleResult res = runReschedule(event);
    if (res.rejected) {
        log(QStringLiteral("  ⚠ 复调度被拒绝：受影响车保持原计划"));
    }
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
    // 上限取**最小**车容量（任何一台车都要装得下这张单，否则它永远无法被分配）。
    double cap = 0.0;
    for (const logistics::Vehicle& v : config_.vehicles) {
        if (cap <= 0.0 || v.capacityKg < cap) {
            cap = v.capacityKg;
        }
    }
    if (!(demand > 0.0) || demand > cap + 1e-9) {
        appendLog(kindText + QStringLiteral("：送货量须在 (0, 最小车载重上限] 内"), windowBaseMin);
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

    // 冲突提示按**车辆表选中的车**先算一遍（"这单在它那儿最早什么时候到"），
    // 真正的分配仍走车队重规划（顺序轮转基线，#40 换成类型感知分配）。
    const std::vector<Order> pending = remainingOrders();
    logistics::StashState stashIns;
    stashIns.station = active().state.stashStation;
    stashIns.items = active().state.stashItems;
    // R8：紧急 -> 紧急段尾插（FIFO）；普通 -> 普通段 pair-insertion。urgent 由订单自己带。
    const std::vector<logistics::ReachScope> scopes =
        logistics::reachScopesOf(config_.graph, config_.vehicles);
    const logistics::InsertResult inserted = logistics::insertOrder(
        config_.graph, config_.vehicles[activeVehicle_], pending, order, currentPositionId(),
        currentTimeMin(), planWeight_, onboardGoods(), stashIns,
        activeVehicle_ < scopes.size() ? &scopes[activeVehicle_] : nullptr);

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
    // ---- 新单的分配（§6.6）：**不重算任何既有车**，只对选中的车做插入 ----
    // cands 按 Δkey 升序（并列按车辆 ID），首个可行即接受；全不可行才进升级链。
    logistics::RescheduleEvent event;
    event.trigger = logistics::RescheduleTrigger::NewOrder;
    event.newOrder = order;
    const logistics::FleetRescheduleResult res = runReschedule(event);
    if (res.rejected) {
        // 不丢单：该单仍在订单集合里、状态为未服务（不变量 14）。
        appendLog(QStringLiteral("  ⚠ 该单保留未服务状态（各车均无可行插入，不通过违反容量凑解）"),
                  windowBaseMin);
    } else if (logistics::isCrossCluster(config_.graph, order)) {
        // #41 的转运层是「绝不更差」的后置改进，判据①就是跨簇 —— 只在跨簇新单后跑一遍，
        // **不改分配**（§6.6「局部可行即接受，不做局部 vs 全局两版取优」）。
        runTransitPostPass();
    }
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

    // 上限取**最小**车容量（与 insertOrderAt 的校验同口径：任何一台车都要装得下）。
    double cap = 0.0;
    for (const logistics::Vehicle& v : config_.vehicles) {
        if (cap <= 0.0 || v.capacityKg < cap) {
            cap = v.capacityKg;
        }
    }
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

// 手动「插入普通订单…」（规格二 §7.6 / #25 Q9）：**跨簇演示入口**。
//
// 与「插入紧急订单」同形（一次填完 A / B / 货量，B 下拉始终排除当前 A），差别只有：
//   · urgent = false、来源 manual_normal、前缀 M；
//   · 货量整数 3–15 且**严格小于** min([vehicles].capacity_kg)；
//   · 窗口 [当前软件内时刻, 24:00]（与自动刷单同构）；
//   · 不计入自动刷单额度（额度只数事件来源的单，R6）。
// 无头路径（--self-check-actions / --demo / offscreen）退化为随机 A/B，保持自动化可跑。
void MainWindow::onInsertNormalOrder() {
    if (QGuiApplication::platformName() == QStringLiteral("offscreen")) {
        insertRandomOrder(false, logistics::OrderSource::ManualNormal, currentTimeMin());
        return;
    }

    QStringList customers;
    for (const logistics::Node& n : config_.graph.nodes()) {
        if (n.type == logistics::NodeType::Delivery) {
            customers << QString::fromStdString(n.id);
        }
    }
    if (customers.size() < 2) {
        appendLog(QStringLiteral("客户节点不足，无法插入普通取送单"));
        return;
    }

    // 上限 = min(15, min(capacity) - 1)：整数 3–15 且 **<** 最小车载重上限。
    double cap = 0.0;
    for (const logistics::Vehicle& v : config_.vehicles) {
        if (cap <= 0.0 || v.capacityKg < cap) {
            cap = v.capacityKg;
        }
    }
    int maxKg = 15;
    const int capKg = static_cast<int>(std::floor(cap));
    if (capKg - 1 < maxKg) {
        maxKg = capKg - 1;
    }
    if (maxKg < 3) {
        appendLog(QStringLiteral("最小车载重上限过小（%1kg），无法插入 3kg 普通取送单")
                      .arg(cap, 0, 'f', 0));
        return;
    }

    QDialog dialog(this);
    dialog.setWindowTitle(QStringLiteral("插入普通取送单"));

    auto* aCombo = new QComboBox(&dialog);
    aCombo->addItems(customers);
    auto* bCombo = new QComboBox(&dialog);
    auto* demand = new QSpinBox(&dialog);
    demand->setRange(3, maxKg);
    demand->setSingleStep(1);
    demand->setValue(maxKg >= 8 ? 8 : 3);

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
    form->addRow(QStringLiteral("货量(kg)"), demand);

    auto* box = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dialog);
    connect(box, &QDialogButtonBox::accepted, &dialog, [&]() {
        if (aCombo->currentText() == bCombo->currentText()) {
            QMessageBox::warning(&dialog, QStringLiteral("插入普通取送单"),
                                 QStringLiteral("取货点 A 与送达点 B 不能相同"));
            return;
        }
        if (!(demand->value() >= 3 && demand->value() <= 15
              && static_cast<double>(demand->value()) < cap - 1e-9)) {
            QMessageBox::warning(&dialog, QStringLiteral("插入普通取送单"),
                                 QStringLiteral("货量须为 3–15 的整数且小于最小车载重上限"));
            return;
        }
        const std::string id = insertOrderAt(aCombo->currentText().toStdString(),
                                             bCombo->currentText().toStdString(),
                                             demand->value(), false,
                                             logistics::OrderSource::ManualNormal,
                                             currentTimeMin());
        if (id.empty()) {
            QMessageBox::warning(&dialog, QStringLiteral("插入普通取送单"),
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
    // 封路也是事件：受影响车 = 剩余路线**用到**被封闭边（任一方向）的车（§6.5）。
    logistics::RescheduleEvent event;
    event.trigger = logistics::RescheduleTrigger::RoadClosure;
    const std::size_t arrow = closed.find("->");
    if (arrow != std::string::npos) {
        logistics::TrafficChange forward;
        forward.fromId = closed.substr(0, arrow);
        forward.toId = closed.substr(arrow + 2);
        logistics::TrafficChange backward;
        backward.fromId = forward.toId;
        backward.toId = forward.fromId;
        event.changes.push_back(forward);
        event.changes.push_back(backward);
    }
    const logistics::FleetRescheduleInput in = buildRescheduleInput();
    const std::vector<std::size_t> affected =
        logistics::affectedVehiclesForClosure(in.remainingPlans, event.changes);
    if (affected.empty()) {
        appendLog(QStringLiteral("没有车辆的剩余路线用到被封闭的道路 -> 不触发重规划"), atMin);
        syncScene();
        return;
    }
    QStringList names;
    for (std::size_t i = 0; i < affected.size(); ++i) {
        names << QString::fromStdString(config_.vehicles[affected[i]].id);
    }
    appendLog(QStringLiteral("受影响车集合（%1 台）：%2")
                  .arg(affected.size())
                  .arg(names.join(QStringLiteral("、"))),
              atMin);
    const logistics::FleetRescheduleResult res = runReschedule(event);
    if (res.rejected) {
        appendLog(QStringLiteral("  ⚠ 复调度被拒绝：受影响车保持原计划"), atMin);
    }
}

// 该车前进一个节点并执行该节点的动作集（**唯一的状态转移点**，规格二 §6.1）。
// 由 advanceFleetTo 在「全队下一到达时刻」处对**同刻到达的所有车**调用。
void MainWindow::onAdvanceStopFor(std::size_t v) {
    VehicleRuntime& rt = veh(v);
    const logistics::Vehicle& vehicle = config_.vehicles[v];
    if (rt.plan.status != logistics::PlanStatus::Ok || rt.plan.nodes.empty()) {
        return;
    }
    if (rt.nodeIndex + 1 >= rt.plan.nodes.size()) {
        return;   // 该车已走完
    }

    // 逐个节点前进。**状态转移只发生在这一处**：
    // 位置/时刻 -> arriveAt，动作集（卸/装/降载/取回）-> 下面的动作集段，记账 -> deliverAt。
    // 除此之外任何地方都不得改 state_。
    ++veh(v).nodeIndex;
    const std::string node = veh(v).plan.nodes[veh(v).nodeIndex];
    int arrTime = veh(v).state.atTimeMin;
    if (veh(v).nodeIndex < veh(v).plan.nodeArrivalMin.size()) {
        arrTime = veh(v).plan.nodeArrivalMin[veh(v).nodeIndex];
    }
    arriveAt(v, node, arrTime);


    // ---- 按**停靠动作集**执行状态转移（规格一 MC-E09）----
    //
    // 历史：旧实现靠 `nodeTripIndex → Trip` 反查「本趟」的 pickups/offloads/
    // reclaimStation 并按 nodeId 扣货（Trip 已退役，规格一 MC-A01）。
    // 现模型下 stops 的动作集就是真源，且**按订单记账**：
    // 同节点多单（A∪B 合并停靠）下按 nodeId 扣货会把别的单一起扣掉（issue #5 的坑）。
    const bool isStopHere = veh(v).nodeIndex < veh(v).plan.nodeIsStop.size()
                            && veh(v).plan.nodeIsStop[veh(v).nodeIndex];
    const bool cursorMatches = isStopHere && veh(v).stopCursor < veh(v).plan.stops.size()
                               && veh(v).plan.stops[veh(v).stopCursor].nodeId == veh(v).plan.nodes[veh(v).nodeIndex];
    const logistics::Stop* hereStop = cursorMatches ? &veh(v).plan.stops[veh(v).stopCursor] : nullptr;

    if (hereStop != nullptr) {
        // ① 先全部卸：送达 + 降载（停靠内先卸后装）。
        for (const logistics::StopDelivery& d : hereStop->deliveries) {
            for (std::size_t i = 0; i < veh(v).state.onboard.size(); ++i) {
                if (veh(v).state.onboard[i].orderId != d.orderId) {
                    continue;
                }
                veh(v).state.onboard.erase(veh(v).state.onboard.begin() + static_cast<std::ptrdiff_t>(i));
                break;
            }
            for (Order& o : config_.orders) {
                if (o.id == d.orderId) {
                    o.served = true;
                }
            }
            // 转运账本：送达 -> 状态「已送达」（只记已实际执行的卸入，§7.5⑥）。
            for (TransitLedgerEntry& e : transitLedger_) {
                if (e.orderId == d.orderId && !e.delivered) {
                    e.delivered = true;
                    if (e.outTimeMin < 0) {
                        e.outTimeMin = hereStop->arrivalMin;
                        e.pickedUp = true;
                    }
                }
            }
        }
        for (const std::string& orderId : hereStop->transitUnloads) {
            // 转运卸入（跨车）：货从本车挪到站，**不进降载暂存堆**；账本追加一行。
            for (std::size_t i = 0; i < veh(v).state.onboard.size(); ++i) {
                if (veh(v).state.onboard[i].orderId != orderId) {
                    continue;
                }
                veh(v).state.onboard.erase(veh(v).state.onboard.begin()
                                           + static_cast<std::ptrdiff_t>(i));
                break;
            }
            const logistics::TransitLeg* planned = nullptr;
            for (const logistics::TransitLeg& leg : fleetPlan_.transitLegs) {
                if (leg.orderId == orderId && leg.stationId == hereStop->nodeId
                    && leg.inVehicleId == vehicle.id) {
                    planned = &leg;
                    break;
                }
            }
            TransitLedgerEntry e;
            e.orderId = orderId;
            e.stationId = hereStop->nodeId;
            e.kg = orderKg(orderId);
            e.inVehicleId = vehicle.id;
            e.inTimeMin = hereStop->arrivalMin;
            e.outVehicleId = (planned != nullptr) ? planned->outVehicleId : std::string();
            e.outTimeMin = -1;
            e.pickedUp = false;
            e.delivered = false;
            transitLedger_.push_back(e);
            appendLog(QStringLiteral("转运卸入：在 %1 卸下订单 %2 的货 %3kg（等待 %4 取走）")
                          .arg(QString::fromStdString(hereStop->nodeId))
                          .arg(QString::fromStdString(orderId))
                          .arg(e.kg, 0, 'f', 0)
                          .arg(planned != nullptr
                                   ? QString::fromStdString(planned->outVehicleId)
                                   : QStringLiteral("另一台车")));
        }
        for (const std::string& orderId : hereStop->offloads) {
            VehicleState::OffloadKey key;
            key.station = hereStop->nodeId;
            key.orderId = orderId;
            if (!veh(v).state.appliedOffloads.insert(key).second) {
                continue;   // 物理事实只发生一次
            }
            bool removed = false;
            for (std::size_t i = 0; i < veh(v).state.onboard.size(); ++i) {
                if (veh(v).state.onboard[i].orderId != orderId) {
                    continue;
                }
                veh(v).state.stashItems.push_back(veh(v).state.onboard[i]);
                veh(v).state.onboard.erase(veh(v).state.onboard.begin() + static_cast<std::ptrdiff_t>(i));
                removed = true;
                break;
            }
            veh(v).state.stashStation = hereStop->nodeId;
            appendLog(QStringLiteral("降载：在 %1 卸下订单 %2 的货 %3kg（暂存，稍后取回）%4")
                          .arg(QString::fromStdString(hereStop->nodeId))
                          .arg(QString::fromStdString(orderId))
                          .arg(orderKg(orderId), 0, 'f', 0)
                          .arg(removed ? QString() : QStringLiteral("（车上无此货，仅记账）")));
        }
        veh(v).state.loadKg = veh(v).state.sumOnboard();

        // ② 再全部装：取货 + 取回。
        for (const std::string& orderId : hereStop->pickups) {
            if (!veh(v).state.pickedOrderIds.insert(orderId).second) {
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
                veh(v).state.onboard.push_back(item);
                appendLog(QStringLiteral("取货 %1：在 %2 装上包裹 %3kg（送往 %4）")
                              .arg(QString::fromStdString(orderId))
                              .arg(QString::fromStdString(hereStop->nodeId))
                              .arg(target->demandKg, 0, 'f', 0)
                              .arg(QString::fromStdString(target->nodeId)));
            }
        }
        for (const std::string& orderId : hereStop->reclaims) {
            for (std::size_t i = 0; i < veh(v).state.stashItems.size(); ++i) {
                if (veh(v).state.stashItems[i].orderId != orderId) {
                    continue;
                }
                veh(v).state.onboard.push_back(veh(v).state.stashItems[i]);
                appendLog(QStringLiteral("取回暂存货：在 %1 取回订单 %2 的货 %3kg，继续配送")
                              .arg(QString::fromStdString(hereStop->nodeId))
                              .arg(QString::fromStdString(orderId))
                              .arg(veh(v).state.stashItems[i].kg, 0, 'f', 0));
                veh(v).state.stashItems.erase(veh(v).state.stashItems.begin()
                                        + static_cast<std::ptrdiff_t>(i));
                break;
            }
        }
        for (const std::string& orderId : hereStop->transitPickups) {
            // 转运取走（跨车）：货从站上装到本车；账本只补「取走」两列与状态。
            veh(v).state.pickedOrderIds.insert(orderId);
            const Order* target = nullptr;
            for (const Order& o : config_.orders) {
                if (o.id == orderId) {
                    target = &o;
                }
            }
            if (target != nullptr) {
                logistics::OnboardItem item;
                item.nodeId = target->nodeId;
                item.kg = target->demandKg;
                item.orderId = target->id;
                veh(v).state.onboard.push_back(item);
                for (Order& o : config_.orders) {
                    if (o.id == orderId) {
                        o.picked = true;
                    }
                }
            }
            bool recorded = false;
            for (TransitLedgerEntry& e : transitLedger_) {
                if (e.orderId != orderId || e.pickedUp) {
                    continue;
                }
                e.pickedUp = true;
                e.outVehicleId = vehicle.id;
                e.outTimeMin = hereStop->arrivalMin;
                recorded = true;
            }
            appendLog(QStringLiteral("转运取走：在 %1 取走订单 %2 的货 %3kg，继续配送%4")
                          .arg(QString::fromStdString(hereStop->nodeId))
                          .arg(QString::fromStdString(orderId))
                          .arg(orderKg(orderId), 0, 'f', 0)
                          .arg(recorded ? QString() : QStringLiteral("（账本无对应卸入）")));
        }
        if (veh(v).state.stashItems.empty()) {
            veh(v).state.stashStation.clear();
        }
        veh(v).state.loadKg = veh(v).state.sumOnboard();
    }

    // 只有**下标与节点对得上**时才认作一次停靠送达。
    // 曾经这里只看 nodeIsStop 与下标，重规划后两者可能失同步，
    // 于是拿到的是**另一个停靠点**：扣错货（车上有的没扣、没有的扣了），
    // 表现为"2 趟送完 740kg"这种物理上不可能的结果（**已退役口径**：
    // 旧「趟」模型下的历史事故举例，现行单条连续路线不存在「趟」）。
    if (hereStop != nullptr) {
        const Stop& stop = *hereStop;
        ++veh(v).stopCursor;
        const bool transitOnly = stop.pickups.empty() && stop.deliveries.empty();
        if (transitOnly) {
            QString actions;
            const auto addAction = [&actions](const QString& text) {
                if (text.isEmpty()) {
                    return;
                }
                if (!actions.isEmpty()) {
                    actions += QStringLiteral(" ");
                }
                actions += text;
            };
            addAction(stop.transitUnloads.empty()
                          ? QString()
                          : QStringLiteral("转运卸入 %1 张单").arg(stop.transitUnloads.size()));
            addAction(stop.transitPickups.empty()
                          ? QString()
                          : QStringLiteral("转运取走 %1 张单").arg(stop.transitPickups.size()));
            addAction(stop.offloads.empty()
                          ? QString()
                          : QStringLiteral("降载 %1 张单").arg(stop.offloads.size()));
            addAction(stop.reclaims.empty()
                          ? QString()
                          : QStringLiteral("取回 %1 张单").arg(stop.reclaims.size()));
            appendLog(QStringLiteral("中转站 %1（到达 %2）：%3%4")
                          .arg(QString::fromStdString(stop.nodeId))
                          .arg(minutesToClock(stop.arrivalMin))
                          .arg(actions)
                          .arg(stop.transitUnloads.empty() && stop.transitPickups.empty()
                                   ? QString()
                                   : QStringLiteral("（跨车转运，零耗时）")));
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
            const double cap = vehicle.capacityKg;
            recordRun(QStringLiteral("    └ 到达 %1 时车上载重：订单货 %2 kg"
                                     "（载重上限 %3 kg）")
                          .arg(QString::fromStdString(stop.nodeId))
                          .arg(veh(v).state.loadKg, 0, 'f', 0)
                          .arg(cap, 0, 'f', 0));
        }
        deliverAt(v, stop);
    } else if (veh(v).nodeIndex + 1 == veh(v).plan.nodes.size()) {
        appendLog(QStringLiteral("归位 %1（到达 %2），本次配送结束")
                      .arg(QString::fromStdString(veh(v).state.atNodeId))
                      .arg(minutesToClock(veh(v).state.atTimeMin)));
    } else {
        appendLog(QStringLiteral("经过 %1（到达 %2）")
                      .arg(QString::fromStdString(veh(v).state.atNodeId))
                      .arg(minutesToClock(veh(v).state.atTimeMin)));
    }

    // 画布/面板的刷新由 advanceFleetTo 在整批推进结束后统一做一次。
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

// 「推进一站」= 走到**全队**下一到达时刻；「推进一刻」= 走到下一事件刻。
// 两条路径都只经由 advanceFleetTo 改状态（规格二 §6.1）。
void MainWindow::onAdvanceStop() {
    if (atRouteEnd()) {
        reportRouteFinishedOnce();
        return;
    }
    advanceFleetTo(nextFleetArrival());
    if (atRouteEnd()) {
        reportRouteFinishedOnce();
    }
}

void MainWindow::onAdvanceMoment() {
    if (atRouteEnd()) {
        reportRouteFinishedOnce();
        return;
    }
    advanceFleetTo(nextEventMark_);
    if (atRouteEnd()) {
        reportRouteFinishedOnce();
    }
}

// ---- 单一车队时钟原语（规格二 §6.1 的最小版；五级升级链归 #42）----
//
//   repeat:
//       Tnext = min over 车 v of (v 的下一节点到达时刻，若存在)
//       if 无或 Tnext > T: break
//       所有「下一节点到达时刻 == Tnext」的车一起 onAdvanceStopFor(v)
//       settleEventsUpTo(Tnext)        // 到达前逐刻、各自时刻结算（#34 定稿）
//
// 同刻多车一起推进、然后才结算事件 —— 这消掉了「谁先谁后」的歧义（§6.1）。
void MainWindow::advanceFleetTo(int targetMin) {
    if (fleet_.empty()) {
        return;
    }
    bool tailOnly = false;
    while (!tailOnly) {
        int tNext = nextFleetArrival();
        if (tNext == std::numeric_limits<int>::max() || tNext > targetMin) {
            break;
        }
        // ---- 到达**之前**的事件刻：逐个在它自己的时刻结算 ----
        // 结算时车仍在所在路段的起点（半路显示语义），与「推进一刻」逐步走刻同构。
        // 这是 A1–A7 等价性成立的前提（#34 §4.7⑤ 的严格版）。
        while (nextEventMark_ < tNext) {
            advanceClockTo(nextEventMark_);
            settleEventsUpTo(nextEventMark_);
            tNext = nextFleetArrival();
            if (tNext == std::numeric_limits<int>::max() || tNext > targetMin) {
                tailOnly = true;
                break;
            }
        }
        if (tailOnly) {
            break;
        }
        // ---- 同刻到达的所有车一起推进（消掉「谁先谁后」的歧义，§6.1）----
        for (std::size_t v = 0; v < fleet_.size(); ++v) {
            if (vehNextArrival(v) == tNext) {
                onAdvanceStopFor(v);
            }
        }
        int clock = fleetClockMin_;
        for (const VehicleRuntime& rt : fleet_) {
            if (rt.state.atTimeMin > clock) {
                clock = rt.state.atTimeMin;
            }
        }
        fleetClockMin_ = clock;
        // 恰好落在**到达时刻**上的事件刻：到达之后结算（与「推进一刻」同位置同时刻）。
        settleEventsUpTo(fleetClockMin_);
        if (atRouteEnd()) {
            break;
        }
    }
    // ---- 剩余的事件刻（车卡在 i→i+1 途中，位置留在 i）----
    //
    // **不能挂 `!atRouteEnd()`（规格四 #54 修的等价性缺陷）**：那样「车队已走完」时
    // 这一段就被整个跳过，targetMin 之前剩余的事件刻**不被结算**；而「推进一刻」路径
    // 每刻都结算。两条路径于是在同一时刻对「待处理额度」看到不同状态 ——
    // A 还能抽到普通订单、B 已显示「待处理已满 6」，事件序列从那一行起分叉。
    // 默认数据在规格四之前只有 34 个事件刻、且完工时刻正好落在某个到达上，
    // 跳过的刻数是 0，缺陷看不见；跨城尺度把事件刻推到 46 个、完工时刻不落在到达上，
    // 跳过的刻就非 0 了（实测 A 1958 步 / B 1019 步、时钟 495:30 vs 589:12）。
    // 正确语义：**结算到 targetMin 为止的所有事件刻**，与车队是否走完无关 ——
    // 事件刻由软件时钟驱动（D24），不该因为"车都回库了"就漏结算。
    while (nextEventMark_ <= targetMin) {
        advanceClockTo(nextEventMark_);
        settleEventsUpTo(nextEventMark_);
    }
    if (!atRouteEnd() && fleetClockMin_ < targetMin) {
        advanceClockTo(targetMin);
    }
    syncScene();
    updatePanels();
}

bool MainWindow::atRouteEnd() const {
    for (std::size_t v = 0; v < fleet_.size(); ++v) {
        if (!vehAtEnd(v)) {
            return false;
        }
    }
    return true;   // 全部车都走完（或没有车）
}

// 「走完了」与「计划不可行/为空」是两件事：后者也满足 atRouteEnd()，
// 但把它当成"配送完成"会播报假成功（界面上同时写着「规划不可行」）。
bool MainWindow::routeTrulyFinished() const {
    if (fleet_.empty()) {
        return false;
    }
    for (const VehicleRuntime& rt : fleet_) {
        if (rt.plan.status != logistics::PlanStatus::Ok || rt.plan.nodes.empty()
            || rt.nodeIndex + 1 < rt.plan.nodes.size()) {
            return false;
        }
    }
    return true;
}

void MainWindow::reportRouteFinishedOnce() {
    if (routeFinishedReported_) {
        return;   // 终点只播报一次，消除原来"每个 tick 都刷一遍"的噪音
    }
    routeFinishedReported_ = true;
    QStringList homes;
    for (std::size_t v = 0; v < fleet_.size(); ++v) {
        homes << QStringLiteral("%1@%2")
                     .arg(QString::fromStdString(config_.vehicles[v].id))
                     .arg(minutesToClock(veh(v).state.atTimeMin));
    }
    appendLog(QStringLiteral("本次配送已完成：全部车辆已回到仓库（%1）")
                  .arg(homes.join(QStringLiteral("、"))));
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
    if (timeMin <= fleetClockMin_) {
        return;   // 时间不倒退
    }
    fleetClockMin_ = timeMin;
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

// 等价性守卫 A2–A7 的终态快照（**车队口径**，规格二 §6.10）：
//   A2 已送达订单集合（按车分组）   A3 各车已发生 penalty   A4 各车在车货
//   A5 站内已执行账本 + 各车物理账本   A6 各车位置与时刻   A7 nextEventMark_
// 全部读与界面上同一份物理事实（VehicleState / 转运账本 / 车队时钟），
// 刻意**不调用 nodeIndexAtTime** —— 那是显示函数，拿它反推等于自证。
QString MainWindow::equivalenceSnapshot() const {
    QString out;
    for (std::size_t v = 0; v < fleet_.size(); ++v) {
        const VehicleRuntime& rt = fleet_[v];
        const QString id = QString::fromStdString(config_.vehicles[v].id);
        // A2 该车已送达订单集合（按车分组；物理事实，不从计划反推）
        QStringList served;
        for (const std::string& orderId : rt.state.deliveredOrderIds) {
            served << QString::fromStdString(orderId);
        }
        out += QStringLiteral("A2 %1 已送达: %2\n").arg(id).arg(served.join(QLatin1Char(',')));
        // A3 该车已发生 penalty（含逐单已发生超时明细）
        out += QStringLiteral("A3 %1 已发生 penalty: %2 min\n")
                   .arg(id)
                   .arg(rt.state.incurredPenaltyMin);
        QStringList lateList;
        for (const VehicleState::LateStop& late : rt.state.deliveredLate) {
            lateList << QStringLiteral("%1@%2:+%3")
                            .arg(QString::fromStdString(late.orderId))
                            .arg(QString::fromStdString(late.nodeId))
                            .arg(late.penaltyMin);
        }
        out += QStringLiteral("A3 %1 已发生超时明细: %2\n").arg(id).arg(lateList.join(QLatin1Char(',')));
        // A4 该车在车货（订单 + kg）
        QStringList onboard;
        for (const logistics::OnboardItem& item : rt.state.onboard) {
            onboard << QStringLiteral("%1:%2kg@%3")
                           .arg(QString::fromStdString(item.orderId))
                           .arg(item.kg, 0, 'f', 0)
                           .arg(QString::fromStdString(item.nodeId));
        }
        out += QStringLiteral("A4 %1 在车货: %2（合计 %3kg / 载重 %4kg）\n")
                   .arg(id)
                   .arg(onboard.join(QLatin1Char(',')))
                   .arg(rt.state.sumOnboard(), 0, 'f', 0)
                   .arg(rt.state.loadKg, 0, 'f', 0);
        // A5 该车的物理账本（已送达停靠数 / 已取货 / 已降载 / 暂存站）
        QStringList picked;
        for (const std::string& orderId : rt.state.pickedOrderIds) {
            picked << QString::fromStdString(orderId);
        }
        QStringList offloaded;
        for (const VehicleState::OffloadKey& key : rt.state.appliedOffloads) {
            offloaded << (QString::fromStdString(key.station) + QLatin1Char('/')
                          + QString::fromStdString(key.orderId));
        }
        QStringList stash;
        for (const logistics::OnboardItem& item : rt.state.stashItems) {
            stash << QStringLiteral("%1:%2kg").arg(QString::fromStdString(item.orderId))
                         .arg(item.kg, 0, 'f', 0);
        }
        out += QStringLiteral("A5 %1 已送达停靠 %2 ｜ 已取货 [%3] ｜ 已降载 [%4] ｜ 暂存站 %5 [%6]\n")
                   .arg(id)
                   .arg(rt.state.servedStops)
                   .arg(picked.join(QLatin1Char(',')))
                   .arg(offloaded.join(QLatin1Char(',')))
                   .arg(QString::fromStdString(rt.state.stashStation))
                   .arg(stash.join(QLatin1Char(',')));
        // A6 该车位置与时刻
        out += QStringLiteral("A6 %1 位置 %2 @ %3\n")
                   .arg(id)
                   .arg(QString::fromStdString(rt.state.atNodeId))
                   .arg(minutesToClock(rt.state.atTimeMin));
        // A6 的另一半（路线面板的数据源）：该车当前计划的节点序列。
        // 终态"全部都到家、全部送达"会让位置/时刻收敛，光看这两项区分不出
        // 「同刻并列时先推进哪一台」——把路线也纳入快照，等价性才是逐字的。
        QStringList routeNodes;
        for (std::size_t i = 0; i < rt.plan.nodes.size(); ++i) {
            routeNodes << QString::fromStdString(rt.plan.nodes[i]);
        }
        out += QStringLiteral("A6 %1 路线: %2（停靠 %3 ｜ 剩余 %4）\n")
                   .arg(id)
                   .arg(routeNodes.join(QLatin1Char('>')))
                   .arg(rt.plan.stops.size())
                   .arg(rt.stopCursor);
    }
    // A5（站内已执行账本）：只含**已实际执行**的卸入（计划期候选不在其中）——
    // 订单 / 站 / kg / 卸入车 / 卸入时刻 / 取走车 / 取走时刻（规格二 §6.10 的 A5）。
    QStringList inStation;
    for (const TransitLedgerEntry& entry : transitLedger_) {
        inStation << QStringLiteral("%1@%2:%3kg 卸入%4@%5 取走%6@%7")
                         .arg(QString::fromStdString(entry.orderId))
                         .arg(QString::fromStdString(entry.stationId))
                         .arg(entry.kg, 0, 'f', 0)
                         .arg(QString::fromStdString(entry.inVehicleId))
                         .arg(entry.inTimeMin >= 0 ? minutesToClock(entry.inTimeMin)
                                                   : QStringLiteral("-"))
                         .arg(entry.pickedUp ? QString::fromStdString(entry.outVehicleId)
                                             : QStringLiteral("-"))
                         .arg(entry.outTimeMin >= 0 ? minutesToClock(entry.outTimeMin)
                                                    : QStringLiteral("-"));
    }
    out += QStringLiteral("A5 站内已执行账本: %1\n").arg(inStation.join(QLatin1Char(';')));
    // A7 车队时钟（全局唯一的下一事件刻）+ 全队当前时钟
    out += QStringLiteral("A7 下一个事件刻 %1 ｜ 车队时钟 %2\n")
               .arg(minutesToClock(nextEventMark_))
               .arg(minutesToClock(fleetClockMin_));
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

    // ---- 路线信息面板（规格二 §7.5①）：车队汇总 + `── 逐车指标 ──` 小节 ----
    // 车队口径：Σ 距离/成本/penalty/停靠/动作；总耗时 = **完工耗时**（主口径），
    // 车时合计作为参考口径并列显示（§3.4：二者不得互相顶替）。
    QString route;
    {
        // 已发生的 penalty / 已送达 是**物理事实**（跨重规划累计），从各车状态汇总。
        int incurred = 0;
        int servedStops = 0;
        int remainingStops = 0;
        int actions = 0;
        int plannedPenalty = 0;
        double distance = 0.0;
        double cost = 0.0;
        for (std::size_t v = 0; v < fleet_.size(); ++v) {
            const VehicleRuntime& rt = fleet_[v];
            incurred += rt.state.incurredPenaltyMin;
            servedStops += rt.state.servedStops;
            const std::size_t from =
                (rt.stopCursor < rt.plan.stops.size()) ? rt.stopCursor : rt.plan.stops.size();
            remainingStops += static_cast<int>(rt.plan.stops.size() - from);
            for (const Stop& st : rt.plan.stops) {
                actions += static_cast<int>(st.pickups.size() + st.deliveries.size());
            }
            if (rt.plan.status == logistics::PlanStatus::Ok) {
                distance += rt.plan.totalDistanceKm;
                cost += rt.plan.totalCostYuan;
                plannedPenalty += rt.plan.totalPenaltyMin;
            }
        }
        const logistics::RouteMetrics& fm = fleetPlan_.metrics;
        route += QStringLiteral("策略：%1\n").arg(strategyBox_ != nullptr
                                                      ? strategyBox_->currentText()
                                                      : QString());
        route += QStringLiteral("总距离：%1 km\n").arg(distance, 0, 'f', 1);
        // 总耗时 = 完工耗时（max 返回 − min 发车）；括号里是「返回时刻」的钟点。
        route += QStringLiteral("总耗时：%1 min（完工耗时；%2 收工）\n")
                     .arg(fleetPlan_.completionTimeMin, 0, 'f', 1)
                     .arg(minutesToClock(static_cast<int>(
                         std::llround(minDepartMin() + fleetPlan_.completionTimeMin))));
        route += QStringLiteral("总成本：%1 元\n").arg(cost, 0, 'f', 1);
        route += QStringLiteral("总 penalty：%1 min（已发生 %2 + 剩余计划 %3）\n")
                     .arg(incurred + plannedPenalty)
                     .arg(incurred)
                     .arg(plannedPenalty);
        route += QStringLiteral("停靠 %1 站 · 动作 %2 次 · 已送达 %3 站\n")
                     .arg(servedStops + remainingStops)
                     .arg(actions)
                     .arg(servedStops);
        route += QStringLiteral("取货准时率：%1\n")
                     .arg(onTimeRateText(fm.pickupOnTimeCount, fm.pickupTotal));
        route += QStringLiteral("送达准时率：%1（紧急 %2 · 普通 %3）\n")
                     .arg(onTimeRateText(fm.deliveryOnTimeCount, fm.deliveryTotal))
                     .arg(onTimeRateText(fm.urgentOnTimeCount, fm.urgentTotal))
                     .arg(onTimeRateText(fm.normalOnTimeCount, fm.normalTotal));
        route += QStringLiteral("平均在车：%1 kg（利用率 %2% · 峰值 %3 kg）\n")
                     .arg(fm.avgLoadKg, 0, 'f', 1)
                     .arg(fm.loadUtilization * 100.0, 0, 'f', 1)
                     .arg(fm.peakLoadKg, 0, 'f', 1);
        // ---- `── 逐车指标 ──`：每车一行 7 项（§7.2：距离/耗时/成本/penalty/
        // 利用率/取货率/送达率）。这是**计划级**结果，与车辆表里的运行态不同源。
        route += QStringLiteral("── 逐车指标 ──\n");
        for (std::size_t v = 0; v < fleet_.size() && v < config_.vehicles.size(); ++v) {
            const VehicleRuntime& rt = fleet_[v];
            const logistics::Vehicle& vehicle = config_.vehicles[v];
            const logistics::RouteMetrics& vm = rt.plan.metrics;
            route += QStringLiteral("%1  距离 %2 km · 耗时 %3 min · 成本 %4 元 · "
                                    "penalty %5 · 停靠 %6 · 利用率 %7% · 取货 %8 · 送达 %9\n")
                         .arg(QString::fromStdString(vehicle.id))
                         .arg(rt.plan.totalDistanceKm, 0, 'f', 1)
                         .arg(rt.plan.totalTimeMin, 0, 'f', 1)
                         .arg(rt.plan.totalCostYuan, 0, 'f', 1)
                         .arg(rt.plan.totalPenaltyMin)
                         .arg(vm.stopCount)
                         .arg(vm.loadUtilization * 100.0, 0, 'f', 1)
                         .arg(onTimeRateText(vm.pickupOnTimeCount, vm.pickupTotal))
                         .arg(onTimeRateText(vm.deliveryOnTimeCount, vm.deliveryTotal));
        }
        // 均衡度（规格二 §4.3 / #40）：峰值利用率极差，越小越均衡。**纯增量一行**，
        // 不新增控件；分母按全部车（空车 utilization 记 0）。计划级结果，与车辆表运行态不同源。
        route += QStringLiteral("均衡度（峰值利用率极差）：%1\n")
                     .arg(fleetPlan_.balanceSpread, 0, 'f', 2);
        if (!fleetPlan_.allOk) {
            route += QStringLiteral("不可行车辆：%1\n")
                         .arg(QString::fromStdString(fleetPlan_.reason));
        }
    }
    routeInfo_->setPlainText(route);

    // ---- 车辆表（#56 重设计）：一车一行，取代「每车一个页签」 ----
    // 列：车辆 | 状态 | 载货(kg) | 额定(kg) | 利用率 | 在途单数 | 位置 | 下一站。
    // 每行 tooltip 里保留固定参数（ID / 起始仓库 / 载重上限 / 发车时刻 / 两因子）
    // 与面板不再单列的运行细节（阶段 / 已送达 / 剩余待送 / 已发生 penalty）——
    // 信息没丢，只是从占版面的标签块搬到了悬停提示里。
    bool vehicleSelectionChanged = false;
    if (vehicleTable_ != nullptr) {
        vehicleTable_->setRowCount(static_cast<int>(config_.vehicles.size()));
        for (std::size_t vi = 0; vi < config_.vehicles.size(); ++vi) {
            const logistics::Vehicle& vehicle = config_.vehicles[vi];
            const VehicleRuntime& rt = veh(vi);
            // 状态 = 空闲 / 配送中 / 已收工。
            // 为什么不再沿用页签里的「紧急 / 普通」：那两项是**计划阶段**，而这张表
            // 回答的是"这台车现在忙不忙"；阶段仍留在 tooltip 里，审计时照样看得到。
            QString statusText;
            if (rt.plan.status != logistics::PlanStatus::Ok
                || rt.stopCursor >= rt.plan.stops.size()) {
                statusText = QStringLiteral("已收工");
            } else if (!rt.state.onboard.empty() || !rt.state.pickedOrderIds.empty()) {
                statusText = QStringLiteral("配送中");
            } else {
                statusText = QStringLiteral("空闲");
            }
            const double cap = vehicle.capacityKg;
            const double load = rt.state.loadKg;   // 物理事实，不从计划反推
            const QString utilText =
                cap > 0.0 ? QStringLiteral("%1%").arg(100.0 * load / cap, 0, 'f', 1)
                          : QStringLiteral("—");
            const std::string nextNode = nextStopNodeId(vi);
            const QString here = QString::fromStdString(rt.state.atNodeId);
            const QStringList cells = {
                QString::fromStdString(vehicle.id),
                statusText,
                // 载货 / 额定 一格并排：一眼能看出「装了多少 / 能装多少」。
                QStringLiteral("%1/%2").arg(load, 0, 'f', 1).arg(cap, 0, 'f', 0),
                utilText,
                QString::number(rt.state.onboard.size()),
                nextNode.empty()
                    ? (rt.plan.homeNodeId.empty()
                           ? here
                           : here + QStringLiteral("（已归位 %1）")
                                        .arg(QString::fromStdString(rt.plan.homeNodeId)))
                    // 规格七 #63：慢车出生在本簇站，下一次停靠可能就在脚下（在站里等货）。
                    // 「T03→T03」是一支零长度箭头，写成「原地待命」才如实。
                    : (nextNode == rt.state.atNodeId
                           ? here + QStringLiteral("（原地待命）")
                           : here + QStringLiteral("→") + QString::fromStdString(nextNode))};
            const QString tooltip =
                QStringLiteral("固定信息\nID：%1\n起始节点：%2\n载重上限：%3 kg\n发车时刻：%4\n"
                               "耗时倍率：%5\n成本倍率：%6\n运行状态\n阶段：%7\n"
                               "已送达 %8 单 · 剩余待送 %9 kg\n已发生 penalty：%10 min")
                    .arg(QString::fromStdString(vehicle.id))
                    .arg(QString::fromStdString(vehicle.startNodeId))
                    .arg(vehicle.capacityKg, 0, 'f', 0)
                    .arg(minutesToClock(vehicle.departTimeMin))
                    .arg(vehicle.timeFactor, 0, 'f', 2)
                    .arg(vehicle.costFactor, 0, 'f', 2)
                    .arg(vehiclePhaseText(vi))
                    .arg(vehicleServedOrderCount(vi))
                    .arg(vehicleRemainingDemandKg(vi), 0, 'f', 0)
                    .arg(rt.state.incurredPenaltyMin);
            for (int col = 0; col < cells.size(); ++col) {
                QTableWidgetItem* item = new QTableWidgetItem(cells[col]);
                item->setToolTip(tooltip);
                item->setFlags(item->flags() & ~Qt::ItemIsEditable);
                vehicleTable_->setItem(static_cast<int>(vi), col, item);
            }
        }
        // 默认落点：选中行应当**有路线可看**（与旧页签同一策略）。
        // 职责封闭（#46）之后行序 0 的快车在默认数据上恒为空车 —— 若把选中停在它身上，
        // 画布上既没有加粗边、也没有可核对的信息（--ui-probe 的「选中车加粗」会落空）。
        // 因此只在「当前选中行没有路线、而别的车有活」时改选第一台有路线的车。
        if (!config_.vehicles.empty()) {
            const int cur = vehicleTable_->currentRow();
            bool curHasRoute = false;
            if (cur >= 0 && static_cast<std::size_t>(cur) < fleet_.size()) {
                curHasRoute = fleet_[static_cast<std::size_t>(cur)].plan.status
                                  == logistics::PlanStatus::Ok
                              && !fleet_[static_cast<std::size_t>(cur)].plan.stops.empty();
            }
            if (!curHasRoute) {
                for (std::size_t vi = 0; vi < fleet_.size(); ++vi) {
                    if (fleet_[vi].plan.status == logistics::PlanStatus::Ok
                        && !fleet_[vi].plan.stops.empty()) {
                        vehicleTable_->setCurrentCell(static_cast<int>(vi), 0);
                        vehicleSelectionChanged = true;
                        break;
                    }
                }
            }
        }
    }

    // ---- 订单「当前位置」与「车辆」两列共用的事实源：货现在在谁手里 ----
    // 「站内」有两种物理来源，必须都算：
    //   ① 转运账本里已卸入、未取走（跨车接力，货在站上等下一台车）；
    //   ② 本车降载暂存 stashItems（同车卸、同车取回）。
    // 两者都只读**已执行**的状态，不做任何计划期推断。
    const auto onboardVehicleOf = [this](const std::string& orderId) -> QString {
        for (std::size_t v = 0; v < fleet_.size(); ++v) {
            for (const logistics::OnboardItem& it : fleet_[v].state.onboard) {
                if (it.orderId == orderId) {
                    return QString::fromStdString(config_.vehicles[v].id);
                }
            }
        }
        return QString();
    };
    const auto stationOfOrder = [this](const std::string& orderId) -> QString {
        for (const TransitLedgerEntry& e : transitLedger_) {
            if (e.orderId == orderId && !e.pickedUp) {
                return QString::fromStdString(e.stationId);
            }
        }
        for (const VehicleRuntime& rt : fleet_) {
            if (rt.state.stashStation.empty()) {
                continue;
            }
            for (const logistics::OnboardItem& it : rt.state.stashItems) {
                if (it.orderId == orderId) {
                    return QString::fromStdString(rt.state.stashStation);
                }
            }
        }
        return QString();
    };
    // 订单「车辆」列：已送达 -> 真正送它的那台车；在车 -> 车上那台；在站 -> 账本里的
    // 取走车（计划中的接力车）；否则看计划里谁排到了它（多台 = 接力，不谎报单台）。
    const auto vehicleTextOfOrder = [this](const Order& o, const QString& onboardVeh,
                                            const QString& station) -> QString {
        if (o.served) {
            for (std::size_t v = 0; v < fleet_.size(); ++v) {
                if (fleet_[v].state.deliveredOrderIds.count(o.id) > 0) {
                    return QString::fromStdString(config_.vehicles[v].id);
                }
            }
            return QStringLiteral("—");
        }
        if (!onboardVeh.isEmpty()) {
            return onboardVeh;
        }
        if (!station.isEmpty()) {
            for (const TransitLedgerEntry& e : transitLedger_) {
                if (e.orderId == o.id && !e.pickedUp && !e.outVehicleId.empty()) {
                    return QString::fromStdString(e.outVehicleId);
                }
            }
            return QStringLiteral("—");
        }
        QStringList owners;
        for (std::size_t v = 0; v < fleet_.size(); ++v) {
            bool hit = false;
            for (const Stop& s : fleet_[v].plan.stops) {
                for (const std::string& pid : s.pickups) {
                    if (pid == o.id) {
                        hit = true;
                    }
                }
                for (const logistics::StopDelivery& d : s.deliveries) {
                    if (d.orderId == o.id) {
                        hit = true;
                    }
                }
            }
            if (hit) {
                owners << QString::fromStdString(config_.vehicles[v].id);
            }
        }
        if (owners.size() == 1) {
            return owners.front();
        }
        if (owners.size() > 1) {
            return QStringLiteral("接力");
        }
        return QStringLiteral("—");
    };

    // ---- 订单表（#56）：**一单一行 8 列**，超时行排最前 ----
    // 每张单的超时分钟数 = 已发生（deliveredLate，物理事实）+ 剩余计划预计（逐单 late），
    // 按订单累加。这取代了原来的「超时订单」独立窗口：同一张单只在一处出现，
    // 状态列内联「超时 +Nmin」并把这些行排到表头 —— 不切窗也第一眼可见。
    std::map<std::string, int> latePenaltyMin;
    for (const VehicleRuntime& rt : fleet_) {
        for (const VehicleState::LateStop& ls : rt.state.deliveredLate) {
            latePenaltyMin[ls.orderId] += ls.penaltyMin;
        }
        if (rt.plan.status != logistics::PlanStatus::Ok) {
            continue;
        }
        for (std::size_t k = rt.stopCursor; k < rt.plan.stops.size(); ++k) {
            for (const logistics::StopDelivery& d : rt.plan.stops[k].deliveries) {
                if (d.late) {
                    latePenaltyMin[d.orderId] += d.penaltyMin;
                }
            }
        }
    }
    // 排序：超时 → 未送达紧急 → 未送达普通 → 已送达。
    // 超时排最前是票面要求：用户第一眼要知道"哪张单出问题了"。
    // 新插入的紧急订单也仍会立刻出现在前面（不必滚动到表尾去找）。
    std::vector<std::size_t> order;
    for (std::size_t pass = 0; pass < 4; ++pass) {
        for (std::size_t i = 0; i < config_.orders.size(); ++i) {
            const Order& o = config_.orders[i];
            const bool late = latePenaltyMin.find(o.id) != latePenaltyMin.end();
            const bool wanted = (pass == 0) ? late
                                : (pass == 1) ? (!late && !o.served && o.urgent)
                                : (pass == 2) ? (!late && !o.served && !o.urgent)
                                              : (!late && o.served);
            if (wanted) {
                order.push_back(i);
            }
        }
    }

    // 某张单是否已被**任何**车取在车上（多车下按全队判）
    const auto pickedAnywhere = [this](const std::string& orderId) {
        for (const VehicleRuntime& rt : fleet_) {
            if (rt.state.pickedOrderIds.count(orderId) > 0) {
                return true;
            }
        }
        return false;
    };
    orderTable_->setRowCount(static_cast<int>(order.size()));
    for (int row = 0; row < static_cast<int>(order.size()); ++row) {
        const Order& o = config_.orders[order[static_cast<std::size_t>(row)]];
        const QString onboardVeh = onboardVehicleOf(o.id);
        const QString station = stationOfOrder(o.id);
        QString status;
        if (!o.served && !station.isEmpty()) {
            // 站内待取走：货已卸进站、还没被下一台车取走（跨簇接力的中间态）。
            // 与「站内待取走(kg)」列是同一事实的两个视角（单 / 公斤）。
            status = QStringLiteral("站内待取走");
        } else if (o.served) {
            status = QStringLiteral("已送达");
        } else if (o.urgent) {
            status = pickedAnywhere(o.id) ? QStringLiteral("执行中") : QStringLiteral("紧急");
        } else {
            status = pickedAnywhere(o.id) ? QStringLiteral("配送中") : QStringLiteral("待配送");
        }
        const std::map<std::string, int>::const_iterator lateIt = latePenaltyMin.find(o.id);
        if (lateIt != latePenaltyMin.end()) {
            status += QStringLiteral(" · 超时 +%1min").arg(lateIt->second);
        }
        QString location;
        if (o.served) {
            location = QStringLiteral("已送达(%1)").arg(QString::fromStdString(o.nodeId));
        } else if (!onboardVeh.isEmpty()) {
            location = QStringLiteral("车上(%1)").arg(onboardVeh);
        } else if (!station.isEmpty()) {
            location = QStringLiteral("站内·%1").arg(station);
        } else {
            location = QStringLiteral("待取·%1")
                           .arg(QString::fromStdString(o.pickupNodeId.empty() ? o.nodeId
                                                                              : o.pickupNodeId));
        }
        orderTable_->setItem(row, 0, new QTableWidgetItem(QString::fromStdString(o.id)));
        // 取送合成一格：`A→B`；取货点缺失时只显示送货点（数据里没有这种单，但不伪造）。
        const QString pickText = o.pickupNodeId.empty()
                                     ? QStringLiteral("—")
                                     : QString::fromStdString(o.pickupNodeId);
        orderTable_->setItem(row, 1,
                             new QTableWidgetItem(pickText + QStringLiteral("→")
                                                  + QString::fromStdString(o.nodeId)));
        orderTable_->setItem(row, 2, new QTableWidgetItem(QString::number(o.demandKg, 'f', 0)));
        orderTable_->setItem(row, 3,
                             new QTableWidgetItem(minutesToClock(o.windowStartMin)
                                                  + QStringLiteral("-")
                                                  + minutesToClock(o.windowEndMin)));
        orderTable_->setItem(row, 4, new QTableWidgetItem(status));
        orderTable_->setItem(row, 5,
                             new QTableWidgetItem(vehicleTextOfOrder(o, onboardVeh, station)));
        orderTable_->setItem(row, 6, new QTableWidgetItem(location));
    }
    // ---- 停靠明细（§7.5③）：9 列，**逐车铺行**，行序 = 车辆行序 → 序号 ----
    // 动作列显示**计数**（取<a>·送<b>）；余货(kg) = `Stop::remainingLoadKg`。
    // 只列**尚未走过**的停靠点：推进一站与重规划效果一致（已送达的行从表里消失）。
    {
        int row = 0;
        for (std::size_t vi = 0; vi < fleet_.size() && vi < config_.vehicles.size(); ++vi) {
            const VehicleRuntime& rt = fleet_[vi];
            const logistics::Vehicle& vehicle = config_.vehicles[vi];
            const std::size_t fromStop =
                (rt.stopCursor < rt.plan.stops.size()) ? rt.stopCursor : rt.plan.stops.size();
            const int rowsForVehicle = static_cast<int>(rt.plan.stops.size() - fromStop);
            stopTable_->setRowCount(row + rowsForVehicle);
            for (int i = 0; i < rowsForVehicle; ++i) {
                const std::size_t idx = fromStop + static_cast<std::size_t>(i);
                const Stop& s = rt.plan.stops[idx];
                const QString actionText = QStringLiteral("取%1·送%2")
                                               .arg(s.pickups.size())
                                               .arg(s.deliveries.size());
                stopTable_->setItem(row, 0,
                                    new QTableWidgetItem(QString::fromStdString(vehicle.id)));
                stopTable_->setItem(row, 1, new QTableWidgetItem(QString::number(i + 1)));
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

    // ---- 中转站表（#56 起 5 列）：子网络 + 下属配送点 + 当前暂存货 + **站内待取走** ----
    // 第 4 列（当前暂存货）= 降载暂存堆 stashItems，只统计"同车卸、同车取回"的寄存；
    // 第 5 列（站内待取走）= 转运账本里 pickedUp == false 的行的 kg 之和，即"跨车接力
    // 卸进站、还没被下一台车取走"的货。两列必须分开：转运卸入**故意**不进 stashItems
    // （见 onAdvanceStopFor 的注释），合并成一列会让跨簇接力期间站内的货凭空消失。
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
        // 物理事实：降载暂存**按车**记（同一站可能有两台车各存一堆），故这里求和。
        transitTable_->setRowCount(static_cast<int>(rows.size()));
        for (int i = 0; i < static_cast<int>(rows.size()); ++i) {
            double stashKg = 0.0;
            for (const VehicleRuntime& rt : fleet_) {
                if (rt.state.stashStation == rows[i].id) {
                    for (const logistics::OnboardItem& it : rt.state.stashItems) {
                        stashKg += it.kg;
                    }
                }
            }
            // 站内待取走：只认**已实际执行**的卸入（账本），且尚未被取走。
            // 已送达的行在送达时同时被标成 pickedUp（贷随车走），不会重复计入。
            double pendingKg = 0.0;
            for (const TransitLedgerEntry& e : transitLedger_) {
                if (e.stationId == rows[i].id && !e.pickedUp) {
                    pendingKg += e.kg;
                }
            }
            transitTable_->setItem(i, 0, new QTableWidgetItem(rows[i].name));
            transitTable_->setItem(i, 1,
                                   new QTableWidgetItem(QString::number(rows[i].sub)));
            transitTable_->setItem(i, 2,
                                   new QTableWidgetItem(QString::number(rows[i].serves)));
            transitTable_->setItem(i, 3,
                                   new QTableWidgetItem(QString::number(stashKg, 'f', 0)));
            transitTable_->setItem(i, 4,
                                   new QTableWidgetItem(QString::number(pendingKg, 'f', 0)));
        }
    }

    // ---- 转运账本（§7.5⑥；#56 起与中转站表同属「中转与配送网」dock）：7 列，逐笔一行 ----
    // 列序：订单 | 中转站 | kg | 卸入(<车> <HH:MM>) | 取走(同形；未取走 —) |
    //       站内等待(min)(未取走 —) | 状态（#62 三态）
    // #62：旧写法 delivered ? 已送达 : 在站 把「已取走、在车上、尚未送达」也说成在站，
    // 用户同时看到「站内待取走=0」自然判成信息冲突。状态必须按物理事实分三态：
    //   !pickedUp            → 在站（货在站上等下一台车；站内待取走计入）
    //   pickedUp && !delivered → 在途（<车> <HH:MM> 取走）（货已上车；站内待取走不计）
    //   delivered            → 已送达（送达时若还没取走也一并补齐 pickedUp，见 deliverAt）
    {
        transitLedgerTable_->setRowCount(static_cast<int>(transitLedger_.size()));
        for (std::size_t i = 0; i < transitLedger_.size(); ++i) {
            const TransitLedgerEntry& e = transitLedger_[i];
            const int row = static_cast<int>(i);
            transitLedgerTable_->setItem(
                row, 0, new QTableWidgetItem(QString::fromStdString(e.orderId)));
            transitLedgerTable_->setItem(
                row, 1, new QTableWidgetItem(QString::fromStdString(e.stationId)));
            transitLedgerTable_->setItem(row, 2,
                                         new QTableWidgetItem(QString::number(e.kg, 'f', 0)));
            transitLedgerTable_->setItem(
                row, 3, new QTableWidgetItem(QString::fromStdString(e.inVehicleId)
                                             + QLatin1Char(' ')
                                             + minutesToClock(e.inTimeMin)));
            if (e.pickedUp) {
                transitLedgerTable_->setItem(
                    row, 4, new QTableWidgetItem(QString::fromStdString(e.outVehicleId)
                                                 + QLatin1Char(' ')
                                                 + minutesToClock(e.outTimeMin)));
                transitLedgerTable_->setItem(
                    row, 5,
                    new QTableWidgetItem(QString::number(e.outTimeMin - e.inTimeMin)));
            } else {
                transitLedgerTable_->setItem(row, 4, new QTableWidgetItem(QStringLiteral("—")));
                transitLedgerTable_->setItem(row, 5, new QTableWidgetItem(QStringLiteral("—")));
            }
            // #62：三态，不再把「已取走但未送达」误报成「在站」。
            // 站内待取走列仍只读 pickedUp == false 的行（上方第 5 列），两列语义一致。
            QString statusText;
            if (e.delivered) {
                statusText = QStringLiteral("已送达");
            } else if (e.pickedUp) {
                statusText = QStringLiteral("在途（%1 %2 取走）")
                                 .arg(QString::fromStdString(e.outVehicleId))
                                 .arg(minutesToClock(e.outTimeMin));
            } else {
                statusText = QStringLiteral("在站");
            }
            QTableWidgetItem* statusCell = new QTableWidgetItem(statusText);
            // 状态列在窄面板里可能被裁掉（「在途（V01 11:34 取走）」较长），
            // tooltip 保留完整文本——用户不必为读一句话去拉宽 dock。
            statusCell->setToolTip(statusText);
            transitLedgerTable_->setItem(row, 6, statusCell);
        }
        // 空态提示与账本行数严格互斥：无行才有提示。
        if (ledgerEmptyHint_ != nullptr) {
            ledgerEmptyHint_->setVisible(transitLedger_.empty());
        }
    }

    // 超时订单独立窗口已退役（#56）：它在订单表里变成「状态」列的「超时 +Nmin」后缀
    // 并排到表头，同一张单不再在两块面板里各显示一遍。这里不再需要第三次填表。

    // 选中行 → 画布加粗：updatePanels 在 syncScene 之后调用，改选后必须再同步一次画布，
    // 否则首刷的加粗落在旧选中车上（用户看到的那台车"没被选中"）。
    if (vehicleSelectionChanged && scene_ != nullptr) {
        scene_->setFleetRoutes(canvasTargetRoutes(), selectedVehicleIndex(),
                               logistics::fastVehicleFlags(config_.vehicles));
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
    out += QStringLiteral("策略：%1    车队：%2 台车    事件间隔：每 %3 min（软件内时间）\n")
               .arg(strategyBox_ != nullptr ? strategyBox_->currentText() : QString())
               .arg(config_.vehicles.size())
               .arg(config_.general.eventIntervalMin);
    out += QString(72, QLatin1Char('=')) + QStringLiteral("\n");
    for (const QString& line : runLog_) {
        out += line + QStringLiteral("\n");
    }
    out += QString(72, QLatin1Char('=')) + QStringLiteral("\n");
    out += QStringLiteral("汇总：事件 %1 次 ｜ 已送达 %2 站 ｜ 期末车上载重 %3 kg\n")
               .arg(runEventCount_)
               .arg(active().state.servedStops)
               .arg(active().state.loadKg, 0, 'f', 0);
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
    if (runLog_.isEmpty() && active().state.servedStops == 0 && runEventCount_ == 0) {
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
                      .arg(active().state.servedStops));
    } else {
        appendLog(QStringLiteral("另存失败（无法写入）：%1").arg(path));
    }
}

void MainWindow::runDemoActions(int rounds) {
    // 演示序列的**固定步骤横幅**（规格二 §7.9 G20）：只钉「有没有跳过一步」，
    // **不钉**「哪台车接单 / 转运增益几分钟」等随实现与数据变的数值。
    // 先把本轮要执行的步骤排成清单，再逐条执行：k/N 的顺序与动作一一对应，
    // 与动作是否真的产生副作用无关（事件抽签失败也照打横幅）。
    std::vector<QString> steps;
    for (int i = 0; i < rounds; ++i) {
        steps.push_back(QStringLiteral("推进一站"));
        steps.push_back(QStringLiteral("模拟路况"));
        if (i % 2 == 1) {
            steps.push_back(QStringLiteral("插入紧急订单"));
        }
        if (i % 3 == 2) {
            steps.push_back(QStringLiteral("封闭道路"));
        }
    }
    std::size_t k = 0;
    for (int i = 0; i < rounds; ++i) {
        const auto banner = [&](const QString& action) {
            ++k;
            appendLog(QStringLiteral("[演示] %1/%2 %3")
                          .arg(k)
                          .arg(steps.size())
                          .arg(action));
        };
        banner(QStringLiteral("推进一站"));
        onAdvanceStop();
        banner(QStringLiteral("模拟路况"));
        onSimulateTraffic();
        if (i % 2 == 1) {
            banner(QStringLiteral("插入紧急订单"));
            onInsertUrgentOrder();
        }
        if (i % 3 == 2) {
            banner(QStringLiteral("封闭道路"));
            onCloseRandomRoad();
        }
        // 每一步都审一次（规格七 #63 的「逐帧」）：计划与已走节点都看。
        auditSlowIsolationNow();
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

// 超时面板的**行数口径**（G10）：面板是**逐车铺行**的（updatePanels 里对 fleet_ 全量循环），
// 所以这里也必须数**全部车**，不能只数车辆表当前选中行那台。
// 为什么以前没暴露：规格三之前默认数据 penalty 恒 0（或迟到单恰好落在 V02 上而当时
// activeVehicle_ 指向它），两种口径给出同一个数。规格四把干线提到跨城尺度后，
// distance/time 策略下唯一迟到的 O07 落在 V02、而选中行停在 V01（空车）——
// 「面板 1 行 vs 计数 0」立刻分叉。这是**计数口径的缺陷**，不是数据问题：
// 修的是「数谁」，不是放宽判据。
int MainWindow::lateOrderCount() const {
    // #56：超时订单并入订单表后，口径必须**按订单去重**。旧实现按车累加，
    // 一张跨簇单在接力中会同时出现在两台车的 metrics.orders 里（各一段腿），
    // 按车计数会把它算成两行，而订单表里只有一行。
    // 仍然读规划器逐单真源（metrics.orders）+ deliveredLate 物理事实，不重算面板公式。
    std::set<std::string> ids;
    for (std::size_t v = 0; v < fleet_.size(); ++v) {
        const VehicleRuntime& rt = fleet_[v];
        for (const logistics::OrderMetric& om : rt.plan.metrics.orders) {
            if (om.late) {
                ids.insert(om.orderId);
            }
        }
        for (const VehicleState::LateStop& ls : rt.state.deliveredLate) {
            ids.insert(ls.orderId);
        }
    }
    return static_cast<int>(ids.size());
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

double MainWindow::stationPendingKg(const std::string& stationId) const {
    // 读**界面上这一格**（第 5 列 = 站内待取走(kg)），不重算求和：
    // 新增列的守卫要能发现"列填错了数据源"，重算一遍公式就会把错误一起重算成"正确"。
    if (transitTable_ == nullptr) {
        return 0.0;
    }
    const QString prefix = QString::fromStdString(stationId) + QStringLiteral("（");
    for (int row = 0; row < transitTable_->rowCount(); ++row) {
        const QTableWidgetItem* nameItem = transitTable_->item(row, 0);
        if (nameItem == nullptr || !nameItem->text().startsWith(prefix)) {
            continue;
        }
        const QTableWidgetItem* cell = transitTable_->item(row, 4);
        bool ok = false;
        const double kg = cell != nullptr ? cell->text().trimmed().toDouble(&ok) : 0.0;
        return ok ? kg : 0.0;
    }
    return 0.0;
}

QStringList MainWindow::transitLedgerHeaders() const {
    QStringList names;
    if (transitLedgerTable_ != nullptr) {
        for (int col = 0; col < transitLedgerTable_->columnCount(); ++col) {
            const QTableWidgetItem* item = transitLedgerTable_->horizontalHeaderItem(col);
            names << (item != nullptr ? item->text() : QString());
        }
    }
    return names;
}

QString MainWindow::transitLedgerSummary() const {
    QString out;
    if (transitLedgerTable_ == nullptr) {
        return out;
    }
    for (int row = 0; row < transitLedgerTable_->rowCount(); ++row) {
        QStringList cells;
        for (int col = 0; col < transitLedgerTable_->columnCount(); ++col) {
            const QTableWidgetItem* item = transitLedgerTable_->item(row, col);
            cells << (item != nullptr ? item->text() : QString());
        }
        out += cells.join(QStringLiteral(" | ")) + QLatin1Char('\n');
    }
    return out;
}

QStringList MainWindow::transitLedgerStatuses() const {
    QStringList out;
    if (transitLedgerTable_ == nullptr) {
        return out;
    }
    for (int row = 0; row < transitLedgerTable_->rowCount(); ++row) {
        const QTableWidgetItem* item = transitLedgerTable_->item(row, 6);
        out << (item != nullptr ? item->text() : QString());
    }
    return out;
}

int MainWindow::transitLedgerRowCount() const {
    return transitLedgerTable_ != nullptr ? transitLedgerTable_->rowCount() : 0;
}

bool MainWindow::transitLedgerEmptyHintVisible() const {
    return ledgerEmptyHint_ != nullptr && ledgerEmptyHint_->isVisible();
}

QString MainWindow::transitLedgerEmptyHintText() const {
    return ledgerEmptyHint_ != nullptr ? ledgerEmptyHint_->text() : QString();
}

int MainWindow::vehicleRowCount() const {
    return vehicleTable_ != nullptr ? vehicleTable_->rowCount() : 0;
}

QStringList MainWindow::vehiclePanelHeaders() const {
    QStringList names;
    if (vehicleTable_ != nullptr) {
        for (int col = 0; col < vehicleTable_->columnCount(); ++col) {
            const QTableWidgetItem* item = vehicleTable_->horizontalHeaderItem(col);
            names << (item != nullptr ? item->text() : QString());
        }
    }
    return names;
}

QString MainWindow::vehicleRowText(int row) const {
    if (vehicleTable_ == nullptr || row < 0 || row >= vehicleTable_->rowCount()) {
        return QString();
    }
    QStringList cells;
    for (int col = 0; col < vehicleTable_->columnCount(); ++col) {
        const QTableWidgetItem* item = vehicleTable_->item(row, col);
        cells << (item != nullptr ? item->text() : QString());
    }
    return cells.join(QStringLiteral(" | "));
}

QString MainWindow::vehicleRowTooltip(int row) const {
    if (vehicleTable_ == nullptr || row < 0 || row >= vehicleTable_->rowCount()) {
        return QString();
    }
    // 取第 0 格的 tooltip：同一行每格挂的是同一份文本（见 updatePanels）。
    const QTableWidgetItem* item = vehicleTable_->item(row, 0);
    return item != nullptr ? item->toolTip() : QString();
}

QString MainWindow::vehiclePanelSummary() const {
    // **逐行**都给出去：--ui-probe 要逐车核对载货/额定/利用率与 tooltip，
    // 只给当前选中行会让选中状态影响断言结果。
    QString out;
    if (vehicleTable_ != nullptr) {
        for (int row = 0; row < vehicleTable_->rowCount(); ++row) {
            out += vehicleRowText(row);
            out += QLatin1Char('\n');
        }
    }
    return out;
}

QStringList MainWindow::orderPanelHeaders() const {
    QStringList names;
    if (orderTable_ != nullptr) {
        for (int col = 0; col < orderTable_->columnCount(); ++col) {
            const QTableWidgetItem* item = orderTable_->horizontalHeaderItem(col);
            names << (item != nullptr ? item->text() : QString());
        }
    }
    return names;
}

QString MainWindow::orderPanelSummary() const {
    QString out;
    if (orderTable_ == nullptr) {
        return out;
    }
    for (int row = 0; row < orderTable_->rowCount(); ++row) {
        QStringList cells;
        for (int col = 0; col < orderTable_->columnCount(); ++col) {
            const QTableWidgetItem* item = orderTable_->item(row, col);
            cells << (item != nullptr ? item->text() : QString());
        }
        out += cells.join(QStringLiteral(" | ")) + QLatin1Char('\n');
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

QStringList MainWindow::rightTabTitles() const {
    QStringList titles;
    if (rightTabs_ == nullptr) {
        return titles;
    }
    for (int i = 0; i < rightTabs_->count(); ++i) {
        titles << rightTabs_->tabText(i);
    }
    return titles;
}

QStringList MainWindow::bottomTabTitles() const {
    QStringList titles;
    if (bottomTabs_ == nullptr) {
        return titles;
    }
    for (int i = 0; i < bottomTabs_->count(); ++i) {
        titles << bottomTabs_->tabText(i);
    }
    return titles;
}

int MainWindow::rightTabCurrentIndex() const {
    return rightTabs_ != nullptr ? rightTabs_->currentIndex() : -1;
}

int MainWindow::bottomTabCurrentIndex() const {
    return bottomTabs_ != nullptr ? bottomTabs_->currentIndex() : -1;
}

void MainWindow::activateRightTab(int index) {
    if (rightTabs_ != nullptr && index >= 0 && index < rightTabs_->count()) {
        rightTabs_->setCurrentIndex(index);
    }
}

int MainWindow::rightVisiblePageCount() const {
    // 数的是**三个已知页面**的可见状态，而不是「QTabWidget 有几页」：
    // QTabWidget 天然只显示当前页；若以后有人把三页改成同屏（QVBoxLayout 等），
    // 这里会得到 3，守卫随即变红。#59 的反向验证正是这么做的。
    int visible = 0;
    for (const QWidget* page : {rightVehiclePage_, rightOrderPage_, rightTransitPage_}) {
        if (page != nullptr && page->isVisible()) {
            ++visible;
        }
    }
    return visible;
}

int MainWindow::bottomVisiblePageCount() const {
    if (bottomTabs_ == nullptr) {
        return -1;
    }
    int visible = 0;
    for (int i = 0; i < bottomTabs_->count(); ++i) {
        const QWidget* page = bottomTabs_->widget(i);
        if (page != nullptr && page->isVisible()) {
            ++visible;
        }
    }
    return visible;
}

bool MainWindow::routePanelVisible() const {
    return routeInfo_ != nullptr && routeInfo_->isVisible();
}

bool MainWindow::stopPanelVisible() const {
    return stopTable_ != nullptr && stopTable_->isVisible();
}

QString MainWindow::logPanelText() const {
    return logView_ != nullptr ? logView_->toPlainText() : QString();
}

QString MainWindow::vehicleRingTooltip(const std::string& nodeId) const {
    return scene_ != nullptr ? scene_->nodeVehicleTooltip(nodeId) : QString();
}

QString MainWindow::routePanelSummary() const {
    return routeInfo_ != nullptr ? routeInfo_->toPlainText() : QString();
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


    // 车队口径的取数助手（多车下「哪台车接了这张单」不再唯一）：
    // 返回该订单所在的车辆下标；找不到返回 fleet_.size()。
    const auto vehicleOfOrder = [this](const std::string& orderId) -> std::size_t {
        for (std::size_t v = 0; v < fleet_.size(); ++v) {
            for (const logistics::Stop& s : veh(v).plan.stops) {
                for (const logistics::StopDelivery& d : s.deliveries) {
                    if (d.orderId == orderId) {
                        return v;
                    }
                }
                for (const std::string& pid : s.pickups) {
                    if (pid == orderId) {
                        return v;
                    }
                }
            }
        }
        return fleet_.size();
    };
    // 该订单是否出现在**任何**车的计划里（取货或送达）
    const auto orderInAnyPlan = [&](const std::string& orderId) -> bool {
        return vehicleOfOrder(orderId) < fleet_.size();
    };

    // ---- 1. 插入一张装得下的紧急取送单：A 取 → B 送，先 A 后 B ----
    // 多车下分配由车队重规划决定（顺序轮转基线），所以逐车扫「该单落在哪台车上」，
    // 再在**那台车**的路线里验先取后送与紧急优先。
    if (cust.size() >= 2) {
        const std::string A = cust[0];
        const std::string B = cust[1];
        const std::size_t before = config_.orders.size();
        const std::string id = insertOrderAt(A, B, 5.0, true,
                                            logistics::OrderSource::ManualUrgent, currentTimeMin());
        expect(!id.empty(), QStringLiteral("紧急取送单应插入成功"));
        expect(config_.orders.size() == before + 1,
               QStringLiteral("紧急单必须并入订单集合（不丢单）"));
        const std::size_t owner = vehicleOfOrder(id);
        expect(owner < fleet_.size(),
               QStringLiteral("紧急单必须落在某台车的计划里（不丢单）"));
        expect(owner < fleet_.size()
                   && veh(owner).plan.status == logistics::PlanStatus::Ok,
               QStringLiteral("接单车辆的计划仍可行"));
        if (owner < fleet_.size()) {
            const logistics::RoutePlan& plan = veh(owner).plan;
            std::size_t ia = plan.nodes.size();
            std::size_t ib = plan.nodes.size();
            for (std::size_t i = 0; i < plan.nodes.size(); ++i) {
                if (plan.nodes[i] == A && ia == plan.nodes.size()) {
                    ia = i;
                }
                if (plan.nodes[i] == B && ib == plan.nodes.size()) {
                    ib = i;
                }
            }
            expect(ia < ib,
                   QStringLiteral("取送顺序：A 必须早于 B（车 %1，A@%2, B@%3）")
                       .arg(QString::fromStdString(config_.vehicles[owner].id))
                       .arg(ia)
                       .arg(ib));
            // 紧急优先守卫（issue #5 放宽）：B 的到达 ≤ 其余普通停靠点；
            // A/B 同点顺路交付时 A 可能排在 B 之前，这是允许的例外。
            bool bServed = false;
            int bArrival = -1;
            for (const logistics::Stop& s : plan.stops) {
                if (s.nodeId == B) {
                    bServed = true;
                    bArrival = s.arrivalMin;
                    break;
                }
            }
            bool bFirst = bServed;
            for (const logistics::Stop& s : plan.stops) {
                if (s.nodeId == A || s.nodeId == B) {
                    continue;
                }
                if (bArrival >= 0 && bArrival > s.arrivalMin) {
                    bFirst = false;
                }
            }
            expect(bServed, QStringLiteral("紧急送达点 B 必须出现在接单车辆的计划停靠点里"));
            expect(bFirst,
                   QStringLiteral("紧急送达点 B 的到达不晚于其它普通停靠点（A/B 顺路交付除外）"));
        }
        // 推进到取货点 A：应进入「执行中（已取未送）」；到达 B：应完成。
        // 多车下**全队**推进（advanceFleetTo 的同刻语义），任一车取到即算。
        const auto pickedAnywhere = [this](const std::string& orderId) -> bool {
            for (const VehicleRuntime& rt : fleet_) {
                if (rt.state.pickedOrderIds.count(orderId) > 0) {
                    return true;
                }
            }
            return false;
        };
        bool picked = false;
        for (int i = 0; i < 60 && !picked; ++i) {
            onAdvanceStop();
            picked = pickedAnywhere(id);
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
            // 多车：在**任何**车的计划停靠点里出现即算（车队口径）
            if (orderInAnyPlan(o.id)) {
                ++inPlan;
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
        double cap = 0.0;   // 最小车容量（= insertOrderAt 的合法上界）
        for (const logistics::Vehicle& v : config_.vehicles) {
            if (cap <= 0.0 || v.capacityKg < cap) {
                cap = v.capacityKg;
            }
        }
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

        // X 必须真的进路线（只看"新增节点"太弱：节点加了但没被配送等于没有需求）。
        // 读**全队**路线而不是车辆表当前选中行那一台车：职责封闭（#46）之后新客户按它的子网归属
        // 落到**该簇的慢车**上，未必是界面当前选中的那台（默认选中的是「第一台有路线的车」）。
        // 只看 active() 会把它判成"没进路线"，那是口径错、不是缺陷。
        bool inPlan = false;
        for (std::size_t v = 0; v < fleet_.size() && !inPlan; ++v) {
            for (const std::string& n : fleet_[v].plan.nodes) {
                if (n == node) {
                    inPlan = true;
                    break;
                }
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
        const VehicleState snapshot = active().state;
        onSimulateTraffic();
        onReplan();
        expect(active().state.atNodeId == snapshot.atNodeId,
               QStringLiteral("重规划不得改变车辆位置"));
        expect(active().state.atTimeMin == snapshot.atTimeMin,
               QStringLiteral("重规划不得改变车辆时刻"));
        expect(active().state.loadKg == snapshot.loadKg,
               QStringLiteral("重规划不得改变车上载重"));
        expect(active().state.onboard.size() == snapshot.onboard.size()
                   && active().state.sumOnboard() == snapshot.sumOnboard(),
               QStringLiteral("重规划不得改变在车货构成（数量 / 合计）"));
        expect(active().state.servedStops == snapshot.servedStops,
               QStringLiteral("重规划不得改变已送达计数"));
        expect(active().state.incurredPenaltyMin == snapshot.incurredPenaltyMin,
               QStringLiteral("重规划不得改变已发生的 penalty"));
        expect(active().state.deliveredLate.size() == snapshot.deliveredLate.size(),
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
    // 多车适配（#39）：本段是**单车**语义的降载守卫（在 A 装满后溢出 1kg ⇒ 降载 +
    // 同车取回）。车队下顺序轮转会把新单分给别的车，断言就不再指向同一台车。
    // 因此这里用**一台车**的独立窗口跑同一段：语义不变、确定性不变。
    if (cust.size() >= 2 && !config_.vehicles.empty()) {
        logistics::Config soloCfg = reloadInitialConfig();
        const std::size_t minCapVehicle = [&soloCfg]() {
            std::size_t best = 0;
            for (std::size_t i = 1; i < soloCfg.vehicles.size(); ++i) {
                if (soloCfg.vehicles[i].capacityKg < soloCfg.vehicles[best].capacityKg) {
                    best = i;
                }
            }
            return best;
        }();
        soloCfg.vehicles = {soloCfg.vehicles[minCapVehicle]};
        MainWindow solo(soloCfg, configPath_);
        const double cap = solo.config_.vehicles.front().capacityKg;
        // 排空 + 等车上有货：事件刷出的紧急单逐站送完；普通货随车推进自然装上。
        {
            bool ready = false;
            for (int i = 0; i < 1200 && !ready && !solo.atRouteEnd(); ++i) {
                bool pendingUrgent = false;
                for (const Order& o : solo.config_.orders) {
                    if (o.urgent && !o.served) {
                        pendingUrgent = true;
                        break;
                    }
                }
                if (pendingUrgent) {
                    solo.onAdvanceStop();   // 紧急单 FIFO 最先，送完即清
                    continue;
                }
                if (solo.active().state.loadKg > 0.0) {
                    ready = true;
                    break;
                }
                solo.onAdvanceStop();       // 车还空着：推进到下一个取货停靠
            }
            bool nonePending = true;
            for (const Order& o : solo.config_.orders) {
                if (o.urgent && !o.served) {
                    nonePending = false;
                    break;
                }
            }
            expect(ready && nonePending && solo.active().state.loadKg > 0.0,
                   QStringLiteral("（前提）未完成紧急单清零且车上有货（在车 %1kg）")
                       .arg(solo.active().state.loadKg, 0, 'f', 0));
        }
        // 在车货必须是**普通单**的货（其送达在普通段 ⇒ 在新紧急 A 处仍在车上）。
        // 紧急段已排空 ⇒ 车上的货必然全部来自普通单，此断言只是把前提显式化。
        bool onboardAllNormal = true;
        for (const logistics::OnboardItem& it : solo.active().state.onboard) {
            const Order* o = solo.findOrderById(it.orderId);
            if (o == nullptr || o->urgent) {
                onboardAllNormal = false;
            }
        }
        expect(onboardAllNormal,
               QStringLiteral("（前提）在车货全部属于普通单（送达在普通段，A 处仍在车上）"));
        const double overflowDemand = cap - solo.active().state.loadKg + 1.0;
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
        const std::string id = solo.insertOrderAt(A, B, overflowDemand, true,
                                            logistics::OrderSource::ManualUrgent, solo.currentTimeMin());
        expect(!id.empty(), QStringLiteral("A 装满后溢出的紧急取送单应插入成功"));
        if (!id.empty()) {
            // 计划层（确定性）：A 装满后溢出 ⇒ **必然**出现降载记录。
            expect(!solo.active().plan.offloads.empty(),
                   QStringLiteral("A 装满后溢出 1kg ⇒ 应出现降载记录"
                                  "（在车 %1kg + 需求 %2kg > 上限 %3kg）")
                       .arg(solo.active().state.loadKg, 0, 'f', 0)
                       .arg(overflowDemand, 0, 'f', 0)
                       .arg(cap, 0, 'f', 0));
            bool conserve = true;
            bool loadOk = true;
            bool sameVehicleCustody = true;
            for (const logistics::OffloadLeg& leg : solo.active().plan.offloads) {
                if (leg.reclaimTimeMin < 0) {
                    conserve = false;   // 期末必须已取回（暂存守恒）
                }
                if (leg.reclaimTimeMin >= 0 && leg.reclaimTimeMin < leg.offloadTimeMin) {
                    sameVehicleCustody = false;   // 取回必须晚于卸下
                }
                // MC-A09 同车卸同车取：取回动作必须出现在**同一条路线**的某停靠里。
                bool reclaimSeen = false;
                for (const logistics::Stop& st : solo.active().plan.stops) {
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
            for (const logistics::Stop& st : solo.active().plan.stops) {
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
            bool pendingStaysZero = true;
            for (int i = 0; i < 600 && !solo.atRouteEnd(); ++i) {
                solo.onAdvanceStop();
                // issue #10：中转站面板第 4 列（当前暂存货）必须逐帧反映车辆物理状态
                //（stashStation/stashItems），无暂存为 0、有暂存为该站的非 0 kg。
                // #56：表变成 5 列；第 5 列（站内待取走）在本段**必须恒为 0** ——
                // 本段只有"同车降载暂存"、没有任何跨车转运，若第 5 列跟着变非 0，
                // 说明它被错接成了 stashItems（正是本票要修的错误数据源）。
                const QStringList rows =
                    solo.transitPanelSummary().split(QLatin1Char('\n'), Qt::SkipEmptyParts);
                bool anyNonZero = false;
                for (const QString& row : rows) {
                    const QStringList cells = row.split(QStringLiteral(" | "));
                    if (cells.size() != 5) {
                        panelColumnsOk = false;
                        continue;
                    }
                    bool isNumber = false;
                    cells[3].trimmed().toDouble(&isNumber);
                    if (!isNumber) {
                        panelNumeric = false;
                    }
                    bool pendingNumeric = false;
                    const double pendingKg = cells[4].trimmed().toDouble(&pendingNumeric);
                    if (!pendingNumeric) {
                        panelNumeric = false;
                    } else if (pendingKg != 0.0) {
                        pendingStaysZero = false;
                    }
                    if (cells[3].trimmed().toDouble() > 0.0) {
                        anyNonZero = true;
                    }
                }
                const bool stashExists =
                    !solo.active().state.stashStation.empty() && !solo.active().state.stashItems.empty();
                if (anyNonZero != stashExists) {
                    panelMatchesState = false;
                }
                if (solo.active().state.loadKg > maxLoad) {
                    maxLoad = solo.active().state.loadKg;
                    maxNode = solo.active().state.atNodeId;
                    QStringList parts;
                    for (const logistics::OnboardItem& it : solo.active().state.onboard) {
                        parts << (QString::fromStdString(it.nodeId) + ":"
                                  + QString::number(it.kg, 'f', 0));
                    }
                    maxBreak = parts.join(QStringLiteral(" | ")).toStdString();
                }
                if (solo.active().state.loadKg > cap + 1e-6) {
                    traceOk = false;
                }
            }
            expect(traceOk, QStringLiteral("全程车辆载重不超过上限（峰值 %1kg @ %2 / 上限 %3，车上明细 %4）")
                               .arg(maxLoad, 0, 'f', 0)
                               .arg(QString::fromStdString(maxNode))
                               .arg(cap, 0, 'f', 0)
                               .arg(QString::fromStdString(maxBreak)));
            expect(panelColumnsOk, QStringLiteral("中转站表始终为 5 列（#56 新增站内待取走）"));
            expect(panelNumeric,
                   QStringLiteral("中转站「当前暂存货 / 站内待取走」两列始终是数字"));
            expect(panelMatchesState,
                   QStringLiteral("中转站「当前暂存货」列必须逐帧反映车辆暂存物理状态"
                                  "（无暂存为 0、有暂存为非 0）"));
            expect(pendingStaysZero,
                   QStringLiteral("（本段无跨车转运）「站内待取走」列必须恒为 0 —— "
                                  "它只读转运账本，不得把降载暂存误算进来"));
            // 若降载真的执行了（未被中途重规划改道），日志里应能看到「降载」
            if (!solo.active().state.appliedOffloads.empty()) {
                expect(solo.runLogText().contains(QStringLiteral("降载")),
                       QStringLiteral("执行降载后日志应出现「降载」"));
                // MC-A09 用户可见证据：同一张降载单的「卸下」与「取回」都出现在
                // 这台车的运行日志里（规格一只有一台车；转运的「卸入车 ≠ 取走车」
                // 归规格二 #41，本票不建转运面板）。
                bool custodyOk = true;
                for (const VehicleState::OffloadKey& key : solo.active().state.appliedOffloads) {
                    const QString unloadMark =
                        QStringLiteral("卸下订单 %1 ").arg(QString::fromStdString(key.orderId));
                    const QString reclaimMark =
                        QStringLiteral("取回订单 %1 ").arg(QString::fromStdString(key.orderId));
                    if (!solo.runLogText().contains(unloadMark)
                        || !solo.runLogText().contains(reclaimMark)) {
                        custodyOk = false;
                    }
                }
                expect(custodyOk,
                       QStringLiteral("MC-A09 卸货车 == 取回车：同一张单同车卸下、同车取回"));
                expect(solo.active().state.stashItems.empty(),
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
    // MainWindow 对象，也不共享 rng_ / state_。
    //
    // **对齐点从「路线终点」改成「同一个固定时刻 T」（规格四 #54）**：
    // 规格四把跨城干线提到 180–240km 之后，事件刻（每 15min 一个）从 34 个涨到 46 个，
    // 而**事件刷单速率不再低于车队运力** —— 每张事件单有 69% 概率是跨簇单（25 个配送点
    // 里跨簇有序对占 69%），而跨城尺度下每张跨簇单要走三段接力、占三台车数小时。
    // 于是「推进到路线终点」这个条件**永远不成立**：实测旧尺度 34 刻 / 12 张事件单就收敛
    // （16:41 结束），新尺度 120 张事件单、跑到 495 小时仍在刷单。
    //
    // 这不是代码缺陷，是「事件注入速率 > 运力」这条**数据事实**的直接后果。
    // 而 D24 等价性的原始语义本来就是「一次推进一刻 == 逐个推进到**同一时刻**」——
    // 终点只是旧数据下恰好能用的一个 T。改成显式 T 之后：
    //   · 判据没有被放宽：仍逐字比对事件日志序列与 A2–A7 终态快照；
    //   · 但 T 必须**取在两条路径都能到达的地方**，否则比较的是两个不同的世界。
    // T 取「发车时刻 + 8h」：覆盖全部城内配送与一段跨城往返，足够让事件刻、
    // 到达、重规划、额度变化都发生若干轮。
    {
        MainWindow pathA(reloadInitialConfig(), configPath_);
        MainWindow pathB(reloadInitialConfig(), configPath_);

        // T 的取法：从**最早发车时刻**起 8 小时。不依赖任何规划结果（规划本身会随事件变），
        // 所以两条路径拿到的是同一个绝对值 —— 这是等价性可比较的前提。
        const int alignMin = pathA.minDepartMin() + 8 * 60;
        int stepsB = 0;
        while (pathB.currentTimeMin() < alignMin && stepsB < 4000) {
            pathB.onAdvanceStop();     // 路径 B：反复推进一站
            ++stepsB;
        }
        int stepsA = 0;
        while (pathA.currentTimeMin() < alignMin && stepsA < 4000) {
            pathA.onAdvanceMoment();   // 路径 A：反复推进一刻
            ++stepsA;
        }
        // 规格六 #60：B（推进一站）会越过对齐时刻落在某个到达上（实测 16:01），而 A 按事件刻
        // 走只能停在 15min 网格上（16:00 / 16:15）。为了「同一时刻、同一世界」，A 最后再
        // **直接推进到 B 的实际落点**——advanceFleetTo 会结算该时刻前的全部事件刻并把时钟
        // 钉到目标值，与「推进一刻逐步走到那里」等价（这正是 D24 要证的）。
        if (pathA.currentTimeMin() < pathB.currentTimeMin()) {
            pathA.advanceFleetTo(pathB.currentTimeMin());
            ++stepsA;
        }
        expect(stepsA > 0 && stepsB > 0,
               QStringLiteral("两条路径都推进了至少一步（推进一刻 %1 步 / 推进一站 %2 步）")
                   .arg(stepsA)
                   .arg(stepsB));
        // 对齐时刻：两条路径都必须**越过** T（推进一刻停在刻上、推进一站越过它），
        // 且车队时钟必须同值 —— 这是「同一时刻、同一世界」的可比较前提。
        expect(pathA.currentTimeMin() >= alignMin && pathB.currentTimeMin() >= alignMin,
               QStringLiteral("两条路径都推进到对齐时刻 %1（A 到 %2 / B 到 %3）")
                   .arg(minutesToClock(alignMin))
                   .arg(minutesToClock(pathA.currentTimeMin()))
                   .arg(minutesToClock(pathB.currentTimeMin())));
        expect(pathA.currentTimeMin() == pathB.currentTimeMin(),
               QStringLiteral("对齐时刻的车队时钟一致（A %1 / B %2）")
                   .arg(minutesToClock(pathA.currentTimeMin()))
                   .arg(minutesToClock(pathB.currentTimeMin())));

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

    // ---- 8b. 同刻多车（Tnext 并列）也必须等价 —— A6 的反向验证载体 ----
    //
    // 规格二 §6.10 的反向验证对象：把 advanceFleetTo 的并列分支改成「只推进第一台」，
    // A6（各车位置与时刻）必须红。默认数据上三台车的到达时刻几乎不并列，这条验证
    // 在默认数据上会空跑 —— 因此用一个**两台车共用首段**的夹具保证每一步都并列：
    //   V01: W→H→A→B→W、V02: W→H→C→D→W，每段 15min ⇒ 到达时刻逐位相同，
    //   且 495/510/525/540 恰好都是事件刻（同刻并列 + 事件刻重合的形态）。
    {
        logistics::Config mirror = reloadInitialConfig();
        mirror.general.eventIntervalMin = 15;
        // 事件权重偏向"路况"（27/30）：路况会**立刻重规划受影响车**，而"谁受影响"
        // 取决于结算时各车的位置/路线 —— 正是并列顺序能被观测到的地方。
        // （订单类事件的落点常常恒定在 Δkey 最小的那台车上，反而测不出差异。）
        mirror.general.eventLevelTraffic = logistics::EventLevel::High;
        mirror.general.eventLevelClosure = logistics::EventLevel::ExtraLow;
        mirror.general.eventLevelCustomer = logistics::EventLevel::ExtraLow;
        mirror.general.eventLevelUrgent = logistics::EventLevel::High;
        mirror.graph = logistics::LogisticsGraph();
        const auto node = [&mirror](const std::string& id, logistics::NodeType type,
                                    const std::string& name, int sub) {
            logistics::Node n;
            n.id = id;
            n.type = type;
            n.name = name;
            n.subNetworkId = sub;
            mirror.graph.addNode(n);
        };
        const auto two = [&mirror](const std::string& a, const std::string& b, double km) {
            logistics::Edge e;
            e.fromId = a;
            e.toId = b;
            e.distanceKm = km;
            e.timeMin = km;
            e.costYuan = km;
            mirror.graph.addEdge(e);
            logistics::Edge back = e;
            back.fromId = b;
            back.toId = a;
            mirror.graph.addEdge(back);
        };
        // 两台车共用一段 W->H（同一时刻并列），之后各走各的：
        //   V01: W→H→A→B→W、V02: W→H→C→D→W，每段 15min ⇒ 到达时刻逐位相同，
        //   且 495/510/525/540 恰好都是事件刻（同刻并列 + 事件刻重合）。
        node("W", logistics::NodeType::Warehouse, "仓库", 0);
        node("H", logistics::NodeType::Delivery, "客户H", 9);
        node("A", logistics::NodeType::Delivery, "客户A", 1);
        node("B", logistics::NodeType::Delivery, "客户B", 1);
        node("C", logistics::NodeType::Delivery, "客户C", 2);
        node("D", logistics::NodeType::Delivery, "客户D", 2);
        two("W", "H", 15.0);
        two("H", "A", 15.0);
        two("A", "B", 15.0);
        two("B", "W", 15.0);
        two("H", "C", 15.0);
        two("C", "D", 15.0);
        two("D", "W", 15.0);
        // #46：职责封闭要求「一台快车 + 若干慢车」。夹具要的是**两台对称的慢车**，
        // 所以再加一台 200kg 的快车 V03 当「簇间车」——它在纯本地单的镜像夹具上没有活
        // （空车不参与并列断言），V01/V02 才是要比较的那两台慢车。
        mirror.vehicles.clear();
        for (const char* id : {"V01", "V02"}) {
            logistics::Vehicle v;
            v.id = id;
            v.startNodeId = "W";
            v.capacityKg = 100.0;
            v.departTimeMin = 480;
            mirror.vehicles.push_back(v);
        }
        {
            logistics::Vehicle fast;
            fast.id = "V03";
            fast.startNodeId = "W";
            fast.capacityKg = 200.0;   // 容量最大 ⇒ 快车（簇间搬运），不接本地簇
            fast.departTimeMin = 480;
            mirror.vehicles.push_back(fast);
        }
        mirror.orders.clear();
        const auto order = [&mirror](const std::string& id, const std::string& a,
                                     const std::string& b) {
            logistics::Order o;
            o.id = id;
            o.pickupNodeId = a;
            o.nodeId = b;
            o.demandKg = 10.0;
            o.windowStartMin = 0;
            o.windowEndMin = 1440;
            o.source = logistics::OrderSource::Initial;
            mirror.orders.push_back(o);
        };
        order("O1", "A", "B");
        order("O2", "C", "D");

        MainWindow tieA(mirror, configPath_);
        MainWindow tieB(mirror, configPath_);
        // （前置）两台车每一步都并列：到达时刻数组逐位相同
        bool tiePremise = tieA.fleet().size() == 3
                          && tieA.fleet()[0].plan.nodeArrivalMin
                                 == tieA.fleet()[1].plan.nodeArrivalMin
                          && tieA.fleet()[0].plan.nodeArrivalMin.size() >= 4;
        expect(tiePremise,
               QStringLiteral("X6（前置）镜像夹具：两台车每一步的到达时刻都并列"));
        int stepsTieA = 0;
        while (!tieA.atRouteEnd() && stepsTieA < 400) {
            tieA.onAdvanceMoment();   // 路径 A：推进一刻
            ++stepsTieA;
        }
        int stepsTieB = 0;
        while (!tieB.atRouteEnd() && stepsTieB < 400) {
            tieB.onAdvanceStop();     // 路径 B：推进一站（并列时同刻一起推进）
            ++stepsTieB;
        }
        expect(tieA.atRouteEnd() && tieB.atRouteEnd(),
               QStringLiteral("X6：两条路径都走完（推进一刻 %1 步 / 推进一站 %2 步）")
                   .arg(stepsTieA)
                   .arg(stepsTieB));
        const QString tieSnapA = tieA.equivalenceSnapshot();
        const QString tieSnapB = tieB.equivalenceSnapshot();
        QString tieDiff;
        {
            const QStringList la = tieSnapA.split(QLatin1Char('\n'));
            const QStringList lb = tieSnapB.split(QLatin1Char('\n'));
            const int n = static_cast<int>(la.size() < lb.size() ? la.size() : lb.size());
            for (int i = 0; i < n; ++i) {
                if (la[i] != lb[i]) {
                    tieDiff = QStringLiteral("第 %1 行 A=「%2」 / B=「%3」").arg(i + 1).arg(la[i], lb[i]);
                    break;
                }
            }
            if (tieDiff.isEmpty() && la.size() != lb.size()) {
                tieDiff = QStringLiteral("行数 A=%1 / B=%2").arg(la.size()).arg(lb.size());
            }
        }
        expect(tieSnapA == tieSnapB,
               QStringLiteral("X6 A1–A7（并列场景）逐字一致（差异：%1）").arg(tieDiff));
        // 事件日志也逐字一致（同刻抽签次数与顺序不被推进粒度改变）
        expect(tieA.eventLogLines() == tieB.eventLogLines(),
               QStringLiteral("X6 A1（并列场景）事件日志逐字一致（%1 / %2 行）")
                   .arg(tieA.eventLogLines().size())
                   .arg(tieB.eventLogLines().size()));

        // ---- X7（§6.1「同刻多车一起推进」的结构守卫）----
        //
        // 这是 A6 那条反向验证（把并列分支改成「只推进第一台」）的**可观测载体**：
        // 双跑在终态会收敛（都到家、都送达），单看终态吃不掉这条反向补丁；
        // 而「一次推进一站必须让并列的两台车都前进」是用户能直接看到的结构事实
        //（车辆表选中行 / 状态栏的位置与时刻）。
        MainWindow tieOne(mirror, configPath_);
        // 期望值从**推进前**的计划里读（推进可能触发事件重规划，事后读会变）
        const int expectedArrival =
            (tieOne.fleet().size() > 0 && tieOne.fleet()[0].plan.nodeArrivalMin.size() > 1)
                ? tieOne.fleet()[0].plan.nodeArrivalMin[1]
                : -1;
        tieOne.onAdvanceStop();   // 全队下一到达时刻 = 两台车并列的那个时刻
        const int t1 = tieOne.fleet().size() > 0 ? tieOne.fleet()[0].state.atTimeMin : -1;
        const int t2 = tieOne.fleet().size() > 1 ? tieOne.fleet()[1].state.atTimeMin : -1;
        expect(tieOne.fleet().size() == 3 && t1 == expectedArrival && t2 == expectedArrival,
               QStringLiteral("X7 并列到达：一次「推进一站」把**两台车**都推到同一时刻 %1"
                              "（实际 V01 %2 / V02 %3）")
                   .arg(expectedArrival)
                   .arg(t1)
                   .arg(t2));
        // 夹具里两台车的第一站都是共用的 H（同刻并列）——断言"两台车都真的前进了一站"。
        expect(tieOne.fleet().size() == 3 && tieOne.fleet()[0].nodeIndex == 1
                   && tieOne.fleet()[1].nodeIndex == 1
                   && tieOne.fleet()[0].state.atNodeId == tieOne.fleet()[1].state.atNodeId,
               QStringLiteral("X7 并列到达：两台车都真的前进了一站（实际下标 %1 / %2）")
                   .arg(tieOne.fleet().size() > 0
                            ? static_cast<long long>(tieOne.fleet()[0].nodeIndex)
                            : -1)
                   .arg(tieOne.fleet().size() > 1
                            ? static_cast<long long>(tieOne.fleet()[1].nodeIndex)
                            : -1));
    }

    // ---- 9. 跨车转运（规格二 §5 / #28）：三阶段规划启用 → 执行 → 账本面板可读 ----
    //
    // **场景被本票（#42）替换**，理由如实记录：
    //   旧场景 = 「在默认数据上插入 D08→D18 普通单 → 时间策略重规划 → 转运被接受」。
    //   那条路成立的前提是**顺序轮转基线**（#41 时期 GUI 的分配路径）。#42 把 GUI 的
    //   初始规划改成 #40 的三阶段分配后，同一张单会落到 Δkey 最小的车上、路线已经很紧，
    //   转运层再没有严格更优的候选 —— 实测（探针）：默认数据全部跨簇点对 × 3 种 kg ×
    //   1/2/3 车组合，接受数**全为 0**。因此这里换成一个三阶段分配本身就需要转运的夹具
    //   （与 routeplanner_tests 的 testTransitPlanFleetIntegration 同一套几何/参数），
    //   守卫的意图不变：**计划期账本 → 实际执行 → 面板逐笔可读**。
    {
        logistics::Config base = reloadInitialConfig();
        base.general.eventIntervalMin = 24 * 60;   // 演示窗口内不触发事件
        // 夹具（规格三 §3 / #48 的走廊形态）：仓库 W、中转站 T（子网络 1）/ T3（子网络 3）、
        // 客户 A1/B2（子网络 1）、A3/B1（子网络 3）。三车同容量同时因子、V01 成本因子高
        // ⇒ 接力车 = V01，「簇 -> 车」稳定落成「每簇一台慢车」。
        // O1（A1→B1）是跨簇普通单 ⇒ **只能**走三段腿：V0x 取 A1 卸 T（L1）、V01 T→T3（L2）、
        // V0y 取 T3 送 B1（L3）。这正是「planFleet 里真的跑通了接力」的最小演示形态。
        // （旧的两车 + 单站夹具在 #48 之后不再成立：三段腿要求两侧各有中转站，且
        //  L1/L3 必须由不同车执行 —— 否则运单会在同一台车里倒挂。）
        base.graph = logistics::LogisticsGraph();
        const auto addNodeFixture = [&base](const std::string& id, logistics::NodeType type,
                                            const std::string& name, int sub) {
            logistics::Node node;
            node.id = id;
            node.type = type;
            node.name = name;
            node.subNetworkId = sub;
            base.graph.addNode(node);
        };
        addNodeFixture("W", logistics::NodeType::Warehouse, "仓库", 0);
        addNodeFixture("T", logistics::NodeType::Transit, "中转站T", 1);
        addNodeFixture("T3", logistics::NodeType::Transit, "中转站T3", 3);
        addNodeFixture("A1", logistics::NodeType::Delivery, "客户A1", 1);
        addNodeFixture("B2", logistics::NodeType::Delivery, "客户B2", 1);
        addNodeFixture("B1", logistics::NodeType::Delivery, "客户B1", 3);
        addNodeFixture("A3", logistics::NodeType::Delivery, "客户A3", 3);
        const auto addEdgeFixture = [&base](const std::string& a, const std::string& b,
                                            double km) {
            logistics::Edge e;
            e.fromId = a;
            e.toId = b;
            e.distanceKm = km;
            e.timeMin = km;
            e.costYuan = km;
            base.graph.addEdge(e);
            logistics::Edge back = e;
            back.fromId = b;
            back.toId = a;
            base.graph.addEdge(back);
        };
        // 走廊：W --4-- T --40-- T3；A1 在 W->T 途中（3+1）；B2 经 T；A3 在 T3 边；
        // A3->B1（4）与 T3->B1（6）等价，L3 因此零绕行。
        addEdgeFixture("W", "T", 4.0);
        addEdgeFixture("W", "A1", 3.0);
        addEdgeFixture("A1", "T", 1.0);
        addEdgeFixture("W", "B2", 6.0);
        addEdgeFixture("T", "B2", 6.0);
        addEdgeFixture("T", "T3", 40.0);
        addEdgeFixture("T3", "A3", 2.0);
        addEdgeFixture("A3", "B1", 4.0);
        addEdgeFixture("T3", "B1", 6.0);
        base.vehicles.clear();
        // 三车同容量、同时因子、V01 成本因子高 ⇒ 接力车 = V01（容量并列取成本因子高）。
        for (const char* vid : {"V01", "V02", "V03"}) {
            logistics::Vehicle v;
            v.id = vid;
            v.startNodeId = "W";
            v.capacityKg = 160.0;
            v.departTimeMin = 480;
            v.timeFactor = 1.00;
            v.costFactor = (std::string(vid) == "V01") ? 1.50 : 0.80;
            base.vehicles.push_back(v);
        }
        base.orders.clear();
        // O2（A1->B2）与 O3（A3->B1）是两簇的本地单（把两台慢车带到各自的中转站边），
        // O1（A1->B1）跨簇 ⇒ 三段接力。
        const auto makeFixtureOrder = [](const std::string& id, const std::string& a,
                                         const std::string& b) {
            logistics::Order o;
            o.id = id;
            o.pickupNodeId = a;
            o.nodeId = b;
            o.demandKg = 10.0;
            o.windowStartMin = 0;
            o.windowEndMin = 1440;
            o.source = logistics::OrderSource::Initial;
            return o;
        };
        base.orders.push_back(makeFixtureOrder("O2", "A1", "B2"));
        base.orders.push_back(makeFixtureOrder("O3", "A3", "B1"));
        base.orders.push_back(makeFixtureOrder("O1", "A1", "B1"));

        MainWindow transit(base, configPath_);
        transit.strategyBox_->setCurrentIndex(1);   // 时间策略
        // 计划期账本：三段接力（#48），两端非空、in≠out、kg=10、dwell ≤ 60
        expect(!transit.fleetPlan_.transitLegs.empty(),
               QStringLiteral("G14：三阶段分配 + 时间策略下应有转运计划（计划期账本非空）"));
        bool planLedgerOk = true;
        for (const logistics::TransitLeg& leg : transit.fleetPlan_.transitLegs) {
            const bool twoEnds = !leg.inVehicleId.empty() && !leg.outVehicleId.empty();
            const int dwell = leg.outTimeMin - leg.inTimeMin;
            if (!twoEnds || leg.inVehicleId == leg.outVehicleId || leg.orderId != "O1"
                || leg.stationId != "T3" || leg.kg < 10.0 - 1e-9 || dwell < 0
                || dwell > logistics::kTransferMaxDwellMin) {
                planLedgerOk = false;
            }
        }
        expect(planLedgerOk,
               QStringLiteral("G14：账本两端非空 / in≠out / 站点 T3 / kg=10 / 0≤dwell≤%1（计划期）")
                   .arg(logistics::kTransferMaxDwellMin));

        // 推进到全队收工：卸入 / 取走 / 送达都必须**真的执行**。
        //
        // #56 新增守卫（票面要求）：中转站表第 5 列「站内待取走(kg)」必须在**中途**
        //（卸入后、取走前）> 0，且**期末**回到 0。
        // 为什么必须看中途：这一列的语义就是"货已卸进站、下一台车还没来取"的瞬态，
        // 期末看一定是 0（货都送达了）——只断言期末等于什么都没测。
        // 为什么这能反向验证：若把它改回只读 stashItems（本段的接力**没有**降载暂存），
        // 中途也会是 0 ⇒ 守卫立刻变红（失败输出见 docs/测试用例表.md 的 R 条目）。
        int steps = 0;
        bool sawPendingFrame = false;   // 前提：真的出现过「卸入后、取走前」的帧
        bool midPendingVisible = true;  // 那些帧上面板该站必须 > 0
        bool endPendingZero = true;     // 期末必须全部回到 0
        while (!transit.atRouteEnd() && steps < 800) {
            transit.onAdvanceStop();
            ++steps;
            // 先读**账本事实**：哪些站当前压着未取走的货
            std::vector<std::string> pendingStations;
            for (const TransitLedgerEntry& e : transit.transitLedger_) {
                if (e.pickedUp) {
                    continue;
                }
                bool seen = false;
                for (const std::string& s : pendingStations) {
                    if (s == e.stationId) {
                        seen = true;
                    }
                }
                if (!seen) {
                    pendingStations.push_back(e.stationId);
                }
            }
            if (pendingStations.empty()) {
                continue;
            }
            sawPendingFrame = true;
            // 再读**界面单元格**（stationPendingKg 读的是表里第 5 列）
            for (const std::string& stationId : pendingStations) {
                if (transit.stationPendingKg(stationId) <= 0.0) {
                    midPendingVisible = false;
                }
            }
        }
        expect(sawPendingFrame,
               QStringLiteral("（前提）跨簇接力中途真的出现「已卸入、未取走」帧 —— "
                              "否则下面两条守卫会空跑"));
        expect(midPendingVisible,
               QStringLiteral("G14/#56：卸入后、取走前，中转站「站内待取走(kg)」必须 > 0"
                              "（读面板单元格；改回只读 stashItems 时本条必红）"));
        // 期末：账本无「在站」行 ⇒ 面板第 5 列必须全部为 0
        for (const logistics::Node& n : transit.config_.graph.nodes()) {
            if (n.type != logistics::NodeType::Transit) {
                continue;
            }
            if (transit.stationPendingKg(n.id) != 0.0) {
                endPendingZero = false;
            }
        }
        expect(endPendingZero,
               QStringLiteral("G14/#56：期末中转站「站内待取走(kg)」必须回到 0"));
        expect(transit.atRouteEnd(), QStringLiteral("G14：转运演示全队走完路线（%1 步）").arg(steps));
        const logistics::Order* after = transit.findOrderById("O1");
        expect(after != nullptr && after->served,
               QStringLiteral("G14：跨车转运的订单最终送达"));

        const int ledgerRows = transit.transitLedgerRowCount();
        expect(ledgerRows >= 1,
               QStringLiteral("G14：转运账本面板出现已执行的卸入（实际 %1 行）").arg(ledgerRows));
        bool runLedgerOk = true;
        int inStationRows = 0;
        for (const TransitLedgerEntry& e : transit.transitLedger_) {
            if (!e.delivered) {
                ++inStationRows;
            }
            const int dwell = e.outTimeMin - e.inTimeMin;
            if (e.orderId.empty() || e.stationId.empty() || e.inVehicleId.empty()
                || e.outVehicleId.empty() || e.inVehicleId == e.outVehicleId
                || e.inTimeMin < 0 || e.outTimeMin < e.inTimeMin
                || dwell > logistics::kTransferMaxDwellMin) {
                runLedgerOk = false;
            }
        }
        expect(runLedgerOk,
               QStringLiteral("G14：账本每笔两端非空、in≠out、dwell≤%1")
                   .arg(logistics::kTransferMaxDwellMin));
        expect(inStationRows == 0,
               QStringLiteral("G14：期末账本无「在站」行（实际 %1 行）").arg(inStationRows));
        const QString ledgerText = transit.transitLedgerSummary();
        expect(ledgerText.contains(QStringLiteral("O1"))
                   && ledgerText.contains(QStringLiteral("已送达"))
                   && !ledgerText.contains(QStringLiteral("| 在站")),
               QStringLiteral("G14：转运账本面板文本：含该单且状态为已送达\n%1").arg(ledgerText));
        expect(transit.transitLedgerHeaders().size() == 7,
               QStringLiteral("G14：转运账本面板 7 列（表头 %1 列）")
                   .arg(transit.transitLedgerHeaders().size()));
        std::printf("[self-check] 转运账本：\n%s", ledgerText.toUtf8().constData());

        // ---- #62：账本三态（在站 / 在途 / 已送达）与「站内待取走」一致 ----
        // 真实执行段在 dwell=0 的夹具里可能没有「取走后、送达前」的可观测帧，
        // 所以这里用同一张**界面表格**摆三行夹具，逐行读状态列单元格（不是读内部布尔）。
        // 反向验证：把状态列改回 delivered ? 已送达 : 在站，第二行必红；把第 5 列
        // 改成读 pickedUp == true 的行，末条「站内待取走」必红。
        {
            const std::vector<TransitLedgerEntry> savedLedger = transit.transitLedger_;
            TransitLedgerEntry station;
            station.orderId = "STA";
            station.stationId = "T3";
            station.kg = 10.0;
            station.inVehicleId = "V03";
            station.inTimeMin = 690;            // 11:30 卸入
            station.outVehicleId = "V01";
            station.outTimeMin = -1;
            TransitLedgerEntry moving = station;
            moving.orderId = "MOV";
            moving.outTimeMin = 694;            // 11:34 取走
            moving.pickedUp = true;
            TransitLedgerEntry done = moving;
            done.orderId = "DONE";
            done.delivered = true;

            transit.transitLedger_.clear();
            transit.transitLedger_.push_back(station);
            transit.transitLedger_.push_back(moving);
            transit.transitLedger_.push_back(done);
            transit.updatePanels();
            const QStringList states = transit.transitLedgerStatuses();
            expect(states.size() == 3,
                   QStringLiteral("#62（前提）账本三态夹具 3 行（实际 %1 行）")
                       .arg(states.size()));
            expect(states.value(0) == QStringLiteral("在站"),
                   QStringLiteral("#62：未取走 ⇒ 状态「在站」（实际「%1」）")
                       .arg(states.value(0)));
            expect(states.value(1) == QStringLiteral("在途（V01 11:34 取走）"),
                   QStringLiteral("#62：取走未送达 ⇒ 状态「在途（V01 11:34 取走）」"
                                  "（实际「%1」）")
                       .arg(states.value(1)));
            expect(states.value(2) == QStringLiteral("已送达"),
                   QStringLiteral("#62：已送达 ⇒ 状态「已送达」（实际「%1」）")
                       .arg(states.value(2)));
            // 第 5 列口径不随状态改动：只算 pickedUp == false 的行（这里 10kg）。
            expect(std::fabs(transit.stationPendingKg("T3") - 10.0) < 1e-6,
                   QStringLiteral("#62：「站内待取走(kg)」仍只读未取走行"
                                  "（实际 %1kg，期望 10kg）")
                       .arg(transit.stationPendingKg("T3")));

            transit.transitLedger_ = savedLedger;
            transit.updatePanels();
        }

        // G18 的动作本身（不是只调底层插单函数）：无头路径（offscreen）退化为随机 A/B，
        // 走的是工具栏「插入普通订单…」同一入口。断读订单集合与来源/前缀。
        const std::size_t beforeSlot = transit.config_.orders.size();
        const int normalInsertNow = transit.currentTimeMin();
        transit.onInsertNormalOrder();
        expect(transit.config_.orders.size() == beforeSlot + 1,
               QStringLiteral("G18：工具栏「插入普通订单…」无头路径可用（退化为随机 A/B，不丢单）"));
        if (transit.config_.orders.size() == beforeSlot + 1) {
            const Order& randomOrder = transit.config_.orders.back();
            expect(randomOrder.source == logistics::OrderSource::ManualNormal
                       && !randomOrder.urgent && !randomOrder.id.empty()
                       && randomOrder.id[0] == 'M',
                   QStringLiteral("G18：该动作来源 manual_normal / 前缀 M（实际 %1）")
                       .arg(QString::fromStdString(randomOrder.id)));
            // G18 / §7.9：窗口必须是 [当前时刻, 24:00]（不是 [now, now+60]——
            // 后者是紧急单窗口，会把普通单误当紧窗单）。这条让「窗口写错」变红。
            expect(randomOrder.windowStartMin == normalInsertNow
                       && randomOrder.windowEndMin == 24 * 60,
                   QStringLiteral("G18：插入普通订单窗口 = [当前时刻 %1, 24:00]（实际 [%2, %3]）")
                       .arg(normalInsertNow)
                       .arg(randomOrder.windowStartMin)
                       .arg(randomOrder.windowEndMin));
        }
    }

    // ---- 10. GUI 的分配必须是三阶段分配（与 CLI --dump-plan 的 ALLOC 同源）----
    //
    // 缺陷背景（#41 关闭时定位）：MainWindow::replanFleet() 曾自己按「订单 ID 升序 +
    // i % fleet_.size()」顺序轮转分派，根本没走 #40 的三阶段分配；只有 CLI 走 planFleet。
    // 于是「界面上的分配」与「命令行 --dump-plan 的 ALLOC」是两套东西。
    //
    // 守卫读**两条用户可见路径**：界面侧的「[分配] ...」运行日志行（--ui-probe 也打印它），
    // 与 CLI 侧 planFleet 的分配表（= --dump-plan 的 ALLOC 行的来源），逐单比对。
    {
        MainWindow fresh(reloadInitialConfig(), configPath_);
        const QString guiAlloc = fresh.allocationSnapshot();
        expect(!guiAlloc.isEmpty(),
               QStringLiteral("X3（前置）界面初始规划必须产生三阶段分配快照"));
        expect(fresh.runLogText().contains(QStringLiteral("[分配] 三阶段分配")),
               QStringLiteral("X3：三阶段分配必须出现在运行日志里（用户可见）"));
        expect(fresh.runLogText().contains(guiAlloc),
               QStringLiteral("X3：运行日志里的分配串与界面快照逐字相同"));

        const logistics::FleetPlan fp =
            logistics::planFleet(fresh.config_.graph, fresh.config_.vehicles,
                                 fresh.config_.orders, logistics::WeightType::Distance);
        const QString cliAlloc = fresh.allocationTextOf(fp);
        // 逐单比对（不是整串比较：逐单才能指出是哪一单不一致）
        const auto splitPairs = [](const QString& text) {
            std::vector<std::pair<std::string, std::string> > out;
            const QStringList parts = text.split(QLatin1Char(','));
            for (int i = 0; i < parts.size(); ++i) {
                const int eq = parts[i].indexOf(QLatin1Char('='));
                if (eq <= 0) {
                    continue;
                }
                out.push_back(std::make_pair(parts[i].left(eq).toStdString(),
                                             parts[i].mid(eq + 1).toStdString()));
            }
            return out;
        };
        const std::vector<std::pair<std::string, std::string> > guiPairs = splitPairs(guiAlloc);
        const std::vector<std::pair<std::string, std::string> > cliPairs = splitPairs(cliAlloc);
        std::size_t allocDiff = 0;
        std::string firstAllocDiff;
        if (guiPairs.size() != cliPairs.size()) {
            allocDiff = 1;
            firstAllocDiff = "单数 " + std::to_string(guiPairs.size()) + " vs "
                             + std::to_string(cliPairs.size());
        } else {
            for (std::size_t i = 0; i < guiPairs.size(); ++i) {
                if (guiPairs[i].first != cliPairs[i].first
                    || guiPairs[i].second != cliPairs[i].second) {
                    ++allocDiff;
                    if (firstAllocDiff.empty()) {
                        firstAllocDiff = guiPairs[i].first + "=" + guiPairs[i].second + " / CLI "
                                         + cliPairs[i].first + "=" + cliPairs[i].second;
                    }
                }
            }
        }
        expect(!guiPairs.empty() && allocDiff == 0,
               QStringLiteral("X3：界面分配与 CLI 三阶段 ALLOC 逐单一致（%1 单；差异 %2 处%3）")
                   .arg(guiPairs.size())
                   .arg(allocDiff)
                   .arg(firstAllocDiff.empty()
                            ? QString()
                            : QString(QStringLiteral("，首个 ") + QString::fromStdString(firstAllocDiff))));

        // 判别力：顺序轮转基线必须与三阶段分配**不同**，否则这条守卫测不到东西
        //（界面若退回轮转基线，逐单比对必然出现差异）。
        const logistics::FleetPlan seq = logistics::planFleetSequentialBaseline(
            fresh.config_.graph, fresh.config_.vehicles, fresh.config_.orders,
            logistics::WeightType::Distance);
        const std::vector<std::pair<std::string, std::string> > seqPairs =
            splitPairs(fresh.allocationTextOf(seq));
        std::size_t seqDiff = 0;
        for (std::size_t i = 0; i < seqPairs.size() && i < cliPairs.size(); ++i) {
            if (seqPairs[i].first != cliPairs[i].first
                || seqPairs[i].second != cliPairs[i].second) {
                ++seqDiff;
            }
        }
        expect(seqDiff > 0,
               QStringLiteral("X3（判别力）：顺序轮转基线与三阶段分配在 %1 单上不同 —— "
                              "界面若退回轮转基线，上面的逐单比对必红")
                   .arg(seqDiff));
    }

    // ---- 11. 复调度（规格二 §6 / #42）：受影响集合、五级播报 ----
    //
    // 全部读**用户可见输出**：「[复调度] ...」分级播报行、各车计划（与路线/车辆面板同源）。
    {
        logistics::Config quiet = reloadInitialConfig();
        quiet.general.eventIntervalMin = 24 * 60;   // 本段不要事件打扰
        // 规格六 #60：X4 测的是「五级链第 ① 级在**只影响一台车**时成立」。参半数据下
        // 快车 V01 扛着 12 张跨簇单的站间腿，干线 T01->T02 的路况事件**结构上**牵动
        // 三台车（L1/L2/L3 本就是一条链）—— 那正是「受影响集合」该做的事，却让本段
        // 测不到第 ① 级。所以这里把夹具换成**只剩簇内单**（跨簇单摘掉）：这是**夹具
        // 范围**，不是判据放宽 —— 判据仍是「只影响一台车 ⇒ 第 ① 级局部重排」。
        // 接力链被路况打断时的收敛由 #49 的封路守卫（integration_tests）负责。
        {
            std::vector<logistics::Order> localOnly;
            for (std::size_t i = 0; i < quiet.orders.size(); ++i) {
                if (!logistics::isCrossCluster(quiet.graph, quiet.orders[i])) {
                    localOnly.push_back(quiet.orders[i]);
                }
            }
            quiet.orders = localOnly;
        }
        MainWindow win(quiet, configPath_);

        // 找一条**只在一台车的剩余路线里**的边（有向、相邻两站）。
        std::string from;
        std::string to;
        std::size_t hit = win.fleet().size();
        for (std::size_t v = 0; v < win.fleet().size() && hit >= win.fleet().size(); ++v) {
            const std::vector<std::string>& route = win.fleet()[v].plan.nodes;
            for (std::size_t i = 1; i < route.size(); ++i) {
                if (route[i] == route[i - 1]) {
                    continue;
                }
                bool onlyHere = true;
                for (std::size_t w = 0; w < win.fleet().size(); ++w) {
                    if (w == v) {
                        continue;
                    }
                    if (logistics::isEdgeOnRoute(win.fleet()[w].plan.nodes, route[i - 1],
                                                 route[i])) {
                        onlyHere = false;
                    }
                }
                if (onlyHere) {
                    from = route[i - 1];
                    to = route[i];
                    hit = v;
                    break;
                }
            }
        }
        expect(hit < win.fleet().size(),
               QStringLiteral("X4（前置）存在只属于一台车剩余路线的边（%1->%2）")
                   .arg(QString::fromStdString(from))
                   .arg(QString::fromStdString(to)));

        // 未受影响车的路线逐字记录 —— 判「路线逐字不变」用
        std::vector<std::vector<std::string> > routesBefore;
        for (std::size_t w = 0; w < win.fleet().size(); ++w) {
            routesBefore.push_back(win.fleet()[w].plan.nodes);
        }
        if (hit < win.fleet().size()) {
            // 路况事件对图的作用：把这条边的当前耗时抬高 50%
            const logistics::Edge* edge = win.config_.graph.findEdge(from, to);
            if (edge != nullptr) {
                win.config_.graph.updateEdgeTime(from, to, edge->timeMin * 1.5);
            }
            logistics::RescheduleEvent event;
            event.trigger = logistics::RescheduleTrigger::Traffic;
            logistics::TrafficChange change;
            change.fromId = from;
            change.toId = to;
            change.increaseRatio = 0.5;
            change.congested = true;
            event.changes.push_back(change);
            event.thresholdRatio = 0.2;
            const logistics::FleetRescheduleResult res = win.runReschedule(event);
            expect(res.level == 1 && !res.rejected,
                   QStringLiteral("X4：只影响一台车 ⇒ 第 ① 级局部重排（实际第 %1 级）")
                       .arg(res.level));
            expect(win.runLogText().contains(QStringLiteral("[复调度] 局部重排（1 台车）")),
                   QStringLiteral("X4：分级播报行「[复调度] 局部重排（1 台车）」可见"));
            // 受影响集合正确 + **未受影响车路线逐字不变**（不变量 5）
            std::size_t changedOthers = 0;
            QString firstChanged;
            for (std::size_t w = 0; w < win.fleet().size(); ++w) {
                if (w == hit) {
                    continue;
                }
                if (win.fleet()[w].plan.nodes != routesBefore[w]) {
                    ++changedOthers;
                    if (firstChanged.isEmpty()) {
                        firstChanged = QString::fromStdString(win.config_.vehicles[w].id);
                    }
                }
            }
            expect(changedOthers == 0,
                   QStringLiteral("X4：未受影响车的路线逐字不变（被改动 %1 台%2）")
                       .arg(changedOthers)
                       .arg(firstChanged.isEmpty() ? QString()
                                                   : QString(QStringLiteral("，首个 ") + firstChanged)));
            expect(win.fleet()[hit].plan.status == logistics::PlanStatus::Ok,
                   QStringLiteral("X4：受影响车重算后计划仍可行"));
            // 确定性：同样输入（重新加载配置 + 同一事件）必得同一计划与同一播报
            MainWindow again(quiet, configPath_);
            const logistics::Edge* edge2 = again.config_.graph.findEdge(from, to);
            if (edge2 != nullptr) {
                again.config_.graph.updateEdgeTime(from, to, edge2->timeMin * 1.5);
            }
            const logistics::FleetRescheduleResult res2 = again.runReschedule(event);
            bool samePlans = (res2.level == res.level) && (res2.log == res.log);
            for (std::size_t w = 0; w < again.fleet().size() && samePlans; ++w) {
                if (again.fleet()[w].plan.nodes != win.fleet()[w].plan.nodes) {
                    samePlans = false;
                }
            }
            expect(samePlans,
                   QStringLiteral("X4：确定性 —— 同输入两次运行的分级、播报与逐车路线完全一致"));
        }
    }

    // ---- 11b. 第 ⑤ 级：不可执行的单被**拒绝**并如实播报（不丢单、不凑解）----
    {
        logistics::Config cfg11 = reloadInitialConfig();
        cfg11.general.eventIntervalMin = 24 * 60;
        MainWindow win(cfg11, configPath_);
        Order ghost;
        ghost.id = "O999";
        ghost.pickupNodeId = "D01";
        ghost.nodeId = "DZZ";   // 图上不存在 ⇒ 任何车都排不出可行解
        ghost.demandKg = 5.0;
        ghost.windowStartMin = 480;
        ghost.windowEndMin = 24 * 60;
        ghost.urgent = false;
        ghost.source = logistics::OrderSource::ManualNormal;
        ghost.served = false;
        win.config_.orders.push_back(ghost);
        logistics::RescheduleEvent event;
        event.trigger = logistics::RescheduleTrigger::NewOrder;
        event.newOrder = ghost;
        const logistics::FleetRescheduleResult res = win.runReschedule(event);
        expect(res.rejected && res.level == 5,
               QStringLiteral("X5：不可执行的单必须被拒绝（第 ⑤ 级，实际第 %1 级）")
                   .arg(res.level));
        expect(win.runLogText().contains(QStringLiteral("[复调度] 拒绝：")),
               QStringLiteral("X5：拒绝必须如实播报（分级播报行「[复调度] 拒绝：...」可见）"));
        const Order* still = win.findOrderById(ghost.id);
        expect(still != nullptr && !still->served,
               QStringLiteral("X5：被拒绝的单不得从订单集合里消失（不丢单）"));
    }

    // ---- 12. #56：超时并入订单表的「状态」列（原「超时订单」独立窗口退役）----
    //
    // 为什么必须单独测：默认数据（distance 策略）迟到单恰为 0，--ui-probe 的
    // 「超时行数 == 迟到订单数」在那份数据上是**空跑**（0 == 0）。这里造一张必然迟到的单，
    // 让「状态列内联超时 + 排到最前」这条口径真的被测到。
    {
        logistics::Config lateCfg = reloadInitialConfig();
        lateCfg.general.eventIntervalMin = 24 * 60;   // 不要事件打扰
        // 把第一张单的窗口止压到 00:01（任何到达都晚于它）⇒ 规划期必然记迟到。
        Order& first = lateCfg.orders.front();
        first.windowStartMin = 0;
        first.windowEndMin = 1;
        MainWindow lateWin(lateCfg, configPath_);
        expect(lateWin.lateOrderCount() >= 1,
               QStringLiteral("（前置）窗口止 00:01 的单必然迟到（lateOrderCount=%1）")
                   .arg(lateWin.lateOrderCount()));
        const QStringList orderRows =
            lateWin.orderPanelSummary().split(QLatin1Char('\n'), Qt::SkipEmptyParts);
        expect(!orderRows.isEmpty()
                   && orderRows.front().contains(QStringLiteral("超时 +")),
               QStringLiteral("#56：订单表第一行应是超时单且状态列内联「超时 +Nmin」"
                              "（实际首行：%1）")
                   .arg(orderRows.isEmpty() ? QString() : orderRows.front()));
        const QStringList firstCells =
            orderRows.isEmpty() ? QStringList() : orderRows.front().split(QStringLiteral(" | "));
        const QRegularExpression lateStatusRe(
            QStringLiteral("^[^|]+ · 超时 \\+([0-9]+)min$"));
        // 列序（#56 lead 复核后合并取送，7 列）：
        // 0 订单 | 1 取送 | 2 kg | 3 窗口 | 4 状态 | 5 车辆 | 6 当前位置。
        const QRegularExpressionMatch lm =
            firstCells.size() == 7 ? lateStatusRe.match(firstCells[4])
                                   : QRegularExpressionMatch();
        expect(firstCells.size() == 7 && lm.hasMatch() && lm.captured(1).toInt() > 0,
               QStringLiteral("#56：超时行的「状态」列形如「<基态> · 超时 +Nmin」且 N>0"
                              "（实际：%1）")
                   .arg(firstCells.size() >= 5 ? firstCells[4] : QString()));
        // 该行仍是**逐单一行**（订单列单张 ID），窗口列显示被改过的该单窗口。
        expect(firstCells.size() == 7
                   && firstCells[0] == QString::fromStdString(first.id)
                   && firstCells[3] == QStringLiteral("00:00-00:01"),
               QStringLiteral("#56：超时行仍逐单一行，且窗口列 = 该单自己的 [00:00-00:01]"
                              "（实际订单=%1 窗口=%2）")
                   .arg(firstCells.size() >= 4 ? firstCells[0] : QString(),
                        firstCells.size() >= 4 ? firstCells[3] : QString()));
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