#include "gui/MainWindow.h"

#include <QAction>
#include <QPainter>
#include <QComboBox>
#include <QDialog>
#include <QDir>
#include <QScreen>
#include <QFileInfo>
#include <QDialogButtonBox>
#include <QFormLayout>
#include <QGraphicsView>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QStringList>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QTableWidget>
#include <QTabWidget>
#include <QTextBrowser>
#include <QTimer>
#include <QToolBar>
#include <QVBoxLayout>
#include <QDockWidget>
#include <QGuiApplication>
#include <QToolBar>

#include <algorithm>
#include <cstdio>
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
    QToolBar* bar = addToolBar(QStringLiteral("操作"));
    bar->setMovable(false);

    strategyBox_ = new QComboBox(this);
    // 顺序与「权重标签」下拉保持一致：距离 -> 耗时 -> 成本
    strategyBox_->addItem(QStringLiteral("最短距离策略"), QVariant(QStringLiteral("distance")));
    strategyBox_->addItem(QStringLiteral("最低耗时策略"), QVariant(QStringLiteral("time")));
    strategyBox_->addItem(QStringLiteral("最低成本策略"), QVariant(QStringLiteral("cost")));
    bar->addWidget(new QLabel(QStringLiteral(" 规划策略: "), this));
    bar->addWidget(strategyBox_);
    connect(strategyBox_, &QComboBox::currentIndexChanged, this, &MainWindow::onStrategyChanged);

    weightBox_ = new QComboBox(this);
    weightBox_->addItem(QStringLiteral("显示距离"), QVariant(QStringLiteral("distance")));
    weightBox_->addItem(QStringLiteral("显示耗时"), QVariant(QStringLiteral("time")));
    weightBox_->addItem(QStringLiteral("显示成本"), QVariant(QStringLiteral("cost")));
    bar->addWidget(new QLabel(QStringLiteral("  权重标签: "), this));
    bar->addWidget(weightBox_);
    connect(weightBox_, &QComboBox::currentIndexChanged, this, &MainWindow::onWeightChanged);

    bar->addSeparator();
    bar->addAction(QStringLiteral("模拟路况"), this, &MainWindow::onSimulateTraffic);
    bar->addAction(QStringLiteral("插入紧急订单"), this, &MainWindow::onInsertUrgentOrder);
    bar->addAction(QStringLiteral("模拟新客户"), this, &MainWindow::onAddRandomCustomer);
    bar->addAction(QStringLiteral("模拟道路封闭"), this, &MainWindow::onCloseRandomRoad);
    bar->addSeparator();
    bar->addAction(QStringLiteral("推进一站"), this, &MainWindow::onAdvanceStop);
    bar->addAction(QStringLiteral("重新规划"), this, &MainWindow::onReplan);
    bar->addAction(QStringLiteral("手工增删…"), this, &MainWindow::onManualEdit);
    bar->addAction(QStringLiteral("图表示…"), this, &MainWindow::onShowGraphTables);

    QAction* debugAction = bar->addAction(QStringLiteral("Debug 模式"));
    debugAction->setCheckable(true);
    connect(debugAction, &QAction::toggled, this, &MainWindow::onDebugToggled);

    debugTimer_ = new QTimer(this);
    connect(debugTimer_, &QTimer::timeout, this, &MainWindow::onDebugTick);
}

void MainWindow::buildDocks() {
    auto* routeDock = new QDockWidget(QStringLiteral("路线信息"), this);
    routeDock->setMinimumWidth(360);
    routeInfo_ = new QTextBrowser(routeDock);
    routeDock->setWidget(routeInfo_);
    addDockWidget(Qt::RightDockWidgetArea, routeDock);

    auto* vehicleDock = new QDockWidget(QStringLiteral("车辆信息"), this);
    vehicleDock->setMinimumWidth(360);
    vehicleInfo_ = new QLabel(vehicleDock);
    vehicleInfo_->setTextFormat(Qt::PlainText);
    vehicleInfo_->setMargin(6);
    vehicleDock->setWidget(vehicleInfo_);
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
    transitTable_ = new QTableWidget(0, 5, transitDock);
    transitTable_->setHorizontalHeaderLabels(
        {QStringLiteral("中转站"), QStringLiteral("子网络"), QStringLiteral("下属配送点"),
         QStringLiteral("峰值暂存"), QStringLiteral("当前暂存")});
    transitTable_->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    transitTable_->horizontalHeader()->setStretchLastSection(true);
    transitTable_->verticalHeader()->setVisible(false);
    transitTable_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    transitDock->setWidget(transitTable_);
    addDockWidget(Qt::RightDockWidgetArea, transitDock);

    // 停靠明细：把每个停靠点的原始到达/等待/送达/离开/剩余载重摊开。
    // 这些字段是设计 §4.6 明确要求记录的，此前只有测试在读、界面上看不到；
    // 报告要求【需求分析】⑵ 也要求呈现"到达时间"，而"等待"能解释早到的影响。
    auto* stopDock = new QDockWidget(QStringLiteral("停靠明细"), this);
    stopDock->setMinimumWidth(360);
    // 列里必须有「趟」：停靠明细是**跨趟拉平**的，不加这一列的话
    // "剩余载重"会从 0 跳回几十公斤，看起来像数据错了，其实是新的一趟开始装货。
    stopTable_ = new QTableWidget(0, 7, stopDock);
    stopTable_->setHorizontalHeaderLabels(
        {QStringLiteral("趟"), QStringLiteral("配送点"), QStringLiteral("原始到达"),
         QStringLiteral("等待(分)"), QStringLiteral("送达"), QStringLiteral("离开"),
         QStringLiteral("送后余载(kg)")});
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
void MainWindow::stockTrace(const std::string& stationId, double& current,
                            double& peak) const {
    // 起点必须是**站内真实存货**（stationStock_），不能从 0 起。
    // 顺路寄存只改库存、不产生 TransitOp，从 0 起算的话面板会恒显示 0，
    // 与"站里实际有货"这一事实矛盾。
    const std::map<std::string, double>::const_iterator base = stationStock_.find(stationId);
    current = (base != stationStock_.end()) ? base->second : 0.0;
    peak = current;
    if (plan_.nodes.empty()) {
        return;
    }
    const std::size_t upto = (nodeIndex_ + 1 < plan_.nodes.size()) ? nodeIndex_ + 1
                                                                  : plan_.nodes.size();
    for (std::size_t i = 0; i < upto; ++i) {
        if (i >= plan_.nodeTripIndex.size()) {
            break;
        }
        const std::size_t t = plan_.nodeTripIndex[i];
        if (t >= plan_.trips.size()) {
            continue;
        }
        // 同一趟里同一节点可能出现多次（如 T→D→T），装卸只按**首次到达**计一次
        bool firstInTrip = true;
        for (std::size_t j = 0; j < i; ++j) {
            if (plan_.nodeTripIndex[j] == t && plan_.nodes[j] == plan_.nodes[i]) {
                firstInTrip = false;
                break;
            }
        }
        if (!firstInTrip) {
            continue;
        }
        for (const logistics::TransitOp& op : plan_.trips[t].transitOps) {
            if (op.nodeId == stationId && plan_.nodes[i] == stationId) {
                current += op.amountKg;
            }
        }
        // 峰值"实时"更新：只有真的存进去了、当前量超过历史峰值才刷新
        if (current > peak) {
            peak = current;
        }
    }
}

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
    state_.departed = false;
}

void MainWindow::reconcileOnboardWithPlan() {
    if (config_.vehicles.empty() || plan_.trips.empty()) {
        return;
    }
    const std::size_t t = currentTripIndex();
    if (t >= plan_.trips.size()) {
        return;
    }
    // 本趟**尚未走过**的停靠点：用平面序列里的位置判断，避免依赖 stops 的下标
    const logistics::Trip& trip = plan_.trips[t];
    std::vector<logistics::OnboardItem> want;
    for (const logistics::Stop& s : trip.stops) {
        bool passed = false;
        for (std::size_t i = 0; i < plan_.nodes.size() && i <= nodeIndex_; ++i) {
            if (plan_.nodes[i] == s.nodeId) { passed = true; break; }
        }
        if (passed) {
            continue;
        }
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
            want.push_back(item);
        }
    }
    state_.onboard.swap(want);
    state_.loadKg = state_.sumOnboard();
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

void MainWindow::syncStationStock() {
    stationStock_.clear();
    for (const logistics::TransitStock& st : plan_.transitStock) {
        if (st.finalKg > 1e-9) {
            stationStock_[st.nodeId] = st.finalKg;
        }
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

void MainWindow::replan() {
    if (config_.vehicles.empty()) {
        appendLog(QStringLiteral("配置中没有车辆，无法规划"));
        return;
    }
    const std::string here = currentPositionId();
    const int now = currentTimeMin();
    const std::vector<Order> remaining = remainingOrders();

    plan_ = logistics::replan(config_.graph, config_.vehicles.front(), remaining, here, now,
                              planWeight_,
                              stationStock_, onboardGoods());
    syncStationStock();
    // 新路线的推进状态归零；当前位置/时刻由显式字段保存，不受本次重算影响
    nodeIndex_ = 0;
    stopCursor_ = 0;
    // 车若正在仓库，就把计划的当前趟装上车——这是"装载"这一物理事件，
    // 且必须发生在每次（重新）规划之后，否则新计划第一趟的货永远上不了车。
    // 换计划后：先把车上的货与计划的当前趟对齐（装载是决策，必须与计划一致），
    // 再处理"车在仓库则该装新的一趟"。
    reconcileOnboardWithPlan();
    loadForCurrentTrip();

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

    const bool wasSingleTrip = (plan_.trips.size() == 1);
    // 换计划之前，把"当前计划里已经走过的停靠点"并入记账（三条重规划路径都要做，
    // 否则界面上的「已送达 N 站」会归 0、「停靠 N 站」会缩水成剩余数）
    plan_ = logistics::replanIncremental(config_.graph, config_.vehicles.front(),
                                         remainingOrders(), remainder, currentTimeMin(),
                                         planWeight_,
                                         report, config_.general.trafficTimeIncreaseMin,
                                         stationStock_, onboardGoods());
    syncStationStock();
    // 新路线的起点就是车辆当前位置，推进游标归零
    nodeIndex_ = 0;
    stopCursor_ = 0;
    // 车若正在仓库，就把计划的当前趟装上车——这是"装载"这一物理事件，
    // 且必须发生在每次（重新）规划之后，否则新计划第一趟的货永远上不了车。
    // 换计划后：先把车上的货与计划的当前趟对齐（装载是决策，必须与计划一致），
    // 再处理"车在仓库则该装新的一趟"。
    reconcileOnboardWithPlan();
    loadForCurrentTrip();
    appendLog(wasSingleTrip
                  ? QStringLiteral("  → 增量式重规划：仅重算受影响的路段，其余原样保留")
                  : QStringLiteral("  → 上一版为多趟方案，退回全量重算"));
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
        currentTimeMin(), planWeight_,
        stationStock_, onboardGoods());

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
    syncStationStock();
    nodeIndex_ = 0;
    stopCursor_ = 0;
    // 车若正在仓库，就把计划的当前趟装上车——这是"装载"这一物理事件，
    // 且必须发生在每次（重新）规划之后，否则新计划第一趟的货永远上不了车。
    // 换计划后：先把车上的货与计划的当前趟对齐（装载是决策，必须与计划一致），
    // 再处理"车在仓库则该装新的一趟"。
    reconcileOnboardWithPlan();
    loadForCurrentTrip();
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
        appendLog(QStringLiteral("本次配送已完成：车辆已在仓库 %1")
                      .arg(QString::fromStdString(currentNodeId_)));
        return;   // 已结束，不再刷新
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
        // 换计划后：先把车上的货与计划的当前趟对齐（装载是决策，必须与计划一致），
    // 再处理"车在仓库则该装新的一趟"。
    reconcileOnboardWithPlan();
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
        // 若当前节点在本趟里有装卸记录，把它翻译成人话（界面不必自己反推趟边界）
        QString handled;
        if (nodeIndex_ < plan_.nodeTripIndex.size()) {
            const std::size_t t = plan_.nodeTripIndex[nodeIndex_];
            if (t < plan_.trips.size()) {
                for (const logistics::TransitOp& op : plan_.trips[t].transitOps) {
                    if (op.nodeId != currentNodeId_) {
                        continue;
                    }
                    handled = (op.amountKg >= 0.0)
                                  ? QStringLiteral("，入库暂存 %1kg").arg(op.amountKg, 0, 'f', 0)
                                  : QStringLiteral("，取货 %1kg 二次配发")
                                        .arg(std::fabs(op.amountKg), 0, 'f', 0);
                }
            }
        }
        appendLog(QStringLiteral("经过 %1（到达 %2）%3")
                      .arg(QString::fromStdString(currentNodeId_))
                      .arg(minutesToClock(currentTimeMin_))
                      .arg(handled));
    }

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

void MainWindow::onDebugToggled(bool on) {
    debugOn_ = on;
    appendLog(on ? QStringLiteral("Debug 模式开启：自动模拟路况 / 插单 / 推进")
                 : QStringLiteral("Debug 模式关闭"));
    if (on) {
        // 固定 1 秒一步。原先给过「展示 2 秒 / 快速 1 秒」两档，用户要求
        // 只留快的那一档并删掉选择卡片，以节约工具栏空间。
        const int intervalMs = 1000;
        debugTickMs_ = intervalMs;
        debugTicks_ = 0;
        debugTimer_->start(intervalMs);

        const double tickSec = intervalMs / 1000.0;
        appendLog(QStringLiteral("  紧急订单每 %1 秒最多插入一单，且同时最多 2 单待处理")
                      .arg(config_.general.urgentOrderIntervalSec, 0, 'f', 0));
    } else {
        debugTimer_->stop();
        appendLog(QStringLiteral("Debug 模式关闭：自动模拟已停止"));
    }
}

void MainWindow::onDebugTick() {
    if (!debugOn_) {
        return;   // 定时器必须可随时关闭，且关闭后不再产生任何副作用
    }
    ++debugTicks_;
    onSimulateTraffic();

    // 紧急订单按配置的 urgent_order_interval_sec **节流**，而不是每个 tick 掷骰子。
    // 早先用 1/4 概率逐 tick 触发：3 秒一个 tick 意味着平均十几秒就多一单紧急订单，
    // 连续几单会把整体规划搅乱（人工测试第 15 关的反馈）。
    const double tickSec = debugTickMs_ / 1000.0;
    const double interval = config_.general.urgentOrderIntervalSec;
    const int ticksPerUrgent = std::max(
        1, static_cast<int>(interval / (tickSec > 0.0 ? tickSec : 1.0)));

    // 同时限制"当前待处理的紧急订单"数量，避免连续堆积干扰规划
    std::size_t pendingUrgent = 0;
    for (const Order& order : config_.orders) {
        if (!order.served && order.urgent) {
            ++pendingUrgent;
        }
    }
    const std::size_t kMaxPendingUrgent = 2;

    if (debugTicks_ % ticksPerUrgent == 0 && pendingUrgent < kMaxPendingUrgent) {
        insertUrgentOrderAction();
    }

    // 道路封闭也纳入自动模拟，但**概率压得很低**：它每次都会改动图结构，
    // 连续发生会大幅破坏网络、让演示失去可读性。每 20 个 tick 最多一次。
    if (++debugClosureCounter_ >= 20) {
        debugClosureCounter_ = 0;
        onCloseRandomRoad();
    }
    onAdvanceStop();
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
        //   本计划共 M 趟 = **本次规划**排出了几趟（重规划会重新分批，所以它会变）
        route += QStringLiteral("趟次：已完成 %1 趟 · 当前第 %2 趟 · 本计划共 %3 趟\n")
                     .arg(state_.completedTrips)
                     .arg(state_.tripNumber)
                     .arg(plan_.trips.size());
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
                for (const logistics::TransitOp& op : trip.transitOps) {
                    route += QStringLiteral("    %1 %2 %3kg\n")
                                 .arg(op.amountKg >= 0.0 ? QStringLiteral("入库暂存")
                                                         : QStringLiteral("取货配发"))
                                 .arg(QString::fromStdString(op.nodeId))
                                 .arg(std::fabs(op.amountKg), 0, 'f', 0);
                }
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

        vehicleInfo_->setText(
            QStringLiteral("ID：%1\n起始仓库：%2\n载重上限：%3 kg\n发车：%4\n"
                           "本趟装载：%5 kg（第 %6 趟）\n当前载重：%7 kg\n"
                           "剩余待送：%8 kg")
                .arg(QString::fromStdString(v.id))
                .arg(QString::fromStdString(v.startNodeId))
                .arg(v.capacityKg, 0, 'f', 0)
                .arg(minutesToClock(v.departTimeMin))
                .arg(tripLoad, 0, 'f', 0)
                .arg(tripNo)
                .arg(currentLoadKg(), 0, 'f', 0)
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
            double  peak = 0.0;
            double  now = 0.0;
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
            // 峰值与当前值都随推进实时变化：峰值是"存进去时才比较是否刷新"，
            // 而不是规划时算好的定值；当前值是此刻站内实际存货。
            stockTrace(n.id, row.now, row.peak);
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
                                   new QTableWidgetItem(QString::number(rows[i].peak, 'f', 0)));
            transitTable_->setItem(i, 4,
                                   new QTableWidgetItem(QString::number(rows[i].now, 'f', 0)));
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

void MainWindow::appendLog(const QString& text) {
    logView_->appendPlainText(text);
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

QString MainWindow::vehiclePanelSummary() const {
    return vehicleInfo_ != nullptr ? vehicleInfo_->text() : QString();
}

QString MainWindow::toolbarActionTexts() const {
    QStringList names;
    for (QAction* a : findChildren<QAction*>()) {
        if (a != nullptr && !a->text().isEmpty() && a->isEnabled()) {
            names << a->text();
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
        double stockBefore = 0.0;
        for (const auto& kv : stationStock_) {
            stockBefore += kv.second;
        }
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
        double stockAfter = 0.0;
        for (const auto& kv : stationStock_) {
            stockAfter += kv.second;
        }
        // 寄存机制已随"中转站退出路由"一并移除（见设计 §16 P25）：
        // 站内不再有存货，因此这里只保留"存货不得增长"这一条。
        expect(stockAfter <= stockBefore + 1e-6,
               QStringLiteral("中转站不参与路由后，站内存货不应增长：%1kg -> %2kg")
                   .arg(stockBefore, 0, 'f', 1).arg(stockAfter, 0, 'f', 1));
    }

    // ---- 趟号：已完成趟数只该在换计划时合并，不能与计划内趟号重复相加 ----
    {
        const int before = completedTripOffset();
        for (int i = 0; i < 3; ++i) {
            onAdvanceStop();
        }
        // 纯推进不换计划：已完成趟数不应变化
        expect(completedTripOffset() == before,
               QStringLiteral("纯推进不应改变已完成趟数：%1 -> %2")
                   .arg(before).arg(completedTripOffset()));
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
        // 所以换计划时允许被校准（见 reconcileOnboardWithPlan）。
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
    // 现在每一步前进一个**节点**（含仓库与中转站），故步数取节点总数
    const std::size_t nodeSteps = plan_.nodes.size();
    for (std::size_t i = 0; i < nodeSteps; ++i) {
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

    // ⑤ 车辆位置标记必须与当前位置一致（画布刷新的依据）
    expect(scene_ != nullptr && scene_->vehiclePosition() == currentNodeId_,
           QStringLiteral("画布上的车辆标记与当前位置一致：标记在 %1，当前位置 %2")
               .arg(QString::fromStdString(scene_ != nullptr ? scene_->vehiclePosition()
                                                             : std::string()))
               .arg(QString::fromStdString(currentNodeId_)));

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
