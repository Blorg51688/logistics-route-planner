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
    lateTable_ = new QTableWidget(0, 5, lateDock);
    lateTable_->setHorizontalHeaderLabels(
        {QStringLiteral("订单"), QStringLiteral("配送点"), QStringLiteral("到达"),
         QStringLiteral("窗口"), QStringLiteral("penalty")});
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
    current = 0.0;
    peak = 0.0;
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
    if (plan_.nodes.empty() || plan_.trips.empty()) {
        return 0.0;
    }
    const std::size_t flat = (nodeIndex_ < plan_.nodeTripIndex.size())
                                 ? nodeIndex_
                                 : plan_.nodes.size() - 1;
    const std::size_t t = (nodeIndex_ < plan_.nodeTripIndex.size())
                              ? plan_.nodeTripIndex[nodeIndex_]
                              : 0;
    if (t >= plan_.trips.size()) {
        return 0.0;
    }
    const logistics::Trip& trip = plan_.trips[t];
    double load = trip.loadKg;

    // 该趟在扁平序列中的起点（flatten 对非首趟跳过重复的首节点）
    std::size_t start = 0;
    for (std::size_t k = 0; k < t; ++k) {
        start += plan_.trips[k].nodes.size() - (k == 0 ? 0 : 1);
    }
    const std::size_t offset = (flat >= start) ? flat - start : 0;

    std::size_t stopIdx = 0;
    for (std::size_t i = 0; i <= offset && i < trip.nodes.size(); ++i) {
        if (i < trip.nodeIsStop.size() && trip.nodeIsStop[i] && stopIdx < trip.stops.size()) {
            load = trip.stops[stopIdx].remainingLoadKg;   // 送达后递减
            ++stopIdx;
        }
        for (const logistics::TransitOp& op : trip.transitOps) {
            if (op.nodeId == trip.nodes[i] && op.amountKg > 0.0) {
                load = 0.0;   // 在中转站卸货，车空了
            }
        }
    }
    return load;
}

std::vector<logistics::OnboardItem> MainWindow::onboardGoods() const {
    std::vector<logistics::OnboardItem> items;
    // 车还在仓库（尚未出发）时车上没有货——此时若把"第一趟待装的货"当成在途货，
    // 会给从未装过车的货记上站内存货。
    if (!config_.vehicles.empty() && currentNodeId_ == config_.vehicles.front().startNodeId) {
        return items;
    }
    if (plan_.nodes.empty() || plan_.trips.empty()
        || nodeIndex_ >= plan_.nodeTripIndex.size()) {
        return items;
    }
    const std::size_t t = plan_.nodeTripIndex[nodeIndex_];
    if (t >= plan_.trips.size()) {
        return items;
    }
    const logistics::Trip& trip = plan_.trips[t];

    // 该趟在扁平序列中的起点
    std::size_t start = 0;
    for (std::size_t k = 0; k < t; ++k) {
        start += plan_.trips[k].nodes.size() - (k == 0 ? 0 : 1);
    }
    const std::size_t offset = (nodeIndex_ >= start) ? nodeIndex_ - start : 0;

    // 本趟里"还没经过"的停靠点，就是车上还载着的货
    std::size_t stopIdx = 0;
    for (std::size_t i = 0; i < trip.nodes.size(); ++i) {
        if (i >= trip.nodeIsStop.size() || !trip.nodeIsStop[i]
            || stopIdx >= trip.stops.size()) {
            continue;
        }
        if (i > offset) {
            const std::string nodeId = trip.stops[stopIdx].nodeId;
            double kg = 0.0;
            for (const Order& o : remainingOrders()) {
                if (o.nodeId == nodeId) {
                    kg += o.demandKg;
                }
            }
            if (kg > 1e-9) {
                logistics::OnboardItem item;
                item.nodeId = nodeId;
                item.kg = kg;
                items.push_back(item);
            }
        }
        ++stopIdx;
    }
    return items;
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
    nodeIndex_ = 0;
    stopCursor_ = 0;
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
    plan_ = logistics::replanIncremental(config_.graph, config_.vehicles.front(),
                                         remainingOrders(), remainder, currentTimeMin(),
                                         planWeight_,
                                         report, config_.general.trafficTimeIncreaseMin,
                                         stationStock_, onboardGoods());
    syncStationStock();
    // 新路线的起点就是车辆当前位置，推进游标归零
    nodeIndex_ = 0;
    stopCursor_ = 0;
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

    // 逐个节点前进：仓库、中转站、配送点、返程都算一次位置变化
    ++nodeIndex_;
    currentNodeId_ = plan_.nodes[nodeIndex_];
    if (nodeIndex_ < plan_.nodeArrivalMin.size()) {
        currentTimeMin_ = plan_.nodeArrivalMin[nodeIndex_];
    }

    if (nodeIndex_ < plan_.nodeIsStop.size() && plan_.nodeIsStop[nodeIndex_]
        && stopCursor_ < plan_.stops.size()) {
        const Stop& stop = plan_.stops[stopCursor_];
        ++stopCursor_;
        appendLog(QStringLiteral("送达 %1（到达 %2%3）")
                      .arg(QString::fromStdString(stop.nodeId))
                      .arg(minutesToClock(stop.arrivalMin))
                      .arg(stop.late ? QStringLiteral("，超时 penalty %1min").arg(stop.penaltyMin)
                                     : QString()));
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
        route += QStringLiteral("总 penalty：%1 min\n").arg(plan_.totalPenaltyMin);
        route += QStringLiteral("停靠 %1 站，已送达 %2 站，共 %3 趟\n")
                     .arg(plan_.stops.size())
                     .arg(stopCursor_)
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
                             .arg(i + 1)
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

        double tripLoad = 0.0;
        std::size_t tripNo = 0;
        if (!plan_.nodes.empty() && !plan_.trips.empty()) {
            const std::size_t t = (nodeIndex_ < plan_.nodeTripIndex.size())
                                      ? plan_.nodeTripIndex[nodeIndex_]
                                      : 0;
            if (t < plan_.trips.size()) {
                tripLoad = plan_.trips[t].loadKg;
                tripNo = t + 1;
            }
        }

        double remainingDemand = 0.0;
        for (const Order& o : remainingOrders()) {
            remainingDemand += o.demandKg;
        }

        vehicleInfo_->setText(
            QStringLiteral("ID：%1\n起始仓库：%2\n载重上限：%3 kg\n发车：%4\n"
                           "本趟装载：%5 kg（第 %6 / %7 趟）\n当前载重：%8 kg\n"
                           "剩余待送：%9 kg")
                .arg(QString::fromStdString(v.id))
                .arg(QString::fromStdString(v.startNodeId))
                .arg(v.capacityKg, 0, 'f', 0)
                .arg(minutesToClock(v.departTimeMin))
                .arg(tripLoad, 0, 'f', 0)
                .arg(tripNo)
                .arg(plan_.trips.size())
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
        stopTable_->setItem(i, 0, new QTableWidgetItem(
            QStringLiteral("第 %1").arg(stopTrip[idx])));
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

    // 超时订单：列出**订单号**而不只是配送点——紧急订单也可能超时，
    // 只记录地点用处不大；并用"窗口"替代"剩余载重"（后者在这里没有观测价值）。
    int lateRows = 0;
    if (plan_.status == logistics::PlanStatus::Ok) {
        for (const Stop& s : plan_.stops) {
            if (!s.late) {
                continue;
            }
            lateTable_->setRowCount(lateRows + 1);
            lateTable_->setItem(lateRows, 0,
                                new QTableWidgetItem(orderIdsAt(s.nodeId)));
            lateTable_->setItem(lateRows, 1,
                                new QTableWidgetItem(QString::fromStdString(s.nodeId)));
            lateTable_->setItem(lateRows, 2,
                                new QTableWidgetItem(minutesToClock(s.arrivalMin)));
            lateTable_->setItem(lateRows, 3, new QTableWidgetItem(windowTextAt(s.nodeId)));
            lateTable_->setItem(lateRows, 4, new QTableWidgetItem(QString::number(s.penaltyMin)));
            ++lateRows;
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
