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
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QTableWidget>
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
    strategyBox_->addItem(QStringLiteral("最短距离策略"), QVariant(QStringLiteral("distance")));
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

    auto* lateDock = new QDockWidget(QStringLiteral("超时订单"), this);
    lateDock->setMinimumWidth(360);
    lateTable_ = new QTableWidget(0, 4, lateDock);
    lateTable_->setHorizontalHeaderLabels(
        {QStringLiteral("配送点"), QStringLiteral("到达"), QStringLiteral("penalty"),
         QStringLiteral("剩余载重")});
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
        scene_->highlightRoute(plan_.nodes);
    } else {
        scene_->clearHighlight();
    }
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
                              config_.general.serviceTimeMin, planWeight_);
    // 新计划里的停靠点全部尚未送达，计数必须归零；
    // 当前位置/时刻由显式字段保存，不受本次重算影响
    servedCount_ = 0;

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
    planWeight_ = (strategyBox_->currentData().toString() == QStringLiteral("cost"))
                      ? WeightType::Cost
                      : WeightType::Distance;
    servedCount_ = 0;
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
    if (trigger) {
        appendLog(QStringLiteral("受影响边位于当前路径且增幅达标 -> 自动触发重规划"));
        replan();
    } else {
        appendLog(QStringLiteral("受影响边不在当前路径或增幅不足 -> 不触发重规划"));
        syncScene();   // 拥堵标记需要重绘
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
        currentTimeMin(), config_.general.serviceTimeMin, planWeight_);

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
    if (plan_.status != logistics::PlanStatus::Ok || plan_.stops.empty()) {
        return;
    }
    if (servedCount_ >= plan_.stops.size()) {
        appendLog(QStringLiteral("全部 %1 站已送达").arg(plan_.stops.size()));
        return;
    }
    const Stop& stop = plan_.stops[servedCount_];
    currentNodeId_ = stop.nodeId;
    currentTimeMin_ = stop.departureMin;
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
    ++servedCount_;
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
        // 配置里的 traffic_change_interval_sec 默认 30 秒，那是对路况模型本身的描述；
        // Debug 模式是**演示加速**，30 秒一次会让人以为"根本没反应"。
        // 这里按 10 倍速取，下限 1.5 秒。
        const int rawMs = static_cast<int>(config_.general.trafficChangeIntervalSec * 1000.0);
        const int intervalMs = std::max(1500, rawMs / 10);
        debugTickMs_ = intervalMs;
        debugTicks_ = 0;
        debugTimer_->start(intervalMs);

        const double tickSec = intervalMs / 1000.0;
        const int ticksPerUrgent = std::max(
            1, static_cast<int>(config_.general.urgentOrderIntervalSec / tickSec));
        appendLog(QStringLiteral("Debug 模式开启：每 %1 秒自动模拟一次"
                                 "（配置值 %2 秒，按 10 倍速加速）")
                      .arg(tickSec, 0, 'f', 1)
                      .arg(config_.general.trafficChangeIntervalSec, 0, 'f', 0));
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
    onAdvanceStop();
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
        logistics::Edge edge;
        if (!logistics::makeSyntheticEdge(config_.graph, from, to, edge)) {
            appendLog(QStringLiteral("添加边失败：端点不存在或坐标重合（%1 -> %2）")
                          .arg(fromEdit->text(), toEdit->text()));
            return;
        }
        if (!config_.graph.addEdge(edge)) {
            appendLog(QStringLiteral("添加边失败：%1 -> %2 已存在")
                          .arg(fromEdit->text(), toEdit->text()));
            return;
        }
        appendLog(QStringLiteral("手工添加边 %1 -> %2").arg(fromEdit->text(), toEdit->text()));
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
        route += QStringLiteral("停靠 %1 站，已送达 %2 站\n\n")
                     .arg(plan_.stops.size())
                     .arg(servedCount_);
        route += QStringLiteral("完整序列：\n");
        for (std::size_t i = 0; i < plan_.nodes.size(); ++i) {
            route += QString::fromStdString(plan_.nodes[i]);
            route += (i + 1 == plan_.nodes.size()) ? QString() : QStringLiteral(" -> ");
        }
    } else {
        route = QStringLiteral("不可行：%1").arg(QString::fromStdString(plan_.reason));
    }
    routeInfo_->setPlainText(route);

    // 车辆信息
    if (!config_.vehicles.empty()) {
        const logistics::Vehicle& v = config_.vehicles.front();
        double load = 0.0;
        for (const Order& o : remainingOrders()) {
            load += o.demandKg;
        }
        vehicleInfo_->setText(QStringLiteral("ID：%1\n起始仓库：%2\n载重上限：%3 kg\n"
                                             "发车：%4\n当前载重：%5 kg")
                                  .arg(QString::fromStdString(v.id))
                                  .arg(QString::fromStdString(v.startNodeId))
                                  .arg(v.capacityKg, 0, 'f', 0)
                                  .arg(minutesToClock(v.departTimeMin))
                                  .arg(load, 0, 'f', 0));
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

    // 超时订单
    int lateRows = 0;
    if (plan_.status == logistics::PlanStatus::Ok) {
        for (const Stop& s : plan_.stops) {
            if (!s.late) {
                continue;
            }
            lateTable_->setRowCount(lateRows + 1);
            lateTable_->setItem(lateRows, 0, new QTableWidgetItem(QString::fromStdString(s.nodeId)));
            lateTable_->setItem(lateRows, 1, new QTableWidgetItem(minutesToClock(s.arrivalMin)));
            lateTable_->setItem(lateRows, 2, new QTableWidgetItem(QString::number(s.penaltyMin)));
            lateTable_->setItem(lateRows, 3,
                                new QTableWidgetItem(QString::number(s.remainingLoadKg, 'f', 0)));
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

int MainWindow::dockCount() const {
    return findChildren<QDockWidget*>().size();
}

void MainWindow::showInteractive(int preferredWidth, int preferredHeight) {
    int width = preferredWidth;
    int height = preferredHeight;

    // 关键：窗口尺寸一旦超过屏幕，侧栏就会被推到可见区域之外，
    // 用户会以为"这些面板根本不存在"。直接最大化是唯一稳妥的做法——
    // 不必猜测用户的屏幕有多大，工具栏与五个面板必然全部可见。
    setMinimumSize(900, 600);

    const QScreen* screen = QGuiApplication::primaryScreen();
    const QRect available = (screen != nullptr) ? screen->availableGeometry() : QRect();
    if (available.isValid() && available.width() < preferredWidth + 60) {
        // 屏幕比期望尺寸还小：退回最大化 + 更宽松的最小尺寸
        setMinimumSize(640, 480);
    }

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
