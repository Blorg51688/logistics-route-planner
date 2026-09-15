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
    explicit MainWindow(logistics::Config config, QWidget* parent = nullptr);

    // 交互式显示：把窗口尺寸限制在可用屏幕之内，避免默认尺寸大于屏幕
    // 导致侧栏或工具栏被推到屏幕之外而"看不见"
    void showInteractive(int preferredWidth, int preferredHeight);

    // 供无头渲染验证使用
    void renderToFile(const QString& path, int width, int height);

    // 供 UI 完整性探针使用（--ui-probe）
    int toolbarActionCount() const;
    int dockCount() const;
    // 中转站面板的文本快照，供 --ui-probe 在无头环境下确定性核对
    QString transitPanelSummary() const;

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
    QString            windowTextAt(const std::string& nodeId) const;
    int                currentTimeMin() const;
    std::vector<logistics::Order> remainingOrders() const;

    logistics::Config     config_;
    GraphScene*           scene_ = nullptr;
    QGraphicsView*        view_ = nullptr;
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
    QTextBrowser*  routeInfo_ = nullptr;
    QLabel*        vehicleInfo_ = nullptr;
    QTableWidget*  orderTable_ = nullptr;
    QTableWidget*  transitTable_ = nullptr;
    QTableWidget*  lateTable_ = nullptr;
    QPlainTextEdit* logView_ = nullptr;

    QTimer* debugTimer_ = nullptr;
    bool    debugOn_ = false;
    int     debugTicks_ = 0;
    int     debugTickMs_ = 3000;
};
