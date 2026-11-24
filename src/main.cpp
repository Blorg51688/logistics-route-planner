// 程序入口：加载配置 -> 构建图形场景。
//
// 不带 --render 时打开交互窗口；带 --render 时**离屏渲染成 PNG 后退出**，
// 便于无头环境下验证渲染结果，也用于产出报告配图。
#include <QApplication>
#include <QDir>
#include <QFileInfo>
#include <QGraphicsView>
#include <QImage>
#include <QPainter>
#include <QRegularExpression>
#include <QStringList>

#include <cstdio>
#include <cstdlib>
#include <map>
#include <string>

#include "core/Config.h"
#include "core/RoutePlanner.h"
#include "gui/GraphScene.h"
#include "gui/MainWindow.h"
#include "io/ConfigLoader.h"

#ifndef DEFAULT_CONFIG_PATH
#define DEFAULT_CONFIG_PATH "config/default.ini"
#endif

namespace {

struct Options {
    std::string            configPath = DEFAULT_CONFIG_PATH;
    std::string            renderPath;
    std::string            windowRenderPath;
    std::string            dumpGraph;      // "" / "list" / "matrix" / "both"
    bool                   uiProbe = false;
    bool                   selfCheckActions = false;
    std::string            planSummary;      // "" / "distance" / "cost"
    std::string            dumpPlan;         // "" / "distance" / "cost" / "time"（仅开发/校验用）
    int                    demoRounds = 0;
    std::string            exportLogPath;   // 空表示不导出
    std::string            planStrategy;   // 空表示不高亮任何路线
    bool                   allLabels = false;
    logistics::WeightType  weight = logistics::WeightType::Distance;
    int                    width = 1360;
    int                    height = 900;
};

bool parseWeight(const std::string& text, logistics::WeightType& out) {
    if (text == "distance") {
        out = logistics::WeightType::Distance;
        return true;
    }
    if (text == "time") {
        out = logistics::WeightType::Time;
        return true;
    }
    if (text == "cost") {
        out = logistics::WeightType::Cost;
        return true;
    }
    return false;
}

const char* weightLabel(logistics::WeightType weight) {
    switch (weight) {
        case logistics::WeightType::Distance: return "最短距离";
        case logistics::WeightType::Time:     return "最短耗时";
        case logistics::WeightType::Cost:     return "最低成本";
    }
    return "?";
}

void usage() {
    std::printf(
        "用法: app [选项]\n"
        "  --config PATH              配置文件（默认内置 config/default.ini）\n"
        "  --weight distance|time|cost  权重标签显示的维度（默认 distance）\n"
        "  --plan distance|time|cost  规划并高亮该策略的路线\n"
        "  --labels all|route         权重标签显示全部边还是仅高亮路线（默认 route）\n"
        "  --render PATH.png          离屏渲染图形场景成 PNG 后退出\n"
        "  --render-window PATH.png   离屏渲染完整窗口（工具栏+侧栏）成 PNG 后退出\n"
        "                              （注意 --width/--height 对交互窗口无效——它总是最大化）\n"
        "  --demo N                   渲染窗口前先自动执行 N 次「推进一站」（按站模拟；验证交互后状态）\n"
        "  --export-log PATH          导出本次运行的日志（事件触发 + 到达配送点时的载重明细）\n"
        "  --dump-graph [list|matrix|both]  输出邻接表 / 邻接矩阵后退出（B3）\n"
        "  --ui-probe                 检查工具栏与侧栏是否完整构造后退出\n"
        "  --self-check-actions       自动验证插单不丢单 / 新客户会被配送 / 事件刷单规则 /\n"
        "                             事件刻结算的等价性（A1–A7）后退出\n"
        "  --plan-summary distance|time|cost  打印该策略的规划汇总后退出（供命令行核对）\n"
        "  --dump-plan distance|time|cost  仅开发/校验用，供 oracle 对拍：输出停靠级计划转储后退出\n"
        "  --width N --height N       窗口/图像尺寸\n");
}

} // namespace

int main(int argc, char** argv) {
    QApplication app(argc, argv);

    Options opt;
    const QStringList args = app.arguments();
    for (int i = 1; i < args.size(); ++i) {
        const QString flag = args[i];
        auto takeNext = [&](std::string& dst) {
            if (i + 1 < args.size()) {
                dst = args[++i].toStdString();
            }
        };
        if (flag == "--config") {
            takeNext(opt.configPath);
        } else if (flag == "--render") {
            takeNext(opt.renderPath);
        } else if (flag == "--render-window") {
            takeNext(opt.windowRenderPath);
        } else if (flag == "--ui-probe") {
            opt.uiProbe = true;
        } else if (flag == "--self-check-actions") {
            opt.selfCheckActions = true;
        } else if (flag == "--plan-summary") {
            takeNext(opt.planSummary);
        } else if (flag == "--dump-plan") {
            takeNext(opt.dumpPlan);
        } else if (flag == "--dump-graph") {
            // 值可省略；省略时取 both
            if (i + 1 < args.size() && !args[i + 1].startsWith(QStringLiteral("--"))) {
                takeNext(opt.dumpGraph);
            } else {
                opt.dumpGraph = "both";
            }
            if (opt.dumpGraph != "list" && opt.dumpGraph != "matrix" && opt.dumpGraph != "both") {
                std::fprintf(stderr, "--dump-graph 只接受 list / matrix / both\n");
                return 2;
            }
        } else if (flag == "--export-log") {
            takeNext(opt.exportLogPath);
        } else if (flag == "--demo") {
            std::string value;
            takeNext(value);
            opt.demoRounds = std::atoi(value.c_str());
        } else if (flag == "--labels") {
            std::string value;
            takeNext(value);
            if (value != "all" && value != "route") {
                std::fprintf(stderr, "--labels 只接受 all 或 route\n");
                return 2;
            }
            opt.allLabels = (value == "all");
        } else if (flag == "--plan") {
            takeNext(opt.planStrategy);
        } else if (flag == "--weight") {
            std::string value;
            takeNext(value);
            if (!parseWeight(value, opt.weight)) {
                std::fprintf(stderr, "未知权重维度: %s\n", value.c_str());
                return 2;
            }
        } else if (flag == "--width") {
            std::string value;
            takeNext(value);
            opt.width = std::atoi(value.c_str());
        } else if (flag == "--height") {
            std::string value;
            takeNext(value);
            opt.height = std::atoi(value.c_str());
        } else if (flag == "--help" || flag == "-h") {
            usage();
            return 0;
        } else {
            std::fprintf(stderr, "未知参数: %s\n", flag.toUtf8().constData());
            usage();
            return 2;
        }
    }

    logistics::Config config;
    std::string error;
    if (!logistics::ConfigLoader::load(opt.configPath, config, error)) {
        std::fprintf(stderr, "配置加载失败: %s\n", error.c_str());
        return 1;
    }
    // 走 stderr：图表表示要能被管道干净地取用（如 --dump-graph > out.txt）
    std::fprintf(stderr, "已加载 %s：节点 %zu，边 %zu，订单 %zu\n", opt.configPath.c_str(),
                 config.graph.nodeCount(), config.graph.edgeCount(), config.orders.size());

    // B3：把图的两种文本表示打印出来。此前这两个函数只有测试在调用，
    // 程序没有任何入口能让人看到，验收时无法展示。
    // 供命令行核对：把规划的汇总值打出来，
    // 这样数据集调整后向导的期望值不会过期
    // 供命令行核对：车队口径的规划汇总（规格二 §7.4 的**前六行**）。
    // 逐车指标行（第 3–5 行）归 #43；本票先把路线行按车铺开（#43 的 102 常量需要它）。
    if (!opt.planSummary.empty()) {
        logistics::WeightType planWeight = logistics::WeightType::Distance;
        if (!parseWeight(opt.planSummary, planWeight)) {
            std::fprintf(stderr, "未知策略: %s\n", opt.planSummary.c_str());
            return 2;
        }
        if (config.vehicles.empty()) {
            std::fprintf(stderr, "配置中没有车辆\n");
            return 1;
        }
        const logistics::FleetPlan fleet =
            logistics::planFleet(config.graph, config.vehicles, config.orders, planWeight);
        if (!fleet.allOk) {
            std::printf("不可行：%s\n", fleet.reason.c_str());
            return 0;
        }
        const logistics::RouteMetrics& m = fleet.metrics;

        // 第 1 行：五项汇总（总耗时 = **完工耗时** max(返回) − min(发车)，§3.4 主口径）。
        std::printf("总距离 %.3f km | 总耗时 %.3f min | 总成本 %.3f 元 | "
                    "总 penalty %d min | 停靠 %d 站 · 动作 %d 次\n",
                    fleet.totalDistanceKm, fleet.completionTimeMin, fleet.totalCostYuan,
                    fleet.totalPenaltyMin, m.stopCount, m.actionCount);

        // 第 2 行：三项新指标（车队口径 = Σ命中/Σ分母；在车/利用率按时间加权，峰值取 max）。
        const auto rateText = [](int hit, int total) -> std::string {
            if (total <= 0) {
                return "—";
            }
            char buf[32];
            std::snprintf(buf, sizeof(buf), "%.1f", 100.0 * static_cast<double>(hit) / total);
            return std::string(buf) + "%";
        };
        const auto ratioText = [](int hit, int total) -> std::string {
            return std::to_string(hit) + "/" + std::to_string(total);
        };
        std::printf("取货准时率 %s（%s） | 送达准时率 %s（%s；紧急 %s · 普通 %s） | "
                    "平均在车 %.1f kg（利用率 %.1f%% · 峰值 %.1f kg）\n",
                    rateText(m.pickupOnTimeCount, m.pickupTotal).c_str(),
                    ratioText(m.pickupOnTimeCount, m.pickupTotal).c_str(),
                    rateText(m.deliveryOnTimeCount, m.deliveryTotal).c_str(),
                    ratioText(m.deliveryOnTimeCount, m.deliveryTotal).c_str(),
                    rateText(m.urgentOnTimeCount, m.urgentTotal).c_str(),
                    rateText(m.normalOnTimeCount, m.normalTotal).c_str(),
                    m.avgLoadKg, m.loadUtilization * 100.0, m.peakLoadKg);

        // 第 3 行：**逐单**超时 + **车辆**（规格二 §7.4：多车下"哪台车迟了"是必要信息）。
        std::string late;
        for (std::size_t vi = 0; vi < fleet.vehicles.size(); ++vi) {
            const logistics::RoutePlan& plan = fleet.vehicles[vi];
            for (const logistics::OrderMetric& om : plan.metrics.orders) {
                if (!om.late) {
                    continue;
                }
                if (!late.empty()) {
                    late += "、";
                }
                late += om.orderId + "@" + om.nodeId + "(车 "
                        + config.vehicles[vi].id + ", +" + std::to_string(om.penaltyMin)
                        + "min)";
            }
        }
        std::printf("超时订单：%s\n", late.empty() ? "（无）" : late.c_str());

        // 第 4–6 行：每车一条完整节点序列（oracle 逐车复算的输入；空车只有起点）。
        for (std::size_t vi = 0; vi < fleet.vehicles.size(); ++vi) {
            const logistics::RoutePlan& plan = fleet.vehicles[vi];
            std::printf("路线 %s", config.vehicles[vi].id.c_str());
            if (plan.nodes.empty()) {
                std::printf(" %s", config.vehicles[vi].startNodeId.c_str());
            } else {
                for (const std::string& id : plan.nodes) {
                    std::printf(" %s", id.c_str());
                }
            }
            std::printf("\n");
        }
        return 0;
    }

    // --dump-plan：**仅开发/校验用，供 oracle 对拍**。输出停靠级的机器可读文本：
    // 每台车一段（PLAN <车辆ID> <start> <return> <depart> + 该车 STOP 行 + METRICS 自报值），
    // 末尾一行 FLEET 车队自报值。oracle 按车复算再汇总，逐车 + 汇总都要与 app 一致。
    //
    // 为什么需要它：--plan-summary 的路线行是**节点序列（含途经节点）**，在含重访时
    // oracle 无法由它唯一还原逐单动作归属（同一节点会被合并/重访）。oracle 不猜动作；
    // 由 app 直接吐停靠级真源，oracle 只对给定停靠序列复算算术。
    // **不改变 --plan-summary 的任何可见输出**（这是独立入口）。
    if (!opt.dumpPlan.empty()) {
        logistics::WeightType dumpWeight = logistics::WeightType::Distance;
        if (!parseWeight(opt.dumpPlan, dumpWeight)) {
            std::fprintf(stderr, "未知策略: %s\n", opt.dumpPlan.c_str());
            return 2;
        }
        if (config.vehicles.empty()) {
            std::fprintf(stderr, "配置中没有车辆\n");
            return 1;
        }
        const logistics::FleetPlan fleet =
            logistics::planFleet(config.graph, config.vehicles, config.orders, dumpWeight);
        // 订单窗口按单查（Stop 只存 orderId，窗口在订单表里）。
        std::map<std::string, const logistics::Order*> orderById;
        for (const logistics::Order& o : config.orders) {
            orderById[o.id] = &o;
        }
        // 自报指标的一行（逐车 / 车队同形）：13 字段由计数与算术量组成，
        // 两率在 oracle 侧用「命中/分母」重算，避免「—」与 100% 的口径歧义。
        const auto printMetrics = [](const char* tag, const std::string& id,
                                     const logistics::RoutePlan& plan,
                                     double capacityKg) {
            const logistics::RouteMetrics& m = plan.metrics;
            std::printf("%s %s %.3f %.3f %.3f %d %d %d %d %d %d %d %d %d %d %d %.3f %.3f %.3f\n",
                        tag, id.c_str(), plan.totalDistanceKm, plan.totalTimeMin,
                        plan.totalCostYuan, plan.totalPenaltyMin, m.stopCount, m.actionCount,
                        m.pickupOnTimeCount, m.pickupTotal, m.deliveryOnTimeCount,
                        m.deliveryTotal, m.urgentOnTimeCount, m.urgentTotal,
                        m.normalOnTimeCount, m.normalTotal, m.avgLoadKg,
                        m.loadUtilization * 100.0, m.peakLoadKg);
        };
        for (std::size_t vi = 0; vi < fleet.vehicles.size(); ++vi) {
            const logistics::Vehicle& vehicle = config.vehicles[vi];
            const logistics::RoutePlan& plan = fleet.vehicles[vi];
            if (plan.status != logistics::PlanStatus::Ok) {
                std::printf("PLAN %s - - %d\n", vehicle.id.c_str(), vehicle.departTimeMin);
                std::printf("INFEASIBLE %s %s\n", vehicle.id.c_str(), plan.reason.c_str());
                continue;
            }
            // PLAN <vehicleId> <startNode> <returnNode> <departMin>
            std::printf("PLAN %s %s %s %d\n", vehicle.id.c_str(),
                        plan.nodes.front().c_str(), plan.nodes.back().c_str(),
                        vehicle.departTimeMin);
            // STOP <idx> <nodeId> <phase:U|N> <rawMin> <waitMin> <arrMin> \
            //      <pickups:id,id..> <deliveries:id:ws:we:penalty,..> <remainingLoadKg>
            for (std::size_t i = 0; i < plan.stops.size(); ++i) {
                const logistics::Stop& s = plan.stops[i];
                std::string pickups;
                for (const std::string& id : s.pickups) {
                    if (!pickups.empty()) {
                        pickups += ",";
                    }
                    pickups += id;
                }
                std::string deliveries;
                for (const logistics::StopDelivery& d : s.deliveries) {
                    int ws = 0;
                    int we = 0;
                    const std::map<std::string, const logistics::Order*>::const_iterator it =
                        orderById.find(d.orderId);
                    if (it != orderById.end()) {
                        ws = it->second->windowStartMin;
                        we = it->second->windowEndMin;
                    }
                    if (!deliveries.empty()) {
                        deliveries += ",";
                    }
                    deliveries += d.orderId + ":" + std::to_string(ws) + ":"
                                  + std::to_string(we) + ":" + std::to_string(d.penaltyMin);
                }
                std::printf("STOP %zu %s %c %d %d %d %s %s %.3f\n", i, s.nodeId.c_str(),
                            (s.phase == logistics::StopPhase::Urgent) ? 'U' : 'N',
                            s.rawArrivalMin, s.waitMin, s.arrivalMin,
                            pickups.empty() ? "-" : pickups.c_str(),
                            deliveries.empty() ? "-" : deliveries.c_str(), s.remainingLoadKg);
            }
            printMetrics("METRICS", vehicle.id, plan, vehicle.capacityKg);
        }
        // 车队自报值（与 --plan-summary 第 1–2 行同源）：13 字段 + 完工耗时口径。
        {
            const logistics::RouteMetrics& m = fleet.metrics;
            std::printf("FLEET %.3f %.3f %.3f %d %d %d %d %d %d %d %d %d %d %d %.3f %.3f %.3f\n",
                        fleet.totalDistanceKm, fleet.completionTimeMin, fleet.totalCostYuan,
                        fleet.totalPenaltyMin, fleet.stopCount, fleet.actionCount,
                        m.pickupOnTimeCount, m.pickupTotal, m.deliveryOnTimeCount,
                        m.deliveryTotal, m.urgentOnTimeCount, m.urgentTotal,
                        m.normalOnTimeCount, m.normalTotal, m.avgLoadKg,
                        m.loadUtilization * 100.0, m.peakLoadKg);
        }
        return 0;
    }

    if (!opt.dumpGraph.empty()) {
        if (opt.dumpGraph == "list" || opt.dumpGraph == "both") {
            std::printf("%s\n", config.graph.toAdjacencyListString().c_str());
        }
        if (opt.dumpGraph == "matrix" || opt.dumpGraph == "both") {
            std::printf("%s\n", config.graph.toAdjacencyMatrixString(opt.weight).c_str());
        }
        return 0;
    }

    // --plan / --labels 只在 --render 分支生效。用在交互窗口或 --render-window
    // 上会被静默忽略（用户以为设了、其实没效果），这里直接报错说明。
    // 注意 --weight 不在此列：--dump-graph matrix 也会读它。
    if (opt.renderPath.empty() && (!opt.planStrategy.empty() || opt.allLabels)) {
        std::fprintf(stderr,
                     "--plan / --labels 只在 --render 时生效；"
                     "交互窗口请用界面上的\"规划策略\"与\"权重标签\"控件\n");
        return 2;
    }

    // ---- 仅图形场景的渲染（报告配图用），不构造窗口 ----
    if (!opt.renderPath.empty()) {
        GraphScene scene;
        scene.build(config.graph, opt.weight);
        scene.setAllLabelsVisible(opt.allLabels);

        if (!opt.planStrategy.empty()) {
            logistics::WeightType planWeight = logistics::WeightType::Distance;
            if (!parseWeight(opt.planStrategy, planWeight)) {
                std::fprintf(stderr, "未知规划策略: %s\n", opt.planStrategy.c_str());
                return 2;
            }
            if (config.vehicles.empty()) {
                std::fprintf(stderr, "配置中没有车辆，无法规划\n");
                return 1;
            }
            // 车队口径（#39）：离屏渲染也走 planFleet，三色路线与界面一致。
            const logistics::FleetPlan fleet =
                logistics::planFleet(config.graph, config.vehicles, config.orders, planWeight);
            std::vector<std::vector<std::string> > routes;
            for (const logistics::RoutePlan& plan : fleet.vehicles) {
                routes.push_back(plan.status == logistics::PlanStatus::Ok
                                     ? plan.nodes
                                     : std::vector<std::string>());
            }
            scene.setFleetRoutes(routes, 0);
            if (fleet.allOk) {
                std::printf("规划（%s）：距离 %.1fkm  完工耗时 %.1fmin（车时合计 %.1fmin）  "
                            "成本 %.1f元  penalty %dmin  停靠 %d 站  车 %zu 台\n",
                            weightLabel(planWeight), fleet.totalDistanceKm,
                            fleet.completionTimeMin, fleet.sumVehicleTimeMin,
                            fleet.totalCostYuan, fleet.totalPenaltyMin, fleet.stopCount,
                            fleet.vehicles.size());
            } else {
                std::fprintf(stderr, "规划不可行: %s\n", fleet.reason.c_str());
            }
        }

        const QRectF area = scene.itemsBoundingRect().adjusted(-30, -30, 30, 30);
        QImage image(opt.width, opt.height, QImage::Format_ARGB32);
        image.fill(Qt::white);
        QPainter painter(&image);
        painter.setRenderHint(QPainter::Antialiasing, true);
        scene.render(&painter, QRectF(0, 0, opt.width, opt.height), area);
        painter.end();
        const QFileInfo info(QString::fromStdString(opt.renderPath));
        QDir().mkpath(info.absolutePath());
        if (!image.save(QString::fromStdString(opt.renderPath))) {
            std::fprintf(stderr, "渲染保存失败: %s\n", opt.renderPath.c_str());
            return 1;
        }
        std::printf("已渲染图形场景 %dx%d -> %s\n", opt.width, opt.height,
                    opt.renderPath.c_str());
        return 0;
    }

    // ---- 其余一律走 MainWindow ----
    //
    // 交互启动、窗口截图、demo 动作、UI 探针**共用这一处构造**。
    // 曾经这里的交互路径是另写的一段裸 QGraphicsView，
    // 结果正常启动时工具栏、四个侧栏、日志、Debug 开关全都不存在——
    // 而当时的验证只用 --render-window，恰好走的是正确的那条路径，
    // 因此完全没有发现。把它们合并到同一处，从结构上杜绝再次分叉。
    MainWindow window(config, opt.configPath);

    if (opt.selfCheckActions) {
        return window.runActionSelfCheck() == 0 ? 0 : 1;
    }

    if (opt.uiProbe) {
        window.show();
        QCoreApplication::processEvents();
        const int actions = window.toolbarActionCount();
        const int docks = window.dockCount();
        std::printf("[ui-probe] 工具栏动作 %d 个，停靠面板 %d 个\n", actions, docks);
        // 打印实际的动作名与面板名：向导里让用户点的按钮必须真实存在，
        // 由 `--ui-probe` 逐条比对，避免界面名称与实现漂移。
        const QString actionTexts = window.toolbarActionTexts();
        std::printf("[ui-probe] 动作: %s\n", actionTexts.toUtf8().constData());
        std::printf("[ui-probe] 面板: %s\n", window.dockTitles().toUtf8().constData());
        // 下限随模拟模式重做同步：原「Debug 模式」更名并新增「推进一刻」「按时间模拟」，
        // 工具栏动作由 15 个增至 17 个（见 docs/records/specs/sim-time-events.md §7.1）。
        if (actions < 17 || docks < 5) {
            std::fprintf(stderr,
                         "[ui-probe] UI 不完整：预期至少 17 个工具栏动作、5 个停靠面板\n");
            return 1;
        }
        // 动作名清单同步：这些名字是向导第 7 关让用户去点的按钮，必须**真实存在**。
        // 读的是界面上真正的动作名（findChildren<QAction*>），不是把预期列表再背一遍。
        for (const char* need : {"按站模拟", "按时间模拟", "推进一刻", "推进一站"}) {
            const QString name = QString::fromUtf8(need);
            if (!actionTexts.contains(name)) {
                std::fprintf(stderr, "[ui-probe] 工具栏缺少动作「%s」\n", need);
                return 1;
            }
        }
        // 中转站面板必须为图中每个中转站列一行，且子网络编号非 0
        // （D17：sub_network_id 必须真正接上行为，不能是死字段）
        std::size_t transitNodes = 0;
        for (const logistics::Node& n : config.graph.nodes()) {
            if (n.type == logistics::NodeType::Transit) {
                ++transitNodes;
            }
        }
        const QString panel = window.transitPanelSummary();
        const std::size_t listed = static_cast<std::size_t>(panel.count(QLatin1Char('\n')));
        if (listed != transitNodes) {
            std::fprintf(stderr, "[ui-probe] 中转站面板行数 %zu != 图中中转站数 %zu\n",
                         listed, transitNodes);
            return 1;
        }
        // 逐行只看**子网络**那一列（第 2 列）。先前用 contains(" | 0 | ") 太松，
        const QStringList panelRows = panel.split(QLatin1Char('\n'), Qt::SkipEmptyParts);
        for (const QString& row : panelRows) {
            const QStringList cells = row.split(QStringLiteral(" | "));
            if (cells.size() >= 2 && cells[1].trimmed() == QStringLiteral("0")) {
                const QByteArray bad = row.toUtf8();
                std::fprintf(stderr, "[ui-probe] 中转站子网络编号为 0：%s\n", bad.constData());
                return 1;
            }
        }

        // 中转站面板的内容快照：无头环境下据此确定性核对（不必靠肉眼裁图）
        //
        // 4 列：中转站 / 子网络 / 下属配送点 / 当前暂存货(kg)（issue #10）。
        // 「当前库存」旧列随 D23 缓冲退役；本列反映紧急降载暂存货（无暂存为 0）。
        // 守卫读**界面真实表头**与**实际单元格**，不重算公式、不背预期清单。
        const QStringList transitHeaders = window.transitPanelHeaders();
        bool hasStashHeader = false;
        for (const QString& h : transitHeaders) {
            if (h.contains(QStringLiteral("当前暂存货"))) {
                hasStashHeader = true;
            }
        }
        if (transitHeaders.size() != 4 || !hasStashHeader) {
            const QByteArray headers = transitHeaders.join(QStringLiteral(" | ")).toUtf8();
            std::fprintf(stderr,
                         "[ui-probe] 中转站面板应有 4 列且含「当前暂存货」表头，实际 [%s]\n",
                         headers.constData());
            return 1;
        }
        for (const QString& row : panelRows) {
            const QStringList cells = row.split(QStringLiteral(" | "));
            if (cells.size() != 4) {
                const QByteArray bad = row.toUtf8();
                std::fprintf(stderr,
                             "[ui-probe] 中转站面板应有 4 列，实际不足：%s\n",
                             bad.constData());
                return 1;
            }
            bool numeric = false;
            cells[3].trimmed().toDouble(&numeric);
            if (!numeric) {
                const QByteArray bad = row.toUtf8();
                std::fprintf(stderr,
                             "[ui-probe] 中转站「当前暂存货」列必须是数字：%s\n",
                             bad.constData());
                return 1;
            }
        }
        const QByteArray transit = panel.toUtf8();
        std::printf("[ui-probe] 中转站面板（%s）:\n%s",
                    transitHeaders.join(QStringLiteral("|")).toUtf8().constData(),
                    transit.constData());
        // 车辆面板（G11）：页签数 == vehicles.size()，每页含「固定信息 / 运行状态」
        // 两块标题；**逐车**核对 ID / 容量 / 两因子与配置一致，且逐车
        // 「车上载重 ≤ 该车容量」（不得再用 vehicles.front() 一笔带过）。
        //
        // 历史：用户手工测试发现面板把"剩余待送总量 740kg"当成"当前载重"显示，
        // 而载重上限只有 200kg —— 数据层没错，是显示口径错了，所以必须查显示值本身
        //（读用户看得到的文本，不重算公式）。
        {
            const int vehicleTabs = window.vehicleTabCount();
            if (config.vehicles.empty() || vehicleTabs != static_cast<int>(config.vehicles.size())) {
                std::fprintf(stderr, "[ui-probe] 车辆信息页签数 %d != 车辆数 %zu\n",
                             vehicleTabs, config.vehicles.size());
                return 1;
            }
            std::printf("[ui-probe] 车辆信息 %d 个标签页：%s\n", vehicleTabs,
                        window.vehicleTabTitles().toUtf8().constData());
            const QStringList tabTitles =
                window.vehicleTabTitles().split(QStringLiteral(" | "));
            for (std::size_t vi = 0; vi < config.vehicles.size(); ++vi) {
                const logistics::Vehicle& v = config.vehicles[vi];
                // 页签标题 = `<车辆 ID> · <容量>kg`
                const QString title = QStringLiteral("%1 · %2kg")
                                          .arg(QString::fromStdString(v.id))
                                          .arg(v.capacityKg, 0, 'f', 0);
                if (vi < static_cast<std::size_t>(tabTitles.size())
                    && tabTitles[static_cast<int>(vi)] != title) {
                    std::fprintf(stderr,
                                 "[ui-probe] 车辆页签标题不符：预期「%s」，实际「%s」\n",
                                 title.toUtf8().constData(),
                                 tabTitles[static_cast<int>(vi)].toUtf8().constData());
                    return 1;
                }
                const QString seg = window.vehiclePanelSection(static_cast<int>(vi));
                // 每页两块标题：固定信息 / 运行状态
                if (!seg.contains(QStringLiteral("固定信息"))
                    || !seg.contains(QStringLiteral("运行状态"))) {
                    std::fprintf(stderr,
                                 "[ui-probe] 车辆 %s 的页签缺少「固定信息 / 运行状态」两块标题：\n%s\n",
                                 v.id.c_str(), seg.toUtf8().constData());
                    return 1;
                }
                // 固定信息逐项与配置一致：ID / 容量 / 两因子
                const auto fieldOk = [&](const char* label, double want) -> bool {
                    const QRegularExpression re(QString::fromUtf8(label)
                                                + QStringLiteral("：([0-9.]+)"));
                    const QRegularExpressionMatch m = re.match(seg);
                    return m.hasMatch() && std::fabs(m.captured(1).toDouble() - want) <= 1e-6;
                };
                const QRegularExpression idRe(QStringLiteral("ID：([^\n：]+)"));
                const QRegularExpressionMatch idm = idRe.match(seg);
                if (!idm.hasMatch() || idm.captured(1).trimmed() != QString::fromStdString(v.id)
                    || !fieldOk("载重上限", v.capacityKg) || !fieldOk("耗时倍率", v.timeFactor)
                    || !fieldOk("成本倍率", v.costFactor)) {
                    std::fprintf(stderr,
                                 "[ui-probe] 车辆 %s 页签固定信息与配置不符"
                                 "（ID/载重上限/耗时倍率/成本倍率）：\n%s\n",
                                 v.id.c_str(), seg.toUtf8().constData());
                    return 1;
                }
                // G13：运行状态**六项**齐备（§7.5②）。
                {
                    const char* need[] = {"当前位置：", "下一站：", "阶段：", "车上载重：",
                                          "剩余待送", "已发生 penalty："};
                    for (const char* key : need) {
                        if (!seg.contains(QString::fromUtf8(key))) {
                            std::fprintf(stderr,
                                         "[ui-probe] 车辆 %s 运行状态缺少「%s」：\n%s\n",
                                         v.id.c_str(), key, seg.toUtf8().constData());
                            return 1;
                        }
                    }
                    const QRegularExpression phaseRe(QStringLiteral("阶段：([^\n]+)"));
                    const QRegularExpressionMatch pm = phaseRe.match(seg);
                    const QString phase = pm.hasMatch() ? pm.captured(1).trimmed() : QString();
                    if (phase != QStringLiteral("紧急") && phase != QStringLiteral("普通")
                        && phase != QStringLiteral("已收工")) {
                        std::fprintf(stderr, "[ui-probe] 车辆 %s 阶段列不是 紧急/普通/已收工：%s\n",
                                     v.id.c_str(), phase.toUtf8().constData());
                        return 1;
                    }
                    const QRegularExpression doneRe(
                        QStringLiteral("已送达 ([0-9]+) 单 · 剩余待送 ([0-9.]+) kg"));
                    if (!doneRe.match(seg).hasMatch()) {
                        std::fprintf(stderr,
                                     "[ui-probe] 车辆 %s 缺少「已送达 X 单 · 剩余待送 Y kg」：\n%s\n",
                                     v.id.c_str(), seg.toUtf8().constData());
                        return 1;
                    }
                }
                // 逐车「车上载重 ≤ 该车容量」（**该车**的 capacity，不用 front()）
                const QRegularExpression totalRe(
                    QStringLiteral("车上载重：([0-9.]+) kg"));
                const QRegularExpressionMatch total = totalRe.match(seg);
                if (!total.hasMatch()) {
                    std::fprintf(stderr, "[ui-probe] 车辆 %s 页签缺少「车上载重」：\n%s\n",
                                 v.id.c_str(), seg.toUtf8().constData());
                    return 1;
                }
                const double shownTotal = total.captured(1).toDouble();
                if (shownTotal > v.capacityKg + 1e-6) {
                    std::fprintf(stderr,
                                 "[ui-probe] 车辆 %s 面板显示的载重超过该车上限："
                                 "车上载重 %.1f > %.1f\n",
                                 v.id.c_str(), shownTotal, v.capacityKg);
                    return 1;
                }
                std::printf("[ui-probe] 车辆 %s 车上载重 %.1fkg <= 该车容量 %.1fkg\n",
                            v.id.c_str(), shownTotal, v.capacityKg);
            }
            // 把逐车面板文本打出来（报告/人工核对用；也是"用户看得到什么"的证据）。
            std::printf("[ui-probe] 车辆面板（%d 页）：\n%s\n", vehicleTabs,
                        window.vehiclePanelSummary().toUtf8().constData());
            if (window.vehiclePanelSummary().contains(QStringLiteral("本趟出发装载"))) {
                std::fprintf(stderr, "[ui-probe] 车辆面板仍含已退役的「本趟出发装载」\n");
                return 1;
            }

            // 时间必须常驻可见：软件内时间是事件刻/窗口/penalty 的共同基准，
            // 只在日志里出现等于"一滚就没了"。这里查状态栏确实带着它。
            const QString clockText = window.statusClockSummary();
            if (!clockText.contains(QStringLiteral("软件内时间"))
                || !clockText.contains(QStringLiteral(":"))) {
                std::fprintf(stderr,
                             "[ui-probe] 状态栏没有常驻显示软件内时间，实际：%s\n",
                             clockText.toUtf8().constData());
                return 1;
            }
            std::printf("[ui-probe] 状态栏时间：%s\n", clockText.toUtf8().constData());
        }

        // ---- G8：停靠明细 **9 列**、无「趟」、含 序号/阶段/动作/余货(kg)（MC-E01）----
        const QString stopPanel = window.stopPanelSummary();
        const std::size_t stopRows =
            static_cast<std::size_t>(stopPanel.count(QLatin1Char('\n')));
        if (stopRows == 0) {
            std::fprintf(stderr, "[ui-probe] 停靠明细面板为空\n");
            return 1;
        }
        std::printf("[ui-probe] 停靠明细 %zu 行（车辆|序号|阶段|配送点|动作|原始到达|等待(分)|送达|余货(kg)）\n",
                    stopRows);
        {
            const QStringList lines = stopPanel.split(QLatin1Char('\n'), Qt::SkipEmptyParts);
            for (int i = 0; i < lines.size() && i < 3; ++i) {
                std::printf("[ui-probe]   %s\n", lines[i].toUtf8().constData());
            }
            // 表头守卫（G8）：停靠明细必须**恰好 9 列**、列名与数据字段一一对应，
            // 且**无「趟」列**（趟结构退役，列改为 车辆/序号）。
            // 历史：服务时间移除时只改数据填充、漏改表头 → 表头 7 列而数据只填 6 列；
            // 这类错位只有读**真实表头**才看得见。
            const QStringList headers = window.stopPanelHeaders();
            const QStringList expect{
                QStringLiteral("车辆"),     QStringLiteral("序号"),   QStringLiteral("阶段"),
                QStringLiteral("配送点"),   QStringLiteral("动作"),   QStringLiteral("原始到达"),
                QStringLiteral("等待(分)"), QStringLiteral("送达"),   QStringLiteral("余货(kg)")};
            if (headers != expect) {
                std::fprintf(stderr,
                             "[ui-probe] 停靠明细表头不符（应 9 列）：实际 [%s]，预期 [%s]\n",
                             headers.join(QStringLiteral(" | ")).toUtf8().constData(),
                             expect.join(QStringLiteral(" | ")).toUtf8().constData());
                return 1;
            }
            for (const QString& h : headers) {
                if (h.contains(QStringLiteral("趟"))) {
                    std::fprintf(stderr, "[ui-probe] 停靠明细不应再有「趟」列：%s\n",
                                 h.toUtf8().constData());
                    return 1;
                }
            }
            // 每行 9 格非空；车辆列 ∈ 配置车辆；序号纯整数且逐车 +1 单调；
            // 阶段 ∈ {紧急, 普通}；动作 = 取<a>·送<b>（计数，不显示订单 ID）；
            // Σ取 == Σ送 == 取送对数（结构关系，见 MC-E01）。
            const QRegularExpression actionRe(QStringLiteral("^取([0-9]+)·送([0-9]+)$"));
            long long sumPick = 0;
            long long sumDeliver = 0;
            int prevSeq = 0;
            QString prevVehicle;
            for (const QString& line : lines) {
                const QStringList cells = line.split(QStringLiteral(" | "));
                if (cells.size() != headers.size()) {
                    std::fprintf(stderr, "[ui-probe] 停靠明细行列数 %d != 表头列数 %d：%s\n",
                                 int(cells.size()), int(headers.size()),
                                 line.toUtf8().constData());
                    return 1;
                }
                for (const QString& c : cells) {
                    if (c.isEmpty()) {
                        std::fprintf(stderr, "[ui-probe] 停靠明细出现空白单元格：%s\n",
                                     line.toUtf8().constData());
                        return 1;
                    }
                }
                const QString& vehicle = cells[0];
                bool isConfigVehicle = false;
                for (const logistics::Vehicle& v : config.vehicles) {
                    if (QString::fromStdString(v.id) == vehicle) {
                        isConfigVehicle = true;
                        break;
                    }
                }
                if (!isConfigVehicle) {
                    std::fprintf(stderr, "[ui-probe] 停靠明细「车辆」列不是配置里的车辆：%s\n",
                                 vehicle.toUtf8().constData());
                    return 1;
                }
                bool seqOk = false;
                const int seq = cells[1].toInt(&seqOk);
                if (!seqOk || seq < 1) {
                    std::fprintf(stderr, "[ui-probe] 停靠明细「序号」不是正整数：%s\n",
                                 cells[1].toUtf8().constData());
                    return 1;
                }
                const int wantSeq = (vehicle == prevVehicle) ? prevSeq + 1 : 1;
                if (seq != wantSeq) {
                    std::fprintf(stderr, "[ui-probe] 停靠明细「序号」不单调（车辆 %s 期望 %d，实际 %d）\n",
                                 vehicle.toUtf8().constData(), wantSeq, seq);
                    return 1;
                }
                prevVehicle = vehicle;
                prevSeq = seq;
                if (cells[2] != QStringLiteral("紧急") && cells[2] != QStringLiteral("普通")) {
                    std::fprintf(stderr, "[ui-probe] 停靠明细「阶段」列不是 紧急/普通：%s\n",
                                 cells[2].toUtf8().constData());
                    return 1;
                }
                const QRegularExpressionMatch am = actionRe.match(cells[4]);
                if (!am.hasMatch()) {
                    std::fprintf(stderr, "[ui-probe] 停靠明细「动作」列不是 取<a>·送<b>：%s\n",
                                 cells[4].toUtf8().constData());
                    return 1;
                }
                sumPick += am.captured(1).toLongLong();
                sumDeliver += am.captured(2).toLongLong();
                bool loadNumeric = false;
                cells[8].toDouble(&loadNumeric);
                if (!loadNumeric) {
                    std::fprintf(stderr, "[ui-probe] 停靠明细「余货(kg)」不是数字：%s\n",
                                 cells[8].toUtf8().constData());
                    return 1;
                }
            }
            const long long pairs = static_cast<long long>(config.orders.size());
            if (sumPick != sumDeliver || sumPick != pairs) {
                std::fprintf(stderr,
                             "[ui-probe] 停靠明细动作计数 Σ取 %lld / Σ送 %lld != 取送对数 %lld\n",
                             sumPick, sumDeliver, pairs);
                return 1;
            }
            std::printf("[ui-probe] 停靠明细表头 9 列、无「趟」；每行 9 格非空；"
                        "Σ取 = Σ送 = %lld（取送对数）\n",
                        pairs);
        }

        // ---- G9：路线信息面板含四片段（MC-E03）----
        {
            const QString routePanel = window.routePanelSummary();
            const QRegularExpression stopsActions(
                QStringLiteral("停靠 ([0-9]+) 站 · 动作 ([0-9]+) 次"));
            const QRegularExpressionMatch sam = stopsActions.match(routePanel);
            if (!sam.hasMatch() || !routePanel.contains(QStringLiteral("取货准时率"))
                || !routePanel.contains(QStringLiteral("送达准时率"))
                || !routePanel.contains(QStringLiteral("平均在车"))) {
                std::fprintf(stderr,
                             "[ui-probe] 路线信息面板缺少四项新指标片段"
                             "（停靠·动作 / 取货准时率 / 送达准时率 / 平均在车）：\n%s\n",
                             routePanel.toUtf8().constData());
                return 1;
            }
            // 趟次三段与「各趟」列表已退役（MC-E03），面板里不得再出现。
            if (routePanel.contains(QStringLiteral("趟次"))
                || routePanel.contains(QStringLiteral("各趟"))) {
                std::fprintf(stderr, "[ui-probe] 路线信息面板仍含已退役的「趟次 / 各趟」：\n%s\n",
                             routePanel.toUtf8().constData());
                return 1;
            }
            // G9（规格二 §7.5①）：`── 逐车指标 ──` 小节 + **每车一行 7 项**。
            if (!routePanel.contains(QStringLiteral("── 逐车指标 ──"))) {
                std::fprintf(stderr, "[ui-probe] 路线信息面板缺少「── 逐车指标 ──」小节：\n%s\n",
                             routePanel.toUtf8().constData());
                return 1;
            }
            const QStringList routeLines =
                routePanel.split(QLatin1Char('\n'), Qt::SkipEmptyParts);
            int metricRows = 0;
            const QRegularExpression vehicleLine(
                QStringLiteral("^([A-Za-z0-9_]+)  距离 ([0-9.]+) km · 耗时 ([0-9.]+) min · "
                               "成本 ([0-9.]+) 元 · penalty ([0-9]+) · 停靠 ([0-9]+) · "
                               "利用率 ([0-9.]+)% · 取货 (.+) · 送达 (.+)$"));
            for (const QString& line : routeLines) {
                if (vehicleLine.match(line).hasMatch()) {
                    ++metricRows;
                }
            }
            if (metricRows != static_cast<int>(config.vehicles.size())) {
                std::fprintf(stderr,
                             "[ui-probe] 逐车指标行数 %d != 车辆数 %zu：\n%s\n", metricRows,
                             config.vehicles.size(), routePanel.toUtf8().constData());
                return 1;
            }
            std::printf("[ui-probe] 路线信息面板：停靠 %s 站 · 动作 %s 次（四片段齐备）；"
                        "逐车指标 %d 行（每行 7 项）\n",
                        sam.captured(1).toUtf8().constData(),
                        sam.captured(2).toUtf8().constData(), metricRows);
        }

        // ---- G10：超时订单面板 **8 列**、逐单一行（MC-E02）----
        {
            const QStringList lateHeaders = window.latePanelHeaders();
            const QStringList expectLate{
                QStringLiteral("车辆"),        QStringLiteral("订单"),   QStringLiteral("配送点"),
                QStringLiteral("档位"),        QStringLiteral("状态"),   QStringLiteral("到达"),
                QStringLiteral("窗口"),        QStringLiteral("penalty(min)")};
            if (lateHeaders != expectLate) {
                std::fprintf(stderr,
                             "[ui-probe] 超时订单表头不符（应 8 列）：实际 [%s]，预期 [%s]\n",
                             lateHeaders.join(QStringLiteral(" | ")).toUtf8().constData(),
                             expectLate.join(QStringLiteral(" | ")).toUtf8().constData());
                return 1;
            }
            const QStringList lateLines =
                window.latePanelSummary().split(QLatin1Char('\n'), Qt::SkipEmptyParts);
            // 行数 == 迟到订单数（G10）：逐单一行，不得把同停靠多单压成一行。
            const int expectLateRows = window.lateOrderCount();
            if (int(lateLines.size()) != expectLateRows) {
                std::fprintf(stderr, "[ui-probe] 超时面板行数 %d != 迟到订单数 %d\n",
                             int(lateLines.size()), expectLateRows);
                return 1;
            }
            const QRegularExpression windowRe(
                QStringLiteral("^[0-9]{2}:[0-9]{2}-[0-9]{2}:[0-9]{2}$"));
            for (const QString& line : lateLines) {
                const QStringList cells = line.split(QStringLiteral(" | "));
                if (cells.size() != lateHeaders.size()) {
                    std::fprintf(stderr, "[ui-probe] 超时订单行列数 %d != 表头列数 %d：%s\n",
                                 int(cells.size()), int(lateHeaders.size()),
                                 line.toUtf8().constData());
                    return 1;
                }
                for (const QString& c : cells) {
                    if (c.isEmpty()) {
                        std::fprintf(stderr, "[ui-probe] 超时订单出现空白单元格：%s\n",
                                     line.toUtf8().constData());
                        return 1;
                    }
                }
                // 逐单一行：订单列是**单张**订单 ID（旧实现用 '/' 把同停靠多单拼成一格）。
                if (cells[1].contains(QLatin1Char('/'))) {
                    std::fprintf(stderr, "[ui-probe] 超时订单未逐单一行（订单列含 '/' 合并）：%s\n",
                                 line.toUtf8().constData());
                    return 1;
                }
                if (cells[3] != QStringLiteral("紧急") && cells[3] != QStringLiteral("普通")) {
                    std::fprintf(stderr, "[ui-probe] 超时订单「档位」列不是 紧急/普通：%s\n",
                                 line.toUtf8().constData());
                    return 1;
                }
                // 窗口 = 该单自己的 [ws, we]（逐单窗口，不是同停靠多单的并集）。
                if (!windowRe.match(cells[6]).hasMatch()) {
                    std::fprintf(stderr, "[ui-probe] 超时订单「窗口」列不是 起-止 时刻：%s\n",
                                 line.toUtf8().constData());
                    return 1;
                }
                bool penOk = false;
                const int pen = cells[7].toInt(&penOk);
                if (!penOk || pen <= 0) {
                    std::fprintf(stderr, "[ui-probe] 超时订单「penalty(min)」不是正整数：%s\n",
                                 line.toUtf8().constData());
                    return 1;
                }
            }
            if (!lateLines.isEmpty()) {
                std::printf("[ui-probe] 超时订单摘录（逐单一行，窗口 = 该单自己的）：\n");
                for (const QString& line : lateLines) {
                    std::printf("[ui-probe]   %s\n", line.toUtf8().constData());
                }
            }
            std::printf("[ui-probe] 超时订单面板表头 8 列（含车辆/档位），%d 行 == 迟到订单数\n",
                        expectLateRows);
        }

        // ---- G15：画布三色归属快照（规格二 §7.5⑤）----
        // 每条车路线边都要有归属；共享边归属 = 行序最前那台车；选中车加粗。
        {
            const QString ownership = window.canvasOwnershipSnapshot();
            const QStringList rows =
                ownership.split(QLatin1Char('\n'), Qt::SkipEmptyParts);
            if (rows.isEmpty()) {
                std::fprintf(stderr, "[ui-probe] 画布归属快照为空（没有任何车路线边）\n");
                return 1;
            }
            const QRegularExpression rowRe(
                QStringLiteral("^([A-Za-z0-9_]+)->([A-Za-z0-9_]+) owner=([A-Za-z0-9_]+) "
                               "selected=([01]) all=([A-Za-z0-9_,]+)$"));
            // 归属必须按行序最前：同一有向边若被多台车经过，owner 必须是行序最靠前的车。
            std::vector<std::string> order;
            for (const logistics::Vehicle& v : config.vehicles) {
                order.push_back(v.id);
            }
            const auto rankOf = [&order](const QString& id) -> int {
                for (std::size_t i = 0; i < order.size(); ++i) {
                    if (QString::fromStdString(order[i]) == id) {
                        return static_cast<int>(i);
                    }
                }
                return -1;
            };
            int selectedRows = 0;
            for (const QString& line : rows) {
                const QRegularExpressionMatch m = rowRe.match(line);
                if (!m.hasMatch()) {
                    std::fprintf(stderr, "[ui-probe] 画布归属行格式不符：%s\n",
                                 line.toUtf8().constData());
                    return 1;
                }
                if (rankOf(m.captured(3)) < 0) {
                    std::fprintf(stderr, "[ui-probe] 画布归属车不是配置里的车辆：%s\n",
                                 line.toUtf8().constData());
                    return 1;
                }
                if (m.captured(4) == QStringLiteral("1")) {
                    ++selectedRows;
                }
                // 共享边归属 = **行序最前**那台车（R8 的反向验证点：改成最后行序必须红）
                const QStringList owners = m.captured(5).split(QLatin1Char(','));
                if (owners.isEmpty()) {
                    std::fprintf(stderr, "[ui-probe] 画布归属行没有 all= 清单：%s\n",
                                 line.toUtf8().constData());
                    return 1;
                }
                int bestRank = -1;
                for (const QString& oid : owners) {
                    const int rank = rankOf(oid);
                    if (rank < 0) {
                        std::fprintf(stderr, "[ui-probe] 画布 all= 里有非配置车辆：%s\n",
                                     line.toUtf8().constData());
                        return 1;
                    }
                    if (bestRank < 0 || rank < bestRank) {
                        bestRank = rank;
                    }
                }
                if (rankOf(m.captured(3)) != bestRank) {
                    std::fprintf(stderr,
                                 "[ui-probe] 共享边归属不是行序最前那台车：%s（期望 rank %d）\n",
                                 line.toUtf8().constData(), bestRank);
                    return 1;
                }
            }
            if (selectedRows == 0) {
                std::fprintf(stderr,
                             "[ui-probe] 画布没有任何「属于选中车」的边（选中加粗失效）\n");
                return 1;
            }
            std::printf("[ui-probe] 画布归属 %zu 条边（归属按行序最前，选中车加粗 %d 条）\n",
                        rows.size(), selectedRows);
        }

        std::printf("[ui-probe] 通过\n");
        return 0;
    }

    if (opt.demoRounds > 0) {
        window.runDemoActions(opt.demoRounds);
        if (!opt.exportLogPath.empty()) {
            const bool ok = window.exportRunLogTo(QString::fromStdString(opt.exportLogPath));
            if (!ok) {
                std::fprintf(stderr, "[export-log] 无法写入 %s\n", opt.exportLogPath.c_str());
                return 1;
            }
            std::printf("[export-log] 已导出 -> %s\n", opt.exportLogPath.c_str());
        }
    }

    if (!opt.windowRenderPath.empty()) {
        window.renderToFile(QString::fromStdString(opt.windowRenderPath), opt.width, opt.height);
        std::printf("已渲染窗口 -> %s\n", opt.windowRenderPath.c_str());
        return 0;
    }

    window.showInteractive();
    return app.exec();
}