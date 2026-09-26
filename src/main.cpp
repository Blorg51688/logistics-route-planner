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
        "  --self-check-actions       自动验证插单不丢单 / 新客户会被配送后退出\n"
        "  --plan-summary distance|time|cost  打印该策略的规划汇总后退出（供人工测试核对）\n"
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
    // 供人工测试向导取用：把规划的汇总值打出来，向导据此显示"应看到什么"，
    // 这样数据集调整后向导的期望值不会过期
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
        // 命令行首次规划：站里还没有任何缓冲库存（仓库发缓冲货要跑完一趟才可能寄存），
        // 故显式传空库存 —— 与 planRoute 的默认值等价，只为让调用形状与其余四个入口一致。
        const std::map<std::string, double> initialStock;
        const logistics::RoutePlan plan =
            logistics::planRoute(config.graph, config.vehicles.front(), config.orders,
                                 planWeight, initialStock);
        if (plan.status != logistics::PlanStatus::Ok) {
            std::printf("不可行：%s\n", plan.reason.c_str());
            return 0;
        }
        std::printf("总距离 %.3f km | 总耗时 %.3f min | 总成本 %.3f 元 | "
                    "总 penalty %d min | 停靠 %zu 站",
                    plan.totalDistanceKm, plan.totalTimeMin, plan.totalCostYuan,
                    plan.totalPenaltyMin, plan.stops.size());
        if (plan.trips.size() > 1) {
            std::printf(" | 共 %zu 趟", plan.trips.size());
        }
        std::printf("\n");

        // 超时停靠点：向导据此显示"应看到什么"，避免把具体站点写死在脚本里
        // （数据一改就过期，第 6 轮的第 8 关就是这么误报的）
        std::string late;
        for (const logistics::Stop& s : plan.stops) {
            if (!s.late) {
                continue;
            }
            if (!late.empty()) {
                late += "、";
            }
            late += s.nodeId + "(+" + std::to_string(s.penaltyMin) + "min)";
        }
        std::printf("超时停靠点：%s\n", late.empty() ? "（无）" : late.c_str());
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
            // 同上：首次规划时站内无库存，显式传空以统一调用形状。
            const std::map<std::string, double> initialStock;
            const logistics::RoutePlan plan =
                logistics::planRoute(config.graph, config.vehicles.front(), config.orders,
                                     planWeight, initialStock);
            if (plan.status == logistics::PlanStatus::Ok) {
                scene.highlightRoute(plan.nodes);
                std::printf("规划（%s）：距离 %.1fkm  耗时 %.1fmin  成本 %.1f元  "
                            "penalty %dmin  停靠 %zu 站\n",
                            weightLabel(planWeight), plan.totalDistanceKm, plan.totalTimeMin,
                            plan.totalCostYuan, plan.totalPenaltyMin, plan.stops.size());
            } else {
                std::fprintf(stderr, "规划不可行: %s\n", plan.reason.c_str());
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
    MainWindow window(config);

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
        // 由 check_wizard_ui_names.py 逐条比对，避免向导指示一个不存在的按钮。
        const QString actionTexts = window.toolbarActionTexts();
        std::printf("[ui-probe] 动作: %s\n", actionTexts.toUtf8().constData());
        std::printf("[ui-probe] 面板: %s\n", window.dockTitles().toUtf8().constData());
        // 下限随模拟模式重做同步：原「Debug 模式」更名并新增「推进一刻」「按时间模拟」，
        // 工具栏动作由 15 个增至 17 个（见 .omd/plans/sim-time-events.md §7.1）。
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
        // G7（缓冲库存 T4）：第 4 列「当前库存」必须存在且为数字——
        // 它把"中转站真的攒下货了"这件事变成用户能看到的东西。
        for (const QString& row : panelRows) {
            const QStringList cells = row.split(QStringLiteral(" | "));
            if (cells.size() < 4) {
                const QByteArray bad = row.toUtf8();
                std::fprintf(stderr,
                             "[ui-probe] 中转站面板应有 4 列（含「当前库存」），实际不足：%s\n",
                             bad.constData());
                return 1;
            }
            bool numberOk = false;
            cells[3].trimmed().toDouble(&numberOk);
            if (!numberOk) {
                const QByteArray bad = row.toUtf8();
                std::fprintf(stderr, "[ui-probe] 中转站「当前库存」列不是数字：%s\n",
                             bad.constData());
                return 1;
            }
        }
        const QByteArray transit = panel.toUtf8();
        std::printf("[ui-probe] 中转站面板（%s）:\n%s", "中转站|子网络|下属配送点|当前库存",
                    transit.constData());
        // 车辆面板：断言**界面上显示的载重**不超过载重上限，且**分解式自洽**。
        //
        // 两段历史：① 用户手工测试发现面板把"剩余待送总量 740kg"当成"当前载重"显示，
        // 而载重上限只有 200kg —— 数据层没错，是显示口径错了，所以必须查显示值本身；
        // ② 缓冲库存机制上线后，"只报订单货"的旧口径又变得**名不副实**（车上明明还有
        // 不属于任何订单的缓冲货）。故改为：**车上载重 = 订单货 + 缓冲货**，
        // 三者都印在面板上，这里逐项校验（读用户看得到的文本，不重算公式）。
        const QString vehiclePanel = window.vehiclePanelSummary();
        {
            const double cap = config.vehicles.empty() ? 0.0 : config.vehicles.front().capacityKg;
            // 「车上载重」是物理事实，必须 <= 载重上限
            const QRegularExpression totalRe(
                QStringLiteral("车上载重：([0-9.]+) kg"));
            const QRegularExpression breakdownRe(
                QStringLiteral("订单货 ([0-9.]+) kg ＋ 缓冲货 ([0-9.]+) kg"));
            const QRegularExpression tripRe(
                QStringLiteral("本趟出发装载：([0-9.]+) kg"));

            const QRegularExpressionMatch total = totalRe.match(vehiclePanel);
            const QRegularExpressionMatch parts = breakdownRe.match(vehiclePanel);
            const QRegularExpressionMatch trip = tripRe.match(vehiclePanel);
            if (!total.hasMatch() || !parts.hasMatch() || !trip.hasMatch()) {
                std::fprintf(stderr,
                             "[ui-probe] 车辆面板缺少「车上载重 / 订单货+缓冲货 / 本趟出发装载」"
                             "三处口径之一：\n%s\n",
                             vehiclePanel.toUtf8().constData());
                return 1;
            }
            const double shownTotal = total.captured(1).toDouble();
            const double orders = parts.captured(1).toDouble();
            const double buffer = parts.captured(2).toDouble();
            const double shownTrip = trip.captured(1).toDouble();

            if (std::fabs((orders + buffer) - shownTotal) > 0.05) {
                std::fprintf(stderr,
                             "[ui-probe] 车辆面板分解式不自洽：订单货 %.1f + 缓冲货 %.1f "
                             "!= 车上载重 %.1f\n",
                             orders, buffer, shownTotal);
                return 1;
            }
            if (shownTotal > cap + 1e-6 || shownTrip > cap + 1e-6) {
                std::fprintf(stderr,
                             "[ui-probe] 面板显示的载重超过上限 %.1fkg：车上载重 %.1f、"
                             "本趟出发装载 %.1f\n",
                             cap, shownTotal, shownTrip);
                return 1;
            }
            std::printf("[ui-probe] 车辆面板 车上载重 %.1fkg（= 订单货 %.1f + 缓冲货 %.1f）"
                        " <= 载重上限 %.1fkg；本趟出发装载 %.1fkg\n",
                        shownTotal, orders, buffer, cap, shownTrip);

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

        // 停靠明细必须每个停靠点一行（这些字段此前只有测试在读，界面上看不到）
        const QString stopPanel = window.stopPanelSummary();
        const std::size_t stopRows =
            static_cast<std::size_t>(stopPanel.count(QLatin1Char('\n')));
        if (stopRows == 0) {
            std::fprintf(stderr, "[ui-probe] 停靠明细面板为空\n");
            return 1;
        }
        std::printf("[ui-probe] 停靠明细 %zu 行"
                    "（趟|配送点|原始到达|等待(分)|送达|送后余货(kg)）\n", stopRows);
        // 打前几行实际数值：这是"数值对不对"的核对依据，光有行数不够
        {
            const QStringList lines = stopPanel.split(QLatin1Char('\n'), Qt::SkipEmptyParts);
            for (int i = 0; i < lines.size() && i < 3; ++i) {
                std::printf("[ui-probe]   %s\n", lines[i].toUtf8().constData());
            }
            // 表头守卫：停靠明细必须恰好 6 列、列名与数据字段一一对应。
            // 此前服务时间移除时只改了数据填充、漏改表头，于是表头 7 列而数据
            // 只填 6 列——"离开"下挂着剩余载重、"送后余货(kg)"整列空白。
            // 这类错位只有读**真实表头**才看得见：旧 printf 只是硬编码描述，不校验。
            {
                const QStringList headers = window.stopPanelHeaders();
                const QStringList expect{
                    QStringLiteral("趟"),   QStringLiteral("配送点"),
                    QStringLiteral("原始到达"), QStringLiteral("等待(分)"),
                    QStringLiteral("送达"), QStringLiteral("送后余货(kg)")};
                if (headers != expect) {
                    std::fprintf(stderr,
                                 "[ui-probe] 停靠明细表头不符：实际 [%s]，预期 [%s]\n",
                                 headers.join(QStringLiteral(" | ")).toUtf8().constData(),
                                 expect.join(QStringLiteral(" | ")).toUtf8().constData());
                    return 1;
                }
                // 每一行的单元格数必须等于表头列数，且不得有空白单元格：
                // "用户看得到的表"里不该出现空列或列名与内容对不上的格子。
                for (const QString& line : lines) {
                    const QStringList cells = line.split(QStringLiteral(" | "));
                    if (cells.size() != headers.size()) {
                        std::fprintf(stderr,
                                     "[ui-probe] 停靠明细行列数 %d != 表头列数 %d：%s\n",
                                     int(cells.size()), int(headers.size()),
                                     line.toUtf8().constData());
                        return 1;
                    }
                    for (const QString& c : cells) {
                        if (c.isEmpty()) {
                            std::fprintf(stderr,
                                         "[ui-probe] 停靠明细出现空白单元格：%s\n",
                                         line.toUtf8().constData());
                            return 1;
                        }
                    }
                }
                std::printf("[ui-probe] 停靠明细表头 6 列且每行 %d 格非空\n",
                            int(headers.size()));
            }
            // 每一行的第一列都必须是「第 N」：趟号映射一旦坏掉，
            // 停靠明细里"余载从 0 跳回几十"就会重新变成看不懂的数字。
            const QRegularExpression tripCell(QStringLiteral("^第 ([0-9]+) \\|"));
            QSet<int> tripNumbers;
            for (const QString& line : lines) {
                const QRegularExpressionMatch tm = tripCell.match(line);
                if (!tm.hasMatch()) {
                    std::fprintf(stderr,
                                 "[ui-probe] 停靠明细行缺少趟号: %s\n",
                                 line.toUtf8().constData());
                    return 1;
                }
                tripNumbers.insert(tm.captured(1).toInt());
            }
            // 多趟方案下，趟号列必须真的出现多个不同的趟号。
            // 用户实测过：切分剩余路线时把整条压成一趟，会让这里全部塌成「第 1」。
            if (window.tripCount() > 1 && tripNumbers.size() < 2) {
                std::fprintf(stderr,
                             "[ui-probe] 共 %d 趟，但停靠明细的趟号只有 %d 种（全塌成同一趟？）\n",
                             window.tripCount(), int(tripNumbers.size()));
                return 1;
            }
            // 趟号必须沿行**单调不减**且都 >= 1：重规划后趟号接着往下编，
            // 不允许出现"已完成的趟又冒出来"或"编号倒退"。
            {
                int prev = 0;
                bool monotone = true;
                for (const QString& line : lines) {
                    const int n = tripCell.match(line).captured(1).toInt();
                    if (n < prev || n < 1) {
                        monotone = false;
                        break;
                    }
                    prev = n;
                }
                if (!monotone) {
                    std::fprintf(stderr, "[ui-probe] 停靠明细的趟号不是单调不减（起点 %d）\n",
                                 window.completedTripOffset() + 1);
                    return 1;
                }
            }
            std::printf("[ui-probe] 停靠明细趟号种类 %d 种（共 %d 趟）\n",
                        int(tripNumbers.size()), window.tripCount());
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
