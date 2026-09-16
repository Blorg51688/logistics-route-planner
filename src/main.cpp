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
        "  --plan distance|cost       规划并高亮该策略的路线\n"
        "  --labels all|route         权重标签显示全部边还是仅高亮路线（默认 route）\n"
        "  --render PATH.png          离屏渲染图形场景成 PNG 后退出\n"
        "  --render-window PATH.png   离屏渲染完整窗口（工具栏+侧栏）成 PNG 后退出\n"
        "                              （注意 --width/--height 对交互窗口无效——它总是最大化）\n"
        "  --demo N                   渲染窗口前先自动触发 N 轮交互（验证交互后状态）\n"
        "  --dump-graph [list|matrix|both]  输出邻接表 / 邻接矩阵后退出（B3）\n"
        "  --ui-probe                 检查工具栏与侧栏是否完整构造后退出\n"
        "  --self-check-actions       自动验证插单不丢单 / 新客户会被配送后退出\n"
        "  --plan-summary distance|cost  打印该策略的规划汇总后退出（供人工测试核对）\n"
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
        const logistics::RoutePlan plan =
            logistics::planRoute(config.graph, config.vehicles.front(), config.orders,
                                 planWeight);
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
            const logistics::RoutePlan plan =
                logistics::planRoute(config.graph, config.vehicles.front(), config.orders,
                                     planWeight);
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
        std::printf("[ui-probe] 动作: %s\n", window.toolbarActionTexts().toUtf8().constData());
        std::printf("[ui-probe] 面板: %s\n", window.dockTitles().toUtf8().constData());
        if (actions < 9 || docks < 5) {
            std::fprintf(stderr,
                         "[ui-probe] UI 不完整：预期至少 9 个工具栏动作、5 个停靠面板\n");
            return 1;
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
        // 会把"峰值暂存/当前暂存为 0"的行也误判。
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
        const QByteArray transit = panel.toUtf8();
        std::printf("[ui-probe] 中转站面板（%s）:\n%s", "中转站|子网络|下属配送点|峰值暂存|当前暂存",
                    transit.constData());
        // 车辆面板：断言**界面上显示的"本趟装载"**不超过载重上限。
        // 用户手工测试发现过：面板把"剩余待送总量 740kg"当成"当前载重"显示，
        // 而载重上限只有 200kg —— 数据层没错，是显示口径错了，所以必须查显示值本身。
        const QString vehiclePanel = window.vehiclePanelSummary();
        {
            const double cap = config.vehicles.empty() ? 0.0 : config.vehicles.front().capacityKg;
            const char* labels[] = {"本趟装载", "当前载重"};
            for (const char* label : labels) {
                const QRegularExpression re(
                    QStringLiteral("%1：([0-9.]+) kg").arg(QString::fromUtf8(label)));
                const QRegularExpressionMatch match = re.match(vehiclePanel);
                if (!match.hasMatch()) {
                    std::fprintf(stderr, "[ui-probe] 车辆面板没有\"%s\"一行\n", label);
                    return 1;
                }
                const double shown = match.captured(1).toDouble();
                if (shown > cap + 1e-6) {
                    std::fprintf(stderr,
                                 "[ui-probe] 面板显示的%s %.1fkg 超过载重上限 %.1fkg\n",
                                 label, shown, cap);
                    return 1;
                }
                std::printf("[ui-probe] 车辆面板%s %.1fkg <= 载重上限 %.1fkg\n",
                            label, shown, cap);
            }
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
                    "（趟|配送点|原始到达|等待(分)|送达|送后余载(kg)）\n", stopRows);
        // 打前几行实际数值：这是"数值对不对"的核对依据，光有行数不够
        {
            const QStringList lines = stopPanel.split(QLatin1Char('\n'), Qt::SkipEmptyParts);
            for (int i = 0; i < lines.size() && i < 3; ++i) {
                std::printf("[ui-probe]   %s\n", lines[i].toUtf8().constData());
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
    }

    if (!opt.windowRenderPath.empty()) {
        window.renderToFile(QString::fromStdString(opt.windowRenderPath), opt.width, opt.height);
        std::printf("已渲染窗口 -> %s\n", opt.windowRenderPath.c_str());
        return 0;
    }

    window.showInteractive();
    return app.exec();
}
