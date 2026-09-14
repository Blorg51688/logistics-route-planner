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
        "  --demo N                   渲染窗口前先自动触发 N 轮交互（验证交互后状态）\n"
        "  --dump-graph [list|matrix|both]  输出邻接表 / 邻接矩阵后退出（B3）\n"
        "  --ui-probe                 检查工具栏与侧栏是否完整构造后退出\n"
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
    if (!opt.dumpGraph.empty()) {
        if (opt.dumpGraph == "list" || opt.dumpGraph == "both") {
            std::printf("%s\n", config.graph.toAdjacencyListString().c_str());
        }
        if (opt.dumpGraph == "matrix" || opt.dumpGraph == "both") {
            std::printf("%s\n", config.graph.toAdjacencyMatrixString(opt.weight).c_str());
        }
        return 0;
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
                                     config.general.serviceTimeMin, planWeight);
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

    if (opt.uiProbe) {
        window.show();
        QCoreApplication::processEvents();
        const int actions = window.toolbarActionCount();
        const int docks = window.dockCount();
        std::printf("[ui-probe] 工具栏动作 %d 个，停靠面板 %d 个\n", actions, docks);
        if (actions < 9 || docks < 5) {
            std::fprintf(stderr,
                         "[ui-probe] UI 不完整：预期至少 9 个工具栏动作、5 个停靠面板\n");
            return 1;
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

    window.showInteractive(opt.width, opt.height);
    return app.exec();
}
