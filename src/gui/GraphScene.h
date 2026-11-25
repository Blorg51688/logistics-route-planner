#pragma once

#include <QColor>
#include <QGraphicsScene>

#include <string>
#include <vector>

#include "core/LogisticsGraph.h"
#include "core/WeightType.h"

class EdgeItem;
class NodeItem;

// 配送网络的图形场景：按配置坐标摆放节点，按有向边绘制箭头与权重标签，
// 并支持路径高亮与权重维度切换。
class GraphScene : public QGraphicsScene {
public:
    explicit GraphScene(QObject* parent = nullptr);

    // 依据图重建全部图元
    void build(const logistics::LogisticsGraph& graph, logistics::WeightType weight);

    // 仅切换权重标签显示的维度，不重算任何业务数据
    void setWeightType(logistics::WeightType weight);

    // 高亮给定节点序列所经的边；传空序列表示清除高亮
    void highlightRoute(const std::vector<std::string>& routeNodes);
    void clearHighlight();

    // ---- 车队四色 + 选中加粗（规格二 §7.5⑤ / 规格三 §7 的第 4 色）----
    // routes[i] = 第 i 台车（= [vehicles] 行序）**剩余**路线的节点序列；空车传空序列。
    // 颜色按行序取（**超出 4 台循环**）：行序 0 红 / 1 蓝 / 2 绿 / 3 紫，四色都是定值。
    // 第 4 色是 #46 补的：原实现只有 3 个 case + default，第 4 台会与第 3 台同色。
    // **共享边**归行序最前那台车、不重复绘制；
    // 加粗判定 = 该边是否属于 selectedVehicle（与颜色归属解耦）。
    void setFleetRoutes(const std::vector<std::vector<std::string> >& routes,
                        int selectedVehicle);
    // 每车路线颜色（代码常量，按行序循环；报告/探针共用同一份）
    static QColor vehicleColor(int vehicleIndex);

    // 标记车辆当前所在节点（空串表示不标记任何节点）
    void setVehiclePosition(const std::string& nodeId);
    // 当前被标记为"车辆所在"的节点 ID —— 供自检核对画布与状态一致
    const std::string& vehiclePosition() const { return vehicleNodeId_; }

    // 是否让全部边都显示权重标签（默认只在被高亮的边上显示，避免密集网络互相遮挡）
    void setAllLabelsVisible(bool on);

    logistics::WeightType weightType() const { return weight_; }

private:
    std::vector<NodeItem*> nodes_;
    std::vector<EdgeItem*> edges_;
    logistics::WeightType weight_ = logistics::WeightType::Distance;
    bool allLabelsVisible_ = false;
    std::string vehicleNodeId_;
    std::vector<std::vector<std::string> > fleetRoutes_;
    int selectedVehicle_ = 0;

    void refreshLabelVisibility();
};