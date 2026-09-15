#pragma once

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

    // 标记车辆当前所在节点（空串表示不标记任何节点）
    void setVehiclePosition(const std::string& nodeId);

    // 是否让全部边都显示权重标签（默认只在被高亮的边上显示，避免密集网络互相遮挡）
    void setAllLabelsVisible(bool on);
    bool allLabelsVisible() const { return allLabelsVisible_; }

    logistics::WeightType weightType() const { return weight_; }
    std::size_t edgeItemCount() const { return edges_.size(); }
    std::size_t nodeItemCount() const { return nodes_.size(); }

private:
    std::vector<NodeItem*> nodes_;
    std::vector<EdgeItem*> edges_;
    logistics::WeightType weight_ = logistics::WeightType::Distance;
    bool allLabelsVisible_ = false;

    void refreshLabelVisibility();
};
