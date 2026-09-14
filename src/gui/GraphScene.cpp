#include "gui/GraphScene.h"

#include <QPen>
#include <QRectF>

#include "core/RoutePlanner.h"
#include "gui/EdgeItem.h"
#include "gui/NodeItem.h"

GraphScene::GraphScene(QObject* parent) : QGraphicsScene(parent) {
    setBackgroundBrush(QColor(250, 250, 252));
}

void GraphScene::build(const logistics::LogisticsGraph& graph, logistics::WeightType weight) {
    clear();          // 由 QGraphicsScene 删除全部图元
    nodes_.clear();
    edges_.clear();
    weight_ = weight;

    for (const logistics::Node& node : graph.nodes()) {
        NodeItem* item = new NodeItem(node);
        addItem(item);
        nodes_.push_back(item);
    }

    // 按 ID 找图元，避免依赖 nodes() 的下标顺序
    for (const logistics::Edge& edge : graph.edges()) {
        NodeItem* source = nullptr;
        NodeItem* target = nullptr;
        for (NodeItem* item : nodes_) {
            if (item->nodeId() == edge.fromId) {
                source = item;
            }
            if (item->nodeId() == edge.toId) {
                target = item;
            }
        }
        if (source == nullptr || target == nullptr) {
            continue;
        }
        EdgeItem* item = new EdgeItem(source, target, edge, weight_);
        addItem(item);
        source->addEdge(item);
        if (target != source) {
            target->addEdge(item);
        }
        edges_.push_back(item);
    }

    setSceneRect(itemsBoundingRect().adjusted(-50, -50, 50, 50));
}

void GraphScene::setWeightType(logistics::WeightType weight) {
    weight_ = weight;
    for (EdgeItem* edge : edges_) {
        edge->setWeightType(weight);
    }
}

void GraphScene::highlightRoute(const std::vector<std::string>& routeNodes) {
    for (EdgeItem* edge : edges_) {
        edge->setHighlighted(
            logistics::isEdgeOnRoute(routeNodes, edge->fromId(), edge->toId()));
    }
    refreshLabelVisibility();
    update();
}

void GraphScene::clearHighlight() {
    highlightRoute({});
}

void GraphScene::setAllLabelsVisible(bool on) {
    allLabelsVisible_ = on;
    refreshLabelVisibility();
    update();
}

// 标签可见性规则：全部显示模式下一律显示；否则只在被高亮的边上显示
void GraphScene::refreshLabelVisibility() {
    for (EdgeItem* edge : edges_) {
        edge->setLabelVisible(allLabelsVisible_ || edge->highlighted());
    }
}
