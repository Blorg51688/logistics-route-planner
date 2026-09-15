#pragma once

#include <QGraphicsItem>
#include <QString>
#include <vector>

#include "core/Node.h"

class EdgeItem;

// 配送网络中的一个节点图元。
// 拖动时会通过 itemChange 通知相邻边重算几何（参考 Qt elasticnodes 示例）。
class NodeItem : public QGraphicsItem {
public:
    enum { Type = UserType + 1 };

    explicit NodeItem(const logistics::Node& node);

    int type() const override { return Type; }

    const std::string& nodeId() const { return node_.id; }

    void addEdge(EdgeItem* edge);

    // 标记"车辆当前所在节点"，用区别于节点类型色与路径高亮色的环表示
    void setVehicleHere(bool on);

    QRectF boundingRect() const override;
    QPainterPath shape() const override;
    void paint(QPainter* painter, const QStyleOptionGraphicsItem* option,
               QWidget* widget) override;

    QVariant itemChange(GraphicsItemChange change, const QVariant& value) override;

    // 节点按圆形绘制，边需要按此半径收进节点边界
    static constexpr qreal kRadius = 18.0;

private:
    logistics::Node node_;
    std::vector<EdgeItem*> edges_;
    bool vehicleHere_ = false;
};
