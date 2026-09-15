#pragma once

#include <QGraphicsItem>
#include <QRectF>
#include <QString>

#include "core/Edge.h"
#include "core/WeightType.h"

class NodeItem;

// 一条有向带权边图元：线段 + 箭头 + 权重标签。
//
// 箭头用「终点 + 基底两点」构造（tip / base / 垂直向量），
// 不采用 Qt diagramscene 示例的 atan2 式写法——那段几何在该示例里
// 是配合"线段 p1 为终点、p2 为起点"的相反约定使用的，直接照搬会把
// 箭头画到节点内部而被遮住（已实际渲染确认）。
//
// 「用 QPainterPathStroker 加宽细线以便点中」这一做法参考 Qt diagramscene（BSD-3）。
class EdgeItem : public QGraphicsItem {
public:
    enum { Type = UserType + 2 };

    EdgeItem(NodeItem* source, NodeItem* target, logistics::Edge edge,
             logistics::WeightType weight);

    int type() const override { return Type; }

    // 节点移动后重算几何
    void adjust();

    void setWeightType(logistics::WeightType weight);
    void setHighlighted(bool on);
    bool highlighted() const { return highlighted_; }

    // 权重标签默认只在被高亮的边上显示，避免密集网络里 72 个标签互相遮挡
    void setLabelVisible(bool on);

    const std::string& fromId() const { return edge_.fromId; }
    const std::string& toId() const { return edge_.toId; }

    QRectF boundingRect() const override;
    QPainterPath shape() const override;
    void paint(QPainter* painter, const QStyleOptionGraphicsItem* option,
               QWidget* widget) override;

private:
    double weightValue() const;
    QString labelText() const;
    QRectF labelRect() const;

    NodeItem*             source_;
    NodeItem*             target_;
    logistics::Edge       edge_;
    logistics::WeightType weight_;
    bool                  highlighted_ = false;
    bool                  labelVisible_ = false;
    QPointF               sourcePoint_;
    QPointF               targetPoint_;
    QPointF               labelCenter_;
};
