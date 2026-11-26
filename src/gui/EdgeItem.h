#pragma once

#include <QColor>
#include <QGraphicsItem>
#include <QRectF>
#include <QString>

#include "core/Edge.h"
#include "core/WeightType.h"
#include "gui/GraphScene.h"   // RoadClass（道路等级判据的唯一落点）

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

    // ---- 车队三色 + 选中加粗（规格二 §7.5⑤）----
    // 颜色 = 归属车的行序颜色（共享边取**行序最前**那台车）；加粗 = 该边是否属于
    // **选中车**（与颜色归属解耦）。两者分别设置，互不推导。
    void setRouteStyle(bool owned, const QColor& color, bool selectedVehicleRoute);
    bool routeOwned() const { return routeOwned_; }
    const QColor& routeColor() const { return routeColor_; }
    bool routeSelected() const { return routeSelected_; }

    // ---- 道路等级（规格四 §2 / #54）----
    // 三档道路在画布上是**底图**样式，与车队角色两色（慢车青 / 快车洋红）叠加：
    // 底图先画（粗细/明度不同），角色色再压在上面（更细，不盖住等级）。
    // 判据由 GraphScene::roadClassOf 按**端点类型与簇号**给出，图元不自己推 ID。
    void setRoadClass(RoadClass cls) { roadClass_ = cls; }
    RoadClass roadClass() const { return roadClass_; }
    bool trunk() const { return roadClass_ == RoadClass::Trunk; }

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
    RoadClass             roadClass_ = RoadClass::City;   // 底图等级（默认城内小路）
    bool                  routeOwned_ = false;      // 是否属于某台车的路线（有归属）
    bool                  routeSelected_ = false;   // 是否属于**选中车**的路线（加粗）
    QColor                routeColor_ = QColor(214, 45, 32);
    QPointF               sourcePoint_;
    QPointF               targetPoint_;
    QPointF               labelCenter_;
};