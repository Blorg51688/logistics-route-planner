#include "gui/NodeItem.h"

#include <QFont>
#include <QPainter>
#include <QPainterPath>
#include <QStyleOptionGraphicsItem>

#include "gui/EdgeItem.h"

namespace {

QColor colorFor(logistics::NodeType type) {
    switch (type) {
        case logistics::NodeType::Warehouse: return QColor(41, 98, 255);    // 蓝
        case logistics::NodeType::Delivery:  return QColor(46, 160, 67);    // 绿
        case logistics::NodeType::Transit:   return QColor(219, 154, 4);    // 橙
    }
    return QColor(120, 120, 120);
}

} // namespace

NodeItem::NodeItem(const logistics::Node& node) : node_(node) {
    setPos(node_.x, node_.y);
    setFlag(ItemIsMovable);
    setFlag(ItemSendsGeometryChanges);   // 位置变化时通知相邻边
    setZValue(1.0);
    setToolTip(QString::fromStdString(node_.id + " " + node_.name + "（"
                                      + std::string(logistics::toString(node_.type)) + "）"));
}

void NodeItem::addEdge(EdgeItem* edge) {
    edges_.push_back(edge);
}

QVariant NodeItem::itemChange(GraphicsItemChange change, const QVariant& value) {
    if (change == ItemPositionHasChanged) {
        for (EdgeItem* edge : edges_) {
            edge->adjust();
        }
    }
    return QGraphicsItem::itemChange(change, value);
}

QRectF NodeItem::boundingRect() const {
    // 下方留出名称文字的空间；四周留出车辆标记环的余量
    const qreal pad = kRadius + 10;
    return QRectF(-pad, -pad, 2 * pad, 2 * pad + 18);
}

void NodeItem::setVehicleHere(bool on) {
    if (vehicleHere_ == on) {
        return;
    }
    vehicleHere_ = on;
    update();
}

QPainterPath NodeItem::shape() const {
    QPainterPath path;
    path.addEllipse(QPointF(0, 0), kRadius, kRadius);
    return path;
}

void NodeItem::paint(QPainter* painter, const QStyleOptionGraphicsItem*, QWidget*) {
    painter->setRenderHint(QPainter::Antialiasing, true);

    painter->setBrush(colorFor(node_.type));
    painter->setPen(QPen(QColor(30, 30, 30), 1.5));
    painter->drawEllipse(QPointF(0, 0), kRadius, kRadius);

    painter->setPen(Qt::white);
    QFont idFont = painter->font();
    idFont.setPointSizeF(8.0);
    idFont.setBold(true);
    painter->setFont(idFont);
    painter->drawText(QRectF(-kRadius, -kRadius, 2 * kRadius, 2 * kRadius),
                      Qt::AlignCenter, QString::fromStdString(node_.id));

    // 车辆当前位置：品红环，与红色路径高亮、与节点类型色都能区分开
    if (vehicleHere_) {
        QPen ring(QColor(214, 0, 154));
        ring.setWidthF(3.5);
        painter->setPen(ring);
        painter->setBrush(Qt::NoBrush);
        painter->drawEllipse(QPointF(0, 0), kRadius + 5.0, kRadius + 5.0);
    }

    painter->setPen(QColor(70, 70, 70));
    QFont nameFont = painter->font();
    nameFont.setPointSizeF(7.0);
    nameFont.setBold(false);
    painter->setFont(nameFont);
    painter->drawText(QRectF(-kRadius - 22, kRadius - 2, 2 * kRadius + 44, 18),
                      Qt::AlignHCenter | Qt::AlignTop,
                      QString::fromStdString(node_.name));
}
