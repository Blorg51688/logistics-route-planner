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
    baseTooltip_ = QString::fromStdString(node_.id + " " + node_.name + "（"
                                           + std::string(logistics::toString(node_.type)) + "）");
    setToolTip(baseTooltip_);
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
    const qreal pad = kRadius + 10 + 24;   // 留出多车环按 slot 外扩的余量
    return QRectF(-pad, -pad, 2 * pad, 2 * pad + 18);
}

void NodeItem::setVehicleHere(bool on) {
    if (vehicleHere_ == on) {
        return;
    }
    vehicleHere_ = on;
    update();
}

// 设置本节点上的车辆环（可能 0..n 台）。与旧 setVehicleHere 并存：旧接口只画单环，
// 新接口按 slot 错开、按角色取色 —— GUI 用新接口，老的离屏/自检代码不必改。
void NodeItem::setVehicleMarks(const std::vector<VehicleMark>& marks) {
    bool same = marks.size() == vehicleMarks_.size();
    for (std::size_t i = 0; same && i < marks.size(); ++i) {
        same = marks[i].vehicleId == vehicleMarks_[i].vehicleId
               && marks[i].fast == vehicleMarks_[i].fast
               && marks[i].slot == vehicleMarks_[i].slot;
    }
    if (same) {
        return;
    }
    vehicleMarks_ = marks;
    // #62：同站多环（如 T03 上 V01 快车 + V04 慢车）看上去像「慢车越簇」。
    // tooltip 逐环写车号与角色——用户悬停即知是两个合法角色叠在一起，而不是一台慢车跑错簇。
    QString tip = baseTooltip_;
    if (!marks.empty()) {
        tip += QStringLiteral("\n车辆环：");
        for (std::size_t i = 0; i < marks.size(); ++i) {
            tip += QStringLiteral(" %1（%2）")
                       .arg(QString::fromStdString(marks[i].vehicleId))
                       .arg(marks[i].fast ? QStringLiteral("快车") : QStringLiteral("慢车"));
        }
    }
    setToolTip(tip);
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

    // 车辆环（规格三 GUI §7）：慢车一种颜色、快车另一种；同节点多车按 slot 往外错开，
    // 否则环完全重合看起来像只有一台车。快车环更粗，一眼能分出「簇间搬运」那台。
    // 颜色刻意避开节点类型色（蓝/绿/橙）与路径高亮的红：慢车=青、快车=洋红。
    for (std::size_t m = 0; m < vehicleMarks_.size(); ++m) {
        const VehicleMark& mark = vehicleMarks_[m];
        QColor ringColor = mark.fast ? QColor(0xD6, 0x00, 0x9A) : QColor(0x00, 0x96, 0x88);
        QPen ring(ringColor);
        ring.setWidthF(mark.fast ? 4.0 : 2.6);
        painter->setPen(ring);
        painter->setBrush(Qt::NoBrush);
        const qreal rad = kRadius + 6.0 + mark.slot * 6.0;
        painter->drawEllipse(QPointF(0, 0), rad, rad);
    }

    // 兼容旧接口：单环、品红（离屏/自检里可能仍用 setVehicleHere）
    if (vehicleHere_ && vehicleMarks_.empty()) {
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
