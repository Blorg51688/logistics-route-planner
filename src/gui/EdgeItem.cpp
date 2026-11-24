#include "gui/EdgeItem.h"

#include <QFontMetricsF>
#include <QPainter>
#include <QPainterPath>
#include <QPainterPathStroker>
#include <QPolygonF>
#include <QStyleOptionGraphicsItem>

#include <cmath>

#include "gui/NodeItem.h"

namespace {

constexpr qreal kArrowSize = 11.0;
constexpr qreal kHitWidth = 10.0;      // 命中区域宽度；1px 的线否则几乎点不中
constexpr qreal kLabelOffset = 13.0;   // 标签相对线段的垂直偏移，避免压在线上
constexpr qreal kLabelPadX = 4.0;
constexpr qreal kLabelPadY = 1.0;

} // namespace

EdgeItem::EdgeItem(NodeItem* source, NodeItem* target, logistics::Edge edge,
                   logistics::WeightType weight)
    : source_(source), target_(target), edge_(edge), weight_(weight) {
    setZValue(0.0);
    setFlag(ItemIsSelectable);
    adjust();
}

double EdgeItem::weightValue() const {
    return logistics::pickWeight(edge_.distanceKm, edge_.timeMin, edge_.costYuan, weight_);
}

QString EdgeItem::labelText() const {
    const QString value = QString::number(weightValue(), 'g', 6);
    switch (weight_) {
        case logistics::WeightType::Distance: return value + "km";
        case logistics::WeightType::Time:     return value + "min";
        case logistics::WeightType::Cost:     return value + "元";
    }
    return value;
}

QRectF EdgeItem::labelRect() const {
    const QFontMetricsF metrics(QFont(QStringLiteral("Sans"), 7.0));
    const QRectF box = metrics.boundingRect(labelText());
    return QRectF(labelCenter_.x() - box.width() / 2.0 - kLabelPadX,
                  labelCenter_.y() - box.height() / 2.0 - kLabelPadY,
                  box.width() + 2 * kLabelPadX,
                  box.height() + 2 * kLabelPadY);
}

void EdgeItem::adjust() {
    // 几何要变了，先通知场景
    prepareGeometryChange();

    const QPointF s = source_->pos();
    const QPointF t = target_->pos();
    const QPointF dir = t - s;
    const double len = std::sqrt(dir.x() * dir.x() + dir.y() * dir.y());

    if (len < 1e-6) {
        sourcePoint_ = s;
        targetPoint_ = t;
        labelCenter_ = s;
        return;
    }

    const QPointF unit(dir.x() / len, dir.y() / len);
    // 两端各收进节点半径，箭头停在节点边界而不是圆心
    sourcePoint_ = s + unit * NodeItem::kRadius;
    targetPoint_ = t - unit * NodeItem::kRadius;

    const QPointF perp(-unit.y(), unit.x());
    labelCenter_ = (sourcePoint_ + targetPoint_) / 2.0 + perp * kLabelOffset;
}

void EdgeItem::setWeightType(logistics::WeightType weight) {
    if (weight_ == weight) {
        return;
    }
    weight_ = weight;
    prepareGeometryChange();   // 标签文字宽度会变，boundingRect 随之变化
    update();
}

void EdgeItem::setHighlighted(bool on) {
    if (highlighted_ == on) {
        return;
    }
    highlighted_ = on;
    setZValue(on ? 0.5 : 0.0);
    update();
}

void EdgeItem::setRouteStyle(bool owned, const QColor& color, bool selectedVehicleRoute) {
    // 归属与加粗**分别**设置：颜色归属行序最前那台车，加粗只看"是否属于选中车"。
    if (routeOwned_ == owned && routeColor_ == color && routeSelected_ == selectedVehicleRoute) {
        return;
    }
    routeOwned_ = owned;
    routeColor_ = color;
    routeSelected_ = selectedVehicleRoute;
    setZValue((owned ? 0.5 : 0.0) + (selectedVehicleRoute ? 0.1 : 0.0));
    update();
}

void EdgeItem::setLabelVisible(bool on) {
    if (labelVisible_ == on) {
        return;
    }
    labelVisible_ = on;
    update();
}

QRectF EdgeItem::boundingRect() const {
    const qreal extra = kArrowSize + kHitWidth;
    const QRectF line(sourcePoint_, QSizeF(targetPoint_.x() - sourcePoint_.x(),
                                           targetPoint_.y() - sourcePoint_.y()));
    return line.normalized().adjusted(-extra, -extra, extra, extra).united(labelRect());
}

QPainterPath EdgeItem::shape() const {
    QPainterPath line;
    line.moveTo(sourcePoint_);
    line.lineTo(targetPoint_);

    QPainterPathStroker stroker;
    stroker.setWidth(kHitWidth);
    return stroker.createStroke(line);
}

void EdgeItem::paint(QPainter* painter, const QStyleOptionGraphicsItem*, QWidget*) {
    painter->setRenderHint(QPainter::Antialiasing, true);

    const QPointF dir = targetPoint_ - sourcePoint_;
    const double len = std::sqrt(dir.x() * dir.x() + dir.y() * dir.y());
    if (len < 1e-6) {
        return;
    }
    const QPointF unit(dir.x() / len, dir.y() / len);
    const QPointF perp(-unit.y(), unit.x());

    // 车队三色（§7.5⑤）：选中车 不透明度 1.00 / 线宽 3；其余车 0.45 / 1.5；
    // 未被任何车使用的边保持中性灰。**权重标签内容仍是基础边权**（不按车缩放）。
    QColor color(130, 130, 130);
    qreal width = 1.2;
    qreal alpha = 1.0;
    if (routeOwned_) {
        color = routeColor_;
        width = routeSelected_ ? 3.0 : 1.5;
        alpha = routeSelected_ ? 1.00 : 0.45;
    } else if (highlighted_) {
        color = QColor(214, 45, 32);
        width = 3.0;
    }
    QColor stroke = color;
    stroke.setAlphaF(alpha);
    QPen pen(stroke);
    pen.setWidthF(width);
    painter->setPen(pen);
    painter->setBrush(stroke);

    painter->drawLine(sourcePoint_, targetPoint_);

    // 箭头：尖端在 targetPoint_，基底沿边回退 kArrowSize，再向两侧各展开半个箭头宽
    const QPointF base = targetPoint_ - unit * kArrowSize;
    const QPointF left = base + perp * (kArrowSize * 0.5);
    const QPointF right = base - perp * (kArrowSize * 0.5);
    QPolygonF head;
    head << targetPoint_ << left << right;
    painter->drawPolygon(head);

    if (!labelVisible_) {
        return;
    }

    // 权重标签：白底圆角框 + 文字，保证压在其他边之上时仍可读
    const QRectF box = labelRect();
    painter->setPen(QPen(QColor(200, 200, 200), 0.8));
    painter->setBrush(QColor(255, 255, 255, 230));
    painter->drawRoundedRect(box, 3.0, 3.0);

    painter->setPen(routeOwned_ ? QColor(40, 40, 40) : QColor(60, 60, 60));
    QFont font = painter->font();
    font.setPointSizeF(7.0);
    painter->setFont(font);
    painter->drawText(box, Qt::AlignCenter, labelText());
}