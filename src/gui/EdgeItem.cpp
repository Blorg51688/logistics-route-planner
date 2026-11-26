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
constexpr qreal kTrunkWidth = 12.0;    // 干线的命中区域宽度（视觉线宽 4.2 的命中余量）
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
    // 高速画得更粗（6.0），命中区必须跟着变宽，否则点在粗线上却没反应。
    stroker.setWidth(trunk() ? kTrunkWidth : kHitWidth);
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

    // ---- 第一层：道路等级（规格四 §2 / #54）。这是**底图**，先画。----
    // 三档的视觉语言（要一眼可分，且都不与角色两色混淆）：
    //   · **跨城高速**：墨蓝粗实线 + 浅色中线（读作"有分隔带的高速公路"）；
    //   · **跨城国道**：土黄中粗**虚线**（读作"绕的普通公路"）；
    //   · **城内小路**：浅灰细实线（退到背景里）。
    // 为什么高速用双线、国道用虚线：只靠颜色区分对色觉障碍不友好，也必须能在
    // 黑白打印/灰度截图里分辨。线型 + 粗细 + 明度三重差异，任一维度失效都还认得出来。
    // 为什么城内小路不用虚线：它数量最多（74 条有向边），全虚线会让画面噪；
    // 而国道只有 8 条，虚线正好强调"这是少数几条绕行路"。
    const QColor kTrunkInk(0x0F, 0x27, 0x40);      // 高速墨蓝
    const QColor kNationalInk(0x8A, 0x62, 0x14);   // 国道土黄
    const QColor kCityInk(0xA9, 0xAF, 0xB8);       // 城内小路灰
    if (roadClass_ == RoadClass::Trunk) {
        QPen base(kTrunkInk);
        base.setWidthF(6.0);
        base.setCapStyle(Qt::FlatCap);
        painter->setPen(base);
        painter->drawLine(sourcePoint_, targetPoint_);
        QPen core(QColor(0xE8, 0xEF, 0xF8));       // 浅色中线 = 分隔带
        core.setWidthF(1.4);
        painter->setPen(core);
        painter->drawLine(sourcePoint_, targetPoint_);
    } else if (roadClass_ == RoadClass::National) {
        QPen base(kNationalInk);
        base.setWidthF(2.8);
        base.setStyle(Qt::DashLine);
        base.setDashPattern(QVector<qreal>() << 5.0 << 3.5);
        base.setCapStyle(Qt::FlatCap);
        painter->setPen(base);
        painter->drawLine(sourcePoint_, targetPoint_);
    } else {
        QPen base(kCityInk);
        base.setWidthF(1.3);
        painter->setPen(base);
        painter->drawLine(sourcePoint_, targetPoint_);
    }

    // ---- 第二层：车队角色两色（§7.5⑤）——**叠加**在底图之上，不盖住等级 ----
    // 选中车 不透明度 1.00 / 线宽 3；其余车 0.45 / 1.5；路径高亮红同档。
    // 为什么压细而不是更粗：底图表达"这是高速还是小路"（结构），角色色表达"谁在跑"
    // （此刻状态）。把角色色做得比底图更粗，用户就再也看不出被跑的那条是高速还是小路了。
    // **权重标签内容仍是基础边权**（不按车缩放）。
    QColor color(130, 130, 130);
    qreal width = 0.0;
    qreal alpha = 1.0;
    bool overlay = true;
    if (routeOwned_) {
        color = routeColor_;
        width = routeSelected_ ? 2.6 : 1.8;
        alpha = routeSelected_ ? 1.00 : 0.85;
    } else if (highlighted_) {
        color = QColor(214, 45, 32);
        width = 2.6;
    } else {
        // 没有任何车跑过：就只剩底图（底图上面已经画过）。这里的颜色只给箭头用 ——
        // 箭头永远跟着"这条路"的等级色走，不跟着"谁在跑"走。
        color = (roadClass_ == RoadClass::Trunk)    ? kTrunkInk
                : (roadClass_ == RoadClass::National) ? kNationalInk
                                                      : kCityInk;
        overlay = false;
    }
    if (overlay) {
        QColor stroke = color;
        stroke.setAlphaF(alpha);
        QPen pen(stroke);
        pen.setWidthF(width);
        pen.setCapStyle(Qt::RoundCap);
        painter->setPen(pen);
        painter->setBrush(stroke);
        painter->drawLine(sourcePoint_, targetPoint_);
    }

    // 箭头：尖端在 targetPoint_，基底沿边回退 kArrowSize，再向两侧各展开半个箭头宽。
    // 箭头也用底图色 —— 方向是**路网属性**，不属于某台车；角色色只盖在线身上。
    const qreal headSize = (roadClass_ == RoadClass::Trunk) ? 14.0 : kArrowSize;
    const QPointF base = targetPoint_ - unit * headSize;
    const QPointF left = base + perp * (headSize * 0.5);
    const QPointF right = base - perp * (headSize * 0.5);
    QPolygonF head;
    head << targetPoint_ << left << right;
    QColor headColor = (roadClass_ == RoadClass::Trunk)    ? kTrunkInk
                           : (roadClass_ == RoadClass::National) ? kNationalInk
                                                                 : kCityInk;
    if (overlay) {
        headColor = color;
        headColor.setAlphaF(alpha);
    }
    painter->setPen(Qt::NoPen);
    painter->setBrush(headColor);
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