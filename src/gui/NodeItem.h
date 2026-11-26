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
    const logistics::Node& node() const { return node_; }   // 道路分级要读节点类型（规格四 §2）

    void addEdge(EdgeItem* edge);

    // 标记"车辆当前所在节点"，用区别于节点类型色与路径高亮色的环表示
    void setVehicleHere(bool on);

    // ---- 车辆标记（规格三 GUI §7：每车一个环；慢车/快车两色）----
    // 一个节点上可能同时停多台车（例如都在 W01 发车）：slot 把环错开，
    // 否则两个环完全重合、看起来只有一台车。slot = 该节点上第几台（0 起，按行序）。
    struct VehicleMark {
        bool fast = false;   // true = 快车环（另一种颜色），false = 慢车环
        int  slot = 0;       // 同一节点上的错开序号
    };
    void setVehicleMarks(const std::vector<VehicleMark>& marks);

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
    bool vehicleHere_ = false;   // 兼容旧接口：单环、慢车色
    std::vector<VehicleMark> vehicleMarks_;
};
