#include "gui/GraphScene.h"

#include <QPen>
#include <QRectF>

#include <map>

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

// 车队四色（规格二 §7.5⑤ + 规格三 §7 的「第 4 色需定值」）：
// 按 [vehicles] 行序取色，**超出 4 台循环**。
//
// 为什么必须补 case 3：原实现是 case 0/1 + default 绿，3 车时恰好覆盖；车队扩到 4 台后
// 第 4 台（行序 3）会落进 default 与第 3 台**同色**，画布上两台车的路线分不出来 ——
// 「谁跑哪条边」这件用户可见的事就断了。四色都取**定值**（不是按比例算），
// 这样报告、探针与截图里的颜色恒定，不随车辆数漂。
QColor GraphScene::vehicleColor(int vehicleIndex) {
    switch (((vehicleIndex % 4) + 4) % 4) {
        case 0:  return QColor(0xC0, 0x39, 0x2B);   // 行序 0 红
        case 1:  return QColor(0x1F, 0x6F, 0xEB);   // 行序 1 蓝
        case 2:  return QColor(0x1A, 0x7F, 0x37);   // 行序 2 绿
        default: return QColor(0x7B, 0x2E, 0xA6);   // 行序 3 紫（与红/蓝/绿都易分辨）
    }
}

QColor GraphScene::roleColor(bool fast) {
    // 慢车青 / 快车洋红：都不是节点类型色（蓝/绿/橙），也不是路径高亮的红。
    return fast ? QColor(0xD6, 0x00, 0x9A) : QColor(0x00, 0x96, 0x88);
}

void GraphScene::setFleetRoutes(const std::vector<std::vector<std::string> >& routes,
                                int selectedVehicle,
                                const std::vector<bool>& fastFlags) {
    fleetRoutes_ = routes;
    selectedVehicle_ = selectedVehicle;
    fleetFastFlags_ = fastFlags;
    for (EdgeItem* edge : edges_) {
        // 共享边归**行序最前**那台车：按 routes 顺序扫，首个命中者即 owner。
        int owner = -1;
        for (std::size_t v = 0; v < routes.size(); ++v) {
            if (logistics::isEdgeOnRoute(routes[v], edge->fromId(), edge->toId())) {
                owner = static_cast<int>(v);
                break;
            }
        }
        const bool owned = owner >= 0;
        // 颜色按**角色**取（慢车一色、快车一色）；快慢由 fastFlags 给出（缺省当慢车）。
        bool ownerFast = false;
        if (owned && static_cast<std::size_t>(owner) < fleetFastFlags_.size()) {
            ownerFast = fleetFastFlags_[static_cast<std::size_t>(owner)];
        }
        const QColor color = owned ? roleColor(ownerFast) : QColor(130, 130, 130);
        // 加粗 = 该边属于**选中车**（与颜色归属解耦）。
        bool selected = false;
        if (owned && selectedVehicle >= 0
            && static_cast<std::size_t>(selectedVehicle) < routes.size()) {
            selected = logistics::isEdgeOnRoute(routes[static_cast<std::size_t>(selectedVehicle)],
                                                edge->fromId(), edge->toId());
        }
        edge->setRouteStyle(owned, color, selected);
    }
    refreshLabelVisibility();
    update();
}

void GraphScene::setVehiclePosition(const std::string& nodeId) {
    vehicleNodeId_ = nodeId;
    for (NodeItem* node : nodes_) {
        node->setVehicleHere(node->nodeId() == nodeId);
    }
}

// 每台车在自己所在节点外画环：先按节点分组（同一节点上的车按行序拿 slot），
// 再一次性把「谁在这里」交给该节点图元 —— 环的错开与配色都在 NodeItem 里完成。
void GraphScene::setVehicleMarks(const std::vector<std::string>& atNodeIds,
                                 const std::vector<bool>& fastFlags) {
    // 每个节点上已分配了几个环（slot 计数）
    std::map<std::string, int> used;
    std::map<std::string, std::vector<NodeItem::VehicleMark> > byNode;
    for (std::size_t v = 0; v < atNodeIds.size(); ++v) {
        const std::string& id = atNodeIds[v];
        if (id.empty()) {
            continue;
        }
        NodeItem::VehicleMark mark;
        mark.fast = (v < fastFlags.size()) ? fastFlags[v] : false;
        mark.slot = used[id];
        ++used[id];
        byNode[id].push_back(mark);
    }
    for (NodeItem* node : nodes_) {
        const std::map<std::string, std::vector<NodeItem::VehicleMark> >::const_iterator it =
            byNode.find(node->nodeId());
        node->setVehicleMarks(it == byNode.end() ? std::vector<NodeItem::VehicleMark>()
                                                : it->second);
    }
    update();
}

void GraphScene::setAllLabelsVisible(bool on) {
    allLabelsVisible_ = on;
    refreshLabelVisibility();
    update();
}

// 标签可见性规则：
//   1) 全部显示模式下一律显示；否则只在被高亮的边上显示
//   2) 双向边的两个方向若**同时可见**，它们画在同一段线上、权重又必然相同
//      （数据集里两个方向的权重一致），标签会完全重叠成"重复显示"。
//      此时只保留节点 ID 字典序较小的一端显示标签。
//      注意只在"两端都可见"时抑制：若路径只经过其中一个方向，那一条仍要显示，
//      否则会把标签整个丢掉。
void GraphScene::refreshLabelVisibility() {
    for (EdgeItem* edge : edges_) {
        // 权重标签：被**任意车**使用的边都显示（内容仍是基础边权，不按车缩放）。
        bool visible = allLabelsVisible_ || edge->highlighted() || edge->routeOwned();
        if (visible) {
            for (EdgeItem* other : edges_) {
                if (other == edge) {
                    continue;
                }
                const bool isTwin = other->fromId() == edge->toId()
                                    && other->toId() == edge->fromId();
                if (!isTwin) {
                    continue;
                }
                const bool twinVisible =
                    allLabelsVisible_ || other->highlighted() || other->routeOwned();
                if (twinVisible && edge->fromId() > other->fromId()) {
                    visible = false;   // 让字典序较小的一端保留标签
                }
                break;
            }
        }
        edge->setLabelVisible(visible);
    }
}