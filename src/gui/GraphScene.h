#pragma once

#include <QColor>
#include <QGraphicsScene>

#include <string>
#include <vector>

#include "core/LogisticsGraph.h"
#include "core/WeightType.h"

class EdgeItem;
class NodeItem;

// 道路等级判据（规格四 §2 / #54）。三档，只看**节点类型**、不看 ID：
//   · **Trunk**（跨城高速）：两端都是仓库/中转站 —— 用户说的「中转站间的连线，类似跨城高速」；
//   · **National**（跨城国道）：任一端是配送点、且两端**不同簇** —— 用户说的「通过用户点
//     将簇连接的路线就是国道等绕的小路」；
//   · **City**（城内小路）：其余（同一簇内的站↔客户、客户↔客户）。
// 判据与生成器 tools/generate_dataset.py::road_class 同一句口径。放在命名空间外是为了让
// 离屏渲染（main.cpp）与测试能直接复用，不必再造一份判据。
enum class RoadClass { Trunk, National, City };

// 簇号：仓库按「本国簇」算（W01↔T01 同城、W02↔T02 同城），与生成器 home_cluster_of 同口径。
// 为什么不能让仓库一律算 0 簇：那样 W01↔D01（同城 1.9km）会被判成跨城国道，
// 画布上把一条城内路画成国道，图就假了。
int homeClusterOf(const logistics::LogisticsGraph& graph, const std::string& warehouseId);
RoadClass roadClassOf(const logistics::LogisticsGraph& graph,
                      const logistics::Node& a, const logistics::Node& b);

// 配送网络的图形场景：按配置坐标摆放节点，按有向边绘制箭头与权重标签，
// 并支持路径高亮与权重维度切换。
class GraphScene : public QGraphicsScene {
public:
    explicit GraphScene(QObject* parent = nullptr);

    // 依据图重建全部图元
    void build(const logistics::LogisticsGraph& graph, logistics::WeightType weight);

    // 仅切换权重标签显示的维度，不重算任何业务数据
    void setWeightType(logistics::WeightType weight);

    // 高亮给定节点序列所经的边；传空序列表示清除高亮
    void highlightRoute(const std::vector<std::string>& routeNodes);
    void clearHighlight();

    // ---- 车队四色 + 选中加粗（规格二 §7.5⑤ / 规格三 §7 的第 4 色）----
    // routes[i] = 第 i 台车（= [vehicles] 行序）**剩余**路线的节点序列；空车传空序列。
    // 颜色按行序取（**超出 4 台循环**）：行序 0 红 / 1 蓝 / 2 绿 / 3 紫，四色都是定值。
    // 第 4 色是 #46 补的：原实现只有 3 个 case + default，第 4 台会与第 3 台同色。
    // **共享边**归行序最前那台车、不重复绘制；
    // 加粗判定 = 该边是否属于 selectedVehicle（与颜色归属解耦）。
    // fastFlags[i] = 第 i 台车是否快车（长度可与 routes 不同：缺省视为慢车）。
    // 颜色按**角色**取（慢车统一一色、快车另一色），不再按行序四色。
    void setFleetRoutes(const std::vector<std::vector<std::string> >& routes,
                        int selectedVehicle,
                        const std::vector<bool>& fastFlags = std::vector<bool>());
    // 每车路线颜色（代码常量，按行序循环；报告/探针共用同一份）。
    // 说明：画布上路线颜色已改为按**角色**（roleColor），vehicleColor 保留给
    // 「按行序区分」的离屏图/历史探针，不再用于 GUI 主画布。
    static QColor vehicleColor(int vehicleIndex);
    // 角色两色：慢车（false）青、快车（true）洋红 —— 与节点类型色、路径高亮红都可分辨。
    static QColor roleColor(bool fast);

    // 每台车在自己所在节点外的**环**（规格三 GUI §7）。
    // atNodeIds[i] = 第 i 台车当前所在节点（空串=不画）；同节点多车按行序 slot 错开。
    // #62：vehicleIds[i] = 第 i 台车车号，写进该节点 tooltip，区分同站叠加的多个环。
    void setVehicleMarks(const std::vector<std::string>& atNodeIds,
                         const std::vector<bool>& fastFlags,
                         const std::vector<std::string>& vehicleIds);
    // #62：读**真实图元**的 tooltip（供探针核对车号可见，不重算预期清单）。
    QString nodeVehicleTooltip(const std::string& nodeId) const;

    // 标记车辆当前所在节点（空串表示不标记任何节点）
    void setVehiclePosition(const std::string& nodeId);
    // 当前被标记为"车辆所在"的节点 ID —— 供自检核对画布与状态一致
    const std::string& vehiclePosition() const { return vehicleNodeId_; }

    // 是否让全部边都显示权重标签（默认只在被高亮的边上显示，避免密集网络互相遮挡）
    void setAllLabelsVisible(bool on);

    logistics::WeightType weightType() const { return weight_; }

private:
    std::vector<NodeItem*> nodes_;
    std::vector<EdgeItem*> edges_;
    logistics::WeightType weight_ = logistics::WeightType::Distance;
    bool allLabelsVisible_ = false;
    std::string vehicleNodeId_;
    std::vector<std::vector<std::string> > fleetRoutes_;
    std::vector<bool> fleetFastFlags_;   // 与 fleetRoutes_ 对应的角色标记（快车=true）
    int selectedVehicle_ = 0;

    void refreshLabelVisibility();
};