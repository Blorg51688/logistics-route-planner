#pragma once

#include <string>

namespace logistics {

// 配送网络中的节点类型
enum class NodeType {
    Warehouse,  // 仓库（起点）
    Delivery,   // 配送点（客户）
    Transit     // 中转站
};

// 节点类型的文本表示。INI 配置与图表示输出共用同一套写法。
const char* toString(NodeType type);

struct Node {
    std::string id;                // 全局唯一 ID
    NodeType    type = NodeType::Delivery;
    double      x = 0.0;           // 坐标，供 GUI 布局使用
    double      y = 0.0;
    std::string name;
    int         subNetworkId = 0;  // 子网络编号，为中转站嵌套子网络预留
};

} // namespace logistics
