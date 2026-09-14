#include "core/Node.h"

namespace logistics {

const char* toString(NodeType type) {
    switch (type) {
        case NodeType::Warehouse: return "warehouse";
        case NodeType::Delivery:  return "delivery";
        case NodeType::Transit:   return "transit";
    }
    return "unknown";
}

bool parseNodeType(const std::string& text, NodeType& out) {
    if (text == "warehouse") {
        out = NodeType::Warehouse;
        return true;
    }
    if (text == "delivery") {
        out = NodeType::Delivery;
        return true;
    }
    if (text == "transit") {
        out = NodeType::Transit;
        return true;
    }
    return false;
}

} // namespace logistics
