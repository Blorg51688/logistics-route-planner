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

} // namespace logistics
