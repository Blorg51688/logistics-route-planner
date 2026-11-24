#pragma once

#include <cmath>
#include <map>
#include <string>
#include <vector>

#include "core/LogisticsGraph.h"
#include "core/RoutePlanner.h"  // Stop / StopDelivery（不变量 ⑦ 判据用）
#include "core/Order.h"
#include "core/Vehicle.h"

// 多个测试共用的图构造夹具，避免 Node / Edge 的构造样板在各测试文件里重复。

// ---------------------------------------------------------------------------
// 不变量 ⑦ 的**例外判据**（规格一 #18 §12 / 迁移清单 MC-B06 第 7 条）
//
// 「同阶段同节点至多一次停靠」的唯一例外是**强制重访**：只有把重访停靠上的
// 那一个动作并入该节点**首次**停靠时，违反 F2（先取后送）或 F3（前缀载重 /
// 先卸后装）才行。本函数对给定的停靠序列数出**非强制**的重复处数（应为 0）。
//
// 判据只看 F2/F3，与规格原文一致；时间/penalty 不属于"能否合并"的判据
// （合并总会给出某个到达时刻，penalty 由目标键负责，不是可行性）。
// ---------------------------------------------------------------------------
namespace fixtures {

inline const logistics::Order* findOrderById(const std::vector<logistics::Order>& orders,
                                             const std::string& id) {
    for (const logistics::Order& o : orders) {
        if (o.id == id) {
            return &o;
        }
    }
    return nullptr;
}

inline int nonForcedRevisitStops(const std::vector<logistics::Stop>& stops,
                                 const std::vector<logistics::Order>& orders, double capacityKg,
                                 double initialLoadKg = 0.0) {
    // 「并入后的动作落点」视图：把某个动作从停靠 j 搬到停靠 i 之后，每条动作的停靠下标。
    struct Act {
        std::string orderId;
        int kind = -1;   // 0 取货 / 1 送达 / 2 降载 / 3 取回
        double kg = 0.0;
    };
    std::vector<std::vector<Act> > base(stops.size());
    for (std::size_t k = 0; k < stops.size(); ++k) {
        for (const std::string& id : stops[k].pickups) {
            const logistics::Order* o = findOrderById(orders, id);
            base[k].push_back(Act{id, 0, (o == nullptr) ? 0.0 : o->demandKg});
        }
        for (const logistics::StopDelivery& d : stops[k].deliveries) {
            const logistics::Order* o = findOrderById(orders, d.orderId);
            base[k].push_back(Act{d.orderId, 1, (o == nullptr) ? 0.0 : o->demandKg});
        }
        for (const std::string& id : stops[k].offloads) {
            const logistics::Order* o = findOrderById(orders, id);
            base[k].push_back(Act{id, 2, (o == nullptr) ? 0.0 : o->demandKg});
        }
        for (const std::string& id : stops[k].reclaims) {
            const logistics::Order* o = findOrderById(orders, id);
            base[k].push_back(Act{id, 3, (o == nullptr) ? 0.0 : o->demandKg});
        }
    }

    // 紧急单的 FIFO 次序 =（到达事件刻, 插入序）；夹具里到达刻就是窗口起点。
    std::vector<const logistics::Order*> urgent;
    for (const logistics::Order& o : orders) {
        if (o.urgent) {
            urgent.push_back(&o);
        }
    }
    for (std::size_t i = 1; i < urgent.size(); ++i) {
        const logistics::Order* key = urgent[i];
        std::size_t j = i;
        while (j > 0 && urgent[j - 1]->windowStartMin > key->windowStartMin) {
            urgent[j] = urgent[j - 1];
            --j;
        }
        urgent[j] = key;
    }

    int bad = 0;
    // 例外判据（规格原文）：把重访动作并入该节点**首次**停靠。因此每个重访停靠
    // 只跟同阶段同节点的**第一个**停靠比一次 —— 与实现的 checkOrderingCore 同源
    // （它按「首次停靠」记 firstAt）。逐个 (i,j) 全比会把"并进第二处可行"也当成
    // 违规，那是比规格更严的另一条规则，不是这条不变量。
    for (std::size_t j = 0; j < stops.size(); ++j) {
        std::size_t i = stops.size();
        for (std::size_t k = 0; k < j; ++k) {
            if (stops[k].nodeId == stops[j].nodeId && stops[k].phase == stops[j].phase) {
                i = k;
                break;
            }
        }
        if (i < j) {
            {
            if (base[j].size() != 1) {
                ++bad;   // 重访停靠只允许承载一个动作
                continue;
            }
            // 构造「把 j 的动作搬到 i」后的视图（顺序无关，只关心下标与载重）。
            std::vector<std::vector<Act> > moved = base;
            const Act act = base[j].front();
            moved[j].clear();
            moved[i].push_back(act);

            // 每条动作的首次停靠下标
            std::map<std::string, int> pickAt;
            std::map<std::string, int> delivAt;
            std::map<std::string, int> reclaimAt;
            for (std::size_t k = 0; k < moved.size(); ++k) {
                for (const Act& a : moved[k]) {
                    if (a.kind == 0 && pickAt.find(a.orderId) == pickAt.end()) {
                        pickAt[a.orderId] = static_cast<int>(k);
                    }
                    if (a.kind == 1 && delivAt.find(a.orderId) == delivAt.end()) {
                        delivAt[a.orderId] = static_cast<int>(k);
                    }
                    if (a.kind == 3 && reclaimAt.find(a.orderId) == reclaimAt.end()) {
                        reclaimAt[a.orderId] = static_cast<int>(k);
                    }
                }
            }
            // F2：先取（或先取回）后送。
            bool ok = true;
            for (const std::map<std::string, int>::value_type& kv : delivAt) {
                const int first = (pickAt.count(kv.first) != 0)
                                      ? pickAt[kv.first]
                                      : ((reclaimAt.count(kv.first) != 0)
                                             ? reclaimAt[kv.first]
                                             : kv.second);
                if (first >= kv.second) {
                    ok = false;
                }
            }
            // F5：订单级 FIFO —— U_k 的送达不晚于 U_{k+1} 的取货。
            for (std::size_t k = 0; ok && k + 1 < urgent.size(); ++k) {
                const std::string& a = urgent[k]->id;
                const std::string& c = urgent[k + 1]->id;
                if (delivAt.count(a) == 0) {
                    continue;
                }
                const int da = delivAt[a];
                const int pc = (pickAt.count(c) != 0) ? pickAt[c] : delivAt[c];
                if (da > pc) {
                    ok = false;
                }
            }
            // F3：前缀载重 ∈ [0, C]，且不出现负卸载（先卸后装）。
            if (ok) {
                double load = initialLoadKg;
                for (std::size_t k = 0; k < moved.size(); ++k) {
                    double after = load;
                    for (const Act& a : moved[k]) {
                        if (a.kind == 1 || a.kind == 2) {
                            after -= a.kg;
                        }
                    }
                    if (after < -1e-9) {
                        ok = false;   // 先卸后装失败
                    }
                    for (const Act& a : moved[k]) {
                        if (a.kind == 0 || a.kind == 3) {
                            after += a.kg;
                        }
                    }
                    if (after > capacityKg + 1e-6 || after < -1e-9) {
                        ok = false;   // 前缀载重越界
                    }
                    load = after;
                }
            }
            if (ok) {
                ++bad;   // 并进去仍然可行 ⇒ 不是强制重访
            }
            }
        }
    }
    return bad;
}

} // namespace fixtures
namespace fixtures {

inline bool nearlyEqual(double a, double b, double tolerance = 1e-9) {
    return std::fabs(a - b) < tolerance;
}

inline logistics::Node makeNode(const std::string& id, logistics::NodeType type,
                               const std::string& name) {
    logistics::Node n;
    n.id = id;
    n.type = type;
    n.name = name;
    return n;
}

// 带子网络编号的节点（规格二 §4.2 的聚类判据：簇 = 取货点 A 的 subNetworkId）。
// 仓库 = 0（不参与聚类）；聚类夹具的配送点用 1 / 2 / 3。
inline logistics::Node makeSubNode(const std::string& id, logistics::NodeType type,
                                   const std::string& name, int subNetworkId) {
    logistics::Node n = makeNode(id, type, name);
    n.subNetworkId = subNetworkId;
    return n;
}

inline logistics::Edge makeEdge(const std::string& from, const std::string& to,
                               double km, double minutes, double yuan) {
    logistics::Edge e;
    e.fromId = from;
    e.toId = to;
    e.distanceKm = km;
    e.timeMin = minutes;
    e.costYuan = yuan;
    return e;
}

// 双向边：真实道路通常两个方向都通行，且能让"返回仓库"必然可达
inline void addTwoWay(logistics::LogisticsGraph& g, const std::string& a, const std::string& b,
                      double km, double minutes, double yuan) {
    g.addEdge(makeEdge(a, b, km, minutes, yuan));
    g.addEdge(makeEdge(b, a, km, minutes, yuan));
}

inline logistics::Vehicle makeVehicle(const std::string& startId, double capacityKg,
                                     int departMin) {
    logistics::Vehicle v;
    v.id = "V01";
    v.startNodeId = startId;
    v.capacityKg = capacityKg;
    v.departTimeMin = departMin;
    return v;
}

// 带因子的车（规格一 MC-B05 / #19 §2）：因子只改**缩放后的耗时与成本**，
// 距离、等待、penalty 都不乘。默认数据只有 1.00，因子代码全错也不会红——
// 所以必须有一条带 1.25 / 1.5 的夹具断言。
inline logistics::Vehicle makeVehicleWithFactors(const std::string& startId, double capacityKg,
                                                int departMin, double timeFactor,
                                                double costFactor) {
    logistics::Vehicle v = makeVehicle(startId, capacityKg, departMin);
    v.timeFactor = timeFactor;
    v.costFactor = costFactor;
    return v;
}

// 订单夹具：全部订单统一 A→B 取送（规格一 #23）。pickup 必填，调用方需显式给出取货点。
inline logistics::Order makeOrder(const std::string& id, const std::string& pickupNodeId,
                                 const std::string& nodeId, double demandKg,
                                 int windowStartMin, int windowEndMin,
                                 bool urgent = false) {
    logistics::Order o;
    o.id = id;
    o.pickupNodeId = pickupNodeId;
    o.nodeId = nodeId;
    o.demandKg = demandKg;
    o.windowStartMin = windowStartMin;
    o.windowEndMin = windowEndMin;
    o.urgent = urgent;
    return o;
}

} // namespace fixtures
