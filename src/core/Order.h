#pragma once

#include <string>

namespace logistics {

// 订单来源（规格一 #24 R5）。**运行时字段**，不属于 [orders] 七列配置契约：
// 配置文件只描述静态订单（initial），其余来源由事件刻刷单 / 手动动作在运行时写入。
// ID 前缀随之区分：initial=O、event_normal=N、event_urgent=U、manual_customer=C；
// 规格二预留 manual_normal=M（手动「插入普通取送单」），四个既有来源语义不变。
//
// manual_urgent 是 R5 四类之外**唯一新增的一类**：工具栏「插入紧急订单」这个手动动作
// 也生成紧急单（前缀仍是 U），但它**不占 R6 的待处理额度**（额度只数事件来源的单）。
// 若不区分，手动插的紧急单会被算进"事件来源"，额度提前耗尽；若共用 Initial，
// 订单表就分不清它是配置里的静态单还是用户刚点的。仅此一处，别再加来源。
enum class OrderSource {
    Initial,         // 配置文件静态订单（O）
    EventNormal,     // 事件刻刷出的普通订单（N）
    EventUrgent,     // 事件刻刷出的紧急订单（U）
    ManualCustomer,  // 手动「模拟新客户」（C）
    ManualNormal,    // 预留：手动「插入普通取送单」（M，规格二）
    ManualUrgent,    // 手动「插入紧急订单」（U；不占自动刷单额度）
};

// 来源对应的 ID 前缀（单字符，供生成订单 ID 时拼接）。
// 前缀按**紧急与否**区分而不是按来源逐一分：手动紧急单与事件紧急单同为 U。
inline const char* orderIdPrefix(OrderSource source) {
    switch (source) {
        case OrderSource::Initial:        return "O";
        case OrderSource::EventNormal:    return "N";
        case OrderSource::EventUrgent:    return "U";
        case OrderSource::ManualCustomer: return "C";
        case OrderSource::ManualNormal:   return "M";
        case OrderSource::ManualUrgent:   return "U";
    }
    return "O";
}

// 一个配送订单。时间窗口统一用「相对当日 00:00 的分钟数」表示。
//
// 规格一（#21 / ADR 0002）起**全部订单统一 A→B 取送式**：
//   - pickupNodeId 是取货点 A，**必填**；A 无窗口、不计 penalty、停靠零耗时；
//   - nodeId 是送达点 B（窗口只属于 B），B 的窗口与 penalty 按订单自己的口径计算。
// 旧「普通单 = 仓库取货」语义已退役。
struct Order {
    std::string id;
    std::string nodeId;              // 送达点 B
    std::string pickupNodeId;        // 取货点 A（必填；A != B）
    double      demandKg = 0.0;      // 货物量
    int         windowStartMin = 0;  // 送达窗口起（只属于 B）
    int         windowEndMin = 0;    // 送达窗口止（只属于 B）
    bool        urgent = false;      // 是否紧急订单
    // 运行时来源（#24 R5）：不参与 [orders] 七列解析，加载出的静态订单恒为 Initial。
    OrderSource source = OrderSource::Initial;
    bool        picked = false;      // 是否已从 A 取到包裹（执行中：已取未送）
    bool        served = false;      // 是否已送达 B
};

} // namespace logistics
