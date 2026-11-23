#pragma once

#include <string>
#include <vector>

#include "core/LogisticsGraph.h"
#include "core/Order.h"
#include "core/Vehicle.h"

namespace logistics {

// 事件等级（规格一 #23 §3.3）：配置里只写等级，权重由代码内固定表派生，
// 配置文件不再出现任何权重数字。
enum class EventLevel {
    ExtraLow = 0,  // 极低 -> 1
    Low = 1,       // 低   -> 3
    Medium = 2,    // 中   -> 9
    High = 3       // 高   -> 27
};

// 等级 → 固定权重表的**唯一**定义（1 / 3 / 9 / 27）。
// 不要在别处再写一张表：配置解析、事件抽签、测试都从这里取。
inline int eventLevelWeight(EventLevel level) {
    switch (level) {
        case EventLevel::ExtraLow: return 1;
        case EventLevel::Low:      return 3;
        case EventLevel::Medium:   return 9;
        case EventLevel::High:     return 27;
    }
    return 1;
}

// 配置文本 → 等级。trim 后不区分大小写；无法识别返回 false（不猜、不回落）。
// 合法取值：extra_low / low / medium / high。
inline bool parseEventLevel(const std::string& text, EventLevel& out) {
    // trim 首尾空白（与 io/StringUtil.h 的 trim 同口径；core 不反向依赖 io），
    // 再统一转小写比较——只做大小写不敏感，不吞掉值内部的内容。
    std::size_t begin = 0;
    std::size_t end = text.size();
    while (begin < end && (text[begin] == ' ' || text[begin] == '\t' || text[begin] == '\r'
                           || text[begin] == '\n')) {
        ++begin;
    }
    while (end > begin && (text[end - 1] == ' ' || text[end - 1] == '\t' || text[end - 1] == '\r'
                            || text[end - 1] == '\n')) {
        --end;
    }
    std::string value = text.substr(begin, end - begin);
    for (char& c : value) {
        if (c >= 'A' && c <= 'Z') {
            c = static_cast<char>(c - 'A' + 'a');
        }
    }
    if (value == "extra_low") {
        out = EventLevel::ExtraLow;
        return true;
    }
    if (value == "low") {
        out = EventLevel::Low;
        return true;
    }
    if (value == "medium") {
        out = EventLevel::Medium;
        return true;
    }
    if (value == "high") {
        out = EventLevel::High;
        return true;
    }
    return false;
}

// 与配置取值同名的小写文本，供日志与报错使用。
inline const char* eventLevelName(EventLevel level) {
    switch (level) {
        case EventLevel::ExtraLow: return "extra_low";
        case EventLevel::Low:      return "low";
        case EventLevel::Medium:   return "medium";
        case EventLevel::High:     return "high";
    }
    return "extra_low";
}

// [general] 节的全局参数
struct GeneralConfig {
    double trafficChangeRatio = 0.1;        // 每次变化的边占比
    double trafficTimeIncreaseMin = 0.2;    // 耗时增加比例下限
    double trafficTimeIncreaseMax = 0.5;    // 耗时增加比例上限
    // 软件内时间的事件模型（设计 D24）：每 eventIntervalMin 分钟一个**事件刻**，每刻恰抽 1 个事件。
    // 注：原先的 traffic_change_interval_sec（真实秒级）与 urgent_order_interval_sec（每 tick 节流）
    // 已被本模型取代并**移除**——事件只由软件内时钟驱动，与真实时间和推进粒度都无关。
    int eventIntervalMin = 15;              // 事件抽取间隔（软件内分钟）
    // 四类事件等级（#23 §3.3 / #24 §2）。缺键回落：traffic=high、customer=medium、
    // urgent=low、closure=extra_low（默认组合 27:9:3:1 -> 67.5/22.5/7.5/2.5%）。
    // 权重由 eventLevelWeight 派生，配置里不写数字。
    EventLevel eventLevelTraffic = EventLevel::High;
    EventLevel eventLevelCustomer = EventLevel::Medium;
    EventLevel eventLevelUrgent = EventLevel::Low;
    EventLevel eventLevelClosure = EventLevel::ExtraLow;
};

// 一次配置加载的完整结果
struct Config {
    LogisticsGraph      graph;
    std::vector<Vehicle> vehicles;
    std::vector<Order>   orders;
    GeneralConfig        general;
};

} // namespace logistics
