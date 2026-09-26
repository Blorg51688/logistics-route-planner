#pragma once

#include "core/Random.h"

// 软件内时间的**模拟事件模型**（设计见 .omd/plans/sim-time-events.md §3.2）。
//
// 与"真实秒级定时器"和"每推进一次就抽一次"两种旧触发方式的区别：
// 事件**只**由软件内时钟驱动——每过一个 `event_interval_min`（默认 15min）的整数倍时刻，
// 恰好抽一次签。因此本头文件只提供两件与 GUI 无关的纯设施：
//   1. 加权抽签  pickEvent
//   2. 事件时刻  nextEventTimeMin
// GUI 只负责在正确的时刻调用它们。
//
// 本文件属核心层：不得引入任何 Qt 头文件。

namespace logistics {

// 四类模拟事件。顺序即抽签的扫描顺序（也决定了并列时的确定性）。
enum class SimEventKind {
    Traffic,      // 路况变化（权重最高）
    UrgentOrder,  // 紧急订单
    NewCustomer,  // 新客户
    RoadClosure   // 道路封闭（破坏性事件，权重最低）
};

// 抽签权重。默认值即本项目采用的权重：10 : 3 : 1 : 1（合计 15）。
struct EventWeights {
    int traffic  = 10;
    int urgent   = 3;
    int customer = 1;
    int closure  = 1;
};

// 加权抽签：按 weights 的比例返回一个事件种类。
//
// 实现契约：
//   - 一次 O(n) 累积前缀比较（不预先算总和再赌一次）。
//   - **权重 <= 0 的种类永不被抽中**（出现次数恒为 0）。
//   - 全部权重 <= 0 时**保底返回 Traffic**：不得除零、不得死循环。
//   - 随机源用项目既有的 Rng（core/Random.h），固定种子必得可复现的序列。
//
// 注：任务书上写的是 `Random&`，但 core/Random.h 里的类名实际是 `Rng`，
// 故此处按实际类型签名（改名会波及本文件写入范围之外的头文件）。
SimEventKind pickEvent(const EventWeights& weights, Rng& rng);

// 下一个事件刻：**严格大于** nowMin 的最小的 intervalMin 整数倍。
//
// 绝对时刻对齐（而非"从上一次事件起算"），故 08:00=480 起、interval=15 时，
// 首个事件刻是 08:15=495，而不是 08:15 之外的任意相对值。
//   nextEventTimeMin(495, 15) == 510   // 正好在刻上，必须前进一格
//   nextEventTimeMin(496, 15) == 510   // 不在刻上，取下一个刻
//   nextEventTimeMin(1439, 15) == 1440 // 可跨日
// intervalMin <= 0 时返回 nowMin + 1（防御性：不得死循环、不得返回自身）。
int nextEventTimeMin(int nowMin, int intervalMin);

// 供日志与测试使用的中文名（未知取值返回"未知事件"，不返回空指针）。
const char* simEventName(SimEventKind kind);

} // namespace logistics
