#pragma once

#include "core/Config.h"   // 只为 eventLevelWeight：等级 -> 权重只有那一处定义
#include "core/Random.h"

// 软件内时间的**模拟事件模型**（设计见 docs/records/specs/sim-time-events.md §3.2）。
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
//
// 「新客户」事件已**退役**（规格一 #24 §5）：该位次改名为 NormalOrder，
// 语义由「自动建节点 + 刷一张普通单」缩为「刷一张普通单」；**位次保持不变**
// ——动顺序会让固定种子下的随机串整条变（分布断言与等价性断言都会红）。
// 动态加节点改由**手动动作**「模拟新客户」承担（R10）。
enum class SimEventKind {
    Traffic,      // 路况变化（权重最高）
    UrgentOrder,  // 紧急订单
    NormalOrder,  // 普通订单（退役的「新客户」事件保位改名）
    RoadClosure   // 道路封闭（破坏性事件，权重最低）
};

// 抽签权重。默认值就是**等级表派生结果**（#23 固定表 1/3/9/27）：
//   路况 = high(27)、紧急 = low(3)、普通 = medium(9)、封路 = extra_low(1)，合计 40
//   ⇒ 67.5% / 7.5% / 22.5% / 2.5%。
// 数值本身不再手写一遍：等级 -> 权重**只在 core/Config.h::eventLevelWeight 一处派生**。
struct EventWeights {
    int traffic  = eventLevelWeight(EventLevel::High);
    int urgent   = eventLevelWeight(EventLevel::Low);
    int customer = eventLevelWeight(EventLevel::Medium);
    int closure  = eventLevelWeight(EventLevel::ExtraLow);
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

// 刷单的**端点抽签**（规格一 #24 R2）：候选 = 当前图全部配送点（n = count 个）。
//   · A 从 n 个里均匀抽一个；
//   · B 从**剩下的 n-1 个**里均匀抽一个（先抽 [0,n-1)，再跳过后移），A != B。
// 实现刻意**不用拒绝采样**：抽签次数固定 2 次，随机流消耗与抽中结果无关。
// 这是等价性纪律（一次推进一刻 == 逐个推进）的前提——见 D24 §7.7。
// count < 2 时返回 false（无法凑出 A != B），**且不消耗随机数**：该分支只由
// 「当前图上有几个配送点」决定，两条推进路径在同一时刻必然同时进入，抽签不错位。
bool pickDistinctPair(int count, Rng& rng, int& a, int& b);

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
