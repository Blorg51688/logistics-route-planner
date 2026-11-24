// 三策略评审基线 · 13 字段 × 3 行（规格一 §6.4）——**单一真值表**。
//
// 本文件是三策略黄金值的**唯一落值处**：integration_tests.cpp（L1 数值断言）与
// cli_plan_summary（L2 文本守卫）都只 include 本文件消费，**不允许各写一份**
// （规格一 §6.4 / §10 裁定 4）。
//
// 捕获记录（六步流程 §6.4，口径 = 规格一单车 V01 200kg·1.00·1.00 那一轮）：
//   · 捕获命令：./build/app --plan-summary distance|cost|time
//   · 捕获日期：2026-11-24
//   · 捕获提交：基线 9425543（#34 事件模型）+ 本票 #35 改动（本文件随该提交落库）
//   · 数据：config/default.ini（25 单 A→B、总重 516kg、全普通；[vehicles] V01 单行
//           200kg·1.00·1.00）；算法：pair-insertion 单条连续路线（#33）
//   · 对拍：tools/oracle.py --plan <dump> <kind> 独立复算，13 字段逐项一致（容差 0.001）
//
// 字段顺序与精度严格照 §6.4 表：
//   1 总距离(km) 0.001 | 2 总耗时(min) 0.001 | 3 总成本(元) 0.001
//   4 总 penalty(min) 整数 | 5 停靠点数 S 整数 | 6 动作数 A 整数（A == 2 × 取送对数）
//   7 取货准时率(%) 0.1 | 8 送达准时率·总(%) 0.1 | 9 送达准时率·紧急(%) 0.1
//   10 送达准时率·普通(%) 0.1 | 11 平均在车货量(kg) 0.1 | 12 载重利用率(%) 0.1
//   13 峰值在车量(kg) 0.1
//
// 第 9 项（送达准时率·紧急）口径：默认数据集无紧急单，分母为 0，stdout 打印「紧急 —」
// （不是 0%/100%）。表里用哨兵 **kNoDenominatorSentinel = -1** 表示「分母 0 记 —」；
// 消费方遇到 -1 时必须断言 urgentTotal == 0 且 stdout 打印「紧急 —」。
//
// —— 旧黄金值（**已作废，只留历史，不是本票目标值**，§9.2 / #29 §5.4 第 6 步）——
//   实测 2026-11-23，HEAD 4e27deb，./build/app --plan-summary distance|cost|time：
//     distance  178.200 km / 378.300 min / 244.200 元 / penalty 166 / 停靠 25 站 / 共 6 趟
//     cost      186.300 km / 461.000 min / 235.900 元 / penalty 207 / 停靠 25 站 / 共 6 趟
//     time      183.200 km / 372.100 min / 255.200 元 / penalty 201 / 停靠 25 站 / 共 6 趟
//   规格一实施后数据与算法全换、趟字段退役，上述三行**必然全部改变**；保留仅为追溯。

#pragma once

namespace plan_golden {

// 第 9 项「送达准时率·紧急」分母为 0 时的哨兵（stdout 应打印「紧急 —」）。
inline constexpr double kNoDenominatorSentinel = -1.0;

// 容差（规格一 §6.4）：距离/耗时/成本 0.001；两率/在车/利用率/峰值 0.1；penalty/停靠/动作 整数。
inline constexpr double kTolMetric = 1e-3;
inline constexpr double kTolRate = 0.1;

struct StrategyGolden {
    const char* name;          // 策略名（与 --plan-summary 参数一致）
    double distanceKm;         // 1  总距离 (km)
    double timeMin;            // 2  总耗时 (min)
    double costYuan;           // 3  总成本 (元)
    int    penaltyMin;         // 4  总 penalty (min)
    int    stopCount;          // 5  停靠点数 S
    int    actionCount;        // 6  动作数 A（== 2 × 取送对数）
    double pickupRatePct;      // 7  取货准时率 (%)
    double deliveryRatePct;    // 8  送达准时率·总 (%)
    double urgentRatePct;      // 9  送达准时率·紧急 (%)  -1 = 分母 0（「—」）
    double normalRatePct;      // 10 送达准时率·普通 (%)
    double avgLoadKg;          // 11 平均在车货量 (kg)
    double utilizationPct;     // 12 载重利用率 (%)
    double peakLoadKg;         // 13 峰值在车量 (kg)
};

inline constexpr StrategyGolden kDistance = {
    "distance", 115.100, 415.200, 141.900, 0, 30, 50,
    100.0, 100.0, kNoDenominatorSentinel, 100.0, 52.5, 26.3, 200.0};

inline constexpr StrategyGolden kCost = {
    "cost", 116.800, 431.600, 140.400, 0, 30, 50,
    100.0, 100.0, kNoDenominatorSentinel, 100.0, 50.5, 25.3, 200.0};

inline constexpr StrategyGolden kTime = {
    "time", 130.200, 378.200, 165.700, 0, 28, 50,
    100.0, 100.0, kNoDenominatorSentinel, 100.0, 56.8, 28.4, 164.0};

// 顺序固定为 distance / cost / time（与 integration_tests 的 plans[] 顺序一致）。
inline constexpr StrategyGolden kAll[3] = {kDistance, kCost, kTime};

} // namespace plan_golden
