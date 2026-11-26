// 规格五验收基线 · **123 个常量 + 每车路线序列** —— 单一真值表（#57 / §7.1–§7.3，**第五轮捕获**）。
//
// 本文件是三策略黄金值的**唯一落值处**：integration_tests.cpp（L1 数值断言）与
// cli_plan_summary（L2 文本守卫）都只 include 本文件消费，**不允许各写一份**
// （规格一 §6.4 / §10 裁定 4；规格二 §7.1 两轮捕获；规格三 §7 第三轮；规格四 #54 第四轮；
//  规格五 #57 第五轮）。
//
// 捕获记录（规格五第 5 轮，#57）：
//   · 捕获命令：QT_QPA_PLATFORM=offscreen ./build/app --plan-summary distance|cost|time
//   · 捕获日期：2026-11-26
//   · 捕获分支：ticket/57-half-crosscluster-homing（基线 main 618084e）
//   · 数据：config/default.ini **订单表未改**（25 单 516kg 全簇内、全普通）——本轮改的是
//     **算法收尾**：跑完全部停靠后按最短路**空驶回主场待命**（慢车 -> 本簇中转站
//     stationOfCluster，快车 -> 起始仓库），见 docs/设计.md §5.13 与 §16 P43。
//   · 为什么 123 个常量全变：末段的终点从「起始仓库」改成「主场」——V03/V04 不再跑
//     180/220km 回 W02，而是 2.4/1.9km 回 T02/T03；距离/耗时/成本/在车积分/利用率
//     全部随之改变。**不是数值微调**，是末段语义变了。
//   · 每车路线序列相应改变：**末尾节点从 W01/W02 变成 T01/T02/T03**（V01 仍在 W01，
//     它在默认数据上是空车，归位点就是起始仓库 ⇒ 无移动）。
//   · 对拍：tools/oracle.py --plan <dump> <kind> 逐车复算 7 字段 + 车队 13 字段，
//           逐项一致（容差 0.001 / 0.1 / 整数；dump 新增 HOME 行与 XACT 行，oracle 均已接）。
//
// 字段顺序与精度严格照 §7.2：
//   **车队级 13 字段 × 3 策略 = 39**
//     1 总距离(km) 0.001 | 2 总耗时(min=完工耗时) 0.001 | 3 总成本(元) 0.001
//     4 总 penalty(min) 整数 | 5 停靠点数 S 整数 | 6 动作数 A 整数（A == 2 × 取送对数）
//     7 取货准时率(%) 0.1 | 8 送达准时率·总(%) 0.1 | 9 送达准时率·紧急(%) 0.1
//     10 送达准时率·普通(%) 0.1 | 11 平均在车货量(kg) 0.1 | 12 载重利用率(%) 0.1
//     13 峰值在车量(kg) 0.1
//   **每车 7 字段 × 4 车 × 3 策略 = 84**
//     1 该车距离(km) 0.001 | 2 该车耗时(min) 0.001 | 3 该车成本(元) 0.001
//     4 该车 penalty(min) 整数 | 5 该车载重利用率(%) 0.1（÷ **该车** capacity）
//     6 该车取货准时率(%) 0.1（分母 = 该车订单数） | 7 该车送达准时率(%) 0.1（同分母）
//   **每车路线序列**（不进 123 个数值，但属冻结集；L1/G17 逐字比对）
//
// 第 9 项（送达准时率·紧急）口径：默认数据集无紧急单，分母为 0，stdout 打印
// 「紧急 —」；表里用哨兵 **kNoDenominatorSentinel = -1** 表示「分母 0 记 —」。
// **每车两率在「空车」上同样是 -1**（快车 V01 在默认数据上没有取货/送达动作）。
//
// —— 历史（**已作废，只留追溯**）——
//   规格四 #54 第四轮（2026-11-26，基线 main 568be1b，跨城尺度）：
//     distance 893.300/696.375/2017.200 | cost 894.300/696.375/2016.960
//     time     900.400/664.750/2034.000
//     作废原因：本轮把末段终点从起始仓库改成主场，距离/耗时/成本/利用率全变。
//   规格三 #47 第三轮 147.900/429.000/156.880、规格二 139.900/429.000/172.970、
//   规格一 115.100/415.200/141.900，均已作废（各轮口径见 docs/测试用例表.md §6.3）。

#pragma once

#include <cstdio>

namespace plan_golden {

// 第 9 项「送达准时率·紧急」/ 每车两率分母为 0 时的哨兵（stdout 应打印「—」）。
inline constexpr double kNoDenominatorSentinel = -1.0;

// 容差（§7.2）：距离/耗时/成本 0.001；两率/在车/利用率/峰值 0.1；penalty/停靠/动作 整数。
inline constexpr double kTolMetric = 1e-3;
inline constexpr double kTolRate = 0.1;

// 冻结的车辆数（[vehicles] 行序：V01/V02/V03/V04）。数据换车数时本表必须重捕。
inline constexpr int kVehicleCount = 4;

// 每车 7 字段（§7.2）。利用率的分母是**该车** capacity；两率分母 = 该车订单数。
struct VehicleGolden {
    const char* id;            // 车辆 ID（[vehicles] 行序，与路线行同序）
    double distanceKm;         // 1 该车距离 (km)
    double timeMin;            // 2 该车耗时 (min)
    double costYuan;           // 3 该车成本 (元)
    int    penaltyMin;         // 4 该车 penalty (min)
    double utilizationPct;     // 5 该车载重利用率 (%)
    double pickupRatePct;      // 6 该车取货准时率 (%)   -1 = 分母 0（空车，「—」）
    double deliveryRatePct;    // 7 该车送达准时率 (%)   -1 = 分母 0（空车，「—」）
};

struct StrategyGolden {
    const char* name;          // 策略名（与 --plan-summary 参数一致）
    // ---- 车队级 13 字段（#29 §5.1，口径不变）----
    double distanceKm;         // 1  总距离 (km)
    double timeMin;            // 2  总耗时 (min，完工耗时)
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
    // ---- 每车 7 字段 × 4 车 ----
    VehicleGolden vehicles[kVehicleCount];
    // ---- 每车路线序列（§7.3：冻结集的一部分，不进 123 数值）----
    // 节点 ID 之间**单个空格**；与 --plan-summary 第 7–10 行「路线 <VID> 」之后的部分逐字一致。
    // 末尾节点 = 该车的**主场**（慢车 T01/T02/T03、快车起始仓库）。
    const char* routeNodes[kVehicleCount];
};

inline constexpr StrategyGolden kDistance = {
    "distance",
    // 车队 13
    495.000, 490.125, 1059.280, 0, 28, 50,
    100.0, 100.0, kNoDenominatorSentinel, 100.0, 26.9, 16.8, 160.0,
    // 每车 7 × 4（行序 = [vehicles]）
    {
        {"V01", 0.000, 0.000, 0.000, 0, 0.0, kNoDenominatorSentinel, kNoDenominatorSentinel},
        {"V02", 32.000, 103.500, 31.360, 0, 42.3, 100.0, 100.0},
        {"V03", 207.000, 256.625, 461.280, 0, 14.2, 100.0, 100.0},
        {"V04", 256.000, 400.125, 566.640, 0, 11.8, 100.0, 100.0},
    },
    {
        "W01",
        "W01 D01 D02 D03 D08 T01 D11 D10 D11 D12 T01 D04 D07 D08 D03 D02 D01 T01",
        "W02 T02 D14 D13 T02 D05 D06 T02 D09 D14 D15 T02 D16 T02",
        "W02 T03 D20 D19 D18 D19 T03 D17 T03 D21 D22 D25 T03 D23 T03 D20 T03 D24 T03",
    }};

inline constexpr StrategyGolden kCost = {
    "cost",
    496.000, 490.125, 1059.040, 0, 28, 50,
    100.0, 100.0, kNoDenominatorSentinel, 100.0, 27.5, 17.2, 160.0,
    {
        {"V01", 0.000, 0.000, 0.000, 0, 0.0, kNoDenominatorSentinel, kNoDenominatorSentinel},
        {"V02", 32.000, 103.500, 31.360, 0, 42.3, 100.0, 100.0},
        {"V03", 208.000, 259.500, 461.040, 0, 15.3, 100.0, 100.0},
        {"V04", 256.000, 400.125, 566.640, 0, 11.8, 100.0, 100.0},
    },
    {
        "W01",
        "W01 D01 D02 D03 D08 T01 D11 D10 D11 D12 T01 D04 D07 D08 D03 D02 D01 T01",
        "W02 T02 D14 D13 D14 T02 D05 D06 T02 D09 D14 D15 D14 T02 D16 T02",
        "W02 T03 D20 D19 D18 D19 T03 D17 T03 D21 D22 D25 T03 D23 T03 D20 T03 D24 T03",
    }};

inline constexpr StrategyGolden kTime = {
    "time",
    502.100, 458.500, 1076.080, 0, 28, 50,
    100.0, 100.0, kNoDenominatorSentinel, 100.0, 27.2, 17.0, 160.0,
    {
        {"V01", 0.000, 0.000, 0.000, 0, 0.0, kNoDenominatorSentinel, kNoDenominatorSentinel},
        {"V02", 37.800, 100.125, 43.680, 0, 43.1, 100.0, 100.0},
        {"V03", 207.200, 245.625, 462.640, 0, 13.6, 100.0, 100.0},
        {"V04", 257.100, 368.500, 569.760, 0, 12.2, 100.0, 100.0},
    },
    {
        "W01",
        "W01 D01 T01 D08 T01 D11 D10 T01 D07 T01 D12 T01 D04 T01 D03 D02 D01 T01",
        "W02 T02 D14 D13 T02 D09 T02 D05 D06 T02 D14 D15 T02 D16 T02",
        "W02 T03 D20 D19 D18 D19 T03 D21 D22 T03 D23 T03 D20 T03 D17 T03 D24 T03 D25 T03",
    }};

// 顺序固定为 distance / cost / time（与 integration_tests 的 plans[] 顺序一致）。
inline constexpr StrategyGolden kAll[3] = {kDistance, kCost, kTime};

} // namespace plan_golden
