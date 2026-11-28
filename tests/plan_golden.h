// 规格八验收基线 · **123 个常量 + 每车路线序列** —— 单一真值表（#65 / §5.14a–§5.15，**第八轮捕获**）。
//
// 捕获记录（规格八第 8 轮，#65「主场待命语义修正」）：
//   · 捕获命令：./build/app --plan-summary distance|cost|time
//   · 捕获日期：2026-11-28
//   · 捕获分支：ticket/65-home-wait-l2-highway（基线 main ca3da39）
//   · 改了两处：V04 班次 14:00 -> 08:00（所有车 08:00 激活，T03 站上待命不计入 W）；
//     以及 relayStripFromVehicle 的拆链兜底（降级摘单时用清空 readyAt 的 Ctx 副本重放，
//     否则同一单既转运又直达）。W=60 / dwell≤60 两个常数不动。
//   · 第八轮数值为什么变：V04 的站内待命 6h37m 如实进耗时（490.750 / 487.875，原 130.750 / 127.875）
//     ⇒ 车队平均在车与利用率同降；cost 账本由 0 笔变 5 笔（L3 接得住 T03 的货），其余 7 张逐单点名降级。
//   · 对拍：tools/oracle.py --plan <dump> <kind> 逐车复算 7 字段 + 车队 13 字段，逐项一致。
//
// —— 第七轮（#63）记录（历史）——
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
// **每车两率在「空车」上同样是 -1**（例如 cost 策略的 V04 在参半数据上没有任何动作）。
//
// —— 历史（**已作废，只留追溯**）——
//   规格六 #60 第六轮（2026-11-27，基线 main a00335c，参半数据 + 走廊批量接力）：
//     distance 1149.700/542.000/4145.260 | cost 2482.300/1863.000/4828.400
//     time     1154.800/537.000/4157.820
//     作废原因：本轮把慢车出生点挪进本簇（断路前提），快车站间腿时刻随之整体前移。
//   规格五 #57 第五轮（2026-11-26，基线 main 618084e，25 单全簇内 + 空闲归位）：
//     distance 495.000/490.125/1059.280 | cost 496.000/490.125/1059.040
//     time     502.100/458.500/1076.080
//   规格四 #54 第四轮（2026-11-26，基线 main 568be1b，跨城尺度）：
//     distance 893.300/696.375/2017.200 | cost 894.300/696.375/2016.960
//     time     900.400/664.750/2034.000
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
    // 末尾节点 = 该车的**主场**（慢车 T01/T02/T03、快车起始仓库 W01）。
    // 规格七 #63 的读法：慢车那三行的**每个节点**都落在本簇（T02 行只有簇 2 的客户），
    // 一条站间高速（T01->T02 ...）都没有；只有快车 V01 那一行跨簇。
    const char* routeNodes[kVehicleCount];
};

inline constexpr StrategyGolden kDistance = {
    "distance",
    // 车队 13
    742.200, 577.000, 3176.940, 0, 40, 50,
    68.0, 100.0, kNoDenominatorSentinel, 100.0, 33.2, 13.0, 160.0,
    // 每车 7 × 4（行序 = [vehicles]）
    {
        {"V01", 645.500, 577.000, 3072.300, 0, 14.6, kNoDenominatorSentinel, kNoDenominatorSentinel},
        {"V02", 34.100, 117.500, 36.560, 0, 44.2, 100.0, 100.0},
        {"V03", 29.800, 274.375, 32.640, 0, 9.7, 85.7, 100.0},
        {"V04", 32.800, 490.750, 35.440, 0, 2.8, 22.2, 100.0},
    },
    {
        "W01 D01 T01 T02 T03 W01",
        "T01 D08 T01 D11 D10 D11 D12 T01 D08 T01 D08 T01 D01 D02 D03 D08 T01 D04 D07 T01",
        "T02 D14 T02 D05 D06 T02 D09 D14 D13 T02 D14 D15 T02 D16 T02",
        "T03 D18 D19 D20 T03 D24 T03 D17 T03 D21 D22 D25 T03 D23 T03",
    }};

inline constexpr StrategyGolden kCost = {
    "cost",
    2556.000, 2274.000, 5092.580, 6069, 39, 50,
    52.0, 72.0, kNoDenominatorSentinel, 72.0, 20.7, 6.3, 160.0,
    {
        {"V01", 2482.300, 2274.000, 5012.100, 6069, 6.1, 0.0, 0.0},
        {"V02", 31.500, 111.000, 33.360, 0, 42.0, 100.0, 100.0},
        {"V03", 20.200, 250.625, 20.880, 0, 6.6, 100.0, 100.0},
        {"V04", 22.000, 601.000, 26.240, 0, 1.0, 0.0, 100.0},
    },
    {
        "W01 D01 T01 D12 D13 D14 T02 D16 D17 T03 T02 D14 D13 D17 T03 D20 T03 D21 T03 T02 D05 D03 D08 T01 D12 D13 D14 D15 D14 T02 D16 D17 T03 D19 D18 D19 T03 T02 D05 D03 D02 D01 W01",
        "T01 D08 T01 D11 D10 D11 D12 T01 D08 T01 D01 D02 D03 D08 T01 D04 D07 T01",
        "T02 D14 T02 D05 D06 T02 D09 D14 D13 D14 T02",
        "T03 D19 T03 D22 T03 D23 T03 D24 T03 D25 T03",
    }};

inline constexpr StrategyGolden kTime = {
    "time",
    747.200, 577.000, 3188.780, 0, 39, 50,
    68.0, 100.0, kNoDenominatorSentinel, 100.0, 33.6, 13.1, 160.0,
    {
        {"V01", 645.500, 577.000, 3072.300, 0, 14.6, kNoDenominatorSentinel, kNoDenominatorSentinel},
        {"V02", 37.800, 99.875, 43.920, 0, 53.3, 100.0, 100.0},
        {"V03", 30.000, 274.375, 34.000, 0, 9.5, 85.7, 100.0},
        {"V04", 33.900, 487.875, 38.560, 0, 2.5, 22.2, 100.0},
    },
    {
        "W01 D01 T01 T02 T03 W01",
        "T01 D08 T01 D11 T01 D01 D02 D03 D08 T01 D08 T01 D07 T01 D04 T01 D10 T01 D12 T01",
        "T02 D14 T02 D05 D06 T02 D09 T02 D14 D13 T02 D14 D15 T02 D16 T02",
        "T03 D18 D19 D20 T03 D24 T03 D17 T03 D21 D22 T03 D23 T03 D25 T03",
    }};

// 顺序固定为 distance / cost / time（与 integration_tests 的 plans[] 顺序一致）。
inline constexpr StrategyGolden kAll[3] = {kDistance, kCost, kTime};

} // namespace plan_golden
