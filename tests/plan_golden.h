// 规格七验收基线 · **123 个常量 + 每车路线序列** —— 单一真值表（#63 / §7.1–§7.3，**第七轮捕获**）。
//
// 本文件是三策略黄金值的**唯一落值处**：integration_tests.cpp（L1 数值断言）与
// cli_plan_summary（L2 文本守卫）都只 include 本文件消费，**不允许各写一份**
// （规格一 §6.4 / §10 裁定 4；规格二 §7.1 两轮捕获；规格三 §7 第三轮；规格四 #54 第四轮；
//  规格五 #57 第五轮；规格六 #60 第六轮；规格七 #63 第七轮）。
//
// 捕获记录（规格七第 7 轮，#63「慢车物理断路」）：
//   · 捕获命令：./build/app --plan-summary distance|cost|time
//   · 捕获日期：2026-11-28
//   · 捕获分支：ticket/63-slow-isolation（基线 main 2de97df）
//   · 数据：config/default.ini 的**车辆起始节点**改为「慢车出生在本簇中转站」
//     —— V02->T01、V03->T02、V04->T03（快车 V01 仍 W01）；V04 的班次 10:30 -> 14:00
//     （出生点挪到 T03 之后，10:30 出发等于在自家站空等 3h47 > 前瞻窗口 60min，
//      1->3 / 2->3 两条走廊会在构造期就废掉；班次贴近 L2 到站时刻是既有注释
//      「班次跟着接货时刻走」的直接延续，不是为数字调参）。
//   · 为什么 123 个常量全变：慢车的本职路线不再含「仓库 <-> 本簇」的通勤段，
//     快车的站间腿时刻随之整体前移 ⇒ 三策略的距离/耗时/成本/停靠/两率全变。
//     **这是数据（出生点）与断路（慢车不再能走跨簇边）两件事的直接结果。**
//   · 本轮**变好**的两个实测事实（如实冻结）：
//       ① penalty = 0（distance / time）：慢车出生在本簇站 ⇒ O11（D14->D06，10:00-10:30
//          的 30min 紧窗）不再被「站上集货交接 + 通勤」推到 11:12，本轮准时。
//       ② 取货准时率 68.0%（17/25，distance / time）：跨簇单的取货**由 L1 慢车在本簇完成**，
//          而该率的口径是「取到货那一刻，即使立刻直达也还赶得上窗口」—— 跨簇单的 A->B
//          直达是 600km 量级（参照量按**基础图**算，不看本车能不能真开那条路），
//          这个口径对「集货 + 干线 + 分拨」天然苛刻（口径是规格二 §3.4 定的，本轮不动它）。
//          它比第六轮的 72.0% 略低，是「慢车不再先跑跨簇通勤，取货更早但对照的直达参照
//          也更长」两件事的净值 —— 两率口径逐字未动。
//   · cost 策略仍是**已知不可行**（P44 / 本票未授权改）：完工 1863min > 12h，
//     penalty 4768min，9 张跨簇单在成本键下走 260km 国道、到站太晚 ⇒ L3 接不住，
//     全部诚实降级为快车直达；V04 因此是真空车（两率打印「—」）。
//   · 对拍：tools/oracle.py --plan <dump> <kind> 逐车复算 7 字段 + 车队 13 字段，
//           逐项一致（容差 0.001 / 0.1 / 整数；dump 的 HOME / XACT 行 oracle 均已接）。
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
    68.0, 100.0, kNoDenominatorSentinel, 100.0, 44.1, 15.4, 160.0,
    // 每车 7 × 4（行序 = [vehicles]）
    {
        {"V01", 645.500, 577.000, 3072.300, 0, 14.6, kNoDenominatorSentinel, kNoDenominatorSentinel},
        {"V02", 34.100, 117.500, 36.560, 0, 44.2, 100.0, 100.0},
        {"V03", 29.800, 274.375, 32.640, 0, 9.7, 85.7, 100.0},
        {"V04", 32.800, 130.750, 35.440, 0, 10.5, 22.2, 100.0},
    },
    {
        "W01 D01 T01 T02 T03 W01",
        "T01 D08 T01 D11 D10 D11 D12 T01 D08 T01 D08 T01 D01 D02 D03 D08 T01 D04 D07 T01",
        "T02 D14 T02 D05 D06 T02 D09 D14 D13 T02 D14 D15 T02 D16 T02",
        "T03 D18 D19 D20 T03 D24 T03 D17 T03 D21 D22 D25 T03 D23 T03",
    }};

inline constexpr StrategyGolden kCost = {
    "cost",
    2079.000, 1863.000, 3865.840, 4768, 29, 50,
    76.0, 68.0, kNoDenominatorSentinel, 68.0, 30.2, 8.4, 160.0,
    {
        {"V01", 2029.900, 1863.000, 3814.800, 4768, 7.8, 50.0, 33.3},
        {"V02", 28.900, 104.500, 30.160, 0, 40.1, 100.0, 100.0},
        {"V03", 20.200, 250.625, 20.880, 0, 6.2, 100.0, 100.0},
        {"V04", 0.000, 0.000, 0.000, 0, 0.0, kNoDenominatorSentinel, kNoDenominatorSentinel},
    },
    {
        "W01 D01 D02 D03 D05 T02 D14 D13 D17 T03 D20 T03 D21 D22 T03 D23 T03 D25 T03 T02 D05 D03 D08 T01 D12 D13 D14 D15 D14 T02 D16 D17 T03 D19 D18 D19 T03 D24 T03 T02 D05 D03 D02 D01 W01",
        "T01 D08 T01 D11 D10 D11 D12 T01 D01 D02 D03 D08 T01 D04 D07 T01",
        "T02 D14 T02 D05 D06 T02 D09 D14 D13 D14 T02",
        "T03",
    }};

inline constexpr StrategyGolden kTime = {
    "time",
    747.200, 577.000, 3188.780, 0, 39, 50,
    68.0, 100.0, kNoDenominatorSentinel, 100.0, 44.9, 15.6, 160.0,
    {
        {"V01", 645.500, 577.000, 3072.300, 0, 14.6, kNoDenominatorSentinel, kNoDenominatorSentinel},
        {"V02", 37.800, 99.875, 43.920, 0, 53.3, 100.0, 100.0},
        {"V03", 30.000, 274.375, 34.000, 0, 9.5, 85.7, 100.0},
        {"V04", 33.900, 127.875, 38.560, 0, 9.6, 22.2, 100.0},
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
