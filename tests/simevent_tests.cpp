// 模拟事件模型（事件抽签 + 事件时刻）的行为测试。
//
// 期望值全部来自**手算/解析式**，不复用实现中的任何计算：
//   - 频率断言用已知比例 27/9/3/1（等级表 1/3/9/27 的默认组合）与显式的容差，
//     而不是把实现的概率再算一遍
//   - 时刻断言用字面量（495 -> 510 等），不是用同一个公式反推
// 这是 AGENTS.md §4.3 的口径：守卫要读"可观察的输出"，不要自证。
#include <string>
#include <vector>

#include "core/Config.h"
#include "core/Random.h"
#include "core/SimEvent.h"
#include "test_util.h"

using logistics::EventLevel;
using logistics::EventWeights;
using logistics::Rng;
using logistics::SimEventKind;
using logistics::eventLevelWeight;
using logistics::nextEventTimeMin;
using logistics::parseEventLevel;
using logistics::pickDistinctPair;
using logistics::pickEvent;
using logistics::simEventName;
using testutil::check;

namespace {

// 切片 1：默认权重（等级表派生结果 27:3:9:1）的抽签分布 = 27/40 · 9/40 · 3/40 · 1/40。
// 固定种子 + 10 万次大样本；四类频率必须落在上述比例的 ±0.01 内。
// 10 万次下二项分布的标准差约 0.0010（路况）与 0.0004（封路），
// 0.01 的容差 ≈ 10 个标准差以上，对本固定种子是稳定的（同种子必得同一序列）。
void testDefaultWeightsDistribution() {
    const int kDraws = 100000;
    Rng rng(20260926u);
    const EventWeights w;   // 默认 = high/low/medium/extra_low 派生 -> 27 : 3 : 9 : 1

    int counts[4] = {0, 0, 0, 0};
    for (int i = 0; i < kDraws; ++i) {
        const SimEventKind kind = pickEvent(w, rng);
        switch (kind) {
            case SimEventKind::Traffic:     ++counts[0]; break;
            case SimEventKind::UrgentOrder: ++counts[1]; break;
            case SimEventKind::NormalOrder: ++counts[2]; break;
            case SimEventKind::RoadClosure: ++counts[3]; break;
        }
    }

    // 27/40 = 0.675、9/40 = 0.225、3/40 = 0.075、1/40 = 0.025
    const double expected[4] = {27.0 / 40.0, 3.0 / 40.0, 9.0 / 40.0, 1.0 / 40.0};
    const char* names[4] = {"路况变化", "紧急订单", "普通订单", "道路封闭"};
    for (int i = 0; i < 4; ++i) {
        const double freq = static_cast<double>(counts[i]) / kDraws;
        const double delta = freq - expected[i];
        check(delta < 0.01 && delta > -0.01,
              std::string("分布 ") + names[i] + " 频率 " + std::to_string(freq)
                  + " 应落在 " + std::to_string(expected[i]) + " ±0.01 内（实际次数 "
                  + std::to_string(counts[i]) + "）");
    }

    check(counts[0] + counts[1] + counts[2] + counts[3] == kDraws,
          "四类计数之和等于抽样次数（没有漏掉的分支）");
}

// 切片 1b：默认权重必须**派生自等级表**（1/3/9/27），且默认组合 Σ=40。
// 这里不重写第二张表：期望值直接引用 core/Config.h 的 eventLevelWeight。
void testDefaultWeightsDerivedFromLevels() {
    const EventWeights w;
    check(w.traffic == eventLevelWeight(EventLevel::High), "默认路况权重 == high(27)");
    check(w.urgent == eventLevelWeight(EventLevel::Low), "默认紧急权重 == low(3)");
    check(w.customer == eventLevelWeight(EventLevel::Medium), "默认普通权重 == medium(9)");
    check(w.closure == eventLevelWeight(EventLevel::ExtraLow), "默认封路权重 == extra_low(1)");
    check(w.traffic + w.urgent + w.customer + w.closure == 40, "默认组合权重和 Σ=40");

    // 等级表本体（#32 已落，本票只保证默认组合 == 该表的派生结果）
    check(eventLevelWeight(EventLevel::ExtraLow) == 1, "等级 extra_low -> 1");
    check(eventLevelWeight(EventLevel::Low) == 3, "等级 low -> 3");
    check(eventLevelWeight(EventLevel::Medium) == 9, "等级 medium -> 9");
    check(eventLevelWeight(EventLevel::High) == 27, "等级 high -> 27");

    // 解析层（#32）的非法等级：不承认、不回落默认
    EventLevel parsed = EventLevel::High;
    check(!parseEventLevel("hihg", parsed), "非法等级 hihg 必须被拒绝");
    check(!parseEventLevel("27", parsed), "数字不是等级");
    check(parseEventLevel("HIGH", parsed) && parsed == EventLevel::High, "等级大小写不敏感");

    // 一般组合的归一化（Q7）：封路改 medium ⇒ 27 : 3 : 9 : 9，Σ=48
    // 用与 pickEvent 相同的前缀和口径手算区间端点：
    //   路况 [1,27]、紧急 [28,30]、普通 [31,39]、封路 [40,48]
    EventWeights mixed;
    mixed.traffic = eventLevelWeight(EventLevel::High);
    mixed.urgent = eventLevelWeight(EventLevel::Low);
    mixed.customer = eventLevelWeight(EventLevel::Medium);
    mixed.closure = eventLevelWeight(EventLevel::Medium);
    check(mixed.traffic + mixed.urgent + mixed.customer + mixed.closure == 48,
          "封路改 medium 后 Σ=48（27:3:9:9）");
}

// 切片 2：全零权重保底返回 Traffic —— 不得死循环、不得除零。
void testAllZeroWeightsFallBackToTraffic() {
    EventWeights w;
    w.traffic = 0; w.urgent = 0; w.customer = 0; w.closure = 0;

    Rng rng(1u);
    bool allTraffic = true;
    for (int i = 0; i < 100; ++i) {
        if (pickEvent(w, rng) != SimEventKind::Traffic) {
            allTraffic = false;
        }
    }
    check(allTraffic, "全 0 权重时保底返回 Traffic（抽 100 次全部如此）");
}

// 切片 2（边界）：负数权重不得崩溃，且不得被抽中。
void testNegativeWeightsDoNotCrashAndAreNeverPicked() {
    EventWeights w;
    w.traffic = -5; w.urgent = -1; w.customer = 0; w.closure = 2;   // 只有 closure 为正

    Rng rng(7u);
    bool allClosure = true;
    for (int i = 0; i < 100; ++i) {
        if (pickEvent(w, rng) != SimEventKind::RoadClosure) {
            allClosure = false;
        }
    }
    check(allClosure, "负权重与 0 权重都不得被抽中，唯一正权重 RoadClosure 必被抽中");
}

// 切片 2（边界）：全负数权重 -> 保底 Traffic。
void testAllNegativeWeightsFallBackToTraffic() {
    EventWeights w;
    w.traffic = -3; w.urgent = -2; w.customer = -1; w.closure = -9;

    Rng rng(99u);
    bool allTraffic = true;
    for (int i = 0; i < 100; ++i) {
        if (pickEvent(w, rng) != SimEventKind::Traffic) {
            allTraffic = false;
        }
    }
    check(allTraffic, "全负权重时同样保底返回 Traffic");
}

// 切片 2：零权重种类在"有其它正权重"时出现次数必须为 0。
void testZeroWeightKindsNeverAppear() {
    EventWeights w;
    w.traffic = 5; w.urgent = 0; w.customer = 0; w.closure = 0;

    Rng rng(424242u);
    int traffic = 0;
    int others = 0;
    for (int i = 0; i < 5000; ++i) {
        if (pickEvent(w, rng) == SimEventKind::Traffic) {
            ++traffic;
        } else {
            ++others;
        }
    }
    check(others == 0, "权重为 0 的三类出现次数必须为 0，实际 " + std::to_string(others));
    check(traffic == 5000, "唯一正权重种类必须每次都被抽中");
}

// 切片 3：nextEventTimeMin 的边界（全部字面量手算）。
void testNextEventTimeMinBoundaries() {
    // 正好在刻上：必须前进一格，而不是返回自身
    check(nextEventTimeMin(495, 15) == 510, "now=495 在刻上 -> 510");
    check(nextEventTimeMin(480, 15) == 495, "08:00=480 在刻上 -> 08:15=495");
    check(nextEventTimeMin(510, 15) == 525, "now=510 在刻上 -> 525");

    // 不在刻上：取下一个刻
    check(nextEventTimeMin(496, 15) == 510, "now=496 不在刻上 -> 510");
    check(nextEventTimeMin(494, 15) == 495, "now=494 -> 495");
    check(nextEventTimeMin(1, 15) == 15, "now=1 -> 15");

    // 跨日
    check(nextEventTimeMin(1439, 15) == 1440, "now=1439 -> 1440（跨日）");
    check(nextEventTimeMin(1440, 15) == 1455, "now=1440 -> 1455");

    // 严格大于：任何输入都不得返回自身
    check(nextEventTimeMin(0, 15) > 0, "now=0 的返回必须严格大于 0");

    // interval <= 0 的防御分支
    check(nextEventTimeMin(495, 0) == 496, "interval=0 防御分支返回 now+1");
    check(nextEventTimeMin(495, -3) == 496, "interval<0 防御分支返回 now+1");
    check(nextEventTimeMin(1439, -1) == 1440, "interval<0 时仍严格前进");
}

// 切片 3：中文名映射（供日志/界面使用，不得返回空指针、不得张冠李戴）。
void testSimEventNames() {
    check(std::string(simEventName(SimEventKind::Traffic)) == "路况变化", "Traffic 中文名");
    check(std::string(simEventName(SimEventKind::UrgentOrder)) == "紧急订单",
          "UrgentOrder 中文名");
    check(std::string(simEventName(SimEventKind::NormalOrder)) == "普通订单",
          "NormalOrder 中文名（「新客户」事件保位改名为「普通订单」）");
    check(std::string(simEventName(SimEventKind::RoadClosure)) == "道路封闭",
          "RoadClosure 中文名");
    check(simEventName(static_cast<SimEventKind>(97)) != nullptr,
          "未知取值返回非空指针（不得崩溃）");
}

// 切片 4：刷单端点抽签 R2 —— A 均匀、B 在**剩下的 n-1 个**里一次抽出，A != B。
// 用有序对频率直接验证"条件均匀"：n=5 时每个有序对 (A,B) 的理论频率是 1/(5*4)=1/20。
// 旧实现（ib==ia 时改成 (ib+1)%n）会让 A 的"下一个点"概率翻倍，这个断言会红。
void testPickDistinctPairIsConditionallyUniform() {
    const int n = 5;
    const int kDraws = 60000;
    std::vector<int> counts(static_cast<std::size_t>(n * n), 0);
    Rng rng(20261124u);
    bool distinct = true;
    for (int i = 0; i < kDraws; ++i) {
        int a = -1;
        int b = -1;
        if (!pickDistinctPair(n, rng, a, b)) {
            distinct = false;
            continue;
        }
        if (a < 0 || a >= n || b < 0 || b >= n || a == b) {
            distinct = false;
            continue;
        }
        ++counts[static_cast<std::size_t>(a * n + b)];
    }
    check(distinct, "任何一次抽签都满足 A != B 且落在 [0,n)");

    const double expected = 1.0 / (n * (n - 1));   // 1/20
    bool uniform = true;
    std::string worst;
    for (int a = 0; a < n; ++a) {
        for (int b = 0; b < n; ++b) {
            if (a == b) {
                check(counts[static_cast<std::size_t>(a * n + b)] == 0,
                      "A == B 的组合一次也不出现（下标 " + std::to_string(a) + ","
                          + std::to_string(b) + "）");
                continue;
            }
            const double freq =
                static_cast<double>(counts[static_cast<std::size_t>(a * n + b)]) / kDraws;
            if (freq < expected - 0.004 || freq > expected + 0.004) {
                uniform = false;
                worst = "下标 " + std::to_string(a) + "->" + std::to_string(b) + " 频率 "
                        + std::to_string(freq);
            }
        }
    }
    check(uniform, "20 个有序对 (A,B) 的频率都落在 1/20 ± 0.004 内（条件均匀 "
                   + (worst.empty() ? std::string("通过") : worst) + "）");

    // A 的边缘分布同样均匀：每个点被抽作 A 的概率 = 1/n
    bool marginal = true;
    for (int a = 0; a < n; ++a) {
        int sum = 0;
        for (int b = 0; b < n; ++b) {
            sum += counts[static_cast<std::size_t>(a * n + b)];
        }
        const double freq = static_cast<double>(sum) / kDraws;
        if (freq < 1.0 / n - 0.006 || freq > 1.0 / n + 0.006) {
            marginal = false;
        }
    }
    check(marginal, "A 的边缘分布均匀（每点被抽作 A 的频率约 1/5）");
}

// 切片 4b：端点抽签的**随机数消耗固定为 2 次**——等价性纪律（D24 §7.7）要求
// 「每类事件随机消耗次数固定」，所以 A/B 必须一次抽出，不能用拒绝采样。
void testPickDistinctPairConsumesExactlyTwoDraws() {
    Rng picked(777u);
    Rng twin(777u);
    int a = 0;
    int b = 0;
    check(pickDistinctPair(6, picked, a, b), "6 个候选可以抽出 A != B");
    twin.nextU32();
    twin.nextU32();
    const unsigned int thirdFromTwin = twin.nextU32();
    check(picked.nextU32() == thirdFromTwin,
          "一次端点抽签恰消耗 2 次随机数（两条随机流的第 3 次取数相同）");

    Rng small(4242u);
    Rng smallTwin(4242u);
    int c = 0;
    int d = 0;
    check(!pickDistinctPair(1, small, c, d), "count=1 凑不出 A != B（返回 false）");
    check(!pickDistinctPair(0, small, c, d), "count=0 凑不出 A != B（返回 false）");
    check(small.nextU32() == smallTwin.nextU32(),
          "count<2 时不消耗随机数（失败分支不影响后续随机流）");
}

} // namespace

int main() {
    testDefaultWeightsDistribution();
    testDefaultWeightsDerivedFromLevels();
    testPickDistinctPairIsConditionallyUniform();
    testPickDistinctPairConsumesExactlyTwoDraws();
    testAllZeroWeightsFallBackToTraffic();
    testNegativeWeightsDoNotCrashAndAreNeverPicked();
    testAllNegativeWeightsFallBackToTraffic();
    testZeroWeightKindsNeverAppear();
    testNextEventTimeMinBoundaries();
    testSimEventNames();
    return testutil::summarize("simevent_tests");
}
