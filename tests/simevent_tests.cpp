// 模拟事件模型（事件抽签 + 事件时刻）的行为测试。
//
// 期望值全部来自**手算/解析式**，不复用实现中的任何计算：
//   - 频率断言用已知比例 10/3/1/1 与显式的容差，而不是把实现的概率再算一遍
//   - 时刻断言用字面量（495 -> 510 等），不是用同一个公式反推
// 这是 AGENTS.md §4.3 的口径：守卫要读"可观察的输出"，不要自证。
#include <string>

#include "core/Random.h"
#include "core/SimEvent.h"
#include "test_util.h"

using logistics::EventWeights;
using logistics::Rng;
using logistics::SimEventKind;
using logistics::nextEventTimeMin;
using logistics::pickEvent;
using logistics::simEventName;
using testutil::check;

namespace {

// 切片 1：默认权重 10:3:1:1 的抽签分布。
// 固定种子 + 10 万次大样本；四类频率必须落在 10/15、3/15、1/15、1/15 的 ±0.01 内。
// 10 万次下二项分布的标准差约 0.0015（路况）与 0.0008（两类低权重），
// 0.01 的容差 ≈ 6 个标准差以上，对本固定种子是稳定的（同种子必得同一序列）。
void testDefaultWeightsDistribution() {
    const int kDraws = 100000;
    Rng rng(20260926u);
    const EventWeights w;   // 默认 10 : 3 : 1 : 1

    int counts[4] = {0, 0, 0, 0};
    for (int i = 0; i < kDraws; ++i) {
        const SimEventKind kind = pickEvent(w, rng);
        switch (kind) {
            case SimEventKind::Traffic:     ++counts[0]; break;
            case SimEventKind::UrgentOrder: ++counts[1]; break;
            case SimEventKind::NewCustomer: ++counts[2]; break;
            case SimEventKind::RoadClosure: ++counts[3]; break;
        }
    }

    const double expected[4] = {10.0 / 15.0, 3.0 / 15.0, 1.0 / 15.0, 1.0 / 15.0};
    const char* names[4] = {"路况变化", "紧急订单", "新客户", "道路封闭"};
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
    check(std::string(simEventName(SimEventKind::NewCustomer)) == "新客户",
          "NewCustomer 中文名");
    check(std::string(simEventName(SimEventKind::RoadClosure)) == "道路封闭",
          "RoadClosure 中文名");
    check(simEventName(static_cast<SimEventKind>(97)) != nullptr,
          "未知取值返回非空指针（不得崩溃）");
}

} // namespace

int main() {
    testDefaultWeightsDistribution();
    testAllZeroWeightsFallBackToTraffic();
    testNegativeWeightsDoNotCrashAndAreNeverPicked();
    testAllNegativeWeightsFallBackToTraffic();
    testZeroWeightKindsNeverAppear();
    testNextEventTimeMinBoundaries();
    testSimEventNames();
    return testutil::summarize("simevent_tests");
}
