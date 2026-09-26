#include "core/SimEvent.h"

namespace logistics {

SimEventKind pickEvent(const EventWeights& weights, Rng& rng) {
    const SimEventKind kinds[4] = {SimEventKind::Traffic, SimEventKind::UrgentOrder,
                                   SimEventKind::NewCustomer, SimEventKind::RoadClosure};
    const int kindWeights[4] = {weights.traffic, weights.urgent,
                                weights.customer, weights.closure};

    // 只累加**正的**权重：权重 <= 0 的种类既不会撑大区间，也永远落不进自己的区间。
    // 用 long long 累加，避免极端权重下 int 溢出。
    long long total = 0;
    for (int i = 0; i < 4; ++i) {
        if (kindWeights[i] > 0) {
            total += kindWeights[i];
        }
    }
    // 保底：全零 / 全负权重。返回 Traffic 而不是死循环或除零。
    // 仍然**消耗一次随机数**：契约是"一次抽签 = 一次 rng 消耗"，否则两条本该看到
    // 同一串事件的路径（按站模拟 / 推进一刻）可能因保底分支而错位。
    if (total <= 0) {
        (void)rng.nextU32();
        return SimEventKind::Traffic;
    }

    // 掷一次骰子落在 [1, total]，再顺着前缀和找到所属区间。
    // 这是单次 O(n) 扫描；没有"先算总和、再赌一次、再校验"的回旋。
    const unsigned long long span = static_cast<unsigned long long>(total);
    const long long roll = static_cast<long long>(rng.nextU32() % span) + 1;

    long long acc = 0;
    for (int i = 0; i < 4; ++i) {
        if (kindWeights[i] <= 0) {
            continue;   // 零/负权重：跳过，绝不进入
        }
        acc += kindWeights[i];
        if (roll <= acc) {
            return kinds[i];
        }
    }
    // 上面 roll <= total == acc 必然命中，这里纯属防御，保持返回值有意义。
    return SimEventKind::Traffic;
}

int nextEventTimeMin(int nowMin, int intervalMin) {
    if (intervalMin <= 0) {
        // 防御分支：间隔非法时至少保证"严格前进"，调用方不会卡在同一个时刻。
        return nowMin + 1;
    }

    // 向下取整的除法：C++ 的 / 对负数向零截断，这里显式修正，
    // 使负的软件内时刻也能得到"严格大于 nowMin 的最小整数倍"。
    long long quotient = nowMin / intervalMin;
    if (nowMin < 0 && quotient * intervalMin != nowMin) {
        --quotient;
    }

    long long next = (quotient + 1) * static_cast<long long>(intervalMin);
    if (next <= nowMin) {
        // 防御：理论不可达（next 必为 (q+1)*interval > nowMin），但绝不允许返回自身。
        next += intervalMin;
    }
    return static_cast<int>(next);
}

const char* simEventName(SimEventKind kind) {
    switch (kind) {
        case SimEventKind::Traffic:     return "路况变化";
        case SimEventKind::UrgentOrder: return "紧急订单";
        case SimEventKind::NewCustomer: return "新客户";
        case SimEventKind::RoadClosure: return "道路封闭";
    }
    return "未知事件";
}

} // namespace logistics
