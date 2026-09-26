#pragma once

namespace logistics {

// 手写 xorshift32 伪随机数发生器。
//
// 不用 <random> **不是因为它被禁用**（§10.1 只禁止把算法主体交给 STL，容器与辅助设施可用），
// 而是因为**确定性种子**让测试与演示可复现：同一种子必得完全相同的序列，
// 因此"随机"逻辑可以被逐值断言而不 flaky。
class Rng {
public:
    explicit Rng(unsigned int seed);

    unsigned int nextU32();
    double nextUnit();                                  // [0, 1)
    int nextInt(int lowInclusive, int highInclusive);   // 闭区间
    double nextRange(double low, double high);          // [low, high)

private:
    unsigned int state_;
};

} // namespace logistics
