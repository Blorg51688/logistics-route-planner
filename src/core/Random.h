#pragma once

namespace logistics {

// 手写 xorshift32 伪随机数发生器。
//
// 核心层不引入 <random>（§10.1 禁止类库），且**确定性种子**让测试与演示可复现：
// 同一种子必得完全相同的序列，因此"随机"逻辑可以被逐值断言而不 flaky。
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
