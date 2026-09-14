#include "core/Random.h"

namespace logistics {

Rng::Rng(unsigned int seed) : state_(seed == 0u ? 0x9E3779B9u : seed) {
    // 先用若干轮迭代把种子位扩散开。
    // 直接用小整数（如 1、2、3…）当状态时，xorshift 的前几个输出**高位几乎不变**：
    // 实测连续 30 个种子下 nextUnit() 的首个结果几乎相同，会让"随机位置"退化到
    // 包围盒的同一角落。扩散后再取用，相邻种子才真正互不相关。
    for (int i = 0; i < 8; ++i) {
        nextU32();
    }
}

unsigned int Rng::nextU32() {
    unsigned int x = state_;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    state_ = x;
    return x;
}

double Rng::nextUnit() {
    // 取高 24 位：精度足够，且避免低位模式的偏差
    return static_cast<double>(nextU32() >> 8) / 16777216.0;
}

int Rng::nextInt(int low, int high) {
    if (high <= low) {
        return low;
    }
    const unsigned int span = static_cast<unsigned int>(high - low + 1);
    return low + static_cast<int>(nextU32() % span);
}

double Rng::nextRange(double low, double high) {
    if (high <= low) {
        return low;
    }
    return low + (high - low) * nextUnit();
}

} // namespace logistics
