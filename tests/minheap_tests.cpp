// 手写最小堆的行为测试。
// 只验证公共接口的可观察行为（压入/弹出顺序、size/empty），
// 不触碰 siftUp / siftDown 等内部实现，以便堆的实现方式可以自由重构。
#include <string>

#include "core/MinHeap.h"
#include "test_util.h"

using logistics::MinHeap;
using testutil::check;

namespace {

// 切片 1：压入一组已知值，必须按 key 非降序弹出
void testPopsInAscendingKeyOrder() {
    MinHeap heap;
    heap.push(5.0, 50);
    heap.push(1.0, 10);
    heap.push(3.0, 30);
    heap.push(2.0, 20);
    heap.push(4.0, 40);

    check(heap.size() == 5, "压入 5 项后 size == 5");
    check(!heap.empty(), "非空堆 empty() 为 false");

    // 期望值来自手写序列，而非代码重算
    const double expectedKeys[] = {1.0, 2.0, 3.0, 4.0, 5.0};
    const int expectedValues[] = {10, 20, 30, 40, 50};

    for (int i = 0; i < 5; ++i) {
        double key = 0.0;
        int value = -1;
        const bool ok = heap.popMin(key, value);
        const std::string at = "第 " + std::to_string(i + 1) + " 次弹出";

        check(ok, at + "成功");
        check(key == expectedKeys[i], at + " key == " + std::to_string(expectedKeys[i]));
        check(value == expectedValues[i], at + " value == " + std::to_string(expectedValues[i]));
    }

    check(heap.empty(), "全部弹出后 empty() 为 true");
    check(heap.size() == 0, "全部弹出后 size == 0");
}

// 切片 2：空堆 popMin 必须返回 false，而不是未定义行为
void testPopOnEmptyHeapReturnsFalse() {
    MinHeap heap;
    check(heap.empty(), "新建堆为空");
    check(heap.size() == 0, "新建堆 size == 0");

    double key = 123.0;
    int value = 456;
    check(!heap.popMin(key, value), "空堆 popMin 返回 false");
    check(key == 123.0, "空堆 popMin 不修改 key");
    check(value == 456, "空堆 popMin 不修改 value");

    // 弹空之后再压入，仍能正常工作（内部状态未被破坏）
    heap.push(9.0, 90);
    check(heap.size() == 1, "空堆弹出后再压入 size == 1");
    double k2 = 0.0;
    int v2 = 0;
    check(heap.popMin(k2, v2), "弹出剩下的一项");
    check(k2 == 9.0 && v2 == 90, "弹出值正确");
    check(heap.empty(), "再次为空");
}

} // namespace

int main() {
    testPopsInAscendingKeyOrder();
    testPopOnEmptyHeapReturnsFalse();
    return testutil::summarize("minheap_tests");
}
