#pragma once

#include <cstddef>
#include <vector>

namespace logistics {

// 手写二叉最小堆，元素为 (key, value)。
//
// 课程要求核心数据结构不得使用类库/模板，因此上浮与下沉逻辑全部自实现，
// 且不引入 <algorithm>（min/max/swap 一律内联自写）。
// 底层动态数组使用 std::vector 承载，符合 设计.md §10.1 的 STL 边界表。
// 非模板：元素类型固定为 Dijkstra 所需的 (权重, 节点下标)。
class MinHeap {
public:
    void push(double key, int value);

    // 取出并移除最小项。空堆时返回 false，且不修改 key / value。
    bool popMin(double& key, int& value);

    bool empty() const { return entries_.empty(); }
    std::size_t size() const { return entries_.size(); }

private:
    struct Entry {
        double key;
        int    value;
    };

    void siftUp(std::size_t index);
    void siftDown(std::size_t index);

    static void swapEntries(Entry& a, Entry& b) {
        const Entry tmp = a;
        a = b;
        b = tmp;
    }

    std::vector<Entry> entries_;
};

} // namespace logistics
