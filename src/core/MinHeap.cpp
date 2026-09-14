#include "core/MinHeap.h"

namespace logistics {

void MinHeap::push(double key, int value) {
    Entry entry;
    entry.key = key;
    entry.value = value;
    entries_.push_back(entry);
    siftUp(entries_.size() - 1);
}

bool MinHeap::popMin(double& key, int& value) {
    if (entries_.empty()) {
        return false;
    }
    key = entries_.front().key;
    value = entries_.front().value;

    entries_.front() = entries_.back();
    entries_.pop_back();
    if (!entries_.empty()) {
        siftDown(0);
    }
    return true;
}

void MinHeap::siftUp(std::size_t index) {
    while (index > 0) {
        const std::size_t parent = (index - 1) / 2;
        if (entries_[parent].key <= entries_[index].key) {
            break;
        }
        swapEntries(entries_[parent], entries_[index]);
        index = parent;
    }
}

void MinHeap::siftDown(std::size_t index) {
    const std::size_t count = entries_.size();
    while (true) {
        const std::size_t left = 2 * index + 1;
        const std::size_t right = left + 1;
        std::size_t smallest = index;

        if (left < count && entries_[left].key < entries_[smallest].key) {
            smallest = left;
        }
        if (right < count && entries_[right].key < entries_[smallest].key) {
            smallest = right;
        }
        if (smallest == index) {
            break;
        }
        swapEntries(entries_[smallest], entries_[index]);
        index = smallest;
    }
}

} // namespace logistics
