#pragma once

#include <ostream>
#include <vector>
#include <memory>

template<typename T>
class PriorityQueue {
public:
    bool empty() const {
        return elements.empty();
    }

    std::unique_ptr<T> pop() {
        swap(0, elements.size() - 1);
        auto top = std::move(elements.back());
        elements.pop_back();
        siftDown(0);
        return top;
    }

    void push(std::unique_ptr<T> elem) {
        elem->index = elements.size();
        elements.emplace_back(std::move(elem));
        siftUp(elements.size() - 1);
    }

    void update(std::size_t index) {
        int parent = getParent(index);
        if (parent >= 0 && *elements[parent] < *elements[index])
            siftUp(index);
        else
            siftDown(index);
    }

    void remove(std::size_t index) {
        swap(index, elements.size() - 1);
        elements.pop_back();
        if (index < elements.size())
            update(index);
    }

    std::ostream &
    print(std::ostream &os, int i = 0, const std::string& tabs = "") const {
        if (i < elements.size()) {
            os << tabs << *elements[i] << std::endl;
            display(getLeftChild(i), tabs + '\t');
            display(getRightChild(i), tabs + '\t');
        }
        return os;
    }

private:
    std::vector<std::unique_ptr<T>> elements;

    int getParent(int index) const {
        return (index + 1) / 2 - 1;
    }

    std::size_t getLeftChild(std::size_t index) const {
        return 2 * (index + 1) - 1;
    }

    std::size_t getRightChild(std::size_t index) const {
        return 2 * (index + 1);
    }

    void siftDown(std::size_t index) {
        std::size_t left = getLeftChild(index);
        std::size_t right = getRightChild(index);
        std::size_t j = index;
        if (left < elements.size() && *elements[j] < *elements[left])
            j = left;
        if (right < elements.size() && *elements[j] < *elements[right])
            j = right;
        if (j != index) {
            swap(index, j);
            siftDown(j);
        }
    }

    void siftUp(std::size_t index) {
        int parent = getParent(index);
        if (parent >= 0 && *elements[parent] < *elements[index]) {
            swap(index, parent);
            siftUp(parent);
        }
    }

    inline void swap(std::size_t first, std::size_t second) {
        std::swap(elements[first], elements[second]);
        elements[first]->index = first;
        elements[second]->index = second;
    }
};

template<typename T>
std::ostream &operator<<(std::ostream &os, const PriorityQueue<T> &queue) {
    return queue.print(os);
}
