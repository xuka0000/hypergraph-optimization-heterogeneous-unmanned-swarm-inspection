/**
 * (c) 2021, Henrik Reinstädtler
 */
#pragma once
#include <algorithm>
#include <functional>
#include <numeric>
#include <vector>
namespace HeiHGM::BMatching {
namespace ds {

template <typename I, typename V, class Comp = std::less<V>>
class IndexedPriorityQueue {
public:
  std::vector<int> _placesinHeap;
  std::vector<I> heap;
  std::vector<V> valueForI;
  Comp cmp;
  void BubbleDown(int index) {
    int length = heap.size();
    int leftChildIndex = 2 * index + 1;
    int rightChildIndex = 2 * index + 2;

    if (leftChildIndex >= length)
      return; // index is a leaf

    int minIndex = index;

    if (cmp(valueForI[heap[index]], valueForI[heap[leftChildIndex]])) {
      minIndex = leftChildIndex;
    }

    if ((rightChildIndex < length) &&
        (cmp(valueForI[heap[minIndex]], valueForI[heap[rightChildIndex]]))) {
      minIndex = rightChildIndex;
    }

    if (minIndex != index) {
      // need to swap
      _placesinHeap[heap[minIndex]] = index;
      _placesinHeap[heap[index]] = minIndex;
      std::swap(heap[index], heap[minIndex]);
      BubbleDown(minIndex);
    }
  }
  void BubbleUp(int index) {
    if (index == 0)
      return;

    int parentIndex = (index - 1) / 2;

    if (cmp(valueForI[heap[parentIndex]], valueForI[heap[index]])) {
      _placesinHeap[heap[parentIndex]] = index;
      _placesinHeap[heap[index]] = parentIndex;

      std::swap(heap[parentIndex], heap[index]);
      BubbleUp(parentIndex);
    }
  }
  void heapify() {
    int length = heap.size();
    for (int i = length - 1; i >= 0; --i) {
      BubbleDown(i);
    }
  }

public:
  int firstI = 0;
  IndexedPriorityQueue(const size_t count, const V init_val,
                       Comp _cmp = std::less<V>{})
      : _placesinHeap(count), valueForI(count, init_val), cmp(_cmp) {
    clear();
  }
  void clear() {
    heap.clear();
    fill(_placesinHeap.begin(), _placesinHeap.end(), -1);
  }
  void print() {
    std::cerr << "Heap " << firstI;
    for (auto a : heap)
      std::cerr << " " << a << "(" << valueForI[a] << ")";
    std::cerr << std::endl;
    for (auto a : valueForI)
      std::cerr << " " << a << "("
                << ")";
    std::cerr << std::endl;
  }
  void resetValues(const std::vector<V> &init_val) {
    valueForI = init_val;
    heapify();
  }
  void resetValue(const V val) {
    std::fill(valueForI.begin(), valueForI.end(), val);
    heapify();
  }
  void push(const I idx) {

    if (_placesinHeap[idx] != -1) {
      std::cerr << _placesinHeap[idx] << std::endl;
      throw std::exception();
    }
    _placesinHeap[idx] = heap.size();
    heap.push_back(idx);
    BubbleUp(heap.size() - 1);
  }
  I top_index() const { return heap[0]; }
  V top_value() const { return valueForI[top_index()]; }
  V value(I idx) const { return valueForI[idx]; }
  // WARNING this adds to an index
  void update(I idx, V value) {
    valueForI[idx] += value;
    if (inHeap(idx)) {
      if (cmp(value, 0))
        BubbleDown(_placesinHeap[idx]);
      else
        BubbleUp(_placesinHeap[idx]);
    }
  }

  void set_all(const std::vector<I> &e) {
    heap = e;
    size_t i = 0;
    for (auto v : e) {
      _placesinHeap[v] = i++;
    }
    // heapify();
  }
  bool empty() const { return heap.empty(); }
  bool inHeap(I idx) { return _placesinHeap[idx] != -1; }
  void removeIdx(I idx) {
    if (!inHeap(idx))
      return;
    auto index = _placesinHeap[idx];
    _placesinHeap[heap.back()] = index;
    heap[index] = heap.back();
    heap.resize(heap.size() - 1);
    int parentIndex = (index - 1) / 2;
    if (cmp(valueForI[heap[parentIndex]], valueForI[heap[index]])) {
      BubbleUp(index);
    } else {
      BubbleDown(index);
    }
    _placesinHeap[idx] = -1;
    // heapify();
  }
};
} // namespace ds
} // namespace HeiHGM::BMatching