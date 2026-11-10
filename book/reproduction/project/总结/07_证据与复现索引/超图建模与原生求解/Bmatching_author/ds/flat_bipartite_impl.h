#pragma once
#include <iostream>
#include <vector>

#include "utils/range.h"

namespace HeiHGM::BMatching {
namespace ds {
namespace {
using HeiHGM::BMatching::utils::Range;
}
template <typename PartType, typename CounterPartType, typename WeightType>
class FlatBipartiteGraphImpl_ {
  FlatBipartiteGraphImpl_(const FlatBipartiteGraphImpl_ &) = delete;
  // negative size means disabled
  size_t _enabledCount = 0;
  size_t _initialCount;
  bool _sorted = false;
  std::vector<WeightType> _weights;
  std::vector<CounterPartType> _contained;
  using CounterPartTypeIterator = size_t;
  // store length properties
  std::vector<std::pair<CounterPartTypeIterator, long>> offset;
  /// Skipps over all disabled and empty
  struct SkippingIterator {

    const std::vector<int> &_Sizes;
    PartType _id;

  public:
    using value_type = PartType;
    SkippingIterator(const std::vector<int> &_sizes, PartType init_val = 0)
        : _Sizes(_sizes), _id(init_val) {
      if (_id != _sizes.size() && _Sizes[_id] < 0) {
        operator++();
      }
    }
    // ! Returns the id of the element the iterator currently points to.
    PartType operator*() const { return _id; }
    SkippingIterator &operator++() {
      assert(_id < _Sizes.size());
      do {
        ++_id;
      } while (_id < _Sizes.size() && _Sizes[_id] < 0);
      return *this;
    }
    // ! Postfix increment. The iterator advances to the next valid element.
    SkippingIterator operator++(int) {
      SkippingIterator copy = *this;
      operator++();
      return copy;
    }
    bool operator!=(const SkippingIterator &rhs) { return _id != rhs._id; }

    bool operator==(const SkippingIterator &rhs) { return _id == rhs._id; }
  };
  struct SkipEmptyIterator {

    const std::vector<std::pair<CounterPartTypeIterator, long>> &offsets;

    PartType _id;

  public:
    using iterator_category = std::input_iterator_tag;
    using value_type = PartType;
    SkipEmptyIterator(
        const std::vector<std::pair<CounterPartTypeIterator, long>> &_offsets,
        PartType init_val = 0)
        : offsets(_offsets), _id(init_val) {
      if (_id < (offsets.size() - 1) && (offsets[_id].second) <= 0) {
        operator++();
      }
    }
    // ! Returns the id of the element the iterator currently points to.
    PartType operator*() const { return _id; }
    SkipEmptyIterator &operator++() {
      assert(_id < (offsets.size() - 1));
      do {
        ++_id;
      } while (_id < (offsets.size() - 1) && offsets[_id].second <= 0);
      return *this;
    }
    // ! Postfix increment. The iterator advances to the next valid element.
    SkipEmptyIterator operator++(int) {
      SkipEmptyIterator copy = *this;
      operator++();
      return copy;
    }
    bool operator!=(const SkipEmptyIterator &rhs) { return _id != rhs._id; }

    bool operator==(const SkipEmptyIterator &rhs) { return _id == rhs._id; }
  };

public:
  FlatBipartiteGraphImpl_(size_t minimum_edge_count, bool already_sorted)
      : _sorted(already_sorted), _weights(0, 0), _contained(0), offset(0) {
    offset.reserve(minimum_edge_count + 100);
    _weights.reserve(minimum_edge_count + 100);
    _contained.reserve(minimum_edge_count * 4);
    offset.push_back(std::make_pair<>(0, 0));
  }
  Range<SkipEmptyIterator> all() const {
    PartType b = 0;
    PartType e = offset.size() - 1;
    return Range(SkipEmptyIterator(offset, b), SkipEmptyIterator(offset, e));
  }
  Range<typename std::vector<CounterPartType>::const_iterator>
  included(PartType e) const {
    assert(std::is_sorted(_contained.begin() + offset[e].first,
                          _contained.begin() + offset[e].first +
                              offset[e].second));
    assert(!disabled(e));
    return Range(_contained.begin() + offset[e].first,
                 _contained.begin() + offset[e].first + offset[e].second);
  }
  const std::vector<WeightType> &weights() const { return _weights; }
  int size(PartType e) const { return offset[e].second; }
  int real_size(PartType e) const {
    return offset[e + 1].first - offset[e].first;
  }
  WeightType weight(PartType e) const {
    assert(e < (offset.size() - 1));
    return _weights[e];
  }
  void setWeight(PartType e, WeightType w) {
    assert(e < _weights.size());
    _weights[e] = w;
  }
  PartType add(const std::vector<CounterPartType> &parts, WeightType weight) {
    for (auto c : parts) {
      _contained.push_back(c);
    }
    offset.back() =
        std::make_pair<>(_contained.size() - parts.size(), parts.size());
    offset.push_back(std::make_pair<>(_contained.size(), 0));
    _weights.push_back(weight);
    _enabledCount++;
    assert(_weights.size() == offset.size() - 1);
    return offset.size() - 2;
  }
  bool disabled(PartType e) const {
    assert(e < offset.size() - 1);
    return offset[e].second < 0;
  }
  bool empty(PartType e) const { return offset[e].second == 0; }
  void sort() {
    for (auto &counter : offset) {
      std::sort(_contained.begin() + counter.first,
                _contained.begin() + counter.first + counter.second);
    }
    _sorted = true;
  }
  void resort(PartType e) {
    std::sort(_contained.begin() + offset[e].first,
              _contained.begin() + offset[e].first + offset[e].second);
  }
  void changeEnablement(PartType e) {
    offset[e].second *= -1;
    if (offset[e].second < 0) {
      _enabledCount--;
    }
    if (offset[e].second > 0) {
      _enabledCount++;
    }
    // assert(e != 9249);
  }
  size_t enabledCount() const { return _enabledCount; }
  size_t initialCount() const { return _initialCount; }
  bool contains(PartType e, CounterPartType c) const {
    assert(!disabled(e));
    auto it =
        std::find(_contained.begin() + offset[e].first,
                  _contained.begin() + offset[e].first + offset[e].second, c);
    return it != _contained.begin() + offset[e].first + offset[e].second;
  }
  void disableIn(PartType e, CounterPartType c) {
    _sorted = false;
    assert(!disabled(e));
    assert(real_size(e) >= size(e));
    auto it =
        std::find(_contained.begin() + offset[e].first,
                  _contained.begin() + offset[e].first + offset[e].second, c);
    assert(it != _contained.begin() + offset[e].first + offset[e].second);
    if (it < (_contained.begin() + offset[e].first + offset[e].second - 1)) {
      std::iter_swap(it, _contained.begin() + offset[e].first +
                             offset[e].second - 1);
    }
    offset[e].second--;
  }
  void disableIn_sorted(PartType e, CounterPartType c) {
    assert(size(e) <= real_size(e));
    assert(_sorted);
    assert(!disabled(e));
    assert(real_size(e) >= size(e));
    auto it = std::lower_bound(
        _contained.begin() + offset[e].first,
        _contained.begin() + offset[e].first + offset[e].second, c);
    assert(*it == c);
    assert(it != _contained.begin() + offset[e].first + offset[e].second);
    std::rotate(it, it + 1,
                _contained.begin() + offset[e].first + offset[e].second);
    offset[e].second--;
  }

  void enableIn(PartType e, CounterPartType c) {
    _sorted = false;
    assert(!disabled(e));
    auto it = std::find(_contained.begin() + offset[e].first + offset[e].second,
                        _contained.begin() + offset[e + 1].first, c);
    assert(it != _contained.begin() + offset[e + 1].first);
    if (it != (_contained.begin() + offset[e].first + offset[e].second)) {
      std::iter_swap(it,
                     _contained.begin() + offset[e].first + offset[e].second);
    }
    offset[e].second++;
  }
  void enableIn_sorted(PartType e, CounterPartType c) {
    assert(_sorted);
    assert(!disabled(e));
    auto it = std::find(_contained.begin() + offset[e].first + offset[e].second,
                        _contained.begin() + offset[e + 1].first, c);
    assert(it != _contained.begin() + offset[e + 1].first);
    if (it != _contained.begin() + (offset[e].first + offset[e].second)) {
      std::iter_swap(it,
                     _contained.begin() + offset[e].first + offset[e].second);
    }
    offset[e].second++;
    std::sort(_contained.begin() + offset[e].first,
              _contained.begin() + offset[e].first + offset[e].second);
  }
  bool isSorted() const { return _sorted; }
  // can only add to the end
  //   void addTo(PartType e, CounterPartType c) {
  //     _contained[e].push_back(c);
  //     if (size(e) < real_size(e)) {
  //       std::swap(_contained[e].back(), _contained[e][_Sizes[e]]);
  //     }
  //     _Sizes[e]++;
  //   }
  // merges b into a and deactivates b
  auto merge(PartType a, PartType b) {
    throw;
    assert(!disabled(b));
    assert(!disabled(a));
    std::vector<CounterPartType> removals;
    // for (auto p : included(b)) {
    //   if (!contains(a, p)) {
    //     addTo(a, p);
    //     removals.push_back(p);
    //   }
    // }
    // changeEnablement(b);
    return removals.size();
  }
  // merges b into a and deactivates b
  //  using assumption of a \cap b= \emptyset
  void merge_sorted(PartType a, PartType b) {
    if (a != offset.size() - 2) {
      // TODO replace with assert
      std::cerr << "can only merge into last edge." << std::endl;
      exit(1);
    };
    assert(!disabled(b));
    assert(!disabled(a));
    // assert(_sorted);
    std::vector<CounterPartType> removals;
    // // calculate difference and store it
    size_t size_before = offset[a].second;
    _contained.resize(_contained.size() + size(b));
    // move the disabled edges to the end
    std::copy_backward(_contained.begin() + offset[a].first + offset[a].second,
                       _contained.begin() + offset[a].first + size_before,
                       _contained.end());

    // // copy in the removals
    std::copy(included(b).begin(), included(b).end(),
              _contained.begin() + offset[a].first + offset[a].second);
    // // and merge, as they are sorted
    std::inplace_merge(_contained.begin() + offset[a].first,
                       _contained.begin() + offset[a].first + offset[a].second,
                       _contained.begin() + offset[a].first + offset[a].second +
                           size(b));
    // // update the size
    offset[a].second += size(b);
    // also update real size
    offset[a + 1].first += size(b);
    changeEnablement(b);
    assert(disabled(b));
    // return {};
  }
  void unmerge(PartType a, PartType b, size_t removals) { throw; }
  auto count() const { return offset.size() - 1; }
  bool unmerge_sorted(PartType a, PartType b) {

    assert(disabled(b));
    if (disabled(a)) {
      changeEnablement(a);
    }
    assert(!disabled(a));
    if (a != offset.size() - 2) {
      std::cerr << "can only unmerge the last edge on stack. a = " << a
                << " offset_s:" << offset.size() << std::endl;
      exit(1);
    }
    changeEnablement(b);
    // update meta data to match
    offset.back().first -= size(b);
    assert(!disabled(a)); // if "empty" remove edge
    if (offset[a].first == offset.back().first) {
      offset.resize(offset.size() - 1);
      return true;
    }
    return false;

    // resort(b);
    //  std::vector<CounterPartType> residual;
    //  std::set_difference(included(a).begin(), included(a).end(),
    //                      removals.begin(), removals.end(),
    //                      std::back_inserter(residual));
    //  std::copy(_contained[a].begin() + _Sizes[a], _contained[a].end(),
    //            _contained[a].begin() + residual.size());
    //  std::copy(residual.begin(), residual.end(), _contained[a].begin());
    // _contained[a].resize(_contained[a].size() + residual.size() - _Sizes[a]);
    // _Sizes[a] = residual.size();
  }
  // Replaces o by n
  void replace(PartType a, CounterPartType o, CounterPartType n) {
    throw;

    // std::replace(_contained[a].begin(),_contained[a].end(),o)
    for (auto it = _contained[a].begin(); it != _contained[a].end(); it++) {
      if (*it == o) {
        (*it) = n;
        return;
      }
    }
  }
  // Replaces o by n
  void replace_sorted(PartType a, CounterPartType o, CounterPartType n) {
    exit(1);
    throw;
    // std::replace(_contained[a].begin(),_contained[a].end(),o)
    throw;
    // for (auto it = _contained[a].begin(); it != _contained[a].end(); it++) {
    //   if (*it == o) {
    //     (*it) = n;
    //     if (offset[a].second < 0) {
    //       return; // Skip over disabled
    //     }
    //     std::sort(_contained[a].begin(), _contained[a].begin() + _Sizes[a]);
    //     return;
    //   }
    // }
  }
};
} // namespace ds
} // namespace HeiHGM::BMatching