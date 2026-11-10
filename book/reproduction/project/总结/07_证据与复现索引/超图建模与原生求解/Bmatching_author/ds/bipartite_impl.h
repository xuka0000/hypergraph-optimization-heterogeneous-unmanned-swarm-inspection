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
class BipartiteGraphImpl_ {
  BipartiteGraphImpl_(const BipartiteGraphImpl_ &) = delete;

  // negative size means disabled
  size_t _enabledCount;
  size_t _initialCount;
  bool _sorted = false;
  std::vector<int> _Sizes;
  std::vector<WeightType> _weights;
  std::vector<std::vector<CounterPartType>> _contained;
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
    bool operator!=(const SkippingIterator &rhs) const {
      return _id != rhs._id;
    }

    bool operator==(const SkippingIterator &rhs) const {
      return _id == rhs._id;
    }
  };

public:
  BipartiteGraphImpl_(size_t nEntries, bool already_sorted)
      : _enabledCount(nEntries), _initialCount(nEntries),
        _sorted(already_sorted), _Sizes(nEntries, 0), _weights(nEntries, 0),
        _contained(nEntries) {}
  Range<SkippingIterator> all() const {
    PartType b = 0;
    PartType e = _Sizes.size();
    return Range(SkippingIterator(_Sizes, b), SkippingIterator(_Sizes, e));
  }
  Range<typename std::vector<CounterPartType>::const_iterator>
  included(PartType e) const {
    // TODO(henrik):remove
    assert(!disabled(e));
    return Range(_contained[e].begin(), _contained[e].begin() + _Sizes[e]);
  }
  int size(PartType e) const { return _Sizes[e]; }
  int real_size(PartType e) const { return _contained[e].size(); }
  WeightType weight(PartType e) const { return _weights[e]; }
  void setWeight(PartType e, WeightType w) {
    assert(e < _weights.size());
    _weights[e] = w;
  }
  PartType add(std::vector<CounterPartType> parts) {
    _contained.push_back(parts);
    _Sizes.push_back(parts.size());
    return _Sizes.size() - 1;
  }
  bool disabled(PartType e) const {
    assert(e < _Sizes.size());
    return _Sizes[e] < 0;
  }
  bool empty(PartType e) const { return _Sizes[e] == 0; }
  void sort() {
    size_t i = 0;
    for (auto &counter : _contained) {
      std::sort(counter.begin(), counter.begin() + _Sizes[i++]);
    }
    _sorted = true;
  }
  void resort(PartType e) {
    std::sort(_contained[e].begin(), _contained[e].begin() + _Sizes[e]);
  }
  void changeEnablement(PartType e) {
    _Sizes[e] *= -1;
    if (_Sizes[e] < 0) {
      _enabledCount--;
    }
    if (_Sizes[e] > 0) {
      _enabledCount++;
    }
  }
  size_t enabledCount() const { return _enabledCount; }
  size_t initialCount() const { return _initialCount; }
  bool contains(PartType in, CounterPartType c) const {
    assert(!disabled(in));
    auto it = std::find(_contained[in].begin(),
                        _contained[in].begin() + _Sizes[in], c);
    return it != _contained[in].begin() + _Sizes[in];
  }
  void disableIn(PartType in, CounterPartType c) {
    _sorted = false;
    assert(!disabled(in));
    assert(_contained[in].size() >= _Sizes[in]);
    auto it = std::find(_contained[in].begin(),
                        _contained[in].begin() + _Sizes[in], c);
    assert(it != _contained[in].begin() + _Sizes[in]);
    if (it < (_contained[in].begin() + _Sizes[in] - 1)) {
      std::iter_swap(it, _contained[in].begin() + _Sizes[in] - 1);
    }
    _Sizes[in]--;
  }
  void disableIn_sorted(PartType in, CounterPartType c) {
    assert(_sorted);
    assert(!disabled(in));
    assert(_contained[in].size() >= _Sizes[in]);
    auto it = std::lower_bound(_contained[in].begin(),
                               _contained[in].begin() + _Sizes[in], c);
    assert(*it == c);
    assert(it != _contained[in].begin() + _Sizes[in]);
    std::rotate(it, it + 1, _contained[in].begin() + _Sizes[in]);
    _Sizes[in]--;
  }

  void enableIn(PartType in, CounterPartType c) {
    _sorted = false;
    assert(!disabled(in));
    auto it =
        std::find(_contained[in].begin() + _Sizes[in], _contained[in].end(), c);
    assert(it != _contained[in].end());
    if (it != (_contained[in].begin() + _Sizes[in])) {
      std::iter_swap(it, _contained[in].begin() + _Sizes[in]);
    }
    _Sizes[in]++;
  }
  void enableIn_sorted(PartType in, CounterPartType c) {
    assert(_sorted);
    assert(!disabled(in));
    auto it =
        std::find(_contained[in].begin() + _Sizes[in], _contained[in].end(), c);
    assert(it != _contained[in].end());
    if (it != (_contained[in].begin() + _Sizes[in])) {
      std::iter_swap(it, _contained[in].begin() + _Sizes[in]);
    }
    _Sizes[in]++;
    std::sort(_contained[in].begin(), _contained[in].begin() + _Sizes[in]);
  }
  bool isSorted() const { return _sorted; }
  void addTo(PartType e, CounterPartType c) {
    _contained[e].push_back(c);
    if (size(e) < real_size(e)) {
      std::swap(_contained[e].back(), _contained[e][_Sizes[e]]);
    }
    _Sizes[e]++;
  }
  // merges b into a and deactivates b
  std::vector<CounterPartType> merge(PartType a, PartType b) {
    assert(!disabled(b));
    assert(!disabled(a));

    std::vector<CounterPartType> removals;
    for (auto p : included(b)) {
      if (!contains(a, p)) {
        addTo(a, p);
        removals.push_back(p);
      }
    }
    changeEnablement(b);
    return removals;
  }
  // merges b into a and deactivates b
  std::vector<CounterPartType> merge_sorted(PartType a, PartType b) {
    assert(!disabled(b));
    assert(!disabled(a));
    assert(_sorted);
    std::vector<CounterPartType> removals;
    // calculate difference and store it
    std::set_difference(included(b).begin(), included(b).end(),
                        included(a).begin(), included(a).end(),
                        std::back_inserter(removals));
    size_t size_before = _contained[a].size();
    _contained[a].resize(_contained[a].size() + removals.size());
    // move the disabled edges to the end
    std::copy_backward(_contained[a].begin() + _Sizes[a],
                       _contained[a].begin() + size_before,
                       _contained[a].end());

    // copy in the removals
    std::copy(removals.begin(), removals.end(),
              _contained[a].begin() + _Sizes[a]);
    // and merge, as they are sorted
    std::inplace_merge(_contained[a].begin(), _contained[a].begin() + _Sizes[a],
                       _contained[a].begin() + _Sizes[a] + removals.size());
    // update the size
    _Sizes[a] += removals.size();
    changeEnablement(b);
    return removals;
  }
  void unmerge(PartType a, PartType b, std::vector<CounterPartType> removals) {
    assert(disabled(b));
    if (disabled(a)) {
      changeEnablement(a);
    }
    assert(!disabled(a));
    changeEnablement(b);
    for (auto p : removals) {
      if (contains(a, p)) {
        disableIn(a, p);
      }
    }
  }
  void unmerge_sorted(PartType a, PartType b,
                      const std::vector<CounterPartType> &removals) {
    assert(disabled(b));
    if (disabled(a)) {
      changeEnablement(a);
    }
    assert(!disabled(a));
    changeEnablement(b);
    resort(b);
    std::vector<CounterPartType> residual;
    std::set_difference(included(a).begin(), included(a).end(),
                        removals.begin(), removals.end(),
                        std::back_inserter(residual));
    std::copy(_contained[a].begin() + _Sizes[a], _contained[a].end(),
              _contained[a].begin() + residual.size());
    std::copy(residual.begin(), residual.end(), _contained[a].begin());
    _contained[a].resize(_contained[a].size() + residual.size() - _Sizes[a]);
    _Sizes[a] = residual.size();
  }
  // Replaces o by n
  void replace(PartType a, CounterPartType o, CounterPartType n) {
    // std::replace(_contained[a].begin(),_contained[a].end(),o)
    for (auto it = _contained[a].begin(); it != _contained[a].end(); it++) {
      if (*it == o) {
        (*it) = n;
        return;
      }
    }
  }
  // Replaces o by n
  bool replace_sorted(PartType a, CounterPartType o, CounterPartType n) {
    assert(!disabled(a));
    // std::replace(_contained[a].begin(),_contained[a].end(),o)
    for (auto it = _contained[a].begin(); it != _contained[a].end(); it++) {
      if (*it == o) {
        (*it) = n;
        if (_Sizes[a] < 0) {
          return true; // Skip over disabled
        }
        std::sort(_contained[a].begin(), _contained[a].begin() + _Sizes[a]);
        return true;
      }
    }
    return false;
  }
};
} // namespace ds
} // namespace HeiHGM::BMatching