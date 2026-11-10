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
class WeightSortedBipartiteGraphImpl_ {
  WeightSortedBipartiteGraphImpl_(const WeightSortedBipartiteGraphImpl_ &) =
      delete;
  // negative size means disabled
  size_t _enabledCount;
  size_t _initialCount;
  bool _sorted = false;
  std::vector<int> _Sizes;
  std::vector<WeightType> _weights;
  const std::vector<WeightType> &edge_weight;

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
  WeightSortedBipartiteGraphImpl_(size_t nEntries,
                                  const std::vector<WeightType> &weight_edges)
      : _enabledCount(nEntries), _initialCount(nEntries), _Sizes(nEntries, 0),
        _weights(nEntries, 0), edge_weight(weight_edges), _contained(nEntries) {
  }
  Range<SkippingIterator> all() const {
    PartType b = 0;
    PartType e = _Sizes.size();
    return Range(SkippingIterator(_Sizes, b), SkippingIterator(_Sizes, e));
  }
  Range<typename std::vector<CounterPartType>::const_iterator>
  included(PartType e) const {
    // TODO(henrik):remove
    assert(!disabled(e));
    assert(isSorted(e));
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
    if (_sorted) {
      return;
    }
    for (auto b : all()) {
      resort(b);
    }
    _sorted = true;
  }
  void resort(PartType e) {
    assert(!disabled(e));
    std::sort(_contained[e].begin(), _contained[e].begin() + _Sizes[e],
              [&](auto a, auto b) { return edge_weight[a] > edge_weight[b]; });
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
    assert(!disabled(in));
    assert(isSorted(in));
    assert(_contained[in].size() >= _Sizes[in]);
    auto it = std::find(_contained[in].begin(),
                        _contained[in].begin() + _Sizes[in], c);
    assert(it < _contained[in].begin() + _Sizes[in]);
    if (it < (_contained[in].begin() + _Sizes[in] - 1)) {
      std::iter_swap(it, _contained[in].begin() + _Sizes[in] - 1);
    }
    _Sizes[in]--;
    resort(in);
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
    assert(!disabled(in));
    auto it =
        std::find(_contained[in].begin() + _Sizes[in], _contained[in].end(), c);
    assert(it != _contained[in].end());
    if (it != (_contained[in].begin() + _Sizes[in])) {
      std::iter_swap(it, _contained[in].begin() + _Sizes[in]);
    }
    _Sizes[in]++;
    resort(in);
  }
  bool isSorted() const { return _sorted; }
  bool isSorted(PartType e) const {
    return std::is_sorted(
        _contained[e].begin(), _contained[e].begin() + _Sizes[e],
        [&](auto a, auto b) { return edge_weight[a] > edge_weight[b]; });
    ;
  }

  void addTo(PartType e, CounterPartType c) {
    _contained[e].push_back(c);
    if (size(e) < real_size(e)) {
      std::swap(_contained[e].back(), _contained[e][_Sizes[e]]);
    }
    _Sizes[e]++;
  }

  // Replaces o by n
  bool replace_sorted(PartType a, CounterPartType o, CounterPartType n) {
    assert(!disabled(a));
    assert(isSorted(a));
    // std::replace(_contained[a].begin(),_contained[a].end(),o)
    for (auto it = _contained[a].begin(); it != _contained[a].end(); it++) {
      if (*it == o) {
        (*it) = n;
        if (_Sizes[a] < 0) {
          assert(false);
          return true; // Skip over disabled
        }
        resort(a);
        assert(isSorted(a));
        return true;
      }
    }
    resort(a);

    return false;
  }
  // Replaces o by n
  bool replace(PartType a, CounterPartType o, CounterPartType n) {
    assert(!disabled(a));
    // std::replace(_contained[a].begin(),_contained[a].end(),o)
    for (auto it = _contained[a].begin(); it != _contained[a].end(); it++) {
      if (*it == o) {
        (*it) = n;
        if (_Sizes[a] < 0) {
          return true; // Skip over disabled
        }
        resort(a);
        return true;
      }
    }
    return false;
  }
  /// @brief Checks whether e is in n by using the weight sorted array
  /// @param e
  /// @param n
  /// @return
  bool isIn(CounterPartType e, PartType n) {
    auto target = edge_weight[e];
    for (auto entry = _contained[n].begin();
         entry != _contained[n].begin() + _Sizes[n]; entry++) {
      if (*entry == e) {
        return true;
      }
      if (edge_weight[*entry] < target) {
        return false;
      }
    }
    return false;
    /*
    //alternative impls:
    return std::find(_contained[n].begin(), _contained[n].begin() + _Sizes[n],
                     e) != _contained[n].begin() + _Sizes[n];
    auto it = std::lower_bound(
        _contained[n].begin(), _contained[n].begin() + _Sizes[n], e,
        [&](auto a, auto b) { return edge_weight[a] > edge_weight[b]; });
    if (it >= (_contained[n].begin() + _Sizes[n]) ||
        edge_weight[*it] != edge_weight[e]) {
      return false;
    }
    while (it < _contained[n].begin() + _Sizes[n] &&
           edge_weight[*it] == edge_weight[e]) {
      if (*it == e) {
        return true;
      }
      it++;
    }
    return false;
    */
  }
};
} // namespace ds
} // namespace HeiHGM::BMatching