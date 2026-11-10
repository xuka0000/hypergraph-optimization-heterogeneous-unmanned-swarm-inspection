#pragma once
#include "ds/modifiable_hypergraph.h"
#include "easylogging++.h"
#include "utils/random.h"
#include "utils/range.h"
#include <algorithm>
#include <fstream>
#include <functional>
#include <iostream>
#include <numeric>
#include <vector>

#include "ds/hashing-config.h"

#ifdef HASHING_ACTIVATED
#include "math/wide_integer/uintwide_t.h"

#define HashBits 512
using hash_t = ::math::wide_integer::uintwide_t<HashBits, std::uint32_t>;

#endif
namespace HeiHGM::BMatching {
namespace ds {
namespace {
using HeiHGM::BMatching::utils::Randomize;
using HeiHGM::BMatching::utils::Range;
} // namespace
/**
 * @brief Matching based on MIS problem (Dahlum, Andrade et al.)
 *
 * Result is an array of typename Graph::EdgeTypes. First comes solution, then
 * and then the free ones, the blocked ones (in at least one node)
 * @tparam Graph
 */
template <typename HashType, typename PrimeType>
HashType modulo(HashType a, PrimeType b) {
  return a & b;
}
template <class Graph = StandardIntegerHypergraph
#ifdef HASHING_ACTIVATED
          ,
          typename HashType = hash_t
#endif
          >
class BMatching {
  // std::shared_ptr<std::vector<typename Graph::EdgeType>> neighbors;
  // std::shared_ptr<std::vector<size_t>> neighbors_offsets;
  unsigned int global_age = 0;
  unsigned int global_change = 0;
  unsigned int global_change_node = 0;
#ifdef HASHING_ACTIVATED
  int prime = 7;
  int max_sig = HashBits * log(2.0) / log((double)prime + 1.0) - 1;
  std::vector<HashType> hashes_;
#endif
#define MATCHES 0
#define MATCHABLE 1
#define CAP 2
  std::vector<std::tuple<unsigned int, int, unsigned int>>
      At; // first matches at, second matchableAt, third capacity
  std::vector<unsigned int> age_;
  std::vector<unsigned int> change_;
  std::vector<bool> scanned_;
  std::vector<unsigned int> change_node_;

  std::vector<unsigned int>
      blocks_ed_on_pins; // estimate on how many pins it is blocked or blocks
  std::vector<typename Graph::EdgeType>
      result; // stores how many matches there are at this node
  std::vector<size_t>
      position_in_result; // stores the edges that are in the match
  size_t solution_size = 0;
  size_t free_size;
  size_t solution_size_exact = 0;
  size_t blocked_size_exact = 0;
  void moveFreeToSolution(typename Graph::EdgeType e) {
    auto pos_e = position_in_result[e];
    auto new_pos_e = solution_size;
    auto old = result[solution_size];
    assert((pos_e < free_size + solution_size) &&
           (solution_size <= pos_e)); // Assure that it is free
    std::swap(result[pos_e], result[new_pos_e]);
    position_in_result[e] = new_pos_e;
    position_in_result[old] = pos_e;
    solution_size++;
    free_size--;
  }
  void moveFreeToNonFree(typename Graph::EdgeType e) {
    auto pos_e = position_in_result[e];
    auto new_pos_e = solution_size + free_size - 1;
    auto old = result[new_pos_e];
    assert((solution_size <= pos_e) && (solution_size + free_size > pos_e));
    std::swap(result[pos_e], result[new_pos_e]);
    position_in_result[e] = new_pos_e;
    position_in_result[old] = pos_e;
    free_size--;
  }
  void moveNonFreeToFree(typename Graph::EdgeType e) {
    auto pos_e = position_in_result[e];
    auto new_pos_e = solution_size + free_size;
    auto old = result[new_pos_e];
    assert((solution_size + free_size <= pos_e));
    std::swap(result[pos_e], result[new_pos_e]);
    position_in_result[e] = new_pos_e;
    position_in_result[old] = pos_e;
    free_size++;
  }
  void moveSolutionToFree(typename Graph::EdgeType e) {
    auto pos_e = position_in_result[e];
    auto new_pos_e = solution_size - 1;
    auto old = result[new_pos_e];
    assert((pos_e < solution_size)); // Assure that it is free
    std::swap(result[pos_e], result[new_pos_e]);
    position_in_result[e] = new_pos_e;
    position_in_result[old] = pos_e;
    solution_size--;
    free_size++;
  }

  template <class IteratorType, typename ValueType> struct SkippingIterator {

    const std::vector<ValueType> *_other;
    IteratorType _id;
    IteratorType _end;
    ValueType _lower;

  public:
    using value_type = typename Graph::EdgeType;
    /**
     * @brief Construct a new Skipping Iterator object
     *
     * @param other
     * @param lower
     * @param begin
     * @param end
     */
    SkippingIterator(const std::vector<ValueType> *other, ValueType lower,
                     IteratorType begin, IteratorType end)
        : _other(other), _id(begin), _end(end), _lower(lower) {
      if (_id < _end && _other->at(*_id) < lower) {
        operator++();
      }
    }
    SkippingIterator() = default;

    SkippingIterator(const SkippingIterator &other) = default;
    SkippingIterator &operator=(const SkippingIterator &other) = default;

    SkippingIterator(SkippingIterator &&other) = default;
    SkippingIterator &operator=(SkippingIterator &&other) = default;

    ~SkippingIterator() = default;
    // ! Returns the id of the element the iterator currently points to.
    typename IteratorType::value_type operator*() const { return *_id; }
    SkippingIterator &operator++() {
      // std::cout << _end - _id << " " << *_id << " : " << _other->size()
      //          << std::endl;
      do {
        ++_id;
        // std::cout << *_id << std::endl;
      } while (_id < _end && _other->at(*_id) < _lower);
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
    bool operator<(const SkippingIterator &rhs) { return _id < rhs._id; }
  };
  template <typename ItType, typename ValueType> struct SkipTimestampIterator {

    const std::vector<ValueType> *_other;
    ItType _id;
    ItType _end;
    ValueType _lower;
    const Graph *graph_;

  public:
    /*public std::iterator<std::forward_iterator_tag, // iterator_category
                               ,  // value_type
                               std::ptrdiff_t,            // difference_type
                               const typename Graph::EdgeType *, // pointer
                               typename Graph::EdgeType> */
    using value_type = ItType;
    /**
     * @brief Construct a new Skipping Iterator object
     *
     * @param other
     * @param lower
     * @param begin
     * @param end
     */
    SkipTimestampIterator(const Graph *graph,
                          const std::vector<ValueType> *other, ValueType lower,
                          ItType begin, ItType end)
        : _other(other), _id(begin), _end(end), _lower(lower), graph_(graph) {
      if (_id < _end &&
          (_other->at(_id) < lower || !graph_->nodeIsEnabled(_id))) {
        operator++();
      }
    }
    SkipTimestampIterator() = default;

    SkipTimestampIterator(const SkipTimestampIterator &other) = default;
    SkipTimestampIterator &
    operator=(const SkipTimestampIterator &other) = default;

    SkipTimestampIterator(SkipTimestampIterator &&other) = default;
    SkipTimestampIterator &operator=(SkipTimestampIterator &&other) = default;

    ~SkipTimestampIterator() = default;
    // ! Returns the id of the element the iterator currently points to.
    ItType operator*() const { return _id; }
    SkipTimestampIterator &operator++() {
      // std::cout << _end - _id << " " << *_id << " : " << _other->size()
      //          << std::endl;
      do {
        ++_id;
        // std::cout << *_id << std::endl;
      } while (_id < _end &&
               (_other->at(_id) < _lower || !graph_->nodeIsEnabled(_id)));
      return *this;
    }
    // ! Postfix increment. The iterator advances to the next valid element.
    SkipTimestampIterator operator++(int) {
      SkipTimestampIterator copy = *this;
      operator++();
      return copy;
    }
    bool operator!=(const SkipTimestampIterator &rhs) { return _id != rhs._id; }

    bool operator==(const SkipTimestampIterator &rhs) { return _id == rhs._id; }
    bool operator<(const SkipTimestampIterator &rhs) { return _id < rhs._id; }
  };

public:
  class MergeHistory {
  public:
    struct MergeInfo {
      typename Graph::EdgeType merged;
      typename Graph::EdgeType removed;
      MergeInfo(typename Graph::EdgeType a, typename Graph::EdgeType b)
          : merged(a), removed(b) {}
    };
    struct WeightTransformation {
      typename Graph::EdgeType edge;
      int oldWeight;
      std::vector<typename Graph::EdgeType> deactivated;
      std::vector<typename Graph::EdgeType> center;

      WeightTransformation(typename Graph::EdgeType e, int _oldWeight,
                           std::vector<typename Graph::EdgeType> _deactivated,
                           std::vector<typename Graph::EdgeType> _center =
                               std::vector<typename Graph::EdgeType>())
          : edge(e), oldWeight(_oldWeight), deactivated(_deactivated),
            center(_center) {}
    };

  private:
    std::vector<std::pair<std::vector<MergeInfo>, WeightTransformation>>
        transformations;

  public:
    bool empty() const { return transformations.empty(); }
    size_t size() const { return transformations.size(); }
    void push_history(WeightTransformation w, std::vector<MergeInfo> mergers) {
      assert(mergers.size() != 0);
      transformations.push_back(std::make_pair<>(mergers, w));
    }
    WeightTransformation pop_back_restore(Graph &hypergraph,
                                          BMatching<Graph> &m) {
      throw;
    }
    WeightTransformation pop_back_restore_sorted(Graph &hypergraph,
                                                 BMatching<Graph> &m) {
      auto last = transformations.back();
      for (auto mergesIt = last.first.rbegin(); mergesIt != last.first.rend();
           mergesIt++) {
        hypergraph.unmergeEdge_sorted(mergesIt->merged, mergesIt->removed,
                                      mergesIt->contract_info.first,
                                      mergesIt->contract_info.second);
        m.unmarkFreeEdgeAsUsed(mergesIt->removed);
        m.checkAllPins(mergesIt->removed);
        m.checkAllPins(mergesIt->merged);
      }

      hypergraph.setEdgeWeight(last.second.edge, last.second.oldWeight);
      if (!hypergraph.edgeIsEnabled(last.second.edge)) {
        hypergraph.enableEdge_sorted(last.second.edge);
      }
      transformations.pop_back();
      return last.second;
    }
    auto pop_back() {
      auto last = transformations.back();
      transformations.pop_back();
      return last;
    }
  };

private:
  MergeHistory _merge_history;

public:
  const Graph *_g;
  using Graph_t = Graph;
  using NodeID_t = typename Graph::NodeType;
  using EdgeID_t = typename Graph::EdgeType;
  using NodeType = typename Graph::NodeType;
  using EdgeType = typename Graph::EdgeType;
  using WeightType = typename Graph::WeightType;

  using CapacityFunction =
      std::function<unsigned int(typename Graph::NodeType)>;
  auto capacity(typename Graph::NodeType n) { return std::get<CAP>(At[n]); }
  BMatching(const Graph *g)
      :
#ifdef HASHING_ACTIVATED
        hashes_(g->initialNumEdges(), 0),
#endif
        At(g->initialNumNodes(), std::make_tuple(0, 0, 0)),
        age_(g->initialNumEdges(), 0), scanned_(g->initialNumEdges(), 0),
        change_node_(g->initialNumNodes(), 0),
        blocks_ed_on_pins(g->initialNumEdges(), 0),
        result(g->initialNumEdges(), 0),
        position_in_result(g->initialNumEdges(), 0),
        free_size(g->initialNumEdges()), _g(g) {
    std::iota(result.begin(), result.end(), 0);
    std::iota(position_in_result.begin(), position_in_result.end(), 0);
    for (auto p : _g->nodes()) {
      std::get<MATCHABLE>(At[p]) = _g->nodeDegree(p);
      std::get<CAP>(At[p]) = _g->nodeWeight(p);
    }
  }
  /**
   * @brief Returns how many pins of the edge are blocked or is blocking
   *
   * @param e
   * @return int
   */
  int blockedByOrBlockingPins(typename Graph::EdgeType e) const {
    return blocks_ed_on_pins[e];
  }
#ifdef HASHING_ACTIVATED
  int maxHash() { return max_sig; }
  HashType hash(typename Graph::EdgeType e) const { return hashes_[e]; }
  void computeInitialHashes() {
    TIMED_FUNC(timeOf);
    auto maxEdgeSize = 0;
    for (auto e : _g->edges()) {
      maxEdgeSize = std::max(_g->edgeSize(e), maxEdgeSize);
    }
    int computed = HashBits / maxEdgeSize * log(2);
    prime = std::max(511, (1 << computed) - 1);
    LOG(INFO) << "Choosen modulo:" << prime << " at " << computed << " maxE"
              << maxEdgeSize;
    max_sig = ((double)HashBits) * log(2.0) / log((double)prime + 1.0) - 1;
    LOG(INFO) << max_sig << std::endl;
    assert(hashes_.size() == _g->realEdgeSize());
    std::for_each(free_edges().begin(), free_edges().end(), [&](auto edge) {
      if (_g->edgeSize(edge) < max_sig) {
        hashes_[edge] =
            std::transform_reduce<decltype(_g->pins(edge).begin()), HashType>(
                _g->pins(edge).begin(), _g->pins(edge).end(), 1,
                [](HashType a, HashType b) { return a * b; },
                [&](typename Graph::EdgeType a) {
                  return modulo(a, prime) + 1;
                });
      }
    });
    VLOG(7) << "Hashes recomputed" << std::endl;
  }
  void recomputeHash(typename Graph::EdgeType edge) {
    hashes_[edge] = 0;
    if (_g->edgeSize(edge) < max_sig) {
      hashes_[edge] =
          std::transform_reduce<decltype(_g->pins(edge).begin()), HashType>(
              _g->pins(edge).begin(), _g->pins(edge).end(), 1,
              [](HashType a, HashType b) { return a * b; },
              [&](typename Graph::EdgeType a) { return modulo(a, prime) + 1; });
    }
  }
#endif
  typename Graph::EdgeType randomNonFree() const {
    if (solution_size + free_size < result.size() - 1) {
      auto pos = Randomize::instance().getRandomInt(solution_size + free_size,
                                                    result.size() - 1);
      while (!_g->edgeIsEnabled(result[pos])) {
        pos = Randomize::instance().getRandomInt(solution_size + free_size,
                                                 result.size() - 1);
      }
      return result[pos];
    }
    return 0;
  }
  typename Graph::EdgeType randomEdgeNonExact() const {
    if (solution_size_exact + free_size <
        result.size() - blocked_size_exact - 1) {
      auto pos = Randomize::instance().getRandomInt(
          solution_size_exact, result.size() - blocked_size_exact - 1);
      int tries = 0;
      while (!_g->edgeIsEnabled(result[pos])) {
        tries++;
        pos = Randomize::instance().getRandomInt(
            solution_size_exact, result.size() - blocked_size_exact - 1);
        if (tries > 1000) {
          // sample for max 1000 times
          return 0;
        }
      }
      return result[pos];
    }
    return 0;
  }
  typename Graph::EdgeType randomNonFreeInexact() const {
    if (solution_size + free_size < result.size() - blocked_size_exact - 1) {
      auto pos = Randomize::instance().getRandomInt(
          solution_size + free_size, result.size() - blocked_size_exact - 1);
      int tries = 0;
      while (!_g->edgeIsEnabled(result[pos])) {
        tries++;
        pos = Randomize::instance().getRandomInt(
            solution_size + free_size, result.size() - blocked_size_exact - 1);
        if (tries > 1000) {
          // sample for max 1000 times
          return 0;
        }
      }
      return result[pos];
    }
    return 0;
  }
  bool isMatchable(typename Graph::EdgeType edge) const {
    auto pos_e = position_in_result[edge];
    return solution_size <= pos_e && pos_e < solution_size + free_size;
  }
  // also used for hashes
  unsigned int &age(typename Graph::EdgeType edge) { return age_[edge]; }
  void resetAge() { std::fill(age_.begin(), age_.end(), 0); }
  unsigned int change_timestamp() const { return global_change; }
  unsigned int change_timestamp_node() const { return global_change_node; }

  void resetScannedEdges() { std::fill(scanned_.begin(), scanned_.end(), 0); }
  auto scanned(typename Graph::EdgeType e) & { return scanned_[e]; }
  auto timestamp_node(typename Graph::NodeType n) { return change_node_[n]; }
  void notify_change(typename Graph::EdgeType edge) {
    // change_[edge] = ++global_change;
  }
  void notify_change_along(typename Graph::EdgeType edge) {
    global_change_node++;
    for (auto p : _g->pins(edge)) {
      change_node_[p] = global_change_node;
    }
  }
  void notify_removal_node(typename Graph::NodeType node) {

    for (auto incE : _g->incidentEdges(node)) {
      if (isMatchable(incE)) {
        notify_change_along(incE);
      }

      // TODO: may be notify adjacent
    }
  }
  void NonNotaddToMatching(typename Graph::EdgeType edge) {
    assert(edge >= 0 && edge < position_in_result.size());
    assert(!isInMatching(edge));
    assert(isMatchable(edge));
    assert(_g->edgeIsEnabled(edge));
    VLOG(8) << "Adding edge NONNot " << edge << std::endl;
    moveFreeToSolution(edge);
    // need to check on each pin
    for (auto p : _g->pins(edge)) {
      std::get<MATCHABLE>(At[p])--;
      assert(std::get<MATCHABLE>(At[p]) >= 0);
      if ((++std::get<MATCHES>(At[p])) == capacity(p)) {
        VLOG(8) << "Exhausted " << p << std::endl;
        // full, now move to nonfree all free marked edges
        for (auto incEdge : _g->incidentEdges(p)) {
          blocks_ed_on_pins[incEdge]++; // includes the blocking edge
          if (!isBlocked(incEdge) && isMatchable(incEdge)) {
            for (auto pi : _g->pins(incEdge)) {
              std::get<MATCHABLE>(At[pi])--;
              assert(std::get<MATCHABLE>(At[pi]) >= 0);
            }
            moveFreeToNonFree(incEdge);
            assert(isBlocked(incEdge));
          }
        }
        assert(std::get<MATCHABLE>(At[p]) == 0);
      }
    }
    age_[edge] = ++global_age;
  }
  void addToMatching(typename Graph::EdgeType edge) {
    assert(edge >= 0 && edge < position_in_result.size());
    assert(!isInMatching(edge));
    assert(isMatchable(edge));
    assert(_g->edgeIsEnabled(edge));
    VLOG(8) << "Adding edge " << edge << std::endl;
    moveFreeToSolution(edge);
    // need to check on each pin
    for (auto p : _g->pins(edge)) {
      std::get<MATCHABLE>(At[p])--;
      assert(std::get<MATCHABLE>(At[p]) >= 0);
      if ((++std::get<MATCHES>(At[p])) == capacity(p)) {
        VLOG(8) << "Exhausted " << p << std::endl;
        // full, now move to nonfree all free marked edges
        for (auto incEdge : _g->incidentEdges(p)) {
          blocks_ed_on_pins[incEdge]++; // includes the blocking edge
          if (!isBlocked(incEdge) && isMatchable(incEdge)) {
            for (auto pi : _g->pins(incEdge)) {
              std::get<MATCHABLE>(At[pi])--;
              assert(std::get<MATCHABLE>(At[pi]) >= 0);
            }
            moveFreeToNonFree(incEdge);
            assert(isBlocked(incEdge));
            notify_change_along(incEdge);
          }
        }
        assert(std::get<MATCHABLE>(At[p]) == 0);
      }
    }
    age_[edge] = ++global_age;
  }
  // checks if matching is of capacity b (for asserts)
  bool bMatching() {
    for (auto v : _g->nodes()) {
      std::get<MATCHES>(At[v]) = 0;
    }
    for (auto sol : solution()) {
      for (auto p : _g->pins(sol)) {
        std::get<MATCHES>(At[p])++;
      }
    }
    for (auto v : _g->nodes()) {
      if (std::get<MATCHES>(At[v]) > capacity(v)) {
        return false;
      }
    }
    return true;
  }
  bool bMatchingInternalCorrect() {
    auto copy = At;
    std::fill(copy.begin(), copy.end(), std::make_tuple(0, 0, 0));
    for (auto v : _g->nodes()) {
      for (auto e : _g->incidentEdges(v)) {
        std::get<MATCHABLE>(copy[v]) += isMatchable(e);
        std::get<MATCHES>(copy[v]) += isInMatching(e);
      }
    }
    for (auto v : _g->nodes()) {
      if (std::get<MATCHES>(At[v]) != std::get<MATCHES>(copy[v])) {
        std::cout << "WRONG MAtches" << std::endl;
        return false;
      }
      if (std::get<MATCHABLE>(At[v]) != std::get<MATCHABLE>(copy[v])) {
        std::cout << "WRONG matchable" << std::endl;
        return false;
      }
    }
    return true;
  }
  bool isMaximal() { return free_size == 0; }
  void maximize() {
    while (free_size != 0) {
      addToMatching(result[solution_size]);
    }
  }
  void maximizeWeightratio() {

    bool candidatesRemaining = true;
    while (candidatesRemaining) {
      std::vector<typename Graph::EdgeType> candidates;
      for (auto e = result.begin() + solution_size;
           e != result.begin() + solution_size + free_size; e++) {

        candidates.push_back(*e);
      }
      std::sort(candidates.begin(), candidates.end(), [this](auto a, auto b) {
        return ((double)this->_g->edgeWeight(a)) /
                   ((double)this->_g->edgeSize(a)) >
               ((double)this->_g->edgeWeight(b)) /
                   ((double)this->_g->edgeSize(b));
      });
      if (candidates.size() > 0) {
        addToMatching(candidates[0]);
      }
      candidatesRemaining = candidates.size() > 1;
    }
  }
  /**
   * @brief  Can be used to move never matched edge to non free
   *
   * @param edge
   */
  void markFreeEdgeAsUsed(typename Graph::EdgeType edge) {
    notify_change_along(edge);
    // assert(_g->edgeSize(edge) != 0);
    moveFreeToNonFree(edge);
  }
  void markFreeEdgeAsBlocked(typename Graph::EdgeType edge) {
    notify_change_along(edge);
    for (auto pi : _g->pins(edge)) {
      std::get<MATCHABLE>(At[pi])--;
      assert(std::get<MATCHABLE>(At[pi]) >= 0);
    }
    // assert(_g->edgeSize(edge) != 0);
    moveFreeToNonFree(edge);
  }
  /**
   * @brief checks if an edge is blocked, useful after unmerging an edge (if
   * empty edge will not move)
   *
   * @param edge
   */
  void checkAllPins(typename Graph::EdgeType edge) {
    if (_g->edgeSize(edge) == 0) {
      return;
    }
    for (auto p : _g->pins(edge)) {
      std::get<MATCHES>(At[p]) = 0;
      std::get<MATCHABLE>(At[p]) = 0;
      if (_g->nodeIsEnabled(p)) {
        for (auto e : _g->incidentEdges(p)) {
          std::get<MATCHES>(At[p]) += isInMatching(e);
          std::get<MATCHABLE>(At[p]) += isMatchable(e);
        }
      }
    }
    blocks_ed_on_pins[edge] = 0;
    for (auto p : _g->pins(edge)) {
      blocks_ed_on_pins[edge] += matchesAtNode(p) == capacity(p);
    }
    if (blocks_ed_on_pins[edge] > 0 && isMatchable(edge)) {
      moveFreeToNonFree(edge);
    }
    if (blocks_ed_on_pins[edge] == 0 && isBlocked(edge)) {
      moveNonFreeToFree(edge);
    }
  }
  void unmarkFreeEdgeAsUsed(typename Graph::EdgeType edge) {
    moveNonFreeToFree(edge);
    blocks_ed_on_pins[edge] = 0;
    notify_change_along(edge);
  }
  void NonNotRemoveFromMatching(typename Graph::EdgeType edge) {
    VLOG(8) << "Removing edge non not " << edge << std::endl;
    assert(isInMatching(edge));
    moveSolutionToFree(edge);
    // need to check on each pin
    for (auto p : _g->pins(edge)) {
      std::get<MATCHABLE>(At[p])++;
      if ((std::get<MATCHES>(At[p])--) == capacity(p)) {
        // full, need to check on each pin, whether the capacity constraint is
        // not given anymore,
        for (auto incEdge : _g->incidentEdges(p)) {
          blocks_ed_on_pins[incEdge]--; // on this pin it is now non blocking
          if (isBlocked(incEdge) && !isInMatching(incEdge) &&
              matchableByCapacity(incEdge)) {
            moveNonFreeToFree(incEdge);
            for (auto v : _g->pins(incEdge)) {
              std::get<MATCHABLE>(At[v])++;
            }
          }
        }
      }
    }
  }
  void removeFromMatching(typename Graph::EdgeType edge) {
    VLOG(8) << "Removing edge " << edge << std::endl;
    assert(isInMatching(edge));
    notify_change_along(edge);
    moveSolutionToFree(edge);
    // need to check on each pin
    for (auto p : _g->pins(edge)) {
      std::get<MATCHABLE>(At[p])++;
      if ((std::get<MATCHES>(At[p])--) == capacity(p)) {
        // full, need to check on each pin, whether the capacity constraint is
        // not given anymore,
        for (auto incEdge : _g->incidentEdges(p)) {
          blocks_ed_on_pins[incEdge]--; // on this pin it is now non blocking
          if (isBlocked(incEdge) && !isInMatching(incEdge) &&
              matchableByCapacity(incEdge)) {
            moveNonFreeToFree(incEdge);
            for (auto v : _g->pins(incEdge)) {
              std::get<MATCHABLE>(At[v])++;
            }
          }
        }
      }
    }
  }
  /**
   * @brief Checks if an edge is matchable by recalculation of the capacity
   * constraint.
   *
   * @param edge
   * @return true
   * @return false
   */
  bool matchableByCapacity(typename Graph::EdgeType edge) const {
    return blocks_ed_on_pins[edge] == 0;
  }
  Range<typename std::vector<typename Graph::EdgeType>::const_iterator>
  solution() const {
    return {result.begin(), result.begin() + solution_size};
  }
  Range<typename std::vector<typename Graph::EdgeType>::iterator>
  solution_inexact() {
    return {result.begin() + solution_size_exact,
            result.begin() + solution_size};
  }
  unsigned int matchesAtNode(typename Graph::NodeType node) {
    return std::get<MATCHES>(At[node]);
  }
  unsigned int blockedAtNode(typename Graph::NodeType node) {
    unsigned int blocked = 0;
    for (auto e : _g->incidentEdges(node)) {
      blocked += isBlocked(e);
    }
    return blocked;
  }
  auto matchableAtNode(typename Graph::NodeType node) {
    return std::get<MATCHABLE>(At[node]);
  }
  bool isBlocked(typename Graph::EdgeType e) const {
    return solution_size + free_size <= position_in_result[e];
  }
  /**
   * @brief Iterates over the free edges, that have been touched since "since".
   *
   * @param since
   * @return auto
   */
  auto free_edges_monitored(unsigned int since) {
    return Range<SkippingIterator<
        typename std::vector<typename Graph::EdgeType>::iterator,
        unsigned int>>(
        SkippingIterator<
            typename std::vector<typename Graph::EdgeType>::iterator,
            unsigned int>(&change_, since, result.begin() + solution_size,
                          result.begin() + solution_size + free_size),
        SkippingIterator<
            typename std::vector<typename Graph::EdgeType>::iterator,
            unsigned int>(&change_, since,
                          result.begin() + solution_size + free_size,
                          result.begin() + solution_size + free_size));
  }
  auto nodes_monitored(unsigned int since) {
    return Range<SkipTimestampIterator<typename Graph::NodeType, unsigned int>>(
        SkipTimestampIterator<typename Graph::NodeType, unsigned int>(
            _g, &change_node_, since, 0, _g->initialNumNodes()),
        SkipTimestampIterator<typename Graph::NodeType, unsigned int>(
            _g, &change_node_, since, _g->initialNumNodes(),
            _g->initialNumNodes()));
  }
  /**
   * @brief Iterates over the solution edges, that have been touched since
   * "since".
   *
   * @param since
   * @return auto
   */
  auto solution_edges_monitored(unsigned int since) {
    return Range<SkippingIterator<
        typename std::vector<typename Graph::EdgeType>::iterator,
        unsigned int>>(
        SkippingIterator<
            typename std::vector<typename Graph::EdgeType>::iterator,
            unsigned int>(&change_, since, result.begin(),
                          result.begin() + solution_size),
        SkippingIterator<
            typename std::vector<typename Graph::EdgeType>::iterator,
            unsigned int>(&change_, since, result.begin() + solution_size,
                          result.begin() + solution_size));
  }
  auto solution_edges_monitored_inexact(unsigned int since) {
    return Range<SkippingIterator<
        typename std::vector<typename Graph::EdgeType>::iterator,
        unsigned int>>(
        SkippingIterator<
            typename std::vector<typename Graph::EdgeType>::iterator,
            unsigned int>(&change_, since, result.begin() + solution_size_exact,
                          result.begin() + solution_size),
        SkippingIterator<
            typename std::vector<typename Graph::EdgeType>::iterator,
            unsigned int>(&change_, since, result.begin() + solution_size,
                          result.begin() + solution_size));
  }
  Range<typename std::vector<typename Graph::EdgeType>::iterator> free_edges() {
    return Range<typename std::vector<typename Graph::EdgeType>::iterator>(
        result.begin() + solution_size,
        result.begin() + solution_size + free_size);
  }
  Range<typename std::vector<typename Graph::EdgeType>::iterator>
  non_free_edges() {
    return Range<typename std::vector<typename Graph::EdgeType>::iterator>(
        result.begin() + solution_size + free_size, result.end());
  }
  Range<typename std::vector<typename Graph::EdgeType>::iterator>
  non_free_edges_inexact() {
    return Range<typename std::vector<typename Graph::EdgeType>::iterator>(
        result.begin() + solution_size + free_size,
        result.end() - blocked_size_exact);
  }
  bool isInMatching(typename Graph::EdgeType edge) {
    return position_in_result[edge] < solution_size;
  }
  size_t size() const { return solution_size; }
  int weight() const {
    int w = 0;
    for (auto s : solution()) {
      w += this->_g->edgeWeight(s);
    }
    return w;
  }
  void printInfo() {
    std::cout << "solution_size:" << solution_size << std::endl;
    std::cout << "free_size:" << free_size << std::endl;
    std::cout << "bmatching: " << bMatching() << std::endl;
    std::cout << "weight: " << weight() << std::endl;
    std::cout << "MergeHistory: " << merge_history().size() << std::endl;
  }
  void reset() {
    auto res = solution_inexact();
    std::vector<EdgeType> res1(res.end() - res.begin());
    std::copy(res.begin(), res.end(), res1.begin());
    for (auto r : res1) {
      NonNotRemoveFromMatching(r);
    }
  }
  void save(std::string filename) {
    std::ofstream file(filename);
    file << size() << std::endl;
    for (auto e = result.begin(); e != result.begin() + solution_size; e++) {

      file << *e << std::endl;
    }
  }
  const MergeHistory &merge_history() const { return _merge_history; }
  MergeHistory &merge_history() { return _merge_history; }
  // Sets the current state of the matching as exact.
  // can be only called once.
  void set_exact() {
    assert(solution_size_exact == 0);
    assert(blocked_size_exact == 0);
    solution_size_exact = solution_size;
    blocked_size_exact = result.size() - (solution_size + free_size);
  }
  bool isExact(typename Graph::EdgeType e) {
    return (position_in_result[e] < solution_size_exact) ||
           (position_in_result[e] >= (result.size() - blocked_size_exact));
  }
  void deactivateCenterAndAckknowledge(typename Graph::EdgeType center,
                                       typename Graph::EdgeType new_edge) {
    assert(isMatchable(center));
    assert(new_edge == position_in_result.size());
    // replace center with new edge in array
    position_in_result.push_back(position_in_result[center]);
    result[position_in_result[center]] = new_edge;
    // result.push_back(center);
    position_in_result[center] = position_in_result.size() - 1;
    scanned_.push_back(0);
    // change_.push_back(++global_change);
    blocks_ed_on_pins.push_back(blocks_ed_on_pins[center]);
    age_.push_back(0);
#ifdef HASHING_ACTIVATED
    hashes_.push_back(0);
    recomputeHash(new_edge);
#endif
    assert(!isMatchable(center));
    assert(isMatchable(new_edge));
  }
  void removeRepAndactivateCenter(typename Graph::EdgeType rep,
                                  typename Graph::EdgeType center) {
    assert(rep == position_in_result.size() - 1);
    auto new_pos = position_in_result[rep];

    position_in_result[center] = new_pos;
    result[new_pos] = center;
    position_in_result.resize(position_in_result.size() - 1);
    blocks_ed_on_pins.resize(blocks_ed_on_pins.size() - 1);
  }
  auto nthHeaviestWeightAt(NodeType n, int nth) {
    assert(nth > 0);
    int target = 0;
    for (auto elem : _g->incidentEdges(n)) { // TODO skip matchable edges

      if (isMatchable(elem)) {
        target++;
      }
      if (target == nth) {
        return _g->edgeWeight(elem);
      }
    }
    return 0;
  }
  auto nthHeaviestAt(NodeType n, int nth) {
    assert(nth > 0);
    int target = 0;
    assert(_g->isSorted());
    for (auto elem : _g->incidentEdges(n)) { // TODO skip matchable edges
      if (isMatchable(elem)) {
        target++;
      }
      if (target == nth) {
        return elem;
      }
    }
    return *_g->incidentEdges(n).begin();
  }
  auto nthHeaviestWeightAt(NodeType n, int nth, EdgeType unequal) {
    assert(nth > 0);
    assert(_g->isSorted());
    int target = 0;
    if (nth > 10) {
      return _g->edgeWeight(*(_g->incidentEdges(n).begin() + nth - 1));
    }
    for (auto elem : _g->incidentEdges(n)) {

      if (isMatchable(elem) && unequal != elem) {
        target++;
      }
      if (target == nth) {
        return _g->edgeWeight(elem);
      }
    }
    return 0;
  }
  auto free_edges_size() const { return free_size; }
  auto nthHeaviestAt(NodeType n, int nth, EdgeType unequal) {
    assert(_g->isSorted());
    assert(nth > 0);
    int target = 0;
    for (auto elem : _g->incidentEdges(n)) { // TODO skip matchable edges
      if (isMatchable(elem) && elem != unequal) {
        target++;
      }
      if (target == nth) {
        return elem;
      }
    }
    return *_g->incidentEdges(n).begin();
  }
  auto residual_capacity(NodeType n) { return capacity(n) - matchesAtNode(n); }
};
using StandardIntegerHypergraphBMatching = BMatching<StandardIntegerHypergraph>;
} // namespace ds
} // namespace HeiHGM::BMatching