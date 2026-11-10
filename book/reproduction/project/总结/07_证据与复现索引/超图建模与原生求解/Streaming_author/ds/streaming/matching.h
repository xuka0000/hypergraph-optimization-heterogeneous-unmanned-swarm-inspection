#pragma once

#include <numeric>
#include <vector>

namespace heihgm::ds::streaming {
template <class StreamableHypergraph>
class MatchingResult {
  std::vector<bool> matchedAt;
  std::vector<typename StreamableHypergraph::StreamMember> matched_edges;
  size_t num_vertices;

 public:
  struct InstanceImpl {
    size_t _n, _e;
    InstanceImpl(size_t n, size_t e) : _n(n), _e(e) {}
    auto currentNumNodes() const { return _n; }
    auto currentNumEdges() const { return _e; }
  };
  InstanceImpl instance_;
  InstanceImpl& instance() { return instance_; }
  using Graph_t = StreamableHypergraph;
  using NodeID_t = typename StreamableHypergraph::NodeType;
  using EdgeID_t = typename StreamableHypergraph::EdgeType;
  using NodeType = typename StreamableHypergraph::NodeType;
  using EdgeType = typename StreamableHypergraph::EdgeType;
  using WeightType = typename StreamableHypergraph::WeightType;
  explicit MatchingResult(size_t no_vertices, size_t no_edges)
      : instance_(no_vertices, no_edges),
        num_vertices(no_vertices),
        matchedAt(no_vertices, false) {}
  bool valid() {
    std::fill(matchedAt.begin(), matchedAt.end(), 0);
    for (const auto& edge : matched_edges) {
      for (const auto p : edge.pins) {
        if (matchedAt[p]) {
          return false;
        }
        matchedAt[p] = true;
      }
    }
    return true;
  }
  bool matchable(const typename StreamableHypergraph::StreamMember& edge) const {
    for (const auto p : edge.pins) {
      if (matchedAt[p]) {
        return false;
      }
    }
    return true;
  }
  bool matchable_or(const typename StreamableHypergraph::StreamMember& edge) const {
    bool blocked = false;
    for (const auto p : edge.pins) {
      blocked |= matchedAt[p];
    }
    return !blocked;
  }
  void addToMatching(const typename StreamableHypergraph::StreamMember& edge) {
    matched_edges.push_back(edge);
    for (const auto p : edge.pins) {
      matchedAt[p] = 1;
    }
  }
  size_t weight() {
    size_t result = 0;
    for (const auto& edge : matched_edges) {
      result += edge.w;
    }
    return result;
  }
  size_t size() { return matched_edges.size(); }
  size_t free_edges_size() { return 0; }
  double quality() const { return 0; }
  void save(std::string filename) {}
};
}  // namespace heihgm::ds::streaming