/**
 * @file prune.h
 * @author Henrik Reinstädtler (reinstädtler@stud.uni-heidelberg.de)
 * @brief
 * @version 0.1
 * @date 2022-05-02
 *
 * @copyright Copyright (c) 2022
 *
 */
#pragma once
namespace HeiHGM::BMatching {
namespace bmatching {
namespace reductions_sorted {
/**
 * @brief Prunes the search space, by removing all edges that arent part of the
 * solution. USE ONLY if decisions are final.
 *
 * @tparam BMatching
 * @param hypergraph
 * @param m The matching
 * @return BMatching
 */
template <class BMatching>
void prune_graph_from_blocked_edges(typename BMatching::Graph_t &hypergraph,
                                    BMatching &m, unsigned int ts) {
  TIMED_FUNC(timerObn);

  for (auto e : m.non_free_edges()) {
    if (hypergraph.edgeSize(e) != 0 && hypergraph.edgeIsEnabled(e))
      hypergraph.removeEdge_sorted(e);
  }
}
} // namespace reductions_sorted
} // namespace bmatching
} // namespace HeiHGM::BMatching