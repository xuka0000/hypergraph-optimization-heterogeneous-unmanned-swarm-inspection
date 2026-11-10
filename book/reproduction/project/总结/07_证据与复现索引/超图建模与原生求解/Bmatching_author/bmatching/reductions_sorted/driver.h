#pragma once
#include "bmatching/reductions_sorted/foldings.h"
#include "bmatching/reductions_sorted/prune.h"
#include "bmatching/reductions_sorted/removals.h"
#include "easylogging++.h"

namespace HeiHGM::BMatching {
namespace bmatching {
namespace reductions_sorted {
/**
 * @brief calls all removals/foldings exhaustively.
 *
 * @tparam BMatching
 * @param bm
 * @param mergeHistory to be used to uncontract later
 */
template <class BMatching>
void all_removals_exhaustive(BMatching &bm,
                             typename BMatching::Graph_t &hypergraph,
                             unsigned ts = 0, unsigned int max_runs = 10) {
  unsigned int runs = 0;
  TIMED_FUNC(timerBlkObj2);
  hypergraph.sort_weight();
  do {
    TIMED_SCOPE(timerBlkObj, "reduction-iter");
    unsigned old_ts = ts;
    ts = bm.change_timestamp_node();
    remove_uninteresting_nodes<BMatching>(hypergraph, bm, old_ts);
    // moved here to stop early
    //                                       break
    // calcDifference(remove_uninteresting);
    neighbourhood_removal<BMatching>(hypergraph, bm, old_ts);
    // calcDifference(neighbourhood_removal);
    // std::cout << bm.size() << ";" << bm.weight() << ";" << std::endl;
    weighted_isolated_edge_removal(hypergraph, bm, old_ts);
    // calcDifference(isolated_vertex);
    weighted_domination_removal(hypergraph, bm, old_ts);
    // calcDifference(weighted_domination);
    remove_uninteresting_nodes<BMatching>(hypergraph, bm, old_ts);
    weighted_edge_folding(hypergraph, bm, old_ts);
    //  calcDifference(vertex_folding);
    //  prune_graph_from_blocked_edges(hypergraph, bm, old_ts);
    //  std::cout <<"BEFORE:" << bm.size() << ";" << bm.weight() << ";" <<
    //  std::endl;
    remove_uninteresting_nodes<BMatching>(hypergraph, bm, old_ts);
    weighted_twin_folding(hypergraph, bm, old_ts);
    //  calcDifference(twin_folding);
  } while (ts != bm.change_timestamp_node() && ++runs < max_runs);
  VLOG(7) << "Passes in all:" << runs << std::endl;
  LOG_IF(max_runs == runs, INFO) << "Did not finish all removals." << std::endl;

  // remove_uninteresting_nodes<BMatching>(hypergraph, bm);
}

template <class BMatching>
void all_removals_exhaustive_configurable(
    BMatching &bm, typename BMatching::Graph_t &hypergraph, size_t max_size_nr,
    size_t clique_size_isolated_edge, size_t max_edge_size_isolated_edge,
    size_t max_size_weighted_domination, size_t max_size_twin_folding,
    unsigned ts = 0, unsigned int max_runs = 10) {
  unsigned int runs = 0;
  do {
    unsigned old_ts = ts;
    ts = bm.change_timestamp();
    remove_uninteresting_nodes<BMatching>(hypergraph,
                                          bm); // moved here to stop early
    //                                       break
    // calcDifference(remove_uninteresting);
    neighbourhood_removal<BMatching>(hypergraph, bm, old_ts, max_size_nr);
    // calcDifference(neighbourhood_removal);
    // std::cout << bm.size() << ";" << bm.weight() << ";" << std::endl;
    weighted_isolated_edge_removal(hypergraph, bm, old_ts,
                                   clique_size_isolated_edge,
                                   max_edge_size_isolated_edge);
    // calcDifference(isolated_vertex);
    weighted_domination_removal(hypergraph, bm, old_ts,
                                max_size_weighted_domination);
    // calcDifference(weighted_domination);
    weighted_edge_folding(hypergraph, bm, old_ts);
    // calcDifference(vertex_folding);
    prune_graph_from_blocked_edges(hypergraph, bm, old_ts);
    // std::cout <<"BEFORE:" << bm.size() << ";" << bm.weight() << ";" <<
    // std::endl;
    weighted_twin_folding(hypergraph, bm, old_ts, max_size_twin_folding);
    // calcDifference(twin_folding);
  } while (ts != bm.change_timestamp() && ++runs < max_runs);
  remove_uninteresting_nodes<BMatching>(hypergraph, bm);
}
} // namespace reductions_sorted
} // namespace bmatching
} // namespace HeiHGM::BMatching