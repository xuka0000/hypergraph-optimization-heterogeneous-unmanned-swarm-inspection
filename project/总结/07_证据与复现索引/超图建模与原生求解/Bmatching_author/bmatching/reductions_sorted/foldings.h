
#pragma once
#include "ds/hashing-config.h"
#include "easylogging++.h"
#include "utils/range.h"
#include <algorithm>
#include <set>
#include <vector>
namespace HeiHGM::BMatching {
namespace bmatching {
namespace reductions_sorted {
namespace {
using HeiHGM::BMatching::utils::Range;
}
template <class BMatching>
void weighted_edge_folding(typename BMatching::Graph_t &hypergraph,
                           BMatching &m, unsigned int ts) {
  // store edge that is still there, and the two merges, and the weight of w(e)
  TIMED_FUNC(timerObn);

  for (auto v : m.nodes_monitored(ts)) {
    if (m.residual_capacity(v) != 1 || m.matchableAtNode(v) != 2 ||
        hypergraph.nodeDegree(v) < 2) {
      continue;
    }
    for (auto ep = hypergraph.incidentEdges(v).begin();
         ep < hypergraph.incidentEdges(v).end() - 1;
         ++ep) { // last not be checked because weight has to be higher
      auto e = *ep;
      //  find out if two neighbourhood
      std::vector<typename BMatching::EdgeID_t> neighbours_indistinct;
      int neighbours_count = 0;
      // skip all non size 2 hypergraphs
      // Requires call to remove_uninsteristing nodes before
      if (hypergraph.edgeSize(e) != 2) {
        continue;
      }
      for (auto p : hypergraph.pins(e)) {
        if (m.residual_capacity(p) == 1 && m.matchableAtNode(p) == 2) {
          for (auto e2 : hypergraph.incidentEdges(p)) {
            neighbours_count += (e2 != e && hypergraph.edgeWeight(e2) >
                                                hypergraph.edgeWeight(e));
            if (e2 != e && m.isMatchable(e2) &&
                hypergraph.edgeWeight(e2) <=
                    hypergraph.edgeWeight(
                        e)) { // because of size 2: w(N)-min(N)= max(N)
              neighbours_indistinct.push_back(e2);
            }
          }
        } else if (m.matchableAtNode(p) > 1) {
          neighbours_count++;
          break;
        }
      }
      std::sort(neighbours_indistinct.begin(), neighbours_indistinct.end());
      auto last1 = std::unique(neighbours_indistinct.begin(),
                               neighbours_indistinct.end());
      neighbours_indistinct.erase(last1, neighbours_indistinct.end());
      if (neighbours_indistinct.size() != 2 || neighbours_count != 0) {
        continue;
      }
      // check if neighbours are not adjacent
      bool adjacent = false;
      for (auto p : hypergraph.pins(neighbours_indistinct[0])) {
        adjacent |= std::binary_search(
            hypergraph.pins(neighbours_indistinct[1]).begin(),
            hypergraph.pins(neighbours_indistinct[1]).end(), p);
        if (adjacent) {
          break;
        }
      }
      if (adjacent || (hypergraph.edgeWeight(neighbours_indistinct[0]) +
                       hypergraph.edgeWeight(neighbours_indistinct[1])) <=
                          hypergraph.edgeWeight(e)) {
        continue; // skip because they are neighbouring or higherweight
      }
      // m.addGuarantee(hypergraph.edgeWeight(e));
      auto new_weight = hypergraph.edgeWeight(neighbours_indistinct[0]) +
                        hypergraph.edgeWeight(neighbours_indistinct[1]) -
                        hypergraph.edgeWeight(e);
      assert(new_weight > 0);
      // First  merge them
      // handle center
      m.markFreeEdgeAsBlocked(e);
      hypergraph.removeEdge_sorted(e);

      auto rep = hypergraph.addMergeEdge(neighbours_indistinct[0],
                                         new_weight); // initialise with right

      m.deactivateCenterAndAckknowledge(neighbours_indistinct[0], rep);
      m.markFreeEdgeAsUsed(neighbours_indistinct[1]);
      hypergraph.mergeEdges_sorted(rep, neighbours_indistinct[1]);
      // VLOG(7) << "Merger:" << m2.first << std::endl;
      // VLOG(7) << "Merger second:" << m2.second << std::endl;

      m.merge_history().push_history(
          typename BMatching::MergeHistory::WeightTransformation(
              e, hypergraph.edgeWeight(e), neighbours_indistinct, {e}),
          {typename BMatching::MergeHistory::MergeInfo(
               rep, neighbours_indistinct[0]),
           typename BMatching::MergeHistory::MergeInfo(
               rep, neighbours_indistinct[1])});
#ifdef HASHING_ACTIVATED
      m.recomputeHash(rep);
#endif
      m.notify_change_along(rep);
    }
  }
}

template <class BMatching>
void weighted_vertex_unfolding(typename BMatching::Graph_t &hypergraph,
                               BMatching &m) {
  while (!m.merge_history().empty()) {
    auto last_merge = m.merge_history().pop_back();
    bool remove = false;
    for (auto mergesIt = last_merge.first.rbegin();
         mergesIt != last_merge.first.rend(); mergesIt++) {
      assert(!remove);
      remove =
          hypergraph.unmergeEdge_sorted(mergesIt->merged, mergesIt->removed);
      VLOG(7) << "merged:" << mergesIt->merged << std::endl;
      if (remove) {
        assert(mergesIt == last_merge.first.rend() - 1);
        // now do checks
        bool inSolutionOuter = m.isInMatching(mergesIt->merged);
        VLOG(7) << mergesIt->merged << " was removed and replaced by "
                << mergesIt->removed << " was "
                << (inSolutionOuter ? "in" : " not in") << std::endl;

        // last removed is identity
        m.removeRepAndactivateCenter(mergesIt->merged, mergesIt->removed);

        if (inSolutionOuter) {
          // outer are in matching
          for (auto outer :
               Range(last_merge.first.rbegin(), last_merge.first.rend() - 1)) {
            m.unmarkFreeEdgeAsUsed(outer.removed);
#ifdef DEBUG
            m.checkAllPins(outer.removed);
#endif
            m.addToMatching(outer.removed);

            VLOG(7) << "Adding outer: " << outer.removed << std::endl;
          }
        } else {
          VLOG(7) << "Center in solution " << std::endl;

          for (auto c : last_merge.second.center) {
            VLOG(7) << "Adding center: " << c << std::endl;
            hypergraph.enableEdge_sorted(c);
            m.unmarkFreeEdgeAsUsed(c);
            m.addToMatching(c);
          }
        }
      }
    }
    assert(remove);
  }
}

template <class BMatching>
void weighted_twin_folding(typename BMatching::Graph_t &hypergraph,
                           BMatching &m, unsigned int ts, int max_size = 4) {
  // store edge that is still there, and the two merges, and the weight of
  // w(e)
  TIMED_FUNC(timerObn);
  m.resetScannedEdges();
  // collect and store all candidates.
  std::vector<std::pair<typename BMatching::EdgeID_t,
                        std::pair<typename BMatching::EdgeType,
                                  std::vector<typename BMatching::EdgeID_t>>>>
      candidates;

  for (auto v : m.nodes_monitored(ts)) {
    if (m.residual_capacity(v) != 1 || m.matchableAtNode(v) != 2 ||
        hypergraph.nodeDegree(v) < 2) {
      continue;
    }
    for (auto ep = hypergraph.incidentEdges(v).begin();
         ep < hypergraph.incidentEdges(v).end() - 1; ++ep) {
      auto e = *ep;
      if (hypergraph.edgeSize(e) > max_size || !m.isMatchable(e) ||
          m.scanned(e)) {
        continue;
      }
      m.scanned(e) = true;
      bool only_degree2 = true; // only degree 2
      // QUESTION(henrik): capacity constraint?? or assume, that full edges
      // are not left anymore
      std::set<typename BMatching::EdgeID_t> neighbours;
      assert(m.isMatchable(e));
      for (auto p : hypergraph.pins(e)) {
        only_degree2 &= (m.capacity(p) - m.matchesAtNode(p) == 1) &&
                        m.matchableAtNode(p) == 2;
        if (!only_degree2) {
          break;
        }
        for (auto e2 : hypergraph.incidentEdges(p)) {
          if (e2 != e && m.isMatchable(e2)) {
            neighbours.insert(e2);
          }
        }
      }
      // skip if other degree
      if (!only_degree2) {
        continue;
      }
      // we need them sorted, for comparision later TODO(henrik): maybe
      // uniqueness
      typename BMatching::EdgeType res = 0;
      for (auto n : neighbours) {
        res += n;
      }
      candidates.push_back(std::make_pair<>(
          e, std::make_pair<>(res, std::vector<typename BMatching::EdgeType>(
                                       neighbours.begin(), neighbours.end()))));
    }
  }

  if (candidates.size() < 2) {
    return;
  }
  // std::cout << "TWIN folding candidates: " << candidates.size() <<
  // std::endl;
  //  sort the candidates according to number of neighbours
  std::sort(candidates.begin(), candidates.end(), [](auto a, auto b) {
    return a.second.second.size() < b.second.second.size() &&
           b.second.first < b.second.first;
  });
  auto ax = candidates.begin();
  while (ax != candidates.end()) {
    // find the end iterator of this size
    auto bx = ax + 1;
    for (; bx != candidates.end() &&
           ax->second.second.size() == bx->second.second.size() &&
           bx->second.first == ax->second.first;
         bx++) {
    }
    // std::cout<<bx-ax<<" "<<ax->second.size()<<std::endl;
    for (auto ix = ax; ix != bx; ix++) {
      if (!m.isMatchable(ix->first)) {
        continue;
      }
      // TODO(henrik): what about three candidates or more
      for (auto jx = ix + 1; jx != bx; jx++) {
        if (jx->first == ix->first) {
          continue; // skip empty edges
        }
        if (!m.isMatchable(jx->first)) {
          continue; // skip all matched
        }
        if (!m.isMatchable(ix->first)) {
          break; // early exit
        }
        // check if not neighbouring in one direction (as it is symmetric)
        // TODO(henrik) remove as it is symmetric & own edge is not included
        if (std::find(jx->second.second.begin(), jx->second.second.end(),
                      ix->first) == jx->second.second.end()) {
          // now make sure that neighbours are the same
          bool same_neighbours = true;
          auto iterator_other = jx->second.second.begin();
          for (auto kx = ix->second.second.begin();
               kx != ix->second.second.end(); kx++) {
            if (!m.isMatchable(*kx)) { // only twin fold non neighboring pairs.
              same_neighbours = false;
              break;
            }
            // do {
            iterator_other++;
            // } while (iterator_other < jx->second.end() &&
            //          !m.isMatchable(*iterator_other));
            same_neighbours &= iterator_other < jx->second.second.end() &&
                               ((*kx) == (*iterator_other));
          }
          if (!same_neighbours) {
            continue;
          }
          bool common_pin = false;
          int weight = 0;
          int min_weight = std::numeric_limits<int>().max();
          // check for independence & dominance
          for (auto kx = ix->second.second.begin();
               kx != ix->second.second.end(); kx++) {
            // if (!m.isMatchable(*kx)) { // jump over neighbors
            //   continue;
            // }
            weight += hypergraph.edgeWeight(*kx);
            min_weight = std::min(hypergraph.edgeWeight(*kx), min_weight);
            for (auto lx = kx + 1; lx != ix->second.second.end(); lx++) {
              for (auto p : hypergraph.pins(*kx)) {
                for (auto l : hypergraph.pins(*lx)) {
                  common_pin |= (l == p);
                }
                if (common_pin)
                  break;
              }
            }
          }
          if (common_pin) {
            continue;
          }
          const auto twin_weight = hypergraph.edgeWeight(ix->first) +
                                   hypergraph.edgeWeight(jx->first);
          if (twin_weight >= weight) {
            // std::cout<<"Weight domination"<<std::endl;
            m.addToMatching(ix->first);
            m.addToMatching(jx->first);
            return;
            continue;
          }
          // check if we can fold, last check to not fold single edges, as
          // some error occurs here
          //  && ((weight-min_weight)!=0)
          if (twin_weight > (weight - min_weight)) {
            auto new_weight = weight - twin_weight;
            auto n1x = ix->second.second.begin();

            std::vector<typename BMatching::EdgeID_t> edges;
            auto rep =
                hypergraph.addMergeEdge(*n1x,
                                        new_weight); // initialise with right
            m.deactivateCenterAndAckknowledge(*n1x, rep);
            std::vector<typename BMatching::MergeHistory::MergeInfo> mergers(
                {typename BMatching::MergeHistory::MergeInfo(
                    rep, *n1x)}); // this creates empty edges
            m.markFreeEdgeAsBlocked(ix->first);
            m.markFreeEdgeAsBlocked(jx->first);

            for (auto n2x = n1x + 1; n2x != ix->second.second.end(); n2x++) {
              m.markFreeEdgeAsUsed(*n2x);
              hypergraph.mergeEdges_sorted(rep, *n2x);
              mergers.push_back(
                  typename BMatching::MergeHistory::MergeInfo(rep, *n2x));
              edges.push_back(*n2x);
            }
            // m.addGuarantee(twin_weight);
            std::vector<typename BMatching::EdgeID_t> center{jx->first,
                                                             ix->first};
            m.merge_history().push_history(
                typename BMatching::MergeHistory::WeightTransformation(
                    ix->first, hypergraph.edgeWeight(rep), edges, center),
                mergers);
            hypergraph.removeEdge_sorted(ix->first);
            hypergraph.removeEdge_sorted(jx->first);
            m.notify_change_along(rep);
#ifdef HASHING_ACTIVATED
            m.recomputeHash(rep);
#endif
          }
        }
      }
    }
    ax = bx;
  }
  // std::cout << "TOTAL weight domination pairs: " << count << " of " <<
  // count2
  // << std::endl;
  // std::cout << "TOTAL weight folding pairs: " << count3 << " of " << count2
  // << std::endl;
}
} // namespace reductions_sorted
} // namespace bmatching
} // namespace HeiHGM::BMatching