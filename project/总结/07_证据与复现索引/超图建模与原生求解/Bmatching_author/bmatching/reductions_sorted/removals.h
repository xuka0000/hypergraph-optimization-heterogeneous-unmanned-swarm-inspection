/**
 * @file removals.h
 * @author Henrik Reinstädtler (reinstaedtler@stud.uni-heidelberg.de)
 * @brief
 * @version 0.1
 * @date 2022-05-01
 *
 * @copyright Copyright (c) 2022
 *
 */
#pragma once
#include <iostream>
#include <map>
#include <numeric>

#include "ds/hashing-config.h"

namespace HeiHGM::BMatching {
namespace bmatching {
namespace reductions_sorted {
constexpr bool multiple_times = false;
template <class BMatching>
void remove_uninteresting_nodes(typename BMatching::Graph_t &hypergraph,
                                BMatching &m, unsigned int ts = 0) {
  TIMED_FUNC(timerObn);
  for (auto n :
       m.nodes_monitored(ts)) // TODO(henrik): make nodes also listen to changes
  {
    // is the remainder capacity big enough to add all?
    // blocked at are only the edges, that are non matchable, but not matched
    if (m.residual_capacity(n) >= m.matchableAtNode(n)) {
      // check all edges, if addable (size==1)
      for (auto e : hypergraph.incidentEdges(n)) {

        if (hypergraph.edgeSize(e) == 1 && m.isMatchable(e)) {
          m.addToMatching(e);
        }
      }
      m.notify_removal_node(n);
      hypergraph.removeNode_sorted(n);
    }
  }
}
template <class BMatching>
void neighbourhood_removal(typename BMatching::Graph_t &hypergraph,
                           BMatching &m, unsigned int ts, int max_size = 10) {
  TIMED_FUNC(timerObn);
  auto old_ts = ts;
  m.resetScannedEdges();
  do {
    for (auto v : m.nodes_monitored(old_ts)) {
      auto residual = m.residual_capacity(v);
      if (residual >= m.matchableAtNode(v)) {
        continue;
      }
      int count = 0;
      for (auto e : hypergraph.incidentEdges(v)) {
        if (!m.isMatchable(e)) {
          continue;
        }
        if (residual < ++count) { // early exit
          break;
        }
        if (m.scanned(e)) {
          continue;
        }
        m.scanned(e) = true;
        typename BMatching::WeightType total_dominated_weight =
            0; // can recompute!
        if (hypergraph.edgeSize(e) > max_size) {
          continue;
        }
        for (auto p : hypergraph.pins(e)) {
          if (m.residual_capacity(p) >= (m.matchableAtNode(p))) {
            // Do not have to look at pin at all
            continue;
          }
          const int target = m.residual_capacity(p);
          assert(target > 0);
          assert(m.nthHeaviestWeightAt(p, target, e) != 0);
          total_dominated_weight += m.nthHeaviestWeightAt(p, target, e);

          // early check
          if (total_dominated_weight > hypergraph.edgeWeight(e)) {
            break;
          }
        }
        if (total_dominated_weight <= hypergraph.edgeWeight(e)) {
          m.addToMatching(e);
        }
      }
    }
    old_ts = ts;
    ts = m.change_timestamp();
  } while (multiple_times && ts != old_ts);
}
template <
    class BMatching = HeiHGM::BMatching::ds::StandardIntegerHypergraphBMatching>
void weighted_isolated_edge_removal(typename BMatching::Graph_t &hypergraph,
                                    BMatching &m, unsigned int ts,
                                    const size_t max_clique_size = 80,
                                    const int max_edge_size = 8) {
  TIMED_FUNC(timerObn);
  m.resetAge();
  m.resetScannedEdges();
  // TODO: replace capacity by residual capacity
  for (auto v : m.nodes_monitored(ts)) {
    if (m.matchableAtNode(v) > 0 && // at least one edge has to be free
        m.residual_capacity(v) == 1) {
      auto max_edge = m.nthHeaviestAt(v, 1);
      if (!m.isMatchable(max_edge)) {
        continue;
      }
      if (hypergraph.edgeSize(max_edge) > max_edge_size) {
        continue; // TODO discuss
      }
      // or newer than timestamp
      if (!m.scanned(max_edge)) { // m.age(max_edge) == 0) {
        bool B = true;
        std::vector<typename BMatching::EdgeType> should_be_blocked;
        std::map<typename BMatching::EdgeType, unsigned long> blocking;

        unsigned long count = 1;
        for (auto v2 : hypergraph.pins(max_edge)) {
          if (m.nthHeaviestWeightAt(v2, 1) > hypergraph.edgeWeight(max_edge) ||
              blocking.size() > max_clique_size) {
            B = false;
            break;
          }
          if (m.residual_capacity(v2) == 1) {
            for (auto f : hypergraph.incidentEdges(v2)) {
              if (m.isMatchable(f)) {
                blocking[f] += count;
                m.scanned(f) = true;
              }
            }
            if (count > std::numeric_limits<unsigned long>::max() >>
                2) { // too many to compute.
              B = false;
              // std::cout << "Upper limit reached" << std::endl;
              break;
            }
            count *= 2;
          } else {
            for (auto f : hypergraph.incidentEdges(v2)) {
              if (m.isMatchable(f)) {
                should_be_blocked.push_back(f);
                m.scanned(f) = true; // set also others to scanned (as they
                                     // havve been dominated)
              }
            }
          }
        }
        if (B) {
          bool S = true;
          for (auto e : should_be_blocked) {
            S &= blocking[e] > 0;
          }
          if (S && blocking.size() <= max_clique_size) {
            for (auto fx = blocking.begin(); fx != blocking.end(); fx++) {
              for (auto gx = std::next(fx); gx != blocking.end(); gx++) {
                if ((gx->second & fx->second) == 0) {
                  // neighbor check
                  bool success = false;
                  for (auto p : hypergraph.pins(gx->first)) {
                    if (m.residual_capacity(p) == 1) {
                      for (auto q : hypergraph.pins(fx->first)) { // TODO assume
                                                                  // sorted
                        if (q == p) {
                          success = true;
                          break;
                        }
                      }
                      if (success) {
                        break;
                      }
                    }
                  }
                  if (!success) { // may be automatic stop
                    B = false;
                    break;
                  }
                }
              }
              if (!B) {
                break;
              }
            }
            if (B) {
              m.addToMatching(max_edge);
              // correctly we would need to update the scanned of edges
              // incident to this.
              //  is ok to move data, does not effect
              //    if (e.first != max_edge) {
              //      hypergraph.removeEdge_sorted(
              //          e.first); // directly remove blocked to update nodes
              //    }
              //  }
            } else {
              m.scanned(max_edge) = true;
            }
          } else {
            m.scanned(max_edge) = true;
          }
        } else {
          m.scanned(max_edge) = true;
        }
      }
    }
  }
}
template <typename T> T fast_mod(const T input, const T ceil) {
  // apply the modulo operator only when needed
  // (i.e. when the input is greater than the ceiling) // input % ceil; //

  return input >= ceil ? input % ceil : input;
  // NB: the assumption here is that the numbers are positive
}
// IDEA define max size not only for contained but also a min size
template <class BMatching>
void weighted_domination_removal(typename BMatching::Graph_t &hypergraph,
                                 BMatching &m, unsigned int ts,
                                 int max_size = 6, int max_candidates = 6,
                                 int max_checks = 3) {
  TIMED_FUNC(timerObn);
  auto old_ts = ts;
  m.resetScannedEdges();
#ifdef HASHING_ACTIVATED
  for (auto node : m.nodes_monitored(ts)) {
    for (auto edge : hypergraph.incidentEdges(node)) {
      if (m.scanned(edge)) {
        continue;
      }
      m.scanned(edge) = true;
      m.recomputeHash(edge);
    }
  }
#endif
  m.resetScannedEdges();
  do {
    for (auto v : m.nodes_monitored(ts)) {
      if ((m.residual_capacity(v)) != 1 || m.matchableAtNode(v) < 2) {
        continue;
      }
      int checked = 0;
      for (auto ep = hypergraph.incidentEdges(v).begin();
           ep < hypergraph.incidentEdges(v).end() - 1; ep++) {
        auto e_sub = *ep;
        if (!m.isMatchable(e_sub) || hypergraph.edgeSize(e_sub) > max_size) {
          continue;
        }
        if (m.scanned(e_sub)) {
          continue;
        }
        if (checked++ > max_checks) {
          break;
        }
        m.scanned(e_sub) = true;
        std::vector<std::pair<typename BMatching::EdgeType, bool>> candidates;
        std::map<typename BMatching::EdgeType, unsigned long> nummeration;
        // This reduction seems to be important to shrink the search space
        // ,esp on 1 coauthor/citation coPapersCiteseer.graph.hgr.random100

        for (auto e_superx = ep + 1;
             e_superx < hypergraph.incidentEdges(v).end(); e_superx++) {
          auto e_super = *e_superx;
#ifdef HASHING_ACTIVATED

          auto a_m = m.hash(e_super);
          auto b_m = m.hash(e_sub);
#endif
          // guaranteed by sorting :)
          //  if (hypergraph.edgeWeight(e_sub) <
          //  hypergraph.edgeWeight(e_super))
          //  {
          //    continue;
          //  }
          if (m.isMatchable(e_super) && // maybe just &
              hypergraph.edgeSize(e_super) >= hypergraph.edgeSize(e_sub) &&
              hypergraph.edgeWeight(e_super) < hypergraph.edgeWeight(e_sub)
#ifdef HASHING_ACTIVATED
              && hypergraph.edgeSize(e_super) < m.maxHash() &&
              fast_mod(a_m, b_m) == 0
#endif
          ) {
            candidates.push_back(std::make_pair<>(e_super, true));
          }
          if (candidates.size() > max_candidates) {
            break;
          }
        }

        if (candidates.size() == 0) {
          continue;
        }
        VLOG_IF(candidates.size() > 10, 7)
            << "candidates size:" << candidates.size() << std::endl;
        // std::cout << candidates.size() << std::endl;

        for (auto p : hypergraph.pins(e_sub)) {
          if (p != v) {
            for (auto c = candidates.begin(); c != candidates.end(); c++) {
              // the second check is not needed
              if (c->second && (!hypergraph.edgeIsInPin(c->first, p))) {
                c->second = false;
              }
            }
          }
        }
        // unsigned long count = 1;
        // for (auto p : hypergraph.pins(e_sub)) {
        //   if (p != v) {
        //     for (auto incE : hypergraph.incidentEdges(p)) {
        //       nummeration[incE] += count;
        //     }
        //     count *= 2;
        //   }
        // }
        // std::sort(candidates.begin(), candidates.end(),
        //          [](auto a, auto b) { return a.second > b.second; });
        bool success = false;
        for (auto c : candidates) {
          if (c.second) {
            m.markFreeEdgeAsBlocked(c.first);
            // hypergraph.removeEdge_sorted(c.first); // not needed as we are
            // iterating
            success = true;
          }
        }
        if (success) {
          break;
        }
      }
    }
    // alternative slower impl
    /* } else {
       auto last = m.free_edges_monitored(old_ts).end();
       for (auto ep = m.free_edges_monitored(old_ts).begin(); ep < last; ++ep)
    { auto e_super = *ep; std::vector<std::pair<typename BMatching::EdgeType,
    bool>> candidates; typename BMatching::NodeType same_pin;
         // This reduction seems to be important to shrink the search space
    ,esp
         // on 1 coauthor/citation coPapersCiteseer.graph.hgr.random100
         for (auto p : hypergraph.pins(e_super)) {
           if ((m.capacity(p) - m.matchesAtNode(p)) == 1) {
             bool success = false;
             for (auto e_sub : hypergraph.incidentEdges(p)) {
               if (hypergraph.edgeSize(e_sub) > max_size) {
                 continue;
               }

               if (hypergraph.edgeWeight(e_sub) <
                   hypergraph.edgeWeight(e_super)) {
                 break;
               }
    #ifdef HASHING_ACTIVATED
               auto a_m = m.hash(e_super);
               auto b_m = m.hash(e_sub);
    #endif
               if (m.isMatchable(e_sub) && // maybe just &
                   hypergraph.edgeSize(e_super) >= hypergraph.edgeSize(e_sub)
    && hypergraph.edgeWeight(e_super) < hypergraph.edgeWeight(e_sub) #ifdef
    HASHING_ACTIVATED
                   && fast_mod(a_m, b_m) == 0
    #endif
               ) {
                 success = true;
                 for (auto pin : hypergraph.pins(e_sub)) {
                   if (pin != p && !hypergraph.edgeIsInPin(e_super, p)) {
                     success = false;
                     break;
                   }
                 }
                 if (success) {
                   m.markFreeEdgeAsBlocked(e_super);
                   hypergraph.removeEdge_sorted(e_super);
                   break;
                 }
                 //  found pair
               }
             }
             same_pin = p;
             if (success) {
               break;
             }
           }
         }

         last = m.free_edges_monitored(old_ts).end();
       }
     }*/

    old_ts = ts;
    ts = m.change_timestamp();
  } while (multiple_times && ts != old_ts);
}
} // namespace reductions_sorted
} // namespace bmatching
} // namespace HeiHGM::BMatching
