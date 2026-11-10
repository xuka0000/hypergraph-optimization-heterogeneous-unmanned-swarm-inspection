/**
 * @file ordered.h
 * @author Henrik Reinstädtler (reinstaedtler@stud.uni-heidelberg.de)
 * @brief Static and dynamic greedy heuristics for hypergraph b- matching
 * @version 0.1
 *
 * @copyright Copyright (c) 2023 Henrik Reinstädtler
 *
 */

#pragma once
#include "ds/bmatching.h"
#include "ds/indexed_priority_queue.h"
#include <algorithm>

namespace HeiHGM::BMatching {
namespace bmatching {
/**
 * @brief Computes once the sorting values for Bmatching and adds edges
 * accordingly to this order.
 *
 * @tparam BMatching
 * @param graph the hypergraph to be matched
 * @param eval_function should return a double for each edge. High values are
 * added first
 * @return BMatching
 */
template <class BMatching>
void greedy_static_ordered_matching(
    typename BMatching::Graph_t *graph, BMatching &bmatching,
    std::function<double(typename BMatching::EdgeID_t)> eval_function) {
  std::vector<std::pair<typename BMatching::EdgeID_t,double>> order;
  for (auto e : bmatching.free_edges()) {
    order.push_back(std::make_pair(e,eval_function(e)));
  }
  std::sort(order.begin(), order.end(), [&](auto a, auto b) {
    return a.second > b.second;
  });
  for (auto [e,_] : order) {
    if (bmatching.isMatchable(e)) {
      bmatching.addToMatching(e);
    }
  }
}
template <class BMatching>
void greedy_dynamic_ordered_matching(
    typename BMatching::Graph_t *graph, BMatching &bmatching,
    std::function<double(typename BMatching::EdgeID_t, BMatching &bm)>
        eval_function) {
  while (!bmatching.isMaximal()) {
    typename BMatching::EdgeID_t candidate;
    bool found = false;
    for (auto e : graph->edges()) {
      if (bmatching.isMatchable(e)) {
        if (!found) {
          candidate = e;
          found = true;
        } else {
          if (eval_function(e, bmatching) >
              eval_function(candidate, bmatching)) {
            candidate = e;
          }
        }
      }
    }
    bmatching.addToMatching(candidate);
  }
}
template <class BMatching>
void greedy_dynamic2_ordered_matching(
    typename BMatching::Graph_t *graph, BMatching &bmatching,
    std::function<double(typename BMatching::EdgeID_t, BMatching &bm)>
        eval_function) {
  while (!bmatching.isMaximal()) {
    typename BMatching::EdgeID_t candidate;
    bool found = false;
    for (auto e : bmatching.free_edges()) {
      if (!found) {
        candidate = e;
        found = true;
      } else {
        if (eval_function(e, bmatching) > eval_function(candidate, bmatching)) {
          candidate = e;
        }
      }
    }
    bmatching.addToMatching(candidate);
  }
}
template <class BMatching>
void greedy_priority_ordered_matching(
    typename BMatching::Graph_t *graph, BMatching &bmatching,
    std::function<double(typename BMatching::EdgeID_t, BMatching &bm)>
        eval_function) {
  HeiHGM::BMatching::ds::IndexedPriorityQueue<typename BMatching::EdgeID_t, double>
      queue(graph->realEdgeSize(), 0.0);
  std::vector<double> initialWeights(graph->realEdgeSize());
  std::vector<typename BMatching::EdgeID_t> initialPos(graph->realEdgeSize(),
                                                       0);
  std::iota(initialPos.begin(), initialPos.end(), 0);
  queue.set_all(initialPos);
  std::transform(graph->edges().begin(), graph->edges().end(),
                 initialWeights.begin(), [&](auto edge) {
                   if (bmatching.isMatchable(edge))
                     return eval_function(edge, bmatching);
                   else
                     return 0.0;
                 });
  queue.resetValues(initialWeights);
  while (!bmatching.isMaximal()) {
    auto candidate = queue.top_index();
    queue.removeIdx(candidate);
    if (bmatching.isMatchable(candidate)) {

      bmatching.addToMatching(candidate);
      for (auto p : graph->pins(candidate)) {
        for (auto e : graph->incidentEdges(p)) {
          queue.update(e, eval_function(e, bmatching) - queue.value(e));
        }
      }
    }
  }
}
} // namespace bmatching
} // namespace HeiHGM::BMatching
