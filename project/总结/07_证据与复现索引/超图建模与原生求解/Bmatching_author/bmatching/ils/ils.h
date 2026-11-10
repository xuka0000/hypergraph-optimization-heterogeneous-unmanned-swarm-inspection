/**
 * (c) 2023, Henrik Reinstädtler
 *
 */

#pragma once
#include "absl/container/flat_hash_map.h"
#include "ds/replay_buffer.h"
#include "utils/random.h"
#include <assert.h>
#include <chrono>
#include <iterator>
#include <set>

namespace HeiHGM::BMatching {
namespace bmatching {
namespace ils {
/**
 * @brief Perturbs a solution according to Andrade et al
 *
 * @tparam Matching
 * @param m
 *
 */
template <class BMatching> BMatching perturb(BMatching &m) {
  int alpha = HeiHGM::BMatching::utils::Randomize::instance().getRandomInt(
      1, 2 * m.size());
  int k = 1;
  if (alpha == 1) {
    int i = 1;
    while (HeiHGM::BMatching::utils::Randomize::instance().flipCoin() ==
           1) { // random distribution
      i++;
    }
    k = i + 1;
  }
  BMatching m2 = m;
  if (k == 1) {
    auto vert = m2.randomNonFreeInexact();
    for (auto p : m2._g->pins(vert)) {
      if (m2.capacity(p) == m2.matchesAtNode(p)) {
        // remove one
        bool removed = false;
        for (auto e : m2._g->incidentEdges(p)) {
          if (m2.isInMatching(e) && !m2.isExact(e)) {
            m2.removeFromMatching(e);
            removed = true;
            break;
          }
        }
        if (!removed) {
          break;
        }
      }
    }
    m2.checkAllPins(vert);
    if (m2.isMatchable(vert)) {
      m2.addToMatching(vert);
    }
  } else {
    // select kappa (=4) vertices
    std::vector<typename BMatching::EdgeID_t> cand(
        {m2.randomNonFreeInexact(), m2.randomNonFreeInexact(),
         m2.randomNonFreeInexact(), m2.randomNonFreeInexact()});
    auto x = cand[0];
    auto age = m2.age(x);
    for (auto x2 : cand) {
      if (age > m2.age(x2)) {
        age = m2.age(x2);
        x = x2;
      }
    }
    std::vector<typename BMatching::EdgeID_t> two_away;
    two_away.reserve(k);
    for (auto p1 : m2._g->pins(x)) {
      for (auto e1 : m2._g->incidentEdges(p1)) {
        for (auto p2 : m2._g->pins(e1)) {
          if (p2 != p1) {
            for (auto e2 : m2._g->incidentEdges(p2)) {
              if (e1 != e2) {
                two_away.push_back(e2);
              }
            }
          }
        }
      }
    }
    two_away.push_back(x);
    HeiHGM::BMatching::utils::Randomize::instance().shuffleVector(
        two_away, two_away.size());
    k = std::min((int)two_away.size(), k);
    for (auto it = two_away.begin(); it != two_away.begin() + k; it++) {
      if (!m2.isInMatching(*it)) {
        for (auto p : m2._g->pins(*it)) {
          if (m2.capacity(p) == m2.matchesAtNode(p)) {
            // remove one
            for (auto e : m2._g->incidentEdges(p)) {
              if (m2.isInMatching(e)) {
                m2.removeFromMatching(e);
                break;
              }
            }
          }
        }
        assert(m2.isMatchable(*it));
        m2.addToMatching(*it);
      }
    }
  }
  m2.maximize();
  return m2;
}
template <class BMatching, class ReplayBuffer>
void perturb_inplace(BMatching &m2, ReplayBuffer &replay) {
  int alpha = HeiHGM::BMatching::utils::Randomize::instance().getRandomInt(
      1, 2 * m2.size());
  int k = 1;
  if (alpha == 1) {
    int i = 1;
    while (HeiHGM::BMatching::utils::Randomize::instance().flipCoin() ==
           1) { // random distribution
      i++;
    }
    k = i + 1;
  }
  if (k == 1) {
    auto vert = m2.randomNonFreeInexact();
    for (auto p : m2._g->pins(vert)) {
      if (m2.capacity(p) == m2.matchesAtNode(p)) {
        // remove one
        bool removed = false;
        for (auto e : m2._g->incidentEdges(p)) {
          if (m2.isInMatching(e) && !m2.isExact(e)) {
            replay.remove(e);
            m2.removeFromMatching(e);
            removed = true;
            break;
          }
        }
        if (!removed) {
          break;
        }
      }
    }
    //  m2.checkAllPins(vert);
    if (m2.isMatchable(vert)) {
      replay.add(vert);
      m2.addToMatching(vert);
    }
  } else {
    // select kappa (=4) vertices
    std::vector<typename BMatching::EdgeID_t> cand(
        {m2.randomNonFreeInexact(), m2.randomNonFreeInexact(),
         m2.randomNonFreeInexact(), m2.randomNonFreeInexact()});
    auto x = cand[0];
    auto age = m2.age(x);
    for (auto x2 : cand) {
      if (age > m2.age(x2)) {
        age = m2.age(x2);
        x = x2;
      }
    }
    std::vector<typename BMatching::EdgeID_t> two_away;
    two_away.reserve(k);
    for (auto p1 : m2._g->pins(x)) {
      for (auto e1 : m2._g->incidentEdges(p1)) {
        for (auto p2 : m2._g->pins(e1)) {
          if (p2 != p1) {
            for (auto e2 : m2._g->incidentEdges(p2)) {
              if (e1 != e2) {
                two_away.push_back(e2);
              }
            }
          }
        }
      }
    }
    two_away.push_back(x);
    HeiHGM::BMatching::utils::Randomize::instance().shuffleVector(
        two_away, two_away.size());
    k = std::min((int)two_away.size(), k);
    for (auto it = two_away.begin(); it != two_away.begin() + k; it++) {
      if (!m2.isInMatching(*it)) {
        for (auto p : m2._g->pins(*it)) {
          if (m2.capacity(p) == m2.matchesAtNode(p)) {
            // remove one
            for (auto e : m2._g->incidentEdges(p)) {
              if (m2.isInMatching(e)) {
                replay.remove(e);
                m2.removeFromMatching(e);
                break;
              }
            }
          }
        }
        assert(m2.isMatchable(*it));
        replay.add(*it);

        m2.addToMatching(*it);
      }
    }
  }
  while (m2.free_edges_size() > 0) {
    auto f = *m2.free_edges().begin();
    replay.add(f);
    m2.addToMatching(f);
  }
}
/**
 * @brief Same as below, but with managed MIS infrastructure
 *
 * @tparam Matching
 * @param m
 * @return true
 * @return false
 */
template <class M> class NonStoringReplayBuffer {
public:
  void add(typename M::EdgeType e) {}
  void remove(typename M::EdgeType e) {}
};

template <class Matching, class ReplayBuffer = NonStoringReplayBuffer<Matching>>
bool one_two_swap_matching_improvement_managed(Matching &m,
                                               ReplayBuffer &replay,
                                               const int val = 1) {
  TIMED_FUNC(ee);
  auto &hypergraph = *m._g;
  // tightness list built up
  // Do local notify on own
  bool found_imp = false;
  const auto check_val = val - 1;
  std::vector<size_t> onetight_list;
  absl::flat_hash_map<size_t, int> adjact;
  for (auto edge : m.solution_inexact()) // TODO(henrik): Only iterate
                                         // over candidates
  {
    if (!m.isInMatching(edge)) {
      break;
    }
    if (m.scanned(edge) >= check_val) {
      continue;
    }
    m.scanned(edge) = val;
    // sorted list of 1-tight n. edges of x
    adjact.clear();
    for (auto p : hypergraph.pins(edge)) {
      // look only on those were capacity is full
      if (m.matchesAtNode(p) == m.capacity(p)) {
        for (auto e : hypergraph.incidentEdges(p)) {
          // this is only a first check

          if (m.scanned(e) < check_val && !m.isInMatching(e) && !m.isExact(e) &&
              m.blockedByOrBlockingPins(e) <= m.blockedByOrBlockingPins(edge))
              [[unlikely]]
          // only those can be matched
          {
            adjact[e]++;
            // automatically we will only insert neighbors that arent in the
            // matching
            // onetight_list2.insert(e);
          }
        }
      }
    }

    // skip where we can not improve directly
    if (adjact.size() < 2) {
      continue;
    }
    onetight_list.clear();
    for (auto [e, cnt] : adjact) {
      m.scanned(e) = val;
      if (m.blockedByOrBlockingPins(e) == cnt) {
        onetight_list.push_back(e);
      }
    }

    if (onetight_list.size() < 2) {
      continue;
    }
    VLOG(7) << adjact.size() << " " << onetight_list.size() << std::endl;
    std::sort(onetight_list.begin(), onetight_list.end(), [&](auto a, auto b) {
      return m._g->edgeWeight(a) > m._g->edgeWeight(b);
    });
    auto x_weight = hypergraph.edgeWeight(edge);
    // attempt to find edges that don't share a pin where capacity is full
    for (auto vi = onetight_list.begin();
         vi < onetight_list.begin() + onetight_list.size() - 1; vi++) {
      auto v = *vi;
      bool success_f = false;
      auto weight_v = hypergraph.edgeWeight(v);
      auto wi = std::next(vi);
      // check if only blocked by one edge
      if (weight_v + hypergraph.edgeWeight(*wi) < x_weight) {
        break;
      }
      for (; wi != onetight_list.end(); wi++) {
        auto w = *wi;
        auto weight_w = hypergraph.edgeWeight(w);

        if (x_weight >= weight_v + weight_w) {
          break;
        }

        bool found = false;
        for (auto p1 : hypergraph.pins(v)) {
          for (auto p2 : hypergraph.pins(w)) {
            if (p1 == p2 && (((int)m.matchesAtNode(p1) + 2) >=
                             (((int)m.capacity(p1))) // no place for one
                             //// seems expensive
                             )) {
              found = true;
              break;
            }
          }
          if (found) {
            break;
          }
        }
        if (found) {
          continue;
        }
        m.NonNotRemoveFromMatching(edge);
        replay.remove(edge);
        replay.add(w);
        replay.add(v);
        m.NonNotaddToMatching(w);
        m.NonNotaddToMatching(v);
        while (m.free_edges_size() > 0) {
          auto f = *m.free_edges().begin();
          replay.add(f);
          m.NonNotaddToMatching(f);
        }
        found_imp = true;
        success_f = true;
        break;
      }
      if (success_f) {
        break;
      }
    }
  }
  return found_imp;
}

template <class BMatching>
BMatching iterated_local_search(BMatching &m, const int max_tries = 15,
                                const double max_time = 1800.0) {
  BMatching m2 = m;
  BMatching best_result = m;
  bool stopping_criterion = false;
  int tries = 0;
  auto best_size = m.weight();
  int round = 0;
  auto t1 = std::chrono::high_resolution_clock::now();
  static NonStoringReplayBuffer<BMatching> replay;

  while (!stopping_criterion) {
    m2 = perturb(m);
    // m.resetScannedEdges();
    round++;
    while (one_two_swap_matching_improvement_managed(m2, replay, round)) {
    }
    if (m2.weight() > m.weight()) {
      VLOG(7) << "Improved " << m.weight() << " " << tries << std::endl;
      m = m2;
    } else {
      if (HeiHGM::BMatching::utils::Randomize::instance().getRandomInt(
              1, 1 + (m.weight() - m2.weight()) * (best_size - m2.weight())) ==
          1) {
        m = m2;
      }
    }
    tries++;
    if (m.weight() > best_size) {
      tries = 0;
      best_size = m.weight();
      best_result = m;
    }
    stopping_criterion =
        tries > max_tries ||
        std::chrono::duration_cast<std::chrono::duration<double>>(
            std::chrono::high_resolution_clock::now() - t1)
                .count() > max_time;
  }

  return best_result;
}
template <class BMatching>
void iterated_local_searc_inplace(BMatching &m, const int max_tries = 15,
                                  const double max_time = 1800) {
  using ReplayBuffer = HeiHGM::BMatching::ds::ReplayBuffer<BMatching>;
  ReplayBuffer replay;
  auto res = m.solution_inexact();
  std::vector<typename BMatching::EdgeType> best_result(res.end() -
                                                        res.begin());

  std::copy(res.begin(), res.end(), best_result.begin());
  bool stopping_criterion = false;
  int tries = 0;
  auto best_size = m.weight();
  auto prev_w = best_size;

  int round = 2;
  m.resetScannedEdges();
  auto t1 = std::chrono::high_resolution_clock::now();
  while (!stopping_criterion) {
    perturb_inplace(m, replay);
    // m.resetScannedEdges();
    round++;
    {
      while (one_two_swap_matching_improvement_managed(m, replay, round)) {
      }
    }
    auto new_w = m.weight();
    if (new_w > prev_w) {
      replay.clear(); // forget changes, accept
      prev_w = m.weight();
    } else {
      if (HeiHGM::BMatching::utils::Randomize::instance().getRandomInt(
              1, 1 + (prev_w - new_w) * (best_size - new_w)) == 1) {
        prev_w = new_w;
        replay.clear(); // forget changes
      } else {
        replay.reverse(m);
        replay.clear();
        // prev_w = m.weight();
      }
    }
    tries++;
    if (new_w > best_size) {
      VLOG(5) << "ACCEPTING " << new_w << " at " << tries << std::endl;
      tries = 0;
      best_size = new_w;
      auto res = m.solution_inexact();
      best_result.resize(res.end() - res.begin());
      std::copy(res.begin(), res.end(), best_result.begin());
      replay.clear(); // accepting new best
    }
    stopping_criterion =
        tries > max_tries ||
        std::chrono::duration_cast<std::chrono::duration<double>>(
            std::chrono::high_resolution_clock::now() - t1)
                .count() > max_time;
  }
  m.reset();
  for (auto b : best_result) {
    m.NonNotaddToMatching(b);
  }
}

} // namespace ils
} // namespace bmatching
} // namespace HeiHGM::BMatching