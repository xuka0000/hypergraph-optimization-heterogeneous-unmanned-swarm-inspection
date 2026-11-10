// Array I/O only: unchanged author reductions -> S,pin -> ILS -> unfold.
// Source: HeiHGM/Bmatching 2f456c32c1d28de28b02822d48e383d769e667ea.
#include <tuple>
#include <limits>
#include <chrono>
#include <functional>
#include "easylogging++.h"
// Disable diagnostic timers only (disabled logging has no PerformanceTracker).
#undef TIMED_FUNC
#define TIMED_FUNC(obj)
#undef TIMED_SCOPE
#define TIMED_SCOPE(obj, name)
#include "ds/bmatching.h"
#include "bmatching/reductions_sorted/driver.h"
#include "bmatching/greedy/ordered.h"
#include "bmatching/ils/ils.h"
INITIALIZE_EASYLOGGINGPP

extern "C" __declspec(dllexport) int solve_pipeline(
    int nr, int ne, const int* ptr, const int* pins, const int* weights,
    const int* capacities, int seed, int max_tries, int* selected,
    double* times, int* stats) {
  using namespace HeiHGM::BMatching;
  using G = ds::StandardIntegerHypergraph;
  using M = ds::StandardIntegerHypergraphBMatching;
  using Clock = std::chrono::steady_clock;
  auto start = Clock::now();
  utils::Randomize::instance().setSeed(seed);
  G graph(ne, nr, true, true);
  for (int e = 0; e < ne; ++e) {
    std::vector<G::NodeType> nodes;
    for (int k = ptr[e]; k < ptr[e+1]; ++k) nodes.push_back(pins[k]);
    graph.addEdge(nodes, weights[e]);
    if (!std::is_sorted(nodes.begin(), nodes.end())) graph.resortEdge(e);
  }
  for (int v = 0; v < nr; ++v) graph.setNodeWeight(v, capacities[v]);
  M matching(&graph);
  auto built = Clock::now();
  bmatching::reductions_sorted::all_removals_exhaustive(matching, graph, 0, 10);
  matching.set_exact();
  stats[0] = static_cast<int>(matching.size());
  stats[1] = static_cast<int>(matching.merge_history().size());
  auto reduced = Clock::now();
  bmatching::greedy_static_ordered_matching<M>(&graph, matching,
      [&graph](G::EdgeType e) {
        return ((double)graph.edgeWeight(e)) / ((double)graph.edgeSize(e));
      });
  stats[2] = matching.weight(); // Reduced objective; not original objective.
  auto greedy = Clock::now();
  // The public perturbation routine requires a nonempty residual candidate set.
  // A fully reduced/selected instance has no move; bypass the invalid empty draw.
  auto remaining = matching.non_free_edges_inexact();
  stats[5] = static_cast<int>(remaining.end() - remaining.begin());
  if (max_tries >= 0 && matching.weight() > 0 && stats[5] > 0)
    matching = bmatching::ils::iterated_local_search(
        matching, max_tries, std::numeric_limits<double>::infinity());
  stats[3] = matching.weight();
  auto improved = Clock::now();
  bmatching::reductions_sorted::weighted_vertex_unfolding(graph, matching);
  int count = 0;
  long long original_objective = 0;
  for (auto e : matching.solution()) {
    if (e >= static_cast<G::EdgeType>(ne) || count >= ne) return -1;
    selected[count++] = static_cast<int>(e);
    original_objective += weights[e];
  }
  stats[4] = static_cast<int>(original_objective);
  auto done = Clock::now();
  times[0] = std::chrono::duration<double>(built-start).count();
  times[1] = std::chrono::duration<double>(reduced-built).count();
  times[2] = std::chrono::duration<double>(greedy-reduced).count();
  times[3] = std::chrono::duration<double>(improved-greedy).count();
  times[4] = std::chrono::duration<double>(done-improved).count();
  return count;
}
