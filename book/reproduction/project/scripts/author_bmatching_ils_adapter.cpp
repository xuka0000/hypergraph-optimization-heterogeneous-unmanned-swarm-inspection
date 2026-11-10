// I/O adapter for byte-preserved HeiHGM/Bmatching public headers.
// No local-search, perturbation, acceptance or stopping logic is implemented here.
#include <tuple>
#include <limits>
#include <chrono>
#include <functional>
#include "easylogging++.h"
// Easylogging's disabled-logs build still expands TIMED_FUNC using an incomplete
// PerformanceTracker type. Suppress only that diagnostic timer macro.
#undef TIMED_FUNC
#define TIMED_FUNC(obj)
#include "ds/bmatching.h"
#include "bmatching/greedy/ordered.h"
#include "bmatching/ils/ils.h"
INITIALIZE_EASYLOGGINGPP

extern "C" __declspec(dllexport) int solve_ils(
    int nr, int ne, const int* ptr, const int* pins, const int* weights,
    const int* capacities, int seed, int max_tries, int* selected,
    double* times, int* greedy_weight) {
  using namespace HeiHGM::BMatching;
  using G = ds::StandardIntegerHypergraph;
  using M = ds::StandardIntegerHypergraphBMatching;
  auto start = std::chrono::steady_clock::now();
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
  auto built = std::chrono::steady_clock::now();
  // Official CLI default bmindegree_dynamic, copied from app/algorithms/greedy.h.
  bmatching::greedy_priority_ordered_matching<M>(
      &graph, matching,
      [&graph](G::EdgeType e, M &bm) {
        double res = graph.edgeWeight(e);
        double deg = 1.0;
        for (auto p : graph.pins(e)) {
          res *= bm.capacity(p) - bm.matchesAtNode(p);
          deg *= graph.nodeDegree(p);
        }
        return res / deg;
      });
  *greedy_weight = matching.weight();
  auto greedy_done = std::chrono::steady_clock::now();
  if (max_tries >= 0 && matching.weight() > 0) {
    // Public optional max_time argument explicitly disabled, as requested.
    matching = bmatching::ils::iterated_local_search(
        matching, max_tries, std::numeric_limits<double>::infinity());
  }
  auto done = std::chrono::steady_clock::now();
  int count = 0;
  for (auto e : matching.solution_inexact()) selected[count++] = static_cast<int>(e);
  times[0] = std::chrono::duration<double>(built-start).count();
  times[1] = std::chrono::duration<double>(greedy_done-built).count();
  times[2] = std::chrono::duration<double>(done-greedy_done).count();
  return count;
}
