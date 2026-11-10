#pragma once
#include <functional>

#include "absl/container/flat_hash_set.h"
#include "app/algorithms/algorithm_impl.h"
#include "bmatching/reductions/driver.h"
#include "bmatching/reductions_sorted/driver.h"
#include "ds/hashing-config.h"
#ifdef __cplusplus
extern "C" {
#endif
#include "ksmd/ksmd_utils.h"
#include "kss/kss_utils.h"
#ifdef __cplusplus
}
#endif
namespace HeiHGM::BMatching::app::algorithms {

namespace {
// only support
// three lined input `npp m k`

void read_file_ks(int *n, int *m, int *k, StandardIntegerHypergraph &instance,
                  int **v_hids, int **v_starts, int **h_begs, int **h_points,
                  int *min_ni) {
  int t_n, t_m, i, j, t_k, x, q, t_npk = 0;
  int *degs;
  int kv;
  int *dim_starts;
  int mv;
  // file is in format   n m k       n:  number of all nodes (on any size) , m
  // the number of hyperedges, and k the number of partitions then m lines
  // follow each in the format a_1 a_2 .... a_k  such that  h_i = (a_1 .... a_k)
  // is the i-th hyperedge with i<m Nodes are 0-based use similar to SPARTAN

  t_k = instance.edgeSize(0);
  t_m = instance.initialNumEdges();

  t_npk = instance.initialNumNodes() / t_k;
  if (instance.initialNumNodes() % t_k != 0) {
    std::cout << "ALLARM" << std::endl;
    exit(1);
  }

  dim_starts = (int *)calloc(
      t_k + 1, sizeof(int)); // first ID of a vertex in i-th dimension
  t_n = 0;
  mv = -1;
  dim_starts[0] = 0;

  // only convert same sized
  for (i = 0; i < t_k; ++i)
    dim_starts[i] = i * t_npk;
  t_n = t_npk * t_k;
  mv = t_npk;

  *min_ni = mv;
  *v_hids = (int *)malloc(
      (t_k * t_m) *
      sizeof(int *)); // big list containing the hyperedge IDs of each node
  *v_starts = (int *)malloc(
      (t_n + 1) * sizeof(int *)); // pointer structure that says where the IDs
                                  // of the i-th node begin in the above list
  *h_points = (int *)malloc(t_m * t_k * sizeof(int *)); // end points for all
                                                        // hyperedges
  *h_begs = (int *)malloc(
      (t_m + 1) *
      sizeof(int *)); // pointer structure that says where the endpoints
                      // of each edge begin in the above list
  degs = (int *)calloc(t_n,
                       sizeof(int)); // degree-array for initialization purposes
  for (i = 0; i < t_n; ++i)
    degs[i] = 0;
  q = 0;
  for (i = 0; i < t_m; ++i) { // first fill the hyperedge array information and
                              // find degree of each node
    (*h_begs)[i] = q;         // the i-th edge begins where the (i-1) ended +1
    for (auto kv : instance.pins(i)) { // read all endpoints

      degs[kv]++;
      (*h_points)[q++] = kv; // store them
    }
  }
  (*h_begs)[t_m] = q;          // last-non existing edge
  (*v_starts)[0] = 0;          // now consider the v_starts
  for (i = 1; i <= t_n; ++i) { // fore ach node
    (*v_starts)[i] = (*v_starts)[i - 1] +
                     degs[i - 1]; // i-th node begins where the (i-1) node
                                  // begins + the number of edges (i-1) has
    degs[i - 1] =
        (*v_starts)[i - 1]; // set it at the beginning for the next step (just
                            // to save some space, we reuse degs array)
  }
  for (i = 0; i < t_m; ++i) { // re-read each edge
    for (j = (*h_begs)[i]; j < (*h_begs)[i + 1]; ++j) {
      x = (*h_points)[j]; // store the hyperedge id to each endpoint
      (*v_hids)[degs[x]] = i;
      degs[x]++; // increment degs[x] so that the next edge containing x will be
                 // saved in the correct position
    }
  }
  *n = t_n;
  *m = t_m;
  *k = t_k;
  free(degs);
}
// This function reads an (n x .. x n) K-D tensor from a.txt
// The first two numbers are n and number of observations/nonzeros inside
void read_tensor_kss(StandardIntegerHypergraph &instance, int *n, int *m,
                     int *k, int **edge_list, int **edge_order, int **degs,
                     int **dim_starts, int **dim_ends) {
  int t_npk = 0, t_m, t_k, kv, i, j, t_n, temp, cx;

  t_k = instance.edgeSize(0);
  t_m = instance.initialNumEdges();

  t_npk = instance.initialNumNodes() / t_k;
  *dim_starts =
      (int *)calloc(t_k, sizeof(int)); // first ID of a vertex in i-th dimension
  *dim_ends =
      (int *)calloc(t_k, sizeof(int)); // first ID of a vertex in i-th dimension
  *edge_order = (int *)malloc(t_m * sizeof(int));
  t_n = instance.initialNumNodes();
  *edge_list = (int *)malloc(t_m * t_k *
                             sizeof(int)); // the tensor will be stored in
                                           // coordinate format ( m  x k list)
  (*dim_starts)[0] = 0; // first dimensions start at zero of array order
  temp = t_npk;
  t_n += temp;
  (*dim_ends)[0] = temp - 1; // where it inits
  for (i = 1; i < t_k; ++i) {
    temp = t_npk;
    t_n += temp;
    (*dim_starts)[i] =
        (*dim_ends)[i - 1] + 1; // and any other dimension starts immediatelly
                                // from where the previous finish
    (*dim_ends)[i] = (*dim_starts)[i] + temp - 1;
  }

  *degs = (int *)calloc(t_n, sizeof(int));
  for (i = 0; i < t_m; i++) {
    (*edge_order)[i] = i;
  }
  for (i = 0; i < t_m; ++i) { // first fill the hyperedge array information and
    // find degree of each node
    int j = 0;
    for (auto kv : instance.pins(i)) { // read all endpoints
      cx = kv;
      (*edge_list)[i * t_k + j++] = cx;
      (*degs)[cx]++;
    }
  }
  *n = t_n;
  *m = t_m;
  *k = t_k;
}
} // namespace
class KSS : public AlgorithmImpl {
public:
  static constexpr absl::string_view AlgorithmName = "karp_sipser";

protected:
  absl::Status Execute(StandardIntegerHypergraph &hypergraph,
                       const AlgorithmConfig &config,
                       StandardIntegerHypergraphBMatching &bm,
                       bool &is_exact) override {
    is_exact = false;
    auto str_params = config.string_params();
    auto int64_params = config.int64_params();

    if (str_params["method"] == "ksmd") {
      int n, m, i, k, min_ni, correct_match = 1;
      int ans;
      int *v_hids, *v_starts, *h_begs, *h_points;
      int *edge_list;
      int *match_set;
      int *match;
      int o = 0; // TODO(reinstaedtler): Make configurable

      read_file_ks(&n, &m, &k, hypergraph, &v_hids, &v_starts, &h_begs,
                   &h_points, &min_ni);
      // convert to data structure used by ksmd
      match_set = (int *)malloc(min_ni * sizeof(int));
      match = (int *)malloc(n * sizeof(int));

      edge_list = (int *)malloc(m * sizeof(int));
      for (i = 0; i < m; ++i)
        edge_list[i] = i;

      ans = ksmd(edge_list, n, m, v_hids, v_starts, h_begs, h_points, &match,
                 &match_set);
      for (int i = 0; i < ans; i++) {
        auto e = match_set[i];
        bm.addToMatching(e);
      }
      free(v_hids);
      free(v_starts);
      free(h_begs);
      free(h_points);
      free(edge_list);
      return absl::OkStatus();
    }
    if (str_params["method"] == "kss") {
      int *match;
      int sc = int64_params["scaling_iterations"];
      HeiHGM::BMatching::app::app_io::Result result;
      int m, n, k, i, correct_match = 1;
      int *edge_list;
      int *degs;
      int *match_set;
      int *edge_order;
      int ans;
      int *dim_starts;
      int *dim_ends;
      int min_ni = -1;
      read_tensor_kss(hypergraph, &n, &m, &k, &edge_list, &edge_order, &degs,
                      &dim_starts, &dim_ends);
      match = (int *)malloc(n * sizeof(int));
      min_ni = dim_ends[0] - dim_starts[0] + 1;
      for (i = 1; i < k; ++i) {
        if (min_ni > dim_ends[i] - dim_starts[i] + 1)
          min_ni = dim_ends[i] - dim_starts[i] + 1;
      }
      match_set = (int *)malloc(min_ni * sizeof(int));
      auto t1_sys = std::chrono::system_clock::now();
      std::chrono::high_resolution_clock::time_point t1 =
          std::chrono::high_resolution_clock::now();
      ans = pm_drive(edge_list, edge_order, n, m, k, &match, degs, dim_starts,
                     dim_ends, sc, &match_set);
      for (int i = 0; i < ans; i++) {
        auto e = match_set[i];
        bm.addToMatching(e);
      }
      free(degs);
      free(match_set);
      free(edge_list);
      free(edge_order);
      free(dim_starts);
      free(dim_ends);
      free(match);
      return absl::OkStatus();
    }
    return absl::NotFoundError("Implementation for this method not found.");
  }
  absl::Status ValidateConfig(const AlgorithmConfig &config) override {
    auto str_params = config.string_params();
    auto int64_params = config.int64_params();
    if (str_params["method"] != "ksmd" && str_params["method"] != "kss") {
      return absl::NotFoundError("only ksmd and kss are supported.");
    }
    if (str_params["method"] == "kss" &&
        int64_params["scaling_iterations"] == 0) {
      return absl::OutOfRangeError(
          "Please specify scaling_iterations > 0 for using this method");
    }
    // disable_hint is optional
    return absl::OkStatus();
  }
};
} // namespace HeiHGM::BMatching::app::algorithms