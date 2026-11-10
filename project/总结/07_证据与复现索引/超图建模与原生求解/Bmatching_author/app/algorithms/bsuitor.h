#pragma once
#include "absl/container/flat_hash_set.h"
#include "app/algorithms/algorithm_impl.h"
#include "bMatching.h"
#include <functional>
#include <tuple>

namespace HeiHGM::BMatching::app::algorithms {
class BSuitor : public AlgorithmImpl {
public:
  static constexpr absl::string_view AlgorithmName = "bsuitor";

protected:
  absl::Status
  Execute(StandardIntegerHypergraph &hypergraph, const AlgorithmConfig &config,
          StandardIntegerHypergraphBMatching &bm, bool &is_exact,
          std::chrono::high_resolution_clock::time_point &t1,
          std::optional<std::chrono::high_resolution_clock::time_point> &t2)
      override {
    is_exact = false;
    CSR G;
    int numThreads, best;

    vector<vector<int>> graphCRSIdx(hypergraph.initialNumNodes());
    vector<vector<double>> graphCRSVal(hypergraph.initialNumNodes());
    int num_edges = 0;
    for (auto e : hypergraph.edges()) {
      graphCRSIdx[*hypergraph.pins(e).begin()].push_back(
          *(hypergraph.pins(e).begin() + 1));
      graphCRSIdx[*(hypergraph.pins(e).begin() + 1)].push_back(
          *hypergraph.pins(e).begin());
      graphCRSVal[*hypergraph.pins(e).begin()].push_back(
          hypergraph.edgeWeight(e));
      graphCRSVal[*(hypergraph.pins(e).begin() + 1)].push_back(
          hypergraph.edgeWeight(e));
      num_edges++;
    }
    G.nVer = hypergraph.initialNumNodes();
    G.verPtr = new int[hypergraph.initialNumNodes() + 1];
    G.verInd = new Edge[2 * num_edges];
    G.verPtr[0] = 0;
    int max = 0, offset;
    int count = 0;
    for (int i = 0; i < G.nVer; i++) {
      offset = graphCRSIdx[i].size();
      G.verPtr[i + 1] = G.verPtr[i] + offset;
      count = G.verPtr[i];
      // cout<<i-1<<" "<<verPtr[i-1]<<" "<<verPtr[i]<<": ";
      for (int j = 0; j < offset; j++) {
        G.verInd[count].id = graphCRSIdx[i][j];
        G.verInd[count].weight = graphCRSVal[i][j];
        count++;
      }
      // cout<<endl;
      if (offset > max)
        max = offset;
    }

    int *b = new int[G.nVer];
    Node *S = new Node[G.nVer]; // Heap data structure
#pragma omp parallel
    numThreads = omp_get_num_threads();

    /************ Assigning bValues **************/
    for (int i = 0; i < G.nVer; i++) {
      int card = 0;
      for (int j = G.verPtr[i]; j < G.verPtr[i + 1]; j++)
        if (G.verInd[j].weight > 0)
          card++;

      if (card > bm.capacity(i)) {
        b[i] = bm.capacity(i);
      } else {
        b[i] = card;
      }
    }
    for (int i = 0; i < G.nVer; i++) {
      if (b[i] > 0) {
        S[i].heap = new Info[b[i]]; // Each heap of size b
      } else {
        S[i].heap = new Info[1]; // Each heap of size b
      }
    }
    auto t1_sys = std::chrono::system_clock::now();
    t1 = std::chrono::high_resolution_clock::now();
    bSuitor(
        &G, b, S, 1,
        false); // TODO(reinstaedtler): Change 1 argument to zero does not work
    t2 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < hypergraph.initialNumNodes(); i++) {
      for (int j = 0; j < S[i].curSize; j++) {
        int a = S[i].heap[j].id;
        if (a < i) {
          for (auto e : hypergraph.incidentEdges(i)) {
            if (*hypergraph.pins(e).begin() == a ||
                *(hypergraph.pins(e).begin() + 1) == a) {
              bm.addToMatching(e);
            }
          }
        }
        count++;
      }
    }
    delete[] b;
    delete[] S;
    return absl::OkStatus();
  }
  absl::Status ValidateConfig(const AlgorithmConfig &config) override {
    return absl::OkStatus();
  }
};
} // namespace HeiHGM::BMatching::app::algorithms