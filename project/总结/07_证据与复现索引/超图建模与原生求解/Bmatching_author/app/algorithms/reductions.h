#pragma once
#include <functional>

#include "absl/container/flat_hash_set.h"
#include "app/algorithms/algorithm_impl.h"
#include "bmatching/reductions_sorted/driver.h"
#include "ds/hashing-config.h"
namespace HeiHGM::BMatching::app::algorithms {
class Reductions : public AlgorithmImpl {
public:
  static constexpr absl::string_view AlgorithmName = "reductions";

protected:
  absl::Status Execute(StandardIntegerHypergraph &hypergraph,
                       const AlgorithmConfig &config,
                       StandardIntegerHypergraphBMatching &bm,
                       bool &is_exact) override {
    is_exact = true;
    auto str_params = config.string_params();
    auto int64_params = config.int64_params();
#ifdef HASHING_ACTIVATED
    bm.computeInitialHashes();
#endif
    if (str_params["configurable"] == "true") {
      HeiHGM::BMatching::bmatching::reductions_sorted::
          all_removals_exhaustive_configurable(
              bm, hypergraph, int64_params["max_size_nr"],
              int64_params["clique_size_isolated_edge"],
              int64_params["max_edge_size_isolated_edge"],
              int64_params["max_size_weighted_domination"],
              int64_params["max_size_twin_folding"], 0,
              int64_params["max_runs"]);
    } else {
      int reps = std::max((int)int64_params["reps"], 1);
      for (int i = 0; i < reps; i++) {
        HeiHGM::BMatching::bmatching::reductions_sorted::
            all_removals_exhaustive(bm, hypergraph, 0,
                                    int64_params.find("max_runs") ==
                                            int64_params.end()
                                        ? 10
                                        : int64_params["max_runs"]);
      }
    }

    if (str_params["disable_hint"] != "true") {
      bm.set_exact();
    }
    return absl::OkStatus();
  }
  absl::Status ValidateConfig(const AlgorithmConfig &config) override {
    auto str_params = config.string_params();
    if (str_params["assume_sorted"] != "true") {
      return absl::NotFoundError(
          "assume_sorted=true is mandatory with this build");
    }
    // disable_hint is optional
    return absl::OkStatus();
  }
};
} // namespace HeiHGM::BMatching::app::algorithms