#pragma once

#include "app/algorithms/algorithm_impl.h"
#include "bmatching/greedy/ordered.h"
#include "bmatching/scip/solver.h"

namespace HeiHGM::BMatching::app::algorithms {
class SCIP : public AlgorithmImpl {
public:
  static constexpr absl::string_view AlgorithmName = "scip";

protected:
  absl::Status Execute(StandardIntegerHypergraph &hypergraph,
                       const AlgorithmConfig &config,
                       StandardIntegerHypergraphBMatching &bm,
                       bool &is_exact) override {
    auto double_config = config.double_params();
    HeiHGM::BMatching::bmatching::scip::SCIPBMatchingSolver<
        StandardIntegerHypergraphBMatching>
        solver(bm, hypergraph);
    is_exact = solver.solve(double_config["timeout"]);
    return absl::OkStatus();
  }
  absl::Status ValidateConfig(const AlgorithmConfig &config) override {
    auto double_config = config.double_params();
    if (double_config.find("timeout") == double_config.end()) {
      return absl::NotFoundError(
          "'timeout' need to be specified in double_params.");
    }
    return absl::OkStatus();
  }
};
} // namespace HeiHGM::BMatching::app::algorithms