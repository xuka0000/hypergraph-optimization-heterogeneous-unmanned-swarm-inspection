#pragma once

#include "app/algorithms/algorithm_impl.h"
#include "bmatching/greedy/ordered.h"
#include "bmatching/ilp/ilp_ball.h"
#include "bmatching/ilp/ilp_exact.h"

namespace HeiHGM::BMatching::app::algorithms {
class PresolvedIlp : public AlgorithmImpl {
public:
  static constexpr absl::string_view AlgorithmName = "presolved_ilp";

protected:
  absl::Status Execute(StandardIntegerHypergraph &hypergraph,
                       const AlgorithmConfig &config,
                       StandardIntegerHypergraphBMatching &bm,
                       bool &is_exact) override {
    auto double_config = config.double_params();
    HeiHGM::BMatching::bmatching::ilp::computePresolvedIlp(&hypergraph, bm, is_exact,
                                                      double_config["timeout"]);
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