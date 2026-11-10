#pragma once

#include "app/algorithms/algorithm_impl.h"
#include "bmatching/greedy/ordered.h"
#include "bmatching/ilp/ilp_ball.h"
#include "bmatching/ilp/ilp_exact.h"
#include "bmatching/scip/solver.h"

namespace HeiHGM::BMatching::app::algorithms {
class ILPAlgorithm : public AlgorithmImpl {
public:
  static constexpr absl::string_view AlgorithmName = "local_improvement";

protected:
  absl::Status Execute(StandardIntegerHypergraph &hypergraph,
                       const AlgorithmConfig &config,
                       StandardIntegerHypergraphBMatching &bm) override {
    auto int_config = config.int64_params();
    auto double_config = config.double_params();
    auto str_config = config.string_params();

    if (str_config["backend"] == "gurobi") {
      HeiHGM::BMatching::bmatching::ilp::iterated_local_search_ilp<
          StandardIntegerHypergraphBMatching>(bm, int_config["iters"],
                                              int_config["distance"],
                                              double_config["timeout"]);
    } else if (str_config["backend"] == "scip") {
      HeiHGM::BMatching::bmatching::scip::iterated_local_search_ilp<
          StandardIntegerHypergraphBMatching>(
          bm, int_config["iters"], int_config["distance"],
          double_config["timeout"], int_config["max_tries"]);
    }
    return absl::OkStatus();
  }
  absl::Status ValidateConfig(const AlgorithmConfig &config) override {
    auto str_config = config.string_params();
    auto int_config = config.int64_params();
    auto double_config = config.double_params();
    if (str_config["backend"] != "gurobi" && str_config["backend"] != "scip") {
      return absl::UnimplementedError(
          "'backend' needs to be either 'gurobi' or 'scip'");
    }
    if (int_config.find("iters") == int_config.end()) {
      return absl::NotFoundError("'iters' need to be specified.");
    }
    if (int_config.find("max_tries") == int_config.end()) {
      return absl::NotFoundError("'max_tries' need to be specified.");
    }
    if (int_config.find("distance") == int_config.end()) {
      return absl::NotFoundError("'distance' need to be specified.");
    }
    if (double_config.find("timeout") == double_config.end()) {
      return absl::NotFoundError("'timeout' need to be specified.");
    }
    return absl::OkStatus();
  }
};
} // namespace HeiHGM::BMatching::app::algorithms