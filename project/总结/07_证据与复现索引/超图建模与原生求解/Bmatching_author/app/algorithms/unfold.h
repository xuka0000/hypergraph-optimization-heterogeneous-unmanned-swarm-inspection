#pragma once
#include <functional>

#include "absl/container/flat_hash_set.h"
#include "app/algorithms/algorithm_impl.h"
#include "bmatching/reductions_sorted/driver.h"

namespace HeiHGM::BMatching::app::algorithms {
class Unfold : public AlgorithmImpl {
public:
  static constexpr absl::string_view AlgorithmName = "unfold";

protected:
  absl::Status Execute(StandardIntegerHypergraph &hypergraph,
                       const AlgorithmConfig &config,
                       StandardIntegerHypergraphBMatching &bm,
                       bool &is_exact) override {
    is_exact = true;
    auto str_params = config.string_params();
    auto int64_params = config.int64_params();
    HeiHGM::BMatching::bmatching::reductions_sorted::weighted_vertex_unfolding(
        hypergraph, bm);

    return absl::OkStatus();
  }
  absl::Status ValidateConfig(const AlgorithmConfig &config) override {
    return absl::OkStatus();
  }
};
} // namespace HeiHGM::BMatching::app::algorithms