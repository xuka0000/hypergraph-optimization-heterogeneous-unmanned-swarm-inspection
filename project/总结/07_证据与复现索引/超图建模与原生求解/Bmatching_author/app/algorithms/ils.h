#pragma once
#include <functional>

#include "absl/container/flat_hash_set.h"
#include "app/algorithms/algorithm_impl.h"
#include "bmatching/ils/ils.h"
namespace HeiHGM::BMatching::app::algorithms {
class ILS : public AlgorithmImpl {
public:
  static constexpr absl::string_view AlgorithmName = "ils";

protected:
  absl::Status Execute(StandardIntegerHypergraph &hypergraph,
                       const AlgorithmConfig &config,
                       StandardIntegerHypergraphBMatching &bm) override {
    if (bm.weight() == 0) {
      return absl::UnimplementedError(
          "ILS algorithms require a apriori solution.");
    }
    auto int_params = config.int64_params();
    auto str_params = config.string_params();
    if (str_params["inplace"] == "true") {
      HeiHGM::BMatching::bmatching::ils::iterated_local_searc_inplace(
          bm, int_params["max_tries"]);
    } else {
      bm = HeiHGM::BMatching::bmatching::ils::iterated_local_search(
          bm, int_params["max_tries"]);
    }
    return absl::OkStatus();
  }
  absl::Status ValidateConfig(const AlgorithmConfig &config) override {
    auto int_params = config.int64_params();
    if (int_params.find("max_tries") == int_params.end()) {
      return absl::NotFoundError("'max_tries' missing. Please specify "
                                 "how long the algorithm should run.");
    }
    return absl::OkStatus();
  }
};
} // namespace HeiHGM::BMatching::app::algorithms