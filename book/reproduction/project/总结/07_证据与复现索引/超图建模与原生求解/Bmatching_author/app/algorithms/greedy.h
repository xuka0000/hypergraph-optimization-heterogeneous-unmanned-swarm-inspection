#pragma once
#include <functional>

#include "absl/container/flat_hash_set.h"
#include "app/algorithms/algorithm_impl.h"
#include "bmatching/greedy/ordered.h"
namespace HeiHGM::BMatching::app::algorithms {
namespace {
constexpr absl::string_view kRatioStatic = "bratio_static";
constexpr absl::string_view kBWeight = "bweight";
constexpr absl::string_view kMultStatic = "bmult_static";
constexpr absl::string_view kMultDynamic = "bmult_dynamic";
constexpr absl::string_view kBRatioDynamic = "bratio_dynamic";
constexpr absl::string_view kBMindegreeDynamic = "bmindegree_dynamic";
constexpr absl::string_view kBMindegree1Dynamic = "bmindegree1_dynamic";
constexpr absl::string_view kDefaultOrder = "default_order";
constexpr absl::string_view kBMaximize = "bmaximize";

constexpr absl::string_view kScap = "S,cap";
constexpr absl::string_view kSmin = "S,min";
constexpr absl::string_view kSpin = "S,pin";
constexpr absl::string_view kSpinCap = "S,pin,cap";
constexpr absl::string_view kSpinMin = "S,pin,min";
constexpr absl::string_view kSscaled = "S,scaled";
constexpr absl::string_view kDcap = "D,cap";
constexpr absl::string_view kDscaled = "D,scaled";

const absl::flat_hash_set<absl::string_view> kAllowedStringSet = {
    kRatioStatic,   kBWeight,
    kMultStatic,    kBMindegreeDynamic,
    kBRatioDynamic, kBMindegree1Dynamic,
    kDefaultOrder,  kBMaximize,
    kScap,          kSmin,
    kSpin,          kSpinCap,
    kSpinMin,       kSscaled,
    kDcap,          kDscaled,
};
} // namespace
class Greedy : public AlgorithmImpl {
public:
  static constexpr absl::string_view AlgorithmName = "greedy";

protected:
  absl::Status Execute(StandardIntegerHypergraph &hypergraph,
                       const AlgorithmConfig &config,
                       StandardIntegerHypergraphBMatching &bm) override {
    auto string_params = config.string_params();
    auto ordering_method = string_params["ordering_method"];
    if (ordering_method == kRatioStatic || ordering_method == kScap) {
      HeiHGM::BMatching::bmatching::greedy_static_ordered_matching<
          StandardIntegerHypergraphBMatching>(
          &hypergraph, bm, [&](typename StandardIntegerHypergraph::EdgeType e) {
            double res = hypergraph.edgeWeight(e);
            for (auto p : hypergraph.pins(e)) {
              res *= bm.capacity(p);
            }
            return res;
          });
      return absl::OkStatus();
    }
    if (ordering_method == kBWeight) {
      HeiHGM::BMatching::bmatching::greedy_static_ordered_matching<
          StandardIntegerHypergraphBMatching>(
          &hypergraph, bm, [&](typename StandardIntegerHypergraph::EdgeType e) {
            double res = hypergraph.edgeWeight(e);
            return res;
          });
      return absl::OkStatus();
    }
    if (ordering_method == kMultStatic || ordering_method == kSmin) {
      HeiHGM::BMatching::bmatching::greedy_static_ordered_matching<
          StandardIntegerHypergraphBMatching>(
          &hypergraph, bm, [&](typename StandardIntegerHypergraph::EdgeType e) {
            double res = hypergraph.edgeWeight(e);
            unsigned int minCap = std::numeric_limits<unsigned int>::max();
            for (auto p : hypergraph.pins(e)) {
              minCap = std::min(bm.capacity(p), minCap);
            }
            return res * minCap;
          });
      return absl::OkStatus();
    }
    if (ordering_method == kSpin) {
      HeiHGM::BMatching::bmatching::greedy_static_ordered_matching<
          StandardIntegerHypergraphBMatching>(
          &hypergraph, bm, [&](typename StandardIntegerHypergraph::EdgeType e) {
            return ((double)hypergraph.edgeWeight(e)) /
                   ((double)hypergraph.edgeSize(e));
          });
      return absl::OkStatus();
    }
    if (ordering_method == kSpinMin) {
      HeiHGM::BMatching::bmatching::greedy_static_ordered_matching<
          StandardIntegerHypergraphBMatching>(
          &hypergraph, bm, [&](typename StandardIntegerHypergraph::EdgeType e) {
            unsigned int minCap = std::numeric_limits<unsigned int>::max();
            for (auto p : hypergraph.pins(e)) {
              minCap = std::min(bm.capacity(p), minCap);
            }
            return ((double)hypergraph.edgeWeight(e)) /
                   ((double)hypergraph.edgeSize(e)) * ((double)minCap);
          });
      return absl::OkStatus();
    }
    if (ordering_method == kSpinCap) {
      HeiHGM::BMatching::bmatching::greedy_static_ordered_matching<
          StandardIntegerHypergraphBMatching>(
          &hypergraph, bm, [&](typename StandardIntegerHypergraph::EdgeType e) {
            int minCap = 1;
            for (auto p : hypergraph.pins(e)) {
              minCap *= bm.capacity(p);
            }
            return ((double)hypergraph.edgeWeight(e)) /
                   ((double)hypergraph.edgeSize(e)) * ((double)minCap);
          });
      return absl::OkStatus();
    }
    if (ordering_method == kSscaled) {
      HeiHGM::BMatching::bmatching::greedy_static_ordered_matching<
          StandardIntegerHypergraphBMatching>(
          &hypergraph, bm, [&](typename StandardIntegerHypergraph::EdgeType e) {
            double scale = 1.0;
            for (auto p : hypergraph.pins(e)) {
              scale *=
                  ((double)bm.capacity(p)) / ((double)hypergraph.nodeDegree(p));
            }
            return ((double)hypergraph.edgeWeight(e)) /
                   ((double)hypergraph.edgeSize(e)) * scale;
          });
      return absl::OkStatus();
    }
    if (ordering_method == kBRatioDynamic || ordering_method == kDcap) {
      HeiHGM::BMatching::bmatching::greedy_priority_ordered_matching<
          StandardIntegerHypergraphBMatching>(
          &hypergraph, bm,
          [&](typename StandardIntegerHypergraph::EdgeType e,
              StandardIntegerHypergraphBMatching &bm) {
            double res = hypergraph.edgeWeight(e);
            for (auto p : hypergraph.pins(e)) {
              res *= bm.capacity(p) - bm.matchesAtNode(p);
            }
            return res;
          });
      return absl::OkStatus();
    }
    if (ordering_method == kBMindegreeDynamic || ordering_method == kDscaled) {
      HeiHGM::BMatching::bmatching::greedy_priority_ordered_matching<
          StandardIntegerHypergraphBMatching>(
          &hypergraph, bm,
          [&hypergraph](typename StandardIntegerHypergraph::EdgeType e,
                        StandardIntegerHypergraphBMatching &bm) {
            double res = hypergraph.edgeWeight(e);
            double deg = 1.0;
            for (auto p : hypergraph.pins(e)) {
              res *= bm.capacity(p) - bm.matchesAtNode(p);
              deg *= hypergraph.nodeDegree(p);
            }
            return res / deg;
          });
      return absl::OkStatus();
    }
    if (ordering_method == kBMindegree1Dynamic) {
      HeiHGM::BMatching::bmatching::greedy_priority_ordered_matching<
          StandardIntegerHypergraphBMatching>(
          &hypergraph, bm,
          [&](typename StandardIntegerHypergraph::EdgeType e,
              StandardIntegerHypergraphBMatching &bm) {
            double deg = 1.0;
            for (auto p : hypergraph.pins(e)) {
              deg *= hypergraph.nodeDegree(p);
            }
            return 1.0 / deg;
          });
      return absl::OkStatus();
    }
    if (ordering_method == kMultDynamic) {
      HeiHGM::BMatching::bmatching::greedy_priority_ordered_matching<
          StandardIntegerHypergraphBMatching>(
          &hypergraph, bm,
          [&](typename StandardIntegerHypergraph::EdgeType e,
              StandardIntegerHypergraphBMatching &bm) {
            double res = hypergraph.edgeWeight(e);
            int minCap = std::numeric_limits<int>::max();
            for (auto p : hypergraph.pins(e)) {
              minCap = std::min(((int)(bm.capacity(p) - bm.matchesAtNode(p))),
                                minCap);
            }
            return res * minCap;
          });
      return absl::OkStatus();
    }
    if (ordering_method == kDefaultOrder) {
      // bm.maximize(); // is context dependent
      for (const auto &e : hypergraph.edges()) {
        if (bm.isMatchable(e)) {
          bm.addToMatching(e);
        }
      }
      return absl::OkStatus();
    }
    if (ordering_method == kBMaximize) {
      bm.maximize(); // is context dependent
      return absl::OkStatus();
    }
    return absl::NotFoundError("No viable implementation found");
  }
  absl::Status ValidateConfig(const AlgorithmConfig &config) override {
    auto string_params = config.string_params();
    if (string_params.find("ordering_method") == string_params.end()) {
      return absl::NotFoundError("'ordering_method' missing. Please specify "
                                 "how the ordering should be done.");
    }
    std::string ordering_method = string_params["ordering_method"];
    if (kAllowedStringSet.find(ordering_method) == kAllowedStringSet.end()) {
      return absl::NotFoundError("'" + ordering_method +
                                 "' is not a valid ordering_method.");
    }
    return absl::OkStatus();
  }
};
} // namespace HeiHGM::BMatching::app::algorithms