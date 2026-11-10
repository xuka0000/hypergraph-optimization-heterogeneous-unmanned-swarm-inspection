#pragma once
#include "absl/status/statusor.h"
#include "app/app_io.pb.h"
#include "ds/bmatching.h"
#include "ds/modifiable_hypergraph.h"
#include <functional>
#include <google/protobuf/util/time_util.h>
#include <map>
#include <optional>

namespace HeiHGM::BMatching::app::algorithms {
namespace {
using HeiHGM::BMatching::app::app_io::AlgorithmConfig;
using HeiHGM::BMatching::app::app_io::AlgorithmRunInformation;
using HeiHGM::BMatching::app::app_io::Hypergraph;
using HeiHGM::BMatching::app::app_io::Result;
using HeiHGM::BMatching::app::app_io::RunConfig;
using HeiHGM::BMatching::ds::StandardIntegerHypergraph;
using HeiHGM::BMatching::ds::StandardIntegerHypergraphBMatching;
} // namespace
/**
 * @brief Algorithms to be run need to provide a AlgorithmName constexpr and
 * override execute. registerImpl to be added to the factory.
 *
 *
 */
class AlgorithmImpl {
protected:
  virtual absl::Status Execute(StandardIntegerHypergraph &hypergraph,
                               const AlgorithmConfig &config,
                               StandardIntegerHypergraphBMatching &bm) {
    return absl::UnimplementedError(
        "Something went wrong and you called an unimplemented method.");
  };
  virtual absl::Status Execute(StandardIntegerHypergraph &hypergraph,
                               const AlgorithmConfig &config,
                               StandardIntegerHypergraphBMatching &bm,
                               bool &is_exact) {
    is_exact = false;
    return Execute(hypergraph, config, bm);
  }
  virtual absl::Status
  Execute(StandardIntegerHypergraph &hypergraph, const AlgorithmConfig &config,
          StandardIntegerHypergraphBMatching &bm, bool &is_exact,
          std::chrono::high_resolution_clock::time_point &t1,
          std::optional<std::chrono::high_resolution_clock::time_point> &t2) {
    return Execute(hypergraph, config, bm, is_exact);
  }

public:
  virtual ~AlgorithmImpl() {}
  absl::StatusOr<AlgorithmRunInformation>
  Run(StandardIntegerHypergraph &hypergraph, const AlgorithmConfig &config,
      StandardIntegerHypergraphBMatching &bmatching) {
    auto t1_sys = std::chrono::system_clock::now();
    std::chrono::high_resolution_clock::time_point t1 =
        std::chrono::high_resolution_clock::now();
    std::optional<std::chrono::high_resolution_clock::time_point> t2_opt =
        std::nullopt;
    bool is_exact = false;
    absl::Status status =
        Execute(hypergraph, config, bmatching, is_exact, t1, t2_opt);
    if (!status.ok()) {
      return status;
    }
    std::chrono::high_resolution_clock::time_point t2 =
        t2_opt.value_or(std::chrono::high_resolution_clock::now());
    auto t2_sys = std::chrono::system_clock::now();
    AlgorithmRunInformation result;
    result.set_is_exact(is_exact);
    result.set_weight(bmatching.weight());
    result.set_size(bmatching.size());
    result.set_edge_count(hypergraph.currentNumEdges());
    result.set_node_count(hypergraph.currentNumNodes());
    result.set_free_edges(bmatching.free_edges_size());
    if (!bmatching.bMatching()) {
      return absl::InternalError("Not a valid bmatching");
    }
    *result.mutable_start_time() =
        google::protobuf::util::TimeUtil::TimeTToTimestamp(
            std::chrono::system_clock::to_time_t(t1_sys));
    *result.mutable_end_time() =
        google::protobuf::util::TimeUtil::TimeTToTimestamp(
            std::chrono::system_clock::to_time_t(t2_sys));
    auto dur = t2 - t1;
    *result.mutable_algo_duration() =
        google::protobuf::util::TimeUtil::NanosecondsToDuration(
            (int64_t)dur.count());

    *result.mutable_algorithm_config() = config;
    return result;
  }
  virtual absl::Status ValidateConfig(const AlgorithmConfig &config) {
    return absl::NotFoundError("Config should be validateable.");
  };
};
class AlgorithmImplFactory {
public:
  AlgorithmImplFactory(const AlgorithmImplFactory &) = delete;
  AlgorithmImplFactory(AlgorithmImplFactory &&) = delete;
  AlgorithmImplFactory &operator=(const AlgorithmImplFactory &) = delete;
  AlgorithmImplFactory &operator=(AlgorithmImplFactory &&) = delete;

  static AlgorithmImplFactory &getInstance() {
    static AlgorithmImplFactory instance;
    return instance;
  }
  std::function<std::unique_ptr<AlgorithmImpl>()>
  operator[](absl::string_view name) {
    return _map[name];
  }
  bool has(absl::string_view name) { return _map.find(name) != _map.end(); }

private:
  AlgorithmImplFactory() {}
  ~AlgorithmImplFactory() = default;
  std::map<absl::string_view, std::function<std::unique_ptr<AlgorithmImpl>()>>
      _map;
  bool addAlgorithmImpl(absl::string_view name,
                        std::function<std::unique_ptr<AlgorithmImpl>()> func) {
    if (_map.find(name) != _map.end()) {
      return false;
    }
    _map[name] = func;
    return true;
  }
  template <class Impl> friend bool registerImpl();
};
template <class Impl> bool registerImpl() {
  auto &instance = AlgorithmImplFactory::getInstance();
  std::function<
      std::unique_ptr<HeiHGM::BMatching::app::algorithms::AlgorithmImpl>()>
      fac = []()
      -> std::unique_ptr<HeiHGM::BMatching::app::algorithms::AlgorithmImpl> {
    return std::make_unique<Impl>();
  };
  return instance.addAlgorithmImpl(Impl::AlgorithmName, fac);
}
absl::StatusOr<Result> Run(const Hypergraph &hypergraph,
                           const RunConfig &config, bool validate);
} // namespace HeiHGM::BMatching::app::algorithms
#define REGISTER_IMPL(I)                                                       \
  static bool impl_##I = HeiHGM::BMatching::app::algorithms::registerImpl<I>();
