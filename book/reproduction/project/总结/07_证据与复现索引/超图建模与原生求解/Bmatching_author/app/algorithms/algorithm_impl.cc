#include "app/algorithms/algorithm_impl.h"
#include "app/app_io.pb.h"
#include "build-info.h"
#include "ds/hashing-config.h"
#include "io/graph_reader.h"
#include "io/hgr_reader.h"
#include "utils/systeminfo.h"

#include <google/protobuf/util/time_util.h>

#define VAL(str) #str
#define TOSTRING(str) VAL(str)
#define HOSTNAME_STRING TOSTRING(BUILD_HOST)
#define USERNAME TOSTRING(BUILD_USER)

namespace HeiHGM::BMatching::app::algorithms {
namespace {
using HeiHGM::BMatching::app::app_io::AlgorithmConfig;
using HeiHGM::BMatching::app::app_io::AlgorithmRunInformation;
using HeiHGM::BMatching::app::app_io::Hypergraph;
using HeiHGM::BMatching::app::app_io::Result;
using HeiHGM::BMatching::app::app_io::RunConfig;
using HeiHGM::BMatching::ds::StandardIntegerHypergraph;
using HeiHGM::BMatching::ds::StandardIntegerHypergraphBMatching;
template <class H>
absl::StatusOr<std::unique_ptr<H>> loadFile(const Hypergraph &hypergraph_conf) {
  std::ifstream hgr_file(hypergraph_conf.file_path());
  if (!hgr_file.is_open()) {
    return absl::NotFoundError("Opening file '" + hypergraph_conf.file_path() +
                               "' failed.");
  }
  if (hypergraph_conf.format() == "hgr") {
    return HeiHGM::BMatching::io::readHypergraph<StandardIntegerHypergraph>(
        hgr_file);
  } else if (hypergraph_conf.format() == "graph") {

    return HeiHGM::BMatching::io::readGraph<StandardIntegerHypergraph>(hgr_file);
  } else {
    return absl::NotFoundError("Not registered file format");
  }
}
} // namespace
absl::StatusOr<Result> Run(const Hypergraph &hypergraph_conf,
                           const RunConfig &config, bool validate) {

  auto hypergraph_status = loadFile<StandardIntegerHypergraph>(hypergraph_conf);

  if (!hypergraph_status.ok()) {
    return hypergraph_status.status();
  }
  auto hypergraph = std::move(hypergraph_status.value());
  // Rewrite weights, if needed
  if (hypergraph_conf.node_weight_type() == "unweighted") {
    for (auto n : hypergraph->nodes()) {
      hypergraph->setNodeWeight(n, 1);
    }
  }
  if (hypergraph_conf.edge_weight_type() == "pins") {
    for (auto e : hypergraph->edges()) {
      hypergraph->setEdgeWeight(e, hypergraph->edgeSize(e));
    }
  }
  if (hypergraph_conf.edge_weight_type() == "uniform") {
    for (auto e : hypergraph->edges()) {
      hypergraph->setEdgeWeight(e, 1);
    }
  }
  // setup capacity
  int standard_capacity = config.capacity();
  if (standard_capacity == 0) {
    return absl::InvalidArgumentError("Please specify a capacity");
  }
  if (standard_capacity != -1) {
    for (auto n : hypergraph->nodes()) {
      hypergraph->setNodeWeight(n, standard_capacity);
    }
  }
  auto &lookup =
      HeiHGM::BMatching::app::algorithms::AlgorithmImplFactory::getInstance();

  // Validate all given configs
  for (const auto &conf : config.algorithm_configs()) {
    if (!lookup.has(conf.algorithm_name())) {
      return absl::NotFoundError("Please specify a valid algorithm name " +
                                 conf.algorithm_name());
    }
  }
  StandardIntegerHypergraphBMatching bmatching((hypergraph.get()));
  Result result;
  // Execute configs & measure time
  auto t1_sys = std::chrono::system_clock::now();
  bool is_exact = true;
  int64_t total_nanos = 0;
  for (const auto &conf : config.algorithm_configs()) {
    std::unique_ptr<HeiHGM::BMatching::app::algorithms::AlgorithmImpl> impl =
        lookup[conf.algorithm_name()]();
    if (validate) {
      auto status = impl->ValidateConfig(conf);

      if (!status.ok()) {
        return status;
      }
    }
    auto res_status = impl->Run(*(hypergraph), conf, bmatching);
    if (!res_status.ok()) {
      std::cerr << conf.algorithm_name() << std::endl;
      return res_status.status();
    }
    auto res = res_status.value();
    is_exact &= res.is_exact();
    total_nanos += google::protobuf::util::TimeUtil::DurationToNanoseconds(
        res.algo_duration());
    result.mutable_algorithm_run_informations()->Add(std::move(res));
  }
  auto t2_sys = std::chrono::system_clock::now();
  result.set_is_exact(is_exact);
  result.set_weight(bmatching.weight());
  result.set_size(bmatching.size());
  // TODO(reinstaedtler): Update to exact timing
  *result.mutable_run_information()->mutable_start_time() =
      google::protobuf::util::TimeUtil::TimeTToTimestamp(
          std::chrono::system_clock::to_time_t(t1_sys));
  *result.mutable_run_information()->mutable_end_time() =
      google::protobuf::util::TimeUtil::TimeTToTimestamp(
          std::chrono::system_clock::to_time_t(t2_sys));
  *result.mutable_run_information()->mutable_algo_duration() =
      google::protobuf::util::TimeUtil::NanosecondsToDuration(total_nanos);

  *result.mutable_run_config() = config;
  *result.mutable_run_information()->mutable_git_sha() = STABLE_VERSION;
#ifdef HASHING_ACTIVATED
  result.mutable_run_information()->set_edge_hashing_enabled(true);
#endif
  *result.mutable_run_information()->mutable_malloc_implementation() =
      MALLOC_IMPLEMENTATION;
  *result.mutable_run_information()->mutable_git_describe() =
      STABLE_SCM_DESCRIBE;
  *result.mutable_run_information()->mutable_machine()->mutable_host_name() =
      HOSTNAME_STRING;
  *result.mutable_run_information()->mutable_machine()->mutable_build_user() =
      USERNAME;
  *result.mutable_run_information()->mutable_exec_host_name() =
      HeiHGM::BMatching::utils::systeminfo::getHostname();
  *result.mutable_run_information()->mutable_exec_user_name() =
      HeiHGM::BMatching::utils::systeminfo::getUsername();
  *result.mutable_hypergraph() = hypergraph_conf;
  return result;
}
} // namespace HeiHGM::BMatching::app::algorithms