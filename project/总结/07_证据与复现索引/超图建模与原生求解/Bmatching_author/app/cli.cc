/**
 * @file cli.cc
 * @brief User-friendly command-line interface for HeiHGM::BMatching.
 *
 * Provides flag-based access to all algorithms instead of requiring textproto.
 *
 * Example:
 *   bmatching_cli --graph input.hgr --capacity 5 \
 *     --algorithms reductions,greedy,unfold --ordering_method bmindegree_dynamic
 */
#include "easylogging++.h"
#include <fstream>
#include <google/protobuf/text_format.h>
#include <google/protobuf/util/json_util.h>
#include <iostream>
#include <string>
#include <vector>

#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "absl/flags/usage.h"
#include "absl/strings/str_split.h"
#include "absl/strings/strip.h"
#include "app/algorithms/algorithm_impl.h"
#include "app/app_io.pb.h"
#include "ds/bmatching.h"
#include "io/hgr_reader.h"

// --- Required flags ---
ABSL_FLAG(std::string, graph, "", "Path to the hypergraph file.");
ABSL_FLAG(std::string, format, "hgr",
          "Input format: 'hgr' (hMetis) or 'graph'.");
ABSL_FLAG(int64_t, capacity, 1,
          "Node capacity. Use -1 to use node weights from file.");
ABSL_FLAG(std::string, algorithms, "",
          "Comma-separated algorithm pipeline, e.g. "
          "'reductions,greedy,unfold'. Available: greedy, ils, ilp_exact, "
          "local_improvement, presolved_ilp, reductions, unfold, scip.");

// --- Greedy params ---
ABSL_FLAG(std::string, ordering_method, "bmindegree_dynamic",
          "Greedy ordering method. Options: default_order, bmaximize, bweight, "
          "bratio_static, bmult_static, bratio_dynamic, bmindegree_dynamic, "
          "bmindegree1_dynamic, S,cap, S,min, S,pin, S,pin,cap, S,pin,min, "
          "S,scaled, D,cap, D,scaled.");

// --- Timeout (shared by ilp_exact, presolved_ilp, scip, local_improvement) ---
ABSL_FLAG(double, timeout, 60.0, "Solver timeout in seconds.");

// --- ILS params ---
ABSL_FLAG(int64_t, max_tries, 1000, "Max iterations for ILS / local search.");
ABSL_FLAG(bool, inplace, false, "Use newer ILS interface internally.");

// --- Local improvement params ---
ABSL_FLAG(int64_t, iters, 10, "Number of local improvement iterations.");
ABSL_FLAG(int64_t, distance, 5,
          "Number of edges per local improvement neighborhood.");
ABSL_FLAG(std::string, backend, "gurobi",
          "Solver backend for local_improvement: 'gurobi' or 'scip'.");

// --- Reductions params ---
ABSL_FLAG(bool, disable_hint, false,
          "Do not hint to subsequent algorithms that the solution is exact.");
ABSL_FLAG(int64_t, max_runs, 10,
          "Maximum reduction rounds.");
ABSL_FLAG(int64_t, reps, 1, "Number of reduction repetitions.");

// --- Weight rewriting ---
ABSL_FLAG(std::string, edge_weight_type, "",
          "Override edge weights: 'pins', 'uniform', or empty for file default.");
ABSL_FLAG(std::string, node_weight_type, "",
          "Override node weights: 'unweighted' or empty for file default.");

// --- Output ---
ABSL_FLAG(std::string, output, "",
          "Output file path. Empty prints to stdout.");
ABSL_FLAG(std::string, output_format, "text",
          "Output format: 'text' (textproto), 'json', or 'binary'.");
ABSL_FLAG(bool, quiet, false,
          "Suppress output, only print weight and size.");

INITIALIZE_EASYLOGGINGPP

namespace {
using HeiHGM::BMatching::app::app_io::AlgorithmConfig;
using HeiHGM::BMatching::app::app_io::Command;
using HeiHGM::BMatching::app::app_io::Hypergraph;
using HeiHGM::BMatching::app::app_io::Result;
using HeiHGM::BMatching::app::app_io::RunConfig;

AlgorithmConfig buildConfig(const std::string &algo_name) {
  AlgorithmConfig config;
  config.set_algorithm_name(algo_name);

  if (algo_name == "greedy") {
    config.mutable_string_params()->insert(
        {"ordering_method", absl::GetFlag(FLAGS_ordering_method)});
  } else if (algo_name == "ils") {
    config.mutable_int64_params()->insert(
        {"max_tries", absl::GetFlag(FLAGS_max_tries)});
    if (absl::GetFlag(FLAGS_inplace)) {
      config.mutable_string_params()->insert({"inplace", "true"});
    }
  } else if (algo_name == "local_improvement") {
    config.mutable_string_params()->insert(
        {"backend", absl::GetFlag(FLAGS_backend)});
    config.mutable_int64_params()->insert(
        {"iters", absl::GetFlag(FLAGS_iters)});
    config.mutable_int64_params()->insert(
        {"distance", absl::GetFlag(FLAGS_distance)});
    config.mutable_double_params()->insert(
        {"timeout", absl::GetFlag(FLAGS_timeout)});
    config.mutable_int64_params()->insert(
        {"max_tries", absl::GetFlag(FLAGS_max_tries)});
  } else if (algo_name == "ilp_exact" || algo_name == "presolved_ilp" ||
             algo_name == "scip") {
    config.mutable_double_params()->insert(
        {"timeout", absl::GetFlag(FLAGS_timeout)});
  } else if (algo_name == "reductions") {
    config.mutable_string_params()->insert({"assume_sorted", "true"});
    if (absl::GetFlag(FLAGS_disable_hint)) {
      config.mutable_string_params()->insert({"disable_hint", "true"});
    }
    config.mutable_int64_params()->insert(
        {"max_runs", absl::GetFlag(FLAGS_max_runs)});
    config.mutable_int64_params()->insert(
        {"reps", absl::GetFlag(FLAGS_reps)});
  } else if (algo_name == "unfold") {
    config.mutable_string_params()->insert({"assume_sorted", "true"});
  }

  return config;
}

void printUsage() {
  std::cerr
      << "Usage: bmatching_cli --graph <file> --algorithms <algo1,algo2,...>\n"
      << "\n"
      << "Examples:\n"
      << "  # Greedy matching\n"
      << "  bmatching_cli --graph input.hgr --capacity 5 \\\n"
      << "    --algorithms greedy --ordering_method bmindegree_dynamic\n"
      << "\n"
      << "  # Reductions + greedy + unfold\n"
      << "  bmatching_cli --graph input.hgr --capacity 3 \\\n"
      << "    --algorithms reductions,greedy,unfold\n"
      << "\n"
      << "  # Reductions + ILS + unfold\n"
      << "  bmatching_cli --graph input.hgr --capacity 1 \\\n"
      << "    --algorithms reductions,greedy,ils,unfold --max_tries 5000\n"
      << "\n"
      << "  # Exact solve with reductions\n"
      << "  bmatching_cli --graph input.hgr --capacity 1 \\\n"
      << "    --algorithms reductions,presolved_ilp,unfold --timeout 300\n"
      << "\n"
      << "Algorithms: greedy, ils, ilp_exact, local_improvement,\n"
      << "            presolved_ilp, reductions, unfold, scip\n";
}
} // namespace

char *program_invocation_name = "bmatching_cli";

int main(int argc, char **argv) {
  absl::SetProgramUsageMessage(
      "HeiHGM::BMatching - b-matching solver for hypergraphs.\n"
      "  bmatching_cli --graph <file> --algorithms <algo1,algo2,...>");
  GOOGLE_PROTOBUF_VERIFY_VERSION;
  absl::ParseCommandLine(argc, argv);
  START_EASYLOGGINGPP(argc, argv);

  // Validate required flags
  if (absl::GetFlag(FLAGS_graph).empty()) {
    std::cerr << "Error: --graph is required.\n\n";
    printUsage();
    return 1;
  }
  if (absl::GetFlag(FLAGS_algorithms).empty()) {
    std::cerr << "Error: --algorithms is required.\n\n";
    printUsage();
    return 1;
  }

  // Build Hypergraph proto
  Hypergraph hypergraph_conf;
  hypergraph_conf.set_file_path(absl::GetFlag(FLAGS_graph));
  hypergraph_conf.set_format(absl::GetFlag(FLAGS_format));
  if (!absl::GetFlag(FLAGS_edge_weight_type).empty()) {
    hypergraph_conf.set_edge_weight_type(absl::GetFlag(FLAGS_edge_weight_type));
  }
  if (!absl::GetFlag(FLAGS_node_weight_type).empty()) {
    hypergraph_conf.set_node_weight_type(absl::GetFlag(FLAGS_node_weight_type));
  }

  // Build RunConfig
  RunConfig run_config;
  run_config.set_capacity(absl::GetFlag(FLAGS_capacity));
  run_config.set_short_name("cli");

  std::vector<std::string> algo_names =
      absl::StrSplit(absl::GetFlag(FLAGS_algorithms), ',', absl::SkipWhitespace());
  if (algo_names.empty()) {
    std::cerr << "Error: no algorithms specified.\n";
    return 1;
  }

  for (const auto &name : algo_names) {
    *run_config.add_algorithm_configs() = buildConfig(name);
  }

  // Run
  auto result =
      HeiHGM::BMatching::app::algorithms::Run(hypergraph_conf, run_config, true);

  if (!result.ok()) {
    std::cerr << "Error: " << result.status().message() << "\n";
    return 1;
  }

  // Output
  if (absl::GetFlag(FLAGS_quiet)) {
    std::cout << "weight: " << result->weight() << "\n";
    std::cout << "size: " << result->size() << "\n";
    std::cout << "exact: " << (result->is_exact() ? "true" : "false") << "\n";
    auto &run_info = result->run_information();
    auto nanos = google::protobuf::util::TimeUtil::DurationToNanoseconds(
        run_info.algo_duration());
    std::cout << "time_ms: " << (nanos / 1000000.0) << "\n";
    return 0;
  }

  std::string output_format = absl::GetFlag(FLAGS_output_format);
  std::string output_path = absl::GetFlag(FLAGS_output);

  if (output_format == "binary") {
    if (output_path.empty()) {
      std::cerr << "Error: --output is required for binary format.\n";
      return 1;
    }
    std::ofstream ofs(output_path, std::ios::binary);
    result->SerializeToOstream(&ofs);
  } else if (output_format == "json") {
    std::string json;
    google::protobuf::util::JsonPrintOptions options;
    options.add_whitespace = true;
    google::protobuf::util::MessageToJsonString(*result, &json, options);
    if (output_path.empty()) {
      std::cout << json << "\n";
    } else {
      std::ofstream ofs(output_path);
      ofs << json << "\n";
    }
  } else {
    // text (default)
    std::string text;
    google::protobuf::TextFormat::PrintToString(*result, &text);
    if (output_path.empty()) {
      std::cout << text;
    } else {
      std::ofstream ofs(output_path);
      ofs << text;
    }
  }

  return 0;
}
