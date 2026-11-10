#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "app/app_io.pb.h"
#include "ds/modifiable_hypergraph.h"
#include "google/protobuf/text_format.h"
#include "io/hgr_reader.h"
#include "io/hgr_writer.h"
#include <fstream>

namespace {
using HeiHGM::BMatching::app::app_io::Hypergraph;
using HeiHGM::BMatching::app::app_io::HypergraphCollection;
using HeiHGM::BMatching::ds::StandardIntegerHypergraph;
} // namespace

ABSL_FLAG(std::string, collection_name, "<noname>",
          "The name of the collection to write.");
ABSL_FLAG(std::string, version_collection, "1.0",
          "The version of the collection");
ABSL_FLAG(std::string, description, "",
          "The description of the collection to write.");
ABSL_FLAG(int, mod, 100, "Modulo which number randomness is used.");
ABSL_FLAG(bool, generate_random, true, "If true generates random<mod> entries");
ABSL_FLAG(std::string, sort, "", "Which sort to be saved.");
ABSL_FLAG(std::string, input_edges_weight, "unweighted",
          "Whether input edges are weighted or not.");
ABSL_FLAG(std::string, input_nodes_weight, "unweighted",
          "Whether input nodes are weighted or not.");

int main(int argc, char **argv) {

  GOOGLE_PROTOBUF_VERIFY_VERSION;
  absl::ParseCommandLine(argc, argv);
  if (argc < 2) {
    std::cerr << "Please provide at least 2 params" << std::endl;
    return 1;
  }

  HypergraphCollection collection;
  std::vector<Hypergraph> hypergraphs;
  int mod = absl::GetFlag(FLAGS_mod);
  std::string collection_name = absl::GetFlag(FLAGS_collection_name);
  std::string sort = absl::GetFlag(FLAGS_sort);
  bool generate_random = absl::GetFlag(FLAGS_generate_random);

  bool entries = false;
  for (int i = 2; i < argc; i++) {
    std::string name(argv[i]);
    if (name == "--") {
      entries = true;
      continue;
    }
    if (!entries) {
      continue;
    }
    std::cerr << name << std::endl;

    std::ifstream istream(name);

    StandardIntegerHypergraph hypergraph =
        *HeiHGM::BMatching::io::readHypergraph<StandardIntegerHypergraph>(istream)
             .value()
             .get();
    std::stringstream weighted_file_ext;
    weighted_file_ext << "random" << mod;
    std::string weighted_filename = name + "." + weighted_file_ext.str();
    if (generate_random) {

      std::ofstream ostream(weighted_filename);
      for (auto n : hypergraph.nodes()) {
        hypergraph.setNodeWeight(n, rand() % hypergraph.nodeDegree(n) + 1);
      }
      for (auto e : hypergraph.edges()) {
        hypergraph.setEdgeWeight(e, rand() % mod + 1);
      }

      HeiHGM::BMatching::io::writeHypergraph(hypergraph, ostream);
    }
    Hypergraph h_config;
    h_config.set_name(name);
    h_config.set_format("hgr");
    h_config.set_file_path(name);
    h_config.set_node_count(hypergraph.currentNumNodes());
    h_config.set_edge_count(hypergraph.currentNumEdges());
    h_config.set_collection(collection_name);
    h_config.set_sort(sort);
    h_config.set_node_weight_type(absl::GetFlag(FLAGS_input_nodes_weight));
    h_config.set_edge_weight_type(absl::GetFlag(FLAGS_input_edges_weight));

    // hypergraphs.push_back(h_config);
    h_config.set_format("hgr");
    h_config.set_file_path(name);
    hypergraphs.push_back(h_config);

    h_config.set_node_weight_type("random(degree)");

    h_config.set_edge_weight_type(weighted_file_ext.str());
    h_config.set_format("hgr");
    h_config.set_file_path(weighted_filename);
    hypergraphs.push_back(h_config);
  }
  collection.set_collection_name(collection_name);
  *collection.mutable_version() = absl::GetFlag(FLAGS_version_collection);
  *collection.mutable_hypergraphs() = {hypergraphs.begin(), hypergraphs.end()};
  std::ofstream exp_file("collection.textproto");
  std::string result;
  google::protobuf::TextFormat::PrintToString(collection, &result);
  exp_file << result;
}