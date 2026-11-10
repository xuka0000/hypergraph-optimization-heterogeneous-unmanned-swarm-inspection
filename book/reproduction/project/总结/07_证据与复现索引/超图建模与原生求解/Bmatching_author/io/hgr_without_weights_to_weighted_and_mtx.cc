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

int main(int argc, char **argv) {
  if (argc < 2) {
    return 1;
  }

  HypergraphCollection collection;
  std::vector<Hypergraph> hypergraphs;
  std::string collection_name(argv[1]);
  std::string sort(argv[2]);

  for (int i = 3; i < argc; i++) {
    std::string name(argv[i]);
    std::ifstream istream(name);
    std::string weighted_filename = std::string(argv[i]) + ".weighted.hgr";
    std::ofstream ostream(weighted_filename);
    std::ofstream ostream2(weighted_filename + ".mtx");

    StandardIntegerHypergraph hypergraph =
        *HeiHGM::BMatching::io::readHypergraph<StandardIntegerHypergraph>(istream)
             .value()
             .get();
    for (auto n : hypergraph.nodes()) {
      hypergraph.setNodeWeight(n, rand() % hypergraph.nodeDegree(n) + 1);
    }
    for (auto e : hypergraph.edges()) {
      if (hypergraph.edgeSize(e) != 2) {
        std::cerr << "[ERROR] edgeSize!=2" << std::endl;
        exit(1);
      }
      hypergraph.setEdgeWeight(e, rand() % 100 + 1);
    }
    std::cerr << name << std::endl;
    HeiHGM::BMatching::io::writeHypergraph(hypergraph, ostream);
    HeiHGM::BMatching::io::writeHypergraphMTXUnigraph(hypergraph, ostream2);
    Hypergraph h_config;
    h_config.set_name(name);
    h_config.set_format("graph");
    h_config.set_file_path(name);
    h_config.set_node_count(hypergraph.currentNumNodes());
    h_config.set_edge_count(hypergraph.currentNumEdges());
    h_config.set_collection(collection_name);
    h_config.set_sort(sort);
    h_config.set_node_weight_type("unweighted");
    h_config.set_edge_weight_type("unweighted");

    // hypergraphs.push_back(h_config);
    h_config.set_format("hgr");
    h_config.set_file_path(name);
    hypergraphs.push_back(h_config);

    h_config.set_node_weight_type("random(degree)");
    h_config.set_edge_weight_type("random(100)");
    h_config.set_format("hgr");
    h_config.set_file_path(weighted_filename);
    hypergraphs.push_back(h_config);

    h_config.set_format("mtx");
    h_config.set_file_path(weighted_filename + ".mtx");
    hypergraphs.push_back(h_config);
  }
  collection.set_collection_name(collection_name);
  *collection.mutable_version() = "1.0";
  *collection.mutable_hypergraphs() = {hypergraphs.begin(), hypergraphs.end()};
  std::ofstream exp_file("collection.textproto");
  std::string result;
  google::protobuf::TextFormat::PrintToString(collection, &result);
  exp_file << result;
}