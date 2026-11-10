#include "io/hgr_reader.h"
#include "io/hgr_writer.h"
#include "runner/hypergraph_storage_system.h"
#include <fstream>
#include <set>

namespace {
using HeiHGM::BMatching::ds::StandardIntegerHypergraph;
using HeiHGM::BMatching::runner::hypergraph_storage_system::HypergraphStorageSystem;
} // namespace
int main(int argc, char **argv) {
  if (argc != 2) {
    std::cerr << "Please provide only one path." << std::endl;
    return 1;
  }
  std::string path(argv[1]);
  int updated_count = 0;
  HypergraphStorageSystem hs(path);
  auto collections = hs.collections();
  for (const auto &col : collections) {
    std::cout << "Checking (skip/yes)?" << col.collection_name() << std::endl;
    std::string control;
    std::cin >> control;
    if (control == "skip") {
      continue;
    }
    std::cout << "Checking collection" << std::endl;
    for (const auto &hypergraph : col.hypergraphs()) {
      if (hypergraph.format() != "hgr") {
        continue;
      }
      std::ifstream file(hypergraph.file_path());
      auto graph = std::move(
          HeiHGM::BMatching::io::readHypergraph<StandardIntegerHypergraph>(file)
              .value());
      bool updated = false;
      for (const auto &e : graph->edges()) {
        std::set setified(graph->pins(e).begin(), graph->pins(e).end());
        if (setified.size() != graph->edgeSize(e)) {
          updated = true;
        }
      }
      if (updated) {
        std::cout << "Updating " << hypergraph.file_path() << std::endl;
        StandardIntegerHypergraph updated_graph(
            graph->initialNumEdges(), graph->initialNumNodes(), true, true);
        for (const auto &e : graph->edges()) {
          std::set setified(graph->pins(e).begin(), graph->pins(e).end());
          for (const auto &p : setified) {
            updated_graph.establishConnection(e, p);
          }
          updated_graph.setEdgeWeight(e, graph->edgeWeight(e));
        }
        for (const auto n : graph->nodes()) {
          updated_graph.setNodeWeight(n, graph->nodeWeight(n));
        }
        std::ofstream o_file(hypergraph.file_path());
        HeiHGM::BMatching::io::writeHypergraph(updated_graph, o_file);
        updated_count++;
      }
    }
  }
  std::cout << "Found " << updated_count << " hypergraphs to update"
            << std::endl;
}