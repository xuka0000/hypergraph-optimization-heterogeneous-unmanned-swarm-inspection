#include "ds/modifiable_hypergraph.h"
#include "io/hgr_writer.h"
#include "io/mtx_reader.h"
#include <fstream>

namespace {
using HeiHGM::BMatching::ds::StandardIntegerHypergraph;
}
int main(int argc, char **argv) {
  if (argc != 3) {
    return 1;
  }
  std::ifstream istream(argv[1]);
  std::ofstream ostream(argv[2]);
  std::ofstream ostream2(std::string(argv[2]) + ".mtx");

  StandardIntegerHypergraph hypergraph =
      HeiHGM::BMatching::io::readMtxAsUndirectedGraph<StandardIntegerHypergraph>(
          istream);
  for (auto e : hypergraph.edges()) {
    hypergraph.setEdgeWeight(e, rand() % 100 + 1);
  }
  HeiHGM::BMatching::io::writeHypergraph(hypergraph, ostream);
  HeiHGM::BMatching::io::writeHypergraphMTXUnigraph(hypergraph, ostream2);
}