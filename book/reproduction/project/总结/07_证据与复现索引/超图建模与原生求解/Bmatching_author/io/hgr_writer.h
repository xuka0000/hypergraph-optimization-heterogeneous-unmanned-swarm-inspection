
#include <iostream>
#include <ostream>
namespace HeiHGM::BMatching {
namespace io {
template <class Hypergraph>
void writeHypergraph(Hypergraph &h, std::ostream &ostream) {
  ostream << h.initialNumEdges() << " " << h.initialNumNodes() << " 11"
          << std::endl;
  int edges = 0;
  for (const auto &e : h.edges()) {
    ostream << h.edgeWeight(e);
    for (auto p : h.pins(e)) {
      ostream << " " << p + 1;
    }
    ostream << std::endl;
    edges++;
  }
  std::cerr << h.currentNumEdges() << std::endl;
  std::cerr << "Written " << edges << std::endl;
  int nodes = 0;
  for (int n = 0; n < h.initialNumNodes(); n++) {
    if (h.nodeIsEnabled(n)) {
      ostream << h.nodeWeight(n) << std::endl;
    } else
      ostream << 1 << std::endl;
    nodes++;
  }
  std::cerr << "Written " << nodes << std::endl;
}
template <class Hypergraph>
void writeHypergraphMTXUnigraph(Hypergraph &h, std::ostream &ostream) {
  ostream << "%%MatrixMarket matrix coordinate real symmetric" << std::endl;
  ostream << h.initialNumNodes() << " " << h.initialNumNodes() << " "
          << h.initialNumEdges() + h.initialNumNodes() << std::endl;
  for (const auto &e : h.edges()) {
    for (auto p : h.pins(e)) {
      ostream << p + 1 << " ";
    }
    ostream << h.edgeWeight(e) << std::endl;
  }
  for (int n = 0; n < h.initialNumNodes(); n++) {
    if (h.nodeIsEnabled(n) && h.nodeDegree(n) > 0) {
      ostream << n + 1 << " " << n + 1 << " " << h.nodeWeight(n) << std::endl;
    } else {
      ostream << n + 1 << " " << n + 1 << " " << 1
              << std::endl; // Defaulting on disabled
    }
  }
}
} // namespace io
} // namespace HeiHGM::BMatching