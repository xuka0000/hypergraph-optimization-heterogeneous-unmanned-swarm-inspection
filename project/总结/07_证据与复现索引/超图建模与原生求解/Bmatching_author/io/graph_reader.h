#include "absl/status/statusor.h"
#include "absl/strings/ascii.h"
#include "absl/strings/string_view.h"
#include "ds/modifiable_hypergraph.h"
#include "easylogging++.h"
#include <iostream>
#include <istream>
#include <memory>
#include <sstream>

namespace HeiHGM::BMatching {
namespace io {
template <class Hypergraph>
absl::StatusOr<std::unique_ptr<Hypergraph>> readGraph(std::istream &stream,
                                                      bool verbose = false) {
  TIMED_FUNC(timerObj);

  if (!stream.good()) {
    return absl::NotFoundError("stream not good error");
  }
  std::string header;
  while (header == "") {
    if (stream.eof()) {
      return absl::NotFoundError(
          "File does not contain enough lines. Could not read header.");
    }
    std::getline(stream, header);
    if (header.length() > 0 && header[0] == '%') {
      header = "";
    }
  }
  std::stringstream s(header);
  typename Hypergraph::EdgeType num_edges = 0;
  typename Hypergraph::NodeType num_nodes = 0;
  size_t mode = 0;
  if (!(s >> num_nodes)) {
    return absl::NotFoundError("Failure to read in num_nodes");
  }
  if (!(s >> num_edges)) {
    return absl::NotFoundError("Failure to read in num_edges");
  }
  if (!(s >> mode)) {
    mode = 0;
  }

  auto h = std::make_unique<Hypergraph>(num_edges, num_nodes, true, true);
  for (typename Hypergraph::NodeType node = 0; node < num_nodes; node++) {
    std::string str = "";
    if (stream.eof()) {
      std::stringstream s;
      s << "File does not contain enough lines. Read " << node << ". Expected "
        << num_nodes << std::endl;
      return absl::NotFoundError(s.str());
    }
    while ((str == "" || str[0] == '%')) {
      std::getline(stream, str);
      absl::StripLeadingAsciiWhitespace(&str);
    };

    std::stringstream line_stream{str};
    typename Hypergraph::WeightType weight = 1;
    // Check if we read in weights
    if ((mode / 10) == 1) {
      line_stream >> weight;
    }
    h->setNodeWeight(node, weight);

    typename Hypergraph::NodeType node2;
    while (line_stream >> node2) {
      typename Hypergraph::WeightType edge_weight = 1;
      if (mode % 10 == 1) {
        line_stream >> edge_weight;
      }
      if (node < node2) {
        h->addEdge({node, node2}, edge_weight);
      }
    }
    if (verbose && node % 1000 == 0) {
      std::cout << node << " loaded." << std::endl;
    }
  }
  h->sort();
  return h;
}
} // namespace io
} // namespace HeiHGM::BMatching