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
absl::StatusOr<std::unique_ptr<Hypergraph>>
readHypergraph(std::istream &stream, bool verbose = false) {
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
  if (!(s >> num_edges)) {
    return absl::NotFoundError("Failure to read in num_edges");
  }
  if (!(s >> num_nodes)) {
    return absl::NotFoundError("Failure to read in num_nodes");
  }
  if (!(s >> mode)) {
    mode = 0;
  }

  auto h = std::make_unique<Hypergraph>(num_edges, num_nodes, true, true);
  for (typename Hypergraph::EdgeType edge = 0; edge < num_edges; edge++) {
    std::string str = "";
    if (stream.eof()) {
      std::stringstream s;
      s << "File does not contain enough lines. Read " << edge << ". Expected "
        << num_edges << std::endl;
      return absl::NotFoundError(s.str());
    }
    while ((str == "" || str[0] == '%')) {
      std::getline(stream, str);
      absl::StripLeadingAsciiWhitespace(&str);
    };

    std::stringstream line_stream{str};
    typename Hypergraph::WeightType weight = 1;
    // Check if we read in weights
    if ((mode % 10) == 1) {
      line_stream >> weight;
    }
    typename Hypergraph::NodeType node;
    bool sorted = true;
    typename Hypergraph::NodeType previousNode = 0;
    std::vector<typename Hypergraph::NodeType> nodes;
    while (line_stream >> node) {
      nodes.push_back(node - 1); // 0 based in memory
      sorted &= previousNode < node;
      previousNode = node;
    }
    h->addEdge(nodes, weight);
    if (!sorted) {
      h->resortEdge(edge);
    }
    if (verbose && edge % 1000 == 0) {
      std::cout << edge << " loaded." << std::endl;
    }
  }
  // check if mode is greater eqal than 10
  // this means weights for the nodes need to be parsed.
  if (mode >= 10) {
    for (typename Hypergraph::NodeType node = 0; node < num_nodes; node++) {

      std::string line;
      while (line.length() < 1 || line[0] == '%') {
        if (stream.eof()) {
          std::cerr << "Stream does not contain enough lines for nodes. Read "
                    << node << ". Expected " << num_nodes << std::endl;
          return absl::NotFoundError("Stream exhausted");
        }

        std::getline(stream, line);
        absl::StripLeadingAsciiWhitespace(&line);
      }
      std::stringstream line_stream{std::string(line)};
      typename Hypergraph::WeightType weight;
      line_stream >> weight;

      h->setNodeWeight(node, weight);
    }
  } else {
    for (const auto n : h->nodes()) {
      h->setNodeWeight(n, 1);
    }
  }
  return h;
}
} // namespace io
} // namespace HeiHGM::BMatching