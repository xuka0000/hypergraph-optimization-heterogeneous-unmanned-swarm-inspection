#include <fstream>
#include <iostream>
#include <sstream>

#include "absl/strings/str_replace.h"
#include "app/algorithms/algorithm_impl.h"
#include "app/app_io.pb.h"
#include "google/protobuf/text_format.h"
#include "io/hgr_writer.h"

int main(int argc, char** argv) {
  std::ifstream file_input(argv[1]);
  std::string root_path(argv[2]);
  heihgm::app::app_io::ExperimentConfig _config;
  std::stringstream buffer;
  buffer << file_input.rdbuf();

  if (!google::protobuf::TextFormat::ParseFromString(buffer.str(), &_config)) {
    std::cerr << "Failure while parsing" << std::endl;
  }
  std::vector<heihgm::app::app_io::Hypergraph> hypergraphs;
  for (auto current : _config.hypergraphs()) {
    std::string old_path = current.file_path();
    std::string new_file_path =
        absl::StrReplaceAll(old_path, {{"?", "."}}) + ".rendered.negative_edge_size";
    *current.mutable_file_path() = absl::StrCat(root_path, "/", current.file_path());
    auto hgr_status =
        heihgm::app::algorithms::loadFile<heihgm::ds::StandardIntegerHypergraph>(current);
    auto hgr = std::move(hgr_status.value());
    int max_w = 0;
    for (auto e : hgr->edges()) {
      max_w = std::max(max_w, hgr->edgeSize(e));
    }
    max_w += 1;
    for (auto e : hgr->edges()) {
      hgr->setEdgeWeight(e, max_w - hgr->edgeSize(e));
    }
    current.set_edge_weight_type("negative_edge_size");
    std::ofstream file(absl::StrCat(root_path, "/", new_file_path));
    heihgm::io::writeHypergraph(*hgr, file);
    current.set_file_path(new_file_path);
    hypergraphs.push_back(current);
    std::cout << "." << std::endl;
  }
  *_config.mutable_hypergraphs() = {hypergraphs.begin(), hypergraphs.end()};
  std::ofstream file_output(absl::StrReplaceAll(std::string(argv[1]), {{"DOT", "."}}));
  std::string result;
  google::protobuf::TextFormat::PrintToString(_config, &result);
  file_output << result;
}