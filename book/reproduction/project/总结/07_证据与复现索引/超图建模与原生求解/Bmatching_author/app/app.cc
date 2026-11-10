/**
 * @file HeiHGM::BMatching.cc
 * @author Henrik Reinstädtler (reinstaedtler@stud.uni-heidelberg.de)
 * @brief  Main program for b-matching in hypergraphs.
 * @version 0.1
 * @date 2022-12-02
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "easylogging++.h"
#include <fstream>
#include <google/protobuf/text_format.h>
#include <iostream>

#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "app/algorithms/algorithm_impl.h"
#include "app/app_io.pb.h"
#include "ds/bmatching.h"
#include "io/hgr_reader.h"

ABSL_FLAG(std::string, command_textproto, "help",
          "The command to run in textproto format.");

INITIALIZE_EASYLOGGINGPP
namespace {
using HeiHGM::BMatching::app::app_io::Result;
using HeiHGM::BMatching::ds::StandardIntegerHypergraph;
using HeiHGM::BMatching::ds::StandardIntegerHypergraphBMatching;
} // namespace
  // needed for libtcmalloc and profiler
char *program_invocation_name = "HeiHGM::BMatching";
int main(int argc, char **argv) {
  GOOGLE_PROTOBUF_VERIFY_VERSION;
  absl::ParseCommandLine(argc, argv);
  START_EASYLOGGINGPP(argc, argv);

  // std::cout<<"Command:"<< absl::GetFlag(FLAGS_command_textproto)<<std::endl;
  HeiHGM::BMatching::app::app_io::Command command;
  if (google::protobuf::TextFormat::ParseFromString(
          absl::GetFlag(FLAGS_command_textproto), &command)) {
    if (command.command() == "run") {
      auto result = HeiHGM::BMatching::app::algorithms::Run(
          command.hypergraph(), command.config(), true);

      result->PrintDebugString();
    }
  } else {
    command.set_command("help");
    std::string command_s;
    command.PrintDebugString();
    std::cerr << "Parsing failed." << std::endl;
    return 1;
  }
  return 0;
}