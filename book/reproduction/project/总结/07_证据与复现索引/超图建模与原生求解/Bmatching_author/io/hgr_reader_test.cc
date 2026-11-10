#include "gmock/gmock-matchers.h"
#include "gtest/gtest.h"

#include "ds/modifiable_hypergraph.h"
#include "io/hgr_reader.h"

namespace {
using HeiHGM::BMatching::ds::StandardIntegerHypergraph;
using ::testing::ElementsAre;
} // namespace

TEST(HgrReaderTest, ReadsSimpleFile) {
  std::stringstream simple_file("%original file from the manual of hmetis\n"
                                "%head\n"
                                "4 11\n"
                                "%body\n"
                                "1 2 \n"
                                "1 7 5 6\n"
                                "5 6 4\n"
                                "2 3 4\n");

  auto hg_s =
      HeiHGM::BMatching::io::readHypergraph<StandardIntegerHypergraph>(simple_file);
  EXPECT_EQ(hg_s.ok(), true);
  auto hg = *hg_s.value().get();

  EXPECT_EQ(hg.currentNumEdges(), 4);
  EXPECT_EQ(hg.currentNumNodes(), 11);
  EXPECT_EQ(hg.edgeSize(0), 2);
  EXPECT_EQ(hg.edgeSize(1), 4);
  EXPECT_EQ(hg.edgeSize(2), 3);
  EXPECT_EQ(hg.edgeSize(3), 3);

  for (auto edge : hg.edges()) {
    EXPECT_EQ(hg.edgeWeight(edge), 1);
  }
  // node weight is 1.
  for (auto node : hg.nodes()) {
    EXPECT_EQ(hg.nodeWeight(node), 1);
  }
}
TEST(HgrReaderTest, ReadsWeightedHyperedgesFile) {
  std::stringstream simple_file("4 7 1\n"
                                "2 1 2\n"
                                "3 1 7 5 6\n"
                                "8 5 6 4\n"
                                "7 2 3 4\n"
                                "%trailing comment");

  auto hg_s =
      HeiHGM::BMatching::io::readHypergraph<StandardIntegerHypergraph>(simple_file);
  EXPECT_EQ(hg_s.ok(), true);
  auto hg = *hg_s.value().get();

  EXPECT_EQ(hg.currentNumEdges(), 4);
  EXPECT_EQ(hg.currentNumNodes(), 7);

  // check weights
  EXPECT_EQ(hg.edgeWeight(0), 2);
  EXPECT_EQ(hg.edgeWeight(1), 3);
  EXPECT_EQ(hg.edgeWeight(2), 8);
  EXPECT_EQ(hg.edgeWeight(3), 7);

  // check nodes, note that this is 0 based
  EXPECT_THAT(hg.incidentEdges(0), ElementsAre(0, 1));
  EXPECT_THAT(hg.incidentEdges(1), ElementsAre(0, 3));
  EXPECT_THAT(hg.incidentEdges(2), ElementsAre(3));
  EXPECT_THAT(hg.incidentEdges(3), ElementsAre(2, 3));
  EXPECT_THAT(hg.incidentEdges(4), ElementsAre(1, 2));
  EXPECT_THAT(hg.incidentEdges(5), ElementsAre(1, 2));
  EXPECT_THAT(hg.incidentEdges(6), ElementsAre(1));

  // and their weight
  for (auto node : hg.nodes()) {
    EXPECT_EQ(hg.nodeWeight(node), 1);
  }
}

TEST(HgrReaderTest, ReadsWeightedHypernodesFile) {
  std::stringstream simple_file("4 7 10\n"
                                "1 2\n"
                                "1 7 5 6\n"
                                "5 6 4\n"
                                "2 3 4\n"
                                "5\n"
                                "1\n"
                                "8\n"
                                "7\n"
                                "3\n"
                                "9\n"
                                "3");

  auto hg_s =
      HeiHGM::BMatching::io::readHypergraph<StandardIntegerHypergraph>(simple_file);
  EXPECT_EQ(hg_s.ok(), true);
  auto hg = *hg_s.value().get();

  EXPECT_EQ(hg.currentNumEdges(), 4);
  EXPECT_EQ(hg.currentNumNodes(), 7);

  // check weights of edges, uniformly one
  EXPECT_EQ(hg.edgeWeight(0), 1);
  EXPECT_EQ(hg.edgeWeight(1), 1);
  EXPECT_EQ(hg.edgeWeight(2), 1);
  EXPECT_EQ(hg.edgeWeight(3), 1);

  // check nodes, note that this is 0 based
  EXPECT_THAT(hg.incidentEdges(0), ElementsAre(0, 1));
  EXPECT_THAT(hg.incidentEdges(1), ElementsAre(0, 3));
  EXPECT_THAT(hg.incidentEdges(2), ElementsAre(3));
  EXPECT_THAT(hg.incidentEdges(3), ElementsAre(2, 3));
  EXPECT_THAT(hg.incidentEdges(4), ElementsAre(1, 2));
  EXPECT_THAT(hg.incidentEdges(5), ElementsAre(1, 2));
  EXPECT_THAT(hg.incidentEdges(6), ElementsAre(1));

  // check nodes weight
  EXPECT_THAT(hg.nodeWeight(0), 5);
  EXPECT_THAT(hg.nodeWeight(1), 1);
  EXPECT_THAT(hg.nodeWeight(2), 8);
  EXPECT_THAT(hg.nodeWeight(3), 7);
  EXPECT_THAT(hg.nodeWeight(4), 3);
  EXPECT_THAT(hg.nodeWeight(5), 9);
  EXPECT_THAT(hg.nodeWeight(6), 3);
}

TEST(HgrReaderTest, ReadsWeightedHyperedgesAndWeightedHypernodesFile) {
  std::stringstream simple_file("4 7 11\n"
                                "2 1 2\n"
                                "3 1 7 5 6\n"
                                "8 5 6 4\n"
                                "7 2 3 4\n"
                                " % start of weights\n"
                                "5\n"
                                "1\n"
                                "8\n"
                                "7\n"
                                "3\n"
                                "9\n"
                                "3\n");

  auto hg_s =
      HeiHGM::BMatching::io::readHypergraph<StandardIntegerHypergraph>(simple_file);
  EXPECT_EQ(hg_s.ok(), true);
  auto hg = *hg_s.value().get();
  EXPECT_EQ(hg.currentNumEdges(), 4);
  EXPECT_EQ(hg.currentNumNodes(), 7);

  // check weights
  EXPECT_EQ(hg.edgeWeight(0), 2);
  EXPECT_EQ(hg.edgeWeight(1), 3);
  EXPECT_EQ(hg.edgeWeight(2), 8);
  EXPECT_EQ(hg.edgeWeight(3), 7);

  // check nodes, note that this is 0 based
  EXPECT_THAT(hg.incidentEdges(0), ElementsAre(0, 1));
  EXPECT_THAT(hg.incidentEdges(1), ElementsAre(0, 3));
  EXPECT_THAT(hg.incidentEdges(2), ElementsAre(3));
  EXPECT_THAT(hg.incidentEdges(3), ElementsAre(2, 3));
  EXPECT_THAT(hg.incidentEdges(4), ElementsAre(1, 2));
  EXPECT_THAT(hg.incidentEdges(5), ElementsAre(1, 2));
  EXPECT_THAT(hg.incidentEdges(6), ElementsAre(1));

  // and their weight
  EXPECT_THAT(hg.nodeWeight(0), 5);
  EXPECT_THAT(hg.nodeWeight(1), 1);
  EXPECT_THAT(hg.nodeWeight(2), 8);
  EXPECT_THAT(hg.nodeWeight(3), 7);
  EXPECT_THAT(hg.nodeWeight(4), 3);
  EXPECT_THAT(hg.nodeWeight(5), 9);
  EXPECT_THAT(hg.nodeWeight(6), 3);
}