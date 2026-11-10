
#include "gmock/gmock-matchers.h"
#include "gtest/gtest.h"

#include "bmatching/reductions_sorted/foldings.h"
#include "ds/bmatching.h"
#include "ds/modifiable_hypergraph.h"
#include "io/hgr_reader.h"

namespace {
using HeiHGM::BMatching::ds::StandardIntegerHypergraph;
using HeiHGM::BMatching::ds::StandardIntegerHypergraphBMatching;
using BMatching = StandardIntegerHypergraphBMatching;

using ::testing::Contains;
using ::testing::ElementsAre;
using ::testing::Not;
} // namespace
INITIALIZE_EASYLOGGINGPP
TEST(ModifiableHypergraphTest, HypergraphMergeTest) {
  std::stringstream simple_file("4 7 1\n"
                                "8 1 2\n"
                                "3 1 7 5 6\n"
                                "8 5 6 4\n"
                                "7 2 3 4\n"
                                "%trailing comment");

  auto hg_s =
      HeiHGM::BMatching::io::readHypergraph<StandardIntegerHypergraph>(simple_file);

  EXPECT_EQ(hg_s.ok(), true);
  StandardIntegerHypergraph hg = *hg_s.value().get();
  hg.sort_weight();

  EXPECT_EQ(hg.currentNumEdges(), 4);
  EXPECT_EQ(hg.currentNumNodes(), 7);
  EXPECT_EQ(hg.edgeWeight(0), 8);
  typename StandardIntegerHypergraph::EdgeType e = 0;
  typename StandardIntegerHypergraph::EdgeType left = 1;
  typename StandardIntegerHypergraph::EdgeType right = 3;
  EXPECT_EQ(hg.edgeWeight(left), 3);
  EXPECT_EQ(hg.edgeWeight(right), 7);
  auto new_weight =
      hg.edgeWeight(left) + hg.edgeWeight(right) - hg.edgeWeight(0);
  EXPECT_EQ(new_weight, 2);

  std::vector<typename StandardIntegerHypergraph::NodeType> nodes(
      hg.pins(left).begin(), hg.pins(left).end());
  auto rep = hg.addMergeEdge(left, new_weight);
  EXPECT_THAT(hg.pins(rep), ElementsAre(0, 4, 5, 6));
  for (auto p : hg.pins(rep)) {
    EXPECT_THAT(hg.incidentEdges(p), Contains(rep));
    EXPECT_THAT(hg.incidentEdges(p), Not(Contains(left)));
  }
  EXPECT_EQ(hg.edgeWeight(rep), 2);
  EXPECT_EQ(hg.edgeIsEnabled(left), false);
  hg.mergeEdges_sorted(rep, right);
  EXPECT_THAT(hg.pins(rep), ElementsAre(0, 1, 2, 3, 4, 5, 6));
  EXPECT_THAT(hg.edgeIsEnabled(right), false);
  for (auto p : hg.pins(rep)) {
    EXPECT_THAT(hg.incidentEdges(p), Contains(rep));
    EXPECT_THAT(hg.incidentEdges(p), Not(Contains(right)));
  }
  // store merge
  auto last_merge =
      std::make_pair<typename BMatching::MergeHistory::WeightTransformation,
                     std::vector<typename BMatching::MergeHistory::MergeInfo>>(
          {e, hg.edgeWeight(e), {left, right}, {e}},
          {typename BMatching::MergeHistory::MergeInfo(rep, left),
           typename BMatching::MergeHistory::MergeInfo(rep, right)});
  // test unmerges
  auto mergesIt = last_merge.second.back();
  bool remove = hg.unmergeEdge_sorted(mergesIt.merged, mergesIt.removed);
  EXPECT_EQ(remove, false);
  EXPECT_EQ(hg.edgeIsEnabled(right), true);
  EXPECT_EQ(hg.edgeSize(right), 3);

  mergesIt = last_merge.second.front();
  remove = hg.unmergeEdge_sorted(mergesIt.merged, mergesIt.removed);
  EXPECT_EQ(remove, true);
  EXPECT_EQ(left, 1);
  EXPECT_EQ(hg.edgeIsEnabled(left), true);
  EXPECT_EQ(hg.edgeSize(left), 4);

  // now unmerge
}
TEST(ModifiableHypergraphTest, HypergraphFoldingTest) {
  std::stringstream simple_file("4 7 11\n"
                                "8 1 2\n"
                                "3 1 7 5 6\n"
                                "8 5 6 4\n"
                                "7 2 3 4\n"
                                "1\n"
                                "1\n"
                                "1\n"
                                "1\n"
                                "1\n"
                                "1\n"
                                "1\n"
                                "%trailing comment");

  auto hg_s =
      HeiHGM::BMatching::io::readHypergraph<StandardIntegerHypergraph>(simple_file);
  EXPECT_EQ(hg_s.ok(), true);
  StandardIntegerHypergraph hg = *hg_s.value().get();
  BMatching bmatching(&hg);
  hg.sort_weight();
  HeiHGM::BMatching::bmatching::reductions_sorted::weighted_edge_folding<>(
      hg, bmatching, 0);
  EXPECT_EQ(bmatching.merge_history().size(), 1);
}