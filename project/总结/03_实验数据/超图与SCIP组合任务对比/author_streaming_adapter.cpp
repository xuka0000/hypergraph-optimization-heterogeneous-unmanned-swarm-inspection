#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <numeric>
#include <string>
#include <vector>
#include "author_matching_with_output_accessor.h"
struct Edge { std::vector<size_t> pins; int w; int id; };
struct Graph { using NodeType=size_t; using EdgeType=size_t; using WeightType=int; using StreamMember=Edge; };
template<class G, typename DT=float> struct Storage {
  std::vector<DT> phi; std::vector<typename G::StreamMember> stack;
  size_t stack_size() const { return stack.size(); }
};
template<class G> using MatchingResult=heihgm::ds::streaming::MatchingResult<G>;
using ResultType=heihgm::ds::streaming::MatchingResult<Graph>;
using FloatType=float;
void Stream(const Edge& member, Storage<Graph>& storage, FloatType eps) {
    FloatType weight = 0;
    for (auto p : member.pins) {
      weight += storage.phi[p];
    }
    if (member.w > weight * (1.0 + eps)) {
      auto offset = member.w - weight;
      storage.stack.push_back(member);
      for (auto p : member.pins) {
        storage.phi[p] += offset;
      }
    }
  }
template<class StreamableHypergraph, typename DT>
auto Transform(Storage<StreamableHypergraph,DT>& storage) {
    auto matchingResult = std::make_unique<MatchingResult<StreamableHypergraph>>(
        storage.phi.size(), storage.stack_size());
    for (auto ix = storage.stack.rbegin(); ix != storage.stack.rend(); ix++) {
      if (matchingResult->matchable(*ix)) {
        matchingResult->addToMatching(*ix);
      }
    }
    return matchingResult;
  }
extern "C" __declspec(dllexport)
int solve(int n, int m, const int* ptr, const int* pins, const int* weights,
          float eps, int* selected, double* elapsed) {
  const auto t0=std::chrono::steady_clock::now();
  Storage<Graph> storage; storage.phi.assign(n, 0.0f);
  for(int e=0;e<m;++e) {
    Edge member; member.id=e; member.w=weights[e];
    for(int k=ptr[e];k<ptr[e+1];++k) member.pins.push_back(pins[k]);
    Stream(member,storage,eps);
  }
  auto result=Transform(storage);
  const auto t1=std::chrono::steady_clock::now();
  *elapsed=std::chrono::duration<double>(t1-t0).count();
  int count=0;
  for(const auto& edge:result->observed_edges()) selected[count++]=edge.id;
  return count;
}
