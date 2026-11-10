#include "utils/random.h"
#include "utils/range.h"

#include <vector>

namespace HeiHGM::BMatching {

namespace ds {

namespace {
using HeiHGM::BMatching::utils::Randomize;
using HeiHGM::BMatching::utils::Range;
} // namespace

template <class BMatching> class ReplayBuffer {
  std::vector<std::pair<typename BMatching::EdgeType, char>> _replay;

public:
  ReplayBuffer(int start_buffer_size = 64) {
    _replay.reserve(start_buffer_size);
  }
  ReplayBuffer(const ReplayBuffer &) = delete;
  void add(typename BMatching::EdgeType e) { _replay.push_back({e, 1}); }
  void remove(typename BMatching::EdgeType e) { _replay.push_back({e, 0}); }
  auto size() { return _replay.size(); }
  void clear() { _replay.clear(); }
  void reverse(BMatching &matching) {
    for (auto [e, type] : Range(_replay.rbegin(), _replay.rend())) {
      if (type == 0) {
        matching.NonNotaddToMatching(e);
      } else {
        matching.NonNotRemoveFromMatching(e);
      }
    }
  }
};
} // namespace ds
} // namespace HeiHGM::BMatching