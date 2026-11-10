#pragma once
namespace HeiHGM::BMatching::utils {
template <class Iterator> struct Range {
  Iterator _b, _e;
  auto begin() { return _b; }
  auto end() { return _e; }
  const auto begin() const { return _b; }
  const auto end() const { return _e; }
  using value_type = typename Iterator::value_type;
  Range(Iterator begin, Iterator end) : _b(begin), _e(end) {}
};
} // namespace HeiHGM::BMatching::utils