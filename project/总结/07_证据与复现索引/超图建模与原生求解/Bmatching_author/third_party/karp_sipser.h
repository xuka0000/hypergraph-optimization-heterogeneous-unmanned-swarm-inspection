#pragma once
#include "absl/status/statusor.h"
#include "app/app_io.pb.h"
namespace HeiHGM::BMatching {
namespace third_party {
namespace karp_sipser {

absl::StatusOr<HeiHGM::BMatching::app::app_io::Result>
kssRun(HeiHGM::BMatching::app::app_io::Hypergraph h,
       HeiHGM::BMatching::app::app_io::RunConfig run_config);
absl::StatusOr<HeiHGM::BMatching::app::app_io::Result>
ksmdRun(HeiHGM::BMatching::app::app_io::Hypergraph h,
        HeiHGM::BMatching::app::app_io::RunConfig run_config);
} // namespace karp_sipser
} // namespace third_party
} // namespace HeiHGM::BMatching