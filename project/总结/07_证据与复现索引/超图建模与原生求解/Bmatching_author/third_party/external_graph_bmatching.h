#pragma once
#include "absl/status/statusor.h"
#include "app/app_io.pb.h"
#ifdef HeiHGM::BMatching_USE_BSUITOR
#include "third_party/bsuitor.h"
#endif
#ifdef HeiHGM::BMatching_USE_KARP_SIPSER
#include "third_party/karp_sipser.h"
#endif
namespace HeiHGM::BMatching {
namespace third_party {

absl::StatusOr<HeiHGM::BMatching::app::app_io::Result>
Run(HeiHGM::BMatching::app::app_io::Hypergraph h,
    HeiHGM::BMatching::app::app_io::RunConfig run_config) {
  if (!run_config.is_external()) {
    std::cerr << "Can only execute external rules" << std::endl;
    exit(1);
  }
  if (run_config.external_name() == "bsuitor") {
#ifdef HeiHGM::BMatching_USE_BSUITOR
    return HeiHGM::BMatching::third_party::bsuitor::Run(h, run_config);
#else
    return absl::NotFoundError("Please recompile with bsuitor=enabled");
#endif
  }
  if (run_config.external_name() == "ksmd") {
#ifdef HeiHGM::BMatching_USE_KARP_SIPSER
    return HeiHGM::BMatching::third_party::karp_sipser::ksmdRun(h, run_config);
#else
    return absl::NotFoundError("Please recompile with karp_sipser=enabled");
#endif
  }
  if (run_config.external_name() == "kss") {
#ifdef HeiHGM::BMatching_USE_KARP_SIPSER
    return HeiHGM::BMatching::third_party::karp_sipser::kssRun(h, run_config);
#else
    return absl::NotFoundError("Please recompile with karp_sipser=enabled");
#endif
  }
  return absl::NotFoundError("not found");
}
} // namespace third_party
} // namespace HeiHGM::BMatching