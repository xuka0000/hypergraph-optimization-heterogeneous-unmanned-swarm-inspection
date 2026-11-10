#pragma once
#include "app/app_io.pb.h"
namespace HeiHGM::BMatching {
namespace third_party {
namespace bsuitor {

HeiHGM::BMatching::app::app_io::Result
Run(HeiHGM::BMatching::app::app_io::Hypergraph h,
    HeiHGM::BMatching::app::app_io::RunConfig run_config);
}
} // namespace third_party
} // namespace HeiHGM::BMatching