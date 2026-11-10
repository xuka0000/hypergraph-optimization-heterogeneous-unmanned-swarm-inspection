#include "third_party/karp_sipser.h"
#include "absl/status/statusor.h"
#include "utils/systeminfo.h"
#ifdef __cplusplus
extern "C" {
#endif
#include "ksmd/ksmd_utils.h"
#include "kss/kss_utils.h"
#ifdef __cplusplus
}
#endif
#include <google/protobuf/util/time_util.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>

#include "build-info.h"
#define VAL(str) #str
#define TOSTRING(str) VAL(str)
#define HOSTNAME_STRING TOSTRING(BUILD_HOST)
#define USERNAME TOSTRING(BUILD_USER)

namespace HeiHGM::BMatching::third_party::karp_sipser {
namespace {

int verify_matching(int n, int *match, int nmatches, int k, int *edge_list,
                    int *match_set) {
  int correct = 1;
  int nmatch = 0;
  int *count_matches = (int *)calloc(n, sizeof(int));
  for (int i = 0; i < n; ++i) {
    int x = match[i];
    if (x >= 0) {
      nmatch++;
      count_matches[x]++;
      if (count_matches[x] == 2) {
        correct = 0;
        break;
      }
    }
  }
  if (correct && nmatch / k == nmatches) {
    for (int i = 0; i < nmatches; ++i) {
      for (int z = 0; z < k; ++z) {
        count_matches[edge_list[match_set[i] * k + z]]--;
        if (count_matches[edge_list[match_set[i] * k + z]] < 0) {
          correct = 0;
          break;
        }
      }
    }
  } else {
    correct = 0;
  }
  free(count_matches);
  return correct;
}

int verify_matching_ksmd(int n, int *match, int nmatches, int k, int *h_points,
                         int *h_begs, int *match_set) {
  int correct = 1;
  int nmatch = 0;
  int *count_matches = (int *)calloc(n, sizeof(int));
  for (int i = 0; i < n; ++i) {
    int x = match[i];
    if (x >= 0) {
      nmatch++;
      count_matches[x]++;
      if (count_matches[x] == 2) {
        correct = 0;
        break;
      }
    }
  }
  if (correct && nmatch / k == nmatches) {
    for (int i = 0; i < nmatches; ++i) {
      for (int z = 0; z < k; ++z) {
        count_matches[h_points[h_begs[match_set[i]] + z]]--;
        if (count_matches[h_points[h_begs[match_set[i]] + z]] < 0) {
          correct = 0;
          break;
        }
      }
    }
  } else {
    correct = 0;
  }
  free(count_matches);
  return correct;
}

} // namespace
absl::StatusOr<HeiHGM::BMatching::app::app_io::Result>
kssRun(HeiHGM::BMatching::app::app_io::Hypergraph h,
       HeiHGM::BMatching::app::app_io::RunConfig run_config) {
  if (h.format() != "txt") {
    std::cerr << "Can only run on txt." << std::endl;
    exit(1);
  }
  if (run_config.capacity() != 1) {
    std::cerr << "Can only run on capacity 1." << std::endl;
    exit(1);
  }
  int *match;
  int sc = atoi(run_config.external_settings().c_str());
  int o = 0; // TODO(reinstaedtler): Make configurable
  HeiHGM::BMatching::app::app_io::Result result;
  int m, n, k, i, correct_match = 1;
  int *edge_list;
  int *degs;
  int *match_set;
  int *edge_order;
  int ans;
  int *dim_starts;
  int *dim_ends;
  int min_ni = -1;
  ::read_tensor_kss(const_cast<char *>(h.file_path().c_str()), &n, &m, &k,
                    &edge_list, &edge_order, &degs, &dim_starts, &dim_ends, o);
  match = (int *)malloc(n * sizeof(int));
  min_ni = dim_ends[0] - dim_starts[0] + 1;
  for (i = 1; i < k; ++i) {
    if (min_ni > dim_ends[i] - dim_starts[i] + 1)
      min_ni = dim_ends[i] - dim_starts[i] + 1;
  }
  match_set = (int *)malloc(min_ni * sizeof(int));
  auto t1_sys = std::chrono::system_clock::now();
  std::chrono::high_resolution_clock::time_point t1 =
      std::chrono::high_resolution_clock::now();
  ans = pm_drive(edge_list, edge_order, n, m, k, &match, degs, dim_starts,
                 dim_ends, sc, &match_set);
  std::chrono::high_resolution_clock::time_point t2 =
      std::chrono::high_resolution_clock::now();
  auto t2_sys = std::chrono::system_clock::now();
  result.set_weight(ans);
  result.set_size(ans);
  *result.mutable_run_information()->mutable_start_time() =
      google::protobuf::util::TimeUtil::TimeTToTimestamp(
          std::chrono::system_clock::to_time_t(t1_sys));
  *result.mutable_run_information()->mutable_end_time() =
      google::protobuf::util::TimeUtil::TimeTToTimestamp(
          std::chrono::system_clock::to_time_t(t2_sys));
  auto dur = t2 - t1;
  *result.mutable_run_information()->mutable_algo_duration() =
      google::protobuf::util::TimeUtil::NanosecondsToDuration(
          (int64_t)dur.count());

  *result.mutable_run_config() = run_config;
  *result.mutable_run_information()->mutable_git_sha() = STABLE_VERSION;
  *result.mutable_run_information()->mutable_git_describe() =
      STABLE_SCM_DESCRIBE;
  *result.mutable_run_information()->mutable_malloc_implementation() =
      MALLOC_IMPLEMENTATION;
  *result.mutable_run_information()->mutable_machine()->mutable_host_name() =
      HOSTNAME_STRING;
  *result.mutable_run_information()->mutable_machine()->mutable_build_user() =
      USERNAME;
  *result.mutable_run_information()->mutable_exec_host_name() =
      HeiHGM::BMatching::utils::systeminfo::getHostname();
  *result.mutable_run_information()->mutable_exec_user_name() =
      HeiHGM::BMatching::utils::systeminfo::getUsername();
  *result.mutable_hypergraph() = h;

  correct_match = verify_matching(n, match, ans, k, edge_list, match_set);
  if (!correct_match) {
    return absl::NotFoundError("Invalid match produced");
  }
  free(degs);
  free(match_set);
  free(edge_list);
  free(edge_order);
  free(dim_starts);
  free(dim_ends);
  free(match);
  return result;
}
absl::StatusOr<HeiHGM::BMatching::app::app_io::Result>
ksmdRun(HeiHGM::BMatching::app::app_io::Hypergraph h,
        HeiHGM::BMatching::app::app_io::RunConfig run_config) {
  if (h.format() != "txt") {
    std::cerr << "Can only run on txt." << std::endl;
    exit(1);
  }
  if (run_config.capacity() != 1) {
    std::cerr << "Can only run on capacity 1." << std::endl;
    exit(1);
  }
  int *match;
  int o = 0; // TODO(reinstaedtler): Make configurable
  HeiHGM::BMatching::app::app_io::Result result;
  int n, m, i, k, min_ni, correct_match = 1;
  int ans;
  int *v_hids, *v_starts, *h_begs, *h_points;
  int *edge_list;
  int *match_set;

  read_file_ks(&n, &m, &k, const_cast<char *>(h.file_path().c_str()), &v_hids,
               &v_starts, &h_begs, &h_points, o, &min_ni);
  match_set = (int *)malloc(min_ni * sizeof(int));
  match = (int *)malloc(n * sizeof(int));

  edge_list = (int *)malloc(m * sizeof(int));
  for (i = 0; i < m; ++i)
    edge_list[i] = i;

  auto t1_sys = std::chrono::system_clock::now();
  std::chrono::high_resolution_clock::time_point t1 =
      std::chrono::high_resolution_clock::now();
  ans = ksmd(edge_list, n, m, v_hids, v_starts, h_begs, h_points, &match,
             &match_set);
  std::chrono::high_resolution_clock::time_point t2 =
      std::chrono::high_resolution_clock::now();
  auto t2_sys = std::chrono::system_clock::now();
  result.set_weight(ans);
  result.set_size(ans);
  *result.mutable_run_information()->mutable_start_time() =
      google::protobuf::util::TimeUtil::TimeTToTimestamp(
          std::chrono::system_clock::to_time_t(t1_sys));
  *result.mutable_run_information()->mutable_end_time() =
      google::protobuf::util::TimeUtil::TimeTToTimestamp(
          std::chrono::system_clock::to_time_t(t2_sys));
  auto dur = t2 - t1;
  *result.mutable_run_information()->mutable_algo_duration() =
      google::protobuf::util::TimeUtil::NanosecondsToDuration(
          (int64_t)dur.count());

  *result.mutable_run_config() = run_config;
  *result.mutable_run_information()->mutable_git_sha() = STABLE_VERSION;
  *result.mutable_run_information()->mutable_git_describe() =
      STABLE_SCM_DESCRIBE;
  *result.mutable_run_information()->mutable_malloc_implementation() =
      MALLOC_IMPLEMENTATION;
  *result.mutable_run_information()->mutable_machine()->mutable_host_name() =
      HOSTNAME_STRING;
  *result.mutable_run_information()->mutable_machine()->mutable_build_user() =
      USERNAME;
  *result.mutable_run_information()->mutable_exec_host_name() =
      HeiHGM::BMatching::utils::systeminfo::getHostname();
  *result.mutable_run_information()->mutable_exec_user_name() =
      HeiHGM::BMatching::utils::systeminfo::getUsername();
  *result.mutable_hypergraph() = h;

  correct_match =
      verify_matching_ksmd(n, match, ans, k, h_points, h_begs, match_set);
  if (!correct_match) {
    return absl::NotFoundError("Invalid match produced");
  }
  free(v_hids);
  free(v_starts);
  free(h_begs);
  free(h_points);
  free(edge_list);

  free(match);
  return result;
}
} // namespace HeiHGM::BMatching::third_party::karp_sipser