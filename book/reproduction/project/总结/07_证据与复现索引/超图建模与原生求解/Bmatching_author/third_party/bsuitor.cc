#include "third_party/bsuitor.h"
#include "app/app_io.pb.h"
#include "bMatching.h"
#include "build-info.h"
#include "utils/systeminfo.h"
#include <google/protobuf/util/time_util.h>
#include <tuple>

#define VAL(str) #str
#define TOSTRING(str) VAL(str)
#define HOSTNAME_STRING TOSTRING(BUILD_HOST)
#define USERNAME TOSTRING(BUILD_USER)

namespace HeiHGM::BMatching::third_party::bsuitor {
namespace {
std::tuple<bool, int, int> MatchingResults(CSR *g, Node *S, int n) {
  bool flag = false;
  float weight = 0.0;
  int count = 0;
  std::cout << "n:" << n << std::endl;
  // #pragma omp parallel for schedule(static)
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < S[i].curSize; j++) {
      int a = S[i].heap[j].id;
      weight += S[i].heap[j].weight;
      count++;
      if (a != -1 && !S[a].find_id(i)) {
        cout << "(" << i << "," << a << ") :";
        for (int k = 0; k < S[i].curSize; k++)
          cout << S[i].heap[k].id << "(" << S[i].heap[k].weight << ")"
               << " ";
        cout << " I ";
        for (int k = 0; k < S[a].curSize; k++)
          cout << S[a].heap[k].id << "(" << S[a].heap[k].weight << ")"
               << " ";
        cout << endl;

        cout << "i: ";
        for (int k = 0; k < n; k++)
          if (k != i && S[k].find_id(i))
            cout << k << " ";
        cout << endl;
        cout << "a: ";
        for (int k = 0; k < n; k++)
          if (k != a && S[k].find_id(a))
            cout << k << " ";
        cout << endl;
        cout << "i: ";
        for (int k = g->verPtr[i]; k < g->verPtr[i + 1]; k++)
          cout << g->verInd[k].id << " " << g->verInd[k].weight << ",";
        cout << endl;
        cout << "a: ";
        for (int k = g->verPtr[a]; k < g->verPtr[a + 1]; k++)
          cout << g->verInd[k].id << " " << g->verInd[k].weight << ",";
        cout << endl;
        flag = true;
        break;
      }
    }
    if (flag)
      break;
  }
  std::cerr << "Count:" << count << std::endl;
  if (flag)
    return std::make_tuple<bool, int, int>(false, 0, 0);
  else {
    if (count == 0)
      cout << "It is an empty matching" << endl;
    else
      cout << "Matching Weight: " << weight / 2.0 << endl;
    auto res = std::make_tuple(true, weight / 2.0, count);
    return res;
  }
}
} // namespace
HeiHGM::BMatching::app::app_io::Result
Run(HeiHGM::BMatching::app::app_io::Hypergraph h,
    HeiHGM::BMatching::app::app_io::RunConfig run_config) {
  if (h.format() != "mtx") {
    std::cerr << "Can only run on mtx." << std::endl;
    exit(1);
  }
  HeiHGM::BMatching::app::app_io::Result result;
  CSR G;
  int numThreads, best;

  G.readMtxG(const_cast<char *>(h.file_path().c_str()));

  int *b = new int[G.nVer];
  Node *S = new Node[G.nVer]; // Heap data structure
#pragma omp parallel
  numThreads = omp_get_num_threads();

  /************ Assigning bValues **************/
  for (int i = 0; i < G.nVer; i++) {
    if (run_config.capacity() > 0) {
      int card = 0;
      for (int j = G.verPtr[i]; j < G.verPtr[i + 1]; j++)
        if (G.verInd[j].weight > 0)
          card++;

      if (card > run_config.capacity()) {
        b[i] = run_config.capacity();
      } else {
        b[i] = card;
      }
    } else {
      std::cerr << "predefined weights are currently unsupported with pothen."
                << std::endl;
      exit(1);
    }
  }
  for (int i = 0; i < G.nVer; i++) {
    if (b[i] > 0) {
      S[i].heap = new Info[b[i]]; // Each heap of size b
    } else {
      S[i].heap = new Info[1]; // Each heap of size b
    }
  }
  auto t1_sys = std::chrono::system_clock::now();
  std::chrono::high_resolution_clock::time_point t1 =
      std::chrono::high_resolution_clock::now();
  bSuitor(
      &G, b, S, 1,
      false); // TODO(reinstaedtler): Change 1 argument to zero does not work
  std::chrono::high_resolution_clock::time_point t2 =
      std::chrono::high_resolution_clock::now();
  auto t2_sys = std::chrono::system_clock::now();

  std::tuple<bool, int, int> bmatching = MatchingResults(&G, S, G.nVer);
  if (!std::get<0>(bmatching)) {
    std::cerr << h.file_path() << " " << run_config.capacity() << std::endl;
    std::cerr << "Not a matching." << std::endl;
    exit(1);
  }
  result.set_weight(std::get<1>(bmatching));
  result.set_size(std::get<2>(bmatching));
  // TODO(reinstaedtler): Update to exact timing
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

  for (int i = 0; i < G.nVer; i++)
    delete S[i].heap;
  delete[] S;

  return result;
}
} // namespace HeiHGM::BMatching::third_party::bsuitor