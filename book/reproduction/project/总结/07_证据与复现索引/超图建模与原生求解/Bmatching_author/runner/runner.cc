
#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "app/algorithms/algorithm_impl.h"
#include "app/app_io.pb.h"
#include "easylogging++.h"
#include "ncurses.h"
#include "third_party/external_graph_bmatching.h"
#include "utils/random.h"
#include <chrono>
#include <fstream>
#include <google/protobuf/text_format.h>
#include <iostream>
#include <sstream>
#ifdef HeiHGM::BMatching_MEMORY__REQ_CHECK
#include "utils/systeminfo.h"
#include <sys/sysinfo.h>
#endif
#include <thread>

ABSL_FLAG(std::string, experiment_path, "",
          "The path to the experiment including definitions file.");
ABSL_FLAG(bool, ncurses, false, "If ncurses should be enabled");
ABSL_FLAG(bool, random_order, false,
          "If experiments should be randomly sorted.");
#ifdef HeiHGM::BMatching_MEMORY__REQ_CHECK
ABSL_FLAG(long, min_free_memory, 1024,
          "Memory that should be free at least before starting work in a "
          "thread (in MegaByte).");
ABSL_FLAG(double, max_alloc_memory, -1,
          "Max memory allocated to this process in total in MB. runner will "
          "only queue as long as this number is not used. This overrides "
          "--min_free_memory.");
ABSL_FLAG(double, default_sleep_time, 1000,
          "Time to wait between execution for memory measurement (ms).");
ABSL_FLAG(double, sleep_time_violated, 10000,
          "Time to wait between polling memory info (ms).");
#endif

namespace {
using HeiHGM::BMatching::app::app_io::ExperimentConfig;
using HeiHGM::BMatching::app::app_io::ExperimentResultPart;
using HeiHGM::BMatching::app::app_io::Hypergraph;
using HeiHGM::BMatching::app::app_io::Result;
using HeiHGM::BMatching::app::app_io::RunConfig;

} // namespace
  // needed for libtcmalloc and profiler
char *program_invocation_name = "runner";
INITIALIZE_EASYLOGGINGPP

int main(int argc, char **argv) {
  using namespace std ::chrono_literals;

  absl::ParseCommandLine(argc, argv);
  std::string exp_path = absl::GetFlag(FLAGS_experiment_path);
  bool ncurses_enabled = absl::GetFlag(FLAGS_ncurses);
  bool random_order_enabled = absl::GetFlag(FLAGS_random_order);
#ifdef HeiHGM::BMatching_MEMORY__REQ_CHECK
  const long min_memory = absl::GetFlag(FLAGS_min_free_memory) * 1024 * 1024;
  const double max_memory_allc = absl::GetFlag(FLAGS_max_alloc_memory);
  const auto default_sleep = absl::GetFlag(FLAGS_default_sleep_time) * 1.0ms;
  const auto sleep_time_violated =
      absl::GetFlag(FLAGS_sleep_time_violated) * 1.0ms;

#else
  const long min_memory = 0;
  double max_memory_allc, default_sleep, sleep_time_violated;
  std::cerr << "WARNING: compiled without memory check. Please be carefull!"
            << std::endl;
#endif
  if (exp_path == "") {
    std::cerr << "Please specify a path." << std::endl;
    return 1;
  }
  std::cout << "Path: " << exp_path << std::endl;
  std::ifstream exp_file(exp_path + "/experiment.textproto");
  if (!exp_file.good()) {
    std::cerr << "The 'experiment.textproto' must exist." << std::endl;
    return 1;
  }
  std::stringstream buffer;
  buffer << exp_file.rdbuf();

  ExperimentConfig experiment_config;
  if (!google::protobuf::TextFormat::ParseFromString(buffer.str(),
                                                     &experiment_config)) {
    std::cerr << "Error parsing 'experiment.textproto'." << std::endl;
    return 1;
  }
  std::cout << " Starting experiment with name: '"
            << experiment_config.experiment_name() << "'" << std::endl;
  std::cout << experiment_config.hypergraphs_size()
            << " hypergraphs in this experiment." << std::endl;
  std::cout << experiment_config.run_configs_size()
            << " run_configs in this experiment." << std::endl;
  std::mutex io_output, exp_config_access, array_access;
  if (ncurses_enabled) {
    WINDOW *win = initscr();
  }
  std::vector<std::thread> waiters;
  std::atomic<int> iterator = -1;
  std::atomic<int> running_processes = 0;
  int last_commit = 0;
  const int run_config_size = experiment_config.run_configs_size();
  std::vector<size_t> id_order(experiment_config.hypergraphs_size() *
                               experiment_config.run_configs_size() *
                               experiment_config.repetitions());
  const auto conc_procs = experiment_config.concurrent_processes();
  std::iota(id_order.begin(), id_order.end(), 0);
  if (random_order_enabled) {
    std::cout << "Shuffling " << id_order.size() << " elements." << std::endl;
    HeiHGM::BMatching::utils::Randomize::instance().shuffleVector(
        id_order, id_order.size());
  }
  for (int i = 0; i < conc_procs; i++) {
    waiters.push_back(std::thread([i, &iterator, &io_output, &experiment_config,
                                   &array_access, &last_commit, exp_path,
                                   run_config_size, ncurses_enabled, &id_order,
                                   min_memory, max_memory_allc, default_sleep,
                                   sleep_time_violated, &running_processes,
                                   conc_procs]() {
      {
        std::stringstream s;
        s << i << "/" << conc_procs << "/" << running_processes
          << ": Warming up.";
        if (ncurses_enabled) {
          mvprintw(i + 3, 2, s.str().c_str());
        } else {
          std::lock_guard<std::mutex> lock(io_output);
          std::cout << s.str() << std::endl;
        }
      }
      if (max_memory_allc > 0) {
        // Delayed starting
        do {
          auto timeout = i * (running_processes + 1) * 5s;
          {
            std::lock_guard<std::mutex> lock(io_output);
            std::cout << i << "/" << conc_procs << "/" << running_processes
                      << " sleep for " << timeout.count() << "s" << std::endl;
          }
          std::this_thread::sleep_for(timeout);

        } while (running_processes < 4 && running_processes != 0);
        {
          std::lock_guard<std::mutex> lock(io_output);
          std::cout << i << "/" << conc_procs << "/" << running_processes
                    << " awakes." << std::endl;
        }
      }
      std::vector<Result> results;
      while (true) {
        Hypergraph current;
        RunConfig run_config;
        int curr_iter = 0;
        {
          std::lock_guard<std::mutex> lock(array_access);
          iterator++;
          if (iterator >= (experiment_config.hypergraphs_size() *
                           experiment_config.run_configs_size() *
                           experiment_config.repetitions())) {
            std::lock_guard<std::mutex> lock2(io_output);
            std::stringstream s;
            s << i << "/" << conc_procs << "/" << running_processes
              << ": Stopped successfully.";
            if (ncurses_enabled) {
              mvaddstr(i + 3, 2, s.str().c_str());
            } else {
              std::cout << s.str() << std::endl;
            }
            break;
          }
          // run experiments at the same time
          curr_iter = id_order[iterator] / experiment_config.repetitions();
          current =
              experiment_config.hypergraphs()[curr_iter / run_config_size];
          run_config =
              experiment_config.run_configs()[curr_iter % run_config_size];
        }
        {
          std::stringstream s;
          s << i << "/" << conc_procs << "/" << running_processes << " ["
            << iterator << "]: " << current.name() << " ["
            << run_config.short_name() << " " << run_config.capacity() << "]";
          std::lock_guard<std::mutex> lock2(io_output);
          if (ncurses_enabled) {
            mvaddstr(i + 3, 2, s.str().c_str());
          } else {
            std::cout << s.str() << std::endl;
          }
        }
        // Do some work
        // adjust file path
        *current.mutable_file_path() = absl::StrCat(
            experiment_config.root_path(), "/", current.file_path());
        absl::StatusOr<Result> result;
#ifdef HeiHGM::BMatching_MEMORY__REQ_CHECK
        if (max_memory_allc < 0) {
          struct sysinfo is;
          while (sysinfo(&is) >= 0 && is.freeram < min_memory) {
            {
              std::lock_guard<std::mutex> lock(io_output);

              std::cout << i << "/" << conc_procs
                        << " is waiting for memory. Free " << is.freeram
                        << " of " << is.totalram << " Expecting " << min_memory
                        << std::endl;
            }
            using namespace std::chrono_literals;
            std::this_thread::sleep_for(2000ms);
          }
        } else {
          std::this_thread::sleep_for(
              default_sleep * (rand() % 12 + 1)); // randomize to read in file
          auto mem_usage =
              HeiHGM::BMatching::utils::systeminfo::process_mem_usage();
          auto projected_mem =
              (mem_usage / running_processes) *
              // do project more memory if small number running
              (running_processes < 4 ? running_processes * 3
                                     : (running_processes + 2));
          while (max_memory_allc <= mem_usage ||
                 (running_processes > 0 && projected_mem >= max_memory_allc)) {
            std::lock_guard<std::mutex> lock(io_output);
            std::cout << i << "/" << conc_procs << "/" << running_processes
                      << ":  All'cd " << mem_usage << " of allowed "
                      << max_memory_allc << " Running: " << running_processes
                      << " Per process: " << mem_usage / running_processes
                      << " Projected: "
                      << projected_mem // project two processes to run
                      << std::endl;
            std::this_thread::sleep_for(sleep_time_violated);
            mem_usage =
                HeiHGM::BMatching::utils::systeminfo::process_mem_usage();
            projected_mem = (mem_usage / running_processes) *
                            // do project more memory if small number running
                            (running_processes < 4 ? running_processes * 3
                                                   : (running_processes + 2));
          }
        }
#endif
        running_processes++;
        if (!run_config.is_external()) {
          result = HeiHGM::BMatching::app::algorithms::Run(current, run_config,
                                                           false);
        } else {
          result = HeiHGM::BMatching::third_party::Run(current, run_config);
        }
        running_processes--;
        // save result
        if (!result.ok()) {
          std::lock_guard<std::mutex> lock(io_output);
          std::lock_guard<std::mutex> lock2(array_access);
          std::cerr << "Failure in task #" << curr_iter << "("
                    << result.status() << "," << current.file_path() << " "
                    << run_config.short_name() << " " << run_config.capacity()
                    << ")" << std::endl;
          std::cerr << "Last commit in task #" << last_commit << std::endl;
          std::cerr << "Call with --resume " << last_commit
                    << " to resume this work." << std::endl;
          iterator = std::numeric_limits<int>::max();
        } else {
          results.push_back(result.value());
        }
        // saving part;
        ExperimentResultPart part;
        part.set_concurrent_processes(conc_procs);
        part.set_this_process(i);
        *part.mutable_results() = {results.begin(), results.end()};
        *part.mutable_experiment_name() = experiment_config.experiment_name();
        std::stringstream file_name_stream;
        file_name_stream << exp_path << "/result-" << i << "-" << conc_procs
                         << ".binary_proto";
        std::ofstream output(file_name_stream.str());
        part.SerializeToOstream(&output);
      }
      // saving part;
      ExperimentResultPart part;
      part.set_concurrent_processes(conc_procs);
      part.set_this_process(i);
      *part.mutable_results() = {results.begin(), results.end()};
      *part.mutable_experiment_name() = experiment_config.experiment_name();
      std::stringstream file_name_stream;
      file_name_stream << exp_path << "/result-" << i << "-" << conc_procs
                       << ".binary_proto";
      std::ofstream output(file_name_stream.str());
      part.SerializeToOstream(&output);
    }));
  }
  std::vector<char> list = {'X', 'O', '%', 'O'};
  int j = 0;
  while (iterator < (experiment_config.hypergraphs_size() *
                     experiment_config.run_configs_size() *
                     experiment_config.repetitions())) {
    {
      std::stringstream percent_bar;
      float percent =
          ((float)iterator) / ((float)experiment_config.hypergraphs_size() *
                               experiment_config.run_configs_size() *
                               experiment_config.repetitions());
      for (int i = 0; i < percent * 80; i++) {
        percent_bar << (i % 2 ? "X" : "O");
      }
      j = (j + 1) % 4;
      percent_bar << list[j];
      std::lock_guard<std::mutex> lock2(io_output);
      if (ncurses_enabled) {
        mvaddstr(1, 2, percent_bar.str().c_str());
        int val = iterator;
        mvprintw(2, 2, "%d", val);
      }
    }
    if (ncurses_enabled) {
      refresh();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

  for (auto &t : waiters) {
    t.join();
  }
  endwin();
}