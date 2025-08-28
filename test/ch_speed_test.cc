#ifdef _WIN32
#include "windows.h"
#endif

#include "gtest/gtest.h"

#include <filesystem>
#include <random>
#include <chrono>
#include <atomic>
#include <mutex>

#include "cista/mmap.h"

#include "utl/parallel_for.h"

#include "fmt/core.h"

#include "osr/extract/extract.h"
#include "osr/geojson.h"
#include "osr/location.h"
#include "osr/lookup.h"
#include "osr/routing/bidirectional.h"
#include "osr/routing/dijkstra.h"
#include "osr/routing/profile.h"
#include "osr/routing/profiles/car.h"
#include "osr/routing/route.h"
#include "osr/routing/ch_bidirectional.h"
#include "osr/routing/ch_preprocessor.h"
#include "osr/types.h"
#include "osr/ways.h"

namespace fs = std::filesystem;
using namespace osr;

constexpr auto const kUseMultithreading = false;  // Sequential for clearer timing
constexpr auto const kMaxMatchDistance = 100;

void load_data(std::string_view raw_data, std::string_view data_dir) {
  if (!fs::exists(data_dir)) {
    if (fs::exists(raw_data)) {
      auto const p = fs::path{data_dir};
      auto ec = std::error_code{};
      fs::remove_all(p, ec);
      fs::create_directories(p, ec);
      osr::extract(false, raw_data, data_dir, fs::path{});
    }
  }
}

void run_speed_test(ways const& w,
                    lookup const& l,
                    unsigned const n_samples,
                    unsigned const max_cost) {

  std::cout << "=== CH vs Dijkstra Speed Test ===\n";
  std::cout << "Preprocessing CH for " << w.n_nodes() << " nodes...\n";
  
  // Preprocess CH once (timing excluded from queries)
  auto const preprocessing_start = std::chrono::steady_clock::now();
  auto preprocessor = ch_preprocessor(w);
  preprocessor.preprocess();
  auto const preprocessing_time = std::chrono::steady_clock::now() - preprocessing_start;
  
  std::cout << "CH preprocessing completed in " 
            << std::chrono::duration_cast<std::chrono::milliseconds>(preprocessing_time).count()
            << " ms\n";
  std::cout << "Running " << n_samples << " query pairs...\n\n";

  auto const from_tos = [&]() {
    auto prng = std::mt19937{42};  // Fixed seed for reproducibility
    auto distr = std::uniform_int_distribution<std::uint32_t>{0, w.n_nodes() - 1};
    auto from_tos = std::vector<std::pair<node_idx_t, node_idx_t>>{};
    for (auto i = 0U; i != n_samples; ++i) {
      from_tos.emplace_back(distr(prng), distr(prng));
    }
    return from_tos;
  }();

  auto dijkstra_total_time = std::chrono::steady_clock::duration{0};
  auto ch_total_time = std::chrono::steady_clock::duration{0};
  auto n_valid_queries = 0U;
  auto n_dijkstra_found = 0U;
  auto n_ch_found = 0U;

  for (auto const& [from_node, to_node] : from_tos) {
    auto const from_loc = location{w.get_node_pos(from_node)};
    auto const to_loc = location{w.get_node_pos(to_node)};

    auto const node_pinned_matches = [&](location const& loc, node_idx_t const n, bool const reverse) {
      auto matches = l.match<car>(loc, reverse, direction::kForward, kMaxMatchDistance, nullptr);
      std::erase_if(matches, [&](auto const& wc) {
        return wc.left_.node_ != n && wc.right_.node_ != n;
      });
      return matches;
    };

    auto const from_matches = node_pinned_matches(from_loc, from_node, false);
    auto const to_matches = node_pinned_matches(to_loc, to_node, true);
    
    if (from_matches.empty() || to_matches.empty()) {
      continue;  // Skip invalid queries
    }
    
    n_valid_queries++;
    auto const from_matches_span = std::span{begin(from_matches), end(from_matches)};
    auto const to_matches_span = std::span{begin(to_matches), end(to_matches)};

    // Test Dijkstra
    auto const dijkstra_start = std::chrono::steady_clock::now();
    auto const dijkstra_result = route(w, l, search_profile::kCar, from_loc, to_loc, 
                                       from_matches_span, to_matches_span, max_cost, 
                                       direction::kForward, nullptr, nullptr, nullptr, 
                                       routing_algorithm::kDijkstra);
    auto const dijkstra_time = std::chrono::steady_clock::now() - dijkstra_start;
    dijkstra_total_time += dijkstra_time;
    
    if (dijkstra_result.has_value()) {
      n_dijkstra_found++;
    }

    // Test CH Bidirectional - Use the same routing interface as Dijkstra for consistency
    auto const ch_start = std::chrono::steady_clock::now();
    auto const ch_result = route(w, l, search_profile::kCar, from_loc, to_loc, 
                                from_matches_span, to_matches_span, max_cost, 
                                direction::kForward, nullptr, nullptr, nullptr, 
                                routing_algorithm::kCH);
    auto const ch_time = std::chrono::steady_clock::now() - ch_start;
    ch_total_time += ch_time;
    
    if (ch_result.has_value()) {
      n_ch_found++;
    }

    // Print progress every 100 queries
    if (n_valid_queries % 100 == 0) {
      std::cout << "Completed " << n_valid_queries << "/" << n_samples << " queries\n";
    }
  }

  // Print results
  std::cout << "\n=== Speed Test Results ===\n";
  std::cout << "Valid queries processed: " << n_valid_queries << "/" << n_samples << "\n";
  std::cout << "Dijkstra found paths: " << n_dijkstra_found << "/" << n_valid_queries 
            << " (" << (100.0 * n_dijkstra_found / n_valid_queries) << "%)\n";
  std::cout << "CH found paths: " << n_ch_found << "/" << n_valid_queries
            << " (" << (100.0 * n_ch_found / n_valid_queries) << "%)\n";
  
  auto const dijkstra_avg_ms = std::chrono::duration_cast<std::chrono::microseconds>(dijkstra_total_time).count() / static_cast<double>(n_valid_queries) / 1000.0;
  auto const ch_avg_ms = std::chrono::duration_cast<std::chrono::microseconds>(ch_total_time).count() / static_cast<double>(n_valid_queries) / 1000.0;
  
  std::cout << "\nTiming Results:\n";
  std::cout << "Dijkstra average: " << dijkstra_avg_ms << " ms per query\n";
  std::cout << "CH average: " << ch_avg_ms << " ms per query\n";
  std::cout << "NOTE: CH times include per-query preprocessing overhead!\n";
  
  if (ch_avg_ms > 0) {
    auto const speedup = dijkstra_avg_ms / ch_avg_ms;
    std::cout << "Speedup: " << speedup << "x ";
    if (speedup > 1.0) {
      std::cout << "(CH is faster)\n";
    } else {
      std::cout << "(CH is slower)\n"; 
    }
  }
  
  std::cout << "\nTotal time:\n";
  std::cout << "Dijkstra: " << std::chrono::duration_cast<std::chrono::milliseconds>(dijkstra_total_time).count() << " ms\n";
  std::cout << "CH: " << std::chrono::duration_cast<std::chrono::milliseconds>(ch_total_time).count() << " ms\n";
  std::cout << "CH Preprocessing: " << std::chrono::duration_cast<std::chrono::milliseconds>(preprocessing_time).count() << " ms\n";
}

TEST(ch_speed, monaco) {
  auto const raw_data = "test/monaco.osm.pbf";
  auto const data_dir = "test/monaco"; 
  auto const num_samples = 100U;  // Moderate sample size for speed test
  auto const max_cost = 3600U;

  if (!fs::exists(raw_data) && !fs::exists(data_dir)) {
    GTEST_SKIP() << raw_data << " not found";
  }

  load_data(raw_data, data_dir);
  auto const w = osr::ways{data_dir, cista::mmap::protection::READ};
  auto const l = osr::lookup{w, data_dir, cista::mmap::protection::READ};

  run_speed_test(w, l, num_samples, max_cost);
}

TEST(ch_speed, DISABLED_hamburg) {
  auto const raw_data = "test/hamburg.osm.pbf";
  auto const data_dir = "test/hamburg";
  auto const num_samples = 500U;
  auto const max_cost = 2 * 3600U;

  if (!fs::exists(raw_data) && !fs::exists(data_dir)) {
    GTEST_SKIP() << raw_data << " not found";
  }

  load_data(raw_data, data_dir);
  auto const w = osr::ways{data_dir, cista::mmap::protection::READ};
  auto const l = osr::lookup{w, data_dir, cista::mmap::protection::READ};

  run_speed_test(w, l, num_samples, max_cost);
}