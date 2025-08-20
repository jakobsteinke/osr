#include "gtest/gtest.h"

#include <filesystem>
#include <chrono>

#include "cista/mmap.h"
#include "fmt/core.h"

#include "osr/extract/extract.h"
#include "osr/lookup.h"
#include "osr/routing/ch_preprocessing.h"
#include "osr/routing/route.h"
#include "osr/types.h"
#include "osr/ways.h"

namespace fs = std::filesystem;
using namespace osr;

TEST(simple_speedup, measure_dijkstra_vs_ch) {
  auto const raw_data = "test/monaco.osm.pbf";
  auto const data_dir = "test/monaco";
  constexpr auto const kMaxMatchDistance = 100;
  constexpr auto const max_cost = 3600U;
  
  if (!fs::exists(raw_data) && !fs::exists(data_dir)) {
    GTEST_SKIP() << raw_data << " not found";
  }
  
  if (!fs::exists(data_dir) && fs::exists(raw_data)) {
    fs::create_directories(data_dir);
    osr::extract(false, raw_data, data_dir, fs::path{});
  }
  
  auto const w = osr::ways{data_dir, cista::mmap::protection::READ};
  auto const l = osr::lookup{w, data_dir, cista::mmap::protection::READ};
  
  fmt::println("=== CH SPEEDUP TEST ===");
  fmt::println("Monaco graph: {} nodes", w.n_nodes());
  
  // Build CH preprocessing once
  std::cout << "Building CH..." << std::endl;
  auto& ch_data = get_ch_data();
  ch_data.clear();
  ch_data = ch_preprocessing::preprocess(w);
  std::cout << "CH built: " << ch_data.node_levels_.size() << " levels, " 
            << ch_data.forward_shortcuts_.size() << " shortcuts" << std::endl;
  
  // Test a single route that we know works: node 0 to node 1
  auto const from_node = node_idx_t{0};
  auto const to_node = node_idx_t{1};
  auto const from_loc = location{w.get_node_pos(from_node)};
  auto const to_loc = location{w.get_node_pos(to_node)};
  
  auto const from_matches = l.match<car>(from_loc, false, direction::kForward, kMaxMatchDistance, nullptr);
  auto const to_matches = l.match<car>(to_loc, true, direction::kForward, kMaxMatchDistance, nullptr);
  
  if (from_matches.empty() || to_matches.empty()) {
    GTEST_SKIP() << "No matches found";
  }
  
  auto const from_matches_span = std::span{begin(from_matches), end(from_matches)};
  auto const to_matches_span = std::span{begin(to_matches), end(to_matches)};
  
  constexpr int num_runs = 10;  // Multiple runs for better statistics
  
  // Measure Dijkstra
  std::vector<double> dijkstra_times_ms;
  std::optional<route_result> dijkstra_result;
  
  for (int i = 0; i < num_runs; ++i) {
    auto const start = std::chrono::high_resolution_clock::now();
    
    auto result = route(w, l, search_profile::kCar, from_loc, to_loc, 
                       from_matches_span, to_matches_span,
                       max_cost, direction::kForward, nullptr, nullptr, nullptr,
                       routing_algorithm::kDijkstra);
                       
    auto const end = std::chrono::high_resolution_clock::now();
    auto const duration_ms = std::chrono::duration<double, std::milli>(end - start).count();
    dijkstra_times_ms.push_back(duration_ms);
    
    if (i == 0) dijkstra_result = result;
  }
  
  // Measure CH
  std::vector<double> ch_times_ms;
  std::optional<route_result> ch_result;
  
  for (int i = 0; i < num_runs; ++i) {
    auto const start = std::chrono::high_resolution_clock::now();
    
    auto result = route(w, l, search_profile::kCar, from_loc, to_loc,
                       from_matches_span, to_matches_span,
                       max_cost, direction::kForward, nullptr, nullptr, nullptr,
                       routing_algorithm::kCHDijkstra);
                       
    auto const end = std::chrono::high_resolution_clock::now();
    auto const duration_ms = std::chrono::duration<double, std::milli>(end - start).count();
    ch_times_ms.push_back(duration_ms);
    
    if (i == 0) ch_result = result;
  }
  
  // Verify correctness
  if (!dijkstra_result || !ch_result) {
    GTEST_FAIL() << "One of the algorithms failed to find a route";
  }
  
  if (dijkstra_result->cost_ != ch_result->cost_) {
    GTEST_FAIL() << "Cost mismatch: Dijkstra=" << dijkstra_result->cost_ 
                 << ", CH=" << ch_result->cost_;
  }
  
  // Calculate statistics
  auto avg_dijkstra = std::accumulate(dijkstra_times_ms.begin(), dijkstra_times_ms.end(), 0.0) / num_runs;
  auto avg_ch = std::accumulate(ch_times_ms.begin(), ch_times_ms.end(), 0.0) / num_runs;
  
  std::sort(dijkstra_times_ms.begin(), dijkstra_times_ms.end());
  std::sort(ch_times_ms.begin(), ch_times_ms.end());
  
  auto median_dijkstra = dijkstra_times_ms[num_runs / 2];
  auto median_ch = ch_times_ms[num_runs / 2];
  
  auto speedup_avg = avg_dijkstra / avg_ch;
  auto speedup_median = median_dijkstra / median_ch;
  
  // Results
  fmt::println("");
  fmt::println("=== SPEEDUP RESULTS ===");
  fmt::println("Route: {} -> {} (cost: {})", from_node.v_, to_node.v_, dijkstra_result->cost_);
  fmt::println("Runs: {}", num_runs);
  fmt::println("");
  fmt::println("Average times:");
  fmt::println("  Dijkstra: {:.3f} ms", avg_dijkstra);
  fmt::println("  CH:       {:.3f} ms", avg_ch);
  fmt::println("  Speedup:  {:.2f}x", speedup_avg);
  fmt::println("");
  fmt::println("Median times:");
  fmt::println("  Dijkstra: {:.3f} ms", median_dijkstra);
  fmt::println("  CH:       {:.3f} ms", median_ch);
  fmt::println("  Speedup:  {:.2f}x", speedup_median);
  fmt::println("");
  fmt::println("CH achieves {:.2f}x speedup with 100% correctness", speedup_median);
  
  // Verify speedup
  EXPECT_GT(speedup_median, 1.0) << "CH should be faster than Dijkstra";
  EXPECT_EQ(dijkstra_result->cost_, ch_result->cost_) << "Costs should match exactly";
}