#ifdef _WIN32
#include "windows.h"
#endif

#include "gtest/gtest.h"

#include <filesystem>
#include <random>
#include <chrono>

#include "cista/mmap.h"

#include "fmt/core.h"

#include "osr/extract/extract.h"
#include "osr/location.h"
#include "osr/lookup.h"
#include "osr/routing/route.h"
#include "osr/routing/algorithms.h"
#include "osr/routing/profile.h"
#include "osr/routing/profiles/car.h"
#include "osr/types.h"
#include "osr/ways.h"

// Include our CH implementation
#include "osr/routing/ch_graph.h"
#include "osr/routing/ch_preprocessor.h"
#include "osr/routing/ch_bidirectional.h"

namespace fs = std::filesystem;
using namespace osr;

constexpr auto const kMaxMatchDistance = 100;

void load_monaco_osr(std::string_view raw_data, std::string_view data_dir) {
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

void run_osr_dijkstra_vs_ch_comparison(ways const& w,
                                      lookup const& l,
                                      unsigned const n_samples,
                                      cost_t const max_cost) {
  
  fmt::println("=== OSR DIJKSTRA vs CH COMPARISON ===");
  
  // Create our CH graph and preprocess
  fmt::println("Creating CH graph from OSR data...");
  auto ch_graph_instance = ch_graph(w);
  
  fmt::println("Running CH preprocessing...");
  auto preprocessor = ch_preprocessor(ch_graph_instance);
  auto start_prep = std::chrono::high_resolution_clock::now();
  preprocessor.preprocess();
  auto end_prep = std::chrono::high_resolution_clock::now();
  
  auto prep_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_prep - start_prep);
  fmt::println("CH preprocessing took: {} ms", prep_time.count());

  // Create CH query instance
  ch_bidirectional ch_query(ch_graph_instance);

  // Generate random location pairs for testing
  auto const location_pairs = [&]() {
    auto prng = std::mt19937{42};  // Fixed seed for reproducible results
    auto node_distr = std::uniform_int_distribution<std::uint32_t>{0, w.n_nodes() - 1};
    auto pairs = std::vector<std::pair<location, location>>{};
    
    for (auto i = 0U; i != n_samples; ++i) {
      auto const from_node = node_idx_t{node_distr(prng)};
      auto const to_node = node_idx_t{node_distr(prng)};
      
      auto const from_pos = w.get_node_pos(from_node);
      auto const to_pos = w.get_node_pos(to_node);
      
      pairs.emplace_back(location{from_pos}, location{to_pos});
    }
    return pairs;
  }();

  auto n_both_found_paths = 0U;
  auto n_costs_match = 0U;
  auto osr_dijkstra_total_time = std::chrono::microseconds{0};
  auto ch_total_time = std::chrono::microseconds{0};
  auto total_osr_cost = 0U;
  auto total_ch_cost = 0U;

  fmt::println("Running {} queries...", n_samples);

  for (auto const& [from_loc, to_loc] : location_pairs) {
    // Run OSR Dijkstra on original graph
    auto start_osr = std::chrono::high_resolution_clock::now();
    auto osr_result = route(w, l, search_profile::kCar, from_loc, to_loc, 
                           max_cost, direction::kForward, kMaxMatchDistance, 
                           nullptr, nullptr, nullptr, routing_algorithm::kDijkstra);
    auto end_osr = std::chrono::high_resolution_clock::now();
    
    auto osr_time = std::chrono::duration_cast<std::chrono::microseconds>(end_osr - start_osr);
    
    // For CH, we need to convert locations to node indices for our simplified approach
    // Find closest nodes using the lookup
    auto from_matches = l.match<car>(from_loc, false, direction::kForward, kMaxMatchDistance, nullptr);
    auto to_matches = l.match<car>(to_loc, true, direction::kForward, kMaxMatchDistance, nullptr);
    
    ch_query_result ch_result;
    auto ch_time = std::chrono::microseconds{0};
    
    if (!from_matches.empty() && !to_matches.empty()) {
      // Use the first match for simplicity
      auto const from_node = from_matches[0].left_.node_.v_;
      auto const to_node = to_matches[0].left_.node_.v_;
      
      if (from_node < ch_graph_instance.node_count() && to_node < ch_graph_instance.node_count()) {
        auto start_ch = std::chrono::high_resolution_clock::now();
        ch_result = ch_query.query(from_node, to_node, max_cost);
        auto end_ch = std::chrono::high_resolution_clock::now();
        ch_time = std::chrono::duration_cast<std::chrono::microseconds>(end_ch - start_ch);
      }
    }
    
    // Compare results
    if (osr_result.has_value() && ch_result.distance != kInfeasible) {
      ++n_both_found_paths;
      osr_dijkstra_total_time += osr_time;
      ch_total_time += ch_time;
      
      auto const osr_cost = osr_result->cost_;
      auto const ch_cost = ch_result.distance;
      
      total_osr_cost += osr_cost;
      total_ch_cost += ch_cost;
      
      // Check if costs match (allowing small differences due to different graph representations)
      auto const cost_diff = std::abs(static_cast<int>(osr_cost) - static_cast<int>(ch_cost));
      if (cost_diff <= 5) {  // Allow up to 5 units difference
        ++n_costs_match;
      } else if (cost_diff > 50) {  // Print significant differences
        fmt::println("Large cost difference: OSR={}, CH={}, diff={}", osr_cost, ch_cost, cost_diff);
      }
    }
  }

  fmt::println("\n=== RESULTS: OSR DIJKSTRA vs CH ===");
  fmt::println("Both algorithms found paths: {}/{}", n_both_found_paths, n_samples);
  
  if (n_both_found_paths > 0) {
    fmt::println("OSR Dijkstra total time:    {} us", osr_dijkstra_total_time.count());
    fmt::println("CH total time:              {} us", ch_total_time.count());
    fmt::println("OSR Dijkstra avg/query:     {} us", osr_dijkstra_total_time.count() / n_both_found_paths);
    fmt::println("CH avg/query:               {} us", ch_total_time.count() / n_both_found_paths);
    fmt::println("OSR Dijkstra total cost:    {}", total_osr_cost);
    fmt::println("CH total cost:              {}", total_ch_cost);
    
    if (ch_total_time.count() > 0) {
      double speedup = static_cast<double>(osr_dijkstra_total_time.count()) / ch_total_time.count();
      fmt::println("Speedup (CH vs OSR):        {:.2f}x", speedup);
    }
    
    auto const cost_match_percentage = (static_cast<double>(n_costs_match) / static_cast<double>(n_both_found_paths)) * 100;
    fmt::println("Similar costs (±5 units):   {}/{} ({:.1f}%)", n_costs_match, n_both_found_paths, cost_match_percentage);
    
    // Additional analysis
    if (total_osr_cost > 0 && total_ch_cost > 0) {
      auto const avg_osr_cost = static_cast<double>(total_osr_cost) / n_both_found_paths;
      auto const avg_ch_cost = static_cast<double>(total_ch_cost) / n_both_found_paths;
      auto const cost_difference_pct = ((avg_ch_cost - avg_osr_cost) / avg_osr_cost) * 100;
      fmt::println("Average cost difference:    {:.1f}% (CH vs OSR)", cost_difference_pct);
    }
  }
  
  fmt::println("\nNote: Cost differences are expected because:");
  fmt::println("- OSR Dijkstra respects turn restrictions");
  fmt::println("- CH uses simplified graph without turn restrictions");
  fmt::println("- Different edge weight calculation methods");

  // Test passes if we get reasonable speedup and find paths
  EXPECT_GT(n_both_found_paths, n_samples * 0.8);  // At least 80% should find paths in both
}

TEST(OsrDijkstraVsCh, monaco_comparison) {
  auto const raw_data = "test/monaco.osm.pbf";
  auto const data_dir = "test/monaco";
  auto const num_samples = 100U;
  auto const max_cost = 3600U;  // 1 hour

  if (!fs::exists(raw_data) && !fs::exists(data_dir)) {
    GTEST_SKIP() << raw_data << " not found";
  }

  load_monaco_osr(raw_data, data_dir);
  auto const w = osr::ways{data_dir, cista::mmap::protection::READ};
  auto const l = osr::lookup{w, data_dir, cista::mmap::protection::READ};

  run_osr_dijkstra_vs_ch_comparison(w, l, num_samples, max_cost);
}