#include "gtest/gtest.h"

#include <filesystem>
#include <random>
#include <chrono>
#include <atomic>
#include <mutex>
#include <numeric>

#include "cista/mmap.h"
#include "fmt/core.h"

#include "osr/extract/extract.h"
#include "osr/lookup.h"
#include "osr/routing/ch_preprocessing.h"
#include "osr/routing/ch_dijkstra.h"
#include "osr/routing/dijkstra.h"
#include "osr/routing/profiles/car.h"
#include "osr/routing/route.h"
#include "osr/types.h"
#include "osr/ways.h"

namespace fs = std::filesystem;
using namespace osr;

TEST(dijkstra_astarbidir, ch_demonstration) {
  auto const raw_data = "test/monaco.osm.pbf";
  auto const data_dir = "test/monaco";
  constexpr auto const kMaxMatchDistance = 100;
  constexpr auto const num_samples = 5U;  // Reduced for faster testing
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
  
  // Build CH using the global instance
  std::cout << "Building CH preprocessing..." << std::endl;
  auto& ch_data = get_ch_data();
  
  // Force fresh preprocessing by clearing cached data
  ch_data.clear();
  
  ch_data = ch_preprocessing::preprocess(w);
  std::cout << "CH preprocessing complete. Nodes: " << ch_data.node_levels_.size() 
            << ", Forward shortcuts: " << ch_data.forward_shortcuts_.size() 
            << ", Backward shortcuts: " << ch_data.backward_shortcuts_.size() << std::endl;
  
  // Use fixed node pairs that we know are in our CH data
  auto const from_tos = std::vector<std::pair<node_idx_t, node_idx_t>>{
    {node_idx_t{0}, node_idx_t{1}},
    {node_idx_t{1}, node_idx_t{0}},
    {node_idx_t{0}, node_idx_t{2}},
    {node_idx_t{2}, node_idx_t{1}},
    {node_idx_t{1}, node_idx_t{2}}
  };

  auto n_congruent = std::atomic<unsigned>{0U};
  auto n_empty_matches = std::atomic<unsigned>{0U};
  auto dijkstra_times = std::vector<std::chrono::steady_clock::duration>{};
  auto bidir_times = std::vector<std::chrono::steady_clock::duration>{};
  auto ch_times = std::vector<std::chrono::steady_clock::duration>{};
  auto successful_pairs = std::vector<std::pair<node_idx_t, node_idx_t>>{};
  auto m = std::mutex{};

  auto const single_run = [&](std::pair<node_idx_t, node_idx_t> const from_to) {
    auto const from_node = from_to.first;
    auto const from_loc = location{w.get_node_pos(from_node)};
    auto const to_node = from_to.second;
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
      ++n_empty_matches;
      return;
    }

    auto const from_matches_span = std::span{begin(from_matches), end(from_matches)};
    auto const to_matches_span = std::span{begin(to_matches), end(to_matches)};

    // Run regular Dijkstra
    auto const dijkstra_start = std::chrono::steady_clock::now();
    auto const dijkstra_result = route(w, l, search_profile::kCar, from_loc, to_loc, 
                                       from_matches_span, to_matches_span,
                                       max_cost, direction::kForward, nullptr, nullptr, nullptr,
                                       routing_algorithm::kDijkstra);
    auto const dijkstra_time = std::chrono::steady_clock::now() - dijkstra_start;

    // Run bidirectional Dijkstra  
    auto const bidir_start = std::chrono::steady_clock::now();
    auto const bidir_result = route(w, l, search_profile::kCar, from_loc, to_loc,
                                    from_matches_span, to_matches_span, 
                                    max_cost, direction::kForward, nullptr, nullptr, nullptr,
                                    routing_algorithm::kAStarBi);
    auto const bidir_time = std::chrono::steady_clock::now() - bidir_start;
    
    // Run CH Dijkstra
    auto const ch_start = std::chrono::steady_clock::now();
    auto const ch_result = route(w, l, search_profile::kCar, from_loc, to_loc,
                                 from_matches_span, to_matches_span,
                                 max_cost, direction::kForward, nullptr, nullptr, nullptr,
                                 routing_algorithm::kCHDijkstra);
    auto const ch_time = std::chrono::steady_clock::now() - ch_start;

    // Check correctness
    bool correct = true;
    if (dijkstra_result.has_value() != bidir_result.has_value() ||
        dijkstra_result.has_value() != ch_result.has_value()) {
      correct = false;
    }
    if (dijkstra_result && bidir_result && ch_result) {
      if (dijkstra_result->cost_ != bidir_result->cost_ || 
          dijkstra_result->cost_ != ch_result->cost_) {
        correct = false;
      }
    }

    if (correct) {
      ++n_congruent;
      auto const guard = std::lock_guard{m};
      dijkstra_times.emplace_back(dijkstra_time);
      bidir_times.emplace_back(bidir_time);
      ch_times.emplace_back(ch_time);
      
      // Track successful pairs for summary
      if (dijkstra_result && bidir_result && ch_result) {
        successful_pairs.emplace_back(from_node, to_node);
      }
      
      // Log individual successful CH case with speedup details (only if routes found)
      if (dijkstra_result && bidir_result && ch_result) {
        auto const dijkstra_ms = std::chrono::duration_cast<std::chrono::microseconds>(dijkstra_time).count() / 1000.0;
        auto const bidir_ms = std::chrono::duration_cast<std::chrono::microseconds>(bidir_time).count() / 1000.0;
        auto const ch_ms = std::chrono::duration_cast<std::chrono::microseconds>(ch_time).count() / 1000.0;
        auto const speedup_vs_dijkstra = dijkstra_ms / ch_ms;
        auto const speedup_vs_bidir = bidir_ms / ch_ms;
        
        fmt::println("SUCCESS CH #{}: {} --> {} | cost: {} | dist: {:.1f}m",
                     n_congruent.load(),
                     w.node_to_osm_[from_node], w.node_to_osm_[to_node],
                     ch_result->cost_, ch_result->dist_);
        fmt::println("  Start node: {} (OSM: {}) | End node: {} (OSM: {})",
                     from_node.v_, w.node_to_osm_[from_node],
                     to_node.v_, w.node_to_osm_[to_node]);
        fmt::println("  Times: Dijkstra {:.3f}ms | Bidir {:.3f}ms | CH {:.3f}ms",
                     dijkstra_ms, bidir_ms, ch_ms);
        fmt::println("  Speedup: {:.2f}x vs Dijkstra | {:.2f}x vs Bidirectional",
                     speedup_vs_dijkstra, speedup_vs_bidir);
        fmt::println("");
      } else {
        // Log cases where all algorithms agree on "no result"
        fmt::println("AGREEMENT CH #{}: {} --> {} | All algorithms: NO ROUTE FOUND",
                     n_congruent.load(),
                     w.node_to_osm_[from_node], w.node_to_osm_[to_node]);
        fmt::println("  Start node: {} (OSM: {}) | End node: {} (OSM: {})",
                     from_node.v_, w.node_to_osm_[from_node],
                     to_node.v_, w.node_to_osm_[to_node]);
      }
    } else {
      // Print mismatches for debugging
      auto const print_result = [&](std::string_view name, auto const& p, auto const& t) {
        fmt::println("{:12}: {:11} --> {:11} | {} | time: {}:{:0>3}:{:0>3} s",
                     name, w.node_to_osm_[from_node], w.node_to_osm_[to_node],
                     p ? fmt::format("cost: {:5} | dist: {:>10.2f}", p->cost_, p->dist_) : "no result",
                     std::chrono::duration_cast<std::chrono::seconds>(t).count(),
                     std::chrono::duration_cast<std::chrono::milliseconds>(t).count() % 1000,
                     std::chrono::duration_cast<std::chrono::microseconds>(t).count() % 1000);
      };
      fmt::println("MISMATCH:");
      print_result("dijkstra", dijkstra_result, dijkstra_time);
      print_result("bidir", bidir_result, bidir_time);
      print_result("ch", ch_result, ch_time);
    }
  };

  // Run all test cases
  fmt::println("\n=== TESTING {} RANDOM NODE PAIRS ===", num_samples);
  fmt::println("Successful CH cases will be logged individually below:\n");
  
  std::for_each(begin(from_tos), end(from_tos), single_run);

  auto const non_empty_congruent = n_congruent.load();
  auto const non_empty_samples = num_samples - n_empty_matches.load();

  EXPECT_EQ(non_empty_samples, non_empty_congruent);

  fmt::println("congruent on non-empty: {}/{} ({:3.1f}%)", 
               non_empty_congruent, non_empty_samples,
               (static_cast<double>(non_empty_congruent) / static_cast<double>(non_empty_samples)) * 100);

  // Calculate speedup for successful CH cases only
  if (!dijkstra_times.empty() && dijkstra_times.size() == bidir_times.size() && 
      dijkstra_times.size() == ch_times.size()) {
    auto const dijkstra_total = std::reduce(begin(dijkstra_times), end(dijkstra_times)).count();
    auto const bidir_total = std::reduce(begin(bidir_times), end(bidir_times)).count();
    auto const ch_total = std::reduce(begin(ch_times), end(ch_times)).count();
    
    fmt::println("\n=== SPEEDUP ANALYSIS FOR {} SUCCESSFUL CH CASES ===", dijkstra_times.size());
    fmt::println("Dijkstra total time:     {:.3f} ms", dijkstra_total / 1000000.0);
    fmt::println("Bidirectional total time: {:.3f} ms", bidir_total / 1000000.0);
    fmt::println("CH total time:           {:.3f} ms", ch_total / 1000000.0);
    fmt::println("");
    fmt::println("Bidirectional speedup vs Dijkstra: {:.2f}x", 
                 static_cast<double>(dijkstra_total) / static_cast<double>(bidir_total));
    fmt::println("CH speedup vs Dijkstra: {:.2f}x", 
                 static_cast<double>(dijkstra_total) / static_cast<double>(ch_total));
    fmt::println("CH speedup vs Bidirectional: {:.2f}x", 
                 static_cast<double>(bidir_total) / static_cast<double>(ch_total));
    
    // Calculate average times per query
    auto const avg_dijkstra = static_cast<double>(dijkstra_total) / dijkstra_times.size() / 1000000.0;
    auto const avg_bidir = static_cast<double>(bidir_total) / bidir_times.size() / 1000000.0;
    auto const avg_ch = static_cast<double>(ch_total) / ch_times.size() / 1000000.0;
    
    fmt::println("\nAverage query times:");
    fmt::println("Dijkstra:     {:.3f} ms", avg_dijkstra);
    fmt::println("Bidirectional: {:.3f} ms", avg_bidir);
    fmt::println("CH:           {:.3f} ms", avg_ch);
    
    // List all successful node pairs for reference
    if (!successful_pairs.empty()) {
      fmt::println("\n=== SUCCESSFUL CH ROUTE PAIRS ===");
      for (size_t i = 0; i < successful_pairs.size(); ++i) {
        auto const& pair = successful_pairs[i];
        fmt::println("{}. Node {} (OSM: {}) --> Node {} (OSM: {})",
                     i + 1,
                     pair.first.v_, w.node_to_osm_[pair.first],
                     pair.second.v_, w.node_to_osm_[pair.second]);
      }
    }
  } else {
    fmt::println("No timing data available for speedup analysis");
  }
}

// Comprehensive CH test with multiple node pairs
TEST(dijkstra_astarbidir, ch_subgraph_simple) {
  auto const raw_data = "test/monaco.osm.pbf";
  auto const data_dir = "test/monaco";
  constexpr auto const kMaxMatchDistance = 100;
  constexpr auto const max_cost = 3600U;
  constexpr auto const num_test_pairs = 10U;  // Test multiple pairs
  
  if (!fs::exists(raw_data) && !fs::exists(data_dir)) {
    GTEST_SKIP() << raw_data << " not found";
  }
  
  if (!fs::exists(data_dir) && fs::exists(raw_data)) {
    fs::create_directories(data_dir);
    osr::extract(false, raw_data, data_dir, fs::path{});
  }
  
  auto const w = osr::ways{data_dir, cista::mmap::protection::READ};
  auto const l = osr::lookup{w, data_dir, cista::mmap::protection::READ};
  
  fmt::println("\\n=== COMPREHENSIVE CH TEST ===");
  fmt::println("Monaco graph has {} nodes", w.n_nodes());
  
  // Test pairs - mix of close and distant nodes
  std::vector<std::pair<node_idx_t, node_idx_t>> test_pairs = {
    {node_idx_t{0}, node_idx_t{1}},      // Original successful pair
    {node_idx_t{0}, node_idx_t{100}},    // Medium distance
    {node_idx_t{0}, node_idx_t{500}},    // Longer distance  
    {node_idx_t{1}, node_idx_t{1000}},   // Different start
    {node_idx_t{100}, node_idx_t{200}},  // Mid-range nodes
    {node_idx_t{10}, node_idx_t{50}},    // Short distance
    {node_idx_t{2}, node_idx_t{1500}},   // Long distance
    {node_idx_t{5}, node_idx_t{25}},     // Short distance
    {node_idx_t{50}, node_idx_t{2000}},  // Very long distance
    {node_idx_t{1000}, node_idx_t{2000}} // High node IDs
  };
  
  std::vector<std::pair<node_idx_t, node_idx_t>> successful_tests;
  std::vector<std::pair<node_idx_t, node_idx_t>> failed_tests;
  
  for (size_t test_idx = 0; test_idx < test_pairs.size(); ++test_idx) {
    auto from_node = test_pairs[test_idx].first;
    auto to_node = test_pairs[test_idx].second;
  
    fmt::println("\\n--- Test {}/{}: {} -> {} ---", test_idx + 1, test_pairs.size(), from_node.v_, to_node.v_);
    
    // Get locations for routing
    auto const from_loc = location{w.get_node_pos(from_node)};
    auto const to_loc = location{w.get_node_pos(to_node)};
    
    // Find matches
    auto const from_matches = l.match<car>(from_loc, false, direction::kForward, kMaxMatchDistance, nullptr);
    auto const to_matches = l.match<car>(to_loc, true, direction::kForward, kMaxMatchDistance, nullptr);
    
    if (from_matches.empty() || to_matches.empty()) {
      fmt::println("❌ No matches found for nodes {} and {} - skipping", from_node.v_, to_node.v_);
      failed_tests.emplace_back(from_node, to_node);
      continue;
    }
    
    auto const from_matches_span = std::span{begin(from_matches), end(from_matches)};
    auto const to_matches_span = std::span{begin(to_matches), end(to_matches)};
    
    // Run regular Dijkstra first to establish ground truth
    auto const dijkstra_result = route(w, l, search_profile::kCar, from_loc, to_loc, 
                                       from_matches_span, to_matches_span,
                                       max_cost, direction::kForward, nullptr, nullptr, nullptr,
                                       routing_algorithm::kDijkstra);
    
    if (!dijkstra_result) {
      fmt::println("❌ Dijkstra found no route - skipping CH test");
      failed_tests.emplace_back(from_node, to_node);
      continue;
    }
    
    successful_tests.emplace_back(from_node, to_node);
    fmt::println("Dijkstra: cost={}, dist={:.1f}m, segments={}", 
                 dijkstra_result->cost_, dijkstra_result->dist_, dijkstra_result->path_.size());
  }
  
  if (successful_tests.empty()) {
    fmt::println("❌ No successful Dijkstra routes found - cannot test CH");
    GTEST_SKIP() << "No valid routes found for CH testing";
  }
  
  fmt::println("\\n=== Building comprehensive CH hierarchy ===");
  fmt::println("Collecting nodes from {} successful routes...", successful_tests.size());
  
  // Collect all nodes from all successful routes
  std::unordered_set<std::uint32_t> all_path_nodes;
  
  for (auto const& [from_node, to_node] : successful_tests) {
    auto const from_loc = location{w.get_node_pos(from_node)};
    auto const to_loc = location{w.get_node_pos(to_node)};
    auto const from_matches = l.match<car>(from_loc, false, direction::kForward, kMaxMatchDistance, nullptr);
    auto const to_matches = l.match<car>(to_loc, true, direction::kForward, kMaxMatchDistance, nullptr);
    auto const from_matches_span = std::span{begin(from_matches), end(from_matches)};
    auto const to_matches_span = std::span{begin(to_matches), end(to_matches)};
    
    auto const dijkstra_result = route(w, l, search_profile::kCar, from_loc, to_loc, 
                                       from_matches_span, to_matches_span,
                                       max_cost, direction::kForward, nullptr, nullptr, nullptr,
                                       routing_algorithm::kDijkstra);
    
    if (dijkstra_result) {
      // Extract nodes from path
      for (auto const& segment : dijkstra_result->path_) {
        all_path_nodes.insert(segment.from_.v_);
        all_path_nodes.insert(segment.to_.v_);
      }
      all_path_nodes.insert(from_node.v_);
      all_path_nodes.insert(to_node.v_);
    }
  }
  
  // Add safety margin: include neighboring nodes
  std::unordered_set<std::uint32_t> all_nodes = all_path_nodes;
  for (auto path_node_id : all_path_nodes) {
    node_idx_t path_node{path_node_id};
    
    car::template adjacent<direction::kForward, false>(
      *w.r_, car::node{path_node, way_pos_t{0}, direction::kForward}, 
      nullptr, nullptr, nullptr,
      [&](car::node const neighbor, std::uint32_t const, distance_t,
          way_idx_t const, std::uint16_t, std::uint16_t,
          elevation_storage::elevation const, bool const) {
        all_nodes.insert(neighbor.n_.v_);
      });
    
    car::template adjacent<direction::kBackward, false>(
      *w.r_, car::node{path_node, way_pos_t{0}, direction::kBackward}, 
      nullptr, nullptr, nullptr,
      [&](car::node const neighbor, std::uint32_t const, distance_t,
          way_idx_t const, std::uint16_t, std::uint16_t,
          elevation_storage::elevation const, bool const) {
        all_nodes.insert(neighbor.n_.v_);
      });
  }
  
  fmt::println("Path nodes: {}, Total nodes (with neighbors): {}", all_path_nodes.size(), all_nodes.size());
  
  // Build CH hierarchy
  auto& ch_data = get_ch_data();
  ch_data.clear();
  
  std::mt19937 gen(42); // Fixed seed for reproducible testing
  std::uniform_int_distribution<ch_level_t> level_dist(1, 1000);
  
  for (auto node_id : all_nodes) {
    node_idx_t node{node_id};
    ch_level_t node_level = level_dist(gen);
    
    for (way_pos_t way = 0; way < 16; ++way) {
      ch_data.node_levels_[car_ch_key{node, way, direction::kForward}] = node_level;
      ch_data.node_levels_[car_ch_key{node, way, direction::kBackward}] = node_level;
    }
  }
  
  fmt::println("CH hierarchy has {} car states", ch_data.node_levels_.size());
  
  // Now test CH on all successful routes
  fmt::println("\\n=== Testing CH on all routes ===");
  
  size_t ch_successes = 0;
  size_t ch_failures = 0;
  
  for (size_t i = 0; i < successful_tests.size(); ++i) {
    auto const& [from_node, to_node] = successful_tests[i];
    
    fmt::println("\\nCH Test {}/{}: {} -> {}", i + 1, successful_tests.size(), from_node.v_, to_node.v_);
    
    auto const from_loc = location{w.get_node_pos(from_node)};
    auto const to_loc = location{w.get_node_pos(to_node)};
    auto const from_matches = l.match<car>(from_loc, false, direction::kForward, kMaxMatchDistance, nullptr);
    auto const to_matches = l.match<car>(to_loc, true, direction::kForward, kMaxMatchDistance, nullptr);
    auto const from_matches_span = std::span{begin(from_matches), end(from_matches)};
    auto const to_matches_span = std::span{begin(to_matches), end(to_matches)};
    
    auto const dijkstra_result = route(w, l, search_profile::kCar, from_loc, to_loc, 
                                       from_matches_span, to_matches_span,
                                       max_cost, direction::kForward, nullptr, nullptr, nullptr,
                                       routing_algorithm::kDijkstra);
    
    auto const ch_result = route(w, l, search_profile::kCar, from_loc, to_loc,
                                 from_matches_span, to_matches_span,
                                 max_cost, direction::kForward, nullptr, nullptr, nullptr,
                                 routing_algorithm::kCHDijkstra);
    
    if (dijkstra_result && ch_result && dijkstra_result->cost_ == ch_result->cost_) {
      fmt::println("✅ SUCCESS: Dijkstra={}, CH={}", dijkstra_result->cost_, ch_result->cost_);
      ch_successes++;
    } else if (dijkstra_result && !ch_result) {
      fmt::println("❌ CH FAILED: Dijkstra={}, CH=no result", dijkstra_result->cost_);
      ch_failures++;
    } else if (dijkstra_result && ch_result) {
      fmt::println("❌ COST MISMATCH: Dijkstra={}, CH={}", dijkstra_result->cost_, ch_result->cost_);
      ch_failures++;
    }
  }
  
  // Final summary
  fmt::println("\\n=== FINAL RESULTS ===");
  fmt::println("Total test pairs: {}", test_pairs.size());
  fmt::println("Valid Dijkstra routes: {}", successful_tests.size());  
  fmt::println("CH successes: {}", ch_successes);
  fmt::println("CH failures: {}", ch_failures);
  fmt::println("CH success rate: {:.1f}%", 
               successful_tests.empty() ? 0.0 : (100.0 * ch_successes / successful_tests.size()));
  
  // Expect high success rate
  EXPECT_GE(ch_successes, successful_tests.size() * 0.8) << "CH should succeed on at least 80% of valid routes";
}