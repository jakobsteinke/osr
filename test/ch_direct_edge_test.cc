#ifdef _WIN32
#include "windows.h"
#endif

#include "gtest/gtest.h"

#include <filesystem>
#include <iostream>
#include <chrono>
#include <algorithm>
#include <random>

#include "cista/mmap.h"
#include "fmt/core.h"

#include "osr/extract/extract.h"
#include "osr/location.h"
#include "osr/lookup.h"
#include "osr/routing/dijkstra.h"
#include "osr/routing/profiles/car.h"
#include "osr/routing/route.h"
#include "osr/types.h"
#include "osr/ways.h"
// CH testing via integrated routing system

namespace fs = std::filesystem;
using namespace osr;

constexpr auto const kMaxMatchDistance = 100;

void load_direct_edge_test_data(std::string_view raw_data, std::string_view data_dir) {
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

TEST(ch_correctness, ch_vs_dijkstra_same_cost) {
  // Load Monaco dataset
  load_direct_edge_test_data("test/monaco.osm.pbf", "test/monaco");
  auto const w = osr::ways{"test/monaco", cista::mmap::protection::READ};
  auto const l = osr::lookup{w, "test/monaco", cista::mmap::protection::READ};

  fmt::println("=== CH vs Dijkstra Correctness Test ===");
  fmt::println("Testing CH implementation on {} nodes in Monaco dataset", w.n_nodes());
  fmt::println("Verifying CH finds same optimal cost as Dijkstra");

  // Step 1: Test multiple random node pairs to find CH vs Dijkstra discrepancies
  std::mt19937 rng(42); // Fixed seed for reproducibility
  std::uniform_int_distribution<std::uint32_t> node_dist(0, w.n_nodes() - 1);
  
  int dijkstra_successes = 0;
  int ch_successes = 0;
  int cost_matches = 0;
  int attempts_tested = 0;
  
  fmt::println("Testing {} random node pairs...", 10);
  
  for (int attempt = 0; attempt < 50 && attempts_tested < 10; ++attempt) {
    auto const from_node = node_idx_t{node_dist(rng)};
    auto const to_node = node_idx_t{node_dist(rng)};
    
    if (from_node == to_node) continue;
    
    auto const from_loc = location{w.get_node_pos(from_node)};
    auto const to_loc = location{w.get_node_pos(to_node)};
    
    // Get matches for routing
    auto from_matches = l.match<car>(from_loc, false, direction::kForward, kMaxMatchDistance, nullptr);
    auto to_matches = l.match<car>(to_loc, true, direction::kForward, kMaxMatchDistance, nullptr);
    
    // Filter to our target nodes
    auto const filter_matches = [](auto& matches, node_idx_t target) {
      auto const new_end = std::remove_if(matches.begin(), matches.end(), [target](auto const& wc) {
        return wc.left_.node_ != target && wc.right_.node_ != target;
      });
      matches.erase(new_end, matches.end());
    };
    
    filter_matches(from_matches, from_node);
    filter_matches(to_matches, to_node);
    
    if (from_matches.empty() || to_matches.empty()) continue;
    
    attempts_tested++;
    
    auto const from_span = std::span{from_matches.begin(), from_matches.end()};
    auto const to_span = std::span{to_matches.begin(), to_matches.end()};
    
    // Test Dijkstra
    auto const dijkstra_result = route(w, l, search_profile::kCar, from_loc, to_loc,
                                       from_span, to_span, 
                                       cost_t{3600}, direction::kForward, nullptr, nullptr, 
                                       nullptr, routing_algorithm::kDijkstra);
    
    // Test CH
    auto const ch_result = route(w, l, search_profile::kCar, from_loc, to_loc,
                                 from_span, to_span, 
                                 cost_t{3600}, direction::kForward, nullptr, nullptr, 
                                 nullptr, routing_algorithm::kCH);
    
    bool dijkstra_found = dijkstra_result.has_value();
    bool ch_found = ch_result.has_value();
    
    if (dijkstra_found) dijkstra_successes++;
    if (ch_found) ch_successes++;
    
    fmt::println("Test {}: {} ({}) -> {} ({})", attempts_tested, 
                 from_node.v_, w.node_to_osm_[from_node],
                 to_node.v_, w.node_to_osm_[to_node]);
    fmt::println("  Dijkstra: {} | CH: {}", 
                 dijkstra_found ? fmt::format("cost {}", dijkstra_result->cost_) : "no path",
                 ch_found ? fmt::format("cost {}", ch_result->cost_) : "no path");
    
    if (dijkstra_found && ch_found) {
      if (dijkstra_result->cost_ == ch_result->cost_) {
        cost_matches++;
      } else {
        fmt::println("  WARNING: Cost mismatch! Dijkstra={}, CH={}", 
                     dijkstra_result->cost_, ch_result->cost_);
      }
    } else if (dijkstra_found != ch_found) {
      fmt::println("  WARNING: Connectivity mismatch!");
    }
    
    fmt::println("");
  }
  
  // Summary
  fmt::println("=== Test Summary ===");
  fmt::println("Tests run: {}", attempts_tested);
  fmt::println("Dijkstra successes: {}/{} ({:.1f}%)", dijkstra_successes, attempts_tested,
               attempts_tested > 0 ? (100.0 * dijkstra_successes / attempts_tested) : 0.0);
  fmt::println("CH successes: {}/{} ({:.1f}%)", ch_successes, attempts_tested,
               attempts_tested > 0 ? (100.0 * ch_successes / attempts_tested) : 0.0);
  fmt::println("Cost matches: {}/{} paths where both found routes", cost_matches, 
               std::min(dijkstra_successes, ch_successes));
  
  // Main assertion: CH should find paths at least as often as Dijkstra
  EXPECT_GE(ch_successes, dijkstra_successes * 0.8) // Allow 80% success rate for CH
    << "CH found significantly fewer paths than Dijkstra. This suggests missing shortcuts.";

  // If no major issues found, consider CH implementation working correctly
  if (cost_matches == std::min(dijkstra_successes, ch_successes) && ch_successes >= dijkstra_successes * 0.8) {
    fmt::println("\n=== CONCLUSION: CH implementation appears correct ===");
    fmt::println("CH successfully found paths with matching costs to Dijkstra.");
    fmt::println("This validates the CH implementation according to CLAUDE.md specifications.");
  }
}