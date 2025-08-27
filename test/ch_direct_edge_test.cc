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
#include "osr/routing/ch_preprocessor.h"
#include "osr/routing/ch_bidirectional.h"

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

TEST(ch_direct_edge, find_dijkstra_then_test_ch_direct) {
  // Load Monaco dataset
  load_direct_edge_test_data("test/monaco.osm.pbf", "test/monaco");
  auto const w = osr::ways{"test/monaco", cista::mmap::protection::READ};
  auto const l = osr::lookup{w, "test/monaco", cista::mmap::protection::READ};

  fmt::println("=== CH Direct Edge Test: Find Dijkstra Path, Add Direct Cost-1 Edge ===");
  fmt::println("Testing {} nodes in Monaco dataset", w.n_nodes());

  // Step 1: Find a path that works with Dijkstra
  node_idx_t successful_from_node{node_idx_t::invalid()};
  node_idx_t successful_to_node{node_idx_t::invalid()};
  cost_t dijkstra_cost{kInfeasible};
  
  // Try different node pairs until we find one where Dijkstra succeeds
  std::mt19937 rng(42); // Fixed seed for reproducibility
  std::uniform_int_distribution<std::uint32_t> node_dist(0, w.n_nodes() - 1);
  
  for (int attempt = 0; attempt < 50 && dijkstra_cost == kInfeasible; ++attempt) {
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
    
    // Test Dijkstra
    auto const from_span = std::span{from_matches.begin(), from_matches.end()};
    auto const to_span = std::span{to_matches.begin(), to_matches.end()};
    
    auto const dijkstra_result = route(w, l, search_profile::kCar, from_loc, to_loc,
                                       from_span, to_span, 
                                       cost_t{3600}, direction::kForward, nullptr, nullptr, 
                                       nullptr, routing_algorithm::kDijkstra);
    
    if (dijkstra_result.has_value()) {
      successful_from_node = from_node;
      successful_to_node = to_node;
      dijkstra_cost = dijkstra_result->cost_;
      
      fmt::println("SUCCESS: Found Dijkstra path on attempt {}", attempt + 1);
      fmt::println("  From node: {} (OSM: {})", from_node.v_, w.node_to_osm_[from_node]);
      fmt::println("  To node: {} (OSM: {})", to_node.v_, w.node_to_osm_[to_node]);
      fmt::println("  Dijkstra cost: {}", dijkstra_cost);
      break;
    }
  }
  
  // Ensure we found a valid path
  ASSERT_NE(dijkstra_cost, kInfeasible) << "Could not find any valid Dijkstra path in 50 attempts";
  
  // Step 2: Set up CH with direct cost-1 shortcut (bypass global manager to avoid re-preprocessing)
  fmt::println("\n=== Setting up CH with direct cost-1 shortcut ===");
  
  // Use our own CH preprocessing to maintain control over shortcuts
  ch_preprocessor ch_prep{w};
  ch_prep.preprocess();  // Normal preprocessing first
  
  // Add our direct shortcut with cost 1 (much better than Dijkstra's cost)
  auto& mutable_shortcuts = const_cast<ch_shortcuts&>(ch_prep.get_shortcuts());
  auto& mutable_levels = const_cast<ch_levels&>(ch_prep.get_levels());
  
  // Ensure proper level ordering for the shortcut to be usable
  auto const from_level = mutable_levels.get_level(successful_from_node);
  auto const to_level = mutable_levels.get_level(successful_to_node);
  
  // Make sure target has higher level than source for upward search
  if (to_level <= from_level) {
    mutable_levels.set_level(successful_to_node, from_level + 1);
    fmt::println("Adjusted level: {} (level {}) -> {} (level {})", 
                 successful_from_node.v_, from_level,
                 successful_to_node.v_, from_level + 1);
  } else {
    fmt::println("Level ordering OK: {} (level {}) -> {} (level {})", 
                 successful_from_node.v_, from_level,
                 successful_to_node.v_, to_level);
  }
  
  // Add bidirectional shortcuts with cost 1
  mutable_shortcuts.add_shortcut(successful_from_node, successful_to_node, node_idx_t{0}, cost_t{1});
  mutable_shortcuts.add_shortcut(successful_to_node, successful_from_node, node_idx_t{0}, cost_t{1});
  
  fmt::println("Added direct cost-1 shortcuts: {} <-> {}", 
               successful_from_node.v_, successful_to_node.v_);
  
  // PHASE 2 DEBUGGING: Verify shortcuts are stored correctly
  fmt::println("\n=== Debugging Shortcut Storage ===");
  auto const& shortcuts_map = mutable_shortcuts.get_all();
  bool found_forward_shortcut = false;
  bool found_backward_shortcut = false;
  
  for (auto const& [edge_pair, shortcut_list] : shortcuts_map) {
    auto const [from, to] = edge_pair;
    if (from == successful_from_node && to == successful_to_node) {
      found_forward_shortcut = true;
      fmt::println("✓ Found forward shortcut: {} -> {} with {} entries", 
                   from.v_, to.v_, shortcut_list.size());
      for (auto const& sc : shortcut_list) {
        fmt::println("  Shortcut cost: {}, via: {}", sc.cost_, sc.via_.v_);
      }
    }
    if (from == successful_to_node && to == successful_from_node) {
      found_backward_shortcut = true;
      fmt::println("✓ Found backward shortcut: {} -> {} with {} entries", 
                   from.v_, to.v_, shortcut_list.size());
      for (auto const& sc : shortcut_list) {
        fmt::println("  Shortcut cost: {}, via: {}", sc.cost_, sc.via_.v_);
      }
    }
  }
  
  if (!found_forward_shortcut) {
    fmt::println("✗ Forward shortcut {} -> {} NOT FOUND in shortcuts map!", 
                 successful_from_node.v_, successful_to_node.v_);
  }
  if (!found_backward_shortcut) {
    fmt::println("✗ Backward shortcut {} -> {} NOT FOUND in shortcuts map!", 
                 successful_to_node.v_, successful_from_node.v_);
  }
  
  fmt::println("Total shortcuts in map: {}", shortcuts_map.size());
  
  // Step 3: Test CH with the direct shortcut - it MUST find cost-1 path
  fmt::println("\n=== Testing CH with direct cost-1 shortcut ===");
  
  auto const from_loc = location{w.get_node_pos(successful_from_node)};
  auto const to_loc = location{w.get_node_pos(successful_to_node)};
  
  // Get the same matches as before
  auto from_matches = l.match<car>(from_loc, false, direction::kForward, kMaxMatchDistance, nullptr);
  auto to_matches = l.match<car>(to_loc, true, direction::kForward, kMaxMatchDistance, nullptr);
  
  // Filter matches
  auto const filter_matches = [](auto& matches, node_idx_t target) {
    auto const new_end = std::remove_if(matches.begin(), matches.end(), [target](auto const& wc) {
      return wc.left_.node_ != target && wc.right_.node_ != target;
    });
    matches.erase(new_end, matches.end());
  };
  
  filter_matches(from_matches, successful_from_node);
  filter_matches(to_matches, successful_to_node);
  
  ASSERT_FALSE(from_matches.empty()) << "From matches should not be empty";
  ASSERT_FALSE(to_matches.empty()) << "To matches should not be empty";
  
  // Run CH search directly (bypass the route() function to avoid re-preprocessing)
  fmt::println("Testing CH bidirectional search directly with our preprocessed data...");
  
  // Create CH bidirectional search using our preprocessed data
  auto ch_search = ch_bidirectional<car>{mutable_levels, mutable_shortcuts};
  
  // Initialize the search
  ch_search.init(cost_t{3600}, from_loc, to_loc);
  
  // Add starting nodes
  for (auto const& match : from_matches) {
    for (auto const* nc : {&match.left_, &match.right_}) {
      if (nc->valid()) {
        car::resolve_start_node(*w.r_, match.way_, nc->node_, from_loc.lvl_, 
                                 direction::kForward, [&](auto const node) {
          ch_search.add_forward_start(node, nc->cost_);
        });
      }
    }
  }
  
  // Add target nodes  
  for (auto const& match : to_matches) {
    for (auto const* nc : {&match.left_, &match.right_}) {
      if (nc->valid()) {
        car::resolve_start_node(*w.r_, match.way_, nc->node_, to_loc.lvl_, 
                                 direction::kForward, [&](auto const node) {
          ch_search.add_backward_start(node, nc->cost_);
        });
      }
    }
  }
  
  // Run the bidirectional search
  bool const found_path = ch_search.run_search(w, cost_t{3600});
  cost_t const ch_cost = ch_search.get_shortest_path_cost();
  
  // Create a mock result similar to what route() would return
  std::optional<path> ch_result;
  if (found_path && ch_cost != kInfeasible) {
    // Create a minimal path object (we mainly care about cost for this test)
    path p;
    p.cost_ = ch_cost;
    p.dist_ = distance_t{0}; // Not important for this test
    ch_result = p;
  }
  
  // Results analysis
  fmt::println("\n=== Results Analysis ===");
  fmt::println("Dijkstra result: cost {}", dijkstra_cost);
  
  if (ch_result.has_value()) {
    fmt::println("CH result: FOUND path with cost {}", ch_result->cost_);
    
    // The CH should find the direct cost-1 edge since it's much better than the Dijkstra path
    EXPECT_EQ(ch_result->cost_, cost_t{1}) 
      << "CH should find the direct cost-1 shortcut, not a longer path";
      
    if (ch_result->cost_ == cost_t{1}) {
      fmt::println("SUCCESS: CH correctly found the direct cost-1 shortcut!");
    } else {
      fmt::println("PARTIAL SUCCESS: CH found a path but not the direct cost-1 shortcut");
      fmt::println("This suggests CH is working but may not be using the optimal shortcut");
    }
  } else {
    fmt::println("CH result: NO PATH FOUND");
    fmt::println("FAILURE: CH should definitely find the direct cost-1 shortcut!");
    
    // This is the critical failure case that needs debugging
    FAIL() << "CH failed to find direct cost-1 shortcut between nodes that Dijkstra connects. "
           << "From: " << successful_from_node.v_ << " -> To: " << successful_to_node.v_ 
           << ". This indicates a bug in CH implementation.";
  }
}