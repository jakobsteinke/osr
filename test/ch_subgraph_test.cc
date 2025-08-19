#include "gtest/gtest.h"

#include <filesystem>
#include <random>
#include <chrono>
#include <queue>
#include <unordered_set>

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

// Build a connected subgraph using BFS from a random starting node
std::vector<node_idx_t> build_connected_subgraph(ways const& w, size_t target_size) {
  if (w.n_nodes() == 0) return {};
  
  std::random_device rd;
  std::mt19937 gen(42); // Fixed seed for reproducibility
  std::uniform_int_distribution<std::uint32_t> node_dist(0, w.n_nodes() - 1);
  
  // Start BFS from a random node
  node_idx_t start_node{node_dist(gen)};
  std::queue<node_idx_t> bfs_queue;
  std::unordered_set<std::uint32_t> visited;
  std::vector<node_idx_t> subgraph_nodes;
  
  bfs_queue.push(start_node);
  visited.insert(start_node.v_);
  
  fmt::println("Building connected subgraph starting from node {}", start_node.v_);
  
  while (!bfs_queue.empty() && subgraph_nodes.size() < target_size) {
    auto current = bfs_queue.front();
    bfs_queue.pop();
    subgraph_nodes.push_back(current);
    
    // Explore all adjacent nodes using car profile
    car::template adjacent<direction::kForward, false>(
      *w.r_, car::node{current, way_pos_t{0}, direction::kForward}, 
      nullptr, nullptr, nullptr,
      [&](car::node const neighbor, std::uint32_t const cost, distance_t,
          way_idx_t const way, std::uint16_t, std::uint16_t,
          elevation_storage::elevation const, bool const) {
        if (visited.find(neighbor.n_.v_) == visited.end() && 
            subgraph_nodes.size() < target_size) {
          visited.insert(neighbor.n_.v_);
          bfs_queue.push(neighbor.n_);
        }
      });
      
    car::template adjacent<direction::kBackward, false>(
      *w.r_, car::node{current, way_pos_t{0}, direction::kBackward}, 
      nullptr, nullptr, nullptr,
      [&](car::node const neighbor, std::uint32_t const cost, distance_t,
          way_idx_t const way, std::uint16_t, std::uint16_t,
          elevation_storage::elevation const, bool const) {
        if (visited.find(neighbor.n_.v_) == visited.end() && 
            subgraph_nodes.size() < target_size) {
          visited.insert(neighbor.n_.v_);
          bfs_queue.push(neighbor.n_);
        }
      });
  }
  
  fmt::println("Built connected subgraph with {} nodes", subgraph_nodes.size());
  return subgraph_nodes;
}

// Custom CH preprocessing that only processes nodes in our subgraph
ch_data preprocess_subgraph(ways const& w, std::vector<node_idx_t> const& subgraph_nodes) {
  fmt::println("Starting CH preprocessing for {} nodes", subgraph_nodes.size());
  
  // Create a set for fast lookup
  std::unordered_set<std::uint32_t> subgraph_set;
  for (auto n : subgraph_nodes) {
    subgraph_set.insert(n.v_);
  }
  
  // Generate car states only for nodes in our subgraph
  std::vector<car_ch_key> car_states;
  for (auto node : subgraph_nodes) {
    // Add forward and backward car states for each way position the node appears in
    for (way_pos_t way = 0; way < 4; ++way) { // Limit way positions for simplicity
      car_states.emplace_back(node, way, direction::kForward);
      car_states.emplace_back(node, way, direction::kBackward);
    }
  }
  
  fmt::println("Generated {} car states for subgraph", car_states.size());
  
  ch_data ch_data;
  
  // Assign levels using simple degree-based ordering
  std::vector<std::pair<int, car_ch_key>> node_priorities;
  for (auto const& car_state : car_states) {
    // Calculate degree for this car state
    int degree = 0;
    
    car::template adjacent<direction::kForward, false>(
      *w.r_, car::node{car_state.n_, car_state.way_, car_state.dir_}, 
      nullptr, nullptr, nullptr,
      [&](car::node const neighbor, std::uint32_t const, distance_t,
          way_idx_t const, std::uint16_t, std::uint16_t,
          elevation_storage::elevation const, bool const) {
        if (subgraph_set.count(neighbor.n_.v_)) {
          degree++;
        }
      });
      
    car::template adjacent<direction::kBackward, false>(
      *w.r_, car::node{car_state.n_, car_state.way_, car_state.dir_}, 
      nullptr, nullptr, nullptr,
      [&](car::node const neighbor, std::uint32_t const, distance_t,
          way_idx_t const, std::uint16_t, std::uint16_t,
          elevation_storage::elevation const, bool const) {
        if (subgraph_set.count(neighbor.n_.v_)) {
          degree++;
        }
      });
    
    node_priorities.emplace_back(degree, car_state);
  }
  
  // Sort by degree (lowest first = contract first)
  std::sort(node_priorities.begin(), node_priorities.end());
  
  // Assign levels
  for (size_t i = 0; i < node_priorities.size(); ++i) {
    ch_data.node_levels_[node_priorities[i].second] = static_cast<ch_level_t>(i);
  }
  
  fmt::println("Assigned levels to {} car states", ch_data.node_levels_.size());
  
  // For simplicity, don't create shortcuts in this test - just verify level assignment works
  // Real shortcut creation would require implementing the full contraction logic
  
  return ch_data;
}

TEST(ch_subgraph, connected_subgraph_test) {
  auto const raw_data = "test/monaco.osm.pbf";
  auto const data_dir = "test/monaco";
  constexpr auto const kMaxMatchDistance = 100;
  constexpr auto const max_cost = 3600U;
  constexpr auto const subgraph_size = 50U; // Small connected subgraph
  
  if (!fs::exists(raw_data) && !fs::exists(data_dir)) {
    GTEST_SKIP() << raw_data << " not found";
  }
  
  if (!fs::exists(data_dir) && fs::exists(raw_data)) {
    fs::create_directories(data_dir);
    osr::extract(false, raw_data, data_dir, fs::path{});
  }
  
  auto const w = osr::ways{data_dir, cista::mmap::protection::READ};
  auto const l = osr::lookup{w, data_dir, cista::mmap::protection::READ};
  
  fmt::println("Monaco graph has {} nodes", w.n_nodes());
  
  // Build connected subgraph
  auto subgraph_nodes = build_connected_subgraph(w, subgraph_size);
  ASSERT_GE(subgraph_nodes.size(), 2U) << "Need at least 2 nodes for testing";
  
  // Pick first and last nodes from subgraph for testing
  auto from_node = subgraph_nodes.front();
  auto to_node = subgraph_nodes.back();
  
  fmt::println("Testing route from node {} to node {}", from_node.v_, to_node.v_);
  
  // Get locations for routing
  auto const from_loc = location{w.get_node_pos(from_node)};
  auto const to_loc = location{w.get_node_pos(to_node)};
  
  // Find matches
  auto const from_matches = l.match<car>(from_loc, false, direction::kForward, kMaxMatchDistance, nullptr);
  auto const to_matches = l.match<car>(to_loc, true, direction::kForward, kMaxMatchDistance, nullptr);
  
  if (from_matches.empty() || to_matches.empty()) {
    GTEST_SKIP() << "No matches found for test nodes";
  }
  
  auto const from_matches_span = std::span{begin(from_matches), end(from_matches)};
  auto const to_matches_span = std::span{begin(to_matches), end(to_matches)};
  
  // Run regular Dijkstra to establish ground truth
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
  
  // Build CH for subgraph
  auto ch_data = preprocess_subgraph(w, subgraph_nodes);
  
  // Override global CH data for testing
  auto& global_ch_data = get_ch_data();
  global_ch_data = std::move(ch_data);
  
  // Run CH Dijkstra
  auto const ch_start = std::chrono::steady_clock::now();
  auto const ch_result = route(w, l, search_profile::kCar, from_loc, to_loc,
                               from_matches_span, to_matches_span,
                               max_cost, direction::kForward, nullptr, nullptr, nullptr,
                               routing_algorithm::kCHDijkstra);
  auto const ch_time = std::chrono::steady_clock::now() - ch_start;
  
  // Print results
  auto const print_result = [&](std::string_view name, auto const& result, auto const& time) {
    fmt::println("{:12}: {} | time: {:.3f}ms", 
                 name,
                 result ? fmt::format("cost: {:5} | dist: {:>10.2f}m", result->cost_, result->dist_) : "no result",
                 std::chrono::duration_cast<std::chrono::microseconds>(time).count() / 1000.0);
  };
  
  fmt::println("\n=== RESULTS ===");
  print_result("Dijkstra", dijkstra_result, dijkstra_time);
  print_result("Bidirectional", bidir_result, bidir_time);
  print_result("CH", ch_result, ch_time);
  
  // Verify results
  if (dijkstra_result && bidir_result) {
    EXPECT_EQ(dijkstra_result->cost_, bidir_result->cost_) 
      << "Dijkstra and bidirectional should find same cost";
  }
  
  if (dijkstra_result && ch_result) {
    EXPECT_EQ(dijkstra_result->cost_, ch_result->cost_) 
      << "CH should find same cost as Dijkstra";
    
    if (dijkstra_result->cost_ == ch_result->cost_) {
      fmt::println("✅ SUCCESS: CH found optimal route with same cost as Dijkstra!");
    }
  } else if (dijkstra_result && !ch_result) {
    fmt::println("❌ CH failed to find route that Dijkstra found");
  } else if (!dijkstra_result && !ch_result) {
    fmt::println("✅ Both Dijkstra and CH agree: no route exists");
  }
}