#ifdef _WIN32
#include "windows.h"
#endif

#include "gtest/gtest.h"

#include <filesystem>
#include <random>
#include <chrono>

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
#include "osr/types.h"
#include "osr/ways.h"

// Include our OSR-integrated CH implementation
#include "osr/routing/ch_graph.h"
#include "osr/routing/ch_preprocessor.h"
#include "osr/routing/ch_bidirectional.h"

namespace fs = std::filesystem;
using namespace osr;

constexpr auto const kUseMultithreading = false;
constexpr auto const kPrintDebugGeojson = false;
constexpr auto const kMaxMatchDistance = 100;

void load_monaco_ch(std::string_view raw_data, std::string_view data_dir) {
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

// Create a simple Dijkstra for comparison using dial queue and OSR types
class osr_dijkstra {
public:
  explicit osr_dijkstra(const ch_graph& graph)
    : graph_(graph),
      node_count_(graph.node_count()),
      distance_(node_count_, kInfeasible),
      predecessor_(node_count_, node_idx_t::invalid()),
      pq_(ch_get_bucket{}) {}

  ch_query_result query(std::uint32_t source, std::uint32_t target, cost_t max_cost = kInfeasible) {
    reset(max_cost);
    
    if (source >= node_count_ || target >= node_count_) {
      return {};
    }
    
    if (source == target) {
      return {0, {source}};
    }
    
    distance_[source] = 0;
    pq_.push(ch_query_label{node_idx_t{source}, 0});
    
    while (!pq_.empty()) {
      auto current = pq_.pop();
      auto const node = current.node.v_;
      auto const dist = current.cost;
      
      if (dist > distance_[node]) continue;
      
      if (node == target) {
        return {dist, reconstruct_path(source, target)};
      }
      
      // Use ALL edges (no level filtering for regular Dijkstra)
      for (const auto& arc : graph_.out_arcs(node)) {
        auto const neighbor = arc.target.v_;
        auto const new_dist = dist + arc.weight;
        
        if (new_dist < distance_[neighbor] && new_dist < kInfeasible) {
          distance_[neighbor] = new_dist;
          predecessor_[neighbor] = node_idx_t{node};
          pq_.push(ch_query_label{node_idx_t{neighbor}, static_cast<cost_t>(new_dist)});
        }
      }
    }
    
    return {};  // No path found
  }

private:
  void reset(cost_t max_cost = kInfeasible) {
    std::fill(distance_.begin(), distance_.end(), kInfeasible);
    std::fill(predecessor_.begin(), predecessor_.end(), node_idx_t::invalid());
    pq_.clear();
    if (max_cost != kInfeasible && max_cost > 0) {
      pq_.n_buckets(max_cost + 1U);
    } else {
      pq_.n_buckets(50000U);
    }
  }

  std::vector<std::uint32_t> reconstruct_path(std::uint32_t source, std::uint32_t target) {
    std::vector<std::uint32_t> path;
    auto current = target;
    
    while (current != source) {
      path.push_back(current);
      current = predecessor_[current].v_;
      if (current == node_idx_t::invalid().v_) break;
    }
    path.push_back(source);
    std::reverse(path.begin(), path.end());
    return path;
  }

  const ch_graph& graph_;
  std::uint32_t node_count_;
  std::vector<cost_t> distance_;
  std::vector<node_idx_t> predecessor_;
  dial<ch_query_label, ch_get_bucket> pq_;
};

void run_monaco_ch_comparison(ways const& w,
                             lookup const& l,
                             unsigned const n_samples,
                             unsigned const max_cost) {
  
  fmt::println("Creating OSR-integrated CH graph from Monaco data...");
  auto graph = ch_graph(w);
  
  fmt::println("Running CH preprocessing...");
  auto preprocessor = ch_preprocessor(graph);
  auto start_prep = std::chrono::high_resolution_clock::now();
  preprocessor.preprocess();
  auto end_prep = std::chrono::high_resolution_clock::now();
  
  auto prep_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_prep - start_prep);
  fmt::println("CH preprocessing took: {} ms", prep_time.count());

  // Create algorithm instances
  osr_dijkstra dijkstra(graph);
  ch_bidirectional ch_query(graph);

  // Generate random query pairs
  auto const from_tos = [&]() {
    auto prng = std::mt19937{};
    auto distr = std::uniform_int_distribution<std::uint32_t>{0, graph.node_count() - 1};
    auto from_tos = std::vector<std::pair<std::uint32_t, std::uint32_t>>{};
    for (auto i = 0U; i != n_samples; ++i) {
      from_tos.emplace_back(distr(prng), distr(prng));
    }
    return from_tos;
  }();

  auto n_congruent = 0U;
  auto n_valid_paths = 0U;
  auto dijkstra_total_time = std::chrono::microseconds{0};
  auto ch_total_time = std::chrono::microseconds{0};
  auto total_dijkstra_cost = 0U;
  auto total_ch_cost = 0U;

  fmt::println("Running {} queries on Monaco OSM graph...", n_samples);

  for (auto const& [from_node, to_node] : from_tos) {
    if (from_node == to_node) continue;
    
    // Run Dijkstra
    auto start_dijkstra = std::chrono::high_resolution_clock::now();
    auto dijkstra_result = dijkstra.query(from_node, to_node);
    auto end_dijkstra = std::chrono::high_resolution_clock::now();
    
    // Run CH
    auto start_ch = std::chrono::high_resolution_clock::now();
    auto ch_result = ch_query.query(from_node, to_node);
    auto end_ch = std::chrono::high_resolution_clock::now();
    
    auto dijkstra_time = std::chrono::duration_cast<std::chrono::microseconds>(end_dijkstra - start_dijkstra);
    auto ch_time = std::chrono::duration_cast<std::chrono::microseconds>(end_ch - start_ch);
    
    // Check if both found valid paths
    if (dijkstra_result.distance != kInfeasible && ch_result.distance != kInfeasible) {
      ++n_valid_paths;
      dijkstra_total_time += dijkstra_time;
      ch_total_time += ch_time;
      total_dijkstra_cost += dijkstra_result.distance;
      total_ch_cost += ch_result.distance;
      
      // Check correctness
      if (dijkstra_result.distance == ch_result.distance) {
        ++n_congruent;
      } else {
        fmt::println("Distance mismatch: nodes {}->{}, Dijkstra={}, CH={}", 
                    from_node, to_node,
                    dijkstra_result.distance, ch_result.distance);
      }
    }
  }

  fmt::println("\n=== MONACO OSM CH vs DIJKSTRA RESULTS ===");
  fmt::println("Valid paths found:   {}/{}", n_valid_paths, n_samples);
  
  if (n_valid_paths > 0) {
    fmt::println("Dijkstra total time: {} us", dijkstra_total_time.count());
    fmt::println("CH total time:       {} us", ch_total_time.count());
    fmt::println("Dijkstra avg/query:  {} us", dijkstra_total_time.count() / n_valid_paths);
    fmt::println("CH avg/query:        {} us", ch_total_time.count() / n_valid_paths);
    fmt::println("Dijkstra total cost: {}", total_dijkstra_cost);
    fmt::println("CH total cost:       {}", total_ch_cost);
    
    if (ch_total_time.count() > 0) {
      double speedup = static_cast<double>(dijkstra_total_time.count()) / ch_total_time.count();
      fmt::println("Speedup:             {:.2f}x", speedup);
    }
    
    fmt::println("Correctness:         {}/{} ({:.1f}%)", n_congruent, n_valid_paths,
                (static_cast<double>(n_congruent) / static_cast<double>(n_valid_paths)) * 100);
  }

  // Test should pass if most queries are correct
  EXPECT_GT(n_congruent, n_valid_paths * 0.95);  // At least 95% correct
}

TEST(ChDijkstraComparison, monaco) {
  auto const raw_data = "test/monaco.osm.pbf";
  auto const data_dir = "test/monaco";
  auto const num_samples = 100U;
  auto const max_cost = 3600U;

  if (!fs::exists(raw_data) && !fs::exists(data_dir)) {
    GTEST_SKIP() << raw_data << " not found";
  }

  load_monaco_ch(raw_data, data_dir);
  auto const w = osr::ways{data_dir, cista::mmap::protection::READ};
  auto const l = osr::lookup{w, data_dir, cista::mmap::protection::READ};

  run_monaco_ch_comparison(w, l, num_samples, max_cost);
}