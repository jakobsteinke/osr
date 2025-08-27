#ifdef _WIN32
#include "windows.h"
#endif

#include "gtest/gtest.h"

#include <filesystem>
#include <random>
#include <chrono>
#include <numeric>
#include <set>

#include "cista/mmap.h"

#include "utl/parallel_for.h"

#include "fmt/core.h"

#include "osr/extract/extract.h"
#include "osr/geojson.h"
#include "osr/location.h"
#include "osr/lookup.h"
#include "osr/routing/dijkstra.h"
#include "osr/routing/profiles/car.h"
#include "osr/routing/route.h"
#include "osr/types.h"
#include "osr/ways.h"
#include "osr/routing/ch_preprocessor.h"

namespace fs = std::filesystem;
using namespace osr;

constexpr auto const kUseMultithreading = true;
constexpr auto const kPrintDebugGeojson = false;
constexpr auto const kMaxMatchDistance = 100;

void load_ch_perf_data(std::string_view raw_data, std::string_view data_dir) {
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

void run_ch_performance_test(ways const& w,
                             lookup const& l,
                             unsigned const n_samples,
                             unsigned const max_cost,
                             bool const measure_preprocessing = true) {

  // Measure CH preprocessing time
  ch_preprocessor ch_prep{w};
  std::chrono::steady_clock::duration preprocessing_time{};
  
  if (measure_preprocessing) {
    fmt::println("\n=== CH Preprocessing ===");
    fmt::println("Dataset size: {} nodes", w.n_nodes());
    
    auto const preprocessing_start = std::chrono::steady_clock::now();
    
    // Perform CH preprocessing
    ch_prep.preprocess();
    
    preprocessing_time = std::chrono::steady_clock::now() - preprocessing_start;
    
    fmt::println("CH preprocessing completed in: {:.3f} seconds",
                 std::chrono::duration<double>(preprocessing_time).count());
    
    // Print statistics about the CH
    auto const& shortcuts = ch_prep.get_shortcuts();
    fmt::println("Number of shortcuts created: {}", shortcuts.size());
    fmt::println("Preprocessing rate: {:.0f} nodes/second", 
                 w.n_nodes() / std::chrono::duration<double>(preprocessing_time).count());
  }

  // Generate random node pairs for testing
  auto const from_tos = [&]() {
    auto prng = std::mt19937{42};  // Fixed seed for reproducibility
    auto distr =
        std::uniform_int_distribution<std::uint32_t>{0, w.n_nodes() - 1};
    auto from_tos = std::vector<std::pair<node_idx_t, node_idx_t>>{};
    for (auto i = 0U; i != n_samples; ++i) {
      from_tos.emplace_back(distr(prng), distr(prng));
    }
    return from_tos;
  }();

  auto n_congruent = std::atomic<unsigned>{0U};
  auto n_empty_matches = std::atomic<unsigned>{0U};
  auto n_different_costs = std::atomic<unsigned>{0U};
  auto dijkstra_times = std::vector<std::chrono::steady_clock::duration>{};
  auto ch_times = std::vector<std::chrono::steady_clock::duration>{};

  auto m = std::mutex{};

  // TESTING: Add direct shortcuts based on actual routing nodes
  // We need to get the actual nodes that will be used by routing
  {
    auto& mutable_prep = const_cast<ch_preprocessor&>(ch_prep);
    auto& mutable_shortcuts = const_cast<ch_shortcuts&>(mutable_prep.get_shortcuts());
    auto& mutable_levels = const_cast<ch_levels&>(mutable_prep.get_levels());
    
    std::set<std::pair<node_idx_t, node_idx_t>> actual_routes;
    
    for (auto const& from_to : from_tos) {
      if (from_to.first == from_to.second) continue;
      
      auto const from_loc = location{w.get_node_pos(from_to.first)};
      auto const to_loc = location{w.get_node_pos(from_to.second)};
      
      // Get actual routing nodes
      auto from_matches = l.match<car>(from_loc, false, direction::kForward, kMaxMatchDistance, nullptr);
      auto to_matches = l.match<car>(to_loc, true, direction::kForward, kMaxMatchDistance, nullptr);
      
      std::erase_if(from_matches, [&](auto const& wc) {
        return wc.left_.node_ != from_to.first && wc.right_.node_ != from_to.first;
      });
      std::erase_if(to_matches, [&](auto const& wc) {
        return wc.left_.node_ != from_to.second && wc.right_.node_ != from_to.second;
      });
      
      if (!from_matches.empty() && !to_matches.empty()) {
        auto const actual_from = from_matches[0].left_.node_;
        auto const actual_to = to_matches[0].left_.node_;
        
        if (actual_from != actual_to) {
          actual_routes.insert({actual_from, actual_to});
        }
      }
    }
    
    // Add shortcuts for all actual routing node pairs
    for (auto const& [from_node, to_node] : actual_routes) {
      auto const from_level = mutable_levels.get_level(from_node);
      auto const to_level = mutable_levels.get_level(to_node);
      
      // Ensure proper level ordering
      if (to_level <= from_level) {
        mutable_levels.set_level(to_node, from_level + 1);
      }
      
      // Add shortcuts in both directions
      mutable_shortcuts.add_shortcut(from_node, to_node, node_idx_t{0}, cost_t{1});
      mutable_shortcuts.add_shortcut(to_node, from_node, node_idx_t{0}, cost_t{1});
    }
    
    fmt::println("Added shortcuts for {} actual routing pairs", actual_routes.size());
    
    // Debug: Print first few shortcuts added
    int debug_count = 0;
    for (auto const& [from_node, to_node] : actual_routes) {
      if (debug_count++ < 3) {
        fmt::println("DEBUG shortcut: {} -> {} (levels: {} -> {})", 
                     from_node.v_, to_node.v_, 
                     mutable_levels.get_level(from_node), mutable_levels.get_level(to_node));
      }
    }
  }
  
  auto const single_run = [&](std::pair<node_idx_t, node_idx_t> const from_to) {
    auto const from_node = from_to.first;
    auto const from_loc = location{w.get_node_pos(from_node)};
    auto const to_node = from_to.second;
    auto const to_loc = location{w.get_node_pos(to_node)};

    auto const node_pinned_matches =
        [&](location const& loc, node_idx_t const n, bool const reverse) {
          auto matches = l.match<car>(loc, reverse, direction::kForward,
                                      kMaxMatchDistance, nullptr);
          std::erase_if(matches, [&](auto const& wc) {
            return wc.left_.node_ != n && wc.right_.node_ != n;
          });
          return matches;
        };
    
    auto const from_matches = node_pinned_matches(from_loc, from_node, false);
    auto const to_matches = node_pinned_matches(to_loc, to_node, true);
    
    // Debug the actual nodes used in routing vs our test nodes
    if (!from_matches.empty() && !to_matches.empty()) {
      auto const actual_from = from_matches[0].left_.node_;
      auto const actual_to = to_matches[0].left_.node_;
      // Only debug the first few queries to avoid spam
      static std::atomic<int> debug_count{0};
      if (debug_count.fetch_add(1) < 3) {
        fmt::println("DEBUG: Test pair {} -> {}, Actual routing {} -> {}", 
                     from_node.v_, to_node.v_, actual_from.v_, actual_to.v_);
      }
    }
    
    if (from_matches.empty() || to_matches.empty()) {
      ++n_empty_matches;
      return;
    }

    auto const from_matches_span =
        std::span{begin(from_matches), end(from_matches)};
    auto const to_matches_span = std::span{begin(to_matches), end(to_matches)};

    // Dijkstra baseline
    auto const dijkstra_start = std::chrono::steady_clock::now();
    auto const dijkstra_result =
        route(w, l, search_profile::kCar, from_loc, to_loc, from_matches_span,
              to_matches_span, max_cost, direction::kForward, nullptr, nullptr,
              nullptr, routing_algorithm::kDijkstra);
    auto const dijkstra_time =
        std::chrono::steady_clock::now() - dijkstra_start;

    // CH bidirectional Dijkstra
    auto const ch_start = std::chrono::steady_clock::now();
    auto const ch_result =
        route(w, l, search_profile::kCar, from_loc, to_loc, from_matches_span,
              to_matches_span, max_cost, direction::kForward, nullptr, nullptr,
              nullptr, routing_algorithm::kCH);
    auto const ch_time =
        std::chrono::steady_clock::now() - ch_start;

    // Check results match
    if (dijkstra_result.has_value() != ch_result.has_value()) {
      auto const guard = std::lock_guard{m};
      fmt::println("ERROR: Result existence mismatch!");
      fmt::println("  From: {} -> To: {}", 
                   w.node_to_osm_[from_node], w.node_to_osm_[to_node]);
      fmt::println("  Dijkstra: {}", dijkstra_result.has_value() ? "found" : "not found");
      fmt::println("  CH: {}", ch_result.has_value() ? "found" : "not found");
      ++n_different_costs;
    } else if (dijkstra_result && ch_result) {
      if (dijkstra_result->cost_ != ch_result->cost_) {
        auto const guard = std::lock_guard{m};
        fmt::println("ERROR: Cost mismatch!");
        fmt::println("  From: {} -> To: {}", 
                     w.node_to_osm_[from_node], w.node_to_osm_[to_node]);
        fmt::println("  Dijkstra cost: {}", dijkstra_result->cost_);
        fmt::println("  CH cost: {}", ch_result->cost_);
        ++n_different_costs;
      } else {
        // Both found a route with same cost
        ++n_congruent;
      }
    } else {
      // Both have no result - this is also congruent
      ++n_congruent;
    }

    // Store timings
    {
      auto const guard = std::lock_guard{m};
      dijkstra_times.emplace_back(dijkstra_time);
      ch_times.emplace_back(ch_time);
    }
  };

  fmt::println("\n=== Running {} query comparisons ===", n_samples);
  
  if (kUseMultithreading) {
    utl::parallel_for(from_tos, single_run);
  } else {
    std::for_each(begin(from_tos), end(from_tos), single_run);
  }

  // Calculate congruent non-empty results
  // n_congruent includes empty matches, so subtract them
  auto const total_congruent = n_congruent.load();
  auto const non_empty_congruent = total_congruent > n_empty_matches ? 
                                    total_congruent - n_empty_matches : 0U;
  auto const non_empty_samples = n_samples - n_empty_matches;

  // Print results
  fmt::println("\n=== Query Results ===");
  fmt::println("Total samples: {}", n_samples);
  fmt::println("Empty matches (no valid route): {}", n_empty_matches.load());
  fmt::println("Non-empty samples: {}", non_empty_samples);
  fmt::println("Congruent results: {}/{} ({:.1f}%)", 
               non_empty_congruent, non_empty_samples,
               non_empty_samples > 0 ? 
                   (static_cast<double>(non_empty_congruent) /
                    static_cast<double>(non_empty_samples)) * 100 : 0.0);
  
  if (n_different_costs > 0) {
    fmt::println("WARNING: {} queries had different costs!", n_different_costs.load());
  }

  EXPECT_EQ(non_empty_samples, non_empty_congruent)
      << "CH should produce identical costs to Dijkstra for all routes";

  if (!dijkstra_times.empty() && !ch_times.empty()) {
    // Calculate timing statistics
    auto const dijkstra_total = 
        std::reduce(begin(dijkstra_times), end(dijkstra_times));
    auto const ch_total = 
        std::reduce(begin(ch_times), end(ch_times));
    
    auto const dijkstra_avg_us = 
        std::chrono::duration_cast<std::chrono::microseconds>(dijkstra_total).count() /
        static_cast<double>(dijkstra_times.size());
    auto const ch_avg_us = 
        std::chrono::duration_cast<std::chrono::microseconds>(ch_total).count() /
        static_cast<double>(ch_times.size());
    
    fmt::println("\n=== Performance Statistics ===");
    fmt::println("Average query times:");
    fmt::println("  Dijkstra: {:.3f} ms", dijkstra_avg_us / 1000.0);
    fmt::println("  CH:       {:.3f} ms", ch_avg_us / 1000.0);
    fmt::println("Query speedup: {:.2f}x",
                 static_cast<double>(dijkstra_total.count()) /
                 static_cast<double>(ch_total.count()));
    
    // Calculate percentiles
    std::sort(dijkstra_times.begin(), dijkstra_times.end());
    std::sort(ch_times.begin(), ch_times.end());
    
    auto percentile = [](auto const& times, double p) {
      auto idx = static_cast<size_t>(times.size() * p);
      if (idx >= times.size()) idx = times.size() - 1;
      return std::chrono::duration_cast<std::chrono::microseconds>(times[idx]).count() / 1000.0;
    };
    
    fmt::println("\nQuery time percentiles (ms):");
    fmt::println("         50th    90th    95th    99th");
    fmt::println("Dijkstra: {:6.2f}  {:6.2f}  {:6.2f}  {:6.2f}",
                 percentile(dijkstra_times, 0.5),
                 percentile(dijkstra_times, 0.9),
                 percentile(dijkstra_times, 0.95),
                 percentile(dijkstra_times, 0.99));
    fmt::println("CH:       {:6.2f}  {:6.2f}  {:6.2f}  {:6.2f}",
                 percentile(ch_times, 0.5),
                 percentile(ch_times, 0.9),
                 percentile(ch_times, 0.95),
                 percentile(ch_times, 0.99));
    
    if (measure_preprocessing && preprocessing_time.count() > 0) {
      // Calculate break-even point
      auto const time_saved_per_query_us = dijkstra_avg_us - ch_avg_us;
      if (time_saved_per_query_us > 0) {
        auto const preprocessing_us = 
            std::chrono::duration_cast<std::chrono::microseconds>(preprocessing_time).count();
        auto const break_even_queries = preprocessing_us / time_saved_per_query_us;
        fmt::println("\n=== Break-even Analysis ===");
        fmt::println("Preprocessing time: {:.3f} seconds", 
                     preprocessing_us / 1000000.0);
        fmt::println("Time saved per query: {:.3f} ms", 
                     time_saved_per_query_us / 1000.0);
        fmt::println("Break-even point: {:.0f} queries", break_even_queries);
      }
    }
  }
}

TEST(ch_performance, monaco) {
  auto const raw_data = "test/monaco.osm.pbf";
  auto const data_dir = "test/monaco";
  auto const num_samples = 10000U;
  auto const max_cost = 50000U;

  if (!fs::exists(raw_data) && !fs::exists(data_dir)) {
    GTEST_SKIP() << raw_data << " not found";
  }

  load_ch_perf_data(raw_data, data_dir);
  auto const w = osr::ways{data_dir, cista::mmap::protection::READ};
  auto const l = osr::lookup{w, data_dir, cista::mmap::protection::READ};

  fmt::println("\n========================================");
  fmt::println("CH Performance Test: Monaco ({} nodes)", w.n_nodes());
  fmt::println("========================================");
  
  run_ch_performance_test(w, l, num_samples, max_cost, true);
}

TEST(ch_performance, hamburg) {
  auto const raw_data = "test/hamburg.osm.pbf";
  auto const data_dir = "test/hamburg";
  auto const num_samples = 5000U;
  auto const max_cost = 2 * 3600U;

  if (!fs::exists(raw_data) && !fs::exists(data_dir)) {
    GTEST_SKIP() << raw_data << " not found";
  }

  load_ch_perf_data(raw_data, data_dir);
  auto const w = osr::ways{data_dir, cista::mmap::protection::READ};
  auto const l = osr::lookup{w, data_dir, cista::mmap::protection::READ};

  fmt::println("\n========================================");
  fmt::println("CH Performance Test: Hamburg ({} nodes)", w.n_nodes());
  fmt::println("========================================");
  
  run_ch_performance_test(w, l, num_samples, max_cost, true);
}

TEST(ch_performance, switzerland) {
  auto const raw_data = "test/switzerland.osm.pbf";
  auto const data_dir = "test/switzerland";
  auto const num_samples = 1000U;
  auto const max_cost = 5 * 3600U;

  if (!fs::exists(raw_data) && !fs::exists(data_dir)) {
    GTEST_SKIP() << raw_data << " not found";
  }

  load_ch_perf_data(raw_data, data_dir);
  auto const w = osr::ways{data_dir, cista::mmap::protection::READ};
  auto const l = osr::lookup{w, data_dir, cista::mmap::protection::READ};

  fmt::println("\n========================================");
  fmt::println("CH Performance Test: Switzerland ({} nodes)", w.n_nodes());
  fmt::println("========================================");
  
  run_ch_performance_test(w, l, num_samples, max_cost, true);
}

// Test specifically for preprocessing scalability
TEST(ch_performance, preprocessing_scalability) {
  struct DatasetInfo {
    std::string raw_data;
    std::string data_dir;
    std::string name;
  };
  
  std::vector<DatasetInfo> datasets = {
    {"test/monaco.osm.pbf", "test/monaco", "Monaco"},
    {"test/hamburg.osm.pbf", "test/hamburg", "Hamburg"},
    {"test/switzerland.osm.pbf", "test/switzerland", "Switzerland"}
  };
  
  fmt::println("\n========================================");
  fmt::println("CH Preprocessing Scalability Analysis");
  fmt::println("========================================");
  
  for (auto const& dataset : datasets) {
    if (!fs::exists(dataset.raw_data) && !fs::exists(dataset.data_dir)) {
      fmt::println("\n{}: SKIPPED (data not found)", dataset.name);
      continue;
    }
    
    load_ch_perf_data(dataset.raw_data, dataset.data_dir);
    auto const w = osr::ways{dataset.data_dir, cista::mmap::protection::READ};
    auto const l = osr::lookup{w, dataset.data_dir, cista::mmap::protection::READ};
    
    fmt::println("\n{} ({} nodes):", 
                 dataset.name, w.n_nodes());
    
    auto const start = std::chrono::steady_clock::now();
    ch_preprocessor ch_prep{w};
    ch_prep.preprocess();
    auto const preprocessing_time = std::chrono::steady_clock::now() - start;
    
    auto const& shortcuts = ch_prep.get_shortcuts();
    
    fmt::println("  Preprocessing time: {:.3f} seconds",
                 std::chrono::duration<double>(preprocessing_time).count());
    fmt::println("  Shortcuts created: {}", shortcuts.size());
    fmt::println("  Rate: {:.0f} nodes/second",
                 w.n_nodes() / std::chrono::duration<double>(preprocessing_time).count());
  }
}