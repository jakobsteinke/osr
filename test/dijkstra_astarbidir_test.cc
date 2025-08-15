#ifdef _WIN32
#include "windows.h"
#endif

#include "gtest/gtest.h"

#include <filesystem>
#include <random>

#include "cista/mmap.h"

#include "utl/parallel_for.h"

#include "fmt/core.h"

#include "osr/extract/extract.h"
#include "osr/geojson.h"
#include "osr/location.h"
#include "osr/lookup.h"
#include "osr/routing/bidirectional.h"
#include "osr/routing/dijkstra.h"
#include "osr/routing/bidirectional_dijkstra.h"
#include "osr/routing/profile.h"
#include "osr/routing/profiles/car.h"
#include "osr/routing/route.h"
#include "osr/types.h"
#include "osr/ways.h"

#include <queue>
#include <unordered_map>
#include <limits>

namespace fs = std::filesystem;
using namespace osr;

constexpr auto const kUseMultithreading = true;
constexpr auto const kPrintDebugGeojson = false;
constexpr auto const kMaxMatchDistance = 100;
constexpr auto const kMaxAllowedPathDifferenceRatio = 0.5;

void load(std::string_view raw_data, std::string_view data_dir) {
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

void run(const ways& w_no_ch,           // No contraction hierarchy
         const ways& w_with_ch,         // With contraction hierarchy
         const lookup& l,
         unsigned n_samples,
         unsigned max_cost) {

          fmt::println("CH fla2g: {}", w_with_ch.contraction_hierarchy_enabled());

  auto const from_tos = [&]() {
    auto prng = std::mt19937{};
    auto distr =
        std::uniform_int_distribution<std::uint32_t>{0, w_no_ch.n_nodes() - 1};
    auto from_tos = std::vector<std::pair<node_idx_t, node_idx_t>>{};
    for (auto i = 0U; i != n_samples; ++i) {
      from_tos.emplace_back(distr(prng), distr(prng));
    }
    return from_tos;
  }();

  auto n_congruent = std::atomic<unsigned>{0U};
  auto n_empty_matches = std::atomic<unsigned>{0U};
  auto reference_times = std::vector<std::chrono::steady_clock::duration>{};
  auto experiment_times = std::vector<std::chrono::steady_clock::duration>{};
  auto bidirdijkstra_times = std::vector<std::chrono::steady_clock::duration>{};
  auto m = std::mutex{};

  auto const single_run = [&](std::pair<node_idx_t, node_idx_t> const from_to) {
    auto const from_node = from_to.first;
    auto const from_loc = location{w_no_ch.get_node_pos(from_node)};
    auto const to_node = from_to.second;
    auto const to_loc = location{w_no_ch.get_node_pos(to_node)};

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

    if (from_matches.empty() || to_matches.empty()) {
      ++n_empty_matches;
    }

    auto const from_matches_span = std::span{begin(from_matches), end(from_matches)};
    auto const to_matches_span = std::span{begin(to_matches), end(to_matches)};

    // Dijkstra: w_no_ch
    auto const reference_start = std::chrono::steady_clock::now();
    auto const reference =
        route(w_no_ch, l, search_profile::kCar, from_loc, to_loc, from_matches_span,
              to_matches_span, max_cost, direction::kForward, nullptr, nullptr,
              nullptr, routing_algorithm::kDijkstra, false); // here param works
    auto const reference_time =
        std::chrono::steady_clock::now() - reference_start;

    // A* bidir: w_no_ch
    auto const experiment_start = std::chrono::steady_clock::now();
    auto const experiment =
        route(w_no_ch, l, search_profile::kCar, from_loc, to_loc, from_matches_span,  // w_no_ch doesnt work
              to_matches_span, max_cost, direction::kForward, nullptr, nullptr,
              nullptr, routing_algorithm::kAStarBi, false);
    auto const experiment_time =
        std::chrono::steady_clock::now() - experiment_start;

    // Bidirectional Dijkstra with CH: w_with_ch  
    auto const bidirdijkstra_start = std::chrono::steady_clock::now();
    auto const bidirdijkstra =
        route(w_with_ch, l, search_profile::kCar, from_loc, to_loc, from_matches_span,
              to_matches_span, max_cost, direction::kForward, nullptr, nullptr,
              nullptr, routing_algorithm::kBidirDijkstra, true); // Enable CH
    auto const bidirdijkstra_time =
        std::chrono::steady_clock::now() - bidirdijkstra_start;

    // Compare results
    if (reference.has_value() != experiment.has_value() ||
        reference.has_value() != bidirdijkstra.has_value() ||
        (reference && experiment && bidirdijkstra &&
        (reference->cost_ + 1 != experiment->cost_ && reference->cost_ != experiment->cost_ ||
         reference->cost_ + 1 != bidirdijkstra->cost_ && reference->cost_ != bidirdijkstra->cost_))) {
      auto const print_result = [&](std::string_view name, auto const& p, auto const& t) {
        fmt::println(
            "{:14}: {:11} --> {:11} | {} | time: "
            "{}:{:0>3}:{:0>3} s",
            name, w_no_ch.node_to_osm_[from_node], w_no_ch.node_to_osm_[to_node],
            p ? fmt::format("cost: {:5} | dist: {:>10.2f}", p->cost_, p->dist_)
              : "no result",
            std::chrono::duration_cast<std::chrono::seconds>(t).count(),
            std::chrono::duration_cast<std::chrono::milliseconds>(t).count() % 1000,
            std::chrono::duration_cast<std::chrono::microseconds>(t).count() % 1000);
      };

      print_result("dijkstra", reference, reference_time);
      print_result("a* bidir", experiment, experiment_time);
      print_result("bidir dijkstra", bidirdijkstra, bidirdijkstra_time);
    } else {
      ++n_congruent;
    }

    if (!from_matches.empty() && !to_matches.empty()) {
      auto const guard = std::lock_guard{m};
      reference_times.emplace_back(reference_time);
      experiment_times.emplace_back(experiment_time);
      bidirdijkstra_times.emplace_back(bidirdijkstra_time);
    } 
  };

  if (kUseMultithreading) {
    utl::parallel_for(from_tos, single_run);
  } else {
    std::for_each(begin(from_tos), end(from_tos), single_run);
  }

  auto const non_empty_congruent = n_congruent - n_empty_matches;
  auto const non_empty_samples = n_samples - n_empty_matches;
  EXPECT_EQ(non_empty_samples, non_empty_congruent);

  fmt::println("congruent on non-empty: {}/{} ({:3.1f}%)", non_empty_congruent,
               non_empty_samples,
               (static_cast<double>(non_empty_congruent) /
                static_cast<double>(non_empty_samples)) * 100);
  if (non_empty_congruent == non_empty_samples) {
    fmt::println(
        "speedup on non-empty (A* bidir): {:.2f}",
        static_cast<double>(
            std::reduce(begin(reference_times), end(reference_times)).count()) /
            static_cast<double>(
                std::reduce(begin(experiment_times), end(experiment_times)).count()));
    fmt::println(
        "speedup on non-empty (bidir dijkstra): {:.2f}",
        static_cast<double>(
            std::reduce(begin(reference_times), end(reference_times)).count()) /
            static_cast<double>(
                std::reduce(begin(bidirdijkstra_times), end(bidirdijkstra_times)).count()));
  }
}


cost_t naive_dijkstra(const osr::ways::routing& r, node_idx_t start, node_idx_t goal, std::optional<node_idx_t> avoid = std::nullopt) {
  using QEntry = std::pair<cost_t, node_idx_t>;
  std::priority_queue<QEntry, std::vector<QEntry>, std::greater<>> q;
  std::unordered_map<node_idx_t, cost_t> dist;

  q.emplace(0, start);
  dist[start] = 0;

  while (!q.empty()) {
    auto [cost, u] = q.top();
    q.pop();

    if (u == goal) return cost;
    if (avoid && u == *avoid) continue;

    for (auto way : r.node_ways_[u]) {
      auto nodes = r.way_nodes_[way];
      for (size_t idx = 0; idx + 1 < nodes.size(); ++idx) {
        node_idx_t v = nodes[idx], w = nodes[idx + 1];
        if (v == u || w == u) {
          node_idx_t n = (v == u) ? w : v;
          if (avoid && n == *avoid) continue;
          cost_t edge_cost = r.way_node_dist_[way][idx];
          cost_t new_cost = cost + edge_cost;
          if (!dist.count(n) || new_cost < dist[n]) {
            dist[n] = new_cost;
            q.emplace(new_cost, n);
          }
        }
      }
    }
  }
  return kInfeasible;
}

// Helper: path v->u + u->w (for "must pass u")
cost_t v_to_w_via_u(const osr::ways::routing& r, node_idx_t v, node_idx_t w, node_idx_t u) {
  cost_t to_u = naive_dijkstra(r, v, u, std::nullopt);
  cost_t u_to_w = naive_dijkstra(r, u, w, std::nullopt);
  if (to_u == kInfeasible || u_to_w == kInfeasible) return kInfeasible;
  return to_u + u_to_w;
}

// Diagnostic test to debug CH issues  
TEST(dijkstra_astarbidir, ch_diagnostic) {
  auto const raw_data = "test/monaco.osm.pbf";
  auto const data_dir = "test/monaco";

  if (!fs::exists(raw_data) && !fs::exists(data_dir)) {
    GTEST_SKIP() << raw_data << " not found";
  }

  load(raw_data, data_dir);

  // Test with a simple case first
  ways w_no_ch{data_dir, cista::mmap::protection::READ};
  w_no_ch.r_->node_ch_level_.resize(w_no_ch.n_nodes(), 0);

  ways w_with_ch{data_dir, cista::mmap::protection::READ};
  w_with_ch.build_contraction_hierarchy();
  
  fmt::println("=== CH DIAGNOSTIC ===");
  fmt::println("Nodes: {}, Ways: {}", w_with_ch.n_nodes(), w_with_ch.n_ways());
  fmt::println("CH enabled: {}", w_with_ch.contraction_hierarchy_enabled());
  fmt::println("Shortcuts created: {}", w_with_ch.r_->shortcuts_.size());

  // Test a few shortcuts
  if (w_with_ch.r_->shortcuts_.size() > 0) {
    fmt::println("First 5 shortcuts:");
    for (size_t i = 0; i < std::min<size_t>(5, w_with_ch.r_->shortcuts_.size()); ++i) {
      const auto& sc = w_with_ch.r_->shortcuts_[i];
      fmt::println("  {}: {} -> {} cost {} via {}", i, sc.from, sc.to, sc.cost, sc.middle);
      
      // Verify shortcut is correct by checking via middle node
      cost_t via_middle = v_to_w_via_u(*w_with_ch.r_, sc.from, sc.to, sc.middle);
      if (via_middle != sc.cost) {
        fmt::println("    ERROR: Expected cost {} but via middle gives {}", sc.cost, via_middle);
      }
    }
  }

  // Test a simple routing case
  auto const l = osr::lookup{w_with_ch, data_dir, cista::mmap::protection::READ};
  
  if (w_with_ch.n_nodes() > 100) {
    node_idx_t from_node{50};
    node_idx_t to_node{100};
    
    auto const from_loc = location{w_no_ch.get_node_pos(from_node)};
    auto const to_loc = location{w_no_ch.get_node_pos(to_node)};
    
    auto const from_matches = l.match<car>(from_loc, false, direction::kForward, 100, nullptr);
    auto const to_matches = l.match<car>(to_loc, true, direction::kForward, 100, nullptr);
    
    if (!from_matches.empty() && !to_matches.empty()) {
      auto const from_matches_span = std::span{begin(from_matches), end(from_matches)};
      auto const to_matches_span = std::span{begin(to_matches), end(to_matches)};
      
      fmt::println("=== ROUTING TEST ===");
      fmt::println("From: {} To: {}", from_node, to_node);
      fmt::println("From matches: {}, To matches: {}", from_matches.size(), to_matches.size());
      
      // Test dijkstra without CH
      fmt::println("Testing dijkstra without CH...");
      auto dijkstra_result = std::optional<path>{};
      try {
        dijkstra_result = route(w_no_ch, l, search_profile::kCar, from_loc, to_loc, 
                               from_matches_span, to_matches_span, 3600U, 
                               direction::kForward, nullptr, nullptr, nullptr, 
                               routing_algorithm::kDijkstra, false);
      } catch (const std::exception& e) {
        fmt::println("Error in dijkstra: {}", e.what());
      }
      
      // Test bidirectional dijkstra with CH
      fmt::println("Testing bidirectional dijkstra with CH...");
      auto bidir_ch_result = std::optional<path>{};
      try {
        bidir_ch_result = route(w_with_ch, l, search_profile::kCar, from_loc, to_loc, 
                               from_matches_span, to_matches_span, 3600U, 
                               direction::kForward, nullptr, nullptr, nullptr, 
                               routing_algorithm::kBidirDijkstra, true);
      } catch (const std::exception& e) {
        fmt::println("Error in bidir+CH: {}", e.what());
      }
      
      fmt::println("Dijkstra result: {}", dijkstra_result.has_value() ? 
        fmt::format("cost {}", dijkstra_result->cost_) : "no path");
      fmt::println("Bidir+CH result: {}", bidir_ch_result.has_value() ? 
        fmt::format("cost {}", bidir_ch_result->cost_) : "no path");
      
      if (dijkstra_result.has_value() && bidir_ch_result.has_value()) {
        auto cost_diff = std::abs(static_cast<int>(dijkstra_result->cost_) - static_cast<int>(bidir_ch_result->cost_));
        fmt::println("Cost difference: {}", cost_diff);
        if (cost_diff > 1) {
          fmt::println("ERROR: Costs don't match!");
        }
      } else if (dijkstra_result.has_value() != bidir_ch_result.has_value()) {
        fmt::println("ERROR: One algorithm found path, other didn't!");
      }
    }
  }
}

TEST(dijkstra_astarbidir, monaco) {
  auto const raw_data = "test/monaco.osm.pbf";
  auto const data_dir = "test/monaco";
  auto const num_samples = 10000U;
  auto const max_cost = 3600U;

  if (!fs::exists(raw_data) && !fs::exists(data_dir)) {
    GTEST_SKIP() << raw_data << " not found";
  }

  load(raw_data, data_dir);

  // No contraction hierarchy:
  ways w_no_ch{data_dir, cista::mmap::protection::READ};
  w_no_ch.r_->node_ch_level_.resize(w_no_ch.n_nodes(), 0);

  // With contraction hierarchy:
  ways w_with_ch{data_dir, cista::mmap::protection::READ};
  w_with_ch.build_contraction_hierarchy();
  fmt::println("After CH build, addr of r_: {}", fmt::ptr(w_with_ch.r_.get()));
  fmt::println("CH flag: {}", w_with_ch.contraction_hierarchy_enabled());
  fmt::println("Number of shortcuts: {}", w_with_ch.r_->shortcuts_.size());
  
  // Use w_with_ch for lookup to have access to CH data
  auto const l = osr::lookup{w_with_ch, data_dir, cista::mmap::protection::READ};

  // Call the new run!
  run(/*w_no_ch*/ w_no_ch, w_with_ch, l, num_samples, max_cost);
}



/*
TEST(dijkstra_astarbidir, monaco) {
  auto const raw_data = "test/monaco.osm.pbf";
  auto const data_dir = "test/monaco";
  auto const num_samples = 10000U;
  auto const max_cost = 3600U;

  if (!fs::exists(raw_data) && !fs::exists(data_dir)) {
    GTEST_SKIP() << raw_data << " not found";
  }

  load(raw_data, data_dir);
  auto w = osr::ways{data_dir, cista::mmap::protection::READ};
  auto const l = osr::lookup{w, data_dir, cista::mmap::protection::READ};
  fmt::println("ways nodes: {}, ways: {}", w.n_nodes(), w.n_ways());
  w.build_contraction_hierarchy();
  ASSERT_EQ(w.r_->node_ch_level_.size(), w.n_nodes());
  fmt::println("shortcuts: {}", w.r_->shortcuts_.size());
  EXPECT_GT(w.r_->shortcuts_.size(), 0); // there should be at least one shortcut in Monaco
  /*for (size_t i = 0; i < std::min<size_t>(50, w.r_->shortcuts_.size()); ++i) {
  const auto& sc = w.r_->shortcuts_[i];
  fmt::println("Shortcut {}: {} -> {} cost {} via {}", i, sc.from, sc.to, sc.cost, sc.middle);
  for (size_t i = 0; i < std::min<size_t>(50, w.r_->shortcuts_.size()); ++i) {
    const auto& sc = w.r_->shortcuts_[i];
    fmt::println("Shortcut {}: {} -> {} cost {} via {}", i, sc.from, sc.to, sc.cost, sc.middle);

    // 1. Shortest v->w via u
    cost_t via_u = v_to_w_via_u(*w.r_, sc.from, sc.to, sc.middle);
    EXPECT_EQ(via_u, sc.cost) << "Shortcut " << i << " cost does not match v->u->w";

    // 2. Shortest v->w avoiding u
    cost_t avoid_u = naive_dijkstra(*w.r_, sc.from, sc.to, sc.middle);
    EXPECT_GE(avoid_u, sc.cost) << "Shortcut " << i << " path avoiding u is shorter!";
  }
  run(w, l, num_samples, max_cost);
}

TEST(dijkstra_astarbidir, hamburg) {
  auto const raw_data = "test/hamburg.osm.pbf";
  auto const data_dir = "test/hamburg";
  auto const num_samples = 5000U;
  auto const max_cost = 2 * 3600U;

  if (!fs::exists(raw_data) && !fs::exists(data_dir)) {
    GTEST_SKIP() << raw_data << " not found";
  }

  load(raw_data, data_dir);
  auto w = osr::ways{data_dir, cista::mmap::protection::READ};
  auto const l = osr::lookup{w, data_dir, cista::mmap::protection::READ};
  fmt::println("ways nodes: {}, ways: {}", w.n_nodes(), w.n_ways());
  w.build_contraction_hierarchy();
  ASSERT_EQ(w.r_->node_ch_level_.size(), w.n_nodes());
  fmt::println("shortcuts: {}", w.r_->shortcuts_.size());
  EXPECT_GT(w.r_->shortcuts_.size(), 0); // there should be at least one shortcut in Monaco
  for (size_t i = 0; i < std::min<size_t>(50, w.r_->shortcuts_.size()); ++i) {
    const auto& sc = w.r_->shortcuts_[i];
    fmt::println("Shortcut {}: {} -> {} cost {} via {}", i, sc.from, sc.to, sc.cost, sc.middle);

    // 1. Shortest v->w via u
    cost_t via_u = v_to_w_via_u(*w.r_, sc.from, sc.to, sc.middle);
    EXPECT_EQ(via_u, sc.cost) << "Shortcut " << i << " cost does not match v->u->w";

    // 2. Shortest v->w avoiding u
    cost_t avoid_u = naive_dijkstra(*w.r_, sc.from, sc.to, sc.middle);
    EXPECT_GE(avoid_u, sc.cost) << "Shortcut " << i << " path avoiding u is shorter!";
  }
  run(w, l, num_samples, max_cost);
}

TEST(dijkstra_astarbidir, switzerland) {
  auto const raw_data = "test/switzerland.osm.pbf";
  auto const data_dir = "test/switzerland";
  auto const num_samples = 1000U;
  auto const max_cost = 5 * 3600U;

  if (!fs::exists(raw_data) && !fs::exists(data_dir)) {
    GTEST_SKIP() << raw_data << " not found";
  }

  load(raw_data, data_dir);
  auto const w = osr::ways{data_dir, cista::mmap::protection::READ};
  auto const l = osr::lookup{w, data_dir, cista::mmap::protection::READ};

  run(w, l, num_samples, max_cost);
}

TEST(dijkstra_astarbidir, DISABLED_germany) {
  auto const raw_data = "test/germany.osm.pbf";
  auto const data_dir = "test/germany";
  constexpr auto const num_samples = 50U;
  constexpr auto const max_cost = 12 * 3600U;

  if (!fs::exists(raw_data) && !fs::exists(data_dir)) {
    GTEST_SKIP() << raw_data << " not found";
  }

  load(raw_data, data_dir);
  auto const w = osr::ways{data_dir, cista::mmap::protection::READ};
  auto const l = osr::lookup{w, data_dir, cista::mmap::protection::READ};

  run(w, l, num_samples, max_cost);
}*/