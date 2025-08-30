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
#include "osr/routing/dijkstra.h"
#include "osr/routing/profiles/car.h"
#include "osr/routing/route.h"
#include "osr/types.h"
#include "osr/ways.h"

namespace fs = std::filesystem;
using namespace osr;

constexpr auto const kUseMultithreading = true;
constexpr auto const kPrintDebugGeojson = false;
constexpr auto const kMaxMatchDistance = 100;

void load_ch_test_data(std::string_view raw_data, std::string_view data_dir) {
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

void run_ch_comparison(ways const& w,
                       lookup const& l,
                       unsigned const n_samples,
                       unsigned const max_cost) {

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
  auto reference_times = std::vector<std::chrono::steady_clock::duration>{};
  auto experiment_times = std::vector<std::chrono::steady_clock::duration>{};

  auto m = std::mutex{};

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
    if (from_matches.empty() || to_matches.empty()) {
      ++n_empty_matches;
      return;
    }

    auto const from_matches_span =
        std::span{begin(from_matches), end(from_matches)};
    auto const to_matches_span = std::span{begin(to_matches), end(to_matches)};

    // Reference: Dijkstra
    auto const reference_start = std::chrono::steady_clock::now();
    auto const reference =
        route(w, l, search_profile::kCar, from_loc, to_loc, from_matches_span,
              to_matches_span, max_cost, direction::kForward, nullptr, nullptr,
              nullptr, routing_algorithm::kDijkstra);
    auto const reference_time =
        std::chrono::steady_clock::now() - reference_start;

    // Experiment: CH
    auto const experiment_start = std::chrono::steady_clock::now();
    auto const experiment =
        route(w, l, search_profile::kCar, from_loc, to_loc, from_matches_span,
              to_matches_span, max_cost, direction::kForward, nullptr, nullptr,
              nullptr, routing_algorithm::kCH);
    auto const experiment_time =
        std::chrono::steady_clock::now() - experiment_start;

    if (reference.has_value() != experiment.has_value() ||
        (reference && experiment && reference->cost_ != experiment->cost_)) {
      auto const print_result = [&](std::string_view name, auto const& p,
                                    auto const& t) {
        fmt::println(
            "{:10}: {:11} --> {:11} | {} | time: "
            "{}:{:0>3}:{:0>3} s",
            name, w.node_to_osm_[from_node], w.node_to_osm_[to_node],
            p ? fmt::format("cost: {:5} | dist: {:>10.2f}", p->cost_, p->dist_)
              : "no result",
            std::chrono::duration_cast<std::chrono::seconds>(t).count(),
            std::chrono::duration_cast<std::chrono::milliseconds>(t).count() %
                1000,
            std::chrono::duration_cast<std::chrono::microseconds>(t).count() %
                1000);
        if (p.has_value() && kPrintDebugGeojson) {
          fmt::println("{}\n", to_featurecollection(w, p));
        }
      };

      fmt::println("MISMATCH FOUND:");
      print_result("dijkstra", reference, reference_time);
      print_result("ch", experiment, experiment_time);
      fmt::println("");

    } else {
      ++n_congruent;
    }

    if (!from_matches.empty() && !to_matches.empty()) {
      auto const guard = std::lock_guard{m};
      reference_times.emplace_back(reference_time);
      experiment_times.emplace_back(experiment_time);
    }
  };

  if (kUseMultithreading) {
    utl::parallel_for(from_tos, single_run);
  } else {
    std::for_each(begin(from_tos), end(from_tos), single_run);
  }

  auto const non_empty_congruent = n_congruent - n_empty_matches;
  auto const non_empty_samples = n_samples - n_empty_matches;

  EXPECT_EQ(non_empty_samples, non_empty_congruent)
      << "CH should produce identical costs to Dijkstra for all routes";

  fmt::println("CH vs Dijkstra Results:");
  fmt::println("  Total samples: {}", n_samples);
  fmt::println("  Empty matches: {}", n_empty_matches.load());
  fmt::println("  Non-empty samples: {}", non_empty_samples);
  fmt::println("  Congruent results: {}/{} ({:3.1f}%)", 
               non_empty_congruent, non_empty_samples,
               non_empty_samples > 0 ? 
                   (static_cast<double>(non_empty_congruent) /
                    static_cast<double>(non_empty_samples)) * 100 : 0.0);
  
  if (non_empty_congruent == non_empty_samples && !reference_times.empty()) {
    auto const dijkstra_total = 
        std::reduce(begin(reference_times), end(reference_times));
    auto const ch_total = 
        std::reduce(begin(experiment_times), end(experiment_times));
    
    auto const dijkstra_avg_ms = 
        std::chrono::duration_cast<std::chrono::microseconds>(dijkstra_total).count() /
        static_cast<double>(reference_times.size()) / 1000.0;
    auto const ch_avg_ms = 
        std::chrono::duration_cast<std::chrono::microseconds>(ch_total).count() /
        static_cast<double>(experiment_times.size()) / 1000.0;
    
    fmt::println("  Average times:");
    fmt::println("    Dijkstra: {:.3f} ms", dijkstra_avg_ms);
    fmt::println("    CH:       {:.3f} ms", ch_avg_ms);
    fmt::println("  CH speedup: {:.2f}x",
                 static_cast<double>(dijkstra_total.count()) /
                 static_cast<double>(ch_total.count()));
  }
}

TEST(ch_dijkstra, monaco) {
  auto const raw_data = "test/monaco.osm.pbf";
  auto const data_dir = "test/monaco";
  auto const num_samples = 10000U;
  auto const max_cost = 50000U; // Increased to allow longer searches

  if (!fs::exists(raw_data) && !fs::exists(data_dir)) {
    GTEST_SKIP() << raw_data << " not found";
  }

  load_ch_test_data(raw_data, data_dir);
  auto const w = osr::ways{data_dir, cista::mmap::protection::READ};
  auto const l = osr::lookup{w, data_dir, cista::mmap::protection::READ};

  fmt::println("\n=== CH vs Dijkstra: Monaco ({} nodes) ===", w.n_nodes());
  run_ch_comparison(w, l, num_samples, max_cost);
}

TEST(ch_dijkstra, hamburg) {
  auto const raw_data = "test/hamburg.osm.pbf";
  auto const data_dir = "test/hamburg";
  auto const num_samples = 5000U;
  auto const max_cost = 2 * 3600U;

  if (!fs::exists(raw_data) && !fs::exists(data_dir)) {
    GTEST_SKIP() << raw_data << " not found";
  }

  load_ch_test_data(raw_data, data_dir);
  auto const w = osr::ways{data_dir, cista::mmap::protection::READ};
  auto const l = osr::lookup{w, data_dir, cista::mmap::protection::READ};

  fmt::println("\n=== CH vs Dijkstra: Hamburg ({} nodes) ===", w.n_nodes());
  run_ch_comparison(w, l, num_samples, max_cost);
}

TEST(ch_dijkstra, switzerland) {
  auto const raw_data = "test/switzerland.osm.pbf";
  auto const data_dir = "test/switzerland";
  auto const num_samples = 1000U;
  auto const max_cost = 5 * 3600U;

  if (!fs::exists(raw_data) && !fs::exists(data_dir)) {
    GTEST_SKIP() << raw_data << " not found";
  }

  load_ch_test_data(raw_data, data_dir);
  auto const w = osr::ways{data_dir, cista::mmap::protection::READ};
  auto const l = osr::lookup{w, data_dir, cista::mmap::protection::READ};

  fmt::println("\n=== CH vs Dijkstra: Switzerland ({} nodes) ===", w.n_nodes());
  run_ch_comparison(w, l, num_samples, max_cost);
}

TEST(ch_dijkstra, DISABLED_germany) {
  auto const raw_data = "test/germany.osm.pbf";
  auto const data_dir = "test/germany";
  constexpr auto const num_samples = 50U;
  constexpr auto const max_cost = 12 * 3600U;

  if (!fs::exists(raw_data) && !fs::exists(data_dir)) {
    GTEST_SKIP() << raw_data << " not found";
  }

  load_ch_test_data(raw_data, data_dir);
  auto const w = osr::ways{data_dir, cista::mmap::protection::READ};
  auto const l = osr::lookup{w, data_dir, cista::mmap::protection::READ};

  fmt::println("\n=== CH vs Dijkstra: Germany ({} nodes) ===", w.n_nodes());
  run_ch_comparison(w, l, num_samples, max_cost);
}

// Additional test specifically for CH preprocessing overhead measurement
TEST(ch_dijkstra, preprocessing_timing) {
  auto const raw_data = "test/monaco.osm.pbf";
  auto const data_dir = "test/monaco";

  if (!fs::exists(raw_data) && !fs::exists(data_dir)) {
    GTEST_SKIP() << raw_data << " not found";
  }

  load_ch_test_data(raw_data, data_dir);
  auto const w = osr::ways{data_dir, cista::mmap::protection::READ};
  auto const l = osr::lookup{w, data_dir, cista::mmap::protection::READ};

  fmt::println("\n=== CH Preprocessing & Query Timing Analysis ===");
  fmt::println("Dataset: Monaco ({} nodes)", w.n_nodes());

  // Measure preprocessing time (this happens on first CH query)
  auto const preprocessing_start = std::chrono::steady_clock::now();
  
  // First CH query triggers preprocessing
  auto const dummy_from = location{geo::latlng{43.7396, 7.4263}};
  auto const dummy_to = location{geo::latlng{43.7313, 7.4186}};
  route(w, l, search_profile::kCar, dummy_from, dummy_to,
        cost_t{3600}, direction::kForward, 100.0,
        nullptr, nullptr, nullptr, routing_algorithm::kCH);
  
  auto const preprocessing_time = std::chrono::steady_clock::now() - preprocessing_start;
  
  fmt::println("CH preprocessing time: {:.3f} seconds",
               std::chrono::duration<double>(preprocessing_time).count());

  // Now run a series of queries to measure pure query time
  constexpr int num_query_tests = 100;
  auto query_times = std::vector<std::chrono::microseconds>{};
  
  std::mt19937 gen(123);
  std::uniform_real_distribution<> lat_dist(43.72, 43.75);
  std::uniform_real_distribution<> lng_dist(7.41, 7.44);
  
  for (int i = 0; i < num_query_tests; ++i) {
    auto const from = location{geo::latlng{lat_dist(gen), lng_dist(gen)}};
    auto const to = location{geo::latlng{lat_dist(gen), lng_dist(gen)}};
    
    auto const start = std::chrono::steady_clock::now();
    route(w, l, search_profile::kCar, from, to,
          cost_t{3600}, direction::kForward, 100.0,
          nullptr, nullptr, nullptr, routing_algorithm::kCH);
    auto const query_time = std::chrono::steady_clock::now() - start;
    
    query_times.push_back(std::chrono::duration_cast<std::chrono::microseconds>(query_time));
  }
  
  auto const total_query_time = std::accumulate(
      query_times.begin(), query_times.end(), std::chrono::microseconds{0});
  auto const avg_query_time = total_query_time / num_query_tests;
  
  fmt::println("Average CH query time: {:.3f} ms ({} queries)",
               avg_query_time.count() / 1000.0, num_query_tests);
  
  // Calculate how many queries needed to amortize preprocessing
  if (avg_query_time.count() > 0) {
    auto const preprocessing_us = 
        std::chrono::duration_cast<std::chrono::microseconds>(preprocessing_time);
    auto const break_even_queries = preprocessing_us.count() / avg_query_time.count();
    fmt::println("Break-even point: {} queries to amortize preprocessing cost",
                 break_even_queries);
  }
}