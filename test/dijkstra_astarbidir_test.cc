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
#include "osr/routing/profile.h"
#include "osr/routing/profiles/car.h"
#include "osr/routing/route.h"
#include "osr/routing/bidirectional_car_dijktra.h"
#include "osr/types.h"
#include "osr/ways.h"

namespace fs = std::filesystem;
using namespace osr;

constexpr auto const kUseMultithreading = true;
constexpr auto const kPrintDebugGeojson = false;
constexpr auto const kMaxMatchDistance = 100;
constexpr auto const kMaxAllowedPathDifferenceRatio = 0.5;
constexpr auto const kEnableCH = true;  // Enable Contraction Hierarchies

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

void run(ways const& w,
         lookup const& l,
         unsigned const n_samples,
         unsigned const max_cost) {

  // Enable CH preprocessing if requested
  if constexpr (kEnableCH) {
    fmt::println("Enabling Contraction Hierarchies preprocessing...");
    auto& bcd = osr::get_bidirectional_car_dijkstra();
    auto const start_time = std::chrono::steady_clock::now();
    bcd.enable_contraction_hierarchies(w, *w.r_);
    auto const end_time = std::chrono::steady_clock::now();
    auto const preprocessing_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    fmt::println("CH preprocessing completed in {} ms", preprocessing_time.count());
  }

  auto const from_tos = [&]() {
    auto prng = std::mt19937{};
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
  auto n_path_traversable = std::atomic<unsigned>{0U};
  auto n_path_not_traversable = std::atomic<unsigned>{0U};
  auto n_cost_match_on_validated_paths = std::atomic<unsigned>{0U};
  auto n_traversable_but_cost_mismatch = std::atomic<unsigned>{0U};
  auto reference_times = std::vector<std::chrono::steady_clock::duration>{};
  auto experiment_times = std::vector<std::chrono::steady_clock::duration>{};

  auto m = std::mutex{};

  auto const single_run = [&](std::pair<node_idx_t, node_idx_t> const from_to) {
    auto const from_node = from_to.first;
    auto const from_loc = location{w.get_node_pos(from_node)};
    auto const to_node = from_to.second;
    auto const to_loc = location{w.get_node_pos(to_node)};
    
    // Track traversability for cost comparison
    auto path_is_traversable = false;

    auto const node_pinned_matches =
        [&](location const& loc, node_idx_t const n, bool const reverse) {
          auto matches = l.match<car>(loc, reverse, direction::kForward,
                                      kMaxMatchDistance, nullptr);
          std::erase_if(matches, [&](auto const& wc) {
            return wc.left_.node_ != n && wc.right_.node_ != n;
          });
          if (matches.size() > 1) {
            // matches.resize(1);
          }
          return matches;
        };
    auto const from_matches = node_pinned_matches(from_loc, from_node, false);
    auto const to_matches = node_pinned_matches(to_loc, to_node, true);
    if (from_matches.empty() || to_matches.empty()) {
      ++n_empty_matches;
    }

    auto const from_matches_span =
        std::span{begin(from_matches), end(from_matches)};
    auto const to_matches_span = std::span{begin(to_matches), end(to_matches)};

    auto const reference_start = std::chrono::steady_clock::now();
    auto const reference =
        route(w, l, search_profile::kCar, from_loc, to_loc, from_matches_span,
              to_matches_span, max_cost, direction::kForward, nullptr, nullptr,
              nullptr, routing_algorithm::kDijkstra);
    auto const reference_time =
        std::chrono::steady_clock::now() - reference_start;
    
    // Check if the Dijkstra path is traversable using CH adjacency maps with level constraints
    if (reference.has_value() && kEnableCH) {
      auto& bcd = osr::get_bidirectional_car_dijkstra();
      
      // Extract the actual node path from the reference route
      std::vector<node_idx_t> node_path;
      bool path_valid = true;
      
      for (auto const& segment : reference->segments_) {
        if (segment.from_ != node_idx_t::invalid()) {
          if (node_path.empty() || node_path.back() != segment.from_) {
            node_path.push_back(segment.from_);
          }
        }
        if (segment.to_ != node_idx_t::invalid()) {
          node_path.push_back(segment.to_);
        }
      }
      
      // Check traversability: verify each edge exists in legal_successors_ with proper level constraints
      if (node_path.size() >= 2) {
        for (size_t i = 0; i < node_path.size() - 1; ++i) {
          bool edge_found = false;
          bool level_valid = false;
          
          // Try all possible car states at the from node
          auto const& from_ways = w.r_->node_ways_[node_path[i]];
          for (way_pos_t from_way_pos{0U}; from_way_pos < from_ways.size(); ++from_way_pos) {
            for (auto from_dir : {direction::kForward, direction::kBackward}) {
              bidirectional_car_dijkstra::car_state from_state{node_path[i], from_way_pos, from_dir};
              
              // Get the level of the current state
              auto from_level_it = bidirectional_car_dijkstra::node_levels_.find(from_state);
              if (from_level_it == bidirectional_car_dijkstra::node_levels_.end()) {
                continue; // State not in CH graph
              }
              auto from_level = from_level_it->second;
              
              // Check if this state exists in legal_successors_
              auto it = bidirectional_car_dijkstra::legal_successors_.find(from_state);
              if (it != bidirectional_car_dijkstra::legal_successors_.end()) {
                // Check if we can reach the next node
                for (auto const& edge : it->second) {
                  if (edge.target.n_ == node_path[i + 1]) {
                    // Check level constraint for this target
                    bidirectional_car_dijkstra::car_state to_state{edge.target.n_, edge.target.way_, edge.target.dir_};
                    auto to_level_it = bidirectional_car_dijkstra::node_levels_.find(to_state);
                    if (to_level_it != bidirectional_car_dijkstra::node_levels_.end()) {
                      auto to_level = to_level_it->second;
                      if (to_level > from_level) {
                        edge_found = true;
                        level_valid = true;
                      } else {
                        edge_found = true;
                        level_valid = false;
                        fmt::println("  Level violation: ({}, {}, {}) [L:{}] -> ({}, {}, {}) [L:{}]",
                                    to_idx(from_state.n), from_state.way, 
                                    from_state.dir == direction::kForward ? "fwd" : "bwd", from_level,
                                    to_idx(to_state.n), to_state.way,
                                    to_state.dir == direction::kForward ? "fwd" : "bwd", to_level);
                      }
                    }
                    break;
                  }
                }
              }
              
              // Also check shortcuts
              if (!edge_found) {
                auto sc_it = bidirectional_car_dijkstra::shortcut_successors_.find(from_state);
                if (sc_it != bidirectional_car_dijkstra::shortcut_successors_.end()) {
                  for (auto const& sc : sc_it->second) {
                    if (sc.target.n_ == node_path[i + 1]) {
                      // Check level constraint for this shortcut target
                      bidirectional_car_dijkstra::car_state to_state{sc.target.n_, sc.target.way_, sc.target.dir_};
                      auto to_level_it = bidirectional_car_dijkstra::node_levels_.find(to_state);
                      if (to_level_it != bidirectional_car_dijkstra::node_levels_.end()) {
                        auto to_level = to_level_it->second;
                        if (to_level > from_level) {
                          edge_found = true;
                          level_valid = true;
                        } else {
                          edge_found = true;
                          level_valid = false;
                          fmt::println("  Shortcut level violation: ({}, {}, {}) [L:{}] => ({}, {}, {}) [L:{}]",
                                      to_idx(from_state.n), from_state.way, 
                                      from_state.dir == direction::kForward ? "fwd" : "bwd", from_level,
                                      to_idx(to_state.n), to_state.way,
                                      to_state.dir == direction::kForward ? "fwd" : "bwd", to_level);
                        }
                      }
                      break;
                    }
                  }
                }
              }
              
              if (edge_found && level_valid) break;
            }
            if (edge_found && level_valid) break;
          }
          
          if (!edge_found) {
            fmt::println("WARNING: Edge {} -> {} from Dijkstra path not found in CH adjacency maps!",
                        w.node_to_osm_[node_path[i]], w.node_to_osm_[node_path[i + 1]]);
            path_valid = false;
          } else if (!level_valid) {
            fmt::println("WARNING: Edge {} -> {} violates CH level constraints!",
                        w.node_to_osm_[node_path[i]], w.node_to_osm_[node_path[i + 1]]);
            path_valid = false;
          }
        }
        
        if (path_valid) {
          fmt::println("✓ Dijkstra path is fully traversable using CH adjacency maps with proper level constraints ({} --> {})", 
                       w.node_to_osm_[from_node], w.node_to_osm_[to_node]);
          ++n_path_traversable;
          path_is_traversable = true;
        } else {
          fmt::println("✗ Dijkstra path is NOT fully traversable using CH adjacency maps or violates level constraints ({} --> {})", 
                       w.node_to_osm_[from_node], w.node_to_osm_[to_node]);
          ++n_path_not_traversable;
          path_is_traversable = false;
        }
        
      }
    }
    
    auto const path_was_validated = reference.has_value() && reference->segments_.size() >= 2 && kEnableCH;

    auto const experiment_start = std::chrono::steady_clock::now();
    auto const experiment =
        route(w, l, search_profile::kCar, from_loc, to_loc, from_matches_span,
              to_matches_span, max_cost, direction::kForward, nullptr, nullptr,
              nullptr, routing_algorithm::kBidirectionalCarDijkstra);
    auto const experiment_time =
        std::chrono::steady_clock::now() - experiment_start;

    // Log the cost comparison for every route with times
    if (reference.has_value() && experiment.has_value()) {
      fmt::println("Route {:11} --> {:11}: dijkstra cost={}, bidir_ch cost={}, match={} | dijkstra time: {}ms, bidir_ch time: {}ms",
                   w.node_to_osm_[from_node], w.node_to_osm_[to_node],
                   reference->cost_, experiment->cost_,
                   reference->cost_ == experiment->cost_ ? "YES" : "NO",
                   std::chrono::duration_cast<std::chrono::milliseconds>(reference_time).count(),
                   std::chrono::duration_cast<std::chrono::milliseconds>(experiment_time).count());
      
      // Track cost matches for validated paths only
      if (path_was_validated && reference->cost_ == experiment->cost_) {
        ++n_cost_match_on_validated_paths;
      }
      
      // Track traversable paths with cost mismatch
      if (path_was_validated && path_is_traversable && reference->cost_ != experiment->cost_) {
        ++n_traversable_but_cost_mismatch;
      }
    } else if (reference.has_value() != experiment.has_value()) {
      fmt::println("Route {:11} --> {:11}: dijkstra={}, bidir_ch={}, match=NO (different existence) | dijkstra time: {}ms, bidir_ch time: {}ms",
                   w.node_to_osm_[from_node], w.node_to_osm_[to_node],
                   reference.has_value() ? "found" : "not_found",
                   experiment.has_value() ? "found" : "not_found",
                   std::chrono::duration_cast<std::chrono::milliseconds>(reference_time).count(),
                   std::chrono::duration_cast<std::chrono::milliseconds>(experiment_time).count());
    }

    if (reference.has_value() != experiment.has_value() ||
        (reference && experiment &&
         (reference->cost_ != experiment->cost_ /*||
          std::abs(reference->dist_ - experiment->dist_) / reference->dist_ >
              kMaxAllowedPathDifferenceRatio*/))) {
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

      print_result("dijkstra", reference, reference_time);
      print_result("bidir car dijkstra", experiment, experiment_time);

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

  EXPECT_EQ(non_empty_samples, non_empty_congruent);

  fmt::println("congruent on non-empty: {}/{} ({:3.1f}%)", non_empty_congruent,
               non_empty_samples,
               (static_cast<double>(non_empty_congruent) /
                static_cast<double>(non_empty_samples)) *
                   100);
  if (non_empty_congruent == non_empty_samples) {
    fmt::println(
        "speedup on non-empty: {:.2f}",
        static_cast<double>(
            std::reduce(begin(reference_times), end(reference_times)).count()) /
            static_cast<double>(
                std::reduce(begin(experiment_times), end(experiment_times))
                    .count()));
  }
  
  auto const total_paths_checked = n_path_traversable + n_path_not_traversable;
  if (total_paths_checked > 0) {
    fmt::println("CH path validation: {}/{} traversable ({:.1f}%), {}/{} not traversable ({:.1f}%)",
                 n_path_traversable.load(), total_paths_checked, 
                 (static_cast<double>(n_path_traversable) / static_cast<double>(total_paths_checked)) * 100,
                 n_path_not_traversable.load(), total_paths_checked,
                 (static_cast<double>(n_path_not_traversable) / static_cast<double>(total_paths_checked)) * 100);
    fmt::println("Cost match on validated paths: {}/{} ({:.1f}%)",
                 n_cost_match_on_validated_paths.load(), total_paths_checked,
                 (static_cast<double>(n_cost_match_on_validated_paths) / static_cast<double>(total_paths_checked)) * 100);
    fmt::println("Traversable but cost mismatch: {}/{} ({:.1f}%)",
                 n_traversable_but_cost_mismatch.load(), total_paths_checked,
                 (static_cast<double>(n_traversable_but_cost_mismatch) / static_cast<double>(total_paths_checked)) * 100);
  }
}

TEST(dijkstra_astarbidir, monaco) {
  auto const raw_data = "test/monaco.osm.pbf";
  auto const data_dir = "test/monaco";
  auto const num_samples = 2000U;
  auto const max_cost = 3600U;

  if (!fs::exists(raw_data) && !fs::exists(data_dir)) {
    GTEST_SKIP() << raw_data << " not found";
  }

  load(raw_data, data_dir);
  auto const w = osr::ways{data_dir, cista::mmap::protection::READ};
  auto const l = osr::lookup{w, data_dir, cista::mmap::protection::READ};

  run(w, l, num_samples, max_cost);
}

TEST(dijkstra_astarbidir, tokelau) {
  auto const raw_data = "test/tokelau-250905.osm.pbf";
  auto const data_dir = "test/tokelau";
  auto const num_samples = 2000U;
  auto const max_cost = 3600U;

  if (!fs::exists(raw_data) && !fs::exists(data_dir)) {
    GTEST_SKIP() << raw_data << " not found";
  }

  load(raw_data, data_dir);
  auto const w = osr::ways{data_dir, cista::mmap::protection::READ};
  auto const l = osr::lookup{w, data_dir, cista::mmap::protection::READ};

  run(w, l, num_samples, max_cost);
}

TEST(dijkstra_astarbidir, hamburg) {
  auto const raw_data = "test/hamburg.osm.pbf";
  auto const data_dir = "test/hamburg";
  auto const num_samples = 5000U;
  auto const max_cost = 3 * 3600U;

  if (!fs::exists(raw_data) && !fs::exists(data_dir)) {
    GTEST_SKIP() << raw_data << " not found";
  }

  load(raw_data, data_dir);
  auto const w = osr::ways{data_dir, cista::mmap::protection::READ};
  auto const l = osr::lookup{w, data_dir, cista::mmap::protection::READ};

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
}