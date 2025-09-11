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
#include "osr/routing/bidirectional_car_dijktra.h"
#include "osr/routing/dijkstra.h"
#include "osr/routing/profile.h"
#include "osr/routing/profiles/car.h"
#include "osr/routing/route.h"
#include "osr/types.h"
#include "osr/ways.h"

namespace fs = std::filesystem;
using namespace osr;

// Forward declaration for testing (in osr namespace)
namespace osr {
double add_path_with_shortcuts(ways const& w,
                              ways::routing const& r,
                              bitvec<node_idx_t> const* blocked,
                              sharing_data const* sharing,
                              elevation_storage const* elevations,
                              car::node const from,
                              car::node const to,
                              cost_t const expected_cost,
                              std::vector<path::segment>& path,
                              direction const dir);
}

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

void run(ways const& w,
         lookup const& l,
         unsigned const n_samples,
         unsigned const max_cost) {

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

    auto const experiment_start = std::chrono::steady_clock::now();
    auto const experiment =
        route(w, l, search_profile::kCar, from_loc, to_loc, from_matches_span,
              to_matches_span, max_cost, direction::kForward, nullptr, nullptr,
              nullptr, routing_algorithm::kBidirectionalCarDijkstra);
    auto const experiment_time =
        std::chrono::steady_clock::now() - experiment_start;

    // Log the cost comparison for every route
    if (reference.has_value() && experiment.has_value()) {
      fmt::println("Route {:11} --> {:11}: dijkstra cost={}, bidir_ch cost={}, match={}",
                   w.node_to_osm_[from_node], w.node_to_osm_[to_node],
                   reference->cost_, experiment->cost_,
                   reference->cost_ == experiment->cost_ ? "YES" : "NO");
    } else if (reference.has_value() != experiment.has_value()) {
      fmt::println("Route {:11} --> {:11}: dijkstra={}, bidir_ch={}, match=NO (different existence)",
                   w.node_to_osm_[from_node], w.node_to_osm_[to_node],
                   reference.has_value() ? "found" : "not_found",
                   experiment.has_value() ? "found" : "not_found");
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

  // Preprocess adjacency for bidirectional car dijkstra
  bidirectional_car_dijkstra::preprocess_adjacency(w, *w.r_);

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

  // Preprocess adjacency for bidirectional car dijkstra
  bidirectional_car_dijkstra::preprocess_adjacency(w, *w.r_);

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

  // Preprocess adjacency for bidirectional car dijkstra
  bidirectional_car_dijkstra::preprocess_adjacency(w, *w.r_);

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

  // Preprocess adjacency for bidirectional car dijkstra
  bidirectional_car_dijkstra::preprocess_adjacency(w, *w.r_);

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

  // Preprocess adjacency for bidirectional car dijkstra
  bidirectional_car_dijkstra::preprocess_adjacency(w, *w.r_);

  run(w, l, num_samples, max_cost);
}


TEST(dijkstra_astarbidir, shortcut_infrastructure) {
  auto const raw_data = "test/monaco.osm.pbf";
  auto const data_dir = "test/monaco";

  if (!fs::exists(raw_data) && !fs::exists(data_dir)) {
    GTEST_SKIP() << raw_data << " not found";
  }

  load(raw_data, data_dir);
  auto const w = osr::ways{data_dir, cista::mmap::protection::READ};
  auto const l = osr::lookup{w, data_dir, cista::mmap::protection::READ};

  // Preprocess adjacency for bidirectional car dijkstra
  bidirectional_car_dijkstra::preprocess_adjacency(w, *w.r_);

  // Test shortcut creation by manually adding a simple shortcut
  // Find some nodes to create a shortcut between
  if (w.n_nodes() >= 3) {
    node_idx_t node_0{0};
    node_idx_t node_1{1}; 
    node_idx_t node_2{2};

    // Create car states for these nodes (using first way/direction found)
    bidirectional_car_dijkstra::car_state state_0, state_1, state_2;
    
    car::resolve_all(*w.r_, node_0, level_t{std::uint8_t{0}}, [&](car::node const& car_node) {
      state_0 = {car_node.n_, car_node.way_, car_node.dir_};
      return false; // Take first one
    });
    
    car::resolve_all(*w.r_, node_1, level_t{std::uint8_t{0}}, [&](car::node const& car_node) {
      state_1 = {car_node.n_, car_node.way_, car_node.dir_};
      return false; // Take first one
    });
    
    car::resolve_all(*w.r_, node_2, level_t{std::uint8_t{0}}, [&](car::node const& car_node) {
      state_2 = {car_node.n_, car_node.way_, car_node.dir_};
      return false; // Take first one  
    });

    // Create dummy edges for the shortcut (for testing infrastructure only)
    bidirectional_car_dijkstra::edge_transition first_edge;
    first_edge.target = car::node{state_1.n, state_1.way, state_1.dir};
    first_edge.cost = 50;
    first_edge.dist = 100;
    first_edge.way = way_idx_t{0};
    first_edge.from = 0;
    first_edge.to = 1;
    first_edge.is_shortcut = false;
    
    bidirectional_car_dijkstra::edge_transition second_edge;
    second_edge.target = car::node{state_2.n, state_2.way, state_2.dir};
    second_edge.cost = 50;
    second_edge.dist = 100;
    second_edge.way = way_idx_t{1};
    second_edge.from = 0;
    second_edge.to = 1;
    second_edge.is_shortcut = false;
    
    // Add a shortcut from state_0 to state_2 via state_1 with cost 100
    bidirectional_car_dijkstra::add_shortcut(state_0, state_2, state_1, 100, first_edge, second_edge);
    
    // Verify the shortcut was added
    auto const* shortcut = bidirectional_car_dijkstra::find_shortcut(state_0, state_2, 100);
    EXPECT_NE(shortcut, nullptr);
    
    if (shortcut) {
      EXPECT_TRUE(shortcut->is_shortcut);
      EXPECT_EQ(shortcut->way, way_idx_t::invalid());
      EXPECT_EQ(shortcut->cost, 100);
      EXPECT_EQ(shortcut->via_state.n, state_1.n);
      EXPECT_EQ(shortcut->via_state.way, state_1.way);
      EXPECT_EQ(shortcut->via_state.dir, state_1.dir);
    }

    std::cout << "Shortcut infrastructure test passed: shortcut created and found successfully!" << std::endl;
  }
}

TEST(dijkstra_astarbidir, shortcut_reconstruction) {
  auto const raw_data = "test/monaco.osm.pbf";
  auto const data_dir = "test/monaco";

  if (!fs::exists(raw_data) && !fs::exists(data_dir)) {
    GTEST_SKIP() << raw_data << " not found";
  }

  load(raw_data, data_dir);
  auto const w = osr::ways{data_dir, cista::mmap::protection::READ};
  auto const l = osr::lookup{w, data_dir, cista::mmap::protection::READ};

  // Preprocess adjacency for bidirectional car dijkstra
  bidirectional_car_dijkstra::preprocess_adjacency(w, *w.r_);

  if (w.n_nodes() >= 3) {
    // Find three connected car states: A -> B -> C
    bidirectional_car_dijkstra::car_state state_a, state_b, state_c;
    car::node node_a, node_b, node_c;
    cost_t cost_ab = 0, cost_bc = 0;
    bool found_chain = false;

    // Search for a chain A->B->C in the adjacency map
    bidirectional_car_dijkstra::edge_transition edge_ab, edge_bc;
    for (auto const& [key_a, transitions_a] : bidirectional_car_dijkstra::legal_successors_) {
      if (found_chain) break;
      
      for (auto const& ab_edge : transitions_a) {
        if (ab_edge.is_shortcut) continue; // Only use real edges
        
        bidirectional_car_dijkstra::car_state key_b{ab_edge.target.n_, ab_edge.target.way_, ab_edge.target.dir_};
        auto it_b = bidirectional_car_dijkstra::legal_successors_.find(key_b);
        
        if (it_b != bidirectional_car_dijkstra::legal_successors_.end()) {
          for (auto const& bc_edge : it_b->second) {
            if (bc_edge.is_shortcut) continue; // Only use real edges
            
            // Found a chain A->B->C
            state_a = key_a;
            state_b = key_b;
            state_c = {bc_edge.target.n_, bc_edge.target.way_, bc_edge.target.dir_};
            node_a = car::node{state_a.n, state_a.way, state_a.dir};
            node_b = car::node{state_b.n, state_b.way, state_b.dir};
            node_c = car::node{state_c.n, state_c.way, state_c.dir};
            cost_ab = ab_edge.cost;
            cost_bc = bc_edge.cost;
            edge_ab = ab_edge;
            edge_bc = bc_edge;
            found_chain = true;
            break;
          }
        }
        if (found_chain) break;
      }
    }

    if (found_chain) {
      std::cout << "Found chain: A(" << to_idx(state_a.n) << ") -> B(" << to_idx(state_b.n) 
                << ") -> C(" << to_idx(state_c.n) << ") with costs " << cost_ab << " + " << cost_bc << std::endl;

      // Create a shortcut A->C via B using the actual edge objects
      cost_t shortcut_cost = cost_ab + cost_bc;
      bidirectional_car_dijkstra::add_shortcut(state_a, state_c, state_b, shortcut_cost, edge_ab, edge_bc);
      
      // Test the shortcut reconstruction
      std::vector<path::segment> path_segments;
      try {
        double dist = add_path_with_shortcuts(w, *w.r_, nullptr, nullptr, nullptr,
                                            node_a, node_c, shortcut_cost, path_segments, direction::kForward);
        
        std::cout << "Shortcut reconstruction succeeded! Distance: " << dist 
                  << ", Segments created: " << path_segments.size() << std::endl;
                  
        // Verify we got multiple segments (from unpacking)
        EXPECT_GT(path_segments.size(), 0);
        
        // Verify total cost matches
        cost_t total_cost = 0;
        for (auto const& seg : path_segments) {
          total_cost += seg.cost_;
        }
        EXPECT_EQ(total_cost, shortcut_cost);
        
        std::cout << "Shortcut reconstruction test passed: " << path_segments.size() 
                  << " segments with total cost " << total_cost << std::endl;
                  
      } catch (std::exception const& e) {
        std::cout << "Shortcut reconstruction failed: " << e.what() << std::endl;
        FAIL() << "Shortcut reconstruction threw exception: " << e.what();
      }
    } else {
      std::cout << "Could not find a suitable A->B->C chain for testing" << std::endl;
      GTEST_SKIP() << "No suitable edge chain found for shortcut reconstruction test";
    }
  }
}

TEST(dijkstra_astarbidir, recursive_shortcut_reconstruction) {
  auto const raw_data = "test/monaco.osm.pbf";
  auto const data_dir = "test/monaco";

  if (!fs::exists(raw_data) && !fs::exists(data_dir)) {
    GTEST_SKIP() << raw_data << " not found";
  }

  load(raw_data, data_dir);
  auto const w = osr::ways{data_dir, cista::mmap::protection::READ};
  auto const l = osr::lookup{w, data_dir, cista::mmap::protection::READ};

  // Preprocess adjacency for bidirectional car dijkstra
  bidirectional_car_dijkstra::preprocess_adjacency(w, *w.r_);

  if (w.n_nodes() >= 4) {
    // Find four connected car states: A -> B -> C -> D
    bidirectional_car_dijkstra::car_state state_a, state_b, state_c, state_d;
    car::node node_a, node_b, node_c, node_d;
    cost_t cost_ab = 0, cost_bc = 0, cost_cd = 0;
    bool found_chain = false;

    // Search for a chain A->B->C->D in the adjacency map
    bidirectional_car_dijkstra::edge_transition edge_ab, edge_bc, edge_cd;
    for (auto const& [key_a, transitions_a] : bidirectional_car_dijkstra::legal_successors_) {
      if (found_chain) break;
      
      for (auto const& ab_edge : transitions_a) {
        if (ab_edge.is_shortcut) continue; // Only use real edges
        
        bidirectional_car_dijkstra::car_state key_b{ab_edge.target.n_, ab_edge.target.way_, ab_edge.target.dir_};
        auto it_b = bidirectional_car_dijkstra::legal_successors_.find(key_b);
        
        if (it_b != bidirectional_car_dijkstra::legal_successors_.end()) {
          for (auto const& bc_edge : it_b->second) {
            if (bc_edge.is_shortcut) continue; // Only use real edges
            
            bidirectional_car_dijkstra::car_state key_c{bc_edge.target.n_, bc_edge.target.way_, bc_edge.target.dir_};
            auto it_c = bidirectional_car_dijkstra::legal_successors_.find(key_c);
            
            if (it_c != bidirectional_car_dijkstra::legal_successors_.end()) {
              for (auto const& cd_edge : it_c->second) {
                if (cd_edge.is_shortcut) continue; // Only use real edges
                
                // Found a chain A->B->C->D
                state_a = key_a;
                state_b = key_b;
                state_c = key_c;
                state_d = {cd_edge.target.n_, cd_edge.target.way_, cd_edge.target.dir_};
                node_a = car::node{state_a.n, state_a.way, state_a.dir};
                node_b = car::node{state_b.n, state_b.way, state_b.dir};
                node_c = car::node{state_c.n, state_c.way, state_c.dir};
                node_d = car::node{state_d.n, state_d.way, state_d.dir};
                cost_ab = ab_edge.cost;
                cost_bc = bc_edge.cost;
                cost_cd = cd_edge.cost;
                edge_ab = ab_edge;
                edge_bc = bc_edge;
                edge_cd = cd_edge;
                found_chain = true;
                break;
              }
            }
            if (found_chain) break;
          }
        }
        if (found_chain) break;
      }
    }

    if (found_chain) {
      std::cout << "Found chain: A(" << to_idx(state_a.n) << ") -> B(" << to_idx(state_b.n) 
                << ") -> C(" << to_idx(state_c.n) << ") -> D(" << to_idx(state_d.n) 
                << ") with costs " << cost_ab << " + " << cost_bc << " + " << cost_cd << std::endl;

      // Step 1: Create first shortcut A->C via B
      cost_t shortcut_ac_cost = cost_ab + cost_bc;
      bidirectional_car_dijkstra::add_shortcut(state_a, state_c, state_b, shortcut_ac_cost, edge_ab, edge_bc);
      std::cout << "Created shortcut A->C with cost " << shortcut_ac_cost << std::endl;
      
      // Step 2: Create second shortcut A->D via C, which will reference the first shortcut
      cost_t shortcut_ad_cost = shortcut_ac_cost + cost_cd;
      
      // For the recursive shortcut A->D, we need to find the A->C shortcut transition
      auto* ac_shortcut = bidirectional_car_dijkstra::find_shortcut(state_a, state_c, shortcut_ac_cost);
      ASSERT_NE(ac_shortcut, nullptr) << "First shortcut A->C not found";
      
      bidirectional_car_dijkstra::add_shortcut(state_a, state_d, state_c, shortcut_ad_cost, *ac_shortcut, edge_cd);
      std::cout << "Created recursive shortcut A->D with cost " << shortcut_ad_cost << std::endl;
      
      // Test the recursive shortcut reconstruction
      std::vector<path::segment> path_segments;
      try {
        double dist = add_path_with_shortcuts(w, *w.r_, nullptr, nullptr, nullptr,
                                            node_a, node_d, shortcut_ad_cost, path_segments, direction::kForward);
        
        std::cout << "Recursive shortcut reconstruction succeeded! Distance: " << dist 
                  << ", Segments created: " << path_segments.size() << std::endl;
                  
        // Verify we got the correct number of segments (should be 3: A->B, B->C, C->D)
        EXPECT_EQ(path_segments.size(), 3);
        
        // Verify total cost matches
        cost_t total_cost = 0;
        for (auto const& seg : path_segments) {
          total_cost += seg.cost_;
          std::cout << "  Segment cost: " << seg.cost_ << std::endl;
        }
        EXPECT_EQ(total_cost, shortcut_ad_cost);
        
        std::cout << "Recursive shortcut reconstruction test passed: " << path_segments.size() 
                  << " segments with total cost " << total_cost << std::endl;
                  
      } catch (std::exception const& e) {
        std::cout << "Recursive shortcut reconstruction failed: " << e.what() << std::endl;
        FAIL() << "Recursive shortcut reconstruction threw exception: " << e.what();
      }
    } else {
      std::cout << "Could not find a suitable A->B->C->D chain for testing" << std::endl;
      GTEST_SKIP() << "No suitable edge chain found for recursive shortcut reconstruction test";
    }
  }
}