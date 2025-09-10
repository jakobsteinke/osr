#pragma once

#include <limits>

#include "utl/verify.h"

#include "osr/elevation_storage.h"
#include "osr/location.h"
#include "osr/routing/dial.h"
#include "osr/routing/profiles/car.h"
#include "osr/routing/sharing_data.h"
#include "osr/types.h"
#include "osr/ways.h"

namespace osr {

struct sharing_data;

struct bidirectional_car_dijkstra {
  using profile_t = car;
  using key = car::key;
  using label = car::label;
  using node = car::node;
  using entry = car::entry;
  using hash = car::hash;
  using cost_map = ankerl::unordered_dense::map<key, entry, hash>;

  // New structures for precomputed adjacency
  struct car_state {
    node_idx_t n;
    way_pos_t way;
    direction dir;
    
    bool operator==(const car_state& other) const {
      return n == other.n && way == other.way && dir == other.dir;
    }
  };

  struct car_state_hash {
    using is_avalanching = void;
    std::size_t operator()(const car_state& k) const {
      using namespace ankerl::unordered_dense::detail;
      auto h1 = wyhash::hash(static_cast<std::uint64_t>(to_idx(k.n)));
      auto h2 = wyhash::hash(static_cast<std::uint64_t>(k.way));
      auto h3 = wyhash::hash(static_cast<std::uint64_t>(k.dir == direction::kForward ? 0 : 1));
      return wyhash::mix(h1, wyhash::mix(h2, h3));
    }
  };

  struct edge_transition {
    node target;
    cost_t cost;
    distance_t dist;
    way_idx_t way;
    std::uint16_t from;
    std::uint16_t to;
  };

  using adjacency_map = ankerl::unordered_dense::map<car_state, std::vector<edge_transition>, car_state_hash>;

  constexpr static auto const kDebug = false;
  constexpr static auto const kDebugMaps = false;  // Set to true to see adjacency map structure

  struct get_bucket {
    cost_t operator()(label const& l) { return l.cost(); }
  };

  void clear_mp() {
    meet_point_1_ = node::invalid();
    meet_point_2_ = node::invalid();
    best_cost_ = kInfeasible;
  }

  static void preprocess_adjacency(ways const& w,
                                   ways::routing const& r,
                                   bitvec<node_idx_t> const* blocked = nullptr,
                                   sharing_data const* sharing = nullptr,
                                   elevation_storage const* elevations = nullptr) {
    legal_successors_.clear();
    legal_predecessors_.clear();
    
    if constexpr (kDebugMaps) {
      std::cout << "Starting adjacency preprocessing for " << w.n_nodes() << " nodes...\n";
    }
    
    // Enumerate all nodes in the graph
    for (auto n = node_idx_t{0}; n < node_idx_t{w.n_nodes()}; ++n) {
      // For each node, get all valid car node states (node_idx_t, way_pos_t, direction)
      car::resolve_all(r, n, level_t{std::uint8_t{0}}, [&](node const car_node) {
        // Build adjacency for both search directions
        build_adjacency_for_node_static<direction::kForward>(w, r, car_node, blocked, sharing, elevations);
        build_adjacency_for_node_static<direction::kBackward>(w, r, car_node, blocked, sharing, elevations);
      });
    }
    
    if constexpr (kDebugMaps) {
      std::cout << "Adjacency preprocessing completed: " 
                << legal_successors_.size() << " successor entries, "
                << legal_predecessors_.size() << " predecessor entries\n";
    }
    
    if constexpr (kDebugMaps) {
      // Print sample entries for debugging
      std::cout << "\nSample successor entries:\n";
      auto succ_count = 0;
      for (auto const& [key, transitions] : legal_successors_) {
        if (succ_count >= 5) break;
        std::cout << "  Key {n=" << to_idx(key.n) << ", way=" << static_cast<int>(key.way) 
                  << ", dir=" << (key.dir == direction::kForward ? "FWD" : "BWD") 
                  << "} -> " << transitions.size() << " transitions\n";
        for (auto const& t : transitions) {
          std::cout << "    -> n=" << to_idx(t.target.n_) << ", way=" << static_cast<int>(t.target.way_)
                    << ", dir=" << (t.target.dir_ == direction::kForward ? "FWD" : "BWD") 
                    << ", cost=" << t.cost << "\n";
        }
        ++succ_count;
      }
      
      std::cout << "\nSample predecessor entries:\n";
      auto pred_count = 0;
      for (auto const& [key, transitions] : legal_predecessors_) {
        if (pred_count >= 5) break;
        std::cout << "  Key {n=" << to_idx(key.n) << ", way=" << static_cast<int>(key.way) 
                  << ", dir=" << (key.dir == direction::kForward ? "FWD" : "BWD") 
                  << "} -> " << transitions.size() << " transitions\n";
        for (auto const& t : transitions) {
          std::cout << "    -> n=" << to_idx(t.target.n_) << ", way=" << static_cast<int>(t.target.way_)
                    << ", dir=" << (t.target.dir_ == direction::kForward ? "FWD" : "BWD") 
                    << ", cost=" << t.cost << "\n";
        }
        ++pred_count;
      }
    }
  }

  void reset(cost_t const max,
             location const& start_loc,
             location const& end_loc) {
    pq1_.clear();
    pq2_.clear();
    pq1_.n_buckets(max + 1U);
    pq2_.n_buckets(max + 1U);
    cost1_.clear();
    cost2_.clear();
    clear_mp();
    start_loc_ = start_loc;
    end_loc_ = end_loc;
    max_reached_1_ = false;
    max_reached_2_ = false;
    // NOTE: Do NOT clear static adjacency maps - they persist across searches
  }

  template <direction SearchDir>
  static void build_adjacency_for_node_static(ways const& w,
                                               ways::routing const& r,
                                               node const n,
                                               bitvec<node_idx_t> const* blocked,
                                               sharing_data const* sharing,
                                               elevation_storage const* elevations) {
    if (blocked != nullptr) {
      build_adjacency_for_node_impl<SearchDir, true>(w, r, n, blocked, sharing, elevations);
    } else {
      build_adjacency_for_node_impl<SearchDir, false>(w, r, n, blocked, sharing, elevations);
    }
  }

  template <direction SearchDir, bool WithBlocked>
  static void build_adjacency_for_node_impl(ways const& w,
                                             ways::routing const& r,
                                             node const n,
                                             bitvec<node_idx_t> const* blocked,
                                             sharing_data const* sharing,
                                             elevation_storage const* elevations) {
    car_state source_key{n.n_, n.way_, n.dir_};
    
    if constexpr (kDebugMaps) {
      std::cout << "\n=== Building adjacency for node ===\n";
      std::cout << "Source: ";
      n.print(std::cout, w);
      std::cout << "\n";
      std::cout << "car_state key: {n=" << to_idx(source_key.n) 
                << ", way=" << static_cast<int>(source_key.way) 
                << ", dir=" << (source_key.dir == direction::kForward ? "FWD" : "BWD") 
                << "}\n";
      std::cout << "Search direction: " << (SearchDir == direction::kForward ? "FORWARD" : "BACKWARD") << "\n";
    }
    
    // Get all adjacent nodes for this search direction
    car::adjacent<SearchDir, WithBlocked>(
        r, n, blocked, sharing, elevations,
        [&](node const target, std::uint32_t const cost, distance_t const dist,
            way_idx_t const way, std::uint16_t const from, std::uint16_t const to,
            elevation_storage::elevation const, bool const) {
          
          if constexpr (kDebugMaps) {
            std::cout << "  -> Adjacent: ";
            target.print(std::cout, w);
            std::cout << " [cost=" << cost << ", dist=" << dist 
                      << ", way=" << to_idx(way) << ", from=" << from 
                      << ", to=" << to << "]\n";
          }
          
          if (SearchDir == direction::kForward) {
            // For forward search, store as successors
            legal_successors_[source_key].push_back({target, static_cast<cost_t>(cost), dist, way, from, to});
          } else {
            // For backward search, store as predecessors  
            legal_predecessors_[source_key].push_back({target, static_cast<cost_t>(cost), dist, way, from, to});
          }
        });
    
    if constexpr (kDebugMaps) {
      auto const& map = (SearchDir == direction::kForward) ? legal_successors_ : legal_predecessors_;
      auto it = map.find(source_key);
      if (it != map.end()) {
        std::cout << "Stored " << it->second.size() << " transitions in " 
                  << (SearchDir == direction::kForward ? "legal_successors" : "legal_predecessors") 
                  << " map\n";
      } else {
        std::cout << "Stored 0 transitions in " 
                  << (SearchDir == direction::kForward ? "legal_successors" : "legal_predecessors") 
                  << " map\n";
      }
      std::cout << "=================================\n";
    }
  }

  template <direction SearchDir, bool WithBlocked>
  void build_adjacency_for_node(ways const& w,
                                ways::routing const& r,
                                node const n,
                                bitvec<node_idx_t> const* blocked,
                                sharing_data const* sharing,
                                elevation_storage const* elevations) {
    build_adjacency_for_node_impl<SearchDir, WithBlocked>(w, r, n, blocked, sharing, elevations);
  }

  void add(ways const& w,
           label const l,
           direction const dir,
           cost_map& cost_map,
           dial<label, get_bucket>& d,
           sharing_data const*) {
    if (cost_map[l.get_node().get_key()].update(l, l.get_node(), l.cost(),
                                                node::invalid())) {
      d.push(l);
    }
  }

  void add_start(ways const& w, label const l, sharing_data const* sharing) {
    if (kDebug) {
      l.get_node().print(std::cout, w);
      std::cout << "starting" << l.get_node().n_ << std::endl;
    }
    if constexpr (kDebugMaps) {
      std::cout << "\n+++ Adding START node: ";
      l.get_node().print(std::cout, w);
      std::cout << " with cost=" << l.cost() << "\n";
    }
    add(w, l, direction::kForward, cost1_, pq1_, sharing);
  }

  void add_end(ways const& w, label const l, sharing_data const* sharing) {
    if (kDebug) {
      l.get_node().print(std::cout, w);
      std::cout << "ending" << l.get_node().n_ << std::endl;
    }
    if constexpr (kDebugMaps) {
      std::cout << "\n+++ Adding END node: ";
      l.get_node().print(std::cout, w);
      std::cout << " with cost=" << l.cost() << "\n";
    }
    add(w, l, direction::kBackward, cost2_, pq2_, sharing);
  }

  template <direction SearchDir>
  cost_t get_cost(node const n) const {
    if (SearchDir == direction::kForward) {
      auto const it = cost1_.find(n.get_key());
      return it != end(cost1_) ? it->second.cost(n) : kInfeasible;
    } else {
      auto const it = cost2_.find(n.get_key());
      return it != end(cost2_) ? it->second.cost(n) : kInfeasible;
    }
  }

  cost_t get_cost_to_mp(node const n1, node const n2) const {
    auto const f_cost = get_cost<direction::kForward>(n1);
    auto const b_cost = get_cost<direction::kBackward>(n2);
    if (f_cost == kInfeasible || b_cost == kInfeasible) {
      return kInfeasible;
    }
    return f_cost + b_cost;
  }

  template <direction SearchDir, bool WithBlocked>
  void handle_end_of_way_meetpoint(ways const& w,
                                   ways::routing const& r,
                                   node const curr,
                                   cost_map& costs,
                                   bitvec<node_idx_t> const* blocked,
                                   sharing_data const* sharing,
                                   elevation_storage const* elevations) {
    auto const evaluate_meetpoint = [&](cost_t cost, cost_t other_cost,
                                        node meetpoint1, node meetpoint2) {
      if constexpr (kDebug) {
        std::cout << "  potential MEETPOINT found by ";
        meetpoint1.print(std::cout, w);
      }
      auto const tentative = cost + other_cost;
      if (tentative < best_cost_) {
        meet_point_1_ = meetpoint1;
        meet_point_2_ = meetpoint2;
        best_cost_ = static_cast<cost_t>(tentative);

        if constexpr (kDebug) {
          std::cout << " with cost " << best_cost_ << " -> ACCEPTED\n";
        }
      } else if constexpr (kDebug) {
        std::cout << " -> DOMINATED\n";
      }
    };

    auto const opposite_cost_map =
        opposite(SearchDir) == direction::kForward ? &cost1_ : &cost2_;
    auto const opposite_candidate = opposite_cost_map->find(curr.get_key());
    auto const curr_cost = get_cost<SearchDir>(curr);
    
    if (opposite_candidate != end(*opposite_cost_map)) {
      auto const other_cost = opposite_candidate->second.cost(curr);
      if (other_cost != kInfeasible) {
        evaluate_meetpoint(curr_cost, other_cost, curr, curr);
      } else {
        auto const pred_it = costs.find(curr.get_key());
        if (pred_it == end(costs)) {
          return;
        }
        auto const pred = pred_it->second.pred(curr);
        if (!pred.has_value()) {
          return;
        }
        // Look up adjacency for opposite direction
        car_state curr_key{curr.n_, curr.way_, curr.dir_};
        auto const& opp_adj_map = (opposite(SearchDir) == direction::kForward) ? legal_successors_ : legal_predecessors_;
        
        auto opp_it = opp_adj_map.find(curr_key);
        if (opp_it != opp_adj_map.end()) {
          for (auto const& edge : opp_it->second) {
            if (edge.target.get_key() != pred->get_key()) {
              continue;
            }
            auto const opposite_it =
                opposite_cost_map->find(edge.target.get_key());
            if (opposite_it == end(*opposite_cost_map)) {
              continue;
            }
            auto const opposite_curr = opposite_it->second.pred(edge.target);
            if (!opposite_curr.has_value() ||
                opposite_curr->get_key() != curr.get_key()) {
              continue;
            }
            auto const opposite_curr_cost =
                opposite_candidate->second.cost(*opposite_curr);
            auto const pred_cost = get_cost<SearchDir>(*pred);
            auto const opposite_pred_cost =
                opposite_it->second.cost(edge.target);
            auto const evaluate_meetpoint_with_potential_u_turn_cost =
                [&](cost_t const cost_1, cost_t const cost_2,
                    node const meet_1, node const meet_2) {
                  evaluate_meetpoint(
                      cost_1, cost_2,
                      SearchDir == direction::kForward ? meet_1 : meet_2,
                      SearchDir == direction::kForward ? meet_2 : meet_1);
                };
            if (pred_cost + opposite_pred_cost >
                curr_cost + opposite_curr_cost) {
              evaluate_meetpoint_with_potential_u_turn_cost(
                  pred_cost, opposite_pred_cost, *pred, edge.target);
            } else {
              evaluate_meetpoint_with_potential_u_turn_cost(
                  curr_cost, opposite_curr_cost, curr, *opposite_curr);
            }
          }
        }
      }
    }
  }

  template <direction SearchDir, bool WithBlocked>
  bool run_single(ways const& w,
                  ways::routing const& r,
                  cost_t const max,
                  bitvec<node_idx_t> const* blocked,
                  sharing_data const* sharing,
                  elevation_storage const* elevations,
                  dial<label, get_bucket>& pq,
                  cost_map& costs) {
    if (pq.empty()) {
      return true;
    }

    auto const l = pq.pop();
    auto const curr = l.get_node();
    auto const curr_cost = get_cost<SearchDir>(curr);
    
    if constexpr (kDebugMaps) {
      std::cout << "  EXTRACT ";
      l.get_node().print(std::cout, w);
      std::cout << " with cost=" << l.cost() << " (actual_cost=" << curr_cost << ")\n";
    }
    
    if (curr_cost < l.cost()) {
      if constexpr (kDebugMaps) {
        std::cout << "  Skipping due to dominated cost\n";
      }
      return true;
    }
    
    if constexpr (kDebug) {
      std::cout << "EXTRACT ";
      l.get_node().print(std::cout, w);
      std::cout << "\n";
    }

    // Look up precomputed adjacency
    car_state curr_key{curr.n_, curr.way_, curr.dir_};
    auto const& adj_map = (SearchDir == direction::kForward) ? legal_successors_ : legal_predecessors_;
    
    auto it = adj_map.find(curr_key);
    if constexpr (kDebugMaps) {
      std::cout << "  Looking up adjacency for key {n=" << to_idx(curr_key.n) 
                << ", way=" << static_cast<int>(curr_key.way) 
                << ", dir=" << (curr_key.dir == direction::kForward ? "FWD" : "BWD") 
                << "} in " << (SearchDir == direction::kForward ? "successors" : "predecessors") << "\n";
      
      if (it == adj_map.end()) {
        std::cout << "  WARNING: No adjacency found for this key!\n";
      } else {
        std::cout << "  Found " << it->second.size() << " adjacent transitions\n";
      }
    }
    
    if constexpr (kDebugMaps) {
      if (it != adj_map.end() && !it->second.empty()) {
        std::cout << "\n>>> Using adjacency map for ";
        curr.print(std::cout, w);
        std::cout << "\n    Map type: " << (SearchDir == direction::kForward ? "legal_successors" : "legal_predecessors");
        std::cout << "\n    Found " << it->second.size() << " transitions:\n";
        for (auto const& edge : it->second) {
          std::cout << "      -> ";
          edge.target.print(std::cout, w);
          std::cout << " [cost=" << edge.cost << "]\n";
        }
      }
    }
    if (it != adj_map.end()) {
      for (auto const& edge : it->second) {
        if constexpr (kDebug) {
          std::cout << "  NEIGHBOR ";
          edge.target.print(std::cout, w);
        }
        auto const total = curr_cost + edge.cost;
        if (total >= max) {
          if (SearchDir == direction::kForward) {
            max_reached_1_ = true;
          } else {
            max_reached_2_ = true;
          }
          continue;
        }
        if (total < max &&
            costs[edge.target.get_key()].update(
                l, edge.target, static_cast<cost_t>(total), curr)) {

          auto next = label{edge.target, static_cast<cost_t>(total)};
          next.track(l, r, edge.way, edge.target.get_node(), false);
          pq.push(std::move(next));

          // Meetpoint checking is done after settling nodes

          if constexpr (kDebug) {
            std::cout << " -> PUSH\n";
          }
        } else {
          if constexpr (kDebug) {
            std::cout << " -> DOMINATED\n";
          }
        }
      }
    }

    handle_end_of_way_meetpoint<SearchDir, WithBlocked>(w, r, curr, costs, blocked, sharing, elevations);

    // μ-termination: only check after finding first meetpoint
    if (best_cost_ != kInfeasible && !pq1_.empty() && !pq2_.empty()) {
      auto const min_f = pq1_.buckets_[pq1_.get_next_bucket()].back().cost();
      auto const min_r = pq2_.buckets_[pq2_.get_next_bucket()].back().cost();
      // Terminate only if BOTH searches have costs exceeding the best meetpoint cost
      if (min_f > best_cost_ && min_r > best_cost_) {
        if (kDebug) {
          std::cout << "μ-termination: both searches exceeded best cost " 
                    << min_f << " " << min_r << " > " << best_cost_ << std::endl;
        }
        return false;
      }
    }

    return true;
  }

  template <direction SearchDir, bool WithBlocked>
  bool run(ways const& w,
           ways::routing const& r,
           cost_t const max,
           bitvec<node_idx_t> const* blocked,
           sharing_data const* sharing,
           elevation_storage const* elevations) {
    if constexpr (kDebugMaps) {
      std::cout << "\n=== Starting bidirectional search with max_cost=" << max << " ===\n";
      std::cout << "Initial PQ sizes: pq1=" << pq1_.size() << ", pq2=" << pq2_.size() << "\n";
    }
    
    while (!pq1_.empty() || !pq2_.empty()) {
      if (!pq1_.empty()) {
        if (!run_single<SearchDir, WithBlocked>(w, r, max, blocked, sharing,
                                                elevations, pq1_, cost1_)) {
          break;
        }
      }
      if (!pq2_.empty()) {
        if (!run_single<opposite(SearchDir), WithBlocked>(
              w, r, max, blocked, sharing, elevations, pq2_, cost2_)) {
          break;
        }
      }
    }
    
    if (best_cost_ != kInfeasible && best_cost_ > max) {
      clear_mp();
      return false;
    }
    return !max_reached_1_ || !max_reached_2_;
  }

  bool run(ways const& w,
           ways::routing const& r,
           cost_t const max,
           bitvec<node_idx_t> const* blocked,
           sharing_data const* sharing,
           elevation_storage const* elevations,
           direction const dir) {
    if (blocked == nullptr) {
      return dir == direction::kForward
                 ? run<direction::kForward, false>(w, r, max, blocked, sharing,
                                                   elevations)
                 : run<direction::kBackward, false>(w, r, max, blocked, sharing,
                                                    elevations);
    } else {
      return dir == direction::kForward
                 ? run<direction::kForward, true>(w, r, max, blocked, sharing,
                                                  elevations)
                 : run<direction::kBackward, true>(w, r, max, blocked, sharing,
                                                   elevations);
    }
  }

  dial<label, get_bucket> pq1_{get_bucket{}};
  dial<label, get_bucket> pq2_{get_bucket{}};
  location start_loc_;
  location end_loc_;
  node meet_point_1_;
  node meet_point_2_;
  cost_t best_cost_;
  cost_map cost1_;
  cost_map cost2_;
  bool max_reached_1_;
  bool max_reached_2_;
  static adjacency_map legal_successors_;
  static adjacency_map legal_predecessors_;
};

// Static member definitions
inline bidirectional_car_dijkstra::adjacency_map bidirectional_car_dijkstra::legal_successors_;
inline bidirectional_car_dijkstra::adjacency_map bidirectional_car_dijkstra::legal_predecessors_;

}  // namespace osr