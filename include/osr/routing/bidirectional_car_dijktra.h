#pragma once

#include <algorithm>
#include <limits>
#include <optional>
#include <queue>
#include <random>
#include <vector>

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
    
    bool operator<(const car_state& other) const {
      if (n != other.n) return to_idx(n) < to_idx(other.n);
      if (way != other.way) return way < other.way;
      return dir < other.dir;
    }
    
    bool operator>(const car_state& other) const {
      return other < *this;
    }
    
    bool operator<=(const car_state& other) const {
      return !(other < *this);
    }
    
    bool operator>=(const car_state& other) const {
      return !(*this < other);
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
    std::optional<node> middle_node;  // For CH shortcuts - stores the bypassed node
    
    // Constructor for normal edges
    edge_transition(node const& t, cost_t c, distance_t d, way_idx_t w, 
                   std::uint16_t f, std::uint16_t to_idx)
      : target(t), cost(c), dist(d), way(w), from(f), to(to_idx), middle_node() {}
    
    // Constructor for shortcuts
    edge_transition(node const& t, cost_t c, node const& middle)
      : target(t), cost(c), dist(0), way(way_idx_t::invalid()), 
        from(0), to(0), middle_node(middle) {}
  };

  using adjacency_map = ankerl::unordered_dense::map<car_state, std::vector<edge_transition>, car_state_hash>;

  constexpr static auto const kDebug = false;
  constexpr static auto const kDebugMaps = false;  // Set to true to see adjacency map structure

  struct get_bucket {
    cost_t operator()(label const& l) { return l.cost(); }
  };
  
  bidirectional_car_dijkstra() {}

  void clear_mp() {
    meet_point_1_ = node::invalid();
    meet_point_2_ = node::invalid();
    best_cost_ = kInfeasible;
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
    
    // Only clear adjacency maps if CH is not preprocessed
    if (!is_preprocessed_) {
      legal_successors_.clear();
      legal_predecessors_.clear();
    }
  }

  template <direction SearchDir, bool WithBlocked>
  void build_adjacency_for_node(ways const& w,
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
            legal_successors_[source_key].emplace_back(target, static_cast<cost_t>(cost), dist, way, from, to);
          } else {
            // For backward search, store as predecessors  
            legal_predecessors_[source_key].emplace_back(target, static_cast<cost_t>(cost), dist, way, from, to);
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
        std::cout << "No transitions stored for this node in " 
                  << (SearchDir == direction::kForward ? "legal_successors" : "legal_predecessors") 
                  << " map\n";
      }
      std::cout << "=================================\n";
    }
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
        
        // Build adjacency if not already cached
        if (opp_adj_map.find(curr_key) == opp_adj_map.end()) {
          build_adjacency_for_node<opposite(SearchDir), WithBlocked>(w, r, curr, blocked, sharing, elevations);
        }
        
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

  // Enumerate all reachable car_states for CH preprocessing
  template <bool WithBlocked>
  void enumerate_all_car_states(ways const& w,
                                ways::routing const& r,
                                bitvec<node_idx_t> const* blocked,
                                sharing_data const* sharing,
                                elevation_storage const* elevations) {
    all_nodes_.clear();
    ankerl::unordered_dense::set<car_state, car_state_hash> visited;
    
    if constexpr (kDebugMaps) {
      std::cout << "\n=== Enumerating all car states ===\n";
    }
    
    // Start from all physical nodes and enumerate all car states
    for (node_idx_t n{0U}; to_idx(n) < r.node_ways_.size(); ++n) {
      if constexpr (WithBlocked) {
        if (blocked && blocked->test(n)) {
          continue;
        }
      }
      
      auto const& ways = r.node_ways_[n];
      for (way_pos_t way_pos{0U}; way_pos < ways.size(); ++way_pos) {
        for (auto dir : {direction::kForward, direction::kBackward}) {
          car_state state{n, way_pos, dir};
          if (visited.find(state) == visited.end()) {
            visited.insert(state);
            all_nodes_.push_back(state);
          }
        }
      }
    }
    
    if constexpr (kDebugMaps) {
      std::cout << "Enumerated " << all_nodes_.size() << " car states\n";
      std::cout << "=====================================\n";
    }
  }

  // Assign random levels to all car states
  void assign_random_levels() {
    node_levels_.clear();
    
    if constexpr (kDebugMaps) {
      std::cout << "\n=== Assigning random levels ===\n";
      std::cout << "Number of nodes: " << all_nodes_.size() << "\n";
    }
    
    // Create a vector with levels 1 to n
    std::vector<std::uint32_t> levels;
    levels.reserve(all_nodes_.size());
    for (std::uint32_t i = 1; i <= all_nodes_.size(); ++i) {
      levels.push_back(i);
    }
    
    // Shuffle the levels randomly
    std::shuffle(levels.begin(), levels.end(), rng_);
    
    // Assign shuffled levels to nodes
    for (std::size_t i = 0; i < all_nodes_.size(); ++i) {
      node_levels_[all_nodes_[i]] = levels[i];
    }
    
    if constexpr (kDebugMaps) {
      std::cout << "Assigned random levels to " << node_levels_.size() << " nodes\n";
      // Show first few assignments for debugging
      for (std::size_t i = 0; i < std::min(static_cast<std::size_t>(5), all_nodes_.size()); ++i) {
        auto const& state = all_nodes_[i];
        std::cout << "  Node {" << to_idx(state.n) << "," << static_cast<int>(state.way) 
                  << "," << (state.dir == direction::kForward ? "FWD" : "BWD") 
                  << "} -> level " << node_levels_[state] << "\n";
      }
      std::cout << "===============================\n";
    }
  }

  // Witness search - check if shortcut (v,w) via u is necessary
  template <bool WithBlocked>
  bool witness_search(ways const& w,
                      ways::routing const& r,
                      car_state const& source,
                      car_state const& target,
                      car_state const& contracted_node,
                      cost_t const shortcut_cost,
                      bitvec<node_idx_t> const* blocked,
                      sharing_data const* sharing,
                      elevation_storage const* elevations) {
    
    if constexpr (kDebugMaps) {
      std::cout << "  Witness search from {" << to_idx(source.n) << "," 
                << static_cast<int>(source.way) << "," 
                << (source.dir == direction::kForward ? "FWD" : "BWD") << "} to {"
                << to_idx(target.n) << "," << static_cast<int>(target.way) << ","
                << (target.dir == direction::kForward ? "FWD" : "BWD") 
                << "} avoiding {" << to_idx(contracted_node.n) << "," 
                << static_cast<int>(contracted_node.way) << ","
                << (contracted_node.dir == direction::kForward ? "FWD" : "BWD")
                << "} cost=" << shortcut_cost << "\n";
    }
    
    // Priority queue for Dijkstra
    std::priority_queue<std::pair<cost_t, car_state>,
                       std::vector<std::pair<cost_t, car_state>>,
                       std::greater<>> pq;
    
    ankerl::unordered_dense::map<car_state, cost_t, car_state_hash> distances;
    
    pq.emplace(0, source);
    distances[source] = 0;
    
    while (!pq.empty()) {
      auto const [cost, current] = pq.top();
      pq.pop();
      
      // If we reached target, compare cost
      if (current == target) {
        if constexpr (kDebugMaps) {
          std::cout << "    Found witness path with cost " << cost 
                    << " (shortcut cost " << shortcut_cost << ")\n";
        }
        return cost <= shortcut_cost; // Witness found if alternative is cheaper/equal
      }
      
      // Skip if we already found a better path to this node
      auto const it = distances.find(current);
      if (it != distances.end() && cost > it->second) {
        continue;
      }
      
      // Don't expand beyond shortcut cost
      if (cost >= shortcut_cost) {
        continue;
      }
      
      // Skip the contracted node
      if (current == contracted_node) {
        continue;
      }
      
      // Expand neighbors (only to nodes with level > contracted node level)
      auto const u_level = node_levels_[contracted_node];
      
      // Build adjacency if not cached
      if (legal_successors_.find(current) == legal_successors_.end()) {
        node curr_node{current.n, current.way, current.dir};
        build_adjacency_for_node<direction::kForward, WithBlocked>(
            w, r, curr_node, blocked, sharing, elevations);
      }
      
      // Expand normal edges
      auto const adj_it = legal_successors_.find(current);
      if (adj_it != legal_successors_.end()) {
        for (auto const& edge : adj_it->second) {
          car_state neighbor{edge.target.n_, edge.target.way_, edge.target.dir_};
          
          // CH constraint: only expand to nodes with level > contracted node level
          if (node_levels_[neighbor] <= u_level) {
            continue;
          }
          
          cost_t new_cost = cost + edge.cost;
          auto const neighbor_it = distances.find(neighbor);
          
          if (neighbor_it == distances.end() || new_cost < neighbor_it->second) {
            distances[neighbor] = new_cost;
            pq.emplace(new_cost, neighbor);
          }
        }
      }
      
      // ALSO expand existing shortcuts (from previous contractions)
      auto const shortcut_it = shortcut_successors_.find(current);
      if (shortcut_it != shortcut_successors_.end()) {
        for (auto const& shortcut : shortcut_it->second) {
          car_state neighbor{shortcut.target.n_, shortcut.target.way_, shortcut.target.dir_};
          
          // CH constraint: only expand to nodes with level > contracted node level
          if (node_levels_[neighbor] <= u_level) {
            continue;
          }
          
          cost_t new_cost = cost + shortcut.cost;
          auto const neighbor_it = distances.find(neighbor);
          
          if (neighbor_it == distances.end() || new_cost < neighbor_it->second) {
            distances[neighbor] = new_cost;
            pq.emplace(new_cost, neighbor);
            
            if constexpr (kDebugMaps) {
              fmt::println("    Using existing shortcut {} -> {} [cost={}] in witness search", 
                         current.n, neighbor.n, shortcut.cost);
            }
          }
        }
      }
    }
    
    if constexpr (kDebugMaps) {
      std::cout << "    No witness path found - shortcut needed\n";
    }
    return false; // No witness found, shortcut is needed
  }

  // Main CH preprocessing method
  template <bool WithBlocked>
  void preprocess_contraction_hierarchies(ways const& w,
                                          ways::routing const& r,
                                          bitvec<node_idx_t> const* blocked,
                                          sharing_data const* sharing,
                                          elevation_storage const* elevations) {
    
    if constexpr (kDebugMaps) {
      std::cout << "\n=== Starting CH Preprocessing ===\n";
    }
    
    // Step 1: Enumerate all car states
    enumerate_all_car_states<WithBlocked>(w, r, blocked, sharing, elevations);
    
    // Step 2: Assign random levels
    assign_random_levels();
    
    // Step 3: Sort nodes by level for contraction order
    std::sort(all_nodes_.begin(), all_nodes_.end(), 
              [&](car_state const& a, car_state const& b) {
                return node_levels_[a] < node_levels_[b];
              });
    
    if constexpr (kDebugMaps) {
      std::cout << "Contracting nodes in level order...\n";
    }
    
    std::size_t shortcuts_added = 0;
    
    // Step 4: Contract nodes in level order
    for (std::size_t idx = 0; idx < all_nodes_.size(); ++idx) {
      auto const& contracted_node = all_nodes_[idx];
      auto const contracted_level = node_levels_[contracted_node];
      
      if constexpr (kDebugMaps) {
        if (idx % 1000 == 0) {
          std::cout << "Contracting node " << idx << "/" << all_nodes_.size() 
                    << " (level " << contracted_level << ")\n";
        }
      }
      
      // Find all predecessors with higher level
      std::vector<car_state> predecessors;
      if (legal_predecessors_.find(contracted_node) == legal_predecessors_.end()) {
        node curr_node{contracted_node.n, contracted_node.way, contracted_node.dir};
        build_adjacency_for_node<direction::kBackward, WithBlocked>(
            w, r, curr_node, blocked, sharing, elevations);
      }
      
      auto const pred_it = legal_predecessors_.find(contracted_node);
      if (pred_it != legal_predecessors_.end()) {
        for (auto const& edge : pred_it->second) {
          car_state pred{edge.target.n_, edge.target.way_, edge.target.dir_};
          if (node_levels_[pred] > contracted_level) {
            predecessors.push_back(pred);
          }
        }
      }
      
      // Find all successors with higher level
      std::vector<car_state> successors;
      if (legal_successors_.find(contracted_node) == legal_successors_.end()) {
        node curr_node{contracted_node.n, contracted_node.way, contracted_node.dir};
        build_adjacency_for_node<direction::kForward, WithBlocked>(
            w, r, curr_node, blocked, sharing, elevations);
      }
      
      auto const succ_it = legal_successors_.find(contracted_node);
      if (succ_it != legal_successors_.end()) {
        for (auto const& edge : succ_it->second) {
          car_state succ{edge.target.n_, edge.target.way_, edge.target.dir_};
          if (node_levels_[succ] > contracted_level) {
            successors.push_back(succ);
          }
        }
      }
      
      // Check all predecessor-successor pairs for shortcuts
      for (auto const& pred : predecessors) {
        // Ensure pred's forward adjacency is built
        if (legal_successors_.find(pred) == legal_successors_.end()) {
          node pred_node{pred.n, pred.way, pred.dir};
          build_adjacency_for_node<direction::kForward, WithBlocked>(
              w, r, pred_node, blocked, sharing, elevations);
        }
        
        auto const& pred_edges = legal_successors_[pred];
        cost_t cost_to_contracted = kInfeasible;
        
        // Find cost from predecessor to contracted node
        for (auto const& edge : pred_edges) {
          car_state target{edge.target.n_, edge.target.way_, edge.target.dir_};
          if (target == contracted_node) {
            cost_to_contracted = edge.cost;
            break;
          }
        }
        
        if (cost_to_contracted == kInfeasible) continue;
        
        for (auto const& succ : successors) {
          if (pred == succ) continue; // Skip self-loops
          
          // Ensure succ's backward adjacency is built
          if (legal_predecessors_.find(succ) == legal_predecessors_.end()) {
            node succ_node{succ.n, succ.way, succ.dir};
            build_adjacency_for_node<direction::kBackward, WithBlocked>(
                w, r, succ_node, blocked, sharing, elevations);
          }
          
          auto const& succ_edges = legal_predecessors_[succ];
          cost_t cost_from_contracted = kInfeasible;
          
          // Find cost from contracted node to successor
          for (auto const& edge : succ_edges) {
            car_state source{edge.target.n_, edge.target.way_, edge.target.dir_};
            if (source == contracted_node) {
              cost_from_contracted = edge.cost;
              break;
            }
          }
          
          if (cost_from_contracted == kInfeasible) continue;
          
          cost_t shortcut_cost = cost_to_contracted + cost_from_contracted;
          
          // Perform witness search
          bool witness_exists = witness_search<WithBlocked>(
              w, r, pred, succ, contracted_node, shortcut_cost,
              blocked, sharing, elevations);
          
          if (!witness_exists) {
            // Add shortcut from pred to succ
            node contracted_car_node{contracted_node.n, contracted_node.way, contracted_node.dir};
            node succ_node{succ.n, succ.way, succ.dir};
            
            // Add shortcut to static shortcut maps
            // Check if shortcut already exists and update if necessary
            bool shortcut_added = false;
            auto& pred_shortcuts = shortcut_successors_[pred];
            for (auto& edge : pred_shortcuts) {
              car_state edge_target{edge.target.n_, edge.target.way_, edge.target.dir_};
              if (edge_target == succ) {
                if (shortcut_cost < edge.cost) {
                  edge.cost = shortcut_cost;
                  edge.middle_node = contracted_car_node;
                  shortcut_added = true;
                }
                break;
              }
            }
            
            if (!shortcut_added) {
              pred_shortcuts.emplace_back(succ_node, shortcut_cost, contracted_car_node);
              shortcut_predecessors_[succ].emplace_back(
                  node{pred.n, pred.way, pred.dir}, shortcut_cost, contracted_car_node);
              ++shortcuts_added;
            }
          }
        }
      }
    }
    
    is_preprocessed_ = true;
    
    // Always print shortcut count
    fmt::println("CH preprocessing complete! Added {} shortcuts", shortcuts_added);
    
    if constexpr (kDebugMaps) {
      std::cout << "===================================\n";
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
    if (pq.empty()) return true;

    auto const l = pq.pop();
    auto const curr = l.get_node();
    auto const curr_cost = get_cost<SearchDir>(curr);
    
    if (curr_cost < l.cost()) {
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
    
    // If not in map, need to build adjacency for this node
    if (adj_map.find(curr_key) == adj_map.end()) {
      build_adjacency_for_node<SearchDir, WithBlocked>(w, r, curr, blocked, sharing, elevations);
    }
    
    if constexpr (kDebugMaps) {
      auto it = adj_map.find(curr_key);
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
    
    auto it = adj_map.find(curr_key);
    if (it != adj_map.end()) {
      // Get current node level for CH filtering
      std::uint32_t curr_level = 0;
      if (is_preprocessed_) {
        auto level_it = node_levels_.find(curr_key);
        if (level_it != node_levels_.end()) {
          curr_level = level_it->second;
        }
      }
      
      for (auto const& edge : it->second) {
        if constexpr (kDebug) {
          std::cout << "  NEIGHBOR ";
          edge.target.print(std::cout, w);
        }
        
        // CH level filtering: forward search goes upward, backward goes upward too
        if (is_preprocessed_) {
          car_state target_state{edge.target.n_, edge.target.way_, edge.target.dir_};
          auto target_level_it = node_levels_.find(target_state);
          if (target_level_it != node_levels_.end()) {
            auto target_level = target_level_it->second;
            // Only follow edges to higher level nodes
            if (target_level <= curr_level) {
              if constexpr (kDebugMaps) {
                fmt::println("  CH SKIP: target level {} <= curr level {} (SearchDir={})", 
                           target_level, curr_level, 
                           SearchDir == direction::kForward ? "FWD" : "BWD");
              }
              continue;
            } else {
              if constexpr (kDebugMaps) {
                fmt::println("  CH ALLOW: target level {} > curr level {} (SearchDir={})", 
                           target_level, curr_level,
                           SearchDir == direction::kForward ? "FWD" : "BWD");
              }
            }
          } else {
            if constexpr (kDebugMaps) {
              fmt::println("  CH SKIP: target level not found");
            }
            continue;
          }
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

    // 2) Process shortcuts if CH is enabled
    if (is_preprocessed_) {
      auto const& shortcut_map = (SearchDir == direction::kForward) ? shortcut_successors_ : shortcut_predecessors_;
      auto shortcut_it = shortcut_map.find(curr_key);
      if (shortcut_it != shortcut_map.end()) {
        // Get current node level for CH filtering
        std::uint32_t curr_level = 0;
        auto level_it = node_levels_.find(curr_key);
        if (level_it != node_levels_.end()) {
          curr_level = level_it->second;
        }

        if constexpr (kDebugMaps) {
          std::cout << "\n>>> Processing shortcuts for ";
          curr.print(std::cout, w);
          std::cout << "\n    Found " << shortcut_it->second.size() << " shortcuts\n";
        }

        for (auto const& shortcut : shortcut_it->second) {
          // CH level filtering for shortcuts
          car_state target_state{shortcut.target.n_, shortcut.target.way_, shortcut.target.dir_};
          auto target_level_it = node_levels_.find(target_state);
          if (target_level_it != node_levels_.end()) {
            auto target_level = target_level_it->second;
            // Only follow shortcuts to higher level nodes
            if (target_level <= curr_level) {
              if constexpr (kDebugMaps) {
                fmt::println("  CH SKIP SHORTCUT: target level {} <= curr level {} (SearchDir={})", 
                           target_level, curr_level, 
                           SearchDir == direction::kForward ? "FWD" : "BWD");
              }
              continue;
            } else {
              if constexpr (kDebugMaps) {
                fmt::println("  CH ALLOW SHORTCUT: target level {} > curr level {} (SearchDir={})", 
                           target_level, curr_level,
                           SearchDir == direction::kForward ? "FWD" : "BWD");
              }
            }
          } else {
            if constexpr (kDebugMaps) {
              fmt::println("  CH SKIP SHORTCUT: target level not found");
            }
            continue;
          }

          auto const total = curr_cost + shortcut.cost;
          if (total >= max) {
            if (SearchDir == direction::kForward) {
              max_reached_1_ = true;
            } else {
              max_reached_2_ = true;
            }
            continue;
          }

          if (total < max &&
              costs[shortcut.target.get_key()].update(
                  l, shortcut.target, static_cast<cost_t>(total), curr)) {

            auto next = label{shortcut.target, static_cast<cost_t>(total)};
            // For shortcuts, we need to track the middle node info
            next.track(l, r, shortcut.way, shortcut.target.get_node(), false);
            pq.push(std::move(next));

            if constexpr (kDebugMaps) {
              std::cout << "    -> SHORTCUT PUSH\n";
            }
          } else {
            if constexpr (kDebugMaps) {
              std::cout << "    -> SHORTCUT DOMINATED\n";
            }
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
    while (!pq1_.empty() || !pq2_.empty()) {
      if (!pq1_.empty() &&
          !run_single<SearchDir, WithBlocked>(w, r, max, blocked, sharing,
                                              elevations, pq1_, cost1_)) {
        break;
      }
      if (!pq2_.empty() &&
          !run_single<opposite(SearchDir), WithBlocked>(
              w, r, max, blocked, sharing, elevations, pq2_, cost2_)) {
        break;
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

  // Unpack shortcuts recursively to get the original path
  /*std::vector<node> unpack_shortcut(node const& from, node const& to) const {
    if constexpr (kDebugMaps) {
      std::cout << "Unpacking shortcut from ";
      // Note: can't print without ways reference here
      std::cout << " to ";
      std::cout << "\n";
    }
    
    car_state from_state{from.n_, from.way_, from.dir_};
    
    // Find the shortcut edge
    auto const adj_it = legal_successors_.find(from_state);
    if (adj_it == legal_successors_.end()) {
      return {from, to}; // No adjacency info, return direct path
    }
    
    for (auto const& edge : adj_it->second) {
      car_state target_state{edge.target.n_, edge.target.way_, edge.target.dir_};
      car_state to_state{to.n_, to.way_, to.dir_};
      
      if (target_state == to_state && edge.middle_node.has_value()) {
        // This is a shortcut, unpack recursively
        auto const& middle = edge.middle_node.value();
        
        auto left_path = unpack_shortcut(from, middle);
        auto right_path = unpack_shortcut(middle, to);
        
        // Combine paths, avoiding duplicate middle node
        std::vector<node> result = left_path;
        result.insert(result.end(), right_path.begin() + 1, right_path.end());
        
        if constexpr (kDebugMaps) {
          std::cout << "Unpacked shortcut into " << result.size() << " nodes\n";
        }
        
        return result;
      }
    }
    
    // No shortcut found, return direct path
    return {from, to};
  }*/

  std::vector<node> unpack_shortcut(node const& from, node const& to) const {
  car_state from_state{from.n_, from.way_, from.dir_};

  // look in shortcuts, not legal edges
  auto it = shortcut_successors_.find(from_state);
  if (it == shortcut_successors_.end()) {
    return {from, to};
  }

  car_state to_state{to.n_, to.way_, to.dir_};
  for (auto const& edge : it->second) {
    car_state target_state{edge.target.n_, edge.target.way_, edge.target.dir_};
    if (target_state == to_state && edge.middle_node.has_value()) {
      auto const& m = *edge.middle_node;
      auto left  = unpack_shortcut(from, m);
      auto right = unpack_shortcut(m, to);
      left.insert(left.end(), right.begin() + 1, right.end());
      return left;
    }
  }
  return {from, to};
}


  // Public interface for CH preprocessing
  void enable_contraction_hierarchies(ways const& w,
                                      ways::routing const& r,
                                      bitvec<node_idx_t> const* blocked = nullptr,
                                      sharing_data const* sharing = nullptr,
                                      elevation_storage const* elevations = nullptr) {
    if (blocked == nullptr) {
      preprocess_contraction_hierarchies<false>(w, r, blocked, sharing, elevations);
    } else {
      preprocess_contraction_hierarchies<true>(w, r, blocked, sharing, elevations);
    }
  }

  bool is_ch_enabled() const { return is_preprocessed_; }
  bool is_ch_preprocessed() const { return is_preprocessed_; }

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
  adjacency_map legal_successors_;
  adjacency_map legal_predecessors_;
  
  // Global CH data shared across all instances
  static inline bool is_preprocessed_ = false;
  static inline ankerl::unordered_dense::map<car_state, std::uint32_t, car_state_hash> node_levels_;
  static inline std::vector<car_state> all_nodes_;  // All reachable car_states
  static inline adjacency_map shortcut_successors_;
  static inline adjacency_map shortcut_predecessors_;
  static inline std::mt19937 rng_{std::random_device{}()};
};

}  // namespace osr