#pragma once

#include <algorithm>
#include <limits>
#include <queue>
#include <random>
#include <vector>

#include "osr/routing/ch_data.h"
#include "osr/routing/dijkstra.h"
#include "osr/routing/profiles/car.h"
#include "osr/types.h"
#include "osr/ways.h"

namespace osr {

struct ch_preprocessing {
  static constexpr bool kDebug = false;
  
  struct node_importance {
    car_ch_key key_;
    int edge_diff_;  // shortcuts_created - neighbors_removed
    int degree_;
    
    bool operator<(node_importance const& other) const {
      // Lower importance values get contracted first
      if (edge_diff_ != other.edge_diff_) {
        return edge_diff_ < other.edge_diff_;
      }
      return degree_ < other.degree_;
    }
  };
  
  struct witness_search_entry {
    node_idx_t node_;
    cost_t cost_;
    
    bool operator>(witness_search_entry const& other) const {
      return cost_ > other.cost_;
    }
  };
  
  static ch_data preprocess(ways const& w) {
    ch_data ch;
    auto const& r = *w.r_;
    
    // Collect all car states from all nodes
    std::vector<car_ch_key> all_car_states;
    for (auto n = node_idx_t{0}; n.v_ < w.n_nodes(); ++n.v_) {
      car::resolve_all(r, n, level_t{static_cast<std::uint8_t>(0U)}, [&](car::node const car_n) {
        car_ch_key key{car_n.n_, car_n.way_, car_n.dir_};
        all_car_states.push_back(key);
      });
    }
    
    if (kDebug) {
      std::cout << "Total car states: " << all_car_states.size() << std::endl;
    }
    
    // For now, use a simpler approach: contract leaf nodes first, then nodes by degree
    // This ensures we maintain connectivity while building hierarchy
    
    std::vector<node_importance> importance_queue;
    ankerl::unordered_dense::set<car_ch_key, car_ch_key_hash> empty_contracted;
    
    for (auto const& key : all_car_states) {
      std::vector<std::pair<car_ch_key, cost_t>> incoming;
      std::vector<std::pair<car_ch_key, cost_t>> outgoing;
      collect_car_neighbors(w, ch, key, empty_contracted, incoming, outgoing);
      
      auto const degree = static_cast<int>(incoming.size() + outgoing.size());
      
      // Prefer leaf nodes (degree <= 1), then lowest degree
      int priority = degree == 0 ? 0 : (degree == 1 ? 1 : degree + 10);
      
      importance_queue.push_back({key, priority, degree});
    }
    
    // Sort by importance (lowest first)
    std::sort(importance_queue.begin(), importance_queue.end());
    
    // Initialize all car states with max level (uncontracted)
    for (auto const& key : all_car_states) {
      ch.node_levels_[key] = static_cast<ch_level_t>(all_car_states.size());
    }
    
    ankerl::unordered_dense::set<car_ch_key, car_ch_key_hash> contracted;
    
    ch_level_t current_level = 0;
    for (auto const& importance : importance_queue) {
      auto const& key_to_contract = importance.key_;
      if (kDebug && current_level < 10) {  // Only debug first few contractions
        std::cout << "Contracting car state (node=" << key_to_contract.n_ 
                  << ", way=" << key_to_contract.way_ << ", dir=" << (key_to_contract.dir_ == direction::kForward ? "fwd" : "bwd")
                  << ") at level " << current_level << std::endl;
      }
      
      // Assign the level when the car state is actually contracted
      ch.node_levels_[key_to_contract] = current_level;
      
      std::vector<std::pair<car_ch_key, cost_t>> incoming;
      std::vector<std::pair<car_ch_key, cost_t>> outgoing;
      
      collect_car_neighbors(w, ch, key_to_contract, contracted, incoming, outgoing);
      
      for (auto const& [v_key, cost_v_u] : incoming) {
        for (auto const& [w_key, cost_u_w] : outgoing) {
          if (v_key == w_key) continue;
          
          // Use validated car routing cost instead of simple addition
          auto const validated_cost = validate_car_shortcut_cost(w, v_key, w_key, key_to_contract);
          if (!validated_cost.has_value()) {
            // No valid car routing path - skip this shortcut
            continue;
          }
          
          auto const shortcut_cost = validated_cost.value();
          
          if (!needs_car_shortcut(w, ch, v_key, w_key, shortcut_cost, key_to_contract, contracted)) {
            continue;
          }
          
          ch.add_shortcut(v_key, w_key, shortcut_cost, key_to_contract, key_to_contract,
                         way_idx_t::invalid(), way_idx_t::invalid());
          
          if (kDebug && current_level < 5) {  // Only debug first few contractions
            std::cout << "  Added shortcut (" << v_key.n_ << "," << v_key.way_ << ") -> (" 
                     << w_key.n_ << "," << w_key.way_ << ") cost=" << shortcut_cost 
                     << " (validated vs simple=" << (cost_v_u + cost_u_w) << ")" << std::endl;
          }
        }
      }
      
      contracted.insert(key_to_contract);
      ++current_level;
    }
    
    return ch;
  }
  
private:
  static std::optional<cost_t> validate_car_shortcut_cost(ways const& w,
                                                           car_ch_key const& from_key,
                                                           car_ch_key const& to_key,
                                                           car_ch_key const& via_key) {
    // Use more sophisticated pathfinding with Dijkstra to get actual car routing cost
    auto const& r = *w.r_;
    
    car::node const from_node{from_key.n_, from_key.way_, from_key.dir_};
    car::node const via_node{via_key.n_, via_key.way_, via_key.dir_};
    car::node const to_node{to_key.n_, to_key.way_, to_key.dir_};
    
    // Use a simple Dijkstra-like search to find the actual cost
    std::priority_queue<car_witness_entry, 
                       std::vector<car_witness_entry>,
                       std::greater<car_witness_entry>> pq;
    ankerl::unordered_dense::map<car_ch_key, cost_t, car_ch_key_hash> costs;
    
    pq.push({from_key, 0});
    costs[from_key] = 0;
    
    bool found_via = false;
    cost_t cost_to_via = 0;
    
    // Search from → via
    while (!pq.empty()) {
      auto const [curr_key, curr_cost] = pq.top();
      pq.pop();
      
      if (curr_cost > costs[curr_key]) {
        continue;
      }
      
      if (curr_key.n_ == via_key.n_ && curr_key.way_ == via_key.way_ && curr_key.dir_ == via_key.dir_) {
        found_via = true;
        cost_to_via = curr_cost;
        break;
      }
      
      // Explore neighbors
      car::node const curr_node{curr_key.n_, curr_key.way_, curr_key.dir_};
      car::template adjacent<direction::kForward, false>(
          r, curr_node, nullptr, nullptr, nullptr,
          [&](car::node const neighbor, std::uint32_t const cost, distance_t,
              way_idx_t const, std::uint16_t, std::uint16_t,
              elevation_storage::elevation const, bool const) {
            car_ch_key neighbor_key{neighbor.n_, neighbor.way_, neighbor.dir_};
            auto const new_cost = static_cast<cost_t>(curr_cost + cost);
            
            auto it = costs.find(neighbor_key);
            if (it == costs.end() || new_cost < it->second) {
              costs[neighbor_key] = new_cost;
              pq.push({neighbor_key, new_cost});
            }
          });
    }
    
    if (!found_via) {
      return std::nullopt; // Can't reach via node
    }
    
    // Clear for second search: via → to
    pq = std::priority_queue<car_witness_entry, 
                            std::vector<car_witness_entry>,
                            std::greater<car_witness_entry>>();
    costs.clear();
    
    pq.push({via_key, 0});
    costs[via_key] = 0;
    
    // Search via → to
    while (!pq.empty()) {
      auto const [curr_key, curr_cost] = pq.top();
      pq.pop();
      
      if (curr_cost > costs[curr_key]) {
        continue;
      }
      
      if (curr_key.n_ == to_key.n_ && curr_key.way_ == to_key.way_ && curr_key.dir_ == to_key.dir_) {
        return cost_to_via + curr_cost; // Found path
      }
      
      // Explore neighbors
      car::node const curr_node{curr_key.n_, curr_key.way_, curr_key.dir_};
      car::template adjacent<direction::kForward, false>(
          r, curr_node, nullptr, nullptr, nullptr,
          [&](car::node const neighbor, std::uint32_t const cost, distance_t,
              way_idx_t const, std::uint16_t, std::uint16_t,
              elevation_storage::elevation const, bool const) {
            car_ch_key neighbor_key{neighbor.n_, neighbor.way_, neighbor.dir_};
            auto const new_cost = static_cast<cost_t>(curr_cost + cost);
            
            auto it = costs.find(neighbor_key);
            if (it == costs.end() || new_cost < it->second) {
              costs[neighbor_key] = new_cost;
              pq.push({neighbor_key, new_cost});
            }
          });
    }
    
    return std::nullopt; // Can't reach target
  }

  static node_importance calculate_node_importance(ways const& w,
                                                    ch_data const& ch,
                                                    car_ch_key const& key,
                                                    ankerl::unordered_dense::set<car_ch_key, car_ch_key_hash> const& contracted) {
    std::vector<std::pair<car_ch_key, cost_t>> incoming;
    std::vector<std::pair<car_ch_key, cost_t>> outgoing;
    
    collect_car_neighbors(w, ch, key, contracted, incoming, outgoing);
    
    int shortcuts_needed = 0;
    
    // Calculate how many shortcuts would be created
    for (auto const& [v_key, cost_v_u] : incoming) {
      for (auto const& [w_key, cost_u_w] : outgoing) {
        if (v_key == w_key) continue;
        
        // Use validated cost for importance calculation too
        auto const validated_cost = validate_car_shortcut_cost(w, v_key, w_key, key);
        if (!validated_cost.has_value()) {
          continue; // No valid car routing path
        }
        
        auto const shortcut_cost = validated_cost.value();
        
        if (needs_car_shortcut(w, ch, v_key, w_key, shortcut_cost, key, contracted)) {
          shortcuts_needed++;
        }
      }
    }
    
    auto const degree = static_cast<int>(incoming.size() + outgoing.size());
    auto const edge_diff = shortcuts_needed - degree;
    
    return {key, edge_diff, degree};
  }
  static void collect_car_neighbors(ways const& w,
                                    ch_data const& ch,
                                    car_ch_key const& car_state,
                                    ankerl::unordered_dense::set<car_ch_key, car_ch_key_hash> const& contracted,
                                    std::vector<std::pair<car_ch_key, cost_t>>& incoming,
                                    std::vector<std::pair<car_ch_key, cost_t>>& outgoing) {
    auto const& r = *w.r_;
    car::node const n{car_state.n_, car_state.way_, car_state.dir_};
    
    // Get outgoing neighbors
    car::template adjacent<direction::kForward, false>(
        r, n, nullptr, nullptr, nullptr,
        [&](car::node const neighbor, std::uint32_t const cost,
            distance_t, way_idx_t const, std::uint16_t, std::uint16_t,
            elevation_storage::elevation const, bool const) {
          car_ch_key neighbor_key{neighbor.n_, neighbor.way_, neighbor.dir_};
          if (contracted.find(neighbor_key) == contracted.end()) {
            outgoing.emplace_back(neighbor_key, cost);
          }
        });
    
    // Get incoming neighbors  
    car::template adjacent<direction::kBackward, false>(
        r, n, nullptr, nullptr, nullptr,
        [&](car::node const neighbor, std::uint32_t const cost,
            distance_t, way_idx_t const, std::uint16_t, std::uint16_t,
            elevation_storage::elevation const, bool const) {
          car_ch_key neighbor_key{neighbor.n_, neighbor.way_, neighbor.dir_};
          if (contracted.find(neighbor_key) == contracted.end()) {
            incoming.emplace_back(neighbor_key, cost);
          }
        });
    
    // Add shortcuts from this car state
    auto const* fwd_shortcuts = ch.get_forward_shortcuts(car_state);
    if (fwd_shortcuts) {
      for (auto const& sc : *fwd_shortcuts) {
        if (contracted.find(sc.to_) == contracted.end()) {
          outgoing.emplace_back(sc.to_, sc.cost_);
        }
      }
    }
    
    auto const* bwd_shortcuts = ch.get_backward_shortcuts(car_state);
    if (bwd_shortcuts) {
      for (auto const& sc : *bwd_shortcuts) {
        if (contracted.find(sc.from_) == contracted.end()) {
          incoming.emplace_back(sc.from_, sc.cost_);
        }
      }
    }
  }
  
  struct car_witness_entry {
    car_ch_key key_;
    cost_t cost_;
    
    bool operator>(car_witness_entry const& other) const {
      return cost_ > other.cost_;
    }
  };
  
  static bool needs_car_shortcut(ways const& w,
                                  ch_data const& ch,
                                  car_ch_key const& from,
                                  car_ch_key const& to,
                                  cost_t shortcut_cost,
                                  car_ch_key const& contracted_key,
                                  ankerl::unordered_dense::set<car_ch_key, car_ch_key_hash> const& contracted) {
    
    std::priority_queue<car_witness_entry, 
                       std::vector<car_witness_entry>,
                       std::greater<car_witness_entry>> pq;
    ankerl::unordered_dense::map<car_ch_key, cost_t, car_ch_key_hash> costs;
    
    pq.push({from, 0});
    costs[from] = 0;
    
    while (!pq.empty()) {
      auto const [curr_key, curr_cost] = pq.top();
      pq.pop();
      
      if (curr_cost > shortcut_cost) {
        break;
      }
      
      if (curr_key == to) {
        return false;  // Found witness path - no shortcut needed
      }
      
      if (costs[curr_key] < curr_cost) {
        continue;
      }
      
      explore_car_neighbors_for_witness(w, ch, curr_key, to, curr_cost, 
                                       shortcut_cost, contracted_key, 
                                       contracted, pq, costs);
    }
    
    return true;  // No witness found - shortcut needed
  }
  
  static void explore_car_neighbors_for_witness(
      ways const& w,
      ch_data const& ch,
      car_ch_key const& curr_key,
      car_ch_key const& target,
      cost_t curr_cost,
      cost_t max_cost,
      car_ch_key const& excluded_key,
      ankerl::unordered_dense::set<car_ch_key, car_ch_key_hash> const& contracted,
      std::priority_queue<car_witness_entry, 
                         std::vector<car_witness_entry>,
                         std::greater<car_witness_entry>>& pq,
      ankerl::unordered_dense::map<car_ch_key, cost_t, car_ch_key_hash>& costs) {
    
    auto const& r = *w.r_;
    car::node const n{curr_key.n_, curr_key.way_, curr_key.dir_};
    
    // Explore adjacent car states
    car::template adjacent<direction::kForward, false>(
        r, n, nullptr, nullptr, nullptr,
        [&](car::node const neighbor, std::uint32_t const cost,
            distance_t, way_idx_t const, std::uint16_t, std::uint16_t,
            elevation_storage::elevation const, bool const) {
          car_ch_key neighbor_key{neighbor.n_, neighbor.way_, neighbor.dir_};
          if (neighbor_key == excluded_key || 
              contracted.find(neighbor_key) != contracted.end()) {
            return;
          }
          
          auto const new_cost = static_cast<cost_t>(curr_cost + cost);
          if (new_cost <= max_cost) {
            auto it = costs.find(neighbor_key);
            if (it == costs.end() || new_cost < it->second) {
              costs[neighbor_key] = new_cost;
              pq.push({neighbor_key, new_cost});
            }
          }
        });
    
    // Explore shortcuts from this car state
    auto const* shortcuts = ch.get_forward_shortcuts(curr_key);
    if (shortcuts) {
      for (auto const& sc : *shortcuts) {
        if (sc.to_ == excluded_key || 
            contracted.find(sc.to_) != contracted.end()) {
          continue;
        }
        
        auto const new_cost = static_cast<cost_t>(curr_cost + sc.cost_);
        if (new_cost <= max_cost) {
          auto it = costs.find(sc.to_);
          if (it == costs.end() || new_cost < it->second) {
            costs[sc.to_] = new_cost;
            pq.push({sc.to_, new_cost});
          }
        }
      }
    }
  }
};

}  // namespace osr