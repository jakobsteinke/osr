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
    double routing_centrality_;  // How important this node is for routing
    double connectivity_score_;  // How critical for overall connectivity
    
    bool operator<(node_importance const& other) const {
      // Lower importance values get contracted first
      // Higher routing_centrality and connectivity_score mean MORE important -> contract LATER
      
      // Primary: routing centrality (higher = more important = contract later)
      if (std::abs(routing_centrality_ - other.routing_centrality_) > 0.001) {
        return routing_centrality_ < other.routing_centrality_;
      }
      
      // Secondary: connectivity score (higher = more important = contract later) 
      if (std::abs(connectivity_score_ - other.connectivity_score_) > 0.001) {
        return connectivity_score_ < other.connectivity_score_;
      }
      
      // Tertiary: edge difference (fewer shortcuts needed = less important = contract earlier)
      if (edge_diff_ != other.edge_diff_) {
        return edge_diff_ < other.edge_diff_;
      }
      
      // Final: degree (lower degree = less important = contract earlier)
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
    
    // Use a simpler, more conservative approach: contract by degree only
    // Higher degree nodes get lower levels (contracted later) to preserve connectivity
    
    std::vector<node_importance> importance_queue;
    ankerl::unordered_dense::set<car_ch_key, car_ch_key_hash> empty_contracted;
    
    if (kDebug) {
      std::cout << "Calculating node degrees..." << std::endl;
    }
    
    for (auto const& key : all_car_states) {
      std::vector<std::pair<car_ch_key, cost_t>> incoming;
      std::vector<std::pair<car_ch_key, cost_t>> outgoing;
      collect_car_neighbors(w, ch, key, empty_contracted, incoming, outgoing);
      
      auto const degree = static_cast<int>(incoming.size() + outgoing.size());
      
      // Simple strategy: use degree as the primary importance metric
      // Low degree nodes get contracted first (high level)
      // High degree nodes get contracted last (low level)
      double routing_centrality = static_cast<double>(degree);
      double connectivity_score = static_cast<double>(degree);
      
      // Edge difference calculation (simplified)
      int edge_diff = degree;
      
      importance_queue.push_back({key, edge_diff, degree, routing_centrality, connectivity_score});
    }
    
    // Sort by importance (lowest degree first - these get contracted early)
    std::sort(importance_queue.begin(), importance_queue.end(), 
              [](node_importance const& a, node_importance const& b) {
                return a.degree_ < b.degree_;
              });
    
    ankerl::unordered_dense::set<car_ch_key, car_ch_key_hash> contracted;
    
    ch_level_t current_level = 0;
    for (auto const& importance : importance_queue) {
      auto const& key_to_contract = importance.key_;
      
      // CRITICAL FIX: Assign REVERSE levels - first contracted gets HIGHEST level
      // This ensures peripheral nodes (contracted early) get high levels
      // and important nodes (contracted late) get low levels  
      auto const final_level = static_cast<ch_level_t>(importance_queue.size() - current_level - 1);
      ch.node_levels_[key_to_contract] = final_level;
      
      if (kDebug && current_level < 30) {  // Show more contractions
        std::cout << "Contracting car state (node=" << key_to_contract.n_ 
                  << ", way=" << key_to_contract.way_ << ", dir=" << (key_to_contract.dir_ == direction::kForward ? "fwd" : "bwd")
                  << ") at level " << final_level << " (contraction order=" << current_level << ")" << std::endl;
      }
      
      std::vector<std::pair<car_ch_key, cost_t>> incoming;
      std::vector<std::pair<car_ch_key, cost_t>> outgoing;
      
      collect_car_neighbors(w, ch, key_to_contract, contracted, incoming, outgoing);
      
      for (auto const& [v_key, cost_v_u] : incoming) {
        for (auto const& [w_key, cost_u_w] : outgoing) {
          if (v_key == w_key) continue;
          
          // Temporarily use simple cost addition for testing
          auto const shortcut_cost = cost_v_u + cost_u_w;
          
          // Temporarily disable shortcut creation to test basic CH functionality
          // if (!needs_car_shortcut(w, ch, v_key, w_key, shortcut_cost, key_to_contract, contracted)) {
          //   continue;
          // }
          // 
          // ch.add_shortcut(v_key, w_key, shortcut_cost, key_to_contract, key_to_contract,
          //                way_idx_t::invalid(), way_idx_t::invalid());
          
          // Debug output disabled since shortcuts are not being created
          // if (kDebug && current_level < 20) {  // Show more contractions
          //   std::cout << "  Added shortcut (" << v_key.n_ << "," << v_key.way_ << ") -> (" 
          //            << w_key.n_ << "," << w_key.way_ << ") cost=" << shortcut_cost 
          //            << " (validated vs simple=" << (cost_v_u + cost_u_w) << ")" << std::endl;
          // }
        }
      }
      
      contracted.insert(key_to_contract);
      ++current_level;
    }
    
    return ch;
  }
  
private:
  static double calculate_routing_centrality(ways const& w, 
                                            car_ch_key const& center_key,
                                            std::vector<car_ch_key> const& all_states) {
    // Approximate betweenness centrality using local sampling
    // Sample nearby nodes and count shortest paths going through center node
    
    auto const& r = *w.r_;
    constexpr int kMaxSampleRadius = 3; // Local neighborhood radius
    constexpr int kMaxSamples = 20;     // Limit computational cost
    
    // Get local neighborhood around center node
    std::vector<car_ch_key> local_nodes;
    get_local_neighborhood(w, center_key, kMaxSampleRadius, local_nodes);
    
    if (local_nodes.size() < 3) {
      return 0.0; // Not enough neighbors for meaningful centrality
    }
    
    // Sample pairs from local neighborhood and count paths through center
    int paths_through_center = 0;
    int total_paths = 0;
    int samples = std::min(static_cast<int>(local_nodes.size()), kMaxSamples);
    
    for (int i = 0; i < samples && i < local_nodes.size(); ++i) {
      for (int j = i + 1; j < samples && j < local_nodes.size(); ++j) {
        if (local_nodes[i] == center_key || local_nodes[j] == center_key) {
          continue;
        }
        
        // Check if shortest path from i to j goes through center
        if (path_goes_through_center(w, local_nodes[i], local_nodes[j], center_key)) {
          paths_through_center++;
        }
        total_paths++;
        
        if (total_paths > 50) break; // Computational limit
      }
      if (total_paths > 50) break;
    }
    
    return total_paths > 0 ? static_cast<double>(paths_through_center) / total_paths : 0.0;
  }
  
  static double calculate_connectivity_score(ways const& w,
                                           car_ch_key const& node_key,
                                           std::vector<std::pair<car_ch_key, cost_t>> const& incoming,
                                           std::vector<std::pair<car_ch_key, cost_t>> const& outgoing) {
    // Calculate how critical this node is for local connectivity
    
    double score = 0.0;
    
    // Base score from degree (more connections = more important)
    int total_degree = static_cast<int>(incoming.size() + outgoing.size());
    score += std::log(1.0 + total_degree) * 2.0;
    
    // Bridge score: if removing this node would disconnect components
    // Simplified: nodes with high degree connecting different areas are important
    if (total_degree >= 3) {
      // Check if neighbors are well-connected to each other
      int neighbor_connections = count_neighbor_interconnections(w, incoming, outgoing);
      double expected_connections = (total_degree * (total_degree - 1)) / 2.0;
      
      if (neighbor_connections < expected_connections * 0.5) {
        // Neighbors are poorly connected -> this node is a bridge
        score += 5.0;
      }
    }
    
    // Penalty for leaf nodes (degree 0 or 1) - these are peripheral
    if (total_degree <= 1) {
      score -= 10.0; // Strong penalty for leaf nodes
    }
    
    return std::max(0.0, score);
  }
  
  static void get_local_neighborhood(ways const& w,
                                   car_ch_key const& center,
                                   int max_radius,
                                   std::vector<car_ch_key>& result) {
    // BFS to find local neighborhood within max_radius
    auto const& r = *w.r_;
    
    ankerl::unordered_dense::set<car_ch_key, car_ch_key_hash> visited;
    std::queue<std::pair<car_ch_key, int>> queue; // (node, distance)
    
    queue.push({center, 0});
    visited.insert(center);
    
    while (!queue.empty() && result.size() < 100) { // Limit result size
      auto [curr_key, dist] = queue.front();
      queue.pop();
      
      if (dist > 0) { // Don't include center itself
        result.push_back(curr_key);
      }
      
      if (dist >= max_radius) continue;
      
      // Explore neighbors
      car::node const curr_node{curr_key.n_, curr_key.way_, curr_key.dir_};
      car::template adjacent<direction::kForward, false>(
          r, curr_node, nullptr, nullptr, nullptr,
          [&](car::node const neighbor, std::uint32_t const, distance_t,
              way_idx_t const, std::uint16_t, std::uint16_t,
              elevation_storage::elevation const, bool const) {
            car_ch_key neighbor_key{neighbor.n_, neighbor.way_, neighbor.dir_};
            if (visited.find(neighbor_key) == visited.end()) {
              visited.insert(neighbor_key);
              queue.push({neighbor_key, dist + 1});
            }
          });
    }
  }
  
  static bool path_goes_through_center(ways const& w,
                                      car_ch_key const& from,
                                      car_ch_key const& to,
                                      car_ch_key const& center) {
    // Simplified check: is center on the shortest path from 'from' to 'to'?
    // This is computationally expensive, so we use a heuristic
    
    // Heuristic: if center is "between" from and to geographically, it's likely on the path
    auto from_pos = w.get_node_pos(from.n_).as_latlng();
    auto to_pos = w.get_node_pos(to.n_).as_latlng();
    auto center_pos = w.get_node_pos(center.n_).as_latlng();
    
    // Calculate if center is approximately on the line between from and to
    double dist_from_center = distance_to_line(from_pos, to_pos, center_pos);
    double from_to_dist = geo::distance(from_pos, to_pos);
    
    // If center is close to the line and not at the endpoints, it might be on the path
    return dist_from_center < from_to_dist * 0.1; // Within 10% of line distance
  }
  
  static int count_neighbor_interconnections(ways const& w,
                                           std::vector<std::pair<car_ch_key, cost_t>> const& incoming,
                                           std::vector<std::pair<car_ch_key, cost_t>> const& outgoing) {
    // Count how many neighbors are directly connected to each other
    // This is a simplified approximation
    
    std::vector<car_ch_key> all_neighbors;
    for (auto const& [key, _] : incoming) {
      all_neighbors.push_back(key);
    }
    for (auto const& [key, _] : outgoing) {
      all_neighbors.push_back(key);
    }
    
    int connections = 0;
    auto const& r = *w.r_;
    
    // Check connections between neighbors (simplified)
    for (size_t i = 0; i < all_neighbors.size() && i < 10; ++i) {
      for (size_t j = i + 1; j < all_neighbors.size() && j < 10; ++j) {
        // Simplified: assume neighbors on the same node are connected
        if (all_neighbors[i].n_ == all_neighbors[j].n_) {
          connections++;
        }
      }
    }
    
    return connections;
  }
  
  static double distance_to_line(geo::latlng const& p1, geo::latlng const& p2, geo::latlng const& point) {
    // Calculate distance from point to line segment p1-p2
    // Simplified geographic distance calculation
    
    double A = point.lat() - p1.lat();
    double B = point.lng() - p1.lng();
    double C = p2.lat() - p1.lat();
    double D = p2.lng() - p1.lng();
    
    double dot = A * C + B * D;
    double len_sq = C * C + D * D;
    
    if (len_sq < 1e-10) {
      // p1 and p2 are the same point
      return geo::distance(p1, point);
    }
    
    double param = dot / len_sq;
    
    geo::latlng closest;
    if (param < 0) {
      closest = p1;
    } else if (param > 1) {
      closest = p2;  
    } else {
      closest = geo::latlng{p1.lat() + param * C, p1.lng() + param * D};
    }
    
    return geo::distance(closest, point);
  }

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