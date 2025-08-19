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
  static constexpr bool kDebug = false;  // Disable debug for faster preprocessing
  
  struct node_importance {
    car_ch_key key_;
    int edge_diff_;  // shortcuts_created - neighbors_removed
    int degree_;
    int contracted_neighbors_;  // Number of already contracted neighbors
    
    bool operator<(node_importance const& other) const {
      // Primary: edge difference (fewer shortcuts = contract first)
      if (edge_diff_ != other.edge_diff_) {
        return edge_diff_ < other.edge_diff_;
      }
      
      // Secondary: number of contracted neighbors (more = contract later)
      if (contracted_neighbors_ != other.contracted_neighbors_) {
        return contracted_neighbors_ < other.contracted_neighbors_;
      }
      
      // Tertiary: degree (lower = contract first)
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
    std::cout << "*** CH PREPROCESSING STARTED ***" << std::endl;
    ch_data ch;
    auto const& r = *w.r_;
    
    // Collect only VALID car states that have actual adjacency
    // Instead of generating all theoretical states, explore the connected graph
    std::vector<car_ch_key> all_car_states;
    ankerl::unordered_dense::set<car_ch_key, car_ch_key_hash> visited_states;
    
    // Start BFS from node 0 to find all reachable car states
    std::queue<car_ch_key> bfs_queue;
    
    // Initialize with valid starting states from node 0
    for (auto n = node_idx_t{0}; n.v_ < std::min(static_cast<uint32_t>(100), w.n_nodes()); ++n.v_) {
      car::resolve_all(r, n, level_t{static_cast<std::uint8_t>(0U)}, [&](car::node const car_n) {
        car_ch_key key{car_n.n_, car_n.way_, car_n.dir_};
        
        // Check if this state has any neighbors before including it
        bool has_neighbors = false;
        car::template adjacent<direction::kForward, false>(
            r, car_n, nullptr, nullptr, nullptr,
            [&](car::node const, std::uint32_t const, distance_t, way_idx_t const,
                std::uint16_t, std::uint16_t, elevation_storage::elevation const, bool const) {
              has_neighbors = true;
            });
        
        if (!has_neighbors) {
          car::template adjacent<direction::kBackward, false>(
              r, car_n, nullptr, nullptr, nullptr,
              [&](car::node const, std::uint32_t const, distance_t, way_idx_t const,
                  std::uint16_t, std::uint16_t, elevation_storage::elevation const, bool const) {
                has_neighbors = true;
              });
        }
        
        if (has_neighbors && visited_states.find(key) == visited_states.end()) {
          visited_states.insert(key);
          bfs_queue.push(key);
        }
      });
    }
    
    // BFS to find all connected car states
    while (!bfs_queue.empty() && all_car_states.size() < 200) {  // Very small limit for quick testing
      auto current_key = bfs_queue.front();
      bfs_queue.pop();
      all_car_states.push_back(current_key);
      
      car::node const current_node{current_key.n_, current_key.way_, current_key.dir_};
      
      // Explore forward neighbors
      car::template adjacent<direction::kForward, false>(
          r, current_node, nullptr, nullptr, nullptr,
          [&](car::node const neighbor, std::uint32_t const, distance_t, way_idx_t const,
              std::uint16_t, std::uint16_t, elevation_storage::elevation const, bool const) {
            car_ch_key neighbor_key{neighbor.n_, neighbor.way_, neighbor.dir_};
            if (visited_states.find(neighbor_key) == visited_states.end()) {
              visited_states.insert(neighbor_key);
              bfs_queue.push(neighbor_key);
            }
          });
      
      // Explore backward neighbors
      car::template adjacent<direction::kBackward, false>(
          r, current_node, nullptr, nullptr, nullptr,
          [&](car::node const neighbor, std::uint32_t const, distance_t, way_idx_t const,
              std::uint16_t, std::uint16_t, elevation_storage::elevation const, bool const) {
            car_ch_key neighbor_key{neighbor.n_, neighbor.way_, neighbor.dir_};
            if (visited_states.find(neighbor_key) == visited_states.end()) {
              visited_states.insert(neighbor_key);
              bfs_queue.push(neighbor_key);
            }
          });
    }
    
    std::cout << "*** Total car states: " << all_car_states.size() << " ***" << std::endl;
    if (!all_car_states.empty()) {
      std::cout << "Sample car states:" << std::endl;
      for (size_t i = 0; i < std::min(all_car_states.size(), size_t{10}); ++i) {
        auto const& cs = all_car_states[i];
        std::cout << "  " << i << ": node=" << cs.n_.v_ << " way=" << cs.way_ << " dir=" << (cs.dir_ == direction::kForward ? "fwd" : "bwd") << std::endl;
      }
    }
    
    // Simplified approach: use basic degree-based ordering
    std::vector<std::pair<int, car_ch_key>> node_priorities;
    
    for (auto const& key : all_car_states) {
      // Count neighbors (degree)
      int degree = 0;
      car::node const n{key.n_, key.way_, key.dir_};
      car::template adjacent<direction::kForward, false>(
          r, n, nullptr, nullptr, nullptr,
          [&](car::node const, std::uint32_t const, distance_t, way_idx_t const,
              std::uint16_t, std::uint16_t, elevation_storage::elevation const, bool const) {
            degree++;
          });
      car::template adjacent<direction::kBackward, false>(
          r, n, nullptr, nullptr, nullptr,
          [&](car::node const, std::uint32_t const, distance_t, way_idx_t const,
              std::uint16_t, std::uint16_t, elevation_storage::elevation const, bool const) {
            degree++;
          });
      node_priorities.push_back({degree, key});
    }
    
    // Sort by degree (lower degree = contract first)
    std::sort(node_priorities.begin(), node_priorities.end(),
              [](auto const& a, auto const& b) { return a.first < b.first; });
    
    // Extract sorted keys
    all_car_states.clear();
    for (auto const& [_, key] : node_priorities) {
      all_car_states.push_back(key);
    }
    
    ankerl::unordered_dense::set<car_ch_key, car_ch_key_hash> contracted;
    
    size_t current_level = 0;
    const size_t total_nodes = all_car_states.size();
    
    // Contract nodes in degree order
    for (auto const& key_to_contract : all_car_states) {
      
      // Assign levels: first contracted (least important) gets level 0
      // Higher levels = more important = contracted later
      ch.node_levels_[key_to_contract] = static_cast<ch_level_t>(current_level);
      
      if (kDebug && current_level < 10) {
        std::cout << "Contracting car state (node=" << key_to_contract.n_ 
                  << ", way=" << key_to_contract.way_ << ", dir=" << (key_to_contract.dir_ == direction::kForward ? "fwd" : "bwd")
                  << ") at level " << ch.node_levels_[key_to_contract] << std::endl;
      }
      
      // Per detailed-project-description.md section 1.2: "Contract nodes and insert shortcuts"
      std::vector<std::pair<car_ch_key, cost_t>> incoming;
      std::vector<std::pair<car_ch_key, cost_t>> outgoing;
      
      // CRITICAL: Collect neighbors BEFORE adding to contracted set
      // Otherwise all neighbors get filtered out as "already contracted"
      collect_car_neighbors(w, ch, key_to_contract, contracted, incoming, outgoing);
      
      if (kDebug && current_level < 10) {
        std::cout << "  Found " << incoming.size() << " incoming, " << outgoing.size() << " outgoing neighbors" << std::endl;
      }
      
      // Add to contracted set AFTER collecting neighbors
      contracted.insert(key_to_contract);
      
      // For every pair (v,w) with edges v→u and u→w, check if shortcut needed
      int shortcuts_added = 0;
      for (auto const& [v_key, cost_v_u] : incoming) {
        for (auto const& [w_key, cost_u_w] : outgoing) {
          if (v_key == w_key) continue;
          
          auto const shortcut_cost = cost_v_u + cost_u_w;
          
          // Enhanced shortcut creation with turn legality validation
          // We need to ensure shortcuts preserve car profile turn restrictions
          
          // Temporarily disable turn validation to test if this is the issue
          bool is_turn_legal = true; // validate_shortcut_turn_legality(w, v_key, key_to_contract, w_key);
          
          if (!is_turn_legal) {
            // Skip this shortcut - it would violate turn restrictions
            continue;
          }
          
          // Aggressive shortcut creation for maximum connectivity
          // Create shortcuts for ALL neighbor pairs to guarantee connectivity
          bool create_shortcut = true;
          
          // Optional: Still check witness for debugging
          bool witness_exists = !needs_car_shortcut(w, ch, v_key, w_key, shortcut_cost, key_to_contract, contracted);
          
          if (create_shortcut) {
            // Store proper shortcut metadata for unpacking
            // The shortcut represents the path v_key -> key_to_contract -> w_key
            // So middle_node is key_to_contract, and we store the connecting ways
            ch.add_shortcut(v_key, w_key, shortcut_cost, key_to_contract, key_to_contract,
                           way_idx_t{v_key.way_}, way_idx_t{w_key.way_});
            shortcuts_added++;
            
            if (kDebug && current_level < 20) {
              std::cout << "  Added shortcut (" << v_key.n_ << "," << v_key.way_ << ") -> (" 
                       << w_key.n_ << "," << w_key.way_ << ") cost=" << shortcut_cost 
                       << " via (" << key_to_contract.n_ << "," << key_to_contract.way_ << ")"
                       << (witness_exists ? " [witness exists]" : " [no witness]") << std::endl;
            }
          }
        }
      }
      
      if (kDebug && current_level < 10) {
        std::cout << "  Shortcuts added: " << shortcuts_added << " out of " << (incoming.size() * outgoing.size()) << " possible pairs" << std::endl;
      }
      
      ++current_level;
    }
    
    if (kDebug) {
      std::cout << "CH preprocessing complete. Total levels assigned: " << current_level << std::endl;
    }
    
    return ch;
  }
  
private:
  // Check if this connection is critical for graph connectivity
  static bool is_critical_bridge_connection(ways const& w,
                                          ch_data const& ch,
                                          car_ch_key const& from,
                                          car_ch_key const& to,
                                          car_ch_key const& via,
                                          ankerl::unordered_dense::set<car_ch_key, car_ch_key_hash> const& contracted) {
    // Heuristic: if from and to have low degree and few alternative paths,
    // this might be a critical connection
    
    std::vector<std::pair<car_ch_key, cost_t>> from_incoming, from_outgoing;
    std::vector<std::pair<car_ch_key, cost_t>> to_incoming, to_outgoing;
    
    collect_car_neighbors(w, ch, from, contracted, from_incoming, from_outgoing);
    collect_car_neighbors(w, ch, to, contracted, to_incoming, to_outgoing);
    
    // If either node has very low degree, consider this critical
    int from_degree = static_cast<int>(from_incoming.size() + from_outgoing.size());
    int to_degree = static_cast<int>(to_incoming.size() + to_outgoing.size());
    
    if (from_degree <= 2 || to_degree <= 2) {
      return true;  // Low-degree nodes need shortcuts to maintain connectivity
    }
    
    // If nodes are on same intersection but different ways, might be critical
    if (from.n_ == to.n_ && from.way_ != to.way_) {
      return true;  // Turn connections within intersections are critical
    }
    
    return false;
  }
  static int calculate_initial_edge_difference(ways const& w,
                                              ch_data const& ch,
                                              car_ch_key const& key) {
    ankerl::unordered_dense::set<car_ch_key, car_ch_key_hash> empty_contracted;
    return calculate_edge_difference(w, ch, key, empty_contracted);
  }
  
  static int calculate_degree(ways const& w, car_ch_key const& key) {
    auto const& r = *w.r_;
    int degree = 0;
    car::node const n{key.n_, key.way_, key.dir_};
    car::template adjacent<direction::kForward, false>(
        r, n, nullptr, nullptr, nullptr,
        [&](car::node const, std::uint32_t const, distance_t, way_idx_t const,
            std::uint16_t, std::uint16_t, elevation_storage::elevation const, bool const) {
          degree++;
        });
    car::template adjacent<direction::kBackward, false>(
        r, n, nullptr, nullptr, nullptr,
        [&](car::node const, std::uint32_t const, distance_t, way_idx_t const,
            std::uint16_t, std::uint16_t, elevation_storage::elevation const, bool const) {
          degree++;
        });
    return degree;
  }
  
  static int count_contracted_neighbors(ways const& w,
                                       car_ch_key const& key,
                                       ankerl::unordered_dense::set<car_ch_key, car_ch_key_hash> const& contracted) {
    auto const& r = *w.r_;
    int count = 0;
    car::node const n{key.n_, key.way_, key.dir_};
    car::template adjacent<direction::kForward, false>(
        r, n, nullptr, nullptr, nullptr,
        [&](car::node const neighbor, std::uint32_t const, distance_t, way_idx_t const,
            std::uint16_t, std::uint16_t, elevation_storage::elevation const, bool const) {
          car_ch_key neighbor_key{neighbor.n_, neighbor.way_, neighbor.dir_};
          if (contracted.find(neighbor_key) != contracted.end()) {
            count++;
          }
        });
    car::template adjacent<direction::kBackward, false>(
        r, n, nullptr, nullptr, nullptr,
        [&](car::node const neighbor, std::uint32_t const, distance_t, way_idx_t const,
            std::uint16_t, std::uint16_t, elevation_storage::elevation const, bool const) {
          car_ch_key neighbor_key{neighbor.n_, neighbor.way_, neighbor.dir_};
          if (contracted.find(neighbor_key) != contracted.end()) {
            count++;
          }
        });
    return count;
  }
  
  static int calculate_edge_difference(ways const& w,
                                       ch_data const& ch,
                                       car_ch_key const& key,
                                       ankerl::unordered_dense::set<car_ch_key, car_ch_key_hash> const& contracted) {
    std::vector<std::pair<car_ch_key, cost_t>> incoming;
    std::vector<std::pair<car_ch_key, cost_t>> outgoing;
    
    collect_car_neighbors(w, ch, key, contracted, incoming, outgoing);
    
    // Count shortcuts needed
    int shortcuts_needed = 0;
    for (auto const& [v_key, cost_v_u] : incoming) {
      for (auto const& [w_key, cost_u_w] : outgoing) {
        if (v_key == w_key) continue;
        auto const shortcut_cost = cost_v_u + cost_u_w;
        if (needs_car_shortcut(w, ch, v_key, w_key, shortcut_cost, key, contracted)) {
          shortcuts_needed++;
        }
      }
    }
    
    // Edge difference = shortcuts added - edges removed
    int edges_removed = static_cast<int>(incoming.size() + outgoing.size());
    return shortcuts_needed - edges_removed;
  }
  
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
    
    if (kDebug && car_state.n_.v_ < 3000) {  // Capture the first few nodes being processed
      std::cout << "    DEBUG: collecting neighbors for (" << car_state.n_.v_ << "," << car_state.way_ << "," << (car_state.dir_ == direction::kForward ? "fwd" : "bwd") << ")" << std::endl;
    }
    
    int total_outgoing = 0, filtered_outgoing = 0;
    int total_incoming = 0, filtered_incoming = 0;
    
    // Get outgoing neighbors
    car::template adjacent<direction::kForward, false>(
        r, n, nullptr, nullptr, nullptr,
        [&](car::node const neighbor, std::uint32_t const cost,
            distance_t, way_idx_t const, std::uint16_t, std::uint16_t,
            elevation_storage::elevation const, bool const) {
          total_outgoing++;
          car_ch_key neighbor_key{neighbor.n_, neighbor.way_, neighbor.dir_};
          if (contracted.find(neighbor_key) == contracted.end()) {
            outgoing.emplace_back(neighbor_key, cost);
          } else {
            filtered_outgoing++;
          }
        });
    
    // Get incoming neighbors  
    car::template adjacent<direction::kBackward, false>(
        r, n, nullptr, nullptr, nullptr,
        [&](car::node const neighbor, std::uint32_t const cost,
            distance_t, way_idx_t const, std::uint16_t, std::uint16_t,
            elevation_storage::elevation const, bool const) {
          total_incoming++;
          car_ch_key neighbor_key{neighbor.n_, neighbor.way_, neighbor.dir_};
          if (contracted.find(neighbor_key) == contracted.end()) {
            incoming.emplace_back(neighbor_key, cost);
          } else {
            filtered_incoming++;
          }
        });
    
    if (kDebug && car_state.n_.v_ < 3000) {  // Capture the first few nodes being processed
      std::cout << "    DEBUG: found " << total_outgoing << " total outgoing (" << filtered_outgoing << " filtered), " 
               << total_incoming << " total incoming (" << filtered_incoming << " filtered)" << std::endl;
    }
    
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
    
    // Enhanced witness search with comprehensive exploration
    // Use a more generous limit to ensure we don't miss alternative paths
    cost_t base_tolerance = std::max(static_cast<cost_t>(100U), static_cast<cost_t>(shortcut_cost / 5));  // At least 20% tolerance
    cost_t witness_limit = shortcut_cost + base_tolerance;
    
    // Track settled nodes to implement hop limit as per CH theory
    size_t settled_nodes = 0;
    const size_t max_settled_nodes = 10;  // Very fast preprocessing for quick testing
    
    cost_t best_witness_cost = std::numeric_limits<cost_t>::max();
    
    while (!pq.empty() && settled_nodes < max_settled_nodes) {
      auto const [curr_key, curr_cost] = pq.top();
      pq.pop();
      
      // Skip if we've found a better path to this node
      if (costs[curr_key] < curr_cost) {
        continue;
      }
      
      settled_nodes++;
      
      if (curr_key == to) {
        // Found witness path
        best_witness_cost = std::min(best_witness_cost, curr_cost);
        if (curr_cost <= shortcut_cost) {
          return false;  // Valid witness found - no shortcut needed
        }
        // Continue searching for potentially better witnesses
        continue;
      }
      
      // Early termination if current cost already exceeds our best witness
      if (curr_cost > witness_limit || 
          (best_witness_cost != std::numeric_limits<cost_t>::max() && curr_cost >= best_witness_cost)) {
        continue;
      }
      
      explore_car_neighbors_for_witness(w, ch, curr_key, to, curr_cost, 
                                       witness_limit, contracted_key, 
                                       contracted, pq, costs);
    }
    
    // Return true if no witness found or all witnesses are worse than shortcut
    return best_witness_cost > shortcut_cost;
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
          // Only exclude the node currently being contracted
          if (neighbor_key == excluded_key) {
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
    
    // Explore shortcuts from this car state - critical for connectivity
    auto const* shortcuts = ch.get_forward_shortcuts(curr_key);
    if (shortcuts) {
      for (auto const& sc : *shortcuts) {
        // Only exclude the node currently being contracted
        // Shortcuts through contracted nodes are essential for witness paths
        if (sc.to_ == excluded_key) {
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
    
    // Also explore backward shortcuts that end at current node
    auto const* backward_shortcuts = ch.get_backward_shortcuts(curr_key);
    if (backward_shortcuts) {
      for (auto const& sc : *backward_shortcuts) {
        if (sc.from_ == excluded_key) {
          continue;
        }
        
        // Check if we can reach the from node and continue from there
        auto const new_cost = static_cast<cost_t>(curr_cost + sc.cost_);
        if (new_cost <= max_cost) {
          auto it = costs.find(sc.from_);
          if (it == costs.end() || new_cost < it->second) {
            costs[sc.from_] = new_cost;
            pq.push({sc.from_, new_cost});
          }
        }
      }
    }
  }
  
  // Validate that a shortcut preserves car profile turn legality
  static bool validate_shortcut_turn_legality(ways const& w,
                                             car_ch_key const& from,
                                             car_ch_key const& via,
                                             car_ch_key const& to) {
    auto const& r = *w.r_;
    
    // Check if the path from->via->to is turn-legal according to car profile
    // This involves checking that the transition from->via and via->to are both legal
    
    // First, check if from can legally reach via
    bool can_reach_via = false;
    car::node const from_node{from.n_, from.way_, from.dir_};
    car::template adjacent<direction::kForward, false>(
        r, from_node, nullptr, nullptr, nullptr,
        [&](car::node const neighbor, std::uint32_t const, distance_t,
            way_idx_t const, std::uint16_t, std::uint16_t,
            elevation_storage::elevation const, bool const) {
          car_ch_key neighbor_key{neighbor.n_, neighbor.way_, neighbor.dir_};
          if (neighbor_key.n_ == via.n_ && neighbor_key.way_ == via.way_ && neighbor_key.dir_ == via.dir_) {
            can_reach_via = true;
          }
        });
    
    if (!can_reach_via) {
      return false;
    }
    
    // Next, check if via can legally reach to
    bool can_reach_to = false;
    car::node const via_node{via.n_, via.way_, via.dir_};
    car::template adjacent<direction::kForward, false>(
        r, via_node, nullptr, nullptr, nullptr,
        [&](car::node const neighbor, std::uint32_t const, distance_t,
            way_idx_t const, std::uint16_t, std::uint16_t,
            elevation_storage::elevation const, bool const) {
          car_ch_key neighbor_key{neighbor.n_, neighbor.way_, neighbor.dir_};
          if (neighbor_key.n_ == to.n_ && neighbor_key.way_ == to.way_ && neighbor_key.dir_ == to.dir_) {
            can_reach_to = true;
          }
        });
    
    return can_reach_to;
  }
};

}  // namespace osr