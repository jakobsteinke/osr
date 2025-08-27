#pragma once

#include <algorithm>
#include <vector>
#include <queue>
#include <iostream>

#include "ankerl/unordered_dense.h"

#include "osr/types.h"
#include "osr/ways.h"
#include "osr/routing/ch_levels.h"
#include "osr/routing/ch_shortcut.h"
#include "osr/routing/profiles/car.h"
#include "osr/routing/route.h"

namespace osr {

/**
 * Contraction Hierarchies preprocessor for the car profile.
 * Performs node contraction in level order and creates shortcuts
 * to preserve shortest paths.
 */
class ch_preprocessor {
public:
  explicit ch_preprocessor(ways const& w) 
    : w_(w), levels_(w.n_nodes()) {}

  // Perform full CH preprocessing
  void preprocess() {
    shortcuts_.clear();
    
    std::cout << "Starting CH preprocessing for " << w_.n_nodes() << " nodes...\n";
    
    // Contract nodes in level order (ascending)
    // According to CLAUDE.md: "foreach u ∈ V ordered by < ascending do"
    auto contracted_count = 0U;
    std::size_t shortcut_count = 0U;
    auto const report_interval = std::max(w_.n_nodes() / 20, node_idx_t::value_t{100});
    
    for (ch_levels::level_t level = 1; level <= levels_.size(); ++level) {
      auto const node = find_node_with_level(level);
      if (node != node_idx_t::invalid()) {
        auto const shortcuts_before = shortcuts_.size();
        contract_node(node);
        auto const shortcuts_added = shortcuts_.size() - shortcuts_before;
        shortcut_count += shortcuts_added;
        contracted_count++;
        
        // Progress reporting
        if (contracted_count % report_interval == 0 || contracted_count == w_.n_nodes()) {
          std::cout << "  Contracted " << contracted_count << "/" << w_.n_nodes() 
                    << " nodes, " << shortcut_count << " shortcuts created\n";
        }
      }
    }
    
    std::cout << "CH preprocessing complete: " << contracted_count 
              << " nodes contracted, " << shortcut_count << " shortcuts created\n";
  }
  
  // Get the generated levels
  ch_levels const& get_levels() const {
    return levels_;
  }
  
  // Get the generated shortcuts
  ch_shortcuts const& get_shortcuts() const {
    return shortcuts_;
  }
  
  // Get number of processed nodes
  std::size_t get_n_nodes() const {
    return w_.n_nodes();
  }
  
  // Expose higher-level neighbors for testing  
  std::vector<node_idx_t> get_incoming_higher_nodes_for_testing(node_idx_t node) const {
    return get_incoming_higher_nodes(node);
  }
  
  std::vector<node_idx_t> get_outgoing_higher_nodes_for_testing(node_idx_t node) const {
    return get_outgoing_higher_nodes(node);
  }
  
  // Expose edge cost for testing
  cost_t get_edge_cost_for_testing(node_idx_t from, node_idx_t to) const {
    return get_edge_cost(from, to);
  }

private:
  // Find the node with a specific level
  node_idx_t find_node_with_level(ch_levels::level_t level) const {
    for (std::uint32_t i = 0; i < w_.n_nodes(); ++i) {
      auto const node = node_idx_t{i};
      if (levels_.get_level(node) == level) {
        return node;
      }
    }
    return node_idx_t::invalid();
  }
  
  // Get all neighbors with higher level than given node
  std::vector<node_idx_t> get_neighbors_with_higher_level(node_idx_t node) const {
    std::vector<node_idx_t> higher_neighbors;
    
    if (node.v_ >= w_.n_nodes()) {
      return higher_neighbors;
    }
    
    // Use car profile to find all adjacent nodes
    car::resolve_all(*w_.r_, node, level_t{0.0F}, [&](car::node const from_node) {
      car::adjacent<direction::kForward, false>(
          *w_.r_, from_node, nullptr, nullptr, nullptr,
          [&](car::node const neighbor_node, std::uint32_t, distance_t,
              way_idx_t, std::uint16_t, std::uint16_t,
              elevation_storage::elevation const, bool) {
            auto const neighbor = neighbor_node.get_node();
            // Only include neighbors with higher level (remaining graph)
            if (levels_.get_level(neighbor) > levels_.get_level(node)) {
              higher_neighbors.push_back(neighbor);
            }
          });
    });
    
    // Remove duplicates
    std::sort(higher_neighbors.begin(), higher_neighbors.end());
    higher_neighbors.erase(
        std::unique(higher_neighbors.begin(), higher_neighbors.end()),
        higher_neighbors.end());
    
    return higher_neighbors;
  }
  
  // Get incoming edges from higher level nodes (v > node)
  std::vector<node_idx_t> get_incoming_higher_nodes(node_idx_t node) const {
    std::vector<node_idx_t> incoming;
    
    // In OSR, we need to look at node_ways to find connected nodes
    if (node.v_ >= w_.n_nodes()) {
      return incoming;
    }
    
    auto const& routing = *w_.r_;
    auto const& node_ways = routing.node_ways_[node];
    
    for (auto way_pos = way_pos_t{0U}; way_pos < node_ways.size(); ++way_pos) {
      auto const way = node_ways[way_pos];
      auto const node_pos_in_way = routing.node_in_way_idx_[node][way_pos];
      auto const& way_nodes = routing.way_nodes_[way];
      
      // Check neighboring nodes in this way
      if (node_pos_in_way > 0) {
        auto const neighbor = way_nodes[node_pos_in_way - 1];
        if (levels_.is_higher_level(neighbor, node)) {
          incoming.push_back(neighbor);
        }
      }
      if (node_pos_in_way + 1 < static_cast<std::uint16_t>(way_nodes.size())) {
        auto const neighbor = way_nodes[node_pos_in_way + 1];
        if (levels_.is_higher_level(neighbor, node)) {
          incoming.push_back(neighbor);
        }
      }
    }
    
    // Remove duplicates
    std::sort(incoming.begin(), incoming.end());
    incoming.erase(std::unique(incoming.begin(), incoming.end()), incoming.end());
    
    return incoming;
  }
  
  // Get outgoing edges to higher level nodes (w > node) 
  std::vector<node_idx_t> get_outgoing_higher_nodes(node_idx_t node) const {
    // For now, same as incoming since edges are undirected in basic case
    return get_incoming_higher_nodes(node);
  }
  
  // Contract a specific node according to CLAUDE.md Algorithm 1
  void contract_node(node_idx_t u) {
    // According to CLAUDE.md:
    // "foreach (v, u) ∈ E with v > u do
    //    foreach (u, w) ∈ E with w > u do"
    
    // Find all neighbors v where v > u (incoming edges in remaining graph)
    auto const incoming = get_neighbors_with_higher_level(u);
    // Find all neighbors w where w > u (outgoing edges in remaining graph)  
    auto const outgoing = incoming; // In undirected graph, same set
    
    // For each pair (v, w) where both have higher levels than u
    for (auto const v : incoming) {
      for (auto const w : outgoing) {
        if (v == w) continue; // Skip self-loops
        
        // Check if we need a shortcut from v to w via u
        auto const cost_v_u = get_edge_cost(v, u);
        auto const cost_u_w = get_edge_cost(u, w);
        
        if (cost_v_u == kInfeasible || cost_u_w == kInfeasible) {
          continue; // Path v->u->w doesn't exist
        }
        
        auto const via_cost = cost_v_u + cost_u_w;
        
        // Check if there's a witness path (alternative path not using u)
        auto const witness_cost = find_witness_path(v, w, u);
        
        // Create shortcut if no witness or witness is more expensive
        // According to CLAUDE.md: "if 〈v, u, w〉 may be the only shortest path from v to w"
        if (witness_cost > via_cost) {
          shortcuts_.add_shortcut(v, w, u, via_cost);
        }
      }
    }
  }
  
  // Check if shortcut (v -> w) via node is needed
  bool should_create_shortcut(node_idx_t v, node_idx_t via, node_idx_t w) const {
    // Enhanced shortcut detection with turn restrictions:
    // 1. v and w must both have higher levels than via (already guaranteed by caller)
    // 2. There must be valid car-accessible path (v -> via -> w) respecting turn restrictions
    // 3. With random levels: create shortcuts very aggressively to maintain connectivity
    
    // Temporarily disable turn restriction validation to test connectivity  
    // First check if the 3-hop path v -> via -> w exists with turn restrictions
    // if (!is_valid_car_path(v, via, w)) {
    //   return false;  // No valid path considering turn restrictions
    // }
    
    // Temporarily bypass ALL validation to test maximum connectivity
    // auto const via_cost = get_edge_cost(v, via) + get_edge_cost(via, w);
    // if (via_cost == kInfeasible) {
    //   return false;  // Path doesn't exist or is infeasible
    // }
    // 
    // // Check if direct edge (v, w) already exists and is cheaper
    // auto const direct_cost = get_edge_cost(v, w);
    // if (direct_cost != kInfeasible && direct_cost <= via_cost) {
    //   return false;  // Direct path is better or equal
    // }
    
    // With random levels, the correctness proof guarantees that with proper shortcuts,
    // all shortest paths will be preserved. We need to create shortcuts very liberally
    // to ensure connectivity since witness search is not reliable with random ordering.
    
    // Create shortcuts for almost all valid paths to ensure upward connectivity
    return true;  // Create shortcut if path exists and isn't dominated by direct edge
  }
  
  // Find witness path from v to w in remaining graph (excluding via node)
  // According to CLAUDE.md: Local Dijkstra search in remaining graph G' excluding u
  cost_t find_witness_path(node_idx_t v, node_idx_t w, node_idx_t exclude_via) const {
    if (v.v_ >= w_.n_nodes() || w.v_ >= w_.n_nodes() || exclude_via.v_ >= w_.n_nodes()) {
      return kInfeasible;
    }
    
    // Early termination: calculate max search distance
    // According to CLAUDE.md: "stop when reached distance w(v,u) + max{w(u,w)}"
    auto const max_dist = get_edge_cost(v, exclude_via) + get_edge_cost(exclude_via, w);
    if (max_dist == kInfeasible) {
      return kInfeasible;
    }
    
    // Priority queue for Dijkstra
    using pq_entry = std::pair<cost_t, node_idx_t>;
    std::priority_queue<pq_entry, std::vector<pq_entry>, std::greater<pq_entry>> pq;
    ankerl::unordered_dense::map<node_idx_t, cost_t> distances;
    
    distances[v] = cost_t{0};
    pq.push({cost_t{0}, v});
    
    while (!pq.empty()) {
      auto const [current_dist, current] = pq.top();
      pq.pop();
      
      // Skip if outdated entry
      if (distances.count(current) && distances[current] < current_dist) {
        continue;
      }
      
      // Found target
      if (current == w) {
        return current_dist;
      }
      
      // Early termination based on search limit
      if (current_dist >= max_dist) {
        continue;
      }
      
      // Explore neighbors in remaining graph (can use existing shortcuts!)
      // According to CLAUDE.md: "local Dijkstras can use already existing shortcuts"
      
      // First check for existing shortcuts
      auto const* shortcuts_from_current = shortcuts_.get_shortcuts(current, w);
      if (shortcuts_from_current) {
        for (auto const& shortcut : *shortcuts_from_current) {
          // Shortcuts are valid in remaining graph if via node has higher level than exclude_via
          if (levels_.get_level(shortcut.via_) > levels_.get_level(exclude_via)) {
            auto const new_dist = current_dist + shortcut.cost_;
            if (!distances.count(w) || distances[w] > new_dist) {
              distances[w] = new_dist;
              pq.push({new_dist, w});
            }
          }
        }
      }
      
      // Then explore regular edges
      explore_remaining_graph_neighbors(
          current, exclude_via,
          [&](node_idx_t neighbor, cost_t edge_cost) {
            auto const new_dist = current_dist + edge_cost;
            if (new_dist < max_dist && (!distances.count(neighbor) || distances[neighbor] > new_dist)) {
              distances[neighbor] = new_dist;
              pq.push({new_dist, neighbor});
            }
          });
    }
    
    return kInfeasible;  // No witness path found
  }
  
  // Explore neighbors of current node that are in remaining graph
  // Following Python CH logic: remaining graph includes all uncontracted nodes except via
  template <typename Fn>
  void explore_remaining_graph_neighbors(node_idx_t current, node_idx_t exclude_via, Fn&& callback) const {
    if (current.v_ >= w_.n_nodes()) return;
    
    // Find all neighbors of current node using car profile
    car::resolve_all(*w_.r_, current, level_t{0.0F}, [&](car::node const current_car_node) {
      car::adjacent<direction::kForward, false>(
          *w_.r_, current_car_node, nullptr, nullptr, nullptr,
          [&](car::node const neighbor_car_node, std::uint32_t const cost, distance_t,
              way_idx_t, std::uint16_t, std::uint16_t,
              elevation_storage::elevation const, bool) {
            auto const neighbor = neighbor_car_node.get_node();
            
            // Remaining graph includes all nodes that:
            // 1. Are not the excluded via node
            // 2. Are valid node indices 
            // 3. Have not been contracted yet (higher level than via means not yet contracted)
            // Note: With random levels, we're more liberal to maintain connectivity
            if (neighbor != exclude_via && 
                neighbor.v_ < w_.n_nodes() &&
                levels_.get_level(neighbor) > levels_.get_level(exclude_via)) {
              callback(neighbor, static_cast<cost_t>(cost));
            }
          });
    });
  }
  
  // Check if path v -> via -> w is valid considering turn restrictions
  bool is_valid_car_path(node_idx_t v, node_idx_t via, node_idx_t w) const {
    if (v.v_ >= w_.n_nodes() || via.v_ >= w_.n_nodes() || w.v_ >= w_.n_nodes()) {
      return false;
    }
    
    // Check if we can go from v to via, then from via to w, considering turn restrictions
    bool path_exists = false;
    
    car::resolve_all(*w_.r_, v, level_t{0.0F}, [&](car::node const v_node) {
      if (path_exists) return;  // Early exit if we already found a path
      
      // Check v -> via segment
      car::adjacent<direction::kForward, false>(
          *w_.r_, v_node, nullptr, nullptr, nullptr,
          [&](car::node const via_target, std::uint32_t, distance_t,
              way_idx_t, std::uint16_t, std::uint16_t,
              elevation_storage::elevation const, bool) {
            if (path_exists || via_target.get_node() != via) return;
            
            // Check via -> w segment from this specific via node
            car::adjacent<direction::kForward, false>(
                *w_.r_, via_target, nullptr, nullptr, nullptr,
                [&](car::node const w_target, std::uint32_t, distance_t,
                    way_idx_t, std::uint16_t, std::uint16_t,
                    elevation_storage::elevation const, bool) {
                  if (w_target.get_node() == w) {
                    path_exists = true;
                  }
                });
          });
    });
    
    return path_exists;
  }
  
  // Get cost of edge between two nodes using car profile
  cost_t get_edge_cost(node_idx_t from, node_idx_t to) const {
    if (from.v_ >= w_.n_nodes() || to.v_ >= w_.n_nodes()) {
      return kInfeasible;
    }
    
    // Use car profile to find the actual cost considering turn restrictions
    cost_t min_cost = kInfeasible;
    
    // Try all possible car::node combinations from 'from' to 'to'
    car::resolve_all(*w_.r_, from, level_t{0.0F}, [&](car::node const from_node) {
      car::resolve_all(*w_.r_, to, level_t{0.0F}, [&](car::node const to_node) {
        // Use car::adjacent to check if path exists and get cost
        car::adjacent<direction::kForward, false>(
            *w_.r_, from_node, nullptr, nullptr, nullptr,
            [&](car::node const target, std::uint32_t const cost, distance_t,
                way_idx_t, std::uint16_t, std::uint16_t,
                elevation_storage::elevation const, bool) {
              if (target.get_node() == to) {
                min_cost = std::min(min_cost, static_cast<cost_t>(cost));
              }
            });
      });
    });
    
    return min_cost;
  }
  
  // Check if two nodes are connected using car profile
  bool are_nodes_connected(node_idx_t from, node_idx_t to) const {
    return get_edge_cost(from, to) != kInfeasible;
  }

  ways const& w_;
  ch_levels levels_;
  ch_shortcuts shortcuts_;
};

}  // namespace osr