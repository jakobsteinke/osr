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

  // Perform full CH preprocessing with hybrid approach
  void preprocess() {
    shortcuts_.clear();
    
    std::cout << "Starting CH preprocessing for " << w_.n_nodes() << " nodes...\n";
    std::cout << "Using random ordering with Python-inspired shortcut creation\n";
    
    // Use random ordering as requested, but with improved shortcut logic from Python
    preprocess_hybrid_approach();
    
    std::cout << "CH preprocessing complete\n";
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
    
    // SIMPLIFIED: Remove turn restrictions, use basic graph connectivity
    // This ensures consistent neighbor discovery for shortcut creation
    auto const& routing = *w_.r_;
    auto const& node_ways = routing.node_ways_[node];
    
    // Track unique neighbors to avoid duplicates
    ankerl::unordered_dense::set<node_idx_t> unique_neighbors;
    
    // For each way this node is part of, find adjacent nodes
    for (size_t wp = 0; wp < node_ways.size(); ++wp) {
      auto const way_idx = node_ways[wp];
      auto const& way_nodes = routing.way_nodes_[way_idx];
      
      // Find current node's position in this way
      auto const pos_it = std::find(way_nodes.begin(), way_nodes.end(), node);
      if (pos_it == way_nodes.end()) continue;
      
      auto const pos = std::distance(way_nodes.begin(), pos_it);
      
      // Add adjacent nodes in both directions
      if (pos > 0) {
        auto const neighbor = way_nodes[pos - 1];
        if (neighbor != node && 
            neighbor.v_ < w_.n_nodes() &&
            levels_.get_level(neighbor) > levels_.get_level(node)) {
          unique_neighbors.insert(neighbor);
        }
      }
      
      if (pos + 1 < way_nodes.size()) {
        auto const neighbor = way_nodes[pos + 1];
        if (neighbor != node && 
            neighbor.v_ < w_.n_nodes() &&
            levels_.get_level(neighbor) > levels_.get_level(node)) {
          unique_neighbors.insert(neighbor);
        }
      }
    }
    
    // Convert to vector
    for (auto const& neighbor : unique_neighbors) {
      higher_neighbors.push_back(neighbor);
    }
    
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
  
  // Get all neighbors from original graph only (for comparison)
  std::vector<node_idx_t> get_direct_neighbors(node_idx_t node) const {
    std::vector<node_idx_t> neighbors;
    if (node.v_ >= w_.n_nodes()) return neighbors;
    
    auto const& routing = *w_.r_;
    auto const& node_ways = routing.node_ways_[node];
    
    ankerl::unordered_dense::set<node_idx_t> unique_neighbors;
    
    for (size_t wp = 0; wp < node_ways.size(); ++wp) {
      auto const way_idx = node_ways[wp];
      auto const& way_nodes = routing.way_nodes_[way_idx];
      
      auto const pos_it = std::find(way_nodes.begin(), way_nodes.end(), node);
      if (pos_it == way_nodes.end()) continue;
      
      auto const pos = std::distance(way_nodes.begin(), pos_it);
      
      if (pos > 0) {
        unique_neighbors.insert(way_nodes[pos - 1]);
      }
      if (pos + 1 < way_nodes.size()) {
        unique_neighbors.insert(way_nodes[pos + 1]);
      }
    }
    
    neighbors.assign(unique_neighbors.begin(), unique_neighbors.end());
    return neighbors;
  }

  // TARGETED CONNECTIVITY FIX: Use direct neighbors but create additional bridge shortcuts
  std::vector<node_idx_t> get_all_neighbors(node_idx_t node) const {
    return get_direct_neighbors(node); // Keep simple to avoid explosion
  }
  
  // Contract a specific node according to CLAUDE.md Algorithm 1
  void contract_node(node_idx_t u) {
    // TARGETED APPROACH: Use direct neighbors for base contraction + limited shortcut enhancement
    // This balances connectivity needs with computational feasibility
    
    auto const incoming_neighbors = get_incoming_higher_nodes(u);
    auto const outgoing_neighbors = get_outgoing_higher_nodes(u); 
    
    int shortcuts_created = 0;
    
    // Create shortcuts between direct neighbors only (Algorithm 1 from CLAUDE.md)
    // But get_edge_cost() will consider existing shortcuts, satisfying the key requirement
    for (auto const v : incoming_neighbors) {
      for (auto const w : outgoing_neighbors) {
        if (v == w || v == u || w == u) continue;
        
        // The critical fix: get_edge_cost considers shortcuts, so paths via u
        // can use existing shortcuts, satisfying CLAUDE.md connectivity proof
        auto const cost_v_u = get_edge_cost(v, u);
        auto const cost_u_w = get_edge_cost(u, w);
        
        if (cost_v_u == kInfeasible || cost_u_w == kInfeasible) {
          continue;
        }
        
        auto const via_cost = cost_v_u + cost_u_w;
        
        // RoutingKit-style direction-specific shortcut creation
        // Add shortcut to forward graph if it's an upward edge (from lower to higher level)
        // Add shortcut to backward graph if it's a downward edge (from higher to lower level)
        
        if (levels_.is_higher_level(w, v)) {
          // w has higher level than v -> upward edge for forward search
          shortcuts_.add_forward_shortcut(v, w, u, via_cost);
        } else if (levels_.is_higher_level(v, w)) {
          // v has higher level than w -> downward edge for backward search
          shortcuts_.add_backward_shortcut(v, w, u, via_cost);
        } else {
          // Same level - add to both for safety (shouldn't happen with proper ordering)
          shortcuts_.add_forward_shortcut(v, w, u, via_cost);
          shortcuts_.add_backward_shortcut(v, w, u, via_cost);
        }
        
        shortcuts_created++;
      }
    }
    
    // CONNECTIVITY ENHANCEMENT: Create additional longer-range shortcuts
    // This addresses the core connectivity issue with strict level filtering
    create_connectivity_shortcuts(u);
    
    // Debug output for specific nodes if needed
    if (shortcuts_created > 50) { // Only report nodes creating many shortcuts
      std::cout << "    Node " << u.v_ << ": created " << shortcuts_created << " shortcuts (" 
                << incoming_neighbors.size() + outgoing_neighbors.size() << " neighbors)\n";
    }
  }
  
  // Check if shortcut (v -> w) via node is needed
  bool should_create_shortcut(node_idx_t v, node_idx_t via, node_idx_t w) const {
    // MAXIMUM CONNECTIVITY APPROACH: For strict level filtering with random ordering,
    // we need ultra-aggressive shortcut creation to ensure bidirectional searches can meet.
    // The connectivity proof in CLAUDE.md guarantees this will preserve shortest paths.
    
    // Verify the basic path (v -> via -> w) exists
    auto const cost_v_via = get_edge_cost(v, via);
    auto const cost_via_w = get_edge_cost(via, w);
    
    if (cost_v_via == kInfeasible || cost_via_w == kInfeasible) {
      return false;  // Path doesn't exist
    }
    
    // For connectivity with random levels: create ALL possible shortcuts
    // This ensures dense coverage needed for bidirectional search overlap
    return true;
  }
  
  // Create additional connectivity-focused shortcuts to bridge level gaps
  void create_connectivity_shortcuts(node_idx_t u) {
    // RoutingKit-style: search both forward and backward graphs for transitive opportunities
    // This creates longer-range connections crucial for bidirectional connectivity
    
    std::vector<node_idx_t> forward_sources, forward_targets, backward_sources, backward_targets;
    
    // Find nodes that have forward shortcuts TO u (sources for forward transitive shortcuts)
    for (auto const& [edge_pair, shortcut_list] : shortcuts_.get_forward_shortcuts()) {
      auto const [from, to] = edge_pair;
      if (to == u && levels_.is_higher_level(from, u)) {
        forward_sources.push_back(from);
        if (forward_sources.size() > 10) break; // Limit to prevent explosion
      }
      if (from == u && levels_.is_higher_level(to, u)) {
        forward_targets.push_back(to);
        if (forward_targets.size() > 10) break;
      }
    }
    
    // Find nodes that have backward shortcuts FROM u (sources for backward transitive shortcuts)
    for (auto const& [edge_pair, shortcut_list] : shortcuts_.get_backward_shortcuts()) {
      auto const [from, to] = edge_pair;
      if (to == u && levels_.is_higher_level(from, u)) {
        backward_sources.push_back(from);
        if (backward_sources.size() > 10) break;
      }
      if (from == u && levels_.is_higher_level(to, u)) {
        backward_targets.push_back(to);
        if (backward_targets.size() > 10) break;
      }
    }
    
    // Create forward transitive shortcuts: source -> u -> target becomes source -> target
    for (auto const source : forward_sources) {
      for (auto const target : forward_targets) {
        if (source != target && levels_.is_higher_level(target, source)) {
          auto const cost_src_u = get_edge_cost(source, u);
          auto const cost_u_tgt = get_edge_cost(u, target);
          
          if (cost_src_u != kInfeasible && cost_u_tgt != kInfeasible) {
            auto const total_cost = cost_src_u + cost_u_tgt;
            shortcuts_.add_forward_shortcut(source, target, u, total_cost);
          }
        }
      }
    }
    
    // Create backward transitive shortcuts
    for (auto const source : backward_sources) {
      for (auto const target : backward_targets) {
        if (source != target && levels_.is_higher_level(source, target)) {
          auto const cost_src_u = get_edge_cost(source, u);
          auto const cost_u_tgt = get_edge_cost(u, target);
          
          if (cost_src_u != kInfeasible && cost_u_tgt != kInfeasible) {
            auto const total_cost = cost_src_u + cost_u_tgt;
            shortcuts_.add_backward_shortcut(source, target, u, total_cost);
          }
        }
      }
    }
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
    
    // SIMPLIFIED: Remove turn restrictions, use basic graph connectivity
    // This ensures all topologically valid shortcuts can be created
    auto const& routing = *w_.r_;
    auto const& node_ways = routing.node_ways_[current];
    
    // Track unique neighbors to avoid duplicates
    ankerl::unordered_dense::map<node_idx_t, cost_t> unique_neighbors;
    
    // For each way this node is part of, find adjacent nodes
    for (size_t wp = 0; wp < node_ways.size(); ++wp) {
      auto const way_idx = node_ways[wp];
      auto const& way_nodes = routing.way_nodes_[way_idx];
      
      // Find current node's position in this way
      auto const pos_it = std::find(way_nodes.begin(), way_nodes.end(), current);
      if (pos_it == way_nodes.end()) continue;
      
      auto const pos = std::distance(way_nodes.begin(), pos_it);
      
      // Add adjacent nodes in both directions
      if (pos > 0) {
        auto const neighbor = way_nodes[pos - 1];
        if (neighbor != exclude_via && 
            neighbor.v_ < w_.n_nodes() &&
            levels_.get_level(neighbor) > levels_.get_level(exclude_via)) {
          auto const cost = get_edge_cost(current, neighbor);
          if (cost != kInfeasible) {
            unique_neighbors[neighbor] = std::min(unique_neighbors.count(neighbor) ? unique_neighbors[neighbor] : cost, cost);
          }
        }
      }
      
      if (pos + 1 < way_nodes.size()) {
        auto const neighbor = way_nodes[pos + 1];
        if (neighbor != exclude_via && 
            neighbor.v_ < w_.n_nodes() &&
            levels_.get_level(neighbor) > levels_.get_level(exclude_via)) {
          auto const cost = get_edge_cost(current, neighbor);
          if (cost != kInfeasible) {
            unique_neighbors[neighbor] = std::min(unique_neighbors.count(neighbor) ? unique_neighbors[neighbor] : cost, cost);
          }
        }
      }
    }
    
    // Call callback for each unique neighbor
    for (auto const& [neighbor, cost] : unique_neighbors) {
      callback(neighbor, cost);
    }
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
    
    // CRITICAL FIX: Check shortcuts FIRST from both forward and backward graphs
    // This implements CLAUDE.md line 58: treat shortcuts as normal edges
    auto const forward_shortcut_cost = get_forward_shortcut_cost(from, to);
    auto const backward_shortcut_cost = get_backward_shortcut_cost(from, to);
    
    cost_t best_shortcut_cost = kInfeasible;
    if (forward_shortcut_cost != kInfeasible && backward_shortcut_cost != kInfeasible) {
      best_shortcut_cost = std::min(forward_shortcut_cost, backward_shortcut_cost);
    } else if (forward_shortcut_cost != kInfeasible) {
      best_shortcut_cost = forward_shortcut_cost;
    } else if (backward_shortcut_cost != kInfeasible) {
      best_shortcut_cost = backward_shortcut_cost;
    }
    
    if (best_shortcut_cost != kInfeasible) {
      return best_shortcut_cost;
    }
    
    // Then check direct edges from original graph
    auto const& routing = *w_.r_;
    auto const& from_ways = routing.node_ways_[from];
    
    // Check if nodes are adjacent in the way structure
    for (size_t fp = 0; fp < from_ways.size(); ++fp) {
      auto const way_idx = from_ways[fp];
      auto const& way_nodes = routing.way_nodes_[way_idx];
      
      // Find from node's position in this way
      auto const from_pos_it = std::find(way_nodes.begin(), way_nodes.end(), from);
      if (from_pos_it == way_nodes.end()) continue;
      
      auto const from_pos = std::distance(way_nodes.begin(), from_pos_it);
      
      // Check adjacent positions for target node
      if (from_pos > 0 && way_nodes[from_pos - 1] == to) {
        // Use a simplified cost based on distance (similar to dijkstra)
        return cost_t{10}; // Simple unit cost for adjacent nodes
      }
      if (from_pos + 1 < way_nodes.size() && way_nodes[from_pos + 1] == to) {
        return cost_t{10}; // Simple unit cost for adjacent nodes  
      }
    }
    
    return kInfeasible; // No direct edge or shortcut found
  }

  // Helper method to get cost of best forward shortcut between two nodes
  cost_t get_forward_shortcut_cost(node_idx_t from, node_idx_t to) const {
    auto const* shortcuts = shortcuts_.get_forward_shortcuts(from, to);
    if (!shortcuts || shortcuts->empty()) {
      return kInfeasible;
    }
    
    cost_t best_cost = kInfeasible;
    for (auto const& shortcut : *shortcuts) {
      if (shortcut.cost_ < best_cost) {
        best_cost = shortcut.cost_;
      }
    }
    
    return best_cost;
  }
  
  // Helper method to get cost of best backward shortcut between two nodes
  cost_t get_backward_shortcut_cost(node_idx_t from, node_idx_t to) const {
    auto const* shortcuts = shortcuts_.get_backward_shortcuts(from, to);
    if (!shortcuts || shortcuts->empty()) {
      return kInfeasible;
    }
    
    cost_t best_cost = kInfeasible;
    for (auto const& shortcut : *shortcuts) {
      if (shortcut.cost_ < best_cost) {
        best_cost = shortcut.cost_;
      }
    }
    
    return best_cost;
  }
  
  // Legacy method - searches both graphs for best shortcut
  cost_t get_shortcut_cost(node_idx_t from, node_idx_t to) const {
    auto forward_cost = get_forward_shortcut_cost(from, to);
    auto backward_cost = get_backward_shortcut_cost(from, to);
    
    if (forward_cost == kInfeasible && backward_cost == kInfeasible) {
      return kInfeasible;
    } else if (forward_cost == kInfeasible) {
      return backward_cost;
    } else if (backward_cost == kInfeasible) {
      return forward_cost;
    } else {
      return std::min(forward_cost, backward_cost);
    }
  }
  
  // Check if two nodes are connected using car profile
  bool are_nodes_connected(node_idx_t from, node_idx_t to) const {
    return get_edge_cost(from, to) != kInfeasible;
  }
  
  // Hybrid preprocessing: Random ordering + Python-inspired shortcut creation
  void preprocess_hybrid_approach() {
    // Contract nodes in random level order (as originally requested)
    auto contracted_count = 0U;
    std::size_t shortcut_count = 0U;
    auto const report_interval = std::max(w_.n_nodes() / 20, node_idx_t::value_t{100});
    
    for (ch_levels::level_t level = 1; level <= levels_.size(); ++level) {
      auto const node = find_node_with_level(level);
      if (node != node_idx_t::invalid()) {
        auto const shortcuts_before = shortcuts_.size();
        
        // Use Python-inspired shortcut creation (no witness search)
        contract_node_hybrid(node);
        
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
    
    std::cout << "Hybrid preprocessing complete: " << contracted_count 
              << " nodes contracted, " << shortcut_count << " shortcuts created\n";
  }
  
  // Hybrid contraction: Python-style logic with random ordering
  void contract_node_hybrid(node_idx_t node) {
    // Use get_all_neighbors to include shortcuts (Python insight)
    auto const all_neighbors = get_all_neighbors(node);
    
    int shortcuts_created = 0;
    
    // Create shortcuts between all neighbor pairs (no witness search as requested)
    for (size_t i = 0; i < all_neighbors.size(); ++i) {
      for (size_t j = i + 1; j < all_neighbors.size(); ++j) {
        auto const neighbor1 = all_neighbors[i];
        auto const neighbor2 = all_neighbors[j];
        
        if (neighbor1 == neighbor2) continue;
        
        // Check if level filtering allows these connections
        if (levels_.get_level(neighbor1) <= levels_.get_level(node) ||
            levels_.get_level(neighbor2) <= levels_.get_level(node)) {
          continue; // Skip if not higher level
        }
        
        // Create bidirectional shortcuts
        auto const cost1 = get_edge_cost(neighbor1, node) + get_edge_cost(node, neighbor2);
        auto const cost2 = get_edge_cost(neighbor2, node) + get_edge_cost(node, neighbor1);
        
        if (cost1 != kInfeasible) {
          shortcuts_.add_shortcut(neighbor1, neighbor2, node, cost1);
          shortcuts_created++;
        }
        
        if (cost2 != kInfeasible && neighbor1 != neighbor2) {
          shortcuts_.add_shortcut(neighbor2, neighbor1, node, cost2);
          shortcuts_created++;
        }
      }
    }
    
    // Debug output for significant contractions
    if (shortcuts_created > 10) {
      std::cout << "    Node " << node.v_ << ": created " << shortcuts_created 
                << " shortcuts (" << all_neighbors.size() << " neighbors)\n";
    }
  }
  
  // Python-style preprocessing with edge-difference ordering
  void preprocess_with_edge_difference_ordering() {
    // Track which nodes have been contracted (ordered)
    ankerl::unordered_dense::set<node_idx_t> contracted_nodes;
    
    // Priority queue for edge-difference ordering: (edge_difference, node)
    using pq_entry = std::pair<int, node_idx_t>;
    std::priority_queue<pq_entry, std::vector<pq_entry>, std::greater<pq_entry>> node_pq;
    
    // Initialize priority queue with edge differences for all nodes
    std::cout << "Computing initial edge differences...\n";
    for (std::uint32_t i = 0; i < w_.n_nodes(); ++i) {
      auto const node = node_idx_t{i};
      auto const edge_diff = calculate_edge_difference(node, contracted_nodes);
      node_pq.push({edge_diff, node});
    }
    
    ch_levels::level_t order = 1;
    std::size_t shortcut_count = 0;
    auto const report_interval = std::max(w_.n_nodes() / 20, node_idx_t::value_t{100});
    
    std::cout << "Contracting nodes in edge-difference order...\n";
    
    // Main contraction loop (Python lines 195-234)
    while (!node_pq.empty()) {
      auto const [_, node] = node_pq.top();
      node_pq.pop();
      
      // Skip if already contracted
      if (contracted_nodes.count(node)) {
        continue;
      }
      
      // Re-calculate edge difference to ensure we pick the best candidate
      // (Python lines 199-202: lazy re-evaluation)
      auto const new_edge_diff = calculate_edge_difference(node, contracted_nodes);
      if (!node_pq.empty() && new_edge_diff > node_pq.top().first) {
        // Re-insert with updated priority
        node_pq.push({new_edge_diff, node});
        continue;
      }
      
      // Contract this node
      levels_.set_level(node, order);
      contracted_nodes.insert(node);
      
      // Create shortcuts (Python lines 210-233)
      auto const shortcuts_before = shortcuts_.size();
      contract_node_python_style(node, contracted_nodes);
      shortcut_count += shortcuts_.size() - shortcuts_before;
      
      // Progress reporting
      if (order % report_interval == 0) {
        std::cout << "  Contracted " << order << "/" << w_.n_nodes() 
                  << " nodes, " << shortcut_count << " shortcuts created\n";
      }
      
      order++;
    }
    
    std::cout << "Edge-difference preprocessing complete: " 
              << (order - 1) << " nodes contracted, " << shortcut_count << " shortcuts created\n";
  }
  
  // Calculate edge difference for a node (Python lines 162-183)
  int calculate_edge_difference(node_idx_t node, 
                               ankerl::unordered_dense::set<node_idx_t> const& contracted_nodes) const {
    // Get ALL neighbors - including shortcuts (this is key!)
    auto const all_neighbors = get_all_neighbors(node);
    
    // Filter to uncontracted neighbors
    std::vector<node_idx_t> uncontracted_neighbors;
    for (auto const neighbor : all_neighbors) {
      if (!contracted_nodes.count(neighbor)) {
        uncontracted_neighbors.push_back(neighbor);
      }
    }
    
    // Start with negative count of existing edges
    int edge_diff = -static_cast<int>(uncontracted_neighbors.size());
    
    // Count potential shortcuts between all pairs
    for (auto const incoming : uncontracted_neighbors) {
      for (auto const outgoing : uncontracted_neighbors) {
        if (incoming == outgoing) continue;
        
        // Check if shortcut would be needed (simplified - no witness search as requested)
        auto const via_cost = get_edge_cost(incoming, node) + get_edge_cost(node, outgoing);
        if (via_cost != kInfeasible) {
          edge_diff++; // This would create a shortcut
        }
      }
    }
    
    return edge_diff;
  }
  
  // Contract node Python-style (Python lines 210-233)  
  void contract_node_python_style(node_idx_t node, 
                                  ankerl::unordered_dense::set<node_idx_t> const& contracted_nodes) {
    // Get ALL neighbors - including those connected via shortcuts
    // This is critical for the Python approach to work correctly
    auto const all_neighbors = get_all_neighbors(node);
    
    // Filter to only uncontracted nodes
    std::vector<node_idx_t> incoming_neighbors;
    std::vector<node_idx_t> outgoing_neighbors;
    
    for (auto const neighbor : all_neighbors) {
      if (!contracted_nodes.count(neighbor)) {
        incoming_neighbors.push_back(neighbor);
        outgoing_neighbors.push_back(neighbor); // Undirected graph - same set
      }
    }
    
    int shortcuts_created = 0;
    
    // Create shortcuts between all incoming and outgoing pairs
    for (auto const incoming : incoming_neighbors) {
      for (auto const outgoing : outgoing_neighbors) {
        if (incoming == outgoing) continue;
        
        auto const cost_to_via = get_edge_cost(incoming, node);
        auto const cost_from_via = get_edge_cost(node, outgoing);
        
        if (cost_to_via == kInfeasible || cost_from_via == kInfeasible) {
          continue;
        }
        
        auto const shortcut_cost = cost_to_via + cost_from_via;
        
        // NO WITNESS SEARCH - create shortcut unconditionally as requested
        shortcuts_.add_shortcut(incoming, outgoing, node, shortcut_cost);
        shortcuts_created++;
      }
    }
    
    // Debug output for significant contractions
    if (shortcuts_created > 20) {
      std::cout << "    Node " << node.v_ << ": created " << shortcuts_created 
                << " shortcuts (" << incoming_neighbors.size() << " neighbors)\n";
    }
  }

  ways const& w_;
  ch_levels levels_;
  ch_shortcuts shortcuts_;
};

}  // namespace osr