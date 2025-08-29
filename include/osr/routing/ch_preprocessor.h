#pragma once

#include <vector>
#include <queue>
#include <random>
#include <algorithm>
#include <functional>
#include <numeric>

#include "osr/routing/ch_graph.h"

namespace osr {

struct ch_pq_entry {
  std::uint32_t node;
  cost_t distance;
  
  bool operator>(const ch_pq_entry& other) const {
    return distance > other.distance;
  }
};

class ch_preprocessor {
public:
  explicit ch_preprocessor(ch_graph& graph) 
    : graph_(graph), 
      node_count_(graph.node_count()),
      distance_(node_count_, kInfeasible) {}

  void preprocess() {
    contract_nodes();
  }

private:
  void contract_nodes() {
    // Contract nodes in level order (ascending)
    std::vector<std::uint32_t> nodes_by_level(node_count_);
    std::iota(nodes_by_level.begin(), nodes_by_level.end(), 0U);
    
    std::sort(nodes_by_level.begin(), nodes_by_level.end(),
              [this](std::uint32_t a, std::uint32_t b) {
                return graph_.level(a) < graph_.level(b);
              });

    for (auto node : nodes_by_level) {
      contract_node(node);
    }
  }

  void contract_node(std::uint32_t node) {
    auto const& in_arcs = graph_.in_arcs(node);
    auto const& out_arcs = graph_.out_arcs(node);
    
    // For each incoming edge (v, u) and outgoing edge (u, w)
    for (auto const& in_arc : in_arcs) {
      auto const v = in_arc.target.v_;
      auto const v_u_weight = in_arc.weight;
      
      // Only consider higher-level neighbors as required by CH
      if (graph_.level(v) <= graph_.level(node)) continue;
      
      for (auto const& out_arc : out_arcs) {
        auto const w = out_arc.target.v_;
        auto const u_w_weight = out_arc.weight;
        
        // Only consider higher-level neighbors as required by CH  
        if (graph_.level(w) <= graph_.level(node)) continue;
        if (v == w) continue;  // No self-loops
        
        auto const path_weight = v_u_weight + u_w_weight;
        if (path_weight >= kInfeasible) continue;  // Overflow check
        
        // Check if we need to add a shortcut by running witness search
        if (needs_shortcut(v, w, path_weight, node)) {
          graph_.add_shortcut(v, w, path_weight, node);
        }
      }
    }
  }

  bool needs_shortcut(std::uint32_t source, std::uint32_t target, cost_t max_distance, std::uint32_t forbidden_node) {
    // Run local Dijkstra search to find witness path
    // Only search in remaining graph (nodes with level > forbidden_node level)
    
    std::fill(distance_.begin(), distance_.end(), kInfeasible);
    std::priority_queue<ch_pq_entry, std::vector<ch_pq_entry>, std::greater<ch_pq_entry>> pq;
    
    distance_[source] = 0;
    pq.push({source, 0});
    
    while (!pq.empty()) {
      auto current = pq.top();
      pq.pop();
      
      auto const node = current.node;
      auto const dist = current.distance;
      
      if (dist > max_distance) break;  // Early termination
      if (dist > distance_[node]) continue;  // Outdated entry
      
      if (node == target) {
        return dist > max_distance;  // Need shortcut if no better witness path found
      }
      
      // Expand to higher-level neighbors only
      for (auto const& arc : graph_.out_arcs(node)) {
        auto const neighbor = arc.target.v_;
        
        // Skip if this is the forbidden node or a lower-level node
        if (neighbor == forbidden_node || graph_.level(neighbor) <= graph_.level(forbidden_node)) {
          continue;
        }
        
        auto const new_dist = dist + arc.weight;
        if (new_dist < distance_[neighbor] && new_dist < kInfeasible) {
          distance_[neighbor] = new_dist;
          pq.push({neighbor, static_cast<cost_t>(new_dist)});
        }
      }
    }
    
    // No path found or path longer than max_distance -> need shortcut
    return true;
  }

  ch_graph& graph_;
  std::uint32_t node_count_;
  std::vector<cost_t> distance_;  // Reused for witness searches
};

}  // namespace osr