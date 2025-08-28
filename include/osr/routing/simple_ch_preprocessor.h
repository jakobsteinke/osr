#pragma once

#include <vector>
#include <queue>
#include <random>
#include <algorithm>
#include <functional>
#include <numeric>

#include "osr/routing/simple_ch_graph.h"

namespace osr {

// Simple priority queue for Dijkstra witness searches
struct pq_entry {
  unsigned node;
  unsigned distance;
  
  bool operator>(const pq_entry& other) const {
    return distance > other.distance;
  }
};

class simple_ch_preprocessor {
public:
  explicit simple_ch_preprocessor(simple_ch_graph& graph) 
    : graph_(graph), 
      node_count_(graph.node_count()),
      distance_(node_count_, kInvalidWeight) {}

  void preprocess() {
    assign_random_levels();
    contract_nodes();
  }

private:
  void assign_random_levels() {
    std::vector<unsigned> order(node_count_);
    std::iota(order.begin(), order.end(), 0);
    
    // Random shuffle to create random node ordering
    std::random_device rd;
    std::mt19937 gen(rd());
    std::shuffle(order.begin(), order.end(), gen);
    
    // Assign levels based on random order
    for (unsigned i = 0; i < node_count_; ++i) {
      graph_.set_level(order[i], i + 1);
    }
  }

  void contract_nodes() {
    // Contract nodes in level order (ascending)
    std::vector<unsigned> nodes_by_level(node_count_);
    std::iota(nodes_by_level.begin(), nodes_by_level.end(), 0);
    
    std::sort(nodes_by_level.begin(), nodes_by_level.end(),
              [this](unsigned a, unsigned b) {
                return graph_.level(a) < graph_.level(b);
              });

    for (unsigned node : nodes_by_level) {
      contract_node(node);
    }
  }

  void contract_node(unsigned node) {
    const auto& in_arcs = graph_.in_arcs(node);
    const auto& out_arcs = graph_.out_arcs(node);
    
    // For each incoming edge (v, u) and outgoing edge (u, w)
    for (const auto& in_arc : in_arcs) {
      unsigned v = in_arc.node;
      unsigned v_u_weight = in_arc.weight;
      
      // Only consider higher-level neighbors as required by CH
      if (graph_.level(v) <= graph_.level(node)) continue;
      
      for (const auto& out_arc : out_arcs) {
        unsigned w = out_arc.node;
        unsigned u_w_weight = out_arc.weight;
        
        // Only consider higher-level neighbors as required by CH  
        if (graph_.level(w) <= graph_.level(node)) continue;
        if (v == w) continue;  // No self-loops
        
        unsigned path_weight = v_u_weight + u_w_weight;
        if (path_weight >= kInvalidWeight) continue;  // Overflow check
        
        // Check if we need to add a shortcut by running witness search
        if (needs_shortcut(v, w, path_weight, node)) {
          graph_.add_shortcut(v, w, path_weight, node);
        }
      }
    }
    
    // DO NOT remove edges - level filtering in query handles this
    // graph_.remove_node_edges(node);
  }

  bool needs_shortcut(unsigned source, unsigned target, unsigned max_distance, unsigned forbidden_node) {
    // Run local Dijkstra search to find witness path
    // Only search in remaining graph (nodes with level > forbidden_node level)
    
    std::fill(distance_.begin(), distance_.end(), kInvalidWeight);
    std::priority_queue<pq_entry, std::vector<pq_entry>, std::greater<pq_entry>> pq;
    
    distance_[source] = 0;
    pq.push({source, 0});
    
    while (!pq.empty()) {
      auto current = pq.top();
      pq.pop();
      
      unsigned node = current.node;
      unsigned dist = current.distance;
      
      if (dist > max_distance) break;  // Early termination
      if (dist > distance_[node]) continue;  // Outdated entry
      
      if (node == target) {
        return dist > max_distance;  // Need shortcut if no better witness path found
      }
      
      // Expand to higher-level neighbors only
      for (const auto& arc : graph_.out_arcs(node)) {
        unsigned neighbor = arc.node;
        
        // Skip if this is the forbidden node or a lower-level node
        if (neighbor == forbidden_node || graph_.level(neighbor) <= graph_.level(forbidden_node)) {
          continue;
        }
        
        unsigned new_dist = dist + arc.weight;
        if (new_dist < distance_[neighbor] && new_dist < kInvalidWeight) {
          distance_[neighbor] = new_dist;
          pq.push({neighbor, new_dist});
        }
      }
    }
    
    // No path found or path longer than max_distance -> need shortcut
    return true;
  }

  simple_ch_graph& graph_;
  unsigned node_count_;
  std::vector<unsigned> distance_;  // Reused for witness searches
};

}  // namespace osr