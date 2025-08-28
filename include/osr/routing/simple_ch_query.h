#pragma once

#include <vector>
#include <algorithm>

#include "osr/routing/simple_ch_graph.h"
#include "osr/routing/dial.h"

namespace osr {

struct query_result {
  unsigned distance = kInvalidWeight;
  std::vector<unsigned> path;
};

// Label for dial priority queue - just stores node and cost
struct ch_label {
  unsigned node;
  unsigned cost;
  
  explicit ch_label(unsigned n = kInvalidNode, unsigned c = kInvalidWeight) 
    : node(n), cost(c) {}
};

// Bucket selector for dial queue - extracts cost from label
struct get_bucket {
  unsigned operator()(const ch_label& l) const { return l.cost; }
};

class simple_ch_query {
public:
  explicit simple_ch_query(const simple_ch_graph& graph)
    : graph_(graph),
      node_count_(graph.node_count()),
      forward_distance_(node_count_, kInvalidWeight),
      backward_distance_(node_count_, kInvalidWeight),
      forward_predecessor_(node_count_, kInvalidNode),
      backward_predecessor_(node_count_, kInvalidNode),
      forward_queue_(get_bucket{}),
      backward_queue_(get_bucket{}) {}

  query_result query(unsigned source, unsigned target, unsigned max_cost = kInvalidWeight) {
    reset(max_cost);
    
    if (source >= node_count_ || target >= node_count_) {
      return {};
    }
    
    if (source == target) {
      return {0, {source}};
    }
    
    // Initialize search with dial queues
    forward_distance_[source] = 0;
    backward_distance_[target] = 0;
    
    forward_queue_.push(ch_label{source, 0});
    backward_queue_.push(ch_label{target, 0});
    
    unsigned shortest_path_length = kInvalidWeight;
    unsigned meeting_node = kInvalidNode;
    
    bool forward_turn = true;
    
    // Main bidirectional search loop using dial queues
    while (!forward_queue_.empty() || !backward_queue_.empty()) {
      bool forward_finished = forward_queue_.empty() || 
                             (!forward_queue_.empty() && forward_queue_.buckets_[forward_queue_.get_next_bucket()].back().cost >= shortest_path_length);
      bool backward_finished = backward_queue_.empty() || 
                              (!backward_queue_.empty() && backward_queue_.buckets_[backward_queue_.get_next_bucket()].back().cost >= shortest_path_length);
      
      if (forward_finished && backward_finished) {
        break;
      }
      
      if (forward_finished) forward_turn = false;
      if (backward_finished) forward_turn = true;
      
      if (forward_turn) {
        settle_forward_node(shortest_path_length, meeting_node);
        forward_turn = false;
      } else {
        settle_backward_node(shortest_path_length, meeting_node);
        forward_turn = true;
      }
    }
    
    if (meeting_node == kInvalidNode) {
      return {};  // No path found
    }
    
    return {shortest_path_length, reconstruct_path(source, target, meeting_node)};
  }

private:
  void reset(unsigned max_cost = kInvalidWeight) {
    std::fill(forward_distance_.begin(), forward_distance_.end(), kInvalidWeight);
    std::fill(backward_distance_.begin(), backward_distance_.end(), kInvalidWeight);
    std::fill(forward_predecessor_.begin(), forward_predecessor_.end(), kInvalidNode);
    std::fill(backward_predecessor_.begin(), backward_predecessor_.end(), kInvalidNode);
    
    // Reset dial queues with appropriate bucket count
    forward_queue_.clear();
    backward_queue_.clear();
    if (max_cost != kInvalidWeight && max_cost > 0) {
      forward_queue_.n_buckets(max_cost + 1U);
      backward_queue_.n_buckets(max_cost + 1U);
    } else {
      // Default reasonable bucket count for unknown max cost
      forward_queue_.n_buckets(10000U);
      backward_queue_.n_buckets(10000U);  
    }
  }

  void settle_forward_node(unsigned& shortest_path_length, unsigned& meeting_node) {
    if (forward_queue_.empty()) return;
    
    auto current = forward_queue_.pop();
    unsigned node = current.node;
    unsigned distance = current.cost;
    
    if (distance > forward_distance_[node]) return;  // Outdated entry
    
    // Check for meeting with backward search
    if (backward_distance_[node] != kInvalidWeight) {
      unsigned total_distance = distance + backward_distance_[node];
      if (total_distance < shortest_path_length) {
        shortest_path_length = total_distance;
        meeting_node = node;
      }
    }
    
    // Expand upward edges only (level filtering)
    for (const auto& arc : graph_.out_arcs(node)) {
      unsigned neighbor = arc.node;
      
      // Level filtering: only go to higher levels
      if (graph_.level(neighbor) <= graph_.level(node)) continue;
      
      unsigned new_distance = distance + arc.weight;
      if (new_distance < forward_distance_[neighbor] && new_distance < kInvalidWeight) {
        forward_distance_[neighbor] = new_distance;
        forward_predecessor_[neighbor] = node;
        forward_queue_.push(ch_label{neighbor, new_distance});
      }
    }
  }

  void settle_backward_node(unsigned& shortest_path_length, unsigned& meeting_node) {
    if (backward_queue_.empty()) return;
    
    auto current = backward_queue_.pop();
    unsigned node = current.node;
    unsigned distance = current.cost;
    
    if (distance > backward_distance_[node]) return;  // Outdated entry
    
    // Check for meeting with forward search
    if (forward_distance_[node] != kInvalidWeight) {
      unsigned total_distance = forward_distance_[node] + distance;
      if (total_distance < shortest_path_length) {
        shortest_path_length = total_distance;
        meeting_node = node;
      }
    }
    
    // Expand upward edges only (level filtering) on reversed graph
    for (const auto& arc : graph_.in_arcs(node)) {
      unsigned neighbor = arc.node;
      
      // Level filtering: only go to higher levels (on reversed graph)
      if (graph_.level(neighbor) <= graph_.level(node)) continue;
      
      unsigned new_distance = distance + arc.weight;
      if (new_distance < backward_distance_[neighbor] && new_distance < kInvalidWeight) {
        backward_distance_[neighbor] = new_distance;
        backward_predecessor_[neighbor] = node;
        backward_queue_.push(ch_label{neighbor, new_distance});
      }
    }
  }

  std::vector<unsigned> reconstruct_path(unsigned source, unsigned target, unsigned meeting_node) {
    std::vector<unsigned> path;
    
    // Build path from source to meeting node
    std::vector<unsigned> forward_path;
    unsigned current = meeting_node;
    while (current != source) {
      forward_path.push_back(current);
      current = forward_predecessor_[current];
      if (current == kInvalidNode) break;
    }
    forward_path.push_back(source);
    std::reverse(forward_path.begin(), forward_path.end());
    
    // Build path from meeting node to target
    std::vector<unsigned> backward_path;
    current = meeting_node;
    while (current != target) {
      current = backward_predecessor_[current];
      if (current == kInvalidNode) break;
      backward_path.push_back(current);
    }
    
    // Combine paths
    path = forward_path;
    path.insert(path.end(), backward_path.begin(), backward_path.end());
    
    return path;
  }

  const simple_ch_graph& graph_;
  unsigned node_count_;
  
  std::vector<unsigned> forward_distance_;
  std::vector<unsigned> backward_distance_;
  std::vector<unsigned> forward_predecessor_;
  std::vector<unsigned> backward_predecessor_;
  
  dial<ch_label, get_bucket> forward_queue_;
  dial<ch_label, get_bucket> backward_queue_;
};

}  // namespace osr