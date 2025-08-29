#pragma once

#include <vector>
#include <algorithm>

#include "osr/routing/ch_graph.h"
#include "osr/routing/dial.h"
#include "osr/types.h"

namespace osr {

struct ch_query_result {
  cost_t distance = kInfeasible;
  std::vector<std::uint32_t> path;
};

struct ch_query_label {
  node_idx_t node;
  cost_t cost;
  
  explicit ch_query_label(node_idx_t n = node_idx_t::invalid(), cost_t c = kInfeasible) 
    : node(n), cost(c) {}
};

struct ch_get_bucket {
  cost_t operator()(const ch_query_label& l) const { return l.cost; }
};

class ch_bidirectional {
public:
  explicit ch_bidirectional(const ch_graph& graph)
    : graph_(graph),
      node_count_(graph.node_count()),
      forward_distance_(node_count_, kInfeasible),
      backward_distance_(node_count_, kInfeasible),
      forward_predecessor_(node_count_, node_idx_t::invalid()),
      backward_predecessor_(node_count_, node_idx_t::invalid()),
      forward_queue_(ch_get_bucket{}),
      backward_queue_(ch_get_bucket{}) {}

  ch_query_result query(std::uint32_t source, std::uint32_t target, cost_t max_cost = kInfeasible) {
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
    
    forward_queue_.push(ch_query_label{node_idx_t{source}, 0});
    backward_queue_.push(ch_query_label{node_idx_t{target}, 0});
    
    cost_t shortest_path_length = kInfeasible;
    std::uint32_t meeting_node = std::numeric_limits<std::uint32_t>::max();
    
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
    
    if (meeting_node == std::numeric_limits<std::uint32_t>::max()) {
      return {};  // No path found
    }
    
    return {shortest_path_length, reconstruct_path(source, target, meeting_node)};
  }

private:
  void reset(cost_t max_cost = kInfeasible) {
    std::fill(forward_distance_.begin(), forward_distance_.end(), kInfeasible);
    std::fill(backward_distance_.begin(), backward_distance_.end(), kInfeasible);
    std::fill(forward_predecessor_.begin(), forward_predecessor_.end(), node_idx_t::invalid());
    std::fill(backward_predecessor_.begin(), backward_predecessor_.end(), node_idx_t::invalid());
    
    // Reset dial queues with appropriate bucket count
    forward_queue_.clear();
    backward_queue_.clear();
    if (max_cost != kInfeasible && max_cost > 0) {
      forward_queue_.n_buckets(max_cost + 1U);
      backward_queue_.n_buckets(max_cost + 1U);
    } else {
      // Default reasonable bucket count for unknown max cost
      forward_queue_.n_buckets(10000U);
      backward_queue_.n_buckets(10000U);  
    }
  }

  void settle_forward_node(cost_t& shortest_path_length, std::uint32_t& meeting_node) {
    if (forward_queue_.empty()) return;
    
    auto current = forward_queue_.pop();
    auto const node = current.node.v_;
    auto const distance = current.cost;
    
    if (distance > forward_distance_[node]) return;  // Outdated entry
    
    // Check for meeting with backward search
    if (backward_distance_[node] != kInfeasible) {
      auto const total_distance = distance + backward_distance_[node];
      if (total_distance < shortest_path_length) {
        shortest_path_length = total_distance;
        meeting_node = node;
      }
    }
    
    // Expand upward edges only (level filtering)
    for (auto const& arc : graph_.out_arcs(node)) {
      auto const neighbor = arc.target.v_;
      
      // Level filtering: only go to higher levels
      if (graph_.level(neighbor) <= graph_.level(node)) continue;
      
      auto const new_distance = distance + arc.weight;
      if (new_distance < forward_distance_[neighbor] && new_distance < kInfeasible) {
        forward_distance_[neighbor] = new_distance;
        forward_predecessor_[neighbor] = node_idx_t{node};
        forward_queue_.push(ch_query_label{node_idx_t{neighbor}, static_cast<cost_t>(new_distance)});
      }
    }
  }

  void settle_backward_node(cost_t& shortest_path_length, std::uint32_t& meeting_node) {
    if (backward_queue_.empty()) return;
    
    auto current = backward_queue_.pop();
    auto const node = current.node.v_;
    auto const distance = current.cost;
    
    if (distance > backward_distance_[node]) return;  // Outdated entry
    
    // Check for meeting with forward search
    if (forward_distance_[node] != kInfeasible) {
      auto const total_distance = forward_distance_[node] + distance;
      if (total_distance < shortest_path_length) {
        shortest_path_length = total_distance;
        meeting_node = node;
      }
    }
    
    // Expand upward edges only (level filtering) on reversed graph
    for (auto const& arc : graph_.in_arcs(node)) {
      auto const neighbor = arc.target.v_;
      
      // Level filtering: only go to higher levels (on reversed graph)
      if (graph_.level(neighbor) <= graph_.level(node)) continue;
      
      auto const new_distance = distance + arc.weight;
      if (new_distance < backward_distance_[neighbor] && new_distance < kInfeasible) {
        backward_distance_[neighbor] = new_distance;
        backward_predecessor_[neighbor] = node_idx_t{node};
        backward_queue_.push(ch_query_label{node_idx_t{neighbor}, static_cast<cost_t>(new_distance)});
      }
    }
  }

  std::vector<std::uint32_t> reconstruct_path(std::uint32_t source, std::uint32_t target, std::uint32_t meeting_node) {
    std::vector<std::uint32_t> path;
    
    // Build path from source to meeting node
    std::vector<std::uint32_t> forward_path;
    auto current = meeting_node;
    while (current != source) {
      forward_path.push_back(current);
      current = forward_predecessor_[current].v_;
      if (current == node_idx_t::invalid().v_) break;
    }
    forward_path.push_back(source);
    std::reverse(forward_path.begin(), forward_path.end());
    
    // Build path from meeting node to target
    std::vector<std::uint32_t> backward_path;
    current = meeting_node;
    while (current != target) {
      current = backward_predecessor_[current].v_;
      if (current == node_idx_t::invalid().v_) break;
      backward_path.push_back(current);
    }
    
    // Combine paths
    path = forward_path;
    path.insert(path.end(), backward_path.begin(), backward_path.end());
    
    return path;
  }

  const ch_graph& graph_;
  std::uint32_t node_count_;
  
  std::vector<cost_t> forward_distance_;
  std::vector<cost_t> backward_distance_;
  std::vector<node_idx_t> forward_predecessor_;
  std::vector<node_idx_t> backward_predecessor_;
  
  dial<ch_query_label, ch_get_bucket> forward_queue_;
  dial<ch_query_label, ch_get_bucket> backward_queue_;
};

}  // namespace osr