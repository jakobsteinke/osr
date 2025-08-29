#pragma once

#include <vector>
#include <unordered_map>
#include <random>
#include <algorithm>
#include <numeric>

#include "osr/types.h"
#include "osr/ways.h"
#include "geo/latlng.h"

namespace osr {

struct ch_arc {
  node_idx_t target;
  cost_t weight;
  bool is_shortcut{false};
  node_idx_t shortcut_middle{node_idx_t::invalid()};

  ch_arc(node_idx_t t, cost_t w) : target(t), weight(w) {}
  ch_arc(node_idx_t t, cost_t w, node_idx_t mid) 
    : target(t), weight(w), is_shortcut(true), shortcut_middle(mid) {}
};

class ch_graph {
public:
  explicit ch_graph(ways const& w) {
    build_from_osr(w);
  }

  void build_from_osr(ways const& w) {
    auto const n_nodes = static_cast<std::uint32_t>(w.n_nodes());
    out_arcs_.resize(n_nodes);
    in_arcs_.resize(n_nodes);
    levels_.resize(n_nodes);
    
    // Initialize random levels (1 to n_nodes)
    std::vector<std::uint32_t> level_order(n_nodes);
    std::iota(level_order.begin(), level_order.end(), 0U);
    std::random_device rd;
    std::mt19937 gen(rd());
    std::shuffle(level_order.begin(), level_order.end(), gen);
    
    for (auto i = 0U; i < n_nodes; ++i) {
      levels_[level_order[i]] = i + 1;  // levels 1 to n_nodes
    }

    // Extract edges from OSR ways, ignoring turn restrictions
    for (auto way_idx = way_idx_t{0U}; way_idx < w.n_ways(); ++way_idx) {
      auto const& way = w.r_->way_properties_[way_idx];
      auto const& nodes = w.r_->way_nodes_[way_idx];
      
      if (nodes.size() < 2) continue;
      
      // Add edges between consecutive nodes in both directions (bidirectional)
      for (auto i = 0U; i < nodes.size() - 1; ++i) {
        auto const from = nodes[i];
        auto const to = nodes[i + 1];
        
        if (from != node_idx_t::invalid() && to != node_idx_t::invalid() &&
            from.v_ < n_nodes && to.v_ < n_nodes) {
          
          // Calculate simple distance-based weight (in seconds, like OSR)
          auto const from_pos = w.get_node_pos(from);
          auto const to_pos = w.get_node_pos(to);
          auto const dist = geo::distance(from_pos.as_latlng(), to_pos.as_latlng());
          
          // Use car speed of ~50 km/h = ~14 m/s for weight calculation
          auto const weight = static_cast<cost_t>(dist / 14.0);
          
          // Add bidirectional edges
          add_edge(from.v_, to.v_, weight);
          add_edge(to.v_, from.v_, weight);
        }
      }
    }
  }

  void add_edge(std::uint32_t from, std::uint32_t to, cost_t weight) {
    // Check if edge already exists and update with minimum weight
    auto& out_edges = out_arcs_[from];
    for (auto& arc : out_edges) {
      if (arc.target.v_ == to) {
        if (weight < arc.weight) {
          arc.weight = weight;
        }
        return;
      }
    }
    
    // Add new edge
    out_edges.emplace_back(node_idx_t{to}, weight);
    in_arcs_[to].emplace_back(node_idx_t{from}, weight);
  }

  void add_shortcut(std::uint32_t from, std::uint32_t to, cost_t weight, std::uint32_t middle) {
    // Check if shortcut already exists and update with minimum weight
    auto& out_edges = out_arcs_[from];
    for (auto& arc : out_edges) {
      if (arc.target.v_ == to && arc.is_shortcut) {
        if (weight < arc.weight) {
          arc.weight = weight;
          arc.shortcut_middle = node_idx_t{middle};
        }
        return;
      }
    }
    
    // Add new shortcut
    out_edges.emplace_back(node_idx_t{to}, weight, node_idx_t{middle});
    in_arcs_[to].emplace_back(node_idx_t{from}, weight, node_idx_t{middle});
  }

  std::uint32_t node_count() const {
    return static_cast<std::uint32_t>(out_arcs_.size());
  }

  std::vector<ch_arc> const& out_arcs(std::uint32_t node) const {
    return out_arcs_[node];
  }

  std::vector<ch_arc> const& in_arcs(std::uint32_t node) const {
    return in_arcs_[node];
  }

  std::uint32_t level(std::uint32_t node) const {
    return levels_[node];
  }

  bool is_upward_edge(std::uint32_t from, std::uint32_t to) const {
    return levels_[to] > levels_[from];
  }

  bool is_downward_edge(std::uint32_t from, std::uint32_t to) const {
    return levels_[from] > levels_[to];
  }

private:
  std::vector<std::vector<ch_arc>> out_arcs_;
  std::vector<std::vector<ch_arc>> in_arcs_;
  std::vector<std::uint32_t> levels_;
};

}  // namespace osr