#pragma once

#include <vector>
#include <unordered_map>

#include "ankerl/unordered_dense.h"

#include "osr/types.h"

namespace osr {

struct ch_shortcut {
  node_idx_t from_;
  node_idx_t to_;
  cost_t cost_;
  node_idx_t contracted_node_;
  way_idx_t edge1_;
  way_idx_t edge2_;
  bool is_shortcut1_;
  bool is_shortcut2_;
};

using ch_level_t = std::uint32_t;

struct ch_data {
  ankerl::unordered_dense::map<node_idx_t, ch_level_t> node_levels_;
  
  ankerl::unordered_dense::map<node_idx_t, std::vector<ch_shortcut>> forward_shortcuts_;
  ankerl::unordered_dense::map<node_idx_t, std::vector<ch_shortcut>> backward_shortcuts_;
  
  ch_level_t get_level(node_idx_t n) const {
    auto it = node_levels_.find(n);
    return it != node_levels_.end() ? it->second : 0;
  }
  
  bool is_upward_edge(node_idx_t from, node_idx_t to) const {
    return get_level(from) < get_level(to);
  }
  
  std::vector<ch_shortcut> const* get_forward_shortcuts(node_idx_t n) const {
    auto it = forward_shortcuts_.find(n);
    return it != forward_shortcuts_.end() ? &it->second : nullptr;
  }
  
  std::vector<ch_shortcut> const* get_backward_shortcuts(node_idx_t n) const {
    auto it = backward_shortcuts_.find(n);
    return it != backward_shortcuts_.end() ? &it->second : nullptr;
  }
  
  void add_shortcut(node_idx_t from, node_idx_t to, cost_t cost,
                    node_idx_t contracted_node,
                    way_idx_t edge1, way_idx_t edge2,
                    bool is_shortcut1, bool is_shortcut2) {
    ch_shortcut sc{from, to, cost, contracted_node, 
                   edge1, edge2, is_shortcut1, is_shortcut2};
    forward_shortcuts_[from].push_back(sc);
    backward_shortcuts_[to].push_back(sc);
  }
  
  void clear() {
    node_levels_.clear();
    forward_shortcuts_.clear();
    backward_shortcuts_.clear();
  }
};

}  // namespace osr