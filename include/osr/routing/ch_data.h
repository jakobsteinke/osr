#pragma once

#include <vector>
#include <unordered_map>

#include "ankerl/unordered_dense.h"

#include "osr/types.h"

namespace osr {

// Car-specific key for CH that includes way and direction information
struct car_ch_key {
  car_ch_key() = default;
  car_ch_key(node_idx_t n, way_pos_t w, direction d) : n_{n}, way_{w}, dir_{d} {}
  
  friend bool operator==(car_ch_key const& a, car_ch_key const& b) {
    return a.n_ == b.n_ && a.way_ == b.way_ && a.dir_ == b.dir_;
  }
  
  static car_ch_key invalid() {
    return car_ch_key{node_idx_t::invalid(), way_pos_t{0}, direction::kForward};
  }
  
  node_idx_t n_;
  way_pos_t way_;
  direction dir_;
};

// Hash function for car_ch_key
struct car_ch_key_hash {
  std::size_t operator()(car_ch_key const& k) const {
    // Combine hash of node_idx_t with way and direction
    std::size_t h1 = std::hash<std::uint32_t>{}(k.n_.v_);
    std::size_t h2 = std::hash<std::uint16_t>{}(k.way_);
    std::size_t h3 = std::hash<bool>{}(k.dir_ == direction::kForward);
    return h1 ^ (h2 << 1) ^ (h3 << 2);
  }
};

struct ch_shortcut {
  car_ch_key from_;
  car_ch_key to_;
  cost_t cost_;
  car_ch_key contracted_node_;  // The car state that was contracted
  // Metadata for unpacking - store the middle node and edge info
  car_ch_key middle_node_;
  way_idx_t edge1_;
  way_idx_t edge2_;
};

using ch_level_t = std::uint32_t;

struct ch_data {
  ankerl::unordered_dense::map<car_ch_key, ch_level_t, car_ch_key_hash> node_levels_;
  
  ankerl::unordered_dense::map<car_ch_key, std::vector<ch_shortcut>, car_ch_key_hash> forward_shortcuts_;
  ankerl::unordered_dense::map<car_ch_key, std::vector<ch_shortcut>, car_ch_key_hash> backward_shortcuts_;
  
  ch_level_t get_level(car_ch_key const& k) const {
    auto it = node_levels_.find(k);
    return it != node_levels_.end() ? it->second : std::numeric_limits<ch_level_t>::max();
  }
  
  // Convenience method for backward compatibility
  ch_level_t get_level(node_idx_t n) const {
    // Find the minimum level among all car states for this physical node
    ch_level_t min_level = std::numeric_limits<ch_level_t>::max();
    for (auto const& [key, level] : node_levels_) {
      if (key.n_ == n) {
        min_level = std::min(min_level, level);
      }
    }
    return min_level;
  }
  
  bool is_upward_edge(car_ch_key const& from, car_ch_key const& to) const {
    return get_level(from) < get_level(to);
  }
  
  // Convenience method for backward compatibility  
  bool is_upward_edge(node_idx_t from, node_idx_t to) const {
    return get_level(from) < get_level(to);
  }
  
  std::vector<ch_shortcut> const* get_forward_shortcuts(car_ch_key const& k) const {
    auto it = forward_shortcuts_.find(k);
    return it != forward_shortcuts_.end() ? &it->second : nullptr;
  }
  
  // Convenience method for backward compatibility
  std::vector<ch_shortcut> const* get_forward_shortcuts(node_idx_t n) const {
    // For now, return shortcuts from first found car state for this node
    // TODO: This should be improved to handle car states properly
    for (auto const& [key, shortcuts] : forward_shortcuts_) {
      if (key.n_ == n) {
        return &shortcuts;
      }
    }
    return nullptr;
  }
  
  std::vector<ch_shortcut> const* get_backward_shortcuts(car_ch_key const& k) const {
    auto it = backward_shortcuts_.find(k);
    return it != backward_shortcuts_.end() ? &it->second : nullptr;
  }
  
  // Convenience method for backward compatibility
  std::vector<ch_shortcut> const* get_backward_shortcuts(node_idx_t n) const {
    // For now, return shortcuts from first found car state for this node
    // TODO: This should be improved to handle car states properly
    for (auto const& [key, shortcuts] : backward_shortcuts_) {
      if (key.n_ == n) {
        return &shortcuts;
      }
    }
    return nullptr;
  }
  
  void add_shortcut(car_ch_key const& from, car_ch_key const& to, cost_t cost,
                    car_ch_key const& contracted_node, car_ch_key const& middle_node,
                    way_idx_t edge1, way_idx_t edge2) {
    ch_shortcut sc{from, to, cost, contracted_node, middle_node, edge1, edge2};
    forward_shortcuts_[from].push_back(sc);
    backward_shortcuts_[to].push_back(sc);
  }
  
  // Backward compatibility method - creates basic car_ch_key
  void add_shortcut(node_idx_t from, node_idx_t to, cost_t cost,
                    node_idx_t contracted_node,
                    way_idx_t edge1, way_idx_t edge2,
                    bool is_shortcut1, bool is_shortcut2) {
    // Create basic car_ch_key - this is a temporary solution
    car_ch_key from_key{from, way_pos_t{0}, direction::kForward};
    car_ch_key to_key{to, way_pos_t{0}, direction::kForward};
    car_ch_key contracted_key{contracted_node, way_pos_t{0}, direction::kForward};
    car_ch_key middle_key{contracted_node, way_pos_t{0}, direction::kForward};
    add_shortcut(from_key, to_key, cost, contracted_key, middle_key, edge1, edge2);
  }
  
  void clear() {
    node_levels_.clear();
    forward_shortcuts_.clear();
    backward_shortcuts_.clear();
  }
};

}  // namespace osr