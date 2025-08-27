#pragma once

#include <random>
#include <vector>
#include <algorithm>

#include "osr/types.h"

namespace osr {

/**
 * Storage for CH node levels. Each node gets a unique random level from 1 to n.
 * This defines the total ordering required for contraction hierarchies.
 */
class ch_levels {
public:
  using level_t = std::uint32_t;
  
  explicit ch_levels(std::size_t n_nodes) 
    : levels_(n_nodes) {
    assign_random_levels();
  }

  // Get level for a node (1-based, 1 = lowest, n = highest)
  level_t get_level(node_idx_t node) const {
    return levels_[to_idx(node)];
  }
  
  // Set level for a node (for testing purposes)
  void set_level(node_idx_t node, level_t level) {
    levels_[to_idx(node)] = level;
  }

  // Check if node u has lower level than node v (u < v in ordering)
  bool is_lower_level(node_idx_t u, node_idx_t v) const {
    return get_level(u) < get_level(v);
  }

  // Check if node u has higher level than node v (u > v in ordering)
  bool is_higher_level(node_idx_t u, node_idx_t v) const {
    return get_level(u) > get_level(v);
  }

  std::size_t size() const {
    return levels_.size();
  }

private:
  void assign_random_levels() {
    // Create levels 1, 2, 3, ..., n
    for (std::size_t i = 0; i < levels_.size(); ++i) {
      levels_[i] = static_cast<level_t>(i + 1);
    }
    
    // Shuffle to get random assignment
    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(levels_.begin(), levels_.end(), g);
  }

  std::vector<level_t> levels_;
};

}  // namespace osr