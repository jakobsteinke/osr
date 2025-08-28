#pragma once

#include <vector>
#include <optional>
#include <unordered_map>

#include "ankerl/unordered_dense.h"

#include "osr/types.h"

namespace osr {

/**
 * A shortcut edge from node 'from' to node 'to' via middle node 'via'.
 * This represents a path (from -> via -> to) that was contracted during
 * CH preprocessing.
 */
struct ch_shortcut {
  node_idx_t from_;
  node_idx_t to_;
  node_idx_t via_;  // The contracted node this shortcut goes through
  cost_t cost_;
  
  bool operator==(ch_shortcut const& other) const {
    return from_ == other.from_ && to_ == other.to_ && 
           via_ == other.via_ && cost_ == other.cost_;
  }
};

/**
 * Storage for CH shortcuts with separate forward/backward graphs (RoutingKit-style).
 * Forward graph contains upward edges (from lower to higher level nodes).
 * Backward graph contains downward edges (from higher to lower level nodes).
 * This eliminates the need for runtime level filtering during queries.
 */
class ch_shortcuts {
public:
  using edge_key = std::pair<node_idx_t, node_idx_t>;
  
  struct edge_key_hash {
    using is_avalanching = void;
    auto operator()(edge_key const& k) const noexcept -> std::uint64_t {
      using namespace ankerl::unordered_dense::detail;
      return wyhash::hash(static_cast<std::uint64_t>(to_idx(k.first)) << 32 | 
                         static_cast<std::uint64_t>(to_idx(k.second)));
    }
  };
  
  // Add a shortcut to the appropriate direction-specific graph
  void add_forward_shortcut(node_idx_t from, node_idx_t to, node_idx_t via, cost_t cost) {
    forward_shortcuts_[{from, to}].emplace_back(ch_shortcut{from, to, via, cost});
  }
  
  void add_backward_shortcut(node_idx_t from, node_idx_t to, node_idx_t via, cost_t cost) {
    backward_shortcuts_[{from, to}].emplace_back(ch_shortcut{from, to, via, cost});
  }
  
  // Legacy method for backward compatibility - determines direction automatically
  void add_shortcut(node_idx_t from, node_idx_t to, node_idx_t via, cost_t cost) {
    // For now, add to both until we update all callers
    add_forward_shortcut(from, to, via, cost);
    add_backward_shortcut(from, to, via, cost);
  }
  
  // Get forward shortcuts from 'from' to 'to'
  std::vector<ch_shortcut> const* get_forward_shortcuts(node_idx_t from, node_idx_t to) const {
    auto const it = forward_shortcuts_.find({from, to});
    return it != forward_shortcuts_.end() ? &it->second : nullptr;
  }
  
  // Get backward shortcuts from 'from' to 'to'  
  std::vector<ch_shortcut> const* get_backward_shortcuts(node_idx_t from, node_idx_t to) const {
    auto const it = backward_shortcuts_.find({from, to});
    return it != backward_shortcuts_.end() ? &it->second : nullptr;
  }
  
  // Legacy method - searches both graphs
  std::vector<ch_shortcut> const* get_shortcuts(node_idx_t from, node_idx_t to) const {
    // Check forward first, then backward
    auto forward = get_forward_shortcuts(from, to);
    if (forward) return forward;
    return get_backward_shortcuts(from, to);
  }
  
  // Get the best (cheapest) shortcut from forward graph
  std::optional<ch_shortcut> get_best_forward_shortcut(node_idx_t from, node_idx_t to) const {
    auto const* shortcuts = get_forward_shortcuts(from, to);
    if (!shortcuts || shortcuts->empty()) {
      return std::nullopt;
    }
    
    auto best = shortcuts->begin();
    for (auto it = shortcuts->begin() + 1; it != shortcuts->end(); ++it) {
      if (it->cost_ < best->cost_) {
        best = it;
      }
    }
    return *best;
  }
  
  // Get the best (cheapest) shortcut from backward graph
  std::optional<ch_shortcut> get_best_backward_shortcut(node_idx_t from, node_idx_t to) const {
    auto const* shortcuts = get_backward_shortcuts(from, to);
    if (!shortcuts || shortcuts->empty()) {
      return std::nullopt;
    }
    
    auto best = shortcuts->begin();
    for (auto it = shortcuts->begin() + 1; it != shortcuts->end(); ++it) {
      if (it->cost_ < best->cost_) {
        best = it;
      }
    }
    return *best;
  }
  
  // Legacy method - searches both graphs  
  std::optional<ch_shortcut> get_best_shortcut(node_idx_t from, node_idx_t to) const {
    auto forward = get_best_forward_shortcut(from, to);
    auto backward = get_best_backward_shortcut(from, to);
    
    if (!forward && !backward) return std::nullopt;
    if (!forward) return backward;
    if (!backward) return forward;
    
    return forward->cost_ <= backward->cost_ ? forward : backward;
  }
  
  // Check if shortcuts exist in either graph
  bool has_shortcut(node_idx_t from, node_idx_t to) const {
    return forward_shortcuts_.find({from, to}) != forward_shortcuts_.end() ||
           backward_shortcuts_.find({from, to}) != backward_shortcuts_.end();
  }
  
  // Check if forward shortcut exists
  bool has_forward_shortcut(node_idx_t from, node_idx_t to) const {
    return forward_shortcuts_.find({from, to}) != forward_shortcuts_.end();
  }
  
  // Check if backward shortcut exists
  bool has_backward_shortcut(node_idx_t from, node_idx_t to) const {
    return backward_shortcuts_.find({from, to}) != backward_shortcuts_.end();
  }
  
  // Get total number of shortcuts
  std::size_t size() const {
    std::size_t total = 0;
    for (auto const& [key, shortcuts] : forward_shortcuts_) {
      total += shortcuts.size();
    }
    for (auto const& [key, shortcuts] : backward_shortcuts_) {
      total += shortcuts.size();
    }
    return total;
  }
  
  // Get forward shortcuts count
  std::size_t forward_size() const {
    std::size_t total = 0;
    for (auto const& [key, shortcuts] : forward_shortcuts_) {
      total += shortcuts.size();
    }
    return total;
  }
  
  // Get backward shortcuts count
  std::size_t backward_size() const {
    std::size_t total = 0;
    for (auto const& [key, shortcuts] : backward_shortcuts_) {
      total += shortcuts.size();
    }
    return total;
  }
  
  // Clear all shortcuts
  void clear() {
    forward_shortcuts_.clear();
    backward_shortcuts_.clear();
  }
  
  // Get forward shortcuts (for iteration)
  auto const& get_forward_shortcuts() const {
    return forward_shortcuts_;
  }
  
  // Get backward shortcuts (for iteration)
  auto const& get_backward_shortcuts() const {
    return backward_shortcuts_;
  }
  
  // Legacy method - returns forward shortcuts for compatibility
  auto const& get_all() const {
    return forward_shortcuts_;
  }

private:
  ankerl::unordered_dense::map<edge_key, std::vector<ch_shortcut>, edge_key_hash> forward_shortcuts_;
  ankerl::unordered_dense::map<edge_key, std::vector<ch_shortcut>, edge_key_hash> backward_shortcuts_;
};

}  // namespace osr