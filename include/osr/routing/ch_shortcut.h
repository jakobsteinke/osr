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
 * Storage for CH shortcuts. Maps (from, to) pairs to shortcuts.
 * Multiple shortcuts can exist for the same (from, to) pair if they
 * have different costs or go via different middle nodes.
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
  
  // Add a shortcut
  void add_shortcut(node_idx_t from, node_idx_t to, node_idx_t via, cost_t cost) {
    shortcuts_[{from, to}].emplace_back(ch_shortcut{from, to, via, cost});
  }
  
  // Get all shortcuts from 'from' to 'to'
  std::vector<ch_shortcut> const* get_shortcuts(node_idx_t from, node_idx_t to) const {
    auto const it = shortcuts_.find({from, to});
    return it != shortcuts_.end() ? &it->second : nullptr;
  }
  
  // Get the best (cheapest) shortcut from 'from' to 'to'
  std::optional<ch_shortcut> get_best_shortcut(node_idx_t from, node_idx_t to) const {
    auto const* shortcuts = get_shortcuts(from, to);
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
  
  // Check if a shortcut exists from 'from' to 'to'
  bool has_shortcut(node_idx_t from, node_idx_t to) const {
    return shortcuts_.find({from, to}) != shortcuts_.end();
  }
  
  // Get total number of shortcuts
  std::size_t size() const {
    std::size_t total = 0;
    for (auto const& [key, shortcuts] : shortcuts_) {
      total += shortcuts.size();
    }
    return total;
  }
  
  // Clear all shortcuts
  void clear() {
    shortcuts_.clear();
  }
  
  // Get all shortcut edges (for iteration)
  auto const& get_all() const {
    return shortcuts_;
  }

private:
  ankerl::unordered_dense::map<edge_key, std::vector<ch_shortcut>, edge_key_hash> shortcuts_;
};

}  // namespace osr