#pragma once

#include <vector>
#include <optional>
#include <unordered_map>
#include <mutex>

#include "ankerl/unordered_dense.h"

#include "osr/types.h"
#include "osr/routing/profiles/car.h"

namespace osr {

/**
 * A shortcut edge from node 'from' to node 'to' via middle node 'via'.
 * This represents a path (from -> via -> to) that was contracted during
 * CH preprocessing.
 */
struct ch_shortcut {
  node_idx_t from_;
  node_idx_t to_;
  node_idx_t via_;      // The contracted node this shortcut goes through
  cost_t cost_;
  distance_t distance_; // Total distance of the shortcut path
  
  ch_shortcut() = default;
  
  ch_shortcut(node_idx_t from, node_idx_t to, node_idx_t via, cost_t cost, distance_t distance = 0U)
    : from_(from), to_(to), via_(via), cost_(cost), distance_(distance) {}
  
  bool operator==(ch_shortcut const& other) const {
    return from_ == other.from_ && to_ == other.to_ && 
           via_ == other.via_ && cost_ == other.cost_ && distance_ == other.distance_;
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
  using node_hash = ankerl::unordered_dense::hash<node_idx_t>;
  
  struct edge_key_hash {
    using is_avalanching = void;
    auto operator()(edge_key const& k) const noexcept -> std::uint64_t {
      using namespace ankerl::unordered_dense::detail;
      return wyhash::hash(static_cast<std::uint64_t>(to_idx(k.first)) << 32 | 
                         static_cast<std::uint64_t>(to_idx(k.second)));
    }
  };
  
  // Add a shortcut to the appropriate direction-specific graph (with deduplication)
  void add_forward_shortcut(node_idx_t from, node_idx_t to, node_idx_t via, cost_t cost, distance_t distance = 0U) {
    auto& vec = forward_shortcuts_[{from, to}];
    
    // Keep only the best (min cost) shortcut per (from, to) pair
    auto it = std::min_element(vec.begin(), vec.end(),
                               [](auto const& a, auto const& b) { return a.cost_ < b.cost_; });
    
    if (it == vec.end()) {
      // No existing shortcuts, add the new one
      vec.emplace_back(ch_shortcut{from, to, via, cost, distance});
      forward_outgoing_[from].emplace_back(ch_shortcut{from, to, via, cost, distance});
    } else if (cost < it->cost_) {
      // New shortcut is better, replace the best existing one
      *it = ch_shortcut{from, to, via, cost, distance};
      
      // Update secondary index by 'to' key only (one entry per (from,to))
      auto& out_vec = forward_outgoing_[from];
      auto it_out = std::find_if(out_vec.begin(), out_vec.end(),
          [&](auto const& s){ return s.to_ == to; });
      if (it_out != out_vec.end()) {
        *it_out = *it;                 // replace stale entry
      } else {
        out_vec.emplace_back(*it);     // repair a missing index
      }
      
      // Hygiene: remove any duplicates for this (from,to) pair
      out_vec.erase(std::remove_if(out_vec.begin(), out_vec.end(),
        [&](auto const& s){ return s.to_ == to && &s != &*it_out; }), out_vec.end());
    }
    // else: ignore equal-or-worse shortcuts
  }
  
  void add_backward_shortcut(node_idx_t from, node_idx_t to, node_idx_t via, cost_t cost, distance_t distance = 0U) {
    auto& vec = backward_shortcuts_[{from, to}];
    
    // Keep only the best (min cost) shortcut per (from, to) pair
    auto it = std::min_element(vec.begin(), vec.end(),
                               [](auto const& a, auto const& b) { return a.cost_ < b.cost_; });
    
    if (it == vec.end()) {
      // No existing shortcuts, add the new one
      vec.emplace_back(ch_shortcut{from, to, via, cost, distance});
      backward_incoming_[to].emplace_back(ch_shortcut{from, to, via, cost, distance});
    } else if (cost < it->cost_) {
      // New shortcut is better, replace the best existing one
      *it = ch_shortcut{from, to, via, cost, distance};
      
      // Update secondary index by 'from' key only (one entry per (from,to))
      auto& in_vec = backward_incoming_[to];
      auto it_in = std::find_if(in_vec.begin(), in_vec.end(),
          [&](auto const& s){ return s.from_ == from; });
      if (it_in != in_vec.end()) {
        *it_in = *it;                  // replace stale entry
      } else {
        in_vec.emplace_back(*it);      // repair a missing index
      }
      
      // Hygiene: remove any duplicates for this (from,to) pair
      in_vec.erase(std::remove_if(in_vec.begin(), in_vec.end(),
        [&](auto const& s){ return s.from_ == from && &s != &*it_in; }), in_vec.end());
    }
    // else: ignore equal-or-worse shortcuts
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
    backward_incoming_.clear();
    forward_outgoing_.clear();
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
  
  // Get all forward shortcuts starting from a given source node
  std::vector<ch_shortcut> get_forward_shortcuts_from(node_idx_t from) const {
    auto it = forward_outgoing_.find(from);
    if (it == forward_outgoing_.end()) return {};
    return it->second; // return by value (matches existing API)
  }
  
  // Get all backward shortcuts starting from a given source node
  std::vector<ch_shortcut> get_backward_shortcuts_from(node_idx_t from) const {
    std::vector<ch_shortcut> result;
    for (auto const& [key, shortcuts] : backward_shortcuts_) {
      if (key.first == from) {
        for (auto const& shortcut : shortcuts) {
          result.push_back(shortcut);
        }
      }
    }
    return result;
  }
  
  // Get all backward shortcuts ending at a given target node (for backward search)
  std::vector<ch_shortcut> get_backward_shortcuts_to(node_idx_t to) const {
    auto it = backward_incoming_.find(to);
    if (it == backward_incoming_.end()) return {};
    return it->second; // return by value
  }

private:
  ankerl::unordered_dense::map<edge_key, std::vector<ch_shortcut>, edge_key_hash> forward_shortcuts_;
  ankerl::unordered_dense::map<edge_key, std::vector<ch_shortcut>, edge_key_hash> backward_shortcuts_;
  // Secondary index for incoming backward shortcuts: key = 'to'
  ankerl::unordered_dense::map<node_idx_t, std::vector<ch_shortcut>, node_hash> backward_incoming_;
  // Secondary index for outgoing forward shortcuts: key = 'from'
  ankerl::unordered_dense::map<node_idx_t, std::vector<ch_shortcut>, node_hash> forward_outgoing_;
};

// Thread-safe wrapper for global shortcut storage
class ch_shortcuts_global {
public:
  void add_forward_shortcut(node_idx_t from, node_idx_t to, node_idx_t via, cost_t cost, distance_t distance = 0U) {
    std::lock_guard<std::mutex> lock(mutex_);
    shortcuts_.add_forward_shortcut(from, to, via, cost, distance);
  }
  
  void add_backward_shortcut(node_idx_t from, node_idx_t to, node_idx_t via, cost_t cost, distance_t distance = 0U) {
    std::lock_guard<std::mutex> lock(mutex_);
    shortcuts_.add_backward_shortcut(from, to, via, cost, distance);
  }
  
  std::optional<ch_shortcut> get_best_forward_shortcut(node_idx_t from, node_idx_t to) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return shortcuts_.get_best_forward_shortcut(from, to);
  }
  
  std::optional<ch_shortcut> get_best_backward_shortcut(node_idx_t from, node_idx_t to) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return shortcuts_.get_best_backward_shortcut(from, to);
  }
  
  bool has_forward_shortcut(node_idx_t from, node_idx_t to) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return shortcuts_.has_forward_shortcut(from, to);
  }
  
  bool has_backward_shortcut(node_idx_t from, node_idx_t to) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return shortcuts_.has_backward_shortcut(from, to);
  }
  
  void clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    shortcuts_.clear();
  }
  
  std::size_t size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return shortcuts_.size();
  }

  std::size_t forward_size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return shortcuts_.forward_size();
  }

  std::size_t backward_size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return shortcuts_.backward_size();
  }
  
  // Get all forward shortcuts starting from a given source node  
  std::vector<ch_shortcut> get_forward_shortcuts_from(node_idx_t from) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return shortcuts_.get_forward_shortcuts_from(from);
  }
  
  // Get all backward shortcuts starting from a given source node
  std::vector<ch_shortcut> get_backward_shortcuts_from(node_idx_t from) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return shortcuts_.get_backward_shortcuts_from(from);
  }
  
  // Get all backward shortcuts ending at a given target node (for backward search)
  std::vector<ch_shortcut> get_backward_shortcuts_to(node_idx_t to) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return shortcuts_.get_backward_shortcuts_to(to);
  }
  
private:
  mutable std::mutex mutex_;
  ch_shortcuts shortcuts_;
};

// Global shortcut storage - one per profile type
template <typename Profile>
ch_shortcuts_global& get_ch_shortcuts() {
  static ch_shortcuts_global store;
  return store;
}

// Specialization for car profile (the only one we implement CH for)
template <>
inline ch_shortcuts_global& get_ch_shortcuts<car>() {
  static ch_shortcuts_global store;
  return store;
}

// Helper functions for car profile shortcut management
inline void add_ch_forward_shortcut(node_idx_t from, node_idx_t to, node_idx_t via, cost_t cost, distance_t distance = 0U) {
  get_ch_shortcuts<car>().add_forward_shortcut(from, to, via, cost, distance);
}

inline void add_ch_backward_shortcut(node_idx_t from, node_idx_t to, node_idx_t via, cost_t cost, distance_t distance = 0U) {
  get_ch_shortcuts<car>().add_backward_shortcut(from, to, via, cost, distance);
}

// Removed generic add_ch_shortcut - use add_ch_forward_shortcut or add_ch_backward_shortcut instead
// to ensure shortcuts are added to the correct graph based on level relationships

inline std::optional<ch_shortcut> get_best_ch_forward_shortcut(node_idx_t from, node_idx_t to) {
  return get_ch_shortcuts<car>().get_best_forward_shortcut(from, to);
}

inline std::optional<ch_shortcut> get_best_ch_backward_shortcut(node_idx_t from, node_idx_t to) {
  return get_ch_shortcuts<car>().get_best_backward_shortcut(from, to);
}

inline bool has_ch_forward_shortcut(node_idx_t from, node_idx_t to) {
  return get_ch_shortcuts<car>().has_forward_shortcut(from, to);
}

inline bool has_ch_backward_shortcut(node_idx_t from, node_idx_t to) {
  return get_ch_shortcuts<car>().has_backward_shortcut(from, to);
}

inline void clear_ch_shortcuts() {
  get_ch_shortcuts<car>().clear();
}

inline std::size_t ch_shortcuts_count() {
  return get_ch_shortcuts<car>().size();
}

}  // namespace osr