#pragma once

#include <limits>
#include <vector>

#include "ankerl/unordered_dense.h"

#include "osr/elevation_storage.h"
#include "osr/location.h"
#include "osr/routing/ch_levels.h"
#include "osr/routing/ch_shortcut.h"
#include "osr/routing/dial.h"
#include "osr/routing/sharing_data.h"
#include "osr/types.h"
#include "osr/ways.h"

namespace osr {

struct sharing_data;

/**
 * Bidirectional Dijkstra with Contraction Hierarchies (CH) level filtering.
 * Both forward and backward searches only use upward edges (level filtering).
 */
template <typename Profile>
struct ch_query {
  using profile_t = Profile;
  using key = typename Profile::key;
  using label = typename Profile::label;
  using node = typename Profile::node;
  using entry = typename Profile::entry;
  using hash = typename Profile::hash;
  using cost_map = typename ankerl::unordered_dense::map<key, entry, hash>;

  constexpr static auto const kDebug = false;

  struct get_bucket {
    cost_t operator()(label const& l) { return l.cost(); }
  };

  /**
   * Constructor requires CH preprocessed data.
   */
  explicit ch_query(ch_levels const& levels, ch_shortcuts const& shortcuts)
      : levels_(levels), shortcuts_(shortcuts) {
    // Initialize dial queues properly
    forward_pq_.clear();
    backward_pq_.clear();
  }

  /**
   * Clear meeting point state.
   */
  void clear_meeting_point() {
    forward_meet_point_ = node::invalid();
    backward_meet_point_ = node::invalid();
    tentative_shortest_path_ = kInfeasible;
  }

  /**
   * Reset query state for new search.
   */
  void reset(cost_t const max, location const& start_loc, location const& end_loc) {
    forward_pq_.clear();
    backward_pq_.clear();
    forward_pq_.n_buckets(max + 1U);
    backward_pq_.n_buckets(max + 1U);
    forward_costs_.clear();
    backward_costs_.clear();
    clear_meeting_point();
    start_loc_ = start_loc;
    end_loc_ = end_loc;
    forward_finished_ = false;
    backward_finished_ = false;
  }

  /**
   * Add node to forward search (upward in CH).
   */
  void add_forward_start(ways const& w, label const l, sharing_data const* sharing) {
    if (kDebug) {
      l.get_node().print(std::cout, w);
      std::cout << " CH forward start " << l.get_node().n_ << std::endl;
    }
    add_to_search(l, direction::kForward, forward_costs_, forward_pq_);
  }

  /**
   * Add node to backward search (upward in CH, but on inverted graph).
   */
  void add_backward_start(ways const& w, label const l, sharing_data const* sharing) {
    if (kDebug) {
      l.get_node().print(std::cout, w);
      std::cout << " CH backward start " << l.get_node().n_ << std::endl;
    }
    add_to_search(l, direction::kBackward, backward_costs_, backward_pq_);
  }

  /**
   * Get cost to reach node in given search direction.
   */
  template <direction SearchDir>
  cost_t get_cost(node const n) const {
    if constexpr (SearchDir == direction::kForward) {
      auto const it = forward_costs_.find(n.get_key());
      return it != end(forward_costs_) ? it->second.cost(n) : kInfeasible;
    } else {
      auto const it = backward_costs_.find(n.get_key());
      return it != end(backward_costs_) ? it->second.cost(n) : kInfeasible;
    }
  }

  /**
   * Check if edge (u, v) is allowed in CH query.
   * Both searches only use upward edges: level(v) > level(u).
   */
  bool is_ch_edge_allowed(node_idx_t from, node_idx_t to) const {
    return levels_.is_higher_level(to, from);
  }

  /**
   * Update meeting point if this creates a better path.
   */
  void update_meeting_point(node const forward_node, node const backward_node) {
    auto const forward_cost = get_cost<direction::kForward>(forward_node);
    auto const backward_cost = get_cost<direction::kBackward>(backward_node);
    
    if (forward_cost == kInfeasible || backward_cost == kInfeasible) {
      return;
    }
    
    auto const total_cost = forward_cost + backward_cost;
    
    if (total_cost < tentative_shortest_path_) {
      tentative_shortest_path_ = total_cost;
      forward_meet_point_ = forward_node;
      backward_meet_point_ = backward_node;
    }
  }

  /**
   * Check if search should terminate (abort-on-success).
   * Terminate when all remaining PQ keys > tentative shortest path.
   */
  bool should_terminate() const {
    if (tentative_shortest_path_ == kInfeasible) {
      return false; // No meeting point found yet
    }
    
    // Get next bucket cost for forward search
    auto const forward_next_cost = forward_finished_ || forward_pq_.empty() 
        ? kInfeasible
        : forward_pq_.buckets_[forward_pq_.get_next_bucket()].back().cost();
    
    // Get next bucket cost for backward search
    auto const backward_next_cost = backward_finished_ || backward_pq_.empty()
        ? kInfeasible 
        : backward_pq_.buckets_[backward_pq_.get_next_bucket()].back().cost();
    
    // Terminate if both searches are done or their next costs exceed tentative
    auto const forward_done = forward_finished_ || forward_pq_.empty() || 
                              forward_next_cost >= tentative_shortest_path_;
    
    auto const backward_done = backward_finished_ || backward_pq_.empty() ||
                               backward_next_cost >= tentative_shortest_path_;
    
    return forward_done && backward_done;
  }

  /**
   * Get tentative shortest path cost.
   */
  cost_t get_tentative_cost() const {
    return tentative_shortest_path_;
  }

  /**
   * Get meeting point nodes.
   */
  std::pair<node, node> get_meeting_point() const {
    return {forward_meet_point_, backward_meet_point_};
  }

  /**
   * Check if forward search queue is empty or finished.
   */
  bool is_forward_finished() const {
    return forward_finished_ || forward_pq_.empty();
  }

  /**
   * Check if backward search queue is empty or finished.
   */
  bool is_backward_finished() const {
    return backward_finished_ || backward_pq_.empty();
  }

  /**
   * Mark search direction as finished.
   */
  void mark_finished(direction dir) {
    if (dir == direction::kForward) {
      forward_finished_ = true;
    } else {
      backward_finished_ = true;
    }
  }

private:
  /**
   * Add label to search queue.
   */
  void add_to_search(label const l, 
                     direction const dir,
                     cost_map& cost_map, 
                     dial<label, get_bucket>& pq) {
    // CH search doesn't use heuristic, so cost = actual distance
    if (cost_map[l.get_node().get_key()].update(l, l.get_node(), l.cost(), node::invalid())) {
      pq.push(l);
    }
  }

public:
  // CH preprocessed data (references)
  ch_levels const& levels_;
  ch_shortcuts const& shortcuts_;

  // Search state
  location start_loc_;
  location end_loc_;

  // Priority queues for both directions (properly initialized)
  dial<label, get_bucket> forward_pq_{get_bucket{}};
  dial<label, get_bucket> backward_pq_{get_bucket{}};

  // Distance tracking
  cost_map forward_costs_;
  cost_map backward_costs_;

  // Meeting point state
  node forward_meet_point_{node::invalid()};
  node backward_meet_point_{node::invalid()};
  cost_t tentative_shortest_path_{kInfeasible};

  // Termination state  
  bool forward_finished_ = false;
  bool backward_finished_ = false;
};

}  // namespace osr