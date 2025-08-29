#pragma once

#include <functional>

#include "osr/elevation_storage.h"
#include "osr/location.h"
#include "osr/routing/ch_edge_expansion.h"
#include "osr/routing/ch_levels.h"
#include "osr/routing/ch_shortcut.h"
#include "osr/routing/dial.h"
#include "osr/routing/profiles/car.h"
#include "osr/routing/sharing_data.h"
#include "osr/types.h"
#include "osr/ways.h"

namespace osr {

/**
 * Single-direction CH Dijkstra search.
 * Uses level filtering to only explore upward edges in the CH hierarchy.
 */
template <typename Profile>
class ch_search {
public:
  using profile_t = Profile;
  using key = typename Profile::key;
  using label = typename Profile::label;
  using node = typename Profile::node;
  using entry = typename Profile::entry;
  using hash = typename Profile::hash;
  using cost_map = typename ankerl::unordered_dense::map<key, entry, hash>;

  struct get_bucket {
    cost_t operator()(label const& l) { return l.cost(); }
  };

  explicit ch_search(ch_levels const& levels, ch_shortcuts const& shortcuts)
      : levels_(levels), shortcuts_(shortcuts), expander_(levels, shortcuts) {}

  /**
   * Initialize search with maximum cost limit.
   */
  void init(cost_t const max_cost) {
    pq_.clear();
    // Cap bucket count to reasonable limit to avoid excessive memory usage
    auto const bucket_count = std::min(static_cast<std::uint32_t>(max_cost) + 1U, 100000U);
    pq_.n_buckets(bucket_count);
    costs_.clear();
    finished_ = false;
    initialized_ = true;
  }

  /**
   * Add starting node to search.
   */
  void add_start(node const& start_node, cost_t const initial_cost = cost_t{0}) {
    label const start_label{start_node, initial_cost};
    add_node(start_label);
  }

  /**
   * Check if search has more nodes to process.
   */
  bool has_next() const {
    return initialized_ && !finished_ && !pq_.empty();
  }

  /**
   * Get cost to reach a specific node.
   */
  cost_t get_cost(node const& n) const {
    auto const it = costs_.find(n.get_key());
    return it != costs_.end() ? it->second.cost(n) : kInfeasible;
  }

  /**
   * Perform one step of CH Dijkstra search.
   * Returns the node that was expanded, or invalid node if search is finished.
   */
  template <direction SearchDir, bool WithBlocked = false>
  node expand_step(ways const& w,
                   cost_t const max_cost,
                   bitvec<node_idx_t> const* blocked = nullptr,
                   sharing_data const* sharing = nullptr,
                   elevation_storage const* elevations = nullptr) {
    
    if (!has_next() || finished_) {
      return node::invalid();
    }

    // Get next node from priority queue
    auto const current_label = pq_.pop();
    auto const current_node = current_label.get_node();
    auto const current_cost = get_cost(current_node);

    // Check if this node is outdated (already processed with better cost)
    if (current_cost < current_label.cost()) {
      return expand_step<SearchDir, WithBlocked>(w, max_cost, blocked, sharing, elevations);
    }

    // Check cost limit
    if (current_cost >= max_cost) {
      finished_ = true;
      return node::invalid();
    }

    // Expand neighbors using CH edge filtering
    expand_neighbors<SearchDir, WithBlocked>(
        w, current_node, current_cost, blocked, sharing, elevations);

    return current_node;
  }

  /**
   * Run complete CH search until finished or cost limit reached.
   */
  template <direction SearchDir, bool WithBlocked = false>
  void run_complete(ways const& w,
                    cost_t const max_cost,
                    bitvec<node_idx_t> const* blocked = nullptr,
                    sharing_data const* sharing = nullptr,
                    elevation_storage const* elevations = nullptr) {
    while (has_next()) {
      auto const expanded = expand_step<SearchDir, WithBlocked>(w, max_cost, blocked, sharing, elevations);
      if (expanded == node::invalid()) {
        break;
      }
    }
  }

  /**
   * Get all reached nodes with their costs.
   */
  cost_map const& get_distances() const {
    return costs_;
  }

  /**
   * Check if search is finished.
   */
  bool is_finished() const {
    return finished_;
  }

  /**
   * Mark search as finished.
   */
  void finish() {
    finished_ = true;
  }

  /**
   * Get the minimum cost in the priority queue.
   * Returns kInfeasible if queue is empty.
   */
  cost_t get_next_cost() const {
    if (pq_.empty()) {
      return kInfeasible;
    }
    return pq_.buckets_[pq_.get_next_bucket()].back().cost();
  }

private:
  /**
   * Add node to search with given label.
   */
  void add_node(label const& l) {
    auto const& n = l.get_node();
    if (costs_[n.get_key()].update(l, n, l.cost(), node::invalid())) {
      pq_.push(l);
    }
  }

  /**
   * Expand neighbors of current node using CH edge filtering.
   */
  template <direction SearchDir, bool WithBlocked>
  void expand_neighbors(ways const& w,
                        node const& current,
                        cost_t const current_cost,
                        bitvec<node_idx_t> const* blocked,
                        sharing_data const* sharing,
                        elevation_storage const* elevations) {
    
    // Use CH edge expansion to get only level-filtered neighbors
    expander_.expand_ch_edges<SearchDir, WithBlocked>(
        w, current, level_t{0.0F}, blocked, sharing, elevations,
        [&](node const neighbor, std::uint32_t const edge_cost, distance_t const dist,
            way_idx_t const way, std::uint16_t const way_pos_from, std::uint16_t const way_pos_to,
            elevation_storage::elevation const elevation, bool const uses_elevator) {
          
          // Skip blocked nodes
          if constexpr (WithBlocked) {
            if (blocked != nullptr && (*blocked)[neighbor.get_node()]) {
              return;
            }
          }

          // Calculate new cost via this neighbor
          auto const new_cost = static_cast<cost_t>(current_cost + static_cast<cost_t>(edge_cost));
          
          // Create label for neighbor and try to add/update
          label const neighbor_label{neighbor, new_cost};
          add_node(neighbor_label);
        });
  }

private:
  ch_levels const& levels_;
  ch_shortcuts const& shortcuts_;
  ch_edge_expansion<Profile> expander_;

  // Search state
  dial<label, get_bucket> pq_{get_bucket{}};
  cost_map costs_;
  bool finished_ = false;
  bool initialized_ = false;
};

}  // namespace osr