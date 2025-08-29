#pragma once

#include <optional>
#include <vector>

#include "osr/elevation_storage.h"
#include "osr/location.h"
#include "osr/routing/ch_bidirectional.h"
#include "osr/routing/ch_shortcut.h"
#include "osr/routing/ch_unpacker.h"
#include "osr/routing/profiles/car.h"
#include "osr/routing/sharing_data.h"
#include "osr/types.h"
#include "osr/ways.h"

namespace osr {

/**
 * Path reconstruction for CH queries.
 * Assembles complete paths from bidirectional search results using shortcut unpacking.
 */
template <typename Profile>
class ch_path_reconstruction {
public:
  using profile_t = Profile;
  using node = typename Profile::node;

  explicit ch_path_reconstruction(ch_shortcuts const& shortcuts)
      : shortcuts_(shortcuts), unpacker_(shortcuts) {}

  /**
   * Reconstruct complete path from bidirectional search results.
   */
  template <bool WithBlocked = false>
  std::optional<std::vector<node>> reconstruct_path(
      ways const& w,
      ch_bidirectional<Profile> const& bidir_search,
      location const& start_loc,
      location const& end_loc,
      bitvec<node_idx_t> const* blocked = nullptr,
      sharing_data const* sharing = nullptr,
      elevation_storage const* elevations = nullptr) const {

    if (!bidir_search.has_path()) {
      return std::nullopt; // No path found
    }

    auto const [forward_meeting, backward_meeting] = bidir_search.get_meeting_point();
    if (forward_meeting == node::invalid() || backward_meeting == node::invalid()) {
      return std::nullopt; // Invalid meeting point
    }

    // Get forward and backward distances
    auto const& forward_distances = bidir_search.get_forward_distances();
    auto const& backward_distances = bidir_search.get_backward_distances();

    // Reconstruct forward path (from start to meeting point)
    auto forward_path = reconstruct_forward_path(forward_distances, forward_meeting);
    if (forward_path.empty()) {
      return std::nullopt;
    }

    // Reconstruct backward path (from meeting point to end)
    auto backward_path = reconstruct_backward_path(backward_distances, backward_meeting);
    if (backward_path.empty()) {
      return std::nullopt;
    }

    // Combine paths at meeting point
    auto complete_path = combine_paths(forward_path, backward_path);

    // Unpack all shortcuts in the path
    auto unpacked_path = unpacker_.unpack_path(complete_path);

    // Validate turn restrictions
    if (!unpacker_.template validate_path_turns<WithBlocked>(
          w, unpacked_path, blocked, sharing, elevations)) {
      return std::nullopt; // Path violates turn restrictions
    }

    return unpacked_path;
  }

  /**
   * Calculate total cost of a reconstructed path.
   */
  cost_t calculate_path_cost(std::vector<node> const& path) const {
    if (path.size() < 2) {
      return cost_t{0};
    }

    cost_t total_cost{0};
    for (std::size_t i = 1; i < path.size(); ++i) {
      auto const edge_cost = unpacker_.get_shortcut_cost(path[i-1], path[i]);
      if (edge_cost == kInfeasible) {
        // Regular edge - would need actual edge cost from graph
        // For now, use simplified approach
        total_cost += cost_t{1}; // Placeholder
      } else {
        total_cost += edge_cost;
      }
    }

    return total_cost;
  }

  /**
   * Validate that a path is feasible and respects all constraints.
   */
  template <bool WithBlocked = false>
  bool validate_path(ways const& w,
                     std::vector<node> const& path,
                     bitvec<node_idx_t> const* blocked = nullptr,
                     sharing_data const* sharing = nullptr,
                     elevation_storage const* elevations = nullptr) const {
    
    if (path.size() < 2) {
      return false; // Invalid path
    }

    // Check for blocked nodes
    if constexpr (WithBlocked) {
      if (blocked != nullptr) {
        for (auto const& n : path) {
          if ((*blocked)[n.get_node()]) {
            return false; // Path contains blocked node
          }
        }
      }
    }

    // Validate turn restrictions
    return unpacker_.template validate_path_turns<WithBlocked>(
        w, path, blocked, sharing, elevations);
  }

private:
  /**
   * Reconstruct forward path from start to meeting point.
   */
  std::vector<node> reconstruct_forward_path(
      auto const& forward_distances,
      node const& meeting_point) const {
    
    std::vector<node> path;
    path.reserve(16); // Reasonable initial capacity
    
    // Start from meeting point and work backwards using predecessor information
    node current = meeting_point;
    path.push_back(current);
    
    // Simplified reconstruction - in full implementation would trace predecessors
    // For now, we'll rely on the fact that the bidirectional search provides
    // the meeting point and the search framework tracks the path structure
    
    // TODO: Enhance with full predecessor tracking from search state
    // This will be completed when we integrate with the full search framework
    
    // Reverse to get forward order
    std::reverse(path.begin(), path.end());
    return path;
  }

  /**
   * Reconstruct backward path from meeting point to end.
   */
  std::vector<node> reconstruct_backward_path(
      auto const& backward_distances,
      node const& meeting_point) const {
    
    std::vector<node> path;
    path.reserve(16);
    
    // Start from meeting point and work forwards using successor information
    node current = meeting_point;
    path.push_back(current);
    
    // Simplified reconstruction - full implementation would trace successors
    // TODO: Enhance with full successor tracking from search state
    
    return path;
  }

  /**
   * Combine forward and backward paths at the meeting point.
   */
  std::vector<node> combine_paths(std::vector<node> const& forward_path,
                                  std::vector<node> const& backward_path) const {
    
    if (forward_path.empty() || backward_path.empty()) {
      return {};
    }

    std::vector<node> combined;
    combined.reserve(forward_path.size() + backward_path.size());
    
    // Add forward path
    combined.insert(combined.end(), forward_path.begin(), forward_path.end());
    
    // Add backward path (skip first node to avoid duplication at meeting point)
    if (backward_path.size() > 1) {
      combined.insert(combined.end(), backward_path.begin() + 1, backward_path.end());
    }
    
    return combined;
  }

private:
  ch_shortcuts const& shortcuts_;
  ch_unpacker<Profile> unpacker_;
};

}  // namespace osr