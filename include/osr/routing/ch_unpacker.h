#pragma once

#include <vector>

#include "osr/elevation_storage.h"
#include "osr/location.h"
#include "osr/routing/ch_shortcut.h"
#include "osr/routing/profiles/car.h"
#include "osr/routing/sharing_data.h"
#include "osr/types.h"
#include "osr/ways.h"

namespace osr {

/**
 * Shortcut unpacker for CH path reconstruction.
 * Recursively unpacks shortcuts to obtain paths in the original graph.
 */
template <typename Profile>
class ch_unpacker {
public:
  explicit ch_unpacker(ch_shortcuts const& shortcuts) 
      : shortcuts_(shortcuts) {}

  using node = typename Profile::node;
  
  /**
   * Unpack a single shortcut edge recursively.
   * Returns the unpacked path as a sequence of nodes.
   */
  std::vector<node> unpack_shortcut(node const& from, node const& to) const {
    // Find the shortcut between these nodes
    auto const shortcut = shortcuts_.get_best_shortcut(from.get_node(), to.get_node());
    
    if (!shortcut.has_value()) {
      // No shortcut exists - this is a regular edge
      return {from, to};
    }
    
    // Recursively unpack the shortcut via the middle node
    auto const via_node_idx = shortcut->via_;
    
    // We need to create Profile::node instances for the via node
    // For now, use simplified approach - will be enhanced with proper way resolution
    auto via_node = node::invalid();
    via_node.n_ = via_node_idx;
    
    auto left_path = unpack_shortcut(from, via_node);
    auto right_path = unpack_shortcut(via_node, to);
    
    // Merge paths (avoid duplicating the via node)
    left_path.insert(left_path.end(), right_path.begin() + 1, right_path.end());
    return left_path;
  }

  /**
   * Unpack a complete path containing shortcuts.
   * Takes a path with potential shortcuts and returns fully unpacked path.
   */
  std::vector<node> unpack_path(std::vector<node> const& path) const {
    if (path.size() < 2) {
      return path; // Nothing to unpack
    }
    
    std::vector<node> unpacked_path;
    unpacked_path.reserve(path.size() * 2); // Estimate for unpacked size
    
    // Add first node
    unpacked_path.push_back(path[0]);
    
    // Unpack each edge in the path
    for (std::size_t i = 1; i < path.size(); ++i) {
      auto const edge_path = unpack_shortcut(path[i-1], path[i]);
      
      // Add all nodes except the first (to avoid duplication)
      unpacked_path.insert(unpacked_path.end(), edge_path.begin() + 1, edge_path.end());
    }
    
    return unpacked_path;
  }

  /**
   * Get the first edge from a shortcut for turn restriction checking.
   * Recursively unpacks only the first edge to determine actual incoming way.
   */
  std::pair<node, node> get_first_edge(node const& from, node const& to) const {
    auto const shortcut = shortcuts_.get_best_shortcut(from.get_node(), to.get_node());
    
    if (!shortcut.has_value()) {
      // Regular edge - return as-is
      return {from, to};
    }
    
    // Recursively get first edge of the first part of the shortcut
    auto const via_node_idx = shortcut->via_;
    auto via_node = node::invalid();
    via_node.n_ = via_node_idx;
    
    return get_first_edge(from, via_node);
  }

  /**
   * Check if an edge is a shortcut.
   */
  bool is_shortcut(node const& from, node const& to) const {
    return shortcuts_.get_best_shortcut(from.get_node(), to.get_node()).has_value();
  }

  /**
   * Check if a shortcut edge is valid considering turn restrictions.
   * Unpacks only the first edge to determine actual incoming way for turn checking.
   */
  template <bool WithBlocked = false>
  bool is_shortcut_turn_allowed(ways const& w,
                                node const& predecessor,
                                node const& from, 
                                node const& to,
                                bitvec<node_idx_t> const* blocked = nullptr,
                                sharing_data const* sharing = nullptr,
                                elevation_storage const* elevations = nullptr) const {
    
    if (!is_shortcut(from, to)) {
      // Regular edge - check turn restrictions normally using car::adjacent
      return check_turn_allowed<WithBlocked>(w, predecessor, from, to, blocked, sharing, elevations);
    }
    
    // This is a shortcut - unpack first edge to check turn restrictions
    auto const [first_from, first_to] = get_first_edge(from, to);
    
    // Check turn restriction from predecessor to first actual edge
    return check_turn_allowed<WithBlocked>(w, predecessor, first_from, first_to, blocked, sharing, elevations);
  }

  /**
   * Validate that an entire unpacked path respects turn restrictions.
   */
  template <bool WithBlocked = false>
  bool validate_path_turns(ways const& w,
                           std::vector<node> const& path,
                           bitvec<node_idx_t> const* blocked = nullptr,
                           sharing_data const* sharing = nullptr,
                           elevation_storage const* elevations = nullptr) const {
    
    if (path.size() < 3) {
      return true; // No turns to validate
    }
    
    // Check each turn in the path
    for (std::size_t i = 2; i < path.size(); ++i) {
      auto const& predecessor = path[i-2];
      auto const& current = path[i-1]; 
      auto const& next = path[i];
      
      if (!check_turn_allowed<WithBlocked>(w, predecessor, current, next, blocked, sharing, elevations)) {
        return false; // Turn restriction violated
      }
    }
    
    return true; // All turns are valid
  }

  /**
   * Get the cost of a shortcut edge, accounting for any additional penalties.
   */
  cost_t get_shortcut_cost(node const& from, node const& to) const {
    auto const shortcut = shortcuts_.get_best_shortcut(from.get_node(), to.get_node());
    return shortcut.has_value() ? shortcut->cost_ : kInfeasible;
  }

private:
  /**
   * Check if a turn from predecessor through current to next is allowed.
   */
  template <bool WithBlocked>
  bool check_turn_allowed(ways const& w,
                          node const& predecessor,
                          node const& current,
                          node const& next,
                          bitvec<node_idx_t> const* blocked,
                          sharing_data const* sharing,
                          elevation_storage const* elevations) const {
    
    // Use Profile::adjacent to check if the transition is valid
    bool turn_found = false;
    
    Profile::template adjacent<direction::kForward, WithBlocked>(
        *w.r_, current, blocked, sharing, elevations,
        [&](node const neighbor, std::uint32_t const cost, distance_t const dist,
            way_idx_t const way, std::uint16_t const way_pos_from, std::uint16_t const way_pos_to,
            elevation_storage::elevation const elevation, bool const uses_elevator) {
          
          if (neighbor == next) {
            // Found the neighbor - this turn is allowed
            turn_found = true;
          }
        });
    
    return turn_found;
  }

private:
  ch_shortcuts const& shortcuts_;
};

}  // namespace osr