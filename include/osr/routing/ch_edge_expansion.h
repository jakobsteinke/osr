#pragma once

#include <functional>

#include "osr/routing/ch_levels.h"
#include "osr/routing/ch_shortcut.h"
#include "osr/routing/profiles/car.h"
#include "osr/routing/sharing_data.h"
#include "osr/types.h"
#include "osr/ways.h"

namespace osr {

/**
 * Level-filtered edge expansion for CH queries.
 * Expands only edges (u, v) where level(v) > level(u).
 * Integrates with existing Profile::adjacent for turn restriction handling.
 */
template <typename Profile>
class ch_edge_expansion {
public:
  explicit ch_edge_expansion(ch_levels const& levels, ch_shortcuts const& shortcuts)
      : levels_(levels), shortcuts_(shortcuts) {}

  using node = typename Profile::node;

  /**
   * Expand edges from current node with CH level filtering.
   * Only explores edges to higher-level nodes.
   * 
   * @param w Ways data
   * @param current Current node to expand from
   * @param elevation_level Current elevation level
   * @param blocked Optional blocked nodes
   * @param sharing Optional sharing data for additional nodes
   * @param elevations Optional elevation storage
   * @param callback Function called for each valid neighbor: (node, cost, distance, way_idx, ...)
   */
  template <direction SearchDir, bool WithBlocked, typename Fn>
  void expand_ch_edges(ways const& w,
                       node const& current,
                       level_t const elevation_level,
                       bitvec<node_idx_t> const* blocked,
                       sharing_data const* sharing,
                       elevation_storage const* elevations,
                       Fn&& callback) const {
    
    int original_edges_found = 0;
    int original_edges_allowed = 0;
    
    // Use Profile::adjacent to get all possible neighbors respecting turn restrictions
    Profile::template adjacent<SearchDir, WithBlocked>(
        *w.r_, current, blocked, sharing, elevations,
        [&](node const neighbor, std::uint32_t const cost, distance_t const dist,
            way_idx_t const way, std::uint16_t const way_pos_from, std::uint16_t const way_pos_to,
            elevation_storage::elevation const elevation, bool const uses_elevator) {
          
          original_edges_found++;
          
          // RoutingKit-style: For direct edges, we still need level filtering
        // since we can't pre-filter all original graph edges efficiently
        // TODO: In full RoutingKit implementation, even original edges would be pre-separated
        if (is_ch_edge_allowed<SearchDir>(current.get_node(), neighbor.get_node())) {
          original_edges_allowed++;
          callback(neighbor, cost, dist, way, way_pos_from, way_pos_to, elevation, uses_elevator);
        }
        });
    
    int shortcuts_found = 0;
    int shortcuts_allowed = 0;
    
    // Also expand shortcut edges from current node
    expand_shortcut_edges<SearchDir>(w, current, [&](auto&&... args) {
      shortcuts_found++;
      shortcuts_allowed++;
      callback(args...);
    });
    
    // Debug output for first test only to see search patterns
    static int debug_test = 0;
    static int expansion_count = 0;
    
    if (debug_test == 0 && expansion_count < 15) {
      expansion_count++;
      std::cout << "DEBUG " << expansion_count << " - Node:" << current.get_node().v_ 
                << " Level:" << levels_.get_level(current.get_node())
                << " Dir:" << (SearchDir == direction::kForward ? "F" : "B")
                << " Orig:" << original_edges_allowed << "/" << original_edges_found
                << " Short:" << shortcuts_allowed << "/" << shortcuts_found;
      
      // Show what neighbors we actually expand to
      if (shortcuts_allowed > 0 || original_edges_allowed > 0) {
        std::cout << " -> EXPANDING";
      } else {
        std::cout << " -> BLOCKED";
      }
      std::cout << "\n";
      
      if (expansion_count >= 15) {
        std::cout << "=== STOPPING DEBUG OUTPUT ===\n";
        debug_test = 1;  // Stop further debug output
      }
    }
  }

  /**
   * Check if edge (from, to) is allowed in CH query based on search direction.
   * 
   * RoutingKit-style implementation with relaxed filtering for original graph edges.
   * Shortcut edges are already pre-filtered during preprocessing, but original
   * graph edges still need some filtering since we can't efficiently pre-separate them all.
   */
  template <direction SearchDir>
  bool is_ch_edge_allowed(node_idx_t from, node_idx_t to) const {
    // RELAXED FILTERING for better connectivity with random ordering
    // This allows the searches to have more overlap while maintaining hierarchy benefits
    
    auto const from_level = levels_.get_level(from);
    auto const to_level = levels_.get_level(to);
    
    if constexpr (SearchDir == direction::kForward) {
      // Forward search: prefer upward moves but allow limited downward for connectivity
      if (to_level > from_level) {
        return true;  // Always allow upward moves
      }
      // Allow limited downward moves to improve connectivity
      return false;
      //return (from_level - to_level) <= 100;  // Configurable tolerance
    } else {
      // Backward search: prefer downward moves but allow limited upward for connectivity  
      if (from_level > to_level) {
        return true;  // Always allow downward moves (backward search logic)
      }
      return false;
      // Allow limited upward moves to improve connectivity
      //return (to_level - from_level) <= 100;  // Configurable tolerance
    }
  }
  
  // Legacy method for compatibility - now properly delegates to template version
  bool is_ch_edge_allowed(node_idx_t from, node_idx_t to) const {
    // Default to forward search behavior for legacy calls
    return is_ch_edge_allowed<direction::kForward>(from, to);
  }

private:
  /**
   * Expand shortcut edges from current node using RoutingKit-style direction-specific graphs.
   * Forward search uses forward graph (pre-filtered upward edges).
   * Backward search uses backward graph (pre-filtered downward edges).
   * No runtime level filtering needed - it's "baked in" during preprocessing.
   */
  template <direction SearchDir, typename Fn>
  void expand_shortcut_edges(ways const& w, node const& current, Fn&& callback) const {
    auto const current_node = current.get_node();
    
    if constexpr (SearchDir == direction::kForward) {
      // Forward search: use forward shortcuts graph (pre-filtered upward edges)
      for (auto const& [edge_pair, shortcut_list] : shortcuts_.get_forward_shortcuts()) {
        auto const [from, to] = edge_pair;
        
        if (from == current_node) {
          // Found outgoing shortcut from current node
          for (auto const& shortcut : shortcut_list) {
            // No level filtering needed - forward graph is already filtered
            expand_shortcut_to_node(w, to, shortcut.cost_, callback);
          }
        }
      }
    } else {
      // Backward search: use backward shortcuts graph (pre-filtered downward edges)
      for (auto const& [edge_pair, shortcut_list] : shortcuts_.get_backward_shortcuts()) {
        auto const [from, to] = edge_pair;
        
        if (to == current_node) {
          // Found incoming shortcut to current node (reversed direction)
          for (auto const& shortcut : shortcut_list) {
            // No level filtering needed - backward graph is already filtered
            expand_shortcut_to_node(w, from, shortcut.cost_, callback);
          }
        }
      }
    }
  }
  
private:
  /**
   * Helper to expand a shortcut to a target node.
   */
  template <typename Fn>
  void expand_shortcut_to_node(ways const& w, node_idx_t target_node_idx, cost_t cost, Fn&& callback) const {
    // Create Profile::node for shortcut target
    // We need to properly create all valid car::node instances for the target
    if (target_node_idx.v_ < w.n_nodes()) {
      int node_count = 0;
      // Use Profile::resolve_all to get all valid car::node instances for the target
      Profile::resolve_all(*w.r_, target_node_idx, level_t{0.0F}, 
          [&](node const target_node) {
            node_count++;
            // Call callback with shortcut cost for each valid car::node
            // Use dummy values for way/position/elevation since shortcuts don't track these
            callback(target_node, cost, distance_t{0}, way_idx_t{0}, 
                    0, 0, elevation_storage::elevation{}, false);
          });
      
      // Debug: Log how many car::nodes were created for this shortcut target
      // if (node_count > 0) {
      //   std::cout << "Shortcut expansion: node " << target_node_idx.v_ 
      //             << " -> " << node_count << " car::nodes\n";
      // }
    }
  }

private:
  ch_levels const& levels_;
  ch_shortcuts const& shortcuts_;
};

}  // namespace osr