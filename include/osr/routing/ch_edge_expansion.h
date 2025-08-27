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
          
          // Apply CH level filtering: only allow edges to higher-level nodes
          if (is_ch_edge_allowed(current.get_node(), neighbor.get_node())) {
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
    
    // Temporarily disable debug output
    // Debug output for edge expansion
    // if (original_edges_found > 0 || shortcuts_found > 0) {
    //   std::cout << "CH Edge Expansion - Node:" << current.get_node().v_ 
    //             << " Dir:" << (SearchDir == direction::kForward ? "F" : "B")
    //             << " Original:" << original_edges_allowed << "/" << original_edges_found
    //             << " Shortcuts:" << shortcuts_allowed << "/" << shortcuts_found << "\n";
    // }
  }

  /**
   * Check if edge (from, to) is allowed in CH query.
   * Both forward and backward searches use upward edges: level(to) > level(from).
   */
  bool is_ch_edge_allowed(node_idx_t from, node_idx_t to) const {
    return levels_.is_higher_level(to, from);
  }

private:
  /**
   * Expand shortcut edges from current node.
   * Following the working Python CH implementation:
   * - Forward search: find shortcuts FROM current node (outgoing shortcuts)
   * - Backward search: find shortcuts TO current node (incoming shortcuts) 
   * Both maintain upward level filtering: only go to higher-level nodes.
   */
  template <direction SearchDir, typename Fn>
  void expand_shortcut_edges(ways const& w, node const& current, Fn&& callback) const {
    auto const current_node = current.get_node();
    
    // Special debug for our test nodes
    if (current_node.v_ == 680 || current_node.v_ == 436) {
      std::cout << "SHORTCUT DEBUG: Expanding shortcuts from node " << current_node.v_ 
                << " direction=" << (SearchDir == direction::kForward ? "F" : "B") << "\n";
      std::cout << "  Looking through " << shortcuts_.get_all().size() << " shortcut entries\n";
    }
    
    // Look for shortcuts involving current node
    for (auto const& [edge_pair, shortcut_list] : shortcuts_.get_all()) {
      auto const [from, to] = edge_pair;
      
      if constexpr (SearchDir == direction::kForward) {
        // Forward search: expand outgoing shortcuts (current -> target)
        if (from == current_node) {
          // Special debug for our test case
          if (current_node.v_ == 680) {
            std::cout << "  Found outgoing shortcut from " << from.v_ << " to " << to.v_ << "\n";
          }
          
          for (auto const& shortcut : shortcut_list) {
            // Apply level filtering: only go to higher-level nodes
            if (is_ch_edge_allowed(current_node, to)) {
              std::cout << "Using shortcut from node " << current_node.v_ 
                        << " to node " << to.v_ << " cost=" << shortcut.cost_ << "\n";
              expand_shortcut_to_node(w, to, shortcut.cost_, callback);
            } else {
              std::cout << "Blocked shortcut: " << current_node.v_ << " -> " << to.v_ 
                        << " (level " << levels_.get_level(current_node) << " -> " 
                        << levels_.get_level(to) << ")\n";
            }
          }
        }
      } else {
        // Backward search: expand incoming shortcuts (source -> current)
        // This follows Python logic: backward search uses predecessors
        if (to == current_node) {
          for (auto const& shortcut : shortcut_list) {
            // Apply level filtering: only go to higher-level nodes
            // Note: from the perspective of backward search, we go from current to 'from'
            // but level filtering still requires 'from' to have higher level than current
            if (is_ch_edge_allowed(current_node, from)) {
              std::cout << "Using shortcut from node " << current_node.v_ 
                        << " to node " << from.v_ << " cost=" << shortcut.cost_ << "\n";
              expand_shortcut_to_node(w, from, shortcut.cost_, callback);
            }
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
      if (node_count > 0) {
        std::cout << "Shortcut expansion: node " << target_node_idx.v_ 
                  << " -> " << node_count << " car::nodes\\n";
      }
    }
  }

private:
  ch_levels const& levels_;
  ch_shortcuts const& shortcuts_;
};

}  // namespace osr