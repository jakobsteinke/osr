#pragma once

#include <numeric>

#include "boost/json/object.hpp"

#include "utl/helpers/algorithm.h"

#include "osr/elevation_storage.h"
#include "osr/routing/mode.h"
#include "osr/ways.h"

namespace osr {

struct sharing_data;

/**
 * Simple profile without turn restrictions for CH compatibility testing.
 * Behaves like RoutingKit - simple node traversal without complex turn logic.
 */
struct simple {
  static constexpr auto const kMaxMatchDistance = 200U;
  static constexpr auto const kUturnPenalty = cost_t{0U};  // No U-turn penalty

  using key = node_idx_t;

  struct node {
    friend bool operator==(node, node) = default;

    static constexpr node invalid() noexcept {
      return node{.n_ = node_idx_t::invalid()};
    }

    boost::json::object geojson_properties(ways const&) const {
      return boost::json::object{{"node_id", n_.v_}, {"type", "simple"}};
    }

    constexpr node_idx_t get_node() const noexcept { return n_; }
    constexpr node_idx_t get_key() const noexcept { return n_; }

    static constexpr mode get_mode() noexcept { return mode::kCar; }

    std::ostream& print(std::ostream& out, ways const& w) const {
      return out << "(node=" << w.node_to_osm_[n_] << ")";
    }

    node_idx_t n_;
  };

  using label = node;

  struct entry {
    constexpr cost_t cost(node const& n) const noexcept { 
      return n == n_ ? cost_ : kInfeasible; 
    }
    
    node n_;
    cost_t cost_;
  };

  struct hash {
    std::size_t operator()(node_idx_t const n) const noexcept {
      return std::hash<std::uint32_t>{}(n.v_);
    }
  };

  /**
   * Simple adjacent function without turn restrictions.
   * Just traverses all connected nodes without complex logic.
   */
  template <direction SearchDir, bool WithBlocked, typename Fn>
  static void adjacent(ways::routing const& w,
                       node const n,
                       bitvec<node_idx_t> const* blocked,
                       sharing_data const*,
                       elevation_storage const*,
                       Fn&& fn) {
    
    // Simple traversal: visit all connected nodes via all ways
    for (auto const [way, way_pos] : 
         utl::zip_unchecked(w.node_ways_[n.n_], w.node_in_way_idx_[n.n_])) {
      
      auto const& way_nodes = w.way_nodes_[way];
      auto const way_properties = w.way_properties_[way];
      
      // Skip if way is not accessible for cars
      if (!way_properties.is_car_accessible()) {
        continue;
      }

      // Traverse in both directions (if allowed)
      auto const process_neighbor = [&](std::uint16_t const neighbor_pos) {
        if (neighbor_pos < way_nodes.size()) {
          auto const neighbor_node_idx = way_nodes[neighbor_pos];
          
          if constexpr (WithBlocked) {
            if (blocked && blocked->test(neighbor_node_idx)) {
              return;
            }
          }
          
          // Skip if neighbor is the same as current
          if (neighbor_node_idx == n.n_) {
            return;
          }
          
          // Calculate cost (simple distance-based)
          auto const distance = std::abs(static_cast<int>(neighbor_pos) - static_cast<int>(way_pos));
          auto const cost = cost_t{static_cast<std::uint32_t>(distance * 100)};  // Simple cost model
          
          // Create simple neighbor node
          node const neighbor{.n_ = neighbor_node_idx};
          
          // Call callback with neighbor
          fn(neighbor, cost, distance_t{static_cast<std::uint16_t>(distance * 100)}, 
             way, way_pos, neighbor_pos, elevation_storage::elevation{}, false);
        }
      };

      // Visit neighbors in both directions on the way
      if (way_pos > 0) {
        process_neighbor(way_pos - 1);
      }
      if (way_pos + 1 < way_nodes.size()) {
        process_neighbor(way_pos + 1);
      }
    }
  }

  /**
   * Simple resolve_all - just creates one node per node_idx.
   */
  template <typename Fn>
  static void resolve_all(ways::routing const&, 
                         node_idx_t const node_idx, 
                         level_t const,
                         Fn&& fn) {
    fn(node{.n_ = node_idx});
  }

  /**
   * No complex matching logic - just return the node.
   */
  static std::vector<node> match(ways const&,
                                level_t const,
                                location const& loc,
                                double const,
                                mode const,
                                bool const,
                                direction const) {
    // Simplified: just return invalid node (would need proper implementation for real use)
    return {};
  }

  /**
   * Simple node cost - always feasible.
   */
  static cost_t node_cost(std::uint8_t const) noexcept {
    return cost_t{0U};
  }

  /**
   * Simple edge cost calculation.
   */
  static cost_t edge_cost(ways::routing const&,
                         way_idx_t const,
                         std::uint16_t const,
                         std::uint16_t const,
                         direction const) noexcept {
    return cost_t{100U};  // Simple fixed cost
  }
};

}  // namespace osr