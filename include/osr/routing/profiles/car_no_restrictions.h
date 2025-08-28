#pragma once

#include "osr/routing/profiles/car.h"

namespace osr {

/**
 * Car profile without turn restrictions for CH performance.
 * Inherits all car functionality but bypasses turn restriction checks.
 */
struct car_no_restrictions : public car {
  
  /**
   * Adjacent function that bypasses turn restrictions.
   * Same as car::adjacent but removes the is_restricted check.
   */
  template <direction SearchDir, bool WithBlocked, typename Fn>
  static void adjacent(ways::routing const& w,
                       node const n,
                       bitvec<node_idx_t> const* blocked,
                       sharing_data const*,
                       elevation_storage const*,
                       Fn&& fn) {
    auto way_pos = way_pos_t{0U};
    for (auto const [way, i] :
         utl::zip_unchecked(w.node_ways_[n.n_], w.node_in_way_idx_[n.n_])) {
      auto const expand = [&](direction const way_dir, std::uint16_t const from,
                              std::uint16_t const to) {
        // NOLINTNEXTLINE(clang-analyzer-core.CallAndMessage)
        auto const target_node = w.way_nodes_[way][to];
        if constexpr (WithBlocked) {
          if (blocked->test(target_node)) {
            return;
          }
        }

        auto const target_node_prop = w.node_properties_[target_node];
        if (node_cost(target_node_prop) == kInfeasible) {
          return;
        }

        auto const target_way_prop = w.way_properties_[way];
        if (way_cost(target_way_prop, way_dir, 0U) == kInfeasible) {
          return;
        }

        // REMOVED: Turn restriction check for CH performance
        // if (w.is_restricted<SearchDir>(n.n_, n.way_, way_pos)) {
        //   return;
        // }
        
        // DEBUG: Enable to verify this profile is being used
        // static int call_count = 0;
        // if (call_count < 3) {
        //   std::cout << "car_no_restrictions::adjacent called " << ++call_count << "/3 (no turn restrictions)\n";
        // }

        auto const is_u_turn = way_pos == n.way_ && way_dir == opposite(n.dir_);
        auto const dist = w.way_node_dist_[way][std::min(from, to)];
        auto const target =
            node{target_node, w.get_way_pos(target_node, way, to), way_dir};
        auto const cost = way_cost(target_way_prop, way_dir, dist) +
                          node_cost(target_node_prop) +
                          (is_u_turn ? kUturnPenalty : 0U);
        fn(target, cost, dist, way, from, to, elevation_storage::elevation{},
           false);
      };

      if (i != 0U) {
        expand(flip<SearchDir>(direction::kBackward), i, i - 1);
      }
      if (i != w.way_nodes_[way].size() - 1) {
        expand(SearchDir, i, i + 1);
      }
      ++way_pos;
    }
  }
};

}  // namespace osr