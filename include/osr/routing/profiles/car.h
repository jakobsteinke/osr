#pragma once

#include <bitset>

#include "boost/json/object.hpp"

#include "utl/helpers/algorithm.h"

#include "fmt/core.h"

#include "osr/elevation_storage.h"
#include "osr/routing/mode.h"
#include "osr/routing/route.h"
#include "osr/ways.h"

namespace osr {

struct sharing_data;

struct car {
  static constexpr auto const kMaxMatchDistance = 200U;
  static constexpr auto const kUturnPenalty = cost_t{120U};
  
  // Special way position to mark shortcut-generated nodes
  static constexpr auto const kShortcutWayPos = way_pos_t{std::numeric_limits<way_pos_t>::max() - 1U};

  using key = node_idx_t;

  struct node {
    friend bool operator==(node, node) = default;

    static constexpr node invalid() noexcept {
      return node{
          .n_ = node_idx_t::invalid(), .way_ = 0U, .dir_ = direction::kForward};
    }

    boost::json::object geojson_properties(ways const&) const {
      return boost::json::object{{"node_id", n_.v_}, {"type", "car"}};
    }

    constexpr node_idx_t get_node() const noexcept { return n_; }
    constexpr node_idx_t get_key() const noexcept { return n_; }

    static constexpr mode get_mode() noexcept { return mode::kCar; }
    
    // Helper function to detect shortcut-generated nodes
    static constexpr bool is_shortcut_node(node const n) noexcept {
      return n.way_ == kShortcutWayPos;
    }

    std::ostream& print(std::ostream& out, ways const& w) const {
      if (is_shortcut_node(*this)) {
        return out << "(shortcut_node=" << w.node_to_osm_[n_] << ", dir=" << to_str(dir_) << ")";
      } else {
        return out << "(node=" << w.node_to_osm_[n_] << ", dir=" << to_str(dir_)
                   << ", way=" << w.way_osm_idx_[w.r_->node_ways_[n_][way_]]
                   << ")";
      }
    }

    node_idx_t n_;
    way_pos_t way_;
    direction dir_;
  };

  struct label {
    label(node const n, cost_t const c)
        : n_{n.n_}, way_{n.way_}, dir_{n.dir_}, cost_{c} {}

    constexpr node get_node() const noexcept { return {n_, way_, dir_}; }
    constexpr cost_t cost() const noexcept { return cost_; }

    void track(
        label const&, ways::routing const&, way_idx_t, node_idx_t, bool) {}

    node_idx_t n_;
    way_pos_t way_;
    direction dir_;
    cost_t cost_;
  };

  struct entry {
    static constexpr auto const kMaxWays = way_pos_t{16U};
    static constexpr auto const kN = kMaxWays * 2U /* FWD+BWD */;

    entry() { utl::fill(cost_, kInfeasible); }

    constexpr std::optional<node> pred(node const n) const noexcept {
      // Shortcut nodes don't have valid predecessors in fixed arrays
      if (node::is_shortcut_node(n)) {
        return std::nullopt;
      }
      auto const idx = get_index(n);
      return pred_[idx] == node_idx_t::invalid()
                 ? std::nullopt
                 : std::optional{node{pred_[idx], pred_way_[idx],
                                      to_dir(pred_dir_[idx])}};
    }

    constexpr cost_t cost(node const n) const noexcept {
      // Shortcut nodes use a separate storage mechanism 
      if (node::is_shortcut_node(n)) {
        return kInfeasible; // Should be handled by bidirectional dijkstra's cost maps
      }
      return cost_[get_index(n)];
    }

    /*constexpr*/ bool update(label const&,
                          node const n,
                          cost_t const c,
                          node const pred) noexcept {
      // Shortcut nodes don't use fixed arrays - handled by bidirectional dijkstra's cost maps
      if (node::is_shortcut_node(n)) {
        return true; // Always accept updates for shortcut nodes
      }
      //std::cout << "[DEBUG] update: node=" << n.n_ << " cost=" << c << " pred=" << pred.n_ << std::endl;
      auto const idx = get_index(n);
      if (c < cost_[idx]) {
        cost_[idx] = c;
        pred_[idx] = pred.n_;
        pred_way_[idx] = pred.way_;
        pred_dir_[idx] = to_bool(pred.dir_);
        //std::cout << "[DEBUG]   updated!" << std::endl;
        return true;
      }
      //std::cout << "[DEBUG]   not updated." << std::endl;
      return false;
    }

    void write(node, path&) const {}

    static constexpr node get_node(node_idx_t const n,
                                   std::size_t const index) {
      return node{n, static_cast<way_pos_t>(index % kMaxWays),
                  to_dir((index / kMaxWays) != 0U)};
    }

    static constexpr std::size_t get_index(node const n) {
      // Shortcut nodes should not use fixed array indexing
      if (node::is_shortcut_node(n)) {
        return 0U; // Return safe index, but this should not be used
      }
      return (n.dir_ == direction::kForward ? 0U : 1U) * kMaxWays + n.way_;
    }

    static constexpr direction to_dir(bool const b) {
      return b == false ? direction::kForward : direction::kBackward;
    }

    static constexpr bool to_bool(direction const d) {
      return d == direction::kForward ? false : true;
    }

    std::array<node_idx_t, kN> pred_;
    std::array<way_pos_t, kN> pred_way_;
    std::bitset<kN> pred_dir_;
    std::array<cost_t, kN> cost_;
  };

  struct hash {
    using is_avalanching = void;
    auto operator()(key const n) const noexcept -> std::uint64_t {
      using namespace ankerl::unordered_dense::detail;
      return wyhash::hash(static_cast<std::uint64_t>(to_idx(n)));
    }
  };

  template <typename Fn>
  static void resolve_start_node(ways::routing const& w,
                                 way_idx_t const way,
                                 node_idx_t const n,
                                 level_t,
                                 direction,
                                 Fn&& f) {
    auto const ways = w.node_ways_[n];
    for (auto i = way_pos_t{0U}; i != ways.size(); ++i) {
      if (ways[i] == way) {
        f(node{n, i, direction::kForward});
        f(node{n, i, direction::kBackward});
      }
    }
  }

  template <typename Fn>
  static void resolve_all(ways::routing const& w,
                          node_idx_t const n,
                          level_t,
                          Fn&& f) {
    auto const ways = w.node_ways_[n];
    for (auto i = way_pos_t{0U}; i != ways.size(); ++i) {
      f(node{n, i, direction::kForward});
      f(node{n, i, direction::kBackward});
    }
  }

  template <direction SearchDir, bool WithBlocked, typename Fn>
  static void adjacent(ways::routing const& w,
                       node const n,
                       bitvec<node_idx_t> const* blocked,
                       sharing_data const*,
                       elevation_storage const*,
                       Fn&& fn,
                       bool use_ch = false) { 
    // ===== SHORTCUT NODE DETECTION =====
    // Handle shortcut-generated nodes separately to avoid vector access issues
    if (node::is_shortcut_node(n)) {
      // For shortcut nodes, only process outgoing shortcuts - no regular edges
      if (use_ch && w.contraction_hierarchy_enabled_ && !w.shortcuts_.empty()) {
        auto const current_node = n.n_;
        auto const shortcut_it = w.outgoing_shortcuts_.find(current_node);
        if (shortcut_it != w.outgoing_shortcuts_.end()) {
          auto const& shortcut_indices = shortcut_it->second;
          
          for (auto const shortcut_idx : shortcut_indices) {
            if (shortcut_idx >= w.shortcuts_.size()) continue;
            
            auto const& shortcut = w.shortcuts_[shortcut_idx];
            auto const target_node = shortcut.to;
            
            // Basic validation
            if (target_node == node_idx_t::invalid() || 
                to_idx(target_node) >= w.node_properties_.size()) {
              continue;
            }
            
            auto const target_node_prop = w.node_properties_[target_node];
            if (node_cost(target_node_prop) == kInfeasible || shortcut.cost >= kInfeasible) {
              continue;
            }
            
            // Create shortcut target 
            auto const target = node{target_node, kShortcutWayPos, SearchDir};
            fn(target, shortcut.cost, 0U, way_idx_t::invalid(), 0U, 0U, 
               elevation_storage::elevation{}, true); // Mark as shortcut
          }
        }
      }
      return; // Skip regular processing for shortcut nodes
    }
    
    // ===== DEFENSIVE BOUNDS CHECKING FOR REGULAR NODES =====
    // Validate node index and way position before any vector access
    auto const node_idx = to_idx(n.n_);
    if (node_idx >= w.node_ways_.size() || 
        node_idx >= w.node_in_way_idx_.size()) {
      return; // Skip invalid nodes
    }
    
    // Validate way position for this node
    auto const& node_ways = w.node_ways_[n.n_];
    if (n.way_ >= node_ways.size()) {
      return; // Skip nodes with invalid way positions
    }
    
    // Additional safety check for node_in_way_idx access
    auto const& node_in_way_idx = w.node_in_way_idx_[n.n_];
    if (n.way_ >= node_in_way_idx.size()) {
      return; // Skip nodes with mismatched way index arrays
    }
    
    auto way_pos = way_pos_t{0U};
    for (auto const [way, i] :
         utl::zip_unchecked(w.node_ways_[n.n_], w.node_in_way_idx_[n.n_])) {
      auto const expand = [&](direction const way_dir, std::uint16_t const from,
                              std::uint16_t const to) {
        // Bounds check for way_nodes access
        if (to_idx(way) >= w.way_nodes_.size() || to >= w.way_nodes_[way].size()) {
          return; // Skip invalid way node access
        }
        
        // NOLINTNEXTLINE(clang-analyzer-core.CallAndMessage)
        auto const target_node = w.way_nodes_[way][to];
        // ===== CH LEVEL FILTER =====
        // TEMPORARILY DISABLED: Level filtering for search space reduction
        // TODO: Re-enable once basic CH routing is working
        if (false && use_ch && w.contraction_hierarchy_enabled_ && !w.node_ch_level_.empty()) {
          // Bounds checking to prevent crashes
          auto const curr_idx = to_idx(n.n_);
          auto const target_idx = to_idx(target_node);
          
          if (curr_idx < w.node_ch_level_.size() && target_idx < w.node_ch_level_.size()) {
            auto const curr_level = w.node_ch_level_[n.n_];
            auto const target_level = w.node_ch_level_[target_node];
            
            // Apply level filtering for CH bidirectional search
            // Both forward and backward searches go "upward" to higher levels
            // This is the standard CH approach for bidirectional search
            if (curr_level >= target_level) {
              return; // Skip this edge - both directions only traverse to higher levels
            }
          }
        }
        // ==========================
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
        if (way_cost(target_way_prop, way_dir, 0U) == kInfeasible) { // dijsktra eigene adjacent funciton , wege 
          // in ways oder separat shortcut array (node -> shortcuts)
          // seaparater shortcut loop, restrictions schon in shortcut
          // restrcitions: welche ganz urspürnlichen Kanten wurden ersetzt... u-turns
          // Rekonstruktion vom Pfad: welche anderen shortcuts/Kanten ersetzt -> rekursiv (vielleicht auch bei reestricitons)

          // ODER komplett neuer Graph (ohne ways, richtigungen...)
          return;
        }

        if (w.is_restricted<SearchDir>(n.n_, n.way_, way_pos)) {
          return;
        }

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
      if (i != w.way_nodes_[way].size() - 1U) {
        expand(flip<SearchDir>(direction::kForward), i, i + 1);
      }

      ++way_pos;
    }
    
    // ===== CH SHORTCUT PROCESSING FOR REGULAR NODES =====
    // Regular nodes can also generate shortcuts to shortcut nodes
    if (use_ch && w.contraction_hierarchy_enabled_ && !w.shortcuts_.empty()) {
      auto const current_node = n.n_;
      
      // Safe hash map lookup
      auto const shortcut_it = w.outgoing_shortcuts_.find(current_node);
      if (shortcut_it != w.outgoing_shortcuts_.end()) {
        auto const& shortcut_indices = shortcut_it->second;
        
        for (auto const shortcut_idx : shortcut_indices) {
          // Basic validation
          if (shortcut_idx >= w.shortcuts_.size()) {
            continue;
          }
          
          auto const& shortcut = w.shortcuts_[shortcut_idx];
          auto const target_node = shortcut.to;
          
          // Simple target validation
          if (target_node == node_idx_t::invalid() || 
              to_idx(target_node) >= w.node_properties_.size() ||
              to_idx(target_node) >= w.node_ways_.size()) {
            continue;
          }
          
          auto const target_node_prop = w.node_properties_[target_node];
          if (node_cost(target_node_prop) == kInfeasible) {
            continue;
          }
          
          // Check shortcut validity
          if (shortcut.cost >= kInfeasible) {
            continue;
          }
          
          // Validate that target node has ways
          if (w.node_ways_[target_node].empty()) {
            continue;
          }
          
          // Create shortcut target with special way position marker
          auto const target = node{target_node, kShortcutWayPos, SearchDir};
          fn(target, shortcut.cost, 0U, way_idx_t::invalid(), 0U, 0U, 
             elevation_storage::elevation{}, true); // Mark as shortcut
        }
      }
    }
  }

  static bool is_dest_reachable(ways::routing const& w,
                                node const n,
                                way_idx_t const way,
                                direction const way_dir,
                                direction const search_dir) {
    auto const target_way_prop = w.way_properties_[way];
    if (way_cost(target_way_prop, way_dir, 0U) == kInfeasible) {
      return false;
    }

    if (w.is_restricted(n.n_, n.way_, w.get_way_pos(n.n_, way), search_dir)) {
      return false;
    }

    return true;
  }

  static constexpr cost_t way_cost(way_properties const& e,
                                   direction const dir,
                                   std::uint16_t const dist) {
    if (e.is_car_accessible() &&
        (dir == direction::kForward || !e.is_oneway_car())) {
      return (dist / e.max_speed_m_per_s()) * (e.is_destination() ? 5U : 1U) +
             (e.is_destination() ? 120U : 0U);
    } else {
      return kInfeasible;
    }
  }

  static constexpr cost_t node_cost(node_properties const& n) {
    return n.is_car_accessible() ? 0U : kInfeasible;
  }

  static constexpr double heuristic(double const dist) {
    return dist / (130U / 3.6);
  }
  static constexpr node get_reverse(node const n) {
    return {n.n_, n.way_, opposite(n.dir_)};
  }
};

}  // namespace osr
