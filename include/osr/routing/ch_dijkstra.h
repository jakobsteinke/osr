#pragma once

#include <limits>
#include <optional>

#include "osr/routing/ch_data.h"
#include "osr/routing/dial.h"
#include "osr/routing/profiles/car.h"
#include "osr/types.h"
#include "osr/ways.h"

namespace osr {

struct ch_dijkstra {
  using profile_t = car;
  using key = car::key;
  using label = car::label;
  using node = car::node;
  using entry = car::entry;
  using hash = car::hash;
  using internal_cost_t = std::uint32_t;  // Use larger type for internal calculations

  static constexpr bool kDebug = false;

  // Custom label for CH that uses larger cost type
  struct ch_label {
    ch_label(node const n, internal_cost_t const c)
        : n_{n.n_}, way_{n.way_}, dir_{n.dir_}, cost_{c} {}

    constexpr node get_node() const noexcept { return {n_, way_, dir_}; }
    constexpr internal_cost_t cost() const noexcept { return cost_; }

    node_idx_t n_;
    way_pos_t way_;
    direction dir_;
    internal_cost_t cost_;
  };

  struct get_bucket {
    internal_cost_t operator()(ch_label const& l) { 
      // Mask off the backward bit for bucketing
      return l.cost() & kCostMask;
    }
  };

  // Custom entry that uses larger cost type for CH calculations
  struct ch_entry {
    static constexpr auto const kMaxWays = way_pos_t{16U};
    static constexpr auto const kN = kMaxWays * 2U /* FWD+BWD */;

    ch_entry() { 
      utl::fill(cost_, std::numeric_limits<internal_cost_t>::max()); 
      utl::fill(pred_, node_idx_t::invalid());
    }

    constexpr std::optional<node> pred(node const n) const noexcept {
      auto const idx = get_index(n);
      return pred_[idx] == node_idx_t::invalid()
                 ? std::nullopt
                 : std::optional{node{pred_[idx], pred_way_[idx],
                                      to_dir(pred_dir_[idx])}};
    }

    constexpr internal_cost_t cost(node const n) const noexcept {
      return cost_[get_index(n)];
    }

    constexpr bool update(label const&,
                          node const n,
                          internal_cost_t const c,
                          node const pred) noexcept {
      auto const idx = get_index(n);
      if (c < cost_[idx]) {
        cost_[idx] = c;
        pred_[idx] = pred.n_;
        pred_way_[idx] = pred.way_;
        pred_dir_[idx] = to_bool(pred.dir_);
        return true;
      }
      return false;
    }

    static constexpr std::size_t get_index(node const n) {
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
    std::array<internal_cost_t, kN> cost_;
  };

  void reset(cost_t const max) {
    pq_.clear();
    // Allocate more buckets for CH - estimate larger costs from longer paths
    auto const ch_max = static_cast<internal_cost_t>(max) * 20U; // Allow 20x larger costs
    pq_.n_buckets(ch_max + 1U);
    forward_costs_.clear();
    backward_costs_.clear();
    initial_forward_nodes_.clear();
    initial_backward_nodes_.clear();
    best_cost_ = std::numeric_limits<internal_cost_t>::max();
    meet_point_ = node_idx_t::invalid();
    forward_meet_node_ = node::invalid();
    backward_meet_node_ = node::invalid();
  }

  void add_start(ways const& w, label const l, ch_data const* ch) {
    initial_forward_nodes_.insert(l.get_node().get_key());
    auto const internal_cost = static_cast<internal_cost_t>(l.cost());
    if (forward_costs_[l.get_node().get_key()].update(
            l, l.get_node(), internal_cost, node::invalid())) {
      pq_.push(ch_label{l.get_node(), internal_cost});
      if (kDebug) {
        std::cout << "CH START FWD ";
        l.get_node().print(std::cout, w);
        std::cout << " cost=" << internal_cost << "\n";
      }
    }
  }

  void add_end(ways const& w, label const l, ch_data const* ch) {
    initial_backward_nodes_.insert(l.get_node().get_key());
    auto const internal_cost = static_cast<internal_cost_t>(l.cost());
    if (backward_costs_[l.get_node().get_key()].update(
            l, l.get_node(), internal_cost, node::invalid())) {
      pq_.push(ch_label{l.get_node(), internal_cost | kBackwardBit});
      if (kDebug) {
        std::cout << "CH START BWD ";
        l.get_node().print(std::cout, w);
        std::cout << " cost=" << internal_cost << "\n";
      }
    }
  }

  template <bool WithBlocked>
  bool run(ways const& w,
           ways::routing const& r,
           cost_t const max,
           bitvec<node_idx_t> const* blocked,
           ch_data const* ch,
           sharing_data const* sharing,
           elevation_storage const* elevations) {
    
    while (!pq_.empty()) {
      auto const l = pq_.pop();
      auto const is_forward = (l.cost() & kBackwardBit) == 0;
      auto const actual_cost = l.cost() & kCostMask;
      auto const curr = l.get_node();

      if (kDebug) {
        std::cout << "EXTRACT " << (is_forward ? "FWD" : "BWD") << " ";
        curr.print(std::cout, w);
        std::cout << " cost=" << actual_cost << "\n";
      }

      auto& curr_costs = is_forward ? forward_costs_ : backward_costs_;
      auto& other_costs = is_forward ? backward_costs_ : forward_costs_;
      
      if (curr_costs[curr.get_key()].cost(curr) < actual_cost) {
        continue;
      }

      check_meetpoint(curr, actual_cost, other_costs);

      if (best_cost_ != std::numeric_limits<internal_cost_t>::max() && 
          pq_.empty() == false && 
          (pq_.buckets_[pq_.get_next_bucket()].back().cost() & kCostMask) > best_cost_) {
        if (kDebug) {
          std::cout << "Terminating: both searches > best_cost " << best_cost_ << "\n";
        }
        return true;
      }

      if (is_forward) {
        expand_forward<WithBlocked>(w, r, curr, actual_cost, max, blocked, ch, 
                                    sharing, elevations);
      } else {
        expand_backward<WithBlocked>(w, r, curr, actual_cost, max, blocked, ch,
                                     sharing, elevations);
      }
    }

    return best_cost_ != std::numeric_limits<internal_cost_t>::max();
  }

  template <bool WithBlocked>
  void expand_forward(ways const& w,
                      ways::routing const& r,
                      node const curr,
                      internal_cost_t curr_cost,
                      cost_t max,
                      bitvec<node_idx_t> const* blocked,
                      ch_data const* ch,
                      sharing_data const* sharing,
                      elevation_storage const* elevations) {
    
    car::template adjacent<direction::kForward, WithBlocked>(
        r, curr, blocked, sharing, elevations,
        [&](node const neighbor, std::uint32_t const cost, distance_t,
            way_idx_t const way, std::uint16_t, std::uint16_t,
            elevation_storage::elevation const, bool const) {
          
          // Temporarily disable level filtering to test basic functionality
          // if (ch && !ch->is_upward_edge(curr.n_, neighbor.n_)) {
          //   return;
          // }
          
          // Disable regular edge debug output
          // if (cost > 100) {  // Only debug expensive edges
          //   std::cout << "  Regular edge curr=" << curr_cost << " + edge=" << cost << " = total=" << (curr_cost + cost) << std::endl;
          // }
          process_edge(w, curr, neighbor, static_cast<internal_cost_t>(curr_cost + cost), max, true);
        });

    // Temporarily disable shortcuts to test basic bidirectional search
    // if (ch) {
    //   auto const* shortcuts = ch->get_forward_shortcuts(curr.n_);
    //   if (shortcuts) {
    //     for (auto const& sc : *shortcuts) {
    //       if (ch->is_upward_edge(curr.n_, sc.to_)) {
    //         if (sc.cost_ < 50) {  // Only debug moderately cheap shortcuts
    //           std::cout << "  Shortcut cost=" << sc.cost_ << " total=" << (curr_cost + sc.cost_) << std::endl;
    //         }
    //         car::resolve_all(r, sc.to_, level_t{static_cast<std::uint8_t>(0U)}, [&](node const target) {
    //           process_edge(w, curr, target, static_cast<internal_cost_t>(curr_cost + sc.cost_), max, true);
    //         });
    //       }
    //     }
    //   }
    // }
  }

  template <bool WithBlocked>
  void expand_backward(ways const& w,
                       ways::routing const& r,
                       node const curr,
                       internal_cost_t curr_cost,
                       cost_t max,
                       bitvec<node_idx_t> const* blocked,
                       ch_data const* ch,
                       sharing_data const* sharing,
                       elevation_storage const* elevations) {
    
    car::template adjacent<direction::kBackward, WithBlocked>(
        r, curr, blocked, sharing, elevations,
        [&](node const neighbor, std::uint32_t const cost, distance_t,
            way_idx_t const way, std::uint16_t, std::uint16_t,
            elevation_storage::elevation const, bool const) {
          
          // Temporarily disable level filtering to test basic functionality
          // if (ch && !ch->is_upward_edge(curr.n_, neighbor.n_)) {
          //   return;
          // }
          
          process_edge(w, curr, neighbor, static_cast<internal_cost_t>(curr_cost + cost), max, false);
        });

    // Temporarily disable shortcuts to test basic bidirectional search  
    // if (ch) {
    //   auto const* shortcuts = ch->get_backward_shortcuts(curr.n_);
    //   if (shortcuts) {
    //     for (auto const& sc : *shortcuts) {
    //       if (ch->is_upward_edge(curr.n_, sc.from_)) {
    //         car::resolve_all(r, sc.from_, level_t{static_cast<std::uint8_t>(0U)}, [&](node const target) {
    //           process_edge(w, curr, target, static_cast<internal_cost_t>(curr_cost + sc.cost_), max, false);
    //         });
    //       }
    //     }
    //   }
    // }
  }

  void process_edge(ways const& w,
                   node const curr,
                   node const neighbor,
                   internal_cost_t new_cost,
                   cost_t max,
                   bool is_forward) {
    if (new_cost >= static_cast<internal_cost_t>(max)) {
      return;
    }

    auto& costs = is_forward ? forward_costs_ : backward_costs_;
    
    if (costs[neighbor.get_key()].update(label{neighbor, static_cast<cost_t>(std::min(new_cost, static_cast<internal_cost_t>(std::numeric_limits<cost_t>::max())))}, 
                                         neighbor, new_cost, curr)) {
      auto next = ch_label{neighbor, new_cost | (is_forward ? 0U : kBackwardBit)};
      pq_.push(std::move(next));
      
      if (kDebug) {
        std::cout << "  PUSH " << (is_forward ? "FWD" : "BWD") << " ";
        neighbor.print(std::cout, w);
        std::cout << " cost=" << new_cost << "\n";
      }
    }
  }

  void check_meetpoint(node const curr,
                      internal_cost_t curr_cost,
                      ankerl::unordered_dense::map<key, ch_entry, hash> const& other_costs) {
    
    auto const it = other_costs.find(curr.get_key());
    if (it == other_costs.end()) {
      return;
    }

    // Check the exact current node representation for meetpoint
    auto const other_cost = it->second.cost(curr);
    
    if (other_cost != std::numeric_limits<internal_cost_t>::max()) {
      auto const total_cost = curr_cost + other_cost;
      
      // Reject zero-cost meetpoints from initial nodes (they haven't been properly explored)
      bool is_valid_meetpoint = true;
      if (total_cost == 0 && 
          (initial_forward_nodes_.contains(curr.get_key()) || 
           initial_backward_nodes_.contains(curr.get_key()))) {
        if (kDebug) {
          std::cout << "  REJECTED zero-cost initial node meetpoint at " << curr.n_ 
                   << " cost=" << total_cost << "\n";
        }
        is_valid_meetpoint = false;
      }
      
      if (is_valid_meetpoint && total_cost < best_cost_) {
        best_cost_ = total_cost;
        meet_point_ = curr.n_;
        
        if (&other_costs == &backward_costs_) {
          forward_meet_node_ = curr;
          backward_meet_node_ = curr;
        } else {
          forward_meet_node_ = curr;
          backward_meet_node_ = curr;
        }
        
        if (kDebug) {
          std::cout << "DEBUG: New best meetpoint at node " << curr.n_ 
                   << " cost=" << best_cost_ 
                   << " (curr_cost=" << curr_cost 
                   << " + other_cost=" << other_cost << ")\n";
        }
      }
    }
  }

  internal_cost_t get_cost(node const n, bool is_forward) const {
    auto const& costs = is_forward ? forward_costs_ : backward_costs_;
    auto const it = costs.find(n.get_key());
    return it != costs.end() ? it->second.cost(n) : std::numeric_limits<internal_cost_t>::max();
  }

  static constexpr std::uint32_t kBackwardBit = std::uint32_t{1U} << 31;
  static constexpr std::uint32_t kCostMask = ~kBackwardBit;

  dial<ch_label, get_bucket> pq_{get_bucket{}};
  ankerl::unordered_dense::map<key, ch_entry, hash> forward_costs_;
  ankerl::unordered_dense::map<key, ch_entry, hash> backward_costs_;
  ankerl::unordered_dense::set<key> initial_forward_nodes_;
  ankerl::unordered_dense::set<key> initial_backward_nodes_;
  internal_cost_t best_cost_{std::numeric_limits<internal_cost_t>::max()};
  node_idx_t meet_point_{node_idx_t::invalid()};
  node forward_meet_node_{node::invalid()};
  node backward_meet_node_{node::invalid()};
};

}  // namespace osr