#pragma once

#include <limits>

#include "utl/verify.h"

#include "osr/elevation_storage.h"
#include "osr/location.h"
#include "osr/routing/dial.h"
#include "osr/routing/profiles/car.h"
#include "osr/routing/sharing_data.h"
#include "osr/types.h"
#include "osr/ways.h"

namespace osr {

struct sharing_data;

struct car_successor {
  car::node target;
  std::uint32_t cost;
  distance_t distance;
  way_idx_t way;
  std::uint16_t from;
  std::uint16_t to;
  elevation_storage::elevation elevation;
  bool track;
};

struct car_node_hash {
  using is_avalanching = void;
  auto operator()(car::node const& n) const noexcept -> std::uint64_t {
    using namespace ankerl::unordered_dense::detail;
    auto const combined = (static_cast<std::uint64_t>(to_idx(n.n_)) << 32) |
                          (static_cast<std::uint64_t>(n.way_) << 8) |
                          static_cast<std::uint64_t>(n.dir_);
    return wyhash::hash(combined);
  }
};

using car_adjacency_map = ankerl::unordered_dense::map<car::node, std::vector<car_successor>, car_node_hash>;
inline car_adjacency_map legal_successor;
inline car_adjacency_map legal_predecessor;

struct car_node_entry {
  cost_t cost_ = kInfeasible;
  car::node pred_ = car::node::invalid();

  constexpr cost_t cost(car::node const&) const noexcept {
    return cost_;
  }

  constexpr std::optional<car::node> pred(car::node const&) const noexcept {
    return pred_.n_ == node_idx_t::invalid() ? std::nullopt : std::optional{pred_};
  }

  constexpr bool update(car::label const&, car::node const&, cost_t const c, car::node const pred) noexcept {
    if (c < cost_) {
      cost_ = c;
      pred_ = pred;
      return true;
    }
    return false;
  }

  void write(car::node, path&) const {}
};

struct bidirectional_car_dijkstra {
  using profile_t = car;
  using key = car::node;
  using label = car::label;
  using node = car::node;
  using entry = car_node_entry;
  using hash = car_node_hash;
  using cost_map = ankerl::unordered_dense::map<key, entry, hash>;

  constexpr static auto const kDebug = false;

  struct get_bucket {
    cost_t operator()(label const& l) { return l.cost(); }
  };

  void clear_mp() {
    meet_point_1_ = node::invalid();
    meet_point_2_ = node::invalid();
    best_cost_ = kInfeasible;
  }

  void reset(cost_t const max,
             location const& start_loc,
             location const& end_loc) {
    pq1_.clear();
    pq2_.clear();
    pq1_.n_buckets(max + 1U);
    pq2_.n_buckets(max + 1U);
    cost1_.clear();
    cost2_.clear();
    clear_mp();
    start_loc_ = start_loc;
    end_loc_ = end_loc;
    max_reached_1_ = false;
    max_reached_2_ = false;
  }

  void add(ways const& w,
           label const l,
           direction const dir,
           cost_map& cost_map,
           dial<label, get_bucket>& d,
           sharing_data const*) {
    if (cost_map[l.get_node()].update(l, l.get_node(), l.cost(),
                                      node::invalid())) {
      d.push(l);
    }
  }

  void add_start(ways const& w, label const l, sharing_data const* sharing) {
    if (kDebug) {
      l.get_node().print(std::cout, w);
      std::cout << "starting" << l.get_node().n_ << std::endl;
    }
    add(w, l, direction::kForward, cost1_, pq1_, sharing);
  }

  void add_end(ways const& w, label const l, sharing_data const* sharing) {
    if (kDebug) {
      l.get_node().print(std::cout, w);
      std::cout << "ending" << l.get_node().n_ << std::endl;
    }
    add(w, l, direction::kBackward, cost2_, pq2_, sharing);
  }

  template <direction SearchDir>
  cost_t get_cost(node const n) const {
    if (SearchDir == direction::kForward) {
      auto const it = cost1_.find(n);
      return it != end(cost1_) ? it->second.cost(n) : kInfeasible;
    } else {
      auto const it = cost2_.find(n);
      return it != end(cost2_) ? it->second.cost(n) : kInfeasible;
    }
  }

  cost_t get_cost_to_mp(node const n1, node const n2) const {
    auto const f_cost = get_cost<direction::kForward>(n1);
    auto const b_cost = get_cost<direction::kBackward>(n2);
    if (f_cost == kInfeasible || b_cost == kInfeasible) {
      return kInfeasible;
    }
    return f_cost + b_cost;
  }

  template <direction SearchDir, bool WithBlocked>
  void handle_end_of_way_meetpoint(ways const& w,
                                   ways::routing const& r,
                                   node const curr,
                                   cost_map& costs,
                                   bitvec<node_idx_t> const* blocked,
                                   sharing_data const* sharing,
                                   elevation_storage const* elevations) {
    auto const evaluate_meetpoint = [&](cost_t cost, cost_t other_cost,
                                        node meetpoint1, node meetpoint2) {
      if constexpr (kDebug) {
        std::cout << "  potential MEETPOINT found by ";
        meetpoint1.print(std::cout, w);
      }
      auto const tentative = cost + other_cost;
      if (tentative < best_cost_) {
        meet_point_1_ = meetpoint1;
        meet_point_2_ = meetpoint2;
        best_cost_ = static_cast<cost_t>(tentative);

        if constexpr (kDebug) {
          std::cout << " with cost " << best_cost_ << " -> ACCEPTED\n";
        }
      } else if constexpr (kDebug) {
        std::cout << " -> DOMINATED\n";
      }
    };

    auto const opposite_cost_map =
        opposite(SearchDir) == direction::kForward ? &cost1_ : &cost2_;
    auto const curr_cost = get_cost<SearchDir>(curr);

    // Find all opposite car::nodes at the same physical node_idx_t
    for (auto const& [opposite_node, opposite_entry] : *opposite_cost_map) {
      if (opposite_node.n_ != curr.n_) continue;  // Different physical node

      auto const opposite_cost = opposite_entry.cost(opposite_node);
      if (opposite_cost == kInfeasible) continue;

      // For identical car::nodes, no turn restriction check needed
      if (curr.n_ == opposite_node.n_ && curr.way_ == opposite_node.way_ && curr.dir_ == opposite_node.dir_) {
        evaluate_meetpoint(curr_cost, opposite_cost, curr, opposite_node);
        continue;
      }

      // For different way/direction combinations, check turn restrictions
      // When forward search settles curr and backward search settled opposite_node,
      // we need to check if we can legally transition from curr's state to opposite_node's state
      auto const can_connect = [&]() {
        if (SearchDir == direction::kForward) {
          // Forward search settled curr, backward search settled opposite_node
          // Check if we can go from curr.way_ to opposite_node.way_
          return !r.is_restricted<direction::kForward>(curr.n_, curr.way_, opposite_node.way_);
        } else {
          // Backward search settled curr, forward search settled opposite_node
          // Check if we can go from opposite_node.way_ to curr.way_
          return !r.is_restricted<direction::kForward>(curr.n_, opposite_node.way_, curr.way_);
        }
      }();

      if (!can_connect) continue;

      // Calculate meetpoint cost including any U-turn penalty
      auto const is_u_turn = (curr.way_ == opposite_node.way_) && (curr.dir_ != opposite_node.dir_);
      auto const u_turn_cost = is_u_turn ? car::kUturnPenalty : 0U;

      if (SearchDir == direction::kForward) {
        evaluate_meetpoint(curr_cost, opposite_cost + u_turn_cost, curr, opposite_node);
      } else {
        evaluate_meetpoint(curr_cost, opposite_cost + u_turn_cost, opposite_node, curr);
      }
    }
  }

  template <direction SearchDir, bool WithBlocked>
  bool run_single(ways const& w,
                  ways::routing const& r,
                  cost_t const max,
                  bitvec<node_idx_t> const* blocked,
                  sharing_data const* sharing,
                  elevation_storage const* elevations,
                  dial<label, get_bucket>& pq,
                  cost_map& costs) {
    if (pq.empty()) return true;

    auto const l = pq.pop();
    auto const curr = l.get_node();
    auto const curr_cost = get_cost<SearchDir>(curr);
    
    if (curr_cost < l.cost()) {
      return true;
    }
    
    if constexpr (kDebug) {
      std::cout << "EXTRACT ";
      l.get_node().print(std::cout, w);
      std::cout << "\n";
    }

    auto const& adjacency_map = (SearchDir == direction::kForward)
                                 ? legal_successor : legal_predecessor;
    auto it = adjacency_map.find(curr);
    if (it != adjacency_map.end()) {
      for (auto const& successor : it->second) {
        if constexpr (WithBlocked) {
          if (blocked && blocked->test(successor.target.n_)) {
            continue;
          }
        }
        auto const neighbor = successor.target;
        auto const cost = successor.cost;
        auto const way = successor.way;
        auto const track = successor.track;

        if constexpr (kDebug) {
          std::cout << "  NEIGHBOR ";
          neighbor.print(std::cout, w);
        }
        auto const total = curr_cost + cost;
        if (total >= max) {
          if (SearchDir == direction::kForward) {
            max_reached_1_ = true;
          } else {
            max_reached_2_ = true;
          }
          continue;
        }
        if (total < max &&
            costs[neighbor].update(
                l, neighbor, static_cast<cost_t>(total), curr)) {

          auto next = label{neighbor, static_cast<cost_t>(total)};
          next.track(l, r, way, neighbor.get_node(), track);
          pq.push(std::move(next));

          // Meetpoint checking is done after settling nodes

          if constexpr (kDebug) {
            std::cout << " -> PUSH\n";
          }
        } else {
          if constexpr (kDebug) {
            std::cout << " -> DOMINATED\n";
          }
        }
      }
    }

    handle_end_of_way_meetpoint<SearchDir, WithBlocked>(w, r, curr, costs, blocked, sharing, elevations);

    // μ-termination: only check after finding first meetpoint
    if (best_cost_ != kInfeasible && !pq1_.empty() && !pq2_.empty()) {
      auto const min_f = pq1_.buckets_[pq1_.get_next_bucket()].back().cost();
      auto const min_r = pq2_.buckets_[pq2_.get_next_bucket()].back().cost();
      // Terminate only if BOTH searches have costs exceeding the best meetpoint cost
      if (min_f > best_cost_ && min_r > best_cost_) {
        if (kDebug) {
          std::cout << "μ-termination: both searches exceeded best cost " 
                    << min_f << " " << min_r << " > " << best_cost_ << std::endl;
        }
        return false;
      }
    }

    return true;
  }

  template <direction SearchDir, bool WithBlocked>
  bool run(ways const& w,
           ways::routing const& r,
           cost_t const max,
           bitvec<node_idx_t> const* blocked,
           sharing_data const* sharing,
           elevation_storage const* elevations) {
    while (!pq1_.empty() || !pq2_.empty()) {
      if (!pq1_.empty() &&
          !run_single<SearchDir, WithBlocked>(w, r, max, blocked, sharing,
                                              elevations, pq1_, cost1_)) {
        break;
      }
      if (!pq2_.empty() &&
          !run_single<opposite(SearchDir), WithBlocked>(
              w, r, max, blocked, sharing, elevations, pq2_, cost2_)) {
        break;
      }
    }
    
    if (best_cost_ != kInfeasible && best_cost_ > max) {
      clear_mp();
      return false;
    }
    return !max_reached_1_ || !max_reached_2_;
  }

  bool run(ways const& w,
           ways::routing const& r,
           cost_t const max,
           bitvec<node_idx_t> const* blocked,
           sharing_data const* sharing,
           elevation_storage const* elevations,
           direction const dir) {
    if (blocked == nullptr) {
      return dir == direction::kForward
                 ? run<direction::kForward, false>(w, r, max, blocked, sharing,
                                                   elevations)
                 : run<direction::kBackward, false>(w, r, max, blocked, sharing,
                                                    elevations);
    } else {
      return dir == direction::kForward
                 ? run<direction::kForward, true>(w, r, max, blocked, sharing,
                                                  elevations)
                 : run<direction::kBackward, true>(w, r, max, blocked, sharing,
                                                   elevations);
    }
  }

  dial<label, get_bucket> pq1_{get_bucket{}};
  dial<label, get_bucket> pq2_{get_bucket{}};
  location start_loc_;
  location end_loc_;
  node meet_point_1_;
  node meet_point_2_;
  cost_t best_cost_;
  cost_map cost1_;
  cost_map cost2_;
  bool max_reached_1_;
  bool max_reached_2_;
};

inline void preprocess_car_adjacency(ways const& w,
                              ways::routing const& r,
                              bitvec<node_idx_t> const* blocked = nullptr,
                              sharing_data const* sharing = nullptr,
                              elevation_storage const* elevations = nullptr) {
  legal_successor.clear();
  legal_predecessor.clear();

  for (node_idx_t n{0}; n < w.n_nodes(); ++n) {
    car::resolve_all(r, n, level_t{}, [&](car::node state) {

      car::adjacent<direction::kForward, false>(
        r, state, blocked, sharing, elevations,
        [&](car::node target, std::uint32_t cost, distance_t dist,
            way_idx_t way, std::uint16_t from, std::uint16_t to,
            elevation_storage::elevation elev, bool track) {
          legal_successor[state].emplace_back(
            car_successor{target, cost, dist, way, from, to, elev, track});
        });

      car::adjacent<direction::kBackward, false>(
        r, state, blocked, sharing, elevations,
        [&](car::node target, std::uint32_t cost, distance_t dist,
            way_idx_t way, std::uint16_t from, std::uint16_t to,
            elevation_storage::elevation elev, bool track) {
          legal_predecessor[state].emplace_back(
            car_successor{target, cost, dist, way, from, to, elev, track});
        });
    });
  }
}

}  // namespace osr