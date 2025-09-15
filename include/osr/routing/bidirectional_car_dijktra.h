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

struct bidirectional_car_dijkstra {
  using profile_t = car;
  using key = car::key;
  using label = car::label;
  using node = car::node;
  using entry = car::entry;
  using hash = car::hash;
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
    if (cost_map[l.get_node().get_key()].update(l, l.get_node(), l.cost(),
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
      auto const it = cost1_.find(n.get_key());
      return it != end(cost1_) ? it->second.cost(n) : kInfeasible;
    } else {
      auto const it = cost2_.find(n.get_key());
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
    auto const opposite_candidate = opposite_cost_map->find(curr.get_key());
    auto const curr_cost = get_cost<SearchDir>(curr);
    
    if (opposite_candidate != end(*opposite_cost_map)) {
      auto const other_cost = opposite_candidate->second.cost(curr);
      if (other_cost != kInfeasible) {
        evaluate_meetpoint(curr_cost, other_cost, curr, curr);
      } else {
        auto const pred_it = costs.find(curr.get_key());
        if (pred_it == end(costs)) {
          return;
        }
        auto const pred = pred_it->second.pred(curr);
        if (!pred.has_value()) {
          return;
        }
        auto const& adjacency_map = (opposite(SearchDir) == direction::kForward)
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

            if (neighbor.get_key() != pred->get_key()) {
              continue;
            }
            auto const opposite_it =
                opposite_cost_map->find(neighbor.get_key());
            if (opposite_it == end(*opposite_cost_map)) {
              continue;
            }
            auto const opposite_curr = opposite_it->second.pred(neighbor);
            if (!opposite_curr.has_value() ||
                opposite_curr->get_key() != curr.get_key()) {
              continue;
            }
            auto const opposite_curr_cost =
                opposite_candidate->second.cost(*opposite_curr);
            auto const pred_cost = get_cost<SearchDir>(*pred);
            auto const opposite_pred_cost =
                opposite_it->second.cost(neighbor);
            auto const evaluate_meetpoint_with_potential_u_turn_cost =
                [&](cost_t const cost_1, cost_t const cost_2,
                    node const meet_1, node const meet_2) {
                  evaluate_meetpoint(
                      cost_1, cost_2,
                      SearchDir == direction::kForward ? meet_1 : meet_2,
                      SearchDir == direction::kForward ? meet_2 : meet_1);
                };
            if (pred_cost + opposite_pred_cost >
                curr_cost + opposite_curr_cost) {
              evaluate_meetpoint_with_potential_u_turn_cost(
                  pred_cost, opposite_pred_cost, *pred, neighbor);
            } else {
              evaluate_meetpoint_with_potential_u_turn_cost(
                  curr_cost, opposite_curr_cost, curr, *opposite_curr);
            }
          }
        }
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
            costs[neighbor.get_key()].update(
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