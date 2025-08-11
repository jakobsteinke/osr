#pragma once

#include <limits>
#include <algorithm>
#include "utl/verify.h"
#include "geo/constants.h"
#include "geo/latlng.h"
#include "osr/elevation_storage.h"
#include "osr/location.h"
#include "osr/routing/additional_edge.h"
#include "osr/routing/dial.h"
#include "osr/routing/sharing_data.h"
#include "osr/types.h"
#include "osr/ways.h"

namespace osr {

struct sharing_data;

// Classic bidirectional Dijkstra (no heuristic!)
template <typename Profile>
struct bidirectional_dijkstra {
  using profile_t = Profile;
  using key = typename Profile::key;
  using label = typename Profile::label;
  using node = typename Profile::node;
  using entry = typename Profile::entry;
  using hash = typename Profile::hash;
  using cost_map = typename ankerl::unordered_dense::map<key, entry, hash>;
  bool use_ch_ = false;
  void set_use_ch(bool b) { use_ch_ = b; }
  bool use_ch() const { return use_ch_; }   // <-- add this

  constexpr static auto const kDebug = false;
  constexpr static auto const kLongestNodeDistance = cost_t{300};

  struct get_bucket {
    cost_t operator()(label const& l) { return l.cost(); }
  };

  void clear_mp() {
    meet_point_1_ = meet_point_1_.invalid();
    meet_point_2_ = meet_point_2_.invalid();
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
    radius_ = kLongestNodeDistance;
    max_reached_1_ = false; 
    max_reached_2_ = false; 
  }

  void add(ways const&,
           label const l,
           direction const,
           cost_map& cost_map,
           dial<label, get_bucket>& d,
           sharing_data const*) {
    if (l.cost() < d.n_buckets() - 1 &&
        cost_map[l.get_node().get_key()].update(l, l.get_node(), l.cost(),
                                                node::invalid())) {
      d.push(l);
    }
  }

  void add_start(ways const& w, label const l, sharing_data const* sharing) {
    add(w, l, direction::kForward, cost1_, pq1_, sharing);
  }
  void add_end(ways const& w, label const l, sharing_data const* sharing) {
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

  // --------- MEETING-POINT LOGIC (inspired by bidirectional A*!) ---------
  template <direction SearchDir, bool WithBlocked>
  bool run_single(ways const& w,
                  ways::routing const& r,
                  cost_t const max,
                  bitvec<node_idx_t> const* blocked,
                  sharing_data const* sharing,
                  elevation_storage const* elevations,
                  dial<label, get_bucket>& pq,
                  cost_map& costs) {
    auto const adjusted_max = max; //(max + radius_) / 2U;

    auto const l = pq.pop();
    auto const curr = l.get_node();
    auto const curr_cost = get_cost<SearchDir>(curr);

    if constexpr (kDebug) {
      std::cout << "EXTRACT ";
      l.get_node().print(std::cout, w);
      std::cout << "\n";
    }

    Profile::template adjacent<SearchDir, WithBlocked>(
        r, curr, blocked, sharing, elevations, 
        [&](node const neighbor, std::uint32_t const cost, distance_t,
            way_idx_t const way, std::uint16_t, std::uint16_t,
            elevation_storage::elevation const, bool const track) {
          if constexpr (kDebug) {
            std::cout << "  NEIGHBOR ";
            neighbor.print(std::cout, w);
          }
          auto const total = curr_cost + cost;
          /////// no heuristic
          if (total >= adjusted_max) {
            if (SearchDir == direction::kForward) {
              max_reached_1_ = true;
            } else {
              max_reached_2_ = true;
            }
            return;
          }
          if (total < max &&  ///// no heur
              costs[neighbor.get_key()].update(
                  l, neighbor, static_cast<cost_t>(total), curr)) {

            auto next = label{neighbor, static_cast<cost_t>(total)};
            next.track(l, r, way, neighbor.get_node(), track);
            pq.push(std::move(next));

            if constexpr (kDebug) {
              std::cout << " -> PUSH\n";
            }
          } else {
            if constexpr (kDebug) {
              std::cout << " -> DOMINATED\n";
            }
          }
        },
        false // use_ch_
      );

    // --- Meeting point handling (same as A*, just no heuristic!) ---
    auto const evaluate_meetpoint = [&](cost_t cost, cost_t other_cost,
                                        node meetpoint1, node meetpoint2) {
      if constexpr (kDebug) {
        std::cout << "  potential MEETPOINT found ";
        meetpoint1.print(std::cout, w);
      }
      auto const tentative = cost + other_cost;
      if (tentative < best_cost_) {
        meet_point_1_ = meetpoint1;
        meet_point_2_ = meetpoint2;  ///////
        //assert(tentative == get_cost_to_mp(meet_point_1_, meet_point_2_));
        best_cost_ = static_cast<cost_t>(tentative);
        if constexpr (kDebug) {
          std::cout << " with cost " << best_cost_ << " -> ACCEPTED\n";
        }
      } else if constexpr (kDebug) {
        std::cout << " -> DOMINATED\n";
      }
    };

    auto const handle_end_of_way_meetpoint = [&]() {
      auto const opposite_cost_map =
          opposite(SearchDir) == direction::kForward ? &cost1_ : &cost2_;
      auto const opposite_candidate = opposite_cost_map->find(curr.get_key());
      if (opposite_candidate != end(*opposite_cost_map)) {
        auto const other_cost = opposite_candidate->second.cost(curr);
        if (other_cost != kInfeasible) {
          evaluate_meetpoint(curr_cost, other_cost, curr, curr);
        } else {
          auto const pred_it = costs.find(curr.get_key());
          if (pred_it == end(costs)) return;
          auto const pred = pred_it->second.pred(curr);
          if (!pred.has_value()) return;
          Profile::template adjacent<opposite(SearchDir), WithBlocked>(
              r, curr, blocked, sharing, elevations,
              [&](node const neighbor, std::uint32_t const, distance_t,
                  way_idx_t const, std::uint16_t, std::uint16_t,
                  elevation_storage::elevation const, bool const) {
                if (neighbor.get_key() != pred->get_key()) return;
                auto const opposite_it = opposite_cost_map->find(neighbor.get_key());
                if (opposite_it == end(*opposite_cost_map)) return;
                auto const opposite_curr = opposite_it->second.pred(neighbor);
                if (!opposite_curr.has_value() ||
                    opposite_curr->get_key() != curr.get_key()) {
                  return;
                }
                auto const opposite_curr_cost = opposite_candidate->second.cost(*opposite_curr);
                auto const pred_cost = get_cost<SearchDir>(*pred);
                auto const opposite_pred_cost = opposite_it->second.cost(neighbor);
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
              },
              false // use_ch_
            );
        }
      }
    };

    handle_end_of_way_meetpoint();

    // --- Early stopping rule ---
    if (best_cost_ != kInfeasible) {
      auto const top_f =
          pq1_.empty() ? get_cost<direction::kForward>(meet_point_1_)
                       : pq1_.buckets_[pq1_.get_next_bucket()].back().cost();
      auto const top_r =
          pq2_.empty() ? get_cost<direction::kBackward>(meet_point_2_)
                       : pq2_.buckets_[pq2_.get_next_bucket()].back().cost();
      if (top_f >= best_cost_ && top_r >= best_cost_) {  ////// mind radius       top_f >= best_cost_ && top_r >= best_cost_    top_f + top_r >= best_cost_ + radius_
        if (kDebug) {
          std::cout << "stopping criterion met " << top_f << " " << top_r << " "
                    << best_cost_ << " " << radius_ << std::endl;
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
    if (radius_ == max) {
      return false;
    }
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
  ankerl::unordered_dense::map<key, entry, hash> cost1_;
  ankerl::unordered_dense::map<key, entry, hash> cost2_;
  cost_t radius_;
  bool max_reached_1_;
  bool max_reached_2_;
};

}  // namespace osr
