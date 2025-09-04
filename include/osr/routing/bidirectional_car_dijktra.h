#pragma once

#include <limits>
#include <numeric>
#include <random>

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
  
  struct shortcut {
    node_idx_t target;    // destination node
    cost_t weight;        // shortcut cost
    node_idx_t middle;    // bypassed node (for reconstruction)
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
        car::adjacent<opposite(SearchDir), WithBlocked>(
            r, curr, blocked, sharing, elevations,
            [&](node const neighbor, std::uint32_t const, distance_t,
                way_idx_t const, std::uint16_t, std::uint16_t,
                elevation_storage::elevation const, bool const) {
              if (neighbor.get_key() != pred->get_key()) {
                return;
              }
              auto const opposite_it =
                  opposite_cost_map->find(neighbor.get_key());
              if (opposite_it == end(*opposite_cost_map)) {
                return;
              }
              auto const opposite_curr = opposite_it->second.pred(neighbor);
              if (!opposite_curr.has_value() ||
                  opposite_curr->get_key() != curr.get_key()) {
                return;
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
            });
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

    car::adjacent<SearchDir, WithBlocked>(
        r, curr, blocked, sharing, elevations,
        [&](node const neighbor, std::uint32_t const cost, distance_t,
            way_idx_t const way, std::uint16_t, std::uint16_t,
            elevation_storage::elevation const, bool const track) {
          if constexpr (kDebug) {
            std::cout << "  NEIGHBOR ";
            neighbor.print(std::cout, w);
          }
          
          // CH level filtering: only relax edges to higher-level nodes
          if (get_ch_level(neighbor.get_key()) <= get_ch_level(curr.get_key())) {
            if constexpr (kDebug) {
              std::cout << " -> FILTERED (level " << get_ch_level(neighbor.get_key()) 
                        << " <= " << get_ch_level(curr.get_key()) << ")\n";
            }
            return;
          }
          
          auto const total = curr_cost + cost;
          if (total >= max) {
            if (SearchDir == direction::kForward) {
              max_reached_1_ = true;
            } else {
              max_reached_2_ = true;
            }
            return;
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
        });
    
    // Process shortcuts from current node
    auto const shortcuts_it = shortcuts_.find(curr.get_key());
    if (shortcuts_it != shortcuts_.end()) {
      for (auto const& sc : shortcuts_it->second) {
        // Apply CH level filtering to shortcuts too
        if (get_ch_level(sc.target) <= get_ch_level(curr.get_key())) {
          if constexpr (kDebug) {
            std::cout << "  SHORTCUT to " << sc.target.v_ << " -> FILTERED (level " 
                      << get_ch_level(sc.target) << " <= " << get_ch_level(curr.get_key()) << ")\n";
          }
          continue;
        }
        
        auto const total = curr_cost + sc.weight;
        if (total >= max) {
          if (SearchDir == direction::kForward) {
            max_reached_1_ = true;
          } else {
            max_reached_2_ = true;
          }
          continue;
        }
        
        // Create a node for the shortcut target (use way=0, dir=forward as placeholder)
        auto const target_node = node{sc.target, 0U, direction::kForward};
        if (total < max && costs[target_node.get_key()].update(
                l, target_node, static_cast<cost_t>(total), curr)) {
          
          auto next = label{target_node, static_cast<cost_t>(total)};
          // No tracking for shortcuts (no real way/node info)
          pq.push(std::move(next));
          
          if constexpr (kDebug) {
            std::cout << "  SHORTCUT to " << sc.target.v_ << " cost " << sc.weight 
                      << " -> PUSHED\n";
          }
        } else {
          if constexpr (kDebug) {
            std::cout << "  SHORTCUT to " << sc.target.v_ << " -> DOMINATED\n";
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
  
  // CH level assignment
  ankerl::unordered_dense::map<node_idx_t, std::uint32_t, hash> ch_levels_;
  
  // CH shortcuts
  ankerl::unordered_dense::map<node_idx_t, std::vector<shortcut>, hash> shortcuts_;
  
  void assign_ch_levels(ways const& w) {
    auto const n_nodes = w.n_nodes();
    
    // Create a vector with levels from 1 to n
    std::vector<std::uint32_t> levels(n_nodes);
    std::iota(levels.begin(), levels.end(), 1U);
    
    // Shuffle to randomize levels
    std::random_device rd;
    std::mt19937 gen(rd());
    std::shuffle(levels.begin(), levels.end(), gen);
    
    // Assign levels to nodes
    ch_levels_.clear();
    for (node_idx_t::value_t i = 0; i < n_nodes; ++i) {
      ch_levels_[node_idx_t{i}] = levels[i];
    }
  }
  
  std::uint32_t get_ch_level(node_idx_t n) const {
    auto it = ch_levels_.find(n);
    return it != ch_levels_.end() ? it->second : 0U;
  }
  
  void add_shortcut(node_idx_t from, node_idx_t to, cost_t weight, node_idx_t middle) {
    shortcuts_[from].emplace_back(shortcut{to, weight, middle});
  }
  
  std::size_t get_shortcut_count(node_idx_t from) const {
    auto it = shortcuts_.find(from);
    return it != shortcuts_.end() ? it->second.size() : 0;
  }
  
  void clear_shortcuts() {
    shortcuts_.clear();
  }
};

}  // namespace osr