// [UNCHANGED FROM YOUR LAST MESSAGE EXCEPT: no track() during relax]
// file: include/osr/routing/bidirectional_car_dijktra.h
#pragma once

#include <limits>
#include <numeric>
#include <random>
#include <unordered_map>
#include <queue>
#include <algorithm>

#include "utl/verify.h"

#include "fmt/core.h"

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
  static constexpr auto const kUturnPenalty = cost_t{120U};

  struct get_bucket {
    cost_t operator()(label const& l) { return l.cost(); }
  };

  struct shortcut_anchor {
    way_idx_t way;
    way_pos_t way_pos;
    direction dir;
    std::uint16_t from;
    std::uint16_t to;
    distance_t dist;
  };

  struct shortcut {
    node_idx_t target;
    cost_t     weight;
    node_idx_t middle;
    shortcut_anchor first;
    shortcut_anchor last;
    node_idx_t from;

    shortcut(node_idx_t t, cost_t w, node_idx_t m,
             shortcut_anchor first_a, shortcut_anchor last_a)
      : target(t), weight(w), middle(m),
        first(first_a), last(last_a),
        from(node_idx_t::invalid()) {}

    shortcut(node_idx_t f, node_idx_t t, cost_t w, node_idx_t m,
             shortcut_anchor first_a, shortcut_anchor last_a)
      : target(t), weight(w), middle(m),
        first(first_a), last(last_a),
        from(f) {}
  };

  struct edge_info {
    node_idx_t from;
    node_idx_t to;
    cost_t     cost;
    bool       is_shortcut;

    way_pos_t  first_way_pos;
    direction  first_dir;
    way_idx_t  first_way;
    std::uint16_t first_from, first_to;

    way_pos_t  last_way_pos;
    direction  last_dir;
    way_idx_t  last_way;
    std::uint16_t last_from, last_to;
  };

  template <direction SearchDir, bool WithBlocked, typename Fn>
  static void adjacent(ways::routing const& w,
                       node const n,
                       bitvec<node_idx_t> const* blocked,
                       sharing_data const*,
                       elevation_storage const*,
                       Fn&& fn) {
    // 1) Normal edges
    auto way_pos = way_pos_t{0U};
    for (auto const [way, i] :
         utl::zip_unchecked(w.node_ways_[n.n_], w.node_in_way_idx_[n.n_])) {
      auto const expand = [&](direction const way_dir, std::uint16_t const from,
                              std::uint16_t const to) {
        auto const target_node = w.way_nodes_[way][to];
        if constexpr (WithBlocked) {
          if (blocked->test(target_node)) {
            return;
          }
        }

        auto const target_node_prop = w.node_properties_[target_node];
        if (car::node_cost(target_node_prop) == kInfeasible) {
          return;
        }

        auto const target_way_prop = w.way_properties_[way];
        if (car::way_cost(target_way_prop, way_dir, 0U) == kInfeasible) {
          return;
        }

        if (w.is_restricted<SearchDir>(n.n_, n.way_, way_pos)) {
          return;
        }

        auto const is_u_turn = way_pos == n.way_ && way_dir == opposite(n.dir_);
        auto const dist = w.way_node_dist_[way][std::min(from, to)];
        auto const target =
            node{target_node, w.get_way_pos(target_node, way, to), way_dir};
        auto const cost = car::way_cost(target_way_prop, way_dir, dist) +
                          car::node_cost(target_node_prop) +
                          (is_u_turn ? kUturnPenalty : 0U);

        fn(target, cost, dist, way, from, to, elevation_storage::elevation{},
           /*track=*/false);
      };

      if (i != 0U) {
        expand(flip<SearchDir>(direction::kBackward), i, i - 1);
      }
      if (i != w.way_nodes_[way].size() - 1U) {
        expand(flip<SearchDir>(direction::kForward), i, i + 1);
      }

      ++way_pos;
    }

    // 2) CH Shortcuts
    if constexpr (SearchDir == direction::kForward) {
      if (auto it = shortcuts_.find(n.n_); it != shortcuts_.end()) {
        for (auto const& sc : it->second) {
          if constexpr (WithBlocked) {
            if (blocked->test(sc.target)) continue;
          }
          if (w.is_restricted<SearchDir>(n.n_, n.way_, sc.first.way_pos)) continue;

          bool const is_u_turn =
              (n.way_ == sc.first.way_pos) && (sc.first.dir == opposite(n.dir_));

          auto const neighbor = node{sc.target, sc.last.way_pos, sc.last.dir};
          auto const edge_cost =
              static_cast<std::uint32_t>(sc.weight + (is_u_turn ? kUturnPenalty : 0U));

          fn(neighbor, edge_cost,
             /*dist*/ sc.first.dist,
             /*way*/ way_idx_t::invalid(),
             /*from*/ 0, /*to*/ 0,
             elevation_storage::elevation{}, /*track=*/false);
        }
      }
    } else {
      if (auto it = shortcuts_by_target_.find(n.n_); it != shortcuts_by_target_.end()) {
        for (auto const& sc : it->second) {
          if constexpr (WithBlocked) {
            if (blocked->test(sc.from)) continue;
          }
          if (w.is_restricted<SearchDir>(n.n_, n.way_, sc.last.way_pos)) continue;

          bool const is_u_turn =
              (n.way_ == sc.last.way_pos) && (sc.last.dir == opposite(n.dir_));

          auto const neighbor = node{sc.from, sc.first.way_pos, sc.first.dir};
          auto const edge_cost =
              static_cast<std::uint32_t>(sc.weight + (is_u_turn ? kUturnPenalty : 0U));

          fn(neighbor, edge_cost,
             /*dist*/ sc.last.dist,
             /*way*/ way_idx_t::invalid(),
             /*from*/ 0, /*to*/ 0,
             elevation_storage::elevation{}, /*track=*/false);
        }
      }
    }
  }

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

  void add(ways const&,
           label const l,
           direction const,
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

        fmt::println("MEETPOINT FOUND: {} <-> {} with total cost {}",
                     meetpoint1.get_node().v_, meetpoint2.get_node().v_, best_cost_);

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
        adjacent<opposite(SearchDir), WithBlocked>(
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

    adjacent<SearchDir, WithBlocked>(
        r, curr, blocked, sharing, elevations,
        [&](node const neighbor, std::uint32_t const cost, distance_t,
            way_idx_t const /*way*/,
            std::uint16_t, std::uint16_t,
            elevation_storage::elevation const, bool const /*track_from_adj*/) {

          auto const neighbor_level = get_ch_level(neighbor.get_key());
          auto const curr_level = get_ch_level(curr.get_key());
          if (neighbor_level <= curr_level) {
            if constexpr (kDebug) {
              std::cout << " -> FILTERED (level " << neighbor_level
                        << " <= " << curr_level << ")\n";
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

            // DO NOT track() — costs only.
            pq.push(label{neighbor, static_cast<cost_t>(total)});

            if constexpr (kDebug) {
              std::cout << " -> PUSH\n";
            }
          } else {
            if constexpr (kDebug) {
              std::cout << " -> DOMINATED\n";
            }
          }
        });

    handle_end_of_way_meetpoint<SearchDir, WithBlocked>(w, r, curr, costs, blocked, sharing, elevations);

    if (best_cost_ != kInfeasible && !pq1_.empty() && !pq2_.empty()) {
      auto const min_f = pq1_.buckets_[pq1_.get_next_bucket()].back().cost();
      auto const min_r = pq2_.buckets_[pq2_.get_next_bucket()].back().cost();
      if (min_f > best_cost_ && min_r > best_cost_) {
        fmt::println("μ-TERMINATION: forward {} + backward {} > best {}",
                     min_f, min_r, best_cost_);
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

  static ankerl::unordered_dense::map<node_idx_t, std::vector<shortcut>, hash> shortcuts_;
  static ankerl::unordered_dense::map<node_idx_t, std::vector<shortcut>, hash> shortcuts_by_target_;
  static ankerl::unordered_dense::map<node_idx_t, std::uint32_t, hash> ch_levels_;

  static void set_global_ch_level(node_idx_t n, std::uint32_t level) {
    ch_levels_[n] = level;
  }
  static std::uint32_t get_global_ch_level(node_idx_t n) {
    auto it = ch_levels_.find(n);
    return it != ch_levels_.end() ? it->second : 0U;
  }
  static void clear_global_ch_levels() { ch_levels_.clear(); }

  void assign_ch_levels(ways const& w) {
    auto const n_nodes = w.n_nodes();
    std::vector<std::uint32_t> levels(n_nodes);
    std::iota(levels.begin(), levels.end(), 1U);
    std::random_device rd;
    std::mt19937 gen(rd());
    std::shuffle(levels.begin(), levels.end(), gen);
    clear_global_ch_levels();
    for (node_idx_t::value_t i = 0; i < n_nodes; ++i) {
      set_global_ch_level(node_idx_t{i}, levels[i]);
    }
  }

  std::uint32_t get_ch_level(node_idx_t n) const { return get_global_ch_level(n); }

  static shortcut_anchor make_anchor(ways const& w,
                                     way_idx_t way,
                                     way_pos_t way_pos,
                                     direction dir,
                                     std::uint16_t from,
                                     std::uint16_t to) {
    distance_t step_dist = 0;
    if (way != way_idx_t{} && way != way_idx_t::invalid() && w.r_ != nullptr) {
      auto const wi  = static_cast<std::size_t>(way.v_);
      if (wi < w.r_->way_node_dist_.size()) {
        auto const idx = static_cast<std::size_t>(std::min(from, to));
        if (idx < w.r_->way_node_dist_[way].size()) {
          step_dist = w.r_->way_node_dist_[way][idx];
        }
      }
    }
    return shortcut_anchor{way, way_pos, dir, from, to, step_dist};
  }

  static void add_global_shortcut(node_idx_t from, node_idx_t to, cost_t weight,
                                  node_idx_t middle,
                                  shortcut_anchor first_a,
                                  shortcut_anchor last_a) {
    shortcuts_[from].emplace_back(shortcut{to, weight, middle, first_a, last_a});
    shortcuts_by_target_[to].emplace_back(shortcut{from, to, weight, middle, first_a, last_a});
  }

  static std::size_t get_global_shortcut_count(node_idx_t from) {
    auto it = shortcuts_.find(from);
    return it != shortcuts_.end() ? it->second.size() : 0;
  }

  static void clear_global_shortcuts() {
    shortcuts_.clear();
    shortcuts_by_target_.clear();
  }

  static void clear_global_data() {
    clear_global_shortcuts();
    clear_global_ch_levels();
  }

  std::size_t get_shortcut_count(node_idx_t from) const {
    return get_global_shortcut_count(from);
  }

  void clear_shortcuts() { clear_global_shortcuts(); }
  void clear_data() { clear_global_data(); }

  std::vector<edge_info> get_incoming_edges(ways const& w, node_idx_t u) const {
    std::vector<edge_info> incoming;
    auto const u_level = get_ch_level(u);

    car::resolve_all(*w.r_, u, level_t{}, [&](node const n_at_u) {
      adjacent<direction::kBackward, false>(
          *w.r_, n_at_u, nullptr, nullptr, nullptr,
          [&](node const pred, std::uint32_t const cost, distance_t,
              way_idx_t const way, std::uint16_t from, std::uint16_t to,
              elevation_storage::elevation const, bool const) {
            auto const pred_level = get_ch_level(pred.n_);
            if (pred_level > u_level) {
              edge_info e{};
              e.from = pred.n_;
              e.to = u;
              e.cost = static_cast<cost_t>(cost);
              e.is_shortcut = false;
              e.first_way_pos = pred.way_;
              e.first_dir = pred.dir_;
              e.first_way = way;
              e.first_from = from;
              e.first_to = to;
              e.last_way_pos = n_at_u.way_;
              e.last_dir = n_at_u.dir_;
              e.last_way = way;
              e.last_from = from;
              e.last_to = to;
              incoming.push_back(e);
            }
          });
    });

    if (auto it = shortcuts_by_target_.find(u); it != shortcuts_by_target_.end()) {
      for (auto const& sc : it->second) {
        auto const from_level = get_ch_level(sc.from);
        if (from_level > u_level) {
          edge_info e{};
          e.from = sc.from;
          e.to = u;
          e.cost = sc.weight;
          e.is_shortcut = true;
          e.first_way_pos = sc.first.way_pos;
          e.first_dir = sc.first.dir;
          e.first_way = sc.first.way;
          e.first_from = sc.first.from;
          e.first_to = sc.first.to;
          e.last_way_pos = sc.last.way_pos;
          e.last_dir = sc.last.dir;
          e.last_way = sc.last.way;
          e.last_from = sc.last.from;
          e.last_to = sc.last.to;
          incoming.push_back(e);
        }
      }
    }

    return incoming;
  }

  std::vector<edge_info> get_outgoing_edges(ways const& w, node_idx_t u) const {
    std::vector<edge_info> outgoing;
    auto const u_level = get_ch_level(u);

    car::resolve_all(*w.r_, u, level_t{}, [&](node const n_at_u) {
      adjacent<direction::kForward, false>(
          *w.r_, n_at_u, nullptr, nullptr, nullptr,
          [&](node const succ, std::uint32_t const cost, distance_t,
              way_idx_t const way, std::uint16_t from, std::uint16_t to,
              elevation_storage::elevation const, bool const) {
            auto const succ_level = get_ch_level(succ.n_);
            if (succ_level > u_level) {
              edge_info e{};
              e.from = u;
              e.to = succ.n_;
              e.cost = static_cast<cost_t>(cost);
              e.is_shortcut = false;
              e.first_way_pos = n_at_u.way_;
              e.first_dir = n_at_u.dir_;
              e.first_way = way;
              e.first_from = from;
              e.first_to = to;
              e.last_way_pos = succ.way_;
              e.last_dir = succ.dir_;
              e.last_way = way;
              e.last_from = from;
              e.last_to = to;
              outgoing.push_back(e);
            }
          });
    });

    if (auto it = shortcuts_.find(u); it != shortcuts_.end()) {
      for (auto const& sc : it->second) {
        auto const target_level = get_ch_level(sc.target);
        if (target_level > u_level) {
          edge_info e{};
          e.from = u;
          e.to = sc.target;
          e.cost = sc.weight;
          e.is_shortcut = true;
          e.first_way_pos = sc.first.way_pos;
          e.first_dir = sc.first.dir;
          e.first_way = sc.first.way;
          e.first_from = sc.first.from;
          e.first_to = sc.first.to;
          e.last_way_pos = sc.last.way_pos;
          e.last_dir = sc.last.dir;
          e.last_way = sc.last.way;
          e.last_from = sc.last.from;
          e.last_to = sc.last.to;
          outgoing.push_back(e);
        }
      }
    }

    return outgoing;
  }

  void perform_contraction(ways const& w) {
    auto const n_nodes = w.n_nodes();
    std::vector<node_idx_t> nodes_by_level;
    nodes_by_level.reserve(n_nodes);
    for (node_idx_t::value_t i = 0; i < n_nodes; ++i) {
      nodes_by_level.emplace_back(node_idx_t{i});
    }
    std::sort(nodes_by_level.begin(), nodes_by_level.end(),
              [this](node_idx_t a, node_idx_t b) {
                return get_ch_level(a) < get_ch_level(b);
              });

    std::size_t shortcuts_added = 0;
    std::size_t nodes_contracted = 0;

    for (auto const u : nodes_by_level) {
      auto const node_shortcuts = contract_node(w, u);
      shortcuts_added += node_shortcuts;
      ++nodes_contracted;
      fmt::println("Contracted node {} (level {}): {} shortcuts added, total: {}",
                   u.v_, get_ch_level(u), node_shortcuts, shortcuts_added);
    }

    fmt::println("Contraction complete: {} nodes contracted, {} shortcuts added",
                 nodes_contracted, shortcuts_added);
  }

  std::size_t contract_node(ways const& w, node_idx_t u) {
    auto const incoming = get_incoming_edges(w, u);
    auto const outgoing = get_outgoing_edges(w, u);

    std::size_t shortcuts_added = 0;
    auto const u_level = get_ch_level(u);

    for (auto const& in_edge : incoming) {
      auto const v = in_edge.from;

      for (auto const& out_edge : outgoing) {
        auto const w_target = out_edge.to;

        if (v == w_target) {
          continue;
        }

        if ((*w.r_).is_restricted<direction::kForward>(
                u, in_edge.last_way_pos, out_edge.first_way_pos)) {
          continue;
        }

        auto const shortcut_cost = in_edge.cost + out_edge.cost;
        if (shortcut_cost > kInfeasible) {
          continue;
        }

        auto const witness_cost = witness_search(w, v, w_target, u, u_level,
                                                 in_edge.first_way_pos, in_edge.first_dir);

        if (witness_cost > shortcut_cost) {
          auto const first_a = make_anchor(w,
                                           in_edge.first_way,
                                           in_edge.first_way_pos,
                                           in_edge.first_dir,
                                           in_edge.first_from,
                                           in_edge.first_to);

          auto const last_a  = make_anchor(w,
                                           out_edge.last_way,
                                           out_edge.last_way_pos,
                                           out_edge.last_dir,
                                           out_edge.last_from,
                                           out_edge.last_to);

          add_global_shortcut(v, w_target,
                              static_cast<cost_t>(shortcut_cost), u,
                              first_a, last_a);
          ++shortcuts_added;
        }
      }
    }

    return shortcuts_added;
  }

  cost_t witness_search(ways const& w, node_idx_t v, node_idx_t target,
                        node_idx_t u, std::uint32_t u_level,
                        way_pos_t start_way_pos, direction start_dir) const {

    struct State {
      node_idx_t node;
      way_pos_t way_pos;
      direction dir;
      cost_t cost;
      bool operator>(State const& other) const { return cost > other.cost; }
    };

    std::priority_queue<State, std::vector<State>, std::greater<State>> pq;

    struct StateKey {
      node_idx_t node;
      way_pos_t way_pos;
      direction dir;
      bool operator==(StateKey const& other) const {
        return node == other.node && way_pos == other.way_pos && dir == other.dir;
      }
    };

    struct StateKeyHash {
      std::size_t operator()(StateKey const& k) const {
        return std::hash<std::uint64_t>{}(
          (static_cast<std::uint64_t>(to_idx(k.node)) << 16) |
          (static_cast<std::uint64_t>(k.way_pos) << 8) |
          (k.dir == direction::kForward ? 0 : 1)
        );
      }
    };

    std::unordered_map<StateKey, cost_t, StateKeyHash> dist;

    pq.push({v, start_way_pos, start_dir, 0U});
    dist[{v, start_way_pos, start_dir}] = 0U;

    while (!pq.empty()) {
      auto const curr = pq.top();
      pq.pop();

      if (curr.node == target) {
        return curr.cost;
      }

      auto const state_key = StateKey{curr.node, curr.way_pos, curr.dir};
      auto const dist_it = dist.find(state_key);
      if (dist_it != dist.end() && curr.cost > dist_it->second) {
        continue;
      }

      if (curr.node != u) {
        car::resolve_all(*w.r_, curr.node, level_t{}, [&](node const n) {
          if (n.way_ != curr.way_pos || n.dir_ != curr.dir) {
            return;
          }

          adjacent<direction::kForward, false>(
              *w.r_, n, nullptr, nullptr, nullptr,
              [&](node const succ, std::uint32_t const edge_cost, distance_t,
                  way_idx_t const, std::uint16_t, std::uint16_t,
                  elevation_storage::elevation const, bool const) {

                if (succ.n_ == u) { return; }
                if (get_ch_level(succ.n_) <= u_level) { return; }

                auto const new_cost = curr.cost + edge_cost;
                StateKey const nk{succ.n_, succ.way_, succ.dir_};
                auto const it2 = dist.find(nk);
                if (it2 == dist.end() || new_cost < it2->second) {
                  dist[nk] = new_cost;
                  pq.push({succ.n_, succ.way_, succ.dir_, static_cast<cost_t>(new_cost)});
                }
              });
        });
      }
    }

    return kInfeasible;
  }

  // (Unpacking kept but unused for now)
  std::vector<node_idx_t> unpack_path(node_idx_t from, node_idx_t to) const {
    std::vector<node_idx_t> path;
    unpack_path_recursive(from, to, path);
    fmt::print("Unpacked path ({} nodes): ", path.size());
    for (auto i = 0U; i < std::min(3U, static_cast<unsigned>(path.size())); ++i) {
      fmt::print("{}", path[i].v_);
      if (i < std::min(3U, static_cast<unsigned>(path.size())) - 1) {
        fmt::print(" -> ");
      }
    }
    if (path.size() > 3) {
      fmt::print(" -> ... -> {}", path.back().v_);
    }
    fmt::println("");
    return path;
  }

private:
  void unpack_path_recursive(node_idx_t from, node_idx_t to, std::vector<node_idx_t>& path) const {
    fmt::println("Unpacking path segment: {} -> {}", from.v_, to.v_);

    auto const it = shortcuts_.find(from);
    if (it != shortcuts_.end()) {
      for (auto const& sc : it->second) {
        if (sc.target == to && sc.middle != node_idx_t::invalid()) {
          fmt::println("  Shortcut found: {} -> {} via middle {}",
                       from.v_, to.v_, sc.middle.v_);
          unpack_path_recursive(from, sc.middle, path);
          unpack_path_recursive(sc.middle, to, path);
          return;
        }
      }
    }

    fmt::println("  Direct edge: {} -> {}", from.v_, to.v_);
    if (path.empty() || path.back() != from) {
      path.push_back(from);
    }
    path.push_back(to);
  }

public:
};

inline ankerl::unordered_dense::map<node_idx_t, std::vector<bidirectional_car_dijkstra::shortcut>, bidirectional_car_dijkstra::hash>
    bidirectional_car_dijkstra::shortcuts_{};

inline ankerl::unordered_dense::map<node_idx_t, std::vector<bidirectional_car_dijkstra::shortcut>, bidirectional_car_dijkstra::hash>
    bidirectional_car_dijkstra::shortcuts_by_target_{};

inline ankerl::unordered_dense::map<node_idx_t, std::uint32_t, bidirectional_car_dijkstra::hash>
    bidirectional_car_dijkstra::ch_levels_{};

}  // namespace osr
