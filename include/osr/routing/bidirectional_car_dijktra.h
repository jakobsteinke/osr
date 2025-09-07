#pragma once

#include <algorithm>
#include <limits>
#include <optional>
#include <queue>
#include <random>
#include <vector>

#include "fmt/core.h"
#include "ankerl/unordered_dense.h"

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

  // ======== State & hashing ==================================================
  struct car_state {
    node_idx_t n;
    way_pos_t way;
    direction dir;

    bool operator==(const car_state& other) const {
      return n == other.n && way == other.way && dir == other.dir;
    }
    bool operator<(const car_state& other) const {
      if (n != other.n) return to_idx(n) < to_idx(other.n);
      if (way != other.way) return way < other.way;
      return dir < other.dir;
    }
    bool operator>(const car_state& other) const { return other < *this; }
    bool operator<=(const car_state& other) const { return !(other < *this); }
    bool operator>=(const car_state& other) const { return !(*this < other); }
  };

  struct car_state_hash {
    using is_avalanching = void;
    std::size_t operator()(const car_state& k) const {
      using namespace ankerl::unordered_dense::detail;
      auto h1 = wyhash::hash(static_cast<std::uint64_t>(to_idx(k.n)));
      auto h2 = wyhash::hash(static_cast<std::uint64_t>(k.way));
      auto h3 = wyhash::hash(static_cast<std::uint64_t>(k.dir == direction::kForward ? 0 : 1));
      return wyhash::mix(h1, wyhash::mix(h2, h3));
    }
  };

  // ======== Edge / shortcut transitions =====================================
  struct edge_transition {
    node target;
    cost_t cost;
    distance_t dist;
    way_idx_t way;
    std::uint16_t from;
    std::uint16_t to;
    std::optional<node> middle_node;  // For CH shortcuts - bypassed node

    // Normal edge
    edge_transition(node const& t, cost_t c, distance_t d, way_idx_t w,
                    std::uint16_t f, std::uint16_t to_idx)
        : target(t), cost(c), dist(d), way(w), from(f), to(to_idx), middle_node() {}

    // Shortcut (no distance / way indices here)
    edge_transition(node const& t, cost_t c, node const& middle)
        : target(t), cost(c), dist(0), way(way_idx_t::invalid()),
          from(0), to(0), middle_node(middle) {}
  };

  using adjacency_map =
      ankerl::unordered_dense::map<car_state, std::vector<edge_transition>, car_state_hash>;

  // ======== Config ===========================================================
  constexpr static auto const kDebug = false;
  constexpr static auto const kDebugMaps = false;

  struct get_bucket {
    cost_t operator()(label const& l) { return l.cost(); }
  };

  bidirectional_car_dijkstra() {}

  // ======== Boilerplate state mgmt ===========================================
  void clear_mp() {
    meet_point_1_ = node::invalid();
    meet_point_2_ = node::invalid();
    best_cost_ = kInfeasible;
  }

  void reset(cost_t const max, location const& start_loc, location const& end_loc) {
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

    if (!is_preprocessed_) { // only clear if no global CH
      legal_successors_.clear();
      legal_predecessors_.clear();
    }
  }

  // ======== Adjacency builder (per car node, per direction) ==================
  template <direction SearchDir, bool WithBlocked>
  void build_adjacency_for_node(ways const& w,
                                ways::routing const& r,
                                node const n,
                                bitvec<node_idx_t> const* blocked,
                                sharing_data const* sharing,
                                elevation_storage const* elevations) {
    car_state source_key{n.n_, n.way_, n.dir_};

    if constexpr (kDebugMaps) {
      std::cout << "\n=== Building adjacency for node ===\n";
      std::cout << "Source: ";
      n.print(std::cout, w);
      std::cout << "\n";
    }

    car::adjacent<SearchDir, WithBlocked>(
        r, n, blocked, sharing, elevations,
        [&](node const target, std::uint32_t const cost, distance_t const dist,
            way_idx_t const way, std::uint16_t const from, std::uint16_t const to,
            elevation_storage::elevation const, bool const) {
          if (SearchDir == direction::kForward) {
            legal_successors_[source_key].emplace_back(target, static_cast<cost_t>(cost), dist, way, from, to);
          } else {
            legal_predecessors_[source_key].emplace_back(target, static_cast<cost_t>(cost), dist, way, from, to);
          }
        });
  }

  // ======== PQ add helpers ===================================================
  void add(ways const&,
           label const l,
           direction const,
           cost_map& cost_map,
           dial<label, get_bucket>& d,
           sharing_data const*) {
    if (cost_map[l.get_node().get_key()].update(l, l.get_node(), l.cost(), node::invalid())) {
      d.push(l);
    }
  }

  void add_start(ways const& w, label const l, sharing_data const* sharing) {
    if constexpr (kDebugMaps) {
      std::cout << "\n+++ Adding START node: ";
      l.get_node().print(std::cout, w);
      std::cout << " (cost=" << l.cost() << ")\n";
    }
    add(w, l, direction::kForward, cost1_, pq1_, sharing);
  }

  void add_end(ways const& w, label const l, sharing_data const* sharing) {
    if constexpr (kDebugMaps) {
      std::cout << "\n+++ Adding END node: ";
      l.get_node().print(std::cout, w);
      std::cout << " (cost=" << l.cost() << ")\n";
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
    if (f_cost == kInfeasible || b_cost == kInfeasible) return kInfeasible;
    return f_cost + b_cost;
  }

  // ======== Meetpoint logic at end-of-way ===================================
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
      auto const tentative = cost + other_cost;
      if (tentative < best_cost_) {
        meet_point_1_ = meetpoint1;
        meet_point_2_ = meetpoint2;
        best_cost_ = static_cast<cost_t>(tentative);
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
        if (pred_it == end(costs)) return;
        auto const pred = pred_it->second.pred(curr);
        if (!pred.has_value()) return;

        car_state curr_key{curr.n_, curr.way_, curr.dir_};
        auto const& opp_adj_map =
            (opposite(SearchDir) == direction::kForward) ? legal_successors_ : legal_predecessors_;

        if (opp_adj_map.find(curr_key) == opp_adj_map.end()) {
          build_adjacency_for_node<opposite(SearchDir), WithBlocked>(w, r, curr, blocked, sharing, elevations);
        }

        auto opp_it = opp_adj_map.find(curr_key);
        if (opp_it != opp_adj_map.end()) {
          for (auto const& edge : opp_it->second) {
            if (edge.target.get_key() != pred->get_key()) continue;

            auto const opposite_it = opposite_cost_map->find(edge.target.get_key());
            if (opposite_it == end(*opposite_cost_map)) continue;

            auto const opposite_curr = opposite_it->second.pred(edge.target);
            if (!opposite_curr.has_value() || opposite_curr->get_key() != curr.get_key()) continue;

            auto const opposite_curr_cost = opposite_candidate->second.cost(*opposite_curr);
            auto const pred_cost = get_cost<SearchDir>(*pred);
            auto const opposite_pred_cost = opposite_it->second.cost(edge.target);

            auto const push = [&](cost_t const c1, cost_t const c2, node const m1, node const m2) {
              evaluate_meetpoint(c1, c2,
                                 SearchDir == direction::kForward ? m1 : m2,
                                 SearchDir == direction::kForward ? m2 : m1);
            };
            if (pred_cost + opposite_pred_cost > curr_cost + opposite_curr_cost) {
              push(pred_cost, opposite_pred_cost, *pred, edge.target);
            } else {
              push(curr_cost, opposite_curr_cost, curr, *opposite_curr);
            }
          }
        }
      }
    }
  }

  // ======== Enumerate states & assign random levels ==========================
  template <bool WithBlocked>
  void enumerate_all_car_states(ways const&,
                                ways::routing const& r,
                                bitvec<node_idx_t> const* blocked,
                                sharing_data const*,
                                elevation_storage const*) {
    all_nodes_.clear();
    ankerl::unordered_dense::set<car_state, car_state_hash> visited;

    for (node_idx_t n{0U}; to_idx(n) < r.node_ways_.size(); ++n) {
      if constexpr (WithBlocked) {
        if (blocked && blocked->test(n)) continue;
      }
      auto const& ways_at_node = r.node_ways_[n];
      for (way_pos_t way_pos{0U}; way_pos < ways_at_node.size(); ++way_pos) {
        for (auto dir : {direction::kForward, direction::kBackward}) {
          car_state state{n, way_pos, dir};
          if (visited.find(state) == visited.end()) {
            visited.insert(state);
            all_nodes_.push_back(state);
          }
        }
      }
    }
  }

  void assign_random_levels() {
    node_levels_.clear();

    std::vector<std::uint32_t> levels;
    levels.reserve(all_nodes_.size());
    for (std::uint32_t i = 1; i <= all_nodes_.size(); ++i) levels.push_back(i);
    std::shuffle(levels.begin(), levels.end(), rng_);

    for (std::size_t i = 0; i < all_nodes_.size(); ++i) {
      node_levels_[all_nodes_[i]] = levels[i];
    }
  }

  // ======== Witness search (uses edges + existing shortcuts) =================
  template <bool WithBlocked>
  bool witness_search(ways const& w,
                      ways::routing const& r,
                      car_state const& source,
                      car_state const& target,
                      car_state const& contracted_node,
                      cost_t const shortcut_cost,
                      bitvec<node_idx_t> const* blocked,
                      sharing_data const* sharing,
                      elevation_storage const* elevations) {
    std::priority_queue<std::pair<cost_t, car_state>,
                        std::vector<std::pair<cost_t, car_state>>,
                        std::greater<>>
        pq;

    ankerl::unordered_dense::map<car_state, cost_t, car_state_hash> distances;

    pq.emplace(0, source);
    distances[source] = 0;

    while (!pq.empty()) {
      auto const [cost, current] = pq.top();
      pq.pop();

      if (current == target) {
        return cost <= shortcut_cost; // witness found if ≤ shortcut cost
      }

      auto const it = distances.find(current);
      if (it != distances.end() && cost > it->second) continue;
      if (cost >= shortcut_cost) continue; // prune
      if (current == contracted_node) continue; // avoid u

      auto const u_level = node_levels_[contracted_node];

      // build adjacency for current if needed
      if (legal_successors_.find(current) == legal_successors_.end()) {
        node curr_node{current.n, current.way, current.dir};
        build_adjacency_for_node<direction::kForward, WithBlocked>(
            w, r, curr_node, blocked, sharing, elevations);
      }

      // expand normal edges
      if (auto adj_it = legal_successors_.find(current); adj_it != legal_successors_.end()) {
        for (auto const& e : adj_it->second) {
          car_state nxt{e.target.n_, e.target.way_, e.target.dir_};
          if (node_levels_[nxt] <= u_level) continue; // upward only
          cost_t nc = cost + e.cost;
          auto jt = distances.find(nxt);
          if (jt == distances.end() || nc < jt->second) {
            distances[nxt] = nc;
            pq.emplace(nc, nxt);
          }
        }
      }

      // expand existing shortcuts
      if (auto sc_it = shortcut_successors_.find(current); sc_it != shortcut_successors_.end()) {
        for (auto const& s : sc_it->second) {
          car_state nxt{s.target.n_, s.target.way_, s.target.dir_};
          if (node_levels_[nxt] <= u_level) continue; // upward only
          cost_t nc = cost + s.cost;
          auto jt = distances.find(nxt);
          if (jt == distances.end() || nc < jt->second) {
            distances[nxt] = nc;
            pq.emplace(nc, nxt);
          }
        }
      }
    }
    return false; // no witness found => shortcut needed
  }

  // ======== NEW: helpers to collect preds/succs & relax shortcuts ============
  void collect_predecessors(car_state const& u_state,
                            std::uint32_t u_level,
                            std::vector<std::pair<car_state, cost_t>>& out) {
    out.clear();
    auto add_min = [&](car_state const& s, cost_t c) {
      if (node_levels_[s] <= u_level) return; // CH upward-only
      for (auto& [st, cc] : out) {
        if (st == s) { cc = std::min(cc, c); return; }
      }
      out.emplace_back(s, c);
    };

    if (auto it = legal_predecessors_.find(u_state); it != legal_predecessors_.end()) {
      for (auto const& e : it->second) {
        add_min(car_state{e.target.n_, e.target.way_, e.target.dir_}, e.cost);
      }
    }
    if (auto it = shortcut_predecessors_.find(u_state); it != shortcut_predecessors_.end()) {
      for (auto const& sc : it->second) {
        add_min(car_state{sc.target.n_, sc.target.way_, sc.target.dir_}, sc.cost);
      }
    }
  }

  void collect_successors(car_state const& u_state,
                          std::uint32_t u_level,
                          std::vector<std::pair<car_state, cost_t>>& out) {
    out.clear();
    auto add_min = [&](car_state const& s, cost_t c) {
      if (node_levels_[s] <= u_level) return; // CH upward-only
      for (auto& [st, cc] : out) {
        if (st == s) { cc = std::min(cc, c); return; }
      }
      out.emplace_back(s, c);
    };

    if (auto it = legal_successors_.find(u_state); it != legal_successors_.end()) {
      for (auto const& e : it->second) {
        add_min(car_state{e.target.n_, e.target.way_, e.target.dir_}, e.cost);
      }
    }
    if (auto it = shortcut_successors_.find(u_state); it != shortcut_successors_.end()) {
      for (auto const& sc : it->second) {
        add_min(car_state{sc.target.n_, sc.target.way_, sc.target.dir_}, sc.cost);
      }
    }
  }

  void add_or_relax_shortcut(car_state const& v,
                             car_state const& w,
                             node const& middle_car_node,
                             cost_t shortcut_cost) {
    // forward index
    auto& out_vec = shortcut_successors_[v];
    bool updated = false;
    for (auto& e : out_vec) {
      car_state tgt{e.target.n_, e.target.way_, e.target.dir_};
      if (tgt == w) {
        if (shortcut_cost < e.cost) {
          e.cost = shortcut_cost;
          e.middle_node = middle_car_node;
        }
        updated = true;
        break;
      }
    }
    if (!updated) {
      out_vec.emplace_back(node{w.n, w.way, w.dir}, shortcut_cost, middle_car_node);
    }

    // reverse index
    auto& in_vec = shortcut_predecessors_[w];
    updated = false;
    for (auto& e : in_vec) {
      car_state src{e.target.n_, e.target.way_, e.target.dir_};
      if (src == v) {
        if (shortcut_cost < e.cost) {
          e.cost = shortcut_cost;
          e.middle_node = middle_car_node;
        }
        updated = true;
        break;
      }
    }
    if (!updated) {
      in_vec.emplace_back(node{v.n, v.way, v.dir}, shortcut_cost, middle_car_node);
    }
  }

  // ======== CH preprocessing (node contraction) ==============================
  template <bool WithBlocked>
  void preprocess_contraction_hierarchies(ways const& w,
                                          ways::routing const& r,
                                          bitvec<node_idx_t> const* blocked,
                                          sharing_data const* sharing,
                                          elevation_storage const* elevations) {
    enumerate_all_car_states<WithBlocked>(w, r, blocked, sharing, elevations);
    assign_random_levels();

    std::sort(all_nodes_.begin(), all_nodes_.end(),
              [&](car_state const& a, car_state const& b) {
                return node_levels_[a] < node_levels_[b];
              });

    std::size_t shortcuts_added = 0;

    for (std::size_t idx = 0; idx < all_nodes_.size(); ++idx) {
      auto const& contracted_node = all_nodes_[idx];
      auto const u_level = node_levels_[contracted_node];

      // ensure adjacency present for u (for edge-based preds/succs)
      if (legal_predecessors_.find(contracted_node) == legal_predecessors_.end()) {
        node u_node{contracted_node.n, contracted_node.way, contracted_node.dir};
        build_adjacency_for_node<direction::kBackward, WithBlocked>(
            w, r, u_node, blocked, sharing, elevations);
      }
      if (legal_successors_.find(contracted_node) == legal_successors_.end()) {
        node u_node{contracted_node.n, contracted_node.way, contracted_node.dir};
        build_adjacency_for_node<direction::kForward, WithBlocked>(
            w, r, u_node, blocked, sharing, elevations);
      }

      // gather predecessors/successors using edges + existing shortcuts
      std::vector<std::pair<car_state, cost_t>> predecessors;
      std::vector<std::pair<car_state, cost_t>> successors;
      collect_predecessors(contracted_node, u_level, predecessors);
      collect_successors(contracted_node, u_level, successors);

      node middle_car_node{contracted_node.n, contracted_node.way, contracted_node.dir};

      // pair (v, w) around u
      for (auto const& [v_state, cost_vu] : predecessors) {
        for (auto const& [w_state, cost_uw] : successors) {
          if (v_state == w_state) continue;
          cost_t const sc_cost = static_cast<cost_t>(cost_vu + cost_uw);

          bool has_witness = witness_search<WithBlocked>(
              w, r, v_state, w_state, contracted_node, sc_cost, blocked, sharing, elevations);

          if (!has_witness) {
            add_or_relax_shortcut(v_state, w_state, middle_car_node, sc_cost);
            ++shortcuts_added;
          }
        }
      }
    }

    is_preprocessed_ = true;
    fmt::println("CH preprocessing complete! Added {} shortcuts", shortcuts_added);
  }

  // ======== Dijkstra steps (with CH level filtering + shortcuts) =============
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
    if (curr_cost < l.cost()) return true;

    car_state curr_key{curr.n_, curr.way_, curr.dir_};
    auto const& adj_map =
        (SearchDir == direction::kForward) ? legal_successors_ : legal_predecessors_;

    if (adj_map.find(curr_key) == adj_map.end()) {
      build_adjacency_for_node<SearchDir, WithBlocked>(w, r, curr, blocked, sharing, elevations);
    }

    if (auto it = adj_map.find(curr_key); it != adj_map.end()) {
      std::uint32_t curr_level = 0;
      if (is_preprocessed_) {
        if (auto level_it = node_levels_.find(curr_key); level_it != node_levels_.end()) {
          curr_level = level_it->second;
        }
      }

      for (auto const& edge : it->second) {
        if (is_preprocessed_) {
          car_state tgt{edge.target.n_, edge.target.way_, edge.target.dir_};
          auto tl = node_levels_.find(tgt);
          if (tl == node_levels_.end() || tl->second <= curr_level) {
            //continue; // upward only
          }
        }

        auto const total = curr_cost + edge.cost;
        if (total >= max) {
          if (SearchDir == direction::kForward) { max_reached_1_ = true; }
          else { max_reached_2_ = true; }
          continue;
        }
        if (costs[edge.target.get_key()].update(l, edge.target, static_cast<cost_t>(total), curr)) {
          auto next = label{edge.target, static_cast<cost_t>(total)};
          next.track(l, r, edge.way, edge.target.get_node(), false);
          pq.push(std::move(next));
        }
      }
    }

    // Process shortcuts (if CH)
    if (is_preprocessed_) {
      auto const& sc_map =
          (SearchDir == direction::kForward) ? shortcut_successors_ : shortcut_predecessors_;
      if (auto sc_it = sc_map.find(curr_key); sc_it != sc_map.end()) {
        std::uint32_t curr_level = 0;
        if (auto level_it = node_levels_.find(curr_key); level_it != node_levels_.end()) {
          curr_level = level_it->second;
        }

        for (auto const& sc : sc_it->second) {
          car_state tgt{sc.target.n_, sc.target.way_, sc.target.dir_};
          auto tl = node_levels_.find(tgt);
          if (tl == node_levels_.end() || tl->second <= curr_level) {
            //continue; // upward only
          }

          auto const total = curr_cost + sc.cost;
          if (total >= max) {
            if (SearchDir == direction::kForward) { max_reached_1_ = true; }
            else { max_reached_2_ = true; }
            continue;
          }
          if (costs[sc.target.get_key()].update(l, sc.target, static_cast<cost_t>(total), curr)) {
            auto next = label{sc.target, static_cast<cost_t>(total)};
            next.track(l, r, sc.way, sc.target.get_node(), false);
            pq.push(std::move(next));
          }
        }
      }
    }

    handle_end_of_way_meetpoint<SearchDir, WithBlocked>(w, r, curr, costs, blocked, sharing, elevations);

    // μ-termination: after first meetpoint, stop when both front costs exceed best
    if (best_cost_ != kInfeasible && !pq1_.empty() && !pq2_.empty()) {
      auto const min_f = pq1_.buckets_[pq1_.get_next_bucket()].back().cost();
      auto const min_r = pq2_.buckets_[pq2_.get_next_bucket()].back().cost();
      if (min_f > best_cost_ && min_r > best_cost_) {
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
          !run_single<SearchDir, WithBlocked>(w, r, max, blocked, sharing, elevations, pq1_, cost1_)) {
        break;
      }
      if (!pq2_.empty() &&
          !run_single<opposite(SearchDir), WithBlocked>(w, r, max, blocked, sharing, elevations, pq2_, cost2_)) {
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
                 ? run<direction::kForward, false>(w, r, max, blocked, sharing, elevations)
                 : run<direction::kBackward, false>(w, r, max, blocked, sharing, elevations);
    } else {
      return dir == direction::kForward
                 ? run<direction::kForward, true>(w, r, max, blocked, sharing, elevations)
                 : run<direction::kBackward, true>(w, r, max, blocked, sharing, elevations);
    }
  }

  // ======== Shortcut unpacking (recursive) ===================================
  std::vector<node> unpack_shortcut(node const& from, node const& to) const {
    car_state from_state{from.n_, from.way_, from.dir_};

    // look only in shortcuts
    auto it = shortcut_successors_.find(from_state);
    if (it == shortcut_successors_.end()) {
      return {from, to};
    }

    car_state to_state{to.n_, to.way_, to.dir_};
    for (auto const& edge : it->second) {
      car_state target_state{edge.target.n_, edge.target.way_, edge.target.dir_};
      if (target_state == to_state && edge.middle_node.has_value()) {
        auto const& m = *edge.middle_node;
        auto left = unpack_shortcut(from, m);
        auto right = unpack_shortcut(m, to);
        left.insert(left.end(), right.begin() + 1, right.end()); // avoid dupe middle
        return left;
      }
    }
    return {from, to};
  }

  // ======== Public: enable CH ================================================
  void enable_contraction_hierarchies(ways const& w,
                                      ways::routing const& r,
                                      bitvec<node_idx_t> const* blocked = nullptr,
                                      sharing_data const* sharing = nullptr,
                                      elevation_storage const* elevations = nullptr) {
    if (blocked == nullptr) {
      preprocess_contraction_hierarchies<false>(w, r, blocked, sharing, elevations);
    } else {
      preprocess_contraction_hierarchies<true>(w, r, blocked, sharing, elevations);
    }
  }

  bool is_ch_enabled() const { return is_preprocessed_; }
  bool is_ch_preprocessed() const { return is_preprocessed_; }

  // ======== Members ==========================================================
  dial<label, get_bucket> pq1_{get_bucket{}};
  dial<label, get_bucket> pq2_{get_bucket{}};
  location start_loc_;
  location end_loc_;
  node meet_point_1_;
  node meet_point_2_;
  cost_t best_cost_{kInfeasible};
  cost_map cost1_;
  cost_map cost2_;
  bool max_reached_1_{false};
  bool max_reached_2_{false};
  adjacency_map legal_successors_;
  adjacency_map legal_predecessors_;

  // Global CH data (shared)
  static inline bool is_preprocessed_ = false;
  static inline ankerl::unordered_dense::map<car_state, std::uint32_t, car_state_hash> node_levels_;
  static inline std::vector<car_state> all_nodes_;
  static inline adjacency_map shortcut_successors_;
  static inline adjacency_map shortcut_predecessors_;
  static inline std::mt19937 rng_{std::random_device{}()};
};

}  // namespace osr
