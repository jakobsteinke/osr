#pragma once

#include <limits>
#include <queue>
#include <algorithm>

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

// Contraction Hierarchies Extension (Optional Layer)
namespace ch {

using ch_level_t = std::uint32_t;
using ch_order_t = std::uint32_t;

struct ch_node_data {
  ch_level_t level = 0;
  ch_order_t order = 0;
  bool contracted = false;
};

struct ch_edge {
  car::node target;
  cost_t cost;
  distance_t dist;
  way_idx_t way;
  std::uint16_t from;
  std::uint16_t to;

  // Shortcut information
  bool is_shortcut = false;
  car::node via_node = car::node::invalid();

  ch_edge() = default;
  ch_edge(car::node target_, cost_t cost_, distance_t dist_, way_idx_t way_,
          std::uint16_t from_, std::uint16_t to_)
    : target(target_), cost(cost_), dist(dist_), way(way_), from(from_), to(to_) {}
};

using ch_adjacency_map = ankerl::unordered_dense::map<car::node, std::vector<ch_edge>, car_node_hash>;
using ch_node_map = ankerl::unordered_dense::map<car::node, ch_node_data, car_node_hash>;

// Global CH data structures
inline ch_adjacency_map ch_upward_graph;
inline ch_adjacency_map ch_downward_graph;
inline ch_node_map ch_node_info;
inline bool ch_preprocessed = false;
inline size_t ch_shortcuts_created = 0;

// CH Preprocessing Functions
inline ch_level_t calculate_node_importance(car::node const& node) {
  auto upward_it = ch_upward_graph.find(node);
  auto downward_it = ch_downward_graph.find(node);

  ch_level_t upward_degree = upward_it != ch_upward_graph.end() ? static_cast<ch_level_t>(upward_it->second.size()) : 0;
  ch_level_t downward_degree = downward_it != ch_downward_graph.end() ? static_cast<ch_level_t>(downward_it->second.size()) : 0;

  return upward_degree + downward_degree;
}

inline void assign_node_levels() {
  std::vector<std::pair<ch_level_t, car::node>> importance_list;

  // Calculate importance for all nodes
  for (auto const& [node, successors] : legal_successor) {
    ch_level_t importance = calculate_node_importance(node);
    importance_list.emplace_back(importance, node);
  }

  // Sort by importance (ascending) - only compare the first element (importance)
  std::sort(importance_list.begin(), importance_list.end(),
            [](auto const& a, auto const& b) { return a.first < b.first; });

  // Assign levels based on order
  for (size_t i = 0; i < importance_list.size(); ++i) {
    auto const& node = importance_list[i].second;
    ch_node_info[node].level = static_cast<ch_level_t>(i + 1);
    ch_node_info[node].order = static_cast<ch_order_t>(i + 1);
  }
}

inline bool witness_search(car::node const& from, car::node const& to,
                          cost_t max_cost, car::node const& excluded) {
  ankerl::unordered_dense::map<car::node, cost_t, car_node_hash> distances;

  // Custom comparator for priority queue that only compares cost
  auto cmp = [](std::pair<cost_t, car::node> const& a, std::pair<cost_t, car::node> const& b) {
    return a.first > b.first;  // Min-heap on cost
  };
  std::priority_queue<std::pair<cost_t, car::node>,
                     std::vector<std::pair<cost_t, car::node>>,
                     decltype(cmp)> pq(cmp);

  distances[from] = 0;
  pq.emplace(0, from);

  while (!pq.empty()) {
    auto [cost, current] = pq.top();
    pq.pop();

    if (current.n_ == excluded.n_ && current.way_ == excluded.way_ && current.dir_ == excluded.dir_) {
      continue;
    }

    if (current.n_ == to.n_ && current.way_ == to.way_ && current.dir_ == to.dir_) {
      return cost <= max_cost;
    }

    if (cost > max_cost) continue;

    auto it = distances.find(current);
    if (it != distances.end() && cost > it->second) continue;

    auto succ_it = legal_successor.find(current);
    if (succ_it != legal_successor.end()) {
      for (auto const& successor : succ_it->second) {
        auto const& neighbor = successor.target;
        if (neighbor.n_ == excluded.n_ && neighbor.way_ == excluded.way_ && neighbor.dir_ == excluded.dir_) {
          continue;
        }

        // Only use nodes with higher level than excluded node for witness search
        auto neighbor_level = ch_node_info[neighbor].level;
        auto excluded_level = ch_node_info[excluded].level;
        if (neighbor_level <= excluded_level) continue;

        auto new_cost = cost + successor.cost;
        if (new_cost <= max_cost) {
          auto dist_it = distances.find(neighbor);
          if (dist_it == distances.end() || new_cost < dist_it->second) {
            distances[neighbor] = new_cost;
            pq.emplace(new_cost, neighbor);
          }
        }
      }
    }
  }

  return false;
}

inline void contract_node(car::node const& node) {
  auto incoming_it = legal_predecessor.find(node);
  auto outgoing_it = legal_successor.find(node);

  if (incoming_it == legal_predecessor.end() || outgoing_it == legal_successor.end()) {
    return;
  }

  auto const& incoming = incoming_it->second;
  auto const& outgoing = outgoing_it->second;

  // For each incoming-outgoing pair, check if shortcut is needed
  for (auto const& in_edge : incoming) {
    auto const& in_node = in_edge.target;
    if (ch_node_info[in_node].contracted) continue;

    for (auto const& out_edge : outgoing) {
      auto const& out_node = out_edge.target;
      if (ch_node_info[out_node].contracted) continue;

      // Skip self-loops
      if (in_node.n_ == out_node.n_ && in_node.way_ == out_node.way_ && in_node.dir_ == out_node.dir_) {
        continue;
      }

      cost_t shortcut_cost = in_edge.cost + out_edge.cost;

      // Witness search: check if there's an alternative path with cost <= shortcut_cost
      if (!witness_search(in_node, out_node, shortcut_cost, node)) {
        // Add shortcut edge
        ch_edge shortcut{out_node, shortcut_cost, 0, way_idx_t::invalid(), 0, 0};
        shortcut.is_shortcut = true;
        shortcut.via_node = node;

        ch_upward_graph[in_node].push_back(shortcut);

        // Add reverse shortcut for downward graph
        ch_edge reverse_shortcut{in_node, shortcut_cost, 0, way_idx_t::invalid(), 0, 0};
        reverse_shortcut.is_shortcut = true;
        reverse_shortcut.via_node = node;

        ch_downward_graph[out_node].push_back(reverse_shortcut);

        // Count both forward and reverse shortcuts
        ch_shortcuts_created += 2;
      }
    }
  }

  ch_node_info[node].contracted = true;
}

inline void preprocess_ch() {
  if (ch_preprocessed) return;

  std::cout << "Starting CH preprocessing..." << std::endl;
  ch_shortcuts_created = 0;

  // Initialize CH graphs with original edges
  ch_upward_graph.clear();
  ch_downward_graph.clear();
  ch_node_info.clear();

  // Copy original edges to CH graphs
  for (auto const& [node, successors] : legal_successor) {
    for (auto const& successor : successors) {
      ch_edge edge{successor.target, static_cast<cost_t>(successor.cost), successor.distance,
                   successor.way, successor.from, successor.to};
      ch_upward_graph[node].push_back(edge);

      ch_edge reverse_edge{node, static_cast<cost_t>(successor.cost), successor.distance,
                          successor.way, successor.to, successor.from};
      ch_downward_graph[successor.target].push_back(reverse_edge);
    }
  }

  // Assign levels based on node importance
  assign_node_levels();

  // Contract nodes in order of importance
  std::vector<std::pair<ch_order_t, car::node>> contraction_order;
  for (auto const& [node, data] : ch_node_info) {
    contraction_order.emplace_back(data.order, node);
  }

  // Sort by order (ascending) - only compare the first element (order)
  std::sort(contraction_order.begin(), contraction_order.end(),
            [](auto const& a, auto const& b) { return a.first < b.first; });

  size_t contracted_count = 0;
  for (auto const& [order, node] : contraction_order) {
    contract_node(node);
    contracted_count++;

    if (contracted_count % 1000 == 0) {
      std::cout << "Contracted " << contracted_count << "/" << contraction_order.size() << " nodes" << std::endl;
    }
  }

  ch_preprocessed = true;
  std::cout << "CH preprocessing completed: " << ch_upward_graph.size() << " upward entries, "
            << ch_downward_graph.size() << " downward entries, "
            << ch_shortcuts_created << " shortcuts created" << std::endl;
}

// CH Query: Bidirectional search using only upward edges
inline std::optional<std::pair<cost_t, car::node>> ch_bidirectional_search(
    car::node const& start, car::node const& target, cost_t max_cost) {

  if (!ch_preprocessed) {
    preprocess_ch();
  }

  ankerl::unordered_dense::map<car::node, cost_t, car_node_hash> dist_forward;
  ankerl::unordered_dense::map<car::node, cost_t, car_node_hash> dist_backward;

  using pq_entry = std::pair<cost_t, car::node>;

  // Custom comparator for priority queue that only compares cost
  auto pq_cmp = [](pq_entry const& a, pq_entry const& b) {
    return a.first > b.first;  // Min-heap on cost
  };
  std::priority_queue<pq_entry, std::vector<pq_entry>, decltype(pq_cmp)> pq_forward(pq_cmp);
  std::priority_queue<pq_entry, std::vector<pq_entry>, decltype(pq_cmp)> pq_backward(pq_cmp);

  dist_forward[start] = 0;
  dist_backward[target] = 0;
  pq_forward.emplace(0, start);
  pq_backward.emplace(0, target);

  cost_t best_cost = kInfeasible;
  car::node meeting_point = car::node::invalid();

  while (!pq_forward.empty() || !pq_backward.empty()) {
    // Forward search
    if (!pq_forward.empty()) {
      auto [cost, current] = pq_forward.top();
      pq_forward.pop();

      if (cost >= max_cost) break;

      auto dist_it = dist_forward.find(current);
      if (dist_it != dist_forward.end() && cost > dist_it->second) continue;

      // Check for meeting point
      auto backward_it = dist_backward.find(current);
      if (backward_it != dist_backward.end()) {
        cost_t total_cost = cost + backward_it->second;
        if (total_cost < best_cost) {
          best_cost = total_cost;
          meeting_point = current;
        }
      }

      // Expand only upward edges (higher level nodes)
      auto up_it = ch_upward_graph.find(current);
      if (up_it != ch_upward_graph.end()) {
        auto current_level = ch_node_info[current].level;
        for (auto const& edge : up_it->second) {
          auto neighbor_level = ch_node_info[edge.target].level;
          if (neighbor_level > current_level) {
            cost_t new_cost = cost + edge.cost;
            if (new_cost < max_cost) {
              auto neighbor_dist_it = dist_forward.find(edge.target);
              if (neighbor_dist_it == dist_forward.end() || new_cost < neighbor_dist_it->second) {
                dist_forward[edge.target] = new_cost;
                pq_forward.emplace(new_cost, edge.target);
              }
            }
          }
        }
      }
    }

    // Backward search
    if (!pq_backward.empty()) {
      auto [cost, current] = pq_backward.top();
      pq_backward.pop();

      if (cost >= max_cost) break;

      auto dist_it = dist_backward.find(current);
      if (dist_it != dist_backward.end() && cost > dist_it->second) continue;

      // Check for meeting point
      auto forward_it = dist_forward.find(current);
      if (forward_it != dist_forward.end()) {
        cost_t total_cost = cost + forward_it->second;
        if (total_cost < best_cost) {
          best_cost = total_cost;
          meeting_point = current;
        }
      }

      // Expand only upward edges (higher level nodes) in reverse direction
      auto down_it = ch_downward_graph.find(current);
      if (down_it != ch_downward_graph.end()) {
        auto current_level = ch_node_info[current].level;
        for (auto const& edge : down_it->second) {
          auto neighbor_level = ch_node_info[edge.target].level;
          if (neighbor_level > current_level) {
            cost_t new_cost = cost + edge.cost;
            if (new_cost < max_cost) {
              auto neighbor_dist_it = dist_backward.find(edge.target);
              if (neighbor_dist_it == dist_backward.end() || new_cost < neighbor_dist_it->second) {
                dist_backward[edge.target] = new_cost;
                pq_backward.emplace(new_cost, edge.target);
              }
            }
          }
        }
      }
    }
  }

  if (best_cost < kInfeasible) {
    return std::make_pair(best_cost, meeting_point);
  }

  return std::nullopt;
}

} // namespace ch

}  // namespace osr