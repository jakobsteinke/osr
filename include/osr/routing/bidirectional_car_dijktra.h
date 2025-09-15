#pragma once

#include <limits>
#include <random>
#include <algorithm>
#include <queue>
#include <unordered_map>

#include "utl/verify.h"
#include "utl/zip.h"

#include "osr/elevation_storage.h"
#include "osr/location.h"
#include "osr/routing/dial.h"
#include "osr/routing/profiles/car.h"
#include "osr/routing/sharing_data.h"
#include "osr/types.h"
#include "osr/ways.h"
#include "osr/util/infinite.h"
#include "osr/util/reverse.h"

namespace osr {

struct sharing_data;

struct edge_transition {
  car::node source;
  car::node target;
  cost_t cost;
  distance_t dist;
  way_idx_t way;
  std::uint16_t from;
  std::uint16_t to;

  // Shortcut fields
  bool is_shortcut = false;
  car::node via_state = car::node::invalid();

  // Store complete edge information for reliable unpacking
  edge_transition* first_edge = nullptr;   // Complete A->B edge info
  edge_transition* second_edge = nullptr;  // Complete B->C edge info

  // Default constructor
  edge_transition() = default;

  // Constructor for normal edges
  edge_transition(car::node source_, car::node target_, cost_t cost_, distance_t dist_, way_idx_t way_,
                  std::uint16_t from_, std::uint16_t to_)
    : source(source_), target(target_), cost(cost_), dist(dist_), way(way_), from(from_), to(to_) {
  }

  // Copy constructor for shortcuts
  edge_transition(const edge_transition& other)
    : source(other.source), target(other.target), cost(other.cost), dist(other.dist), way(other.way),
      from(other.from), to(other.to), is_shortcut(other.is_shortcut),
      via_state(other.via_state) {
    // Deep copy edge pointers for shortcuts
    if (other.first_edge) {
      first_edge = new edge_transition(*other.first_edge);
    }
    if (other.second_edge) {
      second_edge = new edge_transition(*other.second_edge);
    }
  }

  // Assignment operator
  edge_transition& operator=(const edge_transition& other) {
    if (this != &other) {
      source = other.source;
      target = other.target;
      cost = other.cost;
      dist = other.dist;
      way = other.way;
      from = other.from;
      to = other.to;
      is_shortcut = other.is_shortcut;
      via_state = other.via_state;

      // Clean up old pointers
      delete first_edge;
      delete second_edge;
      first_edge = nullptr;
      second_edge = nullptr;

      // Deep copy new pointers
      if (other.first_edge) {
        first_edge = new edge_transition(*other.first_edge);
      }
      if (other.second_edge) {
        second_edge = new edge_transition(*other.second_edge);
      }
    }
    return *this;
  }

  // Destructor
  ~edge_transition() {
    delete first_edge;
    delete second_edge;
  }
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

using car_adjacency_map = ankerl::unordered_dense::map<car::node, std::vector<edge_transition>, car_node_hash>;
inline car_adjacency_map legal_successor;
inline car_adjacency_map legal_predecessor;

// CH data structures
inline ankerl::unordered_dense::map<car::node, std::uint32_t, car_node_hash> car_node_levels;
inline ankerl::unordered_dense::map<car::node, std::vector<car::node>, car_node_hash> legal_incoming;

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
    chosen_edge_fwd_.clear();
    chosen_edge_bwd_.clear();
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
      for (auto const& edge : it->second) {
        if constexpr (WithBlocked) {
          if (blocked && blocked->test(edge.target.n_)) {
            continue;
          }
        }

        // CH level filtering: only relax edges to higher-level car::nodes
        auto curr_level_it = car_node_levels.find(curr);
        auto target_level_it = car_node_levels.find(edge.target);
        if (curr_level_it != car_node_levels.end() && target_level_it != car_node_levels.end()) {
          if (target_level_it->second <= curr_level_it->second) {
            continue;
          }
        }

        auto const neighbor = edge.target;
        auto const cost = edge.cost;
        auto const way = edge.way;

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
          next.track(l, r, way, neighbor.get_node(), false);
          pq.push(std::move(next));

          // Record the chosen transition for reconstruction
          auto& chosen_map = (SearchDir == direction::kForward)
                               ? chosen_edge_fwd_
                               : chosen_edge_bwd_;
          chosen_map[neighbor] = edge;

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

  // Chosen edge maps for reconstruction
  using chosen_edge_map = ankerl::unordered_dense::map<car::node, edge_transition, car_node_hash>;
  chosen_edge_map chosen_edge_fwd_;  // edges used by forward search
  chosen_edge_map chosen_edge_bwd_;  // edges used by backward search
};

inline void preprocess_car_adjacency(ways const& w,
                              ways::routing const& r,
                              bitvec<node_idx_t> const* blocked = nullptr,
                              sharing_data const* sharing = nullptr,
                              elevation_storage const* elevations = nullptr) {
  legal_successor.clear();
  legal_predecessor.clear();
  legal_incoming.clear();

  for (node_idx_t n{0}; n < w.n_nodes(); ++n) {
    car::resolve_all(r, n, level_t{}, [&](car::node state) {

      car::adjacent<direction::kForward, false>(
        r, state, blocked, sharing, elevations,
        [&](car::node target, std::uint32_t cost, distance_t dist,
            way_idx_t way, std::uint16_t from, std::uint16_t to,
            elevation_storage::elevation elev, bool track) {
          legal_successor[state].emplace_back(
            edge_transition{state, target, static_cast<cost_t>(cost), dist, way, from, to});
          legal_incoming[target].emplace_back(state);
        });

      car::adjacent<direction::kBackward, false>(
        r, state, blocked, sharing, elevations,
        [&](car::node target, std::uint32_t cost, distance_t dist,
            way_idx_t way, std::uint16_t from, std::uint16_t to,
            elevation_storage::elevation elev, bool track) {
          legal_predecessor[state].emplace_back(
            edge_transition{state, target, static_cast<cost_t>(cost), dist, way, from, to});
          legal_incoming[state].emplace_back(target);
        });
    });
  }
}

inline void assign_random_car_node_levels() {
  std::vector<car::node> all_car_nodes;

  // Collect all car::node states from legal_successor keys
  for (auto const& [car_node, successors] : legal_successor) {
    all_car_nodes.push_back(car_node);
  }

  // Shuffle and assign levels
  std::random_device rd;
  std::mt19937 gen(rd());
  std::shuffle(all_car_nodes.begin(), all_car_nodes.end(), gen);

  car_node_levels.clear();
  for (size_t i = 0; i < all_car_nodes.size(); ++i) {
    car_node_levels[all_car_nodes[i]] = static_cast<std::uint32_t>(i + 1);
  }
}

inline std::uint32_t find_edge_cost(car::node const& from, car::node const& to) {
  if (auto it = legal_successor.find(from); it != legal_successor.end()) {
    for (auto const& successor : it->second) {
      if (successor.target == to) {
        return successor.cost;
      }
    }
  }
  return std::numeric_limits<std::uint32_t>::max();
}

inline bool witness_path_exists(car::node const& from, car::node const& to,
                               std::uint32_t const max_cost, car::node const& excluded) {
  // Simple Dijkstra without the excluded node to find if path exists with cost <= max_cost
  ankerl::unordered_dense::map<car::node, std::uint32_t, car_node_hash> distances;
  auto cmp = [](std::pair<std::uint32_t, car::node> const& a,
                std::pair<std::uint32_t, car::node> const& b) {
    return a.first > b.first;  // min-heap: smaller costs have higher priority
  };
  std::vector<std::pair<std::uint32_t, car::node>> pq;

  distances[from] = 0;
  pq.emplace_back(0, from);

  while (!pq.empty()) {
    std::pop_heap(pq.begin(), pq.end(), cmp);
    auto [cost, current] = pq.back();
    pq.pop_back();

    if (current == excluded) continue;
    if (current == to) return cost <= max_cost;
    if (cost > max_cost) continue;

    if (auto it = distances.find(current); it != distances.end() && cost > it->second) {
      continue;
    }

    if (auto succ_it = legal_successor.find(current); succ_it != legal_successor.end()) {
      for (auto const& successor : succ_it->second) {
        if (successor.target == excluded) continue;
        if (car_node_levels[successor.target] <= car_node_levels[excluded]) continue;

        auto new_cost = cost + successor.cost;
        if (new_cost <= max_cost) {
          if (auto dist_it = distances.find(successor.target);
              dist_it == distances.end() || new_cost < dist_it->second) {
            distances[successor.target] = new_cost;
            pq.emplace_back(new_cost, successor.target);
            std::push_heap(pq.begin(), pq.end(), cmp);
          }
        }
      }
    }
  }

  return false;
}

// Add shortcut between two nodes via intermediate node
inline void add_shortcut(car::node const& from, car::node const& to,
                        cost_t const total_cost, car::node const& via,
                        edge_transition const& first_edge,
                        edge_transition const& second_edge) {
  // Create shortcut for forward direction (successors)
  edge_transition forward_shortcut;
  forward_shortcut.source = from;
  forward_shortcut.target = to;
  forward_shortcut.cost = total_cost;
  forward_shortcut.dist = 0;  // shortcuts have no physical distance
  forward_shortcut.way = way_idx_t::invalid();  // marks as shortcut
  forward_shortcut.from = 0;
  forward_shortcut.to = 0;
  forward_shortcut.is_shortcut = true;
  forward_shortcut.via_state = via;
  // Store deep copies of the edge objects for reconstruction
  forward_shortcut.first_edge = new edge_transition(first_edge);
  forward_shortcut.second_edge = new edge_transition(second_edge);

  // Create shortcut for backward direction (predecessors)
  edge_transition backward_shortcut;
  backward_shortcut.source = to;
  backward_shortcut.target = from;
  backward_shortcut.cost = total_cost;
  backward_shortcut.dist = 0;  // shortcuts have no physical distance
  backward_shortcut.way = way_idx_t::invalid();  // marks as shortcut
  backward_shortcut.from = 0;
  backward_shortcut.to = 0;
  backward_shortcut.is_shortcut = true;
  backward_shortcut.via_state = via;
  // Store deep copies of the edge objects (reversed for predecessors)
  backward_shortcut.first_edge = new edge_transition(second_edge);
  backward_shortcut.second_edge = new edge_transition(first_edge);

  // Add to adjacency maps
  legal_successor[from].push_back(forward_shortcut);
  legal_predecessor[to].push_back(backward_shortcut);
  legal_incoming[to].push_back(from);
}


// Simple witness search to check if shortcut is necessary
inline bool witness_path_exists(car::node const& from, car::node const& to,
                                cost_t const max_cost, car::node const& avoided_node) {
  // Simple BFS-based witness search
  std::unordered_map<car::node, cost_t, car_node_hash> distances;
  std::queue<std::pair<car::node, cost_t>> pq;

  distances[from] = 0;
  pq.push({from, 0});

  while (!pq.empty()) {
    auto [current, current_cost] = pq.front();
    pq.pop();

    if (current_cost > max_cost) continue;

    // Found target with cost <= max_cost
    if (current.n_ == to.n_ && current.way_ == to.way_ && current.dir_ == to.dir_) {
      return true;
    }

    auto it = legal_successor.find(current);
    if (it != legal_successor.end()) {
      for (auto const& edge : it->second) {
        // Skip the avoided node
        if (edge.target.n_ == avoided_node.n_ &&
            edge.target.way_ == avoided_node.way_ &&
            edge.target.dir_ == avoided_node.dir_) {
          continue;
        }

        // Skip if level is too low (only use nodes with higher level than avoided)
        auto target_level_it = car_node_levels.find(edge.target);
        auto avoided_level_it = car_node_levels.find(avoided_node);
        if (target_level_it != car_node_levels.end() && avoided_level_it != car_node_levels.end()) {
          if (target_level_it->second <= avoided_level_it->second) {
            continue;
          }
        }

        cost_t new_cost = current_cost + edge.cost;
        if (new_cost > max_cost) continue;

        auto dist_it = distances.find(edge.target);
        if (dist_it == distances.end() || new_cost < dist_it->second) {
          distances[edge.target] = new_cost;
          pq.push({edge.target, new_cost});
        }
      }
    }
  }

  return false;  // No witness path found
}

inline void contract_car_node(car::node const& u) {
  // Find all predecessors of u with level > level(u)
  std::vector<std::pair<car::node, edge_transition>> valid_predecessors;
  if (auto it = legal_incoming.find(u); it != legal_incoming.end()) {
    for (auto const& v : it->second) {
      if (car_node_levels[v] > car_node_levels[u]) {
        // Find the edge from v to u
        auto pred_it = legal_successor.find(v);
        if (pred_it != legal_successor.end()) {
          for (auto const& edge : pred_it->second) {
            if (edge.target.n_ == u.n_ && edge.target.way_ == u.way_ && edge.target.dir_ == u.dir_) {
              valid_predecessors.emplace_back(v, edge);
              break;
            }
          }
        }
      }
    }
  }

  // Find all successors of u with level > level(u)
  std::vector<edge_transition> valid_successors;
  if (auto it = legal_successor.find(u); it != legal_successor.end()) {
    for (auto const& edge : it->second) {
      if (car_node_levels[edge.target] > car_node_levels[u]) {
        valid_successors.push_back(edge);
      }
    }
  }

  // For each predecessor-successor pair, check if shortcut needed
  for (auto const& [v, v_to_u_edge] : valid_predecessors) {
    for (auto const& u_to_w_edge : valid_successors) {
      auto const& w = u_to_w_edge.target;

      // Skip self-loops
      if (v.n_ == w.n_ && v.way_ == w.way_ && v.dir_ == w.dir_) {
        continue;
      }

      // Calculate shortcut cost: cost(v,u) + cost(u,w)
      auto shortcut_cost = static_cast<cost_t>(v_to_u_edge.cost + u_to_w_edge.cost);

      // Witness search: check if direct path v->w exists with <= shortcut_cost
      if (!witness_path_exists(v, w, shortcut_cost, u)) {
        add_shortcut(v, w, shortcut_cost, u, v_to_u_edge, u_to_w_edge);
      }
    }
  }
}

inline void contract_car_nodes() {
  // Get all car::node states sorted by level
  std::vector<std::pair<std::uint32_t, car::node>> level_sorted;
  for (auto const& [car_node, level] : car_node_levels) {
    level_sorted.emplace_back(level, car_node);
  }

  std::sort(level_sorted.begin(), level_sorted.end(),
            [](auto const& a, auto const& b) {
              return a.first < b.first;  // Sort by level ascending
            });

  for (auto const& [level, u] : level_sorted) {
    contract_car_node(u);
  }
}

inline void preprocess(ways const& w,
                      ways::routing const& r,
                      bitvec<node_idx_t> const* blocked = nullptr,
                      sharing_data const* sharing = nullptr,
                      elevation_storage const* elevations = nullptr) {
  // Step 1: Build basic adjacency maps + legal_incoming
  preprocess_car_adjacency(w, r, blocked, sharing, elevations);

  // Step 2: Assign random levels to all car::node states
  assign_random_car_node_levels();

  // Step 3: Contract car::node states in level order
  contract_car_nodes();
}

// Function to fully unpack shortcuts into a flat sequence of base edges
inline std::vector<edge_transition> unpack_shortcut_to_base_edges(
    edge_transition const& edge,
    bool normalize_for_backward = false) {

  std::vector<edge_transition> result;

  if (!edge.is_shortcut) {
    // Base case: not a shortcut, just return it (normalized if needed)
    if (normalize_for_backward) {
      auto normalized = edge;
      // Swap source and target for backward edges
      normalized.source = edge.target;
      normalized.target = edge.source;
      // Swap from and to indices
      normalized.from = edge.to;
      normalized.to = edge.from;
      result.push_back(normalized);
    } else {
      result.push_back(edge);
    }
    return result;
  }

  // Recursive case: unpack first and second edges
  if (!edge.first_edge || !edge.second_edge) {
    throw std::runtime_error("shortcut missing edge pointers during unpacking");
  }

  auto first_unpacked = unpack_shortcut_to_base_edges(*edge.first_edge, normalize_for_backward);
  auto second_unpacked = unpack_shortcut_to_base_edges(*edge.second_edge, normalize_for_backward);

  // Combine them
  result.insert(result.end(), first_unpacked.begin(), first_unpacked.end());
  result.insert(result.end(), second_unpacked.begin(), second_unpacked.end());

  return result;
}

// Direct path building using structural information - no cost matching needed
inline double add_path_by_edge(ways const& w,
                               ways::routing const& r,
                               edge_transition const& edge,
                               std::vector<path::segment>& path,
                               direction const dir) {
  auto& segment = path.emplace_back();
  segment.way_ = edge.way;
  segment.dist_ = edge.dist;
  segment.cost_ = edge.cost;
  segment.mode_ = edge.target.get_mode();

  if (edge.way != way_idx_t::invalid()) {
    auto const start_idx = dir == direction::kBackward ? edge.to : edge.from;
    auto const end_idx = dir == direction::kBackward ? edge.from : edge.to;
    auto const is_reverse = (start_idx > end_idx);
    auto const is_loop = r.is_loop(edge.way) &&
                        static_cast<unsigned>(std::abs(static_cast<int>(start_idx) - static_cast<int>(end_idx))) ==
                        r.way_nodes_[edge.way].size() - 2U;

    if (is_reverse) {
      segment.from_level_ = r.way_properties_[edge.way].to_level();
      segment.to_level_ = r.way_properties_[edge.way].from_level();
    } else {
      segment.from_level_ = r.way_properties_[edge.way].from_level();
      segment.to_level_ = r.way_properties_[edge.way].to_level();
    }
    segment.from_ = r.way_nodes_[edge.way][start_idx];
    segment.to_ = r.way_nodes_[edge.way][end_idx];

    // Build polyline directly from way geometry using stored indices
    auto j = 0U;
    auto active = false;
    for (auto const [osm_idx, coord] : infinite(
             reverse(utl::zip(w.way_osm_nodes_[edge.way], w.way_polylines_[edge.way]),
                     is_reverse), is_loop)) {
      utl::verify(j++ != 2 * w.way_polylines_[edge.way].size() + 1U, "infinite loop");
      if (!active && w.node_to_osm_[r.way_nodes_[edge.way][start_idx]] == osm_idx) {
        active = true;
      }
      if (active) {
        if (w.node_to_osm_[r.way_nodes_[edge.way][start_idx]] == osm_idx) {
          segment.polyline_.clear(); // Start fresh from here
        }
        segment.polyline_.emplace_back(coord);
        if (w.node_to_osm_[r.way_nodes_[edge.way][end_idx]] == osm_idx) {
          break;
        }
      }
    }
  } else {
    // Fallback for invalid way (should not happen with proper structural storage)
    segment.from_level_ = level_t{0.0F};
    segment.to_level_ = level_t{0.0F};
    segment.from_ = edge.source.get_node();
    segment.to_ = edge.target.get_node();
    segment.polyline_ = {w.get_node_pos(segment.from_).as_latlng(),
                         w.get_node_pos(segment.to_).as_latlng()};
  }

  return static_cast<double>(edge.dist);
}

}  // namespace osr