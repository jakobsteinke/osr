#pragma once

#include <limits>
#include <queue>
#include <random>
#include <algorithm>
#include <unordered_map>
#include <unordered_set>
#include <iostream>

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

// Optimized car state representation
struct car_state {
  node_idx_t n;
  way_pos_t way;
  direction dir;

  bool operator==(const car_state& other) const {
    return n == other.n && way == other.way && dir == other.dir;
  }

  bool operator<(const car_state& other) const {
    if (n != other.n) return n < other.n;
    if (way != other.way) return way < other.way;
    return dir < other.dir;
  }
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
  car_state via_state = {node_idx_t::invalid(), way_pos_t{0}, direction::kForward};

  // Default constructor
  edge_transition() = default;

  // Constructor for normal edges
  edge_transition(car::node source_, car::node target_, cost_t cost_, distance_t dist_, way_idx_t way_,
                  std::uint16_t from_, std::uint16_t to_)
    : source(source_), target(target_), cost(cost_), dist(dist_), way(way_), from(from_), to(to_) {
  }
};

using ch_level_t = std::uint32_t;
using adjacency_map = ankerl::unordered_dense::map<car_state, std::vector<edge_transition>, car_state_hash>;
using level_map = ankerl::unordered_dense::map<car_state, ch_level_t, car_state_hash>;

// Global adjacency maps
inline adjacency_map legal_successors;
inline adjacency_map legal_predecessors;
inline adjacency_map legal_incoming;

// Contraction Hierarchies global data
inline level_map car_state_levels;

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
  using key = car_state;
  using label = car::label;
  using node = car::node;
  using entry = car_node_entry;
  using hash = car_state_hash;
  using cost_map = ankerl::unordered_dense::map<key, entry, hash>;

  constexpr static auto const kDebug = false;

  static car_state make_car_state(node const& n) {
    return {n.n_, n.way_, n.dir_};
  }

  static ch_level_t get_level(car_state const& state) {
    auto it = car_state_levels.find(state);
    return it != car_state_levels.end() ? it->second : 0;
  }

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

  static void build_initial_adjacency(ways const& w,
                                      ways::routing const& r,
                                      bitvec<node_idx_t> const* blocked = nullptr,
                                      sharing_data const* sharing = nullptr,
                                      elevation_storage const* elevations = nullptr) {
    legal_successors.clear();
    legal_predecessors.clear();
    legal_incoming.clear();

    // Enumerate all nodes in the graph
    for (auto n = node_idx_t{0}; n < node_idx_t{w.n_nodes()}; ++n) {
      // For each node, get all valid car node states
      car::resolve_all(r, n, level_t{std::uint8_t{0}}, [&](node const car_node) {
        build_adjacency_for_node(w, r, car_node, blocked, sharing, elevations);
      });
    }
  }

  static void build_adjacency_for_node(ways const& w,
                                       ways::routing const& r,
                                       node const n,
                                       bitvec<node_idx_t> const* blocked,
                                       sharing_data const* sharing,
                                       elevation_storage const* elevations) {
    car_state source_key{n.n_, n.way_, n.dir_};

    // Get all forward adjacent nodes and populate all three maps simultaneously
    car::adjacent<direction::kForward, false>(
        r, n, blocked, sharing, elevations,
        [&](node const target, std::uint32_t const cost, distance_t const dist,
            way_idx_t const way, std::uint16_t const from, std::uint16_t const to,
            elevation_storage::elevation const, bool const) {

          car_state target_key{target.n_, target.way_, target.dir_};

          // Create forward edge transition
          edge_transition forward_edge{n, target, static_cast<cost_t>(cost), dist, way, from, to};

          // Create corresponding backward edge transition
          edge_transition backward_edge{target, n, static_cast<cost_t>(cost), dist, way, to, from};

          // Update all three maps simultaneously
          legal_successors[source_key].push_back(forward_edge);
          legal_incoming[target_key].push_back(forward_edge);
          legal_predecessors[target_key].push_back(backward_edge);
        });
  }

  static void assign_random_levels() {
    car_state_levels.clear();

    auto add_state = [&](car_state const& s) {
      if (car_state_levels.find(s) == car_state_levels.end()) {
        car_state_levels.emplace(s, 0); // temp
      }
    };

    // Collect from successor map
    for (auto const& [src, edges] : legal_successors) {
      add_state(src);
      for (auto const& e : edges) {
        add_state({e.target.n_, e.target.way_, e.target.dir_});
      }
    }
    // Collect from incoming map
    for (auto const& [src, edges] : legal_incoming) {
      add_state(src);
      for (auto const& e : edges) {
        add_state({e.target.n_, e.target.way_, e.target.dir_});
      }
    }

    // Assign levels 1..N randomly
    std::vector<car_state> all_states;
    all_states.reserve(car_state_levels.size());
    for (auto const& kv : car_state_levels) all_states.push_back(kv.first);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::vector<ch_level_t> levels(all_states.size());
    for (ch_level_t i = 0; i < static_cast<ch_level_t>(levels.size()); ++i) levels[i] = i + 1;
    std::shuffle(levels.begin(), levels.end(), gen);

    for (size_t i = 0; i < all_states.size(); ++i) {
      car_state_levels[all_states[i]] = levels[i];
    }
  }

  void add(ways const& w,
           label const l,
           direction const dir,
           cost_map& cost_map,
           dial<label, get_bucket>& d,
           sharing_data const*) {
    if (cost_map[make_car_state(l.get_node())].update(l, l.get_node(), l.cost(),
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
    car_state state = make_car_state(n);
    if (SearchDir == direction::kForward) {
      auto const it = cost1_.find(state);
      return it != end(cost1_) ? it->second.cost(n) : kInfeasible;
    } else {
      auto const it = cost2_.find(state);
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
    for (auto const& [opposite_state, opposite_entry] : *opposite_cost_map) {
      if (opposite_state.n != curr.n_) continue;  // Different physical node

      node opposite_node{opposite_state.n, opposite_state.way, opposite_state.dir};
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

    car_state curr_state = make_car_state(curr);
    auto const& adjacency_map = (SearchDir == direction::kForward)
                                 ? legal_successors : legal_predecessors;
    auto it = adjacency_map.find(curr_state);
    if (it != adjacency_map.end()) {
      for (auto const& edge : it->second) {
        if constexpr (WithBlocked) {
          if (blocked && blocked->test(edge.target.n_)) {
            continue;
          }
        }

        // CH level-based pruning: only explore nodes with higher level
        car_state target_state = make_car_state(edge.target);
        if (bidirectional_car_dijkstra::get_level(target_state) <= bidirectional_car_dijkstra::get_level(curr_state)) {
          continue;
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
            costs[make_car_state(neighbor)].update(
                l, neighbor, static_cast<cost_t>(total), curr)) {

          auto next = label{neighbor, static_cast<cost_t>(total)};
          next.track(l, r, way, neighbor.get_node(), false);
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

// Function no longer needed - using state-based levels instead

inline cost_t witness_search(ways const& w,
                      ways::routing const& r,
                      car::node const from,
                      car::node const to,
                      car_state const avoid_state,
                      cost_t const max_cost) {
  ankerl::unordered_dense::map<car_state, cost_t, car_state_hash> distances;
  std::priority_queue<std::pair<cost_t, car_state>,
                      std::vector<std::pair<cost_t, car_state>>,
                      std::greater<std::pair<cost_t, car_state>>> pq;

  car_state from_state{from.n_, from.way_, from.dir_};
  car_state to_state{to.n_, to.way_, to.dir_};

  distances[from_state] = 0;
  pq.emplace(0, from_state);

  while (!pq.empty()) {
    auto const [current_cost, current_state] = pq.top();
    pq.pop();

    if (current_state == to_state) {
      return current_cost;
    }

    if (current_cost > max_cost) {
      break;
    }

    if (distances.count(current_state) && distances[current_state] < current_cost) {
      continue;
    }

    // Find neighbors using three-map adjacency system
    auto it = legal_successors.find(current_state);
    if (it != legal_successors.end()) {
      for (auto const& edge : it->second) {
        car_state target_state{edge.target.n_, edge.target.way_, edge.target.dir_};

        // Skip if going through the avoided state
        if (target_state == avoid_state) {
          continue;
        }

        // Only explore nodes with higher level (not yet contracted)
        if (bidirectional_car_dijkstra::get_level(target_state) <= bidirectional_car_dijkstra::get_level(avoid_state)) {
          continue;
        }

        auto const new_cost = current_cost + edge.cost;
        if (new_cost <= max_cost &&
            (!distances.count(target_state) || new_cost < distances[target_state])) {
          distances[target_state] = new_cost;
          pq.emplace(new_cost, target_state);
        }
      }
    }
  }

  return kInfeasible;
}

inline void preprocess_car_adjacency(ways const& w,
                              ways::routing const& r,
                              bitvec<node_idx_t> const* blocked = nullptr,
                              sharing_data const* sharing = nullptr,
                              elevation_storage const* elevations = nullptr) {
  std::cout << "Building initial adjacency maps..." << std::endl;

  // Step 1: Build initial three-map adjacency system
  bidirectional_car_dijkstra::build_initial_adjacency(w, r, blocked, sharing, elevations);

  std::cout << "Assigning random levels..." << std::endl;

  // Step 2: Assign random levels to all states
  bidirectional_car_dijkstra::assign_random_levels();

  std::cout << "Starting state-based contraction on " << car_state_levels.size() << " states..." << std::endl;

  // Step 3: Contract states by level (lowest to highest)
  std::vector<std::pair<car_state, ch_level_t>> states_by_level;
  states_by_level.reserve(car_state_levels.size());
  for (auto const& [state, level] : car_state_levels) {
    states_by_level.emplace_back(state, level);
  }

  std::sort(states_by_level.begin(), states_by_level.end(),
           [](auto const& a, auto const& b) { return a.second < b.second; });

  for (auto i = 0U; i < states_by_level.size(); ++i) {
    if (i % 1000 == 0 || i < 10) {
      std::cout << "Contracting state " << (i + 1) << "/" << states_by_level.size() << "..." << std::endl;
    }

    auto const& [contracted_state, level] = states_by_level[i];

    // Find incoming edges (predecessors)
    std::vector<edge_transition> incoming_edges;
    auto incoming_it = legal_incoming.find(contracted_state);
    if (incoming_it != legal_incoming.end()) {
      incoming_edges = incoming_it->second;
    }

    // Find outgoing edges (successors)
    std::vector<edge_transition> outgoing_edges;
    auto outgoing_it = legal_successors.find(contracted_state);
    if (outgoing_it != legal_successors.end()) {
      outgoing_edges = outgoing_it->second;
    }

    // Create shortcuts between incoming and outgoing edges
    for (auto const& incoming : incoming_edges) {
      for (auto const& outgoing : outgoing_edges) {
        auto const shortcut_cost = incoming.cost + outgoing.cost;

        // Witness search to check if shortcut is needed
        auto const witness_cost = witness_search(w, r, incoming.source, outgoing.target, contracted_state, shortcut_cost);

        // If no witness path or witness path is longer, add shortcut
        if (witness_cost == kInfeasible || witness_cost > shortcut_cost) {
          // Create shortcut edge
          edge_transition shortcut;
          shortcut.source = incoming.source;
          shortcut.target = outgoing.target;
          shortcut.cost = shortcut_cost;
          shortcut.dist = incoming.dist + outgoing.dist;
          shortcut.way = way_idx_t::invalid();
          shortcut.from = 0;
          shortcut.to = 0;
          shortcut.is_shortcut = true;
          shortcut.via_state = contracted_state;

          // Add shortcut to all three maps
          car_state source_state{incoming.source.n_, incoming.source.way_, incoming.source.dir_};
          car_state target_state{outgoing.target.n_, outgoing.target.way_, outgoing.target.dir_};

          legal_successors[source_state].push_back(shortcut);
          legal_incoming[target_state].push_back(shortcut);

          // Also add reverse shortcut to predecessors map
          edge_transition reverse_shortcut;
          reverse_shortcut.source = outgoing.target;
          reverse_shortcut.target = incoming.source;
          reverse_shortcut.cost = shortcut_cost;
          reverse_shortcut.dist = incoming.dist + outgoing.dist;
          reverse_shortcut.way = way_idx_t::invalid();
          reverse_shortcut.from = 0;
          reverse_shortcut.to = 0;
          reverse_shortcut.is_shortcut = true;
          reverse_shortcut.via_state = contracted_state;

          legal_predecessors[target_state].push_back(reverse_shortcut);
        }
      }
    }
  }

  std::cout << "Contraction Hierarchies preprocessing completed!" << std::endl;
}

}  // namespace osr