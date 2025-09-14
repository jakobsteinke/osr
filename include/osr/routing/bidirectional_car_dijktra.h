#pragma once

#include <algorithm>
#include <limits>
#include <queue>
#include <random>
#include <unordered_map>
#include <unordered_set>

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
  // New structures for precomputed adjacency - must be defined first
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

  using profile_t = car;
  using key = car_state;
  using label = car::label;
  using node = car::node;
  using entry = car::entry;
  using hash = car_state_hash;
  using cost_map = ankerl::unordered_dense::map<key, entry, hash>;
  using ch_level_t = std::uint32_t;
  using level_map = ankerl::unordered_dense::map<car_state, ch_level_t, car_state_hash>;


  struct edge_transition {
    node source;
    node target;
    cost_t cost;
    distance_t dist;
    way_idx_t way;
    std::uint16_t from;
    std::uint16_t to;
    
    // Shortcut fields
    bool is_shortcut = false;
    car_state via_state = {node_idx_t::invalid(), way_pos_t{0}, direction::kForward};
    
    // Store complete edge information for reliable unpacking
    edge_transition* first_edge = nullptr;   // Complete A->B edge info
    edge_transition* second_edge = nullptr;  // Complete B->C edge info
    
    // Default constructor
    edge_transition() = default;
    
    // Constructor for normal edges
    edge_transition(node source_, node target_, cost_t cost_, distance_t dist_, way_idx_t way_, 
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
  
  using chosen_edge_map = ankerl::unordered_dense::map<car_state, edge_transition, car_state_hash>;
  using adjacency_map = ankerl::unordered_dense::map<car_state, std::vector<edge_transition>, car_state_hash>;
  using state_set = ankerl::unordered_dense::set<car_state, car_state_hash>;

  constexpr static auto const kDebug = false;

  struct get_bucket {
    cost_t operator()(label const& l) { return l.cost(); }
  };

  static car_state make_car_state(node const& n) {
    return {n.n_, n.way_, n.dir_};
  }

  static ch_level_t get_level(car_state const& state) {
    auto it = car_state_levels_.find(state);
    return it != car_state_levels_.end() ? it->second : 0;
  }

  void clear_mp() {
    meet_point_1_ = node::invalid();
    meet_point_2_ = node::invalid();
    best_cost_ = kInfeasible;
  }

  static void build_initial_adjacency(ways const& w,
                                      ways::routing const& r,
                                      bitvec<node_idx_t> const* blocked = nullptr,
                                      sharing_data const* sharing = nullptr,
                                      elevation_storage const* elevations = nullptr) {
    legal_successors_.clear();
    legal_predecessors_.clear();
    legal_incoming_.clear();

    // Enumerate all nodes in the graph
    for (auto n = node_idx_t{0}; n < node_idx_t{w.n_nodes()}; ++n) {
      // For each node, get all valid car node states (node_idx_t, way_pos_t, direction)
      car::resolve_all(r, n, level_t{std::uint8_t{0}}, [&](node const car_node) {
        // Build unified adjacency - forward pass populates all three maps
        build_adjacency_for_node_static<direction::kForward>(w, r, car_node, blocked, sharing, elevations);
      });
    }
  }

  // Shortcut management functions
  static void add_shortcut(car_state const& from_state,
                          car_state const& to_state,
                          car_state const& via_state,
                          cost_t const total_cost,
                          edge_transition const& first_edge,
                          edge_transition const& second_edge) {
    // Create shortcut edge for successors
    edge_transition shortcut_forward;
    shortcut_forward.source = node{from_state.n, from_state.way, from_state.dir};
    shortcut_forward.target = node{to_state.n, to_state.way, to_state.dir};
    shortcut_forward.cost = total_cost;
    shortcut_forward.dist = 0;  // shortcuts have no physical distance
    shortcut_forward.way = way_idx_t::invalid();  // marks as shortcut
    shortcut_forward.from = 0;
    shortcut_forward.to = 0;
    shortcut_forward.is_shortcut = true;
    shortcut_forward.via_state = via_state;
    // Store deep copies of the edge objects
    shortcut_forward.first_edge = new edge_transition(first_edge);
    shortcut_forward.second_edge = new edge_transition(second_edge);
    
    // Create shortcut edge for predecessors  
    edge_transition shortcut_backward;
    shortcut_backward.source = node{to_state.n, to_state.way, to_state.dir};
    shortcut_backward.target = node{from_state.n, from_state.way, from_state.dir};
    shortcut_backward.cost = total_cost;
    shortcut_backward.dist = 0;  // shortcuts have no physical distance
    shortcut_backward.way = way_idx_t::invalid();  // marks as shortcut
    shortcut_backward.from = 0;
    shortcut_backward.to = 0;
    shortcut_backward.is_shortcut = true;
    shortcut_backward.via_state = via_state;
    // Store deep copies of the edge objects (reversed for predecessors)
    shortcut_backward.first_edge = new edge_transition(second_edge);
    shortcut_backward.second_edge = new edge_transition(first_edge);
    
    // Add to adjacency maps
    legal_successors_[from_state].push_back(shortcut_forward);
    legal_predecessors_[to_state].push_back(shortcut_backward);
    legal_incoming_[to_state].push_back(shortcut_forward);
  }

  static edge_transition const* find_shortcut(car_state const& from_state,
                                             car_state const& to_state,
                                             cost_t const expected_cost) {
    auto it = legal_successors_.find(from_state);
    if (it == legal_successors_.end()) {
      return nullptr;
    }
    
    for (auto const& edge : it->second) {
      if (edge.is_shortcut && 
          edge.target.n_ == to_state.n &&
          edge.target.way_ == to_state.way &&
          edge.target.dir_ == to_state.dir &&
          edge.cost == expected_cost) {
        return &edge;
      }
    }
    
    return nullptr;
  }


  static void assign_random_levels() {
  car_state_levels_.clear();

  auto add_state = [&](car_state const& s) {
    // no-op if present
    if (car_state_levels_.find(s) == car_state_levels_.end()) {
      car_state_levels_.emplace(s, 0); // temp
    }
  };

  // 1) collect from successor map
  for (auto const& [src, edges] : legal_successors_) {
    add_state(src);
    for (auto const& e : edges) {
      add_state({e.target.n_, e.target.way_, e.target.dir_});
    }
  }
  // 2) collect from incoming map (covers targets explicitly)
  for (auto const& [src, edges] : legal_incoming_) {
    add_state(src);
    for (auto const& e : edges) {
      add_state({e.target.n_, e.target.way_, e.target.dir_});
    }
  }

  // Assign levels 1..N randomly
  std::vector<car_state> all_states;
  all_states.reserve(car_state_levels_.size());
  for (auto const& kv : car_state_levels_) all_states.push_back(kv.first);

  std::random_device rd;
  std::mt19937 gen(rd());
  std::vector<ch_level_t> levels(all_states.size());
  for (ch_level_t i = 0; i < static_cast<ch_level_t>(levels.size()); ++i) levels[i] = i + 1;
  std::shuffle(levels.begin(), levels.end(), gen);

  for (size_t i = 0; i < all_states.size(); ++i) {
    car_state_levels_[all_states[i]] = levels[i];
  }
  }

  static void preprocess(ways const& w,
                        ways::routing const& r,
                        bitvec<node_idx_t> const* blocked = nullptr,
                        sharing_data const* sharing = nullptr,
                        elevation_storage const* elevations = nullptr) {
    // Step 1: Build initial adjacency (normal edges)
    build_initial_adjacency(w, r, blocked, sharing, elevations);

    // Step 2: Assign random levels
    assign_random_levels();
  }


public:

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
     // heuristic: assume a few thousand states per small graph
    cost1_.reserve(4096);
    cost2_.reserve(4096);
    chosen_edge_fwd_.reserve(4096);
    chosen_edge_bwd_.reserve(4096);
    clear_mp();
    start_loc_ = start_loc;
    end_loc_ = end_loc;
    max_reached_1_ = false;
    max_reached_2_ = false;
    // NOTE: Do NOT clear static adjacency maps - they persist across searches
  }

  template <direction SearchDir>
  static void build_adjacency_for_node_static(ways const& w,
                                               ways::routing const& r,
                                               node const n,
                                               bitvec<node_idx_t> const* blocked,
                                               sharing_data const* sharing,
                                               elevation_storage const* elevations) {
    if (blocked != nullptr) {
      build_adjacency_for_node_impl<SearchDir, true>(w, r, n, blocked, sharing, elevations);
    } else {
      build_adjacency_for_node_impl<SearchDir, false>(w, r, n, blocked, sharing, elevations);
    }
  }

  template <direction SearchDir, bool WithBlocked>
  static void build_adjacency_for_node_impl(ways const& w,
                                             ways::routing const& r,
                                             node const n,
                                             bitvec<node_idx_t> const* blocked,
                                             sharing_data const* sharing,
                                             elevation_storage const* elevations) {
    static_assert(SearchDir == direction::kForward, "Unified adjacency building only supports forward direction");
    
    car_state source_key{n.n_, n.way_, n.dir_};
    
    // Get all forward adjacent nodes and populate all three maps simultaneously
    car::adjacent<direction::kForward, WithBlocked>(
        r, n, blocked, sharing, elevations,
        [&](node const target, std::uint32_t const cost, distance_t const dist,
            way_idx_t const way, std::uint16_t const from, std::uint16_t const to,
            elevation_storage::elevation const, bool const) {

          car_state target_key{target.n_, target.way_, target.dir_};

          // Create forward edge transition
          edge_transition forward_edge{n, target, static_cast<cost_t>(cost), dist, way, from, to};

          // Create corresponding backward edge transition (same cost, reversed source/target)
          edge_transition backward_edge{target, n, static_cast<cost_t>(cost), dist, way, to, from};

          // Update all three maps simultaneously to ensure perfect symmetry
          legal_successors_[source_key].push_back(forward_edge);
          legal_incoming_[target_key].push_back(forward_edge);
          legal_predecessors_[target_key].push_back(backward_edge);
        });
  }

  template <direction SearchDir, bool WithBlocked>
  void build_adjacency_for_node(ways const& w,
                                ways::routing const& r,
                                node const n,
                                bitvec<node_idx_t> const* blocked,
                                sharing_data const* sharing,
                                elevation_storage const* elevations) {
    build_adjacency_for_node_impl<SearchDir, WithBlocked>(w, r, n, blocked, sharing, elevations);
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
    if (SearchDir == direction::kForward) {
      auto const it = cost1_.find(make_car_state(n));
      return it != end(cost1_) ? it->second.cost(n) : kInfeasible;
    } else {
      auto const it = cost2_.find(make_car_state(n));
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

    auto const curr_cost = get_cost<SearchDir>(curr);
    auto const handle_end_of_way_meetpoint = [&]() {
      auto const opposite_cost_map =
          opposite(SearchDir) == direction::kForward ? &cost1_ : &cost2_;
      auto const opposite_candidate = opposite_cost_map->find(make_car_state(curr));
      if (opposite_candidate != end(*opposite_cost_map)) {
        auto const other_cost = opposite_candidate->second.cost(curr);
        if (other_cost != kInfeasible) {
          evaluate_meetpoint(curr_cost, other_cost, curr, curr);
        } else {
                                      std::cout << "REACHED1\n";

          auto const pred_it = costs.find(make_car_state(curr));
          if (pred_it == end(costs)) {
            return;
          }
                                                std::cout << "REACHED2\n";

          auto const pred = pred_it->second.pred(curr);
          if (!pred.has_value()) {
            return;
          }
                                                std::cout << "REACHED3\n";

          car::template adjacent<opposite(SearchDir), WithBlocked>(
              r, curr, blocked, sharing, elevations,
              [&](node const neighbor, std::uint32_t const, distance_t,
                  way_idx_t const, std::uint16_t, std::uint16_t,
                  elevation_storage::elevation const, bool const) {
                if (neighbor.get_key() != pred->get_key()) {
                  return;
                }
                                                      std::cout << "REACHED4\n";

                auto const opposite_it =
                    opposite_cost_map->find(make_car_state(neighbor));
                if (opposite_it == end(*opposite_cost_map)) {
                  return;
                }
                                                      std::cout << "REACHED5\n";

                auto const opposite_curr = opposite_it->second.pred(neighbor);
                if (!opposite_curr.has_value() ||
                    opposite_curr->get_key() != curr.get_key()) {
                  return;
                }
                                                      std::cout << "REACHED6\n";

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
    };

    handle_end_of_way_meetpoint();
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
    if (pq.empty()) {
      return true;
    }

    auto const l = pq.pop();
    auto const curr = l.get_node();
    auto const curr_cost = get_cost<SearchDir>(curr);


    
    if (curr_cost < l.cost()) {
      return true;
    }

    // Node is being settled - commit finite cost to cost map
    car_state curr_state = make_car_state(curr);
    auto& chosen_map = (SearchDir == direction::kForward) ? chosen_edge_fwd_ : chosen_edge_bwd_;
    auto chosen_it = chosen_map.find(curr_state);
    node pred = (chosen_it != chosen_map.end())
                ? chosen_it->second.source
                : node::invalid();
    costs[curr_state].update(l, curr, l.cost(), pred);

    if constexpr (kDebug) {
      std::cout << "EXTRACT ";
      l.get_node().print(std::cout, w);
      std::cout << "\n";
    }

    // Look up precomputed adjacency
    car_state curr_key{curr.n_, curr.way_, curr.dir_};
    auto const& adj_map = (SearchDir == direction::kForward) ? legal_successors_ : legal_predecessors_;
    
    auto it = adj_map.find(curr_key);
    
    if (it != adj_map.end()) {
      for (auto const& edge : it->second) {
        if constexpr (kDebug) {
          std::cout << "  NEIGHBOR ";
          edge.target.print(std::cout, w);
        }

        // CH level filtering: only relax edges that respect hierarchy
        car_state next_state{edge.target.n_, edge.target.way_, edge.target.dir_};
        ch_level_t curr_level = get_level(curr_key);
        ch_level_t next_level = get_level(next_state);

        // Forward search: only upward edges (next_level > curr_level)
        // Backward search: only downward edges (curr_level > next_level)
        bool level_valid = (SearchDir == direction::kForward) ?
                          (next_level > curr_level) :
                          (curr_level > next_level);
        
        if (!level_valid) {
          //continue;
        }

        auto const total = curr_cost + edge.cost;
        if (total >= max) {
          if (SearchDir == direction::kForward) {
            max_reached_1_ = true;
          } else {
            max_reached_2_ = true;
          }
          continue;
        }
        if (total < max) {
          // Check if this relaxation would improve the current best cost
          car_state target_state = make_car_state(edge.target);
          auto& cost_entry = costs[target_state];
          auto current_target_cost = cost_entry.cost(edge.target);

          if (static_cast<cost_t>(total) < current_target_cost) {
            auto next = label{edge.target, static_cast<cost_t>(total)};
            next.track(l, r, edge.way, edge.target.get_node(), false);
            pq.push(std::move(next));

            // Record the chosen transition for reconstruction (but don't update cost yet)
            auto& chosen_map = (SearchDir == direction::kForward)
                                 ? chosen_edge_fwd_
                                 : chosen_edge_bwd_;
            chosen_map[target_state] = edge;

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
      if (!pq1_.empty()) {
        if (!run_single<SearchDir, WithBlocked>(w, r, max, blocked, sharing,
                                                elevations, pq1_, cost1_)) {
          break;
        }
      }
      if (!pq2_.empty()) {
        if (!run_single<opposite(SearchDir), WithBlocked>(
              w, r, max, blocked, sharing, elevations, pq2_, cost2_)) {
          break;
        }
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
  location start_loc_; // not used
  location end_loc_; // not used
  node meet_point_1_;
  node meet_point_2_;
  cost_t best_cost_;
  cost_map cost1_;
  cost_map cost2_;
  bool max_reached_1_;
  bool max_reached_2_;
  chosen_edge_map chosen_edge_fwd_;  // edges used by forward search
  chosen_edge_map chosen_edge_bwd_;  // edges used by backward search
  static adjacency_map legal_successors_;
  static adjacency_map legal_predecessors_;
  static adjacency_map legal_incoming_;
  static level_map car_state_levels_;
};

// Static member definitions
inline bidirectional_car_dijkstra::adjacency_map bidirectional_car_dijkstra::legal_successors_;
inline bidirectional_car_dijkstra::adjacency_map bidirectional_car_dijkstra::legal_predecessors_;
inline bidirectional_car_dijkstra::adjacency_map bidirectional_car_dijkstra::legal_incoming_;
inline bidirectional_car_dijkstra::level_map bidirectional_car_dijkstra::car_state_levels_;

}  // namespace osr
