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
    edge_transition(node target_, cost_t cost_, distance_t dist_, way_idx_t way_, 
                    std::uint16_t from_, std::uint16_t to_)
      : target(target_), cost(cost_), dist(dist_), way(way_), from(from_), to(to_) {
    }
    
    // Copy constructor for shortcuts
    edge_transition(const edge_transition& other) 
      : target(other.target), cost(other.cost), dist(other.dist), way(other.way),
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

  using adjacency_map = ankerl::unordered_dense::map<car_state, std::vector<edge_transition>, car_state_hash>;

  constexpr static auto const kDebug = false;
  constexpr static auto const kDebugMaps = false;  // Set to true to see adjacency map structure

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
    
    if constexpr (kDebugMaps) {
      std::cout << "Starting adjacency preprocessing for " << w.n_nodes() << " nodes...\n";
    }
    
    // Enumerate all nodes in the graph
    for (auto n = node_idx_t{0}; n < node_idx_t{w.n_nodes()}; ++n) {
      // For each node, get all valid car node states (node_idx_t, way_pos_t, direction)
      car::resolve_all(r, n, level_t{std::uint8_t{0}}, [&](node const car_node) {
        // Build adjacency for both search directions
        build_adjacency_for_node_static<direction::kForward>(w, r, car_node, blocked, sharing, elevations);
        build_adjacency_for_node_static<direction::kBackward>(w, r, car_node, blocked, sharing, elevations);
      });
    }
    
    if constexpr (kDebugMaps) {
      std::cout << "Adjacency preprocessing completed: " 
                << legal_successors_.size() << " successor entries, "
                << legal_predecessors_.size() << " predecessor entries\n";
    }
    
    if constexpr (kDebugMaps) {
      // Print sample entries for debugging
      std::cout << "\nSample successor entries:\n";
      auto succ_count = 0;
      for (auto const& [key, transitions] : legal_successors_) {
        if (succ_count >= 5) break;
        std::cout << "  Key {n=" << to_idx(key.n) << ", way=" << static_cast<int>(key.way) 
                  << ", dir=" << (key.dir == direction::kForward ? "FWD" : "BWD") 
                  << "} -> " << transitions.size() << " transitions\n";
        for (auto const& t : transitions) {
          std::cout << "    -> n=" << to_idx(t.target.n_) << ", way=" << static_cast<int>(t.target.way_)
                    << ", dir=" << (t.target.dir_ == direction::kForward ? "FWD" : "BWD") 
                    << ", cost=" << t.cost << "\n";
        }
        ++succ_count;
      }
      
      std::cout << "\nSample predecessor entries:\n";
      auto pred_count = 0;
      for (auto const& [key, transitions] : legal_predecessors_) {
        if (pred_count >= 5) break;
        std::cout << "  Key {n=" << to_idx(key.n) << ", way=" << static_cast<int>(key.way) 
                  << ", dir=" << (key.dir == direction::kForward ? "FWD" : "BWD") 
                  << "} -> " << transitions.size() << " transitions\n";
        for (auto const& t : transitions) {
          std::cout << "    -> n=" << to_idx(t.target.n_) << ", way=" << static_cast<int>(t.target.way_)
                    << ", dir=" << (t.target.dir_ == direction::kForward ? "FWD" : "BWD") 
                    << ", cost=" << t.cost << "\n";
        }
        ++pred_count;
      }
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
    
    if constexpr (kDebugMaps) {
      std::cout << "Added shortcut: {n=" << to_idx(from_state.n) 
                << ", way=" << static_cast<int>(from_state.way)
                << ", dir=" << (from_state.dir == direction::kForward ? "FWD" : "BWD")
                << "} -> {n=" << to_idx(to_state.n)
                << ", way=" << static_cast<int>(to_state.way)
                << ", dir=" << (to_state.dir == direction::kForward ? "FWD" : "BWD")
                << "} via {n=" << to_idx(via_state.n)
                << ", way=" << static_cast<int>(via_state.way)
                << ", dir=" << (via_state.dir == direction::kForward ? "FWD" : "BWD")
                << "} cost=" << total_cost << std::endl;
    }
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
    
    // Collect all unique car_states from both successor and predecessor maps
    std::vector<car_state> all_states;
    for (auto const& [state, _] : legal_successors_) {
      all_states.push_back(state);
    }
    for (auto const& [state, _] : legal_predecessors_) {
      if (car_state_levels_.find(state) == car_state_levels_.end()) {
        all_states.push_back(state);
      }
    }
    
    // Remove duplicates
    /*std::sort(all_states.begin(), all_states.end(), [](car_state const& a, car_state const& b) {
      if (a.n != b.n) return a.n < b.n;
      if (a.way != b.way) return a.way < b.way;
      return a.dir < b.dir;
    });
    all_states.erase(std::unique(all_states.begin(), all_states.end(), [](car_state const& a, car_state const& b) {
      return a.n == b.n && a.way == b.way && a.dir == b.dir;
    }), all_states.end());*/
    
    // Assign random levels
    std::random_device rd;
    std::mt19937 gen(rd());
    std::vector<ch_level_t> levels;
    for (ch_level_t i = 1; i <= static_cast<ch_level_t>(all_states.size()); ++i) {
      levels.push_back(i);
    }
    std::shuffle(levels.begin(), levels.end(), gen);
    
    for (size_t i = 0; i < all_states.size(); ++i) {
      car_state_levels_[all_states[i]] = levels[i];
    }
    
    if constexpr (kDebugMaps) {
      std::cout << "Assigned random levels to " << all_states.size() << " car_states\n";
    }
  }

  // Local witness search for contraction - only consider nodes with level >= min_level, exclude contracted_node
  static std::unordered_map<car_state, cost_t, car_state_hash> 
  local_witness_search(car_state const& source, 
                      std::vector<car_state> const& targets, 
                      ch_level_t min_level,
                      car_state const& contracted_node,
                      cost_t max_cost) {
    std::unordered_map<car_state, cost_t, car_state_hash> distances;
    std::priority_queue<std::pair<cost_t, car_state>, 
                       std::vector<std::pair<cost_t, car_state>>,
                       std::greater<std::pair<cost_t, car_state>>> pq;
    
    // Initialize distances
    distances[source] = 0;
    pq.push({0, source});
    
    std::unordered_set<car_state, car_state_hash> settled;
    std::unordered_set<car_state, car_state_hash> target_set(targets.begin(), targets.end());
    size_t targets_settled = 0;
    
    while (!pq.empty() && targets_settled < targets.size()) {
      auto [current_cost, current_state] = pq.top();
      pq.pop();
      
      if (settled.count(current_state)) continue;
      if (current_cost > max_cost) break;
      
      settled.insert(current_state);
      
      // Check if this is a target
      if (target_set.count(current_state)) {
        targets_settled++;
      }
      
      // Find outgoing edges from current_state
      auto succ_it = legal_successors_.find(current_state);
      if (succ_it != legal_successors_.end()) {
        for (auto const& edge : succ_it->second) {
          car_state next_state{edge.target.n_, edge.target.way_, edge.target.dir_};
          
          // Skip if this is the contracted node
          if (next_state.n == contracted_node.n && 
              next_state.way == contracted_node.way && 
              next_state.dir == contracted_node.dir) {
            continue;
          }
          
          // Skip if level is too low (exclude contracted node and lower levels)
          if (get_level(next_state) <= min_level) {
            continue;
          }
          
          // Skip if already settled
          if (settled.count(next_state)) continue;
          
          cost_t new_cost = current_cost + edge.cost;
          if (new_cost > max_cost) continue;
          
          auto dist_it = distances.find(next_state);
          if (dist_it == distances.end() || new_cost < dist_it->second) {
            distances[next_state] = new_cost;
            pq.push({new_cost, next_state});
          }
        }
      }
    }
    
    return distances;
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
    
    // Step 3: Contract nodes
    contract_nodes(w, r, blocked, sharing, elevations);
  }

private:
  static void contract_nodes(ways const& w,
                           ways::routing const& r,
                           bitvec<node_idx_t> const* blocked,
                           sharing_data const* sharing,
                           elevation_storage const* elevations) {
    // Get all car_states sorted by level
    std::vector<std::pair<car_state, ch_level_t>> states_by_level;
    for (auto const& [state, level] : car_state_levels_) {
      states_by_level.emplace_back(state, level);
    }
    
    std::sort(states_by_level.begin(), states_by_level.end(), 
              [](auto const& a, auto const& b) { return a.second < b.second; });
    
    if constexpr (kDebugMaps) {
      std::cout << "Starting contraction of " << states_by_level.size() << " car_states...\n";
    }
    
    size_t contracted = 0;
    for (auto const& [contracted_state, contracted_level] : states_by_level) {
      contract_single_node(contracted_state, contracted_level);
      ++contracted;
      
      if constexpr (kDebugMaps) {
        if (contracted % 1000 == 0) {
          std::cout << "Contracted " << contracted << " / " << states_by_level.size() << " nodes\n";
        }
      }
    }
    
    if constexpr (kDebugMaps) {
      std::cout << "Contraction completed!\n";
    }
  }

  static void contract_single_node(car_state const& u, ch_level_t u_level) {
    // Find incoming neighbors with level > u_level
    std::vector<car_state> incoming_neighbors;
    std::vector<cost_t> incoming_costs;
    
    auto pred_it = legal_predecessors_.find(u);
    if (pred_it != legal_predecessors_.end()) {
      for (auto const& edge : pred_it->second) {
        car_state v_state{edge.target.n_, edge.target.way_, edge.target.dir_};
        if (get_level(v_state) > u_level) {
          incoming_neighbors.push_back(v_state);
          incoming_costs.push_back(edge.cost);
        }
      }
    }
    
    // Find outgoing neighbors with level > u_level  
    std::vector<car_state> outgoing_neighbors;
    std::vector<cost_t> outgoing_costs;
    std::vector<edge_transition> outgoing_edges;
    
    auto succ_it = legal_successors_.find(u);
    if (succ_it != legal_successors_.end()) {
      for (auto const& edge : succ_it->second) {
        car_state w_state{edge.target.n_, edge.target.way_, edge.target.dir_};
        if (get_level(w_state) > u_level) {
          outgoing_neighbors.push_back(w_state);
          outgoing_costs.push_back(edge.cost);
          outgoing_edges.push_back(edge);
        }
      }
    }
    
    // For each incoming neighbor, run witness search to all outgoing neighbors
    for (size_t i = 0; i < incoming_neighbors.size(); ++i) {
      car_state const& v_state = incoming_neighbors[i];
      cost_t v_to_u_cost = incoming_costs[i];
      
      // Skip self-loops  
      std::vector<car_state> targets;
      std::vector<size_t> target_indices;
      for (size_t j = 0; j < outgoing_neighbors.size(); ++j) {
        if (outgoing_neighbors[j].n != v_state.n || 
            outgoing_neighbors[j].way != v_state.way || 
            outgoing_neighbors[j].dir != v_state.dir) {
          targets.push_back(outgoing_neighbors[j]);
          target_indices.push_back(j);
        }
      }
      
      if (targets.empty()) continue;
      
      // Calculate maximum possible shortcut cost
      cost_t max_shortcut_cost = v_to_u_cost;  
      for (size_t j : target_indices) {
        max_shortcut_cost = std::max(max_shortcut_cost, static_cast<cost_t>(v_to_u_cost + outgoing_costs[j]));
      }
      
      // Run witness search
      auto distances = local_witness_search(v_state, targets, u_level, u, max_shortcut_cost);
      
      // Check each target and add shortcuts if needed
      for (size_t idx = 0; idx < target_indices.size(); ++idx) {
        size_t j = target_indices[idx];
        car_state const& w_state = outgoing_neighbors[j];
        cost_t shortcut_cost = v_to_u_cost + outgoing_costs[j];
        
        auto dist_it = distances.find(w_state);
        cost_t witness_cost = (dist_it != distances.end()) ? dist_it->second : kInfeasible;
        
        // Add shortcut if witness path is more expensive or doesn't exist
        if (witness_cost > shortcut_cost) {
          // Find the incoming edge from v to u for reconstruction
          edge_transition v_to_u_edge;
          auto v_succ_it = legal_successors_.find(v_state);
          bool found_incoming = false;
          if (v_succ_it != legal_successors_.end()) {
            for (auto const& edge : v_succ_it->second) {
              if (edge.target.n_ == u.n && edge.target.way_ == u.way && edge.target.dir_ == u.dir) {
                v_to_u_edge = edge;
                found_incoming = true;
                break;
              }
            }
          }
          
          if (found_incoming) {
            add_shortcut(v_state, w_state, u, shortcut_cost, v_to_u_edge, outgoing_edges[j]);
          }
        }
      }
    }
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
    car_state source_key{n.n_, n.way_, n.dir_};
    
    if constexpr (kDebugMaps) {
      std::cout << "\n=== Building adjacency for node ===\n";
      std::cout << "Source: ";
      n.print(std::cout, w);
      std::cout << "\n";
      std::cout << "car_state key: {n=" << to_idx(source_key.n) 
                << ", way=" << static_cast<int>(source_key.way) 
                << ", dir=" << (source_key.dir == direction::kForward ? "FWD" : "BWD") 
                << "}\n";
      std::cout << "Search direction: " << (SearchDir == direction::kForward ? "FORWARD" : "BACKWARD") << "\n";
    }
    
    // Get all adjacent nodes for this search direction
    car::adjacent<SearchDir, WithBlocked>(
        r, n, blocked, sharing, elevations,
        [&](node const target, std::uint32_t const cost, distance_t const dist,
            way_idx_t const way, std::uint16_t const from, std::uint16_t const to,
            elevation_storage::elevation const, bool const) {
          
          if constexpr (kDebugMaps) {
            std::cout << "  -> Adjacent: ";
            target.print(std::cout, w);
            std::cout << " [cost=" << cost << ", dist=" << dist 
                      << ", way=" << to_idx(way) << ", from=" << from 
                      << ", to=" << to << "]\n";
          }
          
          if (SearchDir == direction::kForward) {
            // For forward search, store as successors
            legal_successors_[source_key].push_back({target, static_cast<cost_t>(cost), dist, way, from, to});
          } else {
            // For backward search, store as predecessors  
            legal_predecessors_[source_key].push_back({target, static_cast<cost_t>(cost), dist, way, from, to});
          }
        });
    
    if constexpr (kDebugMaps) {
      auto const& map = (SearchDir == direction::kForward) ? legal_successors_ : legal_predecessors_;
      auto it = map.find(source_key);
      if (it != map.end()) {
        std::cout << "Stored " << it->second.size() << " transitions in " 
                  << (SearchDir == direction::kForward ? "legal_successors" : "legal_predecessors") 
                  << " map\n";
      } else {
        std::cout << "Stored 0 transitions in " 
                  << (SearchDir == direction::kForward ? "legal_successors" : "legal_predecessors") 
                  << " map\n";
      }
      std::cout << "=================================\n";
    }
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
    if constexpr (kDebugMaps) {
      std::cout << "\n+++ Adding START node: ";
      l.get_node().print(std::cout, w);
      std::cout << " with cost=" << l.cost() << "\n";
    }
    add(w, l, direction::kForward, cost1_, pq1_, sharing);
  }

  void add_end(ways const& w, label const l, sharing_data const* sharing) {
    if (kDebug) {
      l.get_node().print(std::cout, w);
      std::cout << "ending" << l.get_node().n_ << std::endl;
    }
    if constexpr (kDebugMaps) {
      std::cout << "\n+++ Adding END node: ";
      l.get_node().print(std::cout, w);
      std::cout << " with cost=" << l.cost() << "\n";
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

    auto const opposite_cost_map =
        opposite(SearchDir) == direction::kForward ? &cost1_ : &cost2_;
    auto const opposite_candidate = opposite_cost_map->find(make_car_state(curr));
    auto const curr_cost = get_cost<SearchDir>(curr);
    
    if (opposite_candidate != end(*opposite_cost_map)) {
      auto const other_cost = opposite_candidate->second.cost(curr);
      if (other_cost != kInfeasible) {
        evaluate_meetpoint(curr_cost, other_cost, curr, curr);
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
    if (pq.empty()) {
      return true;
    }

    auto const l = pq.pop();
    auto const curr = l.get_node();
    auto const curr_cost = get_cost<SearchDir>(curr);
    
    if constexpr (kDebugMaps) {
      std::cout << "  EXTRACT ";
      l.get_node().print(std::cout, w);
      std::cout << " with cost=" << l.cost() << " (actual_cost=" << curr_cost << ")\n";
    }
    
    if (curr_cost < l.cost()) {
      if constexpr (kDebugMaps) {
        std::cout << "  Skipping due to dominated cost\n";
      }
      return true;
    }
    
    if constexpr (kDebug) {
      std::cout << "EXTRACT ";
      l.get_node().print(std::cout, w);
      std::cout << "\n";
    }

    // Look up precomputed adjacency
    car_state curr_key{curr.n_, curr.way_, curr.dir_};
    auto const& adj_map = (SearchDir == direction::kForward) ? legal_successors_ : legal_predecessors_;
    
    auto it = adj_map.find(curr_key);
    if constexpr (kDebugMaps) {
      std::cout << "  Looking up adjacency for key {n=" << to_idx(curr_key.n) 
                << ", way=" << static_cast<int>(curr_key.way) 
                << ", dir=" << (curr_key.dir == direction::kForward ? "FWD" : "BWD") 
                << "} in " << (SearchDir == direction::kForward ? "successors" : "predecessors") << "\n";
      
      if (it == adj_map.end()) {
        std::cout << "  WARNING: No adjacency found for this key!\n";
      } else {
        std::cout << "  Found " << it->second.size() << " adjacent transitions\n";
      }
    }
    
    if constexpr (kDebugMaps) {
      if (it != adj_map.end() && !it->second.empty()) {
        std::cout << "\n>>> Using adjacency map for ";
        curr.print(std::cout, w);
        std::cout << "\n    Map type: " << (SearchDir == direction::kForward ? "legal_successors" : "legal_predecessors");
        std::cout << "\n    Found " << it->second.size() << " transitions:\n";
        for (auto const& edge : it->second) {
          std::cout << "      -> ";
          edge.target.print(std::cout, w);
          std::cout << " [cost=" << edge.cost << "]\n";
        }
      }
    }
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
        bool level_valid = next_level > curr_level;/*(SearchDir == direction::kForward) ? 
                          (next_level > curr_level) : 
                          (curr_level > next_level);*/
        
        if (!level_valid) {
          if constexpr (kDebugMaps) {
            std::cout << " -> LEVEL FILTERED (curr=" << curr_level 
                      << ", next=" << next_level << ", dir=" 
                      << (SearchDir == direction::kForward ? "FWD" : "BWD") << ")\n";
          }
          continue;
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
        if (total < max &&
            costs[make_car_state(edge.target)].update(
                l, edge.target, static_cast<cost_t>(total), curr)) {

          auto next = label{edge.target, static_cast<cost_t>(total)};
          next.track(l, r, edge.way, edge.target.get_node(), false);
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
    if constexpr (kDebugMaps) {
      std::cout << "\n=== Starting bidirectional search with max_cost=" << max << " ===\n";
      std::cout << "Initial PQ sizes: pq1=" << pq1_.size() << ", pq2=" << pq2_.size() << "\n";
    }
    
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
  static adjacency_map legal_successors_;
  static adjacency_map legal_predecessors_;
  static level_map car_state_levels_;
};

// Static member definitions
inline bidirectional_car_dijkstra::adjacency_map bidirectional_car_dijkstra::legal_successors_;
inline bidirectional_car_dijkstra::adjacency_map bidirectional_car_dijkstra::legal_predecessors_;
inline bidirectional_car_dijkstra::level_map bidirectional_car_dijkstra::car_state_levels_;

}  // namespace osr