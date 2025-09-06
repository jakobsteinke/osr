#pragma once

#include <limits>
#include <numeric>
#include <random>
#include <unordered_map>
#include <queue>

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

  struct get_bucket {
    cost_t operator()(label const& l) { return l.cost(); }
  };
  
  // Simplified shortcut structure according to CLAUDE.md specifications
  
  struct shortcut {
    node_idx_t target;       // destination node
    cost_t weight;           // shortcut cost
    node_idx_t middle;       // bypassed node (for reconstruction)
    
    // First real edge: how to leave the source node
    way_pos_t first_way_pos; // way position at source to take
    direction first_dir;     // direction to travel on first edge
    
    // Last real edge: how we arrive at target node  
    way_pos_t last_way_pos;  // way position we arrive at target
    direction last_dir;      // direction we're traveling when arriving
    
    node_idx_t from;         // source node (for reverse index)
    
    // Constructor without from (for forward index)
    shortcut(node_idx_t t, cost_t w, node_idx_t m, 
             way_pos_t first_wp, direction first_d,
             way_pos_t last_wp, direction last_d)
      : target(t), weight(w), middle(m), 
        first_way_pos(first_wp), first_dir(first_d),
        last_way_pos(last_wp), last_dir(last_d),
        from(node_idx_t::invalid()) {}
      
    // Constructor with from (for reverse index)
    shortcut(node_idx_t f, node_idx_t t, cost_t w, node_idx_t m,
             way_pos_t first_wp, direction first_d,
             way_pos_t last_wp, direction last_d)
      : target(t), weight(w), middle(m),
        first_way_pos(first_wp), first_dir(first_d),
        last_way_pos(last_wp), last_dir(last_d),
        from(f) {}
  };
  
  struct edge_info {
    node_idx_t from;
    node_idx_t to;
    cost_t cost;
    bool is_shortcut;
    way_pos_t first_way_pos;  // for both edges and shortcuts
    direction first_dir;      // direction of first edge
    way_pos_t last_way_pos;   // where we arrive
    direction last_dir;       // direction when arriving
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
          auto const neighbor_level = get_ch_level(neighbor.get_key());
          auto const curr_level = get_ch_level(curr.get_key());
          //fmt::println("Level check: neighbor {} (level {}) vs curr {} (level {})", 
                       //neighbor.get_key().v_, neighbor_level, curr.get_key().v_, curr_level);
          
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
              //fmt::println("FORWARD search reached max cost {} at node {}", max, curr.get_key().v_);
            } else {
              max_reached_2_ = true;
              //fmt::println("BACKWARD search reached max cost {} at node {}", max, curr.get_key().v_);
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
            //fmt::println("FORWARD shortcut reached max cost {} from {} to {}", max, curr.get_key().v_, sc.target.v_);
          } else {
            max_reached_2_ = true;
            //fmt::println("BACKWARD shortcut reached max cost {} from {} to {}", max, curr.get_key().v_, sc.target.v_);
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
  
  // CH shortcuts (global storage)
  static ankerl::unordered_dense::map<node_idx_t, std::vector<shortcut>, hash> shortcuts_;
  
  // Reverse index: shortcuts by target node (for efficient incoming edge lookup)
  static ankerl::unordered_dense::map<node_idx_t, std::vector<shortcut>, hash> shortcuts_by_target_;
  
  // CH level assignment (global storage)
  static ankerl::unordered_dense::map<node_idx_t, std::uint32_t, hash> ch_levels_;
  
  static void set_global_ch_level(node_idx_t n, std::uint32_t level) {
    ch_levels_[n] = level;
  }
  
  static std::uint32_t get_global_ch_level(node_idx_t n) {
    auto it = ch_levels_.find(n);
    return it != ch_levels_.end() ? it->second : 0U;
  }
  
  static void clear_global_ch_levels() {
    ch_levels_.clear();
  }
  
  void assign_ch_levels(ways const& w) {
    auto const n_nodes = w.n_nodes();
    
    // Create a vector with levels from 1 to n
    std::vector<std::uint32_t> levels(n_nodes);
    std::iota(levels.begin(), levels.end(), 1U);
    
    // Shuffle to randomize levels
    std::random_device rd;
    std::mt19937 gen(rd());
    std::shuffle(levels.begin(), levels.end(), gen);
    
    // Assign levels to nodes using global storage
    clear_global_ch_levels();
    for (node_idx_t::value_t i = 0; i < n_nodes; ++i) {
      set_global_ch_level(node_idx_t{i}, levels[i]);
    }
  }
  
  std::uint32_t get_ch_level(node_idx_t n) const {
    return get_global_ch_level(n);
  }
  
  static void add_global_shortcut(node_idx_t from, node_idx_t to, cost_t weight, 
                                  node_idx_t middle,
                                  way_pos_t first_way_pos, direction first_dir,
                                  way_pos_t last_way_pos, direction last_dir) {
    // Add to forward index (shortcuts from 'from')
    shortcuts_[from].emplace_back(shortcut{to, weight, middle, 
                                           first_way_pos, first_dir,
                                           last_way_pos, last_dir});
    
    // Add to reverse index (shortcuts to 'to')
    shortcuts_by_target_[to].emplace_back(shortcut{from, to, weight, middle,
                                                   first_way_pos, first_dir,
                                                   last_way_pos, last_dir});
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
  
  void clear_shortcuts() {
    clear_global_shortcuts();
  }
  
  void clear_data() {
    clear_global_data();
  }
  
  // Discover all incoming edges to node u from nodes with level > level(u)
  std::vector<edge_info> get_incoming_edges(ways const& w, node_idx_t u) const {
    std::vector<edge_info> incoming;
    auto const u_level = get_ch_level(u);
    
    // Check original graph edges
    car::resolve_all(*w.r_, u, level_t{}, [&](node const n) {
      car::adjacent<direction::kBackward, false>(
          *w.r_, n, nullptr, nullptr, nullptr,
          [&](node const pred, std::uint32_t const cost, distance_t,
              way_idx_t const way, std::uint16_t from, std::uint16_t to,
              elevation_storage::elevation const, bool const) {
            auto const pred_level = get_ch_level(pred.n_);
            if (pred_level > u_level) {
              edge_info edge{pred.n_, u, static_cast<cost_t>(cost), false};
              
              // Find way_pos at pred.n_ for this way
              edge.first_way_pos = pred.way_;
              edge.first_dir = pred.dir_;
              
              // Way position we arrive at u
              edge.last_way_pos = n.way_;
              edge.last_dir = n.dir_;
              
              incoming.push_back(edge);
            }
          });
    });
    
    // Check shortcuts pointing to u (using reverse index for efficiency)
    auto const incoming_shortcuts_it = shortcuts_by_target_.find(u);
    if (incoming_shortcuts_it != shortcuts_by_target_.end()) {
      for (auto const& sc : incoming_shortcuts_it->second) {
        auto const from_level = get_ch_level(sc.from);
        if (from_level > u_level) {
          edge_info edge{sc.from, u, sc.weight, true};
          edge.first_way_pos = sc.first_way_pos;
          edge.first_dir = sc.first_dir;
          edge.last_way_pos = sc.last_way_pos;
          edge.last_dir = sc.last_dir;
          incoming.push_back(edge);
        }
      }
    }
    
    return incoming;
  }
  
  // Discover all outgoing edges from node u to nodes with level > level(u)
  std::vector<edge_info> get_outgoing_edges(ways const& w, node_idx_t u) const {
    std::vector<edge_info> outgoing;
    auto const u_level = get_ch_level(u);
    
    // Check original graph edges
    car::resolve_all(*w.r_, u, level_t{}, [&](node const n) {
      car::adjacent<direction::kForward, false>(
          *w.r_, n, nullptr, nullptr, nullptr,
          [&](node const succ, std::uint32_t const cost, distance_t,
              way_idx_t const way, std::uint16_t from, std::uint16_t to,
              elevation_storage::elevation const, bool const) {
            auto const succ_level = get_ch_level(succ.n_);
            if (succ_level > u_level) {
              edge_info edge{u, succ.n_, static_cast<cost_t>(cost), false};
              
              // Way position at u we use to leave
              edge.first_way_pos = n.way_;
              edge.first_dir = n.dir_;
              
              // Way position we arrive at succ
              edge.last_way_pos = succ.way_;
              edge.last_dir = succ.dir_;
              
              outgoing.push_back(edge);
            }
          });
    });
    
    // Check shortcuts from u
    auto const shortcuts_it = shortcuts_.find(u);
    if (shortcuts_it != shortcuts_.end()) {
      for (auto const& sc : shortcuts_it->second) {
        auto const target_level = get_ch_level(sc.target);
        if (target_level > u_level) {
          edge_info edge{u, sc.target, sc.weight, true};
          edge.first_way_pos = sc.first_way_pos;
          edge.first_dir = sc.first_dir;
          edge.last_way_pos = sc.last_way_pos;
          edge.last_dir = sc.last_dir;
          outgoing.push_back(edge);
        }
      }
    }
    
    return outgoing;
  }
  
  
  // Perform contraction of all nodes in ascending CH level order
  void perform_contraction(ways const& w) {
    auto const n_nodes = w.n_nodes();
    
    // Create list of nodes sorted by CH level
    std::vector<node_idx_t> nodes_by_level;
    nodes_by_level.reserve(n_nodes);
    for (node_idx_t::value_t i = 0; i < n_nodes; ++i) {
      nodes_by_level.emplace_back(node_idx_t{i});
    }
    
    // Sort by CH level (ascending)
    std::sort(nodes_by_level.begin(), nodes_by_level.end(),
              [this](node_idx_t a, node_idx_t b) {
                return get_ch_level(a) < get_ch_level(b);
              });
    
    // Contract nodes in level order
    std::size_t shortcuts_added = 0;
    std::size_t nodes_contracted = 0;
    constexpr auto const kMaxNodesToContract = 800U;
    
    for (auto const u : nodes_by_level) {
      /*if (nodes_contracted >= kMaxNodesToContract) {
        fmt::println("Stopping contraction after {} nodes", kMaxNodesToContract);
        break;
      }*/
      
      auto const node_shortcuts = contract_node(w, u);
      shortcuts_added += node_shortcuts;
      ++nodes_contracted;
      fmt::println("Contracted node {} (level {}): {} shortcuts added, total: {}", 
                   u.v_, get_ch_level(u), node_shortcuts, shortcuts_added);
    }
    
    fmt::println("Contraction complete: {} nodes contracted, {} shortcuts added", nodes_contracted, shortcuts_added);
  }
  
  // Contract a single node u
  std::size_t contract_node(ways const& w, node_idx_t u) {
    auto const incoming = get_incoming_edges(w, u);
    auto const outgoing = get_outgoing_edges(w, u);
    
    std::size_t shortcuts_added = 0;
    auto const u_level = get_ch_level(u);
    
    // For each incoming edge from v to u
    for (auto const& in_edge : incoming) {
      auto const v = in_edge.from;
      
      // For each outgoing edge from u to w  
      for (auto const& out_edge : outgoing) {
        auto const w_target = out_edge.to;
        
        // Skip self-loops
        if (v == w_target) {
          continue;
        }
        
        // Check if turn from (v,u) to (u,w) is allowed
        // We need to check if we can go from in_edge.last_way_pos to out_edge.first_way_pos at u
        if ((*w.r_).is_restricted<direction::kForward>(u, in_edge.last_way_pos, out_edge.first_way_pos)) {
          continue;
        }
        
        auto const shortcut_cost = in_edge.cost + out_edge.cost;
        
        // Check for overflow
        if (shortcut_cost > kInfeasible) {
          continue;
        }
        
        // Run witness search to check if shortest v->w path goes through u
        auto const witness_cost = witness_search(w, v, w_target, u, u_level, 
                                                 in_edge.first_way_pos, in_edge.first_dir);
        
        // Add shortcut if the path through u is necessary (no witness or witness is more expensive)
        if (witness_cost > shortcut_cost) {
          add_global_shortcut(v, w_target, static_cast<cost_t>(shortcut_cost), u,
                            in_edge.first_way_pos, in_edge.first_dir,
                            out_edge.last_way_pos, out_edge.last_dir);
          ++shortcuts_added;
        }
      }
    }
    
    return shortcuts_added;
  }
  



  // Witness search: Check if shortest v->w path in remaining graph (level >= u_level) goes through u
  // Returns cost of shortest path NOT going through u, or kInfeasible if no such path exists
  cost_t witness_search(ways const& w, node_idx_t v, node_idx_t target,
                       node_idx_t u, std::uint32_t u_level,
                       way_pos_t start_way_pos, direction start_dir) const {
    
    // We run a Dijkstra from v to target on the remaining graph (nodes with level > u_level)
    // excluding node u. Any path found doesn't go through u.
    
    struct State {
      node_idx_t node;
      way_pos_t way_pos;
      direction dir;
      cost_t cost;
      
      bool operator>(State const& other) const { return cost > other.cost; }
    };
    
    std::priority_queue<State, std::vector<State>, std::greater<State>> pq;
    
    // Map from (node, way_pos, dir) to (cost, predecessor)
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
    
    // Start from all possible states at v with the given way_pos and direction
    pq.push({v, start_way_pos, start_dir, 0U});
    dist[{v, start_way_pos, start_dir}] = 0U;
    
    // Since we exclude u from the search, any path found doesn't go through u
    
    while (!pq.empty()) {
      auto const curr = pq.top();
      pq.pop();
      
      // Check if we've reached target
      if (curr.node == target) {
        // Since we excluded u from search, this path doesn't go through u
        return curr.cost;
      }
      
      // Skip if not optimal
      auto const state_key = StateKey{curr.node, curr.way_pos, curr.dir};
      auto const dist_it = dist.find(state_key);
      if (dist_it != dist.end() && curr.cost > dist_it->second) {
        continue;
      }
      
      // Expand normal edges (only to nodes with level >= u_level)
      car::resolve_all(*w.r_, curr.node, level_t{}, [&](node const n) {
        if (n.way_ != curr.way_pos || n.dir_ != curr.dir) {
          return;
        }
        
        car::adjacent<direction::kForward, false>(
            *w.r_, n, nullptr, nullptr, nullptr,
            [&](node const succ, std::uint32_t const edge_cost, distance_t,
                way_idx_t const, std::uint16_t, std::uint16_t,
                elevation_storage::elevation const, bool const) {
              
              // Only consider nodes in remaining graph (level > u_level, excluding u)
              if (get_ch_level(succ.n_) <= u_level) {
                return;
              }
              
              auto const new_cost = curr.cost + edge_cost;
              auto const new_key = StateKey{succ.n_, succ.way_, succ.dir_};
              
              auto const existing = dist.find(new_key);
              if (existing == dist.end() || new_cost < existing->second) {
                dist[new_key] = new_cost;
                pq.push({succ.n_, succ.way_, succ.dir_, static_cast<cost_t>(new_cost)});
              }
            });
      });
      
      // Expand shortcuts (check turn restrictions)
      auto const shortcuts_it = shortcuts_.find(curr.node);
      if (shortcuts_it != shortcuts_.end()) {
        for (auto const& sc : shortcuts_it->second) {
          // Only to nodes in remaining graph (level > u_level, excluding u)
          if (get_ch_level(sc.target) <= u_level) {
            continue;
          }
          
          // Check turn restriction: can we go from curr.way_pos to sc.first_way_pos?
          if ((*w.r_).is_restricted<direction::kForward>(curr.node, curr.way_pos, sc.first_way_pos)) {
            continue;
          }
          
          auto const new_cost = curr.cost + sc.weight;
          auto const new_key = StateKey{sc.target, sc.last_way_pos, sc.last_dir};
          
          auto const existing = dist.find(new_key);
          if (existing == dist.end() || new_cost < existing->second) {
            dist[new_key] = new_cost;
            pq.push({sc.target, sc.last_way_pos, sc.last_dir, static_cast<cost_t>(new_cost)});
          }
        }
      }
    }
    
    // No path found to target in remaining graph
    return kInfeasible;
  }
  
  // Unpack a path by recursively expanding shortcuts
  std::vector<node_idx_t> unpack_path(node_idx_t from, node_idx_t to) const {
    std::vector<node_idx_t> path;
    unpack_path_recursive(from, to, path);
    return path;
  }
  
private:
  // Recursive helper for path unpacking
  void unpack_path_recursive(node_idx_t from, node_idx_t to, std::vector<node_idx_t>& path) const {
    fmt::println("Unpacking path segment: {} -> {}", from.v_, to.v_);
    
    // Check if there's a shortcut from 'from' to 'to'
    auto const shortcuts_it = shortcuts_.find(from);
    if (shortcuts_it != shortcuts_.end()) {
      for (auto const& sc : shortcuts_it->second) {
        if (sc.target == to && sc.middle != node_idx_t::invalid()) {
          // Found shortcut via middle node - unpack recursively
          fmt::println("  Shortcut found: {} -> {} via middle {}", 
                       from.v_, to.v_, sc.middle.v_);
          unpack_path_recursive(from, sc.middle, path);
          unpack_path_recursive(sc.middle, to, path);
          return;
        }
      }
    }
    
    // No shortcut found - this should be a direct edge in the original graph
    fmt::println("  Direct edge: {} -> {}", from.v_, to.v_);
    if (path.empty() || path.back() != from) {
      path.push_back(from);
    }
    path.push_back(to);
  }
  
public:
};

// Static member definitions
inline ankerl::unordered_dense::map<node_idx_t, std::vector<bidirectional_car_dijkstra::shortcut>, bidirectional_car_dijkstra::hash> bidirectional_car_dijkstra::shortcuts_{};
inline ankerl::unordered_dense::map<node_idx_t, std::vector<bidirectional_car_dijkstra::shortcut>, bidirectional_car_dijkstra::hash> bidirectional_car_dijkstra::shortcuts_by_target_{};
inline ankerl::unordered_dense::map<node_idx_t, std::uint32_t, bidirectional_car_dijkstra::hash> bidirectional_car_dijkstra::ch_levels_{};

}  // namespace osr