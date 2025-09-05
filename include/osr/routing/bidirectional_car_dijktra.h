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
  
  struct turn_anchor {
    node_idx_t at;          // intersection node
    way_idx_t way;          // way ID  
    way_pos_t way_pos;      // position in node_ways_[at]
    direction dir;          // travel direction
    std::uint16_t from, to; // way segment indices
    node_idx_t target;      // target node
    
    // Default constructor for invalid anchor
    turn_anchor() : at(node_idx_t::invalid()), way(way_idx_t::invalid()), 
                    way_pos(0), dir(direction::kForward), from(0), to(0), 
                    target(node_idx_t::invalid()) {}
  };
  
  struct shortcut {
    node_idx_t target;       // destination node
    cost_t weight;           // shortcut cost
    node_idx_t middle;       // bypassed node (for reconstruction)
    turn_anchor first_real_edge;  // first normal edge in path
    turn_anchor last_real_edge;   // last normal edge in path
    node_idx_t from;         // source node (for reverse index)
    
    // Constructor without from (for forward index)
    shortcut(node_idx_t t, cost_t w, node_idx_t m, turn_anchor first, turn_anchor last)
      : target(t), weight(w), middle(m), first_real_edge(first), last_real_edge(last), from(node_idx_t::invalid()) {}
      
    // Constructor with from (for reverse index)
    shortcut(node_idx_t f, node_idx_t t, cost_t w, node_idx_t m, turn_anchor first, turn_anchor last)
      : target(t), weight(w), middle(m), first_real_edge(first), last_real_edge(last), from(f) {}
  };
  
  struct edge_info {
    node_idx_t from;
    node_idx_t to;
    cost_t cost;
    bool is_shortcut;
    turn_anchor first_anchor;  // for shortcuts: first real edge
    turn_anchor last_anchor;   // for shortcuts: last real edge
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
              fmt::println("FORWARD search reached max cost {} at node {}", max, curr.get_key().v_);
            } else {
              max_reached_2_ = true;
              fmt::println("BACKWARD search reached max cost {} at node {}", max, curr.get_key().v_);
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
            fmt::println("FORWARD shortcut reached max cost {} from {} to {}", max, curr.get_key().v_, sc.target.v_);
          } else {
            max_reached_2_ = true;
            fmt::println("BACKWARD shortcut reached max cost {} from {} to {}", max, curr.get_key().v_, sc.target.v_);
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
  
  // CH level assignment
  ankerl::unordered_dense::map<node_idx_t, std::uint32_t, hash> ch_levels_;
  
  // CH shortcuts (global storage)
  static ankerl::unordered_dense::map<node_idx_t, std::vector<shortcut>, hash> shortcuts_;
  
  // Reverse index: shortcuts by target node (for efficient incoming edge lookup)
  static ankerl::unordered_dense::map<node_idx_t, std::vector<shortcut>, hash> shortcuts_by_target_;
  
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
  
  static void add_global_shortcut(node_idx_t from, node_idx_t to, cost_t weight, 
                                  node_idx_t middle, turn_anchor first_real_edge, 
                                  turn_anchor last_real_edge) {
    // For testing: set all shortcut weights to 1
    // weight = 1;
    
    // Add to forward index (shortcuts from 'from') - no 'from' field needed
    shortcuts_[from].emplace_back(shortcut{to, weight, middle, first_real_edge, last_real_edge});
    
    // Add to reverse index (shortcuts to 'to') - include 'from' field
    shortcuts_by_target_[to].emplace_back(shortcut{from, to, weight, middle, first_real_edge, last_real_edge});
  }
  
  static std::size_t get_global_shortcut_count(node_idx_t from) {
    auto it = shortcuts_.find(from);
    return it != shortcuts_.end() ? it->second.size() : 0;
  }
  
  static void clear_global_shortcuts() {
    shortcuts_.clear();
    shortcuts_by_target_.clear();
  }
  
  std::size_t get_shortcut_count(node_idx_t from) const {
    return get_global_shortcut_count(from);
  }
  
  void clear_shortcuts() {
    clear_global_shortcuts();
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
              
              // For normal edge, create turn anchor
              // This edge goes from pred.n_ to u (n.n_)
              edge.first_anchor.at = pred.n_;
              edge.first_anchor.way = way;
              edge.first_anchor.dir = opposite(n.dir_); // Backward search, so flip direction
              edge.first_anchor.from = from;
              edge.first_anchor.to = to;
              edge.first_anchor.target = n.n_;
              
              // Find way_pos at pred.n_
              auto const ways_at_pred = (*w.r_).node_ways_[pred.n_];
              for (auto i = way_pos_t{0U}; i != ways_at_pred.size(); ++i) {
                if (ways_at_pred[i] == way) {
                  edge.first_anchor.way_pos = i;
                  break;
                }
              }
              
              // Last anchor is at the destination (u)
              edge.last_anchor.at = n.n_;
              edge.last_anchor.way = way;
              edge.last_anchor.dir = opposite(n.dir_);
              edge.last_anchor.from = from;
              edge.last_anchor.to = to;
              edge.last_anchor.target = n.n_;
              edge.last_anchor.way_pos = n.way_;
              
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
          edge.first_anchor = sc.first_real_edge;
          edge.last_anchor = sc.last_real_edge;
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
              
              // For normal edge, create turn anchor
              // This edge goes from u (n.n_) to succ.n_
              edge.first_anchor.at = n.n_;
              edge.first_anchor.way = way;
              edge.first_anchor.dir = n.dir_;
              edge.first_anchor.from = from;
              edge.first_anchor.to = to;
              edge.first_anchor.target = succ.n_;
              edge.first_anchor.way_pos = n.way_;
              
              // Last anchor is at the destination (succ.n_)
              edge.last_anchor.at = succ.n_;
              edge.last_anchor.way = way;
              edge.last_anchor.dir = n.dir_;
              edge.last_anchor.from = from;
              edge.last_anchor.to = to;
              edge.last_anchor.target = succ.n_;
              
              // Find way_pos at succ.n_
              auto const ways_at_succ = (*w.r_).node_ways_[succ.n_];
              for (auto i = way_pos_t{0U}; i != ways_at_succ.size(); ++i) {
                if (ways_at_succ[i] == way) {
                  edge.last_anchor.way_pos = i;
                  break;
                }
              }
              
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
          edge.first_anchor = sc.first_real_edge;
          edge.last_anchor = sc.last_real_edge;
          outgoing.push_back(edge);
        }
      }
    }
    
    return outgoing;
  }
  
  // Check if we can legally traverse from edge (v,u) to edge (u,w)
  // Returns true if the turn is allowed according to turn restrictions
  bool is_turn_allowed(ways const& w, 
                       edge_info const& in_edge,   // (v, u)
                       edge_info const& out_edge,  // (u, w)
                       node_idx_t u) const {
    // Get the incoming way_pos (at node u) and outgoing way_pos (at node u)
    way_pos_t incoming_way_pos, outgoing_way_pos;
    
    // For incoming edge, we need the way position at u (the destination)
    if (!in_edge.is_shortcut) {
      // Normal edge: last_anchor is at u
      incoming_way_pos = in_edge.last_anchor.way_pos;
    } else {
      // Shortcut: use last_anchor which represents the last real edge entering u
      incoming_way_pos = in_edge.last_anchor.way_pos;
    }
    
    // For outgoing edge, we need the way position at u (the source)
    if (!out_edge.is_shortcut) {
      // Normal edge: first_anchor is at u
      outgoing_way_pos = out_edge.first_anchor.way_pos;
    } else {
      // Shortcut: use first_anchor which represents the first real edge leaving u
      outgoing_way_pos = out_edge.first_anchor.way_pos;
    }
    
    // Check turn restriction using OSR's is_restricted function
    // Template parameter is search direction (forward for contraction)
    return !(*w.r_).is_restricted<direction::kForward>(u, incoming_way_pos, outgoing_way_pos);
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
    constexpr auto const kMaxShortcutsPerNode = 50U;
    
    // Target-aware batched witness search: for each predecessor v, collect all targets and run optimized Dijkstra
    for (auto const& in_edge : incoming) {
      auto const v = in_edge.from;
      
      // Collect all potential targets with their required costs
      std::vector<TargetNeed> target_needs;
      std::vector<turn_anchor> last_anchors; // parallel to target_needs for anchor tracking
      
      for (auto const& out_edge : outgoing) {
        auto const target = out_edge.to;
        
        // Skip self-loops
        if (v == target) {
          continue;
        }
        
        // Check if the turn from (v,u) to (u,w) is allowed
        /*if (!is_turn_allowed(w, in_edge, out_edge, u)) {
          continue;
        }*/
        
        auto const shortcut_cost = in_edge.cost + out_edge.cost;
        
        // Check for overflow
        if (shortcut_cost > kInfeasible) {
          continue;
        }
        
        // Skip if direct edge/shortcut already exists with cost <= need
        if (has_direct_leq(w, v, target, static_cast<cost_t>(shortcut_cost))) {
          continue;
        }
        
        target_needs.push_back({target, static_cast<cost_t>(shortcut_cost)});
        last_anchors.push_back(out_edge.last_anchor);
      }
      
      // Skip if no valid targets
      if (target_needs.empty()) {
        continue;
      }
      
      // Run target-aware witness search from v (stops early when all targets witnessed)
      auto const distances = witness_search_batched_targets(w, v, u_level, target_needs);
      
      // Check each target against the witness search results
      for (auto i = 0U; i < target_needs.size(); ++i) {
        // Stop if we've created too many shortcuts
        /*if (shortcuts_added >= kMaxShortcutsPerNode) {
          return shortcuts_added;
        }*/
        
        auto const& target_need = target_needs[i];
        auto const dist_it = distances.find(target_need.w);
        auto const witness_cost = (dist_it != distances.end()) ? dist_it->second : kInfeasible;
        
        // Only add shortcut if no cheaper witness path exists
        if (witness_cost >= target_need.need) {
          add_global_shortcut(v, target_need.w, target_need.need, u,
                            in_edge.first_anchor, last_anchors[i]);
          ++shortcuts_added;
        }
      }
    }
    
    return shortcuts_added;
  }
  
  struct TargetNeed {
    node_idx_t w;
    cost_t need;  // = cost(v,u) + cost(u,w)
  };

  // Check if there's a direct edge or shortcut from v to w with cost <= need
  bool has_direct_leq(ways const& w, node_idx_t v, node_idx_t target, cost_t need) const {
    // Check real out-edges of v
    bool found = false;
    car::resolve_all(*w.r_, v, level_t{}, [&](node const n){
      car::adjacent<direction::kForward, false>(
        *w.r_, n, nullptr, nullptr, nullptr,
        [&](node const succ, std::uint32_t const c, distance_t,
            way_idx_t const, std::uint16_t, std::uint16_t,
            elevation_storage::elevation const, bool const) {
          if (succ.n_ == target && c <= need) found = true;
        });
    });
    
    // Check shortcuts from v
    if (!found) {
      if (auto it = shortcuts_.find(v); it != shortcuts_.end()) {
        for (auto const& sc : it->second) {
          if (sc.target == target && sc.weight <= need) {
            found = true;
            break;
          }
        }
      }
    }
    return found;
  }

  // Target-aware batched witness search with global P_max bound (standard CH logic)
  std::unordered_map<node_idx_t, cost_t> witness_search_batched_targets(
      ways const& w,
      node_idx_t start,
      std::uint32_t contracted_level,
      std::vector<TargetNeed> const& targets) const {

    using QItem = std::pair<cost_t, node_idx_t>;
    std::priority_queue<QItem, std::vector<QItem>, std::greater<QItem>> pq;
    std::unordered_map<node_idx_t, cost_t> dist;
    dist.reserve(256);

    // Aggregate minimal need per target node
    ankerl::unordered_dense::map<node_idx_t, cost_t, hash> need;
    need.reserve(targets.size());
    for (auto const& t : targets) {
      auto it = need.find(t.w);
      if (it == need.end()) {
        need.emplace(t.w, t.need);
      } else if (t.need < it->second) {
        it->second = t.need;                // keep minimum per target
      }
    }
    if (need.empty()) return dist;
    
    // Compute P_max exactly after aggregating need
    cost_t P_max = 0U;
    for (auto const& kv : need) {
      if (kv.second > P_max) P_max = kv.second;
    }

    auto relax = [&](node_idx_t v, cost_t nd) {
      if (nd > P_max) return;              // prune by P_max
      auto it = dist.find(v);
      if (it == dist.end()) {
        dist.emplace(v, nd);
        pq.emplace(nd, v);
      } else if (nd < it->second) {
        it->second = nd;
        pq.emplace(nd, v);
      }
    };

    dist.emplace(start, 0U);
    pq.emplace(0U, start);

    while (!pq.empty()) {
      auto [d, u] = pq.top();
      pq.pop();

      if (need.empty()) break;             // all targets witnessed
      if (d > P_max) break;                // global batch bound reached (match Python logic)

      // stale?
      if (auto it = dist.find(u); it != dist.end() && d != it->second) continue;

      // If u is a target and we reached it at cost <= its need, it's witnessed
      if (auto it = need.find(u); it != need.end()) {
        if (d <= it->second) {
          need.erase(it);
          if (need.empty()) break;
          // NOTE: Do NOT change P_max here; P_max is a fixed global bound.
        }
      }

      // relax original edges (remaining graph: level > contracted_level)
      car::resolve_all(*w.r_, u, level_t{}, [&](node const n) {
        car::adjacent<direction::kForward, false>(
          *w.r_, n, nullptr, nullptr, nullptr,
          [&](node const succ, std::uint32_t const cost, distance_t,
              way_idx_t const, std::uint16_t, std::uint16_t,
              elevation_storage::elevation const, bool const) {

            if (get_ch_level(succ.n_) <= contracted_level) return;
            auto const nd = static_cast<cost_t>(d + cost);
            if (nd > P_max) return;        // prune by P_max
            relax(succ.n_, nd);
          });
      });

      // relax existing shortcuts (still respecting level)
      if (auto it = shortcuts_.find(u); it != shortcuts_.end()) {
        for (auto const& sc : it->second) {
          if (get_ch_level(sc.target) <= contracted_level) continue;
          auto const nd = static_cast<cost_t>(d + sc.weight);
          if (nd > P_max) continue;        // prune by P_max
          relax(sc.target, nd);
        }
      }
    }

    return dist;
  }

  // Original witness search: local Dijkstra from start to target on remaining graph
  // Returns the cost of the cheapest path found, or kInfeasible if no path exists
  cost_t witness_search(ways const& w, node_idx_t start, node_idx_t target, 
                        std::uint32_t contracted_level, cost_t distance_bound) const {
    using witness_label = std::pair<cost_t, node_idx_t>;
    std::priority_queue<witness_label, std::vector<witness_label>, std::greater<witness_label>> pq;
    std::unordered_map<node_idx_t, cost_t> distances;
    
    pq.emplace(0U, start);
    distances[start] = 0U;
    
    while (!pq.empty()) {
      auto const [current_cost, current_node] = pq.top();
      pq.pop();
      
      // Found target
      if (current_node == target) {
        return current_cost;
      }
      
      // Exceeded distance bound - terminate search
      if (current_cost >= distance_bound) {
        break;
      }
      
      // Skip if we've found a better path to this node
      auto const dist_it = distances.find(current_node);
      if (dist_it != distances.end() && current_cost > dist_it->second) {
        continue;
      }
      
      // Explore neighbors on remaining graph
      explore_remaining_graph(w, current_node, contracted_level, current_cost,
                              distance_bound, pq, distances);
    }
    
    return kInfeasible; // No witness path found
  }
  
  // Explore neighbors on the remaining graph (level > contracted_level, excluding contracted node)
  void explore_remaining_graph(ways const& w, node_idx_t current, std::uint32_t contracted_level,
                               cost_t current_cost, cost_t distance_bound,
                               std::priority_queue<std::pair<cost_t, node_idx_t>, 
                                                   std::vector<std::pair<cost_t, node_idx_t>>, 
                                                   std::greater<std::pair<cost_t, node_idx_t>>>& pq,
                               std::unordered_map<node_idx_t, cost_t>& distances) const {
    
    // Explore original graph edges
    car::resolve_all(*w.r_, current, level_t{}, [&](node const n) {
      car::adjacent<direction::kForward, false>(
          *w.r_, n, nullptr, nullptr, nullptr,
          [&](node const succ, std::uint32_t const cost, distance_t,
              way_idx_t const, std::uint16_t, std::uint16_t,
              elevation_storage::elevation const, bool const) {
            auto const succ_level = get_ch_level(succ.n_);
            
            // Only consider nodes with higher level (not yet contracted)
            if (succ_level <= contracted_level) {
              return;
            }
            
            auto const new_cost = current_cost + cost;
            if (new_cost >= distance_bound) {
              return;
            }
            
            auto const dist_it = distances.find(succ.n_);
            if (dist_it == distances.end() || new_cost < dist_it->second) {
              distances[succ.n_] = static_cast<cost_t>(new_cost);
              pq.emplace(static_cast<cost_t>(new_cost), succ.n_);
            }
          });
    });
    
    // Explore existing shortcuts
    auto const shortcuts_it = shortcuts_.find(current);
    if (shortcuts_it != shortcuts_.end()) {
      for (auto const& sc : shortcuts_it->second) {
        auto const target_level = get_ch_level(sc.target);
        
        // Only consider targets with higher level (not yet contracted)
        if (target_level <= contracted_level) {
          continue;
        }
        
        auto const new_cost = current_cost + sc.weight;
        if (new_cost >= distance_bound) {
          continue;
        }
        
        auto const dist_it = distances.find(sc.target);
        if (dist_it == distances.end() || new_cost < dist_it->second) {
          distances[sc.target] = new_cost;
          pq.emplace(new_cost, sc.target);
        }
      }
    }
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

}  // namespace osr