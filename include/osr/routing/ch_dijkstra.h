#pragma once

#include <limits>
#include <optional>

#include "osr/routing/ch_data.h"
#include "osr/routing/dial.h"
#include "osr/routing/profiles/car.h"
#include "osr/types.h"
#include "osr/ways.h"

namespace osr {

struct ch_dijkstra {
  using profile_t = car;
  using key = car::key;
  using label = car::label;
  using node = car::node;
  using entry = car::entry;
  using hash = car::hash;
  using internal_cost_t = std::uint32_t;  // Use larger type for internal calculations

  static constexpr bool kDebug = true;  // Enable minimal debug for small test

  // Custom label for CH that uses larger cost type
  struct ch_label {
    ch_label(node const n, internal_cost_t const c)
        : n_{n.n_}, way_{n.way_}, dir_{n.dir_}, cost_{c} {}

    constexpr node get_node() const noexcept { return {n_, way_, dir_}; }
    constexpr internal_cost_t cost() const noexcept { return cost_; }

    node_idx_t n_;
    way_pos_t way_;
    direction dir_;
    internal_cost_t cost_;
  };

  struct get_bucket {
    internal_cost_t operator()(ch_label const& l) { 
      // No masking needed with dual PQ architecture
      return l.cost_;  // Use cost_ directly, not cost() method
    }
  };

  // Custom entry that uses larger cost type for CH calculations
  struct ch_entry {
    static constexpr auto const kMaxWays = way_pos_t{16U};
    static constexpr auto const kN = kMaxWays * 2U /* FWD+BWD */;

    ch_entry() { 
      utl::fill(cost_, std::numeric_limits<internal_cost_t>::max()); 
      utl::fill(pred_, node_idx_t::invalid());
    }

    constexpr std::optional<node> pred(node const n) const noexcept {
      auto const idx = get_index(n);
      return pred_[idx] == node_idx_t::invalid()
                 ? std::nullopt
                 : std::optional{node{pred_[idx], pred_way_[idx],
                                      to_dir(pred_dir_[idx])}};
    }

    constexpr internal_cost_t cost(node const n) const noexcept {
      return cost_[get_index(n)];
    }

    bool update(label const&,
                          node const n,
                          internal_cost_t const c,
                          node const pred) noexcept {
      auto const idx = get_index(n);
      // Disable verbose update debug 
      // if (kDebug) {
      //   std::cout << "DEBUG: ch_entry::update node(" << n.n_ << "," << n.way_ << "," << (n.dir_ == direction::kForward ? "fwd" : "bwd") 
      //            << ") idx=" << idx << " new_cost=" << c << " current_cost=" << cost_[idx] << std::endl;
      // }
      if (c < cost_[idx]) {
        cost_[idx] = c;
        pred_[idx] = pred.n_;
        pred_way_[idx] = pred.way_;
        pred_dir_[idx] = to_bool(pred.dir_);
        // if (kDebug) {
        //   std::cout << "DEBUG: ch_entry::update SUCCESS" << std::endl;
        // }
        return true;
      }
      // if (kDebug) {
      //   std::cout << "DEBUG: ch_entry::update FAILED - cost not better" << std::endl;
      // }
      return false;
    }

    static constexpr std::size_t get_index(node const n) {
      return (n.dir_ == direction::kForward ? 0U : 1U) * kMaxWays + n.way_;
    }

    static constexpr direction to_dir(bool const b) {
      return b == false ? direction::kForward : direction::kBackward;
    }

    static constexpr bool to_bool(direction const d) {
      return d == direction::kForward ? false : true;
    }

    std::array<node_idx_t, kN> pred_;
    std::array<way_pos_t, kN> pred_way_;
    std::bitset<kN> pred_dir_;
    std::array<internal_cost_t, kN> cost_;
  };

  void reset(cost_t const max) {
    pq_forward_.clear();
    pq_backward_.clear();
    // Allocate more buckets for CH - must be large enough for all possible costs
    // Use a minimum of 100000 buckets to handle shortcuts
    auto const ch_max = std::max(static_cast<internal_cost_t>(100000), 
                                  static_cast<internal_cost_t>(max) * 20U);
    pq_forward_.n_buckets(ch_max + 1U);
    pq_backward_.n_buckets(ch_max + 1U);
    if (kDebug) {
      std::cout << "CH: Allocated " << (ch_max + 1U) << " buckets for each dial queue\n";
    }
    forward_costs_.clear();
    backward_costs_.clear();
    initial_forward_nodes_.clear();
    initial_backward_nodes_.clear();
    best_cost_ = std::numeric_limits<internal_cost_t>::max();
    meet_point_ = node_idx_t::invalid();
    forward_meet_node_ = node::invalid();
    backward_meet_node_ = node::invalid();
  }

  void add_start(ways const& w, label const l, ch_data const* ch) {
    initial_forward_nodes_.insert(l.get_node().get_key());
    auto const internal_cost = static_cast<internal_cost_t>(l.cost());
    if (kDebug) {
      std::cout << "CH: add_start called for node " << l.get_node().n_ << " cost=" << internal_cost << std::endl;
    }
    if (forward_costs_[l.get_node().get_key()].update(
            l, l.get_node(), internal_cost, node::invalid())) {
      pq_forward_.push(ch_label{l.get_node(), internal_cost});
      if (kDebug) {
        std::cout << "CH: Successfully added start node to forward PQ" << std::endl;
      }
      if (kDebug) {
        car_ch_key start_key{l.get_node().n_, l.get_node().way_, l.get_node().dir_};
        auto start_level = ch ? ch->get_level(start_key) : 0;
        std::cout << "CH START FWD ";
        l.get_node().print(std::cout, w);
        std::cout << " cost=" << internal_cost << " level=" << start_level;
        std::cout << " [LOOKUP KEY: n=" << start_key.n_.v_ << " way=" << start_key.way_ 
                  << " dir=" << (start_key.dir_ == direction::kForward ? "fwd" : "bwd") << "]";
        if (start_level == std::numeric_limits<ch_level_t>::max()) {
          std::cout << " KEY NOT FOUND IN CH DATA!";
        }
        std::cout << "\n";
      }
    }
  }

  void add_end(ways const& w, label const l, ch_data const* ch) {
    initial_backward_nodes_.insert(l.get_node().get_key());
    auto const internal_cost = static_cast<internal_cost_t>(l.cost());
    if (kDebug) {
      std::cout << "CH: add_end called for node " << l.get_node().n_ << " cost=" << internal_cost << std::endl;
    }
    if (backward_costs_[l.get_node().get_key()].update(
            l, l.get_node(), internal_cost, node::invalid())) {
      pq_backward_.push(ch_label{l.get_node(), internal_cost});
      if (kDebug) {
        std::cout << "CH: Successfully added end node to backward PQ" << std::endl;
      }
      if (kDebug) {
        car_ch_key end_key{l.get_node().n_, l.get_node().way_, l.get_node().dir_};
        auto end_level = ch ? ch->get_level(end_key) : 0;
        std::cout << "CH START BWD ";
        l.get_node().print(std::cout, w);
        std::cout << " cost=" << internal_cost << " level=" << end_level;
        std::cout << " [LOOKUP KEY: n=" << end_key.n_.v_ << " way=" << end_key.way_ 
                  << " dir=" << (end_key.dir_ == direction::kForward ? "fwd" : "bwd") << "]";
        if (end_level == std::numeric_limits<ch_level_t>::max()) {
          std::cout << " KEY NOT FOUND IN CH DATA!";
        }
        std::cout << "\n";
      }
    }
  }

  template <bool WithBlocked>
  bool run(ways const& w,
           ways::routing const& r,
           cost_t const max,
           bitvec<node_idx_t> const* blocked,
           ch_data const* ch,
           sharing_data const* sharing,
           elevation_storage const* elevations) {
    
    size_t forward_processed = 0;
    size_t backward_processed = 0;
    size_t forward_filtered = 0;
    size_t backward_filtered = 0;
    
    if (kDebug) {
      std::cout << "CH: Starting run() with forward PQ size: " << pq_forward_.size() 
                << ", backward PQ size: " << pq_backward_.size() << std::endl;
    }
    
    // Interleaved bidirectional search like bidirectional.h
    while (!pq_forward_.empty() || !pq_backward_.empty()) {
      
      bool forward_step_ok = true;
      bool backward_step_ok = true;
      
      // Forward search step
      if (!pq_forward_.empty()) {
        if (kDebug) {
          std::cout << "CH: Running forward step, PQ size: " << pq_forward_.size() << std::endl;
        }
        forward_step_ok = run_single_step<true, WithBlocked>(w, r, max, blocked, ch, sharing, elevations,
                                                            forward_processed, forward_filtered);
      }
      
      // Backward search step  
      if (!pq_backward_.empty()) {
        if (kDebug) {
          std::cout << "CH: Running backward step, PQ size: " << pq_backward_.size() << std::endl;
        }
        backward_step_ok = run_single_step<false, WithBlocked>(w, r, max, blocked, ch, sharing, elevations,
                                                              backward_processed, backward_filtered);
      }
      
      // Check termination condition after both steps according to CH theory
      // Terminate when both queue minimums exceed best cost
      if (best_cost_ != std::numeric_limits<internal_cost_t>::max() &&
          min_forward_cost_ > best_cost_ && min_backward_cost_ > best_cost_) {
        if (kDebug) {
          std::cout << "CH: Terminating - both searches exceed best_cost: " 
                   << "fwd_min=" << min_forward_cost_ << ", bwd_min=" << min_backward_cost_ 
                   << ", best=" << best_cost_ << std::endl;
        }
        break;
      }
      
      // Also terminate if either search explicitly requests termination
      if (!forward_step_ok || !backward_step_ok) {
        if (kDebug) {
          std::cout << "CH: Terminating - search step returned false" << std::endl;
        }
        break;
      }
    }

    // Always log search completion for analysis
    bool success = best_cost_ != std::numeric_limits<internal_cost_t>::max();
    std::cout << "CH SEARCH " << (success ? "SUCCESS" : "FAILED") << ":\n";
    std::cout << "  Forward processed: " << forward_processed << ", filtered: " << forward_filtered << "\n";
    std::cout << "  Backward processed: " << backward_processed << ", filtered: " << backward_filtered << "\n";
    std::cout << "  Best cost: " << (success ? std::to_string(best_cost_) : "NONE") << "\n";
    if (success) {
      std::cout << "  Meet point: " << meet_point_.v_ << "\n";
    } else {
      // Analyze failure reasons
      std::cout << "  Failure analysis:\n";
      std::cout << "    Final forward min cost: " << min_forward_cost_ << "\n";
      std::cout << "    Final backward min cost: " << min_backward_cost_ << "\n";
      std::cout << "    Forward queue empty: " << (pq_forward_.empty() ? "YES" : "NO") << "\n";
      std::cout << "    Backward queue empty: " << (pq_backward_.empty() ? "YES" : "NO") << "\n";
      
      if (forward_filtered > forward_processed * 2) {
        std::cout << "    High forward filtering ratio suggests level filtering issues\n";
      }
      if (backward_filtered > backward_processed * 2) {
        std::cout << "    High backward filtering ratio suggests level filtering issues\n";
      }
    }

    return best_cost_ != std::numeric_limits<internal_cost_t>::max();
  }

  template <bool IsForward, bool WithBlocked>
  bool run_single_step(ways const& w,
                       ways::routing const& r,
                       cost_t const max,
                       bitvec<node_idx_t> const* blocked,
                       ch_data const* ch,
                       sharing_data const* sharing,
                       elevation_storage const* elevations,
                       size_t& processed_count,
                       size_t& filtered_count) {
    
    auto& pq = IsForward ? pq_forward_ : pq_backward_;
    auto& curr_costs = IsForward ? forward_costs_ : backward_costs_;
    auto& other_costs = IsForward ? backward_costs_ : forward_costs_;
    
    auto const l = pq.pop();
    auto const actual_cost = l.cost();
    auto const curr = l.get_node();
    
    if (kDebug) {
      std::cout << "CH: Processing node " << curr.n_ << " cost=" << actual_cost 
                << " direction=" << (IsForward ? "FWD" : "BWD") << std::endl;
    }
    
    // Skip if we've found a better path to this node already
    if (curr_costs[curr.get_key()].cost(curr) < actual_cost) {
      return true; // Continue search
    }
    
    processed_count++;
    
    // Update minimum cost tracking for termination
    if constexpr (IsForward) {
      min_forward_cost_ = actual_cost;
    } else {
      min_backward_cost_ = actual_cost;
    }
    
    // Check for meetpoints with the other search
    check_meetpoint(w, curr, actual_cost, other_costs, ch, IsForward);
    
    // Termination is now handled at the main loop level for symmetric checking
    
    // Expand current node
    if constexpr (IsForward) {
      expand_forward<WithBlocked>(w, r, curr, actual_cost, max, blocked, ch, 
                                  sharing, elevations, filtered_count);
    } else {
      expand_backward<WithBlocked>(w, r, curr, actual_cost, max, blocked, ch,
                                   sharing, elevations, filtered_count);
    }
    
    return true; // Continue search
  }

  template <bool WithBlocked>
  void expand_forward(ways const& w,
                      ways::routing const& r,
                      node const curr,
                      internal_cost_t curr_cost,
                      cost_t max,
                      bitvec<node_idx_t> const* blocked,
                      ch_data const* ch,
                      sharing_data const* sharing,
                      elevation_storage const* elevations,
                      size_t& filtered_count) {
    
    car::template adjacent<direction::kForward, WithBlocked>(
        r, curr, blocked, sharing, elevations,
        [&](node const neighbor, std::uint32_t const cost, distance_t,
            way_idx_t const way, std::uint16_t, std::uint16_t,
            elevation_storage::elevation const, bool const) {
          
          // Level filtering: forward search with upward requirement
          if (ch) {
            car_ch_key curr_key{curr.n_, curr.way_, curr.dir_};
            car_ch_key neighbor_key{neighbor.n_, neighbor.way_, neighbor.dir_};
            auto curr_level = ch->get_level(curr_key);
            auto neighbor_level = ch->get_level(neighbor_key);
            
            // Forward search: traverse upward edges (lower to higher level)
            bool allow_edge = curr_level < neighbor_level;
            
            if (!allow_edge) {
              filtered_count++;
              if (kDebug) {
                std::cout << "  FWD FILTERED edge (" << curr.n_ << "," << curr.way_ << ")->(" 
                         << neighbor.n_ << "," << neighbor.way_ << ") (level " 
                         << curr_level << " -> " << neighbor_level << ")\n";
              }
              return;
            }
          }
          
          process_edge(w, curr, neighbor, static_cast<internal_cost_t>(curr_cost + cost), max, true);
        });

    // Process shortcuts from current car state
    if (ch) {
      car_ch_key curr_key{curr.n_, curr.way_, curr.dir_};
      auto const* shortcuts = ch->get_forward_shortcuts(curr_key);
      if (shortcuts) {
        if (kDebug && !shortcuts->empty()) {
          std::cout << "  FWD SHORTCUTS available: " << shortcuts->size() << " for node " << curr.n_ << "\n";
        }
        for (auto const& sc : *shortcuts) {
          // Level filtering for shortcuts: only traverse upward edges
          auto curr_level = ch->get_level(curr_key);
          auto to_level = ch->get_level(sc.to_);
          if (curr_level < to_level) {
            if (kDebug) {
              std::cout << "  SHORTCUT (" << curr.n_ << "," << curr.way_ << ")->(" 
                       << sc.to_.n_ << "," << sc.to_.way_ << ") cost=" << sc.cost_ 
                       << " (level " << ch->get_level(curr_key) << " -> " 
                       << ch->get_level(sc.to_) << ")\n";
            }
            // Create proper target car node from shortcut
            auto target = node{sc.to_.n_, sc.to_.way_, sc.to_.dir_};
            process_edge(w, curr, target, static_cast<internal_cost_t>(curr_cost + sc.cost_), max, true);
          } else if (kDebug) {
            std::cout << "  FWD SHORTCUT FILTERED (" << curr.n_ << "," << curr.way_ << ")->(" 
                     << sc.to_.n_ << "," << sc.to_.way_ << ") (level " 
                     << ch->get_level(curr_key) << " -> " << ch->get_level(sc.to_) << ")\n";
          }
        }
      } else if (kDebug) {
        std::cout << "  FWD NO SHORTCUTS for node " << curr.n_ << "\n";
      }
    }
  }

  template <bool WithBlocked>
  void expand_backward(ways const& w,
                       ways::routing const& r,
                       node const curr,
                       internal_cost_t curr_cost,
                       cost_t max,
                       bitvec<node_idx_t> const* blocked,
                       ch_data const* ch,
                       sharing_data const* sharing,
                       elevation_storage const* elevations,
                       size_t& filtered_count) {
    
    car::template adjacent<direction::kBackward, WithBlocked>(
        r, curr, blocked, sharing, elevations,
        [&](node const neighbor, std::uint32_t const cost, distance_t,
            way_idx_t const way, std::uint16_t, std::uint16_t,
            elevation_storage::elevation const, bool const) {
          
          // Level filtering: backward search with downward/lateral requirement
          if (ch) {
            car_ch_key curr_key{curr.n_, curr.way_, curr.dir_};
            car_ch_key neighbor_key{neighbor.n_, neighbor.way_, neighbor.dir_};
            auto curr_level = ch->get_level(curr_key);
            auto neighbor_level = ch->get_level(neighbor_key);
            
            // Backward search: traverse downward and lateral edges
            // Allow edges where curr_level >= neighbor_level (includes lateral edges)
            bool allow_edge = curr_level >= neighbor_level;
            
            if (!allow_edge) {
              filtered_count++;
              if (kDebug) {
                std::cout << "  BWD FILTERED edge (" << neighbor.n_ << "," << neighbor.way_ << ")->(" 
                         << curr.n_ << "," << curr.way_ << ") (level " 
                         << neighbor_level << " -> " << curr_level << ")\n";
              }
              return;
            }
          }
          
          process_edge(w, curr, neighbor, static_cast<internal_cost_t>(curr_cost + cost), max, false);
        });

    // Process shortcuts from current car state
    if (ch) {
      car_ch_key curr_key{curr.n_, curr.way_, curr.dir_};
      auto const* shortcuts = ch->get_backward_shortcuts(curr_key);
      if (shortcuts) {
        for (auto const& sc : *shortcuts) {
          // Level filtering for shortcuts: backward search traverses upward edges in reverse
          auto curr_level = ch->get_level(curr_key);
          auto from_level = ch->get_level(sc.from_);
          if (curr_level > from_level) {
            if (kDebug) {
              std::cout << "  SHORTCUT (" << curr.n_ << "," << curr.way_ << ")->(" 
                       << sc.from_.n_ << "," << sc.from_.way_ << ") cost=" << sc.cost_ 
                       << " (level " << ch->get_level(curr_key) << " -> " 
                       << ch->get_level(sc.from_) << ")\n";
            }
            // Create proper target car node from shortcut
            auto target = node{sc.from_.n_, sc.from_.way_, sc.from_.dir_};
            process_edge(w, curr, target, static_cast<internal_cost_t>(curr_cost + sc.cost_), max, false);
          }
        }
      }
    }
  }

  void process_edge(ways const& w,
                   node const curr,
                   node const neighbor,
                   internal_cost_t new_cost,
                   cost_t max,
                   bool is_forward) {
    if (new_cost >= static_cast<internal_cost_t>(max)) {
      return;
    }

    auto& costs = is_forward ? forward_costs_ : backward_costs_;
    auto& pq = is_forward ? pq_forward_ : pq_backward_;
    
    if (costs[neighbor.get_key()].update(label{neighbor, static_cast<cost_t>(std::min(new_cost, static_cast<internal_cost_t>(std::numeric_limits<cost_t>::max())))}, 
                                         neighbor, new_cost, curr)) {
      auto next = ch_label{neighbor, new_cost};
      pq.push(std::move(next));
      
      if (kDebug) {
        std::cout << "  PUSH " << (is_forward ? "FWD" : "BWD") << " ";
        neighbor.print(std::cout, w);
        std::cout << " cost=" << new_cost << "\n";
      }
    }
  }

  void check_meetpoint(ways const& w,
                      node const curr,
                      internal_cost_t curr_cost,
                      ankerl::unordered_dense::map<key, ch_entry, hash> const& other_costs,
                      ch_data const* ch,
                      bool is_forward_search) {
    
    auto const it = other_costs.find(curr.get_key());
    if (it == other_costs.end()) {
      // Too verbose for normal debugging
      return;
    }

    // Check the exact current node representation for meetpoint
    auto const other_cost = it->second.cost(curr);
    
    if (other_cost != std::numeric_limits<internal_cost_t>::max()) {
      auto const total_cost = curr_cost + other_cost;
      
      // Reject zero-cost meetpoints from initial nodes (they haven't been properly explored)
      bool is_valid_meetpoint = true;
      if (total_cost == 0 && 
          (initial_forward_nodes_.contains(curr.get_key()) || 
           initial_backward_nodes_.contains(curr.get_key()))) {
        if (kDebug) {
          std::cout << "  REJECTED zero-cost initial node meetpoint at " << curr.n_ 
                   << " cost=" << total_cost << "\n";
        }
        is_valid_meetpoint = false;
      }
      
      if (is_valid_meetpoint && total_cost < best_cost_) {
        best_cost_ = total_cost;
        meet_point_ = curr.n_;
        
        if (is_forward_search) {
          forward_meet_node_ = curr;
          backward_meet_node_ = curr;
        } else {
          forward_meet_node_ = curr;
          backward_meet_node_ = curr;
        }
        
        std::cout << "*** MEETPOINT FOUND! Node " << curr.n_ 
                 << " cost=" << best_cost_ 
                 << " (curr_cost=" << curr_cost 
                 << " + other_cost=" << other_cost << ")" << std::endl;
        return;
      } else if (total_cost >= best_cost_) {
        if (kDebug) {
          std::cout << "DEBUG: REJECTED DIRECT meetpoint at node " << curr.n_
                   << " cost=" << total_cost << " >= best_cost=" << best_cost_ << std::endl;
        }
      }
    }
    
    // Handle end-of-way meetpoints - critical for car profile turn legality
    handle_end_of_way_meetpoint(w, curr, curr_cost, other_costs, ch, is_forward_search);
  }
  
  void handle_end_of_way_meetpoint(ways const& w,
                                   node const curr,
                                   internal_cost_t curr_cost,
                                   ankerl::unordered_dense::map<key, ch_entry, hash> const& other_costs,
                                   ch_data const* ch,
                                   bool is_forward_search) {
    if (kDebug) {
      std::cout << "DEBUG: handle_end_of_way_meetpoint called for node " << curr.n_ 
               << " way=" << curr.way_ << " dir=" << (curr.dir_ == direction::kForward ? "fwd" : "bwd")
               << " cost=" << curr_cost << std::endl;
    }
    
    auto& costs = is_forward_search ? forward_costs_ : backward_costs_;
    auto const opposite_cost_map = is_forward_search ? &backward_costs_ : &forward_costs_;
    
    auto const opposite_candidate = opposite_cost_map->find(curr.get_key());
    if (opposite_candidate != opposite_cost_map->end()) {
      auto const other_cost = opposite_candidate->second.cost(curr);
      if (other_cost != std::numeric_limits<internal_cost_t>::max()) {
        // Simple direct meetpoint already handled above
        if (kDebug) {
          std::cout << "DEBUG: Direct meetpoint already handled, skipping end-of-way logic" << std::endl;
        }
        return;
      } else {
        if (kDebug) {
          std::cout << "DEBUG: Found opposite candidate but no direct cost, exploring end-of-way connections" << std::endl;
        }
        auto const pred_it = costs.find(curr.get_key());
        if (pred_it == costs.end()) {
          return;
        }
        auto const pred = pred_it->second.pred(curr);
        if (!pred.has_value()) {
          return;
        }
        
        // Explore opposite direction adjacency to find valid car connections
        auto const opposite_dir = is_forward_search ? direction::kBackward : direction::kForward;
        
        if (is_forward_search) {
          car::template adjacent<direction::kBackward, false>(
              *w.r_, curr, nullptr, nullptr, nullptr,
              [&](node const neighbor, std::uint32_t const, distance_t,
                  way_idx_t const, std::uint16_t, std::uint16_t,
                  elevation_storage::elevation const, bool const) {
                evaluate_end_of_way_connection(neighbor, curr, curr_cost, opposite_candidate, opposite_cost_map, is_forward_search);
              });
        } else {
          car::template adjacent<direction::kForward, false>(
              *w.r_, curr, nullptr, nullptr, nullptr,
              [&](node const neighbor, std::uint32_t const, distance_t,
                  way_idx_t const, std::uint16_t, std::uint16_t,
                  elevation_storage::elevation const, bool const) {
                evaluate_end_of_way_connection(neighbor, curr, curr_cost, opposite_candidate, opposite_cost_map, is_forward_search);
              });
        }
      }
    }
  }

  void evaluate_end_of_way_connection(node const neighbor,
                                      node const curr,
                                      internal_cost_t curr_cost,
                                      ankerl::unordered_dense::map<key, ch_entry, hash>::const_iterator const& opposite_candidate,
                                      ankerl::unordered_dense::map<key, ch_entry, hash> const* opposite_cost_map,
                                      bool is_forward_search) {
    auto& costs = is_forward_search ? forward_costs_ : backward_costs_;
    auto const pred_it = costs.find(curr.get_key());
    if (pred_it == costs.end()) {
      return;
    }
    auto const pred = pred_it->second.pred(curr);
    if (!pred.has_value() || neighbor.get_key() != pred->get_key()) {
      return;
    }
    
    auto const opposite_it = opposite_cost_map->find(neighbor.get_key());
    if (opposite_it == opposite_cost_map->end()) {
      return;
    }
    
    auto const opposite_curr = opposite_it->second.pred(neighbor);
    if (!opposite_curr.has_value() || opposite_curr->get_key() != curr.get_key()) {
      return;
    }
    
    // Found valid car connection - evaluate meetpoint
    auto const total_cost = curr_cost + opposite_candidate->second.cost(*opposite_curr);
    if (total_cost < best_cost_) {
      best_cost_ = total_cost;
      meet_point_ = curr.n_;
      
      if (is_forward_search) {
        forward_meet_node_ = curr;
        backward_meet_node_ = *opposite_curr;
      } else {
        forward_meet_node_ = *opposite_curr;
        backward_meet_node_ = curr;
      }
      
      if (kDebug) {
        std::cout << "DEBUG: END-OF-WAY meetpoint at node " << curr.n_ 
                 << " way=" << curr.way_ << " dir=" << (curr.dir_ == direction::kForward ? "fwd" : "bwd")
                 << " cost=" << best_cost_ 
                 << " (curr_cost=" << curr_cost 
                 << " + other_cost=" << opposite_candidate->second.cost(*opposite_curr) << ")\n";
      }
    }
  }

  internal_cost_t get_cost(node const n, bool is_forward) const {
    auto const& costs = is_forward ? forward_costs_ : backward_costs_;
    auto const it = costs.find(n.get_key());
    return it != costs.end() ? it->second.cost(n) : std::numeric_limits<internal_cost_t>::max();
  }

  // No longer need directional bit masking with dual PQ architecture

  // Use dual priority queues like bidirectional.h for proper CH search
  dial<ch_label, get_bucket> pq_forward_{get_bucket{}};
  dial<ch_label, get_bucket> pq_backward_{get_bucket{}};
  ankerl::unordered_dense::map<key, ch_entry, hash> forward_costs_;
  ankerl::unordered_dense::map<key, ch_entry, hash> backward_costs_;
  ankerl::unordered_dense::set<key> initial_forward_nodes_;
  ankerl::unordered_dense::set<key> initial_backward_nodes_;
  internal_cost_t best_cost_{std::numeric_limits<internal_cost_t>::max()};
  node_idx_t meet_point_{node_idx_t::invalid()};
  node forward_meet_node_{node::invalid()};
  node backward_meet_node_{node::invalid()};
  
  // Track minimum costs for termination condition
  internal_cost_t min_forward_cost_{0};
  internal_cost_t min_backward_cost_{0};
};

}  // namespace osr