#pragma once

#include <algorithm>
#include <iostream>

#include "osr/elevation_storage.h"
#include "osr/location.h"
#include "osr/routing/ch_levels.h"
#include "osr/routing/ch_search.h"
#include "osr/routing/ch_shortcut.h"
#include "osr/routing/profiles/car.h"
#include "osr/routing/sharing_data.h"
#include "osr/types.h"
#include "osr/ways.h"

namespace osr {

/**
 * Bidirectional CH Dijkstra search.
 * Runs forward and backward CH searches simultaneously with abort-on-success termination.
 * Both directions use upward edges (level filtering) as per CH theory.
 */
template <typename Profile>
class ch_bidirectional {
public:
  using profile_t = Profile;
  using key = typename Profile::key;
  using label = typename Profile::label;
  using node = typename Profile::node;
  using entry = typename Profile::entry;
  using hash = typename Profile::hash;

  explicit ch_bidirectional(ch_levels const& levels, ch_shortcuts const& shortcuts)
      : levels_(levels), shortcuts_(shortcuts), 
        forward_search_(levels, shortcuts), 
        backward_search_(levels, shortcuts) {}

  /**
   * Initialize bidirectional search.
   */
  void init(cost_t const max_cost, location const& start_loc, location const& end_loc) {
    forward_search_.init(max_cost);
    backward_search_.init(max_cost);
    
    start_loc_ = start_loc;
    end_loc_ = end_loc;
    
    // Reset meeting point state
    clear_meeting_point();
    search_finished_ = false;
  }

  /**
   * Add starting nodes to forward search.
   */
  void add_forward_start(node const& start_node, cost_t const initial_cost = cost_t{0}) {
    forward_search_.add_start(start_node, initial_cost);
  }

  /**
   * Add starting nodes to backward search.
   */
  void add_backward_start(node const& end_node, cost_t const initial_cost = cost_t{0}) {
    backward_search_.add_start(end_node, initial_cost);
  }

  /**
   * Run complete bidirectional search until termination.
   */
  template <bool WithBlocked = false>
  bool run_search(ways const& w,
                  cost_t const max_cost,
                  bitvec<node_idx_t> const* blocked = nullptr,
                  sharing_data const* sharing = nullptr,
                  elevation_storage const* elevations = nullptr) {
    
    while (!should_terminate()) {
      bool expanded_any = false;

      // Alternate between forward and backward expansion
      if (forward_search_.has_next()) {
        auto const expanded = forward_search_.expand_step<direction::kForward, WithBlocked>(
            w, max_cost, blocked, sharing, elevations);
        
        if (expanded != node::invalid()) {
          expanded_any = true;
          check_meeting_point(expanded, direction::kForward);
        }
      }

      if (backward_search_.has_next() && !should_terminate()) {
        auto const expanded = backward_search_.expand_step<direction::kBackward, WithBlocked>(
            w, max_cost, blocked, sharing, elevations);
        
        if (expanded != node::invalid()) {
          expanded_any = true;
          check_meeting_point(expanded, direction::kBackward);
        }
      }

      if (!expanded_any) {
        break; // Both searches exhausted
      }
    }

    search_finished_ = true;
    
    // Show search summary
    auto const fwd_explored = forward_search_.get_distances().size();
    auto const bwd_explored = backward_search_.get_distances().size();
    bool const found_path = tentative_shortest_path_ != kInfeasible;
    
    std::cout << "SEARCH SUMMARY - Fwd: " << fwd_explored 
              << " nodes, Bwd: " << bwd_explored << " nodes, "
              << (found_path ? "PATH FOUND" : "NO PATH") << "\n";
    
    // if (fwd_explored > 0 || bwd_explored > 0) {  // Show all searches
    //   std::cout << "CH Search Debug - Fwd: " << fwd_explored 
    //             << " nodes, Bwd: " << bwd_explored << " nodes, "
    //             << "Path: " << (found_path ? "FOUND" : "NO_RESULT") 
    //             << (found_path ? " cost=" + std::to_string(tentative_shortest_path_) : "") << "\n";
    // }
    
    return found_path;
  }

  /**
   * Get the shortest path cost found, or kInfeasible if no path exists.
   */
  cost_t get_shortest_path_cost() const {
    return tentative_shortest_path_;
  }

  /**
   * Get the meeting point nodes (forward and backward).
   */
  std::pair<node, node> get_meeting_point() const {
    return {forward_meeting_node_, backward_meeting_node_};
  }

  /**
   * Check if search has found a path.
   */
  bool has_path() const {
    return tentative_shortest_path_ != kInfeasible;
  }

  /**
   * Check if search is finished.
   */
  bool is_finished() const {
    return search_finished_;
  }

  /**
   * Get distances from forward search.
   */
  auto const& get_forward_distances() const {
    return forward_search_.get_distances();
  }

  /**
   * Get distances from backward search.
   */
  auto const& get_backward_distances() const {
    return backward_search_.get_distances();
  }

private:
  /**
   * Clear meeting point state.
   */
  void clear_meeting_point() {
    forward_meeting_node_ = node::invalid();
    backward_meeting_node_ = node::invalid();
    tentative_shortest_path_ = kInfeasible;
  }

  /**
   * Check if a newly expanded node creates a meeting point.
   * Uses car::node granularity to respect turn restrictions and U-turn penalties.
   */
  void check_meeting_point(node const& expanded_node, direction const search_dir) {
    // Validate inputs
    if (expanded_node == node::invalid()) {
      return; // Invalid node - skip
    }
    
    // Get cost from the search that just expanded this node
    auto const expanded_cost = (search_dir == direction::kForward) 
        ? forward_search_.get_cost(expanded_node)
        : backward_search_.get_cost(expanded_node);

    // Skip if we don't have a valid cost for the expanded node
    if (expanded_cost == kInfeasible) {
      return;
    }

    // Check if the other search has also reached this exact car::node
    auto const other_cost = (search_dir == direction::kForward)
        ? backward_search_.get_cost(expanded_node)
        : forward_search_.get_cost(expanded_node);
    
    // Debug output for meeting point checks
    std::cout << "Meeting check: node " << expanded_node.get_node().v_ 
              << " dir=" << (search_dir == direction::kForward ? "F" : "B")
              << " expanded_cost=" << expanded_cost 
              << " other_cost=" << (other_cost == kInfeasible ? -1 : other_cost) << "\n";

    if (other_cost != kInfeasible) {
      // Both searches have reached this exact car::node - direct meeting point
      auto const forward_cost = (search_dir == direction::kForward) ? expanded_cost : other_cost;
      auto const backward_cost = (search_dir == direction::kForward) ? other_cost : expanded_cost;
      
      std::cout << "MEETING POINT FOUND at node " << expanded_node.get_node().v_ 
                << " fwd_cost=" << forward_cost << " bwd_cost=" << backward_cost << "\n";
      evaluate_direct_meetpoint(expanded_node, forward_cost, backward_cost, search_dir);
    }
    
    // Also check for end-of-way meeting points (like in bidirectional A*)
    // This handles cases where searches meet on the same node_idx_t but different car::node
    check_end_of_way_meetpoint(expanded_node, expanded_cost, search_dir);
  }

private:
  /**
   * Evaluate a direct meeting point where both searches reached the same car::node.
   */
  void evaluate_direct_meetpoint(node const& meeting_node, 
                                 cost_t const forward_cost, cost_t const backward_cost,
                                 direction const search_dir) {
    auto const total_cost = forward_cost + backward_cost;
    
    std::cout << "Evaluating meeting point: total_cost=" << total_cost 
              << " current_best=" << tentative_shortest_path_ << "\n";
    
    if (total_cost < tentative_shortest_path_) {
      tentative_shortest_path_ = total_cost;
      forward_meeting_node_ = meeting_node;
      backward_meeting_node_ = meeting_node;
      
      std::cout << "NEW BEST PATH found via node " << meeting_node.get_node().v_ 
                << " cost=" << total_cost << "\n";
    }
  }

  /**
   * Check for end-of-way meeting points where searches meet on same node_idx_t 
   * but potentially different car::node instances.
   */
  void check_end_of_way_meetpoint(node const& expanded_node, cost_t const expanded_cost,
                                  direction const search_dir) {
    // Get the opposite search's distances
    auto const& opposite_distances = (search_dir == direction::kForward)
        ? backward_search_.get_distances()
        : forward_search_.get_distances();
        
    // Only log if opposite search has found nodes
    if (opposite_distances.size() > 0) {
      std::cout << "End-of-way check: node " << expanded_node.get_node().v_ 
                << " vs " << opposite_distances.size() << " opposite nodes\\n";
    }

    // Look for any car::node on the same node_idx_t that the opposite search reached
    auto const target_node_idx = expanded_node.get_node();
    
    for (auto const& [key, entry] : opposite_distances) {
      // Check if this entry corresponds to the same node_idx_t
      // Handle different key types across profiles
      node_idx_t key_node_idx;
      
      if constexpr (std::is_same_v<typename Profile::key, node_idx_t>) {
        // Key is directly node_idx_t (car, car_parking, bike)
        key_node_idx = key;
      } else if constexpr (requires { key.n_; }) {
        // Key has .n_ member (car_sharing) 
        key_node_idx = key.n_;
      } else if constexpr (requires { key.get_node(); }) {
        // Key is a node with get_node() method (foot)
        key_node_idx = key.get_node();
      } else {
        continue; // Skip unknown key types
      }
      
      if (key_node_idx == target_node_idx) {
        // Found a potential meeting point on the same node_idx_t
        // Try to get cost for any car::node on this node_idx_t
        auto other_cost = kInfeasible;
        node other_meeting_node = node::invalid();
        
        // Check if the entry is compatible with our expanded node
        other_cost = entry.cost(expanded_node);
        
        if (other_cost == kInfeasible) {
          // The expanded_node car::node doesn't match the entry - try to find a compatible one
          if constexpr (std::is_same_v<typename Profile::key, node_idx_t>) {
            std::cout << "Entry scan for node " << key_node_idx.v_ 
                      << " (expanded from dir " << (search_dir == direction::kForward ? "F" : "B") << ")\\n";
            
            // For car profile, try to find any car::node on this node_idx_t that has a valid cost
            // We'll iterate through the entry to find the minimum valid cost
            auto min_cost = kInfeasible;
            int valid_costs = 0;
            for (auto const& cost_val : entry.cost_) {
              if (cost_val < min_cost) {
                min_cost = cost_val;
              }
              if (cost_val != kInfeasible) {
                valid_costs++;
              }
            }
            
            std::cout << "  Entry has " << valid_costs << " valid costs, min=" << min_cost << "\\n";
            
            if (min_cost != kInfeasible) {
              other_cost = min_cost;
              // Create a basic node for this node_idx_t - use expanded_node as template but with different node_idx_t
              other_meeting_node = expanded_node;  // Copy structure
              other_meeting_node.n_ = key;         // Replace node_idx_t
              
              std::cout << "Found compatible meeting via entry scan: node " << key_node_idx.v_ 
                        << " cost=" << other_cost << "\\n";
            } else {
              std::cout << "  No valid costs in entry, skipping\\n";
              continue; // No valid cost found in this entry
            }
          } else {
            // For other profiles, skip this optimization for now
            continue;
          }
        } else {
          other_meeting_node = expanded_node;
        }
        
        if (other_cost != kInfeasible) {
          auto const total_cost = expanded_cost + other_cost;
          
          std::cout << "End-of-way meeting candidate: node " << key_node_idx.v_ 
                    << " total_cost=" << total_cost << " current_best=" << tentative_shortest_path_ << "\n";
          
          if (total_cost < tentative_shortest_path_) {
            tentative_shortest_path_ = total_cost;
            
            if (search_dir == direction::kForward) {
              forward_meeting_node_ = expanded_node;
              backward_meeting_node_ = other_meeting_node;
            } else {
              forward_meeting_node_ = other_meeting_node;
              backward_meeting_node_ = expanded_node;
            }
            
            std::cout << "NEW BEST END-OF-WAY PATH found at node " << key_node_idx.v_ 
                      << " cost=" << total_cost << "\n";
          }
        }
      }
    }
  }

  /**
   * Check if search should terminate using abort-on-success criterion.
   * Terminates when both search frontiers exceed the tentative shortest path.
   */
  bool should_terminate() const {
    if (tentative_shortest_path_ == kInfeasible) {
      return false; // No meeting point found yet
    }

    // Check if both searches are exhausted
    auto const forward_finished = !forward_search_.has_next();
    auto const backward_finished = !backward_search_.has_next();

    if (forward_finished && backward_finished) {
      return true; // Both searches completely exhausted
    }

    // Apply abort-on-success criterion:
    // Terminate when both search frontiers have minimum costs exceeding tentative shortest path
    auto const forward_next_cost = forward_search_.get_next_cost();
    auto const backward_next_cost = backward_search_.get_next_cost();

    // If either search is finished, only check the other
    if (forward_finished) {
      return backward_next_cost >= tentative_shortest_path_;
    }
    if (backward_finished) {
      return forward_next_cost >= tentative_shortest_path_;
    }

    // Both searches are active - check both frontiers
    return forward_next_cost >= tentative_shortest_path_ && 
           backward_next_cost >= tentative_shortest_path_;
  }

private:
  ch_levels const& levels_;
  ch_shortcuts const& shortcuts_;

  // Individual search instances
  ch_search<Profile> forward_search_;
  ch_search<Profile> backward_search_;

  // Search state
  location start_loc_;
  location end_loc_;

  // Meeting point tracking
  node forward_meeting_node_{node::invalid()};
  node backward_meeting_node_{node::invalid()};
  cost_t tentative_shortest_path_{kInfeasible};

  bool search_finished_{false};
};

}  // namespace osr