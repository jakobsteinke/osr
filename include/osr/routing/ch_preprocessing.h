#pragma once

#include <algorithm>
#include <limits>
#include <queue>
#include <random>
#include <vector>

#include "osr/routing/ch_data.h"
#include "osr/routing/dijkstra.h"
#include "osr/routing/profiles/car.h"
#include "osr/types.h"
#include "osr/ways.h"

namespace osr {

struct ch_preprocessing {
  static constexpr bool kDebug = true;
  
  struct witness_search_entry {
    node_idx_t node_;
    cost_t cost_;
    
    bool operator>(witness_search_entry const& other) const {
      return cost_ > other.cost_;
    }
  };
  
  static ch_data preprocess(ways const& w) {
    ch_data ch;
    
    auto const num_nodes = w.n_nodes();
    
    std::vector<node_idx_t> contraction_order(num_nodes);
    for (auto i = 0U; i < num_nodes; ++i) {
      contraction_order[i] = node_idx_t{i};
    }
    std::mt19937 gen(42);
    std::shuffle(contraction_order.begin(), contraction_order.end(), gen);
    
    for (ch_level_t level = 0; level < num_nodes; ++level) {
      ch.node_levels_[contraction_order[level]] = level;
    }
    
    ankerl::unordered_dense::set<node_idx_t> contracted;
    
    for (auto const node_to_contract : contraction_order) {
      if (kDebug) {
        std::cout << "Contracting node " << node_to_contract << std::endl;
      }
      
      std::vector<std::pair<node_idx_t, cost_t>> incoming;
      std::vector<std::pair<node_idx_t, cost_t>> outgoing;
      
      collect_neighbors(w, ch, node_to_contract, contracted, incoming, outgoing);
      
      for (auto const& [v, cost_v_u] : incoming) {
        for (auto const& [w_node, cost_u_w] : outgoing) {
          if (v == w_node) continue;
          
          auto const shortcut_cost = cost_v_u + cost_u_w;
          
          if (!needs_shortcut(w, ch, v, w_node, shortcut_cost, node_to_contract, contracted)) {
            continue;
          }
          
          ch.add_shortcut(v, w_node, shortcut_cost, node_to_contract,
                         way_idx_t::invalid(), way_idx_t::invalid(), 
                         false, false);
          
          if (kDebug && (v < 10 || w_node < 10)) {  // Only debug first few nodes
            std::cout << "  Added shortcut " << v << " -> " << w_node 
                     << " cost=" << shortcut_cost << " (v-u=" << cost_v_u 
                     << " + u-w=" << cost_u_w << ")" << std::endl;
          }
        }
      }
      
      contracted.insert(node_to_contract);
    }
    
    return ch;
  }
  
private:
  static void collect_neighbors(ways const& w,
                                ch_data const& ch,
                                node_idx_t node,
                                ankerl::unordered_dense::set<node_idx_t> const& contracted,
                                std::vector<std::pair<node_idx_t, cost_t>>& incoming,
                                std::vector<std::pair<node_idx_t, cost_t>>& outgoing) {
    auto const& r = *w.r_;
    
    car::resolve_all(r, node, level_t{static_cast<std::uint8_t>(0U)}, [&](car::node const n) {
      car::template adjacent<direction::kForward, false>(
          r, n, nullptr, nullptr, nullptr,
          [&](car::node const neighbor, std::uint32_t const cost,
              distance_t, way_idx_t const, std::uint16_t, std::uint16_t,
              elevation_storage::elevation const, bool const) {
            if (contracted.find(neighbor.n_) == contracted.end()) {
              outgoing.emplace_back(neighbor.n_, cost);
            }
          });
      
      car::template adjacent<direction::kBackward, false>(
          r, n, nullptr, nullptr, nullptr,
          [&](car::node const neighbor, std::uint32_t const cost,
              distance_t, way_idx_t const, std::uint16_t, std::uint16_t,
              elevation_storage::elevation const, bool const) {
            if (contracted.find(neighbor.n_) == contracted.end()) {
              incoming.emplace_back(neighbor.n_, cost);
            }
          });
    });
    
    auto const* fwd_shortcuts = ch.get_forward_shortcuts(node);
    if (fwd_shortcuts) {
      for (auto const& sc : *fwd_shortcuts) {
        if (contracted.find(sc.to_) == contracted.end()) {
          outgoing.emplace_back(sc.to_, sc.cost_);
        }
      }
    }
    
    auto const* bwd_shortcuts = ch.get_backward_shortcuts(node);
    if (bwd_shortcuts) {
      for (auto const& sc : *bwd_shortcuts) {
        if (contracted.find(sc.from_) == contracted.end()) {
          incoming.emplace_back(sc.from_, sc.cost_);
        }
      }
    }
  }
  
  static bool needs_shortcut(ways const& w,
                             ch_data const& ch,
                             node_idx_t from,
                             node_idx_t to,
                             cost_t shortcut_cost,
                             node_idx_t contracted_node,
                             ankerl::unordered_dense::set<node_idx_t> const& contracted) {
    std::priority_queue<witness_search_entry, 
                       std::vector<witness_search_entry>,
                       std::greater<witness_search_entry>> pq;
    ankerl::unordered_dense::map<node_idx_t, cost_t> costs;
    
    pq.push({from, 0});
    costs[from] = 0;
    
    while (!pq.empty()) {
      auto const [curr_node, curr_cost] = pq.top();
      pq.pop();
      
      if (curr_cost > shortcut_cost) {
        break;
      }
      
      if (curr_node == to) {
        return false;
      }
      
      if (costs[curr_node] < curr_cost) {
        continue;
      }
      
      explore_neighbors_for_witness(w, ch, curr_node, to, curr_cost, 
                                   shortcut_cost, contracted_node, 
                                   contracted, pq, costs);
    }
    
    return true;
  }
  
  static void explore_neighbors_for_witness(
      ways const& w,
      ch_data const& ch,
      node_idx_t curr_node,
      node_idx_t target,
      cost_t curr_cost,
      cost_t max_cost,
      node_idx_t excluded_node,
      ankerl::unordered_dense::set<node_idx_t> const& contracted,
      std::priority_queue<witness_search_entry, 
                         std::vector<witness_search_entry>,
                         std::greater<witness_search_entry>>& pq,
      ankerl::unordered_dense::map<node_idx_t, cost_t>& costs) {
    
    auto const& r = *w.r_;
    
    car::resolve_all(r, curr_node, level_t{static_cast<std::uint8_t>(0U)}, [&](car::node const n) {
      car::template adjacent<direction::kForward, false>(
          r, n, nullptr, nullptr, nullptr,
          [&](car::node const neighbor, std::uint32_t const cost,
              distance_t, way_idx_t const, std::uint16_t, std::uint16_t,
              elevation_storage::elevation const, bool const) {
            if (neighbor.n_ == excluded_node || 
                contracted.find(neighbor.n_) != contracted.end()) {
              return;
            }
            
            auto const new_cost = static_cast<cost_t>(curr_cost + cost);
            if (new_cost <= max_cost) {
              auto it = costs.find(neighbor.n_);
              if (it == costs.end() || new_cost < it->second) {
                costs[neighbor.n_] = new_cost;
                pq.push({neighbor.n_, new_cost});
              }
            }
          });
    });
    
    auto const* shortcuts = ch.get_forward_shortcuts(curr_node);
    if (shortcuts) {
      for (auto const& sc : *shortcuts) {
        if (sc.to_ == excluded_node || 
            contracted.find(sc.to_) != contracted.end()) {
          continue;
        }
        
        auto const new_cost = static_cast<cost_t>(curr_cost + sc.cost_);
        if (new_cost <= max_cost) {
          auto it = costs.find(sc.to_);
          if (it == costs.end() || new_cost < it->second) {
            costs[sc.to_] = new_cost;
            pq.push({sc.to_, new_cost});
          }
        }
      }
    }
  }
};

}  // namespace osr