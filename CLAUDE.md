# Bidirectional Dijkstra with Contraction Hierarchies (CH) for OSR (car profile)

## General idea

Add a new routing algorithm to OSR (only for the **car profile**): **Bidirectional Dijkstra on a Contraction Hierarchies (CH) graph**.  
Two phases:

1. **Preprocessing (CH construction):** contract nodes, add shortcuts, respect turn restrictions.  
2. **Querying (CH search):** bidirectional Dijkstra with CH level filtering and modified termination, proper meeting-point detection, and full shortcut unpacking.

⚠️ Notes  
- Only the **car profile** is required.  
- Node ordering is **random** and remains correct.  

---

## 1) Preprocessing (CH construction)

### Node ordering
- Assign each node a unique random level in `1..n`; define `<` by level: `u < v ⇔ level(u) < level(v)`.

### Shortcut creation overview
For each node `u` in ascending level order contract the node u:
- For each incoming `(v,u)` with `v > u`  
- and each outgoing `(u,w)` with `w > u`  
- if `<v,u,w>` **may be the only shortest path**, add a shortcut `(v,w)` with weight `w(v,u) + w(u,w)`.

### Shortcut creation details
For the contraction of a node u, we face a many-to-many shortest path problem from source nodes v ∈ S := {v | (v, u) ∈ E and level(v) > level(u)} incident to incoming edges of u to all target nodes w ∈ T := {w | (u, w) ∈ E  and level(w) > level(u)} incident to outgoing edges of u. For such a pair v != w, we want 
to decide whether `<v, u, w>`, if it is a shortest v-w-path, is the only shortest v-w-path. A simple way to implement this is to perform for each source node v a forward shortest-path search starting at v in the current remaining graph until all target nodes T \ {v} are settled. The reamining graph is the graph containing only nodes with level >= Level(u) and all edges between these nodes, **including shortcuts added in previous contractions**. Such a limited search is called a local search. For a given pair (v, w) we add a shortcut edge if and only if the shortest path found by this local search contains u. If there are already edges (v, w) and we add the shortcut (v, w) we keep the original edges (v, w), thus the graph is not simple. 

## OSR specific turn restrictions: 
Take a look at this working implementation of a normal bidirectional Dijktra in OSR for the car profile (no contraction hierachies):

```
#pragma once

#include <limits>

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
          auto const total = curr_cost + cost;
          if (total >= max) {
            if (SearchDir == direction::kForward) {
              max_reached_1_ = true;
            } else {
              max_reached_2_ = true;
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

}  // namespace osr
```

Especially consider how we consider turn restrictions when we traverse the graph. What changes in the graph traversal of bidrectional Dijktra with contraction hierachies for the car profile is that we can also traverse shortcuts. The idea is the following: As you can see in include\osr\routing\profiles\car.h a car node is defined by this triple: node_idx_t n_;  way_pos_t way_; direction dir_;
Let's say we are at node_idx_t = v with way_position_t = 0 and direction = forward: (v, 0, forward). In our normal bidirectional dijktra without contraction hierachies we can can easily get all legal one-step moves from the current car node to adjacent car nodes with other node_idx_t. Let's say w and u are adjacent node_idx_t to v then adjacent(v, 0, forward) could give us : [(w, 1, forward), (u, 3, forward)] for example.

Reminder: deeply analyze include\osr\routing\profiles\car.h
```
template <direction SearchDir, bool WithBlocked, typename Fn>
  static void adjacent(ways::routing const& w,
                       node const n,
                       bitvec<node_idx_t> const* blocked,
                       sharing_data const*,
                       elevation_storage const*,
                       Fn&& fn) {
    auto way_pos = way_pos_t{0U};
    for (auto const [way, i] :
         utl::zip_unchecked(w.node_ways_[n.n_], w.node_in_way_idx_[n.n_])) {
      auto const expand = [&](direction const way_dir, std::uint16_t const from,
                              std::uint16_t const to) {
        // NOLINTNEXTLINE(clang-analyzer-core.CallAndMessage)
        auto const target_node = w.way_nodes_[way][to];
        if constexpr (WithBlocked) {
          if (blocked->test(target_node)) {
            return;
          }
        }

        auto const target_node_prop = w.node_properties_[target_node];
        if (node_cost(target_node_prop) == kInfeasible) {
          return;
        }

        auto const target_way_prop = w.way_properties_[way];
        if (way_cost(target_way_prop, way_dir, 0U) == kInfeasible) {
          return;
        }

        if (w.is_restricted<SearchDir>(n.n_, n.way_, way_pos)) {
          return;
        }

        auto const is_u_turn = way_pos == n.way_ && way_dir == opposite(n.dir_);
        auto const dist = w.way_node_dist_[way][std::min(from, to)];
        auto const target =
            node{target_node, w.get_way_pos(target_node, way, to), way_dir};
        auto const cost = way_cost(target_way_prop, way_dir, dist) +
                          node_cost(target_node_prop) +
                          (is_u_turn ? kUturnPenalty : 0U);
        fn(target, cost, dist, way, from, to, elevation_storage::elevation{},
           false);
      };

      if (i != 0U) {
        expand(flip<SearchDir>(direction::kBackward), i, i - 1);
      }
      if (i != w.way_nodes_[way].size() - 1U) {
        expand(flip<SearchDir>(direction::kForward), i, i + 1);
      }

      ++way_pos;
    }
  }
```

Now with contraction hierachies we also have shorcuts between different node_idx_t. For example let v, w and u be all a distinct node_indx_t. We may have a shortcut (v, w) over the edges (v, u), (u, w). 
Turn restrictions tells us: based on the way_pos_t we used to get into our current node_idx_t, what way_pos_t can we take now to get to the next node_idx_t. When we used a normal edge (no shortcut), in other words, a normal way to get to our node_idx_t we can use adjacent to list all legal normal edges (normal ways) to take as a next step, since we know the way_pos_t  we used to get to our current  node_idx_t. Now there are 2 questions we have to ask ourselves: **What if we used a shortcut to get to our current node_idx_t? And how can we find all legal shortcuts that we can use to get to the next node_idx_t?** For the first question: We always have to store the last "normal edge" of a shortcut. In our example with the shortcut (v, w) over the edges (v, u), (u, w) we would need to store for example (w, 2, forward) if we come to w using the way_pos_t = 2 of node_idx_t = w. That way we can call adjacent on (w, 2, forward) since this is how we came to our current node_idx_t. For the other question: adjacent only returns all legal normal edges (normal ways) we can take to get to the next node_idx_t. We store shortcuts separately in a map from node_idx_t to a list of outgoing shortcuts. Let's say we are currently at (v, 4, forward) and we want to know all the possible edges we can take to get to a next node_idx_t. First we would use adjacent on (v, 4, forward) to consider all normal outgoing edges (normal ways). Then in a separate loop we would go over all outgoing shortcuts of our node_idx_t to list of shortcuts map and return all legal shortcuts. How do we check if a shortcut is legal? We already now how we got to our current node_idx_t, in our example (v, 4, forward). For turn legalty we only have to know the first hop of our shorcut, in other words, the first normal edge this shortcut stands for. In our example the shortcut (v, w) stands for (v, u), (u, w). In a shortcut we always have to store the first "normal edge". In our example with the shortcut (v, w) over the edges (v, u), (u, w) we would need to store for example (u, 3, forward) if we come to u using the way_pos_t = 3 of node_idx_t = v. That way we know that we used (v, 4, forward) to come to v and that we could take the shortcut (v, w) if we go to node_idx_t u using way_pos_t=3 of v. Then we could call is_restricted(v, 3, 4) to check if this shortcut is legal. is_restricted is defined in include\osr\ways.h


## Contraction Pseudo Code
```
contract(u):
  S = { v | (v,u) in E and level(v) > level(u) }
  T = { w | (u,w) in E and level(w) > level(u) }

  for v in S:
    dist = local_dijkstra_from_v(level_min = level(u))  // nodes with level >= level(u)
      // expansions:
      //  - normal edges with is_restricted checks
      //  - existing shortcuts with first-step is_restricted checks

    for w in T, w != v:
      if shortest v->w path includes u:
        add_shortcut(v, w, via=u,
          first_step_at_v = (to_way_pos_at_v, dir_first),
          last_step_at_w  = (from_way_pos_at_w, dir_last),
          cost = cost(v->u)+cost(u->w), distance=...)
```

## Query expansion from a settled state Pseudo Code (used in Witness Search and in Query)
```
expand_from_state(n = (x, way_pos_in, dir_in)):
  // 1) Normal edges
  for each (y, way_pos_out, dir_out) produced by car::adjacent with legality checks:
    relax normal edge (x->y)

  // 2) Shortcuts
  for each shortcut (x->z) in outgoing_shortcuts[x]:
    let needed_to_way_pos = shortcut.first.to_way_pos_at_x
    if is_restricted<SearchDir>(x, way_pos_in, needed_to_way_pos): continue
    relax shortcut (x->z) with cost shortcut.cost_
    // arrival state at z:
    set predecessor carrying last_step_at_z = (from_way_pos_at_z, dir_at_z)
```


**Correctness of Preprocessing**
Let's say there exists a shortest path P that goes through the nodes u->v->w. Without loss of generality we assume that our forward search is currently at node u and our backward search is currently at node w. Let's go through all possible level assignments:
1.: level(u) > level(v) > level(w): in that case our backward search can just take all inverted edges and the two searches meet at node u. 
2.: level(u) < level(v) < level(w): in that case our forward search can just take all normal edges and the two searches meet at node w. 
3.: level(u) < level(v) > level(w): forward search takes edge (u, v), backward search takes (inverted) edge (w, v), both searches meet at v
4.: level(u) > level(v) < level(w): We assume that P is a shortest path, therefore the preproceesing must have produced a shortcut (u, w) via v because level(u) > level(v) and level(w) > level(v) and we did not find any witness, since P is a shortest path and every subpath of P is also a shortest subpath. If level(u) < level(w), the forward search can take this shortcut (u, w), so both searches meet at node w. If level(u) > level(w), the backward search can take the inverted shortcut (w, u), so both searches meet at node u.


## 2) Querying (Bidirectional Dijkstra with CH level filtering)

### Level filtering
- **Forward search:** relax only edges `(u,v)` with `level(v) > level(u)` (upward graph).  
- **Backward search:** relax only edges `(u,v)` with `level(u) > level(v)` (downward graph, on the inverted direction).  

**Termination and meetpoint logic: keep as it is in the normal bidirectional dijktra for car profile (already implemented)**

**Use the same turn restriction aware Query expansion logic as described above**

## 3) Path reconstruction (full shortcut unpacking, no extra restriction checks)

After the query, reconstruct a complete path of **original OSR edges** by **fully unpacking every shortcut**:

- A shortcut `(v,w)` via `u` expands into `<v,u>` + `<u,w>`.  
- If `<v,u>` or `<u,w>` are themselves shortcuts, unpack recursively until only real edges remain.  
- **Do not re-check restrictions** during reconstruction:  
  - Restrictions were already enforced during the query 

The final path is a faithful, drivable sequence of original car-edges with correct costs and penalties.

## 4) Testing with test\dijkstra_astarbidir_test.cc


