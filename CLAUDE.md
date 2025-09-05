# Bidirectional Dijkstra with Contraction Hierarchies (CH) for OSR (car profile)

## General idea

Add a new routing algorithm to OSR (only for the **car profile**): **Bidirectional Dijkstra on a Contraction Hierarchies (CH) graph**.  
Two phases:

1. **Preprocessing (CH construction):** contract nodes, add shortcuts, respect turn restrictions.  
2. **Querying (CH search):** bidirectional Dijkstra with CH level filtering and modified termination, proper meeting-point detection, and full shortcut unpacking.

⚠️ Notes  
- Only the **car profile** is required.  
- Node ordering is **random** and remains correct.  
- Level filtering must be applied exactly as specified.  
- Connectivity problems indicate **missing shortcuts** (not wrong level filtering).

---

## 1) Preprocessing (CH construction)

### Node ordering
- Assign each node a unique random level in `1..n`; define `<` by level: `u < v ⇔ level(u) < level(v)`.

### Shortcut creation
For each node `u` in ascending level order:
- For each incoming `(v,u)` with `v > u`  
- and each outgoing `(u,w)` with `w > u`  
- if `<v,u,w>` **may be the only shortest path**, add a shortcut `(v,w)` with weight `w(v,u) + w(u,w)`.

**Details**
- **Witness search:** local Dijkstra from `v` on the **remaining graph** (`level(x) > level(u)`, `x ≠ u`), stopping when all targets `w` are settled or the distance bound is exceeded. So these local Dijkstras should ignore the node u itself and all nodes with lower level than u. What are nodes with lower level than u? Nodes that have already been contracted. Once a node is contracted, it’s not allowed in later witness searches. So the Dijkstra of a witness search should mark u as already visited instantly and skip nodes with lower level (that already have been contracted). That way it ony explores paths that do not go through the node u currently under contraction, and any node that has already been contracted before.
- **Turn restrictions:** enforced using `car::adjacent` (local searches must honor OSR’s turn semantics, take a look at include\osr\routing\profiles\car.h and include\osr\routing\dijkstra.h).  
- **Existing shortcuts are usable and must be handled like in the query:**
  - When the local Dijkstra expands an edge that is a shortcut, apply the same **first/last real edge legality checks** (Cases 1–4, see below).  
  - This ensures that shortcuts used inside witness searches also respect turn restrictions.  
- If an edge `(v,w)` exists but is heavier than the shortcut, we keep it there and add the shotcut as another edge from v to w. This is because depending on different turn restirctions we may only be able to use the "old" edge `(v,w)`. This way, the graph is not simple.

**Correctness of Preprocessing**
Let's say there exists a shortest path P that goes through the nodes u->v->w. Without loss of generality we assume that our forward search is currently at node u and our backward search is currently at node w. Let's go through all possible level assignments:
1.: level(u) > level(v) > level(w): in that case our backward search can just take all inverted edges and the two searches meet at node u. 
2.: level(u) < level(v) < level(w): in that case our forward search can just take all normal edges and the two searches meet at node w. 
3.: level(u) < level(v) > level(w): forward search takes edge (u, v), backward search takes (inverted) edge (w, v), both searches meet at v
4.: level(u) > level(v) < level(w): We assume that P is a shortest path, therefore the preproceesing must have produced a shortcut (u, w) via v because level(u) > level(v) and level(w) > level(v) and we did not find any witness, since P is a shortest path and every subpath of P is also a shortest subpath. If level(u) < level(w), the forward search can take this shortcut (u, w), so both searches meet at node w. If level(u) > level(w), the backward search can take the inverted shortcut (w, u), so both searches meet at node u.

### What is a “normal edge” in the car profile?

A *normal edge* is the **atomic step** produced by `car::adjacent`: moving along **one segment** of a single way from index `i` to its neighbor `i±1` with a concrete travel **direction**, starting at intersection `node_idx_t at` and ending at `target`.

It is convenient to describe this atomic step with an anchor (this is *exactly* the information you need to replay costs and test turn legality):

```
struct turn_anchor {
node_idx_t at; // intersection where the step begins/ends (node_idx_t)
way_idx_t way; // global id of the way
way_pos_t way_pos; // index of 'way' in node_ways_[at] (needed for is_restricted)
direction dir; // travel direction along the way for this step
uint16_t from; // local index in way_nodes_[way] at 'at'
uint16_t to; // neighbor index (from±1) traversed in this step
node_idx_t target; // OSR node reached by this atomic move
};
```

### Shortcut metadata
Each shortcut `(v,w)` via `u` stores:
- the **middle node** `u` (for recursive unpacking),  
- the **first normal edge** (anchor) of the represented subpath (leaving `v`),  
- the **last normal edge** (anchor) of the represented subpath (entering `w`).  

These two real-edge anchors allow legality checks between chained shortcuts using the same logic as `car::adjacent` (by comparing the **incoming** and **outgoing** way at a junction).

---

## 2) Querying (Bidirectional Dijkstra with CH level filtering)

### Level filtering
- **Forward search:** relax only edges `(u,v)` with `level(v) > level(u)` (upward graph).  
- **Backward search:** relax only edges `(u,v)` with `level(u) > level(v)` (downward graph, on the inverted direction).  

### Edge relaxation semantics (very important)
- **We only traverse edges that are legal under `car::adjacent`.**  
- For **normal edges**, legality is determined directly by `car::adjacent` from the current `car::node` (which captures node, way, direction).  
- For a **shortcut edge**, we **do not** blindly relax it:  
  - To enter a shortcut `(v → …)`, use its **stored first normal edge** to determine the outgoing way at `v` and test turn restrictions against the **incoming way** of how you arrived at `v`.  
  - To enter a shortcut in the opposite direction `(… → v)`, use its **stored last normal edge** to determine the incoming way at `v`.  
  - Only if this check passes (i.e., `!is_restricted(...)`) do we relax the shortcut.  
  - U-turn penalties and per-way costs are applied consistently with how `car::adjacent` would apply them at the entry/exit edges.

This guarantees that **every** relaxed step (normal or shortcut) is already **turn-legal** during the search.

### Termination
- Interleave forward and backward expansions; maintain best tentative distance `µ`.  
- **Abort-on-success:** stop when the minimum key in **both** PQs exceeds `µ`.  
- Do **not** terminate at the first frontier meeting.

---

## 3) Meeting-point detection (OSR/car-specific)

Meeting at a raw `node_idx_t` is **not sufficient** in OSR/car, because turn restrictions depend on the **entering way** and **leaving way**.  
Instead, we must consider **car::nodes** `(node_idx_t, way, direction)`.

### Exact procedure with code

Assume forward search settled `node_idx_t = N` first; later, backward search settles the same `N` via some way `way_j`. We now enumerate forward-settled car-nodes at `N`, test stitchability, and pick the cheapest.

```
void consider_meetpoints_at_node(ways::routing const& w,
  node_idx_t N,
  way_idx_t way_j,
  double& best_mu,
  car::node& best_meet_cand) {

  car::resolve_all(w, N, /level/{}, [&](car::node cand) {
  if (!was_settled_forward(cand)) return;
  way_pos_t incoming_way_pos = cand.way_;

  way_pos_t outgoing_way_pos{};
  bool found = false;
  {
    auto ways_at_N = w.node_ways_[N];
    for (auto i = way_pos_t{0U}; i != ways_at_N.size(); ++i) {
      if (ways_at_N[i] == way_j) { outgoing_way_pos = i; found = true; break; }
    }
  }
  if (!found) return;

  if (w.is_restricted<direction::kForward>(N, incoming_way_pos, outgoing_way_pos)) {
    return;
  }

  double cand_total = total_cost(cand, way_j);

  if (cand_total < best_mu) {
    best_mu = cand_total;
    best_meet_cand = cand;
  }
});
}
```

Notes:
- `was_settled_forward(cand)` means the forward PQ popped/settled that **car::node** `(N, cand.way_, cand.dir_)`.
- `total_cost(cand, way_j)` should sum forward and backward costs (plus potential U-turn penalty if needed).
- If backward settled `N` first, mirror the logic.

---

## 3a) Shortcut legality during query (and witness search)

During the query (and the witness search), when checking if an edge `(v,w)` is usable after `(u,v)`, we must consider whether either edge is a shortcut.  
There are **four cases**:

- **Case 1: Both are normal edges**  
  Use `car::adjacent` to test legality of `(u,v) → (v,w)` directly.  

- **Case 2: `(u,v)` is normal, `(v,w)` is a shortcut**  
  Use the **first normal edge** of `(v,w)` to test legality `(incoming = n.way_) → (outgoing = first_anchor.way_pos)`.  

- **Case 3: `(u,v)` is a shortcut, `(v,w)` is normal**  
  Use the **last normal edge** of `(u,v)` to test legality `(incoming = last_anchor.way_pos) → (outgoing = candidate normal edge’s way_pos)`.  

- **Case 4: Both `(u,v)` and `(v,w)` are shortcuts**  
  Use the **last** anchor of `(u,v)` and the **first** anchor of `(v,w)` to test legality `(last_anchor.way_pos) → (first_anchor.way_pos)`.

### Code example for Case 3 (shortcut → normal)

```
template <direction SearchDir>
std::pair<bool, cost_t>
check_and_cost_normal_after_shortcut(
ways::routing const& w,
turn_anchor const& last_anchor,
way_idx_t next_way,
uint16_t next_from,
uint16_t next_to,
direction next_dir) {

auto const v = last_anchor.at;

way_pos_t incoming_way_pos = last_anchor.way_pos;

way_pos_t outgoing_way_pos{};
bool found = false;
{
auto const ways_at_v = w.node_ways_[v];
for (auto i = way_pos_t{0U}; i != ways_at_v.size(); ++i) {
if (ways_at_v[i] == next_way) { outgoing_way_pos = i; found = true; break; }
}
}
if (!found) return {false, cost_t{}};

if (w.is_restricted<SearchDir>(v, incoming_way_pos, outgoing_way_pos)) {
return {false, cost_t{}};
}

auto const target_node = w.way_nodes_[next_way][next_to];
auto const target_node_prop = w.node_properties_[target_node];
if (car::node_cost(target_node_prop) == kInfeasible) return {false, cost_t{}};

auto const target_way_prop = w.way_properties_[next_way];
if (car::way_cost(target_way_prop, next_dir, 0U) == kInfeasible) return {false, cost_t{}};

auto const is_u_turn =
(outgoing_way_pos == incoming_way_pos) && (next_dir == opposite(last_anchor.dir));

auto const dist = w.way_node_dist_[next_way][std::min(next_from, next_to)];
cost_t step_cost =
car::way_cost(target_way_prop, next_dir, dist) +
car::node_cost(target_node_prop) +
(is_u_turn ? car::kUturnPenalty : 0U);

return {true, step_cost};
}
```


---

## 4) Path reconstruction (full shortcut unpacking, no extra restriction checks)

After the query, reconstruct a complete path of **original OSR edges** by **fully unpacking every shortcut**:

- A shortcut `(v,w)` via `u` expands into `<v,u>` + `<u,w>`.  
- If `<v,u>` or `<u,w>` are themselves shortcuts, unpack recursively until only real edges remain.  
- **Do not re-check restrictions** during reconstruction:  
  - Restrictions were already enforced during the query using  
    - `car::adjacent` (for normal edges) and  
    - shortcut **first/last edge anchors** (for shortcuts).  

The final path is a faithful, drivable sequence of original car-edges with correct costs and penalties.

---

## Testing

- Base tests on `test/dijkstra_astarbidir_test.cc`, adapted for CH-Bidir-Dijkstra.  
- Use **Monaco OSM** for compact regression.  

Verify:
- Shortest-path **costs** are correct (paths may differ geometrically vs. other algorithms).  
- Meeting-point detection selects **stitchable candidates** under restrictions.  
- Shortcut legality cases (1–4 above) are correctly handled (both in queries and witness searches).  
- Reconstruction produces a path of **only original edges** (no shortcuts remain) and aligns with the computed cost.  

---

## Summary

- **Preprocessing:** random levels; local Dijkstra on the remaining graph with `car::adjacent`; add `(v,w)` via `u`; store middle, first normal edge, last normal edge; handle shortcuts in witness searches like in queries.  
- **Querying:** bidirectional Dijkstra with level filtering; only relax legal moves; for shortcuts, check legality using the first/last normal edges (Cases 1–4).  
- **Meeting points:** determined at `car::node` granularity with `resolve_all` + `is_restricted`; code above shows the exact stitch selection; pick the cheapest valid meetpoint.  
- **Reconstruction:** fully unpack all shortcuts into original edges; no extra restriction checks needed.