# Project Description: Bidirectional Dijkstra with Contraction Hierarchies (CH) for OSR (car profile)

<idea>
## General idea

The general idea is that we want to add another routing algorithm to the OSR repository (only for the car profile!). It is a bidrectional dijkstra that works on a preprocessed graph using Contraction Hierachies (ch). So there are basically 2 phases:

1. **Preprocessing:** Apply ch-preprocessing to the graph (OSR doesn't work with a classical graph, more about this later).  
2. **Querying:** Use the bidirectional Dijkstra with ch-level filtering (and slightly changed termination criterium) on the preprocessed graph to find the shortest path between 2 points.

## Important: it only has to work for the car profile! + The random node ordering does not break correctness, so don't change this. The level filtering must be like specified in this document. Also this is a wrong assumption: 
  "This indicates that the level filtering is too strict and preventing
  connectivity. The problem is that with pure random ordering, the forward
  search (which can only go to higher levels) and backward search (which can        
  only go to lower levels) may not have overlapping paths in the hierarchy."
  If it is implemented like described here, there should be no connectivity problems. VERY IMPORTANT: If that is not clear: the backward search should also go up in levels, so only take edges (u, v) where level(v) > level(u), but on the inverted graph

If there are connectivity issues, there are missing shortcuts. Look at this correctness proof:
Annahme o.b.d.A.: kürzeste Pfade
sind eindeutig.
Damit der kürzeste Pfad auch in
G∗ gefunden wird, muss er in zwei
Teile, “Up-Graph” und
“Down-Graph” aufteilbar sein
(bzgl. level).
Es gibt einen Knoten v∗ mit dem
höchsten level aller Knoten in
einem kürzesten Pfad s-t
Angenommen, es gäbe auf dem
kürzesten Weg von s nach v∗ eine
Folge von Knoten u, v, w mit
level(u) > level(v) < level(w)
(down-up).
Dann wäre im Preprocessing ein
Shortcut (u, w) erstellt worden
Shortcut könnte immer noch eine
Down-Kante sein - Argument
wiederholen.
Daraus folgt: Es muss einen Pfad
s-v∗ in G∗ geben nur mit
Up-Kanten, der genauso lang ist
wie der kürzeste Pfad in G. Analog
für Down-Graph.


---

## 1) Preprocessing (CH construction)

First we have to determine a node ordering, each node should get a different level. For this project, the node ordering must be random, so with *n* nodes, each node must get a unique random level between 1 and *n*. This node ordering defines a total order `<`.

Now we can start adding shortcuts to the graph:
Algorithm 1:

```text
foreach u ∈ V ordered by < ascending do
    foreach (v, u) ∈ E with v > u do
        foreach (u, w) ∈ E with w > u do
            if 〈v, u, w〉 “may be” the only shortest path from v to w then
                E := E ∪ {(v, w)}  // use weight w(v, w) := w(v, u) + w(u, w)
```

> **Mind v > u und w > u!**

### Definitions from the paper

> “Such an edge added in Line 5 is called a shortcut edge or just shortcut. They only represent existing paths in the current graph and are used to preserve shortest paths if the node u and all its incident edges is removed from the graph. If an edge (v, w) already exists in G but has larger weight than a new shortcut (v, w), then the Line 5 only reduce the weight of the already existing edge.”

Let’s also define witness paths:

> “A path P = 〈v, . . . , w〉 6 = 〈v, u, w〉 between v and w with w(P ) ≤ w(〈v, u, w〉) is called a witness path or just witness for the triple v, u, w. The name is derived from the fact that such a path witnesses that 〈v, u, w〉 is not a shortest path or not the only shortest path and allows to omit a shortcut.”

> “The whole step of finding witnesses and adding shortcuts for node u, Lines 2–5, is called the contraction of u. Because after that step, for all shortest s-t-paths P in G, with s, t > u, that have u in their interior, exists a shortest s-t-path P ′ in G without u in its interior. Applied recursively, it is easy to see that there even exists an shortest s-t-path P ′ with only nodes > u in its interior. In the context of the contraction of node u, an edge (v, u) ∈ E is called an incoming edge of u, (u, w) an outgoing edge of u. The nodes > u are called remaining nodes, the incident edges are called the remaining edges and the graph induced by the remaining nodes is called the remaining graph.  
> The tuple (G = (V, E), <) consisting of the resulting graph G of Algorithm 1 and the node order < is called a contraction hierarchy (CH). The node order partitions the nodes into n distinct levels.”

Important: Witness search must only explore the remaining graph, i.e. all nodes with level strictly greater than level(u) (and excluding u itself), to ensure shortcuts preserve shortest paths according to CH theory. During contraction of u, witness searches run strictly on the remaining graph (only nodes with level > level(u), excluding u) using the same turn-restricted expansion as queries; CH shortcuts may be used for speed, but must never bypass car::adjacent turn semantics.

### Contraction of node *u* (“may be the only shortest path”)

“This section fills in the details that are omitted in the pseudo code, especially the part how witness paths are located. We will first describe a general approach using Dĳkstra’s algorithm. Let G′ = (V′, E′) be the remaining graph after the contraction of the direct predecessor of u. For the contraction of a node u, we face a many-to-many shortest path problem from source nodes v ∈ S := {v | (v, u) ∈ E′} incident to incoming edges of u to all target nodes w ∈ T := {w | (u, w) ∈ E′} incident to outgoing edges of u. For such a pair v ≠ w, we want to decide whether 〈v, u, w〉, if it is a shortest v–w-path, is the only shortest v–w-path in G′.  
A simple way to implement this is to perform for each source node v a forward shortest-path search starting at v in the current remaining graph G′ excluding u until all target nodes T \\ {v} are settled. Such a limited search is called a local search. Let d_v(w) be the shortest path distance found by this search. We add a shortcut edge if and only if d_v(w) > w(v, u) + w(u, w), i.e., if the shortest v–w-path excluding u will be longer. We can additionally stop the search from a node x when it has reached distance w(v, u) + max { w(u, w) | (u, w) ∈ E′ \\ {(u, v)} }. Mind that these local Dijkstras can use already existing shortcuts.”

### OSR-specific aspects (car profile, turn restrictions)

This is the theory but we have to add some additional logic since we work on the OSR repository (on the car profile). As you can see in the README, OSR does not work with a typical graph and we have turn restrictions. We can view turn restrictions as a relation between incoming edges of a node and its outgoing edges. Because depending on the incoming edge we used to get to this node, we may not be able to take all outgoing edges (e.g., when we drive with a car to a certain point using street A, we may not be allowed to turn right and take street B; on the other hand, if we drive to this point using street C, we may be allowed to use street B). These turn restrictions are considered in the `adjacent` function of the car profile (`include/osr/routing/profiles/car.h`).

So in the preprocessing when we run the local Dijkstras, we have to consider these turn restrictions and only allow shortcuts for paths that are **not restricted** in the original graph. When we later run a query on this preprocessed graph, we have to know for which incoming edges of a node we can take which outgoing shortcuts.

To do this (and also to later be able to reconstruct the found path), for a shortcut *v → w* via *u*, we have to store the information that this shortcut originally goes over *u*. Example: when we come to *v* from *x* and we want to take the shortcut from *v* to *w*, we have to **unpack** it and check if there are turn restrictions if we use *(v, u)* and come from *x*. In theory *(v, u)* could also be a shortcut, so we would need to unpack it **recursively** until we have a normal edge. **Mind that we don't have to unpack the entire shortcut path, we only have to fully unpack the very first edge, since only this edge is relevant for our turn restrictions.**

**Result of preprocessing:** we obtain the preprocessed graph with additional shortcut edges (each shortcut edge stores the node it contracts, in other words shortcut *(v, w)* via *u* stores *u*).

---

## 2) Querying (Bidirectional Dijkstra with CH level filtering)

Now we can run the query with the bidirectional Dijkstra. The OSR repo already has a working bidirectional A* which can be taken as an inspiration (we don't need the heuristic though and we also have to make some other slight adjustments). The code for the bidirectional A* can be found in `include/osr/routing/bidirectional.h`, `include/osr/routing/route.h` and `src/route.cc`.

Compared to a normal bidirectional Dijkstra we make some adjustments:

- The query algorithm does **not** relax edges leading to nodes **lower** than the current node.  
- This property is reflected in two search graphs. The upward graph  
  \( G↑ := (V, E↑) \) with \( E↑ := \{ (u, v) ∈ E \mid u < v \} \)  
  and, analogously, the downward graph  
  \( G↓ := (V, E↓) \) with \( E↓ := \{ (u, v) ∈ E \mid u > v \} \).
- We perform forward search in \( G↑ \) and a backward search in \( G↓ \).
- Forward and backward search are interleaved, we keep track of a tentative shortest-path length and **abort the forward/backward search process when all keys in the respective priority queue are greater than the tentative shortest-path length (abort-on-success criterion)**. Note that **we are not allowed to abort the entire query as soon as both search scopes meet for the first time.**

### Path reconstruction (shortcut unpacking)

The query algorithm described above can return a shortest path in the contraction hierarchy. In order to output a complete description of the computed shortest path, we have to **unpack the shortcut edges** to obtain the represented subpaths in the original graph. We propose a **recursive unpacking** routine based on the fact that each shortcut edge represents a subpath, not necessarily in the original graph, consisting of exactly two edges. As long as there is a shortcut edge in the path, we replace it by the two edges that originated the creation of this shortcut. Finally, we will have a path without any shortcut edges, that is equivalent to a path in the original graph. Consider a shortcut *(v, w)* that represents a path 〈v, u, w〉. This can be done because **we always store the middle node u in the shortcut data structure itself.**

### OSR meeting-point and turn-restriction considerations

- In OSR/car, **meeting at a node_idx_t alone is not sufficient**; we must meet on a full `car::node` (which encodes node, way, direction) to respect turn restrictions and possible U-turn penalties at/near the meeting point.  
- Because `car::adjacent` explores only the arriving way at intersections (and jumps to neighbors on other ways), forward and backward searches might not visit identical `car::node`s even if they share the same `node_idx_t`. This affects both **shortcut creation** (local searches) and **meeting-point detection** during queries.  
- Practical fixes include:
  - When evaluating meetpoints, verify that forward and backward frontiers are **stitchable** respecting restrictions (akin to `handle_end_of_way_meetpoint` in the existing bidirectional A* in `include/osr/routing/bidirectional.h`).  
  - Store enough info on shortcuts and **unpack at least the first edge** on use to check legality from the actual incoming way/direction.  
  - Account for **U-turn penalties** when comparing candidate meetpoints.

---

## Testing

We were given `test/dijkstra_astarbidir_test.cc`. We can adapt/copy this test so our implementation is tested (easiest by integrating our algorithm in `algorithms.h` and `route.h/cc`, analogous to the existing integration for A* Bi).

I already downloaded the Monaco part and have `test/monaco` and `test/monaco.osm.pbf`. **We should test on this one, since it is the smallest.**

We should highly consider this advice from my advisors (essentials paraphrased):

- It is normal that two different algorithms for a route have the same **costs** but very different **paths** (path lengths differ, same duration; cost rounding to seconds increases this effect). Tests therefore **do not** check path equality. Still, reconstructed paths should make sense and match the costs.  
- You can be inspired by the Bidir-A* implementation (esp. how it is wired into `route.cc`). But note differences: **early stopping criteria are different**, and **no heuristic** for our algorithm.  
- Because `car::node` includes node+way+direction, meeting points must be checked at this granularity; **do not** only compare `node_idx_t`. Consider potential **U-turns** at/near the meeting point.  
- Single PQ vs two PQs: for our CH bidir Dijkstra we can keep it simple (one PQ solution possible), but the exact wiring is up to our integration.

---

## Summary

- Preprocessing: random total order `<`, run local Dijkstras with turn restrictions, add shortcuts *(v, w)* via *u* only when 〈v, u, w〉 may be the only shortest path, **store u** in the shortcut for later unpacking.  
- Querying: bidirectional Dijkstra with **level filtering** on the CH graph (forward on `u < v` edges, backward on `u > v` edges), adjusted **abort-on-success** termination, and **meeting-point handling** that respects OSR car-profile turn restrictions and U-turn costs.  
- Reconstruction: recursively **unpack** shortcuts, but only **fully unpack the first edge** at each step where turn legality from the incoming way matters.

Note on working in this repo: when you add a new file, you have to run 'cmake -S . -B build -G "Visual Studio 17 2022" -DCMAKE_BUILD_TYPE=Debug' before building the project
</idea>