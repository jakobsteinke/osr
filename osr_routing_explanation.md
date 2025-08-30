# OSR Routing Explanation

## General OSM Data Routing with OSR

OSR (Open Street Router) enables routing on OpenStreetMap data through the following approach:

### Data Model
OSR doesn't use an explicit graph structure. Instead, it stores OSM nodes and ways more or less as-is, with only "multi nodes" (nodes that are part of multiple ways) being given special routing IDs (`node_idx_t`). This is memory-efficient since only a fraction of OSM nodes are relevant for routing.

### Routing Process

1. **From Coordinate to Graph**: 
   - Uses an R-tree to store bounding boxes of all ways
   - Finds closest `way_idx_t` to your start/end coordinates
   - Identifies the two closest routing nodes ("left" and "right") on that way

2. **Node Neighborhood**:
   - For each node, OSR stores which ways contain it and at what index
   - Neighbors are found by looking at indices i-1 and i+1 in those ways
   - This allows standard Dijkstra's algorithm to be applied

3. **Routing Profiles**:
   - Different profiles (car, bike, foot, etc.) define their own overlay graphs on the base data
   - Each profile provides functions for node resolution, adjacency, and cost calculation
   - Profiles can handle complex scenarios like turn restrictions, level changes (indoor routing), and direction-aware routing

4. **Path Finding**:
   - Currently uses Dijkstra's algorithm with time-based cutoff
   - Profiles compute edge weights based on OSM attributes and distances stored between nodes

### Usage
```bash
# Extract OSM data
./osr-extract -i planet-latest.osm.pbf -o osr-planet

# Run routing backend
./osr-backend -d osr-planet -s web
```

The key innovation is that multiple routing profiles share the same base data structure, making it memory-efficient to support different transportation modes without duplicating the graph for each profile.

## Car Profile Specifics

The car profile defines how routing works specifically for cars on the OSR graph. Here's how it works:

### Node Representation (`car::node`)
A car node consists of three components:
- **`node_idx_t n_`**: The actual OSM node index
- **`way_pos_t way_`**: Which way (street) we're on at this node (a node can be part of multiple ways)
- **`direction dir_`**: The direction we're traveling (forward/backward)

This triplet is crucial because **turn restrictions** depend on which way you arrived at an intersection and which direction you're going.

### Turn Restrictions
The car profile handles turn restrictions through the `adjacent` function. When expanding from a node, it:
1. Checks all ways connected to the current node
2. For each potential move, verifies if it's restricted: `w.is_restricted<SearchDir>(n.n_, n.way_, way_pos)`
3. This checks if coming from way `n.way_` allows turning onto way `way_pos`

### U-Turn Penalty
The profile applies a 120-second penalty for U-turns (when `way_pos == n.way_ && way_dir == opposite(n.dir_)`), discouraging unnecessary direction changes.

### Cost Calculation
Edge costs are computed based on:
- **Speed limits**: `dist / max_speed_m_per_s()`
- **Destination-only roads**: 5x cost multiplier + 120s penalty if `is_destination()`
- **One-way streets**: Returns `kInfeasible` if trying to go backward on a one-way street
- **Car accessibility**: Both nodes and ways must be car-accessible

### Entry Storage
The `entry` struct efficiently stores up to 16 ways × 2 directions = 32 states per node, tracking:
- Cost to reach each (way, direction) combination
- Predecessor for path reconstruction
- This allows the algorithm to remember different costs for arriving at the same node from different ways/directions

### Key Features for Car Routing
1. **Multi-way nodes**: A single OSM node can be reached via multiple ways with different costs/restrictions
2. **Direction-aware**: Tracks forward/backward movement on each way
3. **Turn-aware**: Respects turn restrictions based on incoming way
4. **Efficient storage**: Uses fixed-size arrays to avoid dynamic allocation
5. **Heuristic for A***: Assumes 130 km/h maximum speed for distance-based heuristic

This design allows OSR to handle complex car routing scenarios like highway interchanges, turn restrictions at intersections, and one-way streets without needing a separate graph for the car profile - it's an overlay on the shared OSM data.