#include "osr/ways.h"

#include "utl/parallel_for.h"

#include "cista/io.h"

#include <queue>
#include <unordered_map>
#include <limits>
#include <set>
#include <tuple>
#include <random>
#include <algorithm>

namespace osr {

ways::ways(std::filesystem::path p, cista::mmap::protection const mode)
    : p_{std::move(p)},
      mode_{mode},
      r_{mode == cista::mmap::protection::READ
             ? routing::read(p_)
             : cista::wrapped<routing>{cista::raw::make_unique<routing>()}},
      node_to_osm_{mm("node_to_osm.bin")},
      way_osm_idx_{mm("way_osm_idx.bin")},
      way_polylines_{mm_vec<point>{mm("way_polylines_data.bin")},
                     mm_vec<std::uint64_t>{mm("way_polylines_index.bin")}},
      way_osm_nodes_{mm_vec<osm_node_idx_t>{mm("way_osm_nodes_data.bin")},
                     mm_vec<std::uint64_t>{mm("way_osm_nodes_index.bin")}},
      strings_{mm_vec<char>(mm("strings_data.bin")),
               mm_vec<std::uint64_t>(mm("strings_idx.bin"))},
      way_names_{mm("way_names.bin")},
      way_has_conditional_access_no_{
          mm_vec<std::uint64_t>(mm("way_has_conditional_access_no"))},
      way_conditional_access_no_{mm("way_conditional_access_no")} 
      {
        if (r_->node_ch_level_.size() != n_nodes()) {
          r_->node_ch_level_.resize(n_nodes(), 0);
        }
        auto& r = *r_;
        r.node_ch_level_.resize(n_nodes(), -1);
      }

void ways::build_components() {
  auto q = hash_set<way_idx_t>{};
  auto flood_fill = [&](way_idx_t const way_idx, component_idx_t const c) {
    q.clear();
    q.insert(way_idx);
    while (!q.empty()) {
      auto const next = *q.begin();
      q.erase(q.begin());
      for (auto const n : r_->way_nodes_[next]) {
        for (auto const w : r_->node_ways_[n]) {
          auto& wc = r_->way_component_[w];
          if (wc == component_idx_t::invalid()) {
            wc = c;
            q.insert(w);
          }
        }
      }
    }
  };

  auto next_component_idx = component_idx_t{0U};
  r_->way_component_.resize(n_ways(), component_idx_t::invalid());
  for (auto i = 0U; i != n_ways(); ++i) {
    auto const way_idx = way_idx_t{i};
    auto& c = r_->way_component_[way_idx];
    if (c != component_idx_t::invalid()) {
      continue;
    }
    c = next_component_idx++;
    flood_fill(way_idx, c);
  }
}

void ways::add_restriction(std::vector<resolved_restriction>& rs) {
  using it_t = std::vector<resolved_restriction>::iterator;
  utl::sort(rs, [](auto&& a, auto&& b) { return a.via_ < b.via_; });
  utl::equal_ranges_linear(
      begin(rs), end(rs), [](auto&& a, auto&& b) { return a.via_ == b.via_; },
      [&](it_t const& lb, it_t const& ub) {
        auto const range = std::span{lb, ub};
        r_->node_restrictions_.resize(to_idx(range.front().via_) + 1U);
        r_->node_is_restricted_.set(range.front().via_, true);

        for (auto const& x : range) {
          if (x.type_ == resolved_restriction::type::kNo) {
            r_->node_restrictions_[x.via_].push_back(
                restriction{r_->get_way_pos(x.via_, x.from_),
                            r_->get_way_pos(x.via_, x.to_)});
          } else /* kOnly */ {
            for (auto const [i, from] :
                 utl::enumerate(r_->node_ways_[x.via_])) {
              for (auto const [j, to] :
                   utl::enumerate(r_->node_ways_[x.via_])) {
                if (x.from_ == from && x.to_ != to) {
                  r_->node_restrictions_[x.via_].push_back(restriction{
                      static_cast<way_pos_t>(i), static_cast<way_pos_t>(j)});
                }
              }
            }
          }
        }
      });
  r_->node_restrictions_.resize(node_to_osm_.size());
}

void ways::compute_big_street_neighbors() {
  struct state {
    hash_set<way_idx_t> done_;
  };

  auto pt = utl::get_active_progress_tracker();

  auto is_orig_big_street = std::vector<bool>(n_ways());
  for (auto const [i, p] : utl::enumerate(r_->way_properties_)) {
    is_orig_big_street[i] = p.is_big_street();
  }

  utl::parallel_for_run_threadlocal<state>(
      n_ways(), [&](state& s, std::size_t const i) {
        auto const way = way_idx_t{i};

        if (is_orig_big_street[to_idx(way)]) {
          pt->update_monotonic(i);
          return;
        }

        s.done_.clear();

        auto const expand = [&](way_idx_t const x, bool const go_further,
                                auto&& recurse) {
          for (auto const& n : r_->way_nodes_[x]) {
            for (auto const& w : r_->node_ways_[n]) {
              if (is_orig_big_street[to_idx(w)]) {
                r_->way_properties_[way].is_big_street_ = true;
                return true;
              }

              if (s.done_.emplace(w).second && go_further) {
                if (recurse(x, false, recurse)) {
                  return true;
                }
              }
            }
          }
          return false;
        };

        s.done_.emplace(way);
        expand(way, true, expand);
        pt->update_monotonic(i);
      });
}

void ways::connect_ways() {
  auto pt = utl::get_active_progress_tracker_or_activate("osr");

  {  // Assign graph node ids to every node with >1 way.
    pt->status("Create graph nodes")
        .in_high(node_way_counter_.size())
        .out_bounds(40, 50);

    auto node_idx = node_idx_t{0U};
    node_way_counter_.multi_.for_each_set_bit([&](std::uint64_t const b_idx) {
      auto const i = osm_node_idx_t{b_idx};
      node_to_osm_.push_back(i);
      ++node_idx;
      pt->update(b_idx);
    });
    r_->node_is_restricted_.resize(to_idx(node_idx));
  }

  // Build edges.
  {
    pt->status("Connect ways")
        .in_high(way_osm_nodes_.size())
        .out_bounds(50, 75);
    auto node_ways = mm_paged_vecvec<node_idx_t, way_idx_t>{
        cista::paged<mm_vec32<way_idx_t>>{
            mm_vec32<way_idx_t>{mm("tmp_node_ways_data.bin")}},
        mm_vec<cista::page<std::uint32_t, std::uint16_t>>{
            mm("tmp_node_ways_index.bin")}};
    auto node_in_way_idx = mm_paged_vecvec<node_idx_t, std::uint16_t>{
        cista::paged<mm_vec32<std::uint16_t>>{
            mm_vec32<std::uint16_t>{mm("tmp_node_in_way_idx_data.bin")}},
        mm_vec<cista::page<std::uint32_t, std::uint16_t>>{
            mm("tmp_node_in_way_idx_index.bin")}};
    node_ways.resize(node_to_osm_.size());
    node_in_way_idx.resize(node_to_osm_.size());
    for (auto const [osm_way_idx, osm_nodes, polyline] :
         utl::zip(way_osm_idx_, way_osm_nodes_, way_polylines_)) {
      auto pred_pos = std::make_optional<point>();
      auto from = node_idx_t::invalid();
      auto distance = 0.0;
      auto i = std::uint16_t{0U};
      auto way_idx = way_idx_t{r_->way_nodes_.size()};
      auto dists = r_->way_node_dist_.add_back_sized(0U);
      auto nodes = r_->way_nodes_.add_back_sized(0U);
      for (auto const [osm_node_idx, pos] : utl::zip(osm_nodes, polyline)) {
        if (pred_pos.has_value()) {
          distance += geo::distance(pos, *pred_pos);
        }

        if (node_way_counter_.is_multi(to_idx(osm_node_idx))) {
          auto const to = get_node_idx(osm_node_idx);
          node_ways[to].push_back(way_idx);
          node_in_way_idx[to].push_back(i);
          nodes.push_back(to);

          if (from != node_idx_t::invalid()) {
            dists.push_back(static_cast<std::uint16_t>(std::round(distance)));
          }

          distance = 0.0;
          from = to;

          if (i == std::numeric_limits<std::uint16_t>::max()) {
            fmt::println("error: way with {} nodes", osm_way_idx);
          }

          ++i;
        }

        pred_pos = pos;
      }
      pt->increment();
    }

    for (auto const x : node_ways) {
      r_->node_ways_.emplace_back(x);
    }
    for (auto const x : node_in_way_idx) {
      r_->node_in_way_idx_.emplace_back(x);
    }
  }

  auto e = std::error_code{};
  std::filesystem::remove(p_ / "tmp_node_ways_data.bin", e);
  std::filesystem::remove(p_ / "tmp_node_ways_index.bin", e);
  std::filesystem::remove(p_ / "tmp_node_in_way_idx_data.bin", e);
  std::filesystem::remove(p_ / "tmp_node_in_way_idx_index.bin", e);
}

void ways::sync() {
  node_to_osm_.mmap_.sync();
  way_osm_idx_.mmap_.sync();
  way_polylines_.data_.mmap_.sync();
  way_polylines_.bucket_starts_.mmap_.sync();
  way_osm_nodes_.data_.mmap_.sync();
  way_osm_nodes_.bucket_starts_.mmap_.sync();
  strings_.data_.mmap_.sync();
  strings_.bucket_starts_.mmap_.sync();
  way_names_.mmap_.sync();
}

std::optional<std::string_view> ways::get_access_restriction(
    way_idx_t const way) const {
  if (!way_has_conditional_access_no_.test(way)) {
    return std::nullopt;
  }
  auto const it = std::lower_bound(
      begin(way_conditional_access_no_), end(way_conditional_access_no_), way,
      [](auto&& a, auto&& b) { return a.first < b; });
  utl::verify(
      it != end(way_conditional_access_no_) && it->first == way,
      "access restriction for way with access restriction not found way={}",
      way_osm_idx_[way]);
  return strings_[it->second].view();
}

cista::wrapped<ways::routing> ways::routing::read(
    std::filesystem::path const& p) {
  return cista::read<ways::routing>(p / "routing.bin");
}

void ways::routing::write(std::filesystem::path const& p) const {
  return cista::write(p / "routing.bin", *this);
}

void deduplicate_shortcuts(osr::ways::routing& r) {
  std::sort(begin(r.shortcuts_), end(r.shortcuts_), [](const auto& a, const auto& b) {
    return std::tie(a.from, a.to, a.cost, a.middle) < std::tie(b.from, b.to, b.cost, b.middle);
  });
  auto last = std::unique(begin(r.shortcuts_), end(r.shortcuts_), [](const auto& a, const auto& b) {
    return std::tie(a.from, a.to, a.cost, a.middle) == std::tie(b.from, b.to, b.cost, b.middle);
  });
  r.shortcuts_.resize(static_cast<unsigned int>(std::distance(begin(r.shortcuts_), last)));
}


// Dijkstra for witness search 
bool witness_search(
    const ways::routing& r,
    node_idx_t v,
    node_idx_t w,
    node_idx_t skip_u,
    cost_t max_cost
) {
  // Min-heap: (cost, node)
  using QEntry = std::pair<cost_t, node_idx_t>;
  std::priority_queue<QEntry, std::vector<QEntry>, std::greater<>> q;
  std::unordered_map<node_idx_t, cost_t> dist;

  q.emplace(0, v);
  dist[v] = 0;

  while (!q.empty()) {
    auto [cost, u] = q.top();
    q.pop();

    if (u == w) {
      // Found path to w with cost <= max_cost
      return cost <= max_cost;
    }
    if (cost > max_cost) continue;
    // For each neighbor of u
    for (auto way : r.node_ways_[u]) {
      for (auto n : r.way_nodes_[way]) {
        if (n == u || n == skip_u) continue;
        // Find edge cost u->n (from way_node_dist_)
        cost_t edge_cost = kInfeasible;
        // Get the index of u in way_nodes_[way]
        auto nodes = r.way_nodes_[way];
        for (size_t idx = 0; idx + 1 < nodes.size(); ++idx) {
          if ((nodes[idx] == u && nodes[idx + 1] == n) || (nodes[idx] == n && nodes[idx + 1] == u)) {
            edge_cost = r.way_node_dist_[way][idx];
            break;
          }
        }
        if (edge_cost == kInfeasible) continue;
        cost_t new_cost = cost + edge_cost;
        if (!dist.count(n) || new_cost < dist[n]) {
          dist[n] = new_cost;
          q.emplace(new_cost, n);
        }
      }
    }
  }
  return false; // no witness path found
}

// Build Contraction Hierarchy 
void ways::build_contraction_hierarchy() {
  auto& r = *r_;
  // 1. Initialize CH level for all nodes
  r.node_ch_level_.resize(n_nodes(), 0);
  r.contraction_hierarchy_enabled_ = true;

  // 2. Prepare containers for shortcuts
  r.shortcuts_.clear();
  r.outgoing_shortcuts_.resize(n_nodes());
  r.incoming_shortcuts_.resize(n_nodes());

  // 3. Create random node ordering
  std::vector<node_idx_t> contraction_order;
  for (node_idx_t u{0}; u < n_nodes(); ++u) {
    contraction_order.push_back(u);
  }
  
  // Random shuffle for node ordering
  std::random_device rd;
  std::mt19937 gen(rd());
  std::shuffle(contraction_order.begin(), contraction_order.end(), gen);

  // 4. For each node u in random contraction order
  for (auto u : contraction_order) {
    // (A) Find all neighbors of u
    std::vector<node_idx_t> neighbors;
    for (auto way : r.node_ways_[u]) {
      for (auto n : r.way_nodes_[way]) {
        if (n != u && std::find(neighbors.begin(), neighbors.end(), n) == neighbors.end())
          neighbors.push_back(n);
      }
    }
    // (B) For each (v, w) pair, v != w
    for (auto v : neighbors) {
      for (auto w : neighbors) {
        if (v == w) continue;
        // Get cost v->u
        cost_t v_to_u = kInfeasible;
        for (auto way : r.node_ways_[u]) {
          auto nodes = r.way_nodes_[way];
          for (size_t idx = 0; idx + 1 < nodes.size(); ++idx) {
            if ((nodes[idx] == v && nodes[idx + 1] == u) || (nodes[idx] == u && nodes[idx + 1] == v)) {
              v_to_u = r.way_node_dist_[way][idx];
              break;
            }
          }
          if (v_to_u != kInfeasible) break;
        }
        // Get cost u->w
        cost_t u_to_w = kInfeasible;
        for (auto way : r.node_ways_[u]) {
          auto nodes = r.way_nodes_[way];
          for (size_t idx = 0; idx + 1 < nodes.size(); ++idx) {
            if ((nodes[idx] == u && nodes[idx + 1] == w) || (nodes[idx] == w && nodes[idx + 1] == u)) {
              u_to_w = r.way_node_dist_[way][idx];
              break;
            }
          }
          if (u_to_w != kInfeasible) break;
        }
        if (v_to_u == kInfeasible || u_to_w == kInfeasible) continue;

        cost_t shortcut_cost = v_to_u + u_to_w;

        // (C) Witness search: is there a v-w path avoiding u with cost ≤ shortcut_cost?
        bool witness = witness_search(r, v, w, u, shortcut_cost);
        if (!witness) {
          // (D) Add shortcut (store v->w with cost, and middle node=u for unpacking)
          ways::routing::shortcut sc{v, w, shortcut_cost, u};
          r.shortcuts_.push_back(sc);
          r.outgoing_shortcuts_[v].push_back(sc);
          r.incoming_shortcuts_[w].push_back(sc);
        }
      }
    }
    // (E) Assign level based on contraction order - nodes contracted earlier get higher levels
    // This ensures forward search goes to nodes with higher levels, backward search to lower levels
  }
  
  // Assign levels based on contraction order
  for (size_t i = 0; i < contraction_order.size(); ++i) {
    r.node_ch_level_[contraction_order[i]] = static_cast<std::uint32_t>(i);

  }
  std::set<std::tuple<node_idx_t, node_idx_t, cost_t, node_idx_t>> seen;
  size_t dupes = 0;
  for (const auto& sc : r.shortcuts_) {
      auto key = std::make_tuple(sc.from, sc.to, sc.cost, sc.middle);
      if (!seen.insert(key).second) ++dupes;
  }
  fmt::println("DUPLICATE SHORTCUTS BEFORE DEDUPLICATION: {}", dupes);

  //deduplicate_shortcuts(r);
}

}  // namespace osr