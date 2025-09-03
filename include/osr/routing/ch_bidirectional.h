#pragma once

#include <limits>
#include <vector>
#include <algorithm>  // std::min/max

#include "utl/verify.h"

#include "geo/constants.h"
#include "geo/latlng.h"

#include "osr/elevation_storage.h"
#include "osr/location.h"
#include "osr/routing/additional_edge.h"
#include "osr/routing/ch_shortcut.h"
#include "osr/routing/ch_preprocessor.h"
#include "osr/routing/dial.h"
#include "osr/routing/sharing_data.h"
#include "osr/types.h"
#include "osr/ways.h"

namespace osr {

struct sharing_data;

// Forward declaration
template <typename Profile>
ch_preprocessor<Profile>& get_ch_preprocessor();

template <typename Profile>
struct ch_bidirectional {
  static_assert(std::is_same_v<Profile, car>,
                "CH bidirectional search only supports the car profile");
  
  using profile_t = Profile;
  using key   = typename Profile::key;
  using label = typename Profile::label;
  using node  = typename Profile::node;
  using entry = typename Profile::entry;
  using hash  = typename Profile::hash;
  using cost_map = typename ankerl::unordered_dense::map<key, entry, hash>;

  // --- Path reconstruction structures ---
  struct Parent {
    node prev_;                 // predecessor slot node
    bool is_shortcut_{false};   // true => this step used a CH shortcut
    way_idx_t way_{way_idx_t::invalid()}; // valid for OSM edges
    ch_shortcut sc_{};
    std::uint32_t step_cost_{0}; // cost used for this single hop
  };

  // For debug / printing
  struct EdgeStep {
    node from_;
    node to_;
    bool is_shortcut_;
    way_idx_t way_;     // valid if !is_shortcut_
    ch_shortcut sc_;    // valid if is_shortcut_
    std::uint32_t cost_;
  };

  constexpr static auto const kDebugCH = false;

  struct Stats {
    uint64_t b_lvl_ok=0, b_lvl_skip=0, f_lvl_ok=0, f_lvl_skip=0;
    // forward
    uint64_t f_osm_ok=0, f_osm_skip=0, f_shc_ok=0, f_shc_skip=0;
    cost_t f_min_lvl=std::numeric_limits<cost_t>::max(), f_max_lvl=0;
    // backward
    uint64_t b_osm_ok=0, b_osm_skip=0, b_shc_ok=0, b_shc_skip=0;
    cost_t b_min_lvl=std::numeric_limits<cost_t>::max(), b_max_lvl=0;
  } stats_;

  static inline char d2c(direction d) { return d == direction::kForward ? 'F' : 'B'; }

  struct get_bucket {
    cost_t operator()(label const& l) { return l.cost(); }
  };

  void clear_mp() {
    meet_point_1_ = meet_point_1_.invalid();
    meet_point_2_ = meet_point_2_.invalid();
    best_cost_ = kInfeasible;
  }

  void reset(cost_t const max,
             location const& start_loc,
             location const& end_loc,
             std::vector<cost_t> const& node_levels) {
    pq1_.clear();
    pq2_.clear();
    pq1_.n_buckets(max + 1U);
    pq2_.n_buckets(max + 1U);
    cost1_.clear();
    cost2_.clear();
    parent1_.clear();
    parent2_.clear();
    clear_mp();
    start_loc_ = start_loc;
    end_loc_ = end_loc;
    start_seeds_.clear();
    end_seeds_.clear();
    node_levels_ = &node_levels;
    max_reached_1_ = false;
    max_reached_2_ = false;
    stats_ = {};
  }

  void add_start(ways const& /*w*/, ways::routing const& r, label const l, sharing_data const* /*sharing*/) {
    if (cost1_[l.get_node().get_key()].update(l, l.get_node(), l.cost(), node::invalid())) {
      pq1_.push(l);
    }
    start_seeds_.push_back(l.get_node().n_);
    // make all slot variants visible
    Profile::resolve_all(r, l.get_node().n_, level_t{static_cast<std::uint8_t>(0)},
      [&](node const& alt) {
        if (alt.get_key() != l.get_node().get_key()) {
          auto alt_label = typename Profile::label{alt, l.cost()};
          cost1_[alt.get_key()].update(alt_label, alt, l.cost(), node::invalid());
        }
      });
  }

  void add_end(ways const& /*w*/, ways::routing const& r, label const l, sharing_data const* /*sharing*/) {
    if (cost2_[l.get_node().get_key()].update(l, l.get_node(), l.cost(), node::invalid())) {
      pq2_.push(l);
    }
    end_seeds_.push_back(l.get_node().n_);
    // make all slot variants visible
    Profile::resolve_all(r, l.get_node().n_, level_t{static_cast<std::uint8_t>(0)},
      [&](node const& alt) {
        if (alt.get_key() != l.get_node().get_key()) {
          auto alt_label = typename Profile::label{alt, l.cost()};
          cost2_[alt.get_key()].update(alt_label, alt, l.cost(), node::invalid());
        }
      });
  }

  void debug_seed_connectivity(ways const& w, ways::routing const& /*r*/) {
    if constexpr (kDebugCH) {
      if (!start_seeds_.empty()) {
        fmt::println("[CH] Checking start seeds connectivity ({} seeds):", start_seeds_.size());
        for (auto const& seed : start_seeds_) {
          debug_seed_simple(w, seed);
        }
      }
      if (!end_seeds_.empty()) {
        fmt::println("[CH] Checking end seeds connectivity ({} seeds):", end_seeds_.size());
        for (auto const& seed : end_seeds_) {
          debug_seed_simple(w, seed);
        }
      }
    }
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
    return static_cast<cost_t>(f_cost + b_cost);
  }

  // CH-specific adjacency that includes shortcuts
  template <direction SearchDir, bool WithBlocked, typename Fn>
  void ch_adjacent(ways const& w,
                   ways::routing const& r,
                   node const& curr,
                   bitvec<node_idx_t> const* blocked,
                   sharing_data const* sharing,
                   elevation_storage const* elevations,
                   bool relax_levels,
                   Fn&& fn) {
    std::size_t accepted = 0;

    // 1) Regular OSM edges
    Profile::template adjacent<SearchDir, WithBlocked>(
        r, curr, blocked, sharing, elevations,
        [&](node const neighbor, std::uint32_t const cost, distance_t dist,
            way_idx_t const way, std::uint16_t from, std::uint16_t to,
            elevation_storage::elevation const elev, bool const track) {
          auto& preprocessor = get_ch_preprocessor<Profile>();
          auto curr_level = preprocessor.get_node_level(curr.n_);
          auto neighbor_level = preprocessor.get_node_level(neighbor.n_);
          bool should_explore = relax_levels ? true
            : (neighbor_level > curr_level);  // upward both sides (B is on inverted graph)
          if constexpr (SearchDir==direction::kForward) {
            should_explore ? ++stats_.f_lvl_ok : ++stats_.f_lvl_skip;
          } else {
            should_explore ? ++stats_.b_lvl_ok : ++stats_.b_lvl_skip;
          }
          auto& S = (SearchDir==direction::kForward) ? stats_.f_min_lvl : stats_.b_min_lvl;
          auto& T = (SearchDir==direction::kForward) ? stats_.f_max_lvl : stats_.b_max_lvl;
          S = std::min(S, curr_level);
          T = std::max(T, curr_level);

          if (should_explore) {
            if constexpr (SearchDir==direction::kForward) ++stats_.f_osm_ok; else ++stats_.b_osm_ok;
            fn(neighbor, cost, dist, way, from, to, elev, track);
            ++accepted;
          } else {
            if constexpr (SearchDir==direction::kForward) ++stats_.f_osm_skip; else ++stats_.b_osm_skip;
          }
        });

    // 2) Shortcuts
    auto& shortcuts_store = get_ch_shortcuts<Profile>();

    if (SearchDir == direction::kForward) {
      auto forward_shortcuts = shortcuts_store.get_forward_shortcuts_from(curr.n_);
      auto& preprocessor = get_ch_preprocessor<Profile>();
      auto curr_level = preprocessor.get_node_level(curr.n_);

      for (auto const& shortcut : forward_shortcuts) {
        if constexpr (WithBlocked) {
          if (blocked && blocked->test(shortcut.to_)) continue;
        }
        auto target_level = preprocessor.get_node_level(shortcut.to_);
        bool ok_level = relax_levels || (target_level > curr_level);
        if (!ok_level) {
          if constexpr (SearchDir==direction::kForward) ++stats_.f_shc_skip; else ++stats_.b_shc_skip;
          continue;
        }

        if (!is_first_hop_legal_for_shortcut<direction::kForward, WithBlocked>(
                w, r, curr, blocked, sharing, elevations, shortcut)) {
          if constexpr (SearchDir==direction::kForward) ++stats_.f_shc_skip; else ++stats_.b_shc_skip;
          continue;
        }

        if constexpr (SearchDir==direction::kForward) ++stats_.f_shc_ok; else ++stats_.b_shc_ok;
        Profile::resolve_all(r, shortcut.to_, level_t{static_cast<std::uint8_t>(0)}, [&](node const& target_node) {
          fn(target_node, shortcut.cost_, shortcut.distance_, way_idx_t::invalid(),
             0, 0, elevation_storage::elevation{}, false);
          ++accepted;
        });
      }
    } else {
      auto backward_shortcuts = shortcuts_store.get_backward_shortcuts_to(curr.n_);
      auto& preprocessor = get_ch_preprocessor<Profile>();
      auto curr_level = preprocessor.get_node_level(curr.n_);

      for (auto const& shortcut : backward_shortcuts) {
        if constexpr (WithBlocked) {
          if (blocked && blocked->test(shortcut.from_)) continue;
        }
        auto source_level = preprocessor.get_node_level(shortcut.from_);
        bool ok_level = relax_levels || (source_level > curr_level);
        if (!ok_level) {
          if constexpr (SearchDir==direction::kForward) ++stats_.f_shc_skip; else ++stats_.b_shc_skip;
          continue;
        }

        if (!is_first_hop_legal_for_shortcut<direction::kBackward, WithBlocked>(
                w, r, curr, blocked, sharing, elevations, shortcut)) {
          if constexpr (SearchDir==direction::kForward) ++stats_.f_shc_skip; else ++stats_.b_shc_skip;
          continue;
        }

        if constexpr (SearchDir==direction::kForward) ++stats_.f_shc_ok; else ++stats_.b_shc_ok;
        Profile::resolve_all(r, shortcut.from_, level_t{static_cast<std::uint8_t>(0)}, [&](node const& source_node) {
          fn(source_node, shortcut.cost_, shortcut.distance_, way_idx_t::invalid(),
             0, 0, elevation_storage::elevation{}, false);
          ++accepted;
        });
      }
    }

    if constexpr (kDebugCH) {
      if (accepted == 0 && (cost1_.size() + cost2_.size()) < 20) {
        auto& pre = get_ch_preprocessor<Profile>();
        fmt::println("[CH][zero-up] at node {} lvl={} (no OSM/shortcut passed level test)",
                     w.node_to_osm_[curr.n_], pre.get_node_level(curr.n_));
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

    auto const l = pq.pop();
    auto const curr = l.get_node();
    auto const curr_cost = get_cost<SearchDir>(curr);
    if (curr_cost < l.cost()) {
      return true;
    }

    const bool relax_levels = false; // strict CH

    auto const evaluate_meetpoint = [&](cost_t cost, cost_t other_cost,
                                        node meetpoint1, node meetpoint2) {
      auto tentative = static_cast<cost_t>(cost + other_cost);
      if (tentative < best_cost_) {
        meet_point_1_ = meetpoint1;
        meet_point_2_ = meetpoint2;
        best_cost_ = tentative;
      }
    };

    ch_adjacent<SearchDir, WithBlocked>(
        w, r, curr, blocked, sharing, elevations, relax_levels,
        [&](node const neighbor, std::uint32_t const cost, distance_t,
            way_idx_t const way, std::uint16_t, std::uint16_t,
            elevation_storage::elevation const, bool const track) {

          auto const total = static_cast<cost_t>(curr_cost + cost);
          if (total <= max) {
            auto const opposite_cost_map =
                opposite(SearchDir) == direction::kForward ? &cost1_ : &cost2_;
            auto other_cost = exact_cost_on_same_slot(*opposite_cost_map, neighbor);
            if (other_cost != kInfeasible) {
              if constexpr (SearchDir == direction::kForward) {
                evaluate_meetpoint(total, other_cost, neighbor, neighbor);
              } else {
                evaluate_meetpoint(other_cost, total, neighbor, neighbor);
              }
            }
          }

          if (total > max) {
            if (SearchDir == direction::kForward) {
              max_reached_1_ = true;
            } else {
              max_reached_2_ = true;
            }
            return;
          }

          auto next = label{neighbor, total};
          next.track(l, r, way, neighbor.get_node(), track);
          if (total <= max &&
              costs[neighbor.get_key()].update(next, neighbor, total, curr)) {
            
            // Record parent information for path reconstruction
            Parent pr;
            pr.prev_ = curr;
            pr.is_shortcut_ = (way == way_idx_t::invalid());
            pr.way_ = way;
            pr.step_cost_ = cost;

            if (pr.is_shortcut_) {
              if constexpr (SearchDir == direction::kForward) {
                if (auto sc = get_best_ch_forward_shortcut(curr.n_, neighbor.n_)) {
                  pr.sc_ = *sc;
                }
              } else {
                // backward search used backward shortcuts (high->low on reversed graph)
                if (auto sc = get_best_ch_backward_shortcut(curr.n_, neighbor.n_)) {
                  pr.sc_ = *sc;
                }
              }
            }

            // Store parent in the correct slot
            int si = slot_index(neighbor.get_key(), neighbor);
            if (si >= 0) {
              auto& arr = (SearchDir == direction::kForward)
                            ? parent1_[neighbor.get_key()]
                            : parent2_[neighbor.get_key()];
              arr[si] = pr;
            }
            
            pq.push(std::move(next));
          }
        });

    // same-slot meetpoint on the current node
    auto const handle_end_of_way_meetpoint = [&]() {
      auto const opposite_cost_map =
          opposite(SearchDir) == direction::kForward ? &cost1_ : &cost2_;
      auto other_cost = exact_cost_on_same_slot(*opposite_cost_map, curr);
      if (other_cost != kInfeasible) {
        if constexpr (SearchDir == direction::kForward) {
          evaluate_meetpoint(curr_cost, other_cost, curr, curr);
        } else {
          evaluate_meetpoint(other_cost, curr_cost, curr, curr);
        }
      }
    };

    handle_end_of_way_meetpoint();

    // CH abort-on-success criterion
    if (best_cost_ != kInfeasible) {
      auto const top_f =
          pq1_.empty() ? get_cost<direction::kForward>(meet_point_1_)
                       : pq1_.buckets_[pq1_.get_next_bucket()].back().cost();
      auto const top_r =
          pq2_.empty() ? get_cost<direction::kBackward>(meet_point_2_)
                       : pq2_.buckets_[pq2_.get_next_bucket()].back().cost();
      if (static_cast<cost_t>(top_f + top_r) >= best_cost_) {
        if constexpr (kDebugCH) {
          std::cout << "[CH] stop? topF=" << top_f << " topB=" << top_r
                    << " best=" << best_cost_ << "\n";
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

    debug_seed_connectivity(w, r);

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

    // car-only strict fallback intersection (uses car::entry API)
    finalize_intersection_meetpoint_strict();

    if constexpr (kDebugCH) {
      if (best_cost_ == kInfeasible) {
        std::size_t overlap = 0, shown = 0;
        for (auto const& [k, e1] : cost1_) {
          if (auto it = cost2_.find(k); it != end(cost2_)) {
            ++overlap;
            if (shown < 5) {
              if constexpr (std::is_same_v<Profile, car>) {
                if constexpr (std::is_same_v<key, node_idx_t>) {
                  std::cout << "[CH] overlap key=" << k.v_ << "\n";
                } else {
                  std::cout << "[CH] overlap key=(complex)\n";
                }
                dump_entry(std::cout, k, e1);
                dump_entry(std::cout, k, it->second);
              }
              ++shown;
            }
          }
        }
        std::cout << "[CH] FAILED - exploredF=" << cost1_.size()
                  << " exploredB=" << cost2_.size() 
                  << " overlap=" << overlap << "\n";
        std::cout << "[CH] Stats: F(osm " << stats_.f_osm_ok << "/" << (stats_.f_osm_ok + stats_.f_osm_skip)
                  << " shc " << stats_.f_shc_ok << "/" << (stats_.f_shc_ok + stats_.f_shc_skip)
                  << " lvl " << stats_.f_min_lvl << "-" << stats_.f_max_lvl << ")\n";
        std::cout << "[CH]        B(osm " << stats_.b_osm_ok << "/" << (stats_.b_osm_ok + stats_.b_osm_skip)
                  << " shc " << stats_.b_shc_ok << "/" << (stats_.b_shc_ok + stats_.b_shc_skip)
                  << " lvl " << stats_.b_min_lvl << "-" << stats_.b_max_lvl << ")\n";
        std::cout << "[CH]        F(lvl_ok " << stats_.f_lvl_ok << " lvl_skip " << stats_.f_lvl_skip << ")\n";
        std::cout << "[CH]        B(lvl_ok " << stats_.b_lvl_ok << " lvl_skip " << stats_.b_lvl_skip << ")\n";
      } else {
        std::cout << "[CH] SUCCESS cost=" << best_cost_ << "\n";
      }
    }

    if (best_cost_ != kInfeasible && best_cost_ > max) {
      clear_mp();
      return false;
    }
    return best_cost_ != kInfeasible;
  }

  bool run(ways const& w,
           ways::routing const& r,
           cost_t const max,
           bitvec<node_idx_t> const* blocked,
           sharing_data const* sharing,
           elevation_storage const* elevations,
           direction const dir,
           std::vector<cost_t> const& /*node_levels*/) {

    if (blocked == nullptr) {
      return dir == direction::kForward
                 ? run<direction::kForward, false>(w, r, max, blocked, sharing, elevations)
                 : run<direction::kBackward, false>(w, r, max, blocked, sharing, elevations);
    } else {
      return dir == direction::kForward
                 ? run<direction::kForward, true>(w, r, max, blocked, sharing, elevations)
                 : run<direction::kBackward, true>(w, r, max, blocked, sharing, elevations);
    }
  }

  dial<label, get_bucket> pq1_{get_bucket{}};
  dial<label, get_bucket> pq2_{get_bucket{}};
  location start_loc_;
  location end_loc_;
  std::vector<node_idx_t> start_seeds_;
  std::vector<node_idx_t> end_seeds_;
  node meet_point_1_;
  node meet_point_2_;
  cost_t best_cost_;
  ankerl::unordered_dense::map<key, entry, hash> cost1_;
  ankerl::unordered_dense::map<key, entry, hash> cost2_;
  using parent_array = std::array<std::optional<Parent>, entry::kN>;
  ankerl::unordered_dense::map<key, parent_array, hash> parent1_;  // forward search tree
  ankerl::unordered_dense::map<key, parent_array, hash> parent2_;  // backward search tree
  std::vector<cost_t> const* node_levels_;
  bool max_reached_1_;
  bool max_reached_2_;

  // --- Path reconstruction methods ---

  // Helper to find slot index of a node within a key
  static int slot_index(key k, node const& n) {
    for (int i = 0; i < static_cast<int>(entry::kN); ++i) {
      if (entry::get_node(k, i) == n) return i;  // node equality (slot-aware)
    }
    return -1;
  }
  
  // Walk one tree (pred -> cur) from 'leaf' back to the seed, yield forward-ordered steps.
  template <typename Map>
  std::vector<EdgeStep> collect_chain(Map const& parents, node leaf) const {
    std::vector<EdgeStep> rev;
    auto cur = leaf;
    for (;;) {
      auto it = parents.find(cur.get_key());
      if (it == end(parents)) break;
      int si = slot_index(cur.get_key(), cur);
      if (si < 0) break;
      auto const& opt = it->second[si];
      if (!opt) break;
      auto const& p = *opt;
      rev.push_back(EdgeStep{p.prev_, cur, p.is_shortcut_, p.way_, p.sc_, p.step_cost_});
      cur = p.prev_;
    }
    std::reverse(begin(rev), end(rev));
    return rev;
  }

  // Find a direct OSM edge A(slot)->B(slot) with exact slot match
  bool find_direct_osm_edge_exact(ways::routing const& r,
                                  node const& from,
                                  node const& to,
                                  sharing_data const* sharing,
                                  elevation_storage const* elevations,
                                  EdgeStep& out) const {
    bool found = false;
    Profile::template adjacent<direction::kForward, false>(
        r, from, nullptr, sharing, elevations,
        [&](node const nb, std::uint32_t const c, distance_t,
            way_idx_t const way, std::uint16_t, std::uint16_t,
            elevation_storage::elevation, bool) {
          if (!found && nb == to) {              // <-- slot-aware equality
            out = EdgeStep{from, nb, false, way, {}, c};
            found = true;
          }
        });
    return found;
  }

  // Find a direct OSM edge from A(slot) to any slot on base B; returns actual neighbor slot
  bool find_direct_osm_edge_to_base(ways::routing const& r,
                                    node const& from,
                                    node_idx_t to_base,
                                    sharing_data const* sharing,
                                    elevation_storage const* elevations,
                                    EdgeStep& out) const {
    bool found = false;
    Profile::template adjacent<direction::kForward, false>(
        r, from, nullptr, sharing, elevations,
        [&](node const nb, std::uint32_t const c, distance_t,
            way_idx_t const way, std::uint16_t, std::uint16_t,
            elevation_storage::elevation, bool) {
          if (!found && nb.n_ == to_base) {      // <-- base match, but keep 'nb'
            out = EdgeStep{from, nb, false, way, {}, c};
            found = true;
          }
        });
    return found;
  }

  // Pick an OSM first hop from 'from' towards target base 'target_base'.
  // This mirrors your query-time legality check: chase backward shortcuts
  // until an actual OSM neighbor (to some slot of 'next') exists, then return that neighbor slot step.
  std::optional<EdgeStep> pick_first_hop_to_base(ways::routing const& r,
                                                 node const& from,
                                                 node_idx_t target_base,
                                                 sharing_data const* sharing,
                                                 elevation_storage const* elevations) const {
    node_idx_t next = target_base;
    for (int guard = 0; guard < (1<<12); ++guard) {
      EdgeStep s{};
      if (find_direct_osm_edge_to_base(r, from, next, sharing, elevations, s)) {
        return s; // 'to_' is the actual neighbor slot
      }

      auto sub = get_best_ch_backward_shortcut(from.n_, next);
      if (!sub) return std::nullopt;
      next = sub->via_;
    }
    return std::nullopt;
  }

  // Generic: expand a single forward step (slot->slot) into OSM edges.
  // Tries OSM; else tries forward CH; else tries backward CH.
  void unpack_between(ways const& w,
                      ways::routing const& r,
                      node const& from,
                      node const& to,
                      sharing_data const* sharing,
                      elevation_storage const* elevations,
                      std::vector<EdgeStep>& acc,
                      int depth = 0) const {
    constexpr int kMaxDepth = 1 << 14;
    if (depth > kMaxDepth) return;

    // 1) direct OSM edge with exact slot match?
    {
      EdgeStep s{};
      if (find_direct_osm_edge_exact(r, from, to, sharing, elevations, s)) {
        acc.push_back(s);
        return;
      }
    }

    // 2) Shortcut split: prefer forward (low->high), else use backward (high->low).
    if (auto scF = get_best_ch_forward_shortcut(from.n_, to.n_)) {
      if (auto first = pick_first_hop_to_base(r, from, scF->via_, sharing, elevations)) {
        acc.push_back(*first);
        unpack_between(w, r, first->to_, to, sharing, elevations, acc, depth + 1);
      }
      return;
    }

    if (auto scB = get_best_ch_backward_shortcut(from.n_, to.n_)) {
      if (auto first = pick_first_hop_to_base(r, from, scB->via_, sharing, elevations)) {
        acc.push_back(*first);
        unpack_between(w, r, first->to_, to, sharing, elevations, acc, depth + 1);
      }
      return;
    }

    // No way to split: CH inconsistency or meet-slot mismatch.
    if constexpr (kDebugCH) {
      fmt::println("[CH][unpack] no CH shortcut between {} -> {}", from.n_.v_, to.n_.v_);
    }
  }

public:
  std::vector<EdgeStep> reconstruct_with_shortcuts() const {
    std::vector<EdgeStep> out;

    // Use a single meet slot for both halves
    node meet = meet_point_1_;
    if (meet_point_2_.get_key() != meet.get_key()) {
      fmt::println("[CH][WARN] meet slots differ: mp1={}, mp2={}",
                   meet.get_key().v_, meet_point_2_.get_key().v_);
    }

    // 1) forward side: start -> meet (already forward oriented)
    auto f = collect_chain(parent1_, meet);
    out.insert(out.end(), f.begin(), f.end());

    // 2) backward side: meet -> dest  (follow parent2_ arrows toward the end seed)
    auto cur = meet;
    for (;;) {
      auto it = parent2_.find(cur.get_key());
      if (it == end(parent2_)) break;
      int si = slot_index(cur.get_key(), cur);
      if (si < 0) break;
      auto const& opt = it->second[si];
      if (!opt) break;
      auto const& p = *opt;

      // invert each stored step to be forward-oriented
      out.push_back(EdgeStep{
        /*from=*/cur,
        /*to=*/p.prev_,
        /*is_shortcut=*/p.is_shortcut_,
        /*way=*/p.way_,
        /*sc=*/p.sc_,          // via-node still informative either direction
        /*cost=*/p.step_cost_
      });

      cur = p.prev_;
    }

#ifndef NDEBUG
    // Optional sanity check: the sum of step costs should match the algorithm's best_cost_
    if (best_cost_ != kInfeasible) {
      std::uint64_t sum = 0;
      for (auto const& e : out) sum += e.cost_;
      if (sum != best_cost_) {
        fmt::println("[CH][WARN] reconstructed sum {} != best_cost_ {}", sum, best_cost_);
      }
    }
#endif

    return out;
  }

  std::vector<EdgeStep> reconstruct_unpacked(ways const& w,
                                             ways::routing const& r,
                                             sharing_data const* sharing,
                                             elevation_storage const* elevations) const {
    auto steps = reconstruct_with_shortcuts();
    std::vector<EdgeStep> osm;
    osm.reserve(steps.size());

    for (auto const& e : steps) {
      if (!e.is_shortcut_) {
        osm.push_back(e);
      } else {
        // no "forward-only" assert here; let unpack_between choose the index
        unpack_between(w, r, e.from_, e.to_, sharing, elevations, osm);
      }
    }
    return osm;
  }

private:
  void debug_seed_simple(ways const& w, node_idx_t x) {
    if constexpr (kDebugCH) {
      auto& pre = get_ch_preprocessor<Profile>();
      auto lvl_x = pre.get_node_level(x);

      auto& sh = get_ch_shortcuts<Profile>();
      auto fwd_sh = sh.get_forward_shortcuts_from(x);
      std::size_t up_fwd_sh = 0, down_fwd_sh = 0;
      for (auto const& sc : fwd_sh) {
        auto lt = pre.get_node_level(sc.to_);
        (lt > lvl_x) ? ++up_fwd_sh : ++down_fwd_sh;
      }

      auto bwd_in = sh.get_backward_shortcuts_to(x);
      std::size_t up_bwd_in = 0, down_bwd_in = 0;
      for (auto const& sc : bwd_in) {
        auto lf = pre.get_node_level(sc.from_);
        (lf > lvl_x) ? ++up_bwd_in : ++down_bwd_in;
      }

      fmt::println("[CH][seed-debug] node={} lvl={}  F-shortcuts: up={} down={}  |  B-in shortcuts: up={} down={}",
                   w.node_to_osm_[x], lvl_x, up_fwd_sh, down_fwd_sh, up_bwd_in, down_bwd_in);

      if (up_fwd_sh == 0 && up_bwd_in == 0) {
        fmt::println("[CH][POTENTIAL-ISSUE] Seed {} has no upward shortcuts in either direction!",
                     w.node_to_osm_[x]);
      }
    }
  }

  template <typename Entry>
  static void dump_entry(std::ostream& os, key const& k, Entry const& e) {
    if constexpr (std::is_same_v<Profile, car>) {
      for (std::size_t i = 0; i < Entry::kN; ++i) {
        auto n = Entry::get_node(k, i);
        auto c = e.cost(n);
        if (c != kInfeasible) {
          os << "  slot=" << i << " waypos=" << +n.way_
             << " dir=" << d2c(n.dir_) << " cost=" << c << "\n";
        }
      }
    } else {
      // For non-car profiles we don't know the slot shape; skip.
      (void)os; (void)k; (void)e;
    }
  }

  // STRICT same-slot lookup
  template<typename Map>
  static cost_t exact_cost_on_same_slot(Map const& m, node const& n) {
    auto it = m.find(n.get_key());
    if (it == end(m)) return kInfeasible;
    return it->second.cost(n);
  }

  // ---- QUERY-TIME FIRST-HOP LEGALITY HELPERS ----

  template <direction SearchDir, bool WithBlocked>
  bool osm_has_neighbor_to(ways::routing const& r,
                           node const& curr,
                           node_idx_t target_base,
                           bitvec<node_idx_t> const* blocked,
                           sharing_data const* sharing,
                           elevation_storage const* elevations) const {
    bool ok = false;
    Profile::template adjacent<SearchDir, WithBlocked>(
        r, curr, blocked, sharing, elevations,
        [&](node const neighbor, std::uint32_t const, distance_t,
            way_idx_t const, std::uint16_t, std::uint16_t,
            elevation_storage::elevation const, bool const) {
          if (neighbor.n_ == target_base) ok = true;
        });
    return ok;
  }

  template <direction SearchDir, bool WithBlocked>
  bool is_first_hop_legal_for_shortcut(ways const&,
                                       ways::routing const& r,
                                       node const& curr,
                                       bitvec<node_idx_t> const* blocked,
                                       sharing_data const* sharing,
                                       elevation_storage const* elevations,
                                       ch_shortcut const& shortcut) const {
    auto& pre = get_ch_preprocessor<Profile>();
    node_idx_t next = shortcut.via_;
    int guard = 0;
    constexpr int kMaxFirstHopUnpackDepth = 64;

    for (;;) {
      if (osm_has_neighbor_to<SearchDir, WithBlocked>(r, curr, next, blocked, sharing, elevations)) {
        return true;
      }
      auto sub = get_best_ch_backward_shortcut(curr.n_, next);
      if (!sub.has_value()) {
        return false;
      }
      auto new_next = sub->via_;
      auto lvl_curr = pre.get_node_level(curr.n_);
      auto lvl_next = pre.get_node_level(next);
      auto lvl_new  = pre.get_node_level(new_next);
      if (!(lvl_new < lvl_next && lvl_next <= lvl_curr)) {
        return false;
      }
      next = new_next;
      if (++guard > kMaxFirstHopUnpackDepth) {
        return false;
      }
    }
  }

  // ---- car-only strict same-slot intersection fallback ----
  void finalize_intersection_meetpoint_strict() {
    if constexpr (!std::is_same_v<Profile, car>) {
      // car_sharing (and others): skip this strict fallback because
      // the entry API differs (no Entry::get_node). Expansion-time
      // meetpoint checks are still active.
      return;
    }

    if (best_cost_ != kInfeasible) return;

    for (auto const& [k, eF] : cost1_) {
      auto itB = cost2_.find(k);
      if (itB == end(cost2_)) continue;

      for (std::size_t i = 0; i < entry::kN; ++i) {
        auto n = entry::get_node(k, i);
        auto cF = eF.cost(n);
        if (cF == kInfeasible) continue;
        auto cB = itB->second.cost(n);
        if (cB == kInfeasible) continue;

        auto sum = static_cast<cost_t>(cF + cB);
        if (best_cost_ == kInfeasible || sum < best_cost_) {
          best_cost_   = sum;
          meet_point_1_ = n;
          meet_point_2_ = n;
        }
      }
    }
  }
};

}  // namespace osr
