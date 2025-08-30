#pragma once

#include <limits>
#include <vector>

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
  using profile_t = Profile;
  using key = typename Profile::key;
  using label = typename Profile::label;
  using node = typename Profile::node;
  using entry = typename Profile::entry;
  using hash = typename Profile::hash;
  using cost_map = typename ankerl::unordered_dense::map<key, entry, hash>;

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

  void add_start(ways const& w, ways::routing const& r, label const l, sharing_data const* sharing) {
    if (cost1_[l.get_node().get_key()].update(l, l.get_node(), l.cost(),
                                             node::invalid())) {
      pq1_.push(l);
    }
    // Track seed node for debugging
    start_seeds_.push_back(l.get_node().n_);
    // Make all slot variants of the same base node visible in the forward cost map
    Profile::resolve_all(r, l.get_node().n_, level_t{static_cast<std::uint8_t>(0)},
      [&](node const& alt) {
        if (alt.get_key() != l.get_node().get_key()) {
          // Write the cost for the variant but don't enqueue it.
          auto alt_label = typename Profile::label{alt, l.cost()};
          cost1_[alt.get_key()].update(alt_label, alt, l.cost(), node::invalid());
        }
      });
  }

  void add_end(ways const& w, ways::routing const& r, label const l, sharing_data const* sharing) {
    if (cost2_[l.get_node().get_key()].update(l, l.get_node(), l.cost(),
                                             node::invalid())) {
      pq2_.push(l);
    }
    // Track seed node for debugging
    end_seeds_.push_back(l.get_node().n_);
    // Same for the backward map (end seed)
    Profile::resolve_all(r, l.get_node().n_, level_t{static_cast<std::uint8_t>(0)},
      [&](node const& alt) {
        if (alt.get_key() != l.get_node().get_key()) {
          auto alt_label = typename Profile::label{alt, l.cost()};
          cost2_[alt.get_key()].update(alt_label, alt, l.cost(), node::invalid());
        }
      });
  }

  void debug_seed_connectivity(ways const& w, ways::routing const& r) {
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
    return f_cost + b_cost;
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
    
    // First, explore regular OSM edges using profile adjacency
    Profile::template adjacent<SearchDir, WithBlocked>(
        r, curr, blocked, sharing, elevations,
        [&](node const neighbor, std::uint32_t const cost, distance_t dist,
            way_idx_t const way, std::uint16_t from, std::uint16_t to,
            elevation_storage::elevation const elev, bool const track) {
          
          // Apply CH level filtering based on search direction
          auto& preprocessor = get_ch_preprocessor<Profile>();
          auto curr_level = preprocessor.get_node_level(curr.n_);
          auto neighbor_level = preprocessor.get_node_level(neighbor.n_);
          
          // Both searches move to higher ranks; backward does so on the reverse graph:
          bool should_explore = relax_levels ? true
            : (SearchDir == direction::kForward ? (neighbor_level > curr_level)
                                                : (neighbor_level > curr_level));
          
          // Update level filter counters
          if constexpr (SearchDir==direction::kForward) {
            should_explore ? ++stats_.f_lvl_ok : ++stats_.f_lvl_skip;
          } else {
            should_explore ? ++stats_.b_lvl_ok : ++stats_.b_lvl_skip;
          }
          
          // Update level stats
          auto& S = (SearchDir==direction::kForward) ? stats_.f_min_lvl : stats_.b_min_lvl;
          auto& T = (SearchDir==direction::kForward) ? stats_.f_max_lvl : stats_.b_max_lvl;
          S = std::min(S, curr_level);
          T = std::max(T, curr_level);
          
          // Update OSM edge stats
          if (should_explore) {
            if constexpr (SearchDir==direction::kForward) ++stats_.f_osm_ok; else ++stats_.b_osm_ok;
            fn(neighbor, cost, dist, way, from, to, elev, track);
            ++accepted;
          } else {
            if constexpr (SearchDir==direction::kForward) ++stats_.f_osm_skip; else ++stats_.b_osm_skip;
          }
        });
    
    // Second, explore shortcuts from the global store
    auto& shortcuts_store = get_ch_shortcuts<Profile>();
    
    // For forward search, get all forward shortcuts starting from current node
    if (SearchDir == direction::kForward) {
      auto forward_shortcuts = shortcuts_store.get_forward_shortcuts_from(curr.n_);
      auto& preprocessor = get_ch_preprocessor<Profile>();
      auto curr_level = preprocessor.get_node_level(curr.n_);
      
      
      for (auto const& shortcut : forward_shortcuts) {
        if constexpr (WithBlocked) {
          if (blocked && blocked->test(shortcut.to_)) {
            continue;
          }
        }
        
        // CH level filtering for forward shortcuts: upward
        auto target_level = preprocessor.get_node_level(shortcut.to_);
        bool ok = relax_levels || (target_level > curr_level);
        
        if (ok) {
          // Log a few case(4) examples: center lower than both ends
          if constexpr (kDebugCH) {
            static int kMaxCase4UseLogF = 10;
            static int case4_use_logged_f = 0;
            auto via_level = preprocessor.get_node_level(shortcut.via_);
            bool is_case4 = (via_level < curr_level) && (via_level < target_level);
            if (is_case4 && case4_use_logged_f < kMaxCase4UseLogF) {
              fmt::println("[CH][case4-use-F] curr={}({})  --via {}({})-->  to={}({})  shc_cost={}",
                w.node_to_osm_[curr.n_],      curr_level,
                w.node_to_osm_[shortcut.via_], via_level,
                w.node_to_osm_[shortcut.to_],  target_level,
                shortcut.cost_);
              ++case4_use_logged_f;
            }
          }
          if constexpr (SearchDir==direction::kForward) ++stats_.f_shc_ok; else ++stats_.b_shc_ok;
          // Create target node - need to resolve to proper car::node
          Profile::resolve_all(r, shortcut.to_, level_t{static_cast<std::uint8_t>(0)}, [&](node const& target_node) {
            fn(target_node, shortcut.cost_, shortcut.distance_, way_idx_t::invalid(), 
               0, 0, elevation_storage::elevation{}, false);
            ++accepted;
          });
        } else {
          if constexpr (SearchDir==direction::kForward) ++stats_.f_shc_skip; else ++stats_.b_shc_skip;
        }
      }
    } else {
      // For backward search, get all backward shortcuts ending at current node
      // These represent incoming edges in the reversed graph 
      auto backward_shortcuts = shortcuts_store.get_backward_shortcuts_to(curr.n_);
      auto& preprocessor = get_ch_preprocessor<Profile>();
      auto curr_level = preprocessor.get_node_level(curr.n_);
      
      for (auto const& shortcut : backward_shortcuts) {
        if constexpr (WithBlocked) {
          if (blocked && blocked->test(shortcut.from_)) {
            continue;
          }
        }
        
        auto source_level = preprocessor.get_node_level(shortcut.from_);
        bool ok = relax_levels || (source_level > curr_level);
        
        if (ok) {
          if constexpr (kDebugCH) {
            static int kMaxCase4UseLogB = 10;
            static int case4_use_logged_b = 0;
            auto via_level = preprocessor.get_node_level(shortcut.via_);
            bool is_case4 = (via_level < source_level) && (via_level < curr_level);
            if (is_case4 && case4_use_logged_b < kMaxCase4UseLogB) {
              fmt::println("[CH][case4-use-B] curr={}({})  <--via {}({})--  from={}({})  shc_cost={}",
                w.node_to_osm_[curr.n_],        curr_level,
                w.node_to_osm_[shortcut.via_],  via_level,
                w.node_to_osm_[shortcut.from_], source_level,
                shortcut.cost_);
              ++case4_use_logged_b;
            }
          }
          if constexpr (SearchDir==direction::kForward) ++stats_.f_shc_ok; else ++stats_.b_shc_ok;
          // Create source node - need to resolve to proper car::node
          Profile::resolve_all(r, shortcut.from_, level_t{static_cast<std::uint8_t>(0)}, [&](node const& source_node) {
            fn(source_node, shortcut.cost_, shortcut.distance_, way_idx_t::invalid(), 
               0, 0, elevation_storage::elevation{}, false);
            ++accepted;
          });
        } else {
          if constexpr (SearchDir==direction::kForward) ++stats_.f_shc_skip; else ++stats_.b_shc_skip;
        }
      }
    }
    
    // Add zero-expansion detection at the end
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

    // Classical CH does not need special first-hop relaxation
    const bool relax_levels = false; // strict CH

    auto const evaluate_meetpoint = [&](cost_t cost, cost_t other_cost,
                                        node meetpoint1, node meetpoint2) {
      auto tentative = cost + other_cost;
      if (tentative < best_cost_) {
        meet_point_1_ = meetpoint1;
        meet_point_2_ = meetpoint2;
        // Removed fragile assert - during child expansion the cost map might not be updated yet
        best_cost_ = static_cast<cost_t>(tentative);
      }
    };

    ch_adjacent<SearchDir, WithBlocked>(
        w, r, curr, blocked, sharing, elevations, relax_levels,
        [&](node const neighbor, std::uint32_t const cost, distance_t,
            way_idx_t const way, std::uint16_t, std::uint16_t,
            elevation_storage::elevation const, bool const track) {

          // --- NEW: meetpoint check on the *child* we just relaxed ---
          // Combine our cost to `neighbor` (curr_cost + edge cost) with
          // the opposite side's best cost to the same base node (any slot).
          auto const total = curr_cost + cost;
          if (total <= max) {
            auto const opposite_cost_map =
                opposite(SearchDir) == direction::kForward ? &cost1_ : &cost2_;
            auto [other_cost, other_node] =
                best_cost_on_same_key(*opposite_cost_map, r, neighbor);
            if (other_cost != kInfeasible) {
              if constexpr (SearchDir == direction::kForward) {
                // meet at `neighbor`: F knows total to neighbor, B knows other_cost to neighbor
                evaluate_meetpoint(total, other_cost, neighbor, other_node);
              } else {
                evaluate_meetpoint(other_cost, total, other_node, neighbor);
              }
            }
          }
          // --- END NEW ---

          if constexpr (kDebugCH) {
            std::cout << "  NEIGHBOR ";
            neighbor.print(std::cout, w);
            std::cout << " (level " << (*node_levels_)[neighbor.n_.v_] << ")";
          }
          
          if (total > max) {
            if (SearchDir == direction::kForward) {
              max_reached_1_ = true;
            } else {
              max_reached_2_ = true;
            }
            return;
          }
          
          auto next = label{neighbor, static_cast<cost_t>(total)};
          next.track(l, r, way, neighbor.get_node(), track);
          if (total <= max &&
              costs[neighbor.get_key()].update(
                  next, neighbor, static_cast<cost_t>(total), curr)) {
            pq.push(std::move(next));
          }
        });

    // Robust meetpoint detection with slot-agnostic search
    // Robust meetpoint detection on the current node (same base key only)
    auto const handle_end_of_way_meetpoint = [&]() {
      auto const opposite_cost_map =
          opposite(SearchDir) == direction::kForward ? &cost1_ : &cost2_;

      // Cheapest opposite-side label on the same node key (any slot)
      auto [other_cost, other_node] = best_cost_on_same_key(*opposite_cost_map, r, curr);
      if (other_cost != kInfeasible) {
        if constexpr (SearchDir == direction::kForward) {
          evaluate_meetpoint(curr_cost, other_cost, curr, other_node);
        } else {
          evaluate_meetpoint(other_cost, curr_cost, other_node, curr);
        }
      }
    };

    handle_end_of_way_meetpoint();

    // CH abort-on-success criterion: stop when all keys in PQ are >= best_cost
    if (best_cost_ != kInfeasible) {
      auto const top_f =
          pq1_.empty() ? get_cost<direction::kForward>(meet_point_1_)
                       : pq1_.buckets_[pq1_.get_next_bucket()].back().cost();
      auto const top_r =
          pq2_.empty() ? get_cost<direction::kBackward>(meet_point_2_)
                       : pq2_.buckets_[pq2_.get_next_bucket()].back().cost();
      if (top_f + top_r >= best_cost_) {
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
    // Post-run diagnostics - dump overlap between cost1_ and cost2_ if no meeting found
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
           std::vector<cost_t> const& node_levels) {
    
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
  std::vector<node_idx_t> start_seeds_;  // Track actual seed nodes for debugging
  std::vector<node_idx_t> end_seeds_;    // Track actual seed nodes for debugging
  node meet_point_1_;
  node meet_point_2_;
  cost_t best_cost_;
  ankerl::unordered_dense::map<key, entry, hash> cost1_;
  ankerl::unordered_dense::map<key, entry, hash> cost2_;
  std::vector<cost_t> const* node_levels_;
  bool max_reached_1_;
  bool max_reached_2_;

private:
  void debug_seed_simple(ways const& w, node_idx_t x) {
    if constexpr (kDebugCH) {
      auto& pre = get_ch_preprocessor<Profile>();
      auto lvl_x = pre.get_node_level(x);

      // Count shortcuts visible to the query
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
        (lf > lvl_x) ? ++up_bwd_in : ++down_bwd_in;  // Backward sources must be higher rank
      }

      fmt::println("[CH][seed-debug] node={} lvl={}  F-shortcuts: up={} down={}  |  B-in shortcuts: up={} down={}",
                   w.node_to_osm_[x], lvl_x, up_fwd_sh, down_fwd_sh, up_bwd_in, down_bwd_in);
                   
      if (up_fwd_sh == 0 && up_bwd_in == 0) {
        fmt::println("[CH][POTENTIAL-ISSUE] Seed {} has no upward shortcuts in either direction!", w.node_to_osm_[x]);
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
    }
  }

  template<typename Map>
  std::pair<cost_t, node>
  best_cost_on_same_key(Map const& m, ways::routing const& r, node const& n) const {
    // Fast path: exact slot
    if (auto it = m.find(n.get_key()); it != end(m)) {
      if (auto c = it->second.cost(n); c != kInfeasible) {
        return {c, n};
      }
    }

    // Slow path: try all variants of the same underlying node_idx
    cost_t best = kInfeasible;
    node best_n = node::invalid();
    Profile::resolve_all(r, n.n_, level_t{static_cast<std::uint8_t>(0)},
                         [&](node const& alt) {
      auto it = m.find(alt.get_key());
      if (it == end(m)) return;
      auto c = it->second.cost(alt);
      if (c != kInfeasible && (best == kInfeasible || c < best)) {
        best = c; best_n = alt;
      }
    });
    return {best, best_n};
  }

};

}  // namespace osr