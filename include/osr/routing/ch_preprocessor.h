#pragma once

#include <vector>
#include <random>
#include <algorithm>
#include <numeric>


#include "osr/elevation_storage.h"
#include "osr/routing/additional_edge.h"
#include "osr/routing/dial.h"
#include "osr/routing/sharing_data.h"
#include "osr/routing/ch_shortcut.h"
#include "osr/routing/profiles/car.h"
#include "osr/routing/profiles/car_sharing.h"
#include "osr/types.h"
#include "osr/ways.h"

#include "fmt/core.h"

namespace osr {

struct sharing_data;

template <typename Profile>
struct ch_preprocessor {
  // CH implementation only works for the car profile
  static_assert(std::is_same_v<Profile, car>,
                "CH preprocessing only supports the car profile");

  static constexpr bool kDebugCH = true;
  static constexpr int kMaxCase4AddsToLog = 10;

  using profile_t = Profile;
  using key = typename Profile::key;
  using label = typename Profile::label;
  using node = typename Profile::node;
  using entry = typename Profile::entry;
  using hash = typename Profile::hash;

  struct get_bucket {
    cost_t operator()(label const& l) { return l.cost(); }
  };

  // Default constructor for compatibility with get_ch_preprocessor() function
  // Must call initialize() before using
  ch_preprocessor()
      : w_(nullptr),
        r_(nullptr),
        sharing_(nullptr),
        elevations_(nullptr),
        node_levels_(),
        pq_(get_bucket{}),
        cost_(),
        initialized_(false),
        preprocessed_(false) {}

  explicit ch_preprocessor(ways const& w, 
                          ways::routing const& r,
                          sharing_data const* sharing,
                          elevation_storage const* elevations)
    : w_(&w), r_(&r), sharing_(sharing), elevations_(elevations),
      node_levels_(w.n_nodes(), 0),
      pq_(get_bucket{}), 
      cost_(),
      initialized_(true) {}
  
  void initialize(ways const& w, 
                  ways::routing const& r,
                  sharing_data const* sharing,
                  elevation_storage const* elevations) {
    if (initialized_) {
      // Already bound to a graph; don't mutate node_levels_ post-preprocess
      return;
    }
    w_ = &w;
    r_ = &r;
    sharing_ = sharing;
    elevations_ = elevations;
    node_levels_.assign(w.n_nodes(), 0);
    initialized_ = true;
  }

  void preprocess() {
    if (!initialized_) {
      fmt::println("ERROR: ch_preprocessor not initialized!");
      return;
    }
    if (preprocessed_) {
      return;  // idempotent
    }
    
    // Clear any existing shortcuts from previous runs
    get_ch_shortcuts<Profile>().clear();
    shortcuts_created_ = 0;
    
    // Simple level assignment and contraction (OSR pattern)
    assign_random_levels();
    contract_nodes();
    
    // Debug output to verify shortcuts are being created
    fmt::println("CH preprocessing completed: {} shortcuts created", shortcuts_created_);
    auto& sh = get_ch_shortcuts<Profile>();
    fmt::println("CH shortcuts in store: {}", sh.size());
    auto fw_size = sh.forward_size();
    auto bw_size = sh.backward_size();
    fmt::println("CH upward shortcuts: fwd={} bwd={}", fw_size, bw_size);
    
    // Simple consistency check: forward and backward sizes should match
    if constexpr (kDebugCH) {
      if (fw_size == bw_size) {
        fmt::println("[CH][CONSISTENCY] Forward and backward shortcut counts match");
      } else {
        fmt::println("[CH][MISMATCH] Forward count {} != Backward count {}", fw_size, bw_size);
      }
    }
    
    preprocessed_ = true;
  }

  std::vector<cost_t> const& get_node_levels() const {
    return node_levels_;
  }
  
  cost_t get_node_level(node_idx_t const node_idx) const {
    return node_levels_[node_idx.v_];
  }

private:
  void assign_random_levels() {
    // Simple random level assignment to node_idx_t (OSR pattern)
    fmt::println("CH: Assigning random levels to {} nodes", w_->n_nodes());
    
    std::mt19937 gen(42); // Fixed seed for reproducible results
    std::iota(node_levels_.begin(), node_levels_.end(), 1U);
    std::shuffle(node_levels_.begin(), node_levels_.end(), gen);
  }

  void contract_nodes() {
    // Contract nodes in level order (OSR pattern)
    std::vector<node_idx_t> nodes_by_level;
    for (node_idx_t i{0}; i.v_ < w_->n_nodes(); ++i.v_) {
      nodes_by_level.push_back(i);
    }
    
    std::sort(nodes_by_level.begin(), nodes_by_level.end(),
              [this](node_idx_t a, node_idx_t b) {
                return node_levels_[a.v_] < node_levels_[b.v_];
              });
    
    fmt::println("CH: Contracting {} nodes in level order", nodes_by_level.size());

    for (auto const& node_idx : nodes_by_level) {
      contract_node(node_idx);
    }
  }

  void contract_node(node_idx_t u_idx) {
    auto const u_lvl = get_node_level(u_idx);
    auto& sh = get_ch_shortcuts<Profile>();

    // Per-slot adjacency: build in_map_u/out_map_u for each u_node slot separately
    Profile::resolve_all(*r_, u_idx, level_t{static_cast<std::uint8_t>(0)}, [&](node const& u_node) {
      // Build in_map_u using adjacent<Backward> from this specific u_node slot
      ankerl::unordered_dense::map<key, std::pair<node, std::pair<cost_t, distance_t>>, hash> in_map_u;
      
      Profile::template adjacent<direction::kBackward, false>(
        *r_, u_node, nullptr, sharing_, elevations_,
        [&](node const v, std::uint32_t const c, distance_t const dist,
            way_idx_t const, std::uint16_t, std::uint16_t,
            elevation_storage::elevation, bool) {
          if (v.n_ == u_idx) return;
          if (get_node_level(v.n_) <= u_lvl) return;
          auto k = v.get_key();
          auto it = in_map_u.find(k);
          if (it == end(in_map_u) || c < it->second.second.first) {
            in_map_u[k] = {v, {c, dist}};
          }
        });

      // Build out_map_u using adjacent<Forward> from this specific u_node slot
      ankerl::unordered_dense::map<key, std::pair<node, std::pair<cost_t, distance_t>>, hash> out_map_u;
      
      Profile::template adjacent<direction::kForward, false>(
        *r_, u_node, nullptr, sharing_, elevations_,
        [&](node const w, std::uint32_t const c, distance_t const dist,
            way_idx_t const, std::uint16_t, std::uint16_t,
            elevation_storage::elevation, bool) {
          if (w.n_ == u_idx) return;
          if (get_node_level(w.n_) <= u_lvl) return;
          auto k = w.get_key();
          auto it = out_map_u.find(k);
          if (it == end(out_map_u) || c < it->second.second.first) {
            out_map_u[k] = {w, {c, dist}};
          }
        });

      // Merge incident shortcuts for this specific u_node slot
      // Backward shortcuts ending at u into incoming
      for (auto const& sc : sh.get_backward_shortcuts_to(u_idx)) {
        if (get_node_level(sc.from_) <= u_lvl) continue;  // remaining graph filter
        Profile::resolve_all(*r_, sc.from_, level_t{static_cast<std::uint8_t>(0)}, [&](node const& v_node) {
          if (v_node.n_ != sc.from_) return;
          auto k = v_node.get_key();
          auto it = in_map_u.find(k);
          if (it == end(in_map_u) || sc.cost_ < it->second.second.first) {
            in_map_u[k] = {v_node, {sc.cost_, sc.distance_}};
          }
        });
      }
      
      // Forward shortcuts starting from u into outgoing  
      for (auto const& sc : sh.get_forward_shortcuts_from(u_idx)) {
        if (get_node_level(sc.to_) <= u_lvl) continue;  // remaining graph filter
        Profile::resolve_all(*r_, sc.to_, level_t{static_cast<std::uint8_t>(0)}, [&](node const& w_node) {
          if (w_node.n_ != sc.to_) return;
          auto k = w_node.get_key();
          auto it = out_map_u.find(k);
          if (it == end(out_map_u) || sc.cost_ < it->second.second.first) {
            out_map_u[k] = {w_node, {sc.cost_, sc.distance_}};
          }
        });
      }

      // Convert to vectors for triple enumeration on this u_node slot only
      std::vector<std::pair<node, std::pair<cost_t, distance_t>>> u_in_nodes, u_out_nodes;
      u_in_nodes.reserve(in_map_u.size());  
      u_out_nodes.reserve(out_map_u.size());
      for (auto& kv : in_map_u)  u_in_nodes.push_back(kv.second);
      for (auto& kv : out_map_u) u_out_nodes.push_back(kv.second);
      
      if (u_in_nodes.empty() || u_out_nodes.empty()) return;

      // Removed chatty per-slot logging - now only log when something is added

      // Precompute max_out for this u_node variant
      cost_t max_out = 0;
      for (auto const& [w_node, ow] : u_out_nodes) max_out = std::max(max_out, ow.first);

      std::uint64_t added_here = 0;  // Debug: count shortcuts added from this u_node slot

      // For each incoming v: run ONE local search and decide shortcuts to all w
      for (auto const& [v_node, iv] : u_in_nodes) {
        std::vector<TargetInfo> targets;
        targets.reserve(u_out_nodes.size());
        for (auto const& [w_node, ow] : u_out_nodes) {
          if (v_node.get_key() == w_node.get_key()) continue;
          auto const via_cost = iv.first + ow.first;
          if (via_cost >= kInfeasible) continue;
          
          // Pre-filter with correct orientation (same as final (low,high) pair)
          auto from = v_node.n_, to = w_node.n_;
          auto lv = get_node_level(from);
          auto lw = get_node_level(to);
          if (lv > lw || (lv == lw && to_idx(from) > to_idx(to))) {
            std::swap(from, to);  // from=low, to=high
          }
          if (auto best = sh.get_best_forward_shortcut(from, to)) {
            if (best->cost_ <= via_cost) continue;  // already have as-good-or-better
          }
          
          targets.push_back(TargetInfo{w_node.get_key(), w_node.n_, static_cast<cost_t>(via_cost)});
        }
        if (targets.empty()) continue;

        // CH early-stop bound (computed per slot from u_out_nodes)
        cost_t B = std::min<cost_t>(kInfeasible - 1, static_cast<cost_t>(iv.first + max_out));

        run_local_witness(v_node, u_idx, B, std::span<TargetInfo const>{targets.data(), targets.size()});

        // For all targets without witness ≤ via_cost, add shortcut v->w via u
        for (auto const& t : targets) {
          auto it = distances_by_key_.find(t.k);
          // Add shortcut only if no witness exists or witness path is longer
          if (it == end(distances_by_key_) || it->second > t.via_cost) {
            // CASE 4? (center lower than both ends)
            auto const lv = get_node_level(v_node.n_);
            auto const lu = get_node_level(u_idx);
            auto const lw = get_node_level(t.w_idx);
            bool is_case4 = (lu < lv) && (lu < lw);

            // Emit one upward edge per direction depending on rank order.
            auto emit_vw = [&](node_idx_t v, node_idx_t w) {
              // forward upward (low->high)
              sh.add_forward_shortcut(v, w, u_idx, t.via_cost);
              // backward upward on reverse graph (high->low in original)
              sh.add_backward_shortcut(w, v, u_idx, t.via_cost);
            };

            node_idx_t low_node, high_node;
            if (lv < lw) {
              low_node = v_node.n_;
              high_node = t.w_idx;
            } else if (lv > lw) {
              low_node = t.w_idx;
              high_node = v_node.n_;
            } else { // lv == lw: break ties deterministically to keep strict "upward"
              if (to_idx(v_node.n_) < to_idx(t.w_idx)) {
                low_node = v_node.n_;
                high_node = t.w_idx;
              } else {
                low_node = t.w_idx;
                high_node = v_node.n_;
              }
            }
            
            emit_vw(low_node, high_node);
            ++shortcuts_created_;
            ++added_here;

            if constexpr (kDebugCH) {
              // Sanity: check that indices expose the (low,high) pair with reasonable cost
              // Forward index must expose (low -> high) with some via (store keeps only the best)
              if (auto best = sh.get_best_forward_shortcut(low_node, high_node)) {
                // If 'best' is more expensive than we just emitted, that's a real bug
                if (best->cost_ > t.via_cost) {
                  fmt::println("[CH][BUG] forward index has cost {} > emitted {}, low={} high={} (via u={})",
                               best->cost_, t.via_cost,
                               w_->node_to_osm_[low_node], w_->node_to_osm_[high_node], w_->node_to_osm_[u_idx]);
                }
              } else {
                fmt::println("[CH][BUG] forward index missing low->high pair entirely: low={} high={} (via u={})",
                             w_->node_to_osm_[low_node], w_->node_to_osm_[high_node], w_->node_to_osm_[u_idx]);
              }

              // Backward index must expose (high -> low)
              if (auto best_b = sh.get_best_backward_shortcut(high_node, low_node)) {
                if (best_b->cost_ > t.via_cost) {
                  fmt::println("[CH][BUG] backward index has cost {} > emitted {}, high={} low={} (via u={})",
                               best_b->cost_, t.via_cost,
                               w_->node_to_osm_[high_node], w_->node_to_osm_[low_node], w_->node_to_osm_[u_idx]);
                }
              } else {
                fmt::println("[CH][BUG] backward index missing high->low pair entirely: high={} low={} (via u={})",
                             w_->node_to_osm_[high_node], w_->node_to_osm_[low_node], w_->node_to_osm_[u_idx]);
              }

              if (is_case4 && case4_adds_logged_ < kMaxCase4AddsToLog) {
                fmt::println("[CH][case4-add] v={}({})  --via u={}({})-->  w={}({})  via_cost={}  witness={}",
                  w_->node_to_osm_[v_node.n_], lv,
                  w_->node_to_osm_[u_idx],      lu,
                  w_->node_to_osm_[t.w_idx],    lw,
                  t.via_cost,
                  (it == end(distances_by_key_) ? -1 : static_cast<long long>(it->second)));
                ++case4_adds_logged_;
              }
            }
          }
        }
      }
      
      // Log summary only when shortcuts were actually added
      if (added_here > 0) {
        //fmt::println("[CH] u={} slot added {}", w_->node_to_osm_[u_idx], added_here);
      }
    });

    // clean witness result map; (cost_ and pq_ are reused automatically)
    distances_by_key_.clear();
  }

  struct TargetInfo { key k; node_idx_t w_idx; cost_t via_cost; };

  void run_local_witness(node const& v_node,
                         node_idx_t forbidden_u,
                         cost_t B,
                         std::span<TargetInfo const> targets) {
    ankerl::unordered_dense::set<key, hash> target_keys;
    target_keys.reserve(targets.size());
    for (auto const& t : targets) target_keys.insert(t.k);

    pq_.clear();
    pq_.n_buckets(B + 1U);
    cost_.clear();
    distances_by_key_.clear();

    auto const forb_level = get_node_level(forbidden_u);

    label s{v_node, 0};
    if (cost_[v_node.get_key()].update(s, v_node, 0, node::invalid())) {
      pq_.push(s);
    }

    std::size_t settled_targets = 0;

    while (!pq_.empty()) {
      auto l = pq_.pop();
      auto const cur = l.get_node();
      auto const d   = l.cost();

      if (d > B) break;  // CH early stop

      if (target_keys.contains(cur.get_key())) {
        auto it = distances_by_key_.find(cur.get_key());
        if (it == end(distances_by_key_) || d < it->second) {
          distances_by_key_[cur.get_key()] = d;
        }
        if (++settled_targets == target_keys.size()) break;
      }

      // expand OSM edges inside remaining graph (levels > level(u), exclude u)
      Profile::template adjacent<direction::kForward, false>(
        *r_, cur, nullptr, sharing_, elevations_,
        [&](node const nxt, std::uint32_t const c, distance_t, way_idx_t,
            std::uint16_t, std::uint16_t, elevation_storage::elevation, bool) {
          auto nl = get_node_level(nxt.n_);
          if (nxt.n_ == forbidden_u || nl <= forb_level) return;
          cost_t nd = d + c;
          
          // Strict-improvement check before update
          if (auto it = cost_.find(nxt.get_key()); it != end(cost_)) {
            auto cur_best = it->second.cost(nxt);
            if (cur_best != kInfeasible && nd >= cur_best) return; // no improvement
          }
          
          auto next = label{nxt, nd};
          if (nd <= B && cost_[nxt.get_key()].update(next, nxt, nd, cur)) {
            pq_.push(std::move(next));
          }
        });

      // also expand existing forward shortcuts in the remaining graph
      if (shortcuts_created_ > 0) {
        for (auto const& sh : get_ch_shortcuts<Profile>().get_forward_shortcuts_from(cur.n_)) {
          if (sh.via_ == forbidden_u) continue;  // NEW: never traverse shortcuts via u during u's contraction
          auto nl = get_node_level(sh.to_);
          if (sh.to_ == forbidden_u || nl <= forb_level) continue;
          cost_t nd = d + sh.cost_;
          if (nd > B) continue;
          Profile::resolve_all(*r_, sh.to_, level_t{static_cast<std::uint8_t>(0)}, [&](node const& nxt) {
            if (nxt.n_ != sh.to_) return;
            
            // Strict-improvement check before update
            if (auto it = cost_.find(nxt.get_key()); it != end(cost_)) {
              auto cur_best = it->second.cost(nxt);
              if (cur_best != kInfeasible && nd >= cur_best) return; // no improvement
            }
            
            auto next = label{nxt, nd};
            if (cost_[nxt.get_key()].update(next, nxt, nd, cur)) {
              pq_.push(std::move(next));
            }
          });
        }
      }
    }
  }

  ways const* w_;
  ways::routing const* r_;
  sharing_data const* sharing_;
  elevation_storage const* elevations_;
  std::vector<cost_t> node_levels_;  // Simple node_idx_t -> level mapping
  dial<label, get_bucket> pq_;  // Reused for witness searches
  ankerl::unordered_dense::map<key, entry, hash> cost_;  // Reused for witness searches
  ankerl::unordered_dense::map<key, cost_t, hash> distances_by_key_;
  std::uint32_t shortcuts_created_ = 0;
  int case4_adds_logged_ = 0;
  bool initialized_ = false;
  bool preprocessed_ = false;
};

}  // namespace osr