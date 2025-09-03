#ifdef _WIN32
#include "windows.h"
#endif

#include "gtest/gtest.h"

#include <filesystem>
#include <random>
#include <numeric>
#include <unordered_set>

#include "cista/mmap.h"

#include "utl/parallel_for.h"

#include "fmt/core.h"

#include "osr/extract/extract.h"
#include "osr/geojson.h"
#include "osr/location.h"
#include "osr/lookup.h"
#include "osr/routing/dijkstra.h"
#include "osr/routing/profile.h"
#include "osr/routing/profiles/car.h"
#include "osr/routing/route.h"
#include "osr/routing/ch_bidirectional.h"
#include "osr/types.h"
#include "osr/ways.h"

namespace fs = std::filesystem;
using namespace osr;

constexpr auto const kUseMultithreading = true;
constexpr auto const kPrintDebugGeojson = false;
constexpr auto const kMaxMatchDistance = 100;

static void load_ch_test(std::string_view raw_data, std::string_view data_dir) {
  if (!fs::exists(data_dir)) {
    if (fs::exists(raw_data)) {
      auto const p = fs::path{data_dir};
      auto ec = std::error_code{};
      fs::remove_all(p, ec);
      fs::create_directories(p, ec);
      osr::extract(false, raw_data, data_dir, fs::path{});
    }
  }
}

static void print_ch_paths(osr::ways const& w, node_idx_t from_node, node_idx_t to_node) {
  auto& ch = get_ch_bidirectional<car>(); // same instance used by route_ch

  auto sc = ch.reconstruct_with_shortcuts();
  auto osm = ch.reconstruct_unpacked(w, *w.r_, /*sharing*/nullptr, /*elev*/nullptr);

  auto sum = [](auto const& v) {
    std::uint64_t s = 0;
    for (auto const& e : v) s += e.cost_;
    return s;
  };

  fmt::println("\n[CH] PATH ANALYSIS for {} -> {}", 
               w.node_to_osm_[from_node], w.node_to_osm_[to_node]);
  fmt::println("[CH] Path WITH shortcuts ({} steps)  total_cost={}", sc.size(), sum(sc));
  for (auto const& e : sc) {
    auto from_osm = w.node_to_osm_[e.from_.get_node()];
    auto to_osm   = w.node_to_osm_[e.to_.get_node()];
    if (!e.is_shortcut_) {
      fmt::println("  OSM  {} -> {}  cost={}", from_osm, to_osm, e.cost_);
    } else {
      auto via_osm = w.node_to_osm_[e.sc_.via_];
      fmt::println("  SC   {} -> {}  via={}  cost={}", from_osm, to_osm, via_osm, e.cost_);
    }
  }

  fmt::println("\n[CH] Fully UNPACKED (OSM-only) ({} edges)  total_cost={}", osm.size(), sum(osm));
  for (auto const& e : osm) {
    auto from_osm = w.node_to_osm_[e.from_.get_node()];
    auto to_osm   = w.node_to_osm_[e.to_.get_node()];
    fmt::println("  OSM  {} -> {}  cost={}", from_osm, to_osm, e.cost_);
  }
}

void run_ch_comparison(ways const& w,
                       lookup const& l,
                       unsigned const /*n_samples*/,
                       unsigned const max_cost) {
  // 1) Collect the first 10 pairs where plain Dijkstra finds a path
  constexpr unsigned kTargetPairs = 10U;
  std::vector<std::pair<node_idx_t, node_idx_t>> from_tos;
  from_tos.reserve(kTargetPairs);

  std::mt19937 prng{};
  std::uniform_int_distribution<std::uint32_t> distr(0, w.n_nodes() - 1);
  std::unordered_set<std::uint64_t> seen;

  auto const node_pinned_matches =
      [&](location const& loc, node_idx_t const n, bool const reverse) {
        auto matches = l.match<car>(loc, reverse, direction::kForward,
                                    kMaxMatchDistance, nullptr);
        std::erase_if(matches, [&](auto const& wc) {
          return wc.left_.node_ != n && wc.right_.node_ != n;
        });
        return matches;
      };

  while (from_tos.size() < kTargetPairs) {
    auto const from_node = node_idx_t{distr(prng)};
    auto const to_node   = node_idx_t{distr(prng)};
    if (from_node.v_ == to_node.v_) { continue; }

    auto const key = (static_cast<std::uint64_t>(from_node.v_) << 32) |
                      static_cast<std::uint64_t>(to_node.v_);
    if (!seen.insert(key).second) { continue; }

    auto const from_loc = location{w.get_node_pos(from_node)};
    auto const to_loc   = location{w.get_node_pos(to_node)};

    auto const from_matches = node_pinned_matches(from_loc, from_node, false);
    auto const to_matches   = node_pinned_matches(to_loc,   to_node,   true);
    if (from_matches.empty() || to_matches.empty()) { continue; }

    auto const from_matches_span = std::span{begin(from_matches), end(from_matches)};
    auto const to_matches_span   = std::span{begin(to_matches),   end(to_matches)};

    auto const dijkstra_result =
        route(w, l, search_profile::kCar, from_loc, to_loc, from_matches_span,
              to_matches_span, max_cost, direction::kForward, nullptr, nullptr,
              nullptr, routing_algorithm::kDijkstra);

    if (dijkstra_result.has_value()) {
      fmt::println("Found working pair #{}/{}: {} -> {} (cost: {})",
                   from_tos.size() + 1, kTargetPairs,
                   w.node_to_osm_[from_node], w.node_to_osm_[to_node], dijkstra_result->cost_);
      from_tos.emplace_back(from_node, to_node);
    }
  }

  // 2) PRE-WARM CH (trigger preprocessing once, not timed)
  {
    auto const from_node = from_tos.front().first;
    auto const to_node   = from_tos.front().second;
    auto const from_loc  = location{w.get_node_pos(from_node)};
    auto const to_loc    = location{w.get_node_pos(to_node)};
    auto const fm        = node_pinned_matches(from_loc, from_node, false);
    auto const tm        = node_pinned_matches(to_loc,   to_node,   true);
    auto const fm_span   = std::span{fm.begin(), fm.end()};
    auto const tm_span   = std::span{tm.begin(), tm.end()};
    // This call will perform CH preprocessing if needed. We do not measure it.
    (void)route(w, l, search_profile::kCar, from_loc, to_loc, fm_span, tm_span,
                max_cost, direction::kForward, nullptr, nullptr, nullptr,
                routing_algorithm::kCH);
  }

  // 3) Measure Dijkstra vs CH (CH time now excludes preprocessing)
  auto n_congruent = std::atomic<unsigned>{0U};
  auto n_empty_matches = std::atomic<unsigned>{0U};
  std::vector<std::chrono::steady_clock::duration> dijkstra_times;
  std::vector<std::chrono::steady_clock::duration> ch_dijkstra_times;
  auto m = std::mutex{};

  auto const single_run = [&](std::pair<node_idx_t, node_idx_t> const from_to) {
    auto const from_node = from_to.first;
    auto const to_node   = from_to.second;
    auto const from_loc  = location{w.get_node_pos(from_node)};
    auto const to_loc    = location{w.get_node_pos(to_node)};

    auto node_pinned_matches_local =
        [&](location const& loc, node_idx_t const n, bool const reverse) {
          auto matches = l.match<car>(loc, reverse, direction::kForward,
                                      kMaxMatchDistance, nullptr);
          std::erase_if(matches, [&](auto const& wc) {
            return wc.left_.node_ != n && wc.right_.node_ != n;
          });
          return matches;
        };

    auto const from_matches = node_pinned_matches_local(from_loc, from_node, false);
    auto const to_matches   = node_pinned_matches_local(to_loc,   to_node,   true);
    if (from_matches.empty() || to_matches.empty()) {
      ++n_empty_matches;
    }

    auto const from_matches_span =
        std::span{begin(from_matches), end(from_matches)};
    auto const to_matches_span = std::span{begin(to_matches), end(to_matches)};

    auto const dijkstra_start = std::chrono::steady_clock::now();
    auto const dijkstra_result =
        route(w, l, search_profile::kCar, from_loc, to_loc, from_matches_span,
              to_matches_span, max_cost, direction::kForward, nullptr, nullptr,
              nullptr, routing_algorithm::kDijkstra);
    auto const dijkstra_time =
        std::chrono::steady_clock::now() - dijkstra_start;

    auto const ch_dijkstra_start = std::chrono::steady_clock::now();
    auto const ch_dijkstra_result =
        route(w, l, search_profile::kCar, from_loc, to_loc, from_matches_span,
              to_matches_span, max_cost, direction::kForward, nullptr, nullptr,
              nullptr, routing_algorithm::kCH);
    auto const ch_dijkstra_time =
        std::chrono::steady_clock::now() - ch_dijkstra_start;

    if (dijkstra_result.has_value() != ch_dijkstra_result.has_value() ||
        (dijkstra_result && ch_dijkstra_result &&
         dijkstra_result->cost_ != ch_dijkstra_result->cost_)) {
      auto const print_result = [&](std::string_view name, auto const& p,
                                    auto const& t) {
        fmt::println(
            "{:12}: {:11} --> {:11} | {} | time: "
            "{}:{:0>3}:{:0>3} s",
            name, w.node_to_osm_[from_node], w.node_to_osm_[to_node],
            p ? fmt::format("cost: {:5} | dist: {:>10.2f}", p->cost_, p->dist_)
              : "no result",
            std::chrono::duration_cast<std::chrono::seconds>(t).count(),
            std::chrono::duration_cast<std::chrono::milliseconds>(t).count() % 1000,
            std::chrono::duration_cast<std::chrono::microseconds>(t).count() % 1000);
        if (p.has_value() && kPrintDebugGeojson) {
          fmt::println("{}\n", to_featurecollection(w, p));
        }
      };
      print_result("dijkstra",    dijkstra_result,    dijkstra_time);
      print_result("ch dijkstra", ch_dijkstra_result, ch_dijkstra_time);
    } else {
      ++n_congruent;
    }
    
    // Print CH path details for the first result (regardless of congruence)
    static std::atomic<bool> printed{false};
    if (!printed.exchange(true) && ch_dijkstra_result.has_value()) {
      print_ch_paths(w, from_node, to_node);
    }

    if (!from_matches.empty() && !to_matches.empty()) {
      auto const guard = std::lock_guard{m};
      dijkstra_times.emplace_back(dijkstra_time);
      ch_dijkstra_times.emplace_back(ch_dijkstra_time);
    }
  };

  if (kUseMultithreading) {
    utl::parallel_for(from_tos, single_run);
  } else {
    std::for_each(begin(from_tos), end(from_tos), single_run);
  }

  auto const non_empty_congruent = n_congruent.load();
  auto const non_empty_samples   = static_cast<unsigned>(from_tos.size());

  EXPECT_EQ(non_empty_samples, non_empty_congruent);

  fmt::println("congruent on non-empty: {}/{} ({:3.1f}%)", non_empty_congruent,
               non_empty_samples,
               (static_cast<double>(non_empty_congruent) /
                static_cast<double>(non_empty_samples)) * 100.0);

  if (non_empty_congruent == non_empty_samples &&
      !dijkstra_times.empty() && !ch_dijkstra_times.empty()) {
    fmt::println(
        "speedup on non-empty (CH query only): {:.2f}",
        static_cast<double>(
            std::reduce(begin(dijkstra_times), end(dijkstra_times)).count()) /
            static_cast<double>(
                std::reduce(begin(ch_dijkstra_times), end(ch_dijkstra_times))
                    .count()));
  }
}

TEST(ch_dijkstra, monaco) {
  auto const raw_data = "test/monaco.osm.pbf";
  auto const data_dir = "test/monaco";
  auto const num_samples = 100U;   // kept (collector takes the first 10 that work)
  auto const max_cost = 3600U;

  if (!fs::exists(raw_data) && !fs::exists(data_dir)) {
    GTEST_SKIP() << raw_data << " not found";
  }

  load_ch_test(raw_data, data_dir);
  auto const w = osr::ways{data_dir, cista::mmap::protection::READ};
  auto const l = osr::lookup{w, data_dir, cista::mmap::protection::READ};

  run_ch_comparison(w, l, num_samples, max_cost);
}
