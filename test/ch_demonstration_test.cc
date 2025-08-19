#include "gtest/gtest.h"

#include <filesystem>

#include "cista/mmap.h"

#include "osr/extract/extract.h"
#include "osr/lookup.h"
#include "osr/routing/ch_preprocessing.h"
#include "osr/routing/ch_dijkstra.h"
#include "osr/routing/dijkstra.h"
#include "osr/routing/profiles/car.h"
#include "osr/routing/route.h"
#include "osr/types.h"
#include "osr/ways.h"

namespace fs = std::filesystem;
using namespace osr;

TEST(dijkstra_astarbidir, ch_demonstration) {
  auto const raw_data = "test/monaco.osm.pbf";
  auto const data_dir = "test/monaco";
  
  if (!fs::exists(raw_data) && !fs::exists(data_dir)) {
    GTEST_SKIP() << raw_data << " not found";
  }
  
  if (!fs::exists(data_dir) && fs::exists(raw_data)) {
    fs::create_directories(data_dir);
    osr::extract(false, raw_data, data_dir, fs::path{});
  }
  
  auto const w = osr::ways{data_dir, cista::mmap::protection::READ};
  auto const l = osr::lookup{w, data_dir, cista::mmap::protection::READ};
  
  // Build CH
  std::cout << "Building CH preprocessing..." << std::endl;
  auto ch_data = ch_preprocessing::preprocess(w);
  std::cout << "CH preprocessing complete. Nodes: " << ch_data.node_levels_.size() 
            << ", Forward shortcuts: " << ch_data.forward_shortcuts_.size() 
            << ", Backward shortcuts: " << ch_data.backward_shortcuts_.size() << std::endl;
  
  // Test a simple route
  auto const from = location{w.get_node_pos(node_idx_t{100})};
  auto const to = location{w.get_node_pos(node_idx_t{200})};
  
  // Run bidirectional Dijkstra (fair comparison since CH is also bidirectional)
  auto const dijkstra_result = route(w, l, search_profile::kCar, from, to, 
                                     3600U, direction::kForward, 100.0, 
                                     nullptr, nullptr, nullptr,
                                     routing_algorithm::kAStarBi);
  
  // Run CH Dijkstra
  auto const ch_result = route(w, l, search_profile::kCar, from, to,
                               3600U, direction::kForward, 100.0,
                               nullptr, nullptr, nullptr,
                               routing_algorithm::kCHDijkstra);
  
  std::cout << "Bidirectional Dijkstra result: " 
            << (dijkstra_result.has_value() ? std::to_string(dijkstra_result->cost_) : "no result")
            << std::endl;
  std::cout << "CH result: " 
            << (ch_result.has_value() ? std::to_string(ch_result->cost_) : "no result")
            << std::endl;
  
  if (dijkstra_result.has_value()) {
    EXPECT_TRUE(ch_result.has_value());
    if (ch_result.has_value()) {
      EXPECT_EQ(dijkstra_result->cost_, ch_result->cost_);
    }
  }
}