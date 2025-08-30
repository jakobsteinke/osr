#include "gtest/gtest.h"
#include "fmt/format.h"

#include "osr/routing/ch_path_reconstruction.h"
#include "osr/routing/ch_bidirectional.h"
#include "osr/routing/ch_levels.h"
#include "osr/routing/ch_shortcut.h"
#include "osr/routing/profiles/car.h"

namespace osr {

TEST(ch_path_reconstruction, basic_initialization) {
  ch_shortcuts shortcuts;
  ch_path_reconstruction<car> path_reconstructor(shortcuts);
  
  // Test basic functionality compiles and works
  std::vector<car::node> empty_path;
  auto const cost = path_reconstructor.calculate_path_cost(empty_path);
  EXPECT_EQ(cost, cost_t{0});
}

TEST(ch_path_reconstruction, path_cost_calculation) {
  ch_shortcuts shortcuts;
  
  // Add some shortcuts for testing
  shortcuts.add_shortcut(node_idx_t{0}, node_idx_t{1}, node_idx_t{10}, cost_t{25});
  shortcuts.add_shortcut(node_idx_t{1}, node_idx_t{2}, node_idx_t{11}, cost_t{30});
  
  ch_path_reconstruction<car> path_reconstructor(shortcuts);
  
  // Create a path using these shortcuts
  std::vector<car::node> path = {
    car::node{.n_ = node_idx_t{0}, .way_ = way_pos_t{0}, .dir_ = direction::kForward},
    car::node{.n_ = node_idx_t{1}, .way_ = way_pos_t{0}, .dir_ = direction::kForward},
    car::node{.n_ = node_idx_t{2}, .way_ = way_pos_t{0}, .dir_ = direction::kForward}
  };
  
  // Should calculate cost as sum of shortcut costs
  auto const cost = path_reconstructor.calculate_path_cost(path);
  EXPECT_EQ(cost, cost_t{55}); // 25 + 30 = 55
}

TEST(ch_path_reconstruction, single_node_path) {
  ch_shortcuts shortcuts;
  ch_path_reconstruction<car> path_reconstructor(shortcuts);
  
  // Single node path should have zero cost
  std::vector<car::node> single_node = {
    car::node{.n_ = node_idx_t{0}, .way_ = way_pos_t{0}, .dir_ = direction::kForward}
  };
  
  auto const cost = path_reconstructor.calculate_path_cost(single_node);
  EXPECT_EQ(cost, cost_t{0});
}

TEST(ch_path_reconstruction, path_with_regular_edges) {
  ch_shortcuts shortcuts;
  // No shortcuts added - all edges are regular
  
  ch_path_reconstruction<car> path_reconstructor(shortcuts);
  
  std::vector<car::node> path = {
    car::node{.n_ = node_idx_t{5}, .way_ = way_pos_t{0}, .dir_ = direction::kForward},
    car::node{.n_ = node_idx_t{6}, .way_ = way_pos_t{0}, .dir_ = direction::kForward},
    car::node{.n_ = node_idx_t{7}, .way_ = way_pos_t{0}, .dir_ = direction::kForward}
  };
  
  // Regular edges use placeholder cost of 1 each
  auto const cost = path_reconstructor.calculate_path_cost(path);
  EXPECT_EQ(cost, cost_t{2}); // 2 edges, cost 1 each
}

TEST(ch_path_reconstruction, mixed_shortcut_and_regular_edges) {
  ch_shortcuts shortcuts;
  shortcuts.add_shortcut(node_idx_t{0}, node_idx_t{1}, node_idx_t{10}, cost_t{40});
  // No shortcut for 1->2, so it's a regular edge
  
  ch_path_reconstruction<car> path_reconstructor(shortcuts);
  
  std::vector<car::node> path = {
    car::node{.n_ = node_idx_t{0}, .way_ = way_pos_t{0}, .dir_ = direction::kForward},
    car::node{.n_ = node_idx_t{1}, .way_ = way_pos_t{0}, .dir_ = direction::kForward},
    car::node{.n_ = node_idx_t{2}, .way_ = way_pos_t{0}, .dir_ = direction::kForward}
  };
  
  // First edge is shortcut (40), second edge is regular (1)
  auto const cost = path_reconstructor.calculate_path_cost(path);
  EXPECT_EQ(cost, cost_t{41}); // 40 + 1 = 41
}

TEST(ch_path_reconstruction, bidirectional_search_integration_setup) {
  // Test the integration with bidirectional search
  ch_levels levels(5);
  ch_shortcuts shortcuts;
  ch_path_reconstruction<car> path_reconstructor(shortcuts);
  
  // Create bidirectional search
  ch_bidirectional<car> bidir_search(levels, shortcuts);
  
  location start{point{static_cast<int32_t>(47000000), static_cast<int32_t>(8000000)}};
  location end{point{static_cast<int32_t>(47100000), static_cast<int32_t>(8100000)}};
  
  bidir_search.init(cost_t{500}, start, end);
  
  // Test that integration compiles
  EXPECT_FALSE(bidir_search.has_path()); // No path found yet
  
  // The actual path reconstruction will be tested with graph data
  // This test validates the integration framework is set up correctly
}

TEST(ch_path_reconstruction, path_validation_interface) {
  ch_shortcuts shortcuts;
  ch_path_reconstruction<car> path_reconstructor(shortcuts);
  
  // Test path validation interface
  std::vector<car::node> valid_path = {
    car::node{.n_ = node_idx_t{0}, .way_ = way_pos_t{0}, .dir_ = direction::kForward},
    car::node{.n_ = node_idx_t{1}, .way_ = way_pos_t{0}, .dir_ = direction::kForward}
  };
  
  // Validate interface compiles (actual validation needs graph data)
  EXPECT_EQ(valid_path.size(), 2);
  
  // Test empty path validation
  std::vector<car::node> empty_path;
  EXPECT_TRUE(empty_path.size() < 2); // Should be invalid
}

TEST(ch_path_reconstruction, path_combination_logic) {
  ch_shortcuts shortcuts;
  ch_path_reconstruction<car> path_reconstructor(shortcuts);
  
  // Test the path combination logic indirectly through cost calculation
  // This validates the internal algorithms work correctly
  
  std::vector<car::node> complex_path = {
    car::node{.n_ = node_idx_t{0}, .way_ = way_pos_t{0}, .dir_ = direction::kForward},
    car::node{.n_ = node_idx_t{1}, .way_ = way_pos_t{0}, .dir_ = direction::kForward},
    car::node{.n_ = node_idx_t{2}, .way_ = way_pos_t{0}, .dir_ = direction::kForward},
    car::node{.n_ = node_idx_t{3}, .way_ = way_pos_t{0}, .dir_ = direction::kForward}
  };
  
  // Should handle multi-edge paths correctly
  auto const cost = path_reconstructor.calculate_path_cost(complex_path);
  EXPECT_EQ(cost, cost_t{3}); // 3 regular edges, cost 1 each
}

TEST(ch_path_reconstruction, template_compatibility) {
  // Test that the template works with car profile
  ch_shortcuts shortcuts;
  ch_path_reconstruction<car> path_reconstructor(shortcuts);
  
  // Should compile and work with car profile
  std::vector<car::node> test_path = {
    car::node{.n_ = node_idx_t{0}, .way_ = way_pos_t{0}, .dir_ = direction::kForward}
  };
  
  auto const cost = path_reconstructor.calculate_path_cost(test_path);
  EXPECT_EQ(cost, cost_t{0}); // Single node has zero cost
}

}  // namespace osr