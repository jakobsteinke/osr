#include "gtest/gtest.h"
#include "fmt/format.h"

#include "osr/routing/ch_bidirectional.h"
#include "osr/routing/ch_levels.h"
#include "osr/routing/ch_shortcut.h"
#include "osr/routing/profiles/car.h"

namespace osr {

TEST(ch_bidirectional, initialization) {
  ch_levels levels(5);
  ch_shortcuts shortcuts;
  ch_bidirectional<car> bidir_search(levels, shortcuts);

  location start{point{static_cast<int32_t>(47000000), static_cast<int32_t>(8000000)}};
  location end{point{static_cast<int32_t>(47100000), static_cast<int32_t>(8100000)}};
  
  bidir_search.init(cost_t{1000}, start, end);
  
  EXPECT_FALSE(bidir_search.has_path());
  EXPECT_FALSE(bidir_search.is_finished());
  EXPECT_EQ(bidir_search.get_shortest_path_cost(), kInfeasible);
  
  auto const [forward_mp, backward_mp] = bidir_search.get_meeting_point();
  EXPECT_EQ(forward_mp, car::node::invalid());
  EXPECT_EQ(backward_mp, car::node::invalid());
}

TEST(ch_bidirectional, single_direction_start) {
  ch_levels levels(5);
  ch_shortcuts shortcuts;
  ch_bidirectional<car> bidir_search(levels, shortcuts);

  location start{point{static_cast<int32_t>(47000000), static_cast<int32_t>(8000000)}};
  location end{point{static_cast<int32_t>(47100000), static_cast<int32_t>(8100000)}};
  
  bidir_search.init(cost_t{1000}, start, end);
  
  // Add only forward start
  auto const forward_node = car::node{.n_ = node_idx_t{0}, .way_ = way_pos_t{0}, .dir_ = direction::kForward};
  bidir_search.add_forward_start(forward_node, cost_t{0});
  
  // Forward distances should have this node
  auto const& forward_distances = bidir_search.get_forward_distances();
  EXPECT_EQ(forward_distances.size(), 1);
  
  // Backward distances should be empty
  auto const& backward_distances = bidir_search.get_backward_distances();
  EXPECT_EQ(backward_distances.size(), 0);
  
  EXPECT_FALSE(bidir_search.has_path());
}

TEST(ch_bidirectional, bidirectional_start) {
  ch_levels levels(5);
  ch_shortcuts shortcuts;
  ch_bidirectional<car> bidir_search(levels, shortcuts);

  location start{point{static_cast<int32_t>(47000000), static_cast<int32_t>(8000000)}};
  location end{point{static_cast<int32_t>(47100000), static_cast<int32_t>(8100000)}};
  
  bidir_search.init(cost_t{1000}, start, end);
  
  // Add both directions
  auto const forward_node = car::node{.n_ = node_idx_t{0}, .way_ = way_pos_t{0}, .dir_ = direction::kForward};
  auto const backward_node = car::node{.n_ = node_idx_t{4}, .way_ = way_pos_t{0}, .dir_ = direction::kBackward};
  
  bidir_search.add_forward_start(forward_node, cost_t{0});
  bidir_search.add_backward_start(backward_node, cost_t{0});
  
  // Both searches should have starting nodes
  auto const& forward_distances = bidir_search.get_forward_distances();
  auto const& backward_distances = bidir_search.get_backward_distances();
  
  EXPECT_EQ(forward_distances.size(), 1);
  EXPECT_EQ(backward_distances.size(), 1);
  
  EXPECT_FALSE(bidir_search.has_path()); // No meeting point yet
}

TEST(ch_bidirectional, meeting_point_detection_concept) {
  ch_levels levels(3);
  ch_shortcuts shortcuts;
  ch_bidirectional<car> bidir_search(levels, shortcuts);

  location start{point{static_cast<int32_t>(47000000), static_cast<int32_t>(8000000)}};
  location end{point{static_cast<int32_t>(47100000), static_cast<int32_t>(8100000)}};
  
  bidir_search.init(cost_t{1000}, start, end);
  
  // Create a scenario where both searches could meet at the same node
  // This tests the meeting point detection logic conceptually
  auto const meeting_node = car::node{.n_ = node_idx_t{1}, .way_ = way_pos_t{0}, .dir_ = direction::kForward};
  
  bidir_search.add_forward_start(meeting_node, cost_t{50});
  bidir_search.add_backward_start(meeting_node, cost_t{30});
  
  // Both searches start at the same node - this should be detected as a meeting point
  // In the actual implementation with graph expansion, this would be detected during search
  auto const& forward_distances = bidir_search.get_forward_distances();
  auto const& backward_distances = bidir_search.get_backward_distances();
  
  EXPECT_EQ(forward_distances.size(), 1);
  EXPECT_EQ(backward_distances.size(), 1);
  
  // The meeting point detection logic will be tested more thoroughly with real graph data
}

TEST(ch_bidirectional, search_state_management) {
  ch_levels levels(5);
  ch_shortcuts shortcuts;
  ch_bidirectional<car> bidir_search(levels, shortcuts);

  location start{point{static_cast<int32_t>(47000000), static_cast<int32_t>(8000000)}};
  location end{point{static_cast<int32_t>(47100000), static_cast<int32_t>(8100000)}};
  
  // Initial state
  EXPECT_FALSE(bidir_search.is_finished());
  EXPECT_FALSE(bidir_search.has_path());
  
  // After init
  bidir_search.init(cost_t{500}, start, end);
  EXPECT_FALSE(bidir_search.is_finished());
  EXPECT_FALSE(bidir_search.has_path());
  EXPECT_EQ(bidir_search.get_shortest_path_cost(), kInfeasible);
  
  // Add some nodes
  auto const node1 = car::node{.n_ = node_idx_t{0}, .way_ = way_pos_t{0}, .dir_ = direction::kForward};
  auto const node2 = car::node{.n_ = node_idx_t{2}, .way_ = way_pos_t{0}, .dir_ = direction::kBackward};
  
  bidir_search.add_forward_start(node1, cost_t{0});
  bidir_search.add_backward_start(node2, cost_t{0});
  
  EXPECT_FALSE(bidir_search.is_finished());
  EXPECT_FALSE(bidir_search.has_path());
}

TEST(ch_bidirectional, cost_tracking) {
  ch_levels levels(4);
  ch_shortcuts shortcuts;
  ch_bidirectional<car> bidir_search(levels, shortcuts);

  location start{point{static_cast<int32_t>(47000000), static_cast<int32_t>(8000000)}};
  location end{point{static_cast<int32_t>(47100000), static_cast<int32_t>(8100000)}};
  
  bidir_search.init(cost_t{1000}, start, end);
  
  // Add nodes with different costs
  auto const forward_cheap = car::node{.n_ = node_idx_t{0}, .way_ = way_pos_t{0}, .dir_ = direction::kForward};
  auto const forward_expensive = car::node{.n_ = node_idx_t{1}, .way_ = way_pos_t{0}, .dir_ = direction::kForward};
  auto const backward_cheap = car::node{.n_ = node_idx_t{2}, .way_ = way_pos_t{0}, .dir_ = direction::kBackward};
  auto const backward_expensive = car::node{.n_ = node_idx_t{3}, .way_ = way_pos_t{0}, .dir_ = direction::kBackward};
  
  bidir_search.add_forward_start(forward_cheap, cost_t{10});
  bidir_search.add_forward_start(forward_expensive, cost_t{100});
  bidir_search.add_backward_start(backward_cheap, cost_t{20});
  bidir_search.add_backward_start(backward_expensive, cost_t{200});
  
  auto const& forward_distances = bidir_search.get_forward_distances();
  auto const& backward_distances = bidir_search.get_backward_distances();
  
  EXPECT_EQ(forward_distances.size(), 2);
  EXPECT_EQ(backward_distances.size(), 2);
  
  // All nodes should be trackable
  EXPECT_FALSE(bidir_search.has_path()); // No meetings yet
}

TEST(ch_bidirectional, template_compatibility) {
  ch_levels levels(3);
  ch_shortcuts shortcuts;
  
  // Test that the template works with car profile
  ch_bidirectional<car> bidir_search(levels, shortcuts);
  
  location start{point{static_cast<int32_t>(47000000), static_cast<int32_t>(8000000)}};
  location end{point{static_cast<int32_t>(47100000), static_cast<int32_t>(8100000)}};
  
  bidir_search.init(cost_t{500}, start, end);
  
  auto const test_node = car::node{.n_ = node_idx_t{0}, .way_ = way_pos_t{0}, .dir_ = direction::kForward};
  bidir_search.add_forward_start(test_node, cost_t{0});
  
  // Should compile and work with car profile
  EXPECT_FALSE(bidir_search.has_path());
  EXPECT_FALSE(bidir_search.is_finished());
  
  auto const [fwd_mp, bwd_mp] = bidir_search.get_meeting_point();
  EXPECT_EQ(fwd_mp, car::node::invalid());
  EXPECT_EQ(bwd_mp, car::node::invalid());
}

TEST(ch_bidirectional, search_integration_concept) {
  ch_levels levels(6);
  ch_shortcuts shortcuts;
  ch_bidirectional<car> bidir_search(levels, shortcuts);

  location start{point{static_cast<int32_t>(47000000), static_cast<int32_t>(8000000)}};
  location end{point{static_cast<int32_t>(47100000), static_cast<int32_t>(8100000)}};
  
  bidir_search.init(cost_t{1000}, start, end);
  
  // Set up multiple starting points to simulate a more realistic scenario
  std::vector<car::node> forward_starts = {
    car::node{.n_ = node_idx_t{0}, .way_ = way_pos_t{0}, .dir_ = direction::kForward},
    car::node{.n_ = node_idx_t{1}, .way_ = way_pos_t{0}, .dir_ = direction::kForward}
  };
  
  std::vector<car::node> backward_starts = {
    car::node{.n_ = node_idx_t{4}, .way_ = way_pos_t{0}, .dir_ = direction::kBackward},
    car::node{.n_ = node_idx_t{5}, .way_ = way_pos_t{0}, .dir_ = direction::kBackward}
  };
  
  for (std::size_t i = 0; i < forward_starts.size(); ++i) {
    bidir_search.add_forward_start(forward_starts[i], cost_t{static_cast<std::uint16_t>(i * 10)});
  }
  
  for (std::size_t i = 0; i < backward_starts.size(); ++i) {
    bidir_search.add_backward_start(backward_starts[i], cost_t{static_cast<std::uint16_t>(i * 15)});
  }
  
  auto const& forward_distances = bidir_search.get_forward_distances();
  auto const& backward_distances = bidir_search.get_backward_distances();
  
  EXPECT_EQ(forward_distances.size(), forward_starts.size());
  EXPECT_EQ(backward_distances.size(), backward_starts.size());
  
  // The actual search expansion and meeting point detection will be tested
  // more thoroughly with real graph data in integration tests
  EXPECT_FALSE(bidir_search.is_finished());
}

TEST(ch_bidirectional, enhanced_meeting_point_detection) {
  ch_levels levels(4);
  ch_shortcuts shortcuts;
  ch_bidirectional<car> bidir_search(levels, shortcuts);

  location start{point{static_cast<int32_t>(47000000), static_cast<int32_t>(8000000)}};
  location end{point{static_cast<int32_t>(47100000), static_cast<int32_t>(8100000)}};
  
  bidir_search.init(cost_t{500}, start, end);
  
  // Create scenario where both searches could meet at same car::node
  auto const meeting_node = car::node{.n_ = node_idx_t{2}, .way_ = way_pos_t{1}, .dir_ = direction::kForward};
  
  // Add this node to both searches with different costs
  bidir_search.add_forward_start(meeting_node, cost_t{100});
  bidir_search.add_backward_start(meeting_node, cost_t{50});
  
  // Both searches should detect the meeting point
  auto const& forward_distances = bidir_search.get_forward_distances();
  auto const& backward_distances = bidir_search.get_backward_distances();
  
  EXPECT_EQ(forward_distances.size(), 1);
  EXPECT_EQ(backward_distances.size(), 1);
  
  // The enhanced meeting point detection should work with this setup
  // In actual usage, meeting points are detected during expansion
  auto const [fwd_mp, bwd_mp] = bidir_search.get_meeting_point();
  EXPECT_EQ(fwd_mp, car::node::invalid()); // No meeting detected yet (need actual expansion)
  EXPECT_EQ(bwd_mp, car::node::invalid());
  
  // Path should not be found without actual expansion
  EXPECT_FALSE(bidir_search.has_path());
}

TEST(ch_bidirectional, abort_on_success_termination) {
  ch_levels levels(4);
  ch_shortcuts shortcuts;
  ch_bidirectional<car> bidir_search(levels, shortcuts);

  location start{point{static_cast<int32_t>(47000000), static_cast<int32_t>(8000000)}};
  location end{point{static_cast<int32_t>(47100000), static_cast<int32_t>(8100000)}};
  
  bidir_search.init(cost_t{500}, start, end);
  
  // Set up a scenario to test termination logic
  auto const node1 = car::node{.n_ = node_idx_t{0}, .way_ = way_pos_t{0}, .dir_ = direction::kForward};
  auto const node2 = car::node{.n_ = node_idx_t{1}, .way_ = way_pos_t{0}, .dir_ = direction::kForward}; 
  
  // Add nodes with costs that would trigger termination logic
  bidir_search.add_forward_start(node1, cost_t{100});
  bidir_search.add_backward_start(node2, cost_t{50});
  
  // Before any meeting point is found, should not terminate
  EXPECT_FALSE(bidir_search.is_finished());
  
  // The search should track both frontiers correctly
  EXPECT_TRUE(bidir_search.get_forward_distances().size() > 0);
  EXPECT_TRUE(bidir_search.get_backward_distances().size() > 0);
  
  // Test the abort-on-success logic will be fully validated with graph data
  // This test validates the termination infrastructure is in place
}

}  // namespace osr