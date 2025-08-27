#include "gtest/gtest.h"
#include "fmt/format.h"

#include "osr/routing/ch_query.h"
#include "osr/routing/profiles/car.h"

namespace osr {

/**
 * Test CH query data structures initialization and basic functionality.
 */
TEST(ch_query, data_structures_initialization) {
  // Create mock CH data
  constexpr std::size_t n_nodes = 5;
  ch_levels levels(n_nodes);
  ch_shortcuts shortcuts;

  // Create CH query with car profile
  ch_query<car> query(levels, shortcuts);

  // Verify references are correctly stored
  EXPECT_EQ(&query.levels_, &levels);
  EXPECT_EQ(&query.shortcuts_, &shortcuts);

  // Test initial state
  EXPECT_EQ(query.get_tentative_cost(), kInfeasible);
  EXPECT_TRUE(query.is_forward_finished());  // Empty queues
  EXPECT_TRUE(query.is_backward_finished());

  auto const [forward_mp, backward_mp] = query.get_meeting_point();
  EXPECT_EQ(forward_mp, car::node::invalid());
  EXPECT_EQ(backward_mp, car::node::invalid());

  EXPECT_FALSE(query.should_terminate()); // No meeting point yet
}

TEST(ch_query, reset_functionality) {
  ch_levels levels(10);
  ch_shortcuts shortcuts;
  ch_query<car> query(levels, shortcuts);

  // Create test locations
  location start_loc{point{static_cast<int32_t>(47000000), static_cast<int32_t>(8000000)}};
  location end_loc{point{static_cast<int32_t>(47100000), static_cast<int32_t>(8100000)}};
  cost_t const max_cost = 1000;

  // Reset query
  query.reset(max_cost, start_loc, end_loc);

  // Verify reset state - coordinates should be set (not checking exact values due to scaling)
  EXPECT_NE(query.start_loc_.pos_.lat(), 0.0);
  EXPECT_NE(query.start_loc_.pos_.lng(), 0.0);
  EXPECT_NE(query.end_loc_.pos_.lat(), 0.0);
  EXPECT_NE(query.end_loc_.pos_.lng(), 0.0);

  EXPECT_EQ(query.get_tentative_cost(), kInfeasible);
  EXPECT_FALSE(query.forward_finished_);
  EXPECT_FALSE(query.backward_finished_);

  // Verify queues are reset
  EXPECT_TRUE(query.forward_pq_.empty());
  EXPECT_TRUE(query.backward_pq_.empty());
  EXPECT_TRUE(query.forward_costs_.empty());
  EXPECT_TRUE(query.backward_costs_.empty());
}

TEST(ch_query, level_filtering) {
  ch_levels levels(5);
  ch_shortcuts shortcuts;
  ch_query<car> query(levels, shortcuts);

  // Test level filtering for CH edges
  for (std::uint32_t i = 0; i < 5; ++i) {
    for (std::uint32_t j = 0; j < 5; ++j) {
      node_idx_t const from{i};
      node_idx_t const to{j};

      bool const expected_allowed = levels.is_higher_level(to, from);
      bool const actual_allowed = query.is_ch_edge_allowed(from, to);

      EXPECT_EQ(actual_allowed, expected_allowed)
          << "Edge (" << i << " -> " << j << ") level filtering mismatch. "
          << "From level: " << levels.get_level(from) 
          << ", To level: " << levels.get_level(to);
    }
  }
}

TEST(ch_query, meeting_point_update) {
  ch_levels levels(3);
  ch_shortcuts shortcuts;
  ch_query<car> query(levels, shortcuts);

  // Reset with mock locations
  location start{point{static_cast<int32_t>(47000000), static_cast<int32_t>(8000000)}};
  location end{point{static_cast<int32_t>(47100000), static_cast<int32_t>(8100000)}};
  query.reset(1000, start, end);

  // Create mock car nodes
  auto const node_0 = car::node{.n_ = node_idx_t{0}, .way_ = way_pos_t{0}, .dir_ = direction::kForward};
  auto const node_1 = car::node{.n_ = node_idx_t{1}, .way_ = way_pos_t{0}, .dir_ = direction::kForward};

  // Manually add costs (simulating search progress)
  car::label const forward_label{node_0, cost_t{100}};
  car::label const backward_label{node_1, cost_t{150}};

  // Add to cost maps
  query.forward_costs_[node_0.get_key()] = car::entry{};
  query.forward_costs_[node_0.get_key()].update(forward_label, node_0, cost_t{100}, car::node::invalid());
  
  query.backward_costs_[node_1.get_key()] = car::entry{};
  query.backward_costs_[node_1.get_key()].update(backward_label, node_1, cost_t{150}, car::node::invalid());

  // Test meeting point update
  query.update_meeting_point(node_0, node_1);

  EXPECT_EQ(query.get_tentative_cost(), cost_t{250});
  auto const [forward_mp, backward_mp] = query.get_meeting_point();
  EXPECT_EQ(forward_mp.n_, node_0.n_);
  EXPECT_EQ(backward_mp.n_, node_1.n_);
}

TEST(ch_query, termination_logic) {
  ch_levels levels(3);
  ch_shortcuts shortcuts;
  ch_query<car> query(levels, shortcuts);

  location start{point{static_cast<int32_t>(47000000), static_cast<int32_t>(8000000)}};
  location end{point{static_cast<int32_t>(47100000), static_cast<int32_t>(8100000)}};
  query.reset(1000, start, end);

  // Initially should not terminate (no meeting point)
  EXPECT_FALSE(query.should_terminate());

  // Set tentative shortest path
  query.tentative_shortest_path_ = cost_t{200};

  // With empty queues, should terminate
  EXPECT_TRUE(query.should_terminate());

  // Add labels to queues with costs higher than tentative
  auto const high_cost_node = car::node{.n_ = node_idx_t{0}, .way_ = way_pos_t{0}, .dir_ = direction::kForward};
  car::label const high_cost_label{high_cost_node, cost_t{300}};
  
  query.forward_pq_.push(high_cost_label);
  query.backward_pq_.push(high_cost_label);

  // Should still terminate (all remaining keys > tentative)
  EXPECT_TRUE(query.should_terminate());

  // Add label with cost lower than tentative
  car::label const low_cost_label{high_cost_node, cost_t{100}};
  query.forward_pq_.push(low_cost_label);

  // Should not terminate (some keys < tentative)  
  EXPECT_FALSE(query.should_terminate());
}

TEST(ch_query, search_direction_state) {
  ch_levels levels(3);
  ch_shortcuts shortcuts;
  ch_query<car> query(levels, shortcuts);

  location start{point{static_cast<int32_t>(47000000), static_cast<int32_t>(8000000)}};
  location end{point{static_cast<int32_t>(47100000), static_cast<int32_t>(8100000)}};
  query.reset(1000, start, end);

  // Initially not finished
  EXPECT_FALSE(query.forward_finished_);
  EXPECT_FALSE(query.backward_finished_);

  // Mark forward as finished
  query.mark_finished(direction::kForward);
  EXPECT_TRUE(query.forward_finished_);
  EXPECT_FALSE(query.backward_finished_);

  // Mark backward as finished
  query.mark_finished(direction::kBackward);
  EXPECT_TRUE(query.forward_finished_);
  EXPECT_TRUE(query.backward_finished_);
}

TEST(ch_query, cost_tracking) {
  ch_levels levels(3);
  ch_shortcuts shortcuts;
  ch_query<car> query(levels, shortcuts);

  // Create test node
  auto const test_node = car::node{.n_ = node_idx_t{0}, .way_ = way_pos_t{0}, .dir_ = direction::kForward};

  // Initially should return infeasible cost
  EXPECT_EQ(query.get_cost<direction::kForward>(test_node), kInfeasible);
  EXPECT_EQ(query.get_cost<direction::kBackward>(test_node), kInfeasible);

  // Add entry to forward costs
  car::label const forward_label{test_node, cost_t{100}};
  query.forward_costs_[test_node.get_key()] = car::entry{};
  query.forward_costs_[test_node.get_key()].update(
      forward_label, test_node, cost_t{100}, car::node::invalid());

  // Should now return the cost for forward, still infeasible for backward
  EXPECT_EQ(query.get_cost<direction::kForward>(test_node), cost_t{100});
  EXPECT_EQ(query.get_cost<direction::kBackward>(test_node), kInfeasible);

  // Add entry to backward costs
  car::label const backward_label{test_node, cost_t{200}};
  query.backward_costs_[test_node.get_key()] = car::entry{};
  query.backward_costs_[test_node.get_key()].update(
      backward_label, test_node, cost_t{200}, car::node::invalid());

  // Should now return costs for both directions
  EXPECT_EQ(query.get_cost<direction::kForward>(test_node), cost_t{100});
  EXPECT_EQ(query.get_cost<direction::kBackward>(test_node), cost_t{200});
}

}  // namespace osr